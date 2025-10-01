#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Follow spawned vehicle published by your spawner via Zenoh 'target_gps'
- Compatible with payload: {"timestamp","vehicle_id","targets":[{gps?, world?, semantic_label, ...}, ...]}
- Prefers 'world' (CARLA world coords). Falls back to gps->ENU.
- Longitudinal: time-headway PI (no collision), Lateral: simple Pure Pursuit.
"""

import os, time, json, math, threading
from typing import Optional, Dict, Any, Tuple, List
import carla
import numpy as np

try:
    import zenoh
    ZENOH_AVAILABLE = True
except Exception:
    ZENOH_AVAILABLE = False
    print("[WARN] zenoh not available; will not move without target data")

# ------------------------
# Params
# ------------------------
CARLA_HOST = os.getenv("CARLA_HOST", "localhost")
CARLA_PORT = int(os.getenv("CARLA_PORT", "2000"))
CONTROL_DT  = float(os.getenv("CONTROL_DT",  "0.05"))  # 20Hz

MAX_THROTTLE = float(os.getenv("MAX_THROTTLE", "0.85"))
MAX_BRAKE    = float(os.getenv("MAX_BRAKE",    "0.9"))
MAX_STEER    = float(os.getenv("MAX_STEER",    "1.0"))

DESIRED_HEADWAY = float(os.getenv("DESIRED_HEADWAY", "1.6"))  # [s]
MIN_GAP         = float(os.getenv("MIN_GAP",         "8.0"))  # [m]
CRUISE_V_MAX    = float(os.getenv("CRUISE_V_MAX",    "18.0")) # [m/s] ≈ 65km/h
LA_MIN          = float(os.getenv("LOOKAHEAD_MIN",   "6.0"))  # [m]
LA_MAX          = float(os.getenv("LOOKAHEAD_MAX",   "22.0")) # [m]

ZKEY_LOCKON      = os.getenv("ZKEY_LOCKON",      "lock_on")
ZKEY_TARGET_GPS  = os.getenv("ZKEY_TARGET_GPS",  "target_gps")  # matches spawner
TARGET_TIMEOUT_S = float(os.getenv("TARGET_TIMEOUT_S", "1.2"))

# Fallback map origin for gps->ENU if CARLA map geolocation is unavailable
FALLBACK_LAT0 = os.getenv("ORIGIN_LAT")
FALLBACK_LON0 = os.getenv("ORIGIN_LON")

R_EARTH = 6378137.0
def enu_from_geodetic(lat: float, lon: float, lat0: float, lon0: float):
    lat_r = math.radians(lat); lon_r = math.radians(lon)
    lat0_r = math.radians(lat0); lon0_r = math.radians(lon0)
    x = R_EARTH * (lon_r - lon0_r) * math.cos(lat0_r)  # East
    y = R_EARTH * (lat_r - lat0_r)                      # North
    return x, y

def clamp(v, lo, hi): return max(lo, min(hi, v))

# ------------------------
# Shared state
# ------------------------
class Shared:
    def __init__(self):
        self.m = threading.Lock()
        self.lock_on = False
        self.target_world: Optional[Dict[str,float]] = None  # {"x","y","z"}
        self.target_ts = 0.0
        self.prev_xy = None
        self.prev_t  = None
        self.tgt_speed = 0.0

S = Shared()

def set_lockon(v: bool):
    with S.m:
        S.lock_on = bool(v)

def get_lockon() -> bool:
    with S.m:
        return S.lock_on

def set_target_world(x: float, y: float, z: float):
    with S.m:
        S.target_world = {"x": float(x), "y": float(y), "z": float(z)}
        S.target_ts = time.time()

def get_latest_target_world() -> Tuple[Optional[Dict[str,float]], float]:
    with S.m:
        return S.target_world, S.target_ts

def update_tgt_speed_estimate(tx: float, ty: float, now: float):
    with S.m:
        if S.prev_xy is not None and S.prev_t is not None:
            dt = max(1e-3, now - S.prev_t)
            dx = tx - S.prev_xy[0]; dy = ty - S.prev_xy[1]
            spd = math.hypot(dx, dy) / dt
            S.tgt_speed = 0.7*S.tgt_speed + 0.3*spd
        S.prev_xy = (tx, ty); S.prev_t = now

def get_tgt_speed() -> float:
    with S.m:
        return S.tgt_speed

# ------------------------
# Zenoh receiver
# ------------------------
class ZenohRx:
    def __init__(self):
        self.session = None
        self.sub_lock = None
        self.sub_tgps = None
        self.previous_lockon = False  # Track previous lock-on state
        self.emergency_alert_sent = False  # Track if alert has been sent

    @staticmethod
    def _payload_to_bytes(sample) -> bytes:
        p = getattr(sample, "payload", None)
        if p is None and hasattr(sample, "value"):
            p = getattr(sample.value, "payload", None)
        if p is None: raise TypeError("No payload in sample")
        return p.to_bytes() if hasattr(p, "to_bytes") else bytes(p)

    def publish_emergency_alert(self):
        """Publish emergency alert when lock-on becomes True"""
        try:
            if not self.session or self.emergency_alert_sent:
                return

            alert_payload = {
                'unit_id': 'UNIT-001',
                'alert': {
                    'type': 'chase_started',
                    'message': 'Suspect vehicle locked on - pursuit initiated',
                    'priority': 'CRITICAL',
                    'timestamp': time.time()
                }
            }
            
            message_to_send = {
                'type': 'emergency_alert',
                'data': alert_payload
            }

            topic = 'police/central/alerts/emergency/UNIT-001'
            self.session.put(topic, json.dumps(message_to_send))
            self.emergency_alert_sent = True
            print(f"[ZENOH] Emergency alert published: chase_started")
        except Exception as e:
            print(f"[ZENOH] Failed to publish emergency alert: {e}")

    def publish_chase_ended_alert(self):
        """Publish alert when chase ends"""
        try:
            if not self.session:
                return

            alert_payload = {
                'unit_id': 'UNIT-001',
                'alert': {
                    'type': 'chase_ended',
                    'message': 'Chase concluded, suspect lost or apprehended.',
                    'priority': 'INFO',
                    'timestamp': time.time()
                }
            }

            message_to_send = {
                'type': 'emergency_alert',
                'data': alert_payload
            }

            topic = 'police/central/alerts/emergency/UNIT-001'
            self.session.put(topic, json.dumps(message_to_send))
            print(f"[ZENOH] Chase ended alert published.")
        except Exception as e:
            print(f"[ZENOH] Failed to publish chase ended alert: {e}")

    def _on_lockon(self, sample):
        try:
            raw = self._payload_to_bytes(sample)
            val = json.loads(raw.decode("utf-8"))
            v = val.strip().lower() == "true" if isinstance(val, str) else bool(val)

            # Detect transition from False to True
            if v and not self.previous_lockon:
                print(f"[ZENOH] Lock-on activated! Publishing emergency alert...")
                self.publish_emergency_alert()
            elif not v and self.previous_lockon:
                # Lock-on deactivated, reset alert flag
                self.emergency_alert_sent = False
                print(f"[ZENOH] Lock-on deactivated")
                self.publish_chase_ended_alert()
                try:
                    self.session.put('police/central/suspect/UNIT-001/gps', json.dumps({'type': 'clear_suspect'}))
                    print(f"[ZENOH] Clear suspect message published")
                except Exception as e:
                    print(f"[ZENOH] Failed to publish clear suspect message: {e}")

            self.previous_lockon = v
            set_lockon(v)
            print(f"[ZENOH] lock_on={v}")
        except Exception as e:
            print(f"[ZENOH] lock_on parse error: {e}")

    def _pick_target_from_list(self, targets: List[dict], ego_xy: Tuple[float,float]) -> Optional[dict]:
        if not targets: return None
        # 1) prefer car (semantic_label==14); else any
        cars = [t for t in targets if int(t.get("semantic_label", -1)) == 14]
        cand = cars if cars else targets

        # 2) prefer those having 'world' coords first (no conversion)
        with_world = [t for t in cand if isinstance(t.get("world"), dict)]
        if with_world:
            # choose nearest in world
            def dist_w(t):
                w = t["world"]; return (w["x"]-ego_xy[0])**2 + (w["y"]-ego_xy[1])**2
            return min(with_world, key=dist_w)

        # 3) fallback: who has gps
        with_gps = [t for t in cand if isinstance(t.get("gps"), dict)]
        if with_gps:
            return with_gps[0]

        return None

    def _on_target_gps(self, sample):
        try:
            raw = self._payload_to_bytes(sample)
            val = json.loads(raw.decode("utf-8"))

            # 2 formats:
            # A) {"lat","lon","alt"}  (not used by your spawner, but supported)
            if "lat" in val or "latitude" in val:
                lat = float(val.get("lat",  val.get("latitude")))
                lon = float(val.get("lon",  val.get("longitude")))
                alt = float(val.get("alt",  val.get("altitude", 0.0)))
                # Store as gps-only; conversion to world handled in controller if needed
                set_target_world(float('nan'), float('nan'), 0.0)  # mark gps-only
                # We'll actually convert in controller step when needed; here we just flag presence.
                # For simplicity, we skip storing gps here; controller prefers 'world' from spawner anyway.
                return

            # B) {"timestamp","vehicle_id","targets":[ ... ]}
            targets = val.get("targets", [])
            # ego_xy for nearest selection: not available here; defer in controller using world ego pose.
            # Just store first; controller will recompute best each step from latest payload? -> simpler:
            # Instead: save the LAST payload globally and let controller pick using ego pose.
            with S.m:
                S._last_targets_payload = targets  # type: ignore
                S._last_targets_ts = time.time()   # type: ignore

        except Exception as e:
            print(f"[ZENOH] target_gps parse error: {e}")

    def open(self):
        if not ZENOH_AVAILABLE:
            print("[ZENOH] not available")
            return False
        try:
            self.session = zenoh.open(zenoh.Config())
            self.sub_lock = self.session.declare_subscriber(ZKEY_LOCKON, self._on_lockon)
            self.sub_tgps = self.session.declare_subscriber(ZKEY_TARGET_GPS, self._on_target_gps)
            print(f"[ZENOH] subscribed: '{ZKEY_LOCKON}', '{ZKEY_TARGET_GPS}'")
            return True
        except Exception as e:
            print(f"[ZENOH] open failed: {e}")
            return False

    def close(self):
        try:
            if self.session:
                self.session.put('police/central/suspect/UNIT-001/gps', json.dumps({'type': 'clear_suspect'}))
                print(f"[ZENOH] Clear suspect message published on close")
        except Exception as e:
            print(f"[ZENOH] Failed to publish clear suspect message on close: {e}")
        try:
            if self.sub_lock: self.sub_lock.undeclare()
        except Exception: pass
        try:
            if self.sub_tgps: self.sub_tgps.undeclare()
        except Exception: pass
        try:
            if self.session: self.session.close()
        except Exception: pass
        print("[ZENOH] closed")

# ------------------------
# Controller (Pure Pursuit + Headway PI)
# ------------------------
class Follower:
    def __init__(self, world: carla.World, vehicle: carla.Vehicle, zenoh_session=None):
        self.world = world
        self.vehicle = vehicle
        self.map = world.get_map()
        self.has_geo = False
        self.lat0 = None; self.lon0 = None
        self.zenoh_session = zenoh_session
        self.chase_start_time = None

        try:
            g0 = self.map.transform_to_geolocation(carla.Location(x=0.0,y=0.0,z=0.0))
            self.lat0 = float(g0.latitude); self.lon0 = float(g0.longitude)
            self.has_geo = True
            print(f"[GEO] map origin lat0={self.lat0:.8f}, lon0={self.lon0:.8f}")
        except Exception as e:
            if FALLBACK_LAT0 and FALLBACK_LON0:
                self.lat0 = float(FALLBACK_LAT0); self.lon0 = float(FALLBACK_LON0)
                self.has_geo = True
                print(f"[GEO] using fallback lat0={self.lat0}, lon0={self.lon0}")
            else:
                print(f"[GEO] geolocation not available; gps->ENU fallback disabled")

        # PI
        self.kp = 0.6
        self.ki = 0.15
        self.int_err = 0.0

    def publish_suspect_gps(self, target_location_x: float, target_location_y: float, target_location_z: float):
        """Publish suspect vehicle GPS data to web dashboard"""
        try:
            if not self.zenoh_session:
                return

            suspect_gps_data = {
                'type': 'suspect_gps',
                'timestamp': time.time(),
                'location': {
                    'x': target_location_x,
                    'y': target_location_y,
                    'z': target_location_z
                },
                'latitude': 37.7749 + (target_location_y / 111000),
                'longitude': -122.4194 + (target_location_x / (111000 * np.cos(np.radians(37.7749)))),
                'altitude': target_location_z,
                'chase_duration': time.time() - self.chase_start_time if self.chase_start_time else 0
            }

            topic = 'police/central/suspect/UNIT-001/gps'
            self.zenoh_session.put(topic, json.dumps(suspect_gps_data))
        except Exception as e:
            print(f"[ZENOH] Failed to publish suspect GPS: {e}")

    def _ego_state(self):
        tr = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()
        x = tr.location.x; y = tr.location.y
        yaw = math.radians(tr.rotation.yaw)
        v = math.sqrt(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z)
        return x, y, yaw, v

    def _pick_best_target_now(self, targets: List[dict], ego_xy: Tuple[float,float]) -> Optional[Tuple[float,float,float]]:
        """Pick best target from last payload using ego pose; return (tx,ty,tz) in CARLA world."""
        if not targets: return None
        # prefer cars
        cars = [t for t in targets if int(t.get("semantic_label", -1)) == 14]
        cand = cars if cars else targets

        # prefer world coords
        with_world = [t for t in cand if isinstance(t.get("world"), dict)]
        if with_world:
            def d2(t):
                w=t["world"]; return (w["x"]-ego_xy[0])**2 + (w["y"]-ego_xy[1])**2
            best = min(with_world, key=d2)
            w = best["world"]; return (float(w["x"]), float(w["y"]), float(w.get("z",0.0)))

        # fallback: gps
        with_gps = [t for t in cand if isinstance(t.get("gps"), dict)]
        if with_gps and self.has_geo:
            g = with_gps[0]["gps"]
            if g and (g.get("lat") is not None) and (g.get("lon") is not None):
                ex, ny = enu_from_geodetic(float(g["lat"]), float(g["lon"]), self.lat0, self.lon0)
                return (float(ex), float(ny), float(g.get("alt", 0.0)))
        return None

    def _pure_pursuit(self, dx: float, dy: float, yaw: float, la: float) -> float:
        # world->ego
        sy = math.sin(-yaw); cy = math.cos(-yaw)
        x_local = dx*cy - dy*sy
        y_local = dx*sy + dy*cy
        x_local = max(0.1, x_local)
        curvature = (2.0 * y_local) / (la*la)
        return clamp(curvature, -MAX_STEER, MAX_STEER)

    def _longitudinal_pi(self, desired_gap: float, dist: float, ego_v: float, tgt_v: float) -> Tuple[float,float]:
        err = dist - desired_gap
        self.int_err = clamp(self.int_err + err * CONTROL_DT, -20.0, 20.0)
        u = self.kp*err + self.ki*self.int_err
        rel_v = tgt_v - ego_v
        u += 0.2 * rel_v

        base_th = 0.08
        if err < 0.0:  # too close
            throttle = 0.0
            brake = clamp(min(MAX_BRAKE, -0.15*err + 0.05*abs(rel_v)), 0.0, MAX_BRAKE)
        else:
            throttle = clamp(base_th + 0.12*u, 0.0, MAX_THROTTLE)
            brake = 0.0

        if ego_v > CRUISE_V_MAX and throttle > 0:
            throttle = 0.0
        return throttle, brake

    def step(self) -> None:
        x, y, yaw, ego_v = self._ego_state()
        locked = get_lockon()

        # Track chase start time
        if locked and self.chase_start_time is None:
            self.chase_start_time = time.time()
            print(f"[CHASE] Chase started at {self.chase_start_time}")
        elif not locked and self.chase_start_time is not None:
            print(f"[CHASE] Chase ended, duration: {time.time() - self.chase_start_time:.1f}s")
            self.chase_start_time = None

        # 최신 targets 가져오기
        with S.m:
            targets = getattr(S, "_last_targets_payload", None)
            t_ts     = getattr(S, "_last_targets_ts", 0.0)

        now = time.time()
        valid_targets = bool(targets) and ((now - t_ts) < TARGET_TIMEOUT_S)

        if not locked or not valid_targets:
            # no target → gentle decel
            throttle, brake, steer = 0.0, (0.18 if ego_v>0.5 else 0.0), 0.0
            self.vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer))
            return

        # 타겟 선택 & world 좌표 획득
        best = self._pick_best_target_now(targets, (x,y))
        if best is None:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=(0.15 if ego_v>0.5 else 0.0), steer=0.0))
            return
        tx, ty, tz = best
        set_target_world(tx, ty, tz)  # 저장(디버그)

        # Publish suspect GPS data to web dashboard
        self.publish_suspect_gps(tx, ty, tz)

        dx = tx - x; dy = ty - y
        dist = math.hypot(dx, dy)

        # 타겟 속도 추정
        update_tgt_speed_estimate(tx, ty, now)
        tgt_v = get_tgt_speed()

        # hard protection
        if dist < MIN_GAP:
            self.vehicle.apply_control(carla.VehicleControl(
                throttle=0.0, brake=clamp(0.5 + (MIN_GAP-dist)*0.1, 0.0, MAX_BRAKE), steer=0.0
            ))
            return

        la = clamp(0.5*dist + 0.3*max(ego_v,3.0), LA_MIN, LA_MAX)
        steer = self._pure_pursuit(dx, dy, yaw, la)

        desired_gap = max(MIN_GAP, DESIRED_HEADWAY * max(tgt_v, 1.0))
        throttle, brake = self._longitudinal_pi(desired_gap, dist, ego_v, tgt_v)

        if abs(steer) > 0.6:
            throttle *= 0.7
            brake = clamp(brake + 0.05, 0.0, MAX_BRAKE)

        self.vehicle.apply_control(carla.VehicleControl(
            throttle=float(clamp(throttle,0,1)), brake=float(clamp(brake,0,1)), steer=float(clamp(steer,-MAX_STEER,MAX_STEER))
        ))

# ------------------------
# Helpers
# ------------------------
def find_vehicle(world: carla.World) -> Optional[carla.Vehicle]:
    actors = world.get_actors().filter("vehicle.*")
    prefer = []
    rest = []
    for a in actors:
        rn = ""
        try: rn = a.attributes.get("role_name","").lower()
        except: pass
        (prefer if rn in ("hero","chase","police","ego") else rest).append(a)
    return (prefer[0] if prefer else (rest[0] if rest else None))

# ------------------------
# Main
# ------------------------
def main():
    client = carla.Client(CARLA_HOST, CARLA_PORT); client.set_timeout(10.0)
    world  = client.get_world()
    print(f"[CARLA] connected {CARLA_HOST}:{CARLA_PORT}")

    vehicle = find_vehicle(world)
    if not vehicle:
        raise RuntimeError("No vehicle found. Make sure your spawner has created a 'hero' vehicle.")
    print(f"[CARLA] controlling vehicle: {vehicle.type_id} (id={vehicle.id})")
    vehicle.set_autopilot(False)

    z = ZenohRx(); z.open()
    ctrl = Follower(world, vehicle, zenoh_session=z.session)

    print("[RUN] ready. Waiting targets on 'target_gps'. Press Ctrl+C to exit.")
    last_hb = 0.0
    try:
        while True:
            t0 = time.time()
            ctrl.step()

            if (t0 - last_hb) > 2.0:
                v = vehicle.get_velocity().length()
                lk = get_lockon()
                with S.m:
                    n_targets = len(getattr(S, "_last_targets_payload", []) or [])
                    staleness = (t0 - getattr(S, "_last_targets_ts", 0.0)) if n_targets else 9e9
                print(f"[STATE] lock_on={lk}, targets={n_targets}, target_age={staleness:.2f}s, ego_v={v:.1f} m/s")
                last_hb = t0

            # keep 20Hz
            dt = time.time() - t0
            sl = CONTROL_DT - dt
            if sl > 0: time.sleep(sl)
    except KeyboardInterrupt:
        print("\n[RUN] interrupted.")
    finally:
        try: z.close()
        except: pass
        try: vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.6, steer=0.0))
        except: pass
        print("[RUN] done.")

if __name__ == "__main__":
    main()
