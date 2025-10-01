# ==============================================
# File: police_io/zenoh_interface.py
# ==============================================
from __future__ import annotations
import json
import threading
import time
import os
import math
from typing import Callable, Optional, Tuple, List, Dict, Any

try:
    import zenoh
    _ZENOH_OK = True
except Exception:
    _ZENOH_OK = False

_R_EARTH = 6378137.0
def _enu_from_geodetic(lat: float, lon: float, lat0: float, lon0: float):
    lat_r = math.radians(lat); lon_r = math.radians(lon)
    lat0_r = math.radians(lat0); lon0_r = math.radians(lon0)
    x = _R_EARTH * (lon_r - lon0_r) * math.cos(lat0_r)
    y = _R_EARTH * (lat_r - lat0_r)
    return x, y

def _payload_to_bytes(sample) -> bytes:
    p = getattr(sample, "payload", None)
    if p is None and hasattr(sample, "value"):
        p = getattr(sample.value, "payload", None)
    if p is None: return b""
    for attr in ("to_bytes", "get_payload", "get_value"):
        if hasattr(p, attr):
            try: return getattr(p, attr)()
            except Exception: pass
    try: return bytes(p)
    except Exception: return b""

def _try_parse_xyz_text(s: str):
    ss = s.strip().replace(",", " ").split()
    if len(ss) >= 2:
        try:
            x = float(ss[0]); y = float(ss[1]); z = float(ss[2]) if len(ss) >= 3 else 0.0
            return (x, y, z)
        except Exception:
            return None
    return None

class ZenohInterface:
    """
    RX keys:
      - lock_on (bool-like)
      - target_gps: {"x","y","z"}  or {"timestamp","vehicle_id","targets":[{world?/gps?/role_name/vehicle_id/...}, ...]}
        (raw "x y z" / "x,y,z" 텍스트도 허용)
    """
    def __init__(self, use_mock: bool = False,
                 lockon_key: Optional[str] = None,
                 target_key: Optional[str] = None):
        self._use_mock = use_mock
        self._m = threading.Lock()
        self._lockon = False
        self._lockon_recv_ts = 0.0
        self._target_xyz: Optional[Tuple[float,float,float]] = None
        self._targets_list: Optional[List[Dict[str,Any]]] = None
        self._target_recv_ts = 0.0

        self._lockon_key = lockon_key or os.environ.get("Z_LOCKON_KEY", os.environ.get("ZKEY_LOCKON", "lock_on"))
        self._target_key = target_key or os.environ.get("Z_TARGET_KEY", os.environ.get("ZKEY_TARGET_GPS", "target_gps"))

        self._session = None
        self._sub_lock = None
        self._sub_tgps = None
        self._sub_sniff = None

        self._dbg = os.environ.get("DEBUG_ZENOH", "0") == "1"
        self._dbg_raw = os.environ.get("DEBUG_ZENOH_RAW", "0") == "1"
        self._cnt_lock = 0
        self._cnt_tgps = 0

        self._on_lockon_cb: Optional[Callable[[bool], None]] = None
        self._on_target_cb: Optional[Callable[[Tuple[str, object]], None]] = None

        self._lat0 = float(os.environ.get("ORIGIN_LAT", "0.0")) if os.environ.get("ORIGIN_LAT") else None
        self._lon0 = float(os.environ.get("ORIGIN_LON", "0.0")) if os.environ.get("ORIGIN_LON") else None

        if not self._use_mock:
            if not _ZENOH_OK:
                raise RuntimeError("zenoh not installed; set use_mock=True for dry run")
            self._open_session()
            self._declare_subscribers()
        else:
            print("[ZenohInterface] MOCK mode")

    def set_geo_origin(self, lat0: float, lon0: float):
        with self._m:
            self._lat0, self._lon0 = float(lat0), float(lon0)

    def _open_session(self):
        cfg = zenoh.Config()
        ep_env = os.environ.get("Z_ENDPOINT", "tcp/127.0.0.1:7447")
        if ep_env:
            eps = "[" + ",".join(f'"{ep.strip()}"' for ep in ep_env.split(",")) + "]"
            cfg.insert_json5("connect/endpoints", eps)
        self._session = zenoh.open(cfg)
        print(f"[Zenoh] connected -> {ep_env}")
        print(f"[Zenoh] topics: lock_on='{self._lockon_key}', target='{self._target_key}'")

    def _declare_subscribers(self):
        assert self._session is not None

        def on_lockon(sample):
            try:
                raw = _payload_to_bytes(sample)
                if self._dbg_raw:
                    k = getattr(sample, "key_expr", None) or getattr(sample, "key", None) or "<?>"
                    print(f"[zenoh RAW] {k} -> {raw[:120]!r}{'...' if len(raw)>120 else ''}")
                s = raw.decode("utf-8", errors="ignore").strip()
                val = None
                if s.startswith("{"):
                    d = json.loads(s)
                    for k in ("lockon", "lock_on", "value"):
                        if k in d: val = bool(d[k]); break
                else:
                    ls = s.lower()
                    if ls in ("1","true","t","yes","y","on"): val = True
                    elif ls in ("0","false","f","no","n","off"): val = False
                if val is None:
                    try: val = bool(json.loads(s))
                    except Exception: val = False
                with self._m:
                    self._lockon = bool(val); self._cnt_lock += 1; self._lockon_recv_ts = time.time()
                if self._dbg: print(f"[zenoh<-lock_on] cnt={self._cnt_lock} val={self._lockon}")
                if self._on_lockon_cb: self._on_lockon_cb(self._lockon)
            except Exception as e:
                print("[Zenoh] lock_on parse error:", e)

        def on_target(sample):
            try:
                raw = _payload_to_bytes(sample)
                if self._dbg_raw:
                    k = getattr(sample, "key_expr", None) or getattr(sample, "key", None) or "<?>"
                    print(f"[zenoh RAW] {k} -> {raw[:120]!r}{'...' if len(raw)>120 else ''}")
                s = raw.decode("utf-8", errors="ignore").strip()
                xyz = None; targets = None
                try:
                    d = json.loads(s)
                    if all(k in d for k in ("x","y")):
                        xyz = (float(d["x"]), float(d["y"]), float(d.get("z", 0.0)))
                    elif isinstance(d.get("targets"), list):
                        targets = d["targets"]
                except Exception:
                    xyz = _try_parse_xyz_text(s)

                with self._m:
                    self._target_xyz = xyz
                    self._targets_list = targets
                    self._cnt_tgps += 1
                    self._target_recv_ts = time.time()
                if self._dbg:
                    if xyz is not None:
                        print(f"[zenoh<-target_gps] cnt={self._cnt_tgps} xyz=({xyz[0]:.2f},{xyz[1]:.2f},{xyz[2]:.2f})")
                    elif targets is not None:
                        print(f"[zenoh<-target_gps] cnt={self._cnt_tgps} targets[{len(targets)}]")
                    else:
                        print(f"[zenoh<-target_gps] cnt={self._cnt_tgps} (unknown format)")
                if self._on_target_cb:
                    self._on_target_cb(("xyz", xyz) if xyz is not None else ("targets", targets or []))
            except Exception as e:
                print("[Zenoh] target_gps parse error:", e)

        self._sub_lock = self._session.declare_subscriber(self._lockon_key, on_lockon)
        self._sub_tgps = self._session.declare_subscriber(self._target_key, on_target)

        sniff = os.environ.get("Z_SNIFF", "")
        if sniff:
            def on_any(sample):
                raw = _payload_to_bytes(sample)
                k = getattr(sample, "key_expr", None) or getattr(sample, "key", None) or "<?>"
                try: txt = raw.decode("utf-8", errors="replace")
                except Exception: txt = str(raw)
                print(f"[SNIFF:{k}] {txt[:200]}")
            self._sub_sniff = self._session.declare_subscriber(sniff, on_any)
            print(f"[Zenoh] SNIFF subscribe -> '{sniff}'")

    # API
    def subscribe_lockon(self, cb: Callable[[bool], None]): self._on_lockon_cb = cb
    def subscribe_target(self, cb: Callable[[Tuple[str, object]], None]): self._on_target_cb = cb
    def get_lockon(self) -> bool:
        with self._m: return bool(self._lockon)
    def get_stats(self):
        with self._m:
            return {
                "lockon_count": self._cnt_lock, "target_count": self._cnt_tgps,
                "last_lockon_age": (time.time()-self._lockon_recv_ts) if self._lockon_recv_ts else None,
                "last_target_age": (time.time()-self._target_recv_ts) if self._target_recv_ts else None,
            }
    def get_target_xyz_or_list(self, max_age_sec: float = 1.5):
        now = time.time()
        with self._m:
            age = now - self._target_recv_ts if self._target_recv_ts else 1e9
            if age > max_age_sec: return None, age
            if self._target_xyz is not None: return ("xyz", self._target_xyz), age
            if self._targets_list is not None: return ("targets", self._targets_list), age
            return None, age

    def pick_world_target(self,
                          police_centroid_xy: Tuple[float,float],
                          exclude_ids: Optional[List[int]] = None,
                          exclude_roles: Optional[List[str]] = None,
                          exclude_points_xy: Optional[List[Tuple[float,float]]] = None,
                          exclude_radius: float = 6.0
                          ) -> Optional[Tuple[float,float,float]]:
        """우리 경찰/hero를 타겟에서 제외하여 단일 world 타겟 선택."""
        with self._m:
            if self._target_xyz is not None:
                # xyz로 온 경우: 후단에서 좌표 근접 배제 필요(신원정보 없음)
                return self._target_xyz
            targets = self._targets_list

        if not targets: return None

        ex_ids = set(exclude_ids or [])
        ex_roles = {r.lower() for r in (exclude_roles or [])}
        ex_pts = exclude_points_xy or []

        def is_excluded_by_meta(t: dict) -> bool:
            # id 매칭
            for key in ("vehicle_id", "id", "actor_id"):
                if key in t:
                    try:
                        if int(t[key]) in ex_ids:
                            return True
                    except Exception:
                        pass
            # role_name 매칭
            rn = str(t.get("role_name", "")).lower()
            if rn and rn in ex_roles:
                return True
            return False

        def is_excluded_by_proximity(t: dict) -> bool:
            w = t.get("world")
            if not isinstance(w, dict): return False
            tx, ty = float(w.get("x", 1e9)), float(w.get("y", 1e9))
            for (px, py) in ex_pts:
                if (tx - px)**2 + (ty - py)**2 <= (exclude_radius**2):
                    return True
            return False

        # 1) 차량(semantic_label==14) 우선 + 배제 적용
        cars = []
        others = []
        for t in targets:
            try:
                if is_excluded_by_meta(t) or is_excluded_by_proximity(t):
                    continue
                if int(t.get("semantic_label", -1)) == 14:
                    cars.append(t)
                else:
                    others.append(t)
            except Exception:
                continue

        cand = cars if cars else others
        if not cand:  # 모두 제외되었으면 포기
            return None

        # 2) world 우선(경찰 중심과 최근접)
        with_world = [t for t in cand if isinstance(t.get("world"), dict)]
        if with_world:
            cx, cy = police_centroid_xy
            def d2(t):
                w = t["world"]; return (float(w["x"])-cx)**2 + (float(w["y"])-cy)**2
            best = min(with_world, key=d2)
            w = best["world"]
            return (float(w["x"]), float(w["y"]), float(w.get("z", 0.0)))

        # 3) gps fallback
        with_gps = [t for t in cand if isinstance(t.get("gps"), dict)]
        if with_gps and (self._lat0 is not None) and (self._lon0 is not None):
            g = with_gps[0]["gps"]
            if ("lat" in g or "latitude" in g) and ("lon" in g or "longitude" in g):
                lat = float(g.get("lat", g.get("latitude")))
                lon = float(g.get("lon", g.get("longitude")))
                alt = float(g.get("alt", g.get("altitude", 0.0)))
                ex, ny = _enu_from_geodetic(lat, lon, self._lat0, self._lon0)
                return (ex, ny, alt)

        return None

    # mocks
    def mock_set_lockon(self, val: bool):
        with self._m:
            self._lockon = bool(val); self._lockon_recv_ts = time.time()
    def mock_set_target_xyz(self, xyz: Tuple[float,float,float]):
        with self._m:
            self._target_xyz = tuple(xyz); self._targets_list = None; self._target_recv_ts = time.time()

    def close(self):
        try:
            if self._sub_lock: self._sub_lock.undeclare()
        except Exception: pass
        try:
            if self._sub_tgps: self._sub_tgps.undeclare()
        except Exception: pass
        try:
            if self._sub_sniff: self._sub_sniff.undeclare()
        except Exception: pass
        try:
            if self._session: self._session.close()
        except Exception as e:
            print("[Zenoh] close error:", e)
