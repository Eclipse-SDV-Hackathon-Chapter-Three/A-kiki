# ==============================================
# File: police_control.py (FIXED: use get_target_xyz_or_list, no route viz, 3 cars, exclude hero/police)
# ==============================================
from __future__ import annotations
import carla
import time
import math
from typing import Optional, List, Tuple

from controllers.pid_longitudinal import PIDLongitudinalController
from controllers.stanley_lateral import StanleyLateralController
from waypoints.waypoint_manager import WaypointManager
from waypoints.chase_planner import ChasePlanner
from state.police_state import PoliceState
from police_io.zenoh_interface import ZenohInterface

# ---------- CONFIG ----------
CARLA_HOST = "localhost"
CARLA_PORT = 2000
CONTROL_DT = 0.05  # 20Hz
MAX_BRAKE = 1.0
MAX_THROTTLE = 1.0

# Encirclement formation
RING_RADIUS_MIN = 10.0
RING_RADIUS_MAX = 25.0
RING_RADIUS_DEFAULT = 14.0
TARGET_STALE_TIMEOUT = 1.5  # seconds
ROUTE_REFRESH_NEAR = 0.3
ROUTE_REFRESH_MID = 0.6
ROUTE_REFRESH_FAR = 1.0

# 화면 표시: TARGET 라벨만 유지 (경로/웨이포인트는 표시 안 함)
DRAW_TARGET_LABEL = True

# 타겟 배제(경찰/hero 근접 좌표) 반경
EXCLUDE_RADIUS = 6.0


def connect_carla(host=CARLA_HOST, port=CARLA_PORT, timeout=10.0):
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    return client, world


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _formation_point(cx: float, cy: float, base_radius: float, index: int, n: int, yaw_bias: float = 0.0) -> Tuple[float, float]:
    """
    Evenly distribute n vehicles around a circle centered at (cx,cy).
    yaw_bias allows rotating the ring (e.g., to align with target heading if available).
    """
    angle = (2.0 * math.pi * index / max(1, n)) + yaw_bias
    return cx + base_radius * math.cos(angle), cy + base_radius * math.sin(angle)


def spawn_police_vehicles(world, num_vehicles=3):
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("[Police] ⚠️ 스폰 포인트 없음")
        return []

    blueprint_library = world.get_blueprint_library()
    try:
        police_bp = blueprint_library.filter('vehicle.dodge.charger_police_2020')[0]
        police_bp.set_attribute('role_name', 'police')
    except IndexError:
        print("[Police] ❌ 'vehicle.dodge.charger_police_2020' blueprint not found")
        return []

    import random
    vehicles = []
    for i in range(num_vehicles):
        if not spawn_points:
            break
        sp = random.choice(spawn_points)
        try:
            v = world.spawn_actor(police_bp, sp)
            v.set_autopilot(True)  # start in patrol
            vehicles.append(v)
            spawn_points.remove(sp)
            print(f"[Police] 🚔 {i+1}/{num_vehicles} spawned @({sp.location.x:.1f},{sp.location.y:.1f})")
        except Exception as e:
            print(f"[Police] ❌ spawn fail: {e}")
    return vehicles


def _find_hero(world: carla.World) -> Optional[carla.Actor]:
    """role_name == 'hero' 차량 탐색"""
    try:
        for a in world.get_actors().filter("vehicle.*"):
            rn = (a.attributes.get("role_name", "") or "").lower()
            if rn == "hero":
                return a
    except Exception:
        pass
    return None


def main():
    police_vehicles: List[carla.Vehicle] = []
    zen: Optional[ZenohInterface] = None
    try:
        client, world = connect_carla()
        planner = ChasePlanner(world, sampling_resolution=1.0, enable_emergency_routing=True)

        # 🔕 모든 내부 시각화 무력화 (ChasePlanner의 후보/경로 디버그 출력 차단)
        try:
            planner.visualize_region_points = lambda *args, **kwargs: None
        except Exception:
            pass
        try:
            planner.visualize_route = lambda *args, **kwargs: None
        except Exception:
            pass

        # 3대만 스폰 (컴퓨팅 부담 완화)
        police_vehicles = spawn_police_vehicles(world, num_vehicles=3)
        if not police_vehicles:
            raise RuntimeError("No police vehicles spawned")

        n = len(police_vehicles)
        pids = [PIDLongitudinalController(kp=0.8, ki=0.04, kd=0.15, dt=CONTROL_DT) for _ in range(n)]
        stanleys = [StanleyLateralController(k=1.8, max_steer=1.0) for _ in range(n)]
        wpms = [WaypointManager(arrival_threshold=5.0) for _ in range(n)]
        states = [PoliceState() for _ in range(n)]
        ap_mode = ["patrol"] * n

        # --- Zenoh: real subscription (set use_mock=True for offline test)
        zen = ZenohInterface(use_mock=False)
        lockon_flag = False

        def on_lockon(val: bool):
            nonlocal lockon_flag
            lockon_flag = bool(val)
            print(f"[Zenoh] 🚨 lockon -> {lockon_flag}")

        def on_target(kind_val):
            kind, obj = kind_val
            if kind == "xyz":
                x, y, z = obj
                print(f"[Zenoh] 🎯 target xyz recv: ({x:.2f},{y:.2f},{z:.2f})")
            else:
                print(f"[Zenoh] 🎯 targets list recv: n={len(obj)}")

        zen.subscribe_lockon(on_lockon)
        zen.subscribe_target(on_target)

        # 타겟 좌표 갱신 추적
        last_replan = [0.0] * n
        last_target_xy: Optional[Tuple[float, float]] = None

        while True:
            loop_t0 = time.time()

            # 최신 타겟 가져오기 (⚠️ 이전 코드의 get_target_xyz → get_target_xyz_or_list 로 교체)
            kind_and_val, age = zen.get_target_xyz_or_list(max_age_sec=TARGET_STALE_TIMEOUT)

            # lockon OFF 또는 타겟 없음 → 순찰
            if not lockon_flag or kind_and_val is None:
                for i, v in enumerate(police_vehicles):
                    if v and v.is_alive and ap_mode[i] != "patrol":
                        v.set_autopilot(True)
                        ap_mode[i] = "patrol"
                time.sleep(CONTROL_DT)
                continue

            # 추격 모드 전환
            for i, v in enumerate(police_vehicles):
                if v and v.is_alive and ap_mode[i] != "manual":
                    v.set_autopilot(False)
                    ap_mode[i] = "manual"

            # 경찰/hero 배제 집합 구성
            police_ids = [v.id for v in police_vehicles if v and v.is_alive]
            police_pts = [(v.get_location().x, v.get_location().y) for v in police_vehicles if v and v.is_alive]
            hero = _find_hero(world)
            if hero:
                police_ids.append(hero.id)
                hero_loc = hero.get_location()
                police_pts.append((hero_loc.x, hero_loc.y))

            # 경찰 중심 (targets 선택 힌트)
            if police_pts:
                cx = sum(x for x, _ in police_pts) / len(police_pts)
                cy = sum(y for _, y in police_pts) / len(police_pts)
            else:
                cx = cy = 0.0

            # 단일 world 타겟 좌표 결정
            if kind_and_val[0] == "xyz":
                tx, ty, tz = kind_and_val[1]
                # xyz로 직접 온 경우에도 우리 경찰/hero 근접 좌표면 무시
                if any((tx - px) ** 2 + (ty - py) ** 2 <= (EXCLUDE_RADIUS ** 2) for (px, py) in police_pts):
                    # 가까우면 타겟으로 사용하지 않음 → 순찰 유지
                    for i, v in enumerate(police_vehicles):
                        if v and v.is_alive and ap_mode[i] != "patrol":
                            v.set_autopilot(True)
                            ap_mode[i] = "patrol"
                    time.sleep(CONTROL_DT)
                    continue
            else:
                # targets 리스트에서 hero/police 제외 규칙 적용한 world 좌표 선택
                target_world = zen.pick_world_target(
                    (cx, cy),
                    exclude_ids=police_ids,
                    exclude_roles=["police", "hero", "ego", "chase"],
                    exclude_points_xy=police_pts,
                    exclude_radius=EXCLUDE_RADIUS
                )
                if target_world is None:
                    # 모두 제외되면 순찰 유지
                    for i, v in enumerate(police_vehicles):
                        if v and v.is_alive and ap_mode[i] != "patrol":
                            v.set_autopilot(True)
                            ap_mode[i] = "patrol"
                    time.sleep(CONTROL_DT)
                    continue
                tx, ty, tz = target_world

            # 👉 TARGET 라벨만 표시 (경로/웨이포인트 표시는 하지 않음)
            if DRAW_TARGET_LABEL:
                world.debug.draw_string(
                    carla.Location(tx, ty, tz + 1.5),
                    "TARGET",
                    color=carla.Color(255, 0, 0),
                    life_time=CONTROL_DT * 1.2,
                    persistent_lines=False
                )

            # 포메이션 반경 동적 조정
            dists = []
            for v in police_vehicles:
                if not v or not v.is_alive:
                    dists.append(RING_RADIUS_DEFAULT)
                else:
                    dists.append(v.get_location().distance(carla.Location(tx, ty, tz)))
            mean_d = sum(dists) / max(1, len(dists))
            ring_r = _clamp(mean_d * 0.35, RING_RADIUS_MIN, RING_RADIUS_MAX)

            yaw_bias = 0.0

            # 각 경찰차: 포위 지점 목표 + 경로 갱신 + 제어
            for i, v in enumerate(police_vehicles):
                if not v or not v.is_alive:
                    continue

                # 상태 갱신
                trans = v.get_transform()
                vel = v.get_velocity()
                states[i].update_from_carla_transform(trans, vel)

                # 개인 포위 목표
                gx, gy = _formation_point(tx, ty, ring_r, index=i, n=n, yaw_bias=yaw_bias)
                goal = carla.Location(x=gx, y=gy, z=trans.location.z)

                # 경로 갱신 조건
                now = time.time()
                dist_to_goal = trans.location.distance(goal)
                if dist_to_goal < 20.0:
                    refresh_thr = ROUTE_REFRESH_NEAR
                elif dist_to_goal < 40.0:
                    refresh_thr = ROUTE_REFRESH_MID
                else:
                    refresh_thr = ROUTE_REFRESH_FAR

                moved = False
                if last_target_xy is not None:
                    lx, ly = last_target_xy
                    if math.hypot(tx - lx, ty - ly) > 0.8:
                        moved = True

                if moved or (now - last_replan[i]) > refresh_thr or len(wpms[i].waypoints) < 5:
                    route = planner.plan_route(trans.location, goal, use_optimal=(dist_to_goal > 15.0))
                    if route and len(route) > 2:
                        # 경로/웨이포인트 시각화는 일절 하지 않음
                        # 속도 프로파일
                        if dist_to_goal > 30.0:
                            chase_kmh = 50.0
                        elif dist_to_goal > 15.0:
                            chase_kmh = 40.0
                        else:
                            chase_kmh = 28.0
                        wpms[i].set_waypoints([(x, y, chase_kmh) for (x, y, _) in route])
                        last_replan[i] = now

                # 제어
                target_wp = wpms[i].get_current_target()
                if target_wp is None:
                    v.apply_control(carla.VehicleControl(throttle=0.0, brake=MAX_BRAKE))
                    continue

                steer = stanleys[i].compute_steer(
                    states[i].x, states[i].y, states[i].yaw,
                    [(wp[0], wp[1]) for wp in wpms[i].waypoints],
                    states[i].speed,
                )

                target_speed_ms = (target_wp[2] / 3.6)
                pid_out = pids[i].run_step(target_speed_ms, states[i].speed)
                if pid_out >= 0:
                    throttle = min(MAX_THROTTLE, pid_out)
                    brake = 0.0
                else:
                    throttle = 0.0
                    brake = min(MAX_BRAKE, -pid_out)

                v.apply_control(carla.VehicleControl(steer=float(steer), throttle=float(throttle), brake=float(brake)))
                wpms[i].update_progress(states[i].x, states[i].y)

            last_target_xy = (tx, ty)

            # 루프 주기 유지
            remain = CONTROL_DT - (time.time() - loop_t0)
            if remain > 0:
                time.sleep(remain)

    except (KeyboardInterrupt, SystemExit):
        print("Shutting down police_control (zenoh lockon/target chase)")
    finally:
        print("Destroying vehicles...")
        for v in police_vehicles:
            try:
                if v and v.is_alive:
                    v.destroy()
            except Exception:
                pass
        try:
            if zen:
                zen.close()
        except Exception:
            pass
        print("Cleanup done.")


if __name__ == "__main__":
    main()
