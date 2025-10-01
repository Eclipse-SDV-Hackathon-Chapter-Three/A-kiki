import carla
import time
import threading
import tkinter as tk
from controllers.pid_longitudinal import PIDLongitudinalController
from controllers.stanley_lateral import StanleyLateralController
from state.police_state import PoliceState
from waypoints.chase_planner import ChasePlanner
from waypoints.waypoint_manager import WaypointManager

CARLA_HOST = "localhost"
CARLA_PORT = 2000
CONTROL_DT = 0.05
MAX_BRAKE = 1.0
MAX_THROTTLE = 1.0
TARGET_SPEED = 30.0  # GTA 스타일 기본 속도 증가

# GUI for displaying calculation time
def setup_gui():
    root = tk.Tk()
    root.title("Lockon Time")
    root.geometry("300x100")
    time_label = tk.Label(root, text="Time: 0.0 ms", font=("Helvetica", 16))
    time_label.pack(pady=20)
    return root, time_label

def update_gui(root, label, calc_time_ms):
    if root.winfo_exists():
        label.config(text=f"Time: {calc_time_ms:.2f} ms")
        root.update_idletasks()
        root.update()

def visualize_planner_route(world, route, life_time=60.0):
    for loc in route:
        carla_loc = carla.Location(x=loc[0], y=loc[1], z=loc[2]+0.5)
        world.debug.draw_string(
            carla_loc, 'O', draw_shadow=False,
            color=carla.Color(0,255,0), life_time=life_time,
            persistent_lines=True
        )

# ---------- MAIN ----------
def main():
    gui_root, time_label = setup_gui()
    vehicles = []
    actors = []
    planners = []
    
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(10.0)
    world = client.load_world("Town02")
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("vehicle.dodge.charger_police_2020")[0]

    spawn_points = [
        world.get_map().get_spawn_points()[10],
        world.get_map().get_spawn_points()[11],
        world.get_map().get_spawn_points()[12],
        world.get_map().get_spawn_points()[13],
    ]

    for spawn_point in spawn_points:
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle:
            actors.append(vehicle)
            vehicles.append({
                "ego": vehicle,
                "pid": PIDLongitudinalController(kp=0.8, ki=0.04, kd=0.15, dt=CONTROL_DT),  # 🚨 GTA 스타일 공격적 PID
                "stanley": StanleyLateralController(),  # 더 공격적인 조향
                "state": PoliceState(),
                "wpm": WaypointManager(),
                "new_lockon": False,
                "update_waypoints": False,
                "lockon_time": None,
                "last_route_update": 0,  # 마지막 경로 업데이트 시간 추적
                "ramming_mode": False,   # 🚨 램밍 모드 플래그
                "last_target_distance": 100.0  # 마지막 타겟 거리
            })
            planners.append(ChasePlanner(world, sampling_resolution=1.0))
            print(f"[CARLA] Vehicle spawned at: {vehicle.get_transform()}")
        else:
            print(f"Could not spawn vehicle at {spawn_point}")

    if not vehicles:
        print("No vehicles spawned. Exiting.")
        return

    # ----------------------------
    # 🎯 자율주행 타겟 차량 시스템
    target_vehicle = None
    last_target_position = None

    def spawn_autonomous_target():
        """자율주행 타겟 차량 스폰"""
        nonlocal target_vehicle
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            return None

        import random
        spawn_point = random.choice(spawn_points)

        # 일반 차량 블루프린트 선택
        civilian_bps = [bp for bp in blueprint_library.filter('vehicle.*')
                       if 'police' not in bp.id.lower()]
        if not civilian_bps:
            return None

        target_bp = random.choice(civilian_bps)
        target_bp.set_attribute('role_name', 'target_vehicle')

        try:
            target_vehicle = world.spawn_actor(target_bp, spawn_point)
            target_vehicle.set_autopilot(True)  # 자율주행 활성화
            print(f"[TARGET] Spawned autonomous target: {target_bp.id}")
            print(f"[TARGET] Location: ({spawn_point.location.x:.1f}, {spawn_point.location.y:.1f})")
            return target_vehicle
        except Exception as e:
            print(f"[TARGET] Failed to spawn: {e}")
            return None
    # ----------------------------

    # Mock Zenoh setup with target spawning
    class MockZenoh:
        def __init__(self, vehicle_list):
            self.vehicle_list = vehicle_list
            self.lockon_triggered = False

        def publish_mock_data(self):
            if not self.lockon_triggered:
                print("[MockZenoh] 🎯 Spawning autonomous target and activating chase mode!")

                # 자율주행 타겟 스폰
                if spawn_autonomous_target():
                    print("[MockZenoh] ✅ Target spawned successfully")
                else:
                    print("[MockZenoh] ❌ Failed to spawn target")

                # 모든 경찰차에 락온 신호 전송
                for v in self.vehicle_list:
                    v["state"].lockon = True
                    v["new_lockon"] = True
                    v["lockon_time"] = time.time()

                self.lockon_triggered = True

    zen = MockZenoh(vehicles)

    try:
        t_start = time.time()
        while True:
            loop_start = time.time()

            # After 10 seconds, send mock lockon signal
            if time.time() - t_start > 10:
                zen.publish_mock_data()
                t_start = float('inf')

            for i, v in enumerate(vehicles):
                ego = v["ego"]
                state = v["state"]
                wpm = v["wpm"]
                
                # Update vehicle state
                trans = ego.get_transform()
                vel = ego.get_velocity()
                state.update_from_carla_transform(trans, vel)

                # 🎯 실시간 타겟 추적 시스템
                if state.lockon and target_vehicle and target_vehicle.is_alive:
                    origin = ego.get_location()
                    current_target_location = target_vehicle.get_location()

                    # 🚨 GTA 스타일 충돌 감지 (더 관대하게)
                    distance_to_target = ((current_target_location.x - origin.x) ** 2 +
                                        (current_target_location.y - origin.y) ** 2) ** 0.5

                    # 🎯 타겟 속도 벡터 계산 (예측 추적)
                    target_velocity = target_vehicle.get_velocity()
                    target_speed = (target_velocity.x ** 2 + target_velocity.y ** 2) ** 0.5

                    if distance_to_target < 7.0:  # 7m 이내 충돌! (더 관대하게)
                        print(f"🚨 [Vehicle {i}] TARGET HIT! Distance: {distance_to_target:.1f}m")
                        print(f"🎯 [Vehicle {i}] Destroying target and spawning new one...")
                        target_vehicle.destroy()
                        last_target_position = None  # 위치 초기화
                        time.sleep(2.0)  # 잠시 대기
                        spawn_autonomous_target()
                        continue

                    # 🎯 GTA 스타일 예측 추적 (타겟이 어디로 갈지 예측)
                    target_moved = False
                    predicted_location = current_target_location

                    if last_target_position:
                        target_movement = ((current_target_location.x - last_target_position.x) ** 2 +
                                         (current_target_location.y - last_target_position.y) ** 2) ** 0.5

                        # 🚀 예측 시간 (거리에 따라 조절)
                        prediction_time = min(3.0, distance_to_target / 20.0)  # 3초까지 예측
                        predicted_x = current_target_location.x + target_velocity.x * prediction_time
                        predicted_y = current_target_location.y + target_velocity.y * prediction_time
                        predicted_location = carla.Location(predicted_x, predicted_y, current_target_location.z)

                        if target_movement > 0.8:  # 0.8m 이상 이동했으면 (매우 민감)
                            target_moved = True
                            print(f"[Vehicle {i}] 📍 Target moved {target_movement:.1f}m, speed: {target_speed:.1f}m/s - INTERCEPTING!")
                            print(f"[Vehicle {i}] 🎯 Predicted position: ({predicted_x:.1f}, {predicted_y:.1f})")

                    last_target_position = current_target_location

                    # 🚨 GTA 스타일 극한 추적 모드 (거의 매 프레임)
                    time_since_last_update = time.time() - v.get("last_route_update", 0)

                    # 🎯 거리별 공격성 조절
                    if distance_to_target < 15.0:  # 15m 이내면 극공격 모드
                        update_threshold = 0.3  # 0.3초마다 업데이트 (초고속)
                        waypoint_threshold = 3   # 3개 미만이면 업데이트
                    elif distance_to_target < 30.0:  # 30m 이내면 고공격 모드
                        update_threshold = 0.5  # 0.5초마다
                        waypoint_threshold = 5
                    else:  # 멀면 일반 모드
                        update_threshold = 1.0
                        waypoint_threshold = 8

                    should_update = (v["new_lockon"] or
                                   v["update_waypoints"] or
                                   target_moved or                           # 타겟이 0.8m 이상 움직였으면
                                   distance_to_target > 5.0 or             # 5m 이상 멀면 업데이트 (매우 민감)
                                   len(wpm.waypoints) == 0 or              # 웨이포인트 없으면 업데이트
                                   len(wpm.waypoints) < waypoint_threshold or # 거리별 임계값
                                   time_since_last_update > update_threshold) # 거리별 시간 임계값

                    if should_update:
                        print(f"[Vehicle {i}] 🚨 GTA PURSUIT UPDATE - Reason: target_moved={target_moved}, dist={distance_to_target:.1f}m, waypoints={len(wpm.waypoints)}, time_since_update={time_since_last_update:.1f}s, mode={'RAMMING' if distance_to_target < 15.0 else 'CHASE'}")

                        # 🗑️ 기존 웨이포인트 즉시 제거 (늦은 업데이트 방지)
                        waypoints_before_clear = len(wpm.waypoints)
                        wpm.waypoints.clear()
                        print(f"[Vehicle {i}] 🧹 CLEARED {waypoints_before_clear} old waypoints → 0")

                        # --- Timing Start ---
                        calc_start_time = time.time()
                        # 🎯 GTA 스타일 타겟 선택 (현재 위치 vs 예측 위치)
                        target_for_planning = predicted_location if target_speed > 2.0 else current_target_location
                        distance_to_plan = ((target_for_planning.x - origin.x) ** 2 + (target_for_planning.y - origin.y) ** 2) ** 0.5

                        print(f"[Vehicle {i}] 🎯 GTA PURSUIT: ({origin.x:.1f},{origin.y:.1f}) → ({'PREDICTED' if target_speed > 2.0 else 'CURRENT'}: {target_for_planning.x:.1f},{target_for_planning.y:.1f}), distance: {distance_to_plan:.1f}m")

                        # 🚀 거리별 경로 전략
                        if distance_to_plan < 20.0:  # 20m 이내면 직선 공격
                            success = planners[i].plan_route(origin, target_for_planning, use_optimal=False)  # 단순 직선 경로
                            print(f"[Vehicle {i}] 🚨 DIRECT ASSAULT MODE - straight line attack!")
                        else:  # 멀면 최적 경로
                            success = planners[i].plan_route(origin, target_for_planning, use_optimal=True)
                        # --- Timing End ---
                        calc_end_time = time.time()
                        print(f"[Vehicle {i}] 📊 ROUTE PLANNING: {'SUCCESS' if success else 'FAILED'} in {(calc_end_time - calc_start_time)*1000:.1f}ms")

                        calc_time_ms = (calc_end_time - calc_start_time) * 1000
                        update_gui(gui_root, time_label, calc_time_ms)

                        route = planners[i].get_route()
                        print(f"[Vehicle {i}] 📍 ROUTE RECEIVED: {len(route) if route else 0} waypoints")
                        if route and len(route) > 2:
                            # 🚨 GTA 스타일 극한 속도 (타겟을 무조건 잡는다!)
                            if distance_to_target < 10.0:  # 10m 이내면 램밍 모드
                                chase_speed = min(60.0, 40.0 + (10.0 - distance_to_target) * 2.0)  # 램밍 속도!
                                print(f"[Vehicle {i}] 💥 RAMMING SPEED ACTIVATED: {chase_speed:.1f} km/h")
                            elif distance_to_target < 25.0:  # 25m 이내면 고속 추격
                                chase_speed = min(45.0, 25.0 + distance_to_target * 0.8)
                            else:  # 멀면 일반 추격
                                chase_speed = min(35.0, 20.0 + distance_to_target * 0.3)

                            new_waypoints = [(x, y, chase_speed) for x, y, _ in route]

                            # 웨이포인트 설정 전 확인
                            print(f"[Vehicle {i}] 📋 SETTING WAYPOINTS: {len(new_waypoints)} points, speed: {chase_speed:.1f} km/h")
                            wpm.set_waypoints(new_waypoints)

                            # 즉시 확인
                            actual_waypoints = len(wpm.waypoints)
                            if actual_waypoints != len(new_waypoints):
                                print(f"[Vehicle {i}] ⚠️ WAYPOINT MISMATCH: expected {len(new_waypoints)}, got {actual_waypoints}")

                            print(f"[Vehicle {i}] 🚨 GTA PURSUIT ACTIVE: {len(new_waypoints)} waypoints, 💥SPEED: {chase_speed:.1f} km/h, calc: {calc_time_ms:.1f}ms")
                            print(f"[Vehicle {i}] 🎯 Target: ({current_target_location.x:.1f}, {current_target_location.y:.1f}) [Speed: {target_speed:.1f}m/s]")
                            print(f"[Vehicle {i}] 🚀 Attack vector: ({new_waypoints[0][0]:.1f}, {new_waypoints[0][1]:.1f})")

                            # 마지막 업데이트 시간 기록
                            v["last_route_update"] = time.time()

                            # 🚨 GTA 스타일 시각 효과
                            if distance_to_target < 15.0:  # 가까우면 빨간 경로
                                visualize_planner_route(world, route, life_time=2.0)
                                world.debug.draw_string(
                                    current_target_location + carla.Location(z=3.0), "💥 RAMMING TARGET", draw_shadow=True,
                                    color=carla.Color(r=255, g=0, b=0), life_time=2.0, persistent_lines=False
                                )
                                # 공격 라인 그리기
                                world.debug.draw_line(
                                    ego.get_location() + carla.Location(z=1.0),
                                    current_target_location + carla.Location(z=1.0),
                                    thickness=0.3, color=carla.Color(r=255, g=0, b=0), life_time=1.0
                                )
                            else:  # 멀면 초록 추적
                                visualize_planner_route(world, route, life_time=3.0)
                                world.debug.draw_string(
                                    current_target_location + carla.Location(z=2.0), "🎯 PURSUIT TARGET", draw_shadow=True,
                                    color=carla.Color(r=0, g=255, b=0), life_time=1.5, persistent_lines=False
                                )
                        else:
                            print(f"[Vehicle {i}] ❌ ROUTE PLANNING FAILED! Route length: {len(route) if route else 0}")
                            # 실패시 기존 웨이포인트도 클리어 (무한 루프 방지)
                            if len(wpm.waypoints) > 0:
                                print(f"[Vehicle {i}] 🗑️ CLEARING FAILED ROUTE waypoints: {len(wpm.waypoints)}")
                                wpm.waypoints.clear()

                        v["new_lockon"] = False
                        v["update_waypoints"] = False
                        # 업데이트 완료 시간 기록 (실패 시에도)
                        v["last_route_update"] = time.time()
                    else:
                        # 업데이트 안 할 때도 상태 로그 (더 자세히)
                        print(f"[Vehicle {i}] ⏸️ No update - dist: {distance_to_target:.1f}m, waypoints: {len(wpm.waypoints)}, new_lockon: {v['new_lockon']}, time_since_update: {time_since_last_update:.1f}s")

                elif v["new_lockon"]:  # 타겟이 없을 때만
                    print(f"[Vehicle {i}] ⚠️ No target available for tracking")
                    v["new_lockon"] = False

                # Autopilot if no lockon
                if not state.lockon:
                    ego.set_autopilot(True)
                    continue
                else:
                    ego.set_autopilot(False)

                target = wpm.get_current_target()
                if target is None:
                    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=MAX_BRAKE, steer=0.0))
                    continue

                # 🚨 GTA 스타일 컨트롤 계산 (공격적 제어)
                target = wpm.get_current_target()
                current_target_speed = target[2] if target else TARGET_SPEED

                # 🚨 램밍 모드 감지 및 활성화
                if target_vehicle and target_vehicle.is_alive:
                    current_distance = ((target_vehicle.get_location().x - state.x) ** 2 +
                                      (target_vehicle.get_location().y - state.y) ** 2) ** 0.5

                    # 램밍 모드 조건: 10m 이내 + 접근 중
                    if current_distance < 10.0 and current_distance < v["last_target_distance"]:
                        v["ramming_mode"] = True
                        print(f"[Vehicle {i}] 💥 RAMMING MODE ACTIVATED! Distance: {current_distance:.1f}m")
                    elif current_distance > 15.0:
                        v["ramming_mode"] = False

                    v["last_target_distance"] = current_distance

                # Stanley 제어 (램밍 모드에서 더 공격적)
                steer = v["stanley"].compute_steer(
                    state.x, state.y, state.yaw,
                    [(wp[0], wp[1]) for wp in wpm.waypoints],
                    state.speed,
                    lockon_time=v["lockon_time"]
                )

                # 🚨 램밍 모드에서 조향 증폭
                if v["ramming_mode"]:
                    steer *= 1.5  # 조향 증폭
                    current_target_speed = min(current_target_speed * 1.8, 70.0)  # 속도 증폭

                # PID 제어 (램밍 모드에서 더 공격적)
                pid_out = v["pid"].run_step(
                    current_target_speed, state.speed,
                    lockon_time=v["lockon_time"]
                )

                # 🚨 램밍 모드에서 스로틀 증폭
                if v["ramming_mode"] and pid_out >= 0:
                    throttle = min(MAX_THROTTLE, pid_out * 1.3)  # 스로틀 30% 증폭
                    brake = 0.0
                else:
                    throttle = min(MAX_THROTTLE, pid_out) if pid_out >= 0 else 0.0
                    brake = min(MAX_BRAKE, -pid_out) if pid_out < 0 else 0.0

                control = carla.VehicleControl(steer=float(steer), throttle=float(throttle), brake=float(brake))
                ego.apply_control(control)

                # 웨이포인트 진행 상황 모니터링 (더 자세히)
                waypoints_before = len(wpm.waypoints)
                current_target = wpm.get_current_target()
                target_distance = 0.0
                if current_target:
                    target_distance = ((current_target[0] - state.x) ** 2 + (current_target[1] - state.y) ** 2) ** 0.5
                    print(f"[Vehicle {i}] 🎯 Current target: ({current_target[0]:.1f},{current_target[1]:.1f}), distance: {target_distance:.1f}m")

                wpm.update_progress(state.x, state.y)
                waypoints_after = len(wpm.waypoints)

                if waypoints_before != waypoints_after:
                    print(f"[Vehicle {i}] 📍 Waypoint REACHED! {waypoints_before} → {waypoints_after} remaining")
                elif waypoints_before > 0 and current_target:
                    print(f"[Vehicle {i}] 🚗 Following waypoint: {waypoints_before} remaining, target_dist: {target_distance:.1f}m")

                # 🎯 움직이는 타겟 추적 중에는 웨이포인트 도착 감지 무시
                # (타겟이 계속 움직이므로 절대 "최종 도착"하지 않음)
                if not (state.lockon and target_vehicle and target_vehicle.is_alive):
                    # 일반적인 고정 목적지일 때만 도착 감지
                    if wpm.is_final_reached(state.x, state.y):
                        ego.apply_control(carla.VehicleControl(throttle=0.0, brake=MAX_BRAKE, steer=0.0))
                        continue

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, CONTROL_DT - elapsed))

    except KeyboardInterrupt:
        print("Shutting down.")
    finally:
        if 'gui_root' in locals() and gui_root.winfo_exists():
            gui_root.destroy()
        # 모든 액터 정리 (경찰차 + 타겟 차량)
        if target_vehicle and target_vehicle.is_alive:
            target_vehicle.destroy()
            print("Target vehicle destroyed.")
        for actor in actors:
            actor.destroy()
        print("All actors destroyed.")

if __name__ == "__main__":
    main()