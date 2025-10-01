# police_control_mock_multi_spawn.py
import carla
import time
import random
import math
def calculate_surround_positions(truck, prediction_time=3.0):
    """
    🚔 트럭 포위 작전: 4대의 경찰차가 트럭을 둘러싸는 위치 계산
    Args:
        truck: CARLA Vehicle 객체
        prediction_time: 예측할 시간 (초)
    Returns:
        list: 4개의 포위 위치 [앞, 뒤, 좌, 우]
    """
    current_loc = truck.get_location()
    velocity = truck.get_velocity()
    
    # 트럭 예측 위치 계산
    predicted_x = current_loc.x + velocity.x * prediction_time
    predicted_y = current_loc.y + velocity.y * prediction_time
    predicted_z = current_loc.z
    predicted_loc = carla.Location(x=predicted_x, y=predicted_y, z=predicted_z)
    
    # 트럭의 진행 방향 벡터 계산
    truck_transform = truck.get_transform()
    forward_vector = truck_transform.get_forward_vector()
    right_vector = truck_transform.get_right_vector()
    
    # 포위 거리 설정 (동적 조정)
    truck_speed = math.sqrt(velocity.x**2 + velocity.y**2)
    # 트럭 속도에 따라 포위 거리 조정 (빠를수록 멀리, 느릴수록 가깝게)
    base_distance = 20.0
    speed_factor = max(0.5, min(2.0, truck_speed / 10.0))  # 0.5~2.0배 조정
    surround_distance = base_distance * speed_factor
    
    print(f"[Surround] 트럭 속도: {truck_speed:.1f}m/s, 포위 거리: {surround_distance:.1f}m")
    
    # 🚔 4방향 포위 위치 계산
    positions = []
    
    # 1. 전방 (트럭 진행방향 앞쪽)
    front_pos = carla.Location(
        x=predicted_loc.x + forward_vector.x * surround_distance,
        y=predicted_loc.y + forward_vector.y * surround_distance,
        z=predicted_loc.z
    )
    positions.append(front_pos)
    
    # 2. 후방 (트럭 진행방향 뒤쪽)
    rear_pos = carla.Location(
        x=predicted_loc.x - forward_vector.x * surround_distance,
        y=predicted_loc.y - forward_vector.y * surround_distance,
        z=predicted_loc.z
    )
    positions.append(rear_pos)
    
    # 3. 좌측 (트럭 기준 왼쪽)
    left_pos = carla.Location(
        x=predicted_loc.x - right_vector.x * surround_distance,
        y=predicted_loc.y - right_vector.y * surround_distance,
        z=predicted_loc.z
    )
    positions.append(left_pos)
    
    # 4. 우측 (트럭 기준 오른쪽)
    right_pos = carla.Location(
        x=predicted_loc.x + right_vector.x * surround_distance,
        y=predicted_loc.y + right_vector.y * surround_distance,
        z=predicted_loc.z
    )
    positions.append(right_pos)
    
    return positions

def predict_target_location(truck, prediction_time=3.0, police_location=None):
    """
    트럭의 현재 위치와 속도를 기반으로 예측 위치 계산 (호환성용)
    """
    current_loc = truck.get_location()
    velocity = truck.get_velocity()
    
    if police_location:
        distance = math.hypot(current_loc.x - police_location.x, current_loc.y - police_location.y)
        prediction_time = min(5.0, max(2.0, distance / 20.0))
    
    predicted_x = current_loc.x + velocity.x * prediction_time
    predicted_y = current_loc.y + velocity.y * prediction_time
    predicted_z = current_loc.z
    
    return carla.Location(x=predicted_x, y=predicted_y, z=predicted_z)

def visualize_planner_route(world, route, life_time=60.0):
    for loc in route:
        carla_loc = carla.Location(x=loc[0], y=loc[1], z=loc[2] + 0.5)
        world.debug.draw_string(
            carla_loc,
            "O",
            draw_shadow=False,
            color=carla.Color(0, 255, 0),
            life_time=life_time,
            persistent_lines=True,
        )
from controllers.pid_longitudinal import PIDLongitudinalController
from controllers.stanley_lateral import StanleyLateralController
from controllers.vehicle_unstuck import VehicleUnstuck
from state.police_state import PoliceState
from waypoints.chase_planner import ChasePlanner
from waypoints.waypoint_manager import WaypointManager

CARLA_HOST = "localhost"
CARLA_PORT = 2000
CONTROL_DT = 0.05
MAX_BRAKE = 1.0
MAX_THROTTLE = 1.0
TARGET_SPEED = 15.0
NUM_POLICE = 4

# ---------- MOCK ZENOH ----------
class MockZenoh:
    def __init__(self):
        self.lockon_cb = None
        self.waypoints_cb = None

    def subscribe_lockon(self, cb):
        self.lockon_cb = cb

    def subscribe_waypoints(self, cb):
        self.waypoints_cb = cb

    def publish_mock_data(self):
        if self.lockon_cb:
            self.lockon_cb(True)
        if self.waypoints_cb:
            self.waypoints_cb([{'x': 100.0, 'y': 100.0, 'v': 10.0}])

# ---------- CARLA HELPERS ----------
def connect_carla(host=CARLA_HOST, port=CARLA_PORT, timeout=10.0):
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    #world = client.load_world("Town02")
    world = client.get_world()
    return client, world

def spawn_police_vehicle(world, spawn_point):
    blueprint_library = world.get_blueprint_library()
    police_candidates = [
        "vehicle.dodge.charger_police_2020",
        "vehicle.dodge.charger_police",
        "vehicle.crown.crown_police"
    ]
    police_bp = None
    for cand in police_candidates:
        bps = blueprint_library.filter(cand)
        if bps:
            police_bp = bps[0]
            police_bp.set_attribute("role_name", "police")
            break
    if not police_bp:
        police_bp = blueprint_library.find("vehicle.tesla.model3")
    vehicle = world.spawn_actor(police_bp, spawn_point)
    return vehicle

def spawn_truck(world, spawn_point):
    """트럭을 스폰하고 autopilot 모드로 설정"""
    blueprint_library = world.get_blueprint_library()
    truck_candidates = [
        "vehicle.carlamotors.carlacola",
        "vehicle.tesla.cybertruck",
        "vehicle.carlamotors.firetruck"
    ]
    truck_bp = None
    for cand in truck_candidates:
        bps = blueprint_library.filter(cand)
        if bps:
            truck_bp = bps[0]
            truck_bp.set_attribute("role_name", "target_truck")
            print(f"[Truck] 🚛 트럭 블루프린트 사용: {cand}")
            break
    if not truck_bp:
        truck_bp = blueprint_library.find("vehicle.volkswagen.t2")
    
    truck = world.spawn_actor(truck_bp, spawn_point)
    truck.set_autopilot(True)  # 트럭은 autopilot으로 돌아다님
    print(f"[Truck] 트럭이 autopilot 모드로 스폰됨: {spawn_point}")
    return truck

def visualize_planner_route(world, route, life_time=5.0):
    for loc in route:
        carla_loc = carla.Location(x=loc[0], y=loc[1], z=loc[2] + 0.5)
        world.debug.draw_string(
            carla_loc,
            "O",
            draw_shadow=False,
            color=carla.Color(0, 255, 0),
            life_time=life_time,
            persistent_lines=True,
        )

# ---------- MAIN ----------
def main():
    client, world = connect_carla()
    available_spawn_points = world.get_map().get_spawn_points()
    print(f"[System] 사용 가능한 스폰 포인트: {len(available_spawn_points)}개")
    
    if len(available_spawn_points) < NUM_POLICE + 1:
        print(f"[Error] 스폰 포인트가 부족합니다. 필요: {NUM_POLICE + 1}, 사용가능: {len(available_spawn_points)}")
        return
    
    random.shuffle(available_spawn_points)  # 랜덤 섞기

    # 4대 경찰차 생성 (서로 다른 위치)
    police_vehicles = []
    for i in range(NUM_POLICE):
        spawn_point = available_spawn_points[i]
        print(f"[System] Police {i} 스폰 시도: {spawn_point}")
        ego = spawn_police_vehicle(world, spawn_point)
        actual_transform = ego.get_transform()
        print(f"[CARLA] Police vehicle {i} spawned at: {actual_transform}")
        police_vehicles.append({
            'vehicle': ego,
            'pid': PIDLongitudinalController(kp=0.4, ki=0.02, kd=0.08, dt=CONTROL_DT),
            'stanley': StanleyLateralController(),
            'state': PoliceState(),
            'wpm': WaypointManager(),
            'planner': ChasePlanner(world, sampling_resolution=1.0),
            'unstuck': VehicleUnstuck()
        })

    # 🚛 트럭 스폰 (경찰차들과 다른 위치)
    truck_spawn = available_spawn_points[NUM_POLICE]  
    truck = spawn_truck(world, truck_spawn)
    
    # 타이밍 관련 변수들
    PATROL_DURATION = 5.0        # 순찰 5초
    ROUTE_UPDATE_CYCLE = 10.0     # 5초를 한 주기로 경로 업데이트
    POLICE_UPDATE_OFFSET = 1.25  # 각 경찰차마다 1.25초씩 간격 (5초/4대)
    start_time = time.time()
    last_updates = [0.0] * NUM_POLICE  # 각 경찰차별 마지막 업데이트 시간
    
    # 트럭 상태 관리
    truck_captured = False       # 트럭 포획 여부
    CAPTURE_DISTANCE = 10.0      # 포획 거리 (미터)

    zen = MockZenoh()
    chase_mode_activated = False  # 한번 활성화되면 계속 유지
    initial_chase_setup = False   # 첫 추격 경로 설정 여부
    lockon_time = None

    # Zenoh callbacks
    def lockon_cb(val):
        nonlocal chase_mode_activated, initial_chase_setup, lockon_time
        if bool(val) and not chase_mode_activated:
            chase_mode_activated = True  # 한번 True가 되면 계속 유지
            initial_chase_setup = False  # 첫 추격 경로 설정 대기
            lockon_time = time.time()
            for p in police_vehicles:
                p['state'].lockon = True
            print("[System] 🚨 추격 모드 활성화! 모든 경찰차가 동시에 경로 생성 중...")
        elif chase_mode_activated:
            # 이미 추격모드면 상태 유지
            for p in police_vehicles:
                p['state'].lockon = True

    def wps_cb(wps_list):
        # 트럭 추적 모드에서는 waypoints 콜백 무시
        if not chase_mode_activated:
            for p in police_vehicles:
                p['wpm'].set_waypoints([(wp['x'], wp['y'], wp.get('v', TARGET_SPEED)) for wp in wps_list])
            print("[MockZenoh] waypoints updated")

    zen.subscribe_lockon(lockon_cb)
    zen.subscribe_waypoints(wps_cb)

    print("[System] 🚔 경찰차들이 5초간 순찰 모드로 시작합니다...")
    print("[System] 🚛 트럭이 autopilot으로 돌아다니고 있습니다...")

    try:
        while True:
            loop_start = time.time()
            now = time.time()
            
            # 차량 생존 상태 확인
            if not all(p['vehicle'].is_alive for p in police_vehicles) or not truck.is_alive:
                print("[Error] 차량이 파괴되었습니다. 시뮬레이션을 종료합니다.")
                break

            # 🚨 5초 후 자동 lockon 신호 발생
            if not chase_mode_activated and (now - start_time) > PATROL_DURATION:
                print("[System] ⚡ 5초 경과! 트럭 추적 모드 시작!")
                lockon_cb(True)
                
            # 🚛 트럭 autopilot 상태 유지 (포획되지 않았을 때만)
            if not truck_captured and int(now * 10) % 50 == 0:  # 5초마다 한번씩 autopilot 재설정
                truck.set_autopilot(True)
            elif truck_captured:
                # 포획된 상태에서는 정지 상태 유지
                truck.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
            
            # 🚛 트럭 위치 시각화 (추격 모드일 때)
            if chase_mode_activated:
                truck_loc = truck.get_location()
                if truck_captured:
                    world.debug.draw_string(truck_loc + carla.Location(z=2.0), "🚨 STOPPED", 
                                          draw_shadow=True, color=carla.Color(r=255, g=0, b=0),
                                          life_time=1.0, persistent_lines=False)
                else:
                    world.debug.draw_string(truck_loc + carla.Location(z=2.0), "🚛 TARGET", 
                                          draw_shadow=True, color=carla.Color(r=255, g=255, b=0),
                                          life_time=1.0, persistent_lines=False)
            
            # 🚨 첫 추격 시작시 모든 경찰차가 트럭 포위 작전 개시!
            if chase_mode_activated and not initial_chase_setup and not truck_captured:
                print("[System] 🚔 트럭 포위 작전 개시! 4대의 경찰차가 포위 위치로 이동 중...")
                truck_loc = truck.get_location()
                surround_positions = calculate_surround_positions(truck, prediction_time=3.0)
                
                # 포위 위치들 시각화
                position_names = ["🔴 FRONT", "🔵 REAR", "🟢 LEFT", "🟡 RIGHT"]
                colors = [carla.Color(255, 0, 0), carla.Color(0, 0, 255), carla.Color(0, 255, 0), carla.Color(255, 255, 0)]
                
                for i, (pos, name, color) in enumerate(zip(surround_positions, position_names, colors)):
                    world.debug.draw_string(pos + carla.Location(z=3.0), 
                                          name, 
                                          draw_shadow=True, 
                                          color=color,
                                          life_time=8.0, persistent_lines=True)
                
                # 각 경찰차를 해당 포위 위치로 배치
                for idx, p in enumerate(police_vehicles):
                    origin = p['vehicle'].get_location()
                    target_position = surround_positions[idx % len(surround_positions)]  # 4개 위치 순환
                    
                    print(f"[Police-{idx}] 🎯 {position_names[idx % len(position_names)]} 포위 위치로 이동 시작!")
                    p['planner'].plan_route(origin, target_position)
                    route = p['planner'].get_route()
                    if route:
                        p['wpm'].set_waypoints([(x, y, z) for x, y, z in route])
                        visualize_planner_route(world, route, life_time=5.0)
                        print(f"[Police-{idx}] ✅ 포위 경로 생성 완료 ({len(route)} waypoints)")
                    last_updates[idx] = now
                
                initial_chase_setup = True
                print("[System] 🚔 전체 포위 작전 경로 설정 완료!")

            for idx, p in enumerate(police_vehicles):
                ego = p['vehicle']
                pid = p['pid']
                stanley = p['stanley']
                state = p['state']
                wpm = p['wpm']
                planner = p['planner']
                unstuck_module = p['unstuck']

                # 차량 상태 업데이트
                trans = ego.get_transform()
                vel = ego.get_velocity()
                speed = math.sqrt(vel.x**2 + vel.y**2)
                state.update_from_carla_transform(trans, vel)

                # � 트럭 포위 완성 및 포획 거리 확인
                if chase_mode_activated and not truck_captured:
                    truck_loc = truck.get_location()
                    ego_loc = ego.get_location()
                    distance = math.hypot(truck_loc.x - ego_loc.x, truck_loc.y - ego_loc.y)
                    
                    # 🎯 포위 완성도 체크 (모든 경찰차가 30m 이내에 있는지)
                    all_distances = []
                    for p_check in police_vehicles:
                        p_loc = p_check['vehicle'].get_location()
                        p_distance = math.hypot(truck_loc.x - p_loc.x, truck_loc.y - p_loc.y)
                        all_distances.append(p_distance)
                    
                    surround_complete = all([d <= 30.0 for d in all_distances])
                    if surround_complete and idx == 0:  # 첫 번째 경찰차에서만 체크
                        world.debug.draw_string(truck_loc + carla.Location(z=4.0), 
                                              "🚔 SURROUNDED! 🚔", 
                                              draw_shadow=True, 
                                              color=carla.Color(r=255, g=0, b=255),
                                              life_time=2.0, persistent_lines=False)
                    
                    if distance <= CAPTURE_DISTANCE:
                        truck_captured = True
                        truck.set_autopilot(False)  # autopilot 해제
                        truck.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
                        print(f"[SUCCESS] 🎉 Police-{idx}이 트럭을 포획했습니다! (거리: {distance:.1f}m)")
                        world.debug.draw_string(truck_loc + carla.Location(z=3.0), "🚨 CAPTURED!", 
                                              draw_shadow=True, color=carla.Color(r=255, g=0, b=0),
                                              life_time=30.0, persistent_lines=True)

                # � 포위 모드일 때 5초 주기로 각 경찰차별 순차 포위 위치 업데이트
                if chase_mode_activated and not truck_captured:
                    # 각 경찰차마다 다른 시간에 업데이트 (0, 1.25, 2.5, 3.75초)
                    chase_elapsed = (now - lockon_time) % ROUTE_UPDATE_CYCLE
                    police_update_time = idx * POLICE_UPDATE_OFFSET
                    
                    if abs(chase_elapsed - police_update_time) < 0.1 and (now - last_updates[idx]) >= (ROUTE_UPDATE_CYCLE - 0.2):
                        origin = ego.get_location()
                        
                        # 🚔 트럭 포위 위치 재계산 (실시간 업데이트)
                        surround_positions = calculate_surround_positions(truck, prediction_time=3.0)
                        target_position = surround_positions[idx % len(surround_positions)]
                        
                        # 포위 위치 시각화
                        position_names = ["🔴 FRONT", "🔵 REAR", "🟢 LEFT", "🟡 RIGHT"]
                        position_colors = [carla.Color(255,0,0), carla.Color(0,0,255), carla.Color(0,255,0), carla.Color(255,255,0)]
                        
                        world.debug.draw_string(target_position + carla.Location(z=1.5), 
                                              f"{position_names[idx % len(position_names)]} P{idx}", 
                                              draw_shadow=True, 
                                              color=position_colors[idx % len(position_colors)],
                                              life_time=3.0, persistent_lines=False)
                        
                        planner.plan_route(origin, target_position)
                        route = planner.get_route()
                        if route:
                            wpm.set_waypoints([(x, y, z) for x, y, z in route])
                            visualize_planner_route(world, route, life_time=3.0)
                            print(f"[Police-{idx}] 🚔 {position_names[idx % len(position_names)]} 포위 위치 업데이트 완료")
                        last_updates[idx] = now

                # Unstuck 적용
                control_signal = ego.get_control()
                unstuck_module.check_stuck(control_signal.throttle, speed, ego.get_location(), CONTROL_DT)
                unstuck_control = unstuck_module.compute_unstuck_control(ego.get_location(), CONTROL_DT)
                if unstuck_control:
                    ego.apply_control(unstuck_control)
                    continue  # unstuck 중이면 PID/Stanley 스킵

                # waypoint 재계산 (트럭 추적 중이 아닐 때만)
                # 트럭 추적은 위에서 1초마다 자동으로 처리되므로 여기서는 스킵
                pass
                # 🚔 경찰차 모드 제어: 순찰 vs 추격
                if not chase_mode_activated:
                    # 순찰 모드: autopilot 활성화
                    ego.set_autopilot(True)
                    continue
                else:
                    # 추격 모드: 수동 제어
                    ego.set_autopilot(False)

                # 현재 목표 waypoint
                target = wpm.get_current_target()
                if target is None:
                    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=MAX_BRAKE, steer=0.0))
                    continue

                tx, ty, tz = target
                steer = stanley.compute_steer(state.x, state.y, state.yaw,
                                              [(wp[0], wp[1]) for wp in wpm.waypoints],
                                              state.speed,
                                              lockon_time=lockon_time)
                pid_out = pid.run_step(TARGET_SPEED, state.speed, lockon_time=lockon_time)
                throttle = min(MAX_THROTTLE, pid_out) if pid_out >= 0 else 0.0
                brake = min(MAX_BRAKE, -pid_out) if pid_out < 0 else 0.0
                ego.apply_control(carla.VehicleControl(steer=float(steer), throttle=float(throttle), brake=float(brake)))
                wpm.update_progress(state.x, state.y)

                if wpm.is_final_reached(state.x, state.y):
                    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=MAX_BRAKE, steer=0.0))

            # 메인 루프 종료

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, CONTROL_DT - elapsed))

    except KeyboardInterrupt:
        print("Shutting down police_control.")
    finally:
        # 경찰차들 정리
        for p in police_vehicles:
            if p['vehicle'].is_alive:
                p['vehicle'].destroy()
        print("All police vehicles destroyed safely.")
        
        # 트럭 정리
        if truck.is_alive:
            truck.destroy()
        print("Target truck destroyed safely.")

if __name__ == "__main__":
    main()
