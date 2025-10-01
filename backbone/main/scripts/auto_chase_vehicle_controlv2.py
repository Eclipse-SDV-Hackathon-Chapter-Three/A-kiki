#!/usr/bin/env python3
"""
Auto Chase Vehicle Control Script
충돌 감지 시 자동으로 추격하는 차량 제어 스크립트
"""

import sys
import os
import carla
import numpy as np
import cv2
import threading
import time
import pygame

# 상위 디렉토리의 모듈들을 import하기 위해 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from chase.perception.bounding_box_detector import BoundingBoxDetector
from chase.perception.collision_detector import CollisionDetector
from chase.perception.collision_vehicle_tracker import CollisionVehicleTracker
from chase.planning.simple_chase_planner import SimpleChasePlanner
# from chase.control.vehicle_controller import ChaseVehicleController  # 사용하지 않음
from vehicle.optimized_camera_view import OptimizedCameraView

class AutoChaseVehicleControl:
    """자동 추격 차량 제어 클래스"""
    
    def __init__(self):
        self.client = None
        self.world = None
        self.vehicle = None
        
        # 모듈들
        self.bounding_box_detector = None
        self.collision_detector = None
        self.vehicle_tracker = None
        self.chase_planner = None
        # self.vehicle_controller = None  # 사용하지 않음
        
        # 카메라 뷰
        self.camera_view = None
        
        # 제어 상태
        self.running = True
        self.is_chasing = False
        self.last_update_time = 0.0
        self.update_interval = 0.05  # 50ms 업데이트 간격 (더 빠른 반응)
        
        # IMU 센서
        self.imu_sensor = None
        self.current_imu_data = None
        
        # 통계
        self.chase_statistics = {
            'total_detections': 0,
            'collision_events': 0,
            'tracking_duration': 0.0,
            'chase_distance': 0.0,
            'max_speed_reached': 0.0
        }
        
        print("🚔 Auto Chase Vehicle Control initialized")
    
    def connect_to_carla(self, host='localhost', port=2000):
        """CARLA 서버에 연결"""
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            
            print(f"✅ Connected to CARLA server at {host}:{port}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to connect to CARLA: {e}")
            return False
    
    def spawn_vehicle(self):
        """추격차량 스폰 (수동 차량에서 Y축 +7 위치)"""
        try:
            # 차량 블루프린트 선택
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
            
            if vehicle_bp is None:
                print("❌ Tesla Model 3 blueprint not found")
                return False
            
            # 수동 차량 고정 위치에서 Y축 +7 위치에 스폰
            manual_location = carla.Location(-104.15, 44.15, 1.0)
            chase_location = carla.Location(
                manual_location.x,
                manual_location.y + 15,# Y축 +7  
                manual_location.z
            )
            chase_rotation = carla.Rotation(0, 270.0, 0)  # 수동 차량과 같은 방향
            spawn_point = carla.Transform(chase_location, chase_rotation)
            
            print(f"🎯 Spawning chase vehicle at: {chase_location} (Y+7 from manual vehicle)")
            
            try:
                self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
                if self.vehicle is not None:
                    print(f"✅ Chase vehicle spawned at: {chase_location}")
                    return True
            except Exception as e:
                print(f"⚠️ Failed to spawn at fixed location: {e}")
            
            # 고정 위치에서 실패하면 기본 스폰 포인트 시도
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                print("❌ No spawn points available")
                return False
            
            # 여러 스폰 포인트 시도
            for i, sp in enumerate(spawn_points):
                try:
                    print(f"🔄 Trying spawn point {i+1}/{len(spawn_points)}...")
                    self.vehicle = self.world.spawn_actor(vehicle_bp, sp)
                    if self.vehicle is not None:
                        print(f"✅ Chase vehicle spawned at spawn point {i+1}: {sp.location}")
                        break
                except Exception as e:
                    print(f"⚠️ Spawn point {i+1} failed: {e}")
                    continue
            
            if self.vehicle is None:
                print("❌ All spawn points failed")
                return False
            
            return True
            
        except Exception as e:
            print(f"❌ Error spawning chase vehicle: {e}")
            return False
    
    def setup_camera(self):
        """추격차량 카메라 설정"""
        try:
            # 최적화된 고성능 카메라 뷰 사용
            self.camera_view = OptimizedCameraView(
                self.world, 
                self.vehicle, 
                "Auto Chase Vehicle Camera View",
                enable_bounding_boxes=True,  # 바운딩 박스 활성화
                enable_zenoh=True           # Zenoh 활성화
            )
            
            # 카메라 설정
            camera_location = carla.Location(x=1.5, z=1.4)
            camera_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
            
            if self.camera_view.setup_camera(camera_location, camera_rotation):
                print("📷 Chase vehicle camera attached with bounding box detection")
                return True
            else:
                return False
            
        except Exception as e:
            print(f"❌ Error setting up camera: {e}")
            return False
    
    def setup_imu_sensor(self):
        """IMU 센서 설정"""
        try:
            # IMU 센서 블루프린트 가져오기
            imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
            if imu_bp is None:
                print("⚠️ IMU sensor blueprint not found")
                return False
            
            # IMU 센서 스폰
            imu_transform = carla.Transform()
            self.imu_sensor = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)
            
            # IMU 데이터 콜백 설정
            self.imu_sensor.listen(self.on_imu_data)
            
            print("🧭 IMU sensor attached")
            return True
            
        except Exception as e:
            print(f"❌ Error setting up IMU sensor: {e}")
            return False
    
    def on_imu_data(self, imu_data):
        """IMU 데이터 콜백"""
        try:
            self.current_imu_data = {
                'accelerometer': (imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z),
                'gyroscope': (imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z),
                'compass': imu_data.compass,
                'timestamp': imu_data.timestamp
            }
        except Exception as e:
            print(f"⚠️ Error processing IMU data: {e}")
    
    def setup_modules(self):
        """모듈들 초기화"""
        try:
            # 바운딩 박스 감지기 초기화
            if self.camera_view and self.camera_view.camera:
                self.bounding_box_detector = BoundingBoxDetector(self.world, self.camera_view.camera)
                print("🎯 Bounding box detector initialized")
            
            # 충돌 감지기 초기화
            self.collision_detector = CollisionDetector()
            print("🚨 Collision detector initialized")
            
            # 충돌 차량 추적기 초기화
            self.vehicle_tracker = CollisionVehicleTracker(self.world)
            print("🎯 Vehicle tracker initialized")
            
            # 단순 추격 계획자 초기화
            self.chase_planner = SimpleChasePlanner(self.world, self.vehicle, self.world.get_map())
            # 추격 파라미터 설정 (속도 더욱 감소)
            self.chase_planner.set_chase_parameters(
                max_speed=8.0,         # 최대 속도 (29km/h) - 매우 안전한 속도
                min_distance=5.0,      # 5m 이내에서 박기 - 더 안전한 거리
                chase_distance=100.0   # 100m 이내에서 추격
            )
            # 조향 방향 정상 설정 (타겟을 따라가도록)
            self.chase_planner.set_steering_direction(1)
            print("🧠 Chase planner initialized with high-speed parameters")
            
            # 차량 제어기 초기화 (사용하지 않음 - SimpleChasePlanner에서 직접 제어)
            # self.vehicle_controller = ChaseVehicleController(self.world, self.vehicle)
            # print("🎮 Vehicle controller initialized")
            
            print("✅ All modules initialized")
            return True
            
        except Exception as e:
            print(f"❌ Error setting up modules: {e}")
            return False
    
    def update_perception(self):
        """인식 모듈 업데이트"""
        try:
            if not self.bounding_box_detector:
                return []
            
            # 객체 감지 (범위 확장)
            detected_objects = self.bounding_box_detector.detect_pedestrians_and_vehicles(max_distance=200.0)
            self.chase_statistics['total_detections'] = len(detected_objects)
            
            # 충돌 감지
            collision_events = self.collision_detector.analyze_pedestrian_collision(detected_objects)
            if collision_events:
                self.chase_statistics['collision_events'] += len(collision_events)
                print(f"🚨 Collision detected: {len(collision_events)} events")
            
            return detected_objects, collision_events
            
        except Exception as e:
            print(f"⚠️ Error updating perception: {e}")
            return [], []
    
    def update_tracking(self, detected_objects, collision_events):
        """추적 모듈 업데이트"""
        try:
            if not self.vehicle_tracker:
                return False
            
            # 충돌 차량 감지 및 추적 시작
            if collision_events:
                print(f"🚨 Collision events detected: {len(collision_events)}")
                tracking_started = self.vehicle_tracker.detect_collision_vehicle(detected_objects, collision_events)
                print(f"🎯 Tracking started: {tracking_started}")
            
            # 추적 업데이트 (추적 중이거나 방금 시작된 경우)
            if self.vehicle_tracker.is_tracking:
                is_tracking = self.vehicle_tracker.update_tracking(detected_objects)
                print(f"🎯 Tracking update result: {is_tracking}")
                return is_tracking
            else:
                # 충돌이 감지된 경우에만 추격 시작
                if collision_events:
                    print(f"🚨 Collision detected - starting chase")
                    # 충돌 이벤트가 있는 경우에만 추격 시작
                    if detected_objects:
                        target_vehicle = detected_objects[0]
                        self.vehicle_tracker.start_tracking(target_vehicle)
                        return True
                else:
                    print(f"🚗 Vehicles detected but no collision - waiting for collision event")
                    return False
            
            return False
            
        except Exception as e:
            print(f"⚠️ Error updating tracking: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def update_planning_and_control(self):
        """계획 및 제어 모듈 업데이트"""
        try:
            print(f"🎯 Planning and control update: tracker={self.vehicle_tracker is not None}, is_tracking={self.vehicle_tracker.is_tracking if self.vehicle_tracker else False}")
            
            if not self.vehicle_tracker or not self.vehicle_tracker.is_tracking:
                # 추격 중이 아닌 경우 대기
                print("🛑 Not tracking - waiting for target")
                self._wait_for_target()
                return
            
            # 타겟 차량 가져오기
            target_vehicle = self.vehicle_tracker.get_target_vehicle()
            if not target_vehicle:
                print("⚠️ No target vehicle from tracker")
                return
            
            print(f"🎯 Target vehicle: {target_vehicle}")
            
            # 현재 위치와 회전
            current_location = self.vehicle.get_location()
            current_rotation = self.vehicle.get_transform().rotation
            
            # 추격 시작 (한 번만)
            if not self.is_chasing:
                self.chase_planner.start_chase(target_vehicle)
                self.is_chasing = True
                print("🎯 Started chasing target vehicle")
            
            # 타겟 위치 가져오기
            if isinstance(target_vehicle, dict) and 'world_location' in target_vehicle:
                target_location = carla.Location(
                    x=target_vehicle['world_location'][0],
                    y=target_vehicle['world_location'][1],
                    z=target_vehicle['world_location'][2]
                )
            else:
                target_location = target_vehicle.get_location()
            
            # IMU 데이터 전달
            imu_data = self.current_imu_data if self.current_imu_data else None
            
            # 타겟 바운딩 박스 가져오기
            target_bbox = None
            if isinstance(target_vehicle, dict) and 'bbox_2d' in target_vehicle:
                target_bbox = target_vehicle['bbox_2d']
                print(f"📷 Using target bbox: {target_bbox}")
            else:
                print("⚠️ No target bbox available")
            
            # 카메라 정보 가져오기
            camera_info = None
            if self.camera_view and self.camera_view.camera:
                camera_info = {
                    'width': int(self.camera_view.camera.attributes['image_size_x']),
                    'height': int(self.camera_view.camera.attributes['image_size_y'])
                }
                print(f"📷 Camera info: {camera_info['width']}x{camera_info['height']}")
            
            # 카메라 기반 추격 계획 수립 및 직접 제어 적용
            control_command = self.chase_planner.plan_chase_behavior(
                current_location, current_rotation, target_location, None, imu_data, target_bbox, camera_info
            )
            
            # SimpleChasePlanner에서 이미 vehicle.apply_control()을 호출하므로 별도 적용 불필요
            # self._apply_control_command(control_command)
            
            # 상태 출력
            status = self.chase_planner.get_chase_status()
            print(f"🎯 {status}")
            
        except Exception as e:
            print(f"⚠️ Error updating planning and control: {e}")
            import traceback
            traceback.print_exc()
            self._emergency_stop()
    
    def _wait_for_target(self):
        """타겟 대기"""
        try:
            # 추격 중이었다면 중지
            if self.is_chasing:
                self.is_chasing = False
                print("🎯 Stopped chasing - waiting for target")
            
            # SimpleChasePlanner의 stop_control을 사용하여 제어
            if self.chase_planner:
                self.chase_planner.stop_chase()
            
        except Exception as e:
            print(f"⚠️ Error in wait mode: {e}")
    
    # _apply_control_command 함수 제거 - SimpleChasePlanner에서 직접 제어
    
    def _update_chase_statistics(self, target_distance):
        """추격 통계 업데이트"""
        try:
            # 현재 속도 계산
            current_velocity = self.vehicle.get_velocity()
            current_speed = np.linalg.norm([current_velocity.x, current_velocity.y, current_velocity.z])
            
            # 최대 속도 업데이트
            if current_speed > self.chase_statistics['max_speed_reached']:
                self.chase_statistics['max_speed_reached'] = current_speed
            
            # 추격 거리 업데이트
            self.chase_statistics['chase_distance'] = target_distance
            
            # 추격 시간 업데이트
            if self.is_chasing:
                self.chase_statistics['tracking_duration'] += self.update_interval
            
        except Exception as e:
            print(f"⚠️ Error updating statistics: {e}")
    
    def _print_chase_status(self, target_distance):
        """추격 상태 출력"""
        try:
            # 5초마다 상태 출력
            if int(time.time()) % 5 == 0:
                current_velocity = self.vehicle.get_velocity()
                current_speed = np.linalg.norm([current_velocity.x, current_velocity.y, current_velocity.z])
                
                print(f"🎯 Chase Status: Distance={target_distance:.1f}m, Speed={current_speed:.1f}m/s")
                
                # 추적 상태 출력
                if self.vehicle_tracker:
                    tracking_status = self.vehicle_tracker.get_tracking_status()
                    print(f"   Tracking: {tracking_status['is_tracking']}, Duration: {tracking_status['track_duration']:.1f}s")
                
                # 추격 상태 출력
                if self.chase_planner:
                    chase_status = self.chase_planner.get_chase_status()
                    print(f"   State: {chase_status['current_state']}, Duration: {chase_status['state_duration']:.1f}s")
            
        except Exception as e:
            print(f"⚠️ Error printing chase status: {e}")
    
    def _emergency_stop(self):
        """비상 정지"""
        try:
            # SimpleChasePlanner의 stop_control을 사용하여 제어
            if self.chase_planner:
                self.chase_planner.stop_chase()
            
            print("🛑 Emergency stop applied")
            
        except Exception as e:
            print(f"⚠️ Error applying emergency stop: {e}")
    
    def _trigger_manual_collision(self):
        """수동 충돌 트리거 (테스트용)"""
        try:
            print("🧪 MANUAL COLLISION TRIGGERED!")
            
            # 가상의 충돌 이벤트 생성
            fake_collision_event = {
                'actor_id': 99999,  # 가상 ID
                'collision_score': 0.9,
                'timestamp': time.time(),
                'world_location': (0, 0, 0),  # 현재 위치
                'bbox_2d': {'width': 100, 'height': 50},  # 누워진 상태 시뮬레이션
                'avoid_pedestrian': True,
                'collision_indicators': ['manual_trigger'],
                'test_mode': True
            }
            
            # 충돌 감지기에 직접 전달
            if self.collision_detector:
                collision_events = [fake_collision_event]
                print(f"🚨 Triggering collision detection with fake event")
                
                # 차량 추적기로 전달
                if self.vehicle_tracker:
                    self.vehicle_tracker.detect_collision_vehicle([], collision_events)
            
        except Exception as e:
            print(f"⚠️ Error triggering manual collision: {e}")
            import traceback
            traceback.print_exc()
    
    def display_camera_view(self):
        """카메라 뷰 표시"""
        try:
            if self.camera_view:
                # HUD 정보 업데이트
                if self.vehicle_tracker and self.vehicle_tracker.is_tracking:
                    target_position = self.vehicle_tracker.get_target_position()
                    target_distance = self.vehicle_tracker.get_target_distance(self.vehicle.get_transform().location)
                    
                    if target_position:
                        self.camera_view.set_hud_info(
                            Target=f"({target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f})",
                            Distance=f"{target_distance:.1f} m",
                            Objects=f"{self.chase_statistics['total_detections']} | Collisions: {self.chase_statistics['collision_events']}"
                        )
                
                self.camera_view.display_camera_view()
            
        except Exception as e:
            print(f"⚠️ Error displaying camera view: {e}")
    
    def run(self):
        """메인 실행 루프"""
        try:
            print("🚔 Starting Auto Chase Vehicle Control...")
            
            # CARLA 연결
            if not self.connect_to_carla():
                return
            
            # 추격차량 스폰
            if not self.spawn_vehicle():
                return
            
            # 카메라 설정
            if not self.setup_camera():
                return
            
            # IMU 센서 설정
            self.setup_imu_sensor()
            
            # 모듈들 초기화
            if not self.setup_modules():
                return
            
            print("✅ Auto chase vehicle control ready!")
            print("🎯 Monitoring for collisions...")
            print("🛑 Press ESC to exit")
            
            # 메인 루프
            while self.running:
                try:
                    # 입력 처리
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            self.running = False
                            break
                        elif event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_ESCAPE:
                                self.running = False
                                break
                            elif event.key == pygame.K_c:  # C키로 수동 충돌 트리거
                                self._trigger_manual_collision()
                    
                    if not self.running:
                        break
                    
                    current_time = time.time()
                    
                    # 업데이트 간격 체크
                    if current_time - self.last_update_time < self.update_interval:
                        time.sleep(0.01)  # 10ms 대기
                        continue
                    
                    # 1. 인식 업데이트
                    detected_objects, collision_events = self.update_perception()
                    
                    # 2. 추적 업데이트
                    is_tracking = self.update_tracking(detected_objects, collision_events)
                    print(f"🎯 Main loop - is_tracking: {is_tracking}")
                    
                    # 3. 계획 및 제어 업데이트
                    self.update_planning_and_control()
                    
                    # 4. 카메라 뷰 표시
                    self.display_camera_view()
                    
                    self.last_update_time = current_time
                    
                except KeyboardInterrupt:
                    print("\n🛑 Auto chase vehicle control interrupted by user")
                    break
                except Exception as e:
                    print(f"⚠️ Error in main loop: {e}")
                    time.sleep(0.1)
            
        except Exception as e:
            print(f"❌ Error in main execution: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """리소스 정리"""
        try:
            print("\n🧹 Cleaning up resources...")
            
            # 모듈들 정리
            if self.vehicle_tracker:
                self.vehicle_tracker.reset_tracking()
            if self.chase_planner:
                self.chase_planner.reset_chase()
            # if self.vehicle_controller:  # 사용하지 않음
            #     self.vehicle_controller.destroy()
            
            # 카메라 뷰 정리
            if self.camera_view:
                self.camera_view.cleanup()
                self.camera_view = None
            
            # 차량 정리
            if self.vehicle:
                self.vehicle.destroy()
                self.vehicle = None
            
            print("✅ Cleanup completed")
            
        except Exception as e:
            print(f"⚠️ Error during cleanup: {e}")

def main():
    """메인 함수"""
    try:
        auto_chase_control = AutoChaseVehicleControl()
        auto_chase_control.run()
    except Exception as e:
        print(f"❌ Fatal error: {e}")

if __name__ == "__main__":
    main()
