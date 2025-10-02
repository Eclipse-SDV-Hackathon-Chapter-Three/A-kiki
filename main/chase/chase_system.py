"""
Integrated Chase System
충돌 차량 추격을 위한 통합 시스템
"""

import carla
import numpy as np
import time
import threading
from typing import Dict, List, Optional

# 상위 디렉토리의 모듈들을 import하기 위해 경로 추가
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from chase.perception.bounding_box_detector import BoundingBoxDetector
from chase.perception.collision_detector import CollisionDetector
from chase.perception.collision_vehicle_tracker import CollisionVehicleTracker
from chase.planning.chase_planner import ChasePlanner
from chase.control.vehicle_controller import ChaseVehicleController

class ChaseSystem:
    """통합 추격 시스템"""
    
    def __init__(self, world, chase_vehicle, camera):
        self.world = world
        self.chase_vehicle = chase_vehicle
        self.camera = camera
        
        # 모듈 초기화
        self.bounding_box_detector = BoundingBoxDetector(world, camera)
        self.collision_detector = CollisionDetector()
        self.vehicle_tracker = CollisionVehicleTracker(world)
        self.chase_planner = ChasePlanner(world, chase_vehicle)
        self.vehicle_controller = ChaseVehicleController(world, chase_vehicle)
        
        # 시스템 상태
        self.is_running = False
        self.is_chasing = False
        self.last_update_time = 0.0
        self.update_interval = 0.1  # 100ms 업데이트 간격
        
        # 통계
        self.chase_statistics = {
            'total_detections': 0,
            'collision_events': 0,
            'tracking_duration': 0.0,
            'chase_distance': 0.0,
            'max_speed_reached': 0.0
        }
        
        print("🚨 Integrated Chase System initialized")
    
    def start_chase_system(self):
        """추격 시스템 시작"""
        try:
            self.is_running = True
            self.is_chasing = False
            
            print("🚨 Chase System started - Monitoring for collisions...")
            
            # 메인 추격 루프 시작
            self._chase_loop()
            
        except Exception as e:
            print(f"❌ Error starting chase system: {e}")
            self.is_running = False
    
    def stop_chase_system(self):
        """추격 시스템 중지"""
        try:
            self.is_running = False
            self.is_chasing = False
            
            # 차량 정지
            self.vehicle_controller.stop_vehicle()
            
            # 추적 리셋
            self.vehicle_tracker.reset_tracking()
            self.chase_planner.reset_chase()
            
            print("🛑 Chase System stopped")
            
        except Exception as e:
            print(f"⚠️ Error stopping chase system: {e}")
    
    def _chase_loop(self):
        """메인 추격 루프"""
        try:
            while self.is_running:
                current_time = time.time()
                
                # 업데이트 간격 체크
                if current_time - self.last_update_time < self.update_interval:
                    time.sleep(0.01)  # 10ms 대기
                    continue
                
                # 1. 객체 감지
                detected_objects = self.bounding_box_detector.detect_pedestrians_and_vehicles()
                self.chase_statistics['total_detections'] = len(detected_objects)
                
                # 2. 충돌 감지
                collision_events = self.collision_detector.analyze_pedestrian_collision(detected_objects)
                if collision_events:
                    self.chase_statistics['collision_events'] += len(collision_events)
                    print(f"🚨 Collision detected: {len(collision_events)} events")
                
                # 3. 충돌 차량 추적
                if collision_events:
                    self.vehicle_tracker.detect_collision_vehicle(detected_objects, collision_events)
                
                # 4. 추격 행동 실행
                if self.vehicle_tracker.is_tracking:
                    self._execute_chase_behavior(detected_objects)
                else:
                    # 추격 중이 아닌 경우 대기
                    self._wait_for_target()
                
                self.last_update_time = current_time
                
        except KeyboardInterrupt:
            print("\n🛑 Chase system interrupted by user")
        except Exception as e:
            print(f"❌ Error in chase loop: {e}")
        finally:
            self.stop_chase_system()
    
    def _execute_chase_behavior(self, detected_objects):
        """추격 행동 실행"""
        try:
            # 추적 업데이트
            is_tracking = self.vehicle_tracker.update_tracking(detected_objects)
            
            if not is_tracking:
                print("🎯 Lost target - returning to search mode")
                self.is_chasing = False
                return
            
            # 추격 시작
            if not self.is_chasing:
                self.is_chasing = True
                print("🎯 Started chasing target vehicle")
            
            # 타겟 정보 가져오기
            target_position = self.vehicle_tracker.get_target_position()
            target_velocity = self.vehicle_tracker.get_target_velocity()
            target_distance = self.vehicle_tracker.get_target_distance(self.chase_vehicle.get_transform().location)
            
            # 추격 계획 수립
            control_command = self.chase_planner.plan_chase_behavior(
                target_position, target_velocity, target_distance, True
            )
            
            # 제어 명령 적용
            self._apply_control_command(control_command)
            
            # 통계 업데이트
            self._update_chase_statistics(target_distance)
            
            # 상태 출력
            self._print_chase_status(target_distance)
            
        except Exception as e:
            print(f"⚠️ Error executing chase behavior: {e}")
            self._emergency_stop()
    
    def _wait_for_target(self):
        """타겟 대기"""
        try:
            # 대기 상태 제어 (정지)
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 0.5
            control.steer = 0.0
            control.hand_brake = True
            
            self.chase_vehicle.apply_control(control)
            
        except Exception as e:
            print(f"⚠️ Error in wait mode: {e}")
    
    def _apply_control_command(self, control_command):
        """제어 명령 적용"""
        try:
            control = carla.VehicleControl()
            control.throttle = control_command['throttle']
            control.brake = control_command['brake']
            control.steer = control_command['steer']
            control.hand_brake = control_command['hand_brake']
            
            self.chase_vehicle.apply_control(control)
            
        except Exception as e:
            print(f"⚠️ Error applying control command: {e}")
    
    def _update_chase_statistics(self, target_distance):
        """추격 통계 업데이트"""
        try:
            # 현재 속도 계산
            current_velocity = self.chase_vehicle.get_velocity()
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
                current_velocity = self.chase_vehicle.get_velocity()
                current_speed = np.linalg.norm([current_velocity.x, current_velocity.y, current_velocity.z])
                
                print(f"🎯 Chase Status: Distance={target_distance:.1f}m, Speed={current_speed:.1f}m/s")
                
                # 추적 상태 출력
                tracking_status = self.vehicle_tracker.get_tracking_status()
                print(f"   Tracking: {tracking_status['is_tracking']}, Duration: {tracking_status['track_duration']:.1f}s")
                
                # 추격 상태 출력
                chase_status = self.chase_planner.get_chase_status()
                print(f"   State: {chase_status['current_state']}, Duration: {chase_status['state_duration']:.1f}s")
            
        except Exception as e:
            print(f"⚠️ Error printing chase status: {e}")
    
    def _emergency_stop(self):
        """비상 정지"""
        try:
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            control.hand_brake = True
            
            self.chase_vehicle.apply_control(control)
            
            print("🛑 Emergency stop applied")
            
        except Exception as e:
            print(f"⚠️ Error applying emergency stop: {e}")
    
    def get_system_status(self):
        """시스템 상태 반환"""
        try:
            tracking_status = self.vehicle_tracker.get_tracking_status()
            chase_status = self.chase_planner.get_chase_status()
            
            return {
                'is_running': self.is_running,
                'is_chasing': self.is_chasing,
                'tracking': tracking_status,
                'chase': chase_status,
                'statistics': self.chase_statistics.copy()
            }
            
        except Exception as e:
            print(f"⚠️ Error getting system status: {e}")
            return {
                'is_running': False,
                'is_chasing': False,
                'tracking': {},
                'chase': {},
                'statistics': {}
            }
    
    def get_detection_image(self):
        """감지 이미지 반환 (바운딩 박스 포함)"""
        try:
            # 카메라 이미지 가져오기
            if hasattr(self.camera, 'image') and self.camera.image is not None:
                # 바운딩 박스 그리기
                image_with_boxes = self.bounding_box_detector.draw_bounding_boxes_on_image(
                    self.camera.image, use_3d=False
                )
                return image_with_boxes
            return None
            
        except Exception as e:
            print(f"⚠️ Error getting detection image: {e}")
            return None
    
    def set_chase_parameters(self, follow_distance=None, approach_distance=None, 
                           max_speed=None, update_interval=None):
        """추격 파라미터 설정"""
        try:
            if follow_distance is not None or approach_distance is not None:
                self.chase_planner.set_chase_parameters(follow_distance, approach_distance)
            
            if max_speed is not None:
                self.chase_planner.max_speed = max_speed
            
            if update_interval is not None:
                self.update_interval = update_interval
            
            print(f"🎯 Chase parameters updated")
            
        except Exception as e:
            print(f"⚠️ Error setting chase parameters: {e}")
    
    def reset_system(self):
        """시스템 리셋"""
        try:
            self.stop_chase_system()
            
            # 통계 리셋
            self.chase_statistics = {
                'total_detections': 0,
                'collision_events': 0,
                'tracking_duration': 0.0,
                'chase_distance': 0.0,
                'max_speed_reached': 0.0
            }
            
            print("🔄 Chase system reset")
            
        except Exception as e:
            print(f"⚠️ Error resetting system: {e}")
    
    def destroy(self):
        """리소스 정리"""
        try:
            self.stop_chase_system()
            
            # 모듈 정리
            if hasattr(self.bounding_box_detector, 'cleanup'):
                self.bounding_box_detector.cleanup()
            
            print("🧹 Chase system destroyed")
            
        except Exception as e:
            print(f"⚠️ Error destroying chase system: {e}")
