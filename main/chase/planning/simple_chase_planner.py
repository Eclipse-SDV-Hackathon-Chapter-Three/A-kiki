#!/usr/bin/env python3

"""
Simple Chase Planner
단순한 추격 계획자 - 장애물 회피 없이 직접 추격
"""

import carla
import numpy as np
import math
import time
from typing import Dict, List, Optional, Tuple
from enum import Enum

class ChaseState(Enum):
    """추격 상태"""
    IDLE = "idle"                    # 대기
    CHASING = "chasing"              # 추격

class SimpleChasePlanner:
    """단순한 추격 계획자 - 직접 추격만"""
    
    def __init__(self, world, chase_vehicle, map_inst=None):
        self.world = world
        self.chase_vehicle = chase_vehicle
        self.map = map_inst if map_inst else world.get_map()
        
        # 추격 파라미터
        self.max_speed = 40.0        # 최대 속도 (144km/h)
        self.min_distance = 3.0      # 최소 거리 (박기)
        self.chase_distance = 50.0   # 추격 거리
        
        # 상태
        self.current_state = ChaseState.IDLE
        self.target_vehicle = None
        self.target_location = None
        self.last_update_time = 0
        
        # 조향 제어 개선을 위한 변수들
        self.last_steer = 0.0
        self.steering_direction_multiplier = 1  # 기본값 설정
        self.max_steer_change = 0.005  # 최대 조향 변화량 (매우 부드러운 조향을 위해)
        
        # Control command publisher callback
        self.control_publisher_callback = None
        
        print("🚗 Simple Chase Planner initialized")
    
    def set_steering_direction(self, multiplier):
        """조향 방향 설정"""
        self.steering_direction_multiplier = multiplier
        print(f"🔄 Steering direction set to: {multiplier}")
    
    def set_chase_parameters(self, max_speed=30.0, min_distance=3.0, chase_distance=50.0):
        """추격 파라미터 설정"""
        self.max_speed = max_speed
        self.min_distance = min_distance
        self.chase_distance = chase_distance
        print(f"🎯 Chase parameters: max_speed={max_speed}m/s, min_distance={min_distance}m, chase_distance={chase_distance}m")
    
    def set_control_publisher_callback(self, callback):
        """Set callback function for publishing control commands"""
        self.control_publisher_callback = callback
        print("📡 Control publisher callback set")
    
    def start_chase(self, target_vehicle):
        """추격 시작"""
        self.target_vehicle = target_vehicle
        self.current_state = ChaseState.CHASING
        
        # 조향 상태 리셋 (새로운 추격 시작 시)
        self.last_steer = 0.0
        
        # target_vehicle이 딕셔너리인 경우와 CARLA 객체인 경우 모두 처리
        if isinstance(target_vehicle, dict):
            vehicle_id = target_vehicle.get('actor_id', 'Unknown')
            print(f"🎯 Started chasing vehicle (dict) ID: {vehicle_id}")
        else:
            vehicle_id = target_vehicle.id
            print(f"🎯 Started chasing vehicle (CARLA object) ID: {vehicle_id}")
    
    def stop_chase(self):
        """추격 중지"""
        self.target_vehicle = None
        self.target_location = None
        self.current_state = ChaseState.IDLE
        
        # 정지 제어 적용
        self._get_stop_control()
        
        # Chase stopped (no print to reduce spam)
    
    def plan_chase_behavior(self, current_location, current_rotation, target_location=None, avoid_pedestrians=None, imu_data=None, target_bbox=None, camera_info=None):
        """추격 행동 계획 - 카메라 이미지 기반 추격"""
        try:
            print(f"🎯 Planning chase behavior: state={self.current_state}, has_target={self.target_vehicle is not None}")
            
            if self.current_state != ChaseState.CHASING or not self.target_vehicle:
                print("🛑 Not chasing - returning stop control")
                return self._get_stop_control()
            
            # 타겟 위치 가져오기 (거리 계산용)
            if target_location:
                target_pos = target_location
            else:
                if isinstance(self.target_vehicle, dict):
                    if 'world_location' in self.target_vehicle:
                        target_pos = carla.Location(
                            x=self.target_vehicle['world_location'][0],
                            y=self.target_vehicle['world_location'][1],
                            z=self.target_vehicle['world_location'][2]
                        )
                    else:
                        print("⚠️ Target vehicle dict has no world_location")
                        return self._get_stop_control()
                else:
                    target_pos = self.target_vehicle.get_location()
            
            # 거리 계산
            distance = self._calculate_distance(current_location, target_pos)
            print(f"🎯 Distance to target: {distance:.1f}m")
            
            # 추격 거리 밖이면 중지
            if distance > self.chase_distance:
                print(f"🎯 Target too far: {distance:.1f}m > {self.chase_distance}m")
                return self._get_stop_control()
            
            # 항상 카메라 기반 추격 (RAMMING 제거)
            print(f"🏃 Camera-based chasing! Distance: {distance:.1f}m")
            return self._get_camera_chase_control(target_bbox, current_location, current_rotation, distance, imu_data, camera_info)
            
        except Exception as e:
            print(f"⚠️ Error in plan_chase_behavior: {e}")
            import traceback
            traceback.print_exc()
            return self._get_stop_control()
    
    def _get_chase_control(self, target_pos, current_pos, current_rot, distance, imu_data=None):
        """추격 제어 - 개선된 방향 계산 (IMU 데이터 사용)"""
        # 타겟 방향 계산 (더 정확한 방법)
        target_direction = self._calculate_direction(current_pos, target_pos)
        
        # CARLA 좌표계에서 올바른 각도 계산
        # 현재 위치에서 타겟을 바라보는 방향 계산
        direction_vector = target_pos - current_pos
        direction_vector.z = 0  # 수평면에서만 계산
        
        # CARLA 좌표계에서의 각도 계산
        # CARLA: X축이 북쪽(0도), Y축이 동쪽(90도)
        # atan2(y, x)를 사용하되 CARLA 좌표계에 맞게 조정
        target_yaw = math.degrees(math.atan2(direction_vector.y, direction_vector.x))
        
        # CARLA 좌표계 보정: 수학적 각도를 CARLA 각도로 변환
        # 수학: 0도가 동쪽, 90도가 북쪽
        # CARLA: 0도가 북쪽, 90도가 동쪽
        target_yaw = 90.0 - target_yaw
        
        # IMU 데이터가 있으면 더 정확한 현재 방향 사용
        if imu_data and 'compass' in imu_data:
            current_yaw = imu_data['compass']
            print(f"🧭 Using IMU compass: {current_yaw:.1f}°")
        else:
            current_yaw = current_rot.yaw
            print(f"🧭 Using CARLA rotation: {current_yaw:.1f}°")
        
        # 각도 차이 계산 (정규화)
        angle_diff = self._normalize_angle(target_yaw - current_yaw)
        
        print(f"🎯 Direction calculation:")
        print(f"   Current pos: ({current_pos.x:.1f}, {current_pos.y:.1f}, {current_pos.z:.1f})")
        print(f"   Target pos: ({target_pos.x:.1f}, {target_pos.y:.1f}, {target_pos.z:.1f})")
        print(f"   Direction vector: ({direction_vector.x:.1f}, {direction_vector.y:.1f}, {direction_vector.z:.1f})")
        print(f"   Current yaw: {current_yaw:.1f}°")
        print(f"   Target yaw: {target_yaw:.1f}°")
        print(f"   Angle diff: {angle_diff:.1f}°")
        
        # 속도 계산 (거리에 따라) - 더 빠른 속도 조절
        if distance > 30.0:
            speed = self.max_speed  # 최대 속도 (40m/s)
        elif distance > 20.0:
            speed = self.max_speed * 0.9  # 90% 속도 (36m/s)
        elif distance > 10.0:
            speed = self.max_speed * 0.8  # 80% 속도 (32m/s)
        elif distance > 5.0:
            speed = self.max_speed * 0.7  # 70% 속도 (28m/s)
        else:
            speed = self.max_speed * 0.5  # 50% 속도 (20m/s)
        
        # 조향 계산 - 부드러운 반응성
        # 데드존 적용 (작은 각도 차이는 무시) - 더 큰 데드존으로 덜 예민하게
        if abs(angle_diff) < 5:  # 5도 미만은 데드존 (2도에서 5도로 증가)
            steer = 0.0
        elif abs(angle_diff) > 60:  # 60도 이상 차이
            steer = np.clip(angle_diff / 300.0, -1.0, 1.0)  # 매우 부드러운 조향
        elif abs(angle_diff) > 30:  # 30-60도 차이
            steer = np.clip(angle_diff / 350.0, -1.0, 1.0)  # 부드러운 조향
        elif abs(angle_diff) > 15:  # 15-30도 차이
            steer = np.clip(angle_diff / 400.0, -1.0, 1.0)  # 부드러운 조향
        else:  # 5-15도 차이
            steer = np.clip(angle_diff / 500.0, -1.0, 1.0)  # 매우 부드러운 조향
        
        # 속도에 따른 조향 조절 (빠를수록 부드럽게)
        speed_factor = min(1.0, distance / 30.0)  # 거리가 멀수록 더 부드럽게 (20에서 30으로 증가)
        steer *= speed_factor
        
        # 조향 변화량 제한 (부드러운 조향을 위해)
        steer_change = steer - self.last_steer
        if abs(steer_change) > self.max_steer_change:
            steer = self.last_steer + np.sign(steer_change) * self.max_steer_change
        
        # 조향값을 -1.0 ~ 1.0 범위로 제한
        steer = np.clip(steer, -1.0, 1.0)
        
        # 현재 조향값 저장 (다음 프레임에서 사용)
        self.last_steer = steer
        
        print(f"   Steer value: {steer:.3f} (angle_diff: {angle_diff:.1f}°, speed_factor: {speed_factor:.2f})")
        
        # 제어 명령 생성 - 안전한 추격을 위한 throttle/brake 조절
        control = carla.VehicleControl()
        
        # 거리에 따른 throttle/brake 조절 (안전한 추격)
        if distance > 20.0:
            throttle = 0.8  # 80% throttle
            brake = 0.0
        elif distance > 10.0:
            throttle = 0.6  # 60% throttle
            brake = 0.0
        elif distance > 5.0:
            throttle = 0.4  # 40% throttle
            brake = 0.0
        elif distance > 3.0:
            throttle = 0.2  # 20% throttle
            brake = 0.1  # 10% brake
        else:
            throttle = 0.0  # 0% throttle
            brake = 0.3  # 30% brake (안전 정지)
        
        control.throttle = throttle
        control.brake = brake
        control.steer = steer
        control.hand_brake = False
        control.manual_gear_shift = False
        
        print(f"🎮 Control: throttle={throttle:.1f}, steer={steer:.2f}, speed={speed:.1f}m/s")
        return control
    
    def _get_camera_chase_control(self, target_bbox, current_pos, current_rot, distance, imu_data=None, camera_info=None):
        """카메라 기반 추격 제어 - 타겟이 화면 중앙에 오도록 조향"""
        try:
            # 타겟 바운딩 박스가 없으면 정지
            if not target_bbox:
                print("⚠️ No target bbox - stopping")
                return self._get_stop_control()
            
            # 카메라 해상도 가져오기 (기본값: 1600x900)
            if camera_info:
                screen_width = camera_info.get('width', 1600)
                screen_height = camera_info.get('height', 900)
            else:
                screen_width = 1600
                screen_height = 900
                
            screen_center_x = screen_width // 2
            screen_center_y = screen_height // 2
            
            # 타겟 바운딩 박스 중앙 계산
            bbox_center_x = (target_bbox['x_min'] + target_bbox['x_max']) // 2
            bbox_center_y = (target_bbox['y_min'] + target_bbox['y_max']) // 2
            
            # 타겟이 화면 중앙에서 얼마나 벗어났는지 계산
            offset_x = bbox_center_x - screen_center_x
            offset_y = bbox_center_y - screen_center_y
            
            # 조향 계산 (X축 오프셋을 기반으로) - 개선된 부드러운 조향
            # 화면 중앙에서 벗어난 정도에 따라 조향 강도 결정
            max_offset = screen_width // 2  # 화면 중앙에서 가장자리까지의 거리
            steer_ratio = (offset_x / max_offset) * self.steering_direction_multiplier  # 조향 방향 조절 가능
            
            print(f"📷 Camera-based control:")
            print(f"   Screen center: ({screen_center_x}, {screen_center_y})")
            print(f"   Target center: ({bbox_center_x}, {bbox_center_y})")
            print(f"   Offset: ({offset_x}, {offset_y})")
            print(f"   Steer ratio: {steer_ratio:.4f} (offset_x: {offset_x}, max_offset: {max_offset})")
            print(f"   Target position: {'RIGHT' if offset_x > 0 else 'LEFT' if offset_x < 0 else 'CENTER'}")
            
            # 조향 강도 조절 (부드러운 반응성 - 카메라 중앙 정렬용)
            # 데드존 적용 (작은 오프셋은 무시) - 더 큰 데드존으로 덜 예민하게
            if abs(steer_ratio) < 0.05:  # 5% 미만은 데드존 (2%에서 5%로 증가)
                steer = 0.0
            elif abs(steer_ratio) > 0.5:  # 큰 오프셋 (50% 이상)
                steer = np.clip(steer_ratio * 0.4, -1.0, 1.0)  # 40% 강도 (매우 부드러움)
            elif abs(steer_ratio) > 0.3:  # 중간 오프셋 (30-50%)
                steer = np.clip(steer_ratio * 0.5, -1.0, 1.0)  # 50% 강도 (부드러움)
            elif abs(steer_ratio) > 0.15:  # 작은 오프셋 (15-30%)
                steer = np.clip(steer_ratio * 0.6, -1.0, 1.0)  # 60% 강도 (적당함)
            else:  # 매우 작은 오프셋 (5-15%)
                steer = np.clip(steer_ratio * 0.7, -1.0, 1.0)  # 70% 강도 (약간 예민)
            
            # 조향 변화량 제한 (부드러운 조향을 위해)
            steer_change = steer - self.last_steer
            if abs(steer_change) > self.max_steer_change:
                steer = self.last_steer + np.sign(steer_change) * self.max_steer_change
            
            # 조향값을 -1.0 ~ 1.0 범위로 제한
            steer = np.clip(steer, -1.0, 1.0)
            
            # 현재 조향값 저장 (다음 프레임에서 사용)
            self.last_steer = steer
            
            # 거리에 따른 속도 조절 (더 빠르게)
            if distance > 30.0:
                speed = self.max_speed * 0.9  # 90% 속도 (36m/s)
            elif distance > 20.0:
                speed = self.max_speed * 0.8  # 80% 속도 (32m/s)
            elif distance > 10.0:
                speed = self.max_speed * 0.7  # 70% 속도 (28m/s)
            elif distance > 5.0:
                speed = self.max_speed * 0.6  # 60% 속도 (24m/s)
            else:
                speed = self.max_speed * 0.5  # 50% 속도 (20m/s)
            
            # 거리에 따른 throttle/brake 조절 (안전한 추격)
            if distance > 20.0:
                throttle = 0.8  # 80% throttle
                brake = 0.0
            elif distance > 10.0:
                throttle = 0.6  # 60% throttle
                brake = 0.0
            elif distance > 5.0:
                throttle = 0.4  # 40% throttle
                brake = 0.0
            elif distance > 3.0:
                throttle = 0.2  # 20% throttle
                brake = 0.1  # 10% brake
            else:
                throttle = 0.0  # 0% throttle
                brake = 0.3  # 30% brake (안전 정지)
            
            # 제어 명령 생성
            control = carla.VehicleControl()
            control.throttle = throttle
            control.brake = brake
            control.steer = steer
            control.hand_brake = False
            control.manual_gear_shift = False
            
            print(f"🎮 Camera Control: throttle={throttle:.1f}, steer={steer:.4f}, speed={speed:.1f}m/s")
            print(f"   📷 Target position: {'RIGHT' if offset_x > 0 else 'LEFT' if offset_x < 0 else 'CENTER'}")
            print(f"   🎯 Steering direction: {'RIGHT' if steer > 0 else 'LEFT' if steer < 0 else 'STRAIGHT'}")
            print(f"   📊 Steer ratio: {steer_ratio:.4f} (offset_x: {offset_x}, multiplier: {self.steering_direction_multiplier})")
            print(f"   🔄 Logic: Target is {'RIGHT' if offset_x > 0 else 'LEFT' if offset_x < 0 else 'CENTER'} → Steer {'RIGHT' if steer > 0 else 'LEFT' if steer < 0 else 'STRAIGHT'}")
            print(f"   ⚡ Steer change: {steer - self.last_steer:.4f} (max_change: {self.max_steer_change})")
            
            # 직접 차량 제어 적용
            self.chase_vehicle.apply_control(control)
            
            # Publish control command via callback if set
            if self.control_publisher_callback:
                self.control_publisher_callback(control)
            
            return control
            
        except Exception as e:
            print(f"⚠️ Error in camera chase control: {e}")
            import traceback
            traceback.print_exc()
            return self._get_stop_control()
    
    def _get_stop_control(self):
        """정지 제어"""
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = 1.0
        control.steer = 0.0
        control.hand_brake = True
        control.manual_gear_shift = False
        
        # 직접 차량 제어 적용
        self.chase_vehicle.apply_control(control)
        
        # Publish control command via callback if set
        if self.control_publisher_callback:
            self.control_publisher_callback(control)
        
        return control
    
    def _calculate_distance(self, pos1, pos2):
        """거리 계산"""
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)
    
    def _calculate_direction(self, from_pos, to_pos):
        """방향 벡터 계산"""
        dx = to_pos.x - from_pos.x
        dy = to_pos.y - from_pos.y
        dz = to_pos.z - from_pos.z
        return carla.Vector3D(x=dx, y=dy, z=dz)
    
    def _normalize_angle(self, angle):
        """각도 정규화 (-180 ~ 180)"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def get_current_state(self):
        """현재 상태 반환"""
        return self.current_state
    
    def is_chasing(self):
        """추격 중인지 확인"""
        return self.current_state == ChaseState.CHASING
    
    def get_chase_status(self):
        """추격 상태 정보 반환"""
        if not self.target_vehicle:
            return "No target"
        
        current_pos = self.chase_vehicle.get_location()
        
        # target_vehicle이 딕셔너리인 경우와 CARLA 객체인 경우 모두 처리
        if isinstance(self.target_vehicle, dict):
            vehicle_id = self.target_vehicle.get('actor_id', 'Unknown')
            if 'world_location' in self.target_vehicle:
                target_pos = carla.Location(
                    x=self.target_vehicle['world_location'][0],
                    y=self.target_vehicle['world_location'][1],
                    z=self.target_vehicle['world_location'][2]
                )
            else:
                return f"Chasing vehicle {vehicle_id}, No location data"
        else:
            vehicle_id = self.target_vehicle.id
            target_pos = self.target_vehicle.get_location()
        
        distance = self._calculate_distance(current_pos, target_pos)
        
        return f"Chasing vehicle {vehicle_id}, Distance: {distance:.1f}m"
    
    def reset_chase(self):
        """추격 리셋"""
        self.stop_chase()
        print("🔄 Chase reset")
