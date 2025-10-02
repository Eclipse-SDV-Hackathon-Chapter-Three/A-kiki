#!/usr/bin/env python3

"""
Simple Path Planner
간단하고 효과적인 경로 계획자
"""

import numpy as np
import math
import carla
from typing import Tuple, Optional

class SimplePathPlanner:
    """간단한 경로 계획자"""
    
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.map = world.get_map()
        
        # 추격 파라미터
        self.max_speed = 20.0  # m/s (72 km/h)
        self.follow_distance = 15.0  # m
        self.approach_distance = 50.0  # m
        self.min_distance = 5.0  # m
        
        # 제어 파라미터
        self.max_steer_angle = 0.8
        self.speed_kp = 2.0
        self.steer_kp = 1.5
        
        print("🗺️ Simple Path Planner initialized")
    
    def plan_chase_path(self, target_position: Tuple[float, float, float], 
                       target_distance: float) -> carla.VehicleControl:
        """추격 경로 계획"""
        try:
            if not target_position:
                return self._create_stop_control()
            
            # 현재 차량 상태
            current_transform = self.vehicle.get_transform()
            current_location = current_transform.location
            current_rotation = current_transform.rotation
            current_velocity = self.vehicle.get_velocity()
            current_speed = math.sqrt(current_velocity.x**2 + current_velocity.y**2)
            
            # 타겟 방향 계산
            target_vector = np.array([
                target_position[0] - current_location.x,
                target_position[1] - current_location.y,
                target_position[2] - current_location.z
            ])
            
            # 거리 계산
            distance = np.linalg.norm(target_vector[:2])
            
            # 너무 가까우면 정지
            if distance < self.min_distance:
                return self._create_stop_control()
            
            # 목표 속도 계산
            target_speed = self._calculate_target_speed(distance, current_speed)
            
            # 스티어링 계산
            steer_angle = self._calculate_steering(target_vector, current_rotation)
            
            # 제어 명령 생성
            control = self._create_control_command(target_speed, steer_angle, current_speed)
            
            return control
            
        except Exception as e:
            print(f"⚠️ Error in plan_chase_path: {e}")
            return self._create_stop_control()
    
    def _calculate_target_speed(self, distance: float, current_speed: float) -> float:
        """목표 속도 계산"""
        try:
            if distance > self.approach_distance:
                # 접근 모드 - 빠른 속도
                target_speed = self.max_speed
            elif distance > self.follow_distance:
                # 추격 모드 - 중간 속도
                target_speed = self.max_speed * 0.7
            else:
                # 근접 모드 - 느린 속도
                target_speed = self.max_speed * 0.4
            
            # 현재 속도에 따른 조정
            if current_speed > target_speed:
                target_speed = current_speed * 0.8  # 감속
            
            return min(target_speed, self.max_speed)
            
        except Exception as e:
            print(f"⚠️ Error calculating target speed: {e}")
            return self.max_speed * 0.5
    
    def _calculate_steering(self, target_vector: np.ndarray, current_rotation: carla.Rotation) -> float:
        """스티어링 계산"""
        try:
            # 현재 차량 방향
            current_yaw = math.radians(current_rotation.yaw)
            current_direction = np.array([math.cos(current_yaw), math.sin(current_yaw)])
            
            # 타겟 방향
            target_direction = target_vector[:2]
            if np.linalg.norm(target_direction) > 0:
                target_direction = target_direction / np.linalg.norm(target_direction)
            else:
                target_direction = np.array([1, 0])
            
            # 각도 차이 계산
            dot_product = np.dot(current_direction, target_direction)
            angle_diff = math.acos(np.clip(dot_product, -1.0, 1.0))
            
            # 스티어링 방향
            cross_product = np.cross(current_direction, target_direction)
            steer_direction = 1 if cross_product > 0 else -1
            
            # 스티어링 각도
            steer_angle = steer_direction * min(angle_diff, self.max_steer_angle)
            
            return steer_angle
            
        except Exception as e:
            print(f"⚠️ Error calculating steering: {e}")
            return 0.0
    
    def _create_control_command(self, target_speed: float, steer_angle: float, current_speed: float) -> carla.VehicleControl:
        """제어 명령 생성"""
        try:
            control = carla.VehicleControl()
            
            # 속도 제어
            speed_diff = target_speed - current_speed
            
            if speed_diff > 0:
                # 가속
                control.throttle = min(speed_diff / self.speed_kp, 1.0)
                control.brake = 0.0
            else:
                # 감속
                control.throttle = 0.0
                control.brake = min(abs(speed_diff) / self.speed_kp, 1.0)
            
            # 스티어링 제어
            control.steer = np.clip(steer_angle, -self.max_steer_angle, self.max_steer_angle)
            
            # 기타 제어
            control.hand_brake = False
            control.reverse = False
            
            return control
            
        except Exception as e:
            print(f"⚠️ Error creating control command: {e}")
            return self._create_stop_control()
    
    def _create_stop_control(self) -> carla.VehicleControl:
        """정지 제어 명령"""
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = 1.0
        control.steer = 0.0
        control.hand_brake = True
        control.reverse = False
        return control
    
    def set_parameters(self, max_speed: Optional[float] = None,
                      follow_distance: Optional[float] = None,
                      approach_distance: Optional[float] = None,
                      min_distance: Optional[float] = None):
        """파라미터 설정"""
        if max_speed is not None:
            self.max_speed = max_speed
        if follow_distance is not None:
            self.follow_distance = follow_distance
        if approach_distance is not None:
            self.approach_distance = approach_distance
        if min_distance is not None:
            self.min_distance = min_distance
        
        print(f"🗺️ Parameters updated: max_speed={self.max_speed}, follow_distance={self.follow_distance}")
    
    def get_status(self) -> dict:
        """상태 정보 반환"""
        return {
            'max_speed': self.max_speed,
            'follow_distance': self.follow_distance,
            'approach_distance': self.approach_distance,
            'min_distance': self.min_distance
        }
