"""
Chase Vehicle Controller
Controls the chase vehicle based on planned trajectory
"""

import carla
import numpy as np
import math
import time
from typing import Dict, List, Optional

class ChaseVehicleController:
    """추격차량 제어 클래스"""
    
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        
        # 제어 파라미터
        self.max_steer_angle = 0.5  # 최대 조향각 (라디안)
        self.max_throttle = 1.0
        self.max_brake = 1.0
        
        # PID 제어기
        self.pid_controller = None
        
        # 현재 제어 상태
        self.current_control = carla.VehicleControl()
        self.control_history = []
        
        print("🎮 Chase Vehicle Controller initialized")
    
    def set_pid_controller(self, pid_controller):
        """PID 제어기 설정"""
        self.pid_controller = pid_controller
    
    def control_vehicle(self, target_position, target_velocity=None, target_acceleration=None):
        """차량 제어"""
        try:
            if target_position is None:
                # 안전 정지
                self._emergency_stop()
                return
            
            # 현재 차량 상태
            current_transform = self.vehicle.get_transform()
            current_velocity = self.vehicle.get_velocity()
            
            # 목표 위치로의 제어 계산
            control = self._calculate_control(
                current_transform, current_velocity, 
                target_position, target_velocity, target_acceleration
            )
            
            # 제어 적용
            self.vehicle.apply_control(control)
            self.current_control = control
            
            # 제어 히스토리 업데이트
            self._update_control_history(control)
            
        except Exception as e:
            print(f"⚠️ Error controlling vehicle: {e}")
            self._emergency_stop()
    
    def _calculate_control(self, current_transform, current_velocity, 
                          target_position, target_velocity, target_acceleration):
        """제어 계산"""
        try:
            control = carla.VehicleControl()
            
            # 현재 위치와 목표 위치
            current_pos = np.array([current_transform.location.x, current_transform.location.y])
            target_pos = np.array(target_position[:2])
            
            # 거리와 방향 계산
            distance = np.linalg.norm(target_pos - current_pos)
            direction = target_pos - current_pos
            
            if distance > 0.1:  # 최소 거리 체크
                # 목표 방향으로의 각도 계산
                target_angle = math.atan2(direction[1], direction[0])
                current_angle = math.radians(current_transform.rotation.yaw)
                
                # 조향각 계산
                angle_diff = self._normalize_angle(target_angle - current_angle)
                steer_angle = np.clip(angle_diff / math.pi, -1.0, 1.0)
                control.steer = steer_angle
                
                # 속도 제어
                if target_velocity is not None:
                    target_speed = np.linalg.norm(target_velocity[:2])
                    current_speed = np.linalg.norm([current_velocity.x, current_velocity.y])
                    
                    speed_diff = target_speed - current_speed
                    
                    if speed_diff > 0.5:  # 가속 필요
                        control.throttle = min(speed_diff / 10.0, self.max_throttle)
                        control.brake = 0.0
                    elif speed_diff < -0.5:  # 감속 필요
                        control.throttle = 0.0
                        control.brake = min(-speed_diff / 10.0, self.max_brake)
                    else:  # 유지
                        control.throttle = 0.1
                        control.brake = 0.0
                else:
                    # 기본 속도 제어
                    if distance > 10.0:
                        control.throttle = 0.5
                        control.brake = 0.0
                    elif distance > 5.0:
                        control.throttle = 0.3
                        control.brake = 0.0
                    else:
                        control.throttle = 0.1
                        control.brake = 0.0
                
                # PID 제어기 사용 (설정된 경우)
                if self.pid_controller:
                    pid_output = self.pid_controller.calculate(
                        target_position, current_transform.location, current_velocity
                    )
                    
                    # PID 출력을 제어에 반영
                    if 'throttle' in pid_output:
                        control.throttle = np.clip(pid_output['throttle'], 0, self.max_throttle)
                    if 'brake' in pid_output:
                        control.brake = np.clip(pid_output['brake'], 0, self.max_brake)
                    if 'steer' in pid_output:
                        control.steer = np.clip(pid_output['steer'], -1, 1)
            
            else:
                # 목표에 도달한 경우 정지
                control.throttle = 0.0
                control.brake = 0.5
                control.steer = 0.0
            
            return control
            
        except Exception as e:
            print(f"⚠️ Error calculating control: {e}")
            return carla.VehicleControl()
    
    def _normalize_angle(self, angle):
        """각도 정규화 (-π ~ π)"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _emergency_stop(self):
        """비상 정지"""
        try:
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            control.hand_brake = True
            
            self.vehicle.apply_control(control)
            self.current_control = control
            
            print("🛑 Emergency stop applied")
            
        except Exception as e:
            print(f"⚠️ Error applying emergency stop: {e}")
    
    def _update_control_history(self, control):
        """제어 히스토리 업데이트"""
        try:
            self.control_history.append({
                'timestamp': time.time(),
                'throttle': control.throttle,
                'brake': control.brake,
                'steer': control.steer,
                'hand_brake': control.hand_brake
            })
            
            # 최근 100개 제어 명령만 유지
            if len(self.control_history) > 100:
                self.control_history = self.control_history[-100:]
                
        except Exception as e:
            print(f"⚠️ Error updating control history: {e}")
    
    def get_control_statistics(self):
        """제어 통계 반환"""
        try:
            if not self.control_history:
                return {
                    'total_commands': 0,
                    'avg_throttle': 0.0,
                    'avg_brake': 0.0,
                    'avg_steer': 0.0
                }
            
            total_commands = len(self.control_history)
            throttles = [entry['throttle'] for entry in self.control_history]
            brakes = [entry['brake'] for entry in self.control_history]
            steers = [entry['steer'] for entry in self.control_history]
            
            return {
                'total_commands': total_commands,
                'avg_throttle': np.mean(throttles),
                'avg_brake': np.mean(brakes),
                'avg_steer': np.mean(steers),
                'current_throttle': self.current_control.throttle,
                'current_brake': self.current_control.brake,
                'current_steer': self.current_control.steer
            }
            
        except Exception as e:
            print(f"⚠️ Error getting control statistics: {e}")
            return {
                'total_commands': 0,
                'avg_throttle': 0.0,
                'avg_brake': 0.0,
                'avg_steer': 0.0
            }
    
    def set_control_parameters(self, max_steer_angle=None, max_throttle=None, max_brake=None):
        """제어 파라미터 설정"""
        if max_steer_angle is not None:
            self.max_steer_angle = max_steer_angle
        if max_throttle is not None:
            self.max_throttle = max_throttle
        if max_brake is not None:
            self.max_brake = max_brake
        
        print(f"🎮 Control parameters updated: steer={self.max_steer_angle}, throttle={self.max_throttle}, brake={self.max_brake}")
    
    def get_current_control(self):
        """현재 제어 상태 반환"""
        return self.current_control
    
    def stop_vehicle(self):
        """차량 정지"""
        try:
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 0.5
            control.steer = 0.0
            
            self.vehicle.apply_control(control)
            self.current_control = control
            
            print("🛑 Vehicle stopped")
            
        except Exception as e:
            print(f"⚠️ Error stopping vehicle: {e}")
    
    def destroy(self):
        """리소스 정리"""
        try:
            self.stop_vehicle()
            self.control_history.clear()
            print("🎮 Chase Vehicle Controller destroyed")
            
        except Exception as e:
            print(f"⚠️ Error destroying controller: {e}")

