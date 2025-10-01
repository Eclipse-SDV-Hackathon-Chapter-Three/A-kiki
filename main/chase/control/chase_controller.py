#!/usr/bin/env python3
"""
Chase Controller
추격 제어 클래스
"""

import carla
import time
import math

class ChaseController:
    """추격 제어 클래스"""
    
    def __init__(self, vehicle, chase_planner):
        self.vehicle = vehicle
        self.chase_planner = chase_planner
        self.is_chasing = False
        self.target_actor = None
        self.last_control_time = 0.0
        self.control_interval = 0.1  # 100ms 제어 간격
        
        # 제어 파라미터
        self.max_speed = 8.0  # m/s
        self.min_distance = 5.0  # m
        self.chase_distance = 100.0  # m
        
        # 통계
        self.chase_statistics = {
            'chase_duration': 0.0,
            'max_speed_reached': 0.0,
            'average_distance': 0.0,
            'control_commands': 0
        }
    
    def start_chase(self, target_actor):
        """추격 시작"""
        try:
            self.target_actor = target_actor
            self.is_chasing = True
            self.chase_statistics['chase_duration'] = 0.0
            print(f"🚔 Chase started for target: {target_actor}")
            return True
        except Exception as e:
            print(f"❌ Error starting chase: {e}")
            return False
    
    def stop_chase(self):
        """추격 중지"""
        try:
            self.is_chasing = False
            self.target_actor = None
            print("🛑 Chase stopped")
        except Exception as e:
            print(f"⚠️ Error stopping chase: {e}")
    
    def execute_chase_control(self):
        """추격 제어 실행"""
        try:
            if not self.is_chasing or not self.target_actor:
                return
            
            current_time = time.time()
            
            # 제어 간격 체크
            if current_time - self.last_control_time < self.control_interval:
                return
            
            self.last_control_time = current_time
            
            # 타겟이 유효한지 확인
            if not self.target_actor.is_alive:
                print("⚠️ Target actor is no longer alive, stopping chase")
                self.stop_chase()
                return
            
            # 타겟까지의 거리 계산
            target_location = self.target_actor.get_location()
            vehicle_location = self.vehicle.get_location()
            distance = vehicle_location.distance(target_location)
            
            # 추격 거리 체크
            if distance > self.chase_distance:
                print(f"⚠️ Target too far away ({distance:.1f}m), stopping chase")
                self.stop_chase()
                return
            
            # 제어 명령 생성
            control_command = self._generate_control_command(target_location, distance)
            
            # 제어 명령 적용
            if control_command:
                self.vehicle.apply_control(control_command)
                self.chase_statistics['control_commands'] += 1
                
                # 통계 업데이트
                self._update_chase_statistics(distance)
            
        except Exception as e:
            print(f"⚠️ Error executing chase control: {e}")
    
    def _generate_control_command(self, target_location, distance):
        """제어 명령 생성"""
        try:
            # 기본 제어 명령
            control = carla.VehicleControl()
            
            # 타겟 방향 계산
            vehicle_location = self.vehicle.get_location()
            vehicle_rotation = self.vehicle.get_rotation()
            
            # 타겟 방향 벡터
            target_vector = target_location - vehicle_location
            target_vector.z = 0  # 수평면만 고려
            
            # 차량의 현재 방향 벡터
            vehicle_forward = carla.Vector3D(
                math.cos(math.radians(vehicle_rotation.yaw)),
                math.sin(math.radians(vehicle_rotation.yaw)),
                0
            )
            
            # 각도 차이 계산
            dot_product = vehicle_forward.x * target_vector.x + vehicle_forward.y * target_vector.y
            magnitude_product = (vehicle_forward.x**2 + vehicle_forward.y**2)**0.5 * (target_vector.x**2 + target_vector.y**2)**0.5
            
            if magnitude_product > 0:
                cos_angle = dot_product / magnitude_product
                cos_angle = max(-1, min(1, cos_angle))  # 클램핑
                angle_diff = math.acos(cos_angle)
                
                # 외적을 이용한 방향 판단
                cross_product = vehicle_forward.x * target_vector.y - vehicle_forward.y * target_vector.x
                if cross_product < 0:
                    angle_diff = -angle_diff
                
                # 조향 각도 계산 (라디안을 도로 변환)
                steering_angle = math.degrees(angle_diff)
                steering_angle = max(-1, min(1, steering_angle / 30))  # 정규화
                
                control.steer = steering_angle
            else:
                control.steer = 0.0
            
            # 속도 제어
            if distance > self.min_distance:
                # 가속
                speed_factor = min(1.0, distance / self.chase_distance)
                control.throttle = 0.5 + 0.5 * speed_factor
                control.brake = 0.0
            else:
                # 브레이크
                control.throttle = 0.0
                control.brake = 0.5
            
            # 속도 제한
            current_velocity = self.vehicle.get_velocity()
            current_speed = (current_velocity.x**2 + current_velocity.y**2 + current_velocity.z**2)**0.5
            
            if current_speed > self.max_speed:
                control.throttle = 0.0
                control.brake = 0.3
            
            return control
            
        except Exception as e:
            print(f"⚠️ Error generating control command: {e}")
            return None
    
    def _update_chase_statistics(self, distance):
        """추격 통계 업데이트"""
        try:
            # 추격 시간 업데이트
            self.chase_statistics['chase_duration'] += self.control_interval
            
            # 최대 속도 업데이트
            current_velocity = self.vehicle.get_velocity()
            current_speed = (current_velocity.x**2 + current_velocity.y**2 + current_velocity.z**2)**0.5
            self.chase_statistics['max_speed_reached'] = max(
                self.chase_statistics['max_speed_reached'], 
                current_speed
            )
            
            # 평균 거리 업데이트 (간단한 이동 평균)
            if self.chase_statistics['average_distance'] == 0:
                self.chase_statistics['average_distance'] = distance
            else:
                alpha = 0.1  # 학습률
                self.chase_statistics['average_distance'] = (
                    alpha * distance + 
                    (1 - alpha) * self.chase_statistics['average_distance']
                )
            
        except Exception as e:
            print(f"⚠️ Error updating chase statistics: {e}")
    
    def get_chase_status(self):
        """추격 상태 반환"""
        return {
            'is_chasing': self.is_chasing,
            'target_actor': self.target_actor,
            'statistics': self.chase_statistics.copy()
        }
    
    def reset(self):
        """추격 제어기 리셋"""
        try:
            self.is_chasing = False
            self.target_actor = None
            self.last_control_time = 0.0
            self.chase_statistics = {
                'chase_duration': 0.0,
                'max_speed_reached': 0.0,
                'average_distance': 0.0,
                'control_commands': 0
            }
            print("🔄 Chase controller reset")
        except Exception as e:
            print(f"⚠️ Error resetting chase controller: {e}")

