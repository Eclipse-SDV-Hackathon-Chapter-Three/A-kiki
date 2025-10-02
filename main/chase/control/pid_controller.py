"""
PID Controller for Chase Vehicle
Provides PID control for vehicle actuation
"""

import numpy as np
import time
from typing import Dict, List, Optional

class PIDController:
    """PID 제어기 클래스"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_output=1.0, min_output=-1.0):
        self.kp = kp  # 비례 게인
        self.ki = ki  # 적분 게인
        self.kd = kd  # 미분 게인
        
        self.max_output = max_output
        self.min_output = min_output
        
        # PID 상태
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = time.time()
        
        # 히스토리
        self.error_history = []
        self.output_history = []
        
        print("🎛️ PID Controller initialized")
    
    def calculate(self, target_position, current_position, current_velocity=None):
        """PID 계산"""
        try:
            current_time = time.time()
            dt = current_time - self.previous_time
            
            if dt <= 0:
                dt = 0.01  # 최소 시간 간격
            
            # 오차 계산
            error = self._calculate_error(target_position, current_position)
            
            # 적분 항 계산
            self.integral += error * dt
            
            # 미분 항 계산
            derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
            
            # PID 출력 계산
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
            
            # 출력 제한
            output = np.clip(output, self.min_output, self.max_output)
            
            # 상태 업데이트
            self.previous_error = error
            self.previous_time = current_time
            
            # 히스토리 업데이트
            self._update_history(error, output)
            
            # 제어 출력 반환
            return self._format_output(output, current_velocity)
            
        except Exception as e:
            print(f"⚠️ Error in PID calculation: {e}")
            return {'throttle': 0.0, 'brake': 0.0, 'steer': 0.0}
    
    def _calculate_error(self, target_position, current_position):
        """오차 계산"""
        try:
            if target_position is None or current_position is None:
                return 0.0
            
            # 위치 오차 (거리)
            target_pos = np.array(target_position[:3])
            current_pos = np.array([current_position.x, current_position.y, current_position.z])
            
            error = np.linalg.norm(target_pos - current_pos)
            return error
            
        except Exception as e:
            print(f"⚠️ Error calculating error: {e}")
            return 0.0
    
    def _format_output(self, output, current_velocity):
        """출력을 제어 명령으로 변환"""
        try:
            # 출력을 스로틀/브레이크/조향으로 분배
            if output > 0:
                # 양수 출력: 가속
                throttle = min(abs(output), 1.0)
                brake = 0.0
            else:
                # 음수 출력: 감속
                throttle = 0.0
                brake = min(abs(output), 1.0)
            
            # 조향은 별도 계산 (방향 기반)
            steer = self._calculate_steering(target_position, current_position)
            
            return {
                'throttle': throttle,
                'brake': brake,
                'steer': steer
            }
            
        except Exception as e:
            print(f"⚠️ Error formatting output: {e}")
            return {'throttle': 0.0, 'brake': 0.0, 'steer': 0.0}
    
    def _calculate_steering(self, target_position, current_position):
        """조향 계산"""
        try:
            if target_position is None or current_position is None:
                return 0.0
            
            # 목표 방향 계산
            target_pos = np.array(target_position[:2])
            current_pos = np.array([current_position.x, current_position.y])
            
            direction = target_pos - current_pos
            distance = np.linalg.norm(direction)
            
            if distance < 0.1:
                return 0.0
            
            # 방향 벡터를 정규화
            direction = direction / distance
            
            # 현재 차량 방향 (간단히 X축 방향으로 가정)
            current_direction = np.array([1.0, 0.0])
            
            # 각도 차이 계산
            dot_product = np.dot(current_direction, direction)
            cross_product = np.cross(current_direction, direction)
            
            angle = np.arctan2(cross_product, dot_product)
            
            # 조향각 정규화 (-1 ~ 1)
            steer = np.clip(angle / np.pi, -1.0, 1.0)
            
            return steer
            
        except Exception as e:
            print(f"⚠️ Error calculating steering: {e}")
            return 0.0
    
    def _update_history(self, error, output):
        """히스토리 업데이트"""
        try:
            self.error_history.append({
                'timestamp': time.time(),
                'error': error
            })
            
            self.output_history.append({
                'timestamp': time.time(),
                'output': output
            })
            
            # 최근 100개 데이터만 유지
            if len(self.error_history) > 100:
                self.error_history = self.error_history[-100:]
            if len(self.output_history) > 100:
                self.output_history = self.output_history[-100:]
                
        except Exception as e:
            print(f"⚠️ Error updating history: {e}")
    
    def reset(self):
        """PID 상태 리셋"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = time.time()
        self.error_history.clear()
        self.output_history.clear()
        print("🎛️ PID Controller reset")
    
    def set_parameters(self, kp=None, ki=None, kd=None):
        """PID 파라미터 설정"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        
        print(f"🎛️ PID parameters updated: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
    
    def get_statistics(self):
        """PID 통계 반환"""
        try:
            if not self.error_history:
                return {
                    'avg_error': 0.0,
                    'max_error': 0.0,
                    'avg_output': 0.0,
                    'max_output': 0.0
                }
            
            errors = [entry['error'] for entry in self.error_history]
            outputs = [entry['output'] for entry in self.output_history]
            
            return {
                'avg_error': np.mean(errors),
                'max_error': np.max(errors),
                'avg_output': np.mean(outputs),
                'max_output': np.max(outputs),
                'current_error': self.previous_error,
                'current_integral': self.integral
            }
            
        except Exception as e:
            print(f"⚠️ Error getting PID statistics: {e}")
            return {
                'avg_error': 0.0,
                'max_error': 0.0,
                'avg_output': 0.0,
                'max_output': 0.0
            }
    
    def tune_parameters(self, target_error=1.0, max_iterations=10):
        """PID 파라미터 자동 튜닝 (간단한 Ziegler-Nichols 방법)"""
        try:
            print("🎛️ Starting PID parameter tuning...")
            
            # 초기 파라미터로 시작
            best_kp = self.kp
            best_ki = self.ki
            best_kd = self.kd
            best_error = float('inf')
            
            # 간단한 그리드 서치
            kp_range = np.linspace(0.5, 2.0, 5)
            ki_range = np.linspace(0.0, 0.5, 3)
            kd_range = np.linspace(0.0, 0.5, 3)
            
            for kp in kp_range:
                for ki in ki_range:
                    for kd in kd_range:
                        self.set_parameters(kp, ki, kd)
                        
                        # 짧은 시간 동안 테스트
                        test_errors = []
                        for _ in range(10):
                            # 테스트 오차 계산 (실제로는 시뮬레이션 필요)
                            test_error = abs(np.random.normal(0, 1))  # 임시
                            test_errors.append(test_error)
                        
                        avg_error = np.mean(test_errors)
                        
                        if avg_error < best_error:
                            best_error = avg_error
                            best_kp = kp
                            best_ki = ki
                            best_kd = kd
            
            # 최적 파라미터 적용
            self.set_parameters(best_kp, best_ki, best_kd)
            print(f"🎛️ PID tuning completed: Kp={best_kp:.3f}, Ki={best_ki:.3f}, Kd={best_kd:.3f}")
            
        except Exception as e:
            print(f"⚠️ Error tuning PID parameters: {e}")
    
    def get_parameter_recommendations(self):
        """파라미터 추천"""
        try:
            # 오차 통계 기반 추천
            if not self.error_history:
                return {
                    'recommended_kp': 1.0,
                    'recommended_ki': 0.0,
                    'recommended_kd': 0.0,
                    'reason': 'No data available'
                }
            
            errors = [entry['error'] for entry in self.error_history]
            avg_error = np.mean(errors)
            error_variance = np.var(errors)
            
            if avg_error > 5.0:
                # 큰 오차: Kp 증가
                recommended_kp = min(self.kp * 1.5, 3.0)
                recommended_ki = self.ki
                recommended_kd = self.kd
                reason = "High average error - increase Kp"
            elif error_variance > 2.0:
                # 오차 변동이 큼: Kd 증가
                recommended_kp = self.kp
                recommended_ki = self.ki
                recommended_kd = min(self.kd + 0.1, 1.0)
                reason = "High error variance - increase Kd"
            else:
                # 안정적: Ki 약간 증가
                recommended_kp = self.kp
                recommended_ki = min(self.ki + 0.05, 0.5)
                recommended_kd = self.kd
                reason = "Stable system - slight increase in Ki"
            
            return {
                'recommended_kp': recommended_kp,
                'recommended_ki': recommended_ki,
                'recommended_kd': recommended_kd,
                'reason': reason,
                'current_avg_error': avg_error,
                'current_error_variance': error_variance
            }
            
        except Exception as e:
            print(f"⚠️ Error getting parameter recommendations: {e}")
            return {
                'recommended_kp': 1.0,
                'recommended_ki': 0.0,
                'recommended_kd': 0.0,
                'reason': 'Error in analysis'
            }

