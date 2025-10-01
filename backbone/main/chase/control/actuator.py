"""
Actuator for Chase Vehicle
Handles low-level vehicle actuation
"""

import carla
import numpy as np
import time
from typing import Dict, List, Optional

class Actuator:
    """액추에이터 클래스"""
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        
        # 액추에이터 상태
        self.current_throttle = 0.0
        self.current_brake = 0.0
        self.current_steer = 0.0
        self.current_hand_brake = False
        
        # 액추에이터 히스토리
        self.actuation_history = []
        
        print("⚙️ Actuator initialized")
    
    def apply_control(self, control_command):
        """제어 명령 적용"""
        try:
            # CARLA 제어 객체 생성
            carla_control = carla.VehicleControl()
            
            # 스로틀 제어
            if 'throttle' in control_command:
                self.current_throttle = np.clip(control_command['throttle'], 0.0, 1.0)
                carla_control.throttle = self.current_throttle
            
            # 브레이크 제어
            if 'brake' in control_command:
                self.current_brake = np.clip(control_command['brake'], 0.0, 1.0)
                carla_control.brake = self.current_brake
            
            # 조향 제어
            if 'steer' in control_command:
                self.current_steer = np.clip(control_command['steer'], -1.0, 1.0)
                carla_control.steer = self.current_steer
            
            # 핸드브레이크 제어
            if 'hand_brake' in control_command:
                self.current_hand_brake = bool(control_command['hand_brake'])
                carla_control.hand_brake = self.current_hand_brake
            
            # 후진 제어
            if 'reverse' in control_command:
                carla_control.reverse = bool(control_command['reverse'])
            
            # 제어 적용
            self.vehicle.apply_control(carla_control)
            
            # 액추에이션 히스토리 업데이트
            self._update_actuation_history(control_command)
            
        except Exception as e:
            print(f"⚠️ Error applying control: {e}")
    
    def apply_smooth_control(self, target_control, max_change_rate=0.1):
        """부드러운 제어 적용"""
        try:
            # 현재 제어 상태
            current_control = {
                'throttle': self.current_throttle,
                'brake': self.current_brake,
                'steer': self.current_steer,
                'hand_brake': self.current_hand_brake
            }
            
            # 부드러운 전환 계산
            smooth_control = {}
            
            for key in ['throttle', 'brake', 'steer']:
                if key in target_control:
                    current_val = current_control[key]
                    target_val = target_control[key]
                    
                    # 변화율 제한
                    change = target_val - current_val
                    max_change = max_change_rate
                    
                    if abs(change) > max_change:
                        change = max_change if change > 0 else -max_change
                    
                    smooth_control[key] = current_val + change
                else:
                    smooth_control[key] = current_control[key]
            
            # 핸드브레이크는 즉시 적용
            if 'hand_brake' in target_control:
                smooth_control['hand_brake'] = target_control['hand_brake']
            else:
                smooth_control['hand_brake'] = current_control['hand_brake']
            
            # 제어 적용
            self.apply_control(smooth_control)
            
        except Exception as e:
            print(f"⚠️ Error applying smooth control: {e}")
    
    def emergency_stop(self):
        """비상 정지"""
        try:
            emergency_control = {
                'throttle': 0.0,
                'brake': 1.0,
                'steer': 0.0,
                'hand_brake': True
            }
            
            self.apply_control(emergency_control)
            print("🛑 Emergency stop applied")
            
        except Exception as e:
            print(f"⚠️ Error applying emergency stop: {e}")
    
    def gradual_stop(self, stop_time=2.0):
        """점진적 정지"""
        try:
            start_time = time.time()
            initial_throttle = self.current_throttle
            initial_brake = self.current_brake
            
            while time.time() - start_time < stop_time:
                elapsed = time.time() - start_time
                progress = elapsed / stop_time
                
                # 점진적으로 스로틀 감소, 브레이크 증가
                throttle = initial_throttle * (1 - progress)
                brake = initial_brake + (1.0 - initial_brake) * progress
                
                control = {
                    'throttle': throttle,
                    'brake': brake,
                    'steer': 0.0
                }
                
                self.apply_control(control)
                time.sleep(0.1)  # 100ms 간격
            
            # 최종 정지
            self.apply_control({
                'throttle': 0.0,
                'brake': 1.0,
                'steer': 0.0
            })
            
            print("🛑 Gradual stop completed")
            
        except Exception as e:
            print(f"⚠️ Error applying gradual stop: {e}")
    
    def _update_actuation_history(self, control_command):
        """액추에이션 히스토리 업데이트"""
        try:
            self.actuation_history.append({
                'timestamp': time.time(),
                'throttle': self.current_throttle,
                'brake': self.current_brake,
                'steer': self.current_steer,
                'hand_brake': self.current_hand_brake,
                'command': control_command.copy()
            })
            
            # 최근 200개 액추에이션만 유지
            if len(self.actuation_history) > 200:
                self.actuation_history = self.actuation_history[-200:]
                
        except Exception as e:
            print(f"⚠️ Error updating actuation history: {e}")
    
    def get_current_actuation(self):
        """현재 액추에이션 상태 반환"""
        return {
            'throttle': self.current_throttle,
            'brake': self.current_brake,
            'steer': self.current_steer,
            'hand_brake': self.current_hand_brake
        }
    
    def get_actuation_statistics(self):
        """액추에이션 통계 반환"""
        try:
            if not self.actuation_history:
                return {
                    'total_commands': 0,
                    'avg_throttle': 0.0,
                    'avg_brake': 0.0,
                    'avg_steer': 0.0
                }
            
            total_commands = len(self.actuation_history)
            throttles = [entry['throttle'] for entry in self.actuation_history]
            brakes = [entry['brake'] for entry in self.actuation_history]
            steers = [entry['steer'] for entry in self.actuation_history]
            
            return {
                'total_commands': total_commands,
                'avg_throttle': np.mean(throttles),
                'avg_brake': np.mean(brakes),
                'avg_steer': np.mean(steers),
                'max_throttle': np.max(throttles),
                'max_brake': np.max(brakes),
                'max_steer': np.max(np.abs(steers)),
                'current_throttle': self.current_throttle,
                'current_brake': self.current_brake,
                'current_steer': self.current_steer
            }
            
        except Exception as e:
            print(f"⚠️ Error getting actuation statistics: {e}")
            return {
                'total_commands': 0,
                'avg_throttle': 0.0,
                'avg_brake': 0.0,
                'avg_steer': 0.0
            }
    
    def get_actuation_trend(self, window_size=10):
        """액추에이션 트렌드 분석"""
        try:
            if len(self.actuation_history) < window_size:
                return {
                    'throttle_trend': 0.0,
                    'brake_trend': 0.0,
                    'steer_trend': 0.0
                }
            
            # 최근 window_size개 데이터
            recent_entries = self.actuation_history[-window_size:]
            
            # 트렌드 계산 (선형 회귀)
            timestamps = [entry['timestamp'] for entry in recent_entries]
            throttles = [entry['throttle'] for entry in recent_entries]
            brakes = [entry['brake'] for entry in recent_entries]
            steers = [entry['steer'] for entry in recent_entries]
            
            # 시간 정규화
            timestamps = np.array(timestamps)
            timestamps = (timestamps - timestamps[0]) / (timestamps[-1] - timestamps[0])
            
            # 선형 회귀 계수 계산
            throttle_trend = np.polyfit(timestamps, throttles, 1)[0]
            brake_trend = np.polyfit(timestamps, brakes, 1)[0]
            steer_trend = np.polyfit(timestamps, steers, 1)[0]
            
            return {
                'throttle_trend': throttle_trend,
                'brake_trend': brake_trend,
                'steer_trend': steer_trend
            }
            
        except Exception as e:
            print(f"⚠️ Error getting actuation trend: {e}")
            return {
                'throttle_trend': 0.0,
                'brake_trend': 0.0,
                'steer_trend': 0.0
            }
    
    def reset_actuation(self):
        """액추에이션 리셋"""
        try:
            self.current_throttle = 0.0
            self.current_brake = 0.0
            self.current_steer = 0.0
            self.current_hand_brake = False
            
            # 정지 제어 적용
            self.apply_control({
                'throttle': 0.0,
                'brake': 0.0,
                'steer': 0.0,
                'hand_brake': False
            })
            
            print("⚙️ Actuation reset")
            
        except Exception as e:
            print(f"⚠️ Error resetting actuation: {e}")
    
    def cleanup(self):
        """리소스 정리"""
        try:
            self.reset_actuation()
            self.actuation_history.clear()
            print("⚙️ Actuator cleaned up")
            
        except Exception as e:
            print(f"⚠️ Error cleaning up actuator: {e}")

