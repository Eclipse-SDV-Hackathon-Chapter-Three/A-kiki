"""
Trajectory Generator for Chase Vehicle
Generates smooth trajectories for vehicle control
"""

import numpy as np
import math
from typing import List, Tuple, Optional

class TrajectoryGenerator:
    """궤적 생성 클래스"""
    
    def __init__(self, max_velocity=50.0, max_acceleration=5.0, max_jerk=10.0):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_jerk = max_jerk
        
        self.current_trajectory = []
        self.trajectory_history = []
        
        print("📈 Trajectory Generator initialized")
    
    def generate_trajectory(self, current_position, current_velocity, target_waypoint, 
                           target_velocity=None, time_horizon=2.0, dt=0.1):
        """궤적 생성"""
        try:
            if target_waypoint is None:
                return []
            
            # 현재 상태
            current_pos = np.array([current_position.x, current_position.y, current_position.z])
            current_vel = np.array([current_velocity.x, current_velocity.y, current_velocity.z])
            
            # 목표 상태
            target_pos = np.array(target_waypoint[:3])
            target_vel = np.array(target_velocity[:3]) if target_velocity else np.zeros(3)
            
            # 궤적 생성
            trajectory = self._generate_quintic_polynomial_trajectory(
                current_pos, current_vel, target_pos, target_vel, time_horizon, dt
            )
            
            # 궤적 검증 및 최적화
            trajectory = self._validate_and_optimize_trajectory(trajectory)
            
            self.current_trajectory = trajectory
            self._update_trajectory_history(trajectory)
            
            return trajectory
            
        except Exception as e:
            print(f"⚠️ Error generating trajectory: {e}")
            return []
    
    def _generate_quintic_polynomial_trajectory(self, start_pos, start_vel, end_pos, end_vel, 
                                               time_horizon, dt):
        """5차 다항식 궤적 생성"""
        try:
            num_points = int(time_horizon / dt)
            trajectory = []
            
            for i in range(num_points + 1):
                t = i * dt
                normalized_t = t / time_horizon
                
                # 5차 다항식 계수 계산
                pos = self._quintic_polynomial(start_pos, start_vel, end_pos, end_vel, normalized_t)
                vel = self._quintic_polynomial_derivative(start_pos, start_vel, end_pos, end_vel, normalized_t, time_horizon)
                acc = self._quintic_polynomial_second_derivative(start_pos, start_vel, end_pos, end_vel, normalized_t, time_horizon)
                
                trajectory.append({
                    'time': t,
                    'position': pos.tolist(),
                    'velocity': vel.tolist(),
                    'acceleration': acc.tolist()
                })
            
            return trajectory
            
        except Exception as e:
            print(f"⚠️ Error generating quintic polynomial trajectory: {e}")
            return []
    
    def _quintic_polynomial(self, start_pos, start_vel, end_pos, end_vel, t):
        """5차 다항식 위치 계산"""
        try:
            # 5차 다항식: p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
            # 경계 조건: p(0) = start_pos, p'(0) = start_vel, p(1) = end_pos, p'(1) = end_vel
            # p''(0) = 0, p''(1) = 0 (가속도 경계 조건)
            
            a0 = start_pos
            a1 = start_vel
            a2 = np.zeros(3)
            a3 = 20 * (end_pos - start_pos) - 8 * end_vel - 12 * start_vel
            a4 = 30 * (start_pos - end_pos) + 14 * end_vel + 16 * start_vel
            a5 = 12 * (end_pos - start_pos) - 6 * (end_vel + start_vel)
            
            t2 = t * t
            t3 = t2 * t
            t4 = t3 * t
            t5 = t4 * t
            
            return a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5
            
        except Exception as e:
            print(f"⚠️ Error calculating quintic polynomial: {e}")
            return start_pos
    
    def _quintic_polynomial_derivative(self, start_pos, start_vel, end_pos, end_vel, t, time_horizon):
        """5차 다항식 1차 도함수 (속도)"""
        try:
            a1 = start_vel
            a2 = np.zeros(3)
            a3 = 20 * (end_pos - start_pos) - 8 * end_vel - 12 * start_vel
            a4 = 30 * (start_pos - end_pos) + 14 * end_vel + 16 * start_vel
            a5 = 12 * (end_pos - start_pos) - 6 * (end_vel + start_vel)
            
            t2 = t * t
            t3 = t2 * t
            t4 = t3 * t
            
            return (a1 + 2*a2*t + 3*a3*t2 + 4*a4*t3 + 5*a5*t4) / time_horizon
            
        except Exception as e:
            print(f"⚠️ Error calculating quintic polynomial derivative: {e}")
            return start_vel
    
    def _quintic_polynomial_second_derivative(self, start_pos, start_vel, end_pos, end_vel, t, time_horizon):
        """5차 다항식 2차 도함수 (가속도)"""
        try:
            a2 = np.zeros(3)
            a3 = 20 * (end_pos - start_pos) - 8 * end_vel - 12 * start_vel
            a4 = 30 * (start_pos - end_pos) + 14 * end_vel + 16 * start_vel
            a5 = 12 * (end_pos - start_pos) - 6 * (end_vel + start_vel)
            
            t2 = t * t
            t3 = t2 * t
            
            return (2*a2 + 6*a3*t + 12*a4*t2 + 20*a5*t3) / (time_horizon * time_horizon)
            
        except Exception as e:
            print(f"⚠️ Error calculating quintic polynomial second derivative: {e}")
            return np.zeros(3)
    
    def _validate_and_optimize_trajectory(self, trajectory):
        """궤적 검증 및 최적화"""
        try:
            if not trajectory:
                return trajectory
            
            optimized_trajectory = []
            
            for i, point in enumerate(trajectory):
                # 속도 제한
                velocity = np.array(point['velocity'])
                speed = np.linalg.norm(velocity)
                
                if speed > self.max_velocity:
                    velocity = velocity * (self.max_velocity / speed)
                    point['velocity'] = velocity.tolist()
                
                # 가속도 제한
                acceleration = np.array(point['acceleration'])
                acc_magnitude = np.linalg.norm(acceleration)
                
                if acc_magnitude > self.max_acceleration:
                    acceleration = acceleration * (self.max_acceleration / acc_magnitude)
                    point['acceleration'] = acceleration.tolist()
                
                # 저크 제한 (가속도 변화율)
                if i > 0:
                    prev_acc = np.array(trajectory[i-1]['acceleration'])
                    jerk = acceleration - prev_acc
                    jerk_magnitude = np.linalg.norm(jerk)
                    
                    if jerk_magnitude > self.max_jerk:
                        jerk = jerk * (self.max_jerk / jerk_magnitude)
                        acceleration = prev_acc + jerk
                        point['acceleration'] = acceleration.tolist()
                
                optimized_trajectory.append(point)
            
            return optimized_trajectory
            
        except Exception as e:
            print(f"⚠️ Error validating and optimizing trajectory: {e}")
            return trajectory
    
    def _update_trajectory_history(self, trajectory):
        """궤적 히스토리 업데이트"""
        try:
            self.trajectory_history.append({
                'timestamp': time.time(),
                'trajectory': trajectory.copy(),
                'trajectory_length': len(trajectory)
            })
            
            # 최근 10개 궤적만 유지
            if len(self.trajectory_history) > 10:
                self.trajectory_history = self.trajectory_history[-10:]
                
        except Exception as e:
            print(f"⚠️ Error updating trajectory history: {e}")
    
    def get_trajectory_point_at_time(self, time_offset):
        """특정 시간의 궤적 점 반환"""
        try:
            if not self.current_trajectory:
                return None
            
            # 시간에 따른 보간
            for i, point in enumerate(self.current_trajectory):
                if point['time'] >= time_offset:
                    if i == 0:
                        return point
                    
                    # 선형 보간
                    prev_point = self.current_trajectory[i-1]
                    t = (time_offset - prev_point['time']) / (point['time'] - prev_point['time'])
                    
                    interpolated_point = {
                        'time': time_offset,
                        'position': self._interpolate_vector(prev_point['position'], point['position'], t),
                        'velocity': self._interpolate_vector(prev_point['velocity'], point['velocity'], t),
                        'acceleration': self._interpolate_vector(prev_point['acceleration'], point['acceleration'], t)
                    }
                    
                    return interpolated_point
            
            # 마지막 점 반환
            return self.current_trajectory[-1]
            
        except Exception as e:
            print(f"⚠️ Error getting trajectory point at time: {e}")
            return None
    
    def _interpolate_vector(self, start_vec, end_vec, t):
        """벡터 선형 보간"""
        try:
            start = np.array(start_vec)
            end = np.array(end_vec)
            interpolated = start + t * (end - start)
            return interpolated.tolist()
            
        except Exception as e:
            print(f"⚠️ Error interpolating vector: {e}")
            return start_vec
    
    def get_trajectory_statistics(self):
        """궤적 통계 반환"""
        try:
            if not self.trajectory_history:
                return {
                    'total_trajectories': 0,
                    'avg_trajectory_length': 0.0,
                    'current_trajectory_length': 0
                }
            
            total_trajectories = len(self.trajectory_history)
            trajectory_lengths = [entry['trajectory_length'] for entry in self.trajectory_history]
            avg_trajectory_length = np.mean(trajectory_lengths) if trajectory_lengths else 0.0
            current_trajectory_length = len(self.current_trajectory)
            
            return {
                'total_trajectories': total_trajectories,
                'avg_trajectory_length': avg_trajectory_length,
                'current_trajectory_length': current_trajectory_length
            }
            
        except Exception as e:
            print(f"⚠️ Error getting trajectory statistics: {e}")
            return {
                'total_trajectories': 0,
                'avg_trajectory_length': 0.0,
                'current_trajectory_length': 0
            }
    
    def clear_trajectory(self):
        """궤적 초기화"""
        self.current_trajectory = []
        print("📈 Trajectory cleared")

