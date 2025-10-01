"""
Path Planner for Chase Vehicle
Plans optimal path to target vehicle
"""

import numpy as np
import math
from typing import List, Tuple, Optional

class PathPlanner:
    """경로 계획 클래스"""
    
    def __init__(self, max_speed=50.0, max_acceleration=5.0, safety_distance=10.0):
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.safety_distance = safety_distance
        
        self.current_path = []
        self.path_history = []
        
        print("🗺️ Path Planner initialized")
    
    def plan_path_to_target(self, chase_position, target_position, chase_velocity=None, target_velocity=None):
        """타겟까지의 경로 계획"""
        try:
            if target_position is None:
                return []
            
            # 현재 위치와 타겟 위치
            start_pos = np.array([chase_position.x, chase_position.y, chase_position.z])
            
            # target_position이 Location 객체인지 리스트인지 확인
            if hasattr(target_position, 'x'):  # Location 객체
                target_pos = np.array([target_position.x, target_position.y, target_position.z])
            else:  # 리스트 또는 튜플
                target_pos = np.array(target_position[:3]) if len(target_position) >= 3 else np.array(target_position + [0])
            
            # 기본 경로 (직선)
            path = self._generate_straight_line_path(start_pos, target_pos)
            
            # 속도 고려한 경로 최적화
            if chase_velocity is not None and target_velocity is not None:
                path = self._optimize_path_with_velocity(path, chase_velocity, target_velocity)
            
            # 안전성 검사
            path = self._add_safety_margins(path)
            
            self.current_path = path
            self._update_path_history(path)
            
            return path
            
        except Exception as e:
            print(f"⚠️ Error planning path to target: {e}")
            return []
    
    def _generate_straight_line_path(self, start_pos, target_pos, num_points=10):
        """직선 경로 생성"""
        try:
            path = []
            
            # 시작점과 끝점 사이를 균등하게 나누기
            for i in range(num_points + 1):
                t = i / num_points
                point = start_pos + t * (target_pos - start_pos)
                path.append(point.tolist())
            
            return path
            
        except Exception as e:
            print(f"⚠️ Error generating straight line path: {e}")
            return []
    
    def _optimize_path_with_velocity(self, path, chase_velocity, target_velocity):
        """속도를 고려한 경로 최적화"""
        try:
            if len(path) < 2:
                return path
            
            optimized_path = [path[0]]  # 시작점
            
            for i in range(1, len(path)):
                current_point = np.array(path[i])
                prev_point = np.array(optimized_path[-1])
                
                # 거리 계산
                distance = np.linalg.norm(current_point - prev_point)
                
                # 속도에 따른 점 간격 조정
                if distance > self.max_speed * 0.1:  # 0.1초 간격
                    # 중간점 추가
                    num_intermediate = int(distance / (self.max_speed * 0.1))
                    for j in range(1, num_intermediate + 1):
                        t = j / (num_intermediate + 1)
                        intermediate_point = prev_point + t * (current_point - prev_point)
                        optimized_path.append(intermediate_point.tolist())
                
                optimized_path.append(path[i])
            
            return optimized_path
            
        except Exception as e:
            print(f"⚠️ Error optimizing path with velocity: {e}")
            return path
    
    def _add_safety_margins(self, path):
        """안전 마진 추가"""
        try:
            if len(path) < 2:
                return path
            
            safe_path = []
            
            for i, point in enumerate(path):
                safe_point = point.copy()
                
                # 경로의 방향 계산
                if i > 0:
                    direction = np.array(point) - np.array(path[i-1])
                    direction_norm = np.linalg.norm(direction)
                    
                    if direction_norm > 0:
                        # 수직 방향으로 안전 마진 추가
                        perpendicular = np.array([-direction[1], direction[0], 0])
                        perpendicular = perpendicular / np.linalg.norm(perpendicular)
                        
                        # 안전 마진 적용 (간단한 예시)
                        safety_offset = perpendicular * (self.safety_distance * 0.1)
                        safe_point = (np.array(safe_point) + safety_offset).tolist()
                
                safe_path.append(safe_point)
            
            return safe_path
            
        except Exception as e:
            print(f"⚠️ Error adding safety margins: {e}")
            return path
    
    def _update_path_history(self, path):
        """경로 히스토리 업데이트"""
        try:
            self.path_history.append({
                'timestamp': time.time(),
                'path': path.copy(),
                'path_length': len(path)
            })
            
            # 최근 10개 경로만 유지
            if len(self.path_history) > 10:
                self.path_history = self.path_history[-10:]
                
        except Exception as e:
            print(f"⚠️ Error updating path history: {e}")
    
    def get_next_waypoint(self, current_position, lookahead_distance=5.0):
        """다음 웨이포인트 반환"""
        try:
            if not self.current_path:
                return None
            
            current_pos = np.array([current_position.x, current_position.y, current_position.z])
            
            # 가장 가까운 경로상의 점 찾기
            min_distance = float('inf')
            closest_index = 0
            
            for i, waypoint in enumerate(self.current_path):
                waypoint_pos = np.array(waypoint)
                distance = np.linalg.norm(current_pos - waypoint_pos)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_index = i
            
            # 룩어헤드 거리만큼 앞의 웨이포인트 찾기
            target_index = closest_index
            accumulated_distance = 0.0
            
            for i in range(closest_index + 1, len(self.current_path)):
                waypoint_pos = np.array(self.current_path[i])
                distance = np.linalg.norm(waypoint_pos - current_pos)
                accumulated_distance += distance
                
                if accumulated_distance >= lookahead_distance:
                    target_index = i
                    break
            
            if target_index < len(self.current_path):
                return self.current_path[target_index]
            
            return None
            
        except Exception as e:
            print(f"⚠️ Error getting next waypoint: {e}")
            return None
    
    def calculate_path_length(self, path):
        """경로 길이 계산"""
        try:
            if len(path) < 2:
                return 0.0
            
            total_length = 0.0
            for i in range(1, len(path)):
                point1 = np.array(path[i-1])
                point2 = np.array(path[i])
                total_length += np.linalg.norm(point2 - point1)
            
            return total_length
            
        except Exception as e:
            print(f"⚠️ Error calculating path length: {e}")
            return 0.0
    
    def is_path_valid(self, path):
        """경로 유효성 검사"""
        try:
            if not path or len(path) < 2:
                return False
            
            # 기본 검사: 점들이 너무 멀리 떨어져 있지 않은지
            for i in range(1, len(path)):
                point1 = np.array(path[i-1])
                point2 = np.array(path[i])
                distance = np.linalg.norm(point2 - point1)
                
                if distance > self.max_speed * 2.0:  # 2초 내에 갈 수 없는 거리
                    return False
            
            return True
            
        except Exception as e:
            print(f"⚠️ Error validating path: {e}")
            return False
    
    def get_path_statistics(self):
        """경로 통계 반환"""
        try:
            if not self.path_history:
                return {
                    'total_paths': 0,
                    'avg_path_length': 0.0,
                    'current_path_length': 0
                }
            
            total_paths = len(self.path_history)
            path_lengths = [entry['path_length'] for entry in self.path_history]
            avg_path_length = np.mean(path_lengths) if path_lengths else 0.0
            current_path_length = len(self.current_path)
            
            return {
                'total_paths': total_paths,
                'avg_path_length': avg_path_length,
                'current_path_length': current_path_length
            }
            
        except Exception as e:
            print(f"⚠️ Error getting path statistics: {e}")
            return {
                'total_paths': 0,
                'avg_path_length': 0.0,
                'current_path_length': 0
            }
    
    def clear_path(self):
        """경로 초기화"""
        self.current_path = []
        print("🗺️ Path cleared")

