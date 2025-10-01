"""
Behavior Planner for Chase Vehicle
Plans high-level behaviors and decisions
"""

import numpy as np
import time
from typing import Dict, List, Optional
from enum import Enum

class ChaseBehavior(Enum):
    """추격 행동 상태"""
    SEARCH = "search"          # 타겟 탐색
    APPROACH = "approach"      # 접근
    FOLLOW = "follow"          # 추격
    OVERTAKE = "overtake"      # 추월
    MAINTAIN = "maintain"      # 유지
    EMERGENCY = "emergency"    # 비상

class BehaviorPlanner:
    """행동 계획 클래스"""
    
    def __init__(self, follow_distance=20.0, approach_distance=50.0, emergency_distance=5.0):
        self.follow_distance = follow_distance
        self.approach_distance = approach_distance
        self.emergency_distance = emergency_distance
        
        self.current_behavior = ChaseBehavior.SEARCH
        self.behavior_history = []
        self.decision_timeout = 1.0  # 1초마다 행동 결정
        self.last_decision_time = 0.0
        
        print("🧠 Behavior Planner initialized")
    
    def plan_behavior(self, chase_position, target_position, chase_velocity, target_velocity, 
                     target_distance, is_target_visible):
        """행동 계획"""
        try:
            current_time = time.time()
            
            # 의사결정 주기 체크
            if current_time - self.last_decision_time < self.decision_timeout:
                return self.current_behavior
            
            # 새로운 행동 결정
            new_behavior = self._decide_behavior(
                chase_position, target_position, chase_velocity, 
                target_velocity, target_distance, is_target_visible
            )
            
            # 행동 변경 로깅
            if new_behavior != self.current_behavior:
                print(f"🧠 Behavior changed: {self.current_behavior.value} -> {new_behavior.value}")
                self._log_behavior_change(self.current_behavior, new_behavior, current_time)
                self.current_behavior = new_behavior
            
            self.last_decision_time = current_time
            return self.current_behavior
            
        except Exception as e:
            print(f"⚠️ Error planning behavior: {e}")
            return self.current_behavior
    
    def _decide_behavior(self, chase_position, target_position, chase_velocity, 
                        target_velocity, target_distance, is_target_visible):
        """행동 결정 로직"""
        try:
            # 비상 상황 체크
            if target_distance < self.emergency_distance:
                return ChaseBehavior.EMERGENCY
            
            # 타겟이 보이지 않으면 탐색
            if not is_target_visible or target_position is None:
                return ChaseBehavior.SEARCH
            
            # 거리 기반 행동 결정
            if target_distance > self.approach_distance:
                return ChaseBehavior.APPROACH
            elif target_distance > self.follow_distance:
                return ChaseBehavior.FOLLOW
            else:
                return ChaseBehavior.MAINTAIN
            
        except Exception as e:
            print(f"⚠️ Error deciding behavior: {e}")
            return ChaseBehavior.SEARCH
    
    def _log_behavior_change(self, old_behavior, new_behavior, timestamp):
        """행동 변경 로깅"""
        try:
            self.behavior_history.append({
                'timestamp': timestamp,
                'old_behavior': old_behavior.value,
                'new_behavior': new_behavior.value,
                'duration': timestamp - self.last_decision_time if self.behavior_history else 0.0
            })
            
            # 최근 50개 행동 변경만 유지
            if len(self.behavior_history) > 50:
                self.behavior_history = self.behavior_history[-50:]
                
        except Exception as e:
            print(f"⚠️ Error logging behavior change: {e}")
    
    def get_behavior_parameters(self):
        """현재 행동에 따른 파라미터 반환"""
        try:
            if self.current_behavior == ChaseBehavior.SEARCH:
                return {
                    'target_speed': 30.0,  # 탐색 속도
                    'max_acceleration': 3.0,
                    'lookahead_distance': 15.0,
                    'safety_margin': 5.0
                }
            
            elif self.current_behavior == ChaseBehavior.APPROACH:
                return {
                    'target_speed': 50.0,  # 접근 속도
                    'max_acceleration': 5.0,
                    'lookahead_distance': 20.0,
                    'safety_margin': 8.0
                }
            
            elif self.current_behavior == ChaseBehavior.FOLLOW:
                return {
                    'target_speed': 40.0,  # 추격 속도
                    'max_acceleration': 4.0,
                    'lookahead_distance': 15.0,
                    'safety_margin': 10.0
                }
            
            elif self.current_behavior == ChaseBehavior.MAINTAIN:
                return {
                    'target_speed': 35.0,  # 유지 속도
                    'max_acceleration': 2.0,
                    'lookahead_distance': 10.0,
                    'safety_margin': 15.0
                }
            
            elif self.current_behavior == ChaseBehavior.EMERGENCY:
                return {
                    'target_speed': 20.0,  # 비상 속도
                    'max_acceleration': 8.0,
                    'lookahead_distance': 5.0,
                    'safety_margin': 2.0
                }
            
            else:  # OVERTAKE
                return {
                    'target_speed': 60.0,  # 추월 속도
                    'max_acceleration': 6.0,
                    'lookahead_distance': 25.0,
                    'safety_margin': 12.0
                }
                
        except Exception as e:
            print(f"⚠️ Error getting behavior parameters: {e}")
            return {
                'target_speed': 30.0,
                'max_acceleration': 3.0,
                'lookahead_distance': 15.0,
                'safety_margin': 5.0
            }
    
    def should_overtake(self, chase_position, target_position, chase_velocity, target_velocity):
        """추월 여부 판단"""
        try:
            if target_position is None or chase_velocity is None or target_velocity is None:
                return False
            
            # 속도 비교
            chase_speed = np.linalg.norm([chase_velocity.x, chase_velocity.y, chase_velocity.z])
            target_speed = np.linalg.norm(target_velocity)
            
            # 추격차량이 더 빠르고, 충분한 거리가 있을 때 추월 고려
            speed_difference = chase_speed - target_speed
            distance = np.linalg.norm([
                chase_position.x - target_position[0],
                chase_position.y - target_position[1],
                chase_position.z - target_position[2]
            ])
            
            return speed_difference > 5.0 and distance > 30.0
            
        except Exception as e:
            print(f"⚠️ Error checking overtake condition: {e}")
            return False
    
    def get_behavior_statistics(self):
        """행동 통계 반환"""
        try:
            if not self.behavior_history:
                return {
                    'total_changes': 0,
                    'current_behavior': self.current_behavior.value,
                    'behavior_duration': 0.0
                }
            
            total_changes = len(self.behavior_history)
            current_time = time.time()
            
            # 현재 행동 지속 시간
            behavior_duration = 0.0
            if self.behavior_history:
                last_change = self.behavior_history[-1]['timestamp']
                behavior_duration = current_time - last_change
            
            return {
                'total_changes': total_changes,
                'current_behavior': self.current_behavior.value,
                'behavior_duration': behavior_duration,
                'recent_behaviors': [entry['new_behavior'] for entry in self.behavior_history[-5:]]
            }
            
        except Exception as e:
            print(f"⚠️ Error getting behavior statistics: {e}")
            return {
                'total_changes': 0,
                'current_behavior': self.current_behavior.value,
                'behavior_duration': 0.0
            }
    
    def reset_behavior(self):
        """행동 초기화"""
        self.current_behavior = ChaseBehavior.SEARCH
        self.behavior_history.clear()
        self.last_decision_time = 0.0
        print("🧠 Behavior reset to SEARCH")
    
    def set_behavior_parameters(self, follow_distance=None, approach_distance=None, emergency_distance=None):
        """행동 파라미터 설정"""
        if follow_distance is not None:
            self.follow_distance = follow_distance
        if approach_distance is not None:
            self.approach_distance = approach_distance
        if emergency_distance is not None:
            self.emergency_distance = emergency_distance
        
        print(f"🧠 Behavior parameters updated: follow={self.follow_distance}, approach={self.approach_distance}, emergency={self.emergency_distance}")

