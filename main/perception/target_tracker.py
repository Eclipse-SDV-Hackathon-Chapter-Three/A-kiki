"""
Target Tracker for Chase Vehicle
Tracks and predicts target vehicle position
"""

import numpy as np
import time
from typing import List, Dict, Optional, Tuple

class TargetTracker:
    """타겟 추적 클래스"""
    
    def __init__(self, max_track_distance=100.0, track_timeout=2.0):
        self.max_track_distance = max_track_distance
        self.track_timeout = track_timeout
        
        self.target_vehicle = None
        self.track_history = []
        self.last_update_time = 0.0
        self.track_id = None
        
        print("🎯 Target Tracker initialized")
    
    def update_target(self, detected_vehicles, chase_vehicle_position):
        """타겟 차량 업데이트"""
        try:
            current_time = time.time()
            
            if not detected_vehicles:
                # 탐지된 차량이 없으면 타겟 유지 (타임아웃 체크)
                if self.target_vehicle and current_time - self.last_update_time > self.track_timeout:
                    self.target_vehicle = None
                    self.track_id = None
                    print("🎯 Target lost due to timeout")
                return self.target_vehicle
            
            # 가장 가까운 차량을 타겟으로 선택
            best_target = None
            best_distance = float('inf')
            
            for vehicle in detected_vehicles:
                distance = self._calculate_distance(vehicle, chase_vehicle_position)
                if distance < best_distance and distance < self.max_track_distance:
                    best_distance = distance
                    best_target = vehicle
            
            if best_target:
                # 타겟 업데이트
                if self.target_vehicle is None:
                    self.track_id = current_time  # 새로운 타겟 ID
                    print(f"🎯 New target acquired (ID: {self.track_id:.2f})")
                
                self.target_vehicle = best_target
                self.last_update_time = current_time
                
                # 추적 히스토리 업데이트
                self._update_track_history(best_target, current_time)
            
            return self.target_vehicle
            
        except Exception as e:
            print(f"⚠️ Error updating target: {e}")
            return self.target_vehicle
    
    def _calculate_distance(self, vehicle, chase_position):
        """차량과 추격차량 간 거리 계산"""
        try:
            if 'position' in vehicle:
                # 3D 위치 기반 거리
                target_pos = np.array(vehicle['position'])
                chase_pos = np.array([chase_position.x, chase_position.y, chase_position.z])
                return np.linalg.norm(target_pos - chase_pos)
            
            elif 'bbox' in vehicle:
                # 2D 바운딩 박스 기반 거리 (근사치)
                bbox = vehicle['bbox']
                # 바운딩 박스 중심을 차량 위치로 가정
                target_x, target_y = bbox[0] + bbox[2]/2, bbox[1] + bbox[3]/2
                # 화면 좌표를 월드 좌표로 변환 (근사치)
                # 실제로는 카메라 캘리브레이션 필요
                return np.sqrt(target_x**2 + target_y**2) * 0.1  # 스케일 팩터
            
            return float('inf')
            
        except Exception as e:
            print(f"⚠️ Error calculating distance: {e}")
            return float('inf')
    
    def _update_track_history(self, target, timestamp):
        """추적 히스토리 업데이트"""
        try:
            track_entry = {
                'timestamp': timestamp,
                'position': target.get('position', None),
                'bbox': target.get('bbox', None),
                'confidence': target.get('confidence', 0.0),
                'velocity': target.get('velocity', None)
            }
            
            self.track_history.append(track_entry)
            
            # 최근 5초간의 히스토리만 유지
            cutoff_time = timestamp - 5.0
            self.track_history = [
                entry for entry in self.track_history
                if entry['timestamp'] > cutoff_time
            ]
            
        except Exception as e:
            print(f"⚠️ Error updating track history: {e}")
    
    def get_target_position(self):
        """현재 타겟 위치 반환"""
        if self.target_vehicle and 'position' in self.target_vehicle:
            return self.target_vehicle['position']
        return None
    
    def get_target_velocity(self):
        """현재 타겟 속도 반환"""
        if self.target_vehicle and 'velocity' in self.target_vehicle:
            return self.target_vehicle['velocity']
        return None
    
    def get_target_distance(self, chase_position):
        """타겟과의 거리 반환"""
        if self.target_vehicle:
            return self._calculate_distance(self.target_vehicle, chase_position)
        return float('inf')
    
    def predict_target_position(self, prediction_time=1.0):
        """타겟 위치 예측"""
        try:
            if not self.target_vehicle or len(self.track_history) < 2:
                return None
            
            # 최근 2개 포인트로 속도 계산
            recent_entries = sorted(self.track_history, key=lambda x: x['timestamp'])[-2:]
            
            if len(recent_entries) < 2:
                return None
            
            pos1 = recent_entries[0].get('position')
            pos2 = recent_entries[1].get('position')
            time1 = recent_entries[0]['timestamp']
            time2 = recent_entries[1]['timestamp']
            
            if pos1 is None or pos2 is None:
                return None
            
            # 속도 계산
            velocity = (np.array(pos2) - np.array(pos1)) / (time2 - time1)
            
            # 예측 위치 계산
            predicted_pos = np.array(pos2) + velocity * prediction_time
            
            return predicted_pos.tolist()
            
        except Exception as e:
            print(f"⚠️ Error predicting target position: {e}")
            return None
    
    def get_tracking_statistics(self):
        """추적 통계 반환"""
        try:
            if not self.track_history:
                return {
                    'is_tracking': False,
                    'track_duration': 0.0,
                    'track_points': 0,
                    'avg_confidence': 0.0
                }
            
            current_time = time.time()
            track_duration = current_time - self.track_history[0]['timestamp'] if self.track_history else 0.0
            
            confidences = [entry['confidence'] for entry in self.track_history if 'confidence' in entry]
            avg_confidence = np.mean(confidences) if confidences else 0.0
            
            return {
                'is_tracking': self.target_vehicle is not None,
                'track_duration': track_duration,
                'track_points': len(self.track_history),
                'avg_confidence': avg_confidence,
                'last_update_age': current_time - self.last_update_time
            }
            
        except Exception as e:
            print(f"⚠️ Error getting tracking statistics: {e}")
            return {
                'is_tracking': False,
                'track_duration': 0.0,
                'track_points': 0,
                'avg_confidence': 0.0
            }
    
    def reset_tracking(self):
        """추적 리셋"""
        self.target_vehicle = None
        self.track_history.clear()
        self.last_update_time = 0.0
        self.track_id = None
        print("🎯 Target tracking reset")
    
    def is_target_lost(self):
        """타겟이 손실되었는지 확인"""
        if self.target_vehicle is None:
            return True
        
        current_time = time.time()
        return current_time - self.last_update_time > self.track_timeout

