"""
Collision Vehicle Tracker
충돌을 일으킨 차량을 지속적으로 트래킹하는 모듈
"""

import carla
import numpy as np
import time
import math
from typing import Dict, List, Optional, Tuple
from collections import deque

class CollisionVehicleTracker:
    """충돌 차량 추적 클래스"""
    
    def __init__(self, world, max_track_distance=200.0, track_timeout=None):
        self.world = world
        # self.max_track_distance = max_track_distance
        self.max_track_distance = 2000
        self.track_timeout = track_timeout  # None으로 설정하여 타임아웃 비활성화
        
        # 추적 상태
        self.target_vehicle = None
        self.target_actor_id = None
        self.track_history = deque(maxlen=100)  # 최근 100개 위치 기록
        self.last_update_time = 0.0
        self.is_tracking = False
        
        # 충돌 감지 상태
        self.collision_detected = False
        self.collision_location = None
        self.collision_time = 0.0
        
        # 예측 파라미터
        self.prediction_horizon = 2.0  # 2초 미래 예측
        self.velocity_smoothing_factor = 0.3  # 속도 스무딩 팩터
        
        print("🚨 Collision Vehicle Tracker initialized")
    
    def start_tracking(self, target_vehicle):
        """직접 추격 시작 (충돌 없이)"""
        try:
            if isinstance(target_vehicle, dict):
                self.target_actor_id = target_vehicle.get('actor_id')
                self.target_vehicle = target_vehicle
                print(f"🎯 Starting tracking with dict vehicle: ID={self.target_actor_id}")
            else:
                self.target_actor_id = target_vehicle.id
                self.target_vehicle = target_vehicle
                print(f"🎯 Starting tracking with CARLA vehicle: ID={self.target_actor_id}")
            
            self.is_tracking = True
            self.last_update_time = time.time()
            
            # 추적 히스토리 초기화
            self.track_history.clear()
            
            # 첫 번째 위치 기록
            if isinstance(target_vehicle, dict):
                self._update_track_history(target_vehicle)
            
            print(f"🎯 Direct tracking started for vehicle: {self.target_actor_id}")
            return True
            
        except Exception as e:
            print(f"⚠️ Error starting tracking: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def detect_collision_vehicle(self, detected_objects, collision_events):
        """충돌 차량 감지 및 추적 시작 - 한 번 설정된 차량 ID는 고정"""
        try:
            # 이미 추적 중이면 새로운 차량을 찾지 않음
            if self.is_tracking:
                print(f"🎯 Already tracking vehicle ID: {self.target_actor_id} - continuing existing tracking")
                return True
            
            # 충돌 이벤트가 있는지 확인
            if not collision_events:
                print("🔍 No collision events detected - waiting for accident")
                return False
            
            print(f"🚨 COLLISION EVENTS DETECTED: {len(collision_events)} events")
            for i, event in enumerate(collision_events):
                print(f"   Event {i+1}: Actor {event.get('actor_id')}, Score {event.get('collision_score', 0):.2f}")
            
            # 넘어진 사람이 감지된 경우
            fallen_pedestrians = [event for event in collision_events if event.get('avoid_pedestrian', False)]
            if fallen_pedestrians:
                print(f"🚨 FALLEN PEDESTRIAN DETECTED - AVOIDING: {len(fallen_pedestrians)} pedestrians")
            else:
                print("🚨 COLLISION DETECTED - No fallen pedestrians, but collision events exist")
            
            # 충돌이 감지된 경우
            if not self.collision_detected:
                self.collision_detected = True
                self.collision_time = time.time()
                print("🚨 COLLISION DETECTED - Starting vehicle tracking")
            
            # 충돌 지점에서 가장 가까운 차량 찾기
            print(f"🔍 Looking for vehicles in {len(detected_objects)} detected objects")
            collision_vehicle = self._find_nearest_vehicle_to_collision(detected_objects, collision_events)
            
            if collision_vehicle:
                print(f"🎯 Found collision vehicle: ID={collision_vehicle.get('actor_id')}")
                self._start_tracking_vehicle(collision_vehicle)
                return True
            else:
                print("⚠️ No vehicle found for tracking")
                # 테스트용: 첫 번째 차량을 타겟으로 사용 (강제 추적)
                vehicles = [obj for obj in detected_objects if obj.get('type') == 'vehicle']
                if vehicles:
                    print(f"🧪 TEST: Using first available vehicle as target (forced tracking)")
                    self._start_tracking_vehicle(vehicles[0])
                    return True
                return False
                
        except Exception as e:
            print(f"⚠️ Error detecting collision vehicle: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _find_nearest_vehicle_to_collision(self, detected_objects, collision_events):
        """충돌 지점에서 가장 가까운 차량 찾기"""
        try:
            if not collision_events:
                print("🔍 No collision events for vehicle finding")
                return None
            
            print(f"🔍 Finding nearest vehicle to collision from {len(detected_objects)} detected objects")
            
            # 충돌 지점 계산 (넘어진 사람들의 평균 위치 또는 첫 번째 충돌 이벤트 위치)
            collision_positions = []
            for event in collision_events:
                if 'world_location' in event:
                    collision_positions.append(event['world_location'])
                    print(f"   Collision position: {event['world_location']}")
            
            if not collision_positions:
                print("⚠️ No collision positions found")
                return None
            
            # 평균 충돌 지점 계산
            avg_collision_pos = np.mean(collision_positions, axis=0)
            self.collision_location = avg_collision_pos
            print(f"🎯 Average collision position: {avg_collision_pos}")
            
            # 카메라 기준으로 사고 낸 차량 찾기
            vehicles = [obj for obj in detected_objects if obj.get('type') == 'vehicle']
            print(f"🔍 Found {len(vehicles)} vehicles in detected objects")
            
            if not vehicles:
                print("🔍 No vehicles detected for collision tracking")
                return None
            
            # 카메라 이미지에서 사고 지점 근처의 차량 찾기
            collision_vehicle = self._find_vehicle_near_collision_in_camera(vehicles, collision_events)
            
            if collision_vehicle:
                print(f"🎯 Found collision vehicle in camera: ID={collision_vehicle.get('actor_id')}")
            else:
                print("🔍 No vehicle found within collision range")
            
            return collision_vehicle
            
        except Exception as e:
            print(f"⚠️ Error finding nearest vehicle: {e}")
            return None
    
    def _find_vehicle_near_collision_in_camera(self, vehicles, collision_events):
        """카메라 이미지에서 사고 지점 근처의 차량 찾기"""
        try:
            # 사고 지점의 2D 바운딩 박스 찾기
            collision_bboxes = []
            for event in collision_events:
                if 'bbox_2d' in event:
                    collision_bboxes.append(event['bbox_2d'])
                    print(f"📷 Collision bbox: {event['bbox_2d']}")
            
            if not collision_bboxes:
                print("⚠️ No collision bboxes found - using first vehicle")
                return vehicles[0] if vehicles else None
            
            # 평균 충돌 바운딩 박스 계산
            avg_bbox = {
                'x_min': int(np.mean([bbox['x_min'] for bbox in collision_bboxes])),
                'y_min': int(np.mean([bbox['y_min'] for bbox in collision_bboxes])),
                'x_max': int(np.mean([bbox['x_max'] for bbox in collision_bboxes])),
                'y_max': int(np.mean([bbox['y_max'] for bbox in collision_bboxes]))
            }
            print(f"📷 Average collision bbox: {avg_bbox}")
            
            # 충돌 바운딩 박스와 겹치는 차량 찾기
            collision_vehicle = None
            min_distance = float('inf')
            
            for vehicle in vehicles:
                if 'bbox_2d' in vehicle:
                    vehicle_bbox = vehicle['bbox_2d']
                    # 바운딩 박스 중심점 간 거리 계산
                    collision_center_x = (avg_bbox['x_min'] + avg_bbox['x_max']) // 2
                    collision_center_y = (avg_bbox['y_min'] + avg_bbox['y_max']) // 2
                    vehicle_center_x = (vehicle_bbox['x_min'] + vehicle_bbox['x_max']) // 2
                    vehicle_center_y = (vehicle_bbox['y_min'] + vehicle_bbox['y_max']) // 2
                    
                    distance = np.sqrt((collision_center_x - vehicle_center_x)**2 + (collision_center_y - vehicle_center_y)**2)
                    print(f"📷 Vehicle {vehicle.get('actor_id')} distance from collision: {distance:.1f}px")
                    
                    # 충돌 바운딩 박스와 겹치는지 확인
                    overlap = self._calculate_bbox_overlap(avg_bbox, vehicle_bbox)
                    print(f"📷 Vehicle {vehicle.get('actor_id')} overlap with collision: {overlap:.2f}")
                    
                    if overlap > 0.1 or distance < 100:  # 10% 이상 겹치거나 100px 이내
                        if distance < min_distance:
                            min_distance = distance
                            collision_vehicle = vehicle
                            print(f"✅ New collision vehicle: ID={vehicle.get('actor_id')}, Distance={distance:.1f}px, Overlap={overlap:.2f}")
                else:
                    print(f"⚠️ Vehicle {vehicle.get('actor_id')} has no bbox_2d")
            
            return collision_vehicle
            
        except Exception as e:
            print(f"⚠️ Error finding vehicle near collision in camera: {e}")
            return vehicles[0] if vehicles else None
    
    def _calculate_bbox_overlap(self, bbox1, bbox2):
        """두 바운딩 박스의 겹침 비율 계산"""
        try:
            # 겹치는 영역 계산
            x_overlap = max(0, min(bbox1['x_max'], bbox2['x_max']) - max(bbox1['x_min'], bbox2['x_min']))
            y_overlap = max(0, min(bbox1['y_max'], bbox2['y_max']) - max(bbox1['y_min'], bbox2['y_min']))
            
            overlap_area = x_overlap * y_overlap
            
            # 각 바운딩 박스의 면적
            bbox1_area = (bbox1['x_max'] - bbox1['x_min']) * (bbox1['y_max'] - bbox1['y_min'])
            bbox2_area = (bbox2['x_max'] - bbox2['x_min']) * (bbox2['y_max'] - bbox2['y_min'])
            
            # 겹침 비율 (작은 바운딩 박스 기준)
            min_area = min(bbox1_area, bbox2_area)
            if min_area > 0:
                return overlap_area / min_area
            else:
                return 0.0
                
        except Exception as e:
            print(f"⚠️ Error calculating bbox overlap: {e}")
            return 0.0
    
    def _start_tracking_vehicle(self, vehicle_data):
        """차량 추적 시작"""
        try:
            # CARLA 객체 가져오기
            actor_id = vehicle_data.get('actor_id')
            if actor_id:
                # CARLA 월드에서 실제 객체 가져오기
                for actor in self.world.get_actors():
                    if actor.id == actor_id:
                        self.target_vehicle = actor
                        self.target_actor_id = actor_id
                        self.is_tracking = True
                        self.last_update_time = time.time()
                        
                        # 추적 히스토리 초기화
                        self.track_history.clear()
                        
                        # 첫 번째 위치 기록
                        self._update_track_history(vehicle_data)
                        
                        print(f"🎯 Started tracking vehicle (ID: {self.target_actor_id})")
                        return
                
                print(f"⚠️ Vehicle with ID {actor_id} not found in world")
            else:
                print("⚠️ No actor ID in vehicle data")
            
        except Exception as e:
            print(f"⚠️ Error starting vehicle tracking: {e}")
    
    def update_tracking(self, detected_objects):
        """추적 업데이트 - 고정된 차량 ID 유지"""
        try:
            current_time = time.time()
            
            # 타임아웃 체크 (비활성화됨)
            if self.track_timeout and self.is_tracking and current_time - self.last_update_time > self.track_timeout:
                print("🎯 Target vehicle lost due to timeout")
                self._stop_tracking()
                return False
            
            if not self.is_tracking:
                print("🔍 Not currently tracking any vehicle")
                return False
            
            print(f"🎯 Updating tracking for vehicle ID: {self.target_actor_id}")
            
            # 현재 추적 중인 차량 찾기 (고정된 ID로만)
            current_vehicle = self._find_current_target_vehicle(detected_objects)
            
            if current_vehicle:
                # 추적 업데이트
                self.target_vehicle = current_vehicle
                self.last_update_time = current_time
                self._update_track_history(current_vehicle)
                
                # 거리 체크
                distance = self._get_current_distance()
                if distance > self.max_track_distance:
                    print(f"🎯 Target vehicle too far: {distance:.1f}m")
                    self._stop_tracking()
                    return False
                
                print(f"✅ Successfully tracking vehicle ID: {self.target_actor_id}, Distance: {distance:.1f}m")
                return True
            else:
                # 차량을 찾을 수 없음 - 예측 모드로 전환
                print(f"🎯 Target vehicle ID {self.target_actor_id} not visible - using prediction")
                return self._predict_target_position()
            
        except Exception as e:
            print(f"⚠️ Error updating tracking: {e}")
            return False
    
    def _find_current_target_vehicle(self, detected_objects):
        """현재 추적 중인 차량 찾기 - 고정된 ID로만 검색"""
        try:
            if not self.target_actor_id:
                print("🔍 No target actor ID set")
                return None
            
            print(f"🔍 Looking for target vehicle ID: {self.target_actor_id} in {len(detected_objects)} objects")
            
            for obj in detected_objects:
                if (obj.get('type') == 'vehicle' and 
                    obj.get('actor_id') == self.target_actor_id):
                    print(f"✅ Found target vehicle: ID={self.target_actor_id}")
                    return obj
            
            print(f"⚠️ Target vehicle ID {self.target_actor_id} not found in current detections")
            return None
            
        except Exception as e:
            print(f"⚠️ Error finding current target vehicle: {e}")
            return None
    
    def _update_track_history(self, vehicle_data):
        """추적 히스토리 업데이트"""
        try:
            track_entry = {
                'timestamp': time.time(),
                'position': vehicle_data.get('world_location', (0, 0, 0)),
                'velocity': self._calculate_velocity(vehicle_data),
                'distance': vehicle_data.get('distance', 0),
                'confidence': vehicle_data.get('confidence', 0)
            }
            
            self.track_history.append(track_entry)
            
        except Exception as e:
            print(f"⚠️ Error updating track history: {e}")
    
    def _calculate_velocity(self, vehicle_data):
        """차량 속도 계산"""
        try:
            if len(self.track_history) < 2:
                return (0, 0, 0)
            
            # 최근 2개 위치로 속도 계산
            recent = self.track_history[-1]
            previous = self.track_history[-2]
            
            time_diff = recent['timestamp'] - previous['timestamp']
            if time_diff <= 0:
                return (0, 0, 0)
            
            pos1 = np.array(previous['position'])
            pos2 = np.array(recent['position'])
            
            velocity = (pos2 - pos1) / time_diff
            
            # 속도 스무딩 적용
            if len(self.track_history) > 2:
                old_velocity = self.track_history[-2].get('velocity', (0, 0, 0))
                old_velocity = np.array(old_velocity)
                velocity = (1 - self.velocity_smoothing_factor) * velocity + self.velocity_smoothing_factor * old_velocity
            
            return velocity.tolist()
            
        except Exception as e:
            print(f"⚠️ Error calculating velocity: {e}")
            return (0, 0, 0)
    
    def _predict_target_position(self):
        """타겟 위치 예측"""
        try:
            if len(self.track_history) < 2:
                return False
            
            # 최근 속도 계산
            recent_velocity = self.track_history[-1].get('velocity', (0, 0, 0))
            recent_position = self.track_history[-1].get('position', (0, 0, 0))
            
            # 예측 위치 계산
            velocity = np.array(recent_velocity)
            position = np.array(recent_position)
            
            predicted_position = position + velocity * self.prediction_horizon
            
            # 예측된 위치를 현재 위치로 업데이트
            predicted_vehicle = {
                'world_location': predicted_position.tolist(),
                'actor_id': self.target_actor_id,
                'type': 'vehicle',
                'distance': self._calculate_distance_3d(
                    recent_position, predicted_position.tolist()
                ),
                'confidence': 0.7  # 예측이므로 낮은 신뢰도
            }
            
            self.target_vehicle = predicted_vehicle
            self._update_track_history(predicted_vehicle)
            
            return True
            
        except Exception as e:
            print(f"⚠️ Error predicting target position: {e}")
            return False
    
    def get_target_position(self):
        """현재 타겟 위치 반환"""
        if self.target_vehicle:
            return self.target_vehicle.get('world_location')
        return None
    
    def get_target_velocity(self):
        """현재 타겟 속도 반환"""
        if self.target_vehicle:
            return self.target_vehicle.get('velocity', (0, 0, 0))
        return None
    
    def get_target_distance(self, chase_position):
        """타겟과의 거리 반환"""
        if self.target_vehicle:
            target_pos = self.target_vehicle.get('world_location', (0, 0, 0))
            return self._calculate_distance_3d(
                (chase_position.x, chase_position.y, chase_position.z),
                target_pos
            )
        return float('inf')
    
    def predict_future_position(self, prediction_time=1.0):
        """미래 위치 예측"""
        try:
            if not self.target_vehicle or len(self.track_history) < 2:
                return None
            
            current_position = np.array(self.target_vehicle.get('world_location', (0, 0, 0)))
            current_velocity = np.array(self.target_vehicle.get('velocity', (0, 0, 0)))
            
            # 미래 위치 계산
            future_position = current_position + current_velocity * prediction_time
            
            return future_position.tolist()
            
        except Exception as e:
            print(f"⚠️ Error predicting future position: {e}")
            return None
    
    def _calculate_distance_3d(self, pos1, pos2):
        """3D 거리 계산"""
        try:
            p1 = np.array(pos1)
            p2 = np.array(pos2)
            return np.linalg.norm(p1 - p2)
        except Exception as e:
            return float('inf')
    
    def _get_current_distance(self):
        """현재 추적 중인 차량과의 거리"""
        if self.target_vehicle:
            return self.target_vehicle.get('distance', float('inf'))
        return float('inf')
    
    def _stop_tracking(self):
        """추적 중지"""
        self.target_vehicle = None
        self.target_actor_id = None
        self.is_tracking = False
        self.track_history.clear()
        print("🎯 Tracking stopped")
    
    def get_target_vehicle(self):
        """타겟 차량 객체 반환"""
        if self.target_vehicle and isinstance(self.target_vehicle, dict):
            # 딕셔너리 형태의 차량 데이터를 그대로 반환
            return self.target_vehicle
        elif self.target_vehicle:
            # CARLA 객체인 경우 딕셔너리로 변환
            return {
                'actor_id': self.target_actor_id,
                'world_location': (
                    self.target_vehicle.get_location().x,
                    self.target_vehicle.get_location().y,
                    self.target_vehicle.get_location().z
                ),
                'type': 'vehicle',
                'distance': 0.0,
                'confidence': 1.0
            }
        return None
    
    def get_tracking_status(self):
        """추적 상태 반환"""
        return {
            'is_tracking': self.is_tracking,
            'target_actor_id': self.target_actor_id,
            'collision_detected': self.collision_detected,
            'collision_location': self.collision_location,
            'track_duration': time.time() - self.last_update_time if self.is_tracking else 0,
            'track_points': len(self.track_history),
            'current_distance': self._get_current_distance()
        }
    
    def get_tracking_statistics(self):
        """추적 통계 반환"""
        try:
            if not self.track_history:
                return {
                    'total_track_points': 0,
                    'avg_velocity': 0.0,
                    'max_velocity': 0.0,
                    'track_duration': 0.0
                }
            
            velocities = [entry.get('velocity', (0, 0, 0)) for entry in self.track_history]
            speeds = [np.linalg.norm(v) for v in velocities]
            
            track_duration = 0.0
            if len(self.track_history) > 1:
                track_duration = self.track_history[-1]['timestamp'] - self.track_history[0]['timestamp']
            
            return {
                'total_track_points': len(self.track_history),
                'avg_velocity': np.mean(speeds),
                'max_velocity': np.max(speeds),
                'track_duration': track_duration,
                'current_velocity': speeds[-1] if speeds else 0.0
            }
            
        except Exception as e:
            print(f"⚠️ Error getting tracking statistics: {e}")
            return {
                'total_track_points': 0,
                'avg_velocity': 0.0,
                'max_velocity': 0.0,
                'track_duration': 0.0
            }
    
    def reset_tracking(self):
        """추적 리셋"""
        self._stop_tracking()
        self.collision_detected = False
        self.collision_location = None
        self.collision_time = 0.0
        print("🎯 Tracking reset")
