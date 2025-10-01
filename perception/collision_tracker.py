#!/usr/bin/env python3
"""
Collision Tracker Module
충돌 추적 및 관리 모듈
"""

import time
import logging
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass

@dataclass
class CollisionVehicle:
    """충돌 차량 정보"""
    actor_id: str
    last_seen: float
    location: Tuple[float, float, float]
    collision_time: float
    bbox_area: float
    distance: float

@dataclass
class CollisionEvent:
    """충돌 이벤트 정보"""
    vehicle_id: str
    pedestrian_id: str
    timestamp: float
    location: Tuple[float, float, float]
    severity: str  # 'minor', 'major', 'critical'

class CollisionTracker:
    """충돌 추적 시스템"""
    
    def __init__(self, max_tracking_time: float = 300.0):
        """
        Args:
            max_tracking_time: 최대 추적 시간 (초)
        """
        self.max_tracking_time = max_tracking_time
        self.collision_vehicles: Dict[str, CollisionVehicle] = {}
        self.collision_events: List[CollisionEvent] = []
        self.pedestrian_fall_detected = False
        self.collision_timestamp: Optional[float] = None
        
        # 로깅 설정
        self.logger = logging.getLogger(__name__)
        
    def detect_vehicle_pedestrian_collision(self, vehicles: List[Dict], pedestrians: List[Dict]) -> bool:
        """
        차량과 보행자의 충돌 감지
        
        Args:
            vehicles: 차량 객체 리스트
            pedestrians: 보행자 객체 리스트
            
        Returns:
            bool: 충돌 감지 여부
        """
        try:
            collision_detected = False
            
            for vehicle in vehicles:
                for pedestrian in pedestrians:
                    # 거리 체크 (30m 이내)
                    if vehicle['distance'] <= 30 and pedestrian['distance'] <= 30:
                        # 바운딩 박스 겹침 체크
                        if self._check_bbox_overlap(vehicle['bbox'], pedestrian['bbox']):
                            # 충돌 감지 - 콜백에서 출력
                            
                            # 충돌 차량 정보 저장
                            collision_vehicle = CollisionVehicle(
                                actor_id=vehicle['actor_id'],
                                last_seen=time.time(),
                                location=vehicle['obj'].get('world_location', [0, 0, 0]),
                                collision_time=time.time(),
                                bbox_area=vehicle['area'],
                                distance=vehicle['distance']
                            )
                            
                            self.collision_vehicles[vehicle['actor_id']] = collision_vehicle
                            
                            # 충돌 이벤트 생성
                            collision_event = CollisionEvent(
                                vehicle_id=vehicle['actor_id'],
                                pedestrian_id=pedestrian['actor_id'],
                                timestamp=time.time(),
                                location=vehicle['obj'].get('world_location', [0, 0, 0]),
                                severity=self._calculate_severity(vehicle, pedestrian)
                            )
                            
                            self.collision_events.append(collision_event)
                            collision_detected = True
            
            if collision_detected:
                self.collision_timestamp = time.time()
                
            return collision_detected
            
        except Exception as e:
            self.logger.error(f"⚠️ Error detecting vehicle-pedestrian collision: {e}")
            return False
    
    def detect_pedestrian_fall(self, pedestrians: List[Dict]) -> bool:
        """
        보행자 넘어짐 감지
        
        Args:
            pedestrians: 보행자 객체 리스트
            
        Returns:
            bool: 넘어짐 감지 여부
        """
        try:
            fall_detected = False
            
            for pedestrian in pedestrians:
                aspect_ratio = pedestrian['aspect_ratio']
                area = pedestrian['area']
                distance = pedestrian['distance']
                
                # 넘어진 보행자 조건:
                # 1. aspect_ratio가 매우 낮음 (< 0.4) - 누워있음
                # 2. 또는 aspect_ratio가 매우 높음 (> 3.0) - 세워져 있음
                # 3. 충돌 차량이 근처에 있음
                is_fallen = (aspect_ratio < 0.4 or aspect_ratio > 3.0) and area > 8000
                has_collision_vehicle = len(self.collision_vehicles) > 0
                
                if is_fallen and has_collision_vehicle:
                    self.pedestrian_fall_detected = True
                    fall_detected = True
                    
            return fall_detected
            
        except Exception as e:
            self.logger.error(f"⚠️ Error detecting pedestrian fall: {e}")
            return False
    
    def get_target_collision_vehicle(self, detected_objects: List[Dict]) -> Optional[Dict]:
        """
        충돌 차량들 중 추격할 타겟 차량 선택
        
        Args:
            detected_objects: 현재 감지된 객체들
            
        Returns:
            Optional[Dict]: 타겟 차량 객체
        """
        try:
            if not self.collision_vehicles:
                return None
            
            # 충돌 차량들 중 가장 가까운 차량을 타겟으로 선택
            target_vehicle = None
            min_distance = float('inf')
            
            for obj in detected_objects:
                if obj.get('actor_id') in self.collision_vehicles:
                    distance = obj.get('distance', 100)
                    if distance < min_distance:
                        min_distance = distance
                        target_vehicle = obj
            
            return target_vehicle
            
        except Exception as e:
            self.logger.error(f"⚠️ Error getting target collision vehicle: {e}")
            return None
    
    def update_vehicle_tracking(self, detected_objects: List[Dict]):
        """
        차량 추적 정보 업데이트
        
        Args:
            detected_objects: 현재 감지된 객체들
        """
        try:
            current_time = time.time()
            
            # 현재 감지된 차량들로 추적 정보 업데이트
            for obj in detected_objects:
                actor_id = obj.get('actor_id')
                if actor_id in self.collision_vehicles:
                    self.collision_vehicles[actor_id].last_seen = current_time
                    self.collision_vehicles[actor_id].location = obj.get('world_location', [0, 0, 0])
                    self.collision_vehicles[actor_id].distance = obj.get('distance', 100)
                    self.collision_vehicles[actor_id].bbox_area = self._get_bbox_area(obj.get('bbox_2d', {}))
            
            # 오래된 차량들 정리
            self._cleanup_old_vehicles(current_time)
            
        except Exception as e:
            self.logger.error(f"⚠️ Error updating vehicle tracking: {e}")
    
    def get_collision_statistics(self) -> Dict:
        """
        충돌 통계 정보 반환
        
        Returns:
            Dict: 충돌 통계
        """
        return {
            'collision_vehicles_count': len(self.collision_vehicles),
            'collision_events_count': len(self.collision_events),
            'pedestrian_fall_detected': self.pedestrian_fall_detected,
            'collision_timestamp': self.collision_timestamp,
            'active_vehicle_ids': list(self.collision_vehicles.keys())
        }
    
    def reset(self):
        """충돌 추적 시스템 리셋"""
        self.collision_vehicles.clear()
        self.collision_events.clear()
        self.pedestrian_fall_detected = False
        self.collision_timestamp = None
        self.logger.info("🔄 Collision tracker reset")
    
    def _check_bbox_overlap(self, bbox1: Dict, bbox2: Dict) -> bool:
        """두 바운딩 박스의 겹침 여부 체크"""
        try:
            # bbox 형식: {'x_min': x, 'y_min': y, 'x_max': x+w, 'y_max': y+h}
            x1_min, y1_min = bbox1['x_min'], bbox1['y_min']
            x1_max, y1_max = bbox1['x_max'], bbox1['y_max']
            
            x2_min, y2_min = bbox2['x_min'], bbox2['y_min']
            x2_max, y2_max = bbox2['x_max'], bbox2['y_max']
            
            # 겹침 체크 (약간의 여유를 둠)
            overlap_x = x1_max >= x2_min - 50 and x2_max >= x1_min - 50
            overlap_y = y1_max >= y2_min - 50 and y2_max >= y1_min - 50
            
            return overlap_x and overlap_y
            
        except Exception as e:
            self.logger.error(f"⚠️ Error checking bbox overlap: {e}")
            return False
    
    def _calculate_severity(self, vehicle: Dict, pedestrian: Dict) -> str:
        """충돌 심각도 계산"""
        try:
            vehicle_speed = vehicle.get('speed', 0)
            vehicle_area = vehicle.get('area', 0)
            
            if vehicle_speed > 15 or vehicle_area > 30000:
                return 'critical'
            elif vehicle_speed > 8 or vehicle_area > 20000:
                return 'major'
            else:
                return 'minor'
                
        except Exception:
            return 'minor'
    
    def _get_bbox_area(self, bbox: Dict) -> float:
        """바운딩 박스 면적 계산"""
        try:
            if isinstance(bbox, dict) and 'width' in bbox and 'height' in bbox:
                return bbox['width'] * bbox['height']
            elif isinstance(bbox, list) and len(bbox) >= 4:
                return bbox[2] * bbox[3]
            else:
                return 0
        except Exception:
            return 0
    
    def _cleanup_old_vehicles(self, current_time: float):
        """오래된 차량들 정리"""
        try:
            old_vehicles = []
            for vehicle_id, vehicle_data in self.collision_vehicles.items():
                if current_time - vehicle_data.last_seen > self.max_tracking_time:
                    old_vehicles.append(vehicle_id)
            
            for vehicle_id in old_vehicles:
                del self.collision_vehicles[vehicle_id]
                self.logger.info(f"🧹 Cleaned up old collision vehicle: {vehicle_id}")
                
        except Exception as e:
            self.logger.error(f"⚠️ Error cleaning up old vehicles: {e}")
