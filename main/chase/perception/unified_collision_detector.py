#!/usr/bin/env python3
"""
Unified Collision Detector
통합 충돌 감지 모듈 - 다양한 센서 데이터를 통합하여 충돌 감지
"""

import time
import numpy as np
from typing import List, Dict, Optional, Callable, Any
from dataclasses import dataclass
import logging

@dataclass
class CollisionEvent:
    """충돌 이벤트 데이터 클래스"""
    event_type: str
    description: str
    actor_id: str
    world_location: List[float]
    severity: str  # 'low', 'medium', 'high', 'critical'
    timestamp: float
    confidence: float = 0.0
    additional_data: Dict[str, Any] = None

@dataclass
class DetectedObject:
    """감지된 객체 데이터 클래스"""
    object_id: str
    object_type: str  # 'vehicle', 'pedestrian'
    bbox: Dict[str, float]
    distance: float
    world_location: List[float]
    velocity: float = 0.0
    confidence: float = 1.0

class UnifiedCollisionDetector:
    """통합 충돌 감지 클래스"""
    
    def __init__(self, config: Optional[Dict] = None):
        """
        Args:
            config: 충돌 감지 설정 딕셔너리
        """
        self.config = config or self._get_default_config()
        self.logger = logging.getLogger(__name__)
        
        # 콜백 함수들
        self.collision_callback: Optional[Callable[[CollisionEvent], None]] = None
        self.chase_callback: Optional[Callable[[List[str]], None]] = None
        
        # 충돌 감지 상태
        self.detected_collisions: Dict[str, CollisionEvent] = {}
        self.collision_vehicles: List[str] = []
        self.is_chasing = False
        
        # 통계
        self.collision_statistics = {
            'total_collisions': 0,
            'vehicle_pedestrian_collisions': 0,
            'pedestrian_falls': 0,
            'chase_events': 0
        }
        
        self.logger.info("🚨 Unified Collision Detector initialized")
    
    def _get_default_config(self) -> Dict:
        """기본 설정 반환"""
        return {
            'collision_distance_threshold': 10.0,  # 충돌 감지 거리 (m)
            'pedestrian_fall_height_threshold': 0.5,  # 보행자 넘어짐 높이 임계값 (m)
            'bbox_overlap_tolerance': 0.1,  # 바운딩 박스 겹침 허용 오차
            'collision_cooldown': 2.0,  # 충돌 이벤트 쿨다운 (초)
            'chase_distance_threshold': 30.0,  # 추격 시작 거리 (m)
            'enable_vehicle_pedestrian_collision': True,
            'enable_pedestrian_fall_detection': True,
            'enable_bbox_overlap_detection': True
        }
    
    def set_collision_callback(self, callback: Callable[[CollisionEvent], None]):
        """충돌 감지 콜백 설정"""
        self.collision_callback = callback
    
    def set_chase_callback(self, callback: Callable[[List[str]], None]):
        """추격 시작 콜백 설정"""
        self.chase_callback = callback
    
    def process_objects(self, objects: List[Dict]) -> List[CollisionEvent]:
        """
        감지된 객체들을 처리하여 충돌 이벤트 생성
        
        Args:
            objects: 감지된 객체 리스트 (Zenoh, CARLA 등에서)
            
        Returns:
            List[CollisionEvent]: 감지된 충돌 이벤트 리스트
        """
        try:
            self.logger.debug(f"🔍 Processing {len(objects)} objects")
            
            # 1단계: 객체 분류 및 정규화
            vehicles, pedestrians = self._classify_objects(objects)
            
            # 2단계: 충돌 감지
            collision_events = []
            
            # 차량-보행자 충돌 감지
            if self.config['enable_vehicle_pedestrian_collision']:
                vehicle_pedestrian_events = self._detect_vehicle_pedestrian_collision(vehicles, pedestrians)
                collision_events.extend(vehicle_pedestrian_events)
            
            # 보행자 넘어짐 감지
            if self.config['enable_pedestrian_fall_detection']:
                fall_events = self._detect_pedestrian_fall(pedestrians)
                collision_events.extend(fall_events)
            
            # 3단계: 충돌 이벤트 처리
            if collision_events:
                self._process_collision_events(collision_events)
            
            return collision_events
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing objects: {e}")
            return []
    
    def _classify_objects(self, objects: List[Dict]) -> tuple[List[DetectedObject], List[DetectedObject]]:
        """객체를 차량과 보행자로 분류"""
        vehicles = []
        pedestrians = []
        
        for obj in objects:
            try:
                # 객체 정보 추출
                object_id = obj.get('actor_id', obj.get('id', 'unknown'))
                object_type = obj.get('type', 'unknown')
                bbox = obj.get('bbox_2d', {})
                distance = obj.get('distance', 100.0)
                world_location = obj.get('world_location', [0, 0, 0])
                velocity = obj.get('velocity', 0.0)
                confidence = obj.get('confidence', 1.0)
                
                # DetectedObject 생성
                detected_obj = DetectedObject(
                    object_id=object_id,
                    object_type=object_type,
                    bbox=bbox,
                    distance=distance,
                    world_location=world_location,
                    velocity=velocity,
                    confidence=confidence
                )
                
                # 타입별 분류
                if object_type == 'vehicle':
                    vehicles.append(detected_obj)
                elif object_type == 'pedestrian':
                    pedestrians.append(detected_obj)
                    
            except Exception as e:
                self.logger.warning(f"⚠️ Error classifying object: {e}")
                continue
        
        return vehicles, pedestrians
    
    def _detect_vehicle_pedestrian_collision(self, vehicles: List[DetectedObject], 
                                           pedestrians: List[DetectedObject]) -> List[CollisionEvent]:
        """차량-보행자 충돌 감지"""
        collision_events = []
        
        try:
            for vehicle in vehicles:
                for pedestrian in pedestrians:
                    # 거리 체크
                    if (vehicle.distance < self.config['collision_distance_threshold'] and 
                        pedestrian.distance < self.config['collision_distance_threshold']):
                        
                        # 바운딩 박스 겹침 체크
                        if self._check_bbox_overlap(vehicle.bbox, pedestrian.bbox):
                            # 충돌 이벤트 생성
                            event = CollisionEvent(
                                event_type='vehicle_pedestrian_collision',
                                description=f"Vehicle {vehicle.object_id} colliding with pedestrian {pedestrian.object_id}",
                                actor_id=vehicle.object_id,
                                world_location=vehicle.world_location,
                                severity=self._calculate_collision_severity(vehicle, pedestrian),
                                timestamp=time.time(),
                                confidence=self._calculate_collision_confidence(vehicle, pedestrian),
                                additional_data={
                                    'vehicle_id': vehicle.object_id,
                                    'pedestrian_id': pedestrian.object_id,
                                    'vehicle_distance': vehicle.distance,
                                    'pedestrian_distance': pedestrian.distance
                                }
                            )
                            
                            collision_events.append(event)
                            self.collision_statistics['vehicle_pedestrian_collisions'] += 1
                            
                            # 충돌 차량 목록에 추가
                            if vehicle.object_id not in self.collision_vehicles:
                                self.collision_vehicles.append(vehicle.object_id)
        
        except Exception as e:
            self.logger.error(f"⚠️ Error detecting vehicle-pedestrian collision: {e}")
        
        return collision_events
    
    def _detect_pedestrian_fall(self, pedestrians: List[DetectedObject]) -> List[CollisionEvent]:
        """보행자 넘어짐 감지 (높이 기반만)"""
        fall_events = []
        
        try:
            for pedestrian in pedestrians:
                # 넘어짐 조건 체크 (높이만으로 판단)
                world_location = pedestrian.world_location
                
                # 높이가 낮으면 넘어짐으로 판단 (속도 조건 제거)
                if (len(world_location) >= 3 and 
                    world_location[2] < self.config['pedestrian_fall_height_threshold']):
                    
                    event = CollisionEvent(
                        event_type='pedestrian_fall',
                        description=f"Pedestrian {pedestrian.object_id} may have fallen",
                        actor_id=pedestrian.object_id,
                        world_location=world_location,
                        severity='medium',
                        timestamp=time.time(),
                        confidence=0.8,
                        additional_data={
                            'pedestrian_id': pedestrian.object_id,
                            'height': world_location[2] if len(world_location) >= 3 else 0
                        }
                    )
                    
                    fall_events.append(event)
                    self.collision_statistics['pedestrian_falls'] += 1
        
        except Exception as e:
            self.logger.error(f"⚠️ Error detecting pedestrian fall: {e}")
        
        return fall_events
    
    def _check_bbox_overlap(self, bbox1: Dict[str, float], bbox2: Dict[str, float]) -> bool:
        """두 바운딩 박스의 겹침 여부 체크"""
        try:
            if not bbox1 or not bbox2:
                return False
            
            # 바운딩 박스 좌표 추출
            x1_min = bbox1.get('x_min', 0)
            y1_min = bbox1.get('y_min', 0)
            x1_max = bbox1.get('x_max', 0)
            y1_max = bbox1.get('y_max', 0)
            
            x2_min = bbox2.get('x_min', 0)
            y2_min = bbox2.get('y_min', 0)
            x2_max = bbox2.get('x_max', 0)
            y2_max = bbox2.get('y_max', 0)
            
            # 겹침 체크 (약간의 여유를 둠)
            tolerance = self.config['bbox_overlap_tolerance']
            overlap_x = x1_max >= x2_min - tolerance and x2_max >= x1_min - tolerance
            overlap_y = y1_max >= y2_min - tolerance and y2_max >= y1_min - tolerance
            
            return overlap_x and overlap_y
            
        except Exception as e:
            self.logger.warning(f"⚠️ Error checking bbox overlap: {e}")
            return False
    
    def _calculate_collision_severity(self, vehicle: DetectedObject, pedestrian: DetectedObject) -> str:
        """충돌 심각도 계산"""
        try:
            # 차량 속도와 크기 기반 심각도 계산
            vehicle_speed = vehicle.velocity
            vehicle_area = self._calculate_bbox_area(vehicle.bbox)
            
            if vehicle_speed > 15 or vehicle_area > 30000:
                return 'critical'
            elif vehicle_speed > 8 or vehicle_area > 20000:
                return 'high'
            elif vehicle_speed > 3 or vehicle_area > 10000:
                return 'medium'
            else:
                return 'low'
                
        except Exception:
            return 'medium'
    
    def _calculate_collision_confidence(self, vehicle: DetectedObject, pedestrian: DetectedObject) -> float:
        """충돌 신뢰도 계산"""
        try:
            # 거리와 바운딩 박스 크기 기반 신뢰도 계산
            distance_factor = max(0, 1.0 - (vehicle.distance / self.config['collision_distance_threshold']))
            bbox_factor = min(1.0, self._calculate_bbox_area(vehicle.bbox) / 50000)
            
            confidence = (distance_factor + bbox_factor) / 2.0
            return min(1.0, max(0.0, confidence))
            
        except Exception:
            return 0.5
    
    def _calculate_bbox_area(self, bbox: Dict[str, float]) -> float:
        """바운딩 박스 면적 계산"""
        try:
            if 'width' in bbox and 'height' in bbox:
                return bbox['width'] * bbox['height']
            elif 'x_min' in bbox and 'x_max' in bbox and 'y_min' in bbox and 'y_max' in bbox:
                width = bbox['x_max'] - bbox['x_min']
                height = bbox['y_max'] - bbox['y_min']
                return width * height
            else:
                return 0
        except Exception:
            return 0
    
    def _process_collision_events(self, collision_events: List[CollisionEvent]):
        """충돌 이벤트 처리"""
        try:
            for event in collision_events:
                # 중복 이벤트 방지
                event_key = f"{event.event_type}_{event.actor_id}"
                current_time = time.time()
                
                if event_key in self.detected_collisions:
                    last_event = self.detected_collisions[event_key]
                    if current_time - last_event.timestamp < self.config['collision_cooldown']:
                        continue
                
                # 이벤트 저장
                self.detected_collisions[event_key] = event
                self.collision_statistics['total_collisions'] += 1
                
                # 콜백 호출
                if self.collision_callback:
                    self.collision_callback(event)
                
                self.logger.warning(f"🚨 Collision detected: {event.description}")
            
            # 추격 시작 조건 체크
            if self._should_start_chase(collision_events):
                self._start_chase()
                
        except Exception as e:
            self.logger.error(f"⚠️ Error processing collision events: {e}")
    
    def _should_start_chase(self, collision_events: List[CollisionEvent]) -> bool:
        """추격 시작 여부 판단"""
        try:
            # 충돌 차량이 있고 아직 추격 중이 아닌 경우
            if self.collision_vehicles and not self.is_chasing:
                return True
            
            # 심각한 충돌 이벤트가 있는 경우
            for event in collision_events:
                if event.severity in ['high', 'critical']:
                    return True
            
            return False
            
        except Exception:
            return False
    
    def _start_chase(self):
        """추격 시작"""
        try:
            if not self.is_chasing and self.collision_vehicles:
                self.is_chasing = True
                self.collision_statistics['chase_events'] += 1
                
                self.logger.info(f"🚔 Starting chase for vehicles: {self.collision_vehicles}")
                
                # 추격 콜백 호출
                if self.chase_callback:
                    self.chase_callback(self.collision_vehicles.copy())
                
        except Exception as e:
            self.logger.error(f"⚠️ Error starting chase: {e}")
    
    def stop_chase(self):
        """추격 중지"""
        self.is_chasing = False
        self.collision_vehicles.clear()
        self.logger.info("🛑 Chase stopped")
    
    def get_statistics(self) -> Dict[str, Any]:
        """충돌 감지 통계 반환"""
        return {
            **self.collision_statistics,
            'is_chasing': self.is_chasing,
            'collision_vehicles': self.collision_vehicles.copy(),
            'active_collisions': len(self.detected_collisions)
        }
    
    def reset(self):
        """충돌 감지 상태 리셋"""
        self.detected_collisions.clear()
        self.collision_vehicles.clear()
        self.is_chasing = False
        self.collision_statistics = {
            'total_collisions': 0,
            'vehicle_pedestrian_collisions': 0,
            'pedestrian_falls': 0,
            'chase_events': 0
        }
        self.logger.info("🔄 Collision detector reset")
    
    def cleanup(self):
        """리소스 정리"""
        self.reset()
        self.collision_callback = None
        self.chase_callback = None
        self.logger.info("🧹 Collision detector cleaned up")
