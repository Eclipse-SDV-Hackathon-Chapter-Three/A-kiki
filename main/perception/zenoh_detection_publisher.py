#!/usr/bin/env python3

"""
Zenoh Detection Publisher
감지 데이터를 Zenoh를 통해 발행
"""

import time
from typing import List, Dict, Any
from .bounding_box_detector import BoundingBoxDetector
from .collision_detector import CollisionDetector
from ..communication.zenoh_manager import ZenohManager

class ZenohDetectionPublisher:
    """Zenoh 기반 감지 데이터 발행자"""
    
    def __init__(self, world, camera, zenoh_manager: ZenohManager):
        self.world = world
        self.camera = camera
        self.zenoh_manager = zenoh_manager
        
        # 감지 모듈들
        self.bounding_box_detector = BoundingBoxDetector(camera)
        self.collision_detector = CollisionDetector()
        
        # 발행 설정
        self.publish_interval = 0.1  # 100ms
        self.last_publish_time = 0
        
        print("🔍 Zenoh Detection Publisher initialized")
    
    def detect_and_publish(self, image):
        """감지 및 발행"""
        try:
            current_time = time.time()
            
            # 발행 간격 체크
            if current_time - self.last_publish_time < self.publish_interval:
                return
            
            self.last_publish_time = current_time
            
            # 바운딩 박스 감지
            detected_objects = self.bounding_box_detector.detect_pedestrians_and_vehicles(image)
            
            # 충돌 감지
            collision_events = self.collision_detector.analyze_pedestrian_collision(detected_objects)
            
            # 감지 데이터 발행
            detection_data = {
                'timestamp': current_time,
                'detected_objects': detected_objects,
                'collision_events': collision_events,
                'total_objects': len(detected_objects),
                'collision_count': len(collision_events)
            }
            
            self.zenoh_manager.publish('detection', detection_data)
            
            # 충돌 이벤트가 있으면 별도 발행
            if collision_events:
                collision_data = {
                    'timestamp': current_time,
                    'events': collision_events,
                    'count': len(collision_events)
                }
                self.zenoh_manager.publish('collision', collision_data)
                print(f"🚨 Collision detected and published: {len(collision_events)} events")
            
            return detected_objects, collision_events
            
        except Exception as e:
            print(f"⚠️ Error in detect_and_publish: {e}")
            return [], []
    
    def get_detection_summary(self):
        """감지 요약 정보 반환"""
        return {
            'bounding_box_detector': self.bounding_box_detector.get_detection_summary(),
            'collision_detector': self.collision_detector.get_collision_summary()
        }
