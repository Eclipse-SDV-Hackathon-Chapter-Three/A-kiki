#!/usr/bin/env python3
"""
Zenoh 기반 바운딩 박스 감지기
Zenoh로부터 카메라 이미지를 받아서 바운딩 박스를 감지하고 Zenoh로 전송
"""

import sys
import os
import time
import logging
from typing import Optional, Dict, Any, List, Callable
import numpy as np
import cv2
import json

# Zenoh import
try:
    import zenoh
    from zenoh import Config
except ImportError:
    print("⚠️ Zenoh not available. Please install: pip install zenoh")
    zenoh = None
    Config = None

# CARLA import
try:
    import carla
except ImportError:
    print("⚠️ CARLA not available. Please install: pip install carla")
    carla = None

class ZenohBoundingBoxDetector:
    """Zenoh 기반 바운딩 박스 감지기"""
    
    def __init__(self, config_dict: Dict[str, Any]):
        """초기화"""
        self.config = config_dict
        self.session = None
        self.camera_subscriber = None
        self.bbox_publisher = None
        self.is_connected = False
        
        # 토픽 설정
        self.camera_topic = config_dict.get('camera_topic', 'carla/camera/image')
        self.bbox_topic = config_dict.get('bbox_topic', 'carla/chase/bounding_boxes')
        
        # 감지 설정
        self.max_distance = config_dict.get('max_distance', 200.0)
        self.detection_rate = config_dict.get('detection_rate', 10.0)  # FPS
        
        # 상태
        self.last_detection_time = 0
        self.detected_objects = []
        
        # 로깅 설정
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # 콜백
        self.detection_callback = None
        
    def connect(self) -> bool:
        """Zenoh에 연결"""
        try:
            if not zenoh:
                self.logger.error("Zenoh not available")
                return False
                
            # Zenoh 설정
            zenoh_config = Config()
            if 'locator' in self.config:
                zenoh_config.insert_json5("connect/endpoints", f'["tcp/{self.config["locator"]}"]')
            
            # 세션 생성
            self.session = zenoh.open(zenoh_config)
            
            # 카메라 구독자 생성
            self.camera_subscriber = self.session.declare_subscriber(
                self.camera_topic, 
                self._on_camera_image_received
            )
            
            # 바운딩 박스 퍼블리셔 생성
            self.bbox_publisher = self.session.declare_publisher(self.bbox_topic)
            
            self.is_connected = True
            self.logger.info(f"🌐 Connected to Zenoh")
            self.logger.info(f"📷 Subscribing to: {self.camera_topic}")
            self.logger.info(f"📦 Publishing to: {self.bbox_topic}")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Failed to connect to Zenoh: {e}")
            return False
    
    def disconnect(self):
        """Zenoh 연결 해제"""
        try:
            if self.camera_subscriber:
                self.camera_subscriber.undeclare()
                self.camera_subscriber = None
                
            if self.bbox_publisher:
                self.bbox_publisher.undeclare()
                self.bbox_publisher = None
                
            if self.session:
                self.session.close()
                self.session = None
                
            self.is_connected = False
            self.logger.info("🌐 Disconnected from Zenoh")
            
        except Exception as e:
            self.logger.error(f"❌ Error disconnecting from Zenoh: {e}")
    
    def _on_camera_image_received(self, sample):
        """카메라 이미지 수신 콜백"""
        try:
            # ZBytes 처리
            if hasattr(sample.payload, 'decode'):
                payload_str = sample.payload.decode('utf-8')
            else:
                payload_str = str(sample.payload)
            
            # JSON 파싱
            data = json.loads(payload_str)
            
            # 이미지 디코딩
            if 'data' in data and 'format' in data:
                if data['format'] == 'jpeg':
                    # JPEG 디코딩
                    img_bytes = bytes(data['data'])
                    nparr = np.frombuffer(img_bytes, np.uint8)
                    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if image is not None:
                        # 감지 주기 확인
                        current_time = time.time()
                        if current_time - self.last_detection_time >= 1.0 / self.detection_rate:
                            self._detect_objects(image, data.get('timestamp', current_time))
                            self.last_detection_time = current_time
                    
        except Exception as e:
            self.logger.error(f"❌ Error processing camera image: {e}")
    
    def _detect_objects(self, image: np.ndarray, timestamp: float):
        """객체 감지 및 바운딩 박스 생성"""
        try:
            # 여기서는 간단한 예시로 랜덤 바운딩 박스 생성
            # 실제로는 YOLO, CARLA ground truth 등을 사용
            height, width = image.shape[:2]
            
            # 랜덤 바운딩 박스 생성 (테스트용)
            num_objects = np.random.randint(0, 3)  # 0-2개 객체
            detected_objects = []
            
            for i in range(num_objects):
                # 랜덤 위치와 크기
                x_min = np.random.randint(0, width // 2)
                y_min = np.random.randint(0, height // 2)
                x_max = x_min + np.random.randint(50, 200)
                y_max = y_min + np.random.randint(50, 200)
                
                # 화면 경계 확인
                x_max = min(x_max, width)
                y_max = min(y_max, height)
                
                obj = {
                    'type': 'vehicle' if np.random.random() > 0.5 else 'pedestrian',
                    'actor_id': np.random.randint(100, 999),
                    'bbox_2d': {
                        'x_min': int(x_min),
                        'y_min': int(y_min),
                        'x_max': int(x_max),
                        'y_max': int(y_max),
                        'width': int(x_max - x_min),
                        'height': int(y_max - y_min)
                    },
                    'confidence': np.random.random(),
                    'world_location': [np.random.random() * 100, np.random.random() * 100, 0],
                    'velocity': [np.random.random() * 10, np.random.random() * 10, 0]
                }
                detected_objects.append(obj)
            
            # 바운딩 박스 데이터 퍼블리시
            self._publish_bounding_boxes(detected_objects, timestamp)
            
            # 콜백 호출
            if self.detection_callback:
                self.detection_callback(detected_objects, image, timestamp)
                
        except Exception as e:
            self.logger.error(f"❌ Error detecting objects: {e}")
    
    def _publish_bounding_boxes(self, objects: List[Dict], timestamp: float):
        """바운딩 박스 데이터 퍼블리시"""
        try:
            if not self.bbox_publisher:
                return
            
            # 데이터 패키징
            data = {
                'timestamp': timestamp,
                'objects': objects,
                'total_count': len(objects)
            }
            
            # JSON으로 직렬화
            json_data = json.dumps(data, default=str)
            
            # Zenoh로 퍼블리시
            self.bbox_publisher.put(json_data.encode('utf-8'))
            
            self.logger.debug(f"📦 Published {len(objects)} bounding boxes")
            
        except Exception as e:
            self.logger.error(f"❌ Error publishing bounding boxes: {e}")
    
    def set_detection_callback(self, callback: Callable[[List[Dict], np.ndarray, float], None]):
        """감지 콜백 설정"""
        self.detection_callback = callback
    
    def is_ready(self) -> bool:
        """감지기 준비 상태 확인"""
        return self.is_connected and self.camera_subscriber is not None and self.bbox_publisher is not None

def create_zenoh_bounding_box_detector_config(camera_topic: str = 'carla/camera/image',
                                            bbox_topic: str = 'carla/chase/bounding_boxes',
                                            max_distance: float = 200.0,
                                            detection_rate: float = 10.0,
                                            locator: Optional[str] = None) -> Dict[str, Any]:
    """Zenoh 바운딩 박스 감지기 설정 생성"""
    return {
        'camera_topic': camera_topic,
        'bbox_topic': bbox_topic,
        'max_distance': max_distance,
        'detection_rate': detection_rate,
        'locator': locator
    }

if __name__ == "__main__":
    # 테스트 코드
    config = create_zenoh_bounding_box_detector_config()
    detector = ZenohBoundingBoxDetector(config)
    
    def test_callback(objects, image, timestamp):
        print(f"🔍 Detected {len(objects)} objects at {timestamp}")
        for obj in objects:
            print(f"  - {obj['type']}: {obj['bbox_2d']}")
    
    detector.set_detection_callback(test_callback)
    
    if detector.connect():
        print("✅ Bounding box detector connected")
        
        # 10초간 대기
        time.sleep(10)
        
        detector.disconnect()
        print("✅ Test completed")
    else:
        print("❌ Failed to connect")
