#!/usr/bin/env python3
"""
Zenoh Collision Manager Module
Zenoh 기반 충돌 데이터 관리 모듈
"""

import json
import time
import logging
import numpy as np
import cv2
from typing import List, Dict, Optional, Callable
from dataclasses import dataclass

try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    zenoh = None

@dataclass
class ZenohCameraData:
    """Zenoh 카메라 데이터"""
    image: np.ndarray
    timestamp: float
    camera_id: str

@dataclass
class ZenohBoundingBoxData:
    """Zenoh 바운딩 박스 데이터"""
    objects: List[Dict]
    timestamp: float
    detection_id: str

class ZenohCollisionManager:
    """Zenoh 기반 충돌 데이터 관리자"""
    
    def __init__(self, config: Optional[Dict] = None):
        """
        Args:
            config: Zenoh 설정
        """
        self.config = config or {}
        self.session = None
        self.subscribers = []
        self.publisher = None
        
        # 데이터 저장소
        self.latest_camera_data: Optional[ZenohCameraData] = None
        self.latest_bbox_data: Optional[ZenohBoundingBoxData] = None
        
        # 콜백 함수들
        self.camera_callback: Optional[Callable] = None
        self.bbox_callback: Optional[Callable] = None
        
        # 로깅 설정
        self.logger = logging.getLogger(__name__)
        
    def setup_zenoh(self) -> bool:
        """Zenoh 설정 및 연결"""
        try:
            if not ZENOH_AVAILABLE:
                self.logger.warning("⚠️ Zenoh not available, skipping Zenoh setup")
                return False
            
            # Zenoh 설정
            zenoh_config = zenoh.Config()
            
            # SHM 설정 시도
            try:
                zenoh_config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
                zenoh_config.insert_json5("transport/shared_memory/enabled", "true")
                self.logger.info("🚀 Connecting to SHM-enabled Zenoh router...")
            except Exception:
                self.logger.info("📡 Using peer-to-peer Zenoh connection...")
            
            self.session = zenoh.open(zenoh_config)
            print("✅ Connected to Zenoh router successfully")
            
            # 토픽 구독 설정
            self._setup_subscriptions()
            
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Failed to setup Zenoh: {e}")
            self.session = None
            return False
    
    def _setup_subscriptions(self):
        """Zenoh 토픽 구독 설정"""
        try:
            # 차량 텔레메트리 구독
            telemetry_subscriber = self.session.declare_subscriber(
                "carla/vehicle/telemetry",
                self._on_telemetry_received
            )
            self.subscribers.append(telemetry_subscriber)
            self.logger.info("📡 Subscribed to vehicle telemetry")
            
            # 카메라 이미지 구독
            camera_subscriber = self.session.declare_subscriber(
                "carla/vehicle/camera",
                self._on_camera_received
            )
            self.subscribers.append(camera_subscriber)
            print("📷 Subscribed to camera images")
            
            # 바운딩 박스 구독
            bbox_subscriber = self.session.declare_subscriber(
                "carla/chase/bounding_boxes",
                self._on_bounding_boxes_received
            )
            self.subscribers.append(bbox_subscriber)
            print("🎯 Subscribed to bounding boxes")
            
        except Exception as e:
            self.logger.error(f"❌ Failed to setup subscriptions: {e}")
    
    def set_camera_callback(self, callback: Callable):
        """카메라 데이터 콜백 설정"""
        self.camera_callback = callback
    
    def set_bbox_callback(self, callback: Callable):
        """바운딩 박스 데이터 콜백 설정"""
        self.bbox_callback = callback
    
    def _on_telemetry_received(self, sample):
        """텔레메트리 데이터 수신 처리"""
        try:
            # Zenoh payload 처리
            if hasattr(sample.payload, 'to_string'):
                payload_str = sample.payload.to_string()
            else:
                payload_str = str(sample.payload)
            
            # JSON 파싱
            telemetry_data = json.loads(payload_str)
            self.logger.debug(f"📊 Received telemetry: {len(telemetry_data)} vehicles")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing telemetry: {e}")
    
    def _on_camera_received(self, sample):
        """카메라 이미지 수신 처리"""
        try:
            # Zenoh payload 처리 (zero-copy)
            if hasattr(sample.payload, 'to_bytes'):
                payload_bytes = sample.payload.to_bytes()
            else:
                payload_bytes = bytes(sample.payload)
            
            print(f"📷 Received camera data: {len(payload_bytes)} bytes")
            
            # JPEG 압축 해제 시도
            image = self._decode_camera_image(payload_bytes)
            
            if image is not None:
                # 카메라 데이터 저장
                camera_data = ZenohCameraData(
                    image=image,
                    timestamp=time.time(),
                    camera_id="zenoh_camera"
                )
                self.latest_camera_data = camera_data
                
                # 콜백 호출
                if self.camera_callback:
                    self.camera_callback(image)
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing camera image: {e}")
    
    def _on_bounding_boxes_received(self, sample):
        """바운딩 박스 데이터 수신 처리"""
        try:
            # Zenoh payload 처리
            if hasattr(sample.payload, 'to_string'):
                payload_str = sample.payload.to_string()
            else:
                payload_str = str(sample.payload)
            
            # JSON 파싱
            bbox_data = json.loads(payload_str)
            
            print(f"🎯 Received bounding boxes: {len(bbox_data.get('objects', []))} objects")
            
            # 바운딩 박스 데이터 저장
            bbox_data_obj = ZenohBoundingBoxData(
                objects=bbox_data.get('objects', []),
                timestamp=time.time(),
                detection_id="zenoh_detection"
            )
            self.latest_bbox_data = bbox_data_obj
            
            # 콜백 호출
            if self.bbox_callback:
                self.bbox_callback(bbox_data_obj.objects)
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing bounding boxes: {e}")
    
    def _decode_camera_image(self, payload_bytes: bytes) -> Optional[np.ndarray]:
        """카메라 이미지 디코딩"""
        try:
            # JPEG로 디코딩 시도
            image = cv2.imdecode(np.frombuffer(payload_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is not None:
                self.logger.debug(f"📷 Decoded compressed image: {image.shape}")
                return image
            
            # 압축 해제 실패 시 원시 이미지로 해석
            if len(payload_bytes) > 200000:  # 압축된 이미지로 추정
                self.logger.warning("⚠️ Failed to decode as compressed image")
                return None
            
            # 원시 이미지로 해석
            expected_size = 720 * 1080 * 3  # RGB
            if len(payload_bytes) == expected_size:
                image = np.frombuffer(payload_bytes, dtype=np.uint8).reshape(720, 1080, 3)
                self.logger.debug(f"📷 Interpreted as raw image: {image.shape}")
                return image
            else:
                self.logger.warning(f"⚠️ Unexpected payload size: {len(payload_bytes)} bytes")
                return None
                
        except Exception as e:
            self.logger.error(f"⚠️ Error decoding image: {e}")
            return None
    
    def get_latest_camera_data(self) -> Optional[ZenohCameraData]:
        """최신 카메라 데이터 반환"""
        return self.latest_camera_data
    
    def get_latest_bbox_data(self) -> Optional[ZenohBoundingBoxData]:
        """최신 바운딩 박스 데이터 반환"""
        return self.latest_bbox_data
    
    def publish_collision_alert(self, collision_data: Dict):
        """충돌 경고 발행"""
        try:
            if not self.session:
                return False
            
            # 발행자 설정 (필요시)
            if not self.publisher:
                self.publisher = self.session.declare_publisher("carla/chase/collision_alerts")
            
            # 충돌 데이터 발행
            payload = json.dumps(collision_data)
            self.publisher.put(payload)
            self.logger.info(f"📢 Published collision alert: {collision_data}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"⚠️ Error publishing collision alert: {e}")
            return False
    
    def cleanup(self):
        """리소스 정리"""
        try:
            # 구독자 정리
            for subscriber in self.subscribers:
                subscriber.undeclare()
            self.subscribers.clear()
            
            # 발행자 정리
            if self.publisher:
                self.publisher.undeclare()
                self.publisher = None
            
            # 세션 정리
            if self.session:
                self.session.close()
                self.session = None
            
            self.logger.info("✅ Zenoh collision manager cleaned up")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error during cleanup: {e}")
    
    def is_connected(self) -> bool:
        """Zenoh 연결 상태 확인"""
        return self.session is not None
