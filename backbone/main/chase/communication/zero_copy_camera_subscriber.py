#!/usr/bin/env python3
"""
Zero-Copy Camera Subscriber
Shared Memory 기반 Zero-Copy 카메라 이미지 구독 클래스
"""

import json
import time
import logging
import numpy as np
import cv2
from typing import Optional, Callable, Dict, Any
from multiprocessing import shared_memory
import struct
import threading

try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    zenoh = None

# SHM 헤더 구조
HEADER_SIZE = 16
UINT64 = struct.Struct("<Q")

class ZeroCopyCameraSubscriber:
    """Zero-Copy 카메라 이미지 구독자"""
    
    def __init__(self, shm_name: str = "carla_camera_shm", shm_capacity: int = 2 * 1024 * 1024):
        """
        Args:
            shm_name: Shared Memory 세그먼트 이름
            shm_capacity: SHM 용량 (바이트)
        """
        self.shm_name = shm_name
        self.shm_capacity = shm_capacity
        self.zenoh_session = None
        self.subscriber = None
        self.running = False
        
        # 콜백 함수
        self.camera_callback: Optional[Callable] = None
        self.bbox_callback: Optional[Callable] = None
        
        # 로깅
        self.logger = logging.getLogger(__name__)
        
        # 성능 메트릭
        self.frame_count = 0
        self.last_frame_time = 0.0
        self.fps = 0.0
        
    def setup_zenoh(self) -> bool:
        """Zenoh 설정 및 연결"""
        try:
            if not ZENOH_AVAILABLE:
                self.logger.warning("⚠️ Zenoh not available")
                return False
            
            # Zenoh 설정
            zenoh_config = zenoh.Config()
            zenoh_config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
            
            # 세션 생성
            self.zenoh_session = zenoh.open(zenoh_config)
            self.logger.info("✅ Zero-Copy camera subscriber connected to Zenoh")
            
            # 구독 설정
            self._setup_subscriptions()
            
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Failed to setup Zenoh: {e}")
            self.zenoh_session = None
            return False
    
    def _setup_subscriptions(self):
        """Zenoh 구독 설정"""
        try:
            # 카메라 메타데이터 구독 (SHM 참조용)
            self.subscriber = self.zenoh_session.declare_subscriber(
                "carla/vehicle/camera/meta",
                self._on_camera_meta_received
            )
            self.logger.info("📷 Subscribed to camera metadata (SHM-based)")
            
            # 바운딩 박스 구독 (기존 방식 유지)
            bbox_subscriber = self.zenoh_session.declare_subscriber(
                "carla/chase/bounding_boxes",
                self._on_bounding_boxes_received
            )
            self.logger.info("🎯 Subscribed to bounding boxes")
            
        except Exception as e:
            self.logger.error(f"❌ Failed to setup subscriptions: {e}")
    
    def _on_camera_meta_received(self, sample):
        """카메라 메타데이터 수신 처리 (Zero-Copy)"""
        try:
            # 메타데이터 파싱
            if hasattr(sample.payload, 'to_string'):
                meta_str = sample.payload.to_string()
            else:
                meta_str = str(sample.payload)
            
            meta_data = json.loads(meta_str)
            
            # SHM에서 직접 이미지 읽기 (Zero-Copy)
            image_data = self._read_image_from_shm(meta_data)
            
            if image_data is not None:
                # 성능 메트릭 업데이트
                self._update_metrics()
                
                # 콜백 호출
                if self.camera_callback:
                    self.camera_callback(image_data)
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing camera metadata: {e}")
    
    def _read_image_from_shm(self, meta_data: Dict[str, Any]) -> Optional[np.ndarray]:
        """SHM에서 직접 이미지 읽기 (Zero-Copy)"""
        try:
            shm_name = meta_data.get('shm_name', self.shm_name)
            shm_capacity = meta_data.get('shm_capacity', self.shm_capacity)
            
            # SHM 열기
            shm = shared_memory.SharedMemory(name=shm_name, create=False, size=HEADER_SIZE + shm_capacity)
            buf = shm.buf
            
            try:
                # 헤더 읽기
                seq0 = UINT64.unpack_from(buf, 0)[0]
                if seq0 & 1:  # 쓰기 중이면 스킵
                    return None
                
                size = UINT64.unpack_from(buf, 8)[0]
                if size == 0 or size > shm_capacity:
                    return None
                
                # 데이터 읽기 (Zero-Copy)
                data = bytes(buf[HEADER_SIZE:HEADER_SIZE + size])
                
                # seq1 확인 (원자성 보장)
                seq1 = UINT64.unpack_from(buf, 0)[0]
                if seq0 != seq1 or (seq1 & 1):
                    return None
                
                # JPEG 디코딩 (최소한의 복사)
                image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                
                return image
                
            finally:
                shm.close()
                
        except FileNotFoundError:
            self.logger.warning(f"⚠️ SHM segment not found: {shm_name}")
            return None
        except Exception as e:
            self.logger.error(f"⚠️ Error reading from SHM: {e}")
            return None
    
    def _on_bounding_boxes_received(self, sample):
        """바운딩 박스 데이터 수신 처리"""
        try:
            if hasattr(sample.payload, 'to_string'):
                payload_str = sample.payload.to_string()
            else:
                payload_str = str(sample.payload)
            
            bbox_data = json.loads(payload_str)
            
            # 콜백 호출
            if self.bbox_callback:
                self.bbox_callback(bbox_data.get('objects', []))
                
        except Exception as e:
            self.logger.error(f"⚠️ Error processing bounding boxes: {e}")
    
    def _update_metrics(self):
        """성능 메트릭 업데이트"""
        current_time = time.time()
        self.frame_count += 1
        
        if self.last_frame_time > 0:
            time_diff = current_time - self.last_frame_time
            if time_diff > 0:
                self.fps = 1.0 / time_diff
        
        self.last_frame_time = current_time
        
        # 1초마다 FPS 로그
        if self.frame_count % 30 == 0:
            self.logger.info(f"📊 Zero-Copy Camera FPS: {self.fps:.1f}")
    
    def set_camera_callback(self, callback: Callable):
        """카메라 데이터 콜백 설정"""
        self.camera_callback = callback
    
    def set_bbox_callback(self, callback: Callable):
        """바운딩 박스 데이터 콜백 설정"""
        self.bbox_callback = callback
    
    def start(self):
        """구독 시작"""
        self.running = True
        self.logger.info("🚀 Zero-Copy camera subscriber started")
    
    def stop(self):
        """구독 중지"""
        self.running = False
        self.logger.info("⏹️ Zero-Copy camera subscriber stopped")
    
    def cleanup(self):
        """리소스 정리"""
        try:
            self.stop()
            
            if self.subscriber:
                self.subscriber.undeclare()
                self.subscriber = None
            
            if self.zenoh_session:
                self.zenoh_session.close()
                self.zenoh_session = None
            
            self.logger.info("✅ Zero-Copy camera subscriber cleaned up")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error during cleanup: {e}")
    
    def get_metrics(self) -> Dict[str, Any]:
        """성능 메트릭 반환"""
        return {
            'frame_count': self.frame_count,
            'fps': self.fps,
            'shm_name': self.shm_name,
            'shm_capacity': self.shm_capacity
        }

