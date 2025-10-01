#!/usr/bin/env python3
"""
Zenoh 카메라 이미지 구독자
Zenoh로부터 카메라 이미지를 받아서 처리하는 모듈
"""

import sys
import os
import time
import logging
from typing import Optional, Dict, Any, Callable
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

class ZenohCameraSubscriber:
    """Zenoh를 통한 카메라 이미지 구독자"""
    
    def __init__(self, config_dict: Dict[str, Any]):
        """초기화"""
        self.config = config_dict
        self.session = None
        self.subscriber = None
        self.is_connected = False
        self.topic = config_dict.get('topic', 'carla/camera/image')
        self.callback = None
        self.last_image = None
        self.last_timestamp = 0
        
        # 로깅 설정
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
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
            
            # 구독자 생성
            self.subscriber = self.session.declare_subscriber(self.topic, self._on_image_received)
            
            self.is_connected = True
            self.logger.info(f"🌐 Connected to Zenoh, subscribing to: {self.topic}")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Failed to connect to Zenoh: {e}")
            return False
    
    def disconnect(self):
        """Zenoh 연결 해제"""
        try:
            if self.subscriber:
                self.subscriber.undeclare()
                self.subscriber = None
                
            if self.session:
                self.session.close()
                self.session = None
                
            self.is_connected = False
            self.logger.info("🌐 Disconnected from Zenoh")
            
        except Exception as e:
            self.logger.error(f"❌ Error disconnecting from Zenoh: {e}")
    
    def _on_image_received(self, sample):
        """이미지 수신 콜백"""
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
                        self.last_image = image
                        self.last_timestamp = data.get('timestamp', time.time())
                        
                        # 콜백 호출
                        if self.callback:
                            self.callback(image, self.last_timestamp)
                        
                        self.logger.debug(f"📷 Received image: {image.shape}, timestamp: {self.last_timestamp}")
                    else:
                        self.logger.warning("⚠️ Failed to decode image")
                else:
                    self.logger.warning(f"⚠️ Unsupported image format: {data['format']}")
            else:
                self.logger.warning("⚠️ Invalid image data format")
                
        except Exception as e:
            self.logger.error(f"❌ Error processing image: {e}")
    
    def set_callback(self, callback: Callable[[np.ndarray, float], None]):
        """이미지 수신 콜백 설정"""
        self.callback = callback
    
    def get_latest_image(self) -> Optional[np.ndarray]:
        """최신 이미지 가져오기"""
        return self.last_image
    
    def get_latest_timestamp(self) -> float:
        """최신 타임스탬프 가져오기"""
        return self.last_timestamp
    
    def is_ready(self) -> bool:
        """구독자 준비 상태 확인"""
        return self.is_connected and self.subscriber is not None

def create_zenoh_camera_subscriber_config(topic: str = 'carla/camera/image',
                                        locator: Optional[str] = None) -> Dict[str, Any]:
    """Zenoh 카메라 구독자 설정 생성"""
    return {
        'topic': topic,
        'locator': locator
    }

if __name__ == "__main__":
    # 테스트 코드
    config = create_zenoh_camera_subscriber_config()
    subscriber = ZenohCameraSubscriber(config)
    
    def test_callback(image, timestamp):
        print(f"📷 Received image: {image.shape}, timestamp: {timestamp}")
    
    subscriber.set_callback(test_callback)
    
    if subscriber.connect():
        print("✅ Camera subscriber connected")
        
        # 10초간 대기
        time.sleep(10)
        
        subscriber.disconnect()
        print("✅ Test completed")
    else:
        print("❌ Failed to connect")
