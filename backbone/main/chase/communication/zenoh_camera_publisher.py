#!/usr/bin/env python3
"""
Zenoh 카메라 이미지 퍼블리셔
카메라 이미지를 Zenoh로 전송하는 모듈
"""

import sys
import os
import time
import logging
from typing import Optional, Dict, Any
import numpy as np
import cv2

# Zenoh import
try:
    import zenoh
    from zenoh import Config
except ImportError:
    print("⚠️ Zenoh not available. Please install: pip install zenoh")
    zenoh = None
    Config = None

class ZenohCameraPublisher:
    """Zenoh를 통한 카메라 이미지 퍼블리셔"""
    
    def __init__(self, config_dict: Dict[str, Any]):
        """초기화"""
        self.config = config_dict
        self.session = None
        self.publisher = None
        self.is_connected = False
        self.publish_rate = config_dict.get('publish_rate', 30.0)  # FPS
        self.topic = config_dict.get('topic', 'carla/camera/image')
        self.last_publish_time = 0
        
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
            
            # 퍼블리셔 생성
            self.publisher = self.session.declare_publisher(self.topic)
            
            self.is_connected = True
            self.logger.info(f"🌐 Connected to Zenoh, publishing to: {self.topic}")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Failed to connect to Zenoh: {e}")
            return False
    
    def disconnect(self):
        """Zenoh 연결 해제"""
        try:
            if self.publisher:
                self.publisher.undeclare()
                self.publisher = None
                
            if self.session:
                self.session.close()
                self.session = None
                
            self.is_connected = False
            self.logger.info("🌐 Disconnected from Zenoh")
            
        except Exception as e:
            self.logger.error(f"❌ Error disconnecting from Zenoh: {e}")
    
    def publish_image(self, image: np.ndarray, timestamp: Optional[float] = None) -> bool:
        """카메라 이미지 퍼블리시"""
        try:
            if not self.is_connected or not self.publisher:
                return False
            
            # 퍼블리시 주기 확인
            current_time = time.time()
            if current_time - self.last_publish_time < 1.0 / self.publish_rate:
                return True  # 아직 퍼블리시할 시간이 아님
            
            # 타임스탬프 설정
            if timestamp is None:
                timestamp = current_time
            
            # 이미지 압축 (JPEG)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
            _, encoded_img = cv2.imencode('.jpg', image, encode_param)
            
            # 데이터 패키징
            data = {
                'timestamp': timestamp,
                'width': image.shape[1],
                'height': image.shape[0],
                'channels': image.shape[2] if len(image.shape) == 3 else 1,
                'format': 'jpeg',
                'data': encoded_img.tobytes()
            }
            
            # JSON으로 직렬화
            import json
            json_data = json.dumps(data, default=str)
            
            # Zenoh로 퍼블리시
            self.publisher.put(json_data.encode('utf-8'))
            
            self.last_publish_time = current_time
            self.logger.debug(f"📷 Published image: {image.shape}, size: {len(encoded_img)} bytes")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Error publishing image: {e}")
            return False
    
    def is_ready(self) -> bool:
        """퍼블리셔 준비 상태 확인"""
        return self.is_connected and self.publisher is not None

def create_zenoh_camera_config(topic: str = 'carla/camera/image', 
                              publish_rate: float = 30.0,
                              locator: Optional[str] = None) -> Dict[str, Any]:
    """Zenoh 카메라 설정 생성"""
    return {
        'topic': topic,
        'publish_rate': publish_rate,
        'locator': locator
    }

if __name__ == "__main__":
    # 테스트 코드
    config = create_zenoh_camera_config()
    publisher = ZenohCameraPublisher(config)
    
    if publisher.connect():
        print("✅ Camera publisher connected")
        
        # 테스트 이미지 생성
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # 이미지 퍼블리시
        for i in range(10):
            publisher.publish_image(test_image)
            time.sleep(0.1)
        
        publisher.disconnect()
        print("✅ Test completed")
    else:
        print("❌ Failed to connect")
