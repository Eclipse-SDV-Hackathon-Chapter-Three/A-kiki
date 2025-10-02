"""
Zenoh Bounding Box Publisher
바운딩 박스 탐지 데이터를 Zenoh를 통해 전송
"""

import zenoh
import json
import time
import threading
from typing import Dict, List, Any
import logging

class ZenohBoundingBoxPublisher:
    """Zenoh 바운딩 박스 데이터 퍼블리셔"""
    
    def __init__(self, config=None):
        self.config = config or {}
        self.session = None
        self.publisher = None
        self.is_running = False
        self.publish_thread = None
        self.data_queue = []
        self.queue_lock = threading.Lock()
        
        # Zenoh 설정
        self.zenoh_config = {
            'mode': self.config.get('mode', 'peer'),
            'locators': self.config.get('locators', ['tcp/127.0.0.1:7447']),
            'timeout': self.config.get('timeout', 5.0)
        }
        
        # 퍼블리시 설정
        self.topic = self.config.get('topic', 'carla/bounding_boxes')
        self.publish_rate = self.config.get('publish_rate', 10.0)  # Hz
        
        # 로깅 설정
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        print("🌐 Zenoh Bounding Box Publisher initialized")
    
    def connect(self):
        """Zenoh 세션 연결"""
        try:
            # Zenoh 설정 생성
            zenoh_config = zenoh.Config()
            zenoh_config.insert_json5("mode", f'"{self.zenoh_config["mode"]}"')
            zenoh_config.insert_json5("connect/endpoints", f'["{self.zenoh_config["locators"][0]}"]')
            
            # Zenoh 세션 생성
            self.session = zenoh.open(zenoh_config)
            
            # 퍼블리셔 생성
            self.publisher = self.session.declare_publisher(self.topic)
            
            self.logger.info(f"🌐 Connected to Zenoh on topic: {self.topic}")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Error connecting to Zenoh: {e}")
            return False
    
    def disconnect(self):
        """Zenoh 세션 연결 해제"""
        try:
            self.is_running = False
            
            if self.publish_thread and self.publish_thread.is_alive():
                self.publish_thread.join(timeout=2.0)
            
            if self.publisher:
                self.publisher.undeclare()
                self.publisher = None
            
            if self.session:
                self.session.close()
                self.session = None
            
            self.logger.info("🌐 Disconnected from Zenoh")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error disconnecting from Zenoh: {e}")
    
    def start_publishing(self):
        """데이터 퍼블리싱 시작"""
        try:
            if not self.session or not self.publisher:
                self.logger.error("❌ Not connected to Zenoh")
                return False
            
            self.is_running = True
            self.publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
            self.publish_thread.start()
            
            self.logger.info("📡 Started publishing bounding box data")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Error starting publishing: {e}")
            return False
    
    def stop_publishing(self):
        """데이터 퍼블리싱 중지"""
        try:
            self.is_running = False
            
            if self.publish_thread and self.publish_thread.is_alive():
                self.publish_thread.join(timeout=2.0)
            
            self.logger.info("📡 Stopped publishing bounding box data")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error stopping publishing: {e}")
    
    def publish_bounding_boxes(self, detection_data):
        """바운딩 박스 탐지 데이터 퍼블리시"""
        try:
            if not self.is_running or not self.publisher:
                return False
            
            # 데이터 큐에 추가
            with self.queue_lock:
                self.data_queue.append(detection_data)
                
                # 큐 크기 제한 (최대 10개)
                if len(self.data_queue) > 10:
                    self.data_queue.pop(0)
            
            return True
            
        except Exception as e:
            self.logger.error(f"⚠️ Error publishing bounding boxes: {e}")
            return False
    
    def _publish_loop(self):
        """퍼블리시 루프"""
        try:
            while self.is_running:
                with self.queue_lock:
                    if self.data_queue:
                        # 가장 최근 데이터 가져오기
                        data = self.data_queue[-1]
                        
                        # JSON으로 직렬화
                        json_data = json.dumps(data, indent=2)
                        
                        # Zenoh로 퍼블리시
                        self.publisher.put(json_data.encode('utf-8'))
                        
                        self.logger.debug(f"📡 Published {len(data.get('detections', []))} detections")
                
                # 퍼블리시 주기 대기
                time.sleep(1.0 / self.publish_rate)
                
        except Exception as e:
            self.logger.error(f"⚠️ Error in publish loop: {e}")
    
    def get_connection_status(self):
        """연결 상태 확인"""
        return {
            'connected': self.session is not None and self.publisher is not None,
            'publishing': self.is_running,
            'topic': self.topic,
            'queue_size': len(self.data_queue)
        }
    
    def get_statistics(self):
        """퍼블리시 통계 반환"""
        try:
            with self.queue_lock:
                queue_size = len(self.data_queue)
            
            return {
                'queue_size': queue_size,
                'publish_rate': self.publish_rate,
                'is_running': self.is_running,
                'topic': self.topic
            }
            
        except Exception as e:
            self.logger.error(f"⚠️ Error getting statistics: {e}")
            return {
                'queue_size': 0,
                'publish_rate': 0,
                'is_running': False,
                'topic': 'unknown'
            }


class ZenohBoundingBoxSubscriber:
    """Zenoh 바운딩 박스 데이터 구독자"""
    
    def __init__(self, config=None):
        self.config = config or {}
        self.session = None
        self.subscriber = None
        self.is_running = False
        self.received_data = None
        self.data_lock = threading.Lock()
        self.callback = None
        
        # Zenoh 설정
        self.zenoh_config = {
            'mode': self.config.get('mode', 'peer'),
            'locators': self.config.get('locators', ['tcp/127.0.0.1:7447']),
            'timeout': self.config.get('timeout', 5.0)
        }
        
        # 구독 설정
        self.topic = self.config.get('topic', 'carla/bounding_boxes')
        
        # 로깅 설정
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        print("🌐 Zenoh Bounding Box Subscriber initialized")
    
    def connect(self):
        """Zenoh 세션 연결"""
        try:
            # Zenoh 설정 생성
            zenoh_config = zenoh.Config()
            zenoh_config.insert_json5("mode", f'"{self.zenoh_config["mode"]}"')
            zenoh_config.insert_json5("connect/endpoints", f'["{self.zenoh_config["locators"][0]}"]')
            
            # Zenoh 세션 생성
            self.session = zenoh.open(zenoh_config)
            
            # 구독자 생성
            self.subscriber = self.session.declare_subscriber(self.topic, self._on_data)
            
            self.logger.info(f"🌐 Connected to Zenoh subscribing to: {self.topic}")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Error connecting to Zenoh: {e}")
            return False
    
    def disconnect(self):
        """Zenoh 세션 연결 해제"""
        try:
            self.is_running = False
            
            if self.subscriber:
                self.subscriber.undeclare()
                self.subscriber = None
            
            if self.session:
                self.session.close()
                self.session = None
            
            self.logger.info("🌐 Disconnected from Zenoh")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error disconnecting from Zenoh: {e}")
    
    def start_subscribing(self):
        """데이터 구독 시작"""
        try:
            if not self.session or not self.subscriber:
                self.logger.error("❌ Not connected to Zenoh")
                return False
            
            self.is_running = True
            self.logger.info("📡 Started subscribing to bounding box data")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Error starting subscription: {e}")
            return False
    
    def stop_subscribing(self):
        """데이터 구독 중지"""
        try:
            self.is_running = False
            self.logger.info("📡 Stopped subscribing to bounding box data")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error stopping subscription: {e}")
    
    def _on_data(self, sample):
        """데이터 수신 콜백"""
        try:
            # JSON 데이터 파싱
            data = json.loads(sample.payload.decode('utf-8'))
            
            # 데이터 저장
            with self.data_lock:
                self.received_data = data
            
            # 사용자 콜백 호출
            if self.callback:
                self.callback(data)
            
            self.logger.debug(f"📡 Received {len(data.get('detections', []))} detections")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing received data: {e}")
    
    def get_latest_data(self):
        """최신 데이터 가져오기"""
        with self.data_lock:
            return self.received_data.copy() if self.received_data else None
    
    def set_callback(self, callback):
        """데이터 수신 콜백 설정"""
        self.callback = callback
    
    def get_connection_status(self):
        """연결 상태 확인"""
        return {
            'connected': self.session is not None and self.subscriber is not None,
            'subscribing': self.is_running,
            'topic': self.topic,
            'has_data': self.received_data is not None
        }


def create_zenoh_config(mode='peer', locators=None, topic='carla/bounding_boxes', publish_rate=10.0):
    """Zenoh 설정 생성 헬퍼 함수"""
    if locators is None:
        locators = ['tcp/127.0.0.1:7447']
    
    return {
        'mode': mode,
        'locators': locators,
        'topic': topic,
        'publish_rate': publish_rate,
        'timeout': 5.0
    }


# 사용 예제
if __name__ == "__main__":
    # 퍼블리셔 예제
    print("🌐 Zenoh Bounding Box Publisher Example")
    
    config = create_zenoh_config()
    publisher = ZenohBoundingBoxPublisher(config)
    
    if publisher.connect():
        publisher.start_publishing()
        
        # 테스트 데이터 전송
        test_data = {
            'timestamp': time.time(),
            'camera_id': 'test_camera',
            'detections': [
                {
                    'type': 'vehicle',
                    'class_name': 'car',
                    'bbox_2d': {'x_min': 100, 'y_min': 100, 'x_max': 200, 'y_max': 150},
                    'confidence': 0.95
                }
            ]
        }
        
        publisher.publish_bounding_boxes(test_data)
        
        time.sleep(5)
        publisher.stop_publishing()
        publisher.disconnect()
    
    print("🌐 Example completed")
