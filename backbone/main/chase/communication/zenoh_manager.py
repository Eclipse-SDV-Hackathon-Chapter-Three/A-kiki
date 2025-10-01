#!/usr/bin/env python3

"""
Zenoh Communication Manager
Zenoh를 통한 토픽 기반 통신 관리
"""

import json
import time
from typing import Dict, Any, Optional, Callable
from zenoh import Session, Config, Value, Sample
import threading

class ZenohManager:
    """Zenoh 통신 관리자"""
    
    def __init__(self, config: Optional[Dict] = None):
        self.config = config or {}
        self.session = None
        self.subscribers = {}
        self.publishers = {}
        self.running = False
        self.threads = []
        
        # 토픽 정의
        self.topics = {
            'detection': 'carla/police/detection',
            'collision': 'carla/police/collision',
            'chase_command': 'carla/police/chase_command',
            'vehicle_status': 'carla/police/vehicle_status',
            'pedestrian_status': 'carla/police/pedestrian_status',
            'emergency': 'carla/police/emergency'
        }
        
        print("🌐 Zenoh Manager initialized")
    
    def connect(self):
        """Zenoh 연결"""
        try:
            # Zenoh 설정
            zenoh_config = Config()
            if 'router_endpoint' in self.config:
                zenoh_config.insert_json5("connect/endpoints", f'["{self.config["router_endpoint"]}"]')
            
            # 세션 생성
            self.session = Session.open(zenoh_config)
            self.running = True
            print("✅ Connected to Zenoh")
            return True
            
        except Exception as e:
            print(f"❌ Failed to connect to Zenoh: {e}")
            return False
    
    def disconnect(self):
        """Zenoh 연결 해제"""
        try:
            self.running = False
            
            # 모든 구독자 정리
            for topic, subscriber in self.subscribers.items():
                if subscriber:
                    subscriber.close()
            
            # 모든 발행자 정리
            for topic, publisher in self.publishers.items():
                if publisher:
                    publisher.close()
            
            # 스레드 정리
            for thread in self.threads:
                if thread.is_alive():
                    thread.join(timeout=1.0)
            
            # 세션 정리
            if self.session:
                self.session.close()
            
            print("🌐 Disconnected from Zenoh")
            
        except Exception as e:
            print(f"⚠️ Error disconnecting from Zenoh: {e}")
    
    def publish(self, topic_key: str, data: Dict[str, Any]):
        """데이터 발행"""
        try:
            if not self.session or not self.running:
                return False
            
            topic = self.topics.get(topic_key)
            if not topic:
                print(f"❌ Unknown topic key: {topic_key}")
                return False
            
            # 발행자 가져오기 또는 생성
            if topic_key not in self.publishers:
                self.publishers[topic_key] = self.session.declare_publisher(topic)
            
            # 데이터 직렬화
            json_data = json.dumps(data, default=str)
            value = Value(json_data)
            
            # 발행
            self.publishers[topic_key].put(value)
            return True
            
        except Exception as e:
            print(f"⚠️ Error publishing to {topic_key}: {e}")
            return False
    
    def subscribe(self, topic_key: str, callback: Callable[[Dict[str, Any]], None]):
        """데이터 구독"""
        try:
            if not self.session or not self.running:
                return False
            
            topic = self.topics.get(topic_key)
            if not topic:
                print(f"❌ Unknown topic key: {topic_key}")
                return False
            
            # 구독자 생성
            subscriber = self.session.declare_subscriber(topic, self._create_callback(callback))
            self.subscribers[topic_key] = subscriber
            
            print(f"✅ Subscribed to {topic_key}: {topic}")
            return True
            
        except Exception as e:
            print(f"⚠️ Error subscribing to {topic_key}: {e}")
            return False
    
    def _create_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """콜백 함수 생성"""
        def zenoh_callback(sample: Sample):
            try:
                if sample.payload:
                    data = json.loads(sample.payload.decode('utf-8'))
                    callback(data)
            except Exception as e:
                print(f"⚠️ Error in callback: {e}")
        
        return zenoh_callback
    
    def get_topic(self, topic_key: str) -> Optional[str]:
        """토픽 이름 반환"""
        return self.topics.get(topic_key)
    
    def is_connected(self) -> bool:
        """연결 상태 확인"""
        return self.session is not None and self.running
