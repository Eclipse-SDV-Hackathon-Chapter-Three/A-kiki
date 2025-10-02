#!/usr/bin/env python3
"""
LIDAR Subscriber
Zenoh를 통해 Semantic LIDAR 데이터를 구독하는 클래스
"""

import time
import json
import logging
from typing import List, Dict, Any, Optional, Callable
from collections import deque

try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    zenoh = None

class LidarSubscriber:
    """Zenoh를 통해 LIDAR 데이터를 구독하는 클래스"""
    
    def __init__(self, 
                 raw_data_topic: str = "carla/semantic_lidar/raw",
                 processed_data_topic: str = "carla/semantic_lidar/processed",
                 vehicle_tracking_topic: str = "carla/semantic_lidar/vehicles",
                 accident_alerts_topic: str = "carla/semantic_lidar/accidents",
                 summary_topic: str = "carla/semantic_lidar/summary"):
        
        self.topics = {
            'raw_data': raw_data_topic,
            'processed_data': processed_data_topic,
            'vehicle_tracking': vehicle_tracking_topic,
            'accident_alerts': accident_alerts_topic,
            'summary': summary_topic
        }
        
        self.zenoh_session = None
        self.subscribers = []
        
        # 콜백 함수들
        self.raw_data_callback: Optional[Callable] = None
        self.processed_data_callback: Optional[Callable] = None
        self.vehicle_tracking_callback: Optional[Callable] = None
        self.accident_alerts_callback: Optional[Callable] = None
        self.summary_callback: Optional[Callable] = None
        
        # 데이터 버퍼
        self.data_buffers = {
            'raw_data': deque(maxlen=100),
            'processed_data': deque(maxlen=100),
            'vehicle_tracking': deque(maxlen=100),
            'accident_alerts': deque(maxlen=100),
            'summary': deque(maxlen=100)
        }
        
        # 로깅
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO)
    
    def setup_zenoh(self) -> bool:
        """Zenoh 설정 및 구독"""
        try:
            if not ZENOH_AVAILABLE:
                self.logger.warning("⚠️ Zenoh not available")
                return False
            
            # Zenoh 설정
            zenoh_config = zenoh.Config()
            zenoh_config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
            
            # 세션 생성
            self.zenoh_session = zenoh.open(zenoh_config)
            
            # 각 토픽에 대한 구독자 설정
            self.subscribers.append(
                self.zenoh_session.declare_subscriber(
                    self.topics['raw_data'], 
                    self._on_raw_data_received
                )
            )
            
            self.subscribers.append(
                self.zenoh_session.declare_subscriber(
                    self.topics['processed_data'], 
                    self._on_processed_data_received
                )
            )
            
            self.subscribers.append(
                self.zenoh_session.declare_subscriber(
                    self.topics['vehicle_tracking'], 
                    self._on_vehicle_tracking_received
                )
            )
            
            self.subscribers.append(
                self.zenoh_session.declare_subscriber(
                    self.topics['accident_alerts'], 
                    self._on_accident_alerts_received
                )
            )
            
            self.subscribers.append(
                self.zenoh_session.declare_subscriber(
                    self.topics['summary'], 
                    self._on_summary_received
                )
            )
            
            self.logger.info("✅ LIDAR subscriber setup successful")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Failed to setup LIDAR subscriber: {e}")
            return False
    
    def _on_raw_data_received(self, sample):
        """원시 데이터 수신 처리"""
        try:
            data_str = sample.payload.decode('utf-8')
            data = json.loads(data_str)
            
            # 버퍼에 저장
            self.data_buffers['raw_data'].append(data)
            
            # 콜백 호출
            if self.raw_data_callback:
                self.raw_data_callback(data)
            
            self.logger.debug(f"📡 Received raw LIDAR data: {data.get('point_count', 0)} points")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing raw data: {e}")
    
    def _on_processed_data_received(self, sample):
        """처리된 데이터 수신 처리"""
        try:
            data_str = sample.payload.decode('utf-8')
            data = json.loads(data_str)
            
            # 버퍼에 저장
            self.data_buffers['processed_data'].append(data)
            
            # 콜백 호출
            if self.processed_data_callback:
                self.processed_data_callback(data)
            
            self.logger.debug(f"📡 Received processed LIDAR data: {data.get('total_points', 0)} points")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing processed data: {e}")
    
    def _on_vehicle_tracking_received(self, sample):
        """차량 추적 데이터 수신 처리"""
        try:
            data_str = sample.payload.decode('utf-8')
            data = json.loads(data_str)
            
            # 버퍼에 저장
            self.data_buffers['vehicle_tracking'].append(data)
            
            # 콜백 호출
            if self.vehicle_tracking_callback:
                self.vehicle_tracking_callback(data)
            
            vehicle_count = data.get('vehicle_count', 0)
            self.logger.debug(f"📡 Received vehicle tracking data: {vehicle_count} vehicles")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing vehicle tracking data: {e}")
    
    def _on_accident_alerts_received(self, sample):
        """사고 알림 데이터 수신 처리"""
        try:
            data_str = sample.payload.decode('utf-8')
            data = json.loads(data_str)
            
            # 버퍼에 저장
            self.data_buffers['accident_alerts'].append(data)
            
            # 콜백 호출
            if self.accident_alerts_callback:
                self.accident_alerts_callback(data)
            
            accident_count = data.get('accident_count', 0)
            if accident_count > 0:
                self.logger.warning(f"🚨 Received accident alerts: {accident_count} accidents")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing accident alerts: {e}")
    
    def _on_summary_received(self, sample):
        """요약 데이터 수신 처리"""
        try:
            data_str = sample.payload.decode('utf-8')
            data = json.loads(data_str)
            
            # 버퍼에 저장
            self.data_buffers['summary'].append(data)
            
            # 콜백 호출
            if self.summary_callback:
                self.summary_callback(data)
            
            self.logger.debug(f"📡 Received LIDAR summary data")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing summary data: {e}")
    
    def set_raw_data_callback(self, callback: Callable):
        """원시 데이터 콜백 설정"""
        self.raw_data_callback = callback
    
    def set_processed_data_callback(self, callback: Callable):
        """처리된 데이터 콜백 설정"""
        self.processed_data_callback = callback
    
    def set_vehicle_tracking_callback(self, callback: Callable):
        """차량 추적 콜백 설정"""
        self.vehicle_tracking_callback = callback
    
    def set_accident_alerts_callback(self, callback: Callable):
        """사고 알림 콜백 설정"""
        self.accident_alerts_callback = callback
    
    def set_summary_callback(self, callback: Callable):
        """요약 데이터 콜백 설정"""
        self.summary_callback = callback
    
    def get_latest_data(self, data_type: str) -> Optional[Dict[str, Any]]:
        """최신 데이터 가져오기"""
        try:
            if data_type not in self.data_buffers:
                return None
            
            buffer = self.data_buffers[data_type]
            if not buffer:
                return None
            
            return buffer[-1]
            
        except Exception as e:
            self.logger.error(f"⚠️ Error getting latest data: {e}")
            return None
    
    def get_all_data(self, data_type: str) -> List[Dict[str, Any]]:
        """모든 데이터 가져오기"""
        try:
            if data_type not in self.data_buffers:
                return []
            
            return list(self.data_buffers[data_type])
            
        except Exception as e:
            self.logger.error(f"⚠️ Error getting all data: {e}")
            return []
    
    def get_vehicle_tracking_data(self) -> Optional[Dict[str, Any]]:
        """차량 추적 데이터 가져오기"""
        return self.get_latest_data('vehicle_tracking')
    
    def get_accident_data(self) -> Optional[Dict[str, Any]]:
        """사고 데이터 가져오기"""
        return self.get_latest_data('accident_alerts')
    
    def get_summary_data(self) -> Optional[Dict[str, Any]]:
        """요약 데이터 가져오기"""
        return self.get_latest_data('summary')
    
    def cleanup(self):
        """리소스 정리"""
        try:
            # 구독자 정리
            for subscriber in self.subscribers:
                subscriber.undeclare()
            self.subscribers = []
            
            # 세션 정리
            if self.zenoh_session:
                self.zenoh_session.close()
                self.zenoh_session = None
            
            # 버퍼 정리
            for buffer in self.data_buffers.values():
                buffer.clear()
            
            self.logger.info("✅ LIDAR subscriber cleaned up")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error during LIDAR subscriber cleanup: {e}")

