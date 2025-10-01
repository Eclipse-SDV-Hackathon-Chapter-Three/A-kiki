"""
Collision Detector for Pedestrians
보행자 충돌 감지를 위한 바운딩 박스 분석
"""

import numpy as np
import time
import json
from typing import Dict, List, Tuple, Optional
from collections import deque
import math

# Zenoh imports
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    print("Warning: zenoh not available, collision alarms will not be published")

class CollisionDetector:
    """보행자 충돌 감지 클래스"""
    
    def __init__(self, history_size=10, enable_zenoh=True):
        self.history_size = history_size
        self.pedestrian_history = {}  # {actor_id: deque of bbox_data}
        self.collision_thresholds = {
            'ground_proximity': 0.5,      # 지면 근접성 임계값 (m)
            'min_history': 2,             # 최소 히스토리 길이
            'collision_score_threshold': 0.5  # 충돌 점수 임계값
        }
        self.detected_collisions = {}  # {actor_id: collision_data}
        
        # Zenoh 관련 변수
        self.enable_zenoh = enable_zenoh and ZENOH_AVAILABLE
        self.zenoh_session = None
        self.collision_alarm_topic = "carla/police/collision_alarm"
        self.lock_on_topic = "lock_on"  # 충돌 감지 시 true 전송
        self.collision_alarm_cooldown = 5.0  # 5초 쿨다운
        self._recent_collision_alarms = {}  # 중복 알람 방지
        self._collision_alarm_ttl = 60.0  # 60초 TTL
        self._lock_on_sent = False  # lock_on 메시지 전송 상태
        
        # Zenoh 설정
        if self.enable_zenoh:
            self._setup_zenoh()
        
        print("🚨 Collision Detector initialized")
    
    def _setup_zenoh(self):
        """Zenoh 세션 설정"""
        try:
            if not ZENOH_AVAILABLE:
                print("⚠️ Zenoh not available, collision alarms disabled")
                self.enable_zenoh = False
                return False
            
            # Zenoh 설정
            zenoh_config = zenoh.Config()
            zenoh_config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
            
            # 세션 생성
            self.zenoh_session = zenoh.open(zenoh_config)
            print("✅ Collision Detector Zenoh session created")
            return True
            
        except Exception as e:
            print(f"❌ Error setting up Zenoh for collision detector: {e}")
            self.zenoh_session = None
            self.enable_zenoh = False
            return False
    
    def publish_collision_alarm(self, source, location, actor_id=None, details=None):
        """Publish a collision alarm with location details via Zenoh"""
        try:
            if not self.enable_zenoh or not self.zenoh_session:
                return
            
            now = time.time()
            self._prune_collision_alarm_cache(now)

            location_dict = self._to_location_dict(location)
            actor_key = actor_id if actor_id is not None else f"{source}:{location_dict['x']:.1f}:{location_dict['y']:.1f}:{location_dict['z']:.1f}"
            last_sent = self._recent_collision_alarms.get(actor_key, 0.0)

            if now - last_sent < self.collision_alarm_cooldown:
                return

            self._recent_collision_alarms[actor_key] = now

            alarm_payload = {
                'source': source,
                'actor_id': actor_id,
                'location': location_dict,
                'detected_at': now,
                'chase_active': False,  # CollisionDetector는 추격 상태를 모름
                'details': details or {}
            }

            self.zenoh_session.put(self.collision_alarm_topic, json.dumps(alarm_payload))
            print(f"📡 Collision alarm ({source}) @ {location_dict}")

        except Exception as e:
            print(f"⚠️ Error publishing collision alarm: {e}")

    def publish_lock_on(self, lock_status=True):
        """Publish lock_on status via Zenoh"""
        try:
            if not self.enable_zenoh or not self.zenoh_session:
                return
            
            # lock_on 상태가 변경된 경우에만 전송
            if self._lock_on_sent == lock_status:
                return
            
            self._lock_on_sent = lock_status
            
            # boolean 값을 JSON으로 직렬화
            lock_payload = json.dumps(lock_status)
            
            self.zenoh_session.put(self.lock_on_topic, lock_payload)
            print(f"🎯 Lock on status: {lock_status}")

        except Exception as e:
            print(f"⚠️ Error publishing lock_on status: {e}")

    def _prune_collision_alarm_cache(self, now):
        """Remove stale collision alarm cache entries"""
        try:
            expired_keys = [
                key for key, ts in self._recent_collision_alarms.items()
                if now - ts > self._collision_alarm_ttl
            ]
            for key in expired_keys:
                self._recent_collision_alarms.pop(key, None)
        except Exception as e:
            print(f"⚠️ Error pruning collision alarm cache: {e}")

    def _to_location_dict(self, location):
        """Normalize location into a dict with x, y, z floats"""
        if isinstance(location, dict):
            if {'x', 'y', 'z'}.issubset(location.keys()):
                return {
                    'x': float(location['x']),
                    'y': float(location['y']),
                    'z': float(location['z'])
                }

        if isinstance(location, (tuple, list)) and len(location) >= 3:
            return {
                'x': float(location[0]),
                'y': float(location[1]),
                'z': float(location[2])
            }

        return {'x': 0.0, 'y': 0.0, 'z': 0.0}
    
    def detect_collisions(self, detected_objects: List[Dict]) -> List[Dict]:
        """충돌 감지 메서드 (호환성을 위한 래퍼)"""
        return self.analyze_pedestrian_collision(detected_objects)
    
    def analyze_pedestrian_collision(self, detections: List[Dict]) -> List[Dict]:
        """보행자 충돌 분석 - 박스 중심점이 지면과 가까워졌는지로 판단"""
        try:
            collision_events = []
            
            for detection in detections:
                if detection.get('type') != 'pedestrian':
                    continue
                
                actor_id = detection.get('actor_id')
                if not actor_id:
                    continue
                
                # 히스토리 업데이트
                self._update_pedestrian_history(actor_id, detection)
                
                # 충돌 분석 (지면 근접성 기반)
                collision_data = self._analyze_ground_proximity(actor_id)
                if collision_data:
                    # 넘어진 사람이 감지되면 회피 대상으로 표시
                    collision_data['avoid_pedestrian'] = True
                    collision_events.append(collision_data)
                    self.detected_collisions[actor_id] = collision_data
                    print(f"🚨 FALLEN PEDESTRIAN DETECTED - AVOID: Actor {actor_id}")
                    
                    # Zenoh 충돌 알람 퍼블리시
                    if self.enable_zenoh:
                        event_details = {
                            'type': 'pedestrian_fall',
                            'description': f"Pedestrian {actor_id} has fallen",
                            'severity': 'high',
                            'collision_score': collision_data.get('collision_score', 0.0),
                            'ground_distance': collision_data.get('ground_distance', 0.0)
                        }
                        
                        self.publish_collision_alarm(
                            source="collision_detector",
                            location=collision_data.get('world_location', [0, 0, 0]),
                            actor_id=str(actor_id),
                            details=event_details
                        )
                        
                        # Lock on 상태를 true로 전송
                        self.publish_lock_on(True)
            
            return collision_events
            
        except Exception as e:
            print(f"⚠️ Error analyzing pedestrian collision: {e}")
            return []
    
    def _update_pedestrian_history(self, actor_id: int, detection: Dict):
        """보행자 히스토리 업데이트"""
        try:
            if actor_id not in self.pedestrian_history:
                self.pedestrian_history[actor_id] = deque(maxlen=self.history_size)
            
            # 현재 프레임 데이터 추가
            current_data = {
                'timestamp': time.time(),
                'bbox_2d': detection.get('bbox_2d', {}),
                'world_location': detection.get('world_location', (0, 0, 0)),
                'distance': detection.get('distance', 0),
                'confidence': detection.get('confidence', 0)
            }
            
            self.pedestrian_history[actor_id].append(current_data)
            
        except Exception as e:
            print(f"⚠️ Error updating pedestrian history: {e}")
    
    def _analyze_ground_proximity(self, actor_id: int) -> Optional[Dict]:
        """지면 근접성 분석 - 박스 중심점이 지면과 가까워졌는지로 충돌 판단"""
        try:
            if actor_id not in self.pedestrian_history:
                print(f"🔍 Actor {actor_id}: No history yet")
                return None
            
            history = self.pedestrian_history[actor_id]
            if len(history) < self.collision_thresholds['min_history']:
                print(f"🔍 Actor {actor_id}: Not enough history ({len(history)} frames)")
                return None
            
            # 최근 데이터
            recent_data = history[-1]
            recent_bbox = recent_data.get('bbox_2d', {})
            recent_location = recent_data.get('world_location', (0, 0, 0))
            
            if not recent_bbox:
                print(f"🔍 Actor {actor_id}: Missing bbox data")
                return None
            
            # 박스 중심점의 Z 좌표 (높이)
            bbox_center_z = recent_location[2]
            
            # 지면 근접성 계산 (Z 좌표가 낮을수록 지면에 가까움)
            ground_distance = bbox_center_z
            
            print(f"🔍 Actor {actor_id} ground proximity analysis:")
            print(f"   Bbox center Z: {bbox_center_z:.2f}m")
            print(f"   Ground distance: {ground_distance:.2f}m")
            print(f"   Threshold: {self.collision_thresholds['ground_proximity']:.2f}m")
            
            # 충돌 감지: 박스 중심점이 지면에 가까워졌는지 확인
            if ground_distance <= self.collision_thresholds['ground_proximity']:
                collision_score = min(1.0, (self.collision_thresholds['ground_proximity'] - ground_distance) / self.collision_thresholds['ground_proximity'])
                
                print(f"🚨 COLLISION DETECTED! Actor {actor_id}: Ground proximity")
                print(f"   Ground distance: {ground_distance:.3f}m")
                print(f"   Collision Score: {collision_score:.2f}")
                
                return {
                    'actor_id': actor_id,
                    'collision_score': collision_score,
                    'timestamp': recent_data['timestamp'],
                    'ground_distance': ground_distance,
                    'world_location': recent_location,
                    'bbox_2d': recent_bbox,
                    'collision_indicators': [f"ground_proximity:{ground_distance:.2f}"]
                }
            else:
                print(f"🔍 Actor {actor_id}: Not close enough to ground")
                return None
            
        except Exception as e:
            print(f"⚠️ Error analyzing ground proximity: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    
    def get_collision_summary(self) -> Dict:
        """충돌 요약 정보"""
        try:
            # 충돌이 없고 lock_on이 true인 경우 false로 전송
            if len(self.detected_collisions) == 0 and self._lock_on_sent:
                self.publish_lock_on(False)
            
            return {
                'total_collisions': len(self.detected_collisions),
                'active_pedestrians': len(self.pedestrian_history),
                'collision_events': list(self.detected_collisions.values()),
                'lock_on_status': self._lock_on_sent
            }
            
        except Exception as e:
            return {'total_collisions': 0, 'active_pedestrians': 0, 'collision_events': [], 'lock_on_status': False}
    
    def clear_old_collisions(self, max_age_seconds=30):
        """오래된 충돌 데이터 정리"""
        try:
            current_time = time.time()
            to_remove = []
            
            for actor_id, collision_data in self.detected_collisions.items():
                if current_time - collision_data['timestamp'] > max_age_seconds:
                    to_remove.append(actor_id)
            
            for actor_id in to_remove:
                del self.detected_collisions[actor_id]
            
            # 충돌이 모두 정리되었고 lock_on이 true인 경우 false로 전송
            if len(self.detected_collisions) == 0 and self._lock_on_sent:
                self.publish_lock_on(False)
                
        except Exception as e:
            print(f"⚠️ Error clearing old collisions: {e}")
    
    def cleanup(self):
        """리소스 정리"""
        try:
            # cleanup 시 lock_on을 false로 전송
            if self._lock_on_sent:
                self.publish_lock_on(False)
            
            if self.zenoh_session:
                self.zenoh_session.close()
                self.zenoh_session = None
                print("✅ Collision Detector Zenoh session closed")
        except Exception as e:
            print(f"⚠️ Error during collision detector cleanup: {e}")
