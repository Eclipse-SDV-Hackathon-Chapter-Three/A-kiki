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
            'aspect_ratio_change': 0.3,   # 가로세로 비율 변화 임계값
            'size_change': 0.2,           # 크기 변화 임계값
            'height_drop': 0.3,           # 높이 하락 임계값
            'velocity_stop': 0.3,         # 속도 정지 임계값 (m/s)
            'min_history': 2,             # 최소 히스토리 길이
            'collision_score_threshold': 0.5  # 충돌 점수 임계값
        }
        self.detected_collisions = {}  # {actor_id: collision_data}
        
        # Zenoh 관련 변수
        self.enable_zenoh = enable_zenoh and ZENOH_AVAILABLE
        self.zenoh_session = None
        self.collision_alarm_topic = "carla/police/collision_alarm"
        self.collision_alarm_cooldown = 5.0  # 5초 쿨다운
        self._recent_collision_alarms = {}  # 중복 알람 방지
        self._collision_alarm_ttl = 60.0  # 60초 TTL
        
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
        """보행자 충돌 분석 - 넘어진 사람은 회피하고 차량을 쫓아가기"""
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
                
                # 충돌 분석 (넘어진 사람 감지)
                collision_data = self._analyze_collision_pattern(actor_id)
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
                            'collision_indicators': collision_data.get('collision_indicators', []),
                            'aspect_ratio': collision_data.get('aspect_ratio', 0.0),
                            'height_drop': collision_data.get('height_drop', 0.0)
                        }
                        
                        self.publish_collision_alarm(
                            source="collision_detector",
                            location=collision_data.get('world_location', [0, 0, 0]),
                            actor_id=str(actor_id),
                            details=event_details
                        )
            
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
    
    def _analyze_collision_pattern(self, actor_id: int) -> Optional[Dict]:
        """충돌 패턴 분석 - 보행자가 누워진 상태 감지"""
        try:
            if actor_id not in self.pedestrian_history:
                print(f"🔍 Actor {actor_id}: No history yet")
                return None
            
            history = self.pedestrian_history[actor_id]
            if len(history) < 3:  # 최소 3개 프레임 필요 (더 엄격하게)
                print(f"🔍 Actor {actor_id}: Not enough history ({len(history)} frames)")
                return None
            
            # 최근 2개 프레임 비교
            recent_data = history[-1]
            previous_data = history[-2]
            
            recent_bbox = recent_data.get('bbox_2d', {})
            previous_bbox = previous_data.get('bbox_2d', {})
            
            if not recent_bbox or not previous_bbox:
                print(f"🔍 Actor {actor_id}: Missing bbox data")
                return None
            
            # 가로/세로 비율 계산
            recent_width = recent_bbox.get('width', 0)
            recent_height = max(recent_bbox.get('height', 1), 1)
            previous_width = previous_bbox.get('width', 0)
            previous_height = max(previous_bbox.get('height', 1), 1)
            
            recent_ratio = recent_width / recent_height
            previous_ratio = previous_width / previous_height
            
            print(f"🔍 Actor {actor_id} analysis:")
            print(f"   Recent: W:{recent_width} H:{recent_height} Ratio:{recent_ratio:.2f}")
            print(f"   Previous: W:{previous_width} H:{previous_height} Ratio:{previous_ratio:.2f}")
            
            # 충돌 감지 조건들
            collision_indicators = []
            
            # 1. 가로세로 비율이 급격히 변함 (서서 있던 사람이 누워짐)
            ratio_change = abs(recent_ratio - previous_ratio)
            if ratio_change > self.collision_thresholds['aspect_ratio_change']:
                collision_indicators.append(f"ratio_change:{ratio_change:.2f}")
                print(f"   ✅ Ratio change detected: {ratio_change:.3f}")
            
            # 2. 현재 가로가 세로보다 1.2배 이상 큼 (누워진 상태)
            if recent_ratio >= 1.2:
                collision_indicators.append(f"lying_down:{recent_ratio:.2f}")
                print(f"   ✅ Lying down detected: {recent_ratio:.2f}")
            
            # 3. 높이가 급격히 감소함 (쓰러짐)
            recent_z = recent_data.get('world_location', (0, 0, 0))[2]
            previous_z = previous_data.get('world_location', (0, 0, 0))[2]
            height_drop = previous_z - recent_z
            if height_drop > self.collision_thresholds['height_drop']:
                collision_indicators.append(f"height_drop:{height_drop:.2f}")
                print(f"   ✅ Height drop detected: {height_drop:.3f}")
            
            # 4. 크기가 급격히 변함 (충돌로 인한 변형)
            recent_area = recent_width * recent_height
            previous_area = previous_width * previous_height
            size_change = abs(recent_area - previous_area) / max(previous_area, 1)
            if size_change > self.collision_thresholds['size_change']:
                collision_indicators.append(f"size_change:{size_change:.2f}")
                print(f"   ✅ Size change detected: {size_change:.3f}")
            
            # 충돌 감지 (2개 이상의 조건 만족)
            if len(collision_indicators) >= 2:
                collision_score = min(1.0, len(collision_indicators) / 4.0)
                
                print(f"🚨 COLLISION DETECTED! Actor {actor_id}: {', '.join(collision_indicators)}")
                print(f"   Collision Score: {collision_score:.2f}")
                
                return {
                    'actor_id': actor_id,
                    'collision_score': collision_score,
                    'timestamp': recent_data['timestamp'],
                    'aspect_ratio': recent_ratio,
                    'width': recent_width,
                    'height': recent_height,
                    'world_location': recent_data.get('world_location', (0, 0, 0)),
                    'bbox_2d': recent_bbox,
                    'collision_indicators': collision_indicators,
                    'height_drop': height_drop,
                    'ratio_change': ratio_change
                }
            else:
                print(f"🔍 Actor {actor_id}: No collision indicators met")
                return None
            
        except Exception as e:
            print(f"⚠️ Error analyzing collision pattern: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _analyze_bounding_box_changes(self, recent: Dict, older: Dict) -> Dict:
        """바운딩 박스 변화 분석"""
        try:
            recent_bbox = recent.get('bbox_2d', {})
            older_bbox = older.get('bbox_2d', {})
            
            if not recent_bbox or not older_bbox:
                return {'aspect_ratio_change': 0, 'size_change': 0, 'valid': False}
            
            # 가로세로 비율 계산 (누워있는 상태 감지)
            recent_width = recent_bbox.get('width', 1)
            recent_height = max(recent_bbox.get('height', 1), 1)
            older_width = older_bbox.get('width', 1)
            older_height = max(older_bbox.get('height', 1), 1)
            
            recent_ratio = recent_width / recent_height
            older_ratio = older_width / older_height
            
            # 누워있는 상태 감지 (가로가 세로보다 크면 누운 상태)
            recent_lying = recent_ratio > 1.2  # 가로가 세로보다 20% 이상 크면
            older_standing = older_ratio < 1.0  # 이전에는 서있었던 상태
            
            # 크기 변화 계산
            recent_area = recent_width * recent_height
            older_area = older_width * older_height
            
            aspect_ratio_change = abs(recent_ratio - older_ratio) / max(older_ratio, 0.1)
            size_change = abs(recent_area - older_area) / max(older_area, 1)
            
            # 누워있는 상태 변화 점수
            lying_score = 1.0 if (recent_lying and older_standing) else 0.0
            
            return {
                'aspect_ratio_change': aspect_ratio_change,
                'size_change': size_change,
                'recent_ratio': recent_ratio,
                'older_ratio': older_ratio,
                'lying_score': lying_score,
                'recent_lying': recent_lying,
                'older_standing': older_standing,
                'valid': True
            }
            
        except Exception as e:
            return {'aspect_ratio_change': 0, 'size_change': 0, 'valid': False}
    
    def _analyze_position_changes(self, recent: Dict, older: Dict) -> Dict:
        """위치 변화 분석"""
        try:
            recent_pos = recent.get('world_location', (0, 0, 0))
            older_pos = older.get('world_location', (0, 0, 0))
            
            # 높이 변화 (Z축)
            height_change = recent_pos[2] - older_pos[2]
            height_drop = max(0, -height_change)  # 하락만 고려
            
            # 수평 이동 거리
            horizontal_distance = math.sqrt(
                (recent_pos[0] - older_pos[0])**2 + 
                (recent_pos[1] - older_pos[1])**2
            )
            
            return {
                'height_change': height_change,
                'height_drop': height_drop,
                'horizontal_distance': horizontal_distance,
                'valid': True
            }
            
        except Exception as e:
            return {'height_change': 0, 'height_drop': 0, 'horizontal_distance': 0, 'valid': False}
    
    def _analyze_velocity_changes(self, history: deque) -> Dict:
        """속도 변화 분석"""
        try:
            if len(history) < 2:
                return {'velocity_change': 0, 'sudden_stop': False, 'valid': False}
            
            # 최근 2개 프레임의 속도 계산
            recent = history[-1]
            older = history[-2]
            
            recent_pos = recent.get('world_location', (0, 0, 0))
            older_pos = older.get('world_location', (0, 0, 0))
            
            time_diff = recent.get('timestamp', 0) - older.get('timestamp', 0)
            if time_diff <= 0:
                return {'velocity_change': 0, 'sudden_stop': False, 'valid': False}
            
            # 속도 계산 (m/s)
            distance = math.sqrt(
                (recent_pos[0] - older_pos[0])**2 + 
                (recent_pos[1] - older_pos[1])**2 + 
                (recent_pos[2] - older_pos[2])**2
            )
            velocity = distance / time_diff
            
            # 갑작스러운 정지 감지
            sudden_stop = velocity < self.collision_thresholds['velocity_stop']
            
            return {
                'velocity': velocity,
                'sudden_stop': sudden_stop,
                'valid': True
            }
            
        except Exception as e:
            return {'velocity': 0, 'sudden_stop': False, 'valid': False}
    
    def _calculate_collision_score(self, bbox_analysis: Dict, 
                                 position_analysis: Dict, 
                                 velocity_analysis: Dict) -> float:
        """충돌 점수 계산"""
        try:
            score = 0.0
            weight_sum = 0.0
            
            # 바운딩 박스 변화 점수
            if bbox_analysis.get('valid', False):
                aspect_ratio_score = min(1.0, 
                    bbox_analysis['aspect_ratio_change'] / self.collision_thresholds['aspect_ratio_change'])
                size_change_score = min(1.0, 
                    bbox_analysis['size_change'] / self.collision_thresholds['size_change'])
                
                lying_score = bbox_analysis.get('lying_score', 0.0)  # 누워있는 상태 점수
                
                score += aspect_ratio_score * 0.2
                score += size_change_score * 0.1
                score += lying_score * 0.2  # 누워있는 상태에 높은 가중치
                weight_sum += 0.5
            
            # 위치 변화 점수
            if position_analysis.get('valid', False):
                height_drop_score = min(1.0, 
                    position_analysis['height_drop'] / self.collision_thresholds['height_drop'])
                
                score += height_drop_score * 0.3
                weight_sum += 0.3
            
            # 속도 변화 점수
            if velocity_analysis.get('valid', False):
                if velocity_analysis['sudden_stop']:
                    score += 0.2
                weight_sum += 0.2
            
            # 정규화
            if weight_sum > 0:
                score = score / weight_sum
            
            return min(1.0, score)
            
        except Exception as e:
            return 0.0
    
    def get_collision_summary(self) -> Dict:
        """충돌 요약 정보"""
        try:
            return {
                'total_collisions': len(self.detected_collisions),
                'active_pedestrians': len(self.pedestrian_history),
                'collision_events': list(self.detected_collisions.values())
            }
            
        except Exception as e:
            return {'total_collisions': 0, 'active_pedestrians': 0, 'collision_events': []}
    
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
                
        except Exception as e:
            print(f"⚠️ Error clearing old collisions: {e}")
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.zenoh_session:
                self.zenoh_session.close()
                self.zenoh_session = None
                print("✅ Collision Detector Zenoh session closed")
        except Exception as e:
            print(f"⚠️ Error during collision detector cleanup: {e}")
