#!/usr/bin/env python3
"""
Semantic LIDAR Manager
CARLA Semantic LIDAR 센서 관리 클래스
"""

import carla
import numpy as np
import time
import json
from typing import List, Dict, Any, Optional, Callable
import logging

try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    zenoh = None

class SemanticLidarManager:
    """Semantic LIDAR 센서 관리자"""
    
    def __init__(self, world: carla.World, vehicle: carla.Vector3D = None):
        self.world = world
        self.vehicle = vehicle
        self.semantic_lidar = None
        self.zenoh_session = None
        self.publishers = {}
        
        # 콜백 함수
        self.data_callback: Optional[Callable] = None
        
        # 차량 추적
        self.tracked_vehicles: Dict[int, Dict[str, Any]] = {}  # obj_idx -> vehicle_info
        self.vehicle_tracking_enabled = True
        self.vehicle_tracking_distance_threshold = 50.0  # 50m 이내 차량만 추적
        
        # 사고 감지 (보행자 포인트 클라우드 기반)
        self.accident_detection_enabled = True
        self.pedestrian_ground_threshold = 0.5  # Z 좌표가 0.5m 이하면 사고로 판단
        self.pedestrian_min_points = 5  # 최소 포인트 수
        self.detected_accidents: Dict[int, Dict[str, Any]] = {}  # obj_idx -> accident_info
        self.accident_cooldown = 5.0  # 같은 사고에 대한 중복 알림 방지 (초)
        
        # 사고 차량 추적
        self.accident_vehicle_tracking_enabled = True
        self.accident_vehicle_distance_threshold = 30.0  # 사고 현장 30m 이내 차량 추적
        self.accident_vehicles: Dict[int, Dict[str, Any]] = {}  # 사고 차량 정보
        self.primary_accident_vehicle = None  # 주요 사고 차량 (추적 대상)
        
        # 로깅
        self.logger = logging.getLogger(__name__)
        
        # 시맨틱 라벨 매핑
        self.semantic_labels = {
            0: "None",
            1: "Buildings", 
            2: "Fences",
            3: "Other",
            4: "Pedestrians",
            5: "Poles",
            6: "RoadLines",
            7: "Roads",
            8: "Sidewalks",
            9: "Vegetation",
            10: "Vehicles",
            11: "Walls",
            12: "TrafficSigns",
            13: "Sky",
            14: "Ground",
            15: "Bridge",
            16: "RailTrack",
            17: "GuardRail",
            18: "TrafficLight",
            19: "Static",
            20: "Dynamic",
            21: "Water",
            22: "Terrain"
        }
    
    def setup_zenoh(self) -> bool:
        """Zenoh 설정"""
        try:
            if not ZENOH_AVAILABLE:
                self.logger.warning("⚠️ Zenoh not available")
                return False
            
            # Zenoh 설정
            zenoh_config = zenoh.Config()
            zenoh_config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
            
            # 세션 생성
            self.zenoh_session = zenoh.open(zenoh_config)
            
            # 여러 토픽에 대한 퍼블리셔 설정
            self.publishers = {
                'raw_data': self.zenoh_session.declare_publisher("carla/semantic_lidar/raw"),
                'processed_data': self.zenoh_session.declare_publisher("carla/semantic_lidar/processed"),
                'vehicle_tracking': self.zenoh_session.declare_publisher("carla/semantic_lidar/vehicles"),
                'accident_alerts': self.zenoh_session.declare_publisher("carla/semantic_lidar/accidents"),
                'summary': self.zenoh_session.declare_publisher("carla/semantic_lidar/summary")
            }
            
            self.logger.info("✅ Semantic LIDAR Zenoh setup successful")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Failed to setup Zenoh: {e}")
            return False
    
    def setup_semantic_lidar(self, 
                           location: carla.Location = None,
                           rotation: carla.Rotation = None,
                           channels: int = 32,
                           range_meters: float = 100.0,
                           points_per_second: int = 100000,
                           rotation_frequency: float = 10.0,
                           upper_fov: float = 15.0,
                           lower_fov: float = -25.0) -> bool:
        """Semantic LIDAR 센서 설정"""
        try:
            if not self.vehicle:
                self.logger.error("❌ No vehicle available for semantic LIDAR setup")
                return False
            
            # Blueprint 가져오기
            blueprint_library = self.world.get_blueprint_library()
            semantic_lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')
            
            if not semantic_lidar_bp:
                self.logger.error("❌ Semantic LIDAR blueprint not found")
                return False
            
            # 기본 위치 설정
            if location is None:
                location = carla.Location(x=0.0, y=0.0, z=2.5)
            if rotation is None:
                rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
            
            # 센서 속성 설정
            semantic_lidar_bp.set_attribute('channels', str(channels))
            semantic_lidar_bp.set_attribute('range', str(range_meters))
            semantic_lidar_bp.set_attribute('points_per_second', str(points_per_second))
            semantic_lidar_bp.set_attribute('rotation_frequency', str(rotation_frequency))
            semantic_lidar_bp.set_attribute('upper_fov', str(upper_fov))
            semantic_lidar_bp.set_attribute('lower_fov', str(lower_fov))
            
            # 센서 생성
            transform = carla.Transform(location, rotation)
            self.semantic_lidar = self.world.spawn_actor(semantic_lidar_bp, transform, attach_to=self.vehicle)
            
            # 콜백 함수 설정
            self.semantic_lidar.listen(self._on_semantic_lidar_data)
            
            self.logger.info(f"✅ Semantic LIDAR sensor attached to vehicle")
            self.logger.info(f"   Channels: {channels}, Range: {range_meters}m, FPS: {rotation_frequency}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Failed to setup semantic LIDAR: {e}")
            return False
    
    def _on_semantic_lidar_data(self, data: carla.SemanticLidarMeasurement):
        """Semantic LIDAR 데이터 수신 처리"""
        try:
            # 포인트 클라우드 데이터 추출
            points = np.frombuffer(data.raw_data, dtype=np.dtype([
                ('x', np.float32), ('y', np.float32), ('z', np.float32),
                ('cos_angle', np.float32), ('obj_idx', np.uint32), ('obj_tag', np.uint32)
            ]))
            
            # 포인트 클라우드 처리
            point_cloud = self._process_point_cloud(points)
            
            # 객체별 그룹화
            objects = self._group_objects_by_semantic(point_cloud)
            
            # 차량 추적 (사고 차량 쫓아가기용)
            if self.vehicle_tracking_enabled:
                self._update_vehicle_tracking(point_cloud)
            
            # 사고 감지 (보행자 포인트 클라우드 기반)
            if self.accident_detection_enabled:
                self._detect_pedestrian_accidents(point_cloud)
            
            # 사고 차량 추적 (사고 발생 시)
            if self.accident_vehicle_tracking_enabled and self.detected_accidents:
                self._track_accident_vehicles(point_cloud)
            
            # Zenoh로 퍼블리시
            if self.publishers:
                self._publish_lidar_data(point_cloud, objects, data)
            
            # 콜백 호출
            if self.data_callback:
                self.data_callback(point_cloud, objects)
            
            self.logger.debug(f"📡 Processed {len(points)} semantic LIDAR points, {len(objects)} objects")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error processing semantic LIDAR data: {e}")
    
    def _process_point_cloud(self, points: np.ndarray) -> List[Dict[str, Any]]:
        """포인트 클라우드 데이터 처리"""
        processed_points = []
        
        for point in points:
            # 3D 좌표
            x, y, z = point['x'], point['y'], point['z']
            
            # 거리 계산
            distance = np.sqrt(x**2 + y**2 + z**2)
            
            # 시맨틱 정보
            obj_tag = int(point['obj_tag'])
            obj_idx = int(point['obj_idx'])
            semantic_label = self.semantic_labels.get(obj_tag, f"Unknown_{obj_tag}")
            
            processed_points.append({
                'x': float(x),
                'y': float(y), 
                'z': float(z),
                'distance': float(distance),
                'semantic_label': semantic_label,
                'object_id': obj_idx,
                'semantic_id': obj_tag
            })
        
        return processed_points
    
    def _group_objects_by_semantic(self, point_cloud: List[Dict[str, Any]]) -> Dict[str, List[Dict[str, Any]]]:
        """시맨틱 라벨별로 객체 그룹화"""
        objects = {}
        
        for point in point_cloud:
            label = point['semantic_label']
            if label not in objects:
                objects[label] = []
            objects[label].append(point)
        
        return objects
    
    def _update_vehicle_tracking(self, point_cloud: List[Dict[str, Any]]):
        """차량 추적 업데이트 (사고 차량 쫓아가기용)"""
        try:
            # 차량 포인트만 필터링 (semantic_id = 10)
            vehicle_points = [p for p in point_cloud if p['semantic_id'] == 10]
            
            if not vehicle_points:
                return
            
            # 객체 ID별로 그룹화
            vehicles_by_id = {}
            for point in vehicle_points:
                obj_id = point['object_id']
                if obj_id not in vehicles_by_id:
                    vehicles_by_id[obj_id] = []
                vehicles_by_id[obj_id].append(point)
            
            # 각 차량 정보 업데이트
            for obj_id, points in vehicles_by_id.items():
                if len(points) < 10:  # 포인트가 너무 적으면 스킵
                    continue
                
                # 차량 중심점 계산
                center = self._calculate_center(points)
                distance = np.sqrt(center['x']**2 + center['y']**2 + center['z']**2)
                
                # 거리 필터링
                if distance > self.vehicle_tracking_distance_threshold:
                    continue
                
                # 바운딩 박스 계산
                bbox = self._calculate_bounding_box(points)
                
                # 속도 계산 (이전 프레임과 비교)
                velocity = self._calculate_velocity(obj_id, center)
                
                # 차량 정보 업데이트
                vehicle_info = {
                    'object_id': obj_id,
                    'center': center,
                    'distance': distance,
                    'bounding_box': bbox,
                    'velocity': velocity,
                    'point_count': len(points),
                    'last_seen': time.time(),
                    'is_tracked': True
                }
                
                self.tracked_vehicles[obj_id] = vehicle_info
                
                self.logger.debug(f"🚗 Vehicle {obj_id}: dist={distance:.1f}m, points={len(points)}, vel={velocity}")
            
            # 오래된 추적 정보 정리
            self._cleanup_old_tracks()
            
        except Exception as e:
            self.logger.error(f"⚠️ Error updating vehicle tracking: {e}")
    
    def _calculate_velocity(self, obj_id: int, current_center: Dict[str, float]) -> Dict[str, float]:
        """차량 속도 계산 (이전 프레임과 비교)"""
        try:
            if obj_id not in self.tracked_vehicles:
                return {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
            prev_center = self.tracked_vehicles[obj_id].get('center', current_center)
            prev_time = self.tracked_vehicles[obj_id].get('last_seen', time.time())
            current_time = time.time()
            
            dt = current_time - prev_time
            if dt <= 0:
                return {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
            velocity = {
                'x': (current_center['x'] - prev_center['x']) / dt,
                'y': (current_center['y'] - prev_center['y']) / dt,
                'z': (current_center['z'] - prev_center['z']) / dt
            }
            
            return velocity
            
        except Exception as e:
            self.logger.error(f"⚠️ Error calculating velocity: {e}")
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}
    
    def _cleanup_old_tracks(self):
        """오래된 추적 정보 정리"""
        try:
            current_time = time.time()
            expired_tracks = []
            
            for obj_id, vehicle_info in self.tracked_vehicles.items():
                if current_time - vehicle_info.get('last_seen', 0) > 2.0:  # 2초 이상 안 보이면 제거
                    expired_tracks.append(obj_id)
            
            for obj_id in expired_tracks:
                del self.tracked_vehicles[obj_id]
                self.logger.debug(f"🗑️ Removed expired track: {obj_id}")
                
        except Exception as e:
            self.logger.error(f"⚠️ Error cleaning up old tracks: {e}")
    
    def get_tracked_vehicles(self) -> Dict[int, Dict[str, Any]]:
        """추적 중인 차량 정보 반환"""
        return self.tracked_vehicles.copy()
    
    def get_closest_vehicle(self) -> Optional[Dict[str, Any]]:
        """가장 가까운 차량 반환 (사고 차량 쫓아가기용)"""
        try:
            if not self.tracked_vehicles:
                return None
            
            closest_vehicle = None
            min_distance = float('inf')
            
            for obj_id, vehicle_info in self.tracked_vehicles.items():
                distance = vehicle_info.get('distance', float('inf'))
                if distance < min_distance:
                    min_distance = distance
                    closest_vehicle = vehicle_info
            
            return closest_vehicle
            
        except Exception as e:
            self.logger.error(f"⚠️ Error getting closest vehicle: {e}")
            return None
    
    def get_vehicles_by_distance(self, max_distance: float = 30.0) -> List[Dict[str, Any]]:
        """거리별 차량 필터링"""
        try:
            filtered_vehicles = []
            for obj_id, vehicle_info in self.tracked_vehicles.items():
                if vehicle_info.get('distance', float('inf')) <= max_distance:
                    filtered_vehicles.append(vehicle_info)
            
            # 거리순 정렬
            filtered_vehicles.sort(key=lambda v: v.get('distance', float('inf')))
            return filtered_vehicles
            
        except Exception as e:
            self.logger.error(f"⚠️ Error filtering vehicles by distance: {e}")
            return []
    
    def _detect_pedestrian_accidents(self, point_cloud: List[Dict[str, Any]]):
        """보행자 포인트 클라우드 기반 사고 감지"""
        try:
            # 보행자 포인트만 필터링 (semantic_id = 4)
            pedestrian_points = [p for p in point_cloud if p['semantic_id'] == 4]
            
            if not pedestrian_points:
                return
            
            # 객체 ID별로 그룹화
            pedestrians_by_id = {}
            for point in pedestrian_points:
                obj_id = point['object_id']
                if obj_id not in pedestrians_by_id:
                    pedestrians_by_id[obj_id] = []
                pedestrians_by_id[obj_id].append(point)
            
            # 각 보행자에 대해 사고 감지
            for obj_id, points in pedestrians_by_id.items():
                if len(points) < self.pedestrian_min_points:
                    continue
                
                # 보행자 중심점 계산
                center = self._calculate_center(points)
                z_coord = center.get('z', 0)
                
                # Z 좌표가 바닥에 가까우면 사고로 판단
                if z_coord <= self.pedestrian_ground_threshold:
                    self._handle_pedestrian_accident(obj_id, center, points)
                else:
                    # 정상 상태면 사고 정보에서 제거
                    if obj_id in self.detected_accidents:
                        del self.detected_accidents[obj_id]
            
        except Exception as e:
            self.logger.error(f"⚠️ Error detecting pedestrian accidents: {e}")
    
    def _handle_pedestrian_accident(self, obj_id: int, center: Dict[str, float], points: List[Dict[str, Any]]):
        """보행자 사고 처리"""
        try:
            current_time = time.time()
            
            # 중복 알림 방지 (쿨다운 체크)
            if obj_id in self.detected_accidents:
                last_detected = self.detected_accidents[obj_id].get('last_detected', 0)
                if current_time - last_detected < self.accident_cooldown:
                    return
            
            # 바운딩 박스 계산
            bbox = self._calculate_bounding_box(points)
            
            # 사고 정보 업데이트
            accident_info = {
                'object_id': obj_id,
                'center': center,
                'bounding_box': bbox,
                'point_count': len(points),
                'first_detected': self.detected_accidents.get(obj_id, {}).get('first_detected', current_time),
                'last_detected': current_time,
                'accident_type': 'pedestrian_fallen',
                'severity': self._calculate_accident_severity(center, bbox, points)
            }
            
            self.detected_accidents[obj_id] = accident_info
            
            # 사고 알림 퍼블리시
            self._publish_accident_alert(accident_info)
            
            self.logger.warning(f"🚨 PEDESTRIAN ACCIDENT DETECTED! ID: {obj_id}, Z: {center.get('z', 0):.2f}m, Points: {len(points)}")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error handling pedestrian accident: {e}")
    
    def _calculate_accident_severity(self, center: Dict[str, float], bbox: Dict[str, float], points: List[Dict[str, Any]]) -> str:
        """사고 심각도 계산"""
        try:
            z_coord = center.get('z', 0)
            point_count = len(points)
            
            # Z 좌표가 낮을수록, 포인트가 많을수록 심각
            if z_coord <= 0.2 and point_count >= 20:
                return "critical"
            elif z_coord <= 0.3 and point_count >= 15:
                return "severe"
            elif z_coord <= 0.4 and point_count >= 10:
                return "moderate"
            else:
                return "minor"
                
        except Exception as e:
            self.logger.error(f"⚠️ Error calculating accident severity: {e}")
            return "unknown"
    
    def _publish_accident_alert(self, accident_info: Dict[str, Any]):
        """사고 알림을 Zenoh로 퍼블리시"""
        try:
            if not self.publishers:
                return
            
            # 사고 알림 데이터
            alert_data = {
                'timestamp': time.time(),
                'accident_type': accident_info.get('accident_type', 'unknown'),
                'object_id': accident_info.get('object_id', 0),
                'location': accident_info.get('center', {}),
                'severity': accident_info.get('severity', 'unknown'),
                'point_count': accident_info.get('point_count', 0),
                'bounding_box': accident_info.get('bounding_box', {}),
                'detection_method': 'semantic_lidar_point_cloud',
                'source': 'semantic_lidar_manager'
            }
            
            # JSON으로 직렬화
            import json
            json_data = json.dumps(alert_data)
            
            # Zenoh로 퍼블리시
            self.publishers['accident_alerts'].put(json_data)
            
            self.logger.info(f"📡 Published accident alert for pedestrian {accident_info.get('object_id', 0)}")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error publishing accident alert: {e}")
    
    def get_detected_accidents(self) -> Dict[int, Dict[str, Any]]:
        """감지된 사고 정보 반환"""
        return self.detected_accidents.copy()
    
    def get_active_accidents(self) -> List[Dict[str, Any]]:
        """활성 사고 목록 반환 (최근 10초 이내)"""
        try:
            current_time = time.time()
            active_accidents = []
            
            for obj_id, accident_info in self.detected_accidents.items():
                last_detected = accident_info.get('last_detected', 0)
                if current_time - last_detected <= 10.0:  # 10초 이내
                    active_accidents.append(accident_info)
            
            return active_accidents
            
        except Exception as e:
            self.logger.error(f"⚠️ Error getting active accidents: {e}")
            return []
    
    def clear_accident_history(self):
        """사고 기록 초기화"""
        self.detected_accidents.clear()
        self.accident_vehicles.clear()
        self.primary_accident_vehicle = None
        self.logger.info("🧹 Accident history cleared")
    
    def _track_accident_vehicles(self, point_cloud: List[Dict[str, Any]]):
        """사고 차량 추적 (사고 발생 시 주변 차량들 추적)"""
        try:
            if not self.detected_accidents:
                return
            
            # 사고 현장 위치 계산 (보행자 사고들의 평균 위치)
            accident_locations = []
            for accident in self.detected_accidents.values():
                center = accident.get('center', {})
                if center:
                    accident_locations.append(center)
            
            if not accident_locations:
                return
            
            # 사고 현장 중심점 계산
            accident_center = self._calculate_accident_scene_center(accident_locations)
            
            # 차량 포인트만 필터링 (semantic_id = 10)
            vehicle_points = [p for p in point_cloud if p['semantic_id'] == 10]
            
            if not vehicle_points:
                return
            
            # 객체 ID별로 그룹화
            vehicles_by_id = {}
            for point in vehicle_points:
                obj_id = point['object_id']
                if obj_id not in vehicles_by_id:
                    vehicles_by_id[obj_id] = []
                vehicles_by_id[obj_id].append(point)
            
            # 각 차량에 대해 사고 현장과의 거리 계산
            for obj_id, points in vehicles_by_id.items():
                if len(points) < 10:  # 포인트가 너무 적으면 스킵
                    continue
                
                # 차량 중심점 계산
                center = self._calculate_center(points)
                distance_to_accident = self._calculate_distance_to_accident(center, accident_center)
                
                # 사고 현장 근처 차량만 추적
                if distance_to_accident <= self.accident_vehicle_distance_threshold:
                    self._update_accident_vehicle_info(obj_id, center, points, distance_to_accident)
            
            # 주요 사고 차량 선정 (가장 가까운 차량)
            self._select_primary_accident_vehicle()
            
        except Exception as e:
            self.logger.error(f"⚠️ Error tracking accident vehicles: {e}")
    
    def _calculate_accident_scene_center(self, accident_locations: List[Dict[str, Any]]) -> Dict[str, float]:
        """사고 현장 중심점 계산"""
        try:
            if not accident_locations:
                return {'x': 0, 'y': 0, 'z': 0}
            
            x_coords = [loc.get('x', 0) for loc in accident_locations]
            y_coords = [loc.get('y', 0) for loc in accident_locations]
            z_coords = [loc.get('z', 0) for loc in accident_locations]
            
            return {
                'x': float(np.mean(x_coords)),
                'y': float(np.mean(y_coords)),
                'z': float(np.mean(z_coords))
            }
            
        except Exception as e:
            self.logger.error(f"⚠️ Error calculating accident scene center: {e}")
            return {'x': 0, 'y': 0, 'z': 0}
    
    def _calculate_distance_to_accident(self, vehicle_center: Dict[str, float], 
                                       accident_center: Dict[str, float]) -> float:
        """차량과 사고 현장 간의 거리 계산"""
        try:
            dx = vehicle_center.get('x', 0) - accident_center.get('x', 0)
            dy = vehicle_center.get('y', 0) - accident_center.get('y', 0)
            dz = vehicle_center.get('z', 0) - accident_center.get('z', 0)
            
            return float(np.sqrt(dx**2 + dy**2 + dz**2))
            
        except Exception as e:
            self.logger.error(f"⚠️ Error calculating distance to accident: {e}")
            return float('inf')
    
    def _update_accident_vehicle_info(self, obj_id: int, center: Dict[str, float], 
                                    points: List[Dict[str, Any]], distance_to_accident: float):
        """사고 차량 정보 업데이트"""
        try:
            # 바운딩 박스 계산
            bbox = self._calculate_bounding_box(points)
            
            # 속도 계산
            velocity = self._calculate_velocity(obj_id, center)
            
            # 사고 차량 정보 업데이트
            accident_vehicle_info = {
                'object_id': obj_id,
                'center': center,
                'distance_to_accident': distance_to_accident,
                'bounding_box': bbox,
                'velocity': velocity,
                'point_count': len(points),
                'last_seen': time.time(),
                'is_accident_vehicle': True,
                'priority': self._calculate_vehicle_priority(center, velocity, distance_to_accident)
            }
            
            self.accident_vehicles[obj_id] = accident_vehicle_info
            
            self.logger.debug(f"🚗 Accident vehicle {obj_id}: dist={distance_to_accident:.1f}m, priority={accident_vehicle_info['priority']}")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error updating accident vehicle info: {e}")
    
    def _calculate_vehicle_priority(self, center: Dict[str, float], velocity: Dict[str, float], 
                                  distance_to_accident: float) -> float:
        """차량 우선순위 계산 (높을수록 추적 우선)"""
        try:
            # 거리가 가까울수록 높은 우선순위
            distance_score = max(0, self.accident_vehicle_distance_threshold - distance_to_accident)
            
            # 속도가 빠를수록 높은 우선순위 (도주 차량일 가능성)
            speed = np.sqrt(velocity.get('x', 0)**2 + velocity.get('y', 0)**2 + velocity.get('z', 0)**2)
            speed_score = min(speed * 10, 50)  # 최대 50점
            
            # Z 좌표가 높을수록 높은 우선순위 (차량일 가능성)
            height_score = max(0, center.get('z', 0) * 20)  # 최대 20점
            
            total_priority = distance_score + speed_score + height_score
            return float(total_priority)
            
        except Exception as e:
            self.logger.error(f"⚠️ Error calculating vehicle priority: {e}")
            return 0.0
    
    def _select_primary_accident_vehicle(self):
        """주요 사고 차량 선정 (가장 높은 우선순위)"""
        try:
            if not self.accident_vehicles:
                self.primary_accident_vehicle = None
                return
            
            # 우선순위가 가장 높은 차량 선택
            best_vehicle = None
            best_priority = -1
            
            for obj_id, vehicle_info in self.accident_vehicles.items():
                priority = vehicle_info.get('priority', 0)
                if priority > best_priority:
                    best_priority = priority
                    best_vehicle = vehicle_info
            
            self.primary_accident_vehicle = best_vehicle
            
            if best_vehicle:
                obj_id = best_vehicle.get('object_id', 0)
                priority = best_vehicle.get('priority', 0)
                self.logger.info(f"🎯 Primary accident vehicle selected: {obj_id} (priority: {priority:.1f})")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error selecting primary accident vehicle: {e}")
    
    def get_primary_accident_vehicle(self) -> Optional[Dict[str, Any]]:
        """주요 사고 차량 정보 반환"""
        return self.primary_accident_vehicle
    
    def get_accident_vehicles(self) -> Dict[int, Dict[str, Any]]:
        """모든 사고 차량 정보 반환"""
        return self.accident_vehicles.copy()
    
    def get_accident_vehicles_by_priority(self) -> List[Dict[str, Any]]:
        """우선순위순으로 정렬된 사고 차량 목록 반환"""
        try:
            vehicles = list(self.accident_vehicles.values())
            vehicles.sort(key=lambda v: v.get('priority', 0), reverse=True)
            return vehicles
        except Exception as e:
            self.logger.error(f"⚠️ Error getting accident vehicles by priority: {e}")
            return []
    
    def _publish_lidar_data(self, point_cloud: List[Dict[str, Any]], 
                           objects: Dict[str, List[Dict[str, Any]]], 
                           raw_data: carla.SemanticLidarMeasurement):
        """LIDAR 데이터를 여러 Zenoh 토픽으로 퍼블리시"""
        try:
            if not self.publishers:
                return
            
            current_time = time.time()
            
            # 1. 원시 데이터 퍼블리시 (압축된 형태)
            self._publish_raw_data(raw_data, current_time)
            
            # 2. 처리된 데이터 퍼블리시
            self._publish_processed_data(point_cloud, objects, current_time)
            
            # 3. 차량 추적 데이터 퍼블리시
            self._publish_vehicle_tracking_data(current_time)
            
            # 4. 사고 알림 데이터 퍼블리시
            self._publish_accident_data(current_time)
            
            # 5. 요약 데이터 퍼블리시
            self._publish_summary_data(point_cloud, objects, current_time)
            
        except Exception as e:
            self.logger.error(f"⚠️ Error publishing LIDAR data: {e}")
    
    def _publish_raw_data(self, raw_data: carla.SemanticLidarMeasurement, timestamp: float):
        """원시 LIDAR 데이터 퍼블리시"""
        try:
            # 원시 데이터를 압축된 형태로 퍼블리시
            raw_info = {
                'timestamp': timestamp,
                'frame': raw_data.frame,
                'simulation_time': raw_data.timestamp,
                'point_count': len(raw_data.raw_data) // 24,  # 24 bytes per point
                'data_size_bytes': len(raw_data.raw_data)
            }
            
            import json
            json_data = json.dumps(raw_info)
            self.publishers['raw_data'].put(json_data)
            
        except Exception as e:
            self.logger.error(f"⚠️ Error publishing raw data: {e}")
    
    def _publish_processed_data(self, point_cloud: List[Dict[str, Any]], 
                               objects: Dict[str, List[Dict[str, Any]]], 
                               timestamp: float):
        """처리된 LIDAR 데이터 퍼블리시"""
        try:
            # 처리된 데이터 (요약된 형태)
            processed_data = {
                'timestamp': timestamp,
                'total_points': len(point_cloud),
                'object_types': list(objects.keys()),
                'object_counts': {label: len(points) for label, points in objects.items()},
                'point_cloud_sample': point_cloud[:100] if len(point_cloud) > 100 else point_cloud  # 샘플만 전송
            }
            
            import json
            json_data = json.dumps(processed_data)
            self.publishers['processed_data'].put(json_data)
            
        except Exception as e:
            self.logger.error(f"⚠️ Error publishing processed data: {e}")
    
    def _publish_vehicle_tracking_data(self, timestamp: float):
        """차량 추적 데이터 퍼블리시"""
        try:
            vehicle_data = {
                'timestamp': timestamp,
                'tracked_vehicles': list(self.tracked_vehicles.values()),
                'vehicle_count': len(self.tracked_vehicles),
                'closest_vehicle': self.get_closest_vehicle()
            }
            
            import json
            json_data = json.dumps(vehicle_data)
            self.publishers['vehicle_tracking'].put(json_data)
            
        except Exception as e:
            self.logger.error(f"⚠️ Error publishing vehicle tracking data: {e}")
    
    def _publish_accident_data(self, timestamp: float):
        """사고 데이터 퍼블리시"""
        try:
            active_accidents = self.get_active_accidents()
            
            accident_data = {
                'timestamp': timestamp,
                'active_accidents': active_accidents,
                'accident_count': len(active_accidents),
                'total_detected': len(self.detected_accidents)
            }
            
            import json
            json_data = json.dumps(accident_data)
            self.publishers['accident_alerts'].put(json_data)
            
        except Exception as e:
            self.logger.error(f"⚠️ Error publishing accident data: {e}")
    
    def _publish_summary_data(self, point_cloud: List[Dict[str, Any]], 
                             objects: Dict[str, List[Dict[str, Any]]], 
                             timestamp: float):
        """요약 데이터 퍼블리시"""
        try:
            summary_data = {
                'timestamp': timestamp,
                'sensor_status': {
                    'vehicle_tracking_enabled': self.vehicle_tracking_enabled,
                    'accident_detection_enabled': self.accident_detection_enabled,
                    'tracked_vehicles': len(self.tracked_vehicles),
                    'active_accidents': len(self.get_active_accidents())
                },
                'data_summary': {
                    'total_points': len(point_cloud),
                    'object_types': list(objects.keys()),
                    'point_distribution': {label: len(points) for label, points in objects.items()}
                }
            }
            
            import json
            json_data = json.dumps(summary_data)
            self.publishers['summary'].put(json_data)
            
        except Exception as e:
            self.logger.error(f"⚠️ Error publishing summary data: {e}")
    
    def _publish_semantic_data(self, point_cloud: List[Dict[str, Any]], 
                              objects: Dict[str, List[Dict[str, Any]]], 
                              raw_data: carla.SemanticLidarMeasurement):
        """Zenoh로 시맨틱 데이터 퍼블리시"""
        try:
            # 메타데이터
            metadata = {
                'timestamp': time.time(),
                'frame': raw_data.frame,
                'simulation_time': raw_data.timestamp,
                'total_points': len(point_cloud),
                'object_types': list(objects.keys()),
                'object_counts': {label: len(points) for label, points in objects.items()}
            }
            
            # 요약 데이터 (전체 포인트 클라우드는 너무 큼)
            summary_data = {
                'metadata': metadata,
                'objects_summary': {
                    label: {
                        'count': len(points),
                        'center': self._calculate_center(points),
                        'bounding_box': self._calculate_bounding_box(points)
                    } for label, points in objects.items()
                }
            }
            
            # JSON으로 직렬화
            json_data = json.dumps(summary_data)
            
            # Zenoh로 퍼블리시
            self.publisher.put(json_data)
            
            self.logger.debug(f"📡 Published semantic LIDAR data: {len(point_cloud)} points, {len(objects)} object types")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error publishing semantic LIDAR data: {e}")
    
    def _calculate_center(self, points: List[Dict[str, Any]]) -> Dict[str, float]:
        """포인트들의 중심점 계산"""
        if not points:
            return {'x': 0, 'y': 0, 'z': 0}
        
        x_coords = [p['x'] for p in points]
        y_coords = [p['y'] for p in points]
        z_coords = [p['z'] for p in points]
        
        return {
            'x': float(np.mean(x_coords)),
            'y': float(np.mean(y_coords)),
            'z': float(np.mean(z_coords))
        }
    
    def _calculate_bounding_box(self, points: List[Dict[str, Any]]) -> Dict[str, float]:
        """포인트들의 바운딩 박스 계산"""
        if not points:
            return {'min_x': 0, 'max_x': 0, 'min_y': 0, 'max_y': 0, 'min_z': 0, 'max_z': 0}
        
        x_coords = [p['x'] for p in points]
        y_coords = [p['y'] for p in points]
        z_coords = [p['z'] for p in points]
        
        return {
            'min_x': float(np.min(x_coords)),
            'max_x': float(np.max(x_coords)),
            'min_y': float(np.min(y_coords)),
            'max_y': float(np.max(y_coords)),
            'min_z': float(np.min(z_coords)),
            'max_z': float(np.max(z_coords))
        }
    
    def set_data_callback(self, callback: Callable):
        """데이터 콜백 설정"""
        self.data_callback = callback
    
    def get_semantic_labels(self) -> Dict[int, str]:
        """시맨틱 라벨 매핑 반환"""
        return self.semantic_labels
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.semantic_lidar:
                self.semantic_lidar.destroy()
                self.semantic_lidar = None
                self.logger.info("✅ Semantic LIDAR sensor destroyed")
            
            if self.publishers:
                for publisher in self.publishers.values():
                    publisher.undeclare()
                self.publishers = {}
            
            if self.zenoh_session:
                self.zenoh_session.close()
                self.zenoh_session = None
            
            self.logger.info("✅ Semantic LIDAR manager cleaned up")
            
        except Exception as e:
            self.logger.error(f"⚠️ Error during semantic LIDAR cleanup: {e}")
