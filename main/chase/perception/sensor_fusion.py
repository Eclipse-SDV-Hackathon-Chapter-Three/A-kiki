#!/usr/bin/env python3
"""
Sensor Fusion Module
카메라와 LiDAR 데이터를 퓨전하여 정확한 객체 정보를 생성합니다.
"""

import numpy as np
from typing import List, Dict, Tuple, Optional


class SensorFusion:
    """센서 퓨전 클래스"""
    
    def __init__(self):
        self.fused_objects = []  # 퓨전된 객체 정보
        self.camera_lidar_offset = None  # 카메라-LiDAR 오프셋
        
        # Semantic 클래스 정의
        self.target_semantic_classes = {
            'vehicle': 10,      # 차량
            'lane_marking': 6,  # 차선
            'pedestrian': 4     # 보행자
        }
    
    def calculate_camera_lidar_offset(self, camera_transform, lidar_transform) -> Optional[Dict]:
        """카메라와 LiDAR 간의 위치 오프셋 계산"""
        try:
            # 카메라 위치
            camera_location = camera_transform.location
            
            # LiDAR 위치
            lidar_location = lidar_transform.location
            
            # 오프셋 계산
            offset = {
                'x': lidar_location.x - camera_location.x,
                'y': lidar_location.y - camera_location.y,
                'z': lidar_location.z - camera_location.z
            }
            
            self.camera_lidar_offset = offset
            return offset
            
        except Exception as e:
            print(f"⚠️ Error calculating camera-LiDAR offset: {e}")
            return None
    
    def perform_sensor_fusion(self, camera_objects: List[Dict], lidar_objects: List[Dict]) -> List[Dict]:
        """카메라와 LiDAR 데이터를 퓨전하여 정확한 객체 정보 생성"""
        try:
            fused_objects = []
            
            # 카메라 객체와 LiDAR 객체 매칭
            for cam_obj in camera_objects:
                best_match = None
                best_distance = float('inf')
                
                # 카메라 객체의 중심점을 3D로 변환
                cam_center_3d = self._camera_to_3d_coordinates(cam_obj)
                
                for lidar_obj in lidar_objects:
                    # 거리 계산
                    distance = self._calculate_3d_distance(cam_center_3d, lidar_obj)
                    
                    if distance < best_distance and distance < 5.0:  # 5m 이내 매칭
                        best_distance = distance
                        best_match = lidar_obj
                
                # 퓨전된 객체 생성
                if best_match:
                    fused_obj = self._create_fused_object(cam_obj, best_match, best_distance)
                    fused_objects.append(fused_obj)
                else:
                    # LiDAR 매칭이 없는 경우 카메라 정보만 사용
                    fused_obj = self._create_camera_only_object(cam_obj)
                    fused_objects.append(fused_obj)
            
            self.fused_objects = fused_objects
            return fused_objects
            
        except Exception as e:
            print(f"⚠️ Error performing sensor fusion: {e}")
            return []
    
    def extract_lidar_objects(self, filtered_lidar_data: List[Dict]) -> List[Dict]:
        """필터링된 LiDAR 데이터에서 객체 정보 추출"""
        try:
            if not filtered_lidar_data:
                return []
            
            # 클래스별로 그룹화
            objects_by_class = {}
            for point in filtered_lidar_data:
                semantic_id = point.get('semantic_id', -1)
                class_name = self._get_semantic_class_name(semantic_id)
                
                if class_name not in objects_by_class:
                    objects_by_class[class_name] = []
                objects_by_class[class_name].append(point)
            
            # 각 클래스별로 객체 생성
            lidar_objects = []
            for class_name, points in objects_by_class.items():
                if len(points) < 3:  # 최소 3개 포인트 필요
                    continue
                
                # 객체 중심점과 크기 계산
                center = self._calculate_object_center(points)
                size = self._calculate_object_size(points)
                distance = np.sqrt(center['x']**2 + center['y']**2 + center['z']**2)
                
                # 객체 정보 생성
                obj = {
                    'type': 'lidar_object',
                    'semantic_class': class_name,
                    'semantic_id': points[0].get('semantic_id', -1),
                    'center': center,
                    'size': size,
                    'distance': distance,
                    'point_count': len(points),
                    'points': points,
                    'world_location': {
                        'x': center['x'],
                        'y': center['y'],
                        'z': center['z']
                    },
                    'velocity': 0.0  # LiDAR에서는 속도 계산이 복잡하므로 기본값
                }
                lidar_objects.append(obj)
            
            return lidar_objects
            
        except Exception as e:
            print(f"⚠️ Error extracting LiDAR objects: {e}")
            return []
    
    def _camera_to_3d_coordinates(self, camera_obj: Dict) -> Dict:
        """카메라 2D 좌표를 3D 좌표로 변환 (근사치)"""
        try:
            # 카메라 중심점
            bbox = camera_obj.get('bbox_2d', {})
            center_x = (bbox.get('x_min', 0) + bbox.get('x_max', 0)) / 2
            center_y = (bbox.get('y_min', 0) + bbox.get('y_max', 0)) / 2
            
            # 거리 정보 (카메라 객체에서)
            distance = camera_obj.get('distance', 10.0)
            
            # 카메라 내부 파라미터
            fx = 540.0  # focal length x
            fy = 540.0  # focal length y
            cx = 540.0  # principal point x
            cy = 360.0  # principal point y
            
            # 3D 좌표 계산
            x_3d = (center_x - cx) * distance / fx
            y_3d = (center_y - cy) * distance / fy
            z_3d = distance
            
            return {'x': x_3d, 'y': y_3d, 'z': z_3d}
            
        except Exception as e:
            print(f"⚠️ Error converting camera to 3D: {e}")
            return {'x': 0, 'y': 0, 'z': 10}
    
    def _calculate_3d_distance(self, point1: Dict, point2: Dict) -> float:
        """3D 공간에서 두 점 사이의 거리 계산"""
        try:
            dx = point1['x'] - point2['x']
            dy = point1['y'] - point2['y']
            dz = point1['z'] - point2['z']
            return np.sqrt(dx*dx + dy*dy + dz*dz)
        except:
            return float('inf')
    
    def _create_fused_object(self, camera_obj: Dict, lidar_obj: Dict, distance: float) -> Dict:
        """카메라와 LiDAR 정보를 퓨전한 객체 생성"""
        try:
            fused_obj = {
                'type': 'fused',
                'camera_data': camera_obj,
                'lidar_data': lidar_obj,
                'confidence': (camera_obj.get('confidence', 0.5) + 0.8) / 2,  # LiDAR는 높은 신뢰도
                'distance': lidar_obj.get('distance', camera_obj.get('distance', 10)),
                'semantic_class': self._get_semantic_class_name(lidar_obj.get('semantic_id', -1)),
                'fusion_distance': distance,
                'world_location': lidar_obj.get('world_location', {}),
                'velocity': lidar_obj.get('velocity', camera_obj.get('velocity', 0))
            }
            return fused_obj
        except Exception as e:
            print(f"⚠️ Error creating fused object: {e}")
            return camera_obj
    
    def _create_camera_only_object(self, camera_obj: Dict) -> Dict:
        """카메라 정보만으로 객체 생성"""
        try:
            camera_only_obj = {
                'type': 'camera_only',
                'camera_data': camera_obj,
                'confidence': camera_obj.get('confidence', 0.5) * 0.8,  # 낮은 신뢰도
                'distance': camera_obj.get('distance', 10),
                'semantic_class': 'unknown',
                'fusion_distance': float('inf'),
                'world_location': camera_obj.get('world_location', {}),
                'velocity': camera_obj.get('velocity', 0)
            }
            return camera_only_obj
        except Exception as e:
            print(f"⚠️ Error creating camera-only object: {e}")
            return camera_obj
    
    def _get_semantic_class_name(self, semantic_id: int) -> str:
        """Semantic ID를 클래스 이름으로 변환"""
        for class_name, class_id in self.target_semantic_classes.items():
            if class_id == semantic_id:
                return class_name
        return 'unknown'
    
    def _calculate_object_center(self, points: List[Dict]) -> Dict:
        """포인트들의 중심점 계산"""
        try:
            x_coords = [p['x'] for p in points]
            y_coords = [p['y'] for p in points]
            z_coords = [p['z'] for p in points]
            
            return {
                'x': np.mean(x_coords),
                'y': np.mean(y_coords),
                'z': np.mean(z_coords)
            }
        except:
            return {'x': 0, 'y': 0, 'z': 0}
    
    def _calculate_object_size(self, points: List[Dict]) -> Dict:
        """포인트들의 크기 계산 (bounding box)"""
        try:
            x_coords = [p['x'] for p in points]
            y_coords = [p['y'] for p in points]
            z_coords = [p['z'] for p in points]
            
            return {
                'width': max(x_coords) - min(x_coords),
                'height': max(y_coords) - min(y_coords),
                'depth': max(z_coords) - min(z_coords)
            }
        except:
            return {'width': 0, 'height': 0, 'depth': 0}
    
    def get_fusion_statistics(self) -> Dict:
        """퓨전 통계 정보 반환"""
        try:
            total_objects = len(self.fused_objects)
            fused_count = sum(1 for obj in self.fused_objects if obj.get('type') == 'fused')
            camera_only_count = sum(1 for obj in self.fused_objects if obj.get('type') == 'camera_only')
            
            return {
                'total_objects': total_objects,
                'fused_objects': fused_count,
                'camera_only_objects': camera_only_count,
                'fusion_rate': fused_count / max(total_objects, 1)
            }
        except Exception as e:
            print(f"⚠️ Error getting fusion statistics: {e}")
            return {'total_objects': 0, 'fused_objects': 0, 'camera_only_objects': 0, 'fusion_rate': 0.0}


