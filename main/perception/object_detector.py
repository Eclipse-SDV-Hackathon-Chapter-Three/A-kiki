"""
Object Detector for Chase Vehicle
Detects and classifies objects in sensor data
"""

import numpy as np
import cv2
import time
from typing import List, Dict, Tuple

class ObjectDetector:
    """객체 탐지 클래스"""
    
    def __init__(self):
        self.detected_objects = []
        self.detection_history = []
        
        print("🎯 Object Detector initialized")
    
    def detect_vehicles_in_camera(self, image_data):
        """카메라 이미지에서 차량 탐지"""
        try:
            if image_data is None or 'image' not in image_data:
                return []
            
            image = image_data['image']
            vehicles = []
            
            # 간단한 차량 탐지 (색상 기반)
            # 실제로는 YOLO, R-CNN 등의 딥러닝 모델 사용
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # 차량 색상 범위 정의 (예: 파란색, 빨간색, 흰색 등)
            color_ranges = [
                ([100, 50, 50], [130, 255, 255]),  # 파란색
                ([0, 50, 50], [10, 255, 255]),     # 빨간색
                ([0, 0, 200], [180, 30, 255])      # 흰색
            ]
            
            for i, (lower, upper) in enumerate(color_ranges):
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 1000:  # 최소 면적 필터
                        x, y, w, h = cv2.boundingRect(contour)
                        vehicles.append({
                            'type': 'vehicle',
                            'bbox': (x, y, w, h),
                            'confidence': min(area / 10000, 1.0),
                            'color': ['blue', 'red', 'white'][i]
                        })
            
            return vehicles
            
        except Exception as e:
            print(f"⚠️ Error detecting vehicles in camera: {e}")
            return []
    
    def detect_vehicles_in_lidar(self, lidar_data):
        """LiDAR 데이터에서 차량 탐지"""
        try:
            if lidar_data is None or 'points' not in lidar_data:
                return []
            
            points = lidar_data['points']
            vehicles = []
            
            # 간단한 클러스터링 기반 차량 탐지
            if len(points) > 0:
                # 거리 기반 필터링 (50m 이내)
                distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
                valid_points = points[distances < 50.0]
                
                if len(valid_points) > 100:  # 충분한 포인트가 있을 때
                    # 간단한 클러스터링 (K-means 대신 거리 기반)
                    clusters = self._cluster_points(valid_points)
                    
                    for cluster in clusters:
                        if len(cluster) > 50:  # 최소 포인트 수
                            center = np.mean(cluster, axis=0)
                            size = np.max(cluster, axis=0) - np.min(cluster, axis=0)
                            
                            # 차량 크기 필터링 (대략적인 차량 크기)
                            if 2.0 < size[0] < 8.0 and 1.5 < size[1] < 3.0 and 1.0 < size[2] < 2.5:
                                vehicles.append({
                                    'type': 'vehicle',
                                    'position': center,
                                    'size': size,
                                    'confidence': min(len(cluster) / 500, 1.0),
                                    'points': cluster
                                })
            
            return vehicles
            
        except Exception as e:
            print(f"⚠️ Error detecting vehicles in LiDAR: {e}")
            return []
    
    def detect_vehicles_in_radar(self, radar_data):
        """레이더 데이터에서 차량 탐지"""
        try:
            if radar_data is None or 'points' not in radar_data:
                return []
            
            points = radar_data['points']
            vehicles = []
            
            # 레이더 포인트에서 움직이는 객체 탐지
            if len(points) > 0:
                # 속도가 있는 객체만 필터링
                velocities = np.sqrt(points[:, 3]**2)  # velocity magnitude
                moving_points = points[velocities > 0.1]  # 0.1 m/s 이상
                
                if len(moving_points) > 10:
                    # 클러스터링
                    clusters = self._cluster_points(moving_points[:, :3])
                    
                    for cluster in clusters:
                        if len(cluster) > 5:
                            center = np.mean(cluster, axis=0)
                            avg_velocity = np.mean(moving_points[velocities > 0.1][:, 3])
                            
                            vehicles.append({
                                'type': 'vehicle',
                                'position': center,
                                'velocity': avg_velocity,
                                'confidence': min(len(cluster) / 50, 1.0)
                            })
            
            return vehicles
            
        except Exception as e:
            print(f"⚠️ Error detecting vehicles in radar: {e}")
            return []
    
    def _cluster_points(self, points, min_distance=2.0):
        """포인트 클러스터링"""
        try:
            if len(points) == 0:
                return []
            
            clusters = []
            used = np.zeros(len(points), dtype=bool)
            
            for i, point in enumerate(points):
                if used[i]:
                    continue
                
                cluster = [point]
                used[i] = True
                
                # 거리 기반으로 클러스터 구성
                for j, other_point in enumerate(points[i+1:], i+1):
                    if used[j]:
                        continue
                    
                    distance = np.linalg.norm(point - other_point)
                    if distance < min_distance:
                        cluster.append(other_point)
                        used[j] = True
                
                clusters.append(np.array(cluster))
            
            return clusters
            
        except Exception as e:
            print(f"⚠️ Error clustering points: {e}")
            return []
    
    def fuse_detections(self, camera_vehicles, lidar_vehicles, radar_vehicles):
        """다중 센서 탐지 결과 융합"""
        try:
            all_vehicles = []
            
            # 카메라 탐지 결과 추가
            for vehicle in camera_vehicles:
                vehicle['sensor'] = 'camera'
                all_vehicles.append(vehicle)
            
            # LiDAR 탐지 결과 추가
            for vehicle in lidar_vehicles:
                vehicle['sensor'] = 'lidar'
                all_vehicles.append(vehicle)
            
            # 레이더 탐지 결과 추가
            for vehicle in radar_vehicles:
                vehicle['sensor'] = 'radar'
                all_vehicles.append(vehicle)
            
            # 신뢰도 기반 정렬
            all_vehicles.sort(key=lambda x: x.get('confidence', 0), reverse=True)
            
            # 중복 제거 (간단한 거리 기반)
            fused_vehicles = []
            for vehicle in all_vehicles:
                is_duplicate = False
                for fused in fused_vehicles:
                    if self._is_same_vehicle(vehicle, fused):
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    fused_vehicles.append(vehicle)
            
            return fused_vehicles
            
        except Exception as e:
            print(f"⚠️ Error fusing detections: {e}")
            return []
    
    def _is_same_vehicle(self, vehicle1, vehicle2, threshold=5.0):
        """두 차량이 같은 객체인지 판단"""
        try:
            pos1 = vehicle1.get('position', vehicle1.get('bbox', [0, 0, 0, 0])[:2])
            pos2 = vehicle2.get('position', vehicle2.get('bbox', [0, 0, 0, 0])[:2])
            
            if len(pos1) >= 2 and len(pos2) >= 2:
                distance = np.linalg.norm(np.array(pos1[:2]) - np.array(pos2[:2]))
                return distance < threshold
            
            return False
            
        except Exception as e:
            print(f"⚠️ Error comparing vehicles: {e}")
            return False
    
    def update_detection_history(self, vehicles):
        """탐지 히스토리 업데이트"""
        try:
            self.detection_history.append({
                'timestamp': time.time(),
                'vehicles': vehicles
            })
            
            # 최근 10초간의 히스토리만 유지
            current_time = time.time()
            self.detection_history = [
                entry for entry in self.detection_history
                if current_time - entry['timestamp'] < 10.0
            ]
            
        except Exception as e:
            print(f"⚠️ Error updating detection history: {e}")
    
    def get_detection_statistics(self):
        """탐지 통계 반환"""
        try:
            if not self.detection_history:
                return {'total_detections': 0, 'avg_confidence': 0.0}
            
            total_detections = sum(len(entry['vehicles']) for entry in self.detection_history)
            all_confidences = []
            
            for entry in self.detection_history:
                for vehicle in entry['vehicles']:
                    if 'confidence' in vehicle:
                        all_confidences.append(vehicle['confidence'])
            
            avg_confidence = np.mean(all_confidences) if all_confidences else 0.0
            
            return {
                'total_detections': total_detections,
                'avg_confidence': avg_confidence,
                'history_length': len(self.detection_history)
            }
            
        except Exception as e:
            print(f"⚠️ Error getting detection statistics: {e}")
            return {'total_detections': 0, 'avg_confidence': 0.0}

