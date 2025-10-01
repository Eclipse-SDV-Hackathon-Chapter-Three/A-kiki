#!/usr/bin/env python3
"""
LiDAR Visualization Module
LiDAR 포인트를 카메라 화면에 오버레이하는 기능을 제공합니다.
"""

import numpy as np
import pygame
from typing import List, Dict, Tuple, Optional


class LiDARVisualizer:
    """LiDAR 포인트 시각화 클래스"""
    
    def __init__(self):
        self.lidar_points_2d = []  # 2D 투영된 LiDAR 포인트들
        self.lidar_colors = []  # LiDAR 포인트 색상들
        self.show_lidar_overlay = True  # LiDAR 오버레이 표시 여부
        
        # Semantic 필터링
        self.target_semantic_classes = {
            'vehicle': 10,      # 차량
            'lane_marking': 6,  # 차선
            'pedestrian': 4     # 보행자
        }
        
        # 클래스별 색상 정의
        self.class_colors = {
            'vehicle': (255, 0, 0),      # 빨간색 - 차량
            'lane_marking': (0, 255, 0), # 녹색 - 차선
            'pedestrian': (0, 0, 255)    # 파란색 - 보행자
        }
    
    def filter_semantic_lidar_data(self, semantic_lidar_data: List[Dict]) -> List[Dict]:
        """Semantic LiDAR 데이터 필터링 (모든 포인트 표시)"""
        try:
            if not semantic_lidar_data:
                return []
            
            # 모든 포인트를 표시 (필터링 없음)
            print(f"🔍 모든 LiDAR 포인트 표시: {len(semantic_lidar_data)}개")
            return semantic_lidar_data
            
        except Exception as e:
            print(f"⚠️ Error filtering semantic LiDAR data: {e}")
            return []
    
    def project_lidar_to_camera(self, lidar_points: List[List[float]], 
                               lidar_transform, camera_transform, camera_intrinsic: np.ndarray) -> Tuple[List, List]:
        """LiDAR 3D 포인트를 카메라 2D 좌표로 투영 (CARLA lidar_to_camera.py 정확한 방식)"""
        try:
            if len(lidar_points) == 0:
                return [], []
            
            # LiDAR 포인트를 numpy 배열로 변환
            points = np.array(lidar_points)
            if len(points.shape) == 1:
                points = points.reshape(1, -1)
            
            print(f"🔍 투영 시도: {len(points)}개 포인트")
            print(f"   원본 포인트: x[{points[:, 0].min():.2f}, {points[:, 0].max():.2f}], y[{points[:, 1].min():.2f}, {points[:, 1].max():.2f}], z[{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
            print(f"   LiDAR transform shape: {lidar_transform.shape}")
            print(f"   Camera transform type: {type(camera_transform)}, shape: {camera_transform.shape if hasattr(camera_transform, 'shape') else 'No shape'}")
            print(f"   Camera intrinsic shape: {camera_intrinsic.shape}")
            
            # camera_transform을 numpy array로 변환
            if not hasattr(camera_transform, 'shape'):
                camera_transform = np.array(camera_transform)
                print(f"   Camera transform converted to numpy array: {camera_transform.shape}")
            
            # CARLA lidar_to_camera.py 방식 - 정확히 동일
            # 1. Point cloud in lidar sensor space array of shape (3, p_cloud_size)
            local_lidar_points = points.T  # (3, N)
            
            # 2. Add an extra 1.0 at the end of each 3d point so it becomes of shape (4, p_cloud_size)
            local_lidar_points = np.r_[local_lidar_points, [np.ones(local_lidar_points.shape[1])]]  # (4, N)
            
            # 3. This (4, 4) matrix transforms the points from lidar space to world space
            lidar_2_world = lidar_transform
            
            # 4. Transform the points from lidar space to world space
            world_points = np.dot(lidar_2_world, local_lidar_points)
            
            # 5. This (4, 4) matrix transforms the points from world to sensor coordinates
            world_2_camera = np.array(camera_transform)
            
            # 6. Transform the points from world space to camera space
            sensor_points = np.dot(world_2_camera, world_points)
            
            # 7. Change from UE4's coordinate system to an "standard" camera coordinate system (OpenCV)
            # (x, y, z) -> (y, -z, x)
            point_in_camera_coords = np.array([
                sensor_points[1],      # y -> x
                sensor_points[2] * -1, # -z -> y  
                sensor_points[0]       # x -> z
            ])
            
            print(f"   카메라 좌표계 변환 후: x[{point_in_camera_coords[0].min():.2f}, {point_in_camera_coords[0].max():.2f}], y[{point_in_camera_coords[1].min():.2f}, {point_in_camera_coords[1].max():.2f}], z[{point_in_camera_coords[2].min():.2f}, {point_in_camera_coords[2].max():.2f}]")
            
            # 8. Finally we can use our K matrix to do the actual 3D -> 2D
            points_2d = np.dot(camera_intrinsic, point_in_camera_coords)
            
            # 9. Remember to normalize the x, y values by the 3rd value
            points_2d = np.array([
                points_2d[0, :] / points_2d[2, :],
                points_2d[1, :] / points_2d[2, :],
                points_2d[2, :]
            ])
            
            # 10. At this point, points_2d[0, :] contains all the x and points_2d[1, :] contains all the y values
            points_2d = points_2d.T
            
            print(f"   2D 투영 후: x[{points_2d[:, 0].min():.2f}, {points_2d[:, 0].max():.2f}], y[{points_2d[:, 1].min():.2f}, {points_2d[:, 1].max():.2f}], z[{points_2d[:, 2].min():.2f}, {points_2d[:, 2].max():.2f}]")
            
            # 샘플 포인트들 출력 (처음 5개)
            print(f"   샘플 2D 포인트들:")
            for i in range(min(5, len(points_2d))):
                print(f"     포인트 {i}: ({points_2d[i, 0]:.1f}, {points_2d[i, 1]:.1f}, {points_2d[i, 2]:.1f})")
            
            # 11. Filter points that are out of the screen and behind the camera projection plane
            image_w, image_h = 1080, 720  # 카메라 해상도
            points_in_canvas_mask = (
                (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < image_w) &
                (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < image_h) &
                (np.abs(points_2d[:, 2]) > 0.1)  # z값이 0에 너무 가깝지 않은 포인트 (음수 허용)
            )
            
            print(f"   필터링 조건 확인:")
            print(f"     x 범위: {points_2d[:, 0].min():.2f} ~ {points_2d[:, 0].max():.2f} (0 ~ {image_w})")
            print(f"     y 범위: {points_2d[:, 1].min():.2f} ~ {points_2d[:, 1].max():.2f} (0 ~ {image_h})")
            print(f"     z 범위: {points_2d[:, 2].min():.2f} ~ {points_2d[:, 2].max():.2f} (|z| > 0.1)")
            x_valid = ((points_2d[:, 0] > 0.0) & (points_2d[:, 0] < image_w)).sum()
            y_valid = ((points_2d[:, 1] > 0.0) & (points_2d[:, 1] < image_h)).sum()
            z_valid = (np.abs(points_2d[:, 2]) > 0.1).sum()
            print(f"     x 유효: {x_valid}/{len(points_2d)}")
            print(f"     y 유효: {y_valid}/{len(points_2d)}")
            print(f"     z 유효: {z_valid}/{len(points_2d)}")
            
            valid_points_2d = points_2d[points_in_canvas_mask]
            valid_points_3d = point_in_camera_coords.T[points_in_canvas_mask]
            
            print(f"   화면 범위 내 유효 포인트: {len(valid_points_2d)}개")
            
            if len(valid_points_2d) == 0:
                return [], []
            
            # 거리 기반 색상 계산
            distances = np.sqrt(np.sum(valid_points_3d**2, axis=1))
            colors = self._calculate_lidar_colors(distances)
            
            return valid_points_2d.astype(int), colors
            
        except Exception as e:
            print(f"⚠️ Error projecting LiDAR to camera: {e}")
            return [], []
    
    def _calculate_lidar_colors(self, distances: np.ndarray) -> np.ndarray:
        """거리 기반 LiDAR 포인트 색상 계산"""
        try:
            # 거리 범위 정규화 (0-50m)
            max_distance = 50.0
            normalized_distances = np.clip(distances / max_distance, 0, 1)
            
            # Viridis 컬러맵 사용
            viridis_colors = np.array([
                [0.267, 0.004, 0.329],  # 보라
                [0.282, 0.140, 0.457],  # 파랑
                [0.253, 0.265, 0.529],  # 청록
                [0.206, 0.371, 0.406],  # 녹색
                [0.163, 0.471, 0.558],  # 연녹
                [0.127, 0.567, 0.551],  # 황록
                [0.134, 0.658, 0.517],  # 노랑
                [0.266, 0.748, 0.440],  # 주황
                [0.477, 0.821, 0.318],  # 빨강
                [0.741, 0.873, 0.149],  # 밝은 빨강
                [0.993, 0.906, 0.144]   # 흰색
            ])
            
            # 거리에 따른 색상 보간
            color_indices = (normalized_distances * (len(viridis_colors) - 1)).astype(int)
            color_indices = np.clip(color_indices, 0, len(viridis_colors) - 1)
            
            colors = viridis_colors[color_indices] * 255
            return colors.astype(int)
            
        except Exception as e:
            print(f"⚠️ Error calculating LiDAR colors: {e}")
            return np.array([[255, 255, 255]] * len(distances))
    
    def update_semantic_lidar_data(self, semantic_lidar_data: List[Dict]) -> None:
        """Semantic LiDAR 데이터를 업데이트 (시각화용)"""
        try:
            if not semantic_lidar_data:
                self.lidar_points_2d = []
                self.lidar_colors = []
                return
            
            # Semantic 필터링 적용
            filtered_data = self.filter_semantic_lidar_data(semantic_lidar_data)
            
            if not filtered_data:
                self.lidar_points_2d = []
                self.lidar_colors = []
                return
            
            # LiDAR 포인트 추출 (필터링된 데이터)
            lidar_points_3d = []
            lidar_semantic_tags = []
            for point in filtered_data:
                lidar_points_3d.append([point['x'], point['y'], point['z']])
                lidar_semantic_tags.append(point['semantic_id']) # semantic_id 저장
            
            lidar_points_3d = np.array(lidar_points_3d).T # (3, N)
            
            # 3D -> 2D 투영 (카메라 변환 행렬이 없으면 기본값 사용)
            if hasattr(self, 'camera_transform') and hasattr(self, 'camera_intrinsic'):
                camera_transform = self.camera_transform
                camera_intrinsic = self.camera_intrinsic
            else:
                # 기본 카메라 설정 (1080x720 해상도)
                camera_intrinsic = np.array([
                    [1080, 0, 540],
                    [0, 1080, 360],
                    [0, 0, 1]
                ])
                camera_transform = np.eye(4)
            
            # 3D 포인트를 동차 좌표로 변환
            lidar_points_3d_hom = np.vstack((lidar_points_3d, np.ones((1, lidar_points_3d.shape[1]))))
            
            # 카메라 변환 적용 (4x4 행렬)
            camera_points_3d = np.linalg.inv(camera_transform) @ lidar_points_3d_hom
            
            # 카메라 내부 파라미터 적용 (3x3 행렬)
            lidar_points_2d_hom = camera_intrinsic @ camera_points_3d[:3, :]
            
            # 정규화
            depths = lidar_points_2d_hom[2, :]
            valid_indices = np.where(depths > 0)[0] # 깊이가 양수인 유효한 포인트만
            
            self.lidar_points_2d = (lidar_points_2d_hom[:2, valid_indices] / depths[valid_indices]).T
            
            # 화면 범위 내에 있는 포인트만 필터링
            screen_width, screen_height = (1080, 720)  # 기본 화면 크기
            valid_screen_indices = np.where(
                (self.lidar_points_2d[:, 0] >= 0) & (self.lidar_points_2d[:, 0] < screen_width) &
                (self.lidar_points_2d[:, 1] >= 0) & (self.lidar_points_2d[:, 1] < screen_height)
            )[0]
            
            self.lidar_points_2d = self.lidar_points_2d[valid_screen_indices]
            
            # 유효한 semantic_tag만 저장
            valid_semantic_tags = [lidar_semantic_tags[i] for i in valid_indices[valid_screen_indices]]
            
            # semantic_tag에 따라 색상 지정
            self.lidar_colors = [self._get_color_from_semantic_tag(tag) for tag in valid_semantic_tags]
            
        except Exception as e:
            print(f"⚠️ Error updating semantic LiDAR data: {e}")
            self.lidar_points_2d = []
            self.lidar_colors = []

    def update_visualization(self, semantic_lidar_data: List[Dict], 
                           lidar_transform, camera_transform, camera_intrinsic: np.ndarray) -> None:
        """Semantic LiDAR 데이터를 시각화용으로 업데이트 (차량과 보행자만, 성능 최적화)"""
        try:
            if not semantic_lidar_data:
                self.lidar_points_2d = []
                self.lidar_colors = []
                return
            
            # 필터링 적용 (차량과 보행자만)
            filtered_data = self.filter_semantic_lidar_data(semantic_lidar_data)
            
            if not filtered_data:
                self.lidar_points_2d = []
                self.lidar_colors = []
                return
            
            # 전체 LiDAR 포인트를 일괄 투영 (성능 최적화)
            all_points = []
            
            for point in filtered_data:
                point_coords = [point['x'], point['y'], point['z']]
                all_points.append(point_coords)
            
            # 전체 포인트 일괄 투영
            all_points_2d = []
            all_colors = []
            
            if all_points:
                points_2d, colors = self.project_lidar_to_camera(
                    all_points, lidar_transform, camera_transform, camera_intrinsic
                )
                all_points_2d.extend(points_2d)
                all_colors.extend(colors)
            
            # 결과 저장
            self.lidar_points_2d = all_points_2d
            self.lidar_colors = all_colors
            
            print(f"🎯 LiDAR 시각화: 전체 {len(all_points)}개 포인트 처리")
            print(f"   📍 총 {len(all_points_2d)}개 포인트가 카메라 화면에 투영됨")
            
        except Exception as e:
            print(f"⚠️ Error updating LiDAR visualization: {e}")
    
    def _get_color_for_semantic_id(self, semantic_id: int) -> Tuple[int, int, int]:
        """Semantic ID에 따라 색상 반환"""
        # CARLA semantic segmentation ID에 따른 색상 매핑
        color_map = {
            0: (0, 0, 0),        # Unlabeled - 검은색
            1: (70, 70, 70),     # Road - 회색
            2: (190, 153, 153),  # Sidewalk - 연한 회색
            3: (250, 170, 30),   # Lane marking - 주황색
            4: (220, 20, 60),    # Vehicle - 빨간색
            5: (255, 0, 0),      # Pedestrian - 진한 빨간색
            6: (0, 0, 142),      # Traffic sign - 파란색
            7: (0, 0, 70),       # Traffic light - 진한 파란색
            8: (0, 60, 100),     # Vegetation - 청록색
            9: (0, 0, 90),       # Terrain - 진한 파란색
            10: (0, 0, 110),     # Sky - 파란색
            11: (80, 80, 80),    # Pole - 회색
            12: (0, 0, 230),     # Traffic sign - 밝은 파란색
            13: (128, 64, 128),  # Building - 보라색
            14: (244, 35, 232),  # Wall - 분홍색
            15: (70, 70, 70),    # Fence - 회색
            16: (190, 153, 153), # Ground - 연한 회색
            17: (102, 102, 156), # Bridge - 보라색
            18: (220, 220, 0),   # Rail track - 노란색
            19: (150, 100, 100), # Guard rail - 갈색
            20: (230, 150, 140), # Static - 연한 갈색
            21: (70, 130, 180),  # Dynamic - 하늘색
            22: (220, 20, 60),   # Water - 빨간색
            23: (255, 0, 0),     # Road line - 빨간색
            24: (0, 0, 0),       # Ground - 검은색
            25: (128, 64, 128),  # Ground - 보라색
            26: (70, 70, 70),    # Ground - 회색
            27: (190, 153, 153), # Ground - 연한 회색
            28: (102, 102, 156), # Ground - 보라색
            29: (220, 220, 0),   # Ground - 노란색
            30: (150, 100, 100), # Ground - 갈색
        }
        
        return color_map.get(semantic_id, (255, 255, 255))  # 기본값: 흰색
    
    def draw_overlay(self, screen: pygame.Surface) -> None:
        """pygame 화면에 LiDAR 포인트 오버레이 그리기 (차량과 보행자만, 성능 최적화)"""
        try:
            if not self.show_lidar_overlay or len(self.lidar_points_2d) == 0:
                return
            
            # 성능을 위해 최대 포인트 수 제한
            max_points = 1000
            if len(self.lidar_points_2d) > max_points:
                # 랜덤 샘플링으로 포인트 수 제한
                import random
                indices = random.sample(range(len(self.lidar_points_2d)), max_points)
                points_to_draw = [self.lidar_points_2d[i] for i in indices]
                colors_to_draw = [self.lidar_colors[i] for i in indices]
            else:
                points_to_draw = self.lidar_points_2d
                colors_to_draw = self.lidar_colors
            
            # LiDAR 포인트들을 점으로 그리기
            for point, color in zip(points_to_draw, colors_to_draw):
                # 화면 범위 내에 있는 포인트만 그리기
                x, y = int(point[0]), int(point[1])
                if 0 <= x < screen.get_width() and 0 <= y < screen.get_height():
                    # 차량은 크게, 보행자는 작게
                    if color == (220, 20, 60):  # 차량 (빨간색)
                        point_size = 3
                    else:  # 보행자 (파란색)
                        point_size = 2
                    
                    pygame.draw.circle(screen, color, (x, y), point_size)
            
        except Exception as e:
            print(f"⚠️ Error drawing LiDAR overlay: {e}")
    
    def toggle_overlay(self) -> None:
        """LiDAR 오버레이 표시 토글"""
        self.show_lidar_overlay = not self.show_lidar_overlay
        print(f"🎯 LiDAR 오버레이: {'ON' if self.show_lidar_overlay else 'OFF'}")
    
    def get_class_name(self, semantic_id: int) -> str:
        """Semantic ID를 클래스 이름으로 변환"""
        for class_name, class_id in self.target_semantic_classes.items():
            if class_id == semantic_id:
                return class_name
        return 'unknown'