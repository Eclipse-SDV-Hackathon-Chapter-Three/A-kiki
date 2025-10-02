#!/usr/bin/env python3
"""
Camera Manager
카메라 관리 클래스
"""

import carla
import sys
import os

# 상위 디렉토리의 모듈들을 import하기 위해 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, parent_dir)

from vehicle.optimized_camera_view import OptimizedCameraView

class CameraManager:
    """카메라 관리 클래스"""
    
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.camera_view = None
        self.temp_camera = None
    
    def setup_camera(self):
        """추격차량 카메라 설정 - Zenoh 카메라만 사용"""
        try:
            if not self.vehicle:
                print("❌ No vehicle available for camera setup")
                return False
            
            # OptimizedCameraView 초기화
            self.camera_view = OptimizedCameraView(
                self.world, 
                self.vehicle, 
                "Auto Chase Vehicle Camera View",
                enable_bounding_boxes=True,  # 바운딩 박스 활성화
                enable_zenoh=True           # Zenoh 활성화
            )
            
            # 카메라 설정
            camera_location = carla.Location(x=1.5, z=1.6)
            camera_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
            
            if self.camera_view.setup_camera(camera_location, camera_rotation):
                print("📷 Chase vehicle camera attached with bounding box detection")
                return True
            else:
                return False
                
        except Exception as e:
            print(f"❌ Error setting up camera: {e}")
            return False
    
    def get_camera_image(self):
        """카메라 이미지 반환"""
        try:
            if not self.camera_view:
                return None
            
            return self.camera_view.get_latest_image()
        except Exception as e:
            print(f"⚠️ Error getting camera image: {e}")
            return None
    
    def get_camera_transform(self):
        """카메라 변환 행렬 반환"""
        try:
            if not self.camera_view or not self.camera_view.camera:
                return None
            
            return self.camera_view.camera.get_transform().get_matrix()
        except Exception as e:
            print(f"⚠️ Error getting camera transform: {e}")
            return None
    
    def get_camera_intrinsic(self):
        """카메라 내부 파라미터 반환 (carla_vehicle_spawner.py 설정 기반)"""
        try:
            import numpy as np
            
            # carla_vehicle_spawner.py에서 설정한 카메라 파라미터 사용
            image_size_x = 1080
            image_size_y = 720
            fov = 90  # degrees
            
            # FOV를 픽셀 단위로 변환
            focal_length = image_size_x / (2.0 * np.tan(np.radians(fov) / 2.0))
            
            intrinsic = np.array([
                [focal_length, 0, image_size_x / 2.0],
                [0, focal_length, image_size_y / 2.0],
                [0, 0, 1]
            ])
            
            print(f"📷 카메라 내부 파라미터: {image_size_x}x{image_size_y}, FOV={fov}°, focal_length={focal_length:.1f}")
            return intrinsic
            
        except Exception as e:
            print(f"⚠️ Error getting camera intrinsic: {e}")
            # 기본 카메라 내부 파라미터 반환
            import numpy as np
            return np.array([
                [1080, 0, 540],
                [0, 1080, 360],
                [0, 0, 1]
            ])
    
    def get_bounding_boxes(self, max_distance=200.0):
        """바운딩 박스 감지"""
        try:
            if not self.camera_view or not self.camera_view.bounding_box_detector:
                return []
            
            return self.camera_view.bounding_box_detector.detect_pedestrians_and_vehicles(max_distance)
        except Exception as e:
            print(f"⚠️ Error getting bounding boxes: {e}")
            return []
    
    def cleanup(self):
        """카메라 정리"""
        try:
            if self.camera_view:
                self.camera_view.cleanup()
                self.camera_view = None
            
            if self.temp_camera:
                self.temp_camera.destroy()
                self.temp_camera = None
                
        except Exception as e:
            print(f"⚠️ Error cleaning up camera: {e}")

