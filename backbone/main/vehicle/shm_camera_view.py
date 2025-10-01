"""
SHM-based High-Performance Camera View for Chase Vehicle
추격차량을 위한 SHM 기반 고성능 카메라 뷰
"""

import carla
import numpy as np
import cv2
import pygame
import threading
import time
import mmap
import struct
import json
import zenoh
from typing import Optional, Dict, Any
import sys
import os

# 상위 디렉토리의 모듈들을 import하기 위해 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

try:
    from chase.perception.bounding_box_detector import BoundingBoxDetector
    from chase.perception.zenoh_bounding_box_publisher import ZenohBoundingBoxPublisher, create_zenoh_config
except ImportError as e:
    print(f"⚠️ Warning: Could not import bounding box modules: {e}")
    BoundingBoxDetector = None
    ZenohBoundingBoxPublisher = None
    create_zenoh_config = None

class SHMCameraView:
    """SHM 기반 고성능 카메라 뷰"""
    
    def __init__(self, world, vehicle, title="SHM Camera View", 
                 enable_bounding_boxes=True, enable_zenoh=True):
        self.world = world
        self.vehicle = vehicle
        self.title = title
        self.enable_bounding_boxes = enable_bounding_boxes
        self.enable_zenoh = enable_zenoh
        
        # 카메라 설정
        self.camera = None
        self.image_queue = None
        self.image_lock = threading.Lock()
        
        # SHM 설정
        self.shm_name = "carla_chase_camera_shm"
        self.shm_size = 20 * 1024 * 1024  # 20MB
        self.shm_file = None
        self.shm_mmap = None
        
        # 바운딩 박스 감지
        self.bounding_box_detector = None
        self.zenoh_publisher = None
        
        # Pygame 설정
        pygame.init()
        self.screen = None
        self.clock = pygame.time.Clock()
        
        # 성능 모니터링
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.current_fps = 0
        
        print(f"📷 SHM Camera View initialized for {title}")
        print(f"🎯 Bounding boxes: {'Enabled' if enable_bounding_boxes else 'Disabled'}")
        print(f"🌐 Zenoh: {'Enabled' if enable_zenoh else 'Disabled'}")
    
    def setup_camera(self, camera_location=carla.Location(x=1.5, z=1.4),
                     camera_rotation=carla.Rotation(pitch=0, yaw=0, roll=0),
                     image_size_x=1280, image_size_y=720, fov=60):
        """카메라 설정 및 SHM 초기화"""
        try:
            # 카메라 블루프린트 생성
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', str(image_size_x))
            camera_bp.set_attribute('image_size_y', str(image_size_y))
            camera_bp.set_attribute('fov', str(fov))
            
            # 카메라 스폰
            camera_transform = carla.Transform(camera_location, camera_rotation)
            self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
            
            # 이미지 큐 설정
            import queue
            self.image_queue = queue.Queue()
            self.camera.listen(lambda image: self.image_queue.put(image))
            
            # SHM 초기화
            if not self._setup_shm():
                return False
            
            # 바운딩 박스 감지기 설정
            if self.enable_bounding_boxes and BoundingBoxDetector:
                self.bounding_box_detector = BoundingBoxDetector(self.world, self.camera)
                print("🎯 Bounding box detector initialized")
            elif self.enable_bounding_boxes:
                print("⚠️ BoundingBoxDetector not available")
                self.enable_bounding_boxes = False
            else:
                self.bounding_box_detector = None
            
            # Zenoh 퍼블리셔 설정
            if self.enable_zenoh and ZenohBoundingBoxPublisher and create_zenoh_config:
                zenoh_config = create_zenoh_config(
                    topic='carla/chase/bounding_boxes',
                    publish_rate=10.0
                )
                self.zenoh_publisher = ZenohBoundingBoxPublisher(zenoh_config)
                if self.zenoh_publisher.connect():
                    self.zenoh_publisher.start_publishing()
                    print("🌐 Zenoh publisher connected and started")
                else:
                    print("⚠️ Failed to connect to Zenoh")
                    self.zenoh_publisher = None
            elif self.enable_zenoh:
                print("⚠️ Zenoh modules not available")
                self.enable_zenoh = False
            
            print("📷 SHM camera attached to vehicle")
            return True
            
        except Exception as e:
            print(f"❌ Error setting up camera: {e}")
            return False
    
    def _setup_shm(self):
        """SHM 설정"""
        try:
            # SHM 파일 생성
            self.shm_file = open(f"/dev/shm/{self.shm_name}", "w+b")
            self.shm_file.truncate(self.shm_size)
            
            # 메모리 맵핑
            self.shm_mmap = mmap.mmap(self.shm_file.fileno(), self.shm_size)
            
            print(f"✅ Created SHM: {self.shm_name} ({self.shm_size} bytes)")
            return True
            
        except Exception as e:
            print(f"❌ Error setting up SHM: {e}")
            return False
    
    def _write_frame_to_shm(self, frame, frame_id, timestamp):
        """프레임을 SHM에 쓰기"""
        try:
            if not self.shm_mmap:
                return False
            
            # 프레임 데이터 준비
            frame_data = frame.tobytes()
            frame_size = len(frame_data)
            
            # 헤더 정보 (frame_id, timestamp, width, height, channels, frame_size)
            header = struct.pack('QdIIII', frame_id, timestamp, frame.shape[1], 
                               frame.shape[0], frame.shape[2], frame_size)
            
            # SHM에 쓰기
            self.shm_mmap.seek(0)
            self.shm_mmap.write(header)
            self.shm_mmap.write(frame_data)
            self.shm_mmap.flush()
            
            return True
            
        except Exception as e:
            print(f"⚠️ Error writing frame to SHM: {e}")
            return False
    
    def _read_frame_from_shm(self):
        """SHM에서 프레임 읽기"""
        try:
            if not self.shm_mmap:
                return None
            
            # 헤더 읽기
            self.shm_mmap.seek(0)
            header_data = self.shm_mmap.read(32)  # 8+8+4+4+4+4 = 32 bytes
            if len(header_data) < 32:
                return None
            
            frame_id, timestamp, width, height, channels, frame_size = struct.unpack('QdIIII', header_data)
            
            # 프레임 데이터 읽기
            frame_data = self.shm_mmap.read(frame_size)
            if len(frame_data) < frame_size:
                return None
            
            # numpy 배열로 변환
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            if len(frame) == 0:
                return None
                
            frame = frame.reshape((height, width, channels))
            
            # 유효한 프레임인지 확인
            if frame.shape[0] == 0 or frame.shape[1] == 0 or frame.shape[2] == 0:
                return None
            
            return frame, frame_id, timestamp
            
        except Exception as e:
            return None
    
    def process_camera_image(self, image):
        """카메라 이미지 처리 (SHM에 쓰기)"""
        try:
            # 이미지 변환
            array = np.frombuffer(image.raw_data, dtype=np.uint8)
            array = array.reshape((image.height, image.width, 4))
            array = array[:, :, :3]  # RGBA -> RGB
            array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
            
            # SHM에 쓰기
            frame_id = int(time.time() * 1000)
            timestamp = time.time()
            self._write_frame_to_shm(array, frame_id, timestamp)
            
        except Exception as e:
            print(f"⚠️ Error processing camera image: {e}")
    
    def display_camera_view(self):
        """카메라 뷰 디스플레이 (SHM에서 읽기)"""
        try:
            # SHM에서 프레임 읽기
            result = self._read_frame_from_shm()
            if result is None:
                return
            
            frame, frame_id, timestamp = result
            
            # 유효한 프레임인지 확인
            if frame is None or frame.size == 0:
                return
            
            # 바운딩 박스 처리
            if self.enable_bounding_boxes and self.bounding_box_detector:
                detections = self.bounding_box_detector.detect_pedestrians_and_vehicles()
                frame = self.bounding_box_detector.draw_bounding_boxes_on_image(frame)
                
                if self.enable_zenoh and self.zenoh_publisher:
                    zenoh_data = self.bounding_box_detector.get_detection_data_for_zenoh()
                    self.zenoh_publisher.publish_bounding_boxes(zenoh_data)
            
            # Pygame으로 디스플레이
            if self.screen is None:
                self.screen = pygame.display.set_mode((frame.shape[1], frame.shape[0]), 
                                                    pygame.DOUBLEBUF | pygame.HWSURFACE)
                pygame.display.set_caption(self.title)
            
            # OpenCV -> Pygame 변환
            try:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_surface = pygame.surfarray.make_surface(frame_rgb.swapaxes(0, 1))
                
                # 화면에 그리기
                self.screen.blit(frame_surface, (0, 0))
                pygame.display.flip()
                
                # FPS 계산
                self._update_fps()
                
            except Exception as e:
                print(f"⚠️ Error converting frame: {e}")
                return
            
        except Exception as e:
            print(f"⚠️ Error displaying frame: {e}")
    
    def _update_fps(self):
        """FPS 업데이트"""
        self.fps_counter += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.fps_counter
            self.fps_counter = 0
            self.last_fps_time = current_time
            print(f"📊 SHM FPS: {self.current_fps:.1f}")
    
    def set_hud_info(self, **kwargs):
        """HUD 정보 설정"""
        # 간단한 HUD 정보 출력
        if kwargs:
            info_str = " | ".join([f"{k}: {v}" for k, v in kwargs.items()])
            print(f"📊 HUD: {info_str}")
    
    # 키 바인딩 제거 - 바운딩 박스는 항상 활성화
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.zenoh_publisher:
                self.zenoh_publisher.disconnect()
            
            if self.camera:
                self.camera.destroy()
            
            if self.shm_mmap:
                self.shm_mmap.close()
            
            if self.shm_file:
                self.shm_file.close()
            
            # SHM 파일 삭제
            import os
            try:
                os.unlink(f"/dev/shm/{self.shm_name}")
            except:
                pass
            
            pygame.quit()
            print("✅ SHM Camera View cleanup completed")
            
        except Exception as e:
            print(f"⚠️ Error during cleanup: {e}")
