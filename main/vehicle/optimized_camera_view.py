"""
Optimized Camera View for Chase Vehicle
추격차량을 위한 최적화된 카메라 뷰 (SHM 없이 고성능)
"""

import carla
import numpy as np
import cv2
import pygame
import threading
import time
from typing import Optional, Dict, Any
import sys
import os
import queue

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

class OptimizedCameraView:
    """최적화된 카메라 뷰 (고성능, 안정적)"""
    
    def __init__(self, world, vehicle, title="Optimized Camera View", 
                 enable_bounding_boxes=True, enable_zenoh=True):
        self.world = world
        self.vehicle = vehicle
        self.title = title
        self.enable_bounding_boxes = enable_bounding_boxes
        self.enable_zenoh = enable_zenoh
        
        # 카메라 설정
        self.camera = None
        self.image_queue = queue.Queue(maxsize=2)  # 작은 큐로 지연 최소화
        
        # Zenoh 카메라 데이터 (동기화용)
        self.zenoh_camera_image = None
        self.zenoh_detected_objects = []
        
        # 바운딩 박스 감지
        self.bounding_box_detector = None
        self.zenoh_publisher = None
        
        # Pygame 설정 (안전한 초기화)
        try:
            if not pygame.get_init():
                pygame.init()
            self.screen = None
            self.clock = pygame.time.Clock()
            self.font = pygame.font.SysFont('monospace', 16)
            self.small_font = pygame.font.SysFont('monospace', 14)
            self.hud_info = {}
            print("✅ Pygame initialized successfully")
        except Exception as e:
            print(f"⚠️ Pygame initialization error: {e}")
            self.screen = None
            self.clock = None
            self.font = None
            self.hud_info = {}
        
        # 성능 모니터링
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.current_fps = 0
        
        # 멀티스레딩
        self.running = True
        self.display_thread = None
        
        print(f"📷 Optimized Camera View initialized for {title}")
        print(f"🎯 Bounding boxes: {'Enabled' if enable_bounding_boxes else 'Disabled'}")
        print(f"🌐 Zenoh: {'Enabled' if enable_zenoh else 'Disabled'}")
        
        # Pygame 화면 초기화 (Zenoh 카메라용)
        self._init_pygame_screen()
    
    def _init_pygame_screen(self):
        """Pygame 화면 초기화"""
        try:
            # Pygame 초기화
            if not pygame.get_init():
                pygame.init()
            
            # 소프트웨어 렌더링 모드로 설정 (OpenGL 컨텍스트 문제 해결)
            import os
            os.environ['SDL_VIDEODRIVER'] = 'x11'
            
            # 화면 크기 설정 (카메라 이미지 크기와 일치)
            # CARLA 카메라 설정: 1080x720
            width, height = 1080, 720
            self.screen = pygame.display.set_mode((width, height), pygame.SWSURFACE)
            pygame.display.set_caption(self.title)
            
            # 폰트 초기화
            if not pygame.font.get_init():
                pygame.font.init()
            self.font = pygame.font.Font(None, 24)
            self.small_font = pygame.font.Font(None, 18)
            
            # 화면 크기 저장
            self.screen_width = width
            self.screen_height = height
            
            print(f"✅ Pygame screen initialized: {width}x{height}")
            
        except Exception as e:
            print(f"❌ Failed to initialize pygame screen: {e}")
            self.screen = None
            self.font = None
            self.screen_width = 1080
            self.screen_height = 720
    
    def setup_camera(self, camera_location=carla.Location(x=1.5, z=1.6),
                     camera_rotation=carla.Rotation(pitch=0, yaw=0, roll=0),
                     image_size_x=1080, image_size_y=720, fov=90):
        """카메라 설정"""
        try:
            # 카메라 블루프린트 생성
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', str(image_size_x))
            camera_bp.set_attribute('image_size_y', str(image_size_y))
            camera_bp.set_attribute('fov', str(fov))
            
            # 카메라 스폰
            camera_transform = carla.Transform(camera_location, camera_rotation)
            self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
            
            # 카메라 리스너 설정
            self.camera.listen(lambda image: self.process_camera_image(image))
            
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
            
            # 디스플레이 스레드 삭제됨 - Zenoh 카메라만 사용
            print("📷 Using Zenoh camera only - CARLA display thread removed")
            
            print("📷 Optimized camera attached to vehicle")
            return True
            
        except Exception as e:
            print(f"❌ Error setting up camera: {e}")
            return False
    
    def process_camera_image(self, image):
        """카메라 이미지 처리 (비동기)"""
        try:
            # 이미지 변환 (색상 순서 수정)
            array = np.frombuffer(image.raw_data, dtype=np.uint8)
            array = array.reshape((image.height, image.width, 4))
            array = array[:, :, :3]  # RGBA -> RGB
            # CARLA는 BGRA 순서이므로 BGR로 직접 변환
            array = cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
            
            # 큐에 추가 (오래된 프레임은 버림)
            try:
                self.image_queue.put_nowait(array)
            except queue.Full:
                # 오래된 프레임 제거하고 새 프레임 추가
                try:
                    self.image_queue.get_nowait()
                    self.image_queue.put_nowait(array)
                except queue.Empty:
                    pass
            
        except Exception as e:
            print(f"⚠️ Error processing camera image: {e}")
    
    
    
    def display_camera_view(self):
        """카메라 뷰 디스플레이 (메인 스레드에서 호출)"""
        try:
            if not self.screen:
                return
            
            # Zenoh 카메라 데이터가 있으면 우선 사용
            if self.zenoh_camera_image is not None:
                self._display_zenoh_camera_view()
            else:
                # Zenoh 카메라 데이터가 없으면 대기 화면 표시
                self._display_waiting_screen()
                
        except Exception as e:
            print(f"⚠️ Error in display_camera_view: {e}")
    
    def _display_zenoh_camera_view(self):
        """Zenoh 카메라 뷰 표시"""
        try:
            if self.zenoh_camera_image is None:
                return
            
            # 화면이 없으면 스킵 (이미 _init_pygame_screen에서 초기화됨)
            if self.screen is None:
                return
            
            # 이미지 크기 확인 및 조정
            image_height, image_width = self.zenoh_camera_image.shape[:2]
            print(f"📷 Image size: {image_width}x{image_height}, Screen size: {self.screen_width}x{self.screen_height}")
            
            # BGR to RGB 변환
            frame_rgb = cv2.cvtColor(self.zenoh_camera_image, cv2.COLOR_BGR2RGB)
            
            # 이미지 크기가 화면 크기와 다르면 리사이즈
            if image_width != self.screen_width or image_height != self.screen_height:
                frame_rgb = cv2.resize(frame_rgb, (self.screen_width, self.screen_height))
                print(f"📷 Resized image to: {self.screen_width}x{self.screen_height}")
            
            # Pygame surface로 변환
            frame_surface = pygame.surfarray.make_surface(frame_rgb.swapaxes(0, 1))
            
            # 화면에 그리기
            self.screen.blit(frame_surface, (0, 0))
            
            # 바운딩 박스 그리기 (Zenoh 감지된 객체들)
            if self.zenoh_detected_objects:
                self._draw_zenoh_bounding_boxes()
            
            # HUD 그리기
            self._draw_hud(self.screen)
            
            # 안정적인 화면 업데이트
            try:
                pygame.display.flip()
            except Exception:
                pass  # 화면 업데이트 에러 무시
            
        except Exception as e:
            print(f"⚠️ Error displaying Zenoh camera view: {e}")
            import traceback
            traceback.print_exc()
    
    def _draw_zenoh_bounding_boxes(self):
        """Zenoh 감지된 객체들의 바운딩 박스 그리기"""
        try:
            if not self.zenoh_detected_objects or self.screen is None:
                return
            
            for i, obj in enumerate(self.zenoh_detected_objects):
                bbox = obj.get('bbox_2d', {})
                if not bbox:
                    continue
                
                # CARLA ground truth 형식 처리
                if isinstance(bbox, dict):
                    x_min = int(bbox.get('x_min', 0))
                    y_min = int(bbox.get('y_min', 0))
                    x_max = int(bbox.get('x_max', 0))
                    y_max = int(bbox.get('y_max', 0))
                    width = x_max - x_min
                    height = y_max - y_min
                elif isinstance(bbox, list) and len(bbox) >= 4:
                    # OpenCV 형식: [x, y, w, h]
                    x_min = int(bbox[0])
                    y_min = int(bbox[1])
                    width = int(bbox[2])
                    height = int(bbox[3])
                    x_max = x_min + width
                    y_max = y_min + height
                else:
                    continue
                
                # 화면 경계 체크
                screen_width = self.screen.get_width()
                screen_height = self.screen.get_height()
                
                if x_min < 0 or y_min < 0 or x_max > screen_width or y_max > screen_height:
                    continue
                
                # 최소 크기 체크
                if width < 5 or height < 5:
                    continue
                
                # 바운딩 박스 그리기
                rect = pygame.Rect(x_min, y_min, width, height)
                
                # 객체 타입에 따른 색상
                object_type = obj.get('type', 'unknown')
                if object_type == 'vehicle':
                    color = (255, 0, 0)  # 빨간색
                elif object_type == 'pedestrian':
                    color = (0, 255, 0)  # 초록색
                else:
                    color = (255, 255, 0)  # 노란색
                
                pygame.draw.rect(self.screen, color, rect, 3)
                
                # 라벨 추가
                if self.font:
                    label = f"{object_type}"
                    text = self.font.render(label, True, color)
                    self.screen.blit(text, (x_min, max(0, y_min - 30)))
                    
                    # 거리 정보 추가
                    distance = obj.get('distance', 0)
                    if distance > 0:
                        distance_text = self.small_font.render(f"{distance:.1f}m", True, color)
                        self.screen.blit(distance_text, (x_min, min(screen_height - 20, y_max + 5)))
                        
        except Exception as e:
            print(f"⚠️ Error drawing bounding boxes: {e}")
            import traceback
            traceback.print_exc()
    
    def _display_waiting_screen(self):
        """대기 화면 표시 - 간단한 검은 화면"""
        try:
            if not self.screen:
                return
            
            # 검은 화면만 표시 (HUD 제거됨)
            self.screen.fill((0, 0, 0))
            
            # 안정적인 화면 업데이트
            try:
                pygame.display.flip()
            except Exception:
                pass  # 화면 업데이트 에러 무시
            
        except Exception:
            pass  # 조용히 무시
    
    def set_hud_info(self, **kwargs):
        """HUD 정보 설정 및 저장"""
        self.hud_info = kwargs
    
    def set_zenoh_camera_data(self, image, detected_objects=None):
        """Zenoh 카메라 데이터 설정 (동기화용)"""
        # Zenoh 카메라 데이터가 처음 설정될 때 CARLA 디스플레이 스레드 중지
        if self.zenoh_camera_image is None and image is not None:
            print("📷 Zenoh camera data received - stopping CARLA display thread")
            self.running = False  # _display_loop 스레드 중지
        
        self.zenoh_camera_image = image
        if detected_objects is not None:
            self.zenoh_detected_objects = detected_objects
    
    def get_zenoh_camera_image(self):
        """Zenoh 카메라 이미지 가져오기"""
        return self.zenoh_camera_image
    
    def get_zenoh_detected_objects(self):
        """Zenoh 감지된 객체들 가져오기"""
        return self.zenoh_detected_objects
    
    def _draw_hud(self, screen):
        """HUD 그리기"""
        try:
            if not self.font or not self.hud_info:
                return
            
            y_offset = 10
            for key, value in self.hud_info.items():
                if value:
                    text = self.font.render(f"{key}: {value}", True, (255, 255, 255))
                    screen.blit(text, (10, y_offset))
                    y_offset += 25
                    
        except Exception as e:
            print(f"⚠️ Error drawing HUD: {e}")
    
    
    
    def cleanup(self):
        """리소스 정리"""
        try:
            self.running = False
            
            if self.zenoh_publisher:
                self.zenoh_publisher.disconnect()
            
            if self.camera:
                self.camera.destroy()
            
            pygame.quit()
            print("✅ Optimized Camera View cleanup completed")
            
        except Exception as e:
            print(f"⚠️ Error during cleanup: {e}")