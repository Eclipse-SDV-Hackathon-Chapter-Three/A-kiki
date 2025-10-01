"""
Camera View Module
차량용 공통 카메라 뷰 모듈
"""

import carla
import numpy as np
import cv2
import pygame
import threading
import time
import sys
import os
import os

# chase 모듈 경로 추가
chase_path = os.path.join(os.path.dirname(__file__), '..', 'chase')
if chase_path not in sys.path:
    sys.path.append(chase_path)

try:
    from perception.bounding_box_detector import BoundingBoxDetector
    from perception.zenoh_bounding_box_publisher import ZenohBoundingBoxPublisher, create_zenoh_config
except ImportError as e:
    print(f"⚠️ Warning: Could not import bounding box modules: {e}")
    BoundingBoxDetector = None
    ZenohBoundingBoxPublisher = None
    create_zenoh_config = None

class CameraView:
    """차량용 카메라 뷰 클래스"""
    
    def __init__(self, world, vehicle, window_title="Vehicle Camera View", enable_bounding_boxes=True, enable_zenoh=True):
        self.world = world
        self.vehicle = vehicle
        
        # 카메라 관련
        self.camera = None
        self.camera_image = None
        self.camera_lock = threading.Lock()
        
        # 바운딩 박스 관련
        self.enable_bounding_boxes = enable_bounding_boxes
        self.bounding_box_detector = None
        self.zenoh_publisher = None
        self.enable_zenoh = enable_zenoh
        
        # Pygame 초기화
        pygame.init()
        self.screen = pygame.display.set_mode((1280, 720))
        pygame.display.set_caption(window_title)
        self.clock = pygame.time.Clock()
        
        # HUD 설정
        self.show_hud = True
        self.hud_info = {}
        
        print(f"📷 Camera View initialized for {window_title}")
        print(f"🎯 Bounding boxes: {'Enabled' if enable_bounding_boxes else 'Disabled'}")
        print(f"🌐 Zenoh: {'Enabled' if enable_zenoh else 'Disabled'}")
    
    def setup_camera(self, camera_location=carla.Location(x=1.5, z=1.4), 
                     camera_rotation=carla.Rotation(pitch=0, yaw=0, roll=0),
                     image_size_x=1280, image_size_y=720, fov=90):
        """카메라 설정"""
        try:
            # 카메라 블루프린트
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', str(image_size_x))
            camera_bp.set_attribute('image_size_y', str(image_size_y))
            camera_bp.set_attribute('fov', str(fov))
            
            # 카메라 위치
            camera_transform = carla.Transform(camera_location, camera_rotation)
            
            # 카메라 스폰
            self.camera = self.world.spawn_actor(
                camera_bp, 
                camera_transform, 
                attach_to=self.vehicle
            )
            
            # 카메라 콜백 설정
            self.camera.listen(lambda image: self.process_camera_image(image))
            
            # 바운딩 박스 감지기 초기화
            if self.enable_bounding_boxes and BoundingBoxDetector:
                self.bounding_box_detector = BoundingBoxDetector(self.world, self.camera)
                print("🎯 Bounding box detector initialized")
            elif self.enable_bounding_boxes:
                print("⚠️ BoundingBoxDetector not available")
                self.enable_bounding_boxes = False
            else:
                # 바운딩 박스가 비활성화된 경우에도 빈 감지기 객체 생성 (안전한 접근을 위해)
                self.bounding_box_detector = None
            
            # Zenoh 퍼블리셔 초기화
            if self.enable_zenoh and ZenohBoundingBoxPublisher and create_zenoh_config:
                zenoh_config = create_zenoh_config(
                    topic='carla/chase/bounding_boxes',  # 표준화된 토픽명
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
            
            print("📷 Camera attached to vehicle")
            return True
            
        except Exception as e:
            print(f"❌ Error setting up camera: {e}")
            return False
    
    def process_camera_image(self, image):
        """카메라 이미지 처리"""
        try:
            # CARLA 이미지를 OpenCV 형식으로 변환
            array = np.frombuffer(image.raw_data, dtype=np.uint8)
            array = array.reshape((image.height, image.width, 4))
            array = array[:, :, :3]  # BGRA -> BGR (알파 채널 제거)
            
            # 스레드 안전하게 이미지 저장
            with self.camera_lock:
                self.camera_image = array.copy()
                
        except Exception as e:
            print(f"⚠️ Error processing camera image: {e}")
    
    def display_camera_view(self):
        """카메라 뷰 표시"""
        try:
            with self.camera_lock:
                if self.camera_image is not None:
                    # 바운딩 박스 감지 및 표시
                    display_image = self.camera_image.copy()
                    
                    if self.enable_bounding_boxes and self.bounding_box_detector:
                        # 바운딩 박스 감지
                        detections = self.bounding_box_detector.detect_pedestrians_and_vehicles()
                        print(f"🔍 Detected {len(detections)} objects for drawing")
                        
                        # 이미지에 바운딩 박스 그리기
                        display_image = self.bounding_box_detector.draw_bounding_boxes_on_image(display_image)
                        print(f"🎨 Bounding boxes drawn on image")
                        
                        # Zenoh로 데이터 전송
                        if self.enable_zenoh and self.zenoh_publisher:
                            zenoh_data = self.bounding_box_detector.get_detection_data_for_zenoh()
                            print(f"🌐 Publishing to Zenoh: {len(zenoh_data)} objects")
                            if len(zenoh_data) > 0:
                                print(f"🌐 Zenoh data sample: {zenoh_data[0] if zenoh_data else 'None'}")
                            self.zenoh_publisher.publish_bounding_boxes(zenoh_data)
                        else:
                            print(f"⚠️ Zenoh not enabled or publisher not available: enable_zenoh={self.enable_zenoh}, publisher={self.zenoh_publisher is not None}")
                        
                        # HUD에 탐지 정보 추가
                        self.set_hud_info(
                            Detected_Objects=f"{len(detections)}",
                            Vehicles=f"{len([d for d in detections if d['type'] == 'vehicle'])}",
                            Pedestrians=f"{len([d for d in detections if d['type'] == 'pedestrian'])}"
                        )
                    
                    # OpenCV 이미지를 Pygame 형식으로 변환
                    image_rgb = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
                    image_surface = pygame.surfarray.make_surface(image_rgb.swapaxes(0, 1))
                    
                    # 화면에 표시
                    self.screen.blit(image_surface, (0, 0))
                    
                    # HUD 정보 표시
                    if self.show_hud:
                        self.draw_hud()
                    
                    pygame.display.flip()
                    
        except Exception as e:
            print(f"⚠️ Error displaying camera view: {e}")
    
    def draw_hud(self):
        """HUD 정보 표시"""
        try:
            if self.vehicle is None:
                return
            
            # 차량 정보 가져오기
            location = self.vehicle.get_location()
            velocity = self.vehicle.get_velocity()
            speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6  # km/h
            
            # 폰트 설정
            font = pygame.font.Font(None, 36)
            
            # 기본 정보
            basic_info = [
                f"Location: ({location.x:.1f}, {location.y:.1f}, {location.z:.1f})",
                f"Speed: {speed:.1f} km/h",
                f"Altitude: {location.z:.1f} m"
            ]
            
            # 사용자 정의 정보 추가
            custom_info = []
            for key, value in self.hud_info.items():
                custom_info.append(f"{key}: {value}")
            
            # 모든 정보 합치기
            all_info = basic_info + custom_info
            
            # 텍스트 표시
            y_offset = 10
            for text in all_info:
                text_surface = font.render(text, True, (255, 255, 255))
                # 반투명 배경
                pygame.draw.rect(self.screen, (0, 0, 0, 128), (5, y_offset-5, text_surface.get_width()+10, text_surface.get_height()+5))
                self.screen.blit(text_surface, (10, y_offset))
                y_offset += 40
            
            # 조작 안내
            control_texts = [
                "Press ESC to exit"
            ]
            
            y_offset = 200
            for text in control_texts:
                text_surface = font.render(text, True, (255, 255, 0))
                pygame.draw.rect(self.screen, (0, 0, 0, 128), (5, y_offset-5, text_surface.get_width()+10, text_surface.get_height()+5))
                self.screen.blit(text_surface, (10, y_offset))
                y_offset += 30
                
        except Exception as e:
            print(f"⚠️ Error drawing HUD: {e}")
    
    def handle_input(self):
        """입력 처리 (카메라 뷰 전용)"""
        try:
            # 카메라 뷰 전용 입력만 처리 (pygame.event.get() 사용하지 않음)
            # 실제 입력 처리는 메인 스크립트에서 담당
            return True
            
        except Exception as e:
            print(f"⚠️ Error handling input: {e}")
            return True
    
    def set_hud_info(self, **kwargs):
        """HUD 정보 설정"""
        self.hud_info.update(kwargs)
    
    def clear_hud_info(self):
        """HUD 정보 초기화"""
        self.hud_info.clear()
    
    def get_camera_image(self):
        """현재 카메라 이미지 반환"""
        with self.camera_lock:
            return self.camera_image.copy() if self.camera_image is not None else None
    
    def is_camera_ready(self):
        """카메라가 준비되었는지 확인"""
        return self.camera is not None and self.camera_image is not None
    
    def cleanup(self):
        """리소스 정리"""
        try:
            # Zenoh 퍼블리셔 정리
            if self.zenoh_publisher:
                self.zenoh_publisher.stop_publishing()
                self.zenoh_publisher.disconnect()
                self.zenoh_publisher = None
            
            # 바운딩 박스 감지기 정리
            if self.bounding_box_detector:
                self.bounding_box_detector.cleanup()
                self.bounding_box_detector = None
            
            # 카메라 정리
            if self.camera:
                self.camera.destroy()
                self.camera = None
            
            # Pygame 정리
            pygame.quit()
            
            print("📷 Camera view cleaned up")
            
        except Exception as e:
            print(f"⚠️ Error during camera cleanup: {e}")
    
    def get_bounding_box_detector(self):
        """바운딩 박스 감지기 반환"""
        return self.bounding_box_detector
    
    def get_zenoh_publisher(self):
        """Zenoh 퍼블리셔 반환"""
        return self.zenoh_publisher
    
    def toggle_bounding_boxes(self):
        """바운딩 박스 표시 토글"""
        self.enable_bounding_boxes = not self.enable_bounding_boxes
        print(f"🎯 Bounding boxes: {'Enabled' if self.enable_bounding_boxes else 'Disabled'}")
    
    def toggle_zenoh(self):
        """Zenoh 전송 토글"""
        self.enable_zenoh = not self.enable_zenoh
        print(f"🌐 Zenoh: {'Enabled' if self.enable_zenoh else 'Disabled'}")
