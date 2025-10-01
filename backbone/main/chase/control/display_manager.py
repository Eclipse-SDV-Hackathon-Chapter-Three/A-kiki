#!/usr/bin/env python3
"""
Display Manager
디스플레이 관리 클래스
"""

import time
import pygame

class DisplayManager:
    """디스플레이 관리 클래스"""
    
    def __init__(self):
        self.last_display_time = 0.0
        self.display_interval = 0.033  # 30 FPS
        self.font = None
        self.small_font = None
        
        # Pygame 초기화
        self._init_pygame()
    
    def _init_pygame(self):
        """Pygame 초기화"""
        try:
            pygame.init()
            pygame.font.init()
            
            # 폰트 설정
            self.font = pygame.font.Font(None, 36)
            self.small_font = pygame.font.Font(None, 24)
            
            print("✅ Pygame initialized for display")
        except Exception as e:
            print(f"⚠️ Error initializing pygame: {e}")
    
    def display_camera_view(self, camera_view, zenoh_image, detected_objects, statistics, 
                           lidar_visualizer=None, sensor_fusion=None, fusion_display=None):
        """카메라 뷰 표시 (LiDAR 오버레이 및 센서 퓨전 정보 포함)"""
        try:
            current_time = time.time()
            
            # 디스플레이 간격 체크
            if current_time - self.last_display_time < self.display_interval:
                return
            
            self.last_display_time = current_time
            
            # Zenoh 카메라 이미지가 있으면 표시
            if zenoh_image is not None:
                self._display_zenoh_camera_view(zenoh_image, detected_objects, statistics, 
                                              lidar_visualizer, sensor_fusion, fusion_display)
            elif camera_view:
                # CARLA 카메라 뷰 표시
                camera_view.display()
            
        except Exception as e:
            print(f"⚠️ Error displaying camera view: {e}")
    
    def _display_zenoh_camera_view(self, image, detected_objects, statistics, 
                                  lidar_visualizer=None, sensor_fusion=None, fusion_display=None):
        """Zenoh 카메라 뷰 표시 (LiDAR 오버레이 및 센서 퓨전 정보 포함)"""
        try:
            if image is None:
                return
            
            # 이미지 크기 확인
            height, width = image.shape[:2]
            print(f"📷 Image size: {width}x{height}")
            
            # Pygame 화면 크기
            screen_width, screen_height = 1080, 720
            print(f"📺 Screen size: {screen_width}x{screen_height}")
            
            # 이미지 크기가 다르면 리사이즈
            if width != screen_width or height != screen_height:
                import cv2
                image = cv2.resize(image, (screen_width, screen_height))
                print(f"📷 Resized image to: {screen_width}x{screen_height}")
            
            # OpenCV 이미지를 Pygame Surface로 변환
            image_rgb = image[:, :, ::-1]  # BGR to RGB
            surface = pygame.surfarray.make_surface(image_rgb.swapaxes(0, 1))
            
            # 화면 생성 (한 번만)
            if not hasattr(self, 'screen'):
                self.screen = pygame.display.set_mode((screen_width, screen_height))
                pygame.display.set_caption("Auto Chase Vehicle Control - Zenoh Camera + LiDAR")
                print(f"✅ Pygame screen initialized: {screen_width}x{screen_height}")
            
            # 화면에 이미지 그리기
            self.screen.blit(surface, (0, 0))
            
            # 바운딩 박스 그리기
            self._draw_bounding_boxes(detected_objects)
            
            # LiDAR 오버레이 그리기
            if lidar_visualizer:
                lidar_visualizer.draw_overlay(self.screen)
            
            # 센서 퓨전 정보 표시
            if sensor_fusion and fusion_display:
                fused_objects = getattr(sensor_fusion, 'fused_objects', [])
                fusion_display.draw_fusion_info(self.screen, fused_objects)
                fusion_display.draw_lidar_class_legend(self.screen)
                
                # 센서 상태 표시
                camera_active = True  # 카메라는 활성화되어 있음
                lidar_active = lidar_visualizer is not None
                fusion_display.draw_sensor_status(self.screen, camera_active, lidar_active)
            
            # HUD 그리기
            self._draw_hud(statistics)
            
            # 화면 업데이트
            pygame.display.flip()
            
            # 이벤트 처리
            self._handle_events()
            
        except Exception as e:
            print(f"⚠️ Error displaying Zenoh camera view: {e}")
    
    def _draw_bounding_boxes(self, detected_objects):
        """바운딩 박스 그리기"""
        try:
            print(f"🎯 _draw_bounding_boxes called with {len(detected_objects) if detected_objects else 0} objects")
            if not detected_objects:
                print("⚠️ No detected objects to draw")
                return
            
            for obj in detected_objects:
                bbox = obj.get('bbox_2d', {})
                if not bbox:
                    continue
                
                # 바운딩 박스 좌표
                x_min = int(bbox.get('x_min', 0))
                y_min = int(bbox.get('y_min', 0))
                x_max = int(bbox.get('x_max', 0))
                y_max = int(bbox.get('y_max', 0))
                
                # 화면 경계 체크 및 클리핑
                if x_min < 0 or y_min < 0 or x_max < 0 or y_max < 0:
                    continue
                
                # 화면 경계로 클리핑
                x_min = max(0, min(x_min, 1080))
                y_min = max(0, min(y_min, 720))
                x_max = max(0, min(x_max, 1080))
                y_max = max(0, min(y_max, 720))
                
                # 유효한 바운딩 박스인지 확인
                if x_max <= x_min or y_max <= y_min:
                    continue
                
                # 객체 타입에 따른 색상
                obj_type = obj.get('type', 'unknown')
                if obj_type == 'vehicle':
                    color = (255, 0, 0)  # 빨간색
                elif obj_type == 'pedestrian':
                    color = (0, 255, 0)  # 초록색
                else:
                    color = (255, 255, 0)  # 노란색
                
                # 바운딩 박스 그리기
                width = x_max - x_min
                height = y_max - y_min
                pygame.draw.rect(self.screen, color, (x_min, y_min, width, height), 2)
                
                # 디버그 정보 출력
                print(f"🎯 Drawing bbox: ({x_min}, {y_min}) - ({x_max}, {y_max}) size: {width}x{height}")
                
                # 라벨 그리기
                label = f"{obj_type} ({obj.get('actor_id', 'Unknown')})"
                if self.small_font:
                    text_surface = self.small_font.render(label, True, color)
                    # 라벨 위치 조정 (화면 밖으로 나가지 않도록)
                    label_x = max(0, min(x_min, 1080 - text_surface.get_width()))
                    label_y = max(0, min(y_min - 20, 720 - text_surface.get_height()))
                    self.screen.blit(text_surface, (label_x, label_y))
                
        except Exception as e:
            print(f"⚠️ Error drawing bounding boxes: {e}")
    
    def _draw_hud(self, statistics):
        """HUD 그리기"""
        try:
            if not self.font or not self.small_font:
                return
            
            # 배경 그리기
            hud_rect = pygame.Rect(10, 10, 300, 150)
            pygame.draw.rect(self.screen, (0, 0, 0, 128), hud_rect)
            pygame.draw.rect(self.screen, (255, 255, 255), hud_rect, 2)
            
            # 통계 정보 표시
            y_offset = 20
            line_height = 20
            
            # 제목
            title = self.font.render("Auto Chase Control", True, (255, 255, 255))
            self.screen.blit(title, (20, y_offset))
            y_offset += line_height + 10
            
            # 감지된 객체 수
            detections = statistics.get('total_detections', 0)
            detections_text = self.small_font.render(f"Detections: {detections}", True, (255, 255, 255))
            self.screen.blit(detections_text, (20, y_offset))
            y_offset += line_height
            
            # 충돌 이벤트 수
            collisions = statistics.get('collision_events', 0)
            collisions_text = self.small_font.render(f"Collisions: {collisions}", True, (255, 0, 0))
            self.screen.blit(collisions_text, (20, y_offset))
            y_offset += line_height
            
            # 추격 시간
            chase_duration = statistics.get('chase_duration', 0.0)
            duration_text = self.small_font.render(f"Chase Time: {chase_duration:.1f}s", True, (255, 255, 255))
            self.screen.blit(duration_text, (20, y_offset))
            y_offset += line_height
            
            # 최대 속도
            max_speed = statistics.get('max_speed_reached', 0.0)
            speed_text = self.small_font.render(f"Max Speed: {max_speed:.1f} m/s", True, (255, 255, 255))
            self.screen.blit(speed_text, (20, y_offset))
            y_offset += line_height
            
            # 평균 거리
            avg_distance = statistics.get('average_distance', 0.0)
            distance_text = self.small_font.render(f"Avg Distance: {avg_distance:.1f}m", True, (255, 255, 255))
            self.screen.blit(distance_text, (20, y_offset))
            
        except Exception as e:
            print(f"⚠️ Error drawing HUD: {e}")
    
    def _handle_events(self):
        """이벤트 처리"""
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        return False
            return True
        except Exception as e:
            print(f"⚠️ Error handling events: {e}")
            return True
    
    def cleanup(self):
        """디스플레이 정리"""
        try:
            if hasattr(self, 'screen'):
                pygame.display.quit()
                self.screen = None
            print("✅ Display manager cleaned up")
        except Exception as e:
            print(f"⚠️ Error cleaning up display: {e}")
