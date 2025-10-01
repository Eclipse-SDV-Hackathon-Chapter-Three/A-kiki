#!/usr/bin/env python3
"""
Fusion Display Manager
센서 퓨전 결과를 화면에 표시하는 기능을 제공합니다.
"""

import pygame
from typing import List, Dict, Optional


class FusionDisplayManager:
    """퓨전 정보 화면 표시 관리자"""
    
    def __init__(self):
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 20)
        self.show_fusion_info = True
        
        # 색상 정의
        self.colors = {
            'fused': (0, 255, 0),      # 녹색 - 퓨전 성공
            'camera_only': (255, 255, 0),  # 노란색 - 카메라만
            'lidar_only': (0, 255, 255),   # 청록색 - LiDAR만
            'background': (0, 0, 0, 128),  # 반투명 검은색
            'text': (255, 255, 255),       # 흰색 텍스트
            'border': (100, 100, 100)      # 회색 테두리
        }
    
    def draw_fusion_info(self, screen: pygame.Surface, fused_objects: List[Dict]) -> None:
        """퓨전된 객체 정보를 화면에 표시"""
        try:
            if not self.show_fusion_info or not fused_objects:
                return
            
            # 정보 패널 그리기
            self._draw_info_panel(screen, fused_objects)
            
            # 객체별 상세 정보 표시
            self._draw_object_details(screen, fused_objects)
            
        except Exception as e:
            print(f"⚠️ Error drawing fusion info: {e}")
    
    def _draw_info_panel(self, screen: pygame.Surface, fused_objects: List[Dict]) -> None:
        """퓨전 정보 패널 그리기"""
        try:
            # 통계 계산
            total_objects = len(fused_objects)
            fused_count = sum(1 for obj in fused_objects if obj.get('type') == 'fused')
            camera_only_count = sum(1 for obj in fused_objects if obj.get('type') == 'camera_only')
            
            # 패널 크기 및 위치
            panel_width = 300
            panel_height = 120
            panel_x = screen.get_width() - panel_width - 10
            panel_y = 10
            
            # 배경 그리기
            panel_surface = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
            panel_surface.fill(self.colors['background'])
            
            # 테두리 그리기
            pygame.draw.rect(panel_surface, self.colors['border'], 
                           (0, 0, panel_width, panel_height), 2)
            
            # 텍스트 그리기
            y_offset = 10
            
            # 제목
            title_text = self.font.render("센서 퓨전 정보", True, self.colors['text'])
            panel_surface.blit(title_text, (10, y_offset))
            y_offset += 30
            
            # 총 객체 수
            total_text = self.small_font.render(f"총 객체: {total_objects}개", True, self.colors['text'])
            panel_surface.blit(total_text, (10, y_offset))
            y_offset += 20
            
            # 퓨전 성공
            fused_text = self.small_font.render(f"퓨전 성공: {fused_count}개", True, self.colors['fused'])
            panel_surface.blit(fused_text, (10, y_offset))
            y_offset += 20
            
            # 카메라만
            camera_text = self.small_font.render(f"카메라만: {camera_only_count}개", True, self.colors['camera_only'])
            panel_surface.blit(camera_text, (10, y_offset))
            y_offset += 20
            
            # 퓨전률
            fusion_rate = fused_count / max(total_objects, 1) * 100
            rate_text = self.small_font.render(f"퓨전률: {fusion_rate:.1f}%", True, self.colors['text'])
            panel_surface.blit(rate_text, (10, y_offset))
            
            # 화면에 패널 그리기
            screen.blit(panel_surface, (panel_x, panel_y))
            
        except Exception as e:
            print(f"⚠️ Error drawing info panel: {e}")
    
    def _draw_object_details(self, screen: pygame.Surface, fused_objects: List[Dict]) -> None:
        """객체별 상세 정보 표시"""
        try:
            # 최대 5개 객체만 표시
            display_objects = fused_objects[:5]
            
            # 객체 정보 패널
            panel_width = 400
            panel_height = len(display_objects) * 30 + 40
            panel_x = 10
            panel_y = 10
            
            # 배경 그리기
            panel_surface = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
            panel_surface.fill(self.colors['background'])
            
            # 테두리 그리기
            pygame.draw.rect(panel_surface, self.colors['border'], 
                           (0, 0, panel_width, panel_height), 2)
            
            # 제목
            title_text = self.font.render("객체 상세 정보", True, self.colors['text'])
            panel_surface.blit(title_text, (10, 10))
            
            # 객체별 정보
            y_offset = 40
            for i, obj in enumerate(display_objects):
                obj_type = obj.get('type', 'unknown')
                semantic_class = obj.get('semantic_class', 'unknown')
                confidence = obj.get('confidence', 0.0)
                distance = obj.get('distance', 0.0)
                
                # 텍스트 색상
                if obj_type == 'fused':
                    color = self.colors['fused']
                else:
                    color = self.colors['camera_only']
                
                # 객체 정보 텍스트
                text = f"{i+1}. {semantic_class} ({obj_type}) - 신뢰도: {confidence:.2f} - 거리: {distance:.1f}m"
                text_surface = self.small_font.render(text, True, color)
                panel_surface.blit(text_surface, (10, y_offset))
                y_offset += 25
            
            # 화면에 패널 그리기
            screen.blit(panel_surface, (panel_x, panel_y))
            
        except Exception as e:
            print(f"⚠️ Error drawing object details: {e}")
    
    def draw_lidar_class_legend(self, screen: pygame.Surface) -> None:
        """LiDAR 클래스 범례 표시"""
        try:
            # 범례 패널
            panel_width = 200
            panel_height = 120
            panel_x = screen.get_width() - panel_width - 10
            panel_y = screen.get_height() - panel_height - 10
            
            # 배경 그리기
            panel_surface = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
            panel_surface.fill(self.colors['background'])
            
            # 테두리 그리기
            pygame.draw.rect(panel_surface, self.colors['border'], 
                           (0, 0, panel_width, panel_height), 2)
            
            # 제목
            title_text = self.font.render("LiDAR 클래스", True, self.colors['text'])
            panel_surface.blit(title_text, (10, 10))
            
            # 클래스별 범례
            y_offset = 40
            class_info = [
                ("차량", (255, 0, 0)),
                ("차선", (0, 255, 0)),
                ("보행자", (0, 0, 255))
            ]
            
            for class_name, color in class_info:
                # 색상 원 그리기
                pygame.draw.circle(panel_surface, color, (20, y_offset), 5)
                
                # 클래스 이름
                text = self.small_font.render(class_name, True, self.colors['text'])
                panel_surface.blit(text, (35, y_offset - 8))
                y_offset += 25
            
            # 화면에 패널 그리기
            screen.blit(panel_surface, (panel_x, panel_y))
            
        except Exception as e:
            print(f"⚠️ Error drawing LiDAR legend: {e}")
    
    def toggle_fusion_info(self) -> None:
        """퓨전 정보 표시 토글"""
        self.show_fusion_info = not self.show_fusion_info
        print(f"🔗 퓨전 정보 표시: {'ON' if self.show_fusion_info else 'OFF'}")
    
    def draw_sensor_status(self, screen: pygame.Surface, camera_active: bool, lidar_active: bool) -> None:
        """센서 상태 표시"""
        try:
            # 상태 패널
            panel_width = 150
            panel_height = 80
            panel_x = 10
            panel_y = screen.get_height() - panel_height - 10
            
            # 배경 그리기
            panel_surface = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
            panel_surface.fill(self.colors['background'])
            
            # 테두리 그리기
            pygame.draw.rect(panel_surface, self.colors['border'], 
                           (0, 0, panel_width, panel_height), 2)
            
            # 제목
            title_text = self.font.render("센서 상태", True, self.colors['text'])
            panel_surface.blit(title_text, (10, 10))
            
            # 카메라 상태
            camera_color = self.colors['fused'] if camera_active else (255, 0, 0)
            camera_text = self.small_font.render(f"카메라: {'ON' if camera_active else 'OFF'}", 
                                               True, camera_color)
            panel_surface.blit(camera_text, (10, 35))
            
            # LiDAR 상태
            lidar_color = self.colors['fused'] if lidar_active else (255, 0, 0)
            lidar_text = self.small_font.render(f"LiDAR: {'ON' if lidar_active else 'OFF'}", 
                                              True, lidar_color)
            panel_surface.blit(lidar_text, (10, 55))
            
            # 화면에 패널 그리기
            screen.blit(panel_surface, (panel_x, panel_y))
            
        except Exception as e:
            print(f"⚠️ Error drawing sensor status: {e}")


