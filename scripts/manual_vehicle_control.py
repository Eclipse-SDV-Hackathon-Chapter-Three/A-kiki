#!/usr/bin/env python3
"""
Manual Vehicle Control Script
수동으로 조종하는 차량을 위한 별도 스크립트
"""

import sys
import os
import carla
import pygame
import numpy as np
import cv2
import threading
import time

# 상위 디렉토리의 모듈들을 import하기 위해 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from vehicle import VehicleController, CameraView

class ManualVehicleControl:
    """수동 차량 제어 클래스"""
    
    def __init__(self):
        self.client = None
        self.world = None
        self.vehicle = None
        self.vehicle_controller = None
        self.camera_view = None
        
        # 제어 상태
        self.running = True
        self.control_keys = {
            'throttle': False,
            'brake': False,
            'steer_left': False,
            'steer_right': False
        }
        
        print("🚗 Manual Vehicle Control initialized")
    
    def connect_to_carla(self, host='localhost', port=2000):
        """CARLA 서버에 연결"""
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            
            print(f"✅ Connected to CARLA server at {host}:{port}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to connect to CARLA: {e}")
            return False
    
    def spawn_vehicle(self, spawn_point=None):
        """차량 스폰"""
        try:
            # 차량 컨트롤러 초기화
            self.vehicle_controller = VehicleController(self.world)
            
            # 원래 스폰 위치 사용
            if spawn_point is None:
                spawn_point = carla.Transform(
                    carla.Location(-104.15, 46.15, 1.0),
                    carla.Rotation(yaw=270.0)
                )
            
            # 차량 스폰
            self.vehicle = self.vehicle_controller.spawn_vehicle()
            if self.vehicle is None:
                print("❌ Failed to spawn vehicle")
                return False
            
            print(f"🚗 Vehicle spawned at {spawn_point.location}")
            return True
            
        except Exception as e:
            print(f"❌ Error spawning vehicle: {e}")
            return False
    
    def setup_camera(self):
        """카메라 설정"""
        try:
            # 공통 카메라 뷰 모듈 사용 (바운딩 박스와 Zenoh 비활성화)
            self.camera_view = CameraView(
                self.world, 
                self.vehicle, 
                "Manual Vehicle Control",
                enable_bounding_boxes=False,  # 수동 차량에서는 바운딩 박스 비활성화
                enable_zenoh=False           # 수동 차량에서는 Zenoh 비활성화
            )
            
            # 카메라 설정 (chase_vehicle_control.py와 동일한 설정 사용)
            camera_location = carla.Location(x=1.5, z=1.4)
            camera_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
            
            if self.camera_view.setup_camera(camera_location, camera_rotation):
                print("📷 Camera attached to manual vehicle")
                return True
            else:
                return False
            
        except Exception as e:
            print(f"❌ Error setting up camera: {e}")
            return False
    
    
    def handle_input(self):
        """입력 처리"""
        try:
            # 키보드 입력 처리
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                    elif event.key == pygame.K_w:
                        self.control_keys['throttle'] = True
                    elif event.key == pygame.K_s:
                        self.control_keys['brake'] = True
                    elif event.key == pygame.K_a:
                        self.control_keys['steer_left'] = True
                    elif event.key == pygame.K_d:
                        self.control_keys['steer_right'] = True
                    elif event.key == pygame.K_h:  # HUD 토글
                        if self.camera_view:
                            self.camera_view.show_hud = not self.camera_view.show_hud
                            print(f"HUD {'enabled' if self.camera_view.show_hud else 'disabled'}")
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_w:
                        self.control_keys['throttle'] = False
                    elif event.key == pygame.K_s:
                        self.control_keys['brake'] = False
                    elif event.key == pygame.K_a:
                        self.control_keys['steer_left'] = False
                    elif event.key == pygame.K_d:
                        self.control_keys['steer_right'] = False
                        
        except Exception as e:
            print(f"⚠️ Error handling input: {e}")
    
    def apply_control(self):
        """제어 적용"""
        try:
            if self.vehicle is None:
                return
            
            # 제어 계산
            throttle = 0.0
            brake = 0.0
            steer = 0.0
            
            if self.control_keys['throttle']:
                throttle = 1.0
            if self.control_keys['brake']:
                brake = 1.0
            if self.control_keys['steer_left']:
                steer = -1.0
            elif self.control_keys['steer_right']:
                steer = 1.0
            
            # 차량 제어 적용
            control = carla.VehicleControl(
                throttle=throttle,
                brake=brake,
                steer=steer
            )
            self.vehicle.apply_control(control)
            
        except Exception as e:
            print(f"⚠️ Error applying control: {e}")
    
    def run(self):
        """메인 실행 루프"""
        try:
            print("🚗 Starting Manual Vehicle Control...")
            
            # CARLA 연결
            if not self.connect_to_carla():
                return
            
            # 차량 스폰
            if not self.spawn_vehicle():
                return
            
            # 카메라 설정
            if not self.setup_camera():
                return
            
            print("✅ Manual vehicle control ready!")
            print("🎮 Use WASD keys to control the vehicle")
            print("📷 Camera view will be displayed")
            print("🛑 Press ESC to exit")
            
            # 메인 루프
            while self.running:
                # 입력 처리
                self.handle_input()
                
                # 제어 적용
                self.apply_control()
                
                # 카메라 뷰 표시
                if self.camera_view:
                    self.camera_view.display_camera_view()
                
                # FPS 제한
                if self.camera_view:
                    self.camera_view.clock.tick(60)
            
        except KeyboardInterrupt:
            print("\n🛑 Manual vehicle control interrupted by user")
        except Exception as e:
            print(f"❌ Error in main loop: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """리소스 정리"""
        try:
            print("🧹 Cleaning up resources...")
            
            # 카메라 뷰 정리
            if self.camera_view:
                self.camera_view.cleanup()
                self.camera_view = None
            
            # 차량 정리
            if self.vehicle:
                self.vehicle.destroy()
                self.vehicle = None
            
            print("✅ Cleanup completed")
            
        except Exception as e:
            print(f"⚠️ Error during cleanup: {e}")

def main():
    """메인 함수"""
    try:
        manual_control = ManualVehicleControl()
        manual_control.run()
    except Exception as e:
        print(f"❌ Fatal error: {e}")

if __name__ == "__main__":
    main()
