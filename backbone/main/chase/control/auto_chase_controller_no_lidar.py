#!/usr/bin/env python3
"""
Auto Chase Vehicle Controller (No LiDAR)
자동 추격 차량 제어 메인 클래스 - LiDAR 제거 버전
"""

import sys
import os
import carla
import numpy as np
import cv2
import threading
import time
import pygame
import json
import base64
import signal
import atexit
from typing import Optional, List, Callable, Any, Dict

# Zenoh imports
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    print("Warning: zenoh not available, control commands will not be published")

# 상위 디렉토리의 모듈들을 import하기 위해 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, parent_dir)

from chase.perception.bounding_box_detector import BoundingBoxDetector
from chase.perception.collision_detector import CollisionDetector
from chase.perception.collision_vehicle_tracker import CollisionVehicleTracker
from chase.perception.collision_tracker import CollisionTracker
from chase.perception.unified_collision_detector import UnifiedCollisionDetector, CollisionEvent
from chase.planning.simple_chase_planner import SimpleChasePlanner
from chase.communication.zenoh_collision_manager import ZenohCollisionManager
from chase.communication.zero_copy_camera_subscriber import ZeroCopyCameraSubscriber
from chase.control.vehicle_manager import VehicleManager
from chase.control.camera_manager import CameraManager
from chase.control.sensor_manager import SensorManager
from chase.control.chase_controller import ChaseController
from chase.control.display_manager import DisplayManager
from vehicle.optimized_camera_view import OptimizedCameraView

# 전역 변수 (정리용)
_auto_chase_instance = None

def cleanup_opencv_windows():
    """OpenCV 윈도우 정리 함수"""
    try:
        print("🧹 Cleaning up OpenCV windows...")
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        print("✅ OpenCV windows cleaned up")
    except Exception as e:
        print(f"⚠️ Error cleaning up OpenCV windows: {e}")

def signal_handler(signum, frame):
    """시그널 핸들러"""
    print(f"\n🛑 Received signal {signum}, cleaning up...")
    cleanup_opencv_windows()
    if _auto_chase_instance:
        _auto_chase_instance.cleanup()
    sys.exit(0)

# 시그널 핸들러 등록
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# atexit 핸들러 등록
atexit.register(cleanup_opencv_windows)

class AutoChaseVehicleControl:
    """자동 추격 차량 제어 클래스 (LiDAR 제거 버전)"""
    
    def __init__(self):
        self.client = None
        self.world = None
        self.vehicle = None
        
        # 모듈들
        self.bounding_box_detector = None
        self.collision_detector = None
        self.vehicle_tracker = None
        self.chase_planner = None
        self.collision_tracker = None
        self.zenoh_collision_manager = None
        self.unified_collision_detector = None
        
        # 매니저들
        self.vehicle_manager = None
        self.camera_manager = None
        self.sensor_manager = None
        self.chase_controller = None
        self.display_manager = None
        
        # 카메라 뷰
        self.camera_view = None
        
        # 제어 상태
        self.running = True
        self.is_chasing = False
        self.last_update_time = 0.0
        self.update_interval = 0.05  # 50ms 업데이트 간격 (더 빠른 반응)
        
        # IMU 센서
        self.imu_sensor = None
        self.current_imu_data = None
        self.vehicle_position = None  # 현재 차량 위치 (x, y, z)
        self.vehicle_orientation = None  # 현재 차량 방향 (pitch, yaw, roll)
        self.vehicle_velocity = None  # 현재 차량 속도
        
        # 모듈화된 컴포넌트들 (LiDAR 제거)
        self.sensor_fusion = None
        self.fusion_display = None
        
        # 통계
        self.chase_statistics = {
            'total_detections': 0,
            'collision_events': 0,
            'chase_duration': 0.0,
            'max_speed_reached': 0.0,
            'average_distance': 0.0
        }
        
        # Zenoh 관련 변수 (새로운 모듈로 대체됨)
        self.zenoh_camera_image = None
        self.zenoh_detected_objects = []
        self.use_zenoh_camera_only = False  # CARLA 카메라도 함께 사용 (바운딩 박스 감지용)
        self.temp_camera = None  # 임시 카메라 (ground truth용)
        
        # Zero-Copy 카메라 구독자
        self.zero_copy_camera = None
        self.use_zero_copy = False  # Zero-Copy 사용 여부 (일단 Legacy 모드로)
        
        # 추적 관련 변수
        self.target_vehicle = None  # 추적 대상 차량 (사고 차량)
        self.vehicle_tracking_enabled = True  # 차량 추적 활성화
        self.accident_detection_enabled = True  # 사고 감지 활성화
        
        # 전역 인스턴스 설정
        global _auto_chase_instance
        _auto_chase_instance = self
        
        print("🚔 Auto Chase Vehicle Control initialized (No LiDAR)")
    
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
    
    def setup_zenoh(self):
        """Zenoh 설정 (새로운 모듈 사용)"""
        try:
            # Zenoh 충돌 관리자 초기화
            self.zenoh_collision_manager = ZenohCollisionManager()
            
            # 콜백 함수 설정
            self.zenoh_collision_manager.set_camera_callback(self.on_zenoh_camera_received)
            self.zenoh_collision_manager.set_bbox_callback(self.on_zenoh_bounding_boxes_received)
            
            # Zenoh 연결
            if self.zenoh_collision_manager.setup_zenoh():
                print("✅ Zenoh collision manager setup successful")
                return True
            else:
                print("❌ Failed to setup Zenoh collision manager")
                return False

        except Exception as e:
            print(f"❌ Failed to setup Zenoh: {e}")
            self.zenoh_collision_manager = None
            return False
    
    def on_zenoh_camera_received(self, image):
        """Zenoh 카메라 이미지 수신 콜백"""
        try:
            if image is not None:
                self.zenoh_camera_image = image
        except Exception:
            pass  # 조용히 무시
    
    def on_zenoh_bounding_boxes_received(self, objects):
        """Zenoh 바운딩 박스 수신 콜백"""
        try:
            self.zenoh_detected_objects = objects
            
            # 충돌 추적 시스템으로 처리
            if self.collision_tracker and objects:
                self.process_zenoh_objects(objects)
                
        except Exception:
            pass  # 조용히 무시
    
    def process_zenoh_objects(self, objects):
        """Zenoh 객체들을 통합 충돌 감지 시스템으로 처리"""
        try:
            if not self.unified_collision_detector:
                return
            
            print(f"🔍 Processing {len(objects)} objects from Zenoh")
            
            # 통합 충돌 감지기로 객체 처리
            collision_events = self.unified_collision_detector.process_objects(objects)
            
            # 충돌 이벤트가 있으면 출력
            if collision_events:
                print(f"🚨 {len(collision_events)} collision events detected!")
                for event in collision_events:
                    print(f"   - {event.event_type}: {event.description}")
                
                # 기존 충돌 추적 시스템에도 전달 (호환성)
                if self.collision_tracker:
                    for event in collision_events:
                        # 기존 형식으로 변환
                        legacy_event = {
                            'type': event.event_type,
                            'description': event.description,
                            'actor_id': event.actor_id,
                            'world_location': event.world_location,
                            'severity': event.severity,
                            'timestamp': event.timestamp
                        }
                        if hasattr(self.collision_tracker, 'add_collision_event'):
                            self.collision_tracker.add_collision_event(legacy_event)
                
        except Exception as e:
            print(f"⚠️ Error processing Zenoh objects: {e}")
    
    def _on_collision_detected(self, collision_event: CollisionEvent):
        """충돌 감지 콜백"""
        try:
            print(f"🚨 Collision detected: {collision_event.description}")
            print(f"   Type: {collision_event.event_type}")
            print(f"   Severity: {collision_event.severity}")
            print(f"   Actor ID: {collision_event.actor_id}")
            
            # 충돌 통계 업데이트
            self.chase_statistics['collision_events'] += 1
            
        except Exception as e:
            print(f"⚠️ Error handling collision event: {e}")
    
    def _on_chase_started(self, collision_vehicles: List[str]):
        """추격 시작 콜백"""
        try:
            if not self.is_chasing:
                print(f"🚔 Starting chase for vehicles: {collision_vehicles}")
                self.is_chasing = True
                self.chase_statistics['collision_events'] += 1
                print("✅ Chase started for collision vehicles")
            else:
                print("🚔 Chase already in progress")
                
        except Exception as e:
            print(f"⚠️ Error starting chase: {e}")
    
    def setup_modules(self):
        """모듈들 초기화"""
        try:
            # CARLA 바운딩 박스 감지기 활성화 (Zenoh와 함께 사용)
            print("🎯 Using CARLA bounding box detection alongside Zenoh")
            if self.camera_manager and self.camera_manager.camera_view and self.camera_manager.camera_view.bounding_box_detector:
                self.bounding_box_detector = self.camera_manager.camera_view.bounding_box_detector
                print("✅ CARLA bounding box detector activated")
            else:
                print("⚠️ CARLA bounding box detector not available")
                self.bounding_box_detector = None
            
            # 통합 충돌 감지기 초기화
            self.unified_collision_detector = UnifiedCollisionDetector()
            self.unified_collision_detector.set_collision_callback(self._on_collision_detected)
            self.unified_collision_detector.set_chase_callback(self._on_chase_started)
            print("🚨 Unified Collision Detector initialized")
            
            # 충돌 추적기 초기화 (호환성을 위해 유지)
            self.collision_tracker = CollisionTracker(max_tracking_time=300.0)
            print("🚨 Collision tracker initialized")
            
            # 충돌 감지기 초기화 (호환성을 위해 유지)
            self.collision_detector = CollisionDetector()
            print("🚨 Collision Detector initialized")
            
            # 차량 추적기 초기화
            self.vehicle_tracker = CollisionVehicleTracker(self.world)
            print("🎯 Vehicle tracker initialized")
            
            # 추격 계획기 초기화
            self.chase_planner = SimpleChasePlanner(self.world, self.vehicle)
            print("🚗 Simple Chase Planner initialized")
            
            print("✅ All modules initialized")
            return True
            
        except Exception as e:
            print(f"❌ Error initializing modules: {e}")
            return False
    
    def find_existing_vehicle(self):
        """기존에 존재하는 차량을 찾아서 제어"""
        try:
            # CARLA 월드의 모든 액터 가져오기
            actors = self.world.get_actors()
            vehicles = []
            
            # 모든 차량 찾기
            for actor in actors:
                if 'vehicle' in actor.type_id:
                    vehicles.append(actor)
                    print(f"🚗 Found vehicle: {actor.type_id} (ID: {actor.id}) at {actor.get_location()}")
            
            if not vehicles:
                print("❌ No existing vehicles found in CARLA world")
                return False
            
            # 첫 번째 차량을 chase vehicle로 사용
            self.vehicle = vehicles[0]
            print(f"✅ Using existing vehicle: {self.vehicle.type_id} (ID: {self.vehicle.id})")
            print(f"📍 Vehicle location: {self.vehicle.get_location()}")
            
            return True
            
        except Exception as e:
            print(f"❌ Error finding existing vehicle: {e}")
            return False
    
    def setup_camera(self):
        """추격차량 카메라 설정 - Zenoh 카메라만 사용"""
        try:
            # 최적화된 고성능 카메라 뷰 사용
            self.camera_view = OptimizedCameraView(
                self.world, 
                self.vehicle, 
                "Auto Chase Vehicle Camera View",
                enable_bounding_boxes=True,  # 바운딩 박스 활성화
                enable_zenoh=True           # Zenoh 활성화
            )
            
            # Zenoh 카메라만 사용하는 경우 CARLA 카메라는 설정하지 않음
            if self.use_zenoh_camera_only:
                print("📷 Using Zenoh camera only - skipping CARLA camera setup")
                # Pygame 초기화 (안전한 방식)
                try:
                    # 디스플레이 환경 확인
                    import os
                    if not os.environ.get('DISPLAY'):
                        print("⚠️ No DISPLAY environment variable found")
                        os.environ['DISPLAY'] = ':0'
                        print(f"📺 Set DISPLAY to {os.environ['DISPLAY']}")
                    
                    # Pygame 초기화 (소프트웨어 렌더링 모드)
                    os.environ['SDL_VIDEODRIVER'] = 'x11'
                    pygame.init()
                    
                    # 비디오 모드 확인
                    if pygame.display.get_init():
                        print("✅ Pygame video initialized successfully")
                    else:
                        print("⚠️ Pygame video initialization failed")
                    
                    # 폰트 초기화
                    pygame.font.init()
                    print("✅ Pygame initialized for Zenoh camera display")
                    return True
                    
                except Exception as e:
                    print(f"❌ Pygame initialization failed: {e}")
                    print("💡 Try running with: export DISPLAY=:0")
                    return False
            else:
                # 기존 CARLA 카메라 설정
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
    
    def setup_imu_sensor(self):
        """IMU 센서 설정"""
        try:
            # IMU 센서 블루프린트 가져오기
            imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
            if imu_bp is None:
                print("⚠️ IMU sensor blueprint not found")
                return False
            
            # IMU 센서 스폰
            imu_transform = carla.Transform()
            self.imu_sensor = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)
            
            # IMU 데이터 콜백 설정
            self.imu_sensor.listen(self.on_imu_data)
            
            print("🧭 IMU sensor attached")
            return True
            
        except Exception as e:
            print(f"❌ Error setting up IMU sensor: {e}")
            return False
    
    def on_imu_data(self, imu_data):
        """IMU 데이터 콜백"""
        try:
            self.current_imu_data = {
                'accelerometer': (imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z),
                'gyroscope': (imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z),
                'compass': imu_data.compass,
                'timestamp': imu_data.timestamp
            }
        except Exception as e:
            print(f"⚠️ Error processing IMU data: {e}")
    
    def display_camera_view(self):
        """카메라 뷰 표시 - Zenoh 카메라 데이터 우선"""
        try:
            if self.camera_view:
                # Zenoh 카메라 데이터가 있으면 우선 표시
                if self.zenoh_camera_image is not None:
                    # Zenoh 카메라 데이터를 OptimizedCameraView에 전달
                    self.camera_view.set_zenoh_camera_data(
                        self.zenoh_camera_image, 
                        self.zenoh_detected_objects
                    )
                
                # HUD 정보 업데이트
                hud_info = {
                    'Objects': f"{len(self.zenoh_detected_objects)} detected",
                    'Collisions': f"{self.chase_statistics['collision_events']} events",
                    'Chase Status': "CHASING" if self.is_chasing else "MONITORING"
                }
                
                # 충돌이 감지된 경우 추가 정보
                if self.chase_statistics['collision_events'] > 0:
                    hud_info['Status'] = "🚨 COLLISION DETECTED"
                else:
                    hud_info['Status'] = "✅ Monitoring"
                
                # Zenoh 카메라 데이터가 없을 때 상태 표시
                if self.zenoh_camera_image is None:
                    hud_info['Camera'] = "Waiting for Zenoh camera..."
                    hud_info['Status'] = "📡 Connecting to Zenoh"
                
                # HUD 정보 설정
                if hasattr(self.camera_view, 'set_hud_info'):
                    self.camera_view.set_hud_info(**hud_info)
                
                # 카메라 뷰 표시 (Zenoh 데이터가 없어도 화면 표시)
                self.camera_view.display_camera_view()
            else:
                print("⚠️ No camera view available")
            
        except Exception as e:
            print(f"⚠️ Error displaying camera view: {e}")
            import traceback
            traceback.print_exc()
    
    def run(self):
        """메인 실행 루프"""
        try:
            print("🚔 Starting Auto Chase Vehicle Control (No LiDAR)...")
            
            # CARLA 연결
            if not self.connect_to_carla():
                return False
            
            # Zenoh 설정
            if not self.setup_zenoh():
                print("⚠️ Zenoh setup failed, continuing without Zenoh")
            
            # 기존 차량 찾기
            if not self.find_existing_vehicle():
                return False
            
            # 카메라 설정
            if not self.setup_camera():
                return False
            
            # IMU 센서 설정
            self.setup_imu_sensor()
            
            # 모듈들 초기화
            if not self.setup_modules():
                return False
            
            print("✅ Auto chase vehicle control ready!")
            print("🎯 Monitoring for collisions...")
            print("🛑 Press ESC to exit")
            
            # 메인 루프
            while self.running:
                try:
                    # 입력 처리
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            self.running = False
                            break
                        elif event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_ESCAPE:
                                self.running = False
                                break
                    
                    if not self.running:
                        break
                    
                    current_time = time.time()
                    
                    # 업데이트 간격 체크
                    if current_time - self.last_update_time < self.update_interval:
                        time.sleep(0.01)  # 10ms 대기
                        continue
                    
                    # 1. 인식 업데이트 (바운딩 박스 기반)
                    detected_objects, collision_events = self.update_perception()
                    
                    # 2. 추적 업데이트
                    is_tracking = self.update_tracking(detected_objects, collision_events)
                    
                    # 3. 계획 및 제어 업데이트
                    self.update_planning_and_control()
                    
                    # 4. CARLA 바운딩 박스 데이터를 zenoh_detected_objects에 추가 (pygame 표시용)
                    if self.camera_view and self.camera_view.bounding_box_detector:
                        # CARLA 바운딩 박스 감지 실행 (감지 범위 확장)
                        carla_objects = self.camera_view.bounding_box_detector.detect_pedestrians_and_vehicles(max_distance=200.0)
                        print(f"🔍 CARLA detected objects: {len(carla_objects)}")
                        
                        if carla_objects:
                            # CARLA 바운딩 박스를 zenoh 형식으로 변환
                            self.zenoh_detected_objects = []
                            for obj in carla_objects:
                                print(f"🔍 CARLA object: {obj}")
                                zenoh_obj = {
                                    'type': obj.get('type', 'unknown'),
                                    'actor_id': obj.get('actor_id', 'Unknown'),
                                    'bbox_2d': obj.get('bbox_2d', {}),
                                    'distance': obj.get('distance', 100),
                                    'in_camera_view': True
                                }
                                self.zenoh_detected_objects.append(zenoh_obj)
                            print(f"✅ Added {len(self.zenoh_detected_objects)} objects to zenoh_detected_objects")
                        else:
                            print("⚠️ No CARLA objects detected")
                    
                    # 5. 카메라 뷰 표시
                    self.display_camera_view()
                    
                    self.last_update_time = current_time
                    
                except KeyboardInterrupt:
                    print("\n🛑 Auto chase vehicle control interrupted by user")
                    break
                except Exception as e:
                    print(f"⚠️ Error in main loop: {e}")
                    time.sleep(0.1)
            
        except Exception as e:
            print(f"❌ Error in main execution: {e}")
        finally:
            self.cleanup()
    
    def update_perception(self):
        """인식 모듈 업데이트"""
        try:
            if not self.bounding_box_detector:
                return [], []
            
            # 객체 감지 (범위 확장)
            detected_objects = self.bounding_box_detector.detect_pedestrians_and_vehicles(max_distance=200.0)
            self.chase_statistics['total_detections'] = len(detected_objects)
            
            # 충돌 감지
            collision_events = self.collision_detector.analyze_pedestrian_collision(detected_objects)
            if collision_events:
                self.chase_statistics['collision_events'] += len(collision_events)
                print(f"🚨 Collision detected: {len(collision_events)} events")
            
            return detected_objects, collision_events
            
        except Exception as e:
            print(f"⚠️ Error updating perception: {e}")
            return [], []
    
    def update_tracking(self, detected_objects, collision_events):
        """추적 모듈 업데이트"""
        try:
            if not self.vehicle_tracker:
                return False
            
            # 충돌 이벤트가 있으면 추격 시작
            if collision_events and not self.is_chasing:
                print("🚨 Collision detected - starting chase")
                self.is_chasing = True
                self.chase_statistics['collision_events'] += 1
            
            # 충돌 차량 감지 및 추적 시작 (VEHICLE만 추격)
            if collision_events and not self.vehicle_tracker.is_tracking:
                print("🚨 Collision detected - starting VEHICLE tracking")
                # vehicle만 필터링하여 추격
                vehicle_objects = [obj for obj in detected_objects if obj.get('object_type') == 'vehicle']
                if vehicle_objects:
                    self.vehicle_tracker.detect_collision_vehicle(vehicle_objects, collision_events)
                    print(f"🎯 Tracking {len(vehicle_objects)} vehicles")
                else:
                    print("⚠️ No vehicles found for tracking")
            elif not self.vehicle_tracker.is_tracking and detected_objects:
                # 사고 감지가 없으면 추격하지 않음
                print("🔍 No collision detected - waiting for accident...")
                return False
            
            # 추적 업데이트 (VEHICLE만 전달)
            vehicle_objects = [obj for obj in detected_objects if obj.get('object_type') == 'vehicle']
            is_tracking = self.vehicle_tracker.update_tracking(vehicle_objects)
            
            # 추적 중이면 추격 제어 시작
            if is_tracking and not self.is_chasing:
                print("🎯 Starting chase control")
                self.is_chasing = True
            
            return is_tracking
            
        except Exception as e:
            print(f"⚠️ Error in tracking update: {e}")
            return False
    
    def update_planning_and_control(self):
        """계획 및 제어 모듈 업데이트"""
        try:
            if not self.vehicle_tracker or not self.vehicle_tracker.is_tracking:
                # 추격 중이 아닌 경우 대기
                self._wait_for_target()
                return
            
            # 타겟 차량 가져오기
            target_vehicle = self.vehicle_tracker.get_target_vehicle()
            if not target_vehicle:
                print("⚠️ No target vehicle from tracker")
                return
            
            print(f"🎯 Target vehicle: {target_vehicle}")
            
            # 현재 위치와 회전
            current_location = self.vehicle.get_location()
            current_rotation = self.vehicle.get_transform().rotation
            
            # 추격 시작 (한 번만)
            if not self.is_chasing:
                self.chase_planner.start_chase(target_vehicle)
                self.is_chasing = True
                print("🎯 Started chasing target vehicle")
            
            # 타겟 위치 가져오기
            if isinstance(target_vehicle, dict) and 'world_location' in target_vehicle:
                target_location = carla.Location(
                    x=target_vehicle['world_location'][0],
                    y=target_vehicle['world_location'][1],
                    z=target_vehicle['world_location'][2]
                )
            else:
                target_location = target_vehicle.get_location()
            
            # IMU 데이터 전달
            imu_data = self.current_imu_data if self.current_imu_data else None
            
            # 타겟 바운딩 박스 가져오기
            target_bbox = None
            if isinstance(target_vehicle, dict) and 'bbox_2d' in target_vehicle:
                target_bbox = target_vehicle['bbox_2d']
                print(f"📷 Using target bbox: {target_bbox}")
            else:
                print("⚠️ No target bbox available")
            
            # 카메라 정보 가져오기
            camera_info = None
            if self.camera_view and self.camera_view.camera:
                camera_info = {
                    'width': int(self.camera_view.camera.attributes['image_size_x']),
                    'height': int(self.camera_view.camera.attributes['image_size_y'])
                }
                print(f"📷 Camera info: {camera_info['width']}x{camera_info['height']}")
            
            # 카메라 기반 추격 계획 수립 및 직접 제어 적용
            control_command = self.chase_planner.plan_chase_behavior(
                current_location, current_rotation, target_location, None, imu_data, target_bbox, camera_info
            )
            
            # 상태 출력
            status = self.chase_planner.get_chase_status()
            print(f"🎯 {status}")
            
        except Exception as e:
            print(f"⚠️ Error updating planning and control: {e}")
            import traceback
            traceback.print_exc()
            self._emergency_stop()
    
    def _wait_for_target(self):
        """타겟 대기"""
        try:
            # 추격 중이었다면 중지
            if self.is_chasing:
                self.is_chasing = False
                print("🎯 Stopped chasing - waiting for target")
            
            # SimpleChasePlanner의 stop_control을 사용하여 제어
            if self.chase_planner:
                self.chase_planner.stop_chase()
            
        except Exception as e:
            print(f"⚠️ Error in wait mode: {e}")
    
    def _emergency_stop(self):
        """비상 정지"""
        try:
            # SimpleChasePlanner의 stop_control을 사용하여 제어
            if self.chase_planner:
                self.chase_planner.stop_chase()
            
            print("🛑 Emergency stop applied")
            
        except Exception as e:
            print(f"⚠️ Error applying emergency stop: {e}")
    
    def cleanup(self):
        """리소스 정리"""
        try:
            print("\n🧹 Cleaning up resources...")
            
            # 추격 중지
            if hasattr(self, 'is_chasing'):
                self.is_chasing = False
            
            # 추적 중지
            if self.vehicle_tracker:
                if hasattr(self.vehicle_tracker, 'stop_tracking'):
                    self.vehicle_tracker.stop_tracking()
                print("🎯 Tracking stopped")
            
            # 추격 리셋
            if self.chase_planner:
                if hasattr(self.chase_planner, 'reset'):
                    self.chase_planner.reset()
                print("🔄 Chase reset")
            
            # 임시 카메라 정리
            if self.temp_camera:
                self.temp_camera.destroy()
                self.temp_camera = None
                print("✅ Temporary camera cleaned up")
            
            # 카메라 뷰 정리
            if self.camera_view:
                self.camera_view.cleanup()
                self.camera_view = None
            
            # 차량 정리 (destroy하지 않음 - 다른 시스템에서 사용 중일 수 있음)
            if self.vehicle:
                print("ℹ️ Not destroying vehicle (may be used by other systems)")
            
            # Zero-Copy 카메라 정리
            if self.zero_copy_camera:
                self.zero_copy_camera.cleanup()
                print("✅ Zero-Copy camera subscriber cleaned up")
            
            # 통합 충돌 감지기 정리
            if self.unified_collision_detector:
                self.unified_collision_detector.cleanup()
                print("✅ Unified collision detector cleaned up")
            
            # Zenoh 정리 (legacy)
            if self.zenoh_collision_manager:
                self.zenoh_collision_manager.cleanup()
                print("✅ Zenoh collision manager cleaned up")
            
            print("✅ Cleanup completed")
            
        except Exception as e:
            print(f"⚠️ Error during cleanup: {e}")

def main():
    """메인 함수"""
    auto_chase_control = None
    try:
        auto_chase_control = AutoChaseVehicleControl()
        auto_chase_control.run()
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 확실한 정리
        if auto_chase_control:
            auto_chase_control.cleanup()
        cleanup_opencv_windows()
        print("👋 Goodbye!")

if __name__ == "__main__":
    main()
