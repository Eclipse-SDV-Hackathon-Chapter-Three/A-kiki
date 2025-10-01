#!/usr/bin/env python3
"""
Auto Chase Vehicle Control Script
충돌 감지 시 자동으로 추격하는 차량 제어 스크립트
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

# Zenoh imports
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    print("Warning: zenoh not available, control commands will not be published")

# 상위 디렉토리의 모듈들을 import하기 위해 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from chase.perception.bounding_box_detector import BoundingBoxDetector
from chase.perception.collision_detector import CollisionDetector
from chase.perception.collision_vehicle_tracker import CollisionVehicleTracker
from chase.planning.simple_chase_planner import SimpleChasePlanner
# from chase.control.vehicle_controller import ChaseVehicleController  # 사용하지 않음
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
        print(f"⚠️ Error cleaning OpenCV windows: {e}")

def signal_handler(signum, frame):
    """시그널 핸들러 (Ctrl+C 등)"""
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
    """자동 추격 차량 제어 클래스"""
    
    def __init__(self):
        global _auto_chase_instance
        _auto_chase_instance = self
        
        self.client = None
        self.world = None
        self.vehicle = None
        
        # 모듈들
        self.bounding_box_detector = None
        self.collision_detector = None
        self.vehicle_tracker = None
        self.chase_planner = None
        # self.vehicle_controller = None  # 사용하지 않음
        
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
        
        # Zenoh for control commands
        self.zenoh_session = None
        self.target_vehicle_id = None  # Target vehicle ID to control
        self.zenoh_subscribers = []
        self.zenoh_camera_image = None  # Latest camera image from Zenoh
        self.zenoh_detected_objects = []  # Latest detected objects from Zenoh
        self.use_zenoh_camera_only = False  # Use only Zenoh camera data
        
        # 통계
        self.chase_statistics = {
            'total_detections': 0,
            'collision_events': 0,
            'tracking_duration': 0.0,
            'chase_distance': 0.0,
            'max_speed_reached': 0.0
        }
        
        # print("🚔 Auto Chase Vehicle Control initialized")
    
    def connect_to_carla(self, host='localhost', port=2000):
        """CARLA 서버에 연결"""
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            
            # print(f"✅ Connected to CARLA server at {host}:{port}")
            return True
            
        except Exception as e:
            # print(f"❌ Failed to connect to CARLA: {e}")
            return False
    
    def setup_zenoh(self):
        """Setup Zenoh connection for publishing control commands"""
        try:
            if not ZENOH_AVAILABLE:
                # print("⚠️ Zenoh not available, skipping Zenoh setup")
                return False

            # Initialize Zenoh session
            zenoh_config = zenoh.Config()
            try:
                zenoh_config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
                zenoh_config.insert_json5("transport/shared_memory/enabled", "true")
                # print("🚀 Connecting to SHM-enabled Zenoh router...")
            except Exception:
                print("📡 Using peer-to-peer Zenoh connection...")

            self.zenoh_session = zenoh.open(zenoh_config)
            # print("✅ Connected to Zenoh router successfully")
            
            # Subscribe to vehicle telemetry from CARLA spawner
            telemetry_subscriber = self.zenoh_session.declare_subscriber(
                "carla/vehicle/telemetry", self.on_vehicle_id_received
            )
            self.zenoh_subscribers.append(telemetry_subscriber)
            # print("📡 Subscribed to vehicle telemetry for target vehicle ID")
            
            # Subscribe to camera images from CARLA spawner
            camera_subscriber = self.zenoh_session.declare_subscriber(
                "carla/vehicle/camera", self.on_camera_image_received
            )
            self.zenoh_subscribers.append(camera_subscriber)
            # print("📷 Subscribed to camera images for collision detection")
            
            return True

        except Exception as e:
            # print(f"❌ Failed to setup Zenoh: {e}")
            self.zenoh_session = None
            return False
    
    def on_vehicle_id_received(self, sample):
        """Handle received vehicle ID from CARLA spawner (Zero-copy)"""
        try:
            # print(f"📡 Received telemetry data, payload type: {type(sample.payload)}")
            # print(f"📡 Payload size: {len(sample.payload)} bytes")
            
            # Zero-copy: Use ZBytes to_string() method
            if hasattr(sample.payload, 'to_string'):
                # Use ZBytes to_string() method (zero-copy)
                payload_str = sample.payload.to_string()
                # print(f"📡 Got string from ZBytes: {len(payload_str)} chars")
            elif hasattr(sample.payload, 'to_bytes'):
                # Fallback: use to_bytes() then decode
                payload_str = sample.payload.to_bytes().decode('utf-8')
            else:
                # Final fallback
                payload_str = sample.payload.decode('utf-8')
            
            # Parse JSON (minimal overhead)
            telemetry_data = json.loads(payload_str)
            # print(f"📡 Telemetry data keys: {list(telemetry_data.keys())}")
            
            # Extract vehicle ID from telemetry
            if not self.target_vehicle_id:
                # Try to get vehicle ID from telemetry data
                if 'vehicle_id' in telemetry_data:
                    self.target_vehicle_id = telemetry_data['vehicle_id']
                elif 'id' in telemetry_data:
                    self.target_vehicle_id = telemetry_data['id']
                else:
                    # Default fallback
                    self.target_vehicle_id = 137
                
                # print(f"🎯 Target vehicle ID set to: {self.target_vehicle_id}")
            
        except Exception as e:
            print(f"⚠️ Error processing vehicle ID: {e}")
            # print(f"⚠️ Payload type: {type(sample.payload)}")
            ##Don't print full payload to avoid spam
    
    def on_camera_image_received(self, sample):
        """Handle received camera image from CARLA spawner via Zenoh (Zero-copy)"""
        try:
            # print(f"📷 Received camera data, payload type: {type(sample.payload)}")
            # print(f"📷 Payload size: {len(sample.payload)} bytes")
            
            # Zero-copy: Use ZBytes to_bytes() method
            # 247KB suggests JPEG compressed image data
            if hasattr(sample.payload, 'to_bytes'):
                # Use ZBytes to_bytes() method (zero-copy)
                image_bytes = sample.payload.to_bytes()
                # print(f"📷 Got bytes from ZBytes: {len(image_bytes)} bytes")
                
                # Try to decode as JPEG/PNG first (most likely for 247KB)
                image = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                
                if image is not None:
                    # pass
                    print(f"📷 Decoded compressed image: {image.shape}")
                else:
                    # If JPEG/PNG fails, try raw image formats
                    array = np.frombuffer(image_bytes, dtype=np.uint8)
                    # print(f"📷 Trying raw image decode from {len(array)} bytes")
                    
                    # Try common image dimensions for raw data
                    possible_shapes = [
                        (480, 640, 3),   # 480p RGB (921,600 bytes)
                        (720, 1280, 3),  # 720p RGB (2,764,800 bytes)
                        (900, 1600, 3),  # 900p RGB (4,320,000 bytes)
                        (1080, 1920, 3), # 1080p RGB (6,220,800 bytes)
                    ]
                    
                    for height, width, channels in possible_shapes:
                        expected_size = height * width * channels
                        if len(array) == expected_size:
                            image = array.reshape((height, width, channels))
                            # print(f"📷 Raw image decoded: {height}x{width}x{channels}")
                            break
            else:
                # print("⚠️ ZBytes does not have to_bytes() method")
                return
            
            if image is not None:
                # Store reference (zero-copy)
                self.zenoh_camera_image = image
                # print(f"📷 Stored camera image: {image.shape}")
                
                # Process image for collision detection
                self.process_zenoh_camera_image(image)
            else:
                # print("⚠️ Failed to interpret image data")
                # print(f"⚠️ Array size: {len(array)} bytes")
                pass
        except Exception as e:
            # print(f"⚠️ Error processing camera image: {e}")
            # print(f"⚠️ Payload type: {type(sample.payload)}")
            # print(f"⚠️ Payload attributes: {dir(sample.payload)}")
            pass
    
    def process_zenoh_camera_image(self, image):
        """Process camera image from Zenoh for collision detection"""
        try:
            # print(f"🔍 Processing Zenoh camera image: {image.shape if image is not None else 'None'}")
            
            # Use OpenCV-based object detection
            detected_objects = self.detect_objects_opencv(image)
            
            # Store detected objects for display
            self.zenoh_detected_objects = detected_objects
            
            # print(f"🎯 OpenCV detected {len(detected_objects)} objects from Zenoh camera")
            
            if detected_objects:
                # 객체 정보 출력
                # for i, obj in enumerate(detected_objects):
                    # print(f"  Object {i}: ID={obj.get('actor_id', 'Unknown')}, bbox={obj.get('bbox_2d', [])}")
                
                # 개선된 충돌 감지 (거리와 크기 기반)
                collision_detected = False
                collision_objects = []
                
                for obj in detected_objects:
                    bbox = obj.get('bbox_2d', [])
                    distance = obj.get('distance_estimate', 100)
                    
                    if len(bbox) >= 4:
                        x, y, w, h = bbox[:4]
                        area = w * h
                        
                        # 사람-차량 충돌 감지 조건 (카메라 기반)
                        # 1. 거리가 30m 이내 (충돌 위험 구역)
                        # 2. 충분히 큰 객체 (사람 + 차량 조합)
                        # 3. 화면 중앙에 위치한 객체
                        # 4. 특정 크기 범위의 객체 (사람과 차량이 함께 보이는 경우)
                        
                        aspect_ratio = w / h if h > 0 else 1.0
                        is_close = distance <= 30  # 충돌 위험 거리로 단축
                        is_large = area > 20000   # 사람+차량 조합 크기
                        is_center_large = (area > 12000 and 
                                         abs(x + w/2 - image.shape[1]/2) < image.shape[1]/4)  # 화면 중앙
                        
                        # 사람-차량 충돌 특성:
                        # 1. 가로로 긴 객체 (차량이 사람을 치고 지나가는 경우)
                        is_horizontal_collision = (aspect_ratio > 1.8 and area > 15000)
                        
                        # 2. 세로로 긴 객체 (차량이 사람을 정면에서 치는 경우)
                        is_vertical_collision = (aspect_ratio < 0.6 and area > 18000)
                        
                        # 3. 중간 크기의 불규칙한 객체 (사람과 차량이 겹쳐 보이는 경우)
                        is_irregular_collision = (0.7 < aspect_ratio < 1.3 and 10000 < area < 25000)
                        
                        # 4. 매우 가까운 거리의 큰 객체 (직접 충돌)
                        is_direct_collision = (distance <= 15 and area > 15000)
                        
                        if (is_close or is_large or is_center_large or 
                            is_horizontal_collision or is_vertical_collision or 
                            is_irregular_collision or is_direct_collision):
                            collision_detected = True
                            collision_objects.append(obj)
                            # print(f"🚨 COLLISION detected for object {obj.get('actor_id', 'Unknown')}: "
                            #       f"size={w}x{h}, area={area}, distance={distance}m, aspect={aspect_ratio:.2f}, "
                            #       f"close={is_close}, large={is_large}, center={is_center_large}, "
                            #       f"horizontal={is_horizontal_vehicle}, vertical={is_vertical_vehicle}")
                
                if collision_detected:
                    # print(f"🚨 COLLISION DETECTED from Zenoh camera: {len(collision_objects)} collision objects")
                    self.chase_statistics['collision_events'] += len(collision_objects)
                    
                    # 즉시 추격 시작
                    # print("🚔 Starting immediate chase from collision detection")
                    self.is_chasing = True
                    
                    # 충돌 객체들을 collision_detected로 마킹
                    for obj in collision_objects:
                        obj['collision_detected'] = True
                    
                else:
                    pass
                    # print("✅ No collision detected - no qualifying objects found")
            
        except Exception as e:
            # print(f"⚠️ Error processing Zenoh camera image: {e}")
            import traceback
            traceback.print_exc()
    
    def detect_objects_opencv(self, image):
        """Detect objects using OpenCV - focused on person-vehicle collision detection"""
        try:
            # print(f"🔍 Detecting objects in camera view: {image.shape}")
            
            # Enhanced object detection for person-vehicle collision
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur for better edge detection
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Edge detection with adjusted parameters for person-vehicle detection
            edges = cv2.Canny(blurred, 30, 100)  # Lower thresholds for better detection
            
            # Morphological operations to connect nearby edges
            kernel = np.ones((3,3), np.uint8)
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            detected_objects = []
            height, width = image.shape[:2]
            
            for i, contour in enumerate(contours):
                # Filter by area - adjusted for person-vehicle detection
                area = cv2.contourArea(contour)
                if area > 300:  # Lower threshold for person detection
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Only include objects that are significantly within camera view
                    margin = 30  # Reduced margin for better detection
                    if (x > margin and y > margin and 
                        x + w < width - margin and y + h < height - margin):
                        
                        # Enhanced size check for person-vehicle collision
                        aspect_ratio = w / h if h > 0 else 1.0
                        
                        # Person-vehicle collision characteristics:
                        # 1. Person-like objects: taller than wide (aspect_ratio < 1.0)
                        # 2. Vehicle-like objects: wider than tall (aspect_ratio > 1.0)
                        # 3. Combined objects: mixed characteristics
                        
                        is_person_like = (aspect_ratio < 0.8 and 30 < w < 100 and 50 < h < 200)
                        is_vehicle_like = (aspect_ratio > 1.2 and 80 < w < 300 and 40 < h < 150)
                        is_combined_like = (0.8 <= aspect_ratio <= 1.2 and 60 < w < 200 and 60 < h < 180)
                        
                        if is_person_like or is_vehicle_like or is_combined_like:
                            # Determine object type based on characteristics
                            if is_person_like:
                                object_type = 'person'
                                confidence = min(0.9, area / 8000.0)
                            elif is_vehicle_like:
                                object_type = 'vehicle'
                                confidence = min(0.9, area / 12000.0)
                            else:
                                object_type = 'person_vehicle'  # Possible collision
                                confidence = min(0.9, area / 10000.0)
                            
                            # Create object data
                            obj = {
                                'actor_id': f'collision_object_{i}',
                                'bbox_2d': [x, y, w, h],
                                'world_location': [x, y, 0],
                                'object_type': object_type,
                                'confidence': confidence,
                                'in_camera_view': True,
                                'distance_estimate': self._estimate_distance_from_bbox(w, h),
                                'aspect_ratio': aspect_ratio,
                                'collision_risk': 'high' if object_type == 'person_vehicle' else 'medium'
                            }
                            detected_objects.append(obj)
                            # print(f"🎯 Detected {object_type} {i}: bbox=({x},{y},{w},{h}), area={area}, aspect={aspect_ratio:.2f}")
            
            # print(f"✅ Total collision-related objects detected: {len(detected_objects)}")
            return detected_objects
            
        except Exception as e:
            print(f"⚠️ Error in OpenCV object detection: {e}")
            return []
    
    def _estimate_distance_from_bbox(self, width, height):
        """Estimate distance from bounding box size - optimized for person-vehicle collision detection"""
        # Distance estimation based on person-vehicle collision scenarios
        area = width * height
        aspect_ratio = width / height if height > 0 else 1.0
        
        # Person-like objects (taller than wide)
        if aspect_ratio < 0.8:
            if area > 15000:
                return 15  # Very close person
            elif area > 8000:
                return 25  # Close person
            elif area > 4000:
                return 40  # Medium distance person
            else:
                return 60  # Far person
        
        # Vehicle-like objects (wider than tall)
        elif aspect_ratio > 1.2:
            if area > 30000:
                return 20  # Very close vehicle
            elif area > 15000:
                return 35  # Close vehicle
            elif area > 8000:
                return 50  # Medium distance vehicle
            else:
                return 70  # Far vehicle
        
        # Combined or irregular objects (possible collision scenario)
        else:
            if area > 25000:
                return 12  # Very close collision scenario
            elif area > 12000:
                return 20  # Close collision scenario
            elif area > 6000:
                return 35  # Medium distance collision scenario
            else:
                return 55  # Far collision scenario
    
    
    def handle_zenoh_chase_control(self):
        """Handle chase control based on Zenoh camera data"""
        try:
            # print(f"🚔 Zenoh chase control: {len(self.zenoh_detected_objects)} objects")
            # print(f"🚔 Chase status: is_chasing={self.is_chasing}, collisions={self.chase_statistics['collision_events']}")
            
            # 충돌 이벤트가 있고 아직 추격을 시작하지 않은 경우
            if self.chase_statistics['collision_events'] > 0 and not self.is_chasing:
                # print("🚨 Collision detected - starting chase from Zenoh data")
                
                # 가장 큰 객체를 타겟으로 선택
                if self.zenoh_detected_objects:
                    target_obj = max(self.zenoh_detected_objects, 
                                   key=lambda x: x['bbox_2d'][2] * x['bbox_2d'][3])
                    
                    # print(f"🎯 Target selected: {target_obj['actor_id']}")
                    # print(f"🎯 Target bbox: {target_obj['bbox_2d']}")
                    
                    # 추격 시작
                    self.is_chasing = True
                    # print("🚔 Chase started - is_chasing set to True")
                    
                    # 간단한 추격 제어 명령 생성
                    self.apply_simple_chase_control(target_obj)
                else:
                    # print("⚠️ No detected objects for chase target")
                    pass
            
            # 이미 추격 중인 경우 제어 업데이트
            elif self.is_chasing and self.zenoh_detected_objects:
                # print("🚔 Chase in progress - updating control")
                
                # 가장 큰 객체를 타겟으로 유지
                target_obj = max(self.zenoh_detected_objects, 
                               key=lambda x: x['bbox_2d'][2] * x['bbox_2d'][3])
                
                print(f"🎯 Updating target: {target_obj['actor_id']}")
                self.apply_simple_chase_control(target_obj)
            
            elif self.is_chasing and not self.zenoh_detected_objects:
                print("⚠️ Chase active but no objects detected - stopping")
                self.is_chasing = False
            
            # 추가: 카메라 화면 내 큰 객체 감지 시 추격 시작
            elif not self.is_chasing and self.zenoh_detected_objects:
                # 카메라 화면에 잡힌 객체 중 가장 큰 것을 선택
                camera_objects = [obj for obj in self.zenoh_detected_objects 
                                if obj.get('in_camera_view', False)]
                
                if camera_objects:
                    largest_obj = max(camera_objects, 
                                    key=lambda x: x['bbox_2d'][2] * x['bbox_2d'][3])
                    bbox = largest_obj.get('bbox_2d', [])
                    distance = largest_obj.get('distance_estimate', 100)
                    
                    if len(bbox) >= 4:
                        w, h = bbox[2], bbox[3]
                        area = w * h
                        
                        # 카메라 화면에 잡힌 사람-차량 충돌 가능성이 있는 객체
                        # 사람과 차량이 함께 보이는 경우를 고려한 조건
                        if area > 8000 and distance <= 50:  # 사람-차량 충돌 감지 거리
                            # print(f"🎯 Large camera object detected ({w}x{h}, {distance}m) - starting chase")
                            self.is_chasing = True
                            self.chase_statistics['collision_events'] += 1
                            self.apply_simple_chase_control(largest_obj)
                
        except Exception as e:
            print(f"⚠️ Error in Zenoh chase control: {e}")
            import traceback
            traceback.print_exc()
    
    def apply_simple_chase_control(self, target_obj):
        """Apply improved chase control based on target object"""
        try:
            if not self.vehicle:
                # print("⚠️ No vehicle available for control")
                return
            
            # print(f"🎮 Applying chase control to vehicle {self.vehicle.id}")
            
            # 차량 상태 확인
            vehicle_location = self.vehicle.get_location()
            vehicle_transform = self.vehicle.get_transform()
            # print(f"🚗 Vehicle location: {vehicle_location}")
            # print(f"🚗 Vehicle transform: {vehicle_transform}")
            
            # 차량이 제어 가능한 상태인지 확인
            if self.vehicle.is_at_traffic_light():
                # print("⚠️ Vehicle is at traffic light - control may be limited")
                pass
            
            # 타겟 객체의 바운딩 박스 중심점 계산
            bbox = target_obj['bbox_2d']
            target_center_x = bbox[0] + bbox[2] / 2
            target_center_y = bbox[1] + bbox[3] / 2
            
            # 화면 중심점 (카메라 기준) - 실제 이미지 크기에 맞춤
            if self.zenoh_camera_image is not None:
                screen_center_x = self.zenoh_camera_image.shape[1] / 2
                screen_center_y = self.zenoh_camera_image.shape[0] / 2
            else:
                screen_center_x = 800
                screen_center_y = 450
            
            # 타겟과 화면 중심의 차이 계산
            dx = target_center_x - screen_center_x
            dy = target_center_y - screen_center_y
            
            # print(f"🎯 Target center: ({target_center_x:.1f}, {target_center_y:.1f})")
            # print(f"🎯 Screen center: ({screen_center_x:.1f}, {screen_center_y:.1f})")
            # print(f"🎯 Offset: dx={dx:.1f}, dy={dy:.1f}")
            
            # 개선된 조향 각도 계산
            # 타겟이 화면 중앙에 가까우면 직진, 멀리 있으면 조향
            steer_factor = dx / screen_center_x  # -1 ~ 1 범위
            
            # 부드러운 조향을 위한 감쇠 적용
            steer_gain = 0.8  # 조향 감도 (기존 1.5에서 감소)
            steer = max(-0.8, min(0.8, steer_factor * steer_gain))  # 최대 조향각 제한
            
            # 타겟이 화면 중앙에 가까우면 조향 감소
            center_tolerance = screen_center_x * 0.2  # 화면 중심 20% 범위
            if abs(dx) < center_tolerance:
                steer *= 0.3  # 중앙 근처에서는 조향 크게 감소
            
            # 속도 제어 (거리와 타겟 크기 기반)
            target_size = bbox[2] * bbox[3]
            distance = target_obj.get('distance_estimate', 50)
            
            # 거리가 가까우면 천천히, 멀면 빠르게
            if distance <= 30:
                throttle = 0.2  # 매우 천천히
                brake = 0.1     # 약간 브레이크
            elif distance <= 50:
                throttle = 0.4  # 천천히
                brake = 0.0
            else:
                throttle = 0.6  # 보통 속도
                brake = 0.0
            
            # 타겟 크기가 매우 크면 추가로 감속
            if target_size > 20000:
                throttle *= 0.7
                brake = 0.1
            
            # print(f"🎮 Control calculation: steer_factor={steer_factor:.3f}, "
            #       f"distance={distance}m, target_size={target_size}")
            
            # CARLA 제어 명령 생성
            control = carla.VehicleControl()
            control.throttle = throttle
            control.steer = steer
            control.brake = brake
            control.hand_brake = False
            control.reverse = False
            
            # 제어 명령 적용
            # print(f"🎮 Applying control to vehicle: throttle={throttle:.2f}, "
            #       f"steer={steer:.2f}, brake={brake:.2f}")
            self.vehicle.apply_control(control)
            
            # 차량 속도 확인
            velocity = self.vehicle.get_velocity()
            speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
            # print(f"🚗 Vehicle speed: {speed:.2f} m/s")
            
            # Zenoh로도 제어 명령 발행
            self.publish_control_command(control)
            
        except Exception as e:
            # print(f"⚠️ Error applying simple chase control: {e}")
            import traceback
            traceback.print_exc()
    
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
                    # print(f"🚗 Found vehicle: {actor.type_id} (ID: {actor.id}) at {actor.get_location()}")
            
            if not vehicles:
                # print("❌ No existing vehicles found in CARLA world")
                return False
            
            # 첫 번째 차량을 chase vehicle로 사용
            self.vehicle = vehicles[0]
            # print(f"✅ Using existing vehicle: {self.vehicle.type_id} (ID: {self.vehicle.id})")
            # print(f"📍 Vehicle location: {self.vehicle.get_location()}")
            
            return True
            
        except Exception as e:
            # print(f"❌ Error spawning chase vehicle: {e}")
            return False
    
    def setup_camera(self):
        """추격차량 카메라 설정"""
        try:
            # 최적화된 고성능 카메라 뷰 사용
            self.camera_view = OptimizedCameraView(
                self.world, 
                self.vehicle, 
                "Auto Chase Vehicle Camera View",
                enable_bounding_boxes=True,  # 바운딩 박스 활성화
                enable_zenoh=True           # Zenoh 활성화
            )
            
            # 카메라 설정
            camera_location = carla.Location(x=1.5, z=1.4)
            camera_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
            
            if self.camera_view.setup_camera(camera_location, camera_rotation):
                # 카메라 콜백 설정을 나중에 처리 (segfault 방지)
                self.camera_view.latest_image = None  # 최신 이미지 저장 변수 초기화
                
                print("📷 Chase vehicle camera attached with bounding box detection")
                print("📷 Camera callback will be set up after modules initialization")
                return True
            else:
                return False
            
        except Exception as e:
            print(f"❌ Error setting up camera: {e}")
            import traceback
            traceback.print_exc()
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
    
    def setup_camera_callback(self):
        """카메라 콜백 설정 (segfault 방지를 위해 별도 함수로 분리)"""
        try:
            if self.camera_view and self.camera_view.camera:
                print("📷 Setting up camera callback...")
                
                # 카메라가 이미 다른 콜백을 가지고 있는지 확인
                if hasattr(self.camera_view.camera, '_listeners'):
                    print(f"📷 Camera has {len(self.camera_view.camera._listeners)} existing listeners")
                
                # 콜백 설정을 시도하지 않고 Zenoh 카메라 데이터만 사용
                print("📷 Skipping CARLA camera callback to avoid segfault")
                print("📷 Using Zenoh camera data only (like web dashboard)")
                
                # Zenoh 카메라 데이터만 사용하도록 설정
                self.use_zenoh_camera_only = True
                
                # Pygame 초기화 (pygame view 표시용)
                try:
                    if not pygame.get_init():
                        pygame.init()
                    print("✅ Pygame initialized for camera view")
                except Exception as e:
                    print(f"⚠️ Pygame initialization error: {e}")
                
            else:
                print("⚠️ Camera view or camera not available for callback setup")
                
        except Exception as e:
            print(f"❌ Error setting up camera callback: {e}")
            import traceback
            traceback.print_exc()
            
            # 콜백 설정 실패 시 대안으로 Zenoh 카메라 데이터만 사용
            print("📷 Falling back to Zenoh camera data only")
    
    def on_camera_image(self, carla_image):
        """카메라 이미지 콜백 - 추격차의 카메라에서 받은 이미지"""
        try:
            # 이미지 크기 확인
            if not hasattr(carla_image, 'width') or not hasattr(carla_image, 'height'):
                print("⚠️ Invalid camera image object")
                return
                
            print(f"📷 Camera callback received: {carla_image.width}x{carla_image.height}")
            
            # raw_data 확인
            if not hasattr(carla_image, 'raw_data') or carla_image.raw_data is None:
                print("⚠️ No raw_data in camera image")
                return
            
            # CARLA 이미지를 numpy 배열로 변환
            array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
            
            # 이미지 크기 검증
            expected_size = carla_image.height * carla_image.width * 4  # RGBA
            if len(array) != expected_size:
                print(f"⚠️ Image size mismatch: expected {expected_size}, got {len(array)}")
                return
            
            array = array.reshape((carla_image.height, carla_image.width, 4))
            array = array[:, :, :3]  # RGB만 사용 (A 채널 제거)
            array = array[:, :, ::-1]  # RGB를 BGR로 변환 (OpenCV 형식)
            
            print(f"📷 Converted image shape: {array.shape}")
            
            # 최신 이미지로 저장 (thread-safe)
            if self.camera_view:
                self.camera_view.latest_image = array.copy()  # 복사본 저장
                print("✅ Camera image stored")
                
        except Exception as e:
            print(f"⚠️ Error processing camera image: {e}")
            # Core dump 방지를 위해 traceback 출력을 최소화
            print(f"⚠️ Image object type: {type(carla_image)}")
            print(f"⚠️ Image attributes: {dir(carla_image) if hasattr(carla_image, '__dict__') else 'N/A'}")
    
    def setup_modules(self):
        """모듈들 초기화"""
        try:
            # 바운딩 박스 감지기 초기화
            if self.camera_view and self.camera_view.camera:
                self.bounding_box_detector = BoundingBoxDetector(self.world, self.camera_view.camera)
                print("🎯 Bounding box detector initialized")
            
            # 충돌 감지기 초기화
            self.collision_detector = CollisionDetector()
            print("🚨 Collision detector initialized")
            
            # 충돌 차량 추적기 초기화
            self.vehicle_tracker = CollisionVehicleTracker(self.world)
            print("🎯 Vehicle tracker initialized")
            
            # 단순 추격 계획자 초기화
            self.chase_planner = SimpleChasePlanner(self.world, self.vehicle, self.world.get_map())
            # 추격 파라미터 설정 (속도 더욱 감소)
            self.chase_planner.set_chase_parameters(
                max_speed=8.0,         # 최대 속도 (29km/h) - 매우 안전한 속도
                min_distance=5.0,      # 5m 이내에서 박기 - 더 안전한 거리
                chase_distance=100.0   # 100m 이내에서 추격
            )
            # 조향 방향 정상 설정 (타겟을 따라가도록)
            self.chase_planner.set_steering_direction(1)
            
            # Set control publisher callback
            self.chase_planner.set_control_publisher_callback(self.publish_control_command)
            
            print("🧠 Chase planner initialized with high-speed parameters")
            
            # 차량 제어기 초기화 (사용하지 않음 - SimpleChasePlanner에서 직접 제어)
            # self.vehicle_controller = ChaseVehicleController(self.world, self.vehicle)
            # print("🎮 Vehicle controller initialized")
            
            print("✅ All modules initialized")
            
            # 카메라 콜백 설정 (모듈 초기화 후)
            self.setup_camera_callback()
            
            return True
            
        except Exception as e:
            print(f"❌ Error setting up modules: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def update_perception(self):
        """인식 모듈 업데이트"""
        try:
            if not self.bounding_box_detector:
                return []
            
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
            
            # 충돌 차량 감지 및 추적 시작
            if collision_events:
                print(f"🚨 Collision events detected: {len(collision_events)}")
                tracking_started = self.vehicle_tracker.detect_collision_vehicle(detected_objects, collision_events)
                print(f"🎯 Tracking started: {tracking_started}")
            
            # 추적 업데이트 (추적 중이거나 방금 시작된 경우)
            if self.vehicle_tracker.is_tracking:
                is_tracking = self.vehicle_tracker.update_tracking(detected_objects)
                print(f"🎯 Tracking update result: {is_tracking}")
                return is_tracking
            else:
                # 충돌이 감지된 경우에만 추격 시작
                if collision_events:
                    print(f"🚨 Collision detected - starting chase")
                    # 충돌 이벤트가 있는 경우에만 추격 시작
                    if detected_objects:
                        target_vehicle = detected_objects[0]
                        self.vehicle_tracker.start_tracking(target_vehicle)
                        return True
                else:
                    print(f"🚗 Vehicles detected but no collision - waiting for collision event")
                    return False
            
            return False
            
        except Exception as e:
            print(f"⚠️ Error updating tracking: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def update_planning_and_control(self):
        """계획 및 제어 모듈 업데이트"""
        try:
            print(f"🎯 Planning and control update: tracker={self.vehicle_tracker is not None}, is_tracking={self.vehicle_tracker.is_tracking if self.vehicle_tracker else False}")
            
            if not self.vehicle_tracker or not self.vehicle_tracker.is_tracking:
                # 추격 중이 아닌 경우 대기
                print("🛑 Not tracking - waiting for target")
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
            
            # SimpleChasePlanner에서 이미 vehicle.apply_control()을 호출하므로 별도 적용 불필요
            # self._apply_control_command(control_command)
            
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
    
    # _apply_control_command 함수 제거 - SimpleChasePlanner에서 직접 제어
    
    def publish_control_command(self, control_command):
        """Publish control command to Zenoh for target vehicle"""
        try:
            if not self.zenoh_session or not self.target_vehicle_id:
                return

            # Create control message
            control_data = {
                'target_vehicle_id': self.target_vehicle_id,
                'throttle': float(control_command.throttle),
                'steer': float(control_command.steer),
                'brake': float(control_command.brake),
                'hand_brake': bool(control_command.hand_brake),
                'reverse': bool(control_command.reverse),
                'manual_gear_shift': bool(control_command.manual_gear_shift),
                'gear': int(control_command.gear) if control_command.manual_gear_shift else 0,
                'timestamp': time.time()
            }

            # Publish to Zenoh
            self.zenoh_session.put("carla/vehicle/control", json.dumps(control_data))
            print(f"📡 Published control command for vehicle {self.target_vehicle_id}: "
                  f"throttle={control_data['throttle']:.2f}, steer={control_data['steer']:.2f}, brake={control_data['brake']:.2f}")

        except Exception as e:
            print(f"⚠️ Error publishing control command: {e}")
    
    def _update_chase_statistics(self, target_distance):
        """추격 통계 업데이트"""
        try:
            # 현재 속도 계산
            current_velocity = self.vehicle.get_velocity()
            current_speed = np.linalg.norm([current_velocity.x, current_velocity.y, current_velocity.z])
            
            # 최대 속도 업데이트
            if current_speed > self.chase_statistics['max_speed_reached']:
                self.chase_statistics['max_speed_reached'] = current_speed
            
            # 추격 거리 업데이트
            self.chase_statistics['chase_distance'] = target_distance
            
            # 추격 시간 업데이트
            if self.is_chasing:
                self.chase_statistics['tracking_duration'] += self.update_interval
            
        except Exception as e:
            print(f"⚠️ Error updating statistics: {e}")
    
    def _print_chase_status(self, target_distance):
        """추격 상태 출력"""
        try:
            # 5초마다 상태 출력
            if int(time.time()) % 5 == 0:
                current_velocity = self.vehicle.get_velocity()
                current_speed = np.linalg.norm([current_velocity.x, current_velocity.y, current_velocity.z])
                
                print(f"🎯 Chase Status: Distance={target_distance:.1f}m, Speed={current_speed:.1f}m/s")
                
                # 추적 상태 출력
                if self.vehicle_tracker:
                    tracking_status = self.vehicle_tracker.get_tracking_status()
                    print(f"   Tracking: {tracking_status['is_tracking']}, Duration: {tracking_status['track_duration']:.1f}s")
                
                # 추격 상태 출력
                if self.chase_planner:
                    chase_status = self.chase_planner.get_chase_status()
                    print(f"   State: {chase_status['current_state']}, Duration: {chase_status['state_duration']:.1f}s")
            
        except Exception as e:
            print(f"⚠️ Error printing chase status: {e}")
    
    def _emergency_stop(self):
        """비상 정지"""
        try:
            # SimpleChasePlanner의 stop_control을 사용하여 제어
            if self.chase_planner:
                self.chase_planner.stop_chase()
            
            print("🛑 Emergency stop applied")
            
        except Exception as e:
            print(f"⚠️ Error applying emergency stop: {e}")
    
    
    def display_camera_view(self):
        """카메라 뷰 표시 (pygame만 사용)"""
        try:
            # Zenoh 카메라 기반 시스템인 경우
            if self.use_zenoh_camera_only:
                # Pygame view만 표시 (OpenCV 윈도우 제거)
                if self.camera_view and self.zenoh_camera_image is not None:
                    self.camera_view.set_zenoh_camera_data(
                        self.zenoh_camera_image, 
                        self.zenoh_detected_objects
                    )
                    self.camera_view.display_camera_view()
            else:
                # 기존 CARLA 카메라 방식
                if self.camera_view:
                    # Zenoh 카메라 데이터를 OptimizedCameraView에 동기화
                    if self.zenoh_camera_image is not None:
                        self.camera_view.set_zenoh_camera_data(
                            self.zenoh_camera_image, 
                            self.zenoh_detected_objects
                        )
                    
                    # HUD 정보 업데이트
                    if self.vehicle_tracker and self.vehicle_tracker.is_tracking:
                        target_position = self.vehicle_tracker.get_target_position()
                        target_distance = self.vehicle_tracker.get_target_distance(self.vehicle.get_transform().location)
                        
                        if target_position:
                            self.camera_view.set_hud_info(
                                Target=f"({target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f})",
                                Distance=f"{target_distance:.1f} m",
                                Objects=f"{self.chase_statistics['total_detections']} | Collisions: {self.chase_statistics['collision_events']}"
                            )
                    
                    # OptimizedCameraView의 display_camera_view 호출 (pygame만 사용)
                    self.camera_view.display_camera_view()
            
        except Exception as e:
            print(f"⚠️ Error displaying camera view: {e}")
    
    def display_chase_vehicle_camera(self):
        """Display chase vehicle's camera view - DISABLED (pygame only)"""
        # OpenCV 윈도우는 사용하지 않음 - pygame만 사용
        pass
    
    def display_zenoh_camera_view(self):
        """Display Zenoh camera view - DISABLED (pygame only)"""
        # OpenCV 윈도우는 사용하지 않음 - pygame만 사용
        pass
    
    def run(self):
        """메인 실행 루프"""
        try:
            print("🚀 Starting Auto Chase Vehicle Control...")
            
            # CARLA 연결
            if not self.connect_to_carla():
                return
            
            # Zenoh 설정
            self.setup_zenoh()
            
            # 기존 차량 찾기
            if not self.find_existing_vehicle():
                return
            
            # 카메라 설정
            self.setup_camera()
            
            # IMU 센서 설정
            self.setup_imu_sensor()
            
            # 카메라 콜백 설정
            self.setup_camera_callback()
            
            # 모듈들 초기화
            self.setup_modules()
            
            # Zenoh 구독자 설정
            if self.zenoh_session:
                # 차량 텔레메트리 구독
                telemetry_subscriber = self.zenoh_session.declare_subscriber(
                    "carla/vehicle/telemetry",
                    self.on_vehicle_id_received
                )
                self.zenoh_subscribers.append(telemetry_subscriber)
                
                # 카메라 이미지 구독
                camera_subscriber = self.zenoh_session.declare_subscriber(
                    "carla/vehicle/camera",
                    self.on_camera_image_received
                )
                self.zenoh_subscribers.append(camera_subscriber)
                
                print("📡 Zenoh subscribers configured")
            
            print("🎮 CONTROLS:")
            print("  ESC - Exit")
            print("  C - Manual collision trigger (disabled)")
            
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
                    
                    # 1. 인식 업데이트
                    detected_objects, collision_events = self.update_perception()
                    
                    # 2. 추적 업데이트
                    is_tracking = self.update_tracking(detected_objects, collision_events)
                    # print(f"🎯 Main loop - is_tracking: {is_tracking}")
                    
                    # Zenoh 카메라 기반 추격 제어 (추가)
                    if self.use_zenoh_camera_only:
                        # print(f"🚔 Zenoh-based chase control: {len(self.zenoh_detected_objects)} objects detected")
                        # print(f"🚔 Chase status check: is_chasing={self.is_chasing}, collisions={self.chase_statistics['collision_events']}")
                        self.handle_zenoh_chase_control()
                    
                    # 3. 계획 및 제어 업데이트
                    self.update_planning_and_control()
                    
                    # 4. 카메라 뷰 표시 (단일 윈도우)
                    try:
                        self.display_camera_view()
                    except Exception as display_error:
                        print(f"⚠️ Camera display error: {display_error}")
                    
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
    
    def cleanup(self):
        """리소스 정리"""
        try:
            print("\n🧹 Cleaning up resources...")
            
            # 모듈들 정리
            if self.vehicle_tracker:
                self.vehicle_tracker.reset_tracking()
            if self.chase_planner:
                self.chase_planner.reset_chase()
            
            # 카메라 뷰 정리
            if self.camera_view:
                self.camera_view.cleanup()
                self.camera_view = None
            
            # 차량 정리 (destroy하지 않음 - 다른 시스템에서 사용 중일 수 있음)
            if self.vehicle:
                print(f"ℹ️ Not destroying vehicle {self.vehicle.id} (may be used by other systems)")
                self.vehicle = None
            
            # Zenoh 정리
            for subscriber in self.zenoh_subscribers:
                subscriber.undeclare()
            if self.zenoh_session:
                self.zenoh_session.close()
            
            # OpenCV 창 정리 (강화)
            print("🧹 Cleaning up OpenCV windows...")
            cv2.destroyAllWindows()
            cv2.waitKey(1)  # 윈도우 정리를 위한 추가 대기
            
            # 모든 OpenCV 윈도우 강제 종료
            try:
                # 특정 윈도우 이름으로 종료 시도
                cv2.destroyWindow("Chase Vehicle Camera")
                cv2.destroyWindow("Zenoh Camera View")
                cv2.destroyWindow("Camera View")
            except:
                pass  # 윈도우가 없어도 무시
            
            # 추가 대기 시간
            time.sleep(0.1)
            
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
