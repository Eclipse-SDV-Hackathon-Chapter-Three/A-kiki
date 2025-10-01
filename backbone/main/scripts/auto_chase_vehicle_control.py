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
from chase.perception.collision_tracker import CollisionTracker
from chase.planning.simple_chase_planner import SimpleChasePlanner
from chase.communication.zenoh_collision_manager import ZenohCollisionManager
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
    """자동 추격 차량 제어 클래스"""
    
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
        # self.vehicle_controller = None  # 사용하지 않음
        
        # 카메라 뷰
        self.camera_view = None
        
        # 제어 상태
        self.running = True
        self.is_chasing = False
        self.last_update_time = 0.0
        self.update_interval = 0.05  # 50ms 업데이트 간격 (더 빠른 반응)

        # Emergency alert 상태
        self.emergency_alert_sent = False
        self.chase_start_time = None
        self.target_vehicle_actor = None  # 실제 타겟 차량 액터
        
        # IMU 센서
        self.imu_sensor = None
        self.current_imu_data = None
        
        # 통계
        self.chase_statistics = {
            'total_detections': 0,
            'collision_events': 0,
            'tracking_duration': 0.0,
            'chase_distance': 0.0,
            'max_speed_reached': 0.0
        }
        
        # Zenoh 관련 변수 (새로운 모듈로 대체됨)
        self.zenoh_camera_image = None
        self.zenoh_detected_objects = []
        self.use_zenoh_camera_only = False  # CARLA 카메라도 함께 사용 (바운딩 박스 감지용)
        self.temp_camera = None  # 임시 카메라 (ground truth용)
        
        # 전역 인스턴스 설정
        global _auto_chase_instance
        _auto_chase_instance = self
        
        print("🚔 Auto Chase Vehicle Control initialized")
    
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

                # Emergency alert 및 suspect GPS 토픽을 위한 별도 Zenoh 세션 초기화
                if ZENOH_AVAILABLE:
                    self.zenoh_emergency_session = zenoh.open(zenoh.Config())
                    print("✅ Zenoh emergency session initialized")
                else:
                    self.zenoh_emergency_session = None
                    print("⚠️ Zenoh not available for emergency alerts")

                return True
            else:
                print("❌ Failed to setup Zenoh collision manager")
                return False

        except Exception as e:
            print(f"❌ Failed to setup Zenoh: {e}")
            self.zenoh_collision_manager = None
            self.zenoh_emergency_session = None
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
    
    def on_vehicle_id_received(self, sample):
        """Handle received vehicle ID from CARLA spawner (Zero-copy)"""
        try:
            # Zero-copy: Use ZBytes to_string() method
            if hasattr(sample.payload, 'to_string'):
                payload_str = sample.payload.to_string()
            elif hasattr(sample.payload, 'to_bytes'):
                payload_bytes = sample.payload.to_bytes()
                payload_str = payload_bytes.decode('utf-8')
            else:
                return
            
            # Parse JSON (minimal overhead)
            telemetry_data = json.loads(payload_str)
            
            # Extract vehicle ID from telemetry
            if 'vehicle_id' in telemetry_data:
                self.target_vehicle_id = telemetry_data['vehicle_id']
                print(f"🎯 Target vehicle ID set to: {self.target_vehicle_id}")
            
        except Exception as e:
            print(f"⚠️ Error processing vehicle ID: {e}")
    
    def on_camera_image_received(self, sample):
        """Handle received camera image from CARLA spawner via Zenoh (Zero-copy)"""
        try:
            # Zero-copy: Use ZBytes to_bytes() method
            if hasattr(sample.payload, 'to_bytes'):
                image_bytes = sample.payload.to_bytes()
                
                # Try to decode as JPEG/PNG first
                image = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                
                if image is not None:
                    print(f"📷 Decoded compressed image: {image.shape}")
                else:
                    # If JPEG/PNG fails, try raw image formats
                    array = np.frombuffer(image_bytes, dtype=np.uint8)
                    
                    # Try common image dimensions for raw data
                    for height in [480, 720, 1080]:
                        for width in [640, 1280, 1920]:
                            for channels in [3, 4]:
                                expected_size = height * width * channels
                                if len(array) == expected_size:
                                    image = array.reshape((height, width, channels))
                                    print(f"📷 Raw image decoded: {height}x{width}x{channels}")
                                    break
            else:
                print("⚠️ ZBytes does not have to_bytes() method")
                return
                
                if image is not None:
                    # Store reference (zero-copy)
                    self.zenoh_camera_image = image
                    # Process image for collision detection
                    self.process_zenoh_camera_image(image)
                else:
                    pass  # 조용히 무시
                
        except Exception as e:
            print(f"⚠️ Error processing camera image: {e}")
    
    def on_bounding_boxes_received(self, sample):
        """Handle received bounding boxes from Zenoh"""
        try:
            # Zenoh payload 처리
            if hasattr(sample.payload, 'to_string'):
                payload_str = sample.payload.to_string()
            else:
                payload_str = str(sample.payload)
            
            # JSON 파싱
            import json
            bbox_data = json.loads(payload_str)
            
            print(f"🎯 Received bounding boxes: {len(bbox_data.get('objects', []))} objects")
            
            # 바운딩 박스 데이터 저장
            self.zenoh_detected_objects = bbox_data.get('objects', [])
            
            # 충돌 감지
            if self.zenoh_detected_objects:
                self.process_zenoh_bounding_boxes()
            
        except Exception as e:
            print(f"⚠️ Error processing bounding boxes: {e}")
    
    def process_zenoh_objects(self, objects):
        """Zenoh 객체들을 충돌 추적 시스템으로 처리"""
        try:
            if not self.collision_tracker:
                return
            
            # 1단계: 차량과 보행자 분리
            vehicles = []
            pedestrians = []
            
            for obj in objects:
                bbox = obj.get('bbox_2d', {})
                distance = obj.get('distance', 100)
                object_type = obj.get('type', 'unknown')
                actor_id = obj.get('actor_id', 'Unknown')
                
                # CARLA ground truth 형식 처리
                if isinstance(bbox, dict) and all(key in bbox for key in ['x_min', 'y_min', 'x_max', 'y_max']):
                    x = bbox['x_min']
                    y = bbox['y_min'] 
                    w = bbox['width']
                    h = bbox['height']
                    area = w * h
                    aspect_ratio = w / h if h > 0 else 1.0
                else:
                    continue  # 유효하지 않은 bbox 형식
                
                # 객체 분류
                if object_type == 'vehicle' or aspect_ratio > 1.5:  # 차량
                    vehicles.append({
                        'obj': obj,
                        'bbox': bbox,
                        'distance': distance,
                        'area': area,
                        'aspect_ratio': aspect_ratio,
                        'actor_id': actor_id
                    })
                elif object_type == 'pedestrian' or aspect_ratio < 0.8:  # 보행자
                    pedestrians.append({
                        'obj': obj,
                        'bbox': bbox,
                        'distance': distance,
                        'area': area,
                        'aspect_ratio': aspect_ratio,
                        'actor_id': actor_id
                    })
            
            # 2단계: 충돌 추적 시스템으로 처리
            collision_detected = self.collision_tracker.detect_vehicle_pedestrian_collision(vehicles, pedestrians)
            fall_detected = self.collision_tracker.detect_pedestrian_fall(pedestrians)
            
            # 3단계: 충돌 감지 시에만 출력
            if collision_detected:
                print("🚨 충돌 감지!")
            
            # 4단계: 추격 타겟 결정
            if fall_detected:
                target_vehicle = self.collision_tracker.get_target_collision_vehicle(objects)
                if target_vehicle:
                    self.is_chasing = True
                    self.apply_simple_chase_control(target_vehicle)
            
            # 5단계: 차량 추적 정보 업데이트
            self.collision_tracker.update_vehicle_tracking(objects)
            
            # 6단계: 통계 업데이트
            stats = self.collision_tracker.get_collision_statistics()
            self.chase_statistics['collision_events'] = stats['collision_events_count']
                
        except Exception:
            pass  # 조용히 무시
    
    def process_zenoh_bounding_boxes(self):
        """Process bounding boxes from Zenoh for collision detection"""
        try:
            print(f"🔍 Processing {len(self.zenoh_detected_objects)} bounding boxes from Zenoh")
            
            # 1단계: 차량과 보행자 분리 및 충돌 가능성 체크
            vehicles = []
            pedestrians = []
            
            for obj in self.zenoh_detected_objects:
                bbox = obj.get('bbox_2d', {})
                distance = obj.get('distance', 100)
                object_type = obj.get('type', 'unknown')
                actor_id = obj.get('actor_id', 'Unknown')
                
                # CARLA ground truth 형식 처리
                if isinstance(bbox, dict) and all(key in bbox for key in ['x_min', 'y_min', 'x_max', 'y_max']):
                    x = bbox['x_min']
                    y = bbox['y_min'] 
                    w = bbox['width']
                    h = bbox['height']
                    area = w * h
                    aspect_ratio = w / h if h > 0 else 1.0
                else:
                    continue  # 유효하지 않은 bbox 형식
                
                # 객체 분류
                if object_type == 'vehicle' or aspect_ratio > 1.5:  # 차량
                    vehicles.append({
                        'obj': obj,
                        'bbox': bbox,
                        'distance': distance,
                        'area': area,
                        'aspect_ratio': aspect_ratio,
                        'actor_id': actor_id
                    })
                elif object_type == 'pedestrian' or aspect_ratio < 0.8:  # 보행자
                    pedestrians.append({
                        'obj': obj,
                        'bbox': bbox,
                        'distance': distance,
                        'area': area,
                        'aspect_ratio': aspect_ratio,
                        'actor_id': actor_id
                    })
            
            # 2단계: 차량-보행자 충돌 감지
            self.check_vehicle_pedestrian_collision(vehicles, pedestrians)
            
            # 3단계: 보행자 넘어짐 감지
            self.detect_pedestrian_fall(pedestrians)
            
            # 4단계: 충돌 차량 추격 시작
            if self.pedestrian_fall_detected and self.collision_vehicle_ids:
                self.start_chase_collision_vehicles()
                
        except Exception as e:
            print(f"⚠️ Error processing Zenoh bounding boxes: {e}")
            import traceback
            traceback.print_exc()
    
    def check_vehicle_pedestrian_collision(self, vehicles, pedestrians):
        """차량과 보행자의 충돌 가능성 체크"""
        try:
            collision_detected = False
            
            for vehicle in vehicles:
                for pedestrian in pedestrians:
                    # 거리 체크 (30m 이내)
                    if vehicle['distance'] <= 30 and pedestrian['distance'] <= 30:
                        # 바운딩 박스 겹침 체크
                        if self.check_bbox_overlap(vehicle['bbox'], pedestrian['bbox']):
                            print(f"🚨 VEHICLE-PEDESTRIAN COLLISION DETECTED!")
                            print(f"   Vehicle ID: {vehicle['actor_id']}, Pedestrian ID: {pedestrian['actor_id']}")
                            
                            # 충돌 차량 ID 저장
                            if vehicle['actor_id'] not in self.collision_vehicle_ids:
                                self.collision_vehicle_ids.append(vehicle['actor_id'])
                            
                            # 의심스러운 차량으로 마킹
                            self.suspicious_vehicles[vehicle['actor_id']] = {
                                'last_seen': time.time(),
                                'location': vehicle['obj'].get('world_location', [0, 0, 0]),
                                'collision_time': time.time()
                            }
                            
                            collision_detected = True
                            self.chase_statistics['collision_events'] += 1
            
            if collision_detected:
                print(f"🚨 Collision tracking: {len(self.collision_vehicle_ids)} vehicles involved")
                self.collision_timestamp = time.time()
                
        except Exception as e:
            print(f"⚠️ Error checking vehicle-pedestrian collision: {e}")
    
    def check_bbox_overlap(self, bbox1, bbox2):
        """두 바운딩 박스의 겹침 여부 체크"""
        try:
            # bbox 형식: {'x_min': x, 'y_min': y, 'x_max': x+w, 'y_max': y+h}
            x1_min, y1_min = bbox1['x_min'], bbox1['y_min']
            x1_max, y1_max = bbox1['x_max'], bbox1['y_max']
            
            x2_min, y2_min = bbox2['x_min'], bbox2['y_min']
            x2_max, y2_max = bbox2['x_max'], bbox2['y_max']
            
            # 겹침 체크 (약간의 여유를 둠)
            overlap_x = x1_max >= x2_min - 50 and x2_max >= x1_min - 50
            overlap_y = y1_max >= y2_min - 50 and y2_max >= y1_min - 50
            
            return overlap_x and overlap_y
            
        except Exception as e:
            print(f"⚠️ Error checking bbox overlap: {e}")
            return False
    
    def detect_pedestrian_fall(self, pedestrians):
        """보행자 넘어짐 감지"""
        try:
            for pedestrian in pedestrians:
                # 보행자가 넘어진 상태 감지 (aspect_ratio가 매우 낮거나 높음)
                aspect_ratio = pedestrian['aspect_ratio']
                area = pedestrian['area']
                distance = pedestrian['distance']
                
                # 넘어진 보행자 조건:
                # 1. aspect_ratio가 매우 낮음 (< 0.4) - 누워있음
                # 2. 또는 aspect_ratio가 매우 높음 (> 3.0) - 세워져 있음
                # 3. 충돌 차량이 근처에 있음
                is_fallen = (aspect_ratio < 0.4 or aspect_ratio > 3.0) and area > 8000
                has_collision_vehicle = any(vehicle_id in self.collision_vehicle_ids for vehicle_id in self.suspicious_vehicles.keys())
                
                if is_fallen and has_collision_vehicle:
                    print(f"🚨 PEDESTRIAN FALL DETECTED!")
                    print(f"   Pedestrian ID: {pedestrian['actor_id']}")
                    print(f"   Aspect ratio: {aspect_ratio:.2f}, Area: {area}")
                    print(f"   Distance: {distance:.1f}m")
                    
                    self.pedestrian_fall_detected = True
                    self.chase_statistics['collision_events'] += 1
                    
        except Exception as e:
            print(f"⚠️ Error detecting pedestrian fall: {e}")
    
    def start_chase_collision_vehicles(self):
        """충돌에 관여한 차량들을 추격 시작"""
        try:
            if not self.collision_vehicle_ids or not self.pedestrian_fall_detected:
                return

            print(f"🚔 Starting chase of collision vehicles: {self.collision_vehicle_ids}")

            # 충돌 차량들 중 가장 가까운 차량을 타겟으로 선택
            target_vehicle = None
            min_distance = float('inf')

            for obj in self.zenoh_detected_objects:
                if obj.get('actor_id') in self.collision_vehicle_ids:
                    distance = obj.get('distance', 100)
                    if distance < min_distance:
                        min_distance = distance
                        target_vehicle = obj

            if target_vehicle:
                print(f"🎯 Target collision vehicle: {target_vehicle.get('actor_id')} at {min_distance:.1f}m")

                # 추격 시작
                if not self.is_chasing:
                    self.is_chasing = True
                    self.chase_start_time = time.time()

                    # Emergency alert 발행
                    self.publish_emergency_alert()

                    # 타겟 차량 액터 찾기 (GPS 추적용)
                    try:
                        actor_id = int(target_vehicle.get('actor_id'))
                        self.target_vehicle_actor = self.world.get_actor(actor_id)
                        if self.target_vehicle_actor:
                            print(f"✅ Target vehicle actor found: {actor_id}")
                    except Exception as e:
                        print(f"⚠️ Could not find target vehicle actor: {e}")

                self.apply_simple_chase_control(target_vehicle)
            else:
                print("⚠️ No collision vehicles found in current detection")

        except Exception as e:
            print(f"⚠️ Error starting chase of collision vehicles: {e}")
    
    def process_zenoh_camera_image(self, image):
        """Process camera image from Zenoh - now only used for display"""
        try:
            print(f"🔍 Processing Zenoh camera image for display: {image.shape if image is not None else 'None'}")
            
            # 카메라 이미지만 저장 (충돌 감지는 바운딩 박스 토픽에서 처리)
            if image is not None:
                self.zenoh_camera_image = image
                print(f"📷 Stored camera image: {image.shape}")
            
        except Exception as e:
            print(f"⚠️ Error processing Zenoh camera image: {e}")
            import traceback
            traceback.print_exc()
    
    def detect_objects_opencv(self, image):
        """Detect objects using improved OpenCV method"""
        try:
            if image is None or image.size == 0:
                return []
            
            # 이미지 크기 확인
            height, width = image.shape[:2]
            
            # Convert to HSV for better color detection
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            
            # Use adaptive thresholding for better edge detection
            thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            
            # Morphological operations to connect nearby edges
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            detected_objects = []
            
            for i, contour in enumerate(contours):
                # Filter by area (more restrictive)
                area = cv2.contourArea(contour)
                if area > 2000:  # Increased minimum area threshold
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Filter by aspect ratio and size (vehicle-like objects)
                    aspect_ratio = w / h if h > 0 else 0
                    
                    # Vehicle-like objects: wider than tall, reasonable size
                    if 0.5 < aspect_ratio < 3.0 and w > 30 and h > 20:
                        # Estimate distance based on bounding box size
                        estimated_distance = self._estimate_distance_from_bbox(area, aspect_ratio)
                        
                        # Only detect objects within reasonable distance
                        if estimated_distance <= 100:  # 100m limit
                            # Create object data similar to CARLA format
                            obj = {
                                'actor_id': f'opencv_object_{i}',
                                'bbox_2d': [x, y, w, h],
                                'world_location': [x, y, 0],  # Simplified world location
                                'object_type': 'vehicle' if aspect_ratio > 1.2 else 'pedestrian',
                                'confidence': min(1.0, area / 10000.0),  # Adjusted confidence
                                'distance': estimated_distance,
                                'area': area,
                                'aspect_ratio': aspect_ratio
                    }
                    detected_objects.append(obj)
            
            # Sort by confidence (larger objects first)
            detected_objects.sort(key=lambda x: x['confidence'], reverse=True)
            
            return detected_objects
            
        except Exception as e:
            print(f"⚠️ Error in OpenCV object detection: {e}")
            return []
    
    def _get_bbox_area(self, bbox):
        """Get bounding box area from different formats"""
        try:
            if isinstance(bbox, dict) and 'width' in bbox and 'height' in bbox:
                return bbox['width'] * bbox['height']
            elif isinstance(bbox, list) and len(bbox) >= 4:
                return bbox[2] * bbox[3]
            else:
                return 0
        except Exception as e:
            print(f"⚠️ Error getting bbox area: {e}")
            return 0

    def _estimate_distance_from_bbox(self, area, aspect_ratio):
        """Estimate distance from bounding box area and aspect ratio"""
        try:
            # Rough estimation based on typical vehicle sizes
            # Larger objects are closer, smaller objects are farther
            
            # Base distance estimation (empirical values)
            if aspect_ratio > 2.0:  # Very wide (likely a vehicle)
                base_distance = 50.0
            elif aspect_ratio > 1.5:  # Wide (vehicle)
                base_distance = 40.0
            elif aspect_ratio > 1.0:  # Square-ish (vehicle or large object)
                base_distance = 30.0
            else:  # Tall (pedestrian)
                base_distance = 20.0
            
            # Adjust based on area (larger area = closer)
            if area > 50000:
                distance_factor = 0.5  # Very close
            elif area > 30000:
                distance_factor = 0.7  # Close
            elif area > 15000:
                distance_factor = 1.0  # Medium
            elif area > 8000:
                distance_factor = 1.3  # Far
            else:
                distance_factor = 1.8  # Very far
            
            estimated_distance = base_distance * distance_factor
            return min(estimated_distance, 200.0)  # Cap at 200m
            
        except Exception as e:
            print(f"⚠️ Error estimating distance: {e}")
            return 50.0  # Default distance
    
    def handle_zenoh_chase_control(self):
        """Handle chase control based on Zenoh camera data"""
        try:
            print(f"🚔 Zenoh chase control: {len(self.zenoh_detected_objects)} objects")
            print(f"🚔 Chase status: is_chasing={self.is_chasing}, collisions={self.chase_statistics['collision_events']}")
            
            # 충돌 이벤트가 있고 아직 추격을 시작하지 않은 경우
            if self.chase_statistics['collision_events'] > 0 and not self.is_chasing:
                print("🚨 Collision detected - starting chase from Zenoh data")
                
                # 가장 큰 객체를 타겟으로 선택
                if self.zenoh_detected_objects:
                    target_obj = max(self.zenoh_detected_objects, 
                                   key=lambda x: self._get_bbox_area(x.get('bbox_2d', {})))
                    
                    print(f"🎯 Target selected: {target_obj['actor_id']}")
                    print(f"🎯 Target bbox: {target_obj['bbox_2d']}")
                    
                    # 추격 시작
                    self.is_chasing = True
                    print("🚔 Chase started - is_chasing set to True")
                    
                    # 간단한 추격 제어 명령 생성
                    self.apply_simple_chase_control(target_obj)
                else:
                    print("⚠️ No detected objects for chase target")
            
            # 이미 추격 중인 경우 제어 업데이트
            elif self.is_chasing and self.zenoh_detected_objects:
                print("🚔 Chase in progress - updating control")
                
                # 가장 큰 객체를 타겟으로 유지
                target_obj = max(self.zenoh_detected_objects, 
                               key=lambda x: self._get_bbox_area(x.get('bbox_2d', {})))
                
                # 제어 명령 업데이트
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
                                    key=lambda x: self._get_bbox_area(x.get('bbox_2d', {})))
                    bbox = largest_obj.get('bbox_2d', {})
                    distance = largest_obj.get('distance_estimate', 100)
                    
                    # CARLA 형식 또는 OpenCV 형식 처리
                    if isinstance(bbox, dict) and 'width' in bbox:
                        w, h = bbox['width'], bbox['height']
                    elif isinstance(bbox, list) and len(bbox) >= 4:
                        w, h = bbox[2], bbox[3]
                    else:
                        return
                    
                    area = w * h
                    
                    # 카메라 화면에 잡힌 큰 객체이면서 적당한 거리에 있는 경우
                    if area > 8000 and distance <= 50:  # 사람-차량 충돌 감지 거리
                        self.is_chasing = True
                        self.chase_statistics['collision_events'] += 1
                        self.apply_simple_chase_control(largest_obj)
            
        except Exception as e:
            print(f"⚠️ Error in Zenoh chase control: {e}")
            import traceback
            traceback.print_exc()
    
    def apply_simple_chase_control(self, target_obj):
        """Apply simple chase control based on target object"""
        try:
            if not self.vehicle:
                print("⚠️ No vehicle available for control")
                return
            
            print(f"🎮 Applying chase control to vehicle {self.vehicle.id}")
            
            # 차량 상태 확인
            vehicle_location = self.vehicle.get_location()
            vehicle_transform = self.vehicle.get_transform()
            print(f"🚗 Vehicle location: {vehicle_location}")
            print(f"🚗 Vehicle transform: {vehicle_transform}")
            
            # 차량이 제어 가능한 상태인지 확인
            if self.vehicle.is_at_traffic_light():
                print("⚠️ Vehicle is at traffic light - control may be limited")
            
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
            
            print(f"🎯 Target center: ({target_center_x:.1f}, {target_center_y:.1f})")
            print(f"🎯 Screen center: ({screen_center_x:.1f}, {screen_center_y:.1f})")
            print(f"🎯 Offset: dx={dx:.1f}, dy={dy:.1f}")
            
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
            
            print(f"🎮 Control calculation: steer_factor={steer_factor:.3f}, "
                  f"distance={distance}m, target_size={target_size}")
            
            # CARLA 제어 명령 생성
            control = carla.VehicleControl()
            control.throttle = throttle
            control.steer = steer
            control.brake = brake
            control.hand_brake = False
            control.reverse = False
            
            # 제어 명령 적용
            print(f"🎮 Applying control to vehicle: throttle={throttle:.2f}, "
                  f"steer={steer:.2f}, brake={brake:.2f}")
            self.vehicle.apply_control(control)
            
            # 차량 속도 확인
            velocity = self.vehicle.get_velocity()
            speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
            print(f"🚗 Vehicle speed: {speed:.2f} m/s")
            
        except Exception as e:
            print(f"⚠️ Error applying simple chase control: {e}")
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

    def publish_emergency_alert(self):
        """Emergency alert 발행 (추격 시작 시)"""
        try:
            if not self.zenoh_emergency_session or self.emergency_alert_sent:
                return

            alert_data = {
                'type': 'chase_started',
                'unit_id': 'UNIT-001',  # Police unit ID
                'timestamp': time.time(),
                'message': 'Suspect vehicle locked on - pursuit initiated',
                'priority': 'CRITICAL',
                'chase_start_time': self.chase_start_time
            }

            # Zenoh로 emergency alert 발행
            topic = 'police/central/alerts/emergency/UNIT-001'
            self.zenoh_emergency_session.put(topic, json.dumps(alert_data))

            self.emergency_alert_sent = True
            print(f"🚨 Emergency alert published: Chase started!")

        except Exception as e:
            print(f"⚠️ Error publishing emergency alert: {e}")

    def publish_suspect_gps(self, target_location):
        """범인 차량 GPS 데이터 발행"""
        try:
            if not self.zenoh_emergency_session or not self.is_chasing:
                return

            # CARLA location을 GPS 좌표로 변환 (간단한 변환)
            # 실제로는 CARLA의 world에서 GNSS 센서를 사용해야 하지만
            # 여기서는 단순화된 변환 사용
            suspect_gps_data = {
                'type': 'suspect_gps',
                'timestamp': time.time(),
                'location': {
                    'x': target_location.x,
                    'y': target_location.y,
                    'z': target_location.z
                },
                # 간단한 위도/경도 변환 (Town10 기준 대략적인 변환)
                'latitude': 37.7749 + (target_location.y / 111000),  # 대략적 변환
                'longitude': -122.4194 + (target_location.x / (111000 * np.cos(np.radians(37.7749)))),
                'altitude': target_location.z,
                'chase_duration': time.time() - self.chase_start_time if self.chase_start_time else 0
            }

            # 디버그 로그
            print(f"📍 Publishing suspect GPS: x={target_location.x:.2f}, y={target_location.y:.2f}, z={target_location.z:.2f}")

            # Zenoh로 suspect GPS 발행
            topic = 'police/central/suspect/UNIT-001/gps'
            self.zenoh_emergency_session.put(topic, json.dumps(suspect_gps_data))

        except Exception as e:
            print(f"⚠️ Error publishing suspect GPS: {e}")
    
    def setup_modules(self):
        """모듈들 초기화"""
        try:
            # CARLA 바운딩 박스 감지기 활성화 (Zenoh와 함께 사용)
            print("🎯 Using CARLA bounding box detection alongside Zenoh")
            if self.camera_view and self.camera_view.bounding_box_detector:
                self.bounding_box_detector = self.camera_view.bounding_box_detector
                print("✅ CARLA bounding box detector activated")
            else:
                print("⚠️ CARLA bounding box detector not available")
                self.bounding_box_detector = None
            
            # 충돌 추적기 초기화
            self.collision_tracker = CollisionTracker(max_tracking_time=300.0)
            print("🚨 Collision tracker initialized")
            
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
            print("🧠 Chase planner initialized with high-speed parameters")
            
            # 차량 제어기 초기화 (사용하지 않음 - SimpleChasePlanner에서 직접 제어)
            # self.vehicle_controller = ChaseVehicleController(self.world, self.vehicle)
            # print("🎮 Vehicle controller initialized")
            
            print("✅ All modules initialized")
            return True
            
        except Exception as e:
            print(f"❌ Error setting up modules: {e}")
            return False
    
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

            # Suspect GPS 발행 (추격 중일 때)
            if self.target_vehicle_actor:
                try:
                    suspect_location = self.target_vehicle_actor.get_location()
                    print(f"✅ Using actual target vehicle actor location")
                    self.publish_suspect_gps(suspect_location)
                except Exception as e:
                    # 타겟 액터를 찾을 수 없으면 타겟 위치 사용
                    print(f"⚠️ Target actor unavailable, using target_location: {e}")
                    self.publish_suspect_gps(target_location)
            else:
                # target_vehicle_actor가 없으면 target_location 사용
                print(f"⚠️ No target_vehicle_actor, using target_location from bbox")
                self.publish_suspect_gps(target_location)
            
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
    
    def _trigger_manual_collision(self):
        """수동 충돌 트리거 (테스트용)"""
        try:
            print("🧪 MANUAL COLLISION TRIGGERED!")
            
            # 가상의 충돌 이벤트 생성
            fake_collision_event = {
                'actor_id': 99999,  # 가상 ID
                'collision_score': 0.9,
                'timestamp': time.time(),
                'world_location': (0, 0, 0),  # 현재 위치
                'bbox_2d': {'width': 100, 'height': 50},  # 누워진 상태 시뮬레이션
                'avoid_pedestrian': True,
                'collision_indicators': ['manual_trigger'],
                'test_mode': True
            }
            
            # 충돌 감지기에 직접 전달
            if self.collision_detector:
                collision_events = [fake_collision_event]
                print(f"🚨 Triggering collision detection with fake event")
                
                # 차량 추적기로 전달
                if self.vehicle_tracker:
                    self.vehicle_tracker.detect_collision_vehicle([], collision_events)
            
        except Exception as e:
            print(f"⚠️ Error triggering manual collision: {e}")
            import traceback
            traceback.print_exc()
    
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
                    # print(f"📷 Displaying Zenoh camera data: {self.zenoh_camera_image.shape}")
                
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
            print("🚔 Starting Auto Chase Vehicle Control...")
            
            # CARLA 연결
            if not self.connect_to_carla():
                return
            
            # Zenoh 초기화
            self.setup_zenoh()
            
            # 기존 차량 찾기
            if not self.find_existing_vehicle():
                return
            
            # 카메라 설정
            if not self.setup_camera():
                return
            
            # IMU 센서 설정
            self.setup_imu_sensor()
            
            # 모듈들 초기화
            if not self.setup_modules():
                return
            
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
                            elif event.key == pygame.K_c:  # C키로 수동 충돌 트리거
                                self._trigger_manual_collision()
                    
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
                    
                    # Zenoh 카메라 기반 추격 제어 (추가)
                    if self.use_zenoh_camera_only:
                        if self.zenoh_camera_image is not None:
                            self.handle_zenoh_chase_control()
                    
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
    
    def cleanup(self):
        """리소스 정리"""
        try:
            print("\n🧹 Cleaning up resources...")
            
            # 모듈들 정리
            if self.vehicle_tracker:
                self.vehicle_tracker.reset_tracking()
            if self.chase_planner:
                self.chase_planner.reset_chase()
            # if self.vehicle_controller:  # 사용하지 않음
            #     self.vehicle_controller.destroy()
            
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
                print(f"ℹ️ Not destroying vehicle {self.vehicle.id} (may be used by other systems)")
                self.vehicle = None
            
            # Zenoh 정리
            if self.zenoh_collision_manager:
                self.zenoh_collision_manager.cleanup()
                self.zenoh_collision_manager = None
                print("✅ Zenoh collision manager cleaned up")

            # Zenoh emergency session 정리
            if self.zenoh_emergency_session:
                try:
                    self.zenoh_emergency_session.close()
                    print("✅ Zenoh emergency session closed")
                except Exception as e:
                    print(f"⚠️ Error closing Zenoh emergency session: {e}")
                self.zenoh_emergency_session = None
            
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
