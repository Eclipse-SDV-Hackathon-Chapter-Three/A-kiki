#!/usr/bin/env python3
"""
Auto Chase Vehicle Controller
자동 추격 차량 제어 메인 클래스
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
import struct
from typing import Optional, List, Callable, Any, Dict

# Zenoh imports
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    print("Warning: zenoh not available, control commands will not be published")

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 not available, semantic LiDAR will not be subscribed")

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
from chase.sensors.semantic_lidar_manager import SemanticLidarManager
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
        
        # 모듈화된 컴포넌트들
        self.lidar_visualizer = None
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
        
        # Semantic LIDAR 매니저
        self.semantic_lidar = None
        self.use_semantic_lidar = True  # Semantic LIDAR 사용 여부
        self.semantic_lidar_data = None  # Zenoh에서 받은 Semantic LiDAR 데이터
        self.last_semantic_lidar_time = 0  # 마지막 LiDAR 데이터 수신 시간
        
        # ROS2 Semantic LiDAR
        self.ros2_node = None  # ROS2 노드
        self.ros2_semantic_lidar_sub = None  # ROS2 Semantic LiDAR 구독자
        self.use_ros2_semantic_lidar = True  # ROS2 Semantic LiDAR 사용 여부
        self.target_vehicle = None  # 추적 대상 차량 (사고 차량)
        self.vehicle_tracking_enabled = True  # 차량 추적 활성화
        self.accident_detection_enabled = True  # 사고 감지 활성화
        
        
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
        """Zenoh 설정 (Zero-Copy 우선 사용)"""
        try:
            if not ZENOH_AVAILABLE:
                print("⚠️ Zenoh not available, skipping setup")
                return False
            
            # Zero-Copy 카메라 구독자 설정
            if self.use_zero_copy:
                self.zero_copy_camera = ZeroCopyCameraSubscriber()
                
                # 콜백 함수 설정
                self.zero_copy_camera.set_camera_callback(self.on_zenoh_camera_received)
                self.zero_copy_camera.set_bbox_callback(self.on_zenoh_bounding_boxes_received)
                
                # Zenoh 연결
                if self.zero_copy_camera.setup_zenoh():
                    self.zero_copy_camera.start()
                    print("✅ Zero-Copy camera subscriber setup successful")
                    return True
                else:
                    print("❌ Failed to setup Zero-Copy camera subscriber, falling back to legacy")
                    self.use_zero_copy = False
            
            # Fallback: 기존 ZenohCollisionManager 사용
            if not self.use_zero_copy:
                self.zenoh_collision_manager = ZenohCollisionManager()
                
                # 콜백 함수 설정
                self.zenoh_collision_manager.set_camera_callback(self.on_zenoh_camera_received)
                self.zenoh_collision_manager.set_bbox_callback(self.on_zenoh_bounding_boxes_received)
                
                # Zenoh 연결
                if self.zenoh_collision_manager.setup_zenoh():
                    print("✅ Zenoh collision manager setup successful (legacy mode)")
                    
                    # Semantic LiDAR 데이터 구독 추가
                    self.setup_semantic_lidar_subscription()
                    
                    return True
                else:
                    print("❌ Failed to setup Zenoh collision manager")
                    return False
                
        except Exception as e:
            print(f"❌ Error setting up Zenoh: {e}")
            self.zenoh_collision_manager = None
            return False
    
    def setup_ros2(self):
        """ROS2 설정 및 Semantic LiDAR 구독"""
        try:
            if not ROS2_AVAILABLE:
                print("⚠️ ROS2 not available")
                return False
            
            # ROS2 초기화 (이미 초기화되었는지 확인)
            if not rclpy.ok():
                rclpy.init()
            
            # ROS2 노드 생성
            self.ros2_node = Node('auto_chase_vehicle_control')
            
            # ROS2 Semantic LiDAR 구독자 설정
            self.ros2_semantic_lidar_sub = self.ros2_node.create_subscription(
                PointCloud2,
                '/carla/hero/semantic_lidar/point_cloud',
                self.on_ros2_semantic_lidar_received,
                10
            )
            
            print("✅ ROS2 Semantic LiDAR subscription setup successful")
            return True
            
        except Exception as e:
            print(f"❌ Error setting up ROS2: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def on_ros2_semantic_lidar_received(self, msg):
        """ROS2 Semantic LiDAR 데이터 수신 콜백"""
        try:
            # PointCloud2 메시지를 파싱
            points = self._parse_pointcloud2_msg(msg)
            
            if not points:
                return
            
            # Semantic LiDAR 데이터 저장
            self.semantic_lidar_data = {
                'points': points,
                'point_count': len(points),
                'timestamp': time.time()
            }
            self.last_semantic_lidar_time = time.time()
            
            print(f"🎯 ROS2 Semantic LiDAR: {len(points)}개 포인트 수신")
            
            # 차량 포인트만 추출하여 추격 대상으로 설정
            self._process_semantic_lidar_for_vehicle_tracking(self.semantic_lidar_data)
            
            # LiDAR 시각화 업데이트
            if self.lidar_visualizer and self.camera_manager:
                try:
                    print(f"🎯 Updating LiDAR visualization with {len(points)} points")
                    camera_transform = self.camera_manager.get_camera_transform()
                    camera_intrinsic = self.camera_manager.get_camera_intrinsic()
                    
                    # Semantic LiDAR 데이터 업데이트
                    self.lidar_visualizer.update_semantic_lidar_data(points)
                    
                    # LiDAR transform 가져오기 (CARLA에서)
                    lidar_transform = self.get_lidar_transform()
                    
                    # 시각화 업데이트 (실제 화면에 그리기)
                    self.lidar_visualizer.update_visualization(
                        points, 
                        lidar_transform,
                        camera_transform, 
                        camera_intrinsic
                    )
                    print(f"✅ LiDAR visualization updated successfully")
                except Exception as e:
                    print(f"⚠️ Error updating LiDAR visualization: {e}")
                    import traceback
                    traceback.print_exc()
            else:
                print(f"⚠️ LiDAR visualizer: {self.lidar_visualizer}, Camera manager: {self.camera_manager}")
            
        except Exception as e:
            print(f"❌ Error processing ROS2 semantic LiDAR data: {e}")
            import traceback
            traceback.print_exc()
    
    def _parse_pointcloud2_msg(self, msg):
        """PointCloud2 메시지를 파싱하여 포인트 리스트 반환"""
        try:
            points = []
            point_size = msg.point_step  # 24 bytes per point
            
            for i in range(msg.width):
                offset = i * point_size
                point_bytes = msg.data[offset:offset + point_size]
                
                # PointCloud2 구조: x, y, z, CosAngle (float32) + ObjIdx, ObjTag (uint32)
                x, y, z, cos_angle, obj_idx, obj_tag = struct.unpack('ffffII', point_bytes)
                
                points.append({
                    'x': float(x),
                    'y': float(y),
                    'z': float(z),
                    'cos_angle': float(cos_angle),
                    'object_id': int(obj_idx),
                    'semantic_id': int(obj_tag)
                })
            
            return points
            
        except Exception as e:
            print(f"❌ Error parsing PointCloud2 message: {e}")
            return []
    
    def setup_semantic_lidar_subscription(self):
        """Zenoh에서 Semantic LiDAR 데이터 구독 설정"""
        print("🔍 Setting up Semantic LiDAR subscription...")
        try:
            if not self.zenoh_collision_manager:
                print("⚠️ Zenoh collision manager not available")
                return False
                
            if not self.zenoh_collision_manager.session:
                print("⚠️ Zenoh session not available for semantic LiDAR subscription")
                return False
            
            print("🔍 Zenoh session available, declaring subscriber...")
            # Semantic LiDAR 데이터 구독
            self.zenoh_collision_manager.session.declare_subscriber(
                "carla/semantic_lidar/data", 
                self.on_semantic_lidar_data_received
            )
            print("✅ Semantic LiDAR subscription setup successful")
            return True
            
        except Exception as e:
            print(f"❌ Error setting up semantic LiDAR subscription: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def on_semantic_lidar_data_received(self, sample):
        """Semantic LiDAR 데이터 수신 콜백"""
        print("🎯 Semantic LiDAR data received!")
        try:
            import json
            # Zenoh 버전 호환성 처리
            if hasattr(sample.payload, 'decode'):
                payload_str = sample.payload.decode('utf-8')
            else:
                # ZBytes 객체인 경우
                payload_str = bytes(sample.payload).decode('utf-8')
            
            data = json.loads(payload_str)
            
            # Semantic LiDAR 데이터 저장
            self.semantic_lidar_data = data
            self.last_semantic_lidar_time = time.time()
            
            print(f"🎯 Semantic LiDAR: {data.get('point_count', 0)}개 포인트 수신")
            
            # 차량 포인트만 추출하여 추격 대상으로 설정
            self._process_semantic_lidar_for_vehicle_tracking(data)
            
            # LiDAR 시각화 업데이트
            if self.lidar_visualizer and self.camera_manager:
                try:
                    camera_transform = self.camera_manager.get_camera_transform()
                    camera_intrinsic = self.camera_manager.get_camera_intrinsic()
                    
                    # 포인트 클라우드 데이터 변환
                    points = data.get('points', [])
                    if points:
                        # CARLA 포인트 형식으로 변환
                        carla_points = []
                        for point in points:
                            carla_points.append({
                                'x': point['x'],
                                'y': point['y'], 
                                'z': point['z'],
                                'intensity': point['intensity'],
                                'object_id': point['object_id'],
                                'semantic_id': point['semantic_id']
                            })
                        
                        # LiDAR transform 가져오기
                        lidar_transform = self.get_lidar_transform()
                        
                        # LiDAR 시각화 업데이트
                        self.lidar_visualizer.update_visualization(
                            carla_points, 
                            lidar_transform,
                            camera_transform, 
                            camera_intrinsic
                        )
                        print(f"🎯 LiDAR 시각화 업데이트: {len(carla_points)}개 포인트")
                        
                except Exception as e:
                    print(f"⚠️ Error updating LiDAR visualization: {e}")
            else:
                print("⚠️ LiDAR visualizer or camera manager not available")
            
        except Exception as e:
            print(f"❌ Error processing semantic LiDAR data: {e}")
            import traceback
            traceback.print_exc()
    
    def _process_semantic_lidar_for_vehicle_tracking(self, lidar_data):
        """Semantic LiDAR 데이터에서 차량만 추출하여 추격 대상으로 설정"""
        try:
            points = lidar_data.get('points', [])
            if not points:
                return
            
            # 디버깅: semantic ID 분포 확인
            semantic_counts = {}
            for point in points:
                semantic_id = point.get('semantic_id', 0)
                semantic_counts[semantic_id] = semantic_counts.get(semantic_id, 0) + 1
            
            # 상위 10개 semantic ID 출력
            top_semantic = sorted(semantic_counts.items(), key=lambda x: x[1], reverse=True)[:10]
            print(f"🔍 Semantic ID 분포: {top_semantic}")
            
            # 차량과 보행자 포인트만 필터링
            vehicle_points = []
            pedestrian_points = []
            for point in points:
                semantic_id = point.get('semantic_id')
                if semantic_id == 10:  # 차량 (CARLA Vehicle ID)
                    vehicle_points.append(point)
                elif semantic_id == 4:  # 보행자 (CARLA Pedestrian ID)
                    pedestrian_points.append(point)
            
            if not vehicle_points and not pedestrian_points:
                print("🔍 Semantic LiDAR: 차량/보행자 포인트 없음")
                return
            
            if vehicle_points:
                print(f"🚗 Semantic LiDAR: {len(vehicle_points)}개 차량 포인트 발견")
            if pedestrian_points:
                print(f"🚶 Semantic LiDAR: {len(pedestrian_points)}개 보행자 포인트 발견")
            
            # 차량 포인트들을 그룹화하여 개별 차량으로 인식
            vehicle_objects = self._group_vehicle_points(vehicle_points)
            
            if vehicle_objects:
                # 가장 가까운 차량을 추격 대상으로 설정
                closest_vehicle = min(vehicle_objects, key=lambda v: v['distance'])
                print(f"🎯 추격 대상 차량 설정: 거리 {closest_vehicle['distance']:.2f}m")
                
                # 추격 시작
                self._start_vehicle_chase_from_lidar(closest_vehicle)
            
        except Exception as e:
            print(f"❌ Error processing LiDAR for vehicle tracking: {e}")
    
    def _group_vehicle_points(self, vehicle_points):
        """차량 포인트들을 그룹화하여 개별 차량 객체로 만듦"""
        try:
            import numpy as np
            
            if not vehicle_points:
                return []
            
            # 포인트들을 numpy 배열로 변환
            points_array = np.array([[p['x'], p['y'], p['z']] for p in vehicle_points])
            
            # 간단한 클러스터링 (거리 기반)
            clusters = []
            used_points = set()
            
            for i, point in enumerate(vehicle_points):
                if i in used_points:
                    continue
                
                # 현재 포인트를 중심으로 클러스터 생성
                cluster_points = [point]
                used_points.add(i)
                
                # 3미터 이내의 포인트들을 같은 클러스터로 그룹화
                for j, other_point in enumerate(vehicle_points):
                    if j in used_points:
                        continue
                    
                    distance = np.sqrt(
                        (point['x'] - other_point['x'])**2 + 
                        (point['y'] - other_point['y'])**2 + 
                        (point['z'] - other_point['z'])**2
                    )
                    
                    if distance < 3.0:  # 3미터 이내
                        cluster_points.append(other_point)
                        used_points.add(j)
                
                # 클러스터의 중심점과 거리 계산
                if len(cluster_points) > 5:  # 최소 5개 포인트 이상
                    center_x = np.mean([p['x'] for p in cluster_points])
                    center_y = np.mean([p['y'] for p in cluster_points])
                    center_z = np.mean([p['z'] for p in cluster_points])
                    
                    distance = np.sqrt(center_x**2 + center_y**2 + center_z**2)
                    
                    clusters.append({
                        'center': (center_x, center_y, center_z),
                        'distance': distance,
                        'point_count': len(cluster_points),
                        'points': cluster_points
                    })
            
            return clusters
            
        except Exception as e:
            print(f"❌ Error grouping vehicle points: {e}")
            return []
    
    def _start_vehicle_chase_from_lidar(self, vehicle_object):
        """LiDAR에서 감지된 차량을 추격 시작"""
        try:
            if not self.vehicle_tracker:
                print("⚠️ Vehicle tracker not available")
                return
            
            # 차량 추격 시작
            self.vehicle_tracker.is_tracking = True
            self.vehicle_tracker.target_vehicle_id = f"lidar_vehicle_{id(vehicle_object)}"
            self.vehicle_tracker.target_position = vehicle_object['center']
            self.vehicle_tracker.target_distance = vehicle_object['distance']
            
            print(f"🚗 LiDAR 기반 차량 추격 시작: ID={self.vehicle_tracker.target_vehicle_id}")
            print(f"   위치: {vehicle_object['center']}")
            print(f"   거리: {vehicle_object['distance']:.2f}m")
            print(f"   포인트 수: {vehicle_object['point_count']}")
            
        except Exception as e:
            print(f"❌ Error starting vehicle chase from LiDAR: {e}")
    
    def setup_semantic_lidar(self):
        """Semantic LIDAR 설정"""
        try:
            if not self.use_semantic_lidar:
                print("ℹ️ Semantic LIDAR disabled")
                return True
            
            if not self.vehicle:
                print("❌ No vehicle available for semantic LIDAR setup")
                return False
            
            # Semantic LIDAR 매니저 생성
            self.semantic_lidar = SemanticLidarManager(self.world, self.vehicle)
            
            # Zenoh 설정
            if self.semantic_lidar.setup_zenoh():
                print("✅ Semantic LIDAR Zenoh setup successful")
            else:
                print("⚠️ Semantic LIDAR Zenoh setup failed, continuing without publishing")
            
            # Semantic LIDAR 센서 설정
            if self.semantic_lidar.setup_semantic_lidar(
                location=carla.Location(x=0.0, y=0.0, z=2.5),
                rotation=carla.Rotation(pitch=0, yaw=0, roll=0),
                channels=32,
                range_meters=100.0,
                points_per_second=100000,
                rotation_frequency=10.0
            ):
                print("✅ Semantic LIDAR sensor setup successful")
                return True
            else:
                print("❌ Failed to setup semantic LIDAR sensor")
                return False
                
        except Exception as e:
            print(f"❌ Error setting up semantic LIDAR: {e}")
            return False
    
    def update_vehicle_tracking(self):
        """Semantic LIDAR 기반 차량 추적 업데이트"""
        try:
            if not self.semantic_lidar or not self.vehicle_tracking_enabled:
                return
            
            # 추적 중인 차량들 가져오기
            tracked_vehicles = self.semantic_lidar.get_tracked_vehicles()
            
            if not tracked_vehicles:
                self.target_vehicle = None
                return
            
            # 사고 차량이 있으면 우선 추적, 없으면 가장 가까운 차량 추적
            primary_accident_vehicle = self.semantic_lidar.get_primary_accident_vehicle()
            
            if primary_accident_vehicle:
                # 사고 차량 추적
                self.target_vehicle = primary_accident_vehicle
                distance = primary_accident_vehicle.get('distance_to_accident', 0)
                obj_id = primary_accident_vehicle.get('object_id', 0)
                priority = primary_accident_vehicle.get('priority', 0)
                
                print(f"🚨 TRACKING ACCIDENT VEHICLE {obj_id} at distance {distance:.1f}m (priority: {priority:.1f})")
                
                # 추적 대상 차량 정보를 Zenoh로 퍼블리시
                self._publish_target_vehicle_info(primary_accident_vehicle)
                
            else:
                # 일반 차량 추적
                closest_vehicle = self.semantic_lidar.get_closest_vehicle()
                
                if closest_vehicle:
                    self.target_vehicle = closest_vehicle
                    distance = closest_vehicle.get('distance', 0)
                    obj_id = closest_vehicle.get('object_id', 0)
                    
                    print(f"🎯 Tracking vehicle {obj_id} at distance {distance:.1f}m")
                    
                    # 추적 대상 차량 정보를 Zenoh로 퍼블리시
                    self._publish_target_vehicle_info(closest_vehicle)
            
        except Exception as e:
            print(f"⚠️ Error updating vehicle tracking: {e}")
    
    def _publish_target_vehicle_info(self, vehicle_info: Dict[str, Any]):
        """추적 대상 차량 정보를 Zenoh로 퍼블리시"""
        try:
            if not self.zenoh_collision_manager or not self.zenoh_collision_manager.zenoh_session:
                return
            
            # 추적 대상 차량 정보
            target_data = {
                'timestamp': time.time(),
                'target_vehicle': {
                    'object_id': vehicle_info.get('object_id', 0),
                    'center': vehicle_info.get('center', {}),
                    'distance': vehicle_info.get('distance', 0),
                    'velocity': vehicle_info.get('velocity', {}),
                    'bounding_box': vehicle_info.get('bounding_box', {}),
                    'point_count': vehicle_info.get('point_count', 0)
                },
                'chase_active': self.is_chasing,
                'source': 'semantic_lidar_tracking'
            }
            
            # JSON으로 직렬화
            import json
            json_data = json.dumps(target_data)
            
            # Zenoh로 퍼블리시
            self.zenoh_collision_manager.zenoh_session.put("carla/chase/target_vehicle", json_data)
            
        except Exception as e:
            print(f"⚠️ Error publishing target vehicle info: {e}")
    
    def get_target_vehicle_direction(self) -> Optional[Dict[str, float]]:
        """추적 대상 차량의 방향 벡터 반환"""
        try:
            if not self.target_vehicle:
                return None
            
            center = self.target_vehicle.get('center', {})
            if not center:
                return None
            
            # 차량 중심점으로의 방향 벡터
            direction = {
                'x': center.get('x', 0),
                'y': center.get('y', 0),
                'z': center.get('z', 0)
            }
            
            return direction
            
        except Exception as e:
            print(f"⚠️ Error getting target vehicle direction: {e}")
            return None
    
    def is_target_vehicle_visible(self) -> bool:
        """추적 대상 차량이 보이는지 확인"""
        return self.target_vehicle is not None
    
    def get_target_vehicle_distance(self) -> float:
        """추적 대상 차량까지의 거리 반환"""
        if not self.target_vehicle:
            return float('inf')
        
        return self.target_vehicle.get('distance', float('inf'))
    
    def update_accident_detection(self):
        """Semantic LIDAR 기반 사고 감지 업데이트"""
        try:
            if not self.semantic_lidar or not self.accident_detection_enabled:
                return
            
            # 활성 사고 목록 가져오기
            active_accidents = self.semantic_lidar.get_active_accidents()
            
            if active_accidents:
                print(f"🚨 {len(active_accidents)} active accidents detected!")
                
                for accident in active_accidents:
                    obj_id = accident.get('object_id', 0)
                    severity = accident.get('severity', 'unknown')
                    location = accident.get('center', {})
                    
                    print(f"   Accident ID {obj_id}: {severity} severity at {location}")
                    
                    # 사고 발생 시 추적 모드 활성화
                    if not self.is_chasing:
                        print("🎯 Activating chase mode due to accident detection")
                        self.is_chasing = True
                        
                        # 가장 가까운 차량을 추적 대상으로 설정
                        closest_vehicle = self.semantic_lidar.get_closest_vehicle()
                        if closest_vehicle:
                            self.target_vehicle = closest_vehicle
                            print(f"🎯 Targeting vehicle {closest_vehicle.get('object_id', 0)} for chase")
            
        except Exception as e:
            print(f"⚠️ Error updating accident detection: {e}")
    
    def get_active_accidents(self) -> List[Dict[str, Any]]:
        """활성 사고 목록 반환"""
        try:
            if not self.semantic_lidar:
                return []
            
            return self.semantic_lidar.get_active_accidents()
            
        except Exception as e:
            print(f"⚠️ Error getting active accidents: {e}")
            return []
    
    def clear_accident_history(self):
        """사고 기록 초기화"""
        try:
            if self.semantic_lidar:
                self.semantic_lidar.clear_accident_history()
                print("🧹 Accident history cleared")
        except Exception as e:
            print(f"⚠️ Error clearing accident history: {e}")
    
    
    def on_zenoh_camera_received(self, image):
        """Zenoh 카메라 이미지 수신 콜백 (Zero-Copy 또는 Legacy)"""
        try:
            if image is not None:
                self.zenoh_camera_image = image
                mode = "Zero-Copy" if self.use_zero_copy else "Legacy"
                print(f"📷 Zenoh camera image received ({mode}): {image.shape}")
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
            print(f"🔍 Processing {len(objects)} objects from Zenoh")
            
            if not self.unified_collision_detector:
                print("⚠️ Unified collision detector not initialized")
                return
            
            # 통합 충돌 감지기로 객체 처리 (콜백으로 충돌 감지 및 추격 처리)
            self.unified_collision_detector.process_objects(objects)
                
        except Exception as e:
            print(f"⚠️ Error processing Zenoh objects: {e}")
    
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
    
    def run(self):
        """메인 실행 루프"""
        try:
            print("🚔 Starting Auto Chase Vehicle Control...")
            
            # CARLA 연결
            if not self.connect_to_carla():
                return False
            
            # Zenoh 설정
            if not self.setup_zenoh():
                print("⚠️ Zenoh setup failed, continuing without Zenoh")
            
            # ROS2 설정
            if self.use_ros2_semantic_lidar and ROS2_AVAILABLE:
                if self.setup_ros2():
                    print("✅ ROS2 setup successful")
                else:
                    print("⚠️ ROS2 setup failed, continuing without ROS2 features")
                    self.use_ros2_semantic_lidar = False
            
            # Semantic LIDAR 설정
            if not self.setup_semantic_lidar():
                print("⚠️ Semantic LIDAR setup failed, continuing without LIDAR features")
            
            # 매니저들 초기화
            self.vehicle_manager = VehicleManager(self.world, self.vehicle)
            self.camera_manager = CameraManager(self.world, self.vehicle)
            self.sensor_manager = SensorManager(self.world, self.vehicle)
            self.chase_controller = ChaseController(self.vehicle, self.chase_planner)
            self.display_manager = DisplayManager()
            
            # 기존 차량 찾기
            if not self.vehicle_manager.find_existing_vehicle():
                print("❌ No vehicle found to control")
                return False
            
            # 찾은 차량으로 업데이트
            self.vehicle = self.vehicle_manager.vehicle
            print(f"🎯 Using vehicle: {self.vehicle.type_id} (ID: {self.vehicle.id})")
            
            # 매니저들의 차량 참조 업데이트
            self.camera_manager.vehicle = self.vehicle
            self.sensor_manager.vehicle = self.vehicle
            self.chase_controller.vehicle = self.vehicle
            
            # 차량을 찾은 후 Semantic LiDAR 설정 재시도
            print("🔍 Retrying Semantic LiDAR setup with found vehicle...")
            if self.setup_semantic_lidar():
                print("✅ Semantic LiDAR setup successful after finding vehicle")
            else:
                print("⚠️ Semantic LiDAR setup still failed")
            
            # camera_view 참조 업데이트
            self.camera_view = self.camera_manager.camera_view
            
            # 카메라 설정
            if not self.camera_manager.setup_camera():
                print("❌ Failed to setup camera")
                return False
            
            # IMU 센서 설정 (임시 비활성화)
            print("⚠️ IMU sensor temporarily disabled for stability")
            self.imu_sensor = None
            self.current_imu_data = None
            
            # 모듈들 초기화 (카메라 설정 후)
            if not self.setup_modules():
                print("❌ Failed to initialize modules")
                return False
            
            # LiDAR 시각화 및 센서 퓨전 모듈 초기화
            self._initialize_fusion_modules()
            
            print("✅ Auto chase vehicle control ready!")
            print("🎯 Monitoring for collisions...")
            print("🛑 Press ESC to exit")
            
            # 메인 루프
            while self.running:
                try:
                    current_time = time.time()
                    
                    # 업데이트 간격 체크
                    if current_time - self.last_update_time < self.update_interval:
                        time.sleep(0.001)
                        continue
                    
                    self.last_update_time = current_time
                    
                    # 1. ROS2 노드 스핀 (Semantic LiDAR 데이터 수신)
                    if self.use_ros2_semantic_lidar and self.ros2_node:
                        try:
                            rclpy.spin_once(self.ros2_node, timeout_sec=0.001)
                        except Exception as e:
                            print(f"⚠️ ROS2 spin error: {e}")
                    
                    # 2. 인식 모듈 업데이트
                    detected_objects, collision_events = self.update_perception()
                    
                    # 2. 차량 상태 업데이트 (IMU 기반) - 안전하게
                    try:
                        self._update_vehicle_state()
                    except Exception as state_error:
                        print(f"⚠️ Error updating vehicle state: {state_error}")
                    
                    # 3. Semantic LIDAR 기반 차량 추적 업데이트
                    if self.semantic_lidar:
                        self.update_vehicle_tracking()
                        self.update_accident_detection()
                        
                        # LiDAR 시각화 및 센서 퓨전 업데이트
                        if (hasattr(self.semantic_lidar, 'last_point_cloud') and 
                            self.semantic_lidar.last_point_cloud and 
                            self.lidar_visualizer and self.sensor_fusion):
                            
                            # LiDAR 시각화 업데이트
                            try:
                                camera_transform = self.camera_manager.get_camera_transform()
                                camera_intrinsic = self.camera_manager.get_camera_intrinsic()
                                lidar_transform = self.get_lidar_transform()
                                self.lidar_visualizer.update_visualization(
                                    self.semantic_lidar.last_point_cloud, 
                                    lidar_transform,
                                    camera_transform, 
                                    camera_intrinsic
                                )
                                print(f"🎯 LiDAR 시각화 업데이트 완료: {len(self.semantic_lidar.last_point_cloud)}개 포인트")
                            except Exception as e:
                                print(f"⚠️ LiDAR 시각화 오류: {e}")
                            
                            # 센서 퓨전 수행
                            if detected_objects:
                                # LiDAR 객체 추출
                                filtered_lidar_data = self.lidar_visualizer.filter_semantic_lidar_data(
                                    self.semantic_lidar.last_point_cloud
                                )
                                lidar_objects = self.sensor_fusion.extract_lidar_objects(filtered_lidar_data)
                                
                                # 센서 퓨전
                                fused_objects = self.sensor_fusion.perform_sensor_fusion(detected_objects, lidar_objects)
                                print(f"🔗 센서 퓨전: {len(fused_objects)}개 객체 (카메라: {len(detected_objects)}, LiDAR: {len(lidar_objects)})")
                    
                    # 3. 추적 모듈 업데이트
                    is_tracking = self.update_tracking(detected_objects, collision_events)
                    
                    # Zenoh 카메라 기반 추격 제어 (추가)
                    if self.use_zenoh_camera_only:
                        if self.zenoh_camera_image is not None:
                            self.handle_zenoh_chase_control()
                    
                    # 4. 계획 및 제어 업데이트
                    if is_tracking or self.is_chasing:
                        self.update_planning_and_control()
                    
                    # 4. CARLA 바운딩 박스 데이터를 zenoh_detected_objects에 추가 (pygame 표시용)
                    if self.bounding_box_detector:
                        # CARLA 바운딩 박스 감지 실행 (감지 범위 확장)
                        carla_objects = self.bounding_box_detector.detect_pedestrians_and_vehicles(max_distance=200.0)
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
                            self.zenoh_detected_objects = []
                    
                    # 5. 카메라 뷰 표시 (CARLA 카메라 이미지 사용)
                    if self.camera_view and hasattr(self.camera_view, 'camera_image'):
                        camera_image = self.camera_view.camera_image
                    else:
                        camera_image = self.zenoh_camera_image
                    
                    self.display_manager.display_camera_view(
                        self.camera_view, 
                        camera_image, 
                        self.zenoh_detected_objects,
                        self.chase_statistics,
                        self.lidar_visualizer,
                        self.sensor_fusion,
                        self.fusion_display
                    )
                    
                except KeyboardInterrupt:
                    print("\n🛑 Keyboard interrupt received")
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
            collision_events = []
            if self.collision_detector and detected_objects:
                collision_events = self.collision_detector.detect_collisions(detected_objects)
            
            return detected_objects, collision_events
            
        except Exception as e:
            print(f"⚠️ Error in perception update: {e}")
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
            if not self.is_chasing:
                return
            
            # 1. Semantic LiDAR 기반 추격 제어 우선 사용
            if (self.semantic_lidar_data and 
                self.last_semantic_lidar_time > 0 and 
                time.time() - self.last_semantic_lidar_time < 1.0):  # 1초 이내 데이터
                
                print(f"🎯 Using Semantic LiDAR for chase control")
                self._apply_semantic_lidar_chase_control()
                return
            
            # 2. Fallback: Vehicle tracker 기반 추격
            if (self.vehicle_tracker and 
                self.vehicle_tracker.is_tracking and 
                self.vehicle_tracker.target_position):
                
                print(f"🎯 Using vehicle tracker for chase control")
                self._apply_vehicle_tracker_chase_control()
                return
            
            # 3. Fallback: Zenoh 카메라 기반 추격 제어 (차량만)
            if self.zenoh_detected_objects:
                # 차량만 필터링
                vehicle_objects = [obj for obj in self.zenoh_detected_objects 
                                 if obj.get('object_type') == 'vehicle']
                
                if vehicle_objects:
                    # 가장 큰 차량을 타겟으로 선택
                    target_obj = max(vehicle_objects, 
                                   key=lambda x: self._get_bbox_area(x.get('bbox_2d', {})))
                    
                    print(f"🎯 Applying camera-based chase control to vehicle: {target_obj.get('actor_id', 'Unknown')}")
                    self.apply_simple_chase_control(target_obj)
                else:
                    print("🔍 No vehicles detected in camera data")
            
        except Exception as e:
            print(f"⚠️ Error in planning and control update: {e}")
    
    def _apply_semantic_lidar_chase_control(self):
        """Semantic LiDAR 데이터를 사용한 추격 제어"""
        try:
            if not self.semantic_lidar_data:
                return
            
            # 차량 포인트만 추출
            points = self.semantic_lidar_data.get('points', [])
            vehicle_points = [p for p in points if p.get('semantic_id') == 10]
            
            if not vehicle_points:
                print("🔍 No vehicle points in LiDAR data")
                return
            
            # 가장 가까운 차량 포인트 찾기
            closest_point = min(vehicle_points, key=lambda p: 
                np.sqrt(p['x']**2 + p['y']**2 + p['z']**2))
            
            # 차량 위치를 기준으로 제어
            target_x = closest_point['x']
            target_y = closest_point['y']
            target_z = closest_point['z']
            
            # 거리 계산
            distance = np.sqrt(target_x**2 + target_y**2 + target_z**2)
            
            print(f"🎯 LiDAR 추격: 거리 {distance:.2f}m, 위치 ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            
            # 조향 각도 계산 (Y축 기준)
            steering_angle = np.arctan2(target_y, target_x) * 0.5  # 감쇠 계수
            steering_angle = np.clip(steering_angle, -1.0, 1.0)
            
            # 속도 제어 (거리 기반)
            if distance > 20.0:
                throttle = 0.8
                brake = 0.0
            elif distance > 10.0:
                throttle = 0.5
                brake = 0.0
            elif distance > 5.0:
                throttle = 0.2
                brake = 0.0
            else:
                throttle = 0.0
                brake = 0.3
            
            # 제어 명령 적용
            if self.vehicle:
                control = carla.VehicleControl()
                control.steer = float(steering_angle)
                control.throttle = float(throttle)
                control.brake = float(brake)
                control.hand_brake = False
                control.manual_gear_shift = False
                
                self.vehicle.apply_control(control)
                
                print(f"🎮 LiDAR 제어: 조향 {steering_angle:.3f}, 가속 {throttle:.3f}, 브레이크 {brake:.3f}")
            
        except Exception as e:
            print(f"❌ Error in semantic LiDAR chase control: {e}")
    
    def _apply_vehicle_tracker_chase_control(self):
        """Vehicle tracker를 사용한 추격 제어"""
        try:
            if not self.vehicle_tracker or not self.vehicle_tracker.target_position:
                return
            
            target_pos = self.vehicle_tracker.target_position
            target_distance = self.vehicle_tracker.target_distance
            
            print(f"🎯 Tracker 추격: 거리 {target_distance:.2f}m, 위치 {target_pos}")
            
            # 간단한 추격 로직
            if target_distance > 15.0:
                throttle = 0.7
                brake = 0.0
            elif target_distance > 8.0:
                throttle = 0.4
                brake = 0.0
            else:
                throttle = 0.1
                brake = 0.2
            
            # 조향은 목표 방향으로
            steering_angle = 0.0  # 간단한 구현
            
            if self.vehicle:
                control = carla.VehicleControl()
                control.steer = float(steering_angle)
                control.throttle = float(throttle)
                control.brake = float(brake)
                control.hand_brake = False
                control.manual_gear_shift = False
                
                self.vehicle.apply_control(control)
                
                print(f"🎮 Tracker 제어: 조향 {steering_angle:.3f}, 가속 {throttle:.3f}, 브레이크 {brake:.3f}")
            
        except Exception as e:
            print(f"❌ Error in vehicle tracker chase control: {e}")
    
    def apply_semantic_lidar_chase_control(self, target_vehicle):
        """Semantic LIDAR + IMU 기반 정확한 추격 제어"""
        try:
            if not self.vehicle or not target_vehicle:
                return
            
            # 현재 차량 상태 확인
            if not self.vehicle_position or not self.vehicle_orientation:
                print("⚠️ Vehicle state not available, using fallback control")
                self._fallback_semantic_control(target_vehicle)
                return
            
            # 타겟 차량의 3D 위치 정보
            center = target_vehicle.get('center', {})
            distance = target_vehicle.get('distance', 100)
            velocity = target_vehicle.get('velocity', {})
            
            if not center:
                print("⚠️ No target vehicle center information")
                return
            
            # 좌표 기반 추격 제어
            steer_value, throttle_value, brake_value = self._calculate_coordinate_based_control(
                center, distance, velocity
            )
            
            # 차량 제어 적용
            self._apply_vehicle_control(throttle=throttle_value, steer=steer_value, brake=brake_value)
            
            # 제어 상태 출력
            self._print_coordinate_control_status(center, distance, steer_value, throttle_value, brake_value)
            
        except Exception as e:
            print(f"⚠️ Error applying semantic LIDAR chase control: {e}")
    
    def _fallback_semantic_control(self, target_vehicle):
        """Fallback: 기존 3D 제어 방식"""
        try:
            center = target_vehicle.get('center', {})
            distance = target_vehicle.get('distance', 100)
            velocity = target_vehicle.get('velocity', {})
            
            steer_value = self._calculate_3d_steering_angle(center, distance)
            throttle_value, brake_value = self._calculate_3d_speed_control(distance, velocity)
            
            self._apply_vehicle_control(throttle=throttle_value, steer=steer_value, brake=brake_value)
            print(f"🎯 Fallback 3D 추격: 거리={distance:.1f}m, 조향={steer_value:.2f}")
            
        except Exception as e:
            print(f"⚠️ Error in fallback control: {e}")
    
    def _calculate_coordinate_based_control(self, target_center, distance, target_velocity):
        """좌표 기반 정확한 추격 제어 계산"""
        try:
            # 현재 차량 위치와 방향 확인
            if not self.vehicle_position or not self.vehicle_orientation:
                print("⚠️ Vehicle state not available for coordinate control")
                return 0.0, 0.3, 0.0
            
            my_pos = self.vehicle_position
            my_orientation = self.vehicle_orientation
            my_velocity = self.vehicle_velocity
            
            # 타겟 차량의 절대 좌표 계산 (LiDAR 상대 좌표 + 현재 차량 위치)
            target_world_pos = {
                'x': float(my_pos['x']) + float(target_center.get('x', 0)),
                'y': float(my_pos['y']) + float(target_center.get('y', 0)),
                'z': float(my_pos['z']) + float(target_center.get('z', 0))
            }
            
            # 상대 위치 벡터 계산
            relative_vector = {
                'x': target_world_pos['x'] - float(my_pos['x']),
                'y': target_world_pos['y'] - float(my_pos['y']),
                'z': target_world_pos['z'] - float(my_pos['z'])
            }
            
            # 현재 차량의 yaw 각도 (도 단위)
            my_yaw = float(my_orientation.get('yaw', 0))
            
            # 상대 벡터를 차량 좌표계로 변환
            relative_angle = self._calculate_relative_angle(relative_vector, my_yaw)
            
            # 조향각 계산 (차량 좌표계 기준)
            steer_value = self._calculate_steering_from_angle(relative_angle, distance)
            
            # 속도 제어 계산
            throttle_value, brake_value = self._calculate_velocity_control(
                distance, relative_vector, target_velocity, my_velocity
            )
            
            return steer_value, throttle_value, brake_value
            
        except Exception as e:
            print(f"⚠️ Error calculating coordinate-based control: {e}")
            return 0.0, 0.3, 0.0
    
    def _calculate_relative_angle(self, relative_vector, my_yaw):
        """상대 벡터를 차량 좌표계 기준 각도로 변환"""
        try:
            # 상대 벡터의 각도 계산 (라디안)
            target_angle = np.arctan2(relative_vector['y'], relative_vector['x'])
            
            # 현재 차량의 yaw를 라디안으로 변환
            my_yaw_rad = np.radians(my_yaw)
            
            # 상대 각도 계산 (차량이 바라보는 방향 기준)
            relative_angle = target_angle - my_yaw_rad
            
            # -π ~ π 범위로 정규화
            while relative_angle > np.pi:
                relative_angle -= 2 * np.pi
            while relative_angle < -np.pi:
                relative_angle += 2 * np.pi
            
            return relative_angle
            
        except Exception as e:
            print(f"⚠️ Error calculating relative angle: {e}")
            return 0.0
    
    def _calculate_steering_from_angle(self, relative_angle, distance):
        """상대 각도로부터 조향값 계산"""
        try:
            # 각도를 도 단위로 변환
            angle_degrees = np.degrees(relative_angle)
            
            # 거리 기반 감도 조정 (가까울수록 더 민감하게)
            distance_factor = max(0.5, min(2.0, 20.0 / max(distance, 1.0)))
            
            # 조향값 계산 (비선형 응답)
            if abs(angle_degrees) < 5:  # 5도 이내면 직진
                steer_value = 0.0
            elif abs(angle_degrees) < 15:  # 15도 이내면 부드러운 조향
                steer_value = np.sign(angle_degrees) * (abs(angle_degrees) / 90.0) * 0.5
            elif abs(angle_degrees) < 45:  # 45도 이내면 중간 조향
                steer_value = np.sign(angle_degrees) * (abs(angle_degrees) / 90.0) * 0.7
            else:  # 45도 이상이면 강한 조향
                steer_value = np.sign(angle_degrees) * min(0.9, abs(angle_degrees) / 90.0)
            
            # 거리 팩터 적용
            steer_value *= distance_factor
            
            # 조향값 제한
            steer_value = max(-1.0, min(1.0, steer_value))
            
            return steer_value
            
        except Exception as e:
            print(f"⚠️ Error calculating steering from angle: {e}")
            return 0.0
    
    def _calculate_velocity_control(self, distance, relative_vector, target_velocity, my_velocity):
        """속도 제어 계산 (좌표 기반)"""
        try:
            # 거리 기반 기본 속도
            if distance > 30:
                throttle = 0.7
                brake = 0.0
            elif distance > 20:
                throttle = 0.5
                brake = 0.0
            elif distance > 10:
                throttle = 0.3
                brake = 0.1
            elif distance > 5:
                throttle = 0.1
                brake = 0.3
            else:
                throttle = 0.0
                brake = 0.5
            
            # 타겟 속도에 따른 조정
            if target_velocity and my_velocity:
                target_speed = np.sqrt(
                    target_velocity.get('x', 0)**2 + 
                    target_velocity.get('y', 0)**2 + 
                    target_velocity.get('z', 0)**2
                )
                my_speed = np.sqrt(
                    my_velocity.get('x', 0)**2 + 
                    my_velocity.get('y', 0)**2 + 
                    my_velocity.get('z', 0)**2
                )
                
                # 속도 차이에 따른 조정
                speed_diff = target_speed - my_speed
                if speed_diff > 2.0:  # 타겟이 훨씬 빠름
                    throttle = min(1.0, throttle * 1.3)
                elif speed_diff < -2.0:  # 타겟이 훨씬 느림
                    throttle *= 0.7
                    brake += 0.1
            
            # 값 제한
            throttle = max(0.0, min(1.0, throttle))
            brake = max(0.0, min(1.0, brake))
            
            return throttle, brake
            
        except Exception as e:
            print(f"⚠️ Error calculating velocity control: {e}")
            return 0.3, 0.0
    
    def _print_coordinate_control_status(self, target_center, distance, steer_value, throttle_value, brake_value):
        """좌표 기반 제어 상태 출력"""
        try:
            my_pos = self.vehicle_position
            my_orientation = self.vehicle_orientation
            
            # 타겟의 절대 좌표
            target_world_pos = {
                'x': my_pos['x'] + target_center['x'],
                'y': my_pos['y'] + target_center['y'],
                'z': my_pos['z'] + target_center['z']
            }
            
            # 조향 방향
            if abs(steer_value) < 0.1:
                steer_direction = "직진"
            elif steer_value > 0:
                steer_direction = f"우회전 ({steer_value:.2f})"
            else:
                steer_direction = f"좌회전 ({steer_value:.2f})"
            
            # 속도 상태
            if throttle_value > 0.5:
                speed_status = f"가속 ({throttle_value:.2f})"
            elif throttle_value > 0.2:
                speed_status = f"유지 ({throttle_value:.2f})"
            elif brake_value > 0.3:
                speed_status = f"정지 (브레이크 {brake_value:.2f})"
            else:
                speed_status = f"감속 (브레이크 {brake_value:.2f})"
            
            print(f"🎯 좌표 추격: 내위치=({my_pos['x']:.1f},{my_pos['y']:.1f}), "
                  f"타겟=({target_world_pos['x']:.1f},{target_world_pos['y']:.1f}), "
                  f"거리={distance:.1f}m, {steer_direction}, {speed_status}")
            
        except Exception as e:
            print(f"⚠️ Error printing coordinate control status: {e}")
    
    def _calculate_3d_steering_angle(self, target_center, distance):
        """3D 위치 기반 조향각 계산"""
        try:
            # 타겟 차량의 상대 위치
            x = target_center.get('x', 0)
            y = target_center.get('y', 0)
            z = target_center.get('z', 0)
            
            # Y축 오프셋이 조향에 가장 중요 (좌우)
            # X축은 거리 (전후)
            lateral_offset = y  # 좌우 오프셋
            longitudinal_distance = x  # 전후 거리
            
            # 거리 기반 정규화 (가까울수록 더 민감하게)
            if distance > 0:
                normalized_lateral = lateral_offset / max(distance, 1.0)
            else:
                normalized_lateral = 0
            
            # 조향각 계산 (비선형 응답)
            if abs(normalized_lateral) < 0.1:  # 10% 이내면 직진
                steer_value = 0.0
            elif abs(normalized_lateral) < 0.3:  # 30% 이내면 부드러운 조향
                steer_value = normalized_lateral * 0.6
            else:  # 30% 이상이면 강한 조향
                steer_value = normalized_lateral * 0.9
            
            # 거리 기반 조향 감도 조정
            distance_factor = max(0.3, min(1.5, 15.0 / max(distance, 1.0)))
            steer_value *= distance_factor
            
            # 조향값 제한
            steer_value = max(-1.0, min(1.0, steer_value))
            
            return steer_value
            
        except Exception as e:
            print(f"⚠️ Error calculating 3D steering angle: {e}")
            return 0.0
    
    def _calculate_3d_speed_control(self, distance, velocity):
        """3D 위치 기반 속도 제어"""
        try:
            # 타겟 차량의 속도 계산
            target_speed = np.sqrt(
                velocity.get('x', 0)**2 + 
                velocity.get('y', 0)**2 + 
                velocity.get('z', 0)**2
            )
            
            # 거리 기반 속도 조정
            if distance > 25:
                # 멀리 있으면 가속
                throttle = 0.7
                brake = 0.0
            elif distance > 15:
                # 중간 거리면 적당한 속도
                throttle = 0.5
                brake = 0.0
            elif distance > 10:
                # 가까우면 감속
                throttle = 0.3
                brake = 0.1
            elif distance > 6:
                # 매우 가까우면 브레이크
                throttle = 0.1
                brake = 0.3
            else:
                # 너무 가까우면 강제 정지
                throttle = 0.0
                brake = 0.6
            
            # 타겟 차량 속도에 따른 조정
            if target_speed > 5.0:  # 타겟이 빠르게 움직이면
                throttle = min(1.0, throttle * 1.2)  # 더 빠르게 추격
            elif target_speed < 1.0:  # 타겟이 느리게 움직이면
                throttle *= 0.8  # 속도 감소
                brake += 0.1
            
            # 값 제한
            throttle = max(0.0, min(1.0, throttle))
            brake = max(0.0, min(1.0, brake))
            
            return throttle, brake
            
        except Exception as e:
            print(f"⚠️ Error calculating 3D speed control: {e}")
            return 0.3, 0.0
    
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
                    
                    # 추격 시작
                    self.is_chasing = True
                    
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
                    target_obj = max(camera_objects, 
                                   key=lambda x: self._get_bbox_area(x.get('bbox_2d', {})))
                    
                    # 객체가 충분히 큰 경우에만 추격 시작
                    bbox_area = self._get_bbox_area(target_obj.get('bbox_2d', {}))
                    if bbox_area > 1000:  # 최소 크기 임계값
                        print(f"🎯 Large object detected - starting chase: {target_obj['actor_id']}")
                        self.is_chasing = True
                        self.apply_simple_chase_control(target_obj)
            
        except Exception as e:
            print(f"⚠️ Error in Zenoh chase control: {e}")
    
    def _apply_vehicle_control(self, throttle=0.0, steer=0.0, brake=0.0):
        """Apply vehicle control commands to CARLA vehicle"""
        try:
            if not self.vehicle:
                print("⚠️ No vehicle available for control")
                return
            
            # CARLA 차량 제어 명령 생성
            control = carla.VehicleControl()
            control.throttle = max(0.0, min(1.0, throttle))
            control.steer = max(-1.0, min(1.0, steer))
            control.brake = max(0.0, min(1.0, brake))
            control.hand_brake = False
            control.manual_gear_shift = False
            
            # 차량에 제어 명령 적용
            self.vehicle.apply_control(control)
            
            print(f"🚗 Vehicle control: throttle={control.throttle:.2f}, steer={control.steer:.2f}, brake={control.brake:.2f}")
            
        except Exception as e:
            print(f"⚠️ Error applying vehicle control: {e}")
    
    def apply_simple_chase_control(self, target_obj):
        """Apply improved chase control based on target object"""
        try:
            if not self.vehicle or not target_obj:
                return
            
            bbox = target_obj.get('bbox_2d', {})
            if not bbox:
                return
            
            # 바운딩 박스 중심점 계산
            center_x = bbox.get('x_min', 0) + bbox.get('width', 0) / 2
            center_y = bbox.get('y_min', 0) + bbox.get('height', 0) / 2
            
            # 화면 중심점 (카메라 기준) - 실제 이미지 크기에 맞춤
            if self.zenoh_camera_image is not None:
                screen_center_x = self.zenoh_camera_image.shape[1] / 2
                screen_center_y = self.zenoh_camera_image.shape[0] / 2
            else:
                screen_center_x = 800
                screen_center_y = 450
            
            # 오프셋 계산
            offset_x = center_x - screen_center_x
            offset_y = center_y - screen_center_y
            
            # 거리 정보
            distance = target_obj.get('distance', 100)
            
            print(f"🎯 Target center: ({center_x:.1f}, {center_y:.1f})")
            print(f"📺 Screen center: ({screen_center_x:.1f}, {screen_center_y:.1f})")
            print(f"📏 Offset: ({offset_x:.1f}, {offset_y:.1f}), Distance: {distance:.1f}m")
            
            # 개선된 조향 제어 로직
            steer_value = self._calculate_steering_angle(offset_x, screen_center_x, distance)
            throttle_value, brake_value = self._calculate_speed_control(distance, abs(offset_x))
            
            # 차량 제어 적용
            self._apply_vehicle_control(throttle=throttle_value, steer=steer_value, brake=brake_value)
            
            # 제어 상태 출력
            self._print_control_status(steer_value, throttle_value, brake_value, offset_x, distance)
                
        except Exception as e:
            print(f"⚠️ Error applying chase control: {e}")
    
    def _calculate_steering_angle(self, offset_x, screen_center_x, distance):
        """개선된 조향각 계산"""
        try:
            # 정규화된 오프셋 (-1.0 ~ 1.0)
            normalized_offset = offset_x / screen_center_x
            
            # 거리 기반 조향 감도 조정 (가까울수록 더 민감하게)
            distance_factor = max(0.5, min(2.0, 20.0 / max(distance, 1.0)))
            
            # 기본 조향값 계산 (비선형 응답)
            if abs(normalized_offset) < 0.1:  # 10% 이내면 직진
                base_steer = 0.0
            elif abs(normalized_offset) < 0.3:  # 30% 이내면 부드러운 조향
                base_steer = normalized_offset * 0.5
            else:  # 30% 이상이면 강한 조향
                base_steer = normalized_offset * 0.8
            
            # 거리 팩터 적용
            steer_value = base_steer * distance_factor
            
            # 조향값 제한 (-1.0 ~ 1.0)
            steer_value = max(-1.0, min(1.0, steer_value))
            
            return steer_value
            
        except Exception as e:
            print(f"⚠️ Error calculating steering angle: {e}")
            return 0.0
    
    def _calculate_speed_control(self, distance, offset_magnitude):
        """속도 제어 계산"""
        try:
            # 거리 기반 속도 조정
            if distance > 30:
                # 멀리 있으면 가속
                throttle = 0.6
                brake = 0.0
            elif distance > 15:
                # 중간 거리면 적당한 속도
                throttle = 0.4
                brake = 0.0
            elif distance > 8:
                # 가까우면 감속
                throttle = 0.2
                brake = 0.1
            elif distance > 5:
                # 매우 가까우면 브레이크
                throttle = 0.0
                brake = 0.3
            else:
                # 너무 가까우면 강제 정지
                throttle = 0.0
                brake = 0.5
            
            # 조향 중일 때 속도 감소 (안전을 위해)
            if offset_magnitude > 50:  # 큰 조향이 필요할 때
                throttle *= 0.7  # 속도 30% 감소
                brake += 0.1     # 브레이크 추가
            
            # 값 제한
            throttle = max(0.0, min(1.0, throttle))
            brake = max(0.0, min(1.0, brake))
            
            return throttle, brake
            
        except Exception as e:
            print(f"⚠️ Error calculating speed control: {e}")
            return 0.3, 0.0
    
    def _print_control_status(self, steer_value, throttle_value, brake_value, offset_x, distance):
        """제어 상태 출력"""
        try:
            # 조향 방향
            if abs(steer_value) < 0.1:
                steer_direction = "직진"
            elif steer_value > 0:
                steer_direction = f"우회전 ({steer_value:.2f})"
            else:
                steer_direction = f"좌회전 ({steer_value:.2f})"
            
            # 속도 상태
            if throttle_value > 0.5:
                speed_status = f"가속 ({throttle_value:.2f})"
            elif throttle_value > 0.2:
                speed_status = f"유지 ({throttle_value:.2f})"
            elif brake_value > 0.3:
                speed_status = f"정지 (브레이크 {brake_value:.2f})"
            else:
                speed_status = f"감속 (브레이크 {brake_value:.2f})"
            
            print(f"🚗 제어: {steer_direction}, {speed_status}, 거리: {distance:.1f}m")
            
        except Exception as e:
            print(f"⚠️ Error printing control status: {e}")
    
    def _on_imu_data_received(self, imu_data):
        """IMU 데이터 수신 콜백"""
        try:
            # IMU 데이터만 저장하고, 상태 업데이트는 메인 루프에서 처리
            self.current_imu_data = imu_data
            
        except Exception as e:
            print(f"⚠️ Error processing IMU data: {e}")
    
    def _update_vehicle_state(self):
        """차량 상태 업데이트 (위치, 방향, 속도)"""
        try:
            if not self.vehicle:
                return
            
            # 차량 위치 (CARLA Transform에서) - IMU 없이도 작동
            try:
                transform = self.vehicle.get_transform()
                if transform:
                    location = transform.location
                    rotation = transform.rotation
                    
                    if location and rotation:
                        self.vehicle_position = {
                            'x': float(location.x),
                            'y': float(location.y),
                            'z': float(location.z)
                        }
                        
                        self.vehicle_orientation = {
                            'pitch': float(rotation.pitch),
                            'yaw': float(rotation.yaw),
                            'roll': float(rotation.roll)
                        }
            except Exception as transform_error:
                print(f"⚠️ Error getting vehicle transform: {transform_error}")
            
            # 차량 속도 (CARLA Velocity에서)
            try:
                velocity = self.vehicle.get_velocity()
                if velocity:
                    self.vehicle_velocity = {
                        'x': float(velocity.x),
                        'y': float(velocity.y),
                        'z': float(velocity.z)
                    }
            except Exception as velocity_error:
                print(f"⚠️ Error getting vehicle velocity: {velocity_error}")
            
            # IMU 데이터 처리 (선택적)
            if self.current_imu_data:
                try:
                    self.current_imu_data = {
                        'accelerometer': {
                            'x': float(self.current_imu_data.accelerometer.x),
                            'y': float(self.current_imu_data.accelerometer.y),
                            'z': float(self.current_imu_data.accelerometer.z)
                        },
                        'gyroscope': {
                            'x': float(self.current_imu_data.gyroscope.x),
                            'y': float(self.current_imu_data.gyroscope.y),
                            'z': float(self.current_imu_data.gyroscope.z)
                        },
                        'compass': float(self.current_imu_data.compass)
                    }
                except Exception as imu_error:
                    print(f"⚠️ Error processing IMU data: {imu_error}")
            
        except Exception as e:
            print(f"⚠️ Error updating vehicle state: {e}")
    
    def get_vehicle_world_position(self):
        """현재 차량의 월드 좌표 반환"""
        return self.vehicle_position
    
    def get_vehicle_orientation(self):
        """현재 차량의 방향 반환"""
        return self.vehicle_orientation
    
    def get_vehicle_velocity(self):
        """현재 차량의 속도 반환"""
        return self.vehicle_velocity
    
    def _initialize_fusion_modules(self):
        """LiDAR 시각화 및 센서 퓨전 모듈 초기화"""
        try:
            from chase.sensors.lidar_visualizer import LiDARVisualizer
            from chase.perception.sensor_fusion import SensorFusion
            from chase.control.fusion_display_manager import FusionDisplayManager
            
            self.lidar_visualizer = LiDARVisualizer()
            self.sensor_fusion = SensorFusion()
            self.fusion_display = FusionDisplayManager()
            
            print("✅ LiDAR 시각화 및 센서 퓨전 모듈 초기화 완료")
            
        except Exception as e:
            print(f"⚠️ Error initializing fusion modules: {e}")
            self.lidar_visualizer = None
            self.sensor_fusion = None
            self.fusion_display = None
    
    
    def _get_bbox_area(self, bbox):
        """Get bounding box area from different formats"""
        try:
            if isinstance(bbox, dict) and 'width' in bbox and 'height' in bbox:
                return bbox['width'] * bbox['height']
            elif isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
                return bbox[2] * bbox[3]  # width * height
            return 0
        except:
            return 0
    
    def cleanup(self):
        """리소스 정리"""
        try:
            print("🧹 Cleaning up resources...")
            
            # 추격 중지
            if hasattr(self, 'is_chasing'):
                self.is_chasing = False
            
            # 통합 충돌 감지기 정리
            if self.unified_collision_detector:
                self.unified_collision_detector.cleanup()
                print("🚨 Unified collision detector cleaned up")
            
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
            
            # Semantic LIDAR 정리
            if self.semantic_lidar:
                self.semantic_lidar.cleanup()
                print("✅ Semantic LIDAR cleaned up")
            
            # Zenoh 정리 (legacy)
            if self.zenoh_collision_manager:
                self.zenoh_collision_manager.cleanup()
                print("✅ Zenoh collision manager cleaned up")
            
            # ROS2 정리
            if self.ros2_node and ROS2_AVAILABLE:
                try:
                    self.ros2_node.destroy_node()
                    if rclpy.ok():
                        rclpy.shutdown()
                    print("✅ ROS2 node cleaned up")
                except Exception as e:
                    print(f"⚠️ Error cleaning up ROS2 node: {e}")
            
            # CollisionDetector 정리
            if self.collision_detector:
                self.collision_detector.cleanup()
                print("✅ Collision detector cleaned up")
            
            print("✅ Cleanup completed")
            
        except Exception as e:
            print(f"⚠️ Error during cleanup: {e}")
    
    def get_lidar_transform(self):
        """LiDAR transform을 가져옵니다."""
        try:
            # CARLA world에서 semantic LiDAR 센서 찾기
            for actor in self.world.get_actors():
                if actor.type_id == 'sensor.lidar.ray_cast_semantic':
                    if actor.parent == self.vehicle:
                        return actor.get_transform().get_matrix()
            
            # 기본값: 단위 행렬
            return np.identity(4)
        except Exception as e:
            print(f"⚠️ Error getting LiDAR transform: {e}")
            return np.identity(4)
    
    def _on_collision_detected(self, collision_event: CollisionEvent):
        """충돌 감지 콜백"""
        try:
            print(f"🚨 Collision detected: {collision_event.description}")
            print(f"   Type: {collision_event.event_type}")
            print(f"   Severity: {collision_event.severity}")
            print(f"   Location: {collision_event.world_location}")
            
            # 기존 충돌 처리 로직과 통합
            if self.collision_tracker:
                self.collision_tracker.add_collision_event(collision_event)
                
        except Exception as e:
            print(f"⚠️ Error in collision callback: {e}")
    
    def _on_chase_started(self, chase_event):
        """추격 시작 콜백"""
        try:
            print(f"🚗 Chase started: {chase_event}")
            
            # 추격 상태 업데이트
            if not self.is_chasing:
                self.is_chasing = True
                self.chase_statistics['collision_events'] += 1
                print("✅ Chase started from unified collision detector")
            else:
                print("🚔 Chase already in progress")
                
        except Exception as e:
            print(f"⚠️ Error in chase callback: {e}")

def main():
    """메인 함수"""
    try:
        # 자동 추격 차량 제어 인스턴스 생성
        auto_chase = AutoChaseVehicleControl()
        
        # 실행
        auto_chase.run()
        
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    except Exception as e:
        print(f"❌ Error in main: {e}")
    finally:
        cleanup_opencv_windows()

if __name__ == "__main__":
    main()
