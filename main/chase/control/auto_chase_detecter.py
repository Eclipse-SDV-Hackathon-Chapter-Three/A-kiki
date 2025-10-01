#!/usr/bin/env python3
"""
Auto Chase Vehicle Controller — Headless (No-Movement, No-UI, Quiet)
- 차량 제어(가속/조향/브레이크) 비활성화 → 절대 움직이지 않음
- 모든 창/윈도우( OpenCV/pygame 등) 표시 제거 → 완전 헤드리스
- 출력은 기본적으로 조용(quiet)하며, '인지가 발생했을 때만' 간단히 알림을 출력
"""

# ==== Headless 강제: 어떤 라이브러리도 창을 못 띄우게 ====
import os as _os
_os.environ.setdefault("SDL_VIDEODRIVER", "dummy")     # pygame 창 방지
_os.environ.setdefault("QT_QPA_PLATFORM", "offscreen") # Qt 창 방지

import sys
import os
import time
import signal
import json
import struct
from typing import Optional, List, Any, Dict

import numpy as np
import carla

# OpenCV가 내부 모듈에서 import될 수 있으니, 안전하게 NO-OP 패치
try:
    import cv2
    cv2.imshow = lambda *a, **k: None
    cv2.waitKey = lambda *a, **k: -1
    cv2.destroyAllWindows = lambda *a, **k: None
    cv2.namedWindow = lambda *a, **k: None
except Exception:
    pass

# ---- Optional backends (존재하면 사용) ----
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    # print("[WARN] zenoh not available")

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    # print("[WARN] ROS2 not available")

# 상위 디렉토리 모듈 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, parent_dir)

# 프로젝트 내부 모듈
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
# DisplayManager는 완전 제거 (헤드리스)

_auto_chase_instance = None


# ====== 간결 로그 유틸 (인지 시에만 출력) ======
class DetectorLogger:
    def __init__(self, enable=True):
        self.enable = enable

    def event(self, msg: str):
        if self.enable:
            print(msg, flush=True)

# 전역 디텍션 로거 (필요 시 False로 끄기)
DETECTION_LOGGER = DetectorLogger(enable=True)


def signal_handler(signum, frame):
    # print(f"\n🛑 Received signal {signum}, cleaning up...")
    if _auto_chase_instance:
        _auto_chase_instance.cleanup()
    sys.exit(0)


# 시그널 핸들러 등록 (UI 정리 없음)
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


class AutoChaseVehicleControl:
    """자동 추격 차량 제어 (No-Movement / Headless / Quiet)"""

    def __init__(self):
        # CARLA
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.vehicle: Optional[carla.Vehicle] = None

        # 모듈
        self.bounding_box_detector: Optional[BoundingBoxDetector] = None
        self.collision_detector: Optional[CollisionDetector] = None
        self.vehicle_tracker: Optional[CollisionVehicleTracker] = None
        self.collision_tracker: Optional[CollisionTracker] = None
        self.unified_collision_detector: Optional[UnifiedCollisionDetector] = None
        self.chase_planner: Optional[SimpleChasePlanner] = None

        # 매니저
        self.vehicle_manager: Optional[VehicleManager] = None
        self.camera_manager: Optional[CameraManager] = None
        self.sensor_manager: Optional[SensorManager] = None
        self.chase_controller: Optional[ChaseController] = None  # 생성만, 제어는 미사용

        # 실행 상태
        self.running = True
        self.last_update_time = 0.0
        self.update_interval = 0.05  # 50ms

        # 상태 저장(표시용 로그)
        self.vehicle_position = None
        self.vehicle_orientation = None
        self.vehicle_velocity = None

        # 통계
        self.chase_statistics: Dict[str, Any] = {
            'total_detections': 0,
            'collision_events': 0,
            'chase_duration': 0.0,
            'max_speed_reached': 0.0,
            'average_distance': 0.0
        }

        # Zenoh
        self.zenoh_collision_manager: Optional[ZenohCollisionManager] = None
        self.zero_copy_camera: Optional[ZeroCopyCameraSubscriber] = None
        self.use_zero_copy = False
        self.zenoh_camera_image = None
        self.zenoh_detected_objects: List[Dict[str, Any]] = []

        # LiDAR/ROS2
        self.use_semantic_lidar = True
        self.semantic_lidar: Optional[SemanticLidarManager] = None
        self.semantic_lidar_data = None
        self.last_semantic_lidar_time = 0.0

        self.use_ros2_semantic_lidar = True
        self.ros2_node: Optional[Node] = None
        self.ros2_semantic_lidar_sub = None

        # 추적 대상
        self.target_vehicle = None
        self.vehicle_tracking_enabled = True
        self.accident_detection_enabled = True

        # 최소 시작 로그도 생략
        # print("🚔 Auto Chase Vehicle Control initialized (Headless / No-Movement)")

        global _auto_chase_instance
        _auto_chase_instance = self

    # ---------- 기본 연결 ----------
    def connect_to_carla(self, host='localhost', port=2000):
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            # print(f"✅ Connected to CARLA server at {host}:{port}")
            return True
        except Exception as e:
            # 중요한 에러만 출력
            print(f"[FATAL] Failed to connect to CARLA: {e}", flush=True)
            return False

    # ---------- Zenoh ----------
    def setup_zenoh(self):
        if not ZENOH_AVAILABLE:
            # print("⚠️ Zenoh not available, skipping setup")
            return False
        try:
            if self.use_zero_copy:
                self.zero_copy_camera = ZeroCopyCameraSubscriber()
                self.zero_copy_camera.set_camera_callback(self.on_zenoh_camera_received)
                self.zero_copy_camera.set_bbox_callback(self.on_zenoh_bounding_boxes_received)
                if self.zero_copy_camera.setup_zenoh():
                    self.zero_copy_camera.start()
                    # print("✅ Zero-Copy camera subscriber setup successful")
                    return True
                else:
                    # print("❌ Zero-Copy setup failed, fallback to legacy")
                    self.use_zero_copy = False

            self.zenoh_collision_manager = ZenohCollisionManager()
            self.zenoh_collision_manager.set_camera_callback(self.on_zenoh_camera_received)
            self.zenoh_collision_manager.set_bbox_callback(self.on_zenoh_bounding_boxes_received)
            if self.zenoh_collision_manager.setup_zenoh():
                # print("✅ Zenoh collision manager setup successful (legacy)")
                self._setup_zenoh_semantic_lidar_sub()
                return True
            else:
                # print("❌ Failed to setup Zenoh collision manager")
                return False
        except Exception as e:
            print(f"[WARN] Error setting up Zenoh: {e}", flush=True)
            return False

    def _setup_zenoh_semantic_lidar_sub(self):
        try:
            if not self.zenoh_collision_manager or not self.zenoh_collision_manager.session:
                # print("⚠️ Zenoh session not available for semantic LiDAR subscription")
                return False
            self.zenoh_collision_manager.session.declare_subscriber(
                "carla/semantic_lidar/data", self.on_semantic_lidar_data_received
            )
            # print("✅ Semantic LiDAR subscription via Zenoh ready")
            return True
        except Exception as e:
            print(f"[WARN] Error setting up semantic LiDAR sub: {e}", flush=True)
            return False

    # ---------- ROS2 ----------
    def setup_ros2(self):
        if not ROS2_AVAILABLE:
            return False
        try:
            if not rclpy.ok():
                rclpy.init()
            self.ros2_node = Node('auto_chase_vehicle_control_headless')
            self.ros2_semantic_lidar_sub = self.ros2_node.create_subscription(
                PointCloud2,
                '/carla/hero/semantic_lidar/point_cloud',
                self.on_ros2_semantic_lidar_received,
                10
            )
            return True
        except Exception as e:
            print(f"[WARN] Error setting up ROS2: {e}", flush=True)
            return False

    # ---------- 콜백 ----------
    def on_zenoh_camera_received(self, image):
        try:
            if image is not None:
                self.zenoh_camera_image = image
                # 인지 자체가 아니므로 로그 생략
                # DETECTION_LOGGER.event(f"[CAM] Zenoh camera frame")
        except Exception:
            pass

    def on_zenoh_bounding_boxes_received(self, objects):
        try:
            self.zenoh_detected_objects = objects or []
            if self.unified_collision_detector and objects:
                self.unified_collision_detector.process_objects(objects)
                # 바운딩 박스 수신 시(인지) 최소 로그
                DETECTION_LOGGER.event(f"[DETECT] BBoxes via Zenoh: {len(objects)}")
        except Exception:
            pass

    def on_semantic_lidar_data_received(self, sample):
        try:
            payload = sample.payload.to_bytes() if hasattr(sample.payload, "to_bytes") else bytes(sample.payload)
            data = json.loads(payload.decode("utf-8"))
            self.semantic_lidar_data = data
            self.last_semantic_lidar_time = time.time()
            # LiDAR 수신 자체는 인지 아님 → 로그 생략
            self._process_semantic_lidar_for_vehicle_tracking(data)
        except Exception as e:
            # 중요 에러만
            print(f"[WARN] Error processing Zenoh LiDAR: {e}", flush=True)

    def on_ros2_semantic_lidar_received(self, msg: PointCloud2):
        try:
            points = self._parse_pointcloud2_msg(msg)
            if not points:
                return
            self.semantic_lidar_data = {'points': points, 'point_count': len(points), 'timestamp': time.time()}
            self.last_semantic_lidar_time = time.time()
            # 수신 자체는 로그 생략
            self._process_semantic_lidar_for_vehicle_tracking(self.semantic_lidar_data)
        except Exception as e:
            print(f"[WARN] Error processing ROS2 LiDAR: {e}", flush=True)

    # ---------- 유틸 ----------
    def _parse_pointcloud2_msg(self, msg: PointCloud2):
        try:
            pts = []
            step = msg.point_step
            width = msg.width
            data = msg.data
            for i in range(width):
                offset = i * step
                x, y, z, cos_angle, obj_idx, obj_tag = struct.unpack('ffffII', data[offset:offset + step])
                pts.append({'x': float(x), 'y': float(y), 'z': float(z),
                            'cos_angle': float(cos_angle),
                            'object_id': int(obj_idx), 'semantic_id': int(obj_tag)})
            return pts
        except Exception:
            return []

    # ---------- 모듈 초기화 ----------
    def setup_modules(self):
        try:
            if self.camera_manager and self.camera_manager.camera_view and \
               self.camera_manager.camera_view.bounding_box_detector:
                self.bounding_box_detector = self.camera_manager.camera_view.bounding_box_detector
                # print("✅ CARLA bounding box detector enabled")
            else:
                # print("⚠️ CARLA bounding box detector unavailable")
                self.bounding_box_detector = None

            self.unified_collision_detector = UnifiedCollisionDetector()
            self.unified_collision_detector.set_collision_callback(self._on_collision_detected)
            self.unified_collision_detector.set_chase_callback(self._on_chase_started)

            self.collision_tracker = CollisionTracker(max_tracking_time=300.0)
            self.collision_detector = CollisionDetector()
            self.vehicle_tracker = CollisionVehicleTracker(self.world)
            self.chase_planner = SimpleChasePlanner(self.world, self.vehicle)
            return True
        except Exception as e:
            print(f"[FATAL] Error initializing modules: {e}", flush=True)
            return False

    # ---------- 메인 ----------
    def run(self):
        try:
            # print("🚔 Starting Auto Chase Vehicle Control (Headless / No-Movement)...")

            if not self.connect_to_carla():
                return False

            self.setup_zenoh()

            if self.use_ros2_semantic_lidar and ROS2_AVAILABLE:
                if not self.setup_ros2():
                    self.use_ros2_semantic_lidar = False

            # LiDAR 센서(퍼블리시용/처리용) 세팅
            self._setup_semantic_lidar_sensor()  # 실패해도 구독 경로로 동작 가능

            # 매니저
            self.vehicle_manager = VehicleManager(self.world, self.vehicle)
            self.camera_manager = CameraManager(self.world, self.vehicle)
            self.sensor_manager = SensorManager(self.world, self.vehicle)
            self.chase_controller = ChaseController(self.vehicle, self.chase_planner)  # 제어는 미사용

            if not self.vehicle_manager.find_existing_vehicle():
                print("[FATAL] No vehicle found to control", flush=True)
                return False

            self.vehicle = self.vehicle_manager.vehicle

            # 참조 갱신
            self.camera_manager.vehicle = self.vehicle
            self.sensor_manager.vehicle = self.vehicle
            self.chase_controller.vehicle = self.vehicle

            # 카메라 센서만 세팅 (표시는 안함)
            if not self.camera_manager.setup_camera():
                print("[FATAL] Failed to setup camera", flush=True)
                return False

            if not self.setup_modules():
                return False

            while self.running:
                now = time.time()
                if now - self.last_update_time < self.update_interval:
                    time.sleep(0.001)
                    continue
                self.last_update_time = now

                if self.use_ros2_semantic_lidar and self.ros2_node:
                    try:
                        rclpy.spin_once(self.ros2_node, timeout_sec=0.001)
                    except Exception as e:
                        # print(f"[WARN] ROS2 spin error: {e}")
                        pass

                detected_objects, collision_events = self.update_perception()
                self._update_vehicle_state()

                # LiDAR 기반 추적/사고 감지(제어 없음)
                self.update_vehicle_tracking()
                self.update_accident_detection()

                # 화면/창 출력 없음 (완전 헤드리스)
        except KeyboardInterrupt:
            # print("\n🛑 Keyboard interrupt")
            pass
        except Exception as e:
            print(f"[FATAL] Error in main execution: {e}", flush=True)
        finally:
            self.cleanup()

    # ---------- 인지/추적 ----------
    def update_perception(self):
        try:
            if not self.bounding_box_detector:
                return [], []
            detected = self.bounding_box_detector.detect_pedestrians_and_vehicles(max_distance=200.0)
            if detected:
                DETECTION_LOGGER.event(f"[DETECT] Camera objects: {len(detected)}")
            self.chase_statistics['total_detections'] = len(detected)
            collisions = self.collision_detector.detect_collisions(detected) if (self.collision_detector and detected) else []
            if collisions:
                DETECTION_LOGGER.event(f"[DETECT] Collisions: {len(collisions)}")
            return detected, collisions
        except Exception as e:
            # 중요 에러만
            print(f"[WARN] Error in perception update: {e}", flush=True)
            return [], []

    def update_vehicle_tracking(self):
        try:
            if not self.vehicle_tracker:
                return
            # semantic_lidar_data를 활용한 대상 선정 (표시/로그 전용)
            if self.semantic_lidar_data:
                self._process_semantic_lidar_for_vehicle_tracking(self.semantic_lidar_data)
        except Exception as e:
            print(f"[WARN] Error updating vehicle tracking: {e}", flush=True)

    def update_accident_detection(self):
        try:
            if not self.semantic_lidar or not self.accident_detection_enabled:
                return
            active = self.semantic_lidar.get_active_accidents()
            if active:
                DETECTION_LOGGER.event(f"[DETECT] Active accidents: {len(active)}")
        except Exception as e:
            print(f"[WARN] Error updating accident detection: {e}", flush=True)

    # ---------- LiDAR 처리 ----------
    def _setup_semantic_lidar_sensor(self):
        try:
            if not self.use_semantic_lidar:
                return True
            if not self.vehicle:
                return False
            self.semantic_lidar = SemanticLidarManager(self.world, self.vehicle)
            # Zenoh 퍼블리시 옵션 (실패해도 무시)
            self.semantic_lidar.setup_zenoh()
            ok = self.semantic_lidar.setup_semantic_lidar(
                location=carla.Location(x=0.0, y=0.0, z=2.5),
                rotation=carla.Rotation(pitch=0, yaw=0, roll=0),
                channels=32, range_meters=100.0,
                points_per_second=100000, rotation_frequency=10.0
            )
            # 성공/실패 로그는 조용히 처리
            return ok
        except Exception as e:
            print(f"[WARN] Error setting up semantic LIDAR sensor: {e}", flush=True)
            return False

    def _process_semantic_lidar_for_vehicle_tracking(self, lidar_data: Dict[str, Any]):
        try:
            points = lidar_data.get('points', [])
            if not points:
                return
            vehicle_points = [p for p in points if p.get('semantic_id') == 10]  # 10: vehicle
            if not vehicle_points:
                return
            clusters = self._cluster_points(vehicle_points, radius=3.0, min_pts=6)
            if not clusters:
                return
            closest = min(clusters, key=lambda c: c['distance'])
            self._mark_vehicle_target_for_display_only(closest)
            self._publish_target_vehicle_info(closest)
            DETECTION_LOGGER.event(f"[DETECT] LiDAR vehicle cluster: {closest['point_count']} pts, d={closest['distance']:.1f}m")
        except Exception as e:
            print(f"[WARN] Error in LiDAR vehicle tracking: {e}", flush=True)

    def _cluster_points(self, pts, radius=3.0, min_pts=6):
        clusters = []
        used = set()
        for i, p in enumerate(pts):
            if i in used:
                continue
            group = [p]
            used.add(i)
            for j, q in enumerate(pts):
                if j in used:
                    continue
                d = np.linalg.norm([p['x'] - q['x'], p['y'] - q['y'], p['z'] - q['z']])
                if d < radius:
                    group.append(q)
                    used.add(j)
            if len(group) >= min_pts:
                cx = float(np.mean([g['x'] for g in group]))
                cy = float(np.mean([g['y'] for g in group]))
                cz = float(np.mean([g['z'] for g in group]))
                dist = float(np.linalg.norm([cx, cy, cz]))
                clusters.append({'center': {'x': cx, 'y': cy, 'z': cz},
                                 'distance': dist, 'point_count': len(group)})
        return clusters

    def _mark_vehicle_target_for_display_only(self, vehicle_obj):
        if not self.vehicle_tracker:
            return
        self.vehicle_tracker.is_tracking = True
        self.vehicle_tracker.target_vehicle_id = f"lidar_vehicle_{id(vehicle_obj)}"
        self.vehicle_tracker.target_position = vehicle_obj.get('center')
        self.vehicle_tracker.target_distance = vehicle_obj.get('distance')
        DETECTION_LOGGER.event(f"[DETECT] Target set: {self.vehicle_tracker.target_vehicle_id}")

    def _publish_target_vehicle_info(self, vehicle_info: Dict[str, Any]):
        try:
            if not self.zenoh_collision_manager or not getattr(self.zenoh_collision_manager, "zenoh_session", None):
                return
            data = {
                'timestamp': time.time(),
                'target_vehicle': {
                    'object_id': vehicle_info.get('object_id', 0),
                    'center': vehicle_info.get('center', {}),
                    'distance': vehicle_info.get('distance', 0.0),
                    'velocity': vehicle_info.get('velocity', {}),
                    'point_count': vehicle_info.get('point_count', 0),
                },
                'chase_active': False,
                'source': 'semantic_lidar_tracking_headless'
            }
            self.zenoh_collision_manager.zenoh_session.put("carla/chase/target_vehicle", json.dumps(data))
            # 퍼블리시 성공 로그는 조용히
        except Exception as e:
            print(f"[WARN] Error publishing target vehicle info: {e}", flush=True)

    # ---------- 상태/콜백 ----------
    def _update_vehicle_state(self):
        try:
            if not self.vehicle:
                return
            tr = self.vehicle.get_transform()
            if tr:
                self.vehicle_position = {'x': float(tr.location.x), 'y': float(tr.location.y), 'z': float(tr.location.z)}
                self.vehicle_orientation = {'pitch': float(tr.rotation.pitch),
                                            'yaw': float(tr.rotation.yaw),
                                            'roll': float(tr.rotation.roll)}
            vel = self.vehicle.get_velocity()
            if vel:
                self.vehicle_velocity = {'x': float(vel.x), 'y': float(vel.y), 'z': float(vel.z)}
        except Exception as e:
            # print(f"[WARN] Error updating vehicle state: {e}")
            pass

    def _on_collision_detected(self, collision_event: CollisionEvent):
        try:
            DETECTION_LOGGER.event(f"[DETECT] Collision: {collision_event.description} "
                                   f"(type={collision_event.event_type}, sev={collision_event.severity})")
            if self.collision_tracker:
                self.collision_tracker.add_collision_event(collision_event)
        except Exception:
            pass

    def _on_chase_started(self, chase_event):
        try:
            self.chase_statistics['collision_events'] += 1
            DETECTION_LOGGER.event(f"[DETECT] Chase started")
        except Exception:
            pass

    # ---------- 제어(완전 비활성화) ----------
    def update_planning_and_control(self):
        return

    def _apply_vehicle_control(self, throttle=0.0, steer=0.0, brake=0.0):
        return

    # ---------- 정리 ----------
    def cleanup(self):
        try:
            # print("🧹 Cleaning up resources (headless)...")
            if self.unified_collision_detector:
                self.unified_collision_detector.cleanup()
            if self.vehicle_tracker and hasattr(self.vehicle_tracker, 'stop_tracking'):
                self.vehicle_tracker.stop_tracking()
            if self.chase_planner and hasattr(self.chase_planner, 'reset'):
                self.chase_planner.reset()
            if self.semantic_lidar:
                self.semantic_lidar.cleanup()
            if self.zero_copy_camera:
                self.zero_copy_camera.cleanup()
            if self.zenoh_collision_manager:
                self.zenoh_collision_manager.cleanup()
            if self.ros2_node and ROS2_AVAILABLE:
                try:
                    self.ros2_node.destroy_node()
                    if rclpy.ok():
                        rclpy.shutdown()
                except Exception:
                    pass
            # print("✅ Cleanup completed")
        except Exception as e:
            print(f"[WARN] Error during cleanup: {e}", flush=True)


def main():
    try:
        app = AutoChaseVehicleControl()
        app.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[FATAL] Error in main: {e}", flush=True)


if __name__ == "__main__":
    main()
