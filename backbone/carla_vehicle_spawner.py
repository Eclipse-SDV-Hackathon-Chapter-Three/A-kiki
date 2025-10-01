#!/usr/bin/env python3
"""
CARLA Vehicle Spawner for ROS2 Bridge (Semantic LiDAR only, rear points removed)
- Stable clustering via ObjIdx grouping (no sklearn required)
- Publishes target clusters every interval even if empty (heartbeat)
- LiDAR(Body ENU) -> World ENU (IMU heading + GNSS) -> NED for absolute target coords
- Topics over Zenoh:
    - 'target_gps': targets with world_xyz (UE->Geo) + world_ned (NED)
    - 'lidar_gps' : ego GPS/world
    - 'imu'       : raw IMU
"""

import glob
import os
import sys
import struct
import socket
import pickle
import json
import time
import logging
import threading
import random
from typing import Sequence
from datetime import datetime, timezone
import uuid

# -----------------------------
# Add CARLA Python API to path
# -----------------------------
try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    try:
        sys.path.append(glob.glob('/home/seame/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    except IndexError:
        pass

import carla
import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import math

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False
    print("Warning: cv_bridge not available, using basic image conversion")

from sensor_msgs.msg import (
    CompressedImage, PointCloud2, PointField, NavSatFix, NavSatStatus
)
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32
from visualization_msgs.msg import Marker, MarkerArray

# -----------------------------
# Zenoh (optional)
# -----------------------------
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    print("Warning: zenoh not available, vehicle control via Zenoh will be disabled")


# =========================================================
# Configs / ENV
# =========================================================
CARLA_HOST = os.environ.get("CARLA_HOST", "localhost")
CARLA_PORT = int(os.environ.get("CARLA_PORT", "2000"))

# Publishing cadence
PUB_INTERVAL_SEC = float(os.environ.get("PUB_INTERVAL_SEC", "0.3"))

# Semantic LiDAR downsample stride
DOWNSAMPLE_STRIDE = int(os.environ.get("SEM_LIDAR_STRIDE", "1"))

# Include only these semantic tags: 12 (Pedestrian), 14 (Car)
_env_include = os.environ.get("INCLUDE_TAGS", "")
if _env_include.strip():
    INCLUDE_TAGS = {int(x.strip()) for x in _env_include.split(",") if x.strip().isdigit()}
else:
    INCLUDE_TAGS = {12, 14}

# ObjIdx-based cluster filtering
MIN_CLUSTER_POINTS = int(os.environ.get("MIN_CLUSTER_POINTS", "10"))

# Rear points removal (sensor frame)
def _as_bool(s: str) -> bool:
    return s.lower() in ("1","true","yes","on","y")
REMOVE_REAR_POINTS = _as_bool(os.environ.get("REMOVE_REAR_POINTS", "true"))
REAR_X_MIN = float(os.environ.get("REAR_X_MIN", "0.0"))  # keep x >= this

# Ego-near exclusion
EGO_EXCLUDE_RADIUS = float(os.environ.get("EGO_EXCLUDE_RADIUS", "2.5"))  # spherical
EGO_BOX_ENABLE = _as_bool(os.environ.get("EGO_BOX_ENABLE", "true"))
EGO_BOX_FRONT = float(os.environ.get("EGO_BOX_FRONT", "2.5"))   # +x (forward)
EGO_BOX_BACK  = float(os.environ.get("EGO_BOX_BACK",  "1.5"))   # -x (backward)
EGO_BOX_Y     = float(os.environ.get("EGO_BOX_Y",     "1.2"))   # |y| <= Y
EGO_BOX_Z_UP  = float(os.environ.get("EGO_BOX_Z_UP",  "0.8"))   # z <= +Z_UP
EGO_BOX_Z_DOWN= float(os.environ.get("EGO_BOX_Z_DOWN","2.5"))   # z >= -Z_DOWN

# Palette for 12/14
PALETTE = {
    12: np.array([220, 20, 60], dtype=np.uint8),   # Pedestrian
    14: np.array([0, 0, 142], dtype=np.uint8),     # Car
}

# Names
OBJECT_NAMES = {12: "Pedestrian", 14: "Car"}

def pack_rgb_uint8_to_float(r_u8, g_u8, b_u8) -> float:
    import struct as _st
    rgb_uint32 = (int(r_u8) << 16) | (int(g_u8) << 8) | int(b_u8)
    return _st.unpack('<f', _st.pack('<I', rgb_uint32))[0]


# =========================================================
# NED / ENU helper
# =========================================================
class NedTransformer:
    """
    LiDAR(Body ENU: x fwd, y left, z up) -> World ENU (IMU heading) -> ENU translate (GNSS) -> NED
    """
    def __init__(self, ref_lat, ref_lon, ref_alt):
        self.a = 6378137.0
        self.f = 1/298.257223563
        self.b = self.a * (1 - self.f)
        self.e2 = 1 - (self.b**2 / self.a**2)
        self.ref_lat = math.radians(ref_lat)
        self.ref_lon = math.radians(ref_lon)
        self.ref_alt = ref_alt
        self._ref_ecef = self._lla_to_ecef(ref_lat, ref_lon, ref_alt)
        self._R_ecef2enu = self._rot_ecef_to_enu(self.ref_lat, self.ref_lon)

    def _lla_to_ecef(self, lat_deg, lon_deg, alt):
        lat = math.radians(lat_deg); lon = math.radians(lon_deg)
        N = self.a / math.sqrt(1 - self.e2 * (math.sin(lat)**2))
        x = (N + alt) * math.cos(lat) * math.cos(lon)
        y = (N + alt) * math.cos(lat) * math.sin(lon)
        z = (N*(1 - self.e2) + alt) * math.sin(lat)
        return np.array([x, y, z], dtype=np.float64)

    def _rot_ecef_to_enu(self, lat, lon):
        sL, cL = math.sin(lat), math.cos(lat)
        sλ, cλ = math.sin(lon), math.cos(lon)
        return np.array([
            [-sλ,           cλ,          0.0],    # East
            [-sL*cλ, -sL*sλ,        cL],         # North
            [ cL*cλ,  cL*sλ,        sL],         # Up
        ], dtype=np.float64)

    def lla_to_enu(self, lat_deg, lon_deg, alt):
        ecef = self._lla_to_ecef(lat_deg, lon_deg, alt)
        return (self._R_ecef2enu @ (ecef - self._ref_ecef).reshape(3,1)).reshape(3)

    @staticmethod
    def enu_to_ned(v_enu):
        E, N, U = v_enu
        return np.array([N, E, -U], dtype=np.float64)

    @staticmethod
    def rotz(yaw_rad):
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        return np.array([[c, -s, 0],
                         [s,  c, 0],
                         [0,  0, 1]], dtype=np.float64)

    def body_enu_to_world_enu(self, pts_body_enu, yaw_from_north_rad):
        # ENU yaw 기준으로 변환 (동=0, 반시계+). compass(북 기준) -> ψ_ENU = π/2 - compass
        yaw_enu = (math.pi/2.0) - yaw_from_north_rad
        R = NedTransformer.rotz(yaw_enu)
        return (R @ pts_body_enu.T).T

    def transform_lidar_points_to_ned(self, pts_body_enu, yaw_from_north_rad, ego_lat, ego_lon, ego_alt):
        pts_world_enu = self.body_enu_to_world_enu(pts_body_enu, yaw_from_north_rad)
        t_world_enu = self.lla_to_enu(ego_lat, ego_lon, ego_alt)
        pts_world_enu += t_world_enu.reshape(1,3)
        ned = np.apply_along_axis(NedTransformer.enu_to_ned, 1, pts_world_enu)
        return ned


# =========================================================
# PointCloud2 helpers
# =========================================================
from sensor_msgs.msg import PointField

def _get_point_fields(fields_spec: Sequence[tuple]) -> Sequence[PointField]:
    fields = []
    offset = 0
    dtype_size = {
        PointField.INT8: 1, PointField.UINT8: 1,
        PointField.INT16: 2, PointField.UINT16: 2,
        PointField.INT32: 4, PointField.UINT32: 4,
        PointField.FLOAT32: 4, PointField.FLOAT64: 8
    }
    for name, dtype, count in fields_spec:
        fields.append(PointField(name=name, offset=offset, datatype=dtype, count=count))
        offset += dtype_size[dtype] * count
    return fields, offset


def create_pointcloud2_msg(node: Node, points: np.ndarray, frame_id: str, fields_spec: Sequence[tuple]) -> PointCloud2:
    import struct as _st
    msg = PointCloud2()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    fields, point_step = _get_point_fields(fields_spec)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.height = 1
    msg.width = points.shape[0] if hasattr(points, "shape") else len(points)
    msg.is_dense = True

    buffers = []
    for row in points:
        packed = []
        for (name, dtype, count) in fields_spec:
            val = row[name] if isinstance(row, np.void) else row[name]
            if dtype == PointField.FLOAT32:
                if count == 1:
                    packed.append(_st.pack('<f', float(val)))
                else:
                    for v in val:
                        packed.append(_st.pack('<f', float(v)))
            elif dtype == PointField.UINT32:
                if count == 1:
                    packed.append(_st.pack('<I', int(val)))
                else:
                    for v in val:
                        packed.append(_st.pack('<I', int(v)))
            elif dtype == PointField.INT32:
                if count == 1:
                    packed.append(_st.pack('<i', int(val)))
                else:
                    for v in val:
                        packed.append(_st.pack('<i', int(v)))
            else:
                raise ValueError(f"Unsupported dtype in pack: {dtype}")
        buffers.append(b"".join(packed))

    msg.row_step = msg.point_step * msg.width
    msg.data = b"".join(buffers)
    return msg


# =========================================================
# Marker helpers
# =========================================================
from visualization_msgs.msg import Marker, MarkerArray

OBJECT_NAMES = {12: "Pedestrian", 14: "Car"}

def calculate_bounding_box(points: np.ndarray):
    if len(points) == 0:
        return np.array([0, 0, 0]), np.array([0, 0, 0])
    min_vals = np.min(points, axis=0)
    max_vals = np.max(points, axis=0)
    center = (min_vals + max_vals) / 2.0
    dimensions = max_vals - min_vals
    return center, dimensions


def create_cluster_markers(header, centers, dims_list, sem_labels):
    marker_array = MarkerArray()
    for i, (center, dimensions, sem) in enumerate(zip(centers, dims_list, sem_labels)):
        marker = Marker()
        marker.header = header
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(dimensions[0])
        marker.scale.y = float(dimensions[1])
        marker.scale.z = float(dimensions[2])

        color = PALETTE.get(int(sem), np.array([255,255,255], dtype=np.uint8))/255.0
        marker.color.r = float(color[0]); marker.color.g = float(color[1]); marker.color.b = float(color[2]); marker.color.a = 0.6
        marker.lifetime.sec = 1; marker.lifetime.nanosec = 0
        marker_array.markers.append(marker)

        text = Marker()
        text.header = header
        text.id = i + 1000
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(center[0])
        text.pose.position.y = float(center[1])
        text.pose.position.z = float(center[2] + dimensions[2] / 2 + 0.5)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.5
        text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0
        name = OBJECT_NAMES.get(int(sem), f"Unknown_{int(sem)}")
        text.text = f"{name}"
        text.lifetime.sec = 1; text.lifetime.nanosec = 0
        marker_array.markers.append(text)
    return marker_array


# =========================================================
# CarlaVehicleSpawner
# =========================================================
from sensor_msgs.msg import CompressedImage, PointCloud2, NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32

class CarlaVehicleSpawner(Node):
    def __init__(self):
        node_name = f'carla_vehicle_spawner_{int(time.time())}'
        super().__init__(node_name)

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        self.carla_host = os.environ.get("CARLA_HOST", "localhost")
        self.carla_port = int(os.environ.get("CARLA_PORT", "2000"))
        self.vehicle_blueprint_filter = 'vehicle.dodge.charger_police_2020'
        self.spawn_transform = carla.Transform(
            carla.Location(x=-104.15, y=44.15 + 7.0, z=1.0),
            carla.Rotation(pitch=0.0, yaw=270.0, roll=0.0)
        )

        self.client = None
        self.world = None
        self.vehicle = None
        self.sensors = {}

        self.latest_gnss = {"lat": None, "lon": None, "alt": None}
        self.last_compass = None   # radians (north=0)
        self._ned_ref = None

        # buffers
        self.point_buffer = []   # Nx3 float32 (body ENU, ROS-like)
        self.label_buffer = []   # N uint32 (ObjTag)
        self.objidx_buffer = []  # N uint32 (ObjIdx)
        self.buffer_frames = 3
        self.current_frame = 0
        self.last_publish_time = 0.0
        self.publish_interval = PUB_INTERVAL_SEC

        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
        self.setup_publishers()

        # VSS data generation
        self.setup_vss_data()

        # Police simulation state
        self.simulation_time = 0
        self.emergency_scenarios = self._initialize_emergency_scenarios()
        self.current_scenario = None
        self.scenario_start_time = None

        self.zenoh_session = None
        self.zenoh_subscriber = None
        self.zenoh_emergency_subscriber = None  # Emergency alert 구독
        self.zenoh_pub_target_gps = None
        self.zenoh_pub_lidar_gps = None
        self.zenoh_pub_imu = None
        self.last_control_command = None
        self.control_command_lock = threading.Lock()

        # Emergency alert 상태 (외부에서 제어)
        self.external_emergency_active = False

        if ZENOH_AVAILABLE:
            self.setup_zenoh()

        self.connect_to_carla()
        self.spawn_vehicle()
        self.setup_sensors()

        self.logger.info("CARLA Vehicle Spawner initialized successfully")

    # --------------------------
    def setup_publishers(self):
        self.camera_pub = self.create_publisher(CompressedImage, '/carla/hero/camera/image/compressed', 2000)
        self.telemetry_pub = self.create_publisher(String, '/carla/hero/vehicle_status', 10)
        self.police_id_pub = self.create_publisher(Int32, '/carla/hero/vehicle_name', 10)
        self.odometry_pub = self.create_publisher(PoseStamped, '/carla/hero/odometry', 10)
        self.semantic_lidar_pub = self.create_publisher(PointCloud2, '/carla/hero/semantic_lidar/point_cloud', 10)
        self.cluster_markers_pub = self.create_publisher(MarkerArray, '/carla/hero/cluster_markers', 10)
        self.detected_objects_pub = self.create_publisher(String, '/carla/hero/detected_objects', 10)
        self.gnss_pub = self.create_publisher(NavSatFix, '/carla/hero/gnss', 10)
        self.vehicle_id_pub = self.create_publisher(Int32, '/carla/hero/vehicle_id', 10)

        # Publishers from the other spawner
        self.map_camera_pub = self.create_publisher(CompressedImage, '/carla/map_camera/image/compressed', 10)
        self.vss_publisher = self.create_publisher(String, 'police/vss_data', 10)

        self.timer = self.create_timer(0.1, self.publish_telemetry)   # 10 Hz
        self.control_timer = self.create_timer(0.05, self.apply_control_commands)  # 20 Hz
        self.vss_timer = self.create_timer(0.5, self.publish_vss_data) # 2Hz for VSS data

    # --------------------------
    def connect_to_carla(self):
        try:
            self.client = carla.Client(self.carla_host, self.carla_port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            self.logger.info(f"Connected to CARLA simulator at {self.carla_host}:{self.carla_port}")
        except Exception as e:
            self.logger.error(f"Failed to connect to CARLA: {e}")
            raise

    def cleanup_existing_vehicles(self):
        try:
            vehicles = self.world.get_actors().filter('vehicle.*')
            hero_vehicles = []
            for v in vehicles:
                try:
                    role = v.attributes.get('role_name', '')
                    if role == 'hero':
                        hero_vehicles.append(v)
                except Exception:
                    pass
            if hero_vehicles:
                self.logger.info(f"Found {len(hero_vehicles)} existing hero vehicles. Cleaning up...")
                sensors = self.world.get_actors().filter('sensor.*')
                for hv in hero_vehicles:
                    try:
                        for s in sensors:
                            try:
                                if s.parent and s.parent.id == hv.id:
                                    s.destroy()
                            except Exception:
                                pass
                        hv.destroy()
                        self.logger.info(f"Destroyed existing vehicle: {hv.id}")
                    except Exception as e:
                        self.logger.warning(f"Failed to destroy vehicle {hv.id}: {e}")
                time.sleep(1)
        except Exception as e:
            self.logger.warning(f"Error during vehicle cleanup: {e}")

    def spawn_vehicle(self):
        try:
            self.cleanup_existing_vehicles()
            bp_lib = self.world.get_blueprint_library()
            vehicle_bp = bp_lib.filter(self.vehicle_blueprint_filter)[0]
            vehicle_bp.set_attribute('role_name', 'hero')

            self.vehicle = self.world.spawn_actor(vehicle_bp, self.spawn_transform)
            self.logger.info(f"Spawned vehicle: {self.vehicle.type_id} (ID: {self.vehicle.id})")
            self.vehicle.set_autopilot(False)
        except Exception as e:
            self.logger.error(f"Failed to spawn vehicle: {e}")
            raise

    def setup_sensors(self):
        try:
            bp_lib = self.world.get_blueprint_library()

            camera_bp = bp_lib.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '1080')
            camera_bp.set_attribute('image_size_y', '720')
            camera_bp.set_attribute('fov', '90')
            camera_tf = carla.Transform(carla.Location(x=1.5, z=1.6))
            self.sensors['camera'] = self.world.spawn_actor(camera_bp, camera_tf, attach_to=self.vehicle)
            self.sensors['camera'].listen(self.camera_callback)

            sem_bp = bp_lib.find('sensor.lidar.ray_cast_semantic')
            sem_bp.set_attribute('channels', '32')
            sem_bp.set_attribute('points_per_second', '400000')
            sem_bp.set_attribute('rotation_frequency', '20')
            sem_bp.set_attribute('range', '100')
            sem_bp.set_attribute('upper_fov', '5')
            sem_bp.set_attribute('lower_fov', '-45')
            sem_tf = carla.Transform(carla.Location(x=0.0, z=2.5))
            self.sensors['semantic_lidar'] = self.world.spawn_actor(sem_bp, sem_tf, attach_to=self.vehicle)
            self.sensors['semantic_lidar'].listen(self.semantic_lidar_callback)

            gnss_bp = bp_lib.find('sensor.other.gnss')
            gnss_tf = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.0))
            self.sensors['gnss'] = self.world.spawn_actor(gnss_bp, gnss_tf, attach_to=self.vehicle)
            self.sensors['gnss'].listen(self.gps_callback)

            imu_bp = bp_lib.find('sensor.other.imu')
            imu_tf = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.0))
            self.sensors['imu'] = self.world.spawn_actor(imu_bp, imu_tf, attach_to=self.vehicle)
            self.sensors['imu'].listen(self.imu_callback)

            # Map camera (looking down from above)
            map_bp = bp_lib.find('sensor.camera.rgb')
            map_bp.set_attribute('image_size_x', '1080')
            map_bp.set_attribute('image_size_y', '1080')
            map_bp.set_attribute('fov', '90')
            spawn_points = self.world.get_map().get_spawn_points()
            x_coords = [p.location.x for p in spawn_points]
            y_coords = [p.location.y for p in spawn_points]
            center_x = sum(x_coords) / len(x_coords) if x_coords else 0
            center_y = sum(y_coords) / len(y_coords) if y_coords else 0
            map_location = carla.Location(x=center_x, y=center_y, z=200)
            map_rotation = carla.Rotation(pitch=-90)
            map_transform = carla.Transform(map_location, map_rotation)
            self.sensors['map_camera'] = self.world.spawn_actor(map_bp, map_transform)
            self.sensors['map_camera'].listen(self.map_camera_callback)

            self.logger.info("Sensors setup completed (camera, semantic_lidar, gnss, imu, map_camera)")
        except Exception as e:
            self.logger.error(f"Failed to setup sensors: {e}")

    # --------------------------
    def map_camera_callback(self, image):
        """Process map camera data and publish to ROS2"""
        try:
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]

            ros_image = CompressedImage()
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "map_camera"
            ros_image.format = "jpeg"

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
            _, compressed_data = cv2.imencode('.jpg', array, encode_param)
            ros_image.data = compressed_data.tobytes()

            self.map_camera_pub.publish(ros_image)
        except Exception as e:
            self.logger.error(f"Error in map_camera_callback: {e}")

    def camera_callback(self, image):
        try:
            if not rclpy.ok():
                return
            array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
            array = array[:, :, :3]
            ros_image = CompressedImage()
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "ego_camera"
            ros_image.format = "jpeg"
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            _, compressed_data = cv2.imencode('.jpg', array, encode_param)
            ros_image.data = compressed_data.tobytes()
            self.camera_pub.publish(ros_image)
        except Exception as e:
            self.logger.error(f"Error in camera callback: {e}")

    def imu_callback(self, imu_meas):
        # cache compass
        try:
            self.last_compass = float(imu_meas.compass)
        except Exception:
            self.last_compass = None

        if not (ZENOH_AVAILABLE and self.zenoh_session and self.zenoh_pub_imu):
            return
        try:
            payload = {
                "timestamp": time.time(),
                "vehicle_id": int(self.vehicle.id) if self.vehicle else None,
                "imu": {
                    "accelerometer": {
                        "x": float(imu_meas.accelerometer.x),
                        "y": float(imu_meas.accelerometer.y),
                        "z": float(imu_meas.accelerometer.z),
                    },
                    "gyroscope": {
                        "x": float(imu_meas.gyroscope.x),
                        "y": float(imu_meas.gyroscope.y),
                        "z": float(imu_meas.gyroscope.z),
                    },
                    "compass": float(imu_meas.compass)
                }
            }
            self.zenoh_pub_imu.put(json.dumps(payload))
        except Exception as e:
            self.logger.error(f"Error publishing IMU via Zenoh: {e}")

    # Ego exclusion
    def _apply_ego_exclusion(self, lidar_struct: np.ndarray) -> np.ndarray:
        arr = lidar_struct
        if arr.size == 0:
            return arr

        if REMOVE_REAR_POINTS:
            arr = arr[arr['x'] >= REAR_X_MIN]
            if arr.size == 0:
                return arr

        if EGO_BOX_ENABLE and arr.size > 0:
            x = arr['x']; y = arr['y']; z = arr['z']
            box_mask = (
                (x >= -EGO_BOX_BACK) & (x <= EGO_BOX_FRONT) &
                (np.abs(y) <= EGO_BOX_Y) &
                (z >= -EGO_BOX_Z_DOWN) & (z <= EGO_BOX_Z_UP)
            )
            arr = arr[~box_mask]
            if arr.size == 0:
                return arr

        if EGO_EXCLUDE_RADIUS > 0.0 and arr.size > 0:
            r2 = EGO_EXCLUDE_RADIUS * EGO_EXCLUDE_RADIUS
            dist2 = arr['x'] * arr['x'] + arr['y'] * arr['y'] + arr['z'] * arr['z']
            arr = arr[dist2 >= r2]

        return arr

    def _clusters_from_objidx(self, points_xyz: np.ndarray, labels_sem: np.ndarray, objidx: np.ndarray):
        """
        Group by ObjIdx; discard small groups; assign dominant semantic (mode) per group.
        Returns: centers(list Nx3), dims(list Nx3), sem_labels(list), groups(list of np.ndarray indices)
        """
        if len(points_xyz) == 0:
            return [], [], [], []

        # Ignore objidx=0 (often "none")
        valid = objidx != 0
        if not np.any(valid):
            return [], [], [], []

        objidx_valid = objidx[valid]
        pts_valid = points_xyz[valid]
        sem_valid = labels_sem[valid]

        uniq_ids, start_idx = np.unique(objidx_valid, return_index=True)
        centers = []; dims_list = []; sem_labels = []; groups = []

        # Use sorting to group indices for performance
        order = np.argsort(objidx_valid, kind='mergesort')
        obj_sorted = objidx_valid[order]
        pts_sorted = pts_valid[order]
        sem_sorted = sem_valid[order]

        # boundaries of groups
        boundaries = np.r_[0, np.nonzero(np.diff(obj_sorted))[0] + 1, len(obj_sorted)]

        for i in range(len(boundaries)-1):
            a, b = boundaries[i], boundaries[i+1]
            group_pts = pts_sorted[a:b]
            group_sem = sem_sorted[a:b]
            if group_pts.shape[0] < MIN_CLUSTER_POINTS:
                continue
            # dominant semantic
            uniq_sem, counts = np.unique(group_sem, return_counts=True)
            dom_sem = int(uniq_sem[np.argmax(counts)])
            # center/dims
            c, d = calculate_bounding_box(group_pts)
            centers.append(c); dims_list.append(d); sem_labels.append(dom_sem)
            # original indices (for debugging if needed)
            groups.append((obj_sorted[a], a, b))

        return centers, dims_list, sem_labels, groups

    def _clusters_to_gps(self, header, centers_ros_like, sem_labels):
        """
        Centers in ROS-like body frame (y flipped already relative to UE).
        Convert to UE world, then to Geo if possible.
        """
        results = []
        try:
            sensor_tf: carla.Transform = self.sensors['semantic_lidar'].get_transform()
            world_map = self.world.get_map() if self.world else None
        except Exception:
            sensor_tf = None
            world_map = None

        ts = header.stamp.sec + header.stamp.nanosec * 1e-9

        for i, (center_ros, sem) in enumerate(zip(centers_ros_like, sem_labels)):
            center_ue = carla.Location(x=float(center_ros[0]),
                                       y=float(-center_ros[1]),
                                       z=float(center_ros[2]))
            if sensor_tf is not None:
                world_loc: carla.Location = sensor_tf.transform(center_ue)
                world_xyz = {"x": world_loc.x, "y": world_loc.y, "z": world_loc.z}
            else:
                world_xyz = {"x": float(center_ros[0]), "y": float(center_ros[1]), "z": float(center_ros[2])}

            lat = lon = alt = None
            try:
                if world_map and hasattr(world_map, "transform_to_geolocation"):
                    geoloc = world_map.transform_to_geolocation(
                        carla.Location(x=world_xyz["x"], y=world_xyz["y"], z=world_xyz["z"])
                    )
                    lat, lon, alt = float(geoloc.latitude), float(geoloc.longitude), float(geoloc.altitude)
            except Exception:
                pass

            results.append({
                "id": i,
                "semantic_label": int(sem),
                "name": OBJECT_NAMES.get(int(sem), f"Unknown_{int(sem)}"),
                "gps": {"lat": lat, "lon": lon, "alt": alt} if lat is not None else None,
                "world": world_xyz,
                "lidar_center_ros": { "x": float(center_ros[0]), "y": float(center_ros[1]), "z": float(center_ros[2]) },
                "timestamp": ts
            })
        return results

    # --------------------------
    def semantic_lidar_callback(self, semantic_lidar_data):
        try:
            from std_msgs.msg import Header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"  # RViz fixed frame (ENU-like viewing)

            # fields: x,y,z,rgb(float32), ObjTag(uint32)
            fields = [
                PointField(name='x',      offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y',      offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z',      offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb',    offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='ObjTag', offset=16, datatype=PointField.UINT32,  count=1),
            ]

            lidar = np.frombuffer(bytes(semantic_lidar_data.raw_data),
                                  dtype=np.dtype([
                                      ('x', np.float32),
                                      ('y', np.float32),
                                      ('z', np.float32),
                                      ('CosAngle', np.float32),
                                      ('ObjIdx', np.uint32),
                                      ('ObjTag', np.uint32)
                                  ])).copy()

            # UE → ROS-like axis (flip y)
            lidar['y'] *= -1.0

            # Keep only requested tags (12/14)
            if INCLUDE_TAGS:
                lidar = lidar[np.isin(lidar['ObjTag'], list(INCLUDE_TAGS))]

            # Ego removal
            lidar = self._apply_ego_exclusion(lidar)

            # Downsample
            if DOWNSAMPLE_STRIDE > 1 and len(lidar) > 0:
                lidar = lidar[::DOWNSAMPLE_STRIDE]

            # Buffering
            if len(lidar) > 0:
                pts = np.stack([lidar['x'], lidar['y'], lidar['z']], axis=-1)
                tags = lidar['ObjTag'].astype(np.uint32)
                idxs = lidar['ObjIdx'].astype(np.uint32)
                self.point_buffer.append(pts)
                self.label_buffer.append(tags)
                self.objidx_buffer.append(idxs)
                self.current_frame += 1

            now = time.time()
            should_publish = (len(self.point_buffer) >= self.buffer_frames) or ((now - self.last_publish_time) >= self.publish_interval)

            if should_publish:
                if (len(self.point_buffer) > 0):
                    all_points = np.vstack(self.point_buffer)
                    all_labels = np.hstack(self.label_buffer)
                    all_objidx = np.hstack(self.objidx_buffer)
                else:
                    all_points = np.empty((0,3), dtype=np.float32)
                    all_labels = np.empty((0,), dtype=np.uint32)
                    all_objidx = np.empty((0,), dtype=np.uint32)

                # visualize cloud (colors by tag)
                if len(all_points) > 0:
                    tags_arr = all_labels.astype(np.int32)
                    colors_u8 = np.empty((len(all_points), 3), dtype=np.uint8)
                    mask12 = (tags_arr == 12); mask14 = (tags_arr == 14)
                    if mask12.any(): colors_u8[mask12] = PALETTE[12]
                    if mask14.any(): colors_u8[mask14] = PALETTE[14]
                    rgb_f32 = [pack_rgb_uint8_to_float(r, g, b) for r, g, b in colors_u8]
                    points_list = [
                        (float(all_points[i, 0]),
                         float(all_points[i, 1]),
                         float(all_points[i, 2]),
                         float(rgb_f32[i]),
                         int(tags_arr[i]))
                        for i in range(len(all_points))
                    ]
                else:
                    points_list = []

                cloud = self._create_cloud_rviz(header, fields, points_list)
                self.semantic_lidar_pub.publish(cloud)

                # -------- ObjIdx-based clustering (robust) --------
                centers = []; dims_list = []; sem_labels = []
                try:
                    centers, dims_list, sem_labels, _ = self._clusters_from_objidx(all_points, all_labels, all_objidx)
                except Exception as e:
                    self.logger.error(f"ObjIdx clustering error: {e}")
                    centers, dims_list, sem_labels = [], [], []

                # RViz markers (optional)
                if centers:
                    markers = create_cluster_markers(header, centers, dims_list, sem_labels)
                    self.cluster_markers_pub.publish(markers)

                # ---- GPS centers via UE transform (for Geo/world XYZ) ----
                gps_targets = []
                try:
                    gps_targets = self._clusters_to_gps(header, centers, sem_labels) if centers else []
                except Exception as e:
                    self.logger.error(f"prepare GPS targets error: {e}")

                # ---- NED via IMU+GNSS ----
                try:
                    # set NED ref once
                    if (self._ned_ref is None) and all(v is not None for v in self.latest_gnss.values()):
                        self._ned_ref = (self.latest_gnss["lat"], self.latest_gnss["lon"], self.latest_gnss["alt"])
                        self.get_logger().info(f"[NED] Reference set to LLA {self._ned_ref}")

                    if centers and (self._ned_ref is not None) and all(v is not None for v in self.latest_gnss.values()):
                        ref_lat, ref_lon, ref_alt = self._ned_ref
                        ego_lat = self.latest_gnss["lat"]; ego_lon = self.latest_gnss["lon"]; ego_alt = self.latest_gnss["alt"]
                        tf_ned = NedTransformer(ref_lat, ref_lon, ref_alt)

                        compass = self.last_compass
                        if compass is None:
                            yaw_deg = self.vehicle.get_transform().rotation.yaw
                            compass = math.radians(yaw_deg)

                        centers_arr = np.array(centers, dtype=np.float64)  # body ENU (ROS-like)
                        centers_ned = tf_ned.transform_lidar_points_to_ned(
                            centers_arr,
                            yaw_from_north_rad=float(compass),
                            ego_lat=ego_lat, ego_lon=ego_lon, ego_alt=ego_alt
                        )
                        if gps_targets and (len(gps_targets) == len(centers_ned)):
                            for t, ned in zip(gps_targets, centers_ned):
                                t["world_ned"] = {"N": float(ned[0]), "E": float(ned[1]), "D": float(ned[2])}
                        elif not gps_targets and len(centers_ned) > 0:
                            # publish NED-only targets if Geo unavailable
                            for i, (sem, ned) in enumerate(zip(sem_labels, centers_ned)):
                                gps_targets.append({
                                    "id": i,
                                    "semantic_label": int(sem),
                                    "name": OBJECT_NAMES.get(int(sem), f"Unknown_{int(sem)}"),
                                    "gps": None,
                                    "world": None,
                                    "lidar_center_ros": { "x": float(centers_arr[i,0]), "y": float(centers_arr[i,1]), "z": float(centers_arr[i,2]) },
                                    "world_ned": {"N": float(ned[0]), "E": float(ned[1]), "D": float(ned[2])},
                                    "timestamp": header.stamp.sec + header.stamp.nanosec * 1e-9
                                })
                except Exception as e:
                    self.logger.error(f"NED transform error: {e}")

                # ---- Always publish target_gps (even if empty) ----
                try:
                    payload = {
                        "timestamp": header.stamp.sec + header.stamp.nanosec * 1e-9,
                        "vehicle_id": int(self.vehicle.id) if self.vehicle else None,
                        "targets": gps_targets  # may be []
                    }
                    self._publish_target_gps_zenoh(payload)
                except Exception as e:
                    self.logger.error(f"Error publishing target_gps: {e}")

                # Optional TCP stream
                try:
                    if len(all_points) > 0:
                        payload_pts = {'points': all_points, 'labels': all_labels}
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.settimeout(0.01)
                        s.connect(('localhost', 50051))
                        s.sendall(pickle.dumps(payload_pts) + b'END')
                        s.close()
                except Exception:
                    pass

                # reset buffers & timer
                self.point_buffer = []; self.label_buffer = []; self.objidx_buffer = []
                self.current_frame = 0; self.last_publish_time = now

            # else: do nothing until interval/frames met

        except Exception as e:
            self.logger.error(f"Error in semantic LiDAR callback: {e}")

    # --------------------------
    def gps_callback(self, gps):
        try:
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.latitude = gps.latitude
            msg.longitude = gps.longitude
            msg.altitude = gps.altitude
            msg.position_covariance = [0.0]*9
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.gnss_pub.publish(msg)
            self.latest_gnss = {"lat": float(gps.latitude), "lon": float(gps.longitude), "alt": float(gps.altitude)}
        except Exception as e:
            self.logger.error(f"Error in GNSS callback: {e}")

    def publish_telemetry(self):
        if self.vehicle is None:
            return
        try:
            tf = self.vehicle.get_transform()
            vel = self.vehicle.get_velocity()
            ctrl = self.vehicle.get_control()
            speed_kmh = (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5 * 3.6

            telemetry = {
                'timestamp': time.time(),
                'position': {'x': tf.location.x, 'y': tf.location.y, 'z': tf.location.z},
                'rotation': {'pitch': tf.rotation.pitch, 'yaw': tf.rotation.yaw, 'roll': tf.rotation.roll},
                'velocity': {'x': vel.x, 'y': vel.y, 'z': vel.z},
                'speed': speed_kmh,
                'control': {
                    'throttle': ctrl.throttle,
                    'steering': ctrl.steer,
                    'brake': ctrl.brake,
                    'hand_brake': ctrl.hand_brake,
                    'reverse': ctrl.reverse,
                    'gear': 1
                }
            }
            msg = String(); msg.data = json.dumps(telemetry)
            self.telemetry_pub.publish(msg)

            id_msg = Int32(); id_msg.data = self.vehicle.id
            self.police_id_pub.publish(id_msg); self.vehicle_id_pub.publish(id_msg)

            odom = PoseStamped()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "map"
            odom.pose.position.x = tf.location.x
            odom.pose.position.y = tf.location.y
            odom.pose.position.z = tf.location.z
            self.odometry_pub.publish(odom)

            # zenoh 'lidar_gps' (ego)
            try:
                gps_dict = None
                if all(v is not None for v in self.latest_gnss.values()):
                    gps_dict = dict(self.latest_gnss)
                else:
                    try:
                        world_map = self.world.get_map() if self.world else None
                        if world_map and hasattr(world_map, "transform_to_geolocation"):
                            geoloc = world_map.transform_to_geolocation(tf.location)
                            gps_dict = {
                                "lat": float(geoloc.latitude),
                                "lon": float(geoloc.longitude),
                                "alt": float(geoloc.altitude),
                            }
                    except Exception:
                        gps_dict = None

                payload_ego = {
                    "timestamp": time.time(),
                    "vehicle_id": int(self.vehicle.id),
                    "ego": {
                        "gps": gps_dict,
                        "world": {"x": tf.location.x, "y": tf.location.y, "z": tf.location.z},
                        "rotation": {"pitch": tf.rotation.pitch, "yaw": tf.rotation.yaw, "roll": tf.rotation.roll},
                        "speed_kmh": speed_kmh
                    }
                }
                self._publish_lidar_gps_zenoh(payload_ego)
            except Exception as e:
                self.logger.error(f"Error preparing lidar_gps payload: {e}")

        except Exception as e:
            self.logger.error(f"Error publishing telemetry: {e}")

    # --------------------------
    def setup_zenoh(self):
        try:
            zenoh_config = zenoh.Config()
            try:
                zenoh_config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
                zenoh_config.insert_json5("transport/shared_memory/enabled", "true")
                self.logger.info("🚀 Connecting to SHM-enabled Zenoh router...")
            except Exception:
                self.logger.info("📡 Using peer-to-peer Zenoh connection...")

        # open session
            self.zenoh_session = zenoh.open(zenoh_config)
            self.logger.info("✅ Connected to Zenoh router successfully")

            self.zenoh_subscriber = self.zenoh_session.declare_subscriber(
                "carla/vehicle/control", self.on_control_command
            )
            self.logger.info("🎮 Subscribed to vehicle control commands via Zenoh")

            # Emergency alert 구독
            self.zenoh_emergency_subscriber = self.zenoh_session.declare_subscriber(
                "police/central/alerts/emergency/UNIT-001", self.on_emergency_alert
            )
            self.logger.info("🚨 Subscribed to emergency alerts via Zenoh")

            self.zenoh_pub_target_gps = self.zenoh_session.declare_publisher("target_gps")
            self.logger.info("🛰️ Publishing target GPS to 'target_gps'")
            self.zenoh_pub_lidar_gps = self.zenoh_session.declare_publisher("lidar_gps")
            self.logger.info("📍 Publishing ego GPS to 'lidar_gps'")
            self.zenoh_pub_imu = self.zenoh_session.declare_publisher("imu")
            self.logger.info("🧭 Publishing IMU to 'imu'")
        except Exception as e:
            self.logger.error(f"Failed to setup Zenoh: {e}")
            self.zenoh_session = None
            self.zenoh_subscriber = None
            self.zenoh_emergency_subscriber = None
            self.zenoh_pub_target_gps = None
            self.zenoh_pub_lidar_gps = None
            self.zenoh_pub_imu = None

    def _publish_target_gps_zenoh(self, payload_obj: dict):
        if not (ZENOH_AVAILABLE and self.zenoh_session and self.zenoh_pub_target_gps):
            return
        try:
            self.zenoh_pub_target_gps.put(json.dumps(payload_obj))
        except Exception as e:
            self.logger.error(f"Error publishing target_gps via Zenoh: {e}")

    def _publish_lidar_gps_zenoh(self, payload_obj: dict):
        if not (ZENOH_AVAILABLE and self.zenoh_session and self.zenoh_pub_lidar_gps):
            return
        try:
            self.zenoh_pub_lidar_gps.put(json.dumps(payload_obj))
        except Exception as e:
            self.logger.error(f"Error publishing lidar_gps via Zenoh: {e}")

    def on_control_command(self, sample):
        try:
            control_data = json.loads(sample.payload.decode('utf-8'))
            with self.control_command_lock:
                self.last_control_command = control_data
        except Exception as e:
            self.logger.error(f"Error processing control command: {e}")

    def on_emergency_alert(self, sample):
        """Emergency alert 수신 콜백"""
        try:
            alert_data = json.loads(sample.payload.decode('utf-8'))
            self.logger.info(f"🚨 Emergency alert received: {alert_data}")

            # Emergency 상태 활성화
            if alert_data.get('type') == 'chase_started':
                self.external_emergency_active = True
                self.logger.info("🚨 External emergency activated - Chase started!")

                # Emergency 시나리오 적용 (추격 모드)
                pursuit_scenario = {
                    "name": "pursuit",
                    "duration": 300,  # 5분
                    "lightbar": "CODE3",
                    "siren": True,
                    "availability": "EMERGENCY",
                    "call_type": "PURSUIT",
                    "priority": "CRITICAL"
                }
                self.current_scenario = pursuit_scenario
                self.scenario_start_time = time.time()
                self._apply_scenario(pursuit_scenario)

        except Exception as e:
            self.logger.error(f"Error processing emergency alert: {e}")

    def apply_control_commands(self):
        try:
            if not self.vehicle or not self.last_control_command:
                return
            with self.control_command_lock:
                data = self.last_control_command or {}
                control = carla.VehicleControl()
                control.throttle = float(data.get('throttle', 0.0))
                control.steer = float(data.get('steer', 0.0))
                control.brake = float(data.get('brake', 0.0))
                control.hand_brake = bool(data.get('hand_brake', False))
                control.reverse = bool(data.get('reverse', False))
                control.manual_gear_shift = bool(data.get('manual_gear_shift', False))
                if control.manual_gear_shift:
                    control.gear = int(data.get('gear', 0))
                self.vehicle.apply_control(control)
        except Exception as e:
            self.logger.error(f"Error applying control commands: {e}")

    def setup_vss_data(self):
        """Initialize VSS data structure based on CARLA vehicle"""
        self.unit_config = {
            "unit_number": "UNIT-001",
            "district": "Central District",
            "precinct": "1st Precinct",
            "vehicle_vin": "1CARLA82633A123456",
            "vehicle_brand": "Dodge",
            "vehicle_model": "Charger Police 2020"
        }

        self.officers = {
            "primary": {
                "BadgeNumber": "12345",
                "Name": "Officer Johnson",
                "ShiftStart": datetime.now(timezone.utc).isoformat(),
                "Status": "ON_DUTY"
            },
            "secondary": {
                "BadgeNumber": "12346",
                "Name": "Officer Smith",
                "Status": "ON_DUTY"
            }
        }

        self.police_state = {
            "emergency_active": False,
            "siren_active": False,
            "lightbar_pattern": "OFF",
            "panic_button": False,
            "dash_camera_recording": True,
            "doors_locked": True,
            "availability": "AVAILABLE",
            "active_call": None,
            "last_update": time.time()
        }

    def _initialize_emergency_scenarios(self):
        """Initialize emergency scenarios for police simulation"""
        return [
            {
                "name": "traffic_stop",
                "duration": 300,
                "lightbar": "CODE1",
                "siren": False,
                "availability": "BUSY",
                "call_type": "TRAFFIC STOP",
                "priority": "LOW"
            },
            {
                "name": "pursuit",
                "duration": 180,
                "lightbar": "CODE3",
                "siren": True,
                "availability": "EMERGENCY",
                "call_type": "VEHICLE PURSUIT",
                "priority": "CRITICAL"
            },
            {
                "name": "routine_patrol",
                "duration": 1800,
                "lightbar": "OFF",
                "siren": False,
                "availability": "AVAILABLE",
                "call_type": None,
                "priority": None
            }
        ]

    def carla_to_vss_data(self):
        """Convert CARLA vehicle state to VSS-compliant police data"""
        if self.vehicle is None:
            return None

        try:
            transform = self.vehicle.get_transform()
            velocity = self.vehicle.get_velocity()
            control = self.vehicle.get_control()
            speed_ms = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            speed_kmh = speed_ms * 3.6

            lat = self.latest_gnss.get("lat") or 0.0
            lon = self.latest_gnss.get("lon") or 0.0

            self.simulation_time += 0.5
            self._update_emergency_scenarios()

            vss_data = {
                "metadata": {
                    "unit_id": self.unit_config["unit_number"],
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "source": "CARLA_INTEGRATED",
                    "version": "1.0"
                },
                "Vehicle": {
                    "VehicleIdentification": {
                        "VIN": self.unit_config["vehicle_vin"],
                        "Model": self.unit_config["vehicle_model"],
                        "Brand": self.unit_config["vehicle_brand"],
                        "Police": {
                            "UnitNumber": self.unit_config["unit_number"],
                            "District": self.unit_config["district"],
                            "Precinct": self.unit_config["precinct"]
                        }
                    },
                    "CurrentLocation": {
                        "Latitude": lat,
                        "Longitude": lon,
                        "Altitude": float(transform.location.z),
                        "Heading": float(transform.rotation.yaw % 360),
                        "Timestamp": datetime.now(timezone.utc).isoformat(),
                        "SimulationPosition": {
                            "x": float(transform.location.x),
                            "y": float(transform.location.y)
                        }
                    },
                    "Speed": float(speed_kmh),
                    "Police": {
                        "Status": {
                            "Availability": self.police_state["availability"],
                        },
                        "Emergency": {
                            "Lightbar": {"IsActive": self.police_state["emergency_active"]},
                            "Siren": {"IsActive": self.police_state["siren_active"]},
                            "PanicButton": {"IsPressed": self.police_state["panic_button"]}
                        },
                        "Equipment": {
                            "DashCamera": {"IsRecording": self.police_state["dash_camera_recording"]},
                            "MDT": {"ActiveCall": self.police_state["active_call"]},
                            "BodyCamera": {
                                "Officer1": {
                                    "IsActive": True,
                                    "BatteryLevel": 85
                                },
                                "Officer2": {
                                    "IsActive": True,
                                    "BatteryLevel": 92
                                }
                            }
                        },
                        "Officer": {
                            "Primary": self.officers["primary"],
                            "Secondary": self.officers["secondary"]
                        }
                    },
                }
            }
            return vss_data
        except Exception as e:
            self.logger.error(f"Error converting CARLA data to VSS: {e}")
            return None

    def _update_emergency_scenarios(self):
        current_time = time.time()

        # External emergency가 활성화된 경우 랜덤 시나리오를 건너뜀
        if self.external_emergency_active:
            # External emergency가 활성화되면 current_scenario는 on_emergency_alert에서 설정됨
            return

        # 시나리오 만료 확인
        if self.current_scenario and self.scenario_start_time:
            elapsed = current_time - self.scenario_start_time
            if elapsed >= self.current_scenario["duration"]:
                self.current_scenario = None
                self.scenario_start_time = None
                self._apply_scenario(self.emergency_scenarios[-1])

        # 랜덤 시나리오 생성 (비활성화됨 - 추격 시스템에서 제어)
        # if not self.current_scenario and random.random() < 0.01:
        #     scenario = random.choice(self.emergency_scenarios[:-1])
        #     self.current_scenario = scenario
        #     self.scenario_start_time = current_time
        #     self._apply_scenario(scenario)

        # 패닉 버튼도 비활성화 (추격 시스템에서 제어)
        # if random.random() < 0.00001:
        #     self.police_state["panic_button"] = True
        #     threading.Timer(5.0, lambda: setattr(self.police_state, "panic_button", False)).start()

    def _apply_scenario(self, scenario):
        self.police_state["availability"] = scenario["availability"]
        self.police_state["lightbar_pattern"] = scenario["lightbar"]
        self.police_state["siren_active"] = scenario["siren"]
        self.police_state["emergency_active"] = scenario["lightbar"] != "OFF"
        if scenario["call_type"]:
            self.police_state["active_call"] = {
                "type": scenario["call_type"],
                "priority": scenario["priority"],
                "start_time": datetime.now(timezone.utc).isoformat(),
                "call_id": str(uuid.uuid4())[:8]
            }
        else:
            self.police_state["active_call"] = None

    def publish_vss_data(self):
        vss_data = self.carla_to_vss_data()
        if vss_data:
            try:
                vss_msg = String()
                vss_msg.data = json.dumps(vss_data)
                self.vss_publisher.publish(vss_msg)
            except Exception as e:
                self.logger.error(f"Error publishing VSS data: {e}")

    def check_emergency_status(self):
        pass

    def _create_cloud_rviz(self, header, fields, points_list):
        import ctypes, struct as _st
        cloud_struct = _st.Struct(self._get_struct_fmt(False, fields))
        if not points_list:
            return PointCloud2(
                header=header, height=1, width=0, is_dense=False, is_bigendian=False,
                fields=fields, point_step=cloud_struct.size, row_step=0, data=b''
            )
        buff = ctypes.create_string_buffer(cloud_struct.size * len(points_list))
        offset = 0
        pack = cloud_struct.pack_into
        for p in points_list:
            x, y, z, rgb_f32, tag = p
            pack(buff, offset, float(x), float(y), float(z), float(rgb_f32), int(tag))
            offset += cloud_struct.size
        return PointCloud2(
            header=header, height=1, width=len(points_list), is_dense=False, is_bigendian=False,
            fields=fields, point_step=cloud_struct.size, row_step=cloud_struct.size * len(points_list),
            data=buff.raw
        )

    def _get_struct_fmt(self, is_bigendian, fields):
        fmt = '>' if is_bigendian else '<'
        datatypes = {
            PointField.INT8: ('b', 1),   PointField.UINT8: ('B', 1),
            PointField.INT16: ('h', 2),  PointField.UINT16: ('H', 2),
            PointField.INT32: ('i', 4),  PointField.UINT32: ('I', 4),
            PointField.FLOAT32: ('f', 4),PointField.FLOAT64: ('d', 8),
        }
        offset = 0
        for f in sorted(fields, key=lambda f: f.offset):
            if offset < f.offset:
                fmt += 'x' * (f.offset - offset)
                offset = f.offset
            dt_fmt, dt_len = datatypes[f.datatype]
            fmt += f.count * dt_fmt
            offset += f.count * dt_len
        return fmt

    def cleanup(self):
        try:
            if self.sensors:
                for s in self.sensors.values():
                    try:
                        if s is not None:
                            s.destroy()
                    except Exception:
                        pass
            if self.vehicle is not None:
                try: self.vehicle.destroy()
                except Exception: pass

            if self.zenoh_subscriber:
                try: self.zenoh_subscriber.undeclare()
                except Exception: pass
            if self.zenoh_emergency_subscriber:
                try: self.zenoh_emergency_subscriber.undeclare()
                except Exception: pass
            if self.zenoh_pub_target_gps:
                try: self.zenoh_pub_target_gps.undeclare()
                except Exception: pass
            if self.zenoh_pub_lidar_gps:
                try: self.zenoh_pub_lidar_gps.undeclare()
                except Exception: pass
            if self.zenoh_pub_imu:
                try: self.zenoh_pub_imu.undeclare()
                except Exception: pass
            if self.zenoh_session:
                try: self.zenoh_session.close()
                except Exception: pass

            self.logger.info("CARLA actors and Zenoh connections cleaned up")
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")


# =========================================================
def main(args=None):
    rclpy.init(args=args)
    spawner = None
    try:
        spawner = CarlaVehicleSpawner()
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if spawner is not None:
            spawner.cleanup()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
