#!/usr/bin/env python3
"""
CARLA Vehicle Spawner for ROS2 Bridge (Semantic LiDAR only, rear points removed)
- Keeps original: RGB camera, telemetry, Zenoh vehicle control
- Uses ONLY: Semantic LiDAR (12/14 only, ego-near exclusion by BOX/RADIUS, RViz-friendly PC2)
- Adds: GNSS
- Adds: Cluster centers converted to GPS and published via Zenoh on 'target_gps'
- NEW: Publishes ego vehicle GPS/world pose via Zenoh on 'lidar_gps'
- Removes: regular LiDAR and any related publishers/callbacks
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
from typing import Sequence

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
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False
    print("Warning: cv_bridge not available, using basic image conversion")

from sensor_msgs.msg import (
    CompressedImage, PointCloud2, PointField, NavSatFix, NavSatStatus, Imu
)
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32
from visualization_msgs.msg import Marker, MarkerArray

# -----------------------------
# ID-based clustering support
# -----------------------------
try:
    from id_based_clustering import (
        IDBasedClustering, 
        create_cluster_markers as id_create_cluster_markers, 
        clusters_to_gps_format,
        create_detected_objects_json
    )
    ID_CLUSTERING_AVAILABLE = True
except ImportError:
    ID_CLUSTERING_AVAILABLE = False
    print("Warning: ID-based clustering not available, falling back to DBSCAN")

# -----------------------------
# Optional DBSCAN clustering support (fallback)
# -----------------------------
try:
    from sklearn.cluster import DBSCAN
    SKLEARN_AVAILABLE = True
except Exception:
    SKLEARN_AVAILABLE = False

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

# Semantic LiDAR downsample stride (1: no downsample)
DOWNSAMPLE_STRIDE = int(os.environ.get("SEM_LIDAR_STRIDE", "1"))

# Include only these semantic tags: 12 (Pedestrian), 14 (Car)
_env_include = os.environ.get("INCLUDE_TAGS", "")
if _env_include.strip():
    INCLUDE_TAGS = {int(x.strip()) for x in _env_include.split(",") if x.strip().isdigit()}
else:
    INCLUDE_TAGS = {12, 14}

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

# DBSCAN params
CLUSTER_EPS = 1.2
CLUSTER_MIN_SAMPLES = 10

# Names
OBJECT_NAMES = {12: "Pedestrian", 14: "Car"}


def pack_rgb_uint8_to_float(r_u8, g_u8, b_u8) -> float:
    """RViz 호환: RGB를 24bit로 묶어 float32 비트패턴으로 저장"""
    import struct as _st
    rgb_uint32 = (int(r_u8) << 16) | (int(g_u8) << 8) | int(b_u8)
    return _st.unpack('<f', _st.pack('<I', rgb_uint32))[0]


# =========================================================
# PointCloud2 helpers (generic)
# =========================================================
def _get_point_fields(fields_spec: Sequence[tuple]) -> Sequence[PointField]:
    """fields_spec: [(name, datatype, count), ...]"""
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
    """
    Create PointCloud2 from structured points (dict-like rows).
    fields_spec order must match points fields.
    """
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
def calculate_bounding_box(points: np.ndarray):
    if len(points) == 0:
        return np.array([0, 0, 0]), np.array([0, 0, 0])
    min_vals = np.min(points, axis=0)
    max_vals = np.max(points, axis=0)
    center = (min_vals + max_vals) / 2.0
    dimensions = max_vals - min_vals
    return center, dimensions


def create_cluster_markers(header, clusters, cluster_labels):
    marker_array = MarkerArray()
    for i, (cluster_points, semantic_label) in enumerate(zip(clusters, cluster_labels)):
        if len(cluster_points) == 0:
            continue

        center, dimensions = calculate_bounding_box(cluster_points)

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

        if int(semantic_label) in PALETTE:
            color = PALETTE[int(semantic_label)] / 255.0
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])
            marker.color.a = 0.6

        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0
        marker_array.markers.append(marker)

        # text
        text_marker = Marker()
        text_marker.header = header
        text_marker.id = i + 1000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(center[0])
        text_marker.pose.position.y = float(center[1])
        text_marker.pose.position.z = float(center[2] + dimensions[2] / 2 + 0.5)
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.5
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        name = OBJECT_NAMES.get(int(semantic_label), f"Unknown_{int(semantic_label)}")
        text_marker.text = f"{name}\n({len(cluster_points)} pts)"
        text_marker.lifetime.sec = 1
        text_marker.lifetime.nanosec = 0
        marker_array.markers.append(text_marker)

    return marker_array


# =========================================================
# CarlaVehicleSpawner
# =========================================================
class CarlaVehicleSpawner(Node):
    def __init__(self):
        node_name = f'carla_vehicle_spawner_{int(time.time())}'
        super().__init__(node_name)

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # ---- Configs from original code ----
        self.carla_host = os.environ.get("CARLA_HOST", "localhost")
        self.carla_port = int(os.environ.get("CARLA_PORT", "2000"))
        self.vehicle_blueprint_filter = 'vehicle.dodge.charger_police_2020'
        self.spawn_transform = carla.Transform(
            carla.Location(x=-104.15, y=44.15 + 7.0, z=1.0),
            carla.Rotation(pitch=0.0, yaw=270.0, roll=0.0)
        )

        # ---- State ----
        self.client = None
        self.world = None
        self.vehicle = None
        self.sensors = {}

        # latest GNSS
        self.latest_gnss = {"lat": None, "lon": None, "alt": None}
        
        # ---- ID-based clustering ----
        if ID_CLUSTERING_AVAILABLE:
            self.id_clustering = IDBasedClustering(min_points_per_cluster=1)
            self.logger.info("✅ ID-based clustering enabled")
        else:
            self.id_clustering = None
            self.logger.warning("⚠️ ID-based clustering disabled, using DBSCAN fallback")

        # ---- Buffers for semantic lidar ----
        self.point_buffer = []
        self.label_buffer = []
        self.buffer_frames = 3
        self.current_frame = 0
        self.last_publish_time = 0.0
        self.publish_interval = 0.3  # sec

        # ---- ROS2 publishers (original + added) ----
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
        self.setup_publishers()

        # ---- Zenoh control (original) ----
        self.zenoh_session = None
        self.zenoh_subscriber = None
        self.zenoh_pub_target_gps = None   # target clusters
        self.zenoh_pub_lidar_gps = None    # NEW: ego vehicle pose
        self.target_vehicle_id = None
        self.last_control_command = None
        self.control_command_lock = threading.Lock()
        if ZENOH_AVAILABLE:
            self.setup_zenoh()

        # ---- Connect & spawn (original base) ----
        self.connect_to_carla()
        self.spawn_vehicle()

        # ---- Setup sensors (camera + semantic lidar + gnss) ----
        self.setup_sensors()

        self.logger.info("CARLA Vehicle Spawner initialized successfully")

    # --------------------------
    # Publishers & timers
    # --------------------------
    def setup_publishers(self):
        # Original topics (kept)
        self.camera_pub = self.create_publisher(CompressedImage, '/carla/hero/camera/image/compressed', 2000)
        self.telemetry_pub = self.create_publisher(String, '/carla/hero/vehicle_status', 10)
        self.police_id_pub = self.create_publisher(Int32, '/carla/hero/vehicle_name', 10)
        self.odometry_pub = self.create_publisher(PoseStamped, '/carla/hero/odometry', 10)

        # Semantic LiDAR only
        self.semantic_lidar_pub = self.create_publisher(PointCloud2, '/carla/hero/semantic_lidar/point_cloud', 10)
        self.cluster_markers_pub = self.create_publisher(MarkerArray, '/carla/hero/cluster_markers', 10)
        self.detected_objects_pub = self.create_publisher(String, '/carla/hero/detected_objects', 10)

        # GNSS
        self.gnss_pub = self.create_publisher(NavSatFix, '/carla/hero/gnss', 10)

        # IMU
        self.imu_pub = self.create_publisher(Imu, '/carla/hero/imu', 10)

        # Compatibility ID topic
        self.vehicle_id_pub = self.create_publisher(Int32, '/carla/hero/vehicle_id', 10)

        # Timers
        self.timer = self.create_timer(0.1, self.publish_telemetry)   # 10 Hz
        self.control_timer = self.create_timer(0.05, self.apply_control_commands)  # 20 Hz

    # --------------------------
    # CARLA connect
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

    # --------------------------
    # Vehicle spawn/cleanup
    # --------------------------
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

    # --------------------------
    # Sensors
    # --------------------------
    def setup_sensors(self):
        try:
            bp_lib = self.world.get_blueprint_library()

            # --- Camera (original) ---
            camera_bp = bp_lib.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '1080')
            camera_bp.set_attribute('image_size_y', '720')
            camera_bp.set_attribute('fov', '90')
            camera_tf = carla.Transform(carla.Location(x=1.5, z=1.6))
            self.sensors['camera'] = self.world.spawn_actor(camera_bp, camera_tf, attach_to=self.vehicle)
            self.sensors['camera'].listen(self.camera_callback)

            # --- Semantic LiDAR (only) ---
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

            # --- GNSS ---
            gnss_bp = bp_lib.find('sensor.other.gnss')
            gnss_tf = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.0))
            self.sensors['gnss'] = self.world.spawn_actor(gnss_bp, gnss_tf, attach_to=self.vehicle)
            self.sensors['gnss'].listen(self.gps_callback)

            # --- IMU ---
            imu_bp = bp_lib.find('sensor.other.imu')
            imu_tf = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
            self.sensors['imu'] = self.world.spawn_actor(imu_bp, imu_tf, attach_to=self.vehicle)
            self.sensors['imu'].listen(self.imu_callback)

            self.logger.info("Sensors setup completed (camera, semantic_lidar, gnss, imu)")
        except Exception as e:
            self.logger.error(f"Failed to setup sensors: {e}")

    # --------------------------
    # Callbacks
    # --------------------------
    def camera_callback(self, image):
        try:
            if not rclpy.ok():
                return
            array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
            array = array[:, :, :3]  # BGRA -> BGR
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

    # ---------- EGO EXCLUSION (REAR CUT + BOX/RADIUS) ----------
    def _apply_ego_exclusion(self, lidar_struct: np.ndarray) -> np.ndarray:
        """
        lidar_struct dtype: [('x','f4'),('y','f4'),('z','f4'),('CosAngle','f4'),('ObjIdx','u4'),('ObjTag','u4')]
        Apply:
          1) rear cut (x >= REAR_X_MIN, if enabled)
          2) axis-aligned box exclusion around sensor origin (if enabled)
          3) spherical near-radius exclusion (if > 0)
        """
        arr = lidar_struct
        if arr.size == 0:
            return arr

        # 1) rear cut
        if REMOVE_REAR_POINTS:
            arr = arr[arr['x'] >= REAR_X_MIN]
            if arr.size == 0:
                return arr

        # 2) AABB around sensor
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

        # 3) Spherical near-radius
        if EGO_EXCLUDE_RADIUS > 0.0 and arr.size > 0:
            r2 = EGO_EXCLUDE_RADIUS * EGO_EXCLUDE_RADIUS
            dist2 = arr['x'] * arr['x'] + arr['y'] * arr['y'] + arr['z'] * arr['z']
            arr = arr[dist2 >= r2]

        return arr

    def _clusters_to_gps(self, header, clusters, cluster_labels):
        """
        Convert each cluster's center from LiDAR(ROS) frame to:
        - World XYZ (UE world)
        - GeoLocation (lat, lon, alt) if available
        Returns list of dicts ready to publish over Zenoh.
        """
        results = []
        try:
            # Sensor transform in UE world
            sensor_tf: carla.Transform = self.sensors['semantic_lidar'].get_transform()
            world_map = self.world.get_map() if self.world else None
        except Exception:
            sensor_tf = None
            world_map = None

        ts = header.stamp.sec + header.stamp.nanosec * 1e-9

        for i, (cluster_points, sem) in enumerate(zip(clusters, cluster_labels)):
            if len(cluster_points) == 0:
                continue
            center_ros, dims = calculate_bounding_box(cluster_points)  # ROS-like (y flipped)
            # Convert back to UE sensor frame (undo ROS y flip)
            center_ue = carla.Location(x=float(center_ros[0]),
                                       y=float(-center_ros[1]),
                                       z=float(center_ros[2]))
            # Sensor -> World
            if sensor_tf is not None:
                world_loc: carla.Location = sensor_tf.transform(center_ue)
                world_xyz = {"x": world_loc.x, "y": world_loc.y, "z": world_loc.z}
            else:
                world_xyz = {"x": float(center_ros[0]), "y": float(center_ros[1]), "z": float(center_ros[2])}

            # World -> Geo (if API available)
            lat = lon = alt = None
            try:
                world_map = self.world.get_map() if self.world else None
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
                "world": world_xyz,           # always include world XYZ as fallback
                "lidar_center_ros": {         # debugging/reference
                    "x": float(center_ros[0]), "y": float(center_ros[1]), "z": float(center_ros[2])
                },
                "dimensions": {"w": float(dims[0]), "h": float(dims[1]), "d": float(dims[2])},
                "timestamp": ts
            })
        return results

    def _publish_target_gps_zenoh(self, payload_obj: dict):
        """Publish JSON payload to Zenoh key 'target_gps'."""
        if not (ZENOH_AVAILABLE and self.zenoh_session and self.zenoh_pub_target_gps):
            return
        try:
            self.zenoh_pub_target_gps.put(json.dumps(payload_obj))
        except Exception as e:
            self.logger.error(f"Error publishing target_gps via Zenoh: {e}")

    def _publish_lidar_gps_zenoh(self, payload_obj: dict):
        """Publish ego vehicle pose to Zenoh key 'lidar_gps'."""
        if not (ZENOH_AVAILABLE and self.zenoh_session and self.zenoh_pub_lidar_gps):
            return
        try:
            self.zenoh_pub_lidar_gps.put(json.dumps(payload_obj))
        except Exception as e:
            self.logger.error(f"Error publishing lidar_gps via Zenoh: {e}")

    def semantic_lidar_callback(self, semantic_lidar_data):
        """Publish only tags 12/14, ego-near exclusion (rear-cut + BOX/RADIUS), RViz-friendly PC2, clustering & markers/JSON, and target_gps via Zenoh."""
        try:
            from std_msgs.msg import Header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"  # RViz fixed frame

            # PC2 fields: x,y,z,rgb(float32 packed), ObjTag(uint32)
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

            # UE → ROS axis fix
            lidar['y'] *= -1.0

            # Keep only requested tags (12/14)
            if INCLUDE_TAGS:
                lidar = lidar[np.isin(lidar['ObjTag'], list(INCLUDE_TAGS))]

            # ---- EGO REMOVAL: rear cut + box + radius ----
            lidar = self._apply_ego_exclusion(lidar)

            # Downsample for performance
            if DOWNSAMPLE_STRIDE > 1 and len(lidar) > 0:
                lidar = lidar[::DOWNSAMPLE_STRIDE]

            # Buffering (accumulate a few frames)
            if len(lidar) > 0:
                pts = np.stack([lidar['x'], lidar['y'], lidar['z']], axis=-1)
                tags = lidar['ObjTag'].astype(np.uint32)
                self.point_buffer.append(pts)
                self.label_buffer.append(tags)
                self.current_frame += 1

            now = time.time()
            should_publish = (len(self.point_buffer) >= self.buffer_frames) or ((now - self.last_publish_time) >= self.publish_interval)

            if should_publish and len(self.point_buffer) > 0:
                all_points = np.vstack(self.point_buffer)
                all_labels = np.hstack(self.label_buffer)

                # Colors by tag (12/14) → packed float
                tags_arr = all_labels.astype(np.int32)
                colors_u8 = np.empty((len(all_points), 3), dtype=np.uint8)
                mask12 = (tags_arr == 12)
                mask14 = (tags_arr == 14)
                if mask12.any():
                    colors_u8[mask12] = PALETTE[12]
                if mask14.any():
                    colors_u8[mask14] = PALETTE[14]
                rgb_f32 = [pack_rgb_uint8_to_float(r, g, b) for r, g, b in colors_u8]

                # Build tuples for pack helper
                points_list = [
                    (float(all_points[i, 0]),
                     float(all_points[i, 1]),
                     float(all_points[i, 2]),
                     float(rgb_f32[i]),
                     int(tags_arr[i]))
                    for i in range(len(all_points))
                ]

                # Pack to PointCloud2
                cloud = self._create_cloud_rviz(header, fields, points_list)
                self.semantic_lidar_pub.publish(cloud)

                # --- ID-based Clustering & markers/JSON ---
                clusters = []
                cluster_labels = []
                cluster_ids = []
                
                if ID_CLUSTERING_AVAILABLE and self.id_clustering and len(lidar) > 0:
                    try:
                        # Use original lidar data with ObjIdx for ID-based clustering
                        clusters, cluster_ids, cluster_labels = self.id_clustering.cluster_points(lidar)
                        if clusters:
                            # Use ID-based clustering markers (4 arguments)
                            markers = id_create_cluster_markers(header, clusters, cluster_ids, cluster_labels)
                            self.cluster_markers_pub.publish(markers)
                            
                            # Create detected objects JSON
                            detected_json = create_detected_objects_json(header, clusters, cluster_ids, cluster_labels)
                            msg = String()
                            msg.data = json.dumps(detected_json)
                            self.detected_objects_pub.publish(msg)
                            
                            self.logger.info(f"🎯 ID-based clustering detected {len(clusters)} objects:")
                            for i, (obj_id, semantic_label) in enumerate(zip(cluster_ids, cluster_labels)):
                                object_name = OBJECT_NAMES.get(int(semantic_label), f"Unknown_{int(semantic_label)}")
                                self.logger.info(f"   Object {i}: ID={obj_id}, Type={object_name}, Points={len(clusters[i])}")
                    except Exception as e:
                        self.logger.error(f"❌ Error in ID-based clustering: {e}")
                        # Fallback to DBSCAN
                        clusters, cluster_labels = self._cluster_points(all_points, all_labels)
                        if clusters:
                            markers = create_cluster_markers(header, clusters, cluster_labels)
                            self.cluster_markers_pub.publish(markers)
                            self._publish_detected_objects_json(header, clusters, cluster_labels)
                
                elif SKLEARN_AVAILABLE and len(all_points) >= CLUSTER_MIN_SAMPLES:
                    # Fallback to DBSCAN
                    try:
                        clusters, cluster_labels = self._cluster_points(all_points, all_labels)
                        if clusters:
                            markers = create_cluster_markers(header, clusters, cluster_labels)
                            self.cluster_markers_pub.publish(markers)
                            self._publish_detected_objects_json(header, clusters, cluster_labels)
                            self.logger.info(f"🔍 DBSCAN fallback detected {len(clusters)} objects: "
                                             f"{[OBJECT_NAMES.get(int(l), f'Unknown_{int(l)}') for l in cluster_labels]}")
                    except Exception as e:
                        self.logger.error(f"❌ Error in DBSCAN fallback: {e}")

                # --- Publish GPS centers via Zenoh (target_gps) ---
                try:
                    if ID_CLUSTERING_AVAILABLE and self.id_clustering and clusters:
                        # Use ID-based clustering GPS conversion
                        sensor_transform = self.sensors['semantic_lidar'].get_transform() if self.sensors.get('semantic_lidar') else None
                        world_map = self.world.get_map() if self.world else None
                        gps_targets = clusters_to_gps_format(
                            header, clusters, cluster_ids, cluster_labels, 
                            sensor_transform, world_map
                        )
                    else:
                        # Fallback to original GPS conversion
                        gps_targets = self._clusters_to_gps(header, clusters, cluster_labels) if clusters else []
                    
                    payload = {
                        "timestamp": header.stamp.sec + header.stamp.nanosec * 1e-9,
                        "vehicle_id": int(self.vehicle.id) if self.vehicle else None,
                        "targets": gps_targets
                    }
                    self._publish_target_gps_zenoh(payload)
                except Exception as e:
                    self.logger.error(f"❌ Error preparing target_gps payload: {e}")

                # Optional socket streaming (best-effort)
                try:
                    payload_pts = {'points': all_points, 'labels': all_labels}
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(0.01)
                    s.connect(('localhost', 50051))
                    s.sendall(pickle.dumps(payload_pts) + b'END')
                    s.close()
                except Exception:
                    pass

                # Reset buffers
                self.point_buffer = []
                self.label_buffer = []
                self.current_frame = 0
                self.last_publish_time = now

            elif len(lidar) == 0:
                # Publish empty cloud (keeps topic alive)
                empty_cloud = self._create_cloud_rviz(header, fields, [])
                self.semantic_lidar_pub.publish(empty_cloud)

        except Exception as e:
            self.logger.error(f"Error in semantic LiDAR callback: {e}")

    def gps_callback(self, gps):
        """Publish GNSS as NavSatFix and cache latest for Zenoh ego message."""
        try:
            # ROS publish
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

            # cache latest gnss
            self.latest_gnss = {"lat": float(gps.latitude), "lon": float(gps.longitude), "alt": float(gps.altitude)}
        except Exception as e:
            self.logger.error(f"Error in GNSS callback: {e}")

    def imu_callback(self, imu_data):
        """Publish IMU data as sensor_msgs/Imu."""
        try:
            if not rclpy.ok():
                return
                
            # Create IMU message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu"
            
            # Linear acceleration (m/s^2)
            msg.linear_acceleration.x = float(imu_data.accelerometer.x)
            msg.linear_acceleration.y = float(imu_data.accelerometer.y)
            msg.linear_acceleration.z = float(imu_data.accelerometer.z)
            
            # Angular velocity (rad/s)
            msg.angular_velocity.x = float(imu_data.gyroscope.x)
            msg.angular_velocity.y = float(imu_data.gyroscope.y)
            msg.angular_velocity.z = float(imu_data.gyroscope.z)
            
            # Orientation (quaternion)
            # CARLA provides orientation as rotation, convert to quaternion
            orientation = imu_data.transform.rotation
            # Convert CARLA rotation to quaternion
            import math
            cy = math.cos(orientation.yaw * 0.5)
            sy = math.sin(orientation.yaw * 0.5)
            cp = math.cos(orientation.pitch * 0.5)
            sp = math.sin(orientation.pitch * 0.5)
            cr = math.cos(orientation.roll * 0.5)
            sr = math.sin(orientation.roll * 0.5)
            
            msg.orientation.w = cr * cp * cy + sr * sp * sy
            msg.orientation.x = sr * cp * cy - cr * sp * sy
            msg.orientation.y = cr * sp * cy + sr * cp * sy
            msg.orientation.z = cr * cp * sy - sr * sp * cy
            
            # Covariance matrices (unknown for now)
            msg.linear_acceleration_covariance = [0.0] * 9
            msg.angular_velocity_covariance = [0.0] * 9
            msg.orientation_covariance = [0.0] * 9
            
            # Publish
            self.imu_pub.publish(msg)
            
        except Exception as e:
            self.logger.error(f"Error in IMU callback: {e}")

    # --------------------------
    # Telemetry (original) + lidar_gps publish
    # --------------------------
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
            msg = String()
            msg.data = json.dumps(telemetry)
            self.telemetry_pub.publish(msg)

            # Original + compatibility ID topics
            id_msg = Int32(); id_msg.data = self.vehicle.id
            self.police_id_pub.publish(id_msg)
            self.vehicle_id_pub.publish(id_msg)

            odom = PoseStamped()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "map"
            odom.pose.position.x = tf.location.x
            odom.pose.position.y = tf.location.y
            odom.pose.position.z = tf.location.z
            self.odometry_pub.publish(odom)

            # ----- NEW: publish ego GPS/world to zenoh 'lidar_gps' -----
            try:
                # prefer sensor GNSS; fallback to CARLA geolocation
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
                        "gps": gps_dict,  # may be None if unavailable
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
    # Zenoh control (original) + publishers
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

            self.zenoh_session = zenoh.open(zenoh_config)
            self.logger.info("✅ Connected to Zenoh router successfully")

            # Control subscriber (kept)
            self.zenoh_subscriber = self.zenoh_session.declare_subscriber(
                "carla/vehicle/control", self.on_control_command
            )
            self.logger.info("🎮 Subscribed to vehicle control commands via Zenoh")

            # Publishers
            self.zenoh_pub_target_gps = self.zenoh_session.declare_publisher("target_gps")
            self.logger.info("🛰️ Publishing target GPS to 'target_gps'")
            self.zenoh_pub_lidar_gps = self.zenoh_session.declare_publisher("lidar_gps")
            self.logger.info("📍 Publishing ego GPS to 'lidar_gps'")
        except Exception as e:
            self.logger.error(f"Failed to setup Zenoh: {e}")
            self.zenoh_session = None
            self.zenoh_subscriber = None
            self.zenoh_pub_target_gps = None
            self.zenoh_pub_lidar_gps = None

    def on_control_command(self, sample):
        try:
            control_data = json.loads(sample.payload.decode('utf-8'))
            target_id = control_data.get('target_vehicle_id')
            if target_id and self.vehicle and target_id == self.vehicle.id:
                with self.control_command_lock:
                    self.last_control_command = control_data
        except Exception as e:
            self.logger.error(f"Error processing control command: {e}")

    def apply_control_commands(self):
        try:
            if not self.vehicle or not self.last_control_command:
                return
            with self.control_command_lock:
                if not self.last_control_command:
                    return
                data = self.last_control_command
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

    # --------------------------
    # Helpers (added)
    # --------------------------
    def _cluster_points(self, points: np.ndarray, labels: np.ndarray):
        """DBSCAN clustering by geometry, dominant semantic per cluster."""
        if not SKLEARN_AVAILABLE or len(points) < CLUSTER_MIN_SAMPLES:
            return [], []
        clustering = DBSCAN(eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES).fit(points)
        cluster_labels = clustering.labels_
        valid_mask = cluster_labels != -1
        if not valid_mask.any():
            return [], []
        unique_clusters = np.unique(cluster_labels[valid_mask])
        clusters = []
        cluster_semantics = []
        for cid in unique_clusters:
            mask = cluster_labels == cid
            pts = points[mask]
            sems = labels[mask]
            if len(sems) == 0:
                continue
            uniq, counts = np.unique(sems, return_counts=True)
            dom = uniq[np.argmax(counts)]
            clusters.append(pts)
            cluster_semantics.append(int(dom))
        return clusters, cluster_semantics

    def _publish_detected_objects_json(self, header, clusters, cluster_labels):
        detected = []
        for i, (pts, sem) in enumerate(zip(clusters, cluster_labels)):
            if len(pts) == 0:
                continue
            center, dims = calculate_bounding_box(pts)
            detected.append({
                "id": i,
                "name": OBJECT_NAMES.get(int(sem), f"Unknown_{int(sem)}"),
                "semantic_label": int(sem),
                "position": {"x": float(center[0]), "y": float(center[1]), "z": float(center[2])},
                "dimensions": {"width": float(dims[0]), "height": float(dims[1]), "depth": float(dims[2])},
                "point_count": int(len(pts)),
                "timestamp": header.stamp.sec + header.stamp.nanosec * 1e-9
            })
        msg = String()
        msg.data = json.dumps({
            "detected_objects": detected,
            "total_objects": len(detected),
            "timestamp": header.stamp.sec + header.stamp.nanosec * 1e-9
        })
        self.detected_objects_pub.publish(msg)

    def _create_cloud_rviz(self, header, fields, points_list):
        """
        RViz-friendly cloud with (x,y,z,rgb(float32), ObjTag(uint32)).
        points_list: [(x,y,z,rgb_f32,tag), ...]
        """
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

    # --------------------------
    # Cleanup
    # --------------------------
    def cleanup(self):
        try:
            # Sensors
            if self.sensors:
                for s in self.sensors.values():
                    try:
                        if s is not None:
                            s.destroy()
                    except Exception:
                        pass
            # Vehicle
            if self.vehicle is not None:
                try: self.vehicle.destroy()
                except Exception: pass

            # Zenoh
            if self.zenoh_subscriber:
                try: self.zenoh_subscriber.undeclare()
                except Exception: pass
            if self.zenoh_pub_target_gps:
                try: self.zenoh_pub_target_gps.undeclare()
                except Exception: pass
            if self.zenoh_pub_lidar_gps:
                try: self.zenoh_pub_lidar_gps.undeclare()
                except Exception: pass
            if self.zenoh_session:
                try: self.zenoh_session.close()
                except Exception: pass

            self.logger.info("CARLA actors and Zenoh connections cleaned up")
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")


# =========================================================
# main
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
