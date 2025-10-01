#!/usr/bin/env python3
"""
CARLA Vehicle Spawner for ROS2 Bridge
- Sensors: Semantic LiDAR + GPS only
- RViz-friendly PointCloud2 (x,y,z,rgb,ObjTag) with packed rgb
- Publish ONLY Pedestrian(12) and Car(14)
- EXCLUDE ego points by NEAR-RANGE ONLY (no ObjIdx filtering)
- Spawns a standing pedestrian near the vehicle
- Spawns a stationary vehicle (same model as hero) in front of the hero vehicle
"""

import glob
import os
import sys
import time
import json
import struct
import socket
import pickle
import logging
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN

# -------------------------------------------------------
# Add CARLA Python API to path (fallbacks)
# -------------------------------------------------------
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

import carla  # noqa: E402

# -------------------------------------------------------
# Configs
# -------------------------------------------------------
CARLA_HOST = os.environ.get("CARLA_HOST", "localhost")
CARLA_PORT = int(os.environ.get("CARLA_PORT", "2000"))

# 다운샘플 stride (포인트 많으면 2~3 추천)
DOWNSAMPLE_STRIDE = int(os.environ.get("SEM_LIDAR_STRIDE", "1"))  # 1이면 다운샘플 안 함

# 포함할 태그(보행자 12, 차량 14만)
_env_include = os.environ.get("INCLUDE_TAGS", "")
if _env_include.strip():
    INCLUDE_TAGS = {int(x.strip()) for x in _env_include.split(",") if x.strip().isdigit()}
else:
    INCLUDE_TAGS = {12, 14}

# ⭕ 자차 제거 반경(m): 센서 원점 기준. 작게 잡으면 자차만 지워지고 타차는 남음.
EGO_EXCLUDE_RADIUS = float(os.environ.get("EGO_EXCLUDE_RADIUS", "2.5"))

# 팔레트(두 클래스만 사용)
PALETTE = {
    12: np.array([220, 20, 60], dtype=np.uint8),   # Pedestrian
    14: np.array([0, 0, 142], dtype=np.uint8),     # Car
}

# 클러스터링 파라미터
CLUSTER_EPS = 1.2  # DBSCAN eps 파라미터
CLUSTER_MIN_SAMPLES = 50  # 최소 샘플 수

# 객체 이름 매핑
OBJECT_NAMES = {
    12: "Pedestrian",
    14: "Car"
}

def pack_rgb_uint8_to_float(r_u8, g_u8, b_u8) -> float:
    """RViz 호환: RGB를 24bit로 묶어 float32 비트패턴으로 저장"""
    rgb_uint32 = (int(r_u8) << 16) | (int(g_u8) << 8) | int(b_u8)
    return struct.unpack('<f', struct.pack('<I', rgb_uint32))[0]


def cluster_points(points, labels, eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES):
    """
    포인트 클라우드를 클러스터링하여 객체를 분리
    Args:
        points: (N, 3) 포인트 좌표
        labels: (N,) 시맨틱 라벨
        eps: DBSCAN eps 파라미터
        min_samples: 최소 샘플 수
    Returns:
        clusters: 클러스터별 포인트 인덱스 리스트
        cluster_labels: 클러스터별 시맨틱 라벨
    """
    if len(points) < min_samples:
        return [], []
    
    # DBSCAN 클러스터링
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    cluster_labels = clustering.labels_
    
    # 노이즈 제거 (라벨이 -1인 포인트들)
    valid_mask = cluster_labels != -1
    if not valid_mask.any():
        return [], []
    
    # 클러스터별로 포인트 그룹화
    unique_clusters = np.unique(cluster_labels[valid_mask])
    clusters = []
    cluster_semantic_labels = []
    
    for cluster_id in unique_clusters:
        cluster_mask = cluster_labels == cluster_id
        cluster_points = points[cluster_mask]
        cluster_semantic = labels[cluster_mask]
        
        # 가장 많은 시맨틱 라벨을 클러스터 라벨로 사용
        unique_semantic, counts = np.unique(cluster_semantic, return_counts=True)
        dominant_semantic = unique_semantic[np.argmax(counts)]
        
        clusters.append(cluster_points)
        cluster_semantic_labels.append(dominant_semantic)
    
    return clusters, cluster_semantic_labels


def calculate_bounding_box(points):
    """
    포인트 클라우드의 바운딩 박스 계산
    Args:
        points: (N, 3) 포인트 좌표
    Returns:
        center: 중심점 (x, y, z)
        dimensions: 크기 (width, height, depth)
    """
    if len(points) == 0:
        return np.array([0, 0, 0]), np.array([0, 0, 0])
    
    min_vals = np.min(points, axis=0)
    max_vals = np.max(points, axis=0)
    
    center = (min_vals + max_vals) / 2.0
    dimensions = max_vals - min_vals
    
    return center, dimensions


class CarlaVehicleSpawner(Node):
    def __init__(self):
        node_name = f'carla_vehicle_spawner_{int(time.time())}'
        super().__init__(node_name)

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger("CarlaVehicleSpawner")

        self.client = None
        self.world = None
        self.vehicle = None
        self.pedestrian = None
        self.blocking_vehicle = None
        self.sensors = {}

        # 포인트 클라우드 누적을 위한 버퍼
        self.point_buffer = []
        self.label_buffer = []
        self.buffer_frames = 3  # 3프레임 누적
        self.current_frame = 0
        self.last_publish_time = 0
        self.publish_interval = 0.3  # 0.3초마다 퍼블리시 (RViz Decay Time과 동일)

        # ROS2 pubs
        self.semantic_lidar_pub = self.create_publisher(PointCloud2, '/carla/hero/semantic_lidar/point_cloud', 10)
        self.telemetry_pub = self.create_publisher(String, '/carla/hero/vehicle_status', 10)
        self.vehicle_id_pub = self.create_publisher(Int32, '/carla/hero/vehicle_id', 10)
        self.odometry_pub = self.create_publisher(PoseStamped, '/carla/hero/odometry', 10)
        
        # 클러스터링 및 객체 감지 관련 퍼블리셔
        self.cluster_markers_pub = self.create_publisher(MarkerArray, '/carla/hero/cluster_markers', 10)
        self.detected_objects_pub = self.create_publisher(String, '/carla/hero/detected_objects', 10)

        self.timer = self.create_timer(0.1, self.publish_telemetry)  # 10 Hz

        # Connect & setup
        self.connect_to_carla()
        self.spawn_vehicle()
        self.spawn_pedestrian_near_vehicle()
        self.spawn_blocking_vehicle_in_front()  # 동일 모델 정지 차량
        self.setup_sensors()

        self.logger.info("✅ CARLA Vehicle Spawner initialized successfully")

    # -----------------------------
    # CARLA connect & spawn
    # -----------------------------
    def connect_to_carla(self):
        self.client = carla.Client(CARLA_HOST, CARLA_PORT)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.logger.info(f"Connected to CARLA at {CARLA_HOST}:{CARLA_PORT}")

    def spawn_vehicle(self):
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.*')[0]
        if vehicle_bp.has_attribute("role_name"):
            vehicle_bp.set_attribute("role_name", "hero")
        spawn_point = self.world.get_map().get_spawn_points()[0]
        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        self.logger.info(f"Spawned vehicle id={self.vehicle.id} ({self.vehicle.type_id}) at {spawn_point.location}")

    def spawn_pedestrian_near_vehicle(self, forward_offset=5.0, right_offset=3.0):
        """차량 스폰 지점 기준 앞 5m, 우측 3m에 '서 있는' 보행자 스폰"""
        try:
            if self.vehicle is None:
                self.logger.warning("Vehicle not spawned yet; skipping pedestrian spawn.")
                return

            bp_lib = self.world.get_blueprint_library()
            walker_bps = bp_lib.filter('walker.pedestrian.*')
            if not walker_bps:
                self.logger.warning("No pedestrian blueprints found.")
                return

            walker_bp = np.random.choice(walker_bps)
            if walker_bp.has_attribute('role_name'):
                walker_bp.set_attribute('role_name', 'static_pedestrian')

            v_tf: carla.Transform = self.vehicle.get_transform()
            yaw_deg = v_tf.rotation.yaw
            yaw = math.radians(yaw_deg)

            forward = np.array([math.cos(yaw), math.sin(yaw)])     # x-forward, y-right
            right = np.array([-math.sin(yaw), math.cos(yaw)])

            base = np.array([v_tf.location.x, v_tf.location.y], dtype=float)
            pos_xy = base + forward_offset * forward + right_offset * right

            ped_loc = carla.Location(x=float(pos_xy[0]), y=float(pos_xy[1]), z=v_tf.location.z)
            ped_rot = carla.Rotation(pitch=0.0, yaw=(yaw_deg + 180.0) % 360.0, roll=0.0)
            ped_tf = carla.Transform(ped_loc, ped_rot)

            self.pedestrian = self.world.try_spawn_actor(walker_bp, ped_tf)
            if self.pedestrian is None:
                for i in range(1, 4):
                    jitter = 0.5 * i
                    alt_loc = carla.Location(
                        x=float(pos_xy[0] + jitter),
                        y=float(pos_xy[1] - jitter),
                        z=v_tf.location.z
                    )
                    self.pedestrian = self.world.try_spawn_actor(walker_bp, carla.Transform(alt_loc, ped_rot))
                    if self.pedestrian:
                        break

            if self.pedestrian:
                self.pedestrian.set_simulate_physics(True)
                self.logger.info(f"Spawned standing pedestrian id={self.pedestrian.id} at {self.pedestrian.get_transform().location}")
            else:
                self.logger.warning("Failed to spawn pedestrian near vehicle.")
        except Exception as e:
            self.logger.error(f"Error spawning pedestrian: {e}")

    def spawn_blocking_vehicle_in_front(self, distance_ahead: float = 12.0):
        """
        내 차량 앞 차선 위 지정 거리 지점에 '정지 차량' 스폰
        - 히어로 차량과 동일한 blueprint(type_id) 사용
        - 가능한 경우 색상(color)도 동일하게 설정
        """
        try:
            if self.vehicle is None:
                self.logger.warning("Vehicle not spawned yet; skipping blocking vehicle spawn.")
                return

            world_map = self.world.get_map()
            hero_loc = self.vehicle.get_location()
            hero_wp = world_map.get_waypoint(hero_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            if hero_wp is None:
                self.logger.warning("Failed to get hero waypoint; cannot spawn blocking vehicle.")
                return

            next_wps = hero_wp.next(distance_ahead)
            if not next_wps:
                self.logger.warning("No waypoint found ahead; cannot spawn blocking vehicle.")
                return

            target_wp = next_wps[0]
            target_tf = target_wp.transform
            target_tf.location.z += 0.1  # 살짝 띄워 스폰 충돌 방지

            bp_lib = self.world.get_blueprint_library()
            hero_type = self.vehicle.type_id  # e.g., 'vehicle.tesla.model3'
            block_bp = bp_lib.find(hero_type)

            if block_bp.has_attribute("role_name"):
                block_bp.set_attribute("role_name", "blocking")

            hero_color = self.vehicle.attributes.get('color')
            if hero_color and block_bp.has_attribute('color'):
                block_bp.set_attribute('color', hero_color)

            self.blocking_vehicle = self.world.try_spawn_actor(block_bp, target_tf)
            if self.blocking_vehicle is None:
                offsets = [(1,0), (-1,0), (0,1), (0,-1)]
                for dx, dy in offsets:
                    alt_tf = carla.Transform(
                        carla.Location(
                            x=target_tf.location.x + dx,
                            y=target_tf.location.y + dy,
                            z=target_tf.location.z
                        ),
                        target_tf.rotation
                    )
                    self.blocking_vehicle = self.world.try_spawn_actor(block_bp, alt_tf)
                    if self.blocking_vehicle:
                        break

            if self.blocking_vehicle:
                self.blocking_vehicle.set_autopilot(False)
                self.blocking_vehicle.apply_control(
                    carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=True)
                )
                self.logger.info(
                    f"Spawned blocking vehicle id={self.blocking_vehicle.id} "
                    f"({self.blocking_vehicle.type_id}) at {self.blocking_vehicle.get_transform().location}"
                )
            else:
                self.logger.warning("Failed to spawn blocking vehicle in front.")

        except Exception as e:
            self.logger.error(f"Error spawning blocking vehicle: {e}")

    def setup_sensors(self):
        blueprint_library = self.world.get_blueprint_library()

        # Semantic LiDAR
        sem_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')
        sem_bp.set_attribute('channels', '32')
        # sem_bp.set_attribute('points_per_second', '60000')
        sem_bp.set_attribute('points_per_second', '400000')
        sem_bp.set_attribute('rotation_frequency', '10')
        sem_bp.set_attribute('range', '80')
        sem_bp.set_attribute('upper_fov', '0')
        sem_bp.set_attribute('lower_fov', '-45')

        sem_tf = carla.Transform(carla.Location(x=0.0, z=2.5))
        self.sensors['semantic_lidar'] = self.world.spawn_actor(sem_bp, sem_tf, attach_to=self.vehicle)
        self.sensors['semantic_lidar'].listen(self.semantic_lidar_callback)

        # GPS
        gps_bp = blueprint_library.find('sensor.other.gnss')
        gps_tf = sem_tf
        self.sensors['gps'] = self.world.spawn_actor(gps_bp, gps_tf, attach_to=self.vehicle)
        self.sensors['gps'].listen(self.gps_callback)

        self.logger.info("Sensors setup completed (Semantic LiDAR + GPS)")

    # -----------------------------
    # PointCloud2 helpers (RViz-friendly: x,y,z,rgb,ObjTag)
    # -----------------------------
    def _get_struct_fmt(self, is_bigendian, fields):
        fmt = '>' if is_bigendian else '<'
        from sensor_msgs.msg import PointField
        datatypes = {
            PointField.INT8: ('b', 1),
            PointField.UINT8: ('B', 1),
            PointField.INT16: ('h', 2),
            PointField.UINT16: ('H', 2),
            PointField.INT32: ('i', 4),
            PointField.UINT32: ('I', 4),
            PointField.FLOAT32: ('f', 4),
            PointField.FLOAT64: ('d', 8),
        }
        offset = 0
        for field in sorted(fields, key=lambda f: f.offset):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            dt_fmt, dt_len = datatypes[field.datatype]
            fmt += field.count * dt_fmt
            offset += field.count * dt_len
        return fmt

    def create_cloud(self, header, fields, points_list):
        """
        points_list: iterable of tuples (x:float, y:float, z:float, rgb_float32:float, ObjTag:uint32)
        """
        import ctypes
        from sensor_msgs.msg import PointCloud2

        cloud_struct = struct.Struct(self._get_struct_fmt(False, fields))

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
            header=header,
            height=1,
            width=len(points_list),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=cloud_struct.size,
            row_step=cloud_struct.size * len(points_list),
            data=buff.raw
        )

    def create_cluster_markers(self, header, clusters, cluster_labels):
        """
        클러스터들을 RViz 마커로 변환
        """
        marker_array = MarkerArray()
        
        for i, (cluster_points, semantic_label) in enumerate(zip(clusters, cluster_labels)):
            if len(cluster_points) == 0:
                continue
                
            # 바운딩 박스 계산
            center, dimensions = calculate_bounding_box(cluster_points)
            
            # 마커 생성
            marker = Marker()
            marker.header = header
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # 위치 설정
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0
            
            # 크기 설정
            marker.scale.x = float(dimensions[0])
            marker.scale.y = float(dimensions[1])
            marker.scale.z = float(dimensions[2])
            
            # 색상 설정 (시맨틱 라벨에 따라)
            if semantic_label in PALETTE:
                color = PALETTE[semantic_label] / 255.0
                marker.color.r = float(color[0])
                marker.color.g = float(color[1])
                marker.color.b = float(color[2])
                marker.color.a = 0.6  # 투명도
            
            marker.lifetime.sec = 1
            marker.lifetime.nanosec = 0
            
            marker_array.markers.append(marker)
            
            # 텍스트 라벨 마커
            text_marker = Marker()
            text_marker.header = header
            text_marker.id = i + 1000  # 텍스트는 다른 ID 사용
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(center[0])
            text_marker.pose.position.y = float(center[1])
            text_marker.pose.position.z = float(center[2] + dimensions[2]/2 + 0.5)
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.5  # 텍스트 크기
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            object_name = OBJECT_NAMES.get(semantic_label, f"Unknown_{semantic_label}")
            text_marker.text = f"{object_name}\n({len(cluster_points)} pts)"
            
            text_marker.lifetime.sec = 1
            text_marker.lifetime.nanosec = 0
            
            marker_array.markers.append(text_marker)
        
        return marker_array

    def publish_detected_objects(self, header, clusters, cluster_labels):
        """
        감지된 객체 정보를 JSON 형태로 퍼블리시
        """
        detected_objects = []
        
        for i, (cluster_points, semantic_label) in enumerate(zip(clusters, cluster_labels)):
            if len(cluster_points) == 0:
                continue
                
            center, dimensions = calculate_bounding_box(cluster_points)
            object_name = OBJECT_NAMES.get(semantic_label, f"Unknown_{semantic_label}")
            
            obj_info = {
                "id": i,
                "name": object_name,
                "semantic_label": int(semantic_label),
                "position": {
                    "x": float(center[0]),
                    "y": float(center[1]),
                    "z": float(center[2])
                },
                "dimensions": {
                    "width": float(dimensions[0]),
                    "height": float(dimensions[1]),
                    "depth": float(dimensions[2])
                },
                "point_count": len(cluster_points),
                "timestamp": header.stamp.sec + header.stamp.nanosec * 1e-9
            }
            
            detected_objects.append(obj_info)
        
        # JSON 메시지로 퍼블리시
        msg = String()
        msg.data = json.dumps({
            "detected_objects": detected_objects,
            "total_objects": len(detected_objects),
            "timestamp": header.stamp.sec + header.stamp.nanosec * 1e-9
        })
        self.detected_objects_pub.publish(msg)

    # -----------------------------
    # Semantic LiDAR callback
    # -----------------------------
    def semantic_lidar_callback(self, semantic_lidar_data):
        try:
            from std_msgs.msg import Header
            from sensor_msgs.msg import PointField

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"  # RViz Fixed Frame = map

            fields = [
                PointField(name='x',      offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y',      offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z',      offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb',    offset=12, datatype=PointField.FLOAT32, count=1),  # packed rgb
                PointField(name='ObjTag', offset=16, datatype=PointField.UINT32,  count=1),
            ]

            # Raw buffer → structured array
            lidar = np.frombuffer(bytes(semantic_lidar_data.raw_data),
                                  dtype=np.dtype([
                                      ('x', np.float32),
                                      ('y', np.float32),
                                      ('z', np.float32),
                                      ('CosAngle', np.float32),
                                      ('ObjIdx', np.uint32),
                                      ('ObjTag', np.uint32)
                                  ])).copy()

            # 좌표계 보정 (UE y-right → ROS y-left)
            lidar['y'] *= -1.0

            # === 포함 필터: Pedestrian(12), Car(14)만 남김 ===
            tags_full = lidar['ObjTag']
            mask_inc = (tags_full == 12) | (tags_full == 14)
            lidar = lidar[mask_inc]

            # === 자차 근접 반경 제거 (ID 사용 안 함) ===
            # 센서 원점 기준 반경 EGO_EXCLUDE_RADIUS(m) 내 포인트 삭제
            if EGO_EXCLUDE_RADIUS > 0.0 and len(lidar) > 0:
                r2 = EGO_EXCLUDE_RADIUS * EGO_EXCLUDE_RADIUS
                dist2 = lidar['x'] * lidar['x'] + lidar['y'] * lidar['y'] + lidar['z'] * lidar['z']
                lidar = lidar[dist2 >= r2]

            # 다운샘플
            if DOWNSAMPLE_STRIDE > 1 and len(lidar) > 0:
                lidar = lidar[::DOWNSAMPLE_STRIDE]

            # 포인트를 버퍼에 추가
            if len(lidar) > 0:
                points = np.stack([lidar['x'], lidar['y'], lidar['z']], axis=-1)
                labels = lidar['ObjTag'].astype(np.uint32)
                
                self.point_buffer.append(points)
                self.label_buffer.append(labels)
                self.current_frame += 1

            # 시간 기반 또는 프레임 기반으로 퍼블리시
            current_time = time.time()
            should_publish = (
                len(self.point_buffer) >= self.buffer_frames or 
                (current_time - self.last_publish_time) >= self.publish_interval
            )

            if should_publish and len(self.point_buffer) > 0:
                # 모든 포인트를 하나로 합치기
                all_points = np.vstack(self.point_buffer)
                all_labels = np.hstack(self.label_buffer)
                
                # 색상: 두 클래스만 (uint8) → packed rgb float32
                tags = all_labels.astype(np.int32)
                colors_u8 = np.empty((len(all_points), 3), dtype=np.uint8)
                mask12 = (tags == 12)
                mask14 = ~mask12  # 포함 필터상 12 또는 14만 남아있음
                if mask12.any():
                    colors_u8[mask12] = PALETTE[12]
                if mask14.any():
                    colors_u8[mask14] = PALETTE[14]

                rgb_f32_list = [pack_rgb_uint8_to_float(r, g, b) for r, g, b in colors_u8]

                # pack_into용 튜플 리스트
                points_list = [
                    (float(all_points[i, 0]),
                     float(all_points[i, 1]),
                     float(all_points[i, 2]),
                     float(rgb_f32_list[i]),
                     int(tags[i]))
                    for i in range(len(all_points))
                ]

                # 포인트 클라우드 퍼블리시
                pc2 = self.create_cloud(header, fields, points_list)
                self.semantic_lidar_pub.publish(pc2)

                # 클러스터링 및 객체 감지
                try:
                    # 클러스터링 수행
                    clusters, cluster_labels = cluster_points(all_points, all_labels)
                    
                    if clusters:
                        # RViz 마커 생성 및 퍼블리시
                        marker_array = self.create_cluster_markers(header, clusters, cluster_labels)
                        self.cluster_markers_pub.publish(marker_array)
                        
                        # 감지된 객체 정보 퍼블리시
                        self.publish_detected_objects(header, clusters, cluster_labels)
                        
                        self.logger.info(f"Detected {len(clusters)} objects: {[OBJECT_NAMES.get(l, f'Unknown_{l}') for l in cluster_labels]}")
                    
                except Exception as e:
                    self.logger.error(f"Error in clustering: {e}")

                # 소켓 전송(선택)
                try:
                    payload = {'points': all_points, 'labels': all_labels}
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(0.01)
                    s.connect(('localhost', 50051))
                    s.sendall(pickle.dumps(payload) + b'END')
                    s.close()
                except Exception:
                    pass

                # 버퍼 초기화
                self.point_buffer = []
                self.label_buffer = []
                self.current_frame = 0
                self.last_publish_time = current_time

            # 빈 메시지라도 publish (point_step>0 유지)
            elif len(lidar) == 0:
                pc2 = self.create_cloud(header, fields, [])
                self.semantic_lidar_pub.publish(pc2)

        except Exception as e:
            self.logger.error(f"Error in semantic LiDAR callback: {e}")

    # -----------------------------
    # Telemetry & GPS
    # -----------------------------
    def publish_telemetry(self):
        if not self.vehicle:
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
                'speed_kmh': speed_kmh,
                'control': {'throttle': ctrl.throttle, 'steering': ctrl.steer, 'brake': ctrl.brake}
            }

            msg = String()
            msg.data = json.dumps(telemetry)
            self.telemetry_pub.publish(msg)

            id_msg = Int32()
            id_msg.data = self.vehicle.id
            self.vehicle_id_pub.publish(id_msg)

            odom = PoseStamped()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "map"
            odom.pose.position.x = tf.location.x
            odom.pose.position.y = tf.location.y
            odom.pose.position.z = tf.location.z
            self.odometry_pub.publish(odom)

        except Exception as e:
            self.logger.error(f"Error publishing telemetry: {e}")

    def gps_callback(self, gps_data):
        pass

    # -----------------------------
    # Cleanup
    # -----------------------------
    def cleanup(self):
        try:
            for sensor in self.sensors.values():
                if sensor:
                    sensor.destroy()
            if self.pedestrian:
                try:
                    self.pedestrian.destroy()
                except Exception:
                    pass
            if self.blocking_vehicle:
                try:
                    self.blocking_vehicle.destroy()
                except Exception:
                    pass
            if self.vehicle:
                self.vehicle.destroy()
            self.logger.info("Cleaned up CARLA actors")
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CarlaVehicleSpawner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if node:
            node.cleanup()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()