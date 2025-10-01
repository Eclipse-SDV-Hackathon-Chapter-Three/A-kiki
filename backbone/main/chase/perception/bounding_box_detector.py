"""
Bounding Box Detector for CARLA
CARLA 공식 튜토리얼을 참고한 바운딩 박스 감지
https://carla.readthedocs.io/en/latest/tuto_G_bounding_boxes/
"""

import carla
import numpy as np
import cv2
import time
from typing import List, Dict, Tuple, Optional
import math
import sys
import os

# 상위 디렉토리의 모듈들을 import하기 위해 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

try:
    from chase.perception.collision_detector import CollisionDetector
except ImportError as e:
    print(f"⚠️ Warning: Could not import collision detector: {e}")
    CollisionDetector = None

class BoundingBoxDetector:
    """CARLA 바운딩 박스 감지 클래스 (공식 튜토리얼 기반)"""
    
    def __init__(self, world, camera):
        self.world = world
        self.camera = camera
        self.detected_objects = []
        
        # 카메라 설정
        self._setup_camera()
        
        # 충돌 감지기 초기화
        if CollisionDetector:
            self.collision_detector = CollisionDetector()
            print("🚨 Collision detector enabled")
        else:
            self.collision_detector = None
            print("⚠️ Collision detector not available")
        
        print("🎯 Bounding Box Detector initialized (CARLA official method)")
    
    def _setup_camera(self):
        """카메라 설정 및 투영 행렬 계산"""
        try:
            # 카메라 속성 가져오기
            self.image_w = int(self.camera.attributes['image_size_x'])
            self.image_h = int(self.camera.attributes['image_size_y'])
            self.fov = float(self.camera.attributes['fov'])
            
            # 카메라 투영 행렬 계산 (CARLA 공식 방식)
            self.K = self._build_projection_matrix(self.image_w, self.image_h, self.fov)
            self.K_behind = self._build_projection_matrix(self.image_w, self.image_h, self.fov, is_behind_camera=True)
            
            # 카메라 캘리브레이션 설정
            self.camera.calibration = self.K
            
            print(f"📷 Camera setup: {self.image_w}x{self.image_h}, FOV: {self.fov}")
            
        except Exception as e:
            print(f"⚠️ Error setting up camera: {e}")
            # 기본값 설정 (carla_vehicle_spawner.py와 일치)
            self.image_w = 1080
            self.image_h = 720
            self.fov = 90.0
            self.K = self._build_projection_matrix(self.image_w, self.image_h, self.fov)
            self.K_behind = self._build_projection_matrix(self.image_w, self.image_h, self.fov, is_behind_camera=True)
            self.camera.calibration = self.K
    
    def _build_projection_matrix(self, w, h, fov, is_behind_camera=False):
        """CARLA 공식 방식의 투영 행렬 계산 (올바른 버전)"""
        # FOV를 라디안으로 변환 (수평 FOV 기준)
        fov_rad = fov * np.pi / 180.0
        
        # CARLA 공식에 따른 초점 거리 계산
        # FOV 90도일 때: focal = w / (2 * tan(45°)) = w / 2
        focal_x = w / (2.0 * np.tan(fov_rad / 2.0))
        
        # CARLA는 정사각형 픽셀을 가정하므로 focal_y = focal_x
        focal_y = focal_x
        
        # 카메라 내부 매개변수 행렬 구성
        K = np.zeros((3, 3))
        
        if is_behind_camera:
            K[0, 0] = -focal_x  # fx
            K[1, 1] = -focal_y  # fy
        else:
            K[0, 0] = focal_x   # fx
            K[1, 1] = focal_y   # fy
            
        K[0, 2] = w / 2.0       # cx (주점 x)
        K[1, 2] = h / 2.0       # cy (주점 y)
        K[2, 2] = 1.0           # 스케일 팩터
        
        print(f"📷 Projection matrix (올바른 버전):")
        print(f"   Focal length: fx={focal_x:.1f}, fy={focal_y:.1f}")
        print(f"   Principal point: cx={K[0,2]:.1f}, cy={K[1,2]:.1f}")
        print(f"   Image size: {w}x{h}, FOV: {fov}°")
        print(f"   Aspect ratio: {w/h:.3f}")
        
        return K
    
    
    def _get_image_point(self, loc, K, w2c):
        """3D 좌표를 2D 이미지 좌표로 투영 (CARLA 공식 방법 - 정확한 버전)"""
        try:
            # 입력 좌표 포맷 (loc은 carla.Location 객체)
            point = np.array([loc.x, loc.y, loc.z, 1])
            
            # 월드 좌표를 카메라 좌표로 변환
            point_camera = np.dot(w2c, point)
            
            # 카메라 뒤에 있는 점은 무시
            if point_camera[2] <= 0:
                return None
            
            # CARLA의 좌표계 변환 (UE4 -> OpenCV)
            # CARLA: X(앞), Y(오른쪽), Z(위)
            # OpenCV: X(오른쪽), Y(아래), Z(앞)
            # 좌우/상하 반전 수정
            x_cv = -point_camera[1]  # CARLA -Y -> OpenCV X (좌우 반전)
            y_cv = point_camera[2]   # CARLA Z -> OpenCV Y (상하 반전)
            z_cv = point_camera[0]   # CARLA X -> OpenCV Z
            
            # 3D -> 2D 투영
            point_3d = np.array([x_cv, y_cv, z_cv])
            point_img = np.dot(K, point_3d)
            
            # 정규화
            if point_img[2] != 0:
                u = point_img[0] / point_img[2]
                v = point_img[1] / point_img[2]
            else:
                return None
            
            # 유효한 좌표인지 확인
            if np.isnan(u) or np.isnan(v) or np.isinf(u) or np.isinf(v):
                return None
                
            return np.array([u, v])
            
        except Exception as e:
            return None
    
    def detect_pedestrians_and_vehicles(self, max_distance=300.0):
        """사람과 차량의 바운딩 박스 감지 (CARLA 공식 방법)"""
        try:
            detected_objects = []
            
            if not self.camera or not self.world:
                print("⚠️ Camera or world not available")
                return []
            
            # 월드-카메라 변환 행렬
            world_2_camera = np.array(self.camera.get_transform().get_inverse_matrix())
            
            # 모든 액터 가져오기
            actors = self.world.get_actors()
            
            # 차량과 보행자 필터링
            vehicles = actors.filter('vehicle.*')
            pedestrians = actors.filter('walker.*')
            
            # 차량 감지
            for vehicle in vehicles:
                try:
                    bbox_data = self._get_actor_bounding_box(vehicle, world_2_camera, max_distance, 'vehicle')
                    if bbox_data:
                        detected_objects.append(bbox_data)
                except Exception as e:
                    continue
            
            # 보행자 감지
            for pedestrian in pedestrians:
                try:
                    bbox_data = self._get_actor_bounding_box(pedestrian, world_2_camera, max_distance, 'pedestrian')
                    if bbox_data:
                        detected_objects.append(bbox_data)
                except Exception as e:
                    continue
            
            self.detected_objects = detected_objects
            
            # 충돌 감지 분석
            if self.collision_detector:
                collision_events = self.collision_detector.analyze_pedestrian_collision(detected_objects)
                if collision_events:
                    print(f"🚨 COLLISION DETECTED: {len(collision_events)} events")
                    for event in collision_events:
                        print(f"   - Actor {event['actor_id']}: Score {event['collision_score']:.2f}")
            
            return detected_objects
            
        except Exception as e:
            print(f"❌ Error detecting objects: {e}")
            return []
    
    def _get_actor_bounding_box(self, actor, world_2_camera, max_distance, actor_type):
        """액터의 2D 바운딩 박스 계산 (CARLA 공식 방법)"""
        try:
            # 거리 확인
            actor_location = actor.get_location()
            camera_location = self.camera.get_location()
            distance = actor_location.distance(camera_location)
            
            if distance > max_distance:
                return None
            
            # 3D 바운딩 박스 가져오기
            bbox = actor.bounding_box
            
            # 바운딩 박스의 8개 꼭짓점을 월드 좌표로 가져오기
            verts = [v for v in bbox.get_world_vertices(actor.get_transform())]
            
            # 2D 투영
            x_max = -10000
            x_min = 10000
            y_max = -10000
            y_min = 10000
            
            valid_points = 0
            projected_points = []
            
            for vert in verts:
                p = self._get_image_point(vert, self.K, world_2_camera)
                if p is None:
                    continue
                
                valid_points += 1
                projected_points.append(p)
                
                # 가장 오른쪽 꼭짓점 찾기
                if p[0] > x_max:
                    x_max = p[0]
                # 가장 왼쪽 꼭짓점 찾기
                if p[0] < x_min:
                    x_min = p[0]
                # 가장 높은 꼭짓점 찾기
                if p[1] > y_max:
                    y_max = p[1]
                # 가장 낮은 꼭짓점 찾기
                if p[1] < y_min:
                    y_min = p[1]
            
            # 투영된 점들의 통계 출력
            if projected_points:
                points_array = np.array(projected_points)
                print(f"🔍 Actor {actor.id} projected points:")
                print(f"   X range: {points_array[:, 0].min():.1f} to {points_array[:, 0].max():.1f}")
                print(f"   Y range: {points_array[:, 1].min():.1f} to {points_array[:, 1].max():.1f}")
                print(f"   Valid points: {valid_points}/{len(verts)}")
            
            # 충분한 유효한 점이 있는지 확인
            if valid_points < 4:
                return None
            
            # 바운딩 박스가 화면 내에 있는지 확인 (엄격한 조건)
            bbox_width = x_max - x_min
            bbox_height = y_max - y_min
            
            # 바운딩 박스 크기 제한
            max_bbox_width = self.image_w * 0.8   # 화면 너비의 80% 이하
            max_bbox_height = self.image_h * 0.8  # 화면 높이의 80% 이하
            min_bbox_width = 10   # 최소 10픽셀
            min_bbox_height = 10  # 최소 10픽셀
            
            # 화면 내에 있는지 확인 (엄격한 조건 - 카메라 시야 내에만)
            in_screen = (x_max > 0 and x_min < self.image_w and 
                        y_max > 0 and y_min < self.image_h)
            
            # 크기 조건 확인
            size_valid = (bbox_width > min_bbox_width and bbox_height > min_bbox_height and
                         bbox_width < max_bbox_width and bbox_height < max_bbox_height)
            
            # 추가: Z 좌표 확인 (카메라 앞에 있는지)
            camera_z = self.camera.get_location().z
            actor_z = actor.get_location().z
            in_front_of_camera = actor_z > camera_z - 2.0  # 카메라보다 2m 이상 앞에 있는지
            
            # 디버그 정보 출력
            print(f"🔍 Actor {actor.id} ({actor_type}) filtering:")
            print(f"   In screen: {in_screen} (x: {x_min:.1f}-{x_max:.1f}, y: {y_min:.1f}-{y_max:.1f})")
            print(f"   Size valid: {size_valid} (w: {bbox_width:.1f}, h: {bbox_height:.1f})")
            print(f"   In front: {in_front_of_camera} (camera_z: {camera_z:.1f}, actor_z: {actor_z:.1f})")
            
            if in_screen and size_valid and in_front_of_camera:
                
                # 화면 경계 내로 클리핑 (더 정확한 클리핑)
                x_min_clipped = max(0, min(x_min, self.image_w - 1))
                y_min_clipped = max(0, min(y_min, self.image_h - 1))
                x_max_clipped = max(0, min(x_max, self.image_w - 1))
                y_max_clipped = max(0, min(y_max, self.image_h - 1))
                
                # 최종 바운딩 박스 크기 계산
                final_width = x_max_clipped - x_min_clipped
                final_height = y_max_clipped - y_min_clipped
                
                # 바운딩 박스가 너무 작아진 경우 원본 크기 유지
                if final_width < min_bbox_width or final_height < min_bbox_height:
                    # 원본 크기를 유지하되 화면 경계 내로만 조정
                    x_min_clipped = max(0, x_min)
                    y_min_clipped = max(0, y_min)
                    x_max_clipped = min(self.image_w - 1, x_max)
                    y_max_clipped = min(self.image_h - 1, y_max)
                    final_width = x_max_clipped - x_min_clipped
                    final_height = y_max_clipped - y_min_clipped
                
                # 바운딩 박스 크기 분석
                original_area = (x_max - x_min) * (y_max - y_min)
                clipped_area = final_width * final_height
                screen_area = self.image_w * self.image_h
                area_ratio = clipped_area / screen_area
                
                print(f"✅ Actor {actor.id} detected: {actor_type} at {distance:.1f}m")
                print(f"   Original bbox: ({int(x_min)},{int(y_min)})-({int(x_max)},{int(y_max)})")
                print(f"   Clipped bbox: ({int(x_min_clipped)},{int(y_min_clipped)})-({int(x_max_clipped)},{int(y_max_clipped)})")
                print(f"   Area: {clipped_area} pixels ({area_ratio*100:.1f}% of screen)")
                print(f"   Size: {final_width}x{final_height}")
                
                # 바운딩 박스 중심점 계산
                center_x = (x_min_clipped + x_max_clipped) / 2
                center_y = (y_min_clipped + y_max_clipped) / 2
                screen_center_x = self.image_w / 2
                screen_center_y = self.image_h / 2
                offset_x = center_x - screen_center_x
                offset_y = center_y - screen_center_y
                
                print(f"   Center: ({center_x:.1f}, {center_y:.1f})")
                print(f"   Screen center: ({screen_center_x:.1f}, {screen_center_y:.1f})")
                print(f"   Offset: ({offset_x:.1f}, {offset_y:.1f})")
                
                # 바운딩 박스 크기 경고
                if area_ratio > 0.5:
                    print(f"   ⚠️ WARNING: Bbox too large! ({area_ratio*100:.1f}% of screen)")
                elif area_ratio < 0.001:
                    print(f"   ⚠️ WARNING: Bbox too small! ({area_ratio*100:.3f}% of screen)")
                elif area_ratio > 0.1:
                    print(f"   ℹ️ INFO: Large bbox ({area_ratio*100:.1f}% of screen) - may be close object")
                
                # 위치 경고
                if abs(offset_x) > self.image_w * 0.3 or abs(offset_y) > self.image_h * 0.3:
                    print(f"   ⚠️ WARNING: Bbox far from center! Offset: ({offset_x:.1f}, {offset_y:.1f})")
                
                return {
                    'type': actor_type,
                    'bbox_2d': {
                        'x_min': int(x_min_clipped),
                        'y_min': int(y_min_clipped),
                        'x_max': int(x_max_clipped),
                        'y_max': int(y_max_clipped),
                        'width': int(final_width),
                        'height': int(final_height)
                    },
                    'distance': distance,
                    'actor_id': actor.id,  # 충돌 감지기가 기대하는 필드명
                    'confidence': min(1.0, max(0.0, 1.0 - distance / max_distance)),
                    'world_location': (actor.get_location().x, actor.get_location().y, actor.get_location().z),
                    'velocity': self._get_velocity(actor)
                }
            else:
                # 필터링된 객체에 대한 로깅
                print(f"❌ Actor {actor.id} ({actor_type}) filtered out:")
                if not in_screen:
                    print(f"   Reason: Not in screen (x: {x_min:.1f}-{x_max:.1f}, y: {y_min:.1f}-{y_max:.1f})")
                if not size_valid:
                    print(f"   Reason: Invalid size (w: {bbox_width:.1f}, h: {bbox_height:.1f})")
                if not in_front_of_camera:
                    print(f"   Reason: Behind camera (camera_z: {camera_z:.1f}, actor_z: {actor_z:.1f})")
            
            return None
            
        except Exception as e:
            return None
    
    def _get_velocity(self, actor):
        """액터의 속도 계산"""
        try:
            velocity = actor.get_velocity()
            return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        except Exception as e:
            return 0.0
    
    def draw_bounding_boxes_on_image(self, image, use_3d=False):
        """이미지에 바운딩 박스 그리기 (2D 또는 3D)"""
        try:
            print(f"🎨 Drawing bounding boxes for {len(self.detected_objects)} objects")
            if not self.detected_objects:
                print("⚠️ No detected objects to draw")
                return image
            
            # 이미지 복사
            display_image = image.copy()
            
            for obj in self.detected_objects:
                obj_type = obj['type']
                distance = obj['distance']
                actor_id = obj['actor_id']
                
                # 색상 설정
                if obj_type == 'vehicle':
                    color = (0, 255, 0)  # 녹색
                    # 차량 타입에 따른 이름 표시
                    vehicle_name = self._get_vehicle_name(actor_id)
                    label = f"{vehicle_name} ({distance:.1f}m)"
                else:  # pedestrian
                    color = (0, 0, 255)  # 빨간색
                    # 보행자 타입에 따른 이름 표시
                    pedestrian_name = self._get_pedestrian_name(actor_id)
                    label = f"{pedestrian_name} ({distance:.1f}m)"
                
                if use_3d:
                    # 3D 바운딩 박스 그리기
                    self._draw_3d_bounding_box(display_image, obj, color)
                else:
                    # 2D 바운딩 박스 그리기
                    bbox_2d = obj['bbox_2d']
                    x_min = bbox_2d['x_min']
                    y_min = bbox_2d['y_min']
                    x_max = bbox_2d['x_max']
                    y_max = bbox_2d['y_max']
                    
                    # 사각형 그리기 (두꺼운 선)
                    cv2.rectangle(display_image, (x_min, y_min), (x_max, y_max), color, 4)
                
                # 라벨 그리기
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                if not use_3d:
                    # 2D의 경우 바운딩 박스 위에 라벨
                    bbox_2d = obj['bbox_2d']
                    x_min = bbox_2d['x_min']
                    y_min = bbox_2d['y_min']
                    cv2.rectangle(display_image, (x_min, y_min - label_size[1] - 10), 
                                (x_min + label_size[0] + 10, y_min), color, -1)
                    cv2.putText(display_image, label, (x_min + 5, y_min - 5), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                else:
                    # 3D의 경우 화면 상단에 라벨
                    cv2.rectangle(display_image, (10, 10), (10 + label_size[0] + 10, 10 + label_size[1] + 10), 
                                (0, 0, 0), -1)
                    cv2.putText(display_image, label, (15, 15 + label_size[1]), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            return display_image
            
        except Exception as e:
            print(f"⚠️ Error drawing bounding boxes: {e}")
            return image
    
    def _draw_3d_bounding_box(self, image, obj, color):
        """3D 바운딩 박스를 2D 이미지에 그리기 (CARLA 공식 튜토리얼 방식)"""
        try:
            # 액터 정보 가져오기
            actor_id = obj['actor_id']
            actors = self.world.get_actors()
            actor = None
            
            for a in actors:
                if a.id == actor_id:
                    actor = a
                    break
            
            if not actor:
                return
            
            # CARLA 공식 방식: get_world_vertices() 사용
            bbox_3d = actor.bounding_box
            verts = [v for v in bbox_3d.get_world_vertices(actor.get_transform())]
            
            # 월드 → 카메라 변환 행렬 (CARLA 공식 방식)
            world_2_camera = np.array(self.camera.get_transform().get_inverse_matrix())
            
            # 3D 꼭짓점들을 2D로 투영 (CARLA 공식 방식)
            points_2d = []
            for vert in verts:
                p = self._get_image_point(vert, self.camera.calibration, world_2_camera)
                if 0 <= p[0] < self.image_w and 0 <= p[1] < self.image_h:
                    points_2d.append(p)
            
            if len(points_2d) < 8:  # 3D 박스는 8개 꼭짓점 필요
                return
            
            # CARLA 공식 방식: 12개 모서리 그리기
            edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]
            
            for edge in edges:
                if edge[0] < len(points_2d) and edge[1] < len(points_2d):
                    self._draw_line(image, points_2d[edge[0]], points_2d[edge[1]], color, 2)
            
        except Exception as e:
            print(f"⚠️ Error drawing 3D bounding box: {e}")
    
    def _get_image_point(self, loc, K, w2c):
        """CARLA 공식 방식의 3D → 2D 투영"""
        try:
            # Format the input coordinate (loc is a carla.Position object)
            point = np.array([loc.x, loc.y, loc.z, 1])
            # transform to camera coordinates
            point_camera = np.dot(w2c, point)

            # New we must change from UE4's coordinate system to an "standard"
            # (x, y ,z) -> (y, -z, x)
            # and we remove the fourth componebonent also
            point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

            # now project 3D->2D using the camera matrix
            point_img = np.dot(K, point_camera)
            # normalize
            point_img[0] /= point_img[2]
            point_img[1] /= point_img[2]

            return point_img[0:2]
        except Exception as e:
            return [0, 0]
    
    def _draw_line(self, image, point1, point2, color, thickness=2):
        """두 점 사이에 선 그리기"""
        try:
            pt1 = (int(point1[0]), int(point1[1]))
            pt2 = (int(point2[0]), int(point2[1]))
            cv2.line(image, pt1, pt2, color, thickness)
        except Exception as e:
            pass  # 선을 그릴 수 없는 경우 무시
    
    def get_detection_data_for_zenoh(self):
        """Zenoh용 감지 데이터 포맷"""
        try:
            zenoh_data = {
                'timestamp': time.time(),
                'detections': []
            }
            
            for obj in self.detected_objects:
                detection = {
                    'type': obj['type'],
                    'id': obj['actor_id'],  # actor_id를 id로 매핑
                    'distance': obj['distance'],
                    'confidence': obj['confidence'],
                    'bbox_2d': obj['bbox_2d']
                }
                zenoh_data['detections'].append(detection)
            
            return zenoh_data
            
        except Exception as e:
            print(f"⚠️ Error formatting detection data: {e}")
            return {'timestamp': time.time(), 'detections': []}
    
    def get_detection_stats(self):
        """감지 통계 반환"""
        try:
            stats = {
                'total_detections': len(self.detected_objects),
                'vehicles': len([obj for obj in self.detected_objects if obj['type'] == 'vehicle']),
                'pedestrians': len([obj for obj in self.detected_objects if obj['type'] == 'pedestrian'])
            }
            
            # 충돌 통계 추가
            if self.collision_detector:
                collision_summary = self.collision_detector.get_collision_summary()
                stats.update({
                    'collisions': collision_summary['total_collisions'],
                    'active_pedestrians': collision_summary['active_pedestrians']
                })
            
            return stats
            
        except Exception as e:
            return {'total_detections': 0, 'vehicles': 0, 'pedestrians': 0, 'collisions': 0}
    
    def get_collision_events(self):
        """충돌 이벤트 반환"""
        try:
            if self.collision_detector:
                return self.collision_detector.get_collision_summary()['collision_events']
            return []
        except Exception as e:
            return []
    
    def _get_vehicle_name(self, actor_id):
        """차량 이름 가져오기"""
        try:
            actors = self.world.get_actors()
            for actor in actors:
                if actor.id == actor_id and actor.type_id.startswith('vehicle.'):
                    # CARLA 차량 타입에서 실제 모델명 추출
                    type_id = actor.type_id
                    if 'tesla' in type_id.lower():
                        return "Tesla"
                    elif 'bmw' in type_id.lower():
                        return "BMW"
                    elif 'audi' in type_id.lower():
                        return "Audi"
                    elif 'mercedes' in type_id.lower():
                        return "Mercedes"
                    elif 'ford' in type_id.lower():
                        return "Ford"
                    elif 'chevrolet' in type_id.lower():
                        return "Chevrolet"
                    elif 'nissan' in type_id.lower():
                        return "Nissan"
                    elif 'toyota' in type_id.lower():
                        return "Toyota"
                    elif 'volkswagen' in type_id.lower():
                        return "Volkswagen"
                    elif 'lincoln' in type_id.lower():
                        return "Lincoln"
                    else:
                        # 기본 차량 이름
                        return "Vehicle"
            return "Unknown Vehicle"
        except Exception as e:
            return "Vehicle"
    
    def _get_pedestrian_name(self, actor_id):
        """보행자 이름 가져오기"""
        try:
            actors = self.world.get_actors()
            for actor in actors:
                if actor.id == actor_id and actor.type_id.startswith('walker.'):
                    # CARLA 보행자 타입에서 성별/나이 추출
                    type_id = actor.type_id
                    if 'male' in type_id.lower():
                        if 'young' in type_id.lower():
                            return "Young Man"
                        elif 'old' in type_id.lower():
                            return "Old Man"
                        else:
                            return "Man"
                    elif 'female' in type_id.lower():
                        if 'young' in type_id.lower():
                            return "Young Woman"
                        elif 'old' in type_id.lower():
                            return "Old Woman"
                        else:
                            return "Woman"
                    else:
                        return "Pedestrian"
            return "Person"
        except Exception as e:
            return "Person"