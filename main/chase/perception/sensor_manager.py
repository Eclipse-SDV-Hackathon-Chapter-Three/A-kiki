"""
Sensor Manager for Chase Vehicle
Manages all sensors attached to the chase vehicle
"""

import carla
import numpy as np
import threading
import time

class SensorManager:
    """센서 관리 클래스"""
    
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.sensors = {}
        self.sensor_data = {}
        self.data_lock = threading.Lock()
        
        print("🔍 Sensor Manager initialized")
    
    def add_camera(self, sensor_id="camera", location=carla.Location(0, 0, 2.0), 
                   rotation=carla.Rotation(0, 0, 0), image_size_x=1280, image_size_y=720, fov=90):
        """카메라 센서 추가"""
        try:
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            if camera_bp is None:
                raise RuntimeError("Camera blueprint not found")
            
            camera_bp.set_attribute('image_size_x', str(image_size_x))
            camera_bp.set_attribute('image_size_y', str(image_size_y))
            camera_bp.set_attribute('fov', str(fov))
            
            camera_transform = carla.Transform(location, rotation)
            camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
            
            self.sensors[sensor_id] = camera
            camera.listen(lambda data: self._process_camera_data(sensor_id, data))
            
            print(f"📷 Camera sensor '{sensor_id}' added")
            return camera
            
        except Exception as e:
            print(f"❌ Error adding camera sensor: {e}")
            return None
    
    def add_lidar(self, sensor_id="lidar", location=carla.Location(0, 0, 2.0),
                  rotation=carla.Rotation(0, 0, 0), channels=32, range=100.0):
        """LiDAR 센서 추가"""
        try:
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            if lidar_bp is None:
                raise RuntimeError("LiDAR blueprint not found")
            
            lidar_bp.set_attribute('channels', str(channels))
            lidar_bp.set_attribute('range', str(range))
            lidar_bp.set_attribute('points_per_second', '100000')
            
            lidar_transform = carla.Transform(location, rotation)
            lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
            
            self.sensors[sensor_id] = lidar
            lidar.listen(lambda data: self._process_lidar_data(sensor_id, data))
            
            print(f"📡 LiDAR sensor '{sensor_id}' added")
            return lidar
            
        except Exception as e:
            print(f"❌ Error adding LiDAR sensor: {e}")
            return None
    
    def add_radar(self, sensor_id="radar", location=carla.Location(0, 0, 2.0),
                  rotation=carla.Rotation(0, 0, 0), horizontal_fov=30.0, vertical_fov=30.0):
        """레이더 센서 추가"""
        try:
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            if radar_bp is None:
                raise RuntimeError("Radar blueprint not found")
            
            radar_bp.set_attribute('horizontal_fov', str(horizontal_fov))
            radar_bp.set_attribute('vertical_fov', str(vertical_fov))
            radar_bp.set_attribute('range', '100.0')
            
            radar_transform = carla.Transform(location, rotation)
            radar = self.world.spawn_actor(radar_bp, radar_transform, attach_to=self.vehicle)
            
            self.sensors[sensor_id] = radar
            radar.listen(lambda data: self._process_radar_data(sensor_id, data))
            
            print(f"📡 Radar sensor '{sensor_id}' added")
            return radar
            
        except Exception as e:
            print(f"❌ Error adding radar sensor: {e}")
            return None
    
    def _process_camera_data(self, sensor_id, data):
        """카메라 데이터 처리"""
        try:
            array = np.frombuffer(data.raw_data, dtype=np.uint8)
            array = array.reshape((data.height, data.width, 4))
            array = array[:, :, :3]  # BGRA -> BGR
            
            with self.data_lock:
                self.sensor_data[sensor_id] = {
                    'image': array,
                    'timestamp': time.time(),
                    'frame': data.frame
                }
                
        except Exception as e:
            print(f"⚠️ Error processing camera data: {e}")
    
    def _process_lidar_data(self, sensor_id, data):
        """LiDAR 데이터 처리"""
        try:
            points = np.frombuffer(data.raw_data, dtype=np.float32)
            points = points.reshape(-1, 4)  # x, y, z, intensity
            
            with self.data_lock:
                self.sensor_data[sensor_id] = {
                    'points': points,
                    'timestamp': time.time(),
                    'frame': data.frame
                }
                
        except Exception as e:
            print(f"⚠️ Error processing LiDAR data: {e}")
    
    def _process_radar_data(self, sensor_id, data):
        """레이더 데이터 처리"""
        try:
            points = np.frombuffer(data.raw_data, dtype=np.float32)
            points = points.reshape(-1, 4)  # x, y, z, velocity
            
            with self.data_lock:
                self.sensor_data[sensor_id] = {
                    'points': points,
                    'timestamp': time.time(),
                    'frame': data.frame
                }
                
        except Exception as e:
            print(f"⚠️ Error processing radar data: {e}")
    
    def get_sensor_data(self, sensor_id):
        """센서 데이터 가져오기"""
        with self.data_lock:
            return self.sensor_data.get(sensor_id, None)
    
    def get_all_sensor_data(self):
        """모든 센서 데이터 가져오기"""
        with self.data_lock:
            return self.sensor_data.copy()
    
    def destroy_all_sensors(self):
        """모든 센서 정리"""
        try:
            for sensor_id, sensor in self.sensors.items():
                try:
                    sensor.destroy()
                    print(f"🔍 Sensor '{sensor_id}' destroyed")
                except:
                    pass
            
            self.sensors.clear()
            self.sensor_data.clear()
            
        except Exception as e:
            print(f"⚠️ Error destroying sensors: {e}")
    
    def cleanup(self):
        """리소스 정리"""
        self.destroy_all_sensors()

