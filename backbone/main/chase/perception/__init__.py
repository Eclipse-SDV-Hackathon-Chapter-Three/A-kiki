"""
Perception module for chase vehicle
Handles sensor data processing and object detection
"""

from .sensor_manager import SensorManager
from .object_detector import ObjectDetector
from .target_tracker import TargetTracker

class PerceptionModule:
    """Perception module wrapper"""
    
    def __init__(self, world=None, vehicle=None):
        self.sensor_manager = SensorManager(world, vehicle) if world and vehicle else None
        self.object_detector = ObjectDetector()
        self.target_tracker = TargetTracker()
    
    def add_camera(self, *args, **kwargs):
        return self.sensor_manager.add_camera(*args, **kwargs)
    
    def add_lidar(self, *args, **kwargs):
        return self.sensor_manager.add_lidar(*args, **kwargs)
    
    def add_radar(self, *args, **kwargs):
        return self.sensor_manager.add_radar(*args, **kwargs)
    
    def get_sensor_data(self, sensor_id):
        return self.sensor_manager.get_sensor_data(sensor_id)
    
    def detect_vehicles_in_camera(self, image_data):
        return self.object_detector.detect_vehicles_in_camera(image_data)
    
    def detect_vehicles_in_lidar(self, lidar_data):
        return self.object_detector.detect_vehicles_in_lidar(lidar_data)
    
    def detect_vehicles_in_radar(self, radar_data):
        return self.object_detector.detect_vehicles_in_radar(radar_data)
    
    def fuse_detections(self, camera_vehicles, lidar_vehicles, radar_vehicles):
        return self.object_detector.fuse_detections(camera_vehicles, lidar_vehicles, radar_vehicles)
    
    def update_target(self, detected_vehicles, chase_position):
        return self.target_tracker.update_target(detected_vehicles, chase_position)
    
    def cleanup(self):
        self.sensor_manager.cleanup()

__all__ = [
    'PerceptionModule',
    'SensorManager',
    'ObjectDetector',
    'TargetTracker'
]

