#!/usr/bin/env python3
"""
Sensor Manager
센서 관리 클래스
"""

import carla
import time

class SensorManager:
    """센서 관리 클래스"""
    
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.imu_sensor = None
        self.current_imu_data = None
    
    def setup_imu_sensor(self):
        """IMU 센서 설정"""
        try:
            if not self.vehicle:
                print("❌ No vehicle available for IMU setup")
                return False
            
            # IMU 센서 블루프린트 생성
            imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
            if not imu_bp:
                print("❌ IMU sensor blueprint not found")
                return False
            
            # IMU 센서 속성 설정
            imu_bp.set_attribute('sensor_tick', '0.1')
            
            # IMU 센서 위치 설정 (차량 중심)
            imu_transform = carla.Transform(carla.Location(x=0, y=0, z=0))
            
            # IMU 센서 생성 및 부착
            self.imu_sensor = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)
            
            # 콜백 함수 등록
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
                'accelerometer': {
                    'x': imu_data.accelerometer.x,
                    'y': imu_data.accelerometer.y,
                    'z': imu_data.accelerometer.z
                },
                'gyroscope': {
                    'x': imu_data.gyroscope.x,
                    'y': imu_data.gyroscope.y,
                    'z': imu_data.gyroscope.z
                },
                'compass': imu_data.compass,
                'timestamp': time.time()
            }
        except Exception as e:
            print(f"⚠️ Error processing IMU data: {e}")
    
    def get_imu_data(self):
        """현재 IMU 데이터 반환"""
        return self.current_imu_data
    
    def get_vehicle_velocity(self):
        """차량 속도 반환"""
        try:
            if not self.vehicle:
                return None
            
            velocity = self.vehicle.get_velocity()
            return {
                'x': velocity.x,
                'y': velocity.y,
                'z': velocity.z,
                'speed': (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
            }
        except Exception as e:
            print(f"⚠️ Error getting vehicle velocity: {e}")
            return None
    
    def get_vehicle_acceleration(self):
        """차량 가속도 반환 (IMU 데이터 기반)"""
        try:
            if not self.current_imu_data:
                return None
            
            accel = self.current_imu_data['accelerometer']
            return {
                'x': accel['x'],
                'y': accel['y'],
                'z': accel['z'],
                'magnitude': (accel['x']**2 + accel['y']**2 + accel['z']**2)**0.5
            }
        except Exception as e:
            print(f"⚠️ Error getting vehicle acceleration: {e}")
            return None
    
    def cleanup(self):
        """센서 정리"""
        try:
            if self.imu_sensor:
                self.imu_sensor.destroy()
                self.imu_sensor = None
                print("✅ IMU sensor cleaned up")
        except Exception as e:
            print(f"⚠️ Error cleaning up sensors: {e}")

