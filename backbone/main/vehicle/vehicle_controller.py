import carla
import random
import math
import time

class VehicleController:
    """차량 생성 및 제어 클래스"""
    
    def __init__(self, world):
        self.world = world
        self.vehicle = None
        self.vehicle_control = carla.VehicleControl()
        self.vehicle_control.throttle = 0.0
        self.vehicle_control.brake = 0.0
        self.vehicle_control.steer = 0.0
        self.vehicle_control.hand_brake = False
        self.vehicle_control.reverse = False
        
        # 차량 위치 설정 (횡단보도와 수직)
        self.vehicle_location = carla.Location(-104.15, 44.15, 1.0)
        self.vehicle_yaw = 270.0  # 횡단보도와 수직이 되도록
        
    def spawn_vehicle(self):
        """자동차 스폰 (횡단보도와 수직 방향)"""
        # 자동차 블루프린트 선택
        vehicle_bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        if vehicle_bp is None:
            vehicle_bp = self.world.get_blueprint_library().find('vehicle.audi.tt')
        if vehicle_bp is None:
            vehicle_bp = self.world.get_blueprint_library().find('vehicle.volkswagen.t2')
        
        if vehicle_bp is None:
            raise RuntimeError("No suitable vehicle blueprint found")
            
        vehicle_bp.set_attribute('role_name', 'manual_vehicle')
        
        # 자동차 색상 설정
        try:
            vehicle_color = random.choice(vehicle_bp.get_attribute('color').recommended_values)
            vehicle_bp.set_attribute('color', vehicle_color)
        except:
            print("Could not set vehicle color")
        
        # 자동차 물리 속성 설정
        if vehicle_bp.has_attribute('mass'):
            vehicle_bp.set_attribute('mass', '1500.0')  # 1500kg
        if vehicle_bp.has_attribute('gravity_scale'):
            vehicle_bp.set_attribute('gravity_scale', '1.0')
        
        # 차량 위치 (횡단보도와 수직이 되도록)
        vehicle_location = self.vehicle_location
        
        # 횡단보도와 수직이 되도록 회전 계산
        vehicle_rotation = carla.Rotation(0, self.vehicle_yaw, 0)
        vehicle_transform = carla.Transform(vehicle_location, vehicle_rotation)
        
        print(f"Attempting to spawn vehicle at: {vehicle_location}")
        print(f"Vehicle rotation (yaw): {self.vehicle_yaw} degrees")
        
        self.vehicle = self.world.try_spawn_actor(vehicle_bp, vehicle_transform)
        
        if self.vehicle is None:
            # 다른 위치에서 시도
            print("First vehicle spawn attempt failed, trying alternative locations...")
            spawn_points = self.world.get_map().get_spawn_points()
            for i, alt_spawn in enumerate(spawn_points[:5]):
                print(f"Trying spawn point {i+1}: {alt_spawn.location}")
                self.vehicle = self.world.try_spawn_actor(vehicle_bp, alt_spawn)
                if self.vehicle is not None:
                    print(f"Vehicle successfully spawned at alternative location: {alt_spawn.location}")
                    break
            
            if self.vehicle is None:
                raise RuntimeError("Failed to spawn vehicle at any location")
        
        # 자동차 물리 설정
        self.vehicle.set_simulate_physics(True)
        
        print(f"Vehicle spawned successfully at: {self.vehicle.get_location()}")
        print("🚗 Vehicle physics simulation ENABLED")
        return self.vehicle
    
    def update_control(self, keys):
        """차량 제어 업데이트"""
        if self.vehicle is None:
            return
        
        # 키보드 입력에 따른 제어
        if keys.get('vehicle_throttle', False):
            self.vehicle_control.throttle = min(self.vehicle_control.throttle + 0.1, 1.0)
            self.vehicle_control.brake = 0.0
        else:
            self.vehicle_control.throttle = max(self.vehicle_control.throttle - 0.1, 0.0)
        
        if keys.get('vehicle_brake', False):
            self.vehicle_control.brake = min(self.vehicle_control.brake + 0.1, 1.0)
            self.vehicle_control.throttle = 0.0
        else:
            self.vehicle_control.brake = max(self.vehicle_control.brake - 0.1, 0.0)
        
        if keys.get('vehicle_steer_left', False):
            self.vehicle_control.steer = max(self.vehicle_control.steer - 0.1, -1.0)
        elif keys.get('vehicle_steer_right', False):
            self.vehicle_control.steer = min(self.vehicle_control.steer + 0.1, 1.0)
        else:
            self.vehicle_control.steer = 0.0
        
        if keys.get('vehicle_reverse', False):
            self.vehicle_control.reverse = True
        else:
            self.vehicle_control.reverse = False
        
        if keys.get('vehicle_hand_brake', False):
            self.vehicle_control.hand_brake = True
        else:
            self.vehicle_control.hand_brake = False
        
        # 차량에 제어 적용
        self.vehicle.apply_control(self.vehicle_control)
        
        # 현재 위치 출력 (주기적으로)
        current_time = time.time()
        if not hasattr(self, 'last_position_print') or current_time - self.last_position_print > 1.0:
            self.last_position_print = current_time
            location = self.vehicle.get_location()
            rotation = self.vehicle.get_transform().rotation
            print(f"🚗 CAR Position: X={location.x:.2f}, Y={location.y:.2f}, Z={location.z:.2f}")
            print(f"🚗 CAR Rotation: Pitch={rotation.pitch:.1f}, Yaw={rotation.yaw:.1f}, Roll={rotation.roll:.1f}")
    
    def get_location(self):
        """차량 현재 위치 반환"""
        return self.vehicle.get_location() if self.vehicle else None
    
    def get_velocity(self):
        """차량 현재 속도 반환"""
        return self.vehicle.get_velocity() if self.vehicle else None
    
    def get_speed(self):
        """차량 현재 속력 반환 (m/s)"""
        if self.vehicle is None:
            return 0.0
        
        velocity = self.vehicle.get_velocity()
        return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    
    def apply_impulse_to_pedestrian(self, pedestrian, collision_location):
        """보행자에게 임펄스 적용 (충돌 효과)"""
        if self.vehicle is None or pedestrian is None:
            return
        
        try:
            vehicle_velocity = self.vehicle.get_velocity()
            vehicle_speed = self.get_speed()
            
            print(f"🚗 Vehicle speed at collision: {vehicle_speed:.2f} m/s")
            
            vehicle_location = self.vehicle.get_location()
            pedestrian_location = pedestrian.get_location()
            
            # 충돌 방향 계산
            direction_x = pedestrian_location.x - vehicle_location.x
            direction_y = pedestrian_location.y - vehicle_location.y
            direction_z = pedestrian_location.z - vehicle_location.z
            
            length = math.sqrt(direction_x**2 + direction_y**2 + direction_z**2)
            if length > 0:
                direction_x /= length
                direction_y /= length
                direction_z /= length
            
            # 충격력 계산 (차량 속도에 비례)
            impact_force = min(vehicle_speed * 2.0, 15.0)
            
            # 임펄스 적용
            impulse = carla.Vector3D(
                direction_x * impact_force,
                direction_y * impact_force,
                direction_z * impact_force + 2.0
            )
            
            pedestrian.add_impulse(impulse)
            print(f"💥 Applied impulse to pedestrian: {impulse}")
            
            # 랜덤 회전 적용 (넘어지는 효과)
            random_rotation = carla.Rotation(
                random.uniform(-45, 45),
                random.uniform(-180, 180),
                random.uniform(-45, 45)
            )
            
            current_transform = pedestrian.get_transform()
            new_transform = carla.Transform(
                current_transform.location,
                current_transform.rotation + random_rotation
            )
            pedestrian.set_transform(new_transform)
            
            print("🔄 Pedestrian rotation applied for falling effect")
            
        except Exception as e:
            print(f"⚠️  Error applying impulse: {e}")
    
    def get_location(self):
        """차량 위치 반환"""
        try:
            if self.vehicle:
                return self.vehicle.get_location()
            return None
        except Exception as e:
            print(f"⚠️ Error getting vehicle location: {e}")
            return None
    
    def get_velocity(self):
        """차량 속도 반환"""
        try:
            if self.vehicle:
                return self.vehicle.get_velocity()
            return None
        except Exception as e:
            print(f"⚠️ Error getting vehicle velocity: {e}")
            return None
    
    def get_transform(self):
        """차량 변환 행렬 반환"""
        try:
            if self.vehicle:
                return self.vehicle.get_transform()
            return None
        except Exception as e:
            print(f"⚠️ Error getting vehicle transform: {e}")
            return None
    
    def get_speed(self):
        """차량 속도 (km/h) 반환"""
        try:
            velocity = self.get_velocity()
            if velocity:
                speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6
                return speed
            return 0.0
        except Exception as e:
            print(f"⚠️ Error getting vehicle speed: {e}")
            return 0.0
    
    def spawn_chase_vehicle(self, manual_vehicle_location):
        """추격차량 스폰 (수동차량 위치에서 Y축으로 50만큼 뒤에)"""
        try:
            # 추격차량 블루프린트 선택
            chase_bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
            if chase_bp is None:
                chase_bp = self.world.get_blueprint_library().find('vehicle.audi.tt')
            if chase_bp is None:
                chase_bp = self.world.get_blueprint_library().find('vehicle.volkswagen.t2')
            
            if chase_bp is None:
                raise RuntimeError("No suitable chase vehicle blueprint found")
                
            chase_bp.set_attribute('role_name', 'chase_vehicle')
            
            # 추격차량 색상 설정 (다른 색상으로)
            try:
                chase_color = random.choice(chase_bp.get_attribute('color').recommended_values)
                chase_bp.set_attribute('color', chase_color)
            except:
                print("Could not set chase vehicle color")
            
            # 추격차량 물리 속성 설정
            if chase_bp.has_attribute('mass'):
                chase_bp.set_attribute('mass', '1500.0')
            if chase_bp.has_attribute('gravity_scale'):
                chase_bp.set_attribute('gravity_scale', '1.0')
            
            # 추격차량 위치 (수동차량에서 Y축으로 50만큼 뒤)
            chase_location = carla.Location(
                manual_vehicle_location.x,
                manual_vehicle_location.y - 50.0,  # Y축으로 50만큼 뒤
                manual_vehicle_location.z
            )
            
            # 추격차량 회전 (수동차량과 같은 방향)
            chase_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
            chase_transform = carla.Transform(chase_location, chase_rotation)
            
            # 추격차량 스폰
            self.chase_vehicle = self.world.try_spawn_actor(chase_bp, chase_transform)
            
            if self.chase_vehicle is None:
                raise RuntimeError("Failed to spawn chase vehicle")
            
            print(f"🚔 Chase vehicle spawned at: X={chase_location.x:.2f}, Y={chase_location.y:.2f}, Z={chase_location.z:.2f}")
            
            return self.chase_vehicle
            
        except Exception as e:
            print(f"❌ Error spawning chase vehicle: {e}")
            return None
    
    def get_chase_vehicle_location(self):
        """추격차량 위치 반환"""
        try:
            if hasattr(self, 'chase_vehicle') and self.chase_vehicle:
                return self.chase_vehicle.get_location()
            return None
        except Exception as e:
            print(f"⚠️ Error getting chase vehicle location: {e}")
            return None
    
    def destroy_chase_vehicle(self):
        """추격차량 정리"""
        try:
            if hasattr(self, 'chase_vehicle') and self.chase_vehicle:
                self.chase_vehicle.destroy()
                self.chase_vehicle = None
                print("🚔 Chase vehicle destroyed")
        except Exception as e:
            print(f"⚠️ Error destroying chase vehicle: {e}")

    def handle_input(self):
        """키보드 입력 처리 (시나리오에서 호출)"""
        # 이 메서드는 시나리오의 키보드 입력 처리에서 호출됩니다
        # 실제 제어는 update_control에서 처리됩니다
        pass
    
    def cleanup(self):
        """리소스 정리"""
        self.destroy()
    
    def destroy(self):
        """차량 정리"""
        if self.vehicle:
            self.vehicle.destroy()
            self.vehicle = None
            print("🚗 Vehicle destroyed")
