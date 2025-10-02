#!/usr/bin/env python3
"""
Vehicle Manager
차량 관리 클래스
"""

import carla
import time

class VehicleManager:
    """차량 관리 클래스"""
    
    def __init__(self, world, vehicle=None):
        self.world = world
        self.vehicle = vehicle
        self.vehicle_id = None
        
    def find_existing_vehicle(self):
        """기존에 존재하는 차량을 찾아서 제어"""
        try:
            print("🔍 Searching for existing vehicles...")
            # 모든 차량 액터 찾기
            vehicles = self.world.get_actors().filter('vehicle.*')
            print(f"🔍 Found {len(vehicles)} vehicles in the world")
            
            if not vehicles:
                print("❌ No vehicles found in the world")
                return False
            
            # 각 차량 정보 출력
            for i, vehicle in enumerate(vehicles):
                print(f"  {i+1}. {vehicle.type_id} (ID: {vehicle.id})")
                print(f"      위치: {vehicle.get_location()}")
                print(f"      제어 가능: {vehicle.is_alive}")
                print(f"      role_name: {vehicle.attributes.get('role_name', 'None')}")
                print(f"      type_id에 police 포함: {'police' in vehicle.type_id}")
                print()
            
            # 첫 번째 차량 선택 (또는 특정 조건으로 필터링)
            for vehicle in vehicles:
                if vehicle.attributes.get('role_name') == 'police' or 'police' in vehicle.type_id:
                    self.vehicle = vehicle
                    self.vehicle_id = vehicle.id
                    print(f"🚗 Found police vehicle: {vehicle.type_id} (ID: {vehicle.id}) at {vehicle.get_location()}")
                    return True
            
            # 경찰차가 없으면 첫 번째 차량 사용
            self.vehicle = vehicles[0]
            self.vehicle_id = self.vehicle.id
            print(f"🚗 Found vehicle: {self.vehicle.type_id} (ID: {self.vehicle_id}) at {self.vehicle.get_location()}")
            print(f"✅ Using existing vehicle: {self.vehicle.type_id} (ID: {self.vehicle_id})")
            print(f"📍 Vehicle location: {self.vehicle.get_location()}")
            return True
            
        except Exception as e:
            print(f"❌ Error finding existing vehicle: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def get_vehicle_info(self):
        """차량 정보 반환"""
        if not self.vehicle:
            return None
        
        try:
            location = self.vehicle.get_location()
            rotation = self.vehicle.get_rotation()
            velocity = self.vehicle.get_velocity()
            
            return {
                'id': self.vehicle_id,
                'type': self.vehicle.type_id,
                'location': location,
                'rotation': rotation,
                'velocity': velocity,
                'speed': (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
            }
        except Exception as e:
            print(f"⚠️ Error getting vehicle info: {e}")
            return None
    
    def apply_control(self, control_command):
        """차량에 제어 명령 적용"""
        try:
            if not self.vehicle:
                return False
            
            self.vehicle.apply_control(control_command)
            return True
            
        except Exception as e:
            print(f"⚠️ Error applying control: {e}")
            return False
    
    def get_control(self):
        """현재 차량 제어 상태 반환"""
        try:
            if not self.vehicle:
                return None
            
            return self.vehicle.get_control()
        except Exception as e:
            print(f"⚠️ Error getting control: {e}")
            return None

