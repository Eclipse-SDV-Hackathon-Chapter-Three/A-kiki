import carla
import time
import math
import random
import threading

class PedestrianController:
    """보행자 생성 및 제어 클래스 (pedestrian_walking_scenario.py에서 완전히 추출)"""
    
    def __init__(self, world, pedestrian_start, pedestrian_end, walking_speed=1.0, preferred_walker=None):
        self.world = world
        self.pedestrian = None
        self.walker_controller = None
        
        # 보행자 위치 설정
        self.pedestrian_start = pedestrian_start
        self.pedestrian_end = pedestrian_end
        self.walking_speed = walking_speed
        self.walking_direction = 1  # 1: 시작점에서 끝점으로, -1: 끝점에서 시작점으로
        self.walking_distance = abs(self.pedestrian_end.x - self.pedestrian_start.x)
        self.walking_progress = 0.0  # 0.0 ~ 1.0
        
        # 충돌 관련
        self.collision_detected = False
        self.collision_count = 0
        
        # 진행률 출력 관련
        self.last_progress_percent = -1
        
        # 선호하는 보행자 모델
        self.preferred_walker = preferred_walker
        
    def spawn_pedestrian(self):
        """보행자 스폰 (시작 위치) 및 컨트롤러 설정"""
        # 다양한 보행자 블루프린트 시도 (더 많은 옵션)
        walker_options = [
            'walker.pedestrian.0001',  # 기본 남성
            'walker.pedestrian.0002',  # 기본 여성
            'walker.pedestrian.0003',  # 다른 남성
            'walker.pedestrian.0004',  # 다른 여성
            'walker.pedestrian.0005',  # 다양한 스타일
            'walker.pedestrian.0006',
            'walker.pedestrian.0007',
            'walker.pedestrian.0008',
            'walker.pedestrian.0009',
            'walker.pedestrian.0010',
            'walker.pedestrian.0011',  # 추가 옵션들
            'walker.pedestrian.0012',
            'walker.pedestrian.0013',
            'walker.pedestrian.0014',
            'walker.pedestrian.0015',
            'walker.pedestrian.0016',
            'walker.pedestrian.0017',
            'walker.pedestrian.0018',
            'walker.pedestrian.0019',
            'walker.pedestrian.0020',
            'walker.pedestrian.0021',
            'walker.pedestrian.0022',
            'walker.pedestrian.0023',
            'walker.pedestrian.0024',
            'walker.pedestrian.0025',
            'walker.pedestrian.0026',
            'walker.pedestrian.0027',
            'walker.pedestrian.0028',
            'walker.pedestrian.0029',
            'walker.pedestrian.0030'
        ]
        
        # 먼저 사용 가능한 모든 보행자 확인
        available_walkers = []
        blueprint_library = self.world.get_blueprint_library()
        
        print("🔍 Checking available pedestrian models...")
        for walker_id in walker_options:
            walker_bp = blueprint_library.find(walker_id)
            if walker_bp is not None:
                available_walkers.append(walker_id)
                print(f"  ✅ {walker_id} - Available")
            else:
                print(f"  ❌ {walker_id} - Not available")
        
        # 보행자 선택 로직
        if available_walkers:
            if self.preferred_walker and self.preferred_walker in available_walkers:
                # 선호하는 보행자가 사용 가능한 경우
                selected_walker = self.preferred_walker
                walker_bp = blueprint_library.find(selected_walker)
                print(f"⭐ Using preferred walker: {selected_walker}")
            else:
                # 랜덤 선택
                import random
                selected_walker = random.choice(available_walkers)
                walker_bp = blueprint_library.find(selected_walker)
                print(f"🎲 Randomly selected walker: {selected_walker}")
        else:
            # 모든 walker 시도 실패 시 기본 walker 찾기
            walker_bp = blueprint_library.find('walker.pedestrian')
            if walker_bp is None:
                raise RuntimeError("No suitable walker blueprint found")
            selected_walker = "walker.pedestrian (default)"
            print(f"⚠️ Using default walker: {selected_walker}")
        
        print(f"👤 Final selection: {selected_walker}")
        
        # 보행자 시작 위치
        walker_rotation = carla.Rotation(0, 0, 0)
        walker_transform = carla.Transform(self.pedestrian_start, walker_rotation)
        
        # 보행자 블루프린트 물리 설정
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        if walker_bp.has_attribute('simulate_physics'):
            walker_bp.set_attribute('simulate_physics', 'false')  # 걷는 동안은 비활성화
        if walker_bp.has_attribute('mass'):
            walker_bp.set_attribute('mass', '70.0')
        if walker_bp.has_attribute('gravity_scale'):
            walker_bp.set_attribute('gravity_scale', '1.0')
        
        print(f"Attempting to spawn pedestrian at: {self.pedestrian_start}")
        self.pedestrian = self.world.try_spawn_actor(walker_bp, walker_transform)
        
        if self.pedestrian is None:
            # 다른 위치에서 시도
            print("First pedestrian spawn attempt failed, trying alternative location...")
            alt_location = carla.Location(-119.15, 38.65, 1.2)
            walker_transform = carla.Transform(alt_location, walker_rotation)
            self.pedestrian = self.world.try_spawn_actor(walker_bp, walker_transform)
            
            if self.pedestrian is None:
                raise RuntimeError("Failed to spawn pedestrian")
        
        # 보행자 컨트롤러 생성
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        self.walker_controller = self.world.try_spawn_actor(walker_controller_bp, carla.Transform(), self.pedestrian)
        
        if self.walker_controller is None:
            raise RuntimeError("Failed to spawn walker controller")
        
        # 컨트롤러 시작
        self.walker_controller.start()
        
        # 보행자 속도 설정 (걷기 속도) - 우리가 설정한 속도 강제 사용
        self.walker_controller.set_max_speed(self.walking_speed)
        print(f"🚶 Walker speed set to: {self.walking_speed} m/s (forced)")
        
        # 추가 속도 설정 (혹시 모를 경우를 대비)
        try:
            # walker controller의 다른 속도 관련 메서드들도 시도
            if hasattr(self.walker_controller, 'set_speed'):
                self.walker_controller.set_speed(self.walking_speed)
                print(f"🚶 Additional speed set via set_speed: {self.walking_speed} m/s")
        except Exception as e:
            print(f"⚠️ Could not set additional speed: {e}")
        
        # 초기 목표 설정 (끝점으로 걷기 시작)
        self.walker_controller.go_to_location(self.pedestrian_end)
        
        # 보행자 물리 설정 (걷는 동안은 비활성화)
        self.pedestrian.set_simulate_physics(False)  # 걷는 동안은 물리 시뮬레이션 비활성화
        
        # 보행자 질량 설정 (더 현실적인 충돌을 위해)
        try:
            self.pedestrian.set_mass(70.0)  # 70kg으로 설정
        except:
            pass
        
        
        print(f"Pedestrian spawned successfully at: {self.pedestrian.get_location()}")
        print(f"Walker controller started with speed: {walker_speed if 'walker_speed' in locals() else 'default'}")
        print(f"Initial target: {self.pedestrian_end}")
        print("⚡ Pedestrian physics simulation DISABLED during walking (will be enabled on collision)")
        return self.pedestrian
    
    def update_walking(self, delta_time, toggle_walking_key_pressed=False):
        """보행자 워킹 애니메이션 업데이트 (AI 컨트롤러 사용)"""
        if self.pedestrian is None or self.walker_controller is None:
            return
        
        # 주기적으로 속도 재설정 (혹시 다른 곳에서 변경되었을 경우를 대비)
        if not hasattr(self, 'last_speed_check') or time.time() - self.last_speed_check > 2.0:
            self.walker_controller.set_max_speed(self.walking_speed)
            self.last_speed_check = time.time()
            
        # 워킹 토글 처리 (한 번만 실행되도록)
        if toggle_walking_key_pressed:
            self.walking_direction *= -1  # 방향 반전
            print(f"Walking direction changed to: {'Forward' if self.walking_direction > 0 else 'Backward'}")
            
            # 새로운 목표 위치 설정
            if self.walking_direction > 0:
                target_location = self.pedestrian_end
            else:
                target_location = self.pedestrian_start
            
            self.walker_controller.go_to_location(target_location)
            print(f"🚶 New target: {target_location}")
        
        # 현재 위치 확인 및 진행률 계산
        current_location = self.pedestrian.get_location()
        
        # 시작점에서 끝점까지의 거리 계산
        total_distance = math.sqrt(
            (self.pedestrian_end.x - self.pedestrian_start.x)**2 + 
            (self.pedestrian_end.y - self.pedestrian_start.y)**2
        )
        
        # 현재 진행률 계산
        current_distance = math.sqrt(
            (current_location.x - self.pedestrian_start.x)**2 + 
            (current_location.y - self.pedestrian_start.y)**2
        )
        
        if total_distance > 0:
            self.walking_progress = min(current_distance / total_distance, 1.0)
        
        # 진행률 출력 (10% 단위로)
        progress_percent = int(self.walking_progress * 100)
        if progress_percent % 10 == 0 and progress_percent != self.last_progress_percent:
            print(f"🚶 Walking Progress: {progress_percent}% ({current_location.x:.2f}, {current_location.y:.2f}, {current_location.z:.2f})")
            self.last_progress_percent = progress_percent
        
        # 목표에 도달했는지 확인하고 방향 전환 (더 정확한 거리 체크)
        distance_to_target = math.sqrt(
            (current_location.x - self.pedestrian_end.x)**2 + 
            (current_location.y - self.pedestrian_end.y)**2
        )
        
        if distance_to_target < 1.5 and self.walking_direction > 0:  # 끝점에 도달 (거리 줄임)
            self.walking_direction = -1
            self.walker_controller.go_to_location(self.pedestrian_start)
            print("🚶 Reached end point, walking back to start")
        
        distance_to_start = math.sqrt(
            (current_location.x - self.pedestrian_start.x)**2 + 
            (current_location.y - self.pedestrian_start.y)**2
        )
        
        if distance_to_start < 1.5 and self.walking_direction < 0:  # 시작점에 도달 (거리 줄임)
            self.walking_direction = 1
            self.walker_controller.go_to_location(self.pedestrian_end)
            print("🚶 Reached start point, walking to end")
    
    def apply_physics_effect(self, vehicle_velocity):
        """보행자에 물리 효과 적용 (충돌 시)"""
        if self.pedestrian is None:
            return
        
        try:
            # 보행자 물리 시뮬레이션 활성화
            self.pedestrian.set_simulate_physics(True)
            
            # 차량 속도에 따른 충격 계산
            vehicle_speed = math.sqrt(vehicle_velocity.x**2 + vehicle_velocity.y**2 + vehicle_velocity.z**2)
            
            # 보행자 위치와 차량 위치 계산
            pedestrian_location = self.pedestrian.get_location()
            
            # 충격 방향 계산 (차량에서 보행자로) - 더 정확한 방향
            direction_x = pedestrian_location.x - (pedestrian_location.x - 5.0)  # 대략적인 차량 위치
            direction_y = pedestrian_location.y - (pedestrian_location.y - 5.0)
            direction_z = 0.5  # 위쪽으로 약간의 힘만 (지면 아래로 떨어지지 않도록)
            
            # 정규화
            length = math.sqrt(direction_x**2 + direction_y**2 + direction_z**2)
            if length > 0:
                direction_x /= length
                direction_y /= length
                direction_z /= length
            
            # 충격력 계산 (더 약하게)
            impact_force = min(vehicle_speed * 1.0, 8.0)  # 최대 8.0으로 제한 (더 약하게)
            
            # 충격 적용 (지면 위에 유지되도록)
            impulse = carla.Vector3D(
                direction_x * impact_force,
                direction_y * impact_force,
                direction_z * impact_force + 1.0  # 위쪽으로 약간의 힘만
            )
            self.pedestrian.add_impulse(impulse)
            
            # 랜덤 회전 적용 (넘어진 자세)
            random_rotation = carla.Rotation(
                random.uniform(-30, 30),  # 더 작은 회전
                random.uniform(-90, 90),  # 더 작은 회전
                random.uniform(-30, 30)   # 더 작은 회전
            )
            current_transform = self.pedestrian.get_transform()
            new_transform = carla.Transform(
                current_transform.location,
                carla.Rotation(
                    current_transform.rotation.pitch + random_rotation.pitch,
                    current_transform.rotation.yaw + random_rotation.yaw,
                    current_transform.rotation.roll + random_rotation.roll
                )
            )
            self.pedestrian.set_transform(new_transform)
            
            print(f"💥 Physics effect applied to pedestrian! Force: {impact_force:.2f}")
            
            # 보행자 컨트롤러 중지 (다시 걷지 않음)
            if self.walker_controller:
                self.walker_controller.stop()
                print("🚶 Walker controller stopped permanently")
            
            # 물리 시뮬레이션을 계속 활성화 상태로 유지 (넘어진 상태 유지)
            print("🚶 Pedestrian will remain in fallen state")
            
        except Exception as e:
            print(f"⚠️ Error applying physics effect to pedestrian: {e}")
    
    def get_location(self):
        """보행자 현재 위치 반환"""
        return self.pedestrian.get_location() if self.pedestrian else None
    
    def cleanup(self):
        """리소스 정리"""
        self.destroy()
    
    def destroy(self):
        """보행자 및 컨트롤러 정리"""
        if self.walker_controller:
            try:
                self.walker_controller.stop()
                self.walker_controller.destroy()
            except:
                pass
        
        if self.pedestrian:
            try:
                self.pedestrian.destroy()
            except:
                pass
        
        print("🚶 Pedestrian and controller destroyed")
