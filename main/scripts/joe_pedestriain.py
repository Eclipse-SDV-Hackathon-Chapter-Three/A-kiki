#!/usr/bin/env python3

import carla
import random
import math
import time
import threading
import os

# 기존 모듈들 import
import sys
import os

# 상위 디렉토리의 pedestrian 모듈을 import하기 위해 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from pedestrian import PedestrianController, get_pedestrian_model, list_available_models

class PedestrianWalkingScenario:
    """Simple pedestrian walking scenario without vehicles"""
    
    def __init__(self):
        
        # CARLA 클라이언트 설정
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        
        # 시나리오 설정
        self.running = False
        
        # 보행자 설정 - 더 자연스러운 위치 (도로 위)
        self.pedestrian_start = carla.Location(-120.0, 38.65, 1.10)
        self.pedestrian_end = carla.Location(-95.0, 38.65, 1.10)
        self.pedestrian_controller = None  # 나중에 초기화
        
        # 보행자 모델 선택 (pedestrian_config.py에서 설정)
        self.preferred_pedestrian = get_pedestrian_model()  # 설정 파일에서 가져오기
        
        # 사용 가능한 보행자 모델 목록 출력
        print("👥 Available pedestrian models:")
        list_available_models()
        print(f"🎯 Selected pedestrian: {self.preferred_pedestrian or 'Random'}")
        
        
        
        
        # 이전 프레임 시간
        self.last_time = time.time()
        
        print("🚀 Pedestrian Walking Scenario initialized")

    def _pick_navigation_segment(self, min_distance=10.0, max_attempts=15):
        """Pick two navigation points with reasonable separation"""
        for _ in range(max_attempts):
            start = self.world.get_random_location_from_navigation()
            end = self.world.get_random_location_from_navigation()
            if start and end and start.distance(end) >= min_distance:
                return start, end
        return None, None

    def setup_carla_world(self):
        """CARLA 월드 설정"""
        try:
            # Traffic Manager 비활성화 (RPC 서버 오류 방지)
            try:
                traffic_manager = self.client.get_trafficmanager()
                traffic_manager.set_global_distance_to_leading_vehicle(0.0)
                print("🚦 Traffic Manager configured")
            except Exception as e:
                print(f"⚠️ Traffic Manager configuration failed (continuing): {e}")
            
            # 월드 설정
            weather = carla.WeatherParameters(
                cloudiness=10.0,
                precipitation=0.0,
                sun_altitude_angle=70.0
            )
            self.world.set_weather(weather)
            
            print("🌍 CARLA world setup completed")
            
        except Exception as e:
            print(f"⚠️ Error setting up CARLA world: {e}")
            raise

    def spawn_actors(self):
        """모든 액터 스폰"""
        try:
            # 보행자 컨트롤러 초기화 및 스폰
            print("🚶 Initializing and spawning pedestrian...")
            try:
                self.pedestrian_controller = PedestrianController(
                    self.world,
                    self.pedestrian_start,
                    self.pedestrian_end,
                    walking_speed=2,
                    preferred_walker=self.preferred_pedestrian
                )
                self.pedestrian_controller.spawn_pedestrian()
            except Exception as initial_error:
                print(f"⚠️ Initial pedestrian spawn failed: {initial_error}")
                print("🔄 Trying random navigation locations...")
                nav_start, nav_end = self._pick_navigation_segment()
                if not nav_start or not nav_end:
                    raise RuntimeError("Failed to find navigation locations for pedestrian spawn")
                self.pedestrian_start = nav_start
                self.pedestrian_end = nav_end
                print(f"➡️ New pedestrian path: start={self.pedestrian_start}, end={self.pedestrian_end}")
                self.pedestrian_controller = PedestrianController(
                    self.world,
                    self.pedestrian_start,
                    self.pedestrian_end,
                    walking_speed=2,
                    preferred_walker=self.preferred_pedestrian
                )
                self.pedestrian_controller.spawn_pedestrian()

            print("✅ All actors spawned successfully")
            
        except Exception as e:
            print(f"❌ Error spawning actors: {e}")
            raise



    def setup_spectator(self):
        """스펙테이터 설정"""
        try:
            pass
            # spectator = self.world.get_spectator()
            # 
            # 스펙테이터를 보행자 근처에 위치
            # spectator_location = carla.Location(
                # self.pedestrian_start.x, 
                # self.pedestrian_start.y, 
                # self.pedestrian_start.z + 10.0
            # )
            # spectator_rotation = carla.Rotation(pitch=-45, yaw=0, roll=0)
            # spectator_transform = carla.Transform(spectator_location, spectator_rotation)
            # spectator.set_transform(spectator_transform)
            
            print("👁️ Spectator setup completed")
            
        except Exception as e:
            print(f"⚠️ Error setting up spectator: {e}")


    def update_pedestrian_walking(self, delta_time):
        """보행자 워킹 애니메이션 업데이트"""
        if self.pedestrian_controller:
            self.pedestrian_controller.update_walking(delta_time)



    def display_status(self):
        """상태 표시"""
        try:
            if not self.pedestrian_controller:
                return
                
            pedestrian_location = self.pedestrian_controller.get_location()
            
            if pedestrian_location:
                # 터미널에 상태 출력 (1초마다)
                current_time = time.time()
                if not hasattr(self, 'last_status_print') or current_time - self.last_status_print > 1.0:
                    self.last_status_print = current_time
                    
                    print(f"🚶 Pedestrian: X={pedestrian_location.x:.2f}, Y={pedestrian_location.y:.2f}, Z={pedestrian_location.z:.2f}")
                    print("=" * 60)
            
        except Exception as e:
            print(f"⚠️ Error displaying status: {e}")

    def display_initial_positions(self):
        """초기 위치 표시"""
        try:
            if not self.pedestrian_controller:
                print("⚠️ Controllers not initialized yet")
                return
                
            pedestrian_location = self.pedestrian_controller.get_location()
            
            if pedestrian_location:
                print("=" * 80)
                print("PEDESTRIAN WALKING SCENARIO")
                print("=" * 80)
                print(f"🚶 PEDESTRIAN Start: X={self.pedestrian_start.x:.2f}, Y={self.pedestrian_start.y:.2f}, Z={self.pedestrian_start.z:.2f}")
                print(f"🚶 PEDESTRIAN End: X={self.pedestrian_end.x:.2f}, Y={self.pedestrian_end.y:.2f}, Z={self.pedestrian_end.z:.2f}")
                print(f"🚶 PEDESTRIAN Current: X={pedestrian_location.x:.2f}, Y={pedestrian_location.y:.2f}, Z={pedestrian_location.z:.2f}")
                print(f"🚶 Walking Distance: {self.pedestrian_start.distance(self.pedestrian_end):.2f}m")
                print("=" * 80)
                
        except Exception as e:
            print(f"⚠️ Error displaying initial positions: {e}")

    def run_scenario(self):
        """시나리오 실행"""
        try:
            print("🚀 Starting Pedestrian Walking Scenario...")
            
            # CARLA 월드 설정
            self.setup_carla_world()
            
            # 액터들 스폰
            self.spawn_actors()
            
            # 센서들 설정
            self.setup_spectator()
            
            # 초기 위치 표시
            self.display_initial_positions()
            
            # 시나리오 시작
            self.running = True
            print("=" * 80)
            print("PEDESTRIAN WALKING SCENARIO STARTED!")
            print("=" * 80)
            print("NOTE: Pedestrian will walk automatically between start and end points!")
            print("Press Ctrl+C to exit")
            print("=" * 80)
            
            
            # 메인 루프
            running = True
            last_time = time.time()
            
            while running:
                try:
                    current_time = time.time()
                    delta_time = current_time - last_time
                    last_time = current_time
                    
                    # 보행자 워킹 애니메이션 업데이트
                    if self.pedestrian_controller:
                        self.update_pedestrian_walking(delta_time)
                    
                    # 상태 표시
                    self.display_status()
                    
                    # 월드 업데이트
                    self.world.wait_for_tick()
                    
                    # 간단한 FPS 제한 (60 FPS)
                    time.sleep(1.0/60.0)
                    
                except KeyboardInterrupt:
                    print("\n🛑 Scenario interrupted by user")
                    break
                except Exception as e:
                    print(f"⚠️ Error in main loop: {e}")
                    break
            
            print("✅ Pedestrian Walking Scenario completed!")
            
        except Exception as e:
            print(f"❌ Error running scenario: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """리소스 정리"""
        try:
            print("🧹 Cleaning up...")
            
            self.running = False
            
            # 컨트롤러들 정리
            if hasattr(self, 'pedestrian_controller'):
                self.pedestrian_controller.destroy()
            
            
            print("✅ Cleanup completed")
            
        except Exception as e:
            print(f"⚠️ Error during cleanup: {e}")

def main():
    """메인 함수"""
    try:
        # 시나리오 생성 및 실행
        scenario = PedestrianWalkingScenario()
        scenario.run_scenario()
        
    except Exception as e:
        print(f"❌ Error in main: {e}")

if __name__ == "__main__":
    main()
