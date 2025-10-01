#!/usr/bin/env python3

"""
Zenoh Chase Controller
Zenoh를 통한 추격 제어
"""

import time
from typing import Dict, Any, Optional
from .chase_planner import ChasePlanner
from ..communication.zenoh_manager import ZenohManager

class ZenohChaseController:
    """Zenoh 기반 추격 제어자"""
    
    def __init__(self, world, vehicle, zenoh_manager: ZenohManager):
        self.world = world
        self.vehicle = vehicle
        self.zenoh_manager = zenoh_manager
        
        # 추격 계획자
        self.chase_planner = ChasePlanner(world, vehicle, world.get_map())
        
        # 추격 상태
        self.is_chasing = False
        self.target_position = None
        self.target_velocity = None
        self.last_update_time = 0
        
        # Zenoh 구독 설정
        self._setup_subscriptions()
        
        print("🎯 Zenoh Chase Controller initialized")
    
    def _setup_subscriptions(self):
        """Zenoh 구독 설정"""
        # 감지 데이터 구독
        self.zenoh_manager.subscribe('detection', self._on_detection_data)
        
        # 충돌 데이터 구독
        self.zenoh_manager.subscribe('collision', self._on_collision_data)
        
        # 추격 명령 구독
        self.zenoh_manager.subscribe('chase_command', self._on_chase_command)
    
    def _on_detection_data(self, data: Dict[str, Any]):
        """감지 데이터 처리"""
        try:
            detected_objects = data.get('detected_objects', [])
            collision_events = data.get('collision_events', [])
            
            # 차량 감지 확인
            vehicles = [obj for obj in detected_objects if obj.get('type') == 'vehicle']
            
            if vehicles and not self.is_chasing:
                # 첫 번째 차량을 타겟으로 설정
                target = vehicles[0]
                self._start_chase(target)
            
            # 추격 중이면 업데이트
            if self.is_chasing:
                self._update_chase(detected_objects, collision_events)
                
        except Exception as e:
            print(f"⚠️ Error processing detection data: {e}")
    
    def _on_collision_data(self, data: Dict[str, Any]):
        """충돌 데이터 처리"""
        try:
            events = data.get('events', [])
            if events:
                print(f"🚨 Collision events received: {len(events)} events")
                
                # 충돌이 감지되면 추격 시작
                if not self.is_chasing:
                    self._start_emergency_chase(events)
                    
        except Exception as e:
            print(f"⚠️ Error processing collision data: {e}")
    
    def _on_chase_command(self, data: Dict[str, Any]):
        """추격 명령 처리"""
        try:
            command = data.get('command')
            
            if command == 'start':
                self._start_chase(data.get('target'))
            elif command == 'stop':
                self._stop_chase()
            elif command == 'pause':
                self._pause_chase()
            elif command == 'resume':
                self._resume_chase()
                
        except Exception as e:
            print(f"⚠️ Error processing chase command: {e}")
    
    def _start_chase(self, target: Dict[str, Any]):
        """추격 시작"""
        try:
            if not target:
                return
            
            self.target_position = target.get('world_location')
            self.target_velocity = target.get('velocity', 0.0)
            self.is_chasing = True
            
            print(f"🎯 Chase started - Target: {target.get('actor_id')}")
            
            # 상태 발행
            self._publish_status()
            
        except Exception as e:
            print(f"⚠️ Error starting chase: {e}")
    
    def _start_emergency_chase(self, collision_events: List[Dict[str, Any]]):
        """비상 추격 시작"""
        try:
            # 충돌 지점에서 가장 가까운 차량 찾기
            if collision_events:
                print("🚨 Emergency chase started due to collision")
                self.is_chasing = True
                self._publish_status()
                
        except Exception as e:
            print(f"⚠️ Error starting emergency chase: {e}")
    
    def _update_chase(self, detected_objects: List[Dict[str, Any]], collision_events: List[Dict[str, Any]]):
        """추격 업데이트"""
        try:
            current_time = time.time()
            
            # 업데이트 간격 체크 (50ms)
            if current_time - self.last_update_time < 0.05:
                return
            
            self.last_update_time = current_time
            
            # 타겟 찾기
            vehicles = [obj for obj in detected_objects if obj.get('type') == 'vehicle']
            if not vehicles:
                return
            
            # 가장 가까운 차량 선택
            target = min(vehicles, key=lambda x: x.get('distance', float('inf')))
            
            # 추격 계획 수립
            target_position = target.get('world_location')
            target_velocity = target.get('velocity', 0.0)
            target_distance = target.get('distance', 0.0)
            
            # 회피할 보행자 정보
            avoid_pedestrians = [event for event in collision_events if event.get('avoid_pedestrian', False)]
            
            # 추격 행동 계획
            control_command = self.chase_planner.plan_chase_behavior(
                target_position, target_velocity, target_distance, True, avoid_pedestrians
            )
            
            # 제어 명령 적용
            self._apply_control(control_command)
            
            # 상태 발행
            self._publish_status()
            
        except Exception as e:
            print(f"⚠️ Error updating chase: {e}")
    
    def _apply_control(self, control_command):
        """제어 명령 적용"""
        try:
            if isinstance(control_command, dict):
                # 딕셔너리 형태의 제어 명령
                control = carla.VehicleControl()
                control.throttle = control_command.get('throttle', 0.0)
                control.brake = control_command.get('brake', 0.0)
                control.steer = control_command.get('steer', 0.0)
                control.hand_brake = control_command.get('hand_brake', False)
                self.vehicle.apply_control(control)
            elif isinstance(control_command, carla.VehicleControl):
                # VehicleControl 객체
                self.vehicle.apply_control(control_command)
            
        except Exception as e:
            print(f"⚠️ Error applying control: {e}")
    
    def _stop_chase(self):
        """추격 중지"""
        self.is_chasing = False
        self.target_position = None
        self.target_velocity = None
        self.chase_planner.reset_chase()
        
        # 정지 명령
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = 1.0
        control.steer = 0.0
        control.hand_brake = True
        self.vehicle.apply_control(control)
        
        print("🛑 Chase stopped")
        self._publish_status()
    
    def _pause_chase(self):
        """추격 일시 정지"""
        print("⏸️ Chase paused")
        self._publish_status()
    
    def _resume_chase(self):
        """추격 재개"""
        print("▶️ Chase resumed")
        self._publish_status()
    
    def _publish_status(self):
        """상태 발행"""
        try:
            status = {
                'timestamp': time.time(),
                'is_chasing': self.is_chasing,
                'target_position': self.target_position,
                'target_velocity': self.target_velocity,
                'chase_state': self.chase_planner.get_current_state().value,
                'vehicle_id': self.vehicle.id if self.vehicle else None
            }
            
            self.zenoh_manager.publish('vehicle_status', status)
            
        except Exception as e:
            print(f"⚠️ Error publishing status: {e}")
    
    def get_status(self):
        """현재 상태 반환"""
        return {
            'is_chasing': self.is_chasing,
            'target_position': self.target_position,
            'target_velocity': self.target_velocity,
            'chase_state': self.chase_planner.get_current_state().value
        }
