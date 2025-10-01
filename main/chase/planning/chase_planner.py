#!/usr/bin/env python3

"""
Advanced Chase Planner with CARLA Agents
CARLA agents를 사용한 고급 추격 계획자 - 차선 유지 포함
"""

import carla
import numpy as np
import math
import time
from typing import Dict, List, Optional, Tuple
from enum import Enum

# CARLA agents 모듈 import (로컬 복사본 사용)
from .agents.navigation.local_planner import LocalPlanner, RoadOption
from .agents.navigation.global_route_planner import GlobalRoutePlanner
from .agents.navigation.controller import VehiclePIDController
from .agents.tools.misc import get_speed, is_within_distance, compute_distance

class ChaseState(Enum):
    """추격 상태"""
    IDLE = "idle"                    # 대기
    SEARCHING = "searching"          # 탐색
    APPROACHING = "approaching"      # 접근
    FOLLOWING = "following"          # 추격
    INTERCEPTING = "intercepting"    # 차단
    EMERGENCY = "emergency"          # 비상

class ChasePlanner:
    """CARLA agents를 사용한 고급 추격 계획자 - 차선 유지 포함"""
    
    def __init__(self, world, chase_vehicle, map_inst=None):
        self.world = world
        self.chase_vehicle = chase_vehicle
        self.map = map_inst if map_inst else world.get_map()
        
        # 추격 파라미터
        self.follow_distance = 15.0      # 추격 거리
        self.approach_distance = 50.0    # 접근 거리
        self.intercept_distance = 30.0   # 차단 거리
        self.emergency_distance = 5.0    # 비상 거리
        
        # 속도 제한 (m/s)
        self.max_speed = 25.0           # 최대 속도 (90km/h)
        self.approach_speed = 20.0      # 접근 속도 (72km/h)
        self.follow_speed = 18.0        # 추격 속도 (65km/h)
        self.intercept_speed = 22.0     # 차단 속도 (80km/h)
        
        # 차선 유지 파라미터
        self.lane_keeping_enabled = True
        self.lane_change_threshold = 5.0  # 차선 변경을 고려하는 거리
        self.safe_following_distance = 20.0  # 안전 추격 거리
        self.lane_offset = 0.0  # 차선 오프셋 (차선 유지)
        
        # 현재 상태
        self.current_state = ChaseState.IDLE
        self.target_position = None
        self.target_velocity = None
        self.last_state_change = time.time()
        
        # 회피 대상 (넘어진 보행자)
        self.avoid_pedestrians = []
        
        # CARLA agents 초기화
        self._init_agents()
        
        print("🎯 Advanced Chase Planner initialized with CARLA agents (Lane Keeping)")
    
    def _init_agents(self):
        """CARLA agents 초기화"""
        try:
            # Local Planner 설정 (차선 유지 포함)
            local_planner_dict = {
                'target_speed': self.max_speed * 3.6,  # km/h로 변환
                'sampling_radius': 2.0,
                'lateral_control_dict': {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': 0.05},
                'longitudinal_control_dict': {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': 0.05},
                'max_throttle': 0.8,
                'max_brake': 0.5,
                'max_steering': 0.8,
                'offset': self.lane_offset,  # 차선 오프셋 적용
                'base_min_distance': 3.0,
                'distance_ratio': 0.5,
                'follow_speed_limits': False  # 속도 제한 무시 (추격 중)
            }
            
            self.local_planner = LocalPlanner(
                self.chase_vehicle, 
                opt_dict=local_planner_dict, 
                map_inst=self.map
            )
            
            # Global Route Planner 설정
            self.global_planner = GlobalRoutePlanner(self.map, 2.0)
            
            # PID 컨트롤러 설정
            self.pid_controller = VehiclePIDController(
                self.chase_vehicle,
                args_lateral=local_planner_dict['lateral_control_dict'],
                args_longitudinal=local_planner_dict['longitudinal_control_dict'],
                offset=self.lane_offset,
                max_throttle=local_planner_dict['max_throttle'],
                max_brake=local_planner_dict['max_brake'],
                max_steering=local_planner_dict['max_steering']
            )
            
            print("✅ CARLA agents initialized successfully")
            
        except Exception as e:
            print(f"❌ Error initializing CARLA agents: {e}")
            raise
    
    def plan_chase_behavior(self, target_position, target_velocity, target_distance, is_target_visible, avoid_pedestrians=None):
        """추격 행동 계획 - CARLA agents 사용, 차선 유지 포함"""
        try:
            # 회피 대상 업데이트
            self.avoid_pedestrians = avoid_pedestrians or []
            
            # 상태 결정
            new_state = self._determine_chase_state(target_position, target_velocity, target_distance, is_target_visible)
            
            # 상태 변경 처리
            if new_state != self.current_state:
                self._handle_state_change(new_state)
            
            # 현재 상태에 따른 행동 계획
            if self.current_state == ChaseState.IDLE:
                return self._plan_idle_behavior()
            elif self.current_state == ChaseState.SEARCHING:
                return self._plan_searching_behavior()
            elif self.current_state == ChaseState.APPROACHING:
                return self._plan_approaching_behavior(target_position, target_velocity)
            elif self.current_state == ChaseState.FOLLOWING:
                return self._plan_following_behavior(target_position, target_velocity)
            elif self.current_state == ChaseState.INTERCEPTING:
                return self._plan_intercepting_behavior(target_position, target_velocity)
            elif self.current_state == ChaseState.EMERGENCY:
                return self._plan_emergency_behavior()
            
            return self._get_emergency_control()
            
        except Exception as e:
            print(f"⚠️ Error planning chase behavior: {e}")
            return self._get_emergency_control()
    
    def _determine_chase_state(self, target_position, target_velocity, target_distance, is_target_visible):
        """추격 상태 결정 - 최소거리까지 계속 쫓아가기"""
        try:
            if not is_target_visible or target_position is None:
                return ChaseState.SEARCHING
            
            # 최소거리까지 계속 쫓아가기 (멈추지 않음)
            if target_distance < self.emergency_distance:
                return ChaseState.EMERGENCY  # 매우 가까우면 비상상황
            elif target_distance < self.follow_distance:
                return ChaseState.APPROACHING  # FOLLOWING 대신 APPROACHING으로 계속 접근
            elif target_distance < self.approach_distance:
                return ChaseState.APPROACHING
            else:
                return ChaseState.INTERCEPTING
                
        except Exception as e:
            print(f"⚠️ Error determining chase state: {e}")
            return ChaseState.EMERGENCY
    
    def _handle_state_change(self, new_state):
        """상태 변경 처리"""
        if new_state != self.current_state:
            print(f"🎯 Chase state changed: {self.current_state.value} -> {new_state.value}")
            self.current_state = new_state
            self.last_state_change = time.time()
    
    def _plan_idle_behavior(self):
        """대기 행동 계획"""
        return self._get_emergency_control()
    
    def _plan_searching_behavior(self):
        """탐색 행동 계획 - 차선 유지하며 탐색"""
        try:
            # 현재 위치에서 차선을 따라 탐색
            current_location = self.chase_vehicle.get_location()
            current_waypoint = self.map.get_waypoint(current_location)
            
            # 차선을 따라 앞으로 50m 이동
            target_waypoint = current_waypoint.next(50.0)[0]
            target_location = target_waypoint.transform.location
            
            # 경로 설정 (차선 유지)
            self._set_destination_with_lane_keeping(target_location)
            
            # Local Planner로 제어
            return self.local_planner.run_step()
            
        except Exception as e:
            print(f"⚠️ Error in searching behavior: {e}")
            return self._get_emergency_control()
    
    def _plan_approaching_behavior(self, target_position, target_velocity):
        """접근 행동 계획 - 차선 유지하며 접근"""
        try:
            if target_position is None:
                return self._get_emergency_control()
            
            # 타겟 위치로 접근 (회피 로직 적용, 차선 유지)
            target_location = self._calculate_avoidance_path_with_lane_keeping(target_position)
            
            # 경로 설정
            self._set_destination_with_lane_keeping(target_location)
            
            # Local Planner로 제어
            return self.local_planner.run_step()
            
        except Exception as e:
            print(f"⚠️ Error in approaching behavior: {e}")
            return self._get_emergency_control()
    
    def _plan_following_behavior(self, target_position, target_velocity):
        """추격 행동 계획 - 차선 유지하며 추격"""
        try:
            if target_position is None:
                return self._get_emergency_control()
            
            # 타겟을 따라가되 안전 거리 유지, 차선 유지
            target_location = self._calculate_following_path_with_lane_keeping(target_position, target_velocity)
            
            # 경로 설정
            self._set_destination_with_lane_keeping(target_location)
            
            # Local Planner로 제어
            return self.local_planner.run_step()
            
        except Exception as e:
            print(f"⚠️ Error in following behavior: {e}")
            return self._get_emergency_control()
    
    def _plan_intercepting_behavior(self, target_position, target_velocity):
        """차단 행동 계획 - 차선 유지하며 차단"""
        try:
            if target_position is None:
                return self._get_emergency_control()
            
            # 타겟의 예상 위치로 차단 (차선 유지)
            intercept_location = self._calculate_intercept_path_with_lane_keeping(target_position, target_velocity)
            
            # 경로 설정
            self._set_destination_with_lane_keeping(intercept_location)
            
            # Local Planner로 제어
            return self.local_planner.run_step()
            
        except Exception as e:
            print(f"⚠️ Error in intercepting behavior: {e}")
            return self._get_emergency_control()
    
    def _plan_emergency_behavior(self):
        """비상 행동 계획"""
        try:
            # 긴급 정지
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            control.hand_brake = True
            return control
            
        except Exception as e:
            print(f"⚠️ Error in emergency behavior: {e}")
            return self._get_emergency_control()
    
    def _calculate_avoidance_path_with_lane_keeping(self, target_position):
        """회피 경로 계산 - 넘어진 사람을 피하되 차선 유지"""
        try:
            if not self.avoid_pedestrians:
                # 회피할 사람이 없으면 직접 접근
                return carla.Location(
                    x=target_position[0],
                    y=target_position[1],
                    z=target_position[2]
                )
            
            # 현재 차량 위치
            current_location = self.chase_vehicle.get_transform().location
            current_waypoint = self.map.get_waypoint(current_location)
            
            # 타겟 위치
            target_location = carla.Location(
                x=target_position[0],
                y=target_position[1],
                z=target_position[2]
            )
            target_waypoint = self.map.get_waypoint(target_location)
            
            # 회피할 사람들의 위치
            avoid_positions = []
            for pedestrian in self.avoid_pedestrians:
                if 'world_location' in pedestrian:
                    pos = pedestrian['world_location']
                    avoid_positions.append(carla.Location(x=pos[0], y=pos[1], z=pos[2]))
            
            if not avoid_positions:
                # 회피할 위치가 없으면 직접 접근
                return target_location
            
            # 차선을 고려한 회피 경로 계산
            best_avoidance_location = self._find_safe_lane_position(
                current_waypoint, target_waypoint, avoid_positions
            )
            
            print(f"🚶 AVOIDANCE PATH: Avoiding {len(avoid_positions)} fallen pedestrians with lane keeping")
            
            return best_avoidance_location
            
        except Exception as e:
            print(f"⚠️ Error calculating avoidance path: {e}")
            # 오류 시 직접 접근
            return carla.Location(
                x=target_position[0],
                y=target_position[1],
                z=target_position[2]
            )
    
    def _calculate_following_path_with_lane_keeping(self, target_position, target_velocity):
        """추격 경로 계산 - 안전 거리 유지하며 따라가기, 차선 유지"""
        try:
            # 타겟의 현재 위치
            target_location = carla.Location(
                x=target_position[0],
                y=target_position[1],
                z=target_position[2]
            )
            target_waypoint = self.map.get_waypoint(target_location)
            
            # 현재 차량 위치
            current_location = self.chase_vehicle.get_location()
            current_waypoint = self.map.get_waypoint(current_location)
            
            # 타겟의 속도 벡터
            if target_velocity is not None:
                # target_velocity가 단일 값인 경우 처리
                if isinstance(target_velocity, (int, float)):
                    velocity_magnitude = abs(target_velocity)
                    velocity_vector = np.array([0, 0, 0])  # 방향 정보 없음
                elif hasattr(target_velocity, '__len__') and len(target_velocity) >= 3:
                    velocity_vector = np.array(target_velocity[:3])
                    velocity_magnitude = np.linalg.norm(velocity_vector)
                else:
                    velocity_magnitude = 0
                    velocity_vector = np.array([0, 0, 0])
                
                if velocity_magnitude > 0:
                    # 타겟의 이동 방향 예측 (2초 후 위치)
                    future_position = target_location + carla.Location(
                        x=velocity_vector[0] * 2.0,
                        y=velocity_vector[1] * 2.0,
                        z=velocity_vector[2] * 2.0
                    )
                    future_waypoint = self.map.get_waypoint(future_position)
                    
                    # 안전 거리만큼 뒤에서 따라가기 (차선 유지)
                    follow_location = self._calculate_safe_following_position(
                        current_waypoint, future_waypoint, self.safe_following_distance
                    )
                    return follow_location
            
            # 속도 정보가 없으면 현재 위치에서 안전 거리 유지
            return self._calculate_safe_following_position(
                current_waypoint, target_waypoint, self.safe_following_distance
            )
            
        except Exception as e:
            print(f"⚠️ Error calculating following path: {e}")
            return carla.Location(
                x=target_position[0],
                y=target_position[1],
                z=target_position[2]
            )
    
    def _calculate_intercept_path_with_lane_keeping(self, target_position, target_velocity):
        """차단 경로 계산 - 타겟의 예상 위치로 이동, 차선 유지"""
        try:
            # 타겟의 현재 위치
            target_location = carla.Location(
                x=target_position[0],
                y=target_position[1],
                z=target_position[2]
            )
            target_waypoint = self.map.get_waypoint(target_location)
            
            # 타겟의 속도 벡터
            if target_velocity is not None:
                # target_velocity가 단일 값인 경우 처리
                if isinstance(target_velocity, (int, float)):
                    velocity_magnitude = abs(target_velocity)
                    velocity_vector = np.array([0, 0, 0])  # 방향 정보 없음
                elif hasattr(target_velocity, '__len__') and len(target_velocity) >= 3:
                    velocity_vector = np.array(target_velocity[:3])
                    velocity_magnitude = np.linalg.norm(velocity_vector)
                else:
                    velocity_magnitude = 0
                    velocity_vector = np.array([0, 0, 0])
                
                if velocity_magnitude > 0:
                    # 타겟의 이동 방향 예측 (5초 후 위치)
                    future_position = target_location + carla.Location(
                        x=velocity_vector[0] * 5.0,
                        y=velocity_vector[1] * 5.0,
                        z=velocity_vector[2] * 5.0
                    )
                    future_waypoint = self.map.get_waypoint(future_position)
                    
                    # 예상 위치로 이동 (차선 유지)
                    return future_waypoint.transform.location
            
            # 속도 정보가 없으면 현재 위치로 이동
            return target_waypoint.transform.location
            
        except Exception as e:
            print(f"⚠️ Error calculating intercept path: {e}")
            return carla.Location(
                x=target_position[0],
                y=target_position[1],
                z=target_position[2]
            )
    
    def _find_safe_lane_position(self, current_waypoint, target_waypoint, avoid_positions):
        """안전한 차선 위치 찾기"""
        try:
            # 현재 차선에서 타겟 방향으로 이동
            if current_waypoint.road_id == target_waypoint.road_id:
                # 같은 도로에서 차선 유지
                return target_waypoint.transform.location
            else:
                # 다른 도로로 이동 시 차선 변경 고려
                return self._plan_lane_change_to_target(current_waypoint, target_waypoint, avoid_positions)
                
        except Exception as e:
            print(f"⚠️ Error finding safe lane position: {e}")
            return target_waypoint.transform.location
    
    def _plan_lane_change_to_target(self, current_waypoint, target_waypoint, avoid_positions):
        """타겟으로 가기 위한 차선 변경 계획"""
        try:
            # 현재 차선에서 타겟 방향으로 차선 변경
            if current_waypoint.lane_id < target_waypoint.lane_id:
                # 오른쪽 차선으로 변경
                right_lane = current_waypoint.get_right_lane()
                if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                    return right_lane.transform.location
            elif current_waypoint.lane_id > target_waypoint.lane_id:
                # 왼쪽 차선으로 변경
                left_lane = current_waypoint.get_left_lane()
                if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                    return left_lane.transform.location
            
            # 차선 변경이 불가능하면 현재 차선 유지
            return current_waypoint.transform.location
            
        except Exception as e:
            print(f"⚠️ Error planning lane change: {e}")
            return current_waypoint.transform.location
    
    def _calculate_safe_following_position(self, current_waypoint, target_waypoint, safe_distance):
        """안전한 추격 위치 계산"""
        try:
            # 타겟에서 안전 거리만큼 뒤에 위치
            target_location = target_waypoint.transform.location
            target_forward = target_waypoint.transform.get_forward_vector()
            
            # 뒤로 이동
            safe_location = target_location - carla.Location(
                x=target_forward.x * safe_distance,
                y=target_forward.y * safe_distance,
                z=0
            )
            
            # 해당 위치의 waypoint 찾기
            safe_waypoint = self.map.get_waypoint(safe_location)
            return safe_waypoint.transform.location
            
        except Exception as e:
            print(f"⚠️ Error calculating safe following position: {e}")
            return target_waypoint.transform.location
    
    def _set_destination_with_lane_keeping(self, destination):
        """목적지 설정 - 차선 유지"""
        try:
            # Global Route Planner로 경로 계산
            start_location = self.chase_vehicle.get_location()
            route = self.global_planner.trace_route(start_location, destination)
            
            # Local Planner에 경로 설정 (차선 유지)
            self.local_planner.set_global_plan(route, clean_queue=True)
            
        except Exception as e:
            print(f"⚠️ Error setting destination: {e}")
    
    def _get_emergency_control(self):
        """비상 제어"""
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = 1.0
        control.steer = 0.0
        control.hand_brake = True
        return control
    
    def set_chase_parameters(self, follow_distance=None, approach_distance=None, 
                            intercept_distance=None, emergency_distance=None, 
                            max_speed=None, update_interval=None):
        """추격 파라미터 설정"""
        if follow_distance is not None:
            self.follow_distance = follow_distance
        if approach_distance is not None:
            self.approach_distance = approach_distance
        if intercept_distance is not None:
            self.intercept_distance = intercept_distance
        if emergency_distance is not None:
            self.emergency_distance = emergency_distance
        if max_speed is not None:
            self.max_speed = max_speed
            # Local Planner 속도도 업데이트
            self.local_planner.set_speed(max_speed * 3.6)  # km/h로 변환
        if update_interval is not None:
            print(f"🎯 Update interval set to: {update_interval}s")
        
        print(f"🎯 Chase parameters updated: follow={self.follow_distance}, approach={self.approach_distance}, max_speed={self.max_speed}")
    
    def reset_chase(self):
        """추격 리셋"""
        self.current_state = ChaseState.IDLE
        self.target_position = None
        self.target_velocity = None
        self.last_state_change = time.time()
        self.avoid_pedestrians = []
        print("🎯 Chase reset to IDLE")
    
    def get_current_state(self):
        """현재 상태 반환"""
        return self.current_state
    
    def is_chasing(self):
        """추격 중인지 확인"""
        return self.current_state != ChaseState.IDLE
    
    def get_chase_status(self):
        """추격 상태 정보 반환"""
        return {
            'state': self.current_state.value,
            'is_chasing': self.is_chasing(),
            'target_position': self.target_position,
            'target_velocity': self.target_velocity,
            'avoid_pedestrians_count': len(self.avoid_pedestrians)
        }