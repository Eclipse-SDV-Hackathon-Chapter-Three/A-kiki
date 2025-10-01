# waypoints/chase_planner.py
import carla
import math
import sys
import numpy as np
from scipy.interpolate import splprep, splev
from itertools import product
import time

sys.path.append('/home/seame/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner

class ChasePlanner:
    """
    ChasePlanner using CARLA GlobalRoutePlanner (GRP)
    - Candidate points generated on a circle around origin and destination
    - Only points on drivable road are considered
    - GRP computes shortest route among all origin->destination candidate pairs
    - Best (shortest) route selected
    - Optional visualization
    """

    def __init__(self, world, sampling_resolution=2.0, enable_emergency_routing=False):
        self.world = world
        self.map = world.get_map()
        self.sampling_resolution = sampling_resolution
        
        # Use GlobalRoutePlanner - compatible with both old and new API
        print("[ChasePlanner] Initializing GlobalRoutePlanner...")
        self.grp = GlobalRoutePlanner(self.map, sampling_resolution)
        
        # Emergency routing support for police vehicles
        self.emergency_routing = enable_emergency_routing
        if self.emergency_routing:
            print("[ChasePlanner] 🚨 Emergency routing enabled - making graph bidirectional")
            self._make_graph_bidirectional(penalty=1.5)
        else:
            print("[ChasePlanner] Using standard routing")
        
        self.route = []

        # simple cache: key = (ox, oy, dx, dy) rounded, value = list of (x,y,z)
        self._route_cache = {}
        self._cache_max_entries = 200

    def _make_graph_bidirectional(self, penalty=1.5):
        """
        🚨 Emergency Vehicle Routing: 그래프를 양방향으로 만들어 역주행 허용
        경찰차가 긴급상황에서 역방향 도로 사용 가능하도록 함
        Args:
            penalty: 역방향 이동시 추가 비용 (1.5 = 1.5배 비용)
        """
        print(f"[ChasePlanner] Making graph bidirectional with penalty {penalty}...")
        start_time = time.time()
        
        try:
            # GlobalRoutePlanner의 내부 그래프에 접근
            graph = self.grp._graph
            
            edges_to_add = []
            # 기존 방향 간선들을 순회하며 역방향 간선 추가
            for u, v, data in graph.edges(data=True):
                # 역방향 간선이 없으면 추가 준비
                if not graph.has_edge(v, u):
                    new_data = data.copy()
                    # 역방향 이동시 비용에 패널티 적용
                    if 'length' in new_data:
                        new_data['length'] = data['length'] * penalty
                    edges_to_add.append((v, u, new_data))
            
            # 역방향 간선들을 그래프에 추가
            graph.add_edges_from(edges_to_add)
            
            elapsed = (time.time() - start_time) * 1000.0
            print(f"[ChasePlanner] ✅ Graph bidirectional conversion complete! "
                  f"Added {len(edges_to_add)} reverse edges in {elapsed:.1f}ms")
                  
        except Exception as e:
            print(f"[ChasePlanner] ❌ Failed to make graph bidirectional: {e}")
            print("[ChasePlanner] Continuing with standard routing...")

    # --- Spline smoothing ---
    def smooth_route_spline(self, route, smooth_factor=0.0, num_points=None):
        if len(route) < 4:
            return route

        # 중복 제거
        unique_route = []
        seen = set()
        for pt in route:
            key = (round(pt[0], 2), round(pt[1], 2))
            if key not in seen:
                unique_route.append(pt)
                seen.add(key)
        if len(unique_route) < 4:
            return unique_route

        x, y, z = zip(*unique_route)
        x, y, z = np.array(x), np.array(y), np.array(z)
        try:
            tck, u = splprep([x, y], s=smooth_factor, k=3)
            if num_points is None:
                num_points = max(len(route)*10, 2)
            unew = np.linspace(0, 1, num_points)
            x_new, y_new = splev(unew, tck)
            z_new = np.interp(unew, np.linspace(0, 1, len(z)), z)
            return list(zip(x_new, y_new, z_new))
        except Exception as e:
            print(f"[ChasePlanner] spline smoothing failed: {e}, fallback to raw route")
            return unique_route

    # ---------------- Candidate Points ----------------
    def _get_region_points(self, center_loc: carla.Location, radius=5.0, num_points=10):
        """
        Get num_points evenly spaced around the center location
        🚨 ENHANCED: 반대 차선 포함하여 모든 방향 차선 탐색
        """
        points = []
        unique_waypoints = set()

        # 1️⃣ 원형 패턴으로 기본 포인트 생성
        for i in range(num_points):
            theta = 2 * math.pi * i / num_points
            x = center_loc.x + radius * math.cos(theta)
            y = center_loc.y + radius * math.sin(theta)
            z = center_loc.z
            loc = carla.Location(x, y, z)
            
            # 주변 모든 차선 탐색 (반대 차선 포함)
            wp = self.map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp:
                # 현재 웨이포인트
                wp_key = (round(wp.transform.location.x, 1), round(wp.transform.location.y, 1))
                if wp_key not in unique_waypoints:
                    points.append(wp.transform.location)
                    unique_waypoints.add(wp_key)
                
                # 🚨 반대 방향 차선도 탐색 (양방향 경로 보장)
                try:
                    # 좌측 차선들
                    left_wp = wp
                    for _ in range(3):  # 최대 3개 좌측 차선
                        left_wp = left_wp.get_left_lane()
                        if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                            left_key = (round(left_wp.transform.location.x, 1), round(left_wp.transform.location.y, 1))
                            if left_key not in unique_waypoints:
                                points.append(left_wp.transform.location)
                                unique_waypoints.add(left_key)
                        else:
                            break
                    
                    # 우측 차선들
                    right_wp = wp
                    for _ in range(3):  # 최대 3개 우측 차선
                        right_wp = right_wp.get_right_lane()
                        if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                            right_key = (round(right_wp.transform.location.x, 1), round(right_wp.transform.location.y, 1))
                            if right_key not in unique_waypoints:
                                points.append(right_wp.transform.location)
                                unique_waypoints.add(right_key)
                        else:
                            break
                
                except Exception:
                    pass  # 차선 탐색 실패는 무시

        # 2️⃣ 중심점 자체도 포함 (직접 경로용)
        center_wp = self.map.get_waypoint(center_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if center_wp:
            center_key = (round(center_wp.transform.location.x, 1), round(center_wp.transform.location.y, 1))
            if center_key not in unique_waypoints:
                points.append(center_wp.transform.location)

        print(f"[ChasePlanner] 🎯 Generated {len(points)} unique region points (including opposite lanes)")
        return points

    def visualize_region_points(self, points, color=carla.Color(255,0,0), life_time=30.0):
        for pt in points:
            self.world.debug.draw_string(pt + carla.Location(z=0.2), 'O', draw_shadow=False,
                                         color=color, life_time=life_time, persistent_lines=True)

    # ---------------- Cache ----------------
    def _cache_get(self, origin: carla.Location, dest: carla.Location):
        key = (round(origin.x, 2), round(origin.y, 2), round(dest.x, 2), round(dest.y, 2))
        return self._route_cache.get(key)

    def _cache_set(self, origin: carla.Location, dest: carla.Location, waypoints):
        if len(self._route_cache) > self._cache_max_entries:
            self._route_cache.clear()
        key = (round(origin.x, 2), round(origin.y, 2), round(dest.x, 2), round(dest.y, 2))
        self._route_cache[key] = waypoints

    # ---------------- Route Planning ----------------
    def _plan_route_area_to_area(self, origin: carla.Location, destination: carla.Location, radius=8.0):
        """
        Plan route from origin area to destination area.
        🚨 FORCE SHORTEST PATH: 무조건 최단 경로 선택 (반대 차선 포함)
        """
        origin_points = self._get_region_points(origin, radius)
        destination_points = self._get_region_points(destination, radius)
        
        print(f"[ChasePlanner] 🎯 FORCE SHORTEST PATH: Testing {len(origin_points)} x {len(destination_points)} = {len(origin_points) * len(destination_points)} combinations")
        
        # 시각화 (선택적)
        self.visualize_region_points(origin_points, color=carla.Color(0,255,0))
        self.visualize_region_points(destination_points, color=carla.Color(255,0,0))

        min_dist = float('inf')
        best_route = []
        best_origin = None
        best_dest = None
        valid_routes_count = 0

        # 🚨 모든 조합을 철저히 검사하여 절대 최단 경로 찾기
        for o_pt, d_pt in product(origin_points, destination_points):
            cached = self._cache_get(o_pt, d_pt)
            if cached is not None:
                waypoints = cached
            else:
                try:
                    raw_route = self.grp.trace_route(o_pt, d_pt)
                    if not raw_route:  # 경로가 없으면 스킵
                        continue
                        
                    waypoints = [(wp.transform.location.x, wp.transform.location.y, wp.transform.location.z)
                                for wp, _ in raw_route if wp.lane_type == carla.LaneType.Driving]
                    
                    if len(waypoints) < 2:  # 유효하지 않은 경로
                        continue
                        
                    self._cache_set(o_pt, d_pt, waypoints)
                except Exception as e:
                    continue

            # 🎯 정밀한 거리 계산 (3D 유클리드 거리)
            if len(waypoints) >= 2:
                total_dist = 0
                for i in range(len(waypoints)-1):
                    dx = waypoints[i+1][0] - waypoints[i][0]
                    dy = waypoints[i+1][1] - waypoints[i][1]
                    dz = waypoints[i+1][2] - waypoints[i][2]
                    total_dist += math.sqrt(dx*dx + dy*dy + dz*dz)
                
                valid_routes_count += 1
                
                # 🚨 더 짧은 경로 발견 시 무조건 업데이트
                if total_dist < min_dist:
                    min_dist = total_dist
                    best_route = waypoints
                    best_origin = o_pt
                    best_dest = d_pt
                    print(f"[ChasePlanner] 🎯 NEW SHORTEST: {total_dist:.2f}m (origin: {o_pt}, dest: {d_pt})")

        if best_route:
            print(f"[ChasePlanner] ✅ SHORTEST PATH SELECTED: {min_dist:.2f}m from {valid_routes_count} valid routes")
            print(f"[ChasePlanner] 📍 Best origin: ({best_origin.x:.1f}, {best_origin.y:.1f})")
            print(f"[ChasePlanner] 🎯 Best destination: ({best_dest.x:.1f}, {best_dest.y:.1f})")
        else:
            print(f"[ChasePlanner] ❌ NO VALID ROUTES FOUND from {len(origin_points) * len(destination_points)} combinations")

        return best_route


    # ---------------- External API ----------------
    def plan_route(self, origin: carla.Location, destination: carla.Location, region_radius=8.0, use_optimal=True):
        """
        Enhanced route planning with multiple modes:
        🚨 FORCE SHORTEST PATH: 무조건 최단 경로 (반대 차선 포함)
        - Standard: Area-to-area routing with region exploration
        - GTA Mode: Direct assault for close targets (< 25m)  
        - Emergency: Uses bidirectional graph if enabled
        """
        start_time = time.perf_counter()
        distance_to_target = math.hypot(destination.x - origin.x, destination.y - origin.y)

        print(f"[ChasePlanner] 🎯 FORCE SHORTEST PATH MODE: Origin({origin.x:.1f},{origin.y:.1f}) → Target({destination.x:.1f},{destination.y:.1f}), Distance: {distance_to_target:.1f}m")

        # 🚨 GTA 스타일 직선 공격 모드 (25m 이내)
        if not use_optimal and distance_to_target < 25.0:
            print(f"[ChasePlanner] 🚨 GTA DIRECT ASSAULT MODE - Creating straight line attack! Distance: {distance_to_target:.1f}m")
            
            # 직선 경로 생성 (도로 무시하고 직진!)
            num_direct_points = max(3, int(distance_to_target / 5.0))  # 5m마다 포인트
            direct_route = []

            for i in range(num_direct_points + 1):
                ratio = i / num_direct_points if num_direct_points > 0 else 0
                x = origin.x + (destination.x - origin.x) * ratio
                y = origin.y + (destination.y - origin.y) * ratio
                z = origin.z + (destination.z - origin.z) * ratio
                direct_route.append((x, y, z))

            # 직선 경로도 부드럽게 스무딩하여 제어 안정성 향상
            self.route = self.smooth_route_spline(direct_route, smooth_factor=0.5, num_points=len(direct_route) * 2)
            
            elapsed = (time.perf_counter() - start_time) * 1000.0
            print(f"[ChasePlanner] 💥 DIRECT ATTACK ROUTE: {len(self.route)} points, took {elapsed:.1f}ms")
            return self.route

        # 일반 영역 기반 라우팅
        raw_waypoints = self._plan_route_area_to_area(origin, destination, radius=region_radius)
        if not raw_waypoints:
            # Fallback: Simple point-to-point routing
            print("[ChasePlanner] Area routing failed, trying simple point-to-point...")
            return self._fallback_simple_routing(origin, destination)

        return self._finalize_route(raw_waypoints, start_time)

        # 첫 waypoint: 도로 중앙 snap + forward offset
        first_wp_loc = carla.Location(*raw_waypoints[0])
        first_wp_snap = self.map.get_waypoint(first_wp_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if first_wp_snap:
            first_wp = first_wp_snap.transform.location
            fwd = first_wp_snap.transform.get_forward_vector()
            first_adjusted = carla.Location(
                x=first_wp.x + fwd.x*1.5,
                y=first_wp.y + fwd.y*1.5,
                z=first_wp.z
            )
            raw_waypoints[0] = (first_adjusted.x, first_adjusted.y, first_adjusted.z)

        # --- Spline smoothing with dynamic num_points ---
        total_length = sum(
            math.hypot(raw_waypoints[i+1][0]-raw_waypoints[i][0],
                       raw_waypoints[i+1][1]-raw_waypoints[i][1])
            for i in range(len(raw_waypoints)-1)
        )
        num_points = max(int(total_length / self.sampling_resolution), 2)

        self.route = self.smooth_route_spline(raw_waypoints, smooth_factor=1.0, num_points=num_points)

        # 🚨 Waypoint 출력
        print("[ChasePlanner] Generated waypoints:")
        for idx, (x, y, z) in enumerate(self.route):
            print(f"  {idx}: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        elapsed = (time.perf_counter() - start_time) * 1000.0
        print(f"[ChasePlanner] Route planning took {elapsed:.2f} ms")

        return self.route

    def _fallback_simple_routing(self, origin: carla.Location, destination: carla.Location):
        """Emergency fallback: Simple point-to-point routing"""
        print("[ChasePlanner] Using fallback simple routing...")
        
        try:
            # Get waypoints on road
            start_wp = self.map.get_waypoint(origin, project_to_road=True, lane_type=carla.LaneType.Driving)
            end_wp = self.map.get_waypoint(destination, project_to_road=True, lane_type=carla.LaneType.Driving)

            if not start_wp or not end_wp:
                print("[ChasePlanner] ❌ Could not find valid waypoints, using direct path")
                return [(origin.x, origin.y, origin.z), (destination.x, destination.y, destination.z)]

            # Use GlobalRoutePlanner for basic routing
            raw_route = self.grp.trace_route(start_wp.transform.location, end_wp.transform.location)
            
            if raw_route:
                # Convert waypoint format to coordinate format
                waypoints = [(wp.transform.location.x, wp.transform.location.y, wp.transform.location.z)
                           for wp, _ in raw_route if hasattr(wp, 'transform')]
                
                self.route = self.smooth_route_spline(waypoints, smooth_factor=0.5) if len(waypoints) >= 4 else waypoints
                print(f"[ChasePlanner] ✅ Fallback route found with {len(self.route)} waypoints")
                return self.route
            else:
                print("[ChasePlanner] ❌ No route found, creating direct path")
                return [(origin.x, origin.y, origin.z), (destination.x, destination.y, destination.z)]
                
        except Exception as e:
            print(f"[ChasePlanner] ❌ Fallback routing failed: {e}")
            return [(origin.x, origin.y, origin.z), (destination.x, destination.y, destination.z)]

    def _finalize_route(self, raw_waypoints, start_time):
        """Route post-processing and finalization"""
        if not raw_waypoints:
            self.route = []
            return []

        # 첫 waypoint: 도로 중앙 snap + forward offset
        first_wp_loc = carla.Location(*raw_waypoints[0])
        first_wp_snap = self.map.get_waypoint(first_wp_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if first_wp_snap:
            first_wp = first_wp_snap.transform.location
            fwd = first_wp_snap.transform.get_forward_vector()
            first_adjusted = carla.Location(
                x=first_wp.x + fwd.x*1.5,
                y=first_wp.y + fwd.y*1.5,
                z=first_wp.z
            )
            raw_waypoints[0] = (first_adjusted.x, first_adjusted.y, first_adjusted.z)

        # --- Spline smoothing with dynamic num_points ---
        total_length = sum(
            math.hypot(raw_waypoints[i+1][0]-raw_waypoints[i][0],
                       raw_waypoints[i+1][1]-raw_waypoints[i][1])
            for i in range(len(raw_waypoints)-1)
        )
        num_points = max(int(total_length / self.sampling_resolution), 2)

        self.route = self.smooth_route_spline(raw_waypoints, smooth_factor=1.0, num_points=num_points)

        # 🚨 Waypoint 출력 (디버그용)
        if len(self.route) > 0:
            print(f"[ChasePlanner] Generated {len(self.route)} waypoints")

        elapsed = (time.perf_counter() - start_time) * 1000.0
        print(f"[ChasePlanner] Route planning took {elapsed:.2f} ms")

        return self.route

    # ---------------- Route Access ----------------
    def get_route(self):
        return self.route

    def get_next_waypoint(self, current_location: carla.Location, lookahead=1):
        if not self.route:
            return None
        min_dist = float('inf')
        closest_idx = 0
        for i, (x, y, z) in enumerate(self.route):
            dist = math.hypot(x - current_location.x, y - current_location.y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        target_idx = min(closest_idx + lookahead, len(self.route)-1)
        return self.route[target_idx]

    def visualize_route(self, life_time=60.0):
        for loc in self.route:
            carla_loc = carla.Location(x=loc[0], y=loc[1], z=loc[2]+0.5)
            self.world.debug.draw_string(carla_loc, 'O', draw_shadow=False,
                                         color=carla.Color(0,255,0), life_time=life_time, persistent_lines=True)
