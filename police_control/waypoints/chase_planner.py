import sys
import os

# Add the correct CARLA path to sys.path based on the user's file structure.
# The 'agents' directory is inside '/home/seame/PythonAPI/carla'.
sys.path.append('/home/team1/seame/PythonAPI/carla')

import carla
import time
import math
import numpy as np
from scipy.interpolate import splprep, splev
from itertools import product
from agents.navigation.global_route_planner import GlobalRoutePlanner

class ChasePlanner:
    """
    A modified ChasePlanner that uses CARLA's robust GlobalRoutePlanner (GRP)
    but makes its internal graph bidirectional to allow for emergency vehicle routing
    (e.g., driving against traffic). This provides the best of both worlds:
    a stable, well-connected graph and the flexibility to ignore traffic rules.
    """

    def __init__(self, world, sampling_resolution=2.0, enable_emergency_routing=False):
        self.world = world
        self.map = world.get_map()
        self.sampling_resolution = sampling_resolution

        # Use the older GRP API, which doesn't require a DAO.
        # The graph is built inside the GRP constructor in these versions.
        print("[ChasePlanner] Using legacy GlobalRoutePlanner API.")
        self._grp = GlobalRoutePlanner(self.map, self.sampling_resolution)

        # Make the graph undirected for emergency vehicle routing ONLY if explicitly requested
        # Default is False to match origin system behavior
        self.emergency_routing = enable_emergency_routing
        if self.emergency_routing:
            print("[ChasePlanner] Emergency routing enabled - making graph bidirectional")
            self._make_graph_bidirectional(penalty=1.5)
        else:
            print("[ChasePlanner] Using standard routing (like origin system)")

        self.route = []

    def _make_graph_bidirectional(self, penalty=3.0):
        """
        Modifies the GRP's internal graph (a networkx.DiGraph) to allow travel 
        in both directions, but adds a cost penalty for driving against the original
        direction of the edge.
        """
        print(f"[ChasePlanner] Making graph bidirectional with penalty {penalty}...")
        start_time = time.time()
        graph = self._grp._graph
        
        edges_to_add = []
        # Iterate over all existing directed edges
        for u, v, data in graph.edges(data=True):
            # If the reverse edge doesn't exist, prepare to add it with a penalty
            if not graph.has_edge(v, u):
                new_data = data.copy()
                # The correct attribute for cost in this GRP version is 'length'
                new_data['length'] = data['length'] * penalty
                edges_to_add.append((v, u, new_data))

        graph.add_edges_from(edges_to_add)
        print(f"[ChasePlanner] Graph is now bidirectional. Took {time.time() - start_time:.2f}s.")

    # --- 영역 생성 (원형) - origin에서 가져온 최적경로 알고리즘 ---
    def _get_region_points(self, center_loc: carla.Location, radius=3.0, num_points=4):
        """
        벽 충돌 방지를 위해 포인트 수를 4개로 더 감소 (16가지 조합)
        더 작은 radius=3.0으로 매우 안전한 경로 생성
        """
        points = []
        for i in range(num_points):
            theta = 2 * math.pi * i / num_points
            x = center_loc.x + radius * math.cos(theta)
            y = center_loc.y + radius * math.sin(theta)
            z = center_loc.z
            loc = carla.Location(x, y, z)
            # Driving lane으로 snap
            wp = self.map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp:
                points.append(wp.transform.location)
        return points

    # --- 영역 시각화 ---
    def visualize_region_points(self, points, color=carla.Color(255,0,0), life_time=30.0):
        for pt in points:
            self.world.debug.draw_string(pt + carla.Location(z=0.2), 'O', draw_shadow=False,
                                         color=color, life_time=life_time, persistent_lines=True)

    # --- 영역 기반 최단 경로 계산 (origin에서 가져옴) ---
    def _plan_route_area_to_area(self, origin: carla.Location, destination: carla.Location, radius=3.0):
        """
        안전한 경로 생성을 위한 수정된 알고리즘:
        - 포인트 수 4개 사용 (16가지 조합, 매우 빠르고 안전)
        - 더 작은 radius=3.0으로 벽 충돌 완전 방지
        """
        origin_points = self._get_region_points(origin, radius)
        destination_points = self._get_region_points(destination, radius)
        min_dist = float('inf')
        best_route = []

        # 후보점 시각화
        self.visualize_region_points(origin_points, color=carla.Color(0,255,0))
        self.visualize_region_points(destination_points, color=carla.Color(255,0,0))

        print(f"[ChasePlanner] Testing {len(origin_points)} x {len(destination_points)} = {len(origin_points) * len(destination_points)} route combinations")

        # 영역 내 모든 조합 테스트
        for o_pt, d_pt in product(origin_points, destination_points):
            raw_route = self._grp.trace_route(o_pt, d_pt)
            if not raw_route:
                continue
            waypoints = [(wp.transform.location.x, wp.transform.location.y, wp.transform.location.z)
                         for wp, _ in raw_route if wp.lane_type == carla.LaneType.Driving]
            if not waypoints:
                continue
            dist = sum(math.hypot(waypoints[i+1][0]-waypoints[i][0],
                                  waypoints[i+1][1]-waypoints[i][1]) for i in range(len(waypoints)-1))
            if dist < min_dist:
                min_dist = dist
                best_route = waypoints

        return best_route

    # --- Origin 스타일 스플라인 스무딩 (정확한 복제) ---
    def _smooth_route_spline_origin_style(self, route, smooth_factor=1.0, num_points=None):
        """
        Origin 시스템의 정확한 스무딩 로직 복제 - 벽 충돌 방지
        """
        if len(route) < 4:
            return route

        # 중복 제거 (origin과 동일한 방식)
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
            tck, _ = splprep([x, y], s=smooth_factor, k=3)
            if num_points is None:
                num_points = max(len(route)*10, 2)
            unew = np.linspace(0, 1, num_points)
            x_new, y_new = splev(unew, tck)
            z_new = np.interp(unew, np.linspace(0, 1, len(z)), z)
            return list(zip(x_new, y_new, z_new))
        except Exception as e:
            print(f"[ChasePlanner] spline smoothing failed: {e}, fallback to raw route")
            return unique_route

    def _smooth_route_spline(self, route, smooth_factor=0.1, num_points_scale=5):
        """
        Smooths a given route using B-splines.
        Handles both old format (list of (waypoint, RoadOption)) and new format (list of tuples)
        """
        if len(route) < 4:
            # Handle different input formats
            if isinstance(route[0], tuple) and len(route[0]) >= 3:
                # Already in (x, y, z) format
                return route
            else:
                # Convert from waypoint format
                return [(wp.transform.location.x, wp.transform.location.y, wp.transform.location.z) for wp, _ in route]

        # Extract x, y, z coordinates based on input format
        if isinstance(route[0], tuple) and len(route[0]) >= 3:
            # Input is already in (x, y, z) format
            x, y, z = zip(*route)
        else:
            # Extract x, y, z coordinates from the route (which is a list of (waypoint, RoadOption))
            x, y, z = zip(*[(wp.transform.location.x, wp.transform.location.y, wp.transform.location.z) for wp, _ in route])

        # Remove consecutive duplicate points
        unique_points = []
        for i in range(len(x)):
            if i > 0 and abs(x[i] - x[i-1]) < 0.1 and abs(y[i] - y[i-1]) < 0.1:
                continue
            unique_points.append((x[i], y[i], z[i]))

        if len(unique_points) < 4:
            return unique_points

        x, y, z = zip(*unique_points)
        x, y, z = np.array(x), np.array(y), np.array(z)

        try:
            # Calculate total length of the raw path for determining number of spline points
            total_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
            num_points = max(int(total_length / self.sampling_resolution * num_points_scale), len(x) * 2)

            tck, _ = splprep([x, y], s=smooth_factor, k=3)
            unew = np.linspace(0, 1, num_points)
            x_new, y_new = splev(unew, tck)
            # Interpolate Z separately as it's not part of the 2D spline
            z_new = np.interp(unew, np.linspace(0, 1, len(z)), z)
            return list(zip(x_new, y_new, z_new))
        except Exception as e:
            print(f"[ChasePlanner] Spline smoothing failed: {e}, returning raw (unsmoothed) route.")
            return unique_points

    def plan_route(self, origin: carla.Location, destination: carla.Location, region_radius=3.0, use_optimal=True):
        """
        Plans a route from origin to destination using optimal area-to-area algorithm from origin system
        with performance improvements. Falls back to simple point-to-point if optimal fails.
        """
        if use_optimal:
            # Use optimal area-to-area algorithm from origin system
            print("[ChasePlanner] Using optimal area-to-area pathfinding...")
            raw_waypoints = self._plan_route_area_to_area(origin, destination, radius=region_radius)

            if raw_waypoints:
                # 첫 waypoint 조정 로직 (origin과 동일) - 벽 충돌 방지에 핵심!
                first_wp_loc = carla.Location(*raw_waypoints[0])
                first_wp_snap = self.map.get_waypoint(first_wp_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
                if first_wp_snap:
                    first_wp = first_wp_snap.transform.location
                    first_adjusted = carla.Location(
                        x=first_wp.x + first_wp_snap.transform.get_forward_vector().x * 1.5,
                        y=first_wp.y + first_wp_snap.transform.get_forward_vector().y * 1.5,
                        z=first_wp.z
                    )
                    raw_waypoints[0] = (first_adjusted.x, first_adjusted.y, first_adjusted.z)

                # origin과 동일한 동적 포인트 수 계산
                total_length = sum(
                    math.hypot(raw_waypoints[i+1][0]-raw_waypoints[i][0],
                               raw_waypoints[i+1][1]-raw_waypoints[i][1])
                    for i in range(len(raw_waypoints)-1)
                )
                num_points = max(int(total_length / self.sampling_resolution), 2)

                # 벽 충돌 방지를 위해 매우 보수적인 스무딩 사용
                self.route = self._smooth_route_spline_origin_style(raw_waypoints, smooth_factor=3.0, num_points=num_points)
                print(f"[ChasePlanner] Optimal route found with {len(self.route)} waypoints")
                return self.route
            else:
                print("[ChasePlanner] Optimal pathfinding failed, falling back to simple routing...")

        # 🚨 GTA 스타일 직선 공격 모드 추가
        distance_to_target = ((destination.x - origin.x) ** 2 + (destination.y - origin.y) ** 2) ** 0.5

        if not use_optimal and distance_to_target < 25.0:
            print(f"[ChasePlanner] 🚨 GTA DIRECT ASSAULT MODE - Creating straight line attack!")
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
            self.route = self._smooth_route_spline_origin_style(direct_route, smooth_factor=1.0, num_points=len(direct_route) * 2)
            print(f"[ChasePlanner] 💥 DIRECT ATTACK ROUTE (Smoothed): {len(self.route)} points, straight to target!")
            return self.route

        # Fallback to simple point-to-point routing
        start_wp = self.map.get_waypoint(origin, project_to_road=True, lane_type=carla.LaneType.Driving)
        end_wp = self.map.get_waypoint(destination, project_to_road=True, lane_type=carla.LaneType.Driving)

        if not start_wp or not end_wp:
            print(f"[ChasePlanner] Could not find a valid waypoint for origin or destination.")
            return []

        # trace_route returns a list of (waypoint, RoadOption)
        raw_route = self._grp.trace_route(start_wp.transform.location, end_wp.transform.location)

        if not raw_route:
            print(f"[ChasePlanner] GRP could not find a route!")
            return []

        # Smooth the path to make it drivable
        self.route = self._smooth_route_spline(raw_route)
        print(f"[ChasePlanner] Simple route found with {len(self.route)} waypoints")

        return self.route

    def get_route(self):
        return self.route

    def get_next_waypoint(self, current_location: carla.Location, lookahead=1):
        if not self.route:
            return None
        min_dist = float('inf')
        closest_idx = 0
        for i, (x, y, _) in enumerate(self.route):
            dist = math.hypot(x - current_location.x, y - current_location.y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        target_idx = min(closest_idx + lookahead, len(self.route) - 1)
        return self.route[target_idx]

    def visualize_route(self, life_time=60.0):
        for loc in self.route:
            carla_loc = carla.Location(x=loc[0], y=loc[1], z=loc[2] + 0.5)
            self.world.debug.draw_string(
                carla_loc, 'O', draw_shadow=False,
                color=carla.Color(0, 255, 0), life_time=life_time,
                persistent_lines=True
            )
