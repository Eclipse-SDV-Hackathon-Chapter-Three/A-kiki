# controllers/stanley_lateral.py
import math
import time
import numpy as np
from scipy.interpolate import splprep, splev

class StanleyLateralController:
    def __init__(self, k=1.5, soft_gain=2.5, max_steer=1.0,
                 init_gain=3.5, strong_duration=4.0, lookahead_distance=5.0):
        """
        Stanley lateral controller with smooth strong steering after lockon.

        Args:
            k: heading error gain
            soft_gain: cross track error gain
            max_steer: steering saturation [-1, 1]
            init_gain: multiplier for heading error during strong phase
            strong_duration: seconds of strong phase after lockon
            lookahead_distance: meters ahead for yaw calculation
        """
        self.k = k
        self.soft_gain = soft_gain
        self.max_steer = max_steer
        self.init_gain = init_gain
        self.strong_duration = strong_duration
        self.lookahead_distance = lookahead_distance

    @staticmethod
    def _wrap_to_pi(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def smooth_route_spline(self, route, smooth_factor=0.0, num_points=None):
        """Spline smoothing for waypoints"""
        if len(route) < 4:
            return route

        # 중복 제거
        unique_route = []
        seen = set()
        for pt in route:
            if len(pt) >= 2:
                key = (round(pt[0], 2), round(pt[1], 2))
                if key not in seen:
                    unique_route.append(pt)
                    seen.add(key)
        if len(unique_route) < 4:
            return unique_route

        # 2D 또는 3D 포인트 처리
        if len(unique_route[0]) == 2:
            x, y = zip(*unique_route)
            x, y = np.array(x), np.array(y)
            try:
                tck, u = splprep([x, y], s=smooth_factor, k=3)
                if num_points is None:
                    num_points = max(len(route)*10, 2)
                unew = np.linspace(0, 1, num_points)
                x_new, y_new = splev(unew, tck)
                return list(zip(x_new, y_new))
            except Exception as e:
                print(f"[Stanley] spline smoothing failed: {e}, fallback to raw route")
                return unique_route
        else:
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
                print(f"[Stanley] spline smoothing failed: {e}, fallback to raw route")
                return unique_route

    def compute_steer(self, x, y, yaw, waypoints, speed, lockon_time=None, use_smoothing=True):
        """
        Compute Stanley steering command.
        Args:
            use_smoothing: Whether to apply spline smoothing to waypoints
        """
        if not waypoints or len(waypoints) < 2:
            return 0.0

        # Apply spline smoothing if requested and enough waypoints
        if use_smoothing and len(waypoints) >= 4:
            waypoints = self.smooth_route_spline(waypoints, smooth_factor=0.1)

        # --- Lookahead waypoint selection ---
        nearest_idx = 0
        min_dist = float('inf')
        for i, (wx, wy) in enumerate(waypoints):
            dist = math.hypot(wx - x, wy - y)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        # Lookahead waypoint: 일정 거리 앞
        lookahead_idx = nearest_idx
        for i in range(nearest_idx, len(waypoints)):
            dist = math.hypot(waypoints[i][0]-x, waypoints[i][1]-y)
            if dist >= self.lookahead_distance:
                lookahead_idx = i
                break

        target_x, target_y = waypoints[lookahead_idx]

        # --- Heading error ---
        path_yaw = math.atan2(target_y - y, target_x - x)
        heading_error = self._wrap_to_pi(path_yaw - yaw)

        # --- Cross-track error ---
        nearest_x, nearest_y = waypoints[nearest_idx]
        dx = nearest_x - x
        dy = nearest_y - y
        cross_error = math.sin(path_yaw)*dx - math.cos(path_yaw)*dy
        cte_term = math.atan2(self.soft_gain * cross_error, speed + 1e-3)

        # --- Strong initial phase after lockon ---
        if lockon_time is not None:
            elapsed = time.time() - lockon_time
            if elapsed < self.strong_duration:
                gain_ratio = (self.strong_duration - elapsed)/self.strong_duration
                steer = heading_error * (1 + gain_ratio * (self.init_gain-1)) + cte_term
            else:
                steer = heading_error + cte_term
        else:
            steer = heading_error + cte_term

        # --- Clamp ---
        steer = max(-self.max_steer, min(self.max_steer, steer))

        return steer

