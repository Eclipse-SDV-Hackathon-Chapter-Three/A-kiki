# controllers/stanley_lateral.py
import math
import time

class StanleyLateralController:
    def __init__(self, k=1.0, soft_gain=2.5, max_steer=1.0,
                 init_gain=2.0, strong_duration=4.0, lookahead_distance=5.0):
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

    def compute_steer(self, x, y, yaw, waypoints, speed, lockon_time=None):
        """
        Compute Stanley steering command.
        """
        if not waypoints or len(waypoints) < 2:
            return 0.0

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

