# waypoints/waypoint_manager.py
import math

class WaypointManager:
    """
    Manage waypoint list: selection, popping reached WPs, and final-arrival detection.
    Waypoints are list of tuples e.g. [(x,y,v), ...] where v is desired speed in m/s.
    """

    def __init__(self, arrival_threshold=2.5):
        self.waypoints = []  # list of (x,y,v)
        self.arrival_threshold = arrival_threshold

    def set_waypoints(self, wp_list):
        """Replace waypoint list. Expect list of (x,y,v) or similar."""
        self.waypoints = list(wp_list)

    def get_current_target(self):
        if not self.waypoints:
            return None
        return self.waypoints[0]

    def update_progress(self, veh_x, veh_y):
        """Check distance to current waypoint and pop if reached."""
        if not self.waypoints:
            return
        tx, ty, _ = self.waypoints[0]
        dist = math.hypot(tx - veh_x, ty - veh_y)
        if dist <= self.arrival_threshold:
            # remove current waypoint
            self.waypoints.pop(0)

    def is_final_reached(self, veh_x, veh_y):
        if not self.waypoints:
            return True
        # If only last waypoint and within threshold, treat as reached
        if len(self.waypoints) == 1:
            tx, ty, _ = self.waypoints[0]
            return math.hypot(tx - veh_x, ty - veh_y) <= self.arrival_threshold
        return False
