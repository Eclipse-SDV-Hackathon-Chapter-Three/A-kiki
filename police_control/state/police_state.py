# state/police_state.py
import math

class PoliceState:
    """
    Lightweight container for current ego state.
    Units: positions in meters, yaw in radians, velocity in m/s.
    """

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.speed = 0.0
        self.lockon = False
        self.obstacle = False

    def update_from_carla_transform(self, transform, velocity):
        """
        transform: carla.Transform
        velocity: carla.Vector3D (m/s)
        """
        self.x = transform.location.x
        self.y = transform.location.y
        self.z = transform.location.z
        # carla rotation yaw is in degrees
        self.yaw = math.radians(transform.rotation.yaw)
        # print(self.yaw)
        # print("이 값은 yaw값이노")
        self.vx = velocity.x
        self.vy = velocity.y
        self.speed = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5
