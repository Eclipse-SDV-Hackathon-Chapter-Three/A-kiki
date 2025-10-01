# controllers/vehicle_unstuck.py
import carla
import math

class VehicleUnstuck:
    """
    차량이 끼었을 때 자동 탈출 제어 모듈
    사용법:
        unstuck = VehicleUnstuck()
        unstuck.check_stuck(throttle, speed, current_loc, dt)
        control = unstuck.compute_unstuck_control(current_loc, dt)
    """
    def __init__(self):
        self.stuck_phase = 0
        self.stuck_timer = 0.0
        self.stuck_position = None

    def check_stuck(self, throttle: float, speed: float, current_loc: carla.Location, dt: float):
        """
        끼임 여부 감지
        - throttle > 0.8 이고 speed < 0.5이면 끼임 가능성
        """
        if self.stuck_phase == 0 and throttle > 0.8 and speed < 0.5:
            self.stuck_timer += dt
            if self.stuck_timer > 2.0:
                self.stuck_phase = 1
                self.stuck_timer = 0.0
                self.stuck_position = current_loc
        else:
            self.stuck_timer = 0.0

    def compute_unstuck_control(self, current_loc: carla.Location, dt: float) -> carla.VehicleControl | None:
        """
        탈출 제어 계산
        - phase 1: 후진 + 좌/우 스티어
        - phase 2: 전진 + 조향
        """
        control = None
        self.stuck_timer += dt

        if self.stuck_phase == 1:
            # 후진 + 스티어
            control = carla.VehicleControl(steer=-0.5, throttle=0.8, brake=0.0, reverse=True)
            if self.stuck_position and current_loc.distance(self.stuck_position) > 0.5:
                self.stuck_phase = 2
                self.stuck_timer = 0.0
                self.stuck_position = None
            elif self.stuck_timer > 1.5:
                self.stuck_phase = 2
                self.stuck_timer = 0.0
                self.stuck_position = None

        elif self.stuck_phase == 2:
            # 전진 + 반대 스티어
            control = carla.VehicleControl(steer=0.8, throttle=0.6, brake=0.0, reverse=False)
            if self.stuck_timer > 1.0:
                self.stuck_phase = 0
                self.stuck_timer = 0.0

        return control
