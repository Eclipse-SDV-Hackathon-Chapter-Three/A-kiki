import time

class PIDLongitudinalController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.05,
                 output_limit=1.0, soft_start_duration=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_err = 0.0
        self._last_time = None

        # soft start
        self.soft_start_duration = soft_start_duration

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self._last_time = None

    def run_step(self, target_speed, current_speed, lockon_time=None):
        """
        target_speed, current_speed: m/s
        lockon_time: 락온 시각 (soft start scaling용)
        """
        # ----- Soft start scaling -----
        if lockon_time is not None:
            elapsed = time.time() - lockon_time
            speed_ratio = min(1.0, elapsed / self.soft_start_duration)
            effective_target = target_speed * speed_ratio
        else:
            effective_target = target_speed

        # ----- PID error -----
        error = effective_target - current_speed
        self.integral += error * self.dt
        derivative = (error - self.prev_err) / (self.dt + 1e-6)

        out = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        out = max(-self.output_limit, min(self.output_limit, out))

        self.prev_err = error
        self._last_time = time.time()
        return out

