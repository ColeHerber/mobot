"""PD line-following controller + speed controller.

Steering PD:
  error      = 0.0 - line_position      (target: centered on line)
  derivative = (error - prev_error) / dt
  steering   = Kp * error + Kd * derivative

Derivative is computed on the measurement (not setpoint changes) to avoid
derivative kick when target changes.

Speed P:
  Reduce speed when abs(line_position) is large (tight curves).
  throttle = base_speed - speed_kp * abs(line_position) * base_speed
  Clamped to [min_speed, base_speed].

All gains and limits are loaded from config/params.yaml at construction.
"""


class PIDController:
    def __init__(self, config: dict):
        pid_cfg   = config.get("pid", {})
        speed_cfg = config.get("speed", {})

        self.kp       = float(pid_cfg.get("kp", 1.2))
        self.kd       = float(pid_cfg.get("kd", 0.08))
        self.speed_kp = float(pid_cfg.get("speed_kp", 0.5))

        self.base_speed = float(speed_cfg.get("base_ms", 0.6))
        self.min_speed  = float(speed_cfg.get("min_ms",  0.2))

        self._prev_error: float = 0.0
        self._prev_line_pos: float = 0.0  # for derivative-on-measurement

    def reset(self):
        self._prev_error = 0.0
        self._prev_line_pos = 0.0

    def compute(self, line_position: float, dt: float):
        """Compute steering and throttle for one timestep.

        Args:
            line_position: current line position [-1.0, +1.0], 0 = center
            dt: elapsed time since last call in seconds

        Returns:
            (steering, throttle) — steering in [-1.0, +1.0],
                                    throttle is a fraction of base_speed
        """
        if dt <= 0.0:
            dt = 0.01  # guard

        error = 0.0 - line_position

        # Derivative on measurement to avoid derivative kick
        measurement_delta = line_position - self._prev_line_pos
        derivative = -measurement_delta / dt  # sign: error decreases as pos increases

        steering = self.kp * error + self.kd * derivative
        steering = max(-1.0, min(1.0, steering))

        # Speed: reduce on large lateral error
        throttle = self.base_speed - self.speed_kp * abs(line_position) * self.base_speed
        throttle = max(self.min_speed, min(self.base_speed, throttle))

        self._prev_error    = error
        self._prev_line_pos = line_position

        return steering, throttle
