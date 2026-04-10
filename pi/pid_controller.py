"""PD line-following controller + adaptive speed controller.

Steering PD:
  error      = 0.0 - line_position      (target: centered on line)
  derivative = (error - prev_error) / dt
  steering   = Kp * error + Kd * derivative

Derivative is computed on the measurement (not setpoint changes) to avoid
derivative kick when target changes.

Adaptive speed:
  Ramps up toward base_speed while the robot is well-centered and confident.
  Ramps down quickly toward min_speed when off the line or low confidence.
  "On line" = confidence >= speed_conf_min AND |line_pos| <= speed_pos_max.

All gains and limits are loaded from config/params.yaml at construction.
"""


class PIDController:
    def __init__(self, config: dict):
        pid_cfg   = config.get("pid", {})
        speed_cfg = config.get("speed", {})

        self.kp = float(pid_cfg.get("kp", 1.2))
        self.kd = float(pid_cfg.get("kd", 0.08))

        self.base_speed = float(speed_cfg.get("base_ms", 0.6))
        self.min_speed  = float(speed_cfg.get("min_ms",  0.2))

        # Adaptive speed params
        self._ramp_up   = float(pid_cfg.get("speed_ramp_up",   0.2))  # m/s per second
        self._ramp_down = float(pid_cfg.get("speed_ramp_down",  0.8)) # m/s per second
        self._conf_min  = int(pid_cfg.get("speed_conf_min",    80))   # confidence gate
        self._pos_max   = float(pid_cfg.get("speed_pos_max",   0.3))  # |line_pos| gate

        self._prev_error: float    = 0.0
        self._prev_line_pos: float = 0.0
        self._adaptive_speed: float = self.min_speed  # start slow, ramp up on line

    def reset(self):
        self._prev_error     = 0.0
        self._prev_line_pos  = 0.0
        self._adaptive_speed = self.min_speed

    def compute(self, line_position: float, confidence: int, dt: float):
        """Compute steering and throttle for one timestep.

        Args:
            line_position: current line position [-1.0, +1.0], 0 = center
            confidence:    sensor confidence 0–255
            dt:            elapsed time since last call in seconds

        Returns:
            (steering, throttle) — steering in [-1.0, +1.0],
                                    throttle in [min_speed, base_speed] m/s
        """
        if dt <= 0.0:
            dt = 0.01  # guard

        # ── Steering PD ───────────────────────────────────────────────────────
        error = 0.0 - line_position
        measurement_delta = line_position - self._prev_line_pos
        derivative = -measurement_delta / dt
        steering = self.kp * error + self.kd * derivative
        steering = max(-1.0, min(1.0, steering))

        # ── Adaptive speed ────────────────────────────────────────────────────
        on_line = (confidence >= self._conf_min and abs(line_position) <= self._pos_max)
        if on_line:
            self._adaptive_speed = min(self.base_speed,
                                       self._adaptive_speed + self._ramp_up * dt)
        else:
            self._adaptive_speed = max(self.min_speed,
                                       self._adaptive_speed - self._ramp_down * dt)

        self._prev_error    = error
        self._prev_line_pos = line_position

        return steering, self._adaptive_speed
