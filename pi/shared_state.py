"""Thread-safe shared state for the Mobot control stack."""

import threading


class SharedState:
    """All inter-thread state in one place, protected by a single lock.

    Sensor / IMU data is written by sensor_reader thread.
    VESC data is written by vesc_interface thread.
    Odometry, PID output, and state machine state are written by main loop.
    All fields are readable by any thread.
    """

    def __init__(self):
        self._lock = threading.Lock()

        # ── Sensor (from Teensy via USB serial) ──────────────────────────────
        self.line_position: float = 0.0       # [-1.0, +1.0], 0 = centered
        self.sensor_confidence: int = 0       # 0–255
        self.sensor_flags: int = 0            # 16-bit bitmask
        self.sensor_raw: list[int] = [0] * 16 # normalized 0–1000 per channel

        # ── IMU (BNO085 via I2C) ──────────────────────────────────────────────
        self.heading_rad: float = 0.0         # yaw in radians, world frame
        self.pitch_rad: float = 0.0           # rotation around x-axis (nose up/down)
        self.roll_rad: float = 0.0            # rotation around y-axis (lean left/right)

        # ── VESC (motor encoder via UART) ─────────────────────────────────────
        self.wheel_velocity_ms: float = 0.0  # m/s, positive = forward
        self.motor_rpm: float = 0.0
        self.input_voltage: float = 0.0      # LiPo voltage for monitoring

        # ── Odometry ──────────────────────────────────────────────────────────
        self.x: float = 0.0                  # meters from start
        self.y: float = 0.0

        # ── Control outputs ───────────────────────────────────────────────────
        self.steering: float = 0.0           # [-1.0, +1.0]
        self.throttle: float = 0.0           # [0.0, 1.0] duty fraction

        # ── State machine ─────────────────────────────────────────────────────
        self.state: str = "LINE_FOLLOW"

        # ── Teleop ────────────────────────────────────────────────────────────
        self.teleop_enabled:  bool  = False
        self.teleop_steering: float = 0.0   # [-1.0, +1.0]
        self.teleop_throttle: float = 0.0   # [-1.0, +1.0] normalized
        self.teleop_last_cmd: float = 0.0   # time.monotonic() of last command

        # ── Flags ─────────────────────────────────────────────────────────────
        self.running: bool = True            # cleared to trigger shutdown

    # ── Convenience bulk read / write ─────────────────────────────────────────

    def get(self, *fields):
        """Return a tuple of named field values under the lock."""
        with self._lock:
            return tuple(getattr(self, f) for f in fields)

    def set(self, **kwargs):
        """Set one or more fields atomically under the lock."""
        with self._lock:
            for k, v in kwargs.items():
                setattr(self, k, v)

    def update_sensor(self, line_position: float, confidence: int,
                      flags: int, raw: list[int]):
        with self._lock:
            self.line_position = line_position
            self.sensor_confidence = confidence
            self.sensor_flags = flags
            self.sensor_raw = raw

    def update_imu(self, heading_rad: float, pitch_rad: float = 0.0, roll_rad: float = 0.0):
        with self._lock:
            self.heading_rad = heading_rad
            self.pitch_rad = pitch_rad
            self.roll_rad = roll_rad

    def update_vesc(self, velocity_ms: float, rpm: float, voltage: float):
        with self._lock:
            self.wheel_velocity_ms = velocity_ms
            self.motor_rpm = rpm
            self.input_voltage = voltage

    def update_odometry(self, x: float, y: float):
        with self._lock:
            self.x = x
            self.y = y

    def update_control(self, steering: float, throttle: float, state: str):
        with self._lock:
            self.steering = steering
            self.throttle = throttle
            self.state = state

    def update_teleop(self, enabled: bool, steering: float, throttle: float):
        with self._lock:
            self.teleop_enabled  = enabled
            self.teleop_steering = steering
            self.teleop_throttle = throttle
            if enabled:
                import time
                self.teleop_last_cmd = time.monotonic()
