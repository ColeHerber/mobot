"""Unicycle wheel-inertial odometry (WIO).

Called synchronously from the main control loop — not a separate thread.
Uses BNO085 heading directly (no gyro integration) and VESC wheel velocity.

  x += v * cos(theta) * dt
  y += v * sin(theta) * dt

Reset to (0, 0, theta_0) at the start line before each run.
"""

import math


class Odometry:
    def __init__(self):
        self.x: float = 0.0
        self.y: float = 0.0
        self._theta0: float = 0.0  # heading at reset, used to zero the angle

    def reset(self, heading_rad: float = 0.0):
        """Reset position to origin. heading_rad is the current IMU heading,
        which becomes the forward (+x) direction."""
        self.x = 0.0
        self.y = 0.0
        self._theta0 = heading_rad

    def update(self, velocity_ms: float, heading_rad: float, dt: float):
        """Integrate one timestep.

        Args:
            velocity_ms:  wheel linear velocity in m/s (from VESC)
            heading_rad:  absolute heading from BNO085 in radians
            dt:           elapsed time in seconds since last call
        """
        if dt <= 0.0 or dt > 1.0:
            return  # ignore bad dt (startup, stall, etc.)

        theta = heading_rad - self._theta0
        self.x += velocity_ms * math.cos(theta) * dt
        self.y += velocity_ms * math.sin(theta) * dt

    def distance_from_origin(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y)

    def distance_to(self, wx: float, wy: float) -> float:
        """Euclidean distance from current position to waypoint (wx, wy)."""
        dx = wx - self.x
        dy = wy - self.y
        return math.sqrt(dx * dx + dy * dy)

    def reset_x(self, x: float, y: float = 0.0):
        """Snap position to known landmark coordinates (odometry correction).

        Called by state machine when the robot settles at the base of a DROP
        ramp. Preserves _theta0 so heading reference is unchanged.
        """
        self.x = x
        self.y = y
