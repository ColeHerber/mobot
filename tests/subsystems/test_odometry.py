"""
test_odometry.py — Unit tests for odometry.py

Tests cover:
  - reset() zeroes x, y and stores initial heading
  - Straight-line integration at heading 0
  - 90° heading integration
  - Arbitrary heading
  - distance_from_origin() and distance_to()
  - Bad dt values are ignored
  - 100-step accumulation stays coherent
"""

import math
import sys
import os
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from odometry import Odometry


# ─────────────────────────────────────────────────────────────────────────────
# reset()
# ─────────────────────────────────────────────────────────────────────────────

class TestReset:

    def test_reset_zeroes_x(self):
        odo = Odometry()
        odo.update(1.0, 0.0, 1.0)   # move forward
        odo.reset(0.0)
        assert odo.x == pytest.approx(0.0)

    def test_reset_zeroes_y(self):
        odo = Odometry()
        odo.update(1.0, math.pi / 2, 1.0)  # move sideways
        odo.reset(0.0)
        assert odo.y == pytest.approx(0.0)

    def test_reset_stores_heading(self):
        odo = Odometry()
        odo.reset(math.pi / 4)
        assert odo._theta0 == pytest.approx(math.pi / 4)

    def test_reset_default_heading_is_zero(self):
        odo = Odometry()
        odo.reset()
        assert odo._theta0 == pytest.approx(0.0)
        assert odo.x == pytest.approx(0.0)
        assert odo.y == pytest.approx(0.0)

    def test_reset_clears_accumulated_position(self):
        odo = Odometry()
        for _ in range(100):
            odo.update(1.0, 0.0, 0.01)
        odo.reset(0.0)
        assert odo.x == pytest.approx(0.0)
        assert odo.y == pytest.approx(0.0)


# ─────────────────────────────────────────────────────────────────────────────
# update() — straight line at heading 0
# ─────────────────────────────────────────────────────────────────────────────

class TestStraightLine:

    def test_forward_heading0_updates_x(self):
        """v=1.0, θ=0, dt=1.0 → x=1.0, y≈0."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(1.0, 0.0, 1.0)
        assert odo.x == pytest.approx(1.0, abs=1e-9)
        assert odo.y == pytest.approx(0.0, abs=1e-9)

    def test_forward_small_dt(self):
        """v=0.6, θ=0, dt=0.01 → x=0.006."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(0.6, 0.0, 0.01)
        assert odo.x == pytest.approx(0.006, abs=1e-9)
        assert odo.y == pytest.approx(0.0, abs=1e-9)

    def test_multiple_steps_accumulate(self):
        """100 steps of v=1.0, θ=0, dt=0.01 → x≈1.0."""
        odo = Odometry()
        odo.reset(0.0)
        for _ in range(100):
            odo.update(1.0, 0.0, 0.01)
        assert odo.x == pytest.approx(1.0, abs=1e-9)
        assert odo.y == pytest.approx(0.0, abs=1e-9)

    def test_zero_velocity_no_movement(self):
        """v=0 → no position change."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(0.0, 0.0, 1.0)
        assert odo.x == pytest.approx(0.0)
        assert odo.y == pytest.approx(0.0)


# ─────────────────────────────────────────────────────────────────────────────
# update() — 90° heading (movement along Y axis)
# ─────────────────────────────────────────────────────────────────────────────

class TestNinetyDegreeHeading:

    def test_heading_90deg_updates_y(self):
        """v=1.0, θ=π/2 relative to reset → x≈0, y=1.0."""
        odo = Odometry()
        odo.reset(0.0)   # theta0 = 0
        odo.update(1.0, math.pi / 2, 1.0)
        assert odo.x == pytest.approx(0.0, abs=1e-9)
        assert odo.y == pytest.approx(1.0, abs=1e-9)

    def test_heading_minus_90deg(self):
        """v=1.0, θ=-π/2 → x≈0, y=-1.0."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(1.0, -math.pi / 2, 1.0)
        assert odo.x == pytest.approx(0.0, abs=1e-9)
        assert odo.y == pytest.approx(-1.0, abs=1e-9)

    def test_heading_180deg(self):
        """v=1.0, θ=π → x=-1.0, y≈0 (driving backwards along x)."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(1.0, math.pi, 1.0)
        assert odo.x == pytest.approx(-1.0, abs=1e-9)
        assert odo.y == pytest.approx(0.0, abs=1e-9)

    def test_heading_45deg(self):
        """v=1.0, θ=π/4 → x=y=cos(π/4)≈0.707."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(1.0, math.pi / 4, 1.0)
        assert odo.x == pytest.approx(math.cos(math.pi / 4), abs=1e-9)
        assert odo.y == pytest.approx(math.sin(math.pi / 4), abs=1e-9)


# ─────────────────────────────────────────────────────────────────────────────
# update() — non-zero initial heading (theta0 offset)
# ─────────────────────────────────────────────────────────────────────────────

class TestInitialHeadingOffset:

    def test_reset_heading_pi4_forward_at_pi4(self):
        """Reset at θ0=π/4, then move at heading π/4 → pure +x motion."""
        odo = Odometry()
        odo.reset(math.pi / 4)   # "forward" direction is π/4 in world frame
        odo.update(1.0, math.pi / 4, 1.0)   # same heading as reset → θ-θ0=0
        assert odo.x == pytest.approx(1.0, abs=1e-9)
        assert odo.y == pytest.approx(0.0, abs=1e-9)

    def test_reset_heading_offset_then_perpendicular(self):
        """Reset at θ0=π/4, move at θ=3π/4 → relative heading π/2 → pure +y."""
        odo = Odometry()
        odo.reset(math.pi / 4)
        odo.update(1.0, 3 * math.pi / 4, 1.0)   # relative: π/2
        assert odo.x == pytest.approx(0.0, abs=1e-9)
        assert odo.y == pytest.approx(1.0, abs=1e-9)


# ─────────────────────────────────────────────────────────────────────────────
# Bad dt guard
# ─────────────────────────────────────────────────────────────────────────────

class TestBadDt:

    def test_zero_dt_no_movement(self):
        """dt=0 is ignored (no update, no crash)."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(1.0, 0.0, 0.0)
        assert odo.x == pytest.approx(0.0)
        assert odo.y == pytest.approx(0.0)

    def test_negative_dt_no_movement(self):
        """dt<0 is ignored."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(1.0, 0.0, -0.01)
        assert odo.x == pytest.approx(0.0)

    def test_huge_dt_ignored(self):
        """dt > 1.0 s is ignored (stall/startup guard)."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(1.0, 0.0, 5.0)
        assert odo.x == pytest.approx(0.0)


# ─────────────────────────────────────────────────────────────────────────────
# distance_from_origin()
# ─────────────────────────────────────────────────────────────────────────────

class TestDistanceFromOrigin:

    def test_at_origin_is_zero(self):
        odo = Odometry()
        assert odo.distance_from_origin() == pytest.approx(0.0)

    def test_after_x_move(self):
        odo = Odometry()
        odo.reset(0.0)
        odo.update(1.0, 0.0, 1.0)   # x=1.0
        assert odo.distance_from_origin() == pytest.approx(1.0, abs=1e-9)

    def test_after_y_move(self):
        odo = Odometry()
        odo.reset(0.0)
        odo.update(1.0, math.pi / 2, 1.0)   # y=1.0
        assert odo.distance_from_origin() == pytest.approx(1.0, abs=1e-9)

    def test_pythagorean_3_4_5(self):
        """Move 3 along x then 4 along y → distance = 5."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(3.0, 0.0, 1.0)
        odo.update(4.0, math.pi / 2, 1.0)
        assert odo.distance_from_origin() == pytest.approx(5.0, abs=1e-9)

    def test_symmetry_positive_negative(self):
        """Distance is always non-negative."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(1.0, math.pi, 1.0)   # x=-1.0
        assert odo.distance_from_origin() >= 0.0


# ─────────────────────────────────────────────────────────────────────────────
# distance_to(wx, wy)
# ─────────────────────────────────────────────────────────────────────────────

class TestDistanceTo:

    def test_distance_to_origin_from_origin(self):
        odo = Odometry()
        assert odo.distance_to(0.0, 0.0) == pytest.approx(0.0)

    def test_distance_to_point_on_x(self):
        odo = Odometry()
        assert odo.distance_to(3.0, 0.0) == pytest.approx(3.0, abs=1e-9)

    def test_distance_to_point_on_y(self):
        odo = Odometry()
        assert odo.distance_to(0.0, 4.0) == pytest.approx(4.0, abs=1e-9)

    def test_distance_to_3_4_gives_5(self):
        odo = Odometry()
        assert odo.distance_to(3.0, 4.0) == pytest.approx(5.0, abs=1e-9)

    def test_distance_to_same_as_from_origin_when_at_origin(self):
        odo = Odometry()
        assert odo.distance_to(1.0, 0.0) == odo.distance_from_origin() + 1.0

    def test_distance_to_is_symmetric(self):
        """distance_to(a, b) from (0,0) == distance_to(0,0) from (a,b)."""
        odo = Odometry()
        odo.reset(0.0)
        odo.update(3.0, 0.0, 1.0)
        odo.update(4.0, math.pi / 2, 1.0)
        # Now at (3, 4); distance to origin
        d = odo.distance_to(0.0, 0.0)
        assert d == pytest.approx(5.0, abs=1e-9)

    def test_distance_to_nearby_waypoint(self):
        """Simulate intersection trigger: radius = 0.3, robot within range."""
        odo = Odometry()
        odo.reset(0.0)
        for _ in range(100):
            odo.update(1.0, 0.0, 0.01)   # x ≈ 1.0
        # Waypoint at x=1.05, y=0.0 — within 0.1 m
        d = odo.distance_to(1.05, 0.0)
        assert d < 0.3


# ─────────────────────────────────────────────────────────────────────────────
# 100-step integration accuracy
# ─────────────────────────────────────────────────────────────────────────────

class TestAccumulationAccuracy:

    def test_100hz_10second_forward_run(self):
        """100 Hz × 10 s forward at 0.6 m/s → x ≈ 6.0 m."""
        odo = Odometry()
        odo.reset(0.0)
        for _ in range(1000):
            odo.update(0.6, 0.0, 0.01)
        assert odo.x == pytest.approx(6.0, abs=1e-6)
        assert odo.y == pytest.approx(0.0, abs=1e-6)

    def test_circular_path_returns_to_origin(self):
        """Driving in a full circle should return near origin."""
        odo = Odometry()
        odo.reset(0.0)
        # 360 steps, each 1° = π/180 rad heading change, speed 1.0, dt tiny
        n = 360
        dt = 2 * math.pi / n   # dt such that arc length per step = 1 m at v=1
        for i in range(n):
            heading = (i / n) * 2 * math.pi
            odo.update(1.0, heading, dt)
        # After a full circle, should be back near (0,0)
        assert abs(odo.x) < 0.1
        assert abs(odo.y) < 0.1
