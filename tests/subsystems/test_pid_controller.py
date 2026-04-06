"""
test_pid_controller.py — Unit tests for pid_controller.py

Tests cover:
  - compute(0.0, dt) → steering ≈ 0.0 (centered)
  - compute(positive_pos, dt) → negative steering (corrects right deviation)
  - compute(negative_pos, dt) → positive steering (corrects left deviation)
  - Derivative-on-measurement: direction and sign
  - Speed reduction on large line error
  - Throttle flooring at min_speed
  - Throttle ceiling at base_speed
  - reset() clears previous state
  - dt guard (dt <= 0 uses fallback)
  - Output clamping [-1.0, +1.0] for extreme errors
"""

import sys
import os
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from pid_controller import PIDController


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _make_pid(kp=1.2, kd=0.08, speed_kp=0.5,
              base_ms=0.6, min_ms=0.2, gate_ms=0.3):
    config = {
        "pid": {"kp": kp, "kd": kd, "speed_kp": speed_kp},
        "speed": {"base_ms": base_ms, "min_ms": min_ms, "gate_ms": gate_ms},
    }
    return PIDController(config)


# ─────────────────────────────────────────────────────────────────────────────
# Steering direction tests
# ─────────────────────────────────────────────────────────────────────────────

class TestSteeringDirection:

    def test_centered_gives_zero_steering(self):
        """line_pos=0.0 (centered) → error=0, derivative=0 → steering=0."""
        pid = _make_pid()
        # First call: prev_line_pos defaults to 0.0, so derivative=0 too
        steering, _ = pid.compute(0.0, 0.01)
        assert steering == pytest.approx(0.0)

    def test_robot_right_of_line_steers_left(self):
        """line_pos = +0.5 (robot right of center) → negative steering (steer left)."""
        pid = _make_pid()
        steering, _ = pid.compute(0.5, 0.01)
        assert steering < 0.0, f"Expected negative steering, got {steering}"

    def test_robot_left_of_line_steers_right(self):
        """line_pos = -0.5 (robot left of center) → positive steering (steer right)."""
        pid = _make_pid()
        steering, _ = pid.compute(-0.5, 0.01)
        assert steering > 0.0, f"Expected positive steering, got {steering}"

    def test_full_right_gives_negative_steering(self):
        """line_pos = +1.0 → strongly negative steering."""
        pid = _make_pid()
        steering, _ = pid.compute(1.0, 0.01)
        assert steering < -0.5

    def test_full_left_gives_positive_steering(self):
        """line_pos = -1.0 → strongly positive steering."""
        pid = _make_pid()
        steering, _ = pid.compute(-1.0, 0.01)
        assert steering > 0.5

    def test_steering_proportional_to_error(self):
        """Larger error → larger steering magnitude (P term dominates, no derivative)."""
        pid1 = _make_pid(kd=0.0)  # no derivative
        pid2 = _make_pid(kd=0.0)
        s_small, _ = pid1.compute(0.1, 0.01)
        s_large, _ = pid2.compute(0.5, 0.01)
        assert abs(s_large) > abs(s_small)

    def test_steering_symmetry(self):
        """Equal and opposite errors → equal and opposite steering."""
        pid1 = _make_pid(kd=0.0)
        pid2 = _make_pid(kd=0.0)
        s_pos, _ = pid1.compute(+0.3, 0.01)
        s_neg, _ = pid2.compute(-0.3, 0.01)
        assert s_pos == pytest.approx(-s_neg, abs=1e-9)


# ─────────────────────────────────────────────────────────────────────────────
# Derivative-on-measurement tests
# ─────────────────────────────────────────────────────────────────────────────

class TestDerivativeTerm:

    def test_derivative_smooths_rapid_change(self):
        """
        Line position moving toward center should produce a derivative that
        partially counteracts the P term (braking effect).
        """
        pid = _make_pid(kp=1.0, kd=1.0)
        # Step 1: error=0.5 → P only (no history yet, derivative on measurement = 0)
        s1, _ = pid.compute(0.5, 0.01)
        # Step 2: error=0.0 → line_pos moved 0.5 in 0.01s → derivative huge positive
        #   measurement_delta = 0.0 - 0.5 = -0.5 → derivative = -(-0.5)/0.01 = +50
        #   steering = kp*(0.0) + kd*(50) = +50 → but clamped to 1.0
        s2, _ = pid.compute(0.0, 0.01)
        # Second step should steer right (positive) due to fast convergence derivative
        assert s2 > 0.0, "Expected positive derivative kick when line_pos moves to 0"

    def test_first_call_no_derivative_history(self):
        """First call has no previous position history → derivative = 0."""
        pid = _make_pid(kp=0.0, kd=1.0)  # P=0, only derivative
        # prev_line_pos starts at 0.0; if line_pos=0.0, derivative=0, steering=0
        steering, _ = pid.compute(0.0, 0.01)
        assert steering == pytest.approx(0.0)

    def test_derivative_direction_on_increasing_error(self):
        """Line moving away from center → derivative amplifies steering correction."""
        pid = _make_pid(kp=0.0, kd=1.0)
        # Start at 0.0, then see line_pos = 0.5 (moving right)
        pid.compute(0.0, 0.01)   # prime the history
        # Now line_pos moves to 0.5: measurement_delta = 0.5; derivative = -0.5/0.01 = -50
        steering, _ = pid.compute(0.5, 0.01)
        # derivative is negative → steering is negative (trying to go left, correct for rightward drift)
        assert steering < 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Throttle / speed control tests
# ─────────────────────────────────────────────────────────────────────────────

class TestThrottleControl:

    def test_centered_gives_base_speed(self):
        """line_pos=0.0 → throttle == base_speed."""
        pid = _make_pid(base_ms=0.6)
        _, throttle = pid.compute(0.0, 0.01)
        assert throttle == pytest.approx(0.6)

    def test_large_error_reduces_speed(self):
        """line_pos=1.0 (full offset) → throttle < base_speed."""
        pid = _make_pid(base_ms=0.6, speed_kp=0.5)
        _, throttle = pid.compute(1.0, 0.01)
        expected = 0.6 - 0.5 * 1.0 * 0.6   # = 0.6 - 0.3 = 0.3
        assert throttle == pytest.approx(expected, abs=1e-9)

    def test_throttle_floored_at_min_speed(self):
        """Extreme speed reduction is clamped to min_ms."""
        pid = _make_pid(base_ms=0.6, min_ms=0.2, speed_kp=5.0)
        _, throttle = pid.compute(1.0, 0.01)
        assert throttle >= 0.2

    def test_throttle_ceiling_at_base_speed(self):
        """Throttle never exceeds base_ms."""
        pid = _make_pid(base_ms=0.6)
        _, throttle = pid.compute(-0.01, 0.01)
        assert throttle <= 0.6

    def test_throttle_reduction_symmetric(self):
        """Positive and negative line_pos of same magnitude → same throttle."""
        pid1 = _make_pid()
        pid2 = _make_pid()
        _, t_pos = pid1.compute(+0.4, 0.01)
        _, t_neg = pid2.compute(-0.4, 0.01)
        assert t_pos == pytest.approx(t_neg, abs=1e-9)

    def test_throttle_formula_explicit(self):
        """Verify exact throttle formula: throttle = base - speed_kp * |lp| * base."""
        base = 0.6
        speed_kp = 0.5
        lp = 0.3
        pid = _make_pid(base_ms=base, speed_kp=speed_kp)
        _, throttle = pid.compute(lp, 0.01)
        expected = base - speed_kp * abs(lp) * base
        assert throttle == pytest.approx(expected, abs=1e-9)


# ─────────────────────────────────────────────────────────────────────────────
# Output clamping
# ─────────────────────────────────────────────────────────────────────────────

class TestOutputClamping:

    def test_steering_clamped_plus_one(self):
        """Very large positive error → steering clamped at +1.0."""
        pid = _make_pid(kp=10.0, kd=0.0)
        steering, _ = pid.compute(-1.0, 0.01)   # error = +1.0 * kp=10 → +10, clamped
        assert steering == pytest.approx(1.0)

    def test_steering_clamped_minus_one(self):
        """Very large negative error → steering clamped at -1.0."""
        pid = _make_pid(kp=10.0, kd=0.0)
        steering, _ = pid.compute(1.0, 0.01)   # error = -1.0 * kp=10 → -10, clamped
        assert steering == pytest.approx(-1.0)

    def test_steering_within_bounds_normal_gain(self):
        """Normal gain, moderate error → steering in (-1, +1)."""
        pid = _make_pid()
        steering, _ = pid.compute(0.3, 0.01)
        assert -1.0 <= steering <= 1.0


# ─────────────────────────────────────────────────────────────────────────────
# reset()
# ─────────────────────────────────────────────────────────────────────────────

class TestReset:

    def test_reset_clears_derivative_history(self):
        """After reset(), derivative is zero on next call (no prior history)."""
        pid = _make_pid(kp=0.0, kd=1.0)
        pid.compute(0.5, 0.01)   # prime history
        pid.reset()
        # After reset, prev_line_pos=0; calling with 0.5 should look like first call
        # but now history has 0.0, so derivative = -(0.5 - 0.0)/dt = -50 → steering = -50 → -1.0 clamped
        steering, _ = pid.compute(0.5, 0.01)
        assert steering < 0.0  # derivative term is active post-reset

    def test_reset_clears_prev_error(self):
        """After reset(), prev_error is 0.0."""
        pid = _make_pid()
        pid.compute(0.5, 0.01)
        pid.reset()
        assert pid._prev_error == pytest.approx(0.0)

    def test_reset_clears_prev_line_pos(self):
        """After reset(), _prev_line_pos is 0.0."""
        pid = _make_pid()
        pid.compute(0.8, 0.01)
        pid.reset()
        assert pid._prev_line_pos == pytest.approx(0.0)


# ─────────────────────────────────────────────────────────────────────────────
# dt guard
# ─────────────────────────────────────────────────────────────────────────────

class TestDtGuard:

    def test_zero_dt_uses_fallback(self):
        """dt=0 uses internal fallback (0.01), does not divide by zero."""
        pid = _make_pid()
        steering, throttle = pid.compute(0.3, 0.0)   # should not raise
        assert -1.0 <= steering <= 1.0
        assert 0.0 <= throttle <= 1.0

    def test_negative_dt_uses_fallback(self):
        """Negative dt uses fallback."""
        pid = _make_pid()
        steering, throttle = pid.compute(0.3, -0.01)
        assert -1.0 <= steering <= 1.0


# ─────────────────────────────────────────────────────────────────────────────
# Config loading
# ─────────────────────────────────────────────────────────────────────────────

class TestConfigLoading:

    def test_default_gains_loaded(self):
        pid = _make_pid(kp=1.2, kd=0.08, speed_kp=0.5)
        assert pid.kp == pytest.approx(1.2)
        assert pid.kd == pytest.approx(0.08)
        assert pid.speed_kp == pytest.approx(0.5)

    def test_default_speeds_loaded(self):
        pid = _make_pid(base_ms=0.6, min_ms=0.2)
        assert pid.base_speed == pytest.approx(0.6)
        assert pid.min_speed == pytest.approx(0.2)

    def test_empty_config_uses_code_defaults(self):
        """Missing keys fall back to code defaults without crashing."""
        pid = PIDController({})
        assert pid.kp == pytest.approx(1.2)
        assert pid.kd == pytest.approx(0.08)
