"""
test_pid_controller.py — Unit tests for pid_controller.py

Tests cover:
  - Steering direction and proportionality
  - Derivative-on-measurement behaviour
  - Adaptive speed: ramps up when on line, ramps down when off
  - Speed floor (min_speed) and ceiling (base_speed)
  - reset() clears state including adaptive speed
  - dt guard (dt <= 0 uses fallback)
  - Output clamping [-1.0, +1.0] for steering
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

def _make_pid(kp=1.2, kd=0.08,
              base_ms=0.6, min_ms=0.2,
              speed_ramp_up=0.2, speed_ramp_down=0.8,
              speed_conf_min=80, speed_pos_max=0.3):
    config = {
        "pid": {
            "kp": kp, "kd": kd,
            "speed_ramp_up":   speed_ramp_up,
            "speed_ramp_down": speed_ramp_down,
            "speed_conf_min":  speed_conf_min,
            "speed_pos_max":   speed_pos_max,
        },
        "speed": {"base_ms": base_ms, "min_ms": min_ms},
    }
    return PIDController(config)

CONF_GOOD = 255   # high confidence — "on line"
CONF_NONE = 0     # zero confidence — "off line"


# ─────────────────────────────────────────────────────────────────────────────
# Steering direction tests
# ─────────────────────────────────────────────────────────────────────────────

class TestSteeringDirection:

    def test_centered_gives_zero_steering(self):
        pid = _make_pid()
        steering, _ = pid.compute(0.0, CONF_GOOD, 0.01)
        assert steering == pytest.approx(0.0)

    def test_robot_right_of_line_steers_left(self):
        pid = _make_pid()
        steering, _ = pid.compute(0.5, CONF_GOOD, 0.01)
        assert steering < 0.0

    def test_robot_left_of_line_steers_right(self):
        pid = _make_pid()
        steering, _ = pid.compute(-0.5, CONF_GOOD, 0.01)
        assert steering > 0.0

    def test_full_right_gives_negative_steering(self):
        pid = _make_pid()
        steering, _ = pid.compute(1.0, CONF_GOOD, 0.01)
        assert steering < -0.5

    def test_full_left_gives_positive_steering(self):
        pid = _make_pid()
        steering, _ = pid.compute(-1.0, CONF_GOOD, 0.01)
        assert steering > 0.5

    def test_steering_proportional_to_error(self):
        pid1 = _make_pid(kd=0.0)
        pid2 = _make_pid(kd=0.0)
        s_small, _ = pid1.compute(0.1, CONF_GOOD, 0.01)
        s_large, _ = pid2.compute(0.5, CONF_GOOD, 0.01)
        assert abs(s_large) > abs(s_small)

    def test_steering_symmetry(self):
        pid1 = _make_pid(kd=0.0)
        pid2 = _make_pid(kd=0.0)
        s_pos, _ = pid1.compute(+0.3, CONF_GOOD, 0.01)
        s_neg, _ = pid2.compute(-0.3, CONF_GOOD, 0.01)
        assert s_pos == pytest.approx(-s_neg, abs=1e-9)


# ─────────────────────────────────────────────────────────────────────────────
# Derivative-on-measurement tests
# ─────────────────────────────────────────────────────────────────────────────

class TestDerivativeTerm:

    def test_derivative_smooths_rapid_change(self):
        pid = _make_pid(kp=1.0, kd=1.0)
        pid.compute(0.5, CONF_GOOD, 0.01)
        s2, _ = pid.compute(0.0, CONF_GOOD, 0.01)
        assert s2 > 0.0

    def test_first_call_no_derivative_history(self):
        pid = _make_pid(kp=0.0, kd=1.0)
        steering, _ = pid.compute(0.0, CONF_GOOD, 0.01)
        assert steering == pytest.approx(0.0)

    def test_derivative_direction_on_increasing_error(self):
        pid = _make_pid(kp=0.0, kd=1.0)
        pid.compute(0.0, CONF_GOOD, 0.01)
        steering, _ = pid.compute(0.5, CONF_GOOD, 0.01)
        assert steering < 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Adaptive speed tests
# ─────────────────────────────────────────────────────────────────────────────

class TestAdaptiveSpeed:

    def test_starts_at_min_speed(self):
        """Fresh PID starts at min_speed before any on-line time."""
        pid = _make_pid(base_ms=0.6, min_ms=0.2)
        # First call off-line: stays at min_speed
        _, throttle = pid.compute(0.0, CONF_NONE, 0.01)
        assert throttle == pytest.approx(0.2)

    def test_ramps_up_on_line(self):
        """Speed increases each tick when well-centered with good confidence."""
        pid = _make_pid(base_ms=0.6, min_ms=0.2, speed_ramp_up=1.0, speed_conf_min=50)
        # After 0.4 s of on-line time (40 ticks × dt=0.01) ramp=1.0 → +0.4 m/s
        for _ in range(40):
            _, throttle = pid.compute(0.0, 255, 0.01)
        assert throttle > 0.2 + 0.3  # well above min_speed

    def test_reaches_base_speed(self):
        """Speed caps at base_speed no matter how long on line."""
        pid = _make_pid(base_ms=0.6, min_ms=0.2, speed_ramp_up=10.0, speed_conf_min=50)
        for _ in range(100):
            _, throttle = pid.compute(0.0, 255, 0.01)
        assert throttle == pytest.approx(0.6)

    def test_ramps_down_off_line(self):
        """Speed drops when confidence is low."""
        pid = _make_pid(base_ms=0.6, min_ms=0.2,
                        speed_ramp_up=10.0, speed_ramp_down=10.0, speed_conf_min=50)
        # Ramp up to base
        for _ in range(50):
            pid.compute(0.0, 255, 0.01)
        # Now go off-line
        for _ in range(5):
            _, throttle = pid.compute(0.0, 0, 0.01)
        assert throttle < 0.6

    def test_floors_at_min_speed(self):
        """Speed never drops below min_speed even with sustained off-line."""
        pid = _make_pid(base_ms=0.6, min_ms=0.2, speed_ramp_down=100.0)
        for _ in range(100):
            _, throttle = pid.compute(0.0, CONF_NONE, 0.01)
        assert throttle >= 0.2

    def test_large_line_pos_counts_as_off_line(self):
        """|line_pos| > speed_pos_max → ramp down even with good confidence."""
        pid = _make_pid(base_ms=0.6, min_ms=0.2,
                        speed_ramp_up=10.0, speed_ramp_down=10.0,
                        speed_pos_max=0.3, speed_conf_min=50)
        # Get to base speed
        for _ in range(50):
            pid.compute(0.0, 255, 0.01)
        # Wide deviation → off-line
        for _ in range(5):
            _, throttle = pid.compute(0.8, 255, 0.01)
        assert throttle < 0.6

    def test_speed_symmetric_for_left_right(self):
        """Equal magnitude left/right deviation → same throttle."""
        pid1 = _make_pid()
        pid2 = _make_pid()
        _, t_pos = pid1.compute(+0.4, CONF_GOOD, 0.01)
        _, t_neg = pid2.compute(-0.4, CONF_GOOD, 0.01)
        assert t_pos == pytest.approx(t_neg, abs=1e-9)


# ─────────────────────────────────────────────────────────────────────────────
# Output clamping
# ─────────────────────────────────────────────────────────────────────────────

class TestOutputClamping:

    def test_steering_clamped_plus_one(self):
        pid = _make_pid(kp=10.0, kd=0.0)
        steering, _ = pid.compute(-1.0, CONF_GOOD, 0.01)
        assert steering == pytest.approx(1.0)

    def test_steering_clamped_minus_one(self):
        pid = _make_pid(kp=10.0, kd=0.0)
        steering, _ = pid.compute(1.0, CONF_GOOD, 0.01)
        assert steering == pytest.approx(-1.0)

    def test_steering_within_bounds_normal_gain(self):
        pid = _make_pid()
        steering, _ = pid.compute(0.3, CONF_GOOD, 0.01)
        assert -1.0 <= steering <= 1.0


# ─────────────────────────────────────────────────────────────────────────────
# reset()
# ─────────────────────────────────────────────────────────────────────────────

class TestReset:

    def test_reset_clears_adaptive_speed(self):
        """After reset, adaptive speed returns to min_speed."""
        pid = _make_pid(base_ms=0.6, min_ms=0.2, speed_ramp_up=10.0, speed_conf_min=50)
        for _ in range(50):
            pid.compute(0.0, 255, 0.01)
        pid.reset()
        assert pid._adaptive_speed == pytest.approx(0.2)

    def test_reset_clears_derivative_history(self):
        pid = _make_pid(kp=0.0, kd=1.0)
        pid.compute(0.5, CONF_GOOD, 0.01)
        pid.reset()
        steering, _ = pid.compute(0.5, CONF_GOOD, 0.01)
        assert steering < 0.0

    def test_reset_clears_prev_line_pos(self):
        pid = _make_pid()
        pid.compute(0.8, CONF_GOOD, 0.01)
        pid.reset()
        assert pid._prev_line_pos == pytest.approx(0.0)


# ─────────────────────────────────────────────────────────────────────────────
# dt guard
# ─────────────────────────────────────────────────────────────────────────────

class TestDtGuard:

    def test_zero_dt_uses_fallback(self):
        pid = _make_pid()
        steering, throttle = pid.compute(0.3, CONF_GOOD, 0.0)
        assert -1.0 <= steering <= 1.0
        assert throttle >= 0.0

    def test_negative_dt_uses_fallback(self):
        pid = _make_pid()
        steering, throttle = pid.compute(0.3, CONF_GOOD, -0.01)
        assert -1.0 <= steering <= 1.0


# ─────────────────────────────────────────────────────────────────────────────
# Config loading
# ─────────────────────────────────────────────────────────────────────────────

class TestConfigLoading:

    def test_default_gains_loaded(self):
        pid = _make_pid(kp=1.2, kd=0.08)
        assert pid.kp == pytest.approx(1.2)
        assert pid.kd == pytest.approx(0.08)

    def test_default_speeds_loaded(self):
        pid = _make_pid(base_ms=0.6, min_ms=0.2)
        assert pid.base_speed == pytest.approx(0.6)
        assert pid.min_speed  == pytest.approx(0.2)

    def test_empty_config_uses_code_defaults(self):
        pid = PIDController({})
        assert pid.kp == pytest.approx(1.2)
        assert pid.kd == pytest.approx(0.08)
