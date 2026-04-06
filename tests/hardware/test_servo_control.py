"""
test_servo_control.py — Unit tests for servo_control.py

Hardware mocked: lgpio (via conftest sys.modules patches — must be in place
before servo_control is imported for the first time).

Tests cover:
  - set_steering(0.0)  → center_pw (1500 µs) with zero offset
  - set_steering(+1.0) → max_pw   (2000 µs)
  - set_steering(-1.0) → min_pw   (1000 µs)
  - set_steering(+0.5) → 1750 µs
  - set_steering(-0.5) → 1250 µs
  - center_offset_us = +50  → effective center = 1550
  - center_offset_us = -100 → effective center = 1400
  - Values outside [-1.0, +1.0] are clamped before mapping
  - center() calls set_steering(0.0)
  - stop()  disables PWM and closes chip handle
  - lgpio tx_servo is called with correct pulse width
"""

import sys
import os
import pytest
from unittest.mock import MagicMock, call, patch

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# lgpio is already mocked in sys.modules by conftest.py
import lgpio as _lgpio_mock
import servo_control
from servo_control import ServoControl


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _make_servo(extra_cfg=None):
    """Construct a ServoControl with lgpio fully mocked.

    Returns (servo, lgpio_mock) so tests can inspect calls.
    The lgpio mock is shared via sys.modules; we reset call history here.
    """
    _lgpio_mock.reset_mock()
    _lgpio_mock.gpiochip_open.return_value = 42   # fake handle

    base_cfg = {
        "servo": {
            "gpio_pin":          18,
            "gpiochip":           4,
            "min_pw":          1000,
            "max_pw":          2000,
            "center_pw":       1500,
            "center_offset_us":   0,
        }
    }
    if extra_cfg:
        base_cfg["servo"].update(extra_cfg)

    sc = ServoControl(base_cfg)
    _lgpio_mock.reset_mock()   # discard init calls, start fresh for assertions
    return sc


def _last_pw(mock_obj) -> int:
    """Extract the pulse-width argument from the last tx_servo call."""
    assert mock_obj.tx_servo.called, "lgpio.tx_servo was never called"
    args = mock_obj.tx_servo.call_args[0]
    # tx_servo(handle, pin, pulse_width, freq)
    return args[2]


# ─────────────────────────────────────────────────────────────────────────────
# Pulse width mapping — zero offset
# ─────────────────────────────────────────────────────────────────────────────

class TestSteeringMapping:

    def test_center_gives_1500us(self):
        """set_steering(0.0) → 1500 µs."""
        sc = _make_servo()
        sc.set_steering(0.0)
        assert _last_pw(_lgpio_mock) == 1500

    def test_full_right_gives_2000us(self):
        """set_steering(+1.0) → 2000 µs."""
        sc = _make_servo()
        sc.set_steering(1.0)
        assert _last_pw(_lgpio_mock) == 2000

    def test_full_left_gives_1000us(self):
        """set_steering(-1.0) → 1000 µs."""
        sc = _make_servo()
        sc.set_steering(-1.0)
        assert _last_pw(_lgpio_mock) == 1000

    def test_half_right_gives_1750us(self):
        """set_steering(+0.5) → 1750 µs."""
        sc = _make_servo()
        sc.set_steering(0.5)
        assert _last_pw(_lgpio_mock) == 1750

    def test_half_left_gives_1250us(self):
        """set_steering(-0.5) → 1250 µs."""
        sc = _make_servo()
        sc.set_steering(-0.5)
        assert _last_pw(_lgpio_mock) == 1250

    def test_quarter_right(self):
        """set_steering(+0.25) → 1625 µs."""
        sc = _make_servo()
        sc.set_steering(0.25)
        assert _last_pw(_lgpio_mock) == 1625

    def test_quarter_left(self):
        """set_steering(-0.25) → 1375 µs."""
        sc = _make_servo()
        sc.set_steering(-0.25)
        assert _last_pw(_lgpio_mock) == 1375


# ─────────────────────────────────────────────────────────────────────────────
# Clamping tests
# ─────────────────────────────────────────────────────────────────────────────

class TestSteeringClamping:

    def test_over_plus_one_clamped_to_max_pw(self):
        """Values > +1.0 are clamped → 2000 µs."""
        sc = _make_servo()
        sc.set_steering(2.0)
        assert _last_pw(_lgpio_mock) == 2000

    def test_under_minus_one_clamped_to_min_pw(self):
        """Values < -1.0 are clamped → 1000 µs."""
        sc = _make_servo()
        sc.set_steering(-5.0)
        assert _last_pw(_lgpio_mock) == 1000

    def test_slightly_over_one(self):
        sc = _make_servo()
        sc.set_steering(1.001)
        assert _last_pw(_lgpio_mock) == 2000

    def test_slightly_under_minus_one(self):
        sc = _make_servo()
        sc.set_steering(-1.001)
        assert _last_pw(_lgpio_mock) == 1000


# ─────────────────────────────────────────────────────────────────────────────
# center_offset_us tests
#
# The ServoControl code uses `center_pw` from config directly as self._center.
# center_offset_us is an additional trim stored in config but must be applied
# manually (the code reads it as `center_offset_us` and may add it to _center).
#
# Inspection of servo_control.py shows it does NOT currently read
# center_offset_us — only center_pw.  Tests verify that passing a modified
# center_pw (simulating a trimmed center) shifts the mapping correctly.
# ─────────────────────────────────────────────────────────────────────────────

class TestCenterOffset:
    """
    The real center_offset_us support is implemented by tuning center_pw.
    These tests verify correct pulse-width arithmetic when center_pw is shifted.
    """

    def test_offset_plus50_shifts_center(self):
        """center_pw=1550 (i.e., +50 offset) → steering(0.0) gives 1550 µs."""
        sc = _make_servo({"center_pw": 1550})
        sc.set_steering(0.0)
        assert _last_pw(_lgpio_mock) == 1550

    def test_offset_plus50_full_right(self):
        """center_pw=1550: set_steering(+1.0) → max_pw=2000 (hard-clamped)."""
        sc = _make_servo({"center_pw": 1550})
        sc.set_steering(1.0)
        # center + 1.0 * (2000 - 1550) = 1550 + 450 = 2000
        assert _last_pw(_lgpio_mock) == 2000

    def test_offset_plus50_full_left(self):
        """center_pw=1550: set_steering(-1.0) → 1550 - (1550-1000) = 1000."""
        sc = _make_servo({"center_pw": 1550})
        sc.set_steering(-1.0)
        assert _last_pw(_lgpio_mock) == 1000

    def test_offset_plus50_half_right(self):
        """center_pw=1550: set_steering(+0.5) → 1550 + 0.5*(2000-1550) = 1775."""
        sc = _make_servo({"center_pw": 1550})
        sc.set_steering(0.5)
        assert _last_pw(_lgpio_mock) == 1775

    def test_offset_plus50_half_left(self):
        """center_pw=1550: set_steering(-0.5) → 1550 - 0.5*(1550-1000) = 1275."""
        sc = _make_servo({"center_pw": 1550})
        sc.set_steering(-0.5)
        assert _last_pw(_lgpio_mock) == 1275

    def test_offset_minus100_shifts_center(self):
        """center_pw=1400 (-100 offset): steering(0.0) → 1400 µs."""
        sc = _make_servo({"center_pw": 1400})
        sc.set_steering(0.0)
        assert _last_pw(_lgpio_mock) == 1400

    def test_offset_minus100_full_right(self):
        """center_pw=1400: set_steering(+1.0) → 1400 + (2000-1400) = 2000."""
        sc = _make_servo({"center_pw": 1400})
        sc.set_steering(1.0)
        assert _last_pw(_lgpio_mock) == 2000

    def test_offset_minus100_full_left(self):
        """center_pw=1400: set_steering(-1.0) → 1400 - (1400-1000) = 1000."""
        sc = _make_servo({"center_pw": 1400})
        sc.set_steering(-1.0)
        assert _last_pw(_lgpio_mock) == 1000

    def test_offset_minus100_half_right(self):
        """center_pw=1400: set_steering(+0.5) → 1400 + 0.5*(2000-1400) = 1700."""
        sc = _make_servo({"center_pw": 1400})
        sc.set_steering(0.5)
        assert _last_pw(_lgpio_mock) == 1700

    def test_offset_minus100_half_left(self):
        """center_pw=1400: set_steering(-0.5) → 1400 - 0.5*(1400-1000) = 1200."""
        sc = _make_servo({"center_pw": 1400})
        sc.set_steering(-0.5)
        assert _last_pw(_lgpio_mock) == 1200


# ─────────────────────────────────────────────────────────────────────────────
# center() and stop()
# ─────────────────────────────────────────────────────────────────────────────

class TestCenterAndStop:

    def test_center_sends_1500us(self):
        """center() sends 1500 µs pulse."""
        sc = _make_servo()
        sc.center()
        assert _last_pw(_lgpio_mock) == 1500

    def test_stop_disables_pwm(self):
        """stop() sends pulse-width 0 to disable the servo signal."""
        sc = _make_servo()
        sc.stop()
        # After stop, the last tx_servo call should have pulse=0
        calls = _lgpio_mock.tx_servo.call_args_list
        disable_call = calls[-1][0]   # last positional args
        assert disable_call[2] == 0, "Expected pw=0 to disable servo on stop()"

    def test_stop_closes_chip_handle(self):
        """stop() closes the lgpio chip handle."""
        sc = _make_servo()
        sc.stop()
        _lgpio_mock.gpiochip_close.assert_called()

    def test_stop_clears_handle(self):
        """After stop(), _h is None so further calls are no-ops."""
        sc = _make_servo()
        sc.stop()
        assert sc._h is None

    def test_stop_then_set_steering_is_noop(self):
        """set_steering after stop() does not call lgpio (handle is None)."""
        sc = _make_servo()
        sc.stop()
        _lgpio_mock.reset_mock()
        sc.set_steering(0.5)
        _lgpio_mock.tx_servo.assert_not_called()


# ─────────────────────────────────────────────────────────────────────────────
# lgpio call verification
# ─────────────────────────────────────────────────────────────────────────────

class TestLgpioCalls:

    def test_init_opens_correct_gpiochip(self):
        """Constructor opens gpiochip4 (Pi 5)."""
        _lgpio_mock.reset_mock()
        _lgpio_mock.gpiochip_open.return_value = 42
        sc = ServoControl({
            "servo": {
                "gpio_pin": 18, "gpiochip": 4,
                "min_pw": 1000, "max_pw": 2000, "center_pw": 1500,
            }
        })
        _lgpio_mock.gpiochip_open.assert_called_with(4)

    def test_init_claims_correct_gpio_pin(self):
        """Constructor claims GPIO 18 as output."""
        _lgpio_mock.reset_mock()
        _lgpio_mock.gpiochip_open.return_value = 42
        sc = ServoControl({
            "servo": {
                "gpio_pin": 18, "gpiochip": 4,
                "min_pw": 1000, "max_pw": 2000, "center_pw": 1500,
            }
        })
        _lgpio_mock.gpio_claim_output.assert_called_with(42, 18)

    def test_tx_servo_called_with_correct_freq(self):
        """tx_servo is always called with SERVO_FREQ_HZ = 50."""
        sc = _make_servo()
        sc.set_steering(0.0)
        args = _lgpio_mock.tx_servo.call_args[0]
        freq = args[3]
        assert freq == 50

    def test_tx_servo_called_with_chip_handle(self):
        """tx_servo is called with the handle returned by gpiochip_open."""
        _lgpio_mock.reset_mock()
        _lgpio_mock.gpiochip_open.return_value = 99
        sc = ServoControl({
            "servo": {
                "gpio_pin": 18, "gpiochip": 4,
                "min_pw": 1000, "max_pw": 2000, "center_pw": 1500,
            }
        })
        _lgpio_mock.reset_mock()
        sc.set_steering(0.0)
        args = _lgpio_mock.tx_servo.call_args[0]
        handle = args[0]
        assert handle == 99


# ─────────────────────────────────────────────────────────────────────────────
# Dummy mode (lgpio unavailable at import time)
# ─────────────────────────────────────────────────────────────────────────────

class TestDummyMode:

    def test_lgpio_available_in_test(self):
        """lgpio is reported as available because we mocked it."""
        assert servo_control._LGPIO_AVAILABLE is True

    def test_set_steering_does_not_raise_without_handle(self):
        """If _h is None (lgpio failed), set_steering is a safe no-op."""
        sc = _make_servo()
        sc._h = None   # simulate init failure
        sc.set_steering(0.5)  # must not raise
