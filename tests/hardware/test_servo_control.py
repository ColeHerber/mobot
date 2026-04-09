"""
test_servo_control.py — Unit tests for servo_control.py

Hardware mocked: sysfs /sys/class/pwm/ (via os.path.exists + builtins.open patches).

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
  - stop() disables PWM (writes enable=0)
  - set_steering after stop() is a safe no-op
"""

import os
import sys
import pytest
from unittest.mock import patch, MagicMock

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import servo_control
from servo_control import ServoControl


# ─────────────────────────────────────────────────────────────────────────────
# Sysfs mock infrastructure
# ─────────────────────────────────────────────────────────────────────────────

class SysfsMock:
    """Context manager that mocks os.path.exists and builtins.open for sysfs.

    Tracks all writes by sysfs filename (basename of path). Use
    last_duty_us() to check the most recent duty_cycle write in µs.
    """

    def __init__(self, chip_exists=True, pwm_dir_exists=True):
        self.written = {}
        self._chip_exists   = chip_exists
        self._pwm_dir_exists = pwm_dir_exists
        self._patches = []

    def __enter__(self):
        written = self.written

        def fake_exists(path):
            if "pwmchip" in path and "pwm0" not in path.split("/")[-1]:
                return self._chip_exists
            return self._pwm_dir_exists

        def fake_open(path, mode="r"):
            filename = os.path.basename(path)
            m = MagicMock()
            m.__enter__ = lambda s: s
            m.__exit__ = MagicMock(return_value=False)
            m.write = lambda val: written.__setitem__(filename, val)
            return m

        p1 = patch("os.path.exists", side_effect=fake_exists)
        p2 = patch("builtins.open",  side_effect=fake_open)
        self._patches = [p1, p2]
        for p in self._patches:
            p.start()
        return self

    def __exit__(self, *args):
        for p in self._patches:
            p.stop()

    def make_servo(self, extra_cfg=None):
        """Construct a ServoControl; clears written dict afterwards."""
        cfg = {
            "servo": {
                "gpio_pin":          12,
                "pwmchip":            2,
                "pwm_channel":        0,
                "min_pw":          1000,
                "max_pw":          2000,
                "center_pw":       1500,
                "center_offset_us":   0,
            }
        }
        if extra_cfg:
            cfg["servo"].update(extra_cfg)
        sc = ServoControl(cfg)
        self.written.clear()   # discard init writes; start fresh for assertions
        return sc

    def last_duty_us(self) -> int:
        """Return the last duty_cycle write, converted from ns to µs."""
        ns = self.written.get("duty_cycle")
        assert ns is not None, "duty_cycle was never written to sysfs"
        return int(ns) // 1000


# ─────────────────────────────────────────────────────────────────────────────
# Pulse width mapping — zero offset
# ─────────────────────────────────────────────────────────────────────────────

class TestSteeringMapping:

    def test_center_gives_1500us(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.set_steering(0.0)
            assert s.last_duty_us() == 1500

    def test_full_right_gives_2000us(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.set_steering(1.0)
            assert s.last_duty_us() == 2000

    def test_full_left_gives_1000us(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.set_steering(-1.0)
            assert s.last_duty_us() == 1000

    def test_half_right_gives_1750us(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.set_steering(0.5)
            assert s.last_duty_us() == 1750

    def test_half_left_gives_1250us(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.set_steering(-0.5)
            assert s.last_duty_us() == 1250

    def test_quarter_right(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.set_steering(0.25)
            assert s.last_duty_us() == 1625

    def test_quarter_left(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.set_steering(-0.25)
            assert s.last_duty_us() == 1375


# ─────────────────────────────────────────────────────────────────────────────
# Clamping
# ─────────────────────────────────────────────────────────────────────────────

class TestSteeringClamping:

    def test_over_plus_one_clamped_to_max_pw(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.set_steering(2.0)
            assert s.last_duty_us() == 2000

    def test_under_minus_one_clamped_to_min_pw(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.set_steering(-5.0)
            assert s.last_duty_us() == 1000

    def test_slightly_over_one(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.set_steering(1.001)
            assert s.last_duty_us() == 2000

    def test_slightly_under_minus_one(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.set_steering(-1.001)
            assert s.last_duty_us() == 1000


# ─────────────────────────────────────────────────────────────────────────────
# center_offset_us
# ─────────────────────────────────────────────────────────────────────────────

class TestCenterOffset:

    def test_offset_plus50_shifts_center(self):
        with SysfsMock() as s:
            sc = s.make_servo({"center_offset_us": 50})
            sc.set_steering(0.0)
            assert s.last_duty_us() == 1550

    def test_offset_plus50_full_right(self):
        with SysfsMock() as s:
            sc = s.make_servo({"center_offset_us": 50})
            sc.set_steering(1.0)
            # 1550 + 1.0 * (2000 - 1550) = 2000
            assert s.last_duty_us() == 2000

    def test_offset_plus50_full_left(self):
        with SysfsMock() as s:
            sc = s.make_servo({"center_offset_us": 50})
            sc.set_steering(-1.0)
            # 1550 - 1.0 * (1550 - 1000) = 1000
            assert s.last_duty_us() == 1000

    def test_offset_plus50_half_right(self):
        with SysfsMock() as s:
            sc = s.make_servo({"center_offset_us": 50})
            sc.set_steering(0.5)
            # 1550 + 0.5 * (2000 - 1550) = 1775
            assert s.last_duty_us() == 1775

    def test_offset_plus50_half_left(self):
        with SysfsMock() as s:
            sc = s.make_servo({"center_offset_us": 50})
            sc.set_steering(-0.5)
            # 1550 - 0.5 * (1550 - 1000) = 1275
            assert s.last_duty_us() == 1275

    def test_offset_minus100_shifts_center(self):
        with SysfsMock() as s:
            sc = s.make_servo({"center_offset_us": -100})
            sc.set_steering(0.0)
            assert s.last_duty_us() == 1400

    def test_offset_minus100_full_right(self):
        with SysfsMock() as s:
            sc = s.make_servo({"center_offset_us": -100})
            sc.set_steering(1.0)
            # 1400 + 1.0 * (2000 - 1400) = 2000
            assert s.last_duty_us() == 2000

    def test_offset_minus100_full_left(self):
        with SysfsMock() as s:
            sc = s.make_servo({"center_offset_us": -100})
            sc.set_steering(-1.0)
            # 1400 - 1.0 * (1400 - 1000) = 1000
            assert s.last_duty_us() == 1000

    def test_offset_minus100_half_right(self):
        with SysfsMock() as s:
            sc = s.make_servo({"center_offset_us": -100})
            sc.set_steering(0.5)
            # 1400 + 0.5 * (2000 - 1400) = 1700
            assert s.last_duty_us() == 1700

    def test_offset_minus100_half_left(self):
        with SysfsMock() as s:
            sc = s.make_servo({"center_offset_us": -100})
            sc.set_steering(-0.5)
            # 1400 - 0.5 * (1400 - 1000) = 1200
            assert s.last_duty_us() == 1200


# ─────────────────────────────────────────────────────────────────────────────
# center() and stop()
# ─────────────────────────────────────────────────────────────────────────────

class TestCenterAndStop:

    def test_center_sends_1500us(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.center()
            assert s.last_duty_us() == 1500

    def test_stop_writes_enable_0(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.stop()
            assert s.written.get("enable") == 0

    def test_stop_clears_ready_flag(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.stop()
            assert sc._ready is False

    def test_stop_then_set_steering_is_noop(self):
        with SysfsMock() as s:
            sc = s.make_servo()
            sc.stop()
            s.written.clear()
            sc.set_steering(0.5)
            assert "duty_cycle" not in s.written


# ─────────────────────────────────────────────────────────────────────────────
# Sysfs init sequence
# ─────────────────────────────────────────────────────────────────────────────

class TestSysfsInit:

    def test_period_written_on_init(self):
        with SysfsMock() as s:
            ServoControl({"servo": {
                "gpio_pin": 12, "pwmchip": 2, "pwm_channel": 0,
                "min_pw": 1000, "max_pw": 2000, "center_pw": 1500,
            }})
            # 50 Hz = 20 ms = 20_000_000 ns
            assert s.written.get("period") == 20_000_000

    def test_enable_written_1_on_init(self):
        with SysfsMock() as s:
            ServoControl({"servo": {
                "gpio_pin": 12, "pwmchip": 2, "pwm_channel": 0,
                "min_pw": 1000, "max_pw": 2000, "center_pw": 1500,
            }})
            assert s.written.get("enable") == 1

    def test_no_chip_dir_stays_not_ready(self):
        with SysfsMock(chip_exists=False, pwm_dir_exists=False) as s:
            sc = ServoControl({"servo": {
                "gpio_pin": 12, "pwmchip": 2, "pwm_channel": 0,
                "min_pw": 1000, "max_pw": 2000, "center_pw": 1500,
            }})
            assert sc._ready is False

    def test_set_steering_noop_when_not_ready(self):
        with SysfsMock(chip_exists=False, pwm_dir_exists=False) as s:
            sc = ServoControl({"servo": {
                "gpio_pin": 12, "pwmchip": 2, "pwm_channel": 0,
                "min_pw": 1000, "max_pw": 2000, "center_pw": 1500,
            }})
            s.written.clear()
            sc.set_steering(0.5)
            assert "duty_cycle" not in s.written
