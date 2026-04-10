"""
test_vesc_interface.py — Unit tests for vesc_interface.py

Hardware mocked: serial.Serial, pyvesc (via conftest sys.modules patches).
Tests cover:
  - _rpm_to_ms() conversion formula
  - set_throttle() clamping to [-1.0, +1.0]
  - Thread-safe throttle read-back
  - Instantiation with various config values
  - Dummy-mode operation when pyvesc unavailable
"""

import sys
import os
import threading
import pytest
from unittest.mock import MagicMock, patch, PropertyMock

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import vesc_interface
from vesc_interface import VESCInterface
from shared_state import SharedState


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _make_vesc(config=None):
    """Construct a VESCInterface without opening a real serial port."""
    if config is None:
        config = {
            "vesc": {
                "wheel_circumference_m": 0.204,
                "gear_ratio": 8.0,
            }
        }
    state = SharedState()
    with patch("serial.Serial"):
        vi = VESCInterface(state, config)
    return vi, state


# ─────────────────────────────────────────────────────────────────────────────
# _rpm_to_ms conversion tests
# ─────────────────────────────────────────────────────────────────────────────

class TestRpmToMs:
    """
    Formula: (rpm / gear_ratio / 60) * circumference
    Default: gear_ratio=8.0, circumference=0.204
    """

    def test_canonical_case(self):
        """1000 RPM, gear=8, circ=0.204 → ~0.425 m/s."""
        vi, _ = _make_vesc()
        result = vi._rpm_to_ms(1000.0)
        expected = (1000.0 / 8.0 / 60.0) * 0.204
        assert result == pytest.approx(expected, rel=1e-9)

    def test_canonical_case_exact_value(self):
        """Verify the exact numeric result of the canonical case."""
        vi, _ = _make_vesc()
        # (1000 / 8 / 60) * 0.204 = 125/60 * 0.204 = 2.0833... * 0.204 = 0.425
        assert vi._rpm_to_ms(1000.0) == pytest.approx(0.425, abs=1e-6)

    def test_zero_rpm(self):
        """0 RPM → 0.0 m/s."""
        vi, _ = _make_vesc()
        assert vi._rpm_to_ms(0.0) == pytest.approx(0.0)

    def test_negative_rpm(self):
        """Negative RPM (reverse) → negative velocity."""
        vi, _ = _make_vesc()
        fwd = vi._rpm_to_ms(1000.0)
        rev = vi._rpm_to_ms(-1000.0)
        assert rev == pytest.approx(-fwd)

    def test_proportional_to_rpm(self):
        """Doubling RPM doubles velocity (linear relationship)."""
        vi, _ = _make_vesc()
        v1 = vi._rpm_to_ms(500.0)
        v2 = vi._rpm_to_ms(1000.0)
        assert v2 == pytest.approx(2.0 * v1)

    def test_different_gear_ratio(self):
        """Higher gear ratio → lower velocity for same RPM."""
        config_low  = {"vesc": {"wheel_circumference_m": 0.204, "gear_ratio": 4.0}}
        config_high = {"vesc": {"wheel_circumference_m": 0.204, "gear_ratio": 16.0}}
        vi_low,  _ = _make_vesc(config_low)
        vi_high, _ = _make_vesc(config_high)
        assert vi_low._rpm_to_ms(1000.0) > vi_high._rpm_to_ms(1000.0)

    def test_different_circumference(self):
        """Larger wheel circumference → higher velocity for same RPM."""
        config_small = {"vesc": {"wheel_circumference_m": 0.1,  "gear_ratio": 8.0}}
        config_large = {"vesc": {"wheel_circumference_m": 0.3,  "gear_ratio": 8.0}}
        vi_small, _ = _make_vesc(config_small)
        vi_large, _ = _make_vesc(config_large)
        assert vi_small._rpm_to_ms(1000.0) < vi_large._rpm_to_ms(1000.0)

    def test_high_rpm(self):
        """Very high RPM does not cause overflow or errors."""
        vi, _ = _make_vesc()
        result = vi._rpm_to_ms(100_000.0)
        assert result > 0
        assert isinstance(result, float)

    def test_formula_matches_manual_calculation(self):
        """Cross-check with an independently calculated value."""
        # gear=8, circ=0.204, rpm=2400
        # wheel_rpm = 2400/8 = 300
        # velocity  = 300/60 * 0.204 = 5 * 0.204 = 1.02 m/s
        vi, _ = _make_vesc()
        assert vi._rpm_to_ms(2400.0) == pytest.approx(1.02, abs=1e-9)


# ─────────────────────────────────────────────────────────────────────────────
# set_throttle clamping tests
# ─────────────────────────────────────────────────────────────────────────────

class TestSetThrottle:

    def test_set_nominal_positive(self):
        """A value within range is stored unchanged."""
        vi, _ = _make_vesc()
        vi.set_throttle(0.5)
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(0.5)

    def test_set_nominal_negative(self):
        vi, _ = _make_vesc()
        vi.set_throttle(-0.5)
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(-0.5)

    def test_set_zero(self):
        vi, _ = _make_vesc()
        vi.set_throttle(0.0)
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(0.0)

    def test_set_exactly_plus_one(self):
        vi, _ = _make_vesc()
        vi.set_throttle(1.0)
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(1.0)

    def test_set_exactly_minus_one(self):
        vi, _ = _make_vesc()
        vi.set_throttle(-1.0)
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(-1.0)

    def test_clamp_above_max(self):
        """Values > +1.0 are clamped to +1.0."""
        vi, _ = _make_vesc()
        vi.set_throttle(2.0)
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(1.0)

    def test_clamp_well_above_max(self):
        vi, _ = _make_vesc()
        vi.set_throttle(999.0)
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(1.0)

    def test_clamp_below_min(self):
        """Values < -1.0 are clamped to -1.0."""
        vi, _ = _make_vesc()
        vi.set_throttle(-2.0)
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(-1.0)

    def test_clamp_well_below_min(self):
        vi, _ = _make_vesc()
        vi.set_throttle(-999.0)
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(-1.0)

    def test_throttle_starts_at_zero(self):
        """Default throttle on construction is 0.0."""
        vi, _ = _make_vesc()
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(0.0)

    def test_thread_safe_sequential_writes(self):
        """Multiple sequential writes converge to last written value."""
        vi, _ = _make_vesc()
        for val in [0.1, 0.5, -0.3, 0.9, -1.5, 0.2]:
            vi.set_throttle(val)
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(0.2)

    def test_concurrent_writes_do_not_corrupt(self):
        """Concurrent set_throttle calls from many threads do not corrupt state."""
        vi, _ = _make_vesc()
        errors = []

        def writer(val):
            try:
                vi.set_throttle(val)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=writer, args=(i * 0.01,))
                   for i in range(100)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert not errors
        with vi._throttle_lock:
            assert -1.0 <= vi._throttle <= 1.0


# ─────────────────────────────────────────────────────────────────────────────
# Instantiation / config tests
# ─────────────────────────────────────────────────────────────────────────────

class TestVESCInterfaceInit:

    def test_default_wheel_circumference(self):
        """Default wheel circumference from code is 0.204."""
        config = {"vesc": {}}    # empty → use defaults
        vi, _ = _make_vesc(config)
        assert vi._wheel_circ == pytest.approx(0.204)

    def test_default_gear_ratio(self):
        """Default gear ratio from code is 8.0."""
        config = {"vesc": {}}
        vi, _ = _make_vesc(config)
        assert vi._gear_ratio == pytest.approx(8.0)

    def test_custom_config_applied(self):
        config = {"vesc": {"wheel_circumference_m": 0.150, "gear_ratio": 5.0}}
        vi, _ = _make_vesc(config)
        assert vi._wheel_circ == pytest.approx(0.150)
        assert vi._gear_ratio == pytest.approx(5.0)

    def test_missing_vesc_key_uses_defaults(self):
        """Missing 'vesc' key in config does not crash."""
        vi, _ = _make_vesc({})
        assert vi._wheel_circ == pytest.approx(0.204)
        assert vi._gear_ratio == pytest.approx(8.0)

    def test_thread_is_daemon(self):
        vi, _ = _make_vesc()
        assert vi._thread.daemon is True

    def test_thread_not_started_on_init(self):
        """Thread should NOT be alive immediately after __init__."""
        vi, _ = _make_vesc()
        assert not vi._thread.is_alive()

    def test_port_default(self):
        """VESC port defaults to /dev/ttyACM1 when not set in config."""
        vi, _ = _make_vesc()
        assert vi._port == "/dev/ttyACM1"


# ─────────────────────────────────────────────────────────────────────────────
# Dummy mode (pyvesc unavailable)
# ─────────────────────────────────────────────────────────────────────────────

class TestVESCDummyMode:
    """vesc_interface uses a custom protocol — no pyvesc dependency."""

    def test_no_pyvesc_dependency(self):
        """vesc_interface must not import pyvesc (uses custom wire protocol)."""
        assert not hasattr(vesc_interface, "_PYVESC_AVAILABLE"), (
            "vesc_interface should not use pyvesc"
        )

    def test_set_throttle_still_works(self):
        """set_throttle() is always usable."""
        vi, _ = _make_vesc()
        vi.set_throttle(0.42)
        with vi._throttle_lock:
            assert vi._throttle == pytest.approx(0.42)
