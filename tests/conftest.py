"""
conftest.py — Shared fixtures and sys.modules patching for hardware-free testing.

All hardware libraries are mocked HERE, before any source module is imported,
so that imports of sensor_reader, vesc_interface, and servo_control succeed on
a Mac/Linux dev machine with no Pi hardware attached.
"""

import sys
import types
from unittest.mock import MagicMock, patch
import pytest

# ─────────────────────────────────────────────────────────────────────────────
# Patch hardware-only modules into sys.modules before anything imports them.
# This must happen at collection time, not inside a fixture, so that the
# module-level `import lgpio` / `import pyvesc` etc. resolve cleanly.
# ─────────────────────────────────────────────────────────────────────────────

# ── lgpio ─────────────────────────────────────────────────────────────────────
_lgpio_mock = MagicMock()
_lgpio_mock.gpiochip_open.return_value = 1          # fake chip handle
_lgpio_mock.gpio_claim_output.return_value = 0
_lgpio_mock.tx_servo.return_value = 0
_lgpio_mock.gpiochip_close.return_value = 0
# lgpio raises lgpio.error on real errors; give the mock an exception class
_lgpio_mock.error = type("lgpio_error", (Exception,), {})
sys.modules["lgpio"] = _lgpio_mock

# ── adafruit / board / busio ──────────────────────────────────────────────────
_board_mock  = MagicMock()
_busio_mock  = MagicMock()
_bno08x_mock = MagicMock()
_bno08x_i2c_mock = MagicMock()

# adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR is just an integer constant
_bno08x_mock.BNO_REPORT_ROTATION_VECTOR = 5

sys.modules["board"]              = _board_mock
sys.modules["busio"]              = _busio_mock
sys.modules["adafruit_bno08x"]   = _bno08x_mock
sys.modules["adafruit_bno08x.i2c"] = _bno08x_i2c_mock


# ─────────────────────────────────────────────────────────────────────────────
# Shared config fixture — mirrors params.yaml in dict form
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def mock_config():
    return {
        "pid": {
            "kp": 1.2,
            "kd": 0.08,
            "speed_kp": 0.5,
        },
        "speed": {
            "base_ms": 0.6,
            "gate_ms": 0.3,
            "min_ms":  0.2,
        },
        "servo": {
            "gpio_pin":         18,
            "gpiochip":          4,
            "min_pw":         1000,
            "max_pw":         2000,
            "center_pw":      1500,
            "center_offset_us":  0,
        },
        "vesc": {
            "wheel_circumference_m": 0.204,
            "gear_ratio":            8.0,
        },
        "sensor": {
            "low_confidence_threshold": 30,
            "port": "/dev/ttyACM0",
            "baud": 115200,
        },
        "imu": {
            "i2c_address": 0x4A,
        },
    }


@pytest.fixture
def mock_route():
    """Minimal route with one left-turn intersection and one gate."""
    return {
        "intersections": [
            {
                "id": 1,
                "x": 4.5,
                "y": 0.0,
                "direction": "left",
                "radius_m": 0.3,
            }
        ],
        "gates": [
            {
                "id": 1,
                "x": 2.0,
                "y": 0.0,
                "radius_m": 0.5,
            }
        ],
    }


@pytest.fixture
def shared_state():
    """A real SharedState instance (no hardware needed)."""
    from shared_state import SharedState
    state = SharedState()
    yield state
    state.running = False
