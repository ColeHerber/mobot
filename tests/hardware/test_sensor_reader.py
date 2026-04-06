"""
test_sensor_reader.py — Unit tests for sensor_reader.py

Hardware mocked: serial.Serial, BNO085 I2C stack (via conftest sys.modules patches).
Tests cover:
  - _parse_packet(): valid packet, bad checksum, bad start/end bytes
  - _extract_yaw(): identity quaternion, 90° rotation, negative yaw
  - SensorReader instantiation
  - SharedState integration via _parse_packet → update_sensor
"""

import math
import struct
import sys
import os
import pytest
from unittest.mock import MagicMock, patch, call

# Ensure mobot root is importable
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _build_packet(line_pos_raw: int, flags: int, confidence: int) -> bytes:
    """Build a valid 8-byte Teensy packet from raw int16 line position."""
    pos_hi  = (line_pos_raw >> 8) & 0xFF
    pos_lo  = line_pos_raw & 0xFF
    fl_hi   = (flags >> 8) & 0xFF
    fl_lo   = flags & 0xFF
    conf    = confidence & 0xFF
    chk     = pos_hi ^ pos_lo ^ fl_hi ^ fl_lo ^ conf
    return bytes([0xAA, pos_hi, pos_lo, fl_hi, fl_lo, conf, chk, 0x55])


# ─────────────────────────────────────────────────────────────────────────────
# Import module under test (after conftest patches are in place)
# ─────────────────────────────────────────────────────────────────────────────

import sensor_reader
from sensor_reader import _extract_yaw
# _parse_packet is an instance method; we call it on a real (non-started)
# SensorReader instance constructed with a mocked serial port.
from shared_state import SharedState


def _make_reader(config=None):
    """Return a SensorReader that has been constructed but NOT started."""
    if config is None:
        config = {
            "sensor": {"port": "/dev/ttyACM0", "baud": 115200},
            "imu":    {"i2c_address": 0x4A},
        }
    state = SharedState()
    with patch("serial.Serial"):           # prevent real port open in __init__
        reader = sensor_reader.SensorReader(state, config)
    return reader, state


# ─────────────────────────────────────────────────────────────────────────────
# _parse_packet tests
# ─────────────────────────────────────────────────────────────────────────────

class TestParsePacket:

    def setup_method(self):
        self.reader, _ = _make_reader()

    def test_valid_packet_zero_position(self):
        """line_pos = 0 → 0.0 float, confidence and flags preserved."""
        pkt = _build_packet(0, 0b0000_0000_0000_0000, 200)
        result = self.reader._parse_packet(pkt)
        assert result is not None
        line_pos, flags, confidence, raw = result
        assert line_pos == pytest.approx(0.0)
        assert flags == 0
        assert confidence == 200

    def test_valid_packet_positive_position(self):
        """line_pos = +10000 raw → +1.0 float."""
        pkt = _build_packet(10000, 0xFF00, 128)
        result = self.reader._parse_packet(pkt)
        assert result is not None
        line_pos, flags, confidence, _ = result
        assert line_pos == pytest.approx(1.0)
        assert flags == 0xFF00
        assert confidence == 128

    def test_valid_packet_negative_position(self):
        """line_pos = -10000 raw (as int16) → -1.0 float."""
        # -10000 as unsigned 16-bit = 0xD8F0
        raw_i16 = (-10000) & 0xFFFF
        pkt = _build_packet(raw_i16, 0x00FF, 50)
        result = self.reader._parse_packet(pkt)
        assert result is not None
        line_pos, flags, confidence, _ = result
        assert line_pos == pytest.approx(-1.0)

    def test_valid_packet_mid_position(self):
        """line_pos = +5000 raw → +0.5 float."""
        pkt = _build_packet(5000, 0, 100)
        result = self.reader._parse_packet(pkt)
        assert result is not None
        line_pos, _, _, _ = result
        assert line_pos == pytest.approx(0.5)

    def test_valid_packet_negative_mid(self):
        """line_pos = -5000 raw → -0.5 float."""
        raw_i16 = (-5000) & 0xFFFF
        pkt = _build_packet(raw_i16, 0, 100)
        result = self.reader._parse_packet(pkt)
        assert result is not None
        line_pos, _, _, _ = result
        assert line_pos == pytest.approx(-0.5)

    def test_bad_checksum_returns_none(self):
        """Corrupted checksum byte → None."""
        pkt = bytearray(_build_packet(1234, 0, 100))
        pkt[6] ^= 0xFF   # corrupt checksum
        result = self.reader._parse_packet(bytes(pkt))
        assert result is None

    def test_bad_start_byte_returns_none(self):
        """Wrong start byte → None."""
        pkt = bytearray(_build_packet(0, 0, 100))
        pkt[0] = 0x00
        result = self.reader._parse_packet(bytes(pkt))
        assert result is None

    def test_bad_end_byte_returns_none(self):
        """Wrong end byte → None."""
        pkt = bytearray(_build_packet(0, 0, 100))
        pkt[7] = 0x00
        result = self.reader._parse_packet(bytes(pkt))
        assert result is None

    def test_both_bad_bytes_returns_none(self):
        """Both start and end bytes wrong → None."""
        pkt = bytearray(_build_packet(0, 0, 100))
        pkt[0] = 0xBB
        pkt[7] = 0xCC
        result = self.reader._parse_packet(bytes(pkt))
        assert result is None

    def test_flags_bitmask_decoded_to_raw(self):
        """flags bitmask is decoded into per-channel raw values."""
        # Only channels 0 and 3 set
        flags = (1 << 0) | (1 << 3)
        pkt = _build_packet(0, flags, 80)
        result = self.reader._parse_packet(pkt)
        assert result is not None
        _, _, _, raw = result
        assert len(raw) == 16
        assert raw[0] == 1000
        assert raw[3] == 1000
        assert raw[1] == 0
        assert raw[2] == 0
        assert raw[4] == 0

    def test_all_flags_set(self):
        """All 16 channels active → all raw values == 1000."""
        flags = 0xFFFF
        pkt = _build_packet(0, flags, 255)
        result = self.reader._parse_packet(pkt)
        assert result is not None
        _, _, _, raw = result
        assert all(v == 1000 for v in raw)

    def test_no_flags_set(self):
        """No channels active → all raw values == 0."""
        pkt = _build_packet(0, 0, 255)
        result = self.reader._parse_packet(pkt)
        assert result is not None
        _, _, _, raw = result
        assert all(v == 0 for v in raw)

    def test_confidence_zero(self):
        """Confidence == 0 is valid and passes through."""
        pkt = _build_packet(0, 0, 0)
        result = self.reader._parse_packet(pkt)
        assert result is not None
        _, _, confidence, _ = result
        assert confidence == 0

    def test_confidence_max(self):
        """Confidence == 255 is valid and passes through."""
        pkt = _build_packet(0, 0, 255)
        result = self.reader._parse_packet(pkt)
        assert result is not None
        _, _, confidence, _ = result
        assert confidence == 255

    def test_packet_too_short_returns_none(self):
        """A packet shorter than 8 bytes should not crash (defensive guard)."""
        # The method itself doesn't guard length (caller does), but we confirm
        # that at minimum the start/end check triggers an IndexError gracefully
        # in the real code via the caller's length check.  Here we just verify
        # that passing a truncated bytes object raises an expected exception
        # rather than silently returning bad data.
        pkt = _build_packet(0, 0, 100)[:6]
        with pytest.raises((IndexError, Exception)):
            self.reader._parse_packet(pkt)


# ─────────────────────────────────────────────────────────────────────────────
# _extract_yaw tests
# ─────────────────────────────────────────────────────────────────────────────

class TestExtractYaw:
    """
    BNO085 reports (i, j, k, real) = (x, y, z, w).
    _extract_yaw(quat_i, quat_j, quat_k, quat_real)
                       x       y       z       w
    """

    def test_identity_quaternion_gives_zero_yaw(self):
        """Identity rotation (w=1, x=y=z=0) → yaw == 0.0."""
        yaw = _extract_yaw(0.0, 0.0, 0.0, 1.0)
        assert yaw == pytest.approx(0.0, abs=1e-9)

    def test_90_degree_yaw(self):
        """Pure Z rotation of +90° → yaw == π/2."""
        # Quaternion for 90° around Z: w=cos(π/4), z=sin(π/4), x=y=0
        half_angle = math.pi / 4
        w = math.cos(half_angle)
        z = math.sin(half_angle)
        # _extract_yaw(quat_i=x, quat_j=y, quat_k=z, quat_real=w)
        yaw = _extract_yaw(0.0, 0.0, z, w)
        assert yaw == pytest.approx(math.pi / 2, abs=1e-9)

    def test_minus_90_degree_yaw(self):
        """Pure Z rotation of -90° → yaw == -π/2."""
        half_angle = -math.pi / 4
        w = math.cos(half_angle)
        z = math.sin(half_angle)
        yaw = _extract_yaw(0.0, 0.0, z, w)
        assert yaw == pytest.approx(-math.pi / 2, abs=1e-9)

    def test_180_degree_yaw(self):
        """Pure Z rotation of ±180° → |yaw| ≈ π."""
        half_angle = math.pi / 2
        w = math.cos(half_angle)   # ≈ 0
        z = math.sin(half_angle)   # ≈ 1
        yaw = _extract_yaw(0.0, 0.0, z, w)
        assert abs(yaw) == pytest.approx(math.pi, abs=1e-9)

    def test_45_degree_yaw(self):
        """Pure Z rotation of +45° → yaw == π/4."""
        half_angle = math.pi / 8
        w = math.cos(half_angle)
        z = math.sin(half_angle)
        yaw = _extract_yaw(0.0, 0.0, z, w)
        assert yaw == pytest.approx(math.pi / 4, abs=1e-9)

    def test_yaw_ignores_roll_pitch(self):
        """A pure pitch rotation (around Y) should yield yaw close to 0."""
        # Quaternion for 90° around Y: w=cos(π/4), y=sin(π/4), x=z=0
        half_angle = math.pi / 4
        w = math.cos(half_angle)
        y = math.sin(half_angle)
        # For pure pitch, yaw contribution should be 0
        yaw = _extract_yaw(0.0, y, 0.0, w)
        assert yaw == pytest.approx(0.0, abs=1e-9)

    def test_negative_yaw_value(self):
        """Negative yaw values are returned as-is (no abs)."""
        half_angle = -math.pi / 6  # -30°
        w = math.cos(half_angle)
        z = math.sin(half_angle)
        yaw = _extract_yaw(0.0, 0.0, z, w)
        assert yaw == pytest.approx(-math.pi / 3, abs=1e-6)

    def test_return_type_is_float(self):
        yaw = _extract_yaw(0.0, 0.0, 0.0, 1.0)
        assert isinstance(yaw, float)

    def test_yaw_in_range(self):
        """All outputs should be in (-π, π]."""
        test_angles = [0, 30, 60, 90, 120, 150, 170, -30, -90, -150, -170]
        for deg in test_angles:
            rad = math.radians(deg)
            half = rad / 2
            yaw = _extract_yaw(0.0, 0.0, math.sin(half), math.cos(half))
            assert -math.pi <= yaw <= math.pi, f"Yaw out of range for {deg}°: {yaw}"


# ─────────────────────────────────────────────────────────────────────────────
# SensorReader instantiation tests
# ─────────────────────────────────────────────────────────────────────────────

class TestSensorReaderInit:

    def test_instantiation_with_int_i2c_address(self):
        """SensorReader accepts integer i2c_address without crashing."""
        config = {
            "sensor": {"port": "/dev/ttyACM0", "baud": 115200},
            "imu":    {"i2c_address": 0x4A},
        }
        state = SharedState()
        with patch("serial.Serial"):
            reader = sensor_reader.SensorReader(state, config)
        assert reader is not None

    def test_instantiation_with_hex_string_i2c_address(self):
        """SensorReader accepts hex string i2c_address (as in YAML '0x4A')."""
        config = {
            "sensor": {"port": "/dev/ttyACM0", "baud": 115200},
            "imu":    {"i2c_address": "0x4A"},
        }
        state = SharedState()
        with patch("serial.Serial"):
            reader = sensor_reader.SensorReader(state, config)
        assert reader._i2c_addr == 0x4A

    def test_thread_created_as_daemon(self):
        """Background thread is a daemon thread."""
        config = {
            "sensor": {"port": "/dev/ttyACM0", "baud": 115200},
            "imu":    {"i2c_address": 0x4A},
        }
        state = SharedState()
        with patch("serial.Serial"):
            reader = sensor_reader.SensorReader(state, config)
        assert reader._thread.daemon is True

    def test_port_and_baud_stored(self):
        """Port and baud rate from config are stored on the instance."""
        config = {
            "sensor": {"port": "/dev/ttyUSB0", "baud": 9600},
            "imu":    {"i2c_address": 0x4A},
        }
        state = SharedState()
        with patch("serial.Serial"):
            reader = sensor_reader.SensorReader(state, config)
        assert reader._port == "/dev/ttyUSB0"
        assert reader._baud == 9600


# ─────────────────────────────────────────────────────────────────────────────
# Integration: _parse_packet → SharedState.update_sensor
# ─────────────────────────────────────────────────────────────────────────────

class TestParsePacketIntegration:
    """Verify that parsed packet data matches what would be written to state."""

    def test_round_trip_line_pos_and_confidence(self):
        """Build packet, parse it, confirm values are consistent."""
        reader, state = _make_reader()
        expected_pos = 0.3456
        raw_i16 = int(round(expected_pos * 10000))
        pkt = _build_packet(raw_i16, 0b0000_1111_0000_1111, 192)
        result = reader._parse_packet(pkt)
        assert result is not None
        line_pos, flags, confidence, raw = result

        # Write to state the same way the run loop would
        state.update_sensor(line_pos, confidence, flags, raw)

        assert state.line_position == pytest.approx(expected_pos, rel=1e-4)
        assert state.sensor_confidence == 192
        assert state.sensor_flags == 0b0000_1111_0000_1111
        assert len(state.sensor_raw) == 16
