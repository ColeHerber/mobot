"""
test_line_sensing.py — Unit tests for the Teensy weighted-centroid algorithm.

The Teensy computes a weighted centroid of sensor readings to produce
line_pos in [-1.0, +1.0].  The SensorReader re-derives line_pos from the
packed int16 in the serial packet (already computed on Teensy).

This test suite:
1. Verifies the packet encoding/decoding round-trip preserves line_pos fidelity
2. Simulates the centroid algorithm in Python to verify logical correctness
3. Tests SharedState update via sensor data paths
4. Tests confidence == 0 edge case

The centroid formula (as the Teensy implements it):
    weighted_sum = sum(position[i] * sensor_value[i])
    total        = sum(sensor_value[i])
    centroid     = weighted_sum / total  (range depends on position weights)
    line_pos     = normalized to [-1.0, +1.0]

We use 16 sensors evenly spaced from -1.0 to +1.0.
"""

import sys
import os
import struct
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from shared_state import SharedState


# ─────────────────────────────────────────────────────────────────────────────
# Python implementation of the Teensy weighted centroid
# ─────────────────────────────────────────────────────────────────────────────

NUM_SENSORS = 16

# Sensor positions mapped linearly from -1.0 (leftmost) to +1.0 (rightmost)
SENSOR_POSITIONS = [
    -1.0 + (2.0 / (NUM_SENSORS - 1)) * i
    for i in range(NUM_SENSORS)
]


def compute_centroid(sensor_values: list) -> tuple:
    """
    Simulate the Teensy weighted centroid calculation.

    Args:
        sensor_values: list of NUM_SENSORS floats [0..1000]

    Returns:
        (line_pos, confidence) where:
          line_pos   = weighted centroid in [-1.0, +1.0], or 0.0 if total == 0
          confidence = int(total / NUM_SENSORS / 1000 * 255), 0 if all zero
    """
    total = sum(sensor_values)
    if total == 0:
        return 0.0, 0

    weighted_sum = sum(SENSOR_POSITIONS[i] * sensor_values[i]
                       for i in range(NUM_SENSORS))
    line_pos = weighted_sum / total

    # Confidence: average activation scaled to 0-255
    confidence = int((total / NUM_SENSORS / 1000.0) * 255)
    confidence = max(0, min(255, confidence))

    return line_pos, confidence


def _build_packet(line_pos_float: float, flags: int, confidence: int) -> bytes:
    """Encode line_pos to int16 and build valid 8-byte Teensy packet."""
    raw_i16 = int(round(line_pos_float * 10000))
    raw_i16 = max(-32768, min(32767, raw_i16))
    pos_hi  = (raw_i16 >> 8) & 0xFF
    pos_lo  = raw_i16 & 0xFF
    fl_hi   = (flags >> 8) & 0xFF
    fl_lo   = flags & 0xFF
    conf    = confidence & 0xFF
    chk     = pos_hi ^ pos_lo ^ fl_hi ^ fl_lo ^ conf
    return bytes([0xAA, pos_hi, pos_lo, fl_hi, fl_lo, conf, chk, 0x55])


def _decode_packet_line_pos(pkt: bytes) -> float:
    """Decode line_pos from 8-byte packet (mirrors SensorReader._parse_packet)."""
    assert pkt[0] == 0xAA and pkt[7] == 0x55
    pos_hi, pos_lo = pkt[1], pkt[2]
    raw = struct.unpack('>h', bytes([pos_hi, pos_lo]))[0]
    return raw / 10000.0


# ─────────────────────────────────────────────────────────────────────────────
# Centroid algorithm unit tests
# ─────────────────────────────────────────────────────────────────────────────

class TestCentroidAlgorithm:

    def test_all_left_sensors_active(self):
        """Only leftmost sensors active → line_pos ≈ -1.0."""
        values = [1000] * 3 + [0] * 13   # first 3 sensors on
        line_pos, confidence = compute_centroid(values)
        assert line_pos < -0.7, f"Expected strongly negative, got {line_pos}"
        assert confidence > 0

    def test_all_right_sensors_active(self):
        """Only rightmost sensors active → line_pos ≈ +1.0."""
        values = [0] * 13 + [1000] * 3   # last 3 sensors on
        line_pos, confidence = compute_centroid(values)
        assert line_pos > 0.7, f"Expected strongly positive, got {line_pos}"
        assert confidence > 0

    def test_center_sensors_active(self):
        """Middle 2 sensors active → line_pos ≈ 0.0."""
        values = [0] * 7 + [1000, 1000] + [0] * 7
        line_pos, confidence = compute_centroid(values)
        assert abs(line_pos) < 0.15, f"Expected near zero, got {line_pos}"
        assert confidence > 0

    def test_single_leftmost_sensor(self):
        """Only sensor 0 active → line_pos == SENSOR_POSITIONS[0] == -1.0."""
        values = [1000] + [0] * 15
        line_pos, confidence = compute_centroid(values)
        assert line_pos == pytest.approx(-1.0)

    def test_single_rightmost_sensor(self):
        """Only sensor 15 active → line_pos == SENSOR_POSITIONS[15] == +1.0."""
        values = [0] * 15 + [1000]
        line_pos, confidence = compute_centroid(values)
        assert line_pos == pytest.approx(1.0)

    def test_single_center_sensor_left(self):
        """Sensor 7 (just left of center) → small negative position."""
        values = [0] * 7 + [1000] + [0] * 8
        line_pos, _ = compute_centroid(values)
        assert line_pos < 0

    def test_single_center_sensor_right(self):
        """Sensor 8 (just right of center) → small positive position."""
        values = [0] * 8 + [1000] + [0] * 7
        line_pos, _ = compute_centroid(values)
        assert line_pos > 0

    def test_all_sensors_zero_confidence_zero(self):
        """All sensors zero → confidence == 0."""
        values = [0] * 16
        _, confidence = compute_centroid(values)
        assert confidence == 0

    def test_all_sensors_zero_line_pos_zero(self):
        """All sensors zero → line_pos defaults to 0.0 (no divide-by-zero)."""
        values = [0] * 16
        line_pos, _ = compute_centroid(values)
        assert line_pos == 0.0

    def test_all_sensors_active_symmetric(self):
        """All sensors at equal value → centroid near 0 (symmetric sensor array)."""
        values = [500] * 16
        line_pos, confidence = compute_centroid(values)
        assert abs(line_pos) < 1e-9, f"Expected 0, got {line_pos}"
        assert confidence > 0

    def test_confidence_proportional_to_activation(self):
        """Higher average sensor activation → higher confidence."""
        values_low  = [100] * 16
        values_high = [800] * 16
        _, conf_low  = compute_centroid(values_low)
        _, conf_high = compute_centroid(values_high)
        assert conf_high > conf_low

    def test_single_sensor_max_confidence_less_than_full(self):
        """Only 1 sensor active (1/16 coverage) → confidence < full-array confidence."""
        values_one = [1000] + [0] * 15
        values_all = [1000] * 16
        _, conf_one = compute_centroid(values_one)
        _, conf_all = compute_centroid(values_all)
        assert conf_one < conf_all

    def test_asymmetric_activation_shifts_toward_heavier_side(self):
        """More sensors on right side → line_pos > 0."""
        # 3 left sensors at 500, 8 right sensors at 500
        values = [500] * 3 + [0] * 5 + [500] * 8
        line_pos, _ = compute_centroid(values)
        assert line_pos > 0


# ─────────────────────────────────────────────────────────────────────────────
# Packet round-trip fidelity tests
# ─────────────────────────────────────────────────────────────────────────────

class TestPacketRoundTrip:
    """
    The Teensy sends the centroid result as a packed int16.
    These tests verify the encode→decode round-trip preserves line_pos
    to within the 1/10000 resolution of the int16 encoding.
    """

    @pytest.mark.parametrize("line_pos", [
        0.0, 0.5, -0.5, 1.0, -1.0, 0.1234, -0.9999, 0.3456, -0.7777,
    ])
    def test_round_trip_precision(self, line_pos):
        """Encode line_pos to int16, decode it back; error < 1/10000."""
        pkt = _build_packet(line_pos, 0, 100)
        decoded = _decode_packet_line_pos(pkt)
        assert decoded == pytest.approx(line_pos, abs=1e-4)

    def test_zero_encodes_and_decodes_exactly(self):
        pkt = _build_packet(0.0, 0, 100)
        assert _decode_packet_line_pos(pkt) == 0.0

    def test_plus_one_encodes_to_10000(self):
        """line_pos=+1.0 → int16 raw = 10000."""
        pkt = _build_packet(1.0, 0, 100)
        raw = struct.unpack('>h', bytes([pkt[1], pkt[2]]))[0]
        assert raw == 10000

    def test_minus_one_encodes_to_minus_10000(self):
        """line_pos=-1.0 → int16 raw = -10000."""
        pkt = _build_packet(-1.0, 0, 100)
        raw = struct.unpack('>h', bytes([pkt[1], pkt[2]]))[0]
        assert raw == -10000

    def test_positive_sub_resolution_rounds(self):
        """Values finer than 1/10000 are rounded on encode."""
        # 0.00001 rounds to 0 at int16 resolution
        pkt = _build_packet(0.00001, 0, 100)
        decoded = _decode_packet_line_pos(pkt)
        assert abs(decoded) <= 1e-4


# ─────────────────────────────────────────────────────────────────────────────
# SharedState integration
# ─────────────────────────────────────────────────────────────────────────────

class TestSharedStateLineSensing:
    """Verify SharedState correctly stores sensor data updates."""

    def test_update_sensor_stores_line_position(self):
        state = SharedState()
        state.update_sensor(0.42, 180, 0b1111, [0]*16)
        assert state.line_position == pytest.approx(0.42)

    def test_update_sensor_stores_confidence(self):
        state = SharedState()
        state.update_sensor(0.0, 200, 0, [0]*16)
        assert state.sensor_confidence == 200

    def test_update_sensor_stores_flags(self):
        state = SharedState()
        state.update_sensor(0.0, 100, 0b1010101010101010, [0]*16)
        assert state.sensor_flags == 0b1010101010101010

    def test_update_sensor_stores_raw_values(self):
        state = SharedState()
        raw = list(range(16))
        state.update_sensor(0.0, 100, 0, raw)
        assert state.sensor_raw == raw

    def test_zero_confidence_stored_correctly(self):
        """confidence=0 (line lost) is stored faithfully."""
        state = SharedState()
        state.update_sensor(0.0, 0, 0, [0]*16)
        assert state.sensor_confidence == 0

    def test_default_line_position_is_zero(self):
        state = SharedState()
        assert state.line_position == 0.0

    def test_default_confidence_is_zero(self):
        state = SharedState()
        assert state.sensor_confidence == 0

    def test_successive_updates_overwrite(self):
        """Multiple updates: last one wins."""
        state = SharedState()
        state.update_sensor(0.1, 50,  0, [0]*16)
        state.update_sensor(0.9, 200, 0, [0]*16)
        assert state.line_position  == pytest.approx(0.9)
        assert state.sensor_confidence == 200

    def test_flags_bitmask_left_sensors(self):
        """Flags bitmask for left sensors (lower bits set)."""
        state = SharedState()
        state.update_sensor(-0.8, 150, 0x000F, [0]*16)
        assert state.sensor_flags & 0x000F == 0x000F

    def test_raw_length_always_16(self):
        state = SharedState()
        state.update_sensor(0.0, 100, 0xFF, [1000]*16)
        assert len(state.sensor_raw) == 16
