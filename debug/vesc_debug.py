#!/usr/bin/env python3
"""VESC telemetry debug tool — print live motor data over USB serial.

No pyvesc dependency; implements the VESC wire protocol directly.

Usage:
    python3 vesc_debug.py [--port /dev/ttyACM1] [--baud 115200]

Output (refreshed ~5 Hz):
    RPM: 1234.5  Vel: 0.45 m/s  Volt: 11.4V  Temp: 42.1°C  Duty: 0.350

Wheel velocity is computed from RPM using the same wheel params as the robot:
    wheel_circumference_m = 0.204
    gear_ratio            = 8.0
These can be overridden via --circumference and --gear-ratio flags.
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip3 install pyserial")
    sys.exit(1)

# ── VESC wire protocol ────────────────────────────────────────────────────────

COMM_GET_VALUES = 4


def _crc_ccitt(data: bytes) -> int:
    """CRC-CCITT XModem: poly=0x1021, init=0x0000."""
    crc = 0
    for b in data:
        x = (crc >> 8) ^ b
        x ^= x >> 4
        crc = ((crc << 8) ^ (x << 12) ^ (x << 5) ^ x) & 0xFFFF
    return crc


def _make_packet(payload: bytes) -> bytes:
    """Wrap payload in VESC short/long framing with CRC."""
    n = len(payload)
    if n <= 255:
        header = bytes([0x02, n])
    else:
        header = bytes([0x03, (n >> 8) & 0xFF, n & 0xFF])
    crc = _crc_ccitt(payload)
    return header + payload + bytes([(crc >> 8) & 0xFF, crc & 0xFF, 0x03])


def _find_packet(buf: bytearray) -> tuple[bytes | None, int]:
    """Scan buf for the first valid framed packet.

    Returns (payload, bytes_consumed) or (None, 0) if no complete packet yet.
    """
    i = 0
    while i < len(buf):
        start = buf[i]
        if start == 0x02:
            # Short packet: [0x02, len, ...payload..., crc_hi, crc_lo, 0x03]
            if i + 4 > len(buf):
                break
            length = buf[i + 1]
            end = i + 2 + length + 3  # header(2) + payload + crc(2) + stop(1)
            if end > len(buf):
                break
            payload = bytes(buf[i + 2: i + 2 + length])
            crc_recv = (buf[i + 2 + length] << 8) | buf[i + 2 + length + 1]
            stop = buf[end - 1]
        elif start == 0x03:
            # Long packet: [0x03, len_hi, len_lo, ...payload..., crc_hi, crc_lo, 0x03]
            if i + 5 > len(buf):
                break
            length = (buf[i + 1] << 8) | buf[i + 2]
            end = i + 3 + length + 3  # header(3) + payload + crc(2) + stop(1)
            if end > len(buf):
                break
            payload = bytes(buf[i + 3: i + 3 + length])
            crc_recv = (buf[i + 3 + length] << 8) | buf[i + 3 + length + 1]
            stop = buf[end - 1]
        else:
            i += 1
            continue

        if stop != 0x03 or _crc_ccitt(payload) != crc_recv:
            i += 1
            continue

        return payload, end

    return None, 0


# GetValues response: command byte + big-endian struct
# Fields match pyvesc 1.0.5 / VESC firmware 3.x–5.x.
# Firmware 6.x may append extra bytes — they are safely ignored.
_GV_FMT = ">hh iiii h i h iiii ii b"
_GV_SCALE = (10, 10, 100, 100, 100, 100, 1000, 1, 10,
             10000, 10000, 10000, 10000, 1, 1, 1)
_GV_NAMES = (
    "temp_fet_filtered", "temp_motor_filtered",
    "avg_motor_current", "avg_input_current", "avg_id", "avg_iq",
    "duty_cycle_now", "rpm", "input_voltage",
    "amp_hours", "amp_hours_charged", "watt_hours", "watt_hours_charged",
    "tachometer", "tachometer_abs", "mc_fault_code",
)
_GV_SIZE = struct.calcsize(_GV_FMT)  # 55 bytes

# Pre-built request (never changes)
GET_VALUES_REQUEST = _make_packet(bytes([COMM_GET_VALUES]))


def _parse_get_values(payload: bytes) -> dict | None:
    """Decode a GetValues response payload; return field dict or None."""
    if not payload or payload[0] != COMM_GET_VALUES:
        return None
    body = payload[1:]
    if len(body) < _GV_SIZE:
        return None
    raw = struct.unpack(_GV_FMT, body[:_GV_SIZE])
    return {name: val / scale
            for name, scale, val in zip(_GV_NAMES, _GV_SCALE, raw)}


# ── Debug loop ────────────────────────────────────────────────────────────────

POLL_HZ = 5
POLL_DT = 1.0 / POLL_HZ


def _rpm_to_ms(rpm: float, circumference: float, gear_ratio: float) -> float:
    return (rpm / gear_ratio / 60.0) * circumference


def poll_once(ser: serial.Serial, circumference: float, gear_ratio: float) -> str | None:
    """Send a GetValues request, read response, return formatted string or None."""
    try:
        ser.write(GET_VALUES_REQUEST)
        time.sleep(0.05)
        raw = ser.read(ser.in_waiting or 128)
    except Exception as e:
        return f"[serial error: {e}]"

    if not raw:
        return None

    buf = bytearray(raw)
    while buf:
        payload, consumed = _find_packet(buf)
        if payload is None:
            break
        del buf[:consumed]
        fields = _parse_get_values(payload)
        if fields is None:
            continue
        rpm  = fields["rpm"]
        volt = fields["input_voltage"]
        temp = fields["temp_fet_filtered"]
        duty = fields["duty_cycle_now"]
        vel  = _rpm_to_ms(rpm, circumference, gear_ratio)
        return (f"RPM: {rpm:8.1f}  Vel: {vel:+.3f} m/s  "
                f"Volt: {volt:.1f}V  Temp: {temp:.1f}°C  Duty: {duty:.3f}")

    return None


def main():
    parser = argparse.ArgumentParser(description="VESC USB telemetry debug")
    parser.add_argument("--port",          default="/dev/ttyACM1")
    parser.add_argument("--baud",          type=int,   default=115200)
    parser.add_argument("--circumference", type=float, default=0.204,
                        help="Wheel circumference in metres (default: 0.204)")
    parser.add_argument("--gear-ratio",    type=float, default=8.0,
                        help="Motor-to-wheel gear ratio (default: 8.0)")
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud} baud — press Ctrl+C to exit")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    print("Polling VESC…")
    try:
        while True:
            t0  = time.monotonic()
            msg = poll_once(ser, args.circumference, args.gear_ratio)
            if msg:
                print(f"\r{msg}          ", end="", flush=True)
            elapsed  = time.monotonic() - t0
            sleep_dt = POLL_DT - elapsed
            if sleep_dt > 0:
                time.sleep(sleep_dt)
    except KeyboardInterrupt:
        print("\n\nExiting.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
