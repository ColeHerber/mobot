#!/usr/bin/env python3
"""VESC telemetry debug tool — print live motor data over UART.

Usage:
    python3 vesc_debug.py [--port /dev/serial0] [--baud 115200]

Output (refreshed ~5 Hz):
    RPM: 1234.5  Vel: 0.45 m/s  Volt: 11.4V  Temp: 42.1°C  Duty: 0.350

Wheel velocity is computed from RPM using the same wheel params as the robot:
    wheel_circumference_m = 0.204
    gear_ratio            = 8.0
These can be overridden via --circumference and --gear-ratio flags.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip3 install pyserial")
    sys.exit(1)

try:
    import pyvesc
    from pyvesc.VESC.messages import GetValues
    _PYVESC_OK = True
except ImportError:
    print("ERROR: pyvesc not installed. Run: pip3 install pyvesc")
    sys.exit(1)

POLL_HZ = 5
POLL_DT = 1.0 / POLL_HZ


def rpm_to_ms(rpm: float, circumference: float, gear_ratio: float) -> float:
    """Convert motor RPM to wheel surface speed in m/s."""
    wheel_rps = rpm / gear_ratio / 60.0
    return wheel_rps * circumference


def poll_once(ser, circumference: float, gear_ratio: float) -> str | None:
    """Request GetValues, return formatted string or None on error."""
    try:
        ser.write(pyvesc.encode_request(GetValues))
        time.sleep(0.05)  # give VESC time to respond
        raw = ser.read(ser.in_waiting or 128)
        if not raw:
            return None
        msg, _ = pyvesc.decode(raw)
        if msg is None:
            return None
        rpm   = float(getattr(msg, "rpm", 0))
        volt  = float(getattr(msg, "input_voltage", 0))
        temp  = float(getattr(msg, "temp_fet_filtered", 0))
        duty  = float(getattr(msg, "duty_cycle_now", 0))
        vel   = rpm_to_ms(rpm, circumference, gear_ratio)
        return (f"RPM: {rpm:8.1f}  Vel: {vel:+.3f} m/s  "
                f"Volt: {volt:.1f}V  Temp: {temp:.1f}°C  Duty: {duty:.3f}")
    except Exception as e:
        return f"[error: {e}]"


def main():
    parser = argparse.ArgumentParser(description="VESC UART telemetry debug")
    parser.add_argument("--port",         default="/dev/serial0")
    parser.add_argument("--baud",         type=int,   default=115200)
    parser.add_argument("--circumference",type=float, default=0.204,
                        help="Wheel circumference in metres (default: 0.204)")
    parser.add_argument("--gear-ratio",   type=float, default=8.0,
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
