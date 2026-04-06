#!/usr/bin/env python3
"""Sensor debug tool — stream-print Teensy line-sensor output.

Usage:
    python3 sensor_debug.py [--port /dev/ttyACM0] [--baud 115200]

Output (refreshed each packet):
    Chan: 00  01  02  03  04  05  06  07  08  09  10  11  12  13  14  15
    Norm:  0   0  45 234 890 999 456  12   0   0   0   0   0   0   0   0
    Pos: -0.234  Conf: 187  Flags: 0x00FC

Protocol (matches Teensy firmware expected format):
    Each line from Teensy:
        SENSOR <ch0> <ch1> ... <ch15> <line_pos_x1000> <confidence> <flags_hex>\n

    e.g.:  SENSOR 0 0 45 234 890 999 456 12 0 0 0 0 0 0 0 0 -234 187 00FC

    If the firmware sends a different format, adjust _parse_line() below.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip3 install pyserial")
    sys.exit(1)

CHANNELS = 16
HEADER   = "Chan: " + "  ".join(f"{i:02d}" for i in range(CHANNELS))


def _parse_line(line: str):
    """Parse a SENSOR packet. Returns (raw, line_pos, confidence, flags) or None."""
    line = line.strip()
    if not line.startswith("SENSOR"):
        return None
    parts = line.split()
    # Expected: SENSOR ch0..ch15 line_pos_int conf flags_hex
    if len(parts) != 1 + CHANNELS + 3:
        return None
    try:
        raw        = [int(x) for x in parts[1:17]]
        line_pos   = int(parts[17]) / 1000.0
        confidence = int(parts[18])
        flags      = int(parts[19], 16)
        return raw, line_pos, confidence, flags
    except (ValueError, IndexError):
        return None


def _bar(raw: list[int]) -> str:
    """Format 16 channel values as a space-aligned row."""
    return "Norm: " + "".join(f"{v:4d}" for v in raw)


def main():
    parser = argparse.ArgumentParser(description="Teensy sensor stream debug")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud} baud — press Ctrl+C to exit")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    print(HEADER)

    try:
        while True:
            try:
                raw_bytes = ser.readline()
            except serial.SerialException as e:
                print(f"Read error: {e}")
                time.sleep(0.5)
                continue

            try:
                line = raw_bytes.decode("ascii", errors="replace")
            except Exception:
                continue

            parsed = _parse_line(line)
            if parsed is None:
                # Pass through any non-sensor lines (debug prints, etc.)
                line_stripped = line.strip()
                if line_stripped:
                    print(f"[raw] {line_stripped}")
                continue

            raw, line_pos, conf, flags = parsed

            # Overwrite previous 3 lines with \r for compact display
            print(f"\r{HEADER}", end="")
            print(f"\n\r{_bar(raw)}", end="")
            print(f"\n\rPos: {line_pos:+.3f}  Conf: {conf:3d}  "
                  f"Flags: 0x{flags:04X}", end="", flush=True)
            # Move cursor back up 2 lines for next overwrite
            print("\033[2A", end="", flush=True)

    except KeyboardInterrupt:
        print("\n\nExiting.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
