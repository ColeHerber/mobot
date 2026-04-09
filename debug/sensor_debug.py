#!/usr/bin/env python3
"""Sensor debug tool — stream-print Teensy line-sensor output.

Usage:
    python3 debug/sensor_debug.py [--port /dev/ttyACM0] [--baud 115200]

Switches Teensy into calibration mode (raw CSV) so per-channel ADC values
are visible. Position and confidence are computed locally from the raw values
using the same algorithm as the firmware.

Press Ctrl+C to exit (restores run mode on the Teensy).
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

CHANNELS = 16
ADC_MAX  = 4095


def normalize(raw: int, mn: int, mx: int) -> int:
    if mx <= mn:
        return 0
    if raw <= mn:
        return 0
    if raw >= mx:
        return 1000
    return int((raw - mn) * 1000 / (mx - mn))


def compute(raw: list[int], cal_min: list[int], cal_max: list[int]):
    """Compute line_pos and confidence from raw ADC values."""
    norm = [normalize(raw[i], cal_min[i], cal_max[i]) for i in range(CHANNELS)]
    total = sum(norm)
    flags = sum(1 << i for i in range(CHANNELS) if norm[i] > 500)

    if total < 100:
        line_pos = 0.0
    else:
        weighted = sum(norm[i] * ((i - 7) * 1000 + 500) for i in range(CHANNELS))
        centroid = weighted / total          # -7500 to +7500
        line_pos = max(-1.0, min(1.0, centroid / 7500.0))

    confidence = min(255, int(total * 255 / 16000))
    return norm, line_pos, confidence, flags


def bar(norm: list[int]) -> str:
    """Compact intensity bar using block characters."""
    blocks = " ▁▂▃▄▅▆▇█"
    return "".join(blocks[min(8, v * 9 // 1001)] for v in norm)


def main():
    parser = argparse.ArgumentParser(description="Teensy sensor stream debug")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud} baud — Ctrl+C to exit\n")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.write(b'C')   # calibration mode → raw CSV

    # Auto-calibration: track min/max seen so far
    cal_min = [ADC_MAX] * CHANNELS
    cal_max = [0]       * CHANNELS

    packets_ok  = 0
    packets_bad = 0
    last_print  = time.monotonic()

    CHAN_HDR = "Chan: " + " ".join(f"{i:4d}" for i in range(CHANNELS))

    print("Auto-calibrating from live data (min/max updates as sensors are exposed).")
    print("Move the robot over the line and bare ground to build calibration.\n")
    print(CHAN_HDR)

    try:
        while True:
            try:
                line = ser.readline().decode("ascii", errors="replace").strip()
            except serial.SerialException as e:
                print(f"\nRead error: {e}")
                time.sleep(0.2)
                continue

            if not line:
                continue
            if line.startswith("CAL"):
                print(f"\n[info] {line}")
                continue

            parts = line.split(",")
            if len(parts) != CHANNELS:
                packets_bad += 1
                continue

            try:
                raw = [int(p) for p in parts]
            except ValueError:
                packets_bad += 1
                continue

            packets_ok += 1

            # Update auto-cal
            for i in range(CHANNELS):
                if raw[i] < cal_min[i]:
                    cal_min[i] = raw[i]
                if raw[i] > cal_max[i]:
                    cal_max[i] = raw[i]

            now = time.monotonic()
            if now - last_print >= 0.05:   # ~20 Hz display
                last_print = now
                norm, line_pos, conf, flags = compute(raw, cal_min, cal_max)

                raw_row  = " ".join(f"{v:4d}" for v in raw)
                norm_row = " ".join(f"{v:4d}" for v in norm)
                intensity = bar(norm)

                sys.stdout.write(
                    f"\r{CHAN_HDR}\n"
                    f"\rRaw:  {raw_row}\n"
                    f"\rNorm: {norm_row}\n"
                    f"\r[{intensity}]\n"
                    f"\rPos: {line_pos:+.4f}  Conf: {conf:3d}  "
                    f"ok:{packets_ok} bad:{packets_bad}   "
                    f"\033[4A"   # move cursor up 4 lines for next refresh
                )
                sys.stdout.flush()

    except KeyboardInterrupt:
        print(f"\n\n\n\n\nDone. {packets_ok} good packets, {packets_bad} bad.")
    finally:
        ser.write(b'R')  # restore run mode
        ser.close()


if __name__ == "__main__":
    main()
