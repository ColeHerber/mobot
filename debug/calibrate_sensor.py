#!/usr/bin/env python3
"""Teensy sensor calibration — saves min/max to EEPROM.

Steps:
  1. Hold robot over BARE GROUND for 3 seconds (captures min values)
  2. Slide robot slowly across the full LINE WIDTH (captures max values)
  3. Script saves to EEPROM and exits

Usage:
    python3 debug/calibrate_sensor.py [--port /dev/ttyACM0]
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

CHANNELS    = 16
_CAL_DIR    = os.path.join(os.path.dirname(os.path.dirname(__file__)), "calibrations")


def _default_port() -> str:
    try:
        import pathlib, re
        cfg = (pathlib.Path(__file__).parent.parent / "config" / "params.yaml").read_text()
        m = re.search(r"^\s*port:\s*(\S+)", cfg[cfg.index("sensor:"):], re.MULTILINE)
        if m:
            return m.group(1)
    except Exception:
        pass
    return "/dev/ttyACM0"
_INDEX_FILE = os.path.join(_CAL_DIR, "index.json")


def _autosave(cal_min: list, cal_max: list):
    os.makedirs(_CAL_DIR, exist_ok=True)
    ts    = datetime.now()
    label = ts.strftime("%Y-%m-%d_%H%M%S")
    span  = [cal_max[i] - cal_min[i] for i in range(CHANNELS)]
    low   = [i for i in range(CHANNELS) if span[i] < 200]
    data  = {
        "label":                 label,
        "timestamp":             ts.isoformat(),
        "notes":                 "calibrate_sensor.py",
        "channels":              CHANNELS,
        "cal_min":               cal_min,
        "cal_max":               cal_max,
        "span":                  span,
        "low_contrast_channels": low,
    }
    path = os.path.join(_CAL_DIR, f"{label}.json")
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    index = {}
    if os.path.exists(_INDEX_FILE):
        with open(_INDEX_FILE) as f:
            index = json.load(f)
    index[label] = {"timestamp": ts.isoformat(), "file": f"{label}.json",
                    "notes": "calibrate_sensor.py", "low_contrast_channels": low}
    with open(_INDEX_FILE, "w") as f:
        json.dump(index, f, indent=2)
    print(f"Snapshot saved: calibrations/{label}.json")


def read_raw(ser) -> list[int] | None:
    line = ser.readline().decode("ascii", errors="replace").strip()
    if not line or line.startswith("CAL"):
        return None
    parts = line.split(",")
    if len(parts) != CHANNELS:
        return None
    try:
        return [int(p) for p in parts]
    except ValueError:
        return None


def collect(ser, duration_s: float, label: str) -> tuple[list[int], list[int]]:
    cal_min = [4095] * CHANNELS
    cal_max = [0]    * CHANNELS
    deadline = time.monotonic() + duration_s
    last_print = 0.0

    while time.monotonic() < deadline:
        raw = read_raw(ser)
        if raw is None:
            continue
        for i in range(CHANNELS):
            if raw[i] < cal_min[i]: cal_min[i] = raw[i]
            if raw[i] > cal_max[i]: cal_max[i] = raw[i]

        now = time.monotonic()
        remaining = deadline - now
        if now - last_print >= 0.2:
            last_print = now
            row = " ".join(f"{v:4d}" for v in raw)
            sys.stdout.write(f"\r{label} ({remaining:.1f}s)  Raw: {row}   ")
            sys.stdout.flush()

    print()
    return cal_min, cal_max


def main():
    parser = argparse.ArgumentParser(description="Teensy sensor calibration")
    parser.add_argument("--port", default=_default_port())
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud} baud")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.write(b'C')   # enter calibration mode
    time.sleep(0.1)
    ser.reset_input_buffer()

    print("\n=== STEP 1: Hold robot over BARE GROUND (pavement, not the line) ===")
    input("Press Enter when ready...")
    min1, max1 = collect(ser, 3.0, "Bare ground")

    print("\n=== STEP 2: Slowly slide robot ACROSS THE FULL LINE WIDTH ===")
    input("Press Enter when ready, then move the robot...")
    min2, max2 = collect(ser, 5.0, "Line sweep ")

    # Merge: true min/max across both passes
    final_min = [min(min1[i], min2[i]) for i in range(CHANNELS)]
    final_max = [max(max1[i], max2[i]) for i in range(CHANNELS)]

    print("\n=== Calibration summary ===")
    print("Chan: " + " ".join(f"{i:4d}" for i in range(CHANNELS)))
    print("Min:  " + " ".join(f"{v:4d}" for v in final_min))
    print("Max:  " + " ".join(f"{v:4d}" for v in final_max))
    print("Span: " + " ".join(f"{final_max[i]-final_min[i]:4d}" for i in range(CHANNELS)))

    low_contrast = [i for i in range(CHANNELS) if final_max[i] - final_min[i] < 200]
    if low_contrast:
        print(f"\nWARNING: Low contrast on channels {low_contrast} — check wiring or sensor height.")

    print("\nSaving to Teensy EEPROM...")
    ser.write(b'S')
    time.sleep(0.3)
    # Read response
    resp = ser.read_until(b'\n', size=64).decode("ascii", errors="replace").strip()
    if "CAL_SAVED" in resp:
        print("Saved OK.")
    else:
        print(f"Response: {resp!r} — if empty, save likely succeeded anyway.")

    _autosave(final_min, final_max)

    ser.write(b'R')   # back to run mode
    ser.close()
    print("Done. Teensy is back in run mode.")


if __name__ == "__main__":
    main()
