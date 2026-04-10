#!/usr/bin/env python3
"""Multi-surface sensor calibration — captures ground + line on N concrete types.

For each surface:
  1. Hold robot over BARE GROUND for 3 s
  2. Slowly slide robot across the full LINE WIDTH for 5 s

All passes are merged (global min/max across every surface) and saved to
Teensy EEPROM as a single calibration.  This gives the widest possible
dynamic range so the sensor works on all surface types simultaneously.

Usage:
    python3 debug/calibrate_multi.py [--port /dev/ttyACM0] [--surfaces 5]
    bash scripts/run_calibrate_multi.sh
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
    print("ERROR: pyserial not installed.  Run: pip install pyserial")
    sys.exit(1)

CHANNELS = 16
_CAL_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "calibrations")
_INDEX_FILE = os.path.join(_CAL_DIR, "index.json")

BARS = " ▁▂▃▄▅▆▇█"


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


def _span_bar(span: int) -> str:
    """One-char visual indicator for a channel's span (0–4095 scale)."""
    idx = min(8, span * 9 // 800)   # full bar at span ≥ 800
    return BARS[idx]


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
    """Collect raw values for duration_s seconds; return (cal_min, cal_max)."""
    cal_min = [4095] * CHANNELS
    cal_max = [0]    * CHANNELS
    deadline   = time.monotonic() + duration_s
    last_print = 0.0

    while time.monotonic() < deadline:
        raw = read_raw(ser)
        if raw is None:
            continue
        for i in range(CHANNELS):
            if raw[i] < cal_min[i]: cal_min[i] = raw[i]
            if raw[i] > cal_max[i]: cal_max[i] = raw[i]

        now = time.monotonic()
        if now - last_print >= 0.15:
            last_print = now
            remaining = deadline - now
            row = " ".join(f"{v:4d}" for v in raw)
            sys.stdout.write(f"\r  {label} ({remaining:.1f}s)  Raw: {row}   ")
            sys.stdout.flush()

    print()
    return cal_min, cal_max


def merge(global_min, global_max, new_min, new_max):
    return (
        [min(global_min[i], new_min[i]) for i in range(CHANNELS)],
        [max(global_max[i], new_max[i]) for i in range(CHANNELS)],
    )


def print_summary(cal_min, cal_max, label=""):
    span = [cal_max[i] - cal_min[i] for i in range(CHANNELS)]
    bar  = "".join(_span_bar(s) for s in span)
    low  = [i for i in range(CHANNELS) if span[i] < 200]
    print(f"\n  {'─'*56}")
    if label:
        print(f"  {label}")
    print("  Chan: " + " ".join(f"{i:3d}" for i in range(CHANNELS)))
    print("  Min:  " + " ".join(f"{v:3d}" for v in cal_min))
    print("  Max:  " + " ".join(f"{v:3d}" for v in cal_max))
    print("  Span: " + " ".join(f"{s:3d}" for s in span))
    print(f"  [{bar}]")
    if low:
        print(f"  ⚠ Low contrast channels (span < 200): {low}")
    else:
        print("  ✓ All channels have adequate contrast (span ≥ 200)")
    print(f"  {'─'*56}")


def autosave(cal_min, cal_max, n_surfaces: int):
    os.makedirs(_CAL_DIR, exist_ok=True)
    ts    = datetime.now()
    label = ts.strftime("%Y-%m-%d_%H%M%S")
    span  = [cal_max[i] - cal_min[i] for i in range(CHANNELS)]
    low   = [i for i in range(CHANNELS) if span[i] < 200]
    data  = {
        "label":                 label,
        "timestamp":             ts.isoformat(),
        "notes":                 f"calibrate_multi.py ({n_surfaces} surfaces)",
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
        try:
            with open(_INDEX_FILE) as f:
                index = json.load(f)
        except Exception:
            pass
    index[label] = {
        "timestamp":             ts.isoformat(),
        "file":                  f"{label}.json",
        "notes":                 f"calibrate_multi.py ({n_surfaces} surfaces)",
        "low_contrast_channels": low,
    }
    with open(_INDEX_FILE, "w") as f:
        json.dump(index, f, indent=2)

    print(f"  Snapshot saved: calibrations/{label}.json")


def main():
    parser = argparse.ArgumentParser(description="Multi-surface Teensy sensor calibration")
    parser.add_argument("--port",     default=_default_port())
    parser.add_argument("--baud",     type=int, default=115200)
    parser.add_argument("--surfaces", type=int, default=5,
                        help="Number of distinct concrete surfaces to calibrate (default 5)")
    parser.add_argument("--ground-time", type=float, default=3.0,
                        help="Seconds to hold over bare ground per surface (default 3)")
    parser.add_argument("--line-time",   type=float, default=5.0,
                        help="Seconds to sweep over line per surface (default 5)")
    args = parser.parse_args()

    print(f"\nOpening {args.port} @ {args.baud} baud")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.write(b'C')
    time.sleep(0.1)
    ser.reset_input_buffer()

    print(f"""
╔══════════════════════════════════════════════════════════╗
║        MULTI-SURFACE CALIBRATION  ({args.surfaces} surfaces)          ║
║                                                          ║
║  For each surface you will:                              ║
║    1. Hold still on BARE CONCRETE  ({args.ground_time:.0f} s)              ║
║    2. Slide slowly across the WHITE LINE  ({args.line_time:.0f} s)          ║
║                                                          ║
║  All surfaces are merged into one calibration.           ║
╚══════════════════════════════════════════════════════════╝""")

    global_min = [4095] * CHANNELS
    global_max = [0]    * CHANNELS

    for surface_n in range(1, args.surfaces + 1):
        print(f"\n{'━'*60}")
        print(f"  SURFACE {surface_n} of {args.surfaces}")
        print(f"{'━'*60}")

        # ── Ground pass ──────────────────────────────────────────────────────
        print(f"\n  STEP A — Hold sensor over BARE CONCRETE on surface {surface_n}")
        print(f"           (NOT over the white line)")
        try:
            input("  Press Enter when in position... ")
        except (EOFError, KeyboardInterrupt):
            print("\n[aborted]")
            ser.write(b'R')
            ser.close()
            sys.exit(0)

        gmin, gmax = collect(ser, args.ground_time, f"S{surface_n} ground")

        # ── Line pass ────────────────────────────────────────────────────────
        print(f"\n  STEP B — Slowly slide across the WHITE LINE on surface {surface_n}")
        print(f"           Move the robot so the line passes under the full sensor array")
        try:
            input("  Press Enter, then start moving... ")
        except (EOFError, KeyboardInterrupt):
            print("\n[aborted]")
            ser.write(b'R')
            ser.close()
            sys.exit(0)

        lmin, lmax = collect(ser, args.line_time, f"S{surface_n} line  ")

        # ── Merge this surface into global ───────────────────────────────────
        surf_min = [min(gmin[i], lmin[i]) for i in range(CHANNELS)]
        surf_max = [max(gmax[i], lmax[i]) for i in range(CHANNELS)]
        global_min, global_max = merge(global_min, global_max, surf_min, surf_max)

        span = [surf_max[i] - surf_min[i] for i in range(CHANNELS)]
        bar  = "".join(_span_bar(s) for s in span)
        low  = [i for i in range(CHANNELS) if span[i] < 200]
        print(f"\n  Surface {surface_n} contrast: [{bar}]", end="")
        if low:
            print(f"  ⚠ weak channels: {low}", end="")
        print()

        # Show running merged state after each surface
        merged_span = [global_max[i] - global_min[i] for i in range(CHANNELS)]
        merged_bar  = "".join(_span_bar(s) for s in merged_span)
        print(f"  Merged so far:   [{merged_bar}]")

    # ── Final summary ─────────────────────────────────────────────────────────
    print_summary(global_min, global_max,
                  label=f"Final merged calibration ({args.surfaces} surfaces)")

    # ── Save to EEPROM ────────────────────────────────────────────────────────
    print("\n  Saving to Teensy EEPROM...")
    ser.write(b'S')
    time.sleep(0.3)
    resp = ser.read_until(b'\n', size=64).decode("ascii", errors="replace").strip()
    if "CAL_SAVED" in resp:
        print("  Saved OK.")
    else:
        print(f"  Response: {resp!r} — if empty, save likely succeeded anyway.")

    autosave(global_min, global_max, args.surfaces)

    ser.write(b'R')
    ser.close()
    print("  Done. Teensy is back in run mode.\n")


if __name__ == "__main__":
    main()
