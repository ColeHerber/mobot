#!/usr/bin/env python3
"""Calibration snapshot manager — save and browse Teensy sensor calibrations.

Snapshots are JSON files stored in calibrations/ on the Pi.
They are records only — to re-use a calibration, re-run calibrate_drive.py
in the same conditions.

Usage:
    python3 debug/cal_manager.py save    --label "race_morning_heat1" [--notes "cloudy"]
    python3 debug/cal_manager.py list
    python3 debug/cal_manager.py show    --label "race_morning_heat1"
    python3 debug/cal_manager.py delete  --label "race_morning_heat1"

'save' puts the Teensy in cal mode for a few seconds while stationary to
capture the current min/max environment. Run it immediately after
calibrate_drive.py while conditions are unchanged.
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

CHANNELS   = 16
CAL_DIR    = os.path.join(os.path.dirname(os.path.dirname(__file__)), "calibrations")
INDEX_FILE = os.path.join(CAL_DIR, "index.json")


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


def ensure_dir():
    os.makedirs(CAL_DIR, exist_ok=True)


def load_index() -> dict:
    if not os.path.exists(INDEX_FILE):
        return {}
    with open(INDEX_FILE) as f:
        return json.load(f)


def save_index(index: dict):
    with open(INDEX_FILE, "w") as f:
        json.dump(index, f, indent=2)


def cal_path(label: str) -> str:
    safe = label.replace(" ", "_").replace("/", "-")
    return os.path.join(CAL_DIR, f"{safe}.json")


def capture_snapshot(port: str, baud: int, duration: float) -> tuple[list[int], list[int]]:
    """Open Teensy, enter cal mode, record min/max for duration seconds."""
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.write(b'C')
    time.sleep(0.1)
    ser.reset_input_buffer()

    cal_min = [4095] * CHANNELS
    cal_max = [0]    * CHANNELS
    deadline = time.monotonic() + duration

    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if not line or line.startswith("CAL"):
            continue
        parts = line.split(",")
        if len(parts) != CHANNELS:
            continue
        try:
            raw = [int(p) for p in parts]
        except ValueError:
            continue
        for i in range(CHANNELS):
            if raw[i] < cal_min[i]: cal_min[i] = raw[i]
            if raw[i] > cal_max[i]: cal_max[i] = raw[i]
        remaining = deadline - time.monotonic()
        sys.stdout.write(f"\r  Capturing... {remaining:.1f}s   ")
        sys.stdout.flush()

    print()
    ser.write(b'R')
    ser.close()
    return cal_min, cal_max


def print_cal(cal_min: list, cal_max: list):
    span = [cal_max[i] - cal_min[i] for i in range(CHANNELS)]
    print("Chan: " + " ".join(f"{i:4d}" for i in range(CHANNELS)))
    print("Min:  " + " ".join(f"{v:4d}" for v in cal_min))
    print("Max:  " + " ".join(f"{v:4d}" for v in cal_max))
    print("Span: " + " ".join(f"{v:4d}" for v in span))
    low = [i for i in range(CHANNELS) if span[i] < 200]
    if low:
        print(f"WARNING: Low contrast on channels {low}")


# ─── Commands ─────────────────────────────────────────────────────────────────

def cmd_save(args):
    ensure_dir()
    index = load_index()

    if args.label in index:
        ans = input(f"Label '{args.label}' already exists. Overwrite? [y/N] ").strip().lower()
        if ans != "y":
            print("Aborted.")
            return

    print(f"Connecting to {args.port} — hold robot still...")
    cal_min, cal_max = capture_snapshot(args.port, args.baud, args.duration)
    span = [cal_max[i] - cal_min[i] for i in range(CHANNELS)]
    low  = [i for i in range(CHANNELS) if span[i] < 200]

    data = {
        "label":                 args.label,
        "timestamp":             datetime.now().isoformat(),
        "notes":                 args.notes or "",
        "channels":              CHANNELS,
        "cal_min":               cal_min,
        "cal_max":               cal_max,
        "span":                  span,
        "low_contrast_channels": low,
    }

    path = cal_path(args.label)
    with open(path, "w") as f:
        json.dump(data, f, indent=2)

    index[args.label] = {
        "timestamp":             data["timestamp"],
        "file":                  os.path.basename(path),
        "notes":                 data["notes"],
        "low_contrast_channels": low,
    }
    save_index(index)

    print(f"\nSaved to {path}\n")
    print_cal(cal_min, cal_max)


def cmd_list(args):
    index = load_index()
    if not index:
        print("No saved calibrations.")
        return
    print(f"{'Label':<30}  {'Saved':<20}  {'Low-contrast ch':<18}  Notes")
    print("-" * 88)
    for label, meta in sorted(index.items(), key=lambda x: x[1]["timestamp"]):
        ts  = meta["timestamp"][:19].replace("T", " ")
        low = str(meta.get("low_contrast_channels") or "none")
        print(f"{label:<30}  {ts:<20}  {low:<18}  {meta.get('notes','')}")


def cmd_show(args):
    index = load_index()
    if args.label not in index:
        print(f"ERROR: '{args.label}' not found. Run 'list' to see saved calibrations.")
        sys.exit(1)
    with open(cal_path(args.label)) as f:
        data = json.load(f)
    print(f"Label:  {data['label']}")
    print(f"Saved:  {data['timestamp']}")
    print(f"Notes:  {data.get('notes','')}")
    print()
    print_cal(data["cal_min"], data["cal_max"])


def cmd_delete(args):
    index = load_index()
    if args.label not in index:
        print(f"ERROR: '{args.label}' not found.")
        sys.exit(1)
    ans = input(f"Delete '{args.label}'? [y/N] ").strip().lower()
    if ans != "y":
        print("Aborted.")
        return
    path = cal_path(args.label)
    if os.path.exists(path):
        os.remove(path)
    del index[args.label]
    save_index(index)
    print(f"Deleted '{args.label}'.")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Calibration snapshot manager")
    parser.add_argument("--port", default=_default_port())
    parser.add_argument("--baud", type=int, default=115200)

    sub = parser.add_subparsers(dest="command", required=True)

    p_save = sub.add_parser("save", help="Capture and save a calibration snapshot")
    p_save.add_argument("--label",    required=True)
    p_save.add_argument("--notes",    default="")
    p_save.add_argument("--duration", type=float, default=3.0,
                        help="Seconds to capture stationary (default: 3)")

    sub.add_parser("list", help="List all saved calibrations")

    p_show = sub.add_parser("show", help="Show a saved calibration")
    p_show.add_argument("--label", required=True)

    p_del = sub.add_parser("delete", help="Delete a saved calibration")
    p_del.add_argument("--label", required=True)

    args = parser.parse_args()
    {"save": cmd_save, "list": cmd_list, "show": cmd_show, "delete": cmd_del}[args.command](args)


if __name__ == "__main__":
    main()
