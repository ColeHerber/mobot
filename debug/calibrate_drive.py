#!/usr/bin/env python3
"""Drive-through sensor calibration — push or motor-drive over full course.

Two modes:
  --push   (default) Push robot by hand. Live display only, press Enter to save.
  --drive  Motor drives at very slow speed. Does NOT save automatically —
           you review the contrast and decide whether to save.

Usage:
    python3 debug/calibrate_drive.py --port /dev/ttyACM0 [--push | --drive]
    python3 debug/calibrate_drive.py --vesc /dev/ttyACM1 --drive --duty 0.04

In both modes the Teensy accumulates min/max internally. Press Enter at the
end to save to EEPROM, or Ctrl+C to abort without saving.
"""

import argparse
import json
import os
import sys
import time
import threading
from datetime import datetime

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

try:
    import pyvesc
    from pyvesc import SetDutyCycle
    _PYVESC_OK = True
except ImportError:
    _PYVESC_OK = False

CHANNELS = 16
BLOCKS   = " ▁▂▃▄▅▆▇█"

_CAL_DIR    = os.path.join(os.path.dirname(os.path.dirname(__file__)), "calibrations")


def _default_teensy_port() -> str:
    try:
        import pathlib, re
        cfg = (pathlib.Path(__file__).parent.parent / "config" / "params.yaml").read_text()
        m = re.search(r"^\s*port:\s*(\S+)", cfg[cfg.index("sensor:"):], re.MULTILINE)
        if m:
            return m.group(1)
    except Exception:
        pass
    return "/dev/ttyACM0"


def _default_vesc_port() -> str:
    try:
        import pathlib, re
        cfg = (pathlib.Path(__file__).parent.parent / "config" / "params.yaml").read_text()
        m = re.search(r"^\s*port:\s*(\S+)", cfg[cfg.index("vesc:"):], re.MULTILINE)
        if m:
            return m.group(1)
    except Exception:
        pass
    return "/dev/ttyACM1"
_INDEX_FILE = os.path.join(_CAL_DIR, "index.json")


def _autosave(cal_min: list, cal_max: list, note: str):
    """Write a timestamped calibration snapshot to calibrations/ on the Pi."""
    os.makedirs(_CAL_DIR, exist_ok=True)
    ts    = datetime.now()
    label = ts.strftime("%Y-%m-%d_%H%M%S")
    span  = [cal_max[i] - cal_min[i] for i in range(CHANNELS)]
    low   = [i for i in range(CHANNELS) if span[i] < 200]

    data = {
        "label":                 label,
        "timestamp":             ts.isoformat(),
        "notes":                 note,
        "channels":              CHANNELS,
        "cal_min":               cal_min,
        "cal_max":               cal_max,
        "span":                  span,
        "low_contrast_channels": low,
    }
    path = os.path.join(_CAL_DIR, f"{label}.json")
    with open(path, "w") as f:
        json.dump(data, f, indent=2)

    # Update index
    index = {}
    if os.path.exists(_INDEX_FILE):
        with open(_INDEX_FILE) as f:
            index = json.load(f)
    index[label] = {
        "timestamp":             ts.isoformat(),
        "file":                  f"{label}.json",
        "notes":                 note,
        "low_contrast_channels": low,
    }
    with open(_INDEX_FILE, "w") as f:
        json.dump(index, f, indent=2)

    print(f"Snapshot saved: calibrations/{label}.json")


# ─── VESC helpers ─────────────────────────────────────────────────────────────

class VESCDrive:
    def __init__(self, port: str, baud: int = 115200):
        self._ser = serial.Serial(port, baud, timeout=0.05)
        self._duty = 0.0
        self._lock = threading.Lock()
        self._stop_flag = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def set_duty(self, duty: float):
        with self._lock:
            self._duty = max(-1.0, min(1.0, duty))

    def stop(self):
        self.set_duty(0.0)
        time.sleep(0.1)
        self._stop_flag = True

    def _run(self):
        while not self._stop_flag:
            with self._lock:
                duty = self._duty
            try:
                self._ser.write(pyvesc.encode(SetDutyCycle(duty)))
            except Exception:
                pass
            time.sleep(0.02)   # 50 Hz
        try:
            self._ser.write(pyvesc.encode(SetDutyCycle(0.0)))
        except Exception:
            pass
        self._ser.close()


# ─── Sensor reader thread ──────────────────────────────────────────────────────

def start_sensor_reader(ser, state: dict, cal_min: list, cal_max: list):
    def reader():
        while not state["stop"]:
            try:
                line = ser.readline().decode("ascii", errors="replace").strip()
            except Exception:
                continue
            if not line or not line[0].isdigit() and line[0] != '-':
                continue
            parts = line.split(",")
            if len(parts) != CHANNELS:
                continue
            try:
                raw = [int(p) for p in parts]
            except ValueError:
                continue
            state["raw"] = raw
            state["packets"] += 1
            for i in range(CHANNELS):
                if raw[i] < cal_min[i]: cal_min[i] = raw[i]
                if raw[i] > cal_max[i]: cal_max[i] = raw[i]

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    return t


# ─── Display ──────────────────────────────────────────────────────────────────

CHAN_HDR = "Chan: " + " ".join(f"{i:4d}" for i in range(CHANNELS))


def display(state: dict, cal_min: list, cal_max: list, extra_line: str = ""):
    raw  = state["raw"]
    span = [cal_max[i] - cal_min[i] for i in range(CHANNELS)]
    bar  = "".join(BLOCKS[min(8, s * 9 // 4096)] for s in span)
    raw_row  = " ".join(f"{v:4d}" for v in raw)
    span_row = " ".join(f"{v:4d}" for v in span)
    low = sum(1 for s in span if s < 200)

    sys.stdout.write(
        f"\r{CHAN_HDR}\n"
        f"\rRaw:  {raw_row}\n"
        f"\rSpan: {span_row}\n"
        f"\r[{bar}]  packets:{state['packets']}  low-contrast channels:{low}\n"
        f"\r{extra_line:<72}"
        f"\033[4A"
    )
    sys.stdout.flush()


# ─── Save / abort ─────────────────────────────────────────────────────────────

def save_cal(ser, cal_min: list, cal_max: list, note: str = "calibrate_drive.py"):
    low = [i for i in range(CHANNELS) if cal_max[i] - cal_min[i] < 200]
    if low:
        print(f"\nWARNING: Low contrast on channels {low} — those sensors may not")
        print("         have seen both line and pavement. Re-run if possible.")

    print("\nSaving calibration to Teensy EEPROM...")
    ser.write(b'S')
    time.sleep(0.5)
    resp = ser.read_until(b'\n', size=64).decode("ascii", errors="replace").strip()
    if "CAL_SAVED" in resp or not resp:
        print("Saved OK.")
    else:
        print(f"Teensy response: {resp!r}")

    _autosave(cal_min, cal_max, note)

    print("\n=== Final calibration ===")
    print("Chan: " + " ".join(f"{i:4d}" for i in range(CHANNELS)))
    print("Min:  " + " ".join(f"{v:4d}" for v in cal_min))
    print("Max:  " + " ".join(f"{v:4d}" for v in cal_max))
    print("Span: " + " ".join(f"{cal_max[i]-cal_min[i]:4d}" for i in range(CHANNELS)))


# ─── Modes ────────────────────────────────────────────────────────────────────

def run_push(ser):
    """Push-by-hand mode. Press Enter to save."""
    print("\n=== PUSH MODE ===")
    print("Push the robot slowly over the entire course by hand.")
    print("Cover all line segments and surrounding pavement.")
    print("Watch the Span row build up — all channels should reach > 200.")
    print("Press Enter when done (or Ctrl+C to abort without saving).\n")

    state   = {"raw": [0] * CHANNELS, "packets": 0, "stop": False}
    cal_min = [4095] * CHANNELS
    cal_max = [0]    * CHANNELS

    start_sensor_reader(ser, state, cal_min, cal_max)

    # Wait for Enter in a background thread
    entered = threading.Event()
    threading.Thread(target=lambda: (input(), entered.set()), daemon=True).start()

    try:
        while not entered.is_set():
            display(state, cal_min, cal_max, "Push robot over course — press Enter when done...")
            time.sleep(0.05)
    except KeyboardInterrupt:
        state["stop"] = True
        print("\n\n\n\n\nAborted — calibration NOT saved.")
        ser.write(b'R')
        ser.close()
        sys.exit(0)

    state["stop"] = True
    print("\n\n\n\n")
    save_cal(ser, cal_min, cal_max, note="calibrate_drive.py --push")


def run_drive(ser, vesc_port: str, vesc_baud: int, duty: float):
    """Motor-drive mode. Robot drives itself; does NOT auto-save."""
    if not _PYVESC_OK:
        print("ERROR: pyvesc not installed — cannot use --drive mode.")
        print("       pip install pyvesc")
        sys.exit(1)

    print(f"\n=== DRIVE MODE === (duty={duty:.3f})")
    print("The robot will drive itself at low speed.")
    print("Make sure the course is clear and the robot is at the start line.")
    print("Ctrl+C to stop the motor and decide whether to save.\n")
    input("Press Enter to start driving...")

    vesc = VESCDrive(vesc_port, vesc_baud)
    vesc.start()

    state   = {"raw": [0] * CHANNELS, "packets": 0, "stop": False}
    cal_min = [4095] * CHANNELS
    cal_max = [0]    * CHANNELS

    start_sensor_reader(ser, state, cal_min, cal_max)

    vesc.set_duty(duty)

    try:
        while True:
            display(state, cal_min, cal_max, f"Driving at duty={duty:.3f} — Ctrl+C to stop motor...")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

    vesc.stop()
    state["stop"] = True
    print("\n\n\n\n\nMotor stopped.")

    low = [i for i in range(CHANNELS) if cal_max[i] - cal_min[i] < 200]
    if low:
        print(f"WARNING: Low contrast on channels {low}.")

    ans = input("\nSave calibration to EEPROM? [y/N] ").strip().lower()
    if ans == "y":
        save_cal(ser, cal_min, cal_max, note="calibrate_drive.py --drive")
    else:
        print("Calibration NOT saved.")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Drive-through sensor calibration")
    parser.add_argument("--port",      default=_default_teensy_port(),
                        help="Teensy serial port")
    parser.add_argument("--baud",      type=int, default=115200)
    parser.add_argument("--vesc",      default=_default_vesc_port(),
                        help="VESC serial port (--drive mode only)")
    parser.add_argument("--vesc-baud", type=int, default=115200)
    parser.add_argument("--duty",      type=float, default=0.04,
                        help="Motor duty cycle for --drive mode (default: 0.04 = ~4%%)")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--push",  action="store_true", default=True,
                       help="Push robot by hand (default)")
    group.add_argument("--drive", action="store_true",
                       help="Motor-drive at low speed")
    args = parser.parse_args()

    print(f"Opening Teensy on {args.port} @ {args.baud} baud")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.write(b'C')
    time.sleep(0.1)
    ser.reset_input_buffer()

    try:
        if args.drive:
            run_drive(ser, args.vesc, args.vesc_baud, args.duty)
        else:
            run_push(ser)
    finally:
        ser.write(b'R')
        ser.close()
        print("\nTeensy restored to run mode.")


if __name__ == "__main__":
    main()
