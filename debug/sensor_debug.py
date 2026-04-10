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


def _default_port() -> str:
    """Read Teensy port from config/params.yaml, falling back to /dev/ttyACM0."""
    try:
        import pathlib, re
        cfg = (pathlib.Path(__file__).parent.parent / "config" / "params.yaml").read_text()
        m = re.search(r"^\s*port:\s*(\S+)", cfg[cfg.index("sensor:"):], re.MULTILINE)
        if m:
            return m.group(1)
    except Exception:
        pass
    return "/dev/ttyACM0"


MEAN_SCALE = 7   # must match firmware constant
DISABLE_MASK = (1<<13)|(1<<14)|(1<<15)  # must match firmware SENSOR_DISABLE_MASK


def compute(raw: list[int], _cal_min=None, _cal_max=None):
    """Compute line_pos and confidence using mean-relative normalization.

    Matches firmware send_packet(): white line = dip below array mean.
    Immune to global illumination shifts (sun/shade).
    """
    mean_val = sum(raw) / CHANNELS
    norm = []
    for v in raw:
        dev = mean_val - v          # positive = below mean = white line
        n   = int(dev * MEAN_SCALE)
        v   = max(0, min(1000, n))
        norm.append(0 if (DISABLE_MASK & (1 << i)) else v)

    total = sum(norm)
    flags = sum(1 << i for i in range(CHANNELS) if norm[i] > 500)

    if total < 100:
        line_pos = 0.0
    else:
        peak_idx = max(range(CHANNELS), key=lambda i: norm[i])
        pos_scaled = (peak_idx - 7) * 1000 + 500   # -6500 to +8500
        line_pos = max(-1.0, min(1.0, pos_scaled / 7500.0))

    confidence = min(255, int(total * 255 / 16000))
    return norm, line_pos, confidence, flags


def bar(norm: list[int]) -> str:
    """Compact intensity bar using block characters."""
    blocks = " ▁▂▃▄▅▆▇█"
    return "".join(blocks[min(8, v * 9 // 1001)] for v in norm)


def main():
    parser = argparse.ArgumentParser(description="Teensy sensor stream debug")
    parser.add_argument("--port", default=_default_port())
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

    packets_ok  = 0
    packets_bad = 0
    last_print  = time.monotonic()

    CHAN_HDR = "Chan: " + " ".join(f"{i:4d}" for i in range(CHANNELS))

    print("Mean-relative mode — no calibration needed.")
    print("White line shows as dip below array mean → high Norm value.")
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

            now = time.monotonic()
            if now - last_print >= 0.05:   # ~20 Hz display
                last_print = now
                norm, line_pos, conf, flags = compute(raw)
                mean_val = int(sum(raw) / CHANNELS)

                raw_row  = " ".join(f"{v:4d}" for v in raw)
                norm_row = " ".join(f"{v:4d}" for v in norm)
                intensity = bar(norm)

                sys.stdout.write(
                    f"\r{CHAN_HDR}\n"
                    f"\rRaw:  {raw_row}\n"
                    f"\rNorm: {norm_row}\n"
                    f"\r[{intensity}]\n"
                    f"\rPos: {line_pos:+.4f}  Conf: {conf:3d}  Mean: {mean_val}  "
                    f"ok:{packets_ok} bad:{packets_bad}   "
                    f"\033[5A"   # move cursor up 5 lines for next refresh
                )
                sys.stdout.flush()

    except KeyboardInterrupt:
        print(f"\n\n\n\n\nDone. {packets_ok} good packets, {packets_bad} bad.")
    finally:
        ser.write(b'R')  # restore run mode
        ser.close()


if __name__ == "__main__":
    main()
