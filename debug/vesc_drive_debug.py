#!/usr/bin/env python3
"""Interactive VESC duty-cycle debug tool.

Usage:
    python3 debug/vesc_drive_debug.py [--port <dev>] [--step 0.01] [--max-duty 0.3]

Keys:
    UP    arrow   Increase duty by --step
    DOWN  arrow   Decrease duty by --step
    0             Stop (duty = 0.0)
    q / Ctrl+C    Stop motor and exit

Live telemetry (RPM, velocity, voltage, duty) is shown on every update.
Port defaults to vesc.port in config/params.yaml.
"""

import argparse
import sys
import time
import threading
import termios
import tty

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip3 install pyserial")
    sys.exit(1)

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from vesc_debug import _make_packet, _find_packet, _parse_get_values, GET_VALUES_REQUEST  # noqa: E402
import struct as _struct

COMM_SET_DUTY = 5

def _make_set_duty_packet(duty: float) -> bytes:
    duty = max(-1.0, min(1.0, duty))
    payload = bytes([COMM_SET_DUTY]) + _struct.pack(">i", int(duty * 100000))
    return _make_packet(payload)


# ── Telemetry reader thread ───────────────────────────────────────────────────

class Telemetry:
    def __init__(self, circumference, gear_ratio):
        self._circ  = circumference
        self._gear  = gear_ratio
        self.rpm     = 0.0
        self.voltage = 0.0
        self.duty    = 0.0
        self.vel     = 0.0
        self._lock   = threading.Lock()

    def update(self, rpm, voltage, duty):
        vel = (rpm / self._gear / 60.0) * self._circ
        with self._lock:
            self.rpm = rpm; self.voltage = voltage
            self.duty = duty; self.vel = vel

    def snapshot(self):
        with self._lock:
            return self.rpm, self.voltage, self.duty, self.vel


def _telem_thread(ser, telem, stop_event):
    buf = bytearray()
    while not stop_event.is_set():
        try:
            ser.write(GET_VALUES_REQUEST)
            time.sleep(0.05)
            incoming = ser.read(ser.in_waiting or 0)
        except Exception:
            time.sleep(0.1)
            continue
        if incoming:
            buf.extend(incoming)
            while buf:
                payload, consumed = _find_packet(buf)
                if payload is None:
                    break
                del buf[:consumed]
                fields = _parse_get_values(payload)
                if fields is not None:
                    telem.update(fields["rpm"], fields["input_voltage"],
                                 fields["duty_cycle_now"])
        time.sleep(0.1)


# ── Default port from config ──────────────────────────────────────────────────

def _default_port():
    try:
        import pathlib, re
        cfg = (pathlib.Path(__file__).parent.parent / "config" / "params.yaml").read_text()
        m = re.search(r"^\s*port:\s*(\S+)", cfg[cfg.index("vesc:"):], re.MULTILINE)
        if m:
            return m.group(1)
    except Exception:
        pass
    return "/dev/ttyACM1"


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Interactive VESC duty debug")
    parser.add_argument("--port",          default=_default_port())
    parser.add_argument("--baud",          type=int,   default=115200)
    parser.add_argument("--step",          type=float, default=0.01,
                        help="Duty cycle increment per keypress (default: 0.01 = 1%%)")
    parser.add_argument("--max-duty",      type=float, default=0.3,
                        help="Maximum allowed duty cycle (default: 0.30 = 30%%)")
    parser.add_argument("--circumference", type=float, default=0.204)
    parser.add_argument("--gear-ratio",    type=float, default=8.0)
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud} baud")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    telem      = Telemetry(args.circumference, args.gear_ratio)
    stop_event = threading.Event()
    t = threading.Thread(target=_telem_thread, args=(ser, telem, stop_event), daemon=True)
    t.start()

    duty = 0.0

    def send(d):
        ser.write(_make_set_duty_packet(d))

    def redraw():
        rpm, voltage, cur_duty, vel = telem.snapshot()
        filled = int(abs(duty) / max(args.max_duty, 0.01) * 20)
        bar    = ("█" if duty >= 0 else "▓") * filled + "░" * (20 - filled)
        sign   = "+" if duty >= 0 else "-"
        print(
            f"\rCmd: [{bar}] {duty:+.3f}  |  "
            f"RPM: {rpm:+8.1f}  Vel: {vel:+.3f} m/s  "
            f"Volt: {voltage:.1f}V  Duty(live): {cur_duty:+.3f}    ",
            end="", flush=True,
        )

    print(f"\nUP / DOWN arrows to change duty  |  0 to stop  |  q to quit")
    print(f"Step: {args.step:.3f}   Max: ±{args.max_duty:.2f}\n")

    send(0.0)
    redraw()

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)

            if ch == "\x1b":
                rest = sys.stdin.read(2)
                if rest == "[A":       # UP
                    duty = min(args.max_duty, round(duty + args.step, 4))
                    send(duty)
                elif rest == "[B":     # DOWN
                    duty = max(-args.max_duty, round(duty - args.step, 4))
                    send(duty)
            elif ch == "0":
                duty = 0.0
                send(duty)
            elif ch in ("q", "Q", "\x03"):
                break

            redraw()

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        stop_event.set()
        print("\n\nStopping motor.")
        send(0.0)
        time.sleep(0.1)
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
