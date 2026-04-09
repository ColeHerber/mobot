#!/usr/bin/env python3
"""Sensor debug tool — stream-print Teensy line-sensor binary packets.

Usage:
    python3 debug/sensor_debug.py [--port /dev/ttyACM0] [--baud 115200] [--raw]

Binary packet format (8 bytes, from firmware):
    [0xAA] [pos_hi] [pos_lo] [flags_hi] [flags_lo] [confidence] [checksum] [0x55]
    line_pos  = int16(pos_hi<<8 | pos_lo) / 10000.0  → [-1.0, +1.0]
    flags     = 16-bit bitmask, bit N set if sensor N above threshold
    checksum  = XOR of bytes 1–5
    confidence = 0–255

Use --raw to send 'C' command and print raw ADC CSV (calibration mode).
Use Ctrl+C to exit.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

PACKET_LEN = 8
SOF = 0xAA
EOF = 0x55
CHANNELS = 16


def parse_packet(buf: bytes):
    """Parse an 8-byte binary packet. Returns (line_pos, confidence, flags) or None."""
    if len(buf) != PACKET_LEN:
        return None
    if buf[0] != SOF or buf[7] != EOF:
        return None
    pos_hi, pos_lo, fl_hi, fl_lo, conf, chk = buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]
    expected_chk = pos_hi ^ pos_lo ^ fl_hi ^ fl_lo ^ conf
    if chk != expected_chk:
        return None
    line_pos = (int.from_bytes(bytes([pos_hi, pos_lo]), "big", signed=True)) / 10000.0
    flags = (fl_hi << 8) | fl_lo
    return line_pos, conf, flags


def flags_bar(flags: int) -> str:
    """16-char bar showing which sensors are active (█ = above threshold)."""
    return "".join("█" if (flags >> i) & 1 else "·" for i in range(CHANNELS))


def find_packet(ser) -> bytes | None:
    """Sync to SOF byte, then read 8 bytes. Returns raw packet or None on timeout."""
    b = ser.read(1)
    if not b or b[0] != SOF:
        return None
    rest = ser.read(PACKET_LEN - 1)
    if len(rest) != PACKET_LEN - 1:
        return None
    return bytes([SOF]) + rest


def run_raw_mode(ser):
    """Send 'C' to enter calibration mode and print raw ADC CSV."""
    ser.write(b'C')
    print("Calibration mode — raw ADC values (0–4095). Press Ctrl+C to exit.")
    print("Chan: " + "  ".join(f"{i:02d}" for i in range(CHANNELS)))
    while True:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if not line:
            continue
        if line.startswith("CAL"):
            print(f"[info] {line}")
            continue
        vals = line.split(",")
        if len(vals) == CHANNELS:
            row = "".join(f"{int(v):5d}" for v in vals)
            print(f"\r{row}", end="", flush=True)
        else:
            print(f"[raw] {line}")


def main():
    parser = argparse.ArgumentParser(description="Teensy sensor stream debug")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--raw", action="store_true",
                        help="Calibration mode: print raw ADC values instead of parsed packets")
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud} baud — Ctrl+C to exit")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(0.5)  # let Teensy enumerate

    if args.raw:
        try:
            run_raw_mode(ser)
        except KeyboardInterrupt:
            ser.write(b'R')  # restore run mode
            print("\nRestored run mode. Exiting.")
            ser.close()
        return

    # Run mode — parse binary packets
    ser.write(b'R')
    print("Run mode — binary packets\n")

    packets_ok = 0
    packets_bad = 0
    last_print = time.monotonic()

    try:
        while True:
            pkt = find_packet(ser)
            if pkt is None:
                continue

            parsed = parse_packet(pkt)
            if parsed is None:
                packets_bad += 1
                continue

            packets_ok += 1
            line_pos, conf, flags = parsed

            now = time.monotonic()
            if now - last_print >= 0.05:   # ~20 Hz display refresh
                last_print = now
                bar = flags_bar(flags)
                active = bin(flags).count("1")
                sys.stdout.write(
                    f"\rPos: {line_pos:+.4f}  Conf: {conf:3d}  "
                    f"Active: {active:2d}/16  [{bar}]  "
                    f"ok:{packets_ok} bad:{packets_bad}   "
                )
                sys.stdout.flush()

    except KeyboardInterrupt:
        print(f"\n\nDone. {packets_ok} good packets, {packets_bad} bad.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
