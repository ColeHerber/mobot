#!/usr/bin/env python3
"""Run the Pico bridge safe pin-short test and summarize likely bridges."""

from __future__ import annotations

import argparse
import glob
import os
import re
import select
import sys
import termios
import time


DEFAULT_BAUD = 115200
SAMPLE_RE = re.compile(r"(BASE|FINAL|LEVEL|DRIVE GP\d+/[A-Za-z0-9_]+ LOW)(.*)")
PIN_RE = re.compile(r"GP(\d+)/([A-Za-z0-9_]+)=([01])")
DRIVE_RE = re.compile(r"DRIVE GP(\d+)/([A-Za-z0-9_]+) LOW")


def configure_serial(fd: int, baud: int = DEFAULT_BAUD) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = baud
    attrs[5] = baud
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def serial_exchange(port: str, payload: bytes, timeout_s: float = 1.0) -> bytes:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_serial(fd)
        time.sleep(0.05)
        if payload:
            os.write(fd, payload)

        end = time.time() + timeout_s
        data = bytearray()
        while time.time() < end:
            ready, _, _ = select.select([fd], [], [], 0.05)
            if not ready:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if chunk:
                data.extend(chunk)
        return bytes(data)
    finally:
        os.close(fd)


def usbmodem_ports() -> list[str]:
    return sorted(glob.glob("/dev/cu.usbmodem*"))


def find_control_port(candidates: list[str]) -> str:
    for port in candidates:
        reply = serial_exchange(port, b"h\r", timeout_s=0.8)
        if b"commands:" in reply and b"safe-short-test" in reply:
            return port
    raise RuntimeError(
        "Could not find a Pico control port with the safe-short-test command. "
        "Flash the updated pico_uart_bridge firmware first."
    )


def run_test(control_port: str) -> str:
    reply = serial_exchange(control_port, b"t\r", timeout_s=3.0)
    text = reply.decode("utf-8", "replace")
    if "TEST safe-short end" not in text:
        raise RuntimeError("Pico did not return a complete safe-short test report.")
    return text


def read_levels(control_port: str) -> tuple[str, dict[int, tuple[str, int]]]:
    reply = serial_exchange(control_port, b"l\r", timeout_s=0.8)
    text = reply.decode("utf-8", "replace")
    samples = parse_samples(text)
    for tag, pins in samples:
        if tag == "LEVEL":
            return text, pins
    raise RuntimeError("Pico did not return a LEVEL sample.")


def parse_samples(report: str) -> list[tuple[str, dict[int, tuple[str, int]]]]:
    samples: list[tuple[str, dict[int, tuple[str, int]]]] = []
    for line in report.splitlines():
        match = SAMPLE_RE.search(line)
        if not match:
            continue
        tag = match.group(1)
        pins: dict[int, tuple[str, int]] = {}
        for pin_match in PIN_RE.finditer(line):
            pin = int(pin_match.group(1))
            name = pin_match.group(2)
            value = int(pin_match.group(3))
            pins[pin] = (name, value)
        samples.append((tag, pins))
    return samples


def summarize(samples: list[tuple[str, dict[int, tuple[str, int]]]]) -> list[str]:
    lines: list[str] = []
    base = next((pins for tag, pins in samples if tag == "BASE"), None)
    if base is None:
        return ["No BASE sample found."]

    base_low = [f"GP{pin}/{name}" for pin, (name, value) in base.items() if value == 0]
    if base_low:
        lines.append("Baseline low pins: " + ", ".join(base_low))
    else:
        lines.append("Baseline: all sampled pins read high.")

    bridges: list[str] = []
    for tag, pins in samples:
        drive = DRIVE_RE.match(tag)
        if not drive:
            continue
        driven_pin = int(drive.group(1))
        driven_name = drive.group(2)
        driven_value = pins.get(driven_pin, (driven_name, -1))[1]
        if driven_value != 0:
            bridges.append(f"GP{driven_pin}/{driven_name} did not read low when driven low.")

        for pin, (name, value) in pins.items():
            if pin == driven_pin:
                continue
            base_value = base.get(pin, (name, value))[1]
            if base_value == 1 and value == 0:
                bridges.append(
                    f"Possible bridge: driving GP{driven_pin}/{driven_name} low "
                    f"also pulled GP{pin}/{name} low."
                )

    if bridges:
        lines.extend(bridges)
    else:
        lines.append("No control-pin bridges detected by the safe test.")

    return lines


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run Pico bridge safe GPIO short detection on J3 fixture pins."
    )
    parser.add_argument("--control-port", help="Pico control CDC port. Auto-detected when omitted.")
    parser.add_argument("--raw", action="store_true", help="Print the raw Pico test report too.")
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Passively print pin levels repeatedly. Does not drive any J3 pins.",
    )
    parser.add_argument(
        "--interval",
        default=0.25,
        type=float,
        help="Polling interval for --watch in seconds. Default: 0.25",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        control_port = args.control_port or find_control_port(usbmodem_ports())
        print(f"Pico control: {control_port}")
        if args.watch:
            last = None
            print("Watching passive levels. Press Ctrl-C to stop.")
            try:
                while True:
                    _, pins = read_levels(control_port)
                    state = " ".join(
                        f"GP{pin}/{name}={value}"
                        for pin, (name, value) in sorted(pins.items())
                    )
                    if state != last:
                        print(state)
                        last = state
                    time.sleep(args.interval)
            except KeyboardInterrupt:
                print()
                return 0

        report = run_test(control_port)
        samples = parse_samples(report)
        if args.raw:
            print("\nRaw report:")
            print(report.rstrip())
            print()
        print("Summary:")
        for line in summarize(samples):
            print(f"- {line}")
        return 0
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
