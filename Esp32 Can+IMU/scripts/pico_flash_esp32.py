#!/usr/bin/env python3
"""Flash the ESP32-S3 through the Pico 2 UART bridge.

The Pico bridge exposes two USB CDC ports:
  - CDC0: UART bridge to ESP32 UART0
  - CDC1: control channel for BOOT/RESET/3v3/CAN debug

This script finds the control channel, sends the Pico's "enter bootloader"
command, runs PlatformIO upload through the UART bridge, releases BOOT, then
resets the ESP32.
"""

from __future__ import annotations

import argparse
import glob
import os
import select
import subprocess
import sys
import termios
import time
from pathlib import Path


DEFAULT_ENV = "esp32-s3-wroom-1-ism6hg256x-debug"
DEFAULT_BAUD = 115200


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
        reply = serial_exchange(port, b"h\r", timeout_s=0.7)
        if b"commands:" in reply and b"enter-bootloader" in reply:
            return port
    raise RuntimeError(
        "Could not find the Pico control CDC port. "
        "Make sure the native pico_uart_bridge firmware is running."
    )


def pick_uart_port(candidates: list[str], control_port: str) -> str:
    other_ports = [port for port in candidates if port != control_port]
    if len(other_ports) == 1:
        return other_ports[0]
    if not other_ports:
        raise RuntimeError("Found the control port, but no separate UART bridge port.")
    raise RuntimeError(
        "Multiple possible UART bridge ports found: "
        + ", ".join(other_ports)
        + ". Pass --uart-port explicitly."
    )


def send_control_command(control_port: str, command: bytes, expected: bytes) -> bytes:
    reply = serial_exchange(control_port, command + b"\r", timeout_s=1.5)
    if expected not in reply:
        decoded = reply.decode("utf-8", "replace").strip()
        raise RuntimeError(
            f"Pico control command {command!r} did not return {expected!r}; "
            f"reply was: {decoded!r}"
        )
    return reply


def run_upload(args: argparse.Namespace, uart_port: str) -> int:
    cmd = [
        args.pio,
        "run",
        "-e",
        args.env,
        "-t",
        "upload",
        "--upload-port",
        uart_port,
    ]
    if args.verbose:
        print("+ " + " ".join(cmd))
    if args.dry_run:
        return 0
    return subprocess.run(cmd, cwd=args.project_dir).returncode


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Auto-enter ESP32 bootloader via Pico control CDC, then flash with PlatformIO."
    )
    parser.add_argument(
        "-e",
        "--env",
        default=DEFAULT_ENV,
        help=f"PlatformIO environment to upload. Default: {DEFAULT_ENV}",
    )
    parser.add_argument(
        "--control-port",
        help="Pico control CDC port. Auto-detected when omitted.",
    )
    parser.add_argument(
        "--uart-port",
        help="Pico UART bridge CDC port. Auto-detected when omitted.",
    )
    parser.add_argument(
        "--project-dir",
        default=Path(__file__).resolve().parents[1],
        type=Path,
        help="PlatformIO project directory. Default: repo root.",
    )
    parser.add_argument(
        "--pio",
        default="pio",
        help="PlatformIO executable. Default: pio",
    )
    parser.add_argument(
        "--no-reset-after",
        action="store_true",
        help="Do not send a normal reset after upload finishes.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print detected ports and upload command without changing BOOT/RESET or flashing.",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Print the PlatformIO command before running it.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    candidates = usbmodem_ports()
    if not candidates:
        print("No /dev/cu.usbmodem* ports found.", file=sys.stderr)
        return 2

    try:
        control_port = args.control_port or find_control_port(candidates)
        uart_port = args.uart_port or pick_uart_port(candidates, control_port)

        print(f"Pico control: {control_port}")
        print(f"ESP32 UART:   {uart_port}")
        print(f"PIO env:      {args.env}")

        if args.dry_run:
            return run_upload(args, uart_port)

        print("Entering ESP32 bootloader via Pico BOOT/RESET...")
        send_control_command(control_port, b"e", b"OK bootloader")

        try:
            upload_rc = run_upload(args, uart_port)
        finally:
            print("Releasing ESP32 BOOT via Pico control...")
            send_control_command(control_port, b"u", b"OK boot released")

        if upload_rc != 0:
            return upload_rc

        if not args.no_reset_after:
            print("Resetting ESP32 via Pico RESET...")
            send_control_command(control_port, b"r", b"OK reset")

        return 0
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
