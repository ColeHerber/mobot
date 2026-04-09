#!/usr/bin/env python3
"""Hardware PWM servo debug — uses /sys/class/pwm/ sysfs directly.

Usage:
    python3 debug/servo_hwpwm_debug.py [--pwmchip 0] [--channel 0]
                                        [--center 1500] [--min 1000] [--max 2000]

Keys:
    +  /  =    Increase pulse width by 10 µs
    -          Decrease pulse width by 10 µs
    0          Return to center
    l          Go to min_pw (full left)
    r          Go to max_pw (full right)
    s          Sweep full range once
    q / Ctrl+C Exit

Use alongside servo_debug.py (software PWM) to compare and isolate wiring vs
driver issues:
    Software PWM:  python3 debug/servo_debug.py --pin 12 --gpiochip 4
    Hardware PWM:  python3 debug/servo_hwpwm_debug.py --pwmchip 0 --channel 0
"""

import argparse
import os
import sys
import time
import termios
import tty

PERIOD_NS = 20_000_000   # 50 Hz


def _write(path: str, value):
    with open(path, "w") as f:
        f.write(str(value))


def _pwm_path(chip: int, channel: int, filename: str) -> str:
    return f"/sys/class/pwm/pwmchip{chip}/pwm{channel}/{filename}"


def set_pw(chip: int, channel: int, pw_us: int):
    _write(_pwm_path(chip, channel, "duty_cycle"), pw_us * 1000)
    print(f"\rPulse width: {pw_us} µs    ", end="", flush=True)


def sweep(chip: int, channel: int, min_pw: int, max_pw: int, step: int = 20):
    print("\nSweeping…")
    for pw in range(min_pw, max_pw + 1, step):
        set_pw(chip, channel, pw)
        time.sleep(0.01)
    for pw in range(max_pw, min_pw - 1, -step):
        set_pw(chip, channel, pw)
        time.sleep(0.01)
    print()


def setup(chip: int, channel: int, center_us: int) -> bool:
    chip_dir  = f"/sys/class/pwm/pwmchip{chip}"
    pwm_dir   = f"{chip_dir}/pwm{channel}"
    export    = f"{chip_dir}/export"
    unexport  = f"{chip_dir}/unexport"

    if not os.path.exists(chip_dir):
        print(f"ERROR: {chip_dir} not found.")
        print("       Check that dtoverlay=pwm,pin=12,func=4 is in /boot/firmware/config.txt")
        print("       and the Pi has been rebooted.")
        return False

    # Clean slate: unexport if already exported
    if os.path.exists(pwm_dir):
        try:
            _write(unexport, channel)
            time.sleep(0.1)
        except OSError:
            pass

    try:
        _write(export, channel)
        time.sleep(0.1)
    except OSError as e:
        print(f"ERROR: Failed to export pwm{channel}: {e}")
        return False

    try:
        _write(_pwm_path(chip, channel, "enable"),     0)
        _write(_pwm_path(chip, channel, "period"),     PERIOD_NS)
        _write(_pwm_path(chip, channel, "duty_cycle"), center_us * 1000)
        _write(_pwm_path(chip, channel, "enable"),     1)
    except OSError as e:
        print(f"ERROR: sysfs write failed: {e}")
        return False

    return True


def teardown(chip: int, channel: int, center_us: int):
    try:
        set_pw(chip, channel, center_us)
        time.sleep(0.2)
        _write(_pwm_path(chip, channel, "enable"), 0)
        _write(f"/sys/class/pwm/pwmchip{chip}/unexport", channel)
    except OSError:
        pass


def main():
    parser = argparse.ArgumentParser(description="Hardware PWM servo debug via sysfs")
    parser.add_argument("--pwmchip",  type=int, default=0,    help="pwmchip number (default: 0)")
    parser.add_argument("--channel",  type=int, default=0,    help="PWM channel (default: 0)")
    parser.add_argument("--center",   type=int, default=1500, help="Center µs (default: 1500)")
    parser.add_argument("--min",      type=int, default=1000, dest="min_pw")
    parser.add_argument("--max",      type=int, default=2000, dest="max_pw")
    args = parser.parse_args()

    print(f"Hardware PWM: /sys/class/pwm/pwmchip{args.pwmchip}/pwm{args.channel}")
    print(f"  Period: {PERIOD_NS // 1_000_000} ms (50 Hz)")
    print(f"  Range:  {args.min_pw}–{args.max_pw} µs   Center: {args.center} µs")
    print()

    if not setup(args.pwmchip, args.channel, args.center):
        sys.exit(1)

    print("PWM enabled. If the servo is wired to physical pin 32 (GPIO 12),")
    print("it should now be at center position.")
    print()
    print("Controls:  +/=  increase  |  -  decrease  |  0  center  |  l  min  |  r  max  |  s  sweep  |  q  quit")
    print()

    current = args.center
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch in ("+", "="):
                current = min(args.max_pw, current + 10)
                set_pw(args.pwmchip, args.channel, current)
            elif ch == "-":
                current = max(args.min_pw, current - 10)
                set_pw(args.pwmchip, args.channel, current)
            elif ch == "0":
                current = args.center
                set_pw(args.pwmchip, args.channel, current)
            elif ch == "l":
                current = args.min_pw
                set_pw(args.pwmchip, args.channel, current)
            elif ch == "r":
                current = args.max_pw
                set_pw(args.pwmchip, args.channel, current)
            elif ch == "s":
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                sweep(args.pwmchip, args.channel, args.min_pw, args.max_pw)
                current = args.center
                set_pw(args.pwmchip, args.channel, current)
                tty.setraw(fd)
            elif ch in ("q", "Q", "\x03"):
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print(f"\n\nReturning to center and disabling PWM.")
        teardown(args.pwmchip, args.channel, args.center)


if __name__ == "__main__":
    main()
