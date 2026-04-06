#!/usr/bin/env python3
"""Interactive servo sweep / trim tool using lgpio.

Usage:
    python3 servo_debug.py [--pin 18] [--gpiochip 4] [--center 1500]
                           [--min 1000] [--max 2000] [--step 10]

Keys:
    +  /  =    Increase pulse width by --step µs
    -          Decrease pulse width by --step µs
    0          Return to center
    l          Go to min_pw (full left)
    r          Go to max_pw (full right)
    s          Sweep full range once
    q / Ctrl+C Exit

Current pulse width is printed on every change.

Pi 5 note: gpiochip = 4 (RP1). Pi 4: gpiochip = 0.
"""

import argparse
import sys
import time

try:
    import lgpio
    _LGPIO_OK = True
except ImportError:
    print("ERROR: lgpio not installed.")
    print("       sudo apt install python3-lgpio  OR  pip3 install lgpio")
    sys.exit(1)

SERVO_FREQ = 50   # Hz


def set_pw(h, pin: int, pw: int):
    lgpio.tx_servo(h, pin, pw, SERVO_FREQ)
    print(f"\rPulse width: {pw} µs    ", end="", flush=True)


def sweep(h, pin: int, min_pw: int, max_pw: int, step: int = 20):
    """Sweep from min to max and back."""
    print("\nSweeping…")
    for pw in range(min_pw, max_pw + 1, step):
        set_pw(h, pin, pw)
        time.sleep(0.01)
    for pw in range(max_pw, min_pw - 1, -step):
        set_pw(h, pin, pw)
        time.sleep(0.01)
    print()


def main():
    parser = argparse.ArgumentParser(description="Interactive servo debug tool")
    parser.add_argument("--pin",       type=int, default=18,
                        help="GPIO pin (BCM, default: 18)")
    parser.add_argument("--gpiochip",  type=int, default=4,
                        help="lgpio chip: 4=Pi5, 0=Pi4 (default: 4)")
    parser.add_argument("--center",    type=int, default=1500,
                        help="Center pulse width µs (default: 1500)")
    parser.add_argument("--min",       type=int, default=1000, dest="min_pw",
                        help="Min pulse width µs (default: 1000)")
    parser.add_argument("--max",       type=int, default=2000, dest="max_pw",
                        help="Max pulse width µs (default: 2000)")
    parser.add_argument("--step",      type=int, default=10,
                        help="µs per keypress (default: 10)")
    args = parser.parse_args()

    print(f"Opening gpiochip{args.gpiochip}, servo on GPIO {args.pin}")
    try:
        h = lgpio.gpiochip_open(args.gpiochip)
        lgpio.gpio_claim_output(h, args.pin)
    except lgpio.error as e:
        print(f"ERROR: lgpio init failed: {e}")
        sys.exit(1)

    current = args.center
    set_pw(h, args.pin, current)
    time.sleep(0.5)

    print(f"\nControls:  +/=  increase  |  -  decrease  |  0  center  "
          f"|  l  min  |  r  max  |  s  sweep  |  q  quit")
    print(f"Step: {args.step} µs   Range: [{args.min_pw}, {args.max_pw}]   "
          f"Center: {args.center}\n")

    # Use termios for single-keypress reading (no Enter required)
    import termios
    import tty
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch in ("+", "="):
                current = min(args.max_pw, current + args.step)
                set_pw(h, args.pin, current)
            elif ch == "-":
                current = max(args.min_pw, current - args.step)
                set_pw(h, args.pin, current)
            elif ch == "0":
                current = args.center
                set_pw(h, args.pin, current)
            elif ch == "l":
                current = args.min_pw
                set_pw(h, args.pin, current)
            elif ch == "r":
                current = args.max_pw
                set_pw(h, args.pin, current)
            elif ch == "s":
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                sweep(h, args.pin, args.min_pw, args.max_pw, args.step)
                current = args.center
                set_pw(h, args.pin, current)
                tty.setraw(fd)
            elif ch in ("q", "Q", "\x03"):   # q or Ctrl+C
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print(f"\n\nReturning to center ({args.center} µs) and closing.")
        set_pw(h, args.pin, args.center)
        time.sleep(0.3)
        lgpio.tx_servo(h, args.pin, 0)   # disable PWM
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()
