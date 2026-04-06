#!/usr/bin/env python3
"""Interactive terminal servo center tuner.

Finds the exact center pulse width for the steering servo by letting you
nudge center_offset_us in real time and watching the robot's wheels.

Controls:
  a / ←   → center_offset_us -= 5 µs
  d / →   → center_offset_us += 5 µs
  A / Z   → center_offset_us -= 1 µs  (fine)
  D / X   → center_offset_us += 1 µs  (fine)
  r       → reset offset to 0
  s       → save center_offset_us to params.yaml
  q / ESC → quit without saving

Usage:
    python3 pi/tune_servo_center.py [--config ../config/params.yaml]
"""

import argparse
import curses
import os
import sys
import time

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

log_msgs = []


def load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


def save_yaml(path: str, data: dict) -> None:
    with open(path + ".tmp", "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    os.replace(path + ".tmp", path)


def run(stdscr, config_path: str):
    curses.curs_set(0)
    stdscr.nodelay(True)

    config = load_yaml(config_path)
    servo_cfg = config.setdefault("servo", {})

    center_pw  = int(servo_cfg.get("center_pw",        1500))
    min_pw     = int(servo_cfg.get("min_pw",           1000))
    max_pw     = int(servo_cfg.get("max_pw",           2000))
    offset_us  = int(servo_cfg.get("center_offset_us",    0))

    # Try to initialise servo hardware
    servo = None
    try:
        from servo_control import ServoControl
        servo = ServoControl(config)
    except Exception as e:
        log_msgs.append(f"Servo init failed: {e} — display-only mode")

    saved_offset = offset_us

    def effective_center():
        return center_pw + offset_us

    def apply_servo():
        if servo is not None:
            servo._effective_center = effective_center()
            servo.set_steering(0.0)

    apply_servo()

    while True:
        h, w = stdscr.getmaxyx()
        stdscr.erase()

        def put(row, col, txt, attr=0):
            try:
                stdscr.addstr(row, col, str(txt)[:w - col - 1], attr)
            except curses.error:
                pass

        r = 0
        put(r, 0, "─" * min(w - 1, 62))
        r += 1
        put(r, 0, "  MOBOT SERVO CENTER TUNER", curses.A_BOLD)
        r += 1
        put(r, 0, "─" * min(w - 1, 62))
        r += 2

        ec = effective_center()
        put(r, 2, f"center_pw     : {center_pw:5d} µs  (baseline — do not change)")
        r += 1
        put(r, 2, f"center_offset : {offset_us:+5d} µs  {'[UNSAVED]' if offset_us != saved_offset else '[saved]'}")
        r += 1
        put(r, 2, f"Effective ctr : {ec:5d} µs",      curses.A_BOLD)
        r += 2

        # Bar visualisation
        bar_w = min(w - 6, 50)
        frac  = (ec - min_pw) / max(1, max_pw - min_pw)
        filled = int(frac * bar_w)
        bar = "█" * filled + "░" * (bar_w - filled)
        put(r, 2, f"{min_pw}µs [{bar}] {max_pw}µs")
        r += 2

        put(r, 2, "Controls:", curses.A_UNDERLINE)
        r += 1
        put(r, 4, "a / ←      -5 µs (coarse)")
        r += 1
        put(r, 4, "d / →      +5 µs (coarse)")
        r += 1
        put(r, 4, "A / Z      -1 µs (fine)")
        r += 1
        put(r, 4, "D / X      +1 µs (fine)")
        r += 1
        put(r, 4, "r          reset to 0")
        r += 1
        put(r, 4, "s          SAVE to params.yaml")
        r += 1
        put(r, 4, "q / ESC    quit")
        r += 2

        for msg in log_msgs[-4:]:
            put(r, 2, msg, curses.A_DIM)
            r += 1

        stdscr.refresh()

        key = stdscr.getch()
        if key == -1:
            time.sleep(0.05)
            continue

        if key in (ord('a'), curses.KEY_LEFT):
            offset_us -= 5
        elif key in (ord('d'), curses.KEY_RIGHT):
            offset_us += 5
        elif key in (ord('A'), ord('z'), ord('Z')):
            offset_us -= 1
        elif key in (ord('D'), ord('x'), ord('X')):
            offset_us += 1
        elif key == ord('r'):
            offset_us = 0
            log_msgs.append("Offset reset to 0.")
        elif key == ord('s'):
            config["servo"]["center_offset_us"] = offset_us
            try:
                save_yaml(config_path, config)
                saved_offset = offset_us
                log_msgs.append(f"Saved center_offset_us = {offset_us} µs to {os.path.basename(config_path)}")
            except Exception as e:
                log_msgs.append(f"Save failed: {e}")
        elif key in (ord('q'), 27):   # q or ESC
            break

        # Clamp offset so effective center stays in [min_pw, max_pw]
        offset_us = max(min_pw - center_pw, min(max_pw - center_pw, offset_us))
        apply_servo()

    if servo is not None:
        servo.stop()


def main():
    parser = argparse.ArgumentParser(description="Interactive servo center tuner")
    parser.add_argument("--config", default=None,
                        help="Path to params.yaml")
    args = parser.parse_args()

    script_dir  = os.path.dirname(os.path.abspath(__file__))
    config_path = args.config or os.path.join(script_dir, "..", "config", "params.yaml")
    config_path = os.path.abspath(config_path)

    curses.wrapper(run, config_path)


if __name__ == "__main__":
    main()
