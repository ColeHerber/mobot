#!/usr/bin/env python3
"""Mobot main entry point.

Usage:
    python3 main.py [--dry-run] [--config PATH] [--route PATH]

    --dry-run   Run without actuator output (sensor display only). Useful
                for bench testing with only Teensy + BNO085 connected.
    --config    Path to params.yaml (default: ../config/params.yaml)
    --route     Path to route.yaml  (default: ../config/route.yaml)

Main loop: ~100 Hz
  1. Read shared state (set by sensor_reader + vesc_interface threads)
  2. Update odometry
  3. Run PID controller
  4. Run state machine
  5. Command servo + VESC
  6. Log to CSV (50 Hz decimated)
  7. Update curses display
"""

import argparse
import csv
import curses
import logging
import os
import sys
import time
import datetime

import yaml

# ── Local imports (same directory) ────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(__file__))
from shared_state   import SharedState
from sensor_reader  import SensorReader
from vesc_interface import VESCInterface
from odometry       import Odometry
from pid_controller import PIDController
from state_machine  import StateMachine
from servo_control  import ServoControl

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
log = logging.getLogger("main")

LOOP_HZ       = 100
LOOP_DT       = 1.0 / LOOP_HZ
LOG_DECIMATION = 2   # log every Nth loop iteration → 50 Hz


def load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def open_csv_log(logs_dir: str) -> tuple[csv.writer, object]:
    os.makedirs(logs_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(logs_dir, f"run_{ts}.csv")
    f = open(path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow([
        "time_s", "line_pos", "confidence", "heading_rad",
        "x_m", "y_m", "state", "steering", "throttle",
        "wheel_vel_ms", "input_voltage",
    ])
    log.info("Logging to %s", path)
    return writer, f


def draw_display(stdscr, state: SharedState, loop_hz: float, dry_run: bool):
    """Render the curses dashboard."""
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    (lp, conf, heading, x, y, sm_state,
     steering, throttle, vel, voltage, flags) = state.get(
        "line_position", "sensor_confidence", "heading_rad",
        "x", "y", "state", "steering", "throttle",
        "wheel_velocity_ms", "input_voltage", "sensor_flags",
    )

    def safe_addstr(row, col, text, attr=0):
        try:
            stdscr.addstr(row, col, text[:w - col - 1], attr)
        except curses.error:
            pass

    row = 0
    title = "CMU MOBOT 2026" + (" [DRY RUN]" if dry_run else "")
    safe_addstr(row, 0, title.center(min(w, 60)), curses.A_BOLD)
    row += 1
    safe_addstr(row, 0, "─" * min(w - 1, 60))
    row += 1

    # Sensor bar (16 channels as block chars)
    bar = ""
    for i in range(16):
        bar += "█" if (flags >> i) & 1 else "░"
    safe_addstr(row, 0, f"Sensor  [{bar}]  pos={lp:+.3f}  conf={conf:3d}")
    row += 1

    # Heading + odometry
    import math
    heading_deg = math.degrees(heading)
    safe_addstr(row, 0, f"Heading {heading_deg:+7.1f}°   X={x:+7.3f}m  Y={y:+7.3f}m")
    row += 1

    # State
    safe_addstr(row, 0, f"State   {sm_state:<16}  loop={loop_hz:5.1f} Hz")
    row += 1

    # Control outputs
    steer_bar = int((steering + 1.0) / 2.0 * 20)
    steer_vis = "[" + "=" * steer_bar + " " * (20 - steer_bar) + "]"
    safe_addstr(row, 0, f"Steer   {steer_vis}  {steering:+.3f}")
    row += 1
    safe_addstr(row, 0, f"Throttle {throttle:+.3f}  vel={vel:+.3f} m/s  Vbat={voltage:.1f}V")
    row += 1

    row += 1
    safe_addstr(row, 0, "Ctrl+C to stop", curses.A_DIM)

    stdscr.refresh()


def run(stdscr, args, config, route):
    curses.curs_set(0)
    stdscr.nodelay(True)

    dry_run = args.dry_run

    # ── Init shared state ─────────────────────────────────────────────────────
    state = SharedState()

    # ── Start background threads ──────────────────────────────────────────────
    sensor_reader = SensorReader(state, config)
    sensor_reader.start()

    vesc = VESCInterface(state, config)
    vesc.start()

    # ── Init synchronous components ───────────────────────────────────────────
    odo    = Odometry()
    pid    = PIDController(config)
    sm     = StateMachine(config, route, odo)
    servo  = ServoControl(config)

    # Reset odometry at start line
    heading0 = state.get("heading_rad")[0]
    odo.reset(heading0)

    # ── CSV log ───────────────────────────────────────────────────────────────
    logs_dir = os.path.join(os.path.dirname(__file__), "..", "logs")
    csv_writer, csv_file = open_csv_log(logs_dir)

    # ── Main loop ─────────────────────────────────────────────────────────────
    loop_count  = 0
    log_counter = 0
    last_time   = time.monotonic()
    loop_hz     = 0.0
    hz_window   = []

    try:
        while True:
            now = time.monotonic()
            dt  = now - last_time
            last_time = now

            # Track loop rate (rolling 50-sample window)
            if dt > 0:
                hz_window.append(1.0 / dt)
                if len(hz_window) > 50:
                    hz_window.pop(0)
                loop_hz = sum(hz_window) / len(hz_window)

            # ── Read shared state snapshot ────────────────────────────────────
            (line_pos, confidence, heading, vel, voltage) = state.get(
                "line_position", "sensor_confidence",
                "heading_rad", "wheel_velocity_ms", "input_voltage",
            )

            # ── Odometry ──────────────────────────────────────────────────────
            odo.update(vel, heading, dt)
            state.update_odometry(odo.x, odo.y)

            # ── PID ───────────────────────────────────────────────────────────
            pid_steer, pid_throttle = pid.compute(line_pos, dt)

            # ── State machine ─────────────────────────────────────────────────
            steering, throttle, sm_state = sm.update(
                confidence, pid_steer, pid_throttle, heading, dt
            )

            # ── Actuators ─────────────────────────────────────────────────────
            if not dry_run:
                servo.set_steering(steering)
                vesc.set_throttle(throttle / config["speed"]["base_ms"])
            else:
                steering  = 0.0
                throttle  = 0.0

            state.update_control(steering, throttle, sm_state)

            # ── CSV log (decimated) ───────────────────────────────────────────
            log_counter += 1
            if log_counter >= LOG_DECIMATION:
                log_counter = 0
                csv_writer.writerow([
                    f"{now:.4f}", f"{line_pos:.4f}", confidence,
                    f"{heading:.4f}", f"{odo.x:.4f}", f"{odo.y:.4f}",
                    sm_state, f"{steering:.4f}", f"{throttle:.4f}",
                    f"{vel:.4f}", f"{voltage:.2f}",
                ])

            # ── Curses display ────────────────────────────────────────────────
            if loop_count % 5 == 0:  # ~20 Hz display update
                draw_display(stdscr, state, loop_hz, dry_run)

            loop_count += 1

            # ── Check for keyboard quit ───────────────────────────────────────
            ch = stdscr.getch()
            if ch == ord('q') or ch == 3:  # q or Ctrl+C
                break

            # ── Rate limiter ──────────────────────────────────────────────────
            elapsed = time.monotonic() - now
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        log.info("Shutting down…")
        state.set(running=False)
        if not dry_run:
            vesc.set_throttle(0.0)
            time.sleep(0.1)
            servo.stop()
        csv_file.flush()
        csv_file.close()
        log.info("Clean shutdown complete.")


def main():
    parser = argparse.ArgumentParser(description="CMU Mobot controller")
    parser.add_argument("--dry-run", action="store_true",
                        help="No actuator output — sensor display only")
    parser.add_argument("--config", default=None,
                        help="Path to params.yaml")
    parser.add_argument("--route", default=None,
                        help="Path to route.yaml")
    args = parser.parse_args()

    base = os.path.join(os.path.dirname(__file__), "..", "config")
    config_path = args.config or os.path.join(base, "params.yaml")
    route_path  = args.route  or os.path.join(base, "route.yaml")

    config = load_yaml(config_path)
    route  = load_yaml(route_path)

    curses.wrapper(run, args, config, route)


if __name__ == "__main__":
    main()
