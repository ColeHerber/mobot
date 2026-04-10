#!/usr/bin/env python3
"""Mobot main entry point.

Usage:
    python3 main.py [--dry-run] [--config PATH] [--route PATH] [--no-web]

    --dry-run   Run without actuator output (sensor display only). Useful
                for bench testing with only Teensy + BNO085 connected.
    --config    Path to params.yaml (default: ../config/params.yaml)
    --route     Path to route.yaml  (default: ../config/route.yaml)
    --no-web    Disable the Flask web server (default: enabled)

Main loop: ~100 Hz
  1. Read shared state (set by sensor_reader + vesc_interface threads)
  2. Update odometry
  3. Run PID controller
  4. Run state machine
  5. Command servo + VESC
  6. Log to CSV (50 Hz decimated)
  7. Update curses display
  8. Check for params hot-reload (from web server)
"""

import argparse
import csv
import curses
import logging
import logging.handlers
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
from web_server     import WebServer

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)

# Also log to a rotating file so errors survive curses taking over the terminal
_LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "logs")
os.makedirs(_LOG_DIR, exist_ok=True)
_fh = logging.handlers.RotatingFileHandler(
    os.path.join(_LOG_DIR, "mobot.log"),
    maxBytes=5 * 1024 * 1024,  # 5 MB per file
    backupCount=3,
)
_fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
logging.getLogger().addHandler(_fh)

log = logging.getLogger("main")

LOOP_HZ        = 100
LOOP_DT        = 1.0 / LOOP_HZ
LOG_DECIMATION = 2   # log every Nth loop iteration → 50 Hz
_TELEOP_MAX_MS = 0.1   # max teleop speed (m/s) — keep slow for safety


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

    (lp, conf, heading, pitch, roll, x, y, sm_state,
     steering, throttle, vel, voltage, flags) = state.get(
        "line_position", "sensor_confidence", "heading_rad",
        "pitch_rad", "roll_rad",
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
    pitch_deg   = math.degrees(pitch)
    roll_deg    = math.degrees(roll)
    safe_addstr(row, 0, f"Heading {heading_deg:+7.1f}°   X={x:+7.3f}m  Y={y:+7.3f}m")
    row += 1
    safe_addstr(row, 0, f"Pitch  {pitch_deg:+7.1f}°   Roll  {roll_deg:+7.1f}°")
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


def run(stdscr, args, config, route, config_path, route_path, web_server):
    curses.curs_set(0)
    stdscr.nodelay(True)

    dry_run = args.dry_run

    # ── Init shared state ─────────────────────────────────────────────────────
    state = SharedState()
    state.reset_odometry = False   # web API hook

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

    # Hand servo reference to web server (enables live nudge)
    if web_server is not None:
        web_server.set_servo(servo)

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
    prev_sm_state = state.get("state")[0]

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

            # ── Params hot-reload ─────────────────────────────────────────────
            if web_server is not None and web_server.get_reload_flag():
                try:
                    config = load_yaml(config_path)
                    pid    = PIDController(config)
                    sm     = StateMachine(config, route, odo)
                    servo  = ServoControl(config)
                    web_server.set_servo(servo)
                    log.info("params.yaml hot-reloaded")
                except Exception as e:
                    log.error("Hot-reload failed: %s", e)

            # ── Odometry reset (from web API) ─────────────────────────────────
            if getattr(state, "reset_odometry", False):
                state.reset_odometry = False
                heading_now = state.get("heading_rad")[0]
                odo.reset(heading_now)
                log.info("Odometry reset via web API")

            # ── Read shared state snapshot ────────────────────────────────────
            (line_pos, confidence, heading, pitch, vel, voltage) = state.get(
                "line_position", "sensor_confidence",
                "heading_rad", "pitch_rad",
                "wheel_velocity_ms", "input_voltage",
            )

            # ── Odometry ──────────────────────────────────────────────────────
            odo.update(vel, heading, dt)
            state.update_odometry(odo.x, odo.y)

            # ── PID ───────────────────────────────────────────────────────────
            pid_steer, pid_throttle = pid.compute(line_pos, dt)

            # ── State machine ─────────────────────────────────────────────────
            steering, throttle, sm_state = sm.update(
                confidence, pid_steer, pid_throttle, heading, pitch, dt
            )

            # ── Log state transitions ─────────────────────────────────────────
            if sm_state != prev_sm_state and web_server is not None:
                web_server.log_transition(prev_sm_state, sm_state,
                                          f"conf={confidence}")
                prev_sm_state = sm_state

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
    parser.add_argument("--no-web", action="store_true",
                        help="Disable the Flask web server")
    parser.add_argument("--teleop-only", action="store_true",
                        help="Teleop mode: VESC + servo + web only, no PID/sensors")
    args = parser.parse_args()

    base = os.path.join(os.path.dirname(__file__), "..", "config")
    config_path = args.config or os.path.join(base, "params.yaml")
    route_path  = args.route  or os.path.join(base, "route.yaml")

    config = load_yaml(config_path)
    route  = load_yaml(route_path)

    # ── Web server ────────────────────────────────────────────────────────────
    web_server = None
    if not args.no_web:
        try:
            static_dir = os.path.join(os.path.dirname(__file__), "..", "static")
            web_server = WebServer(
                shared_state=None,   # will be set inside run() after state init
                config_path=config_path,
                route_path=route_path,
                static_dir=static_dir,
            )
            # We need a real SharedState before start(); defer start to run()
        except Exception as e:
            log.warning("WebServer init failed (continuing without web): %s", e)
            web_server = None

    if args.teleop_only:
        curses.wrapper(_run_with_web, args, config, route,
                       config_path, route_path, web_server, teleop_only=True)
    else:
        curses.wrapper(_run_with_web, args, config, route,
                       config_path, route_path, web_server)


def _run_with_web(stdscr, args, config, route, config_path, route_path,
                  web_server, teleop_only=False):
    """Thin wrapper that wires SharedState into WebServer before starting."""
    import curses as _curses
    _curses.curs_set(0)
    stdscr.nodelay(True)

    # Build real SharedState first
    state = SharedState()
    state.reset_odometry = False

    # Wire state into web server and start it
    if web_server is not None:
        web_server._state = state
        try:
            web_server.start()
        except Exception as e:
            log.warning("WebServer start failed: %s", e)
            web_server = None

    if teleop_only:
        _teleop_loop(stdscr, args, config, web_server, state)
    else:
        _run_loop(stdscr, args, config, route, config_path, route_path,
                  web_server, state)


def _teleop_loop(stdscr, args, config, web_server, state):
    """Minimal loop: VESC + servo + teleop only. No sensors, no PID."""
    vesc  = VESCInterface(state, config)
    vesc.start()
    servo = ServoControl(config)
    if web_server is not None:
        web_server.set_servo(servo)

    loop_count = 0
    last_time  = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            dt  = now - last_time
            last_time = now

            robot_en, teleop_en, tele_steer, tele_norm, tele_t = state.get(
                "robot_enabled", "teleop_enabled", "teleop_steering",
                "teleop_throttle", "teleop_last_cmd",
            )

            steering = 0.0
            throttle = 0.0

            duty = 0.0
            if robot_en and teleop_en:
                if tele_t <= 0.0 or (time.monotonic() - tele_t) > 0.3:
                    tele_norm = 0.0
                steering = max(-1.0, min(1.0, tele_steer))
                throttle = tele_norm * _TELEOP_MAX_MS
                duty = throttle / config["speed"]["base_ms"]
                servo.set_steering(steering)
                vesc.set_throttle(duty)
            else:
                vesc.set_throttle(0)

            if loop_count % 20 == 0:
                log.info("CMD en=%s/%s steer=%.3f throttle=%.3f m/s duty=%.3f",
                         robot_en, teleop_en, steering, throttle, duty)

            state.update_control(steering, throttle, "TELEOP" if robot_en else "IDLE")

            if loop_count % 5 == 0:
                draw_display(stdscr, state, 1.0 / max(dt, 1e-6), False)

            loop_count += 1
            ch = stdscr.getch()
            if ch in (ord('q'), 3):
                break

            elapsed = time.monotonic() - now
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        log.info("Teleop shutdown")
        state.set(running=False)
        vesc.set_throttle(0)
        time.sleep(0.1)
        servo.stop()


def _run_loop(stdscr, args, config, route, config_path, route_path,
              web_server, state):
    """Core 100 Hz control loop (separated so state is constructed once)."""
    dry_run = args.dry_run

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

    if web_server is not None:
        web_server.set_servo(servo)

    heading0 = state.get("heading_rad")[0]
    odo.reset(heading0)

    # ── CSV log ───────────────────────────────────────────────────────────────
    logs_dir = os.path.join(os.path.dirname(__file__), "..", "logs")
    csv_writer, csv_file = open_csv_log(logs_dir)

    # ── Main loop ─────────────────────────────────────────────────────────────
    loop_count    = 0
    log_counter   = 0
    last_time     = time.monotonic()
    loop_hz       = 0.0
    hz_window     = []
    prev_sm_state = state.get("state")[0]

    try:
        while True:
            now = time.monotonic()
            dt  = now - last_time
            last_time = now

            if dt > 0:
                hz_window.append(1.0 / dt)
                if len(hz_window) > 50:
                    hz_window.pop(0)
                loop_hz = sum(hz_window) / len(hz_window)

            # ── Params hot-reload ─────────────────────────────────────────────
            if web_server is not None and web_server.get_reload_flag():
                try:
                    config = load_yaml(config_path)
                    pid    = PIDController(config)
                    sm     = StateMachine(config, route, odo)
                    servo  = ServoControl(config)
                    web_server.set_servo(servo)
                    log.info("params.yaml hot-reloaded")
                except Exception as e:
                    log.error("Hot-reload failed: %s", e)

            # ── Odometry reset ────────────────────────────────────────────────
            if getattr(state, "reset_odometry", False):
                state.reset_odometry = False
                heading_now = state.get("heading_rad")[0]
                odo.reset(heading_now)
                log.info("Odometry reset via web API")

            # ── Read shared state snapshot ────────────────────────────────────
            (line_pos, confidence, heading, pitch, vel, voltage) = state.get(
                "line_position", "sensor_confidence",
                "heading_rad", "pitch_rad",
                "wheel_velocity_ms", "input_voltage",
            )

            # ── Odometry ──────────────────────────────────────────────────────
            odo.update(vel, heading, dt)
            state.update_odometry(odo.x, odo.y)

            # ── PID ───────────────────────────────────────────────────────────
            pid_steer, pid_throttle = pid.compute(line_pos, dt)

            # ── State machine ─────────────────────────────────────────────────
            steering, throttle, sm_state = sm.update(
                confidence, pid_steer, pid_throttle, heading, pitch, dt
            )

            # ── Log state transitions ─────────────────────────────────────────
            if sm_state != prev_sm_state and web_server is not None:
                web_server.log_transition(prev_sm_state, sm_state,
                                          f"conf={confidence}")
                prev_sm_state = sm_state

            # ── Teleop override ───────────────────────────────────────────────
            teleop_en, tele_steer, tele_norm, tele_t = state.get(
                "teleop_enabled", "teleop_steering",
                "teleop_throttle", "teleop_last_cmd",
            )
            if teleop_en:
                # Watchdog: kill throttle if no command within 300 ms
                if tele_t <= 0.0 or (time.monotonic() - tele_t) > 0.3:
                    tele_norm = 0.0
                steering = max(-1.0, min(1.0, tele_steer))
                throttle = tele_norm * _TELEOP_MAX_MS
                sm_state = "TELEOP"

            # ── Actuators ─────────────────────────────────────────────────────
            robot_en = state.get("robot_enabled")[0]
            duty = throttle / config["speed"]["base_ms"]
            if not dry_run and robot_en:
                servo.set_steering(steering)
                vesc.set_throttle(duty)
            else:
                if not robot_en:
                    vesc.set_throttle(0)
                steering = 0.0 if not robot_en else steering
                throttle = 0.0 if not robot_en else throttle
                duty     = 0.0 if not robot_en else duty

            # 5 Hz summary log of what's actually being commanded
            if loop_count % 20 == 0:
                log.info("CMD en=%s steer=%.3f throttle=%.3f m/s duty=%.3f  [%s]",
                         robot_en, steering, throttle, duty, sm_state)

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
            if loop_count % 5 == 0:
                draw_display(stdscr, state, loop_hz, dry_run)

            loop_count += 1

            ch = stdscr.getch()
            if ch == ord('q') or ch == 3:
                break

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


if __name__ == "__main__":
    main()
