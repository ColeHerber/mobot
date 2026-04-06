#!/usr/bin/env python3
"""Standalone mapping script — drives the robot at slow speed and records the path.

Usage:
    python3 mapper.py [--dry-run] [--config PATH] [--route PATH] [--speed 0.15]

Records every control frame to logs/map_YYYYMMDD_HHMMSS.json and, on exit,
writes config/mapped_route.yaml with all (x, y) waypoints.

Speed is capped at 0.15 m/s regardless of --speed argument to keep the robot
slow and safe during mapping passes.
"""

import argparse
import json
import logging
import math
import os
import random
import sys
import time
import datetime

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
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
log = logging.getLogger("mapper")

MAX_MAPPING_SPEED_MS = 0.15   # hard cap — never faster during mapping
LOOP_HZ = 100
LOOP_DT = 1.0 / LOOP_HZ


def load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def save_yaml(path: str, data) -> None:
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


class DryRunSensorState:
    """Simulates sensor readings with random walk for --dry-run mode."""

    def __init__(self):
        self._line_pos = 0.0
        self._heading  = 0.0

    def tick(self):
        # Small random walk on line position
        self._line_pos += random.gauss(0, 0.02)
        self._line_pos  = max(-0.8, min(0.8, self._line_pos))
        # Slow heading drift
        self._heading += random.gauss(0, 0.005)

    @property
    def line_pos(self) -> float:
        return self._line_pos

    @property
    def confidence(self) -> int:
        return 200

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def velocity_ms(self) -> float:
        return MAX_MAPPING_SPEED_MS


def run_mapper(args):
    # ── Resolve config paths ──────────────────────────────────────────────────
    script_dir  = os.path.dirname(os.path.abspath(__file__))
    config_root = os.path.join(script_dir, "..", "config")
    logs_root   = os.path.join(script_dir, "..", "logs")

    config_path = args.config or os.path.join(config_root, "params.yaml")
    route_path  = args.route  or os.path.join(config_root, "route.yaml")

    config = load_yaml(config_path)
    route  = load_yaml(route_path)

    # ── Override speed — never faster than MAX_MAPPING_SPEED_MS ──────────────
    requested_speed = min(float(args.speed), MAX_MAPPING_SPEED_MS)
    config.setdefault("speed", {})
    config["speed"]["base_ms"] = requested_speed
    config["speed"]["min_ms"]  = min(float(config["speed"].get("min_ms", 0.2)),
                                     requested_speed)
    log.info("Mapping speed capped to %.3f m/s", requested_speed)

    # ── Init shared state ─────────────────────────────────────────────────────
    state = SharedState()

    dry_run = args.dry_run
    sim     = DryRunSensorState() if dry_run else None

    # ── Start hardware threads (skipped in dry-run) ───────────────────────────
    if not dry_run:
        sensor_reader = SensorReader(state, config)
        sensor_reader.start()

        vesc = VESCInterface(state, config)
        vesc.start()
    else:
        log.info("DRY RUN — no hardware threads started")

    # ── Init synchronous components ───────────────────────────────────────────
    odo   = Odometry()
    pid   = PIDController(config)
    sm    = StateMachine(config, route, odo)
    servo = ServoControl(config)

    if dry_run:
        heading0 = 0.0
    else:
        heading0 = state.get("heading_rad")[0]
    odo.reset(heading0)

    # ── Prepare log output ────────────────────────────────────────────────────
    os.makedirs(logs_root, exist_ok=True)
    ts       = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(logs_root, f"map_{ts}.json")

    path_records = []
    start_time   = time.monotonic()

    log.info("Mapper started — press Ctrl+C to stop and save.")
    log.info("Output: %s", log_path)

    try:
        while True:
            now = time.monotonic()
            t   = now - start_time

            if dry_run:
                sim.tick()
                line_pos   = sim.line_pos
                confidence = sim.confidence
                heading    = sim.heading
                vel        = sim.velocity_ms
                # Integrate simulated odometry
                odo.update(vel, heading, LOOP_DT)
                state.update_odometry(odo.x, odo.y)
            else:
                (line_pos, confidence, heading, vel) = state.get(
                    "line_position", "sensor_confidence",
                    "heading_rad", "wheel_velocity_ms",
                )
                odo.update(vel, heading, LOOP_DT)
                state.update_odometry(odo.x, odo.y)

            # ── PID + state machine ───────────────────────────────────────────
            pid_steer, pid_throttle = pid.compute(line_pos, LOOP_DT)
            steering, throttle, sm_state = sm.update(
                confidence, pid_steer, pid_throttle, heading, LOOP_DT
            )

            # ── Actuators ─────────────────────────────────────────────────────
            if not dry_run:
                servo.set_steering(steering)
                vesc.set_throttle(throttle / config["speed"]["base_ms"])

            state.update_control(steering, throttle, sm_state)

            # ── Record frame ──────────────────────────────────────────────────
            path_records.append({
                "t":          round(t, 4),
                "x":          round(odo.x, 4),
                "y":          round(odo.y, 4),
                "heading":    round(heading, 4),
                "line_pos":   round(line_pos, 4),
                "confidence": confidence,
                "state":      sm_state,
            })

            # ── Rate limiter ──────────────────────────────────────────────────
            elapsed = time.monotonic() - now
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        log.info("Ctrl+C — stopping.")

    finally:
        # ── Stop actuators ────────────────────────────────────────────────────
        if not dry_run:
            try:
                vesc.set_throttle(0.0)
                time.sleep(0.1)
                servo.stop()
            except Exception as e:
                log.warning("Shutdown actuators failed: %s", e)

        state.set(running=False)

        # ── Save JSON log ─────────────────────────────────────────────────────
        total_points = len(path_records)
        output = {
            "metadata": {
                "date":         datetime.datetime.now().isoformat(),
                "speed_ms":     requested_speed,
                "total_points": total_points,
            },
            "path": path_records,
        }
        with open(log_path, "w") as f:
            json.dump(output, f, indent=2)
        log.info("Saved %d points to %s", total_points, log_path)

        # ── Save mapped_route.yaml ────────────────────────────────────────────
        mapped_route_path = os.path.join(config_root, "mapped_route.yaml")
        waypoints = [{"x": r["x"], "y": r["y"]} for r in path_records]
        save_yaml(mapped_route_path, {"waypoints": waypoints})
        log.info("Saved mapped route (%d waypoints) to %s",
                 len(waypoints), mapped_route_path)


def main():
    parser = argparse.ArgumentParser(description="Mobot mapping run — records path at slow speed")
    parser.add_argument("--dry-run", action="store_true",
                        help="No actuator output — simulate with random sensor data")
    parser.add_argument("--config", default=None,
                        help="Path to params.yaml")
    parser.add_argument("--route", default=None,
                        help="Path to route.yaml")
    parser.add_argument("--speed", type=float, default=MAX_MAPPING_SPEED_MS,
                        help=f"Target speed in m/s (capped at {MAX_MAPPING_SPEED_MS})")
    args = parser.parse_args()

    run_mapper(args)


if __name__ == "__main__":
    main()
