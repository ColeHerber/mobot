#!/usr/bin/env python3
"""odo_cal.py — Interactive odometry calibration tool.

Drives the robot through controlled maneuvers, then lets you measure the
actual distance with a tape measure and computes the wheel circumference
correction needed to fix odometry accuracy.

Tests:
  1. Straight run — drive until odo says N meters, stop, measure actual
  2. Trapezoidal profile — distance-keyed ramp up / cruise / ramp down
  3. RPM / velocity spot-check — hold a duty, watch live telemetry
  4. Change cruise duty

Usage:
    python3 odo_cal.py [--config PATH] [--dry-run]
    bash scripts/run_odo_cal.sh [--dry-run]
"""

import argparse
import logging
import math
import os
import sys
import time

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from odometry       import Odometry
from servo_control  import ServoControl
from shared_state   import SharedState
from vesc_interface import VESCInterface

logging.basicConfig(
    level=logging.WARNING,   # suppress routine INFO from hardware threads
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
log = logging.getLogger("odo_cal")

# ── Constants ─────────────────────────────────────────────────────────────────

LOOP_HZ      = 50
LOOP_DT      = 1.0 / LOOP_HZ
DEFAULT_DUTY = 0.12    # ~0.10 m/s on a healthy pack; change via menu option 6
RAMP_TIME_S  = 1.0     # seconds to ramp duty 0 → cruise (straight runs)
SAFETY_DIST_M = 8.0    # hard abort if odo distance exceeds this


# ── Config ────────────────────────────────────────────────────────────────────

def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


# ── UI helpers ────────────────────────────────────────────────────────────────

def _confirm(prompt: str = "Press ENTER to start (Ctrl+C to abort)") -> bool:
    try:
        input(f"\n  {prompt}... ")
        return True
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def _live(elapsed: float, vel: float, rpm: float, dist: float, phase: str) -> None:
    print(
        f"\r  t={elapsed:6.2f}s  vel={vel:+.4f} m/s  rpm={rpm:+7.0f}"
        f"  odo={dist:.4f} m  [{phase:<12s}]   ",
        end="", flush=True,
    )


def _calibration_report(cfg_vesc: dict, odo_dist: float) -> None:
    """Ask for tape-measure reading and print correction."""
    print()
    try:
        raw = input("  Tape-measure actual distance (m), or ENTER to skip: ").strip()
        if not raw:
            return
        actual = float(raw)
    except (ValueError, KeyboardInterrupt):
        print("  [skipped]")
        return

    if actual <= 0 or odo_dist <= 0:
        print("  [skipped — zero or negative value]")
        return

    error_m        = odo_dist - actual
    error_pct      = error_m / actual * 100.0
    current_circ   = float(cfg_vesc.get("wheel_circumference_m", 0.204))
    suggested_circ = current_circ * (actual / odo_dist)

    print()
    print("  Calibration result")
    print("  " + "-" * 46)
    print(f"  Odo reading      : {odo_dist:.4f} m")
    print(f"  Actual measured  : {actual:.4f} m")
    print(f"  Error            : {error_m * 100:+.1f} cm  ({error_pct:+.2f}%)")
    print(f"  Current  wheel_circumference_m : {current_circ:.5f}")
    print(f"  Suggested wheel_circumference_m: {suggested_circ:.5f}")
    if abs(error_pct) > 0.5:
        print()
        print("  --> Update config/params.yaml:")
        print(f"        vesc:")
        print(f"          wheel_circumference_m: {suggested_circ:.5f}")
    else:
        print("  --> Odometry within 0.5% — no correction needed.")
    print("  " + "-" * 46)


# ── Motor helpers ─────────────────────────────────────────────────────────────

def _stop_motor(vesc: VESCInterface, dry_run: bool) -> None:
    if not dry_run:
        vesc.set_throttle(0.0)
    time.sleep(0.3)


# ── Motion primitives ─────────────────────────────────────────────────────────

def run_straight(
    state: SharedState,
    vesc: VESCInterface,
    odo: Odometry,
    servo: ServoControl,
    target_m: float,
    cruise_duty: float,
    dry_run: bool,
) -> float:
    """
    Straight-line run to target odometry distance.

    - Time-based ramp up (RAMP_TIME_S) → cruise → time-based ramp down
      triggered when odo approaches target.
    - Stops when odo distance >= target_m.
    - Returns final odometry distance.
    """
    decel_margin  = cruise_duty * RAMP_TIME_S * 0.5   # rough distance during decel
    decel_trigger = max(target_m * 0.5, target_m - decel_margin)

    heading0 = state.get("heading_rad")[0]
    odo.reset(heading0)
    servo.set_steering(0.0)

    phase       = "ramp_up"
    phase_start = time.monotonic()
    t_start     = time.monotonic()
    duty        = 0.0

    try:
        while True:
            t0      = time.monotonic()
            elapsed = t0 - t_start
            ph_el   = t0 - phase_start

            heading, vel, rpm = state.get("heading_rad", "wheel_velocity_ms", "motor_rpm")
            odo.update(vel, heading, LOOP_DT)
            dist = odo.distance_from_origin()

            if dist >= SAFETY_DIST_M:
                log.warning("Safety distance exceeded — stopping!")
                break

            if phase == "ramp_up":
                duty = min(ph_el / RAMP_TIME_S, 1.0) * cruise_duty
                if ph_el >= RAMP_TIME_S:
                    phase = "cruise"
                    phase_start = t0

            elif phase == "cruise":
                duty = cruise_duty
                if dist >= decel_trigger:
                    phase = "ramp_down"
                    phase_start = t0

            elif phase == "ramp_down":
                duty = max(1.0 - ph_el / RAMP_TIME_S, 0.0) * cruise_duty
                if ph_el >= RAMP_TIME_S or dist >= target_m:
                    break

            if not dry_run:
                vesc.set_throttle(duty)

            _live(elapsed, vel, rpm, dist, phase)

            sleep_t = LOOP_DT - (time.monotonic() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n  [interrupted]")

    finally:
        _stop_motor(vesc, dry_run)

    return odo.distance_from_origin()


def run_trapezoid(
    state: SharedState,
    vesc: VESCInterface,
    odo: Odometry,
    servo: ServoControl,
    target_m: float,
    cruise_duty: float,
    ramp_dist_m: float,
    dry_run: bool,
) -> dict:
    """
    Trapezoidal velocity profile keyed on odometry distance (not time).

    ramp_dist_m — distance over which duty linearly ramps up (and down).
    Cruise phase spans [ramp_dist_m, target_m - ramp_dist_m].
    Returns a dict with odo_dist, total_time, avg_vel, peak_vel.
    """
    if ramp_dist_m * 2 >= target_m:
        print(f"  Warning: ramp ({ramp_dist_m:.2f} m) >= half of target "
              f"({target_m / 2:.2f} m) — clamping ramp to 30% of target.")
        ramp_dist_m = target_m * 0.3

    decel_at = target_m - ramp_dist_m

    heading0 = state.get("heading_rad")[0]
    odo.reset(heading0)
    servo.set_steering(0.0)

    t_start     = time.monotonic()
    phase       = "ramp_up"
    vel_samples = []
    duty        = 0.0

    try:
        while True:
            t0      = time.monotonic()
            elapsed = t0 - t_start

            heading, vel, rpm = state.get("heading_rad", "wheel_velocity_ms", "motor_rpm")
            odo.update(vel, heading, LOOP_DT)
            dist = odo.distance_from_origin()
            vel_samples.append(vel)

            if dist >= SAFETY_DIST_M:
                log.warning("Safety abort!")
                break

            if phase == "ramp_up":
                duty = min(dist / ramp_dist_m, 1.0) * cruise_duty
                if dist >= ramp_dist_m:
                    phase = "cruise"

            elif phase == "cruise":
                duty = cruise_duty
                if dist >= decel_at:
                    phase = "ramp_down"

            elif phase == "ramp_down":
                remaining = target_m - dist
                duty = max(remaining / ramp_dist_m, 0.0) * cruise_duty
                if dist >= target_m:
                    break

            if not dry_run:
                vesc.set_throttle(duty)

            _live(elapsed, vel, rpm, dist, phase)

            sleep_t = LOOP_DT - (time.monotonic() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n  [interrupted]")

    finally:
        total_time = time.monotonic() - t_start
        _stop_motor(vesc, dry_run)

    avg_vel  = sum(vel_samples) / len(vel_samples) if vel_samples else 0.0
    peak_vel = max((abs(v) for v in vel_samples), default=0.0)

    return {
        "odo_dist":   odo.distance_from_origin(),
        "total_time": total_time,
        "avg_vel":    avg_vel,
        "peak_vel":   peak_vel,
    }


def run_spot_check(
    state: SharedState,
    vesc: VESCInterface,
    duty: float,
    duration: float,
    dry_run: bool,
) -> None:
    """Hold a fixed duty cycle and print a live telemetry table."""
    print(f"\n  Holding duty={duty:.3f} for {duration:.1f} s")
    print(f"  {'t (s)':>7}  {'RPM':>8}  {'vel (m/s)':>10}  {'voltage':>9}")
    print("  " + "-" * 44)

    t_start = time.monotonic()
    try:
        while True:
            t0      = time.monotonic()
            elapsed = t0 - t_start
            if elapsed >= duration:
                break

            vel, rpm, voltage = state.get("wheel_velocity_ms", "motor_rpm", "input_voltage")

            print(
                f"\r  {elapsed:7.2f}  {rpm:8.0f}  {vel:10.4f}  {voltage:7.2f} V   ",
                end="", flush=True,
            )

            if not dry_run:
                vesc.set_throttle(duty)

            sleep_t = LOOP_DT - (time.monotonic() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n  [interrupted]")

    finally:
        _stop_motor(vesc, dry_run)

    print()


# ── Main menu ─────────────────────────────────────────────────────────────────

def menu(
    state: SharedState,
    vesc: VESCInterface,
    odo: Odometry,
    servo: ServoControl,
    config: dict,
    dry_run: bool,
) -> None:
    cfg_vesc    = config.get("vesc", {})
    cruise_duty = DEFAULT_DUTY
    tag         = "DRY RUN" if dry_run else "LIVE"

    while True:
        circ  = cfg_vesc.get("wheel_circumference_m", 0.204)
        gratio = cfg_vesc.get("gear_ratio", 8.0)
        print(f"""
+--------------------------------------------------+
|    ODOMETRY CALIBRATION  [{tag:^8s}]           |
|  wheel_circ={circ:.4f} m   gear_ratio={gratio:.1f}        |
|  cruise_duty={cruise_duty:.3f}                               |
+--------------------------------------------------+
|  1. Straight run — 1 m                           |
|  2. Straight run — 2 m                           |
|  3. Straight run — custom distance               |
|  4. Trapezoidal profile (distance-keyed)         |
|  5. RPM / velocity spot-check                    |
|  6. Change cruise duty (current: {cruise_duty:.3f})        |
|  q. Quit                                         |
+--------------------------------------------------+""")

        try:
            choice = input("  Select: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        # ── Straight runs ──────────────────────────────────────────────────
        if choice in ("1", "2", "3"):
            if choice == "1":
                target = 1.0
            elif choice == "2":
                target = 2.0
            else:
                try:
                    target = float(input("  Target distance (m): "))
                except ValueError:
                    print("  Invalid."); continue
                if target <= 0 or target > SAFETY_DIST_M:
                    print(f"  Must be 0 < distance <= {SAFETY_DIST_M} m."); continue

            print(f"\n  Straight run: {target:.2f} m at duty={cruise_duty:.3f}")
            print(f"  Clear at least {target + 1.0:.1f} m of straight path ahead.")
            if not _confirm():
                continue

            print()
            final = run_straight(state, vesc, odo, servo, target, cruise_duty, dry_run)
            print(f"\n\n  Odometry distance : {final:.4f} m")
            print("  Mark where the robot stopped, measure with a tape measure.")
            _calibration_report(cfg_vesc, final)

        # ── Trapezoidal ────────────────────────────────────────────────────
        elif choice == "4":
            try:
                raw_d = input("  Total distance (m) [default 2.0]: ").strip()
                raw_r = input("  Ramp distance each end (m) [default 0.4]: ").strip()
                target    = float(raw_d) if raw_d else 2.0
                ramp_dist = float(raw_r) if raw_r else 0.4
            except ValueError:
                print("  Invalid."); continue

            if target <= 0 or target > SAFETY_DIST_M:
                print(f"  Must be 0 < distance <= {SAFETY_DIST_M} m."); continue

            print(f"\n  Trapezoidal: {target:.2f} m total, {ramp_dist:.2f} m ramp each end,"
                  f" duty={cruise_duty:.3f}")
            print(f"  Clear at least {target + 1.0:.1f} m of straight path ahead.")
            if not _confirm():
                continue

            print()
            result = run_trapezoid(
                state, vesc, odo, servo, target, cruise_duty, ramp_dist, dry_run
            )
            print(f"\n\n  Trapezoidal result")
            print("  " + "-" * 42)
            print(f"  Odo distance : {result['odo_dist']:.4f} m")
            print(f"  Total time   : {result['total_time']:.2f} s")
            print(f"  Peak vel     : {result['peak_vel']:.4f} m/s")
            print(f"  Avg vel      : {result['avg_vel']:.4f} m/s")
            print("  " + "-" * 42)
            print("  Mark where the robot stopped, measure with a tape measure.")
            _calibration_report(cfg_vesc, result["odo_dist"])

        # ── Spot check ─────────────────────────────────────────────────────
        elif choice == "5":
            try:
                raw_duty = input(f"  Duty cycle [-1..1] (default {cruise_duty:.3f}): ").strip()
                raw_dur  = input("  Duration (s) [default 5]: ").strip()
                duty = float(raw_duty) if raw_duty else cruise_duty
                dur  = float(raw_dur)  if raw_dur  else 5.0
            except ValueError:
                print("  Invalid."); continue

            if abs(duty) > 0.3:
                print("  Duty > ±0.3 rejected for spot-check safety. Use option 6 to raise cruise duty.")
                continue

            if not _confirm(f"Press ENTER to spin motor at duty={duty:.3f}"):
                continue

            run_spot_check(state, vesc, duty, dur, dry_run)

        # ── Change cruise duty ──────────────────────────────────────────────
        elif choice == "6":
            try:
                new_duty = float(input(f"  New cruise duty (max ±0.5, current {cruise_duty:.3f}): "))
                if abs(new_duty) > 0.5:
                    print("  Clamped to ±0.5 for safety.")
                    new_duty = math.copysign(0.5, new_duty)
                cruise_duty = new_duty
                print(f"  Cruise duty set to {cruise_duty:.3f}")
            except ValueError:
                print("  Invalid.")

        elif choice == "q":
            break

        else:
            print("  Unknown option.")


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive odometry calibration")
    parser.add_argument("--config",  default=None, help="Path to params.yaml")
    parser.add_argument("--dry-run", action="store_true",
                        help="No actuator output — reads SharedState but motors off")
    args = parser.parse_args()

    script_dir  = os.path.dirname(os.path.abspath(__file__))
    config_root = os.path.join(script_dir, "..", "config")
    config_path = args.config or os.path.join(config_root, "params.yaml")

    config = load_config(config_path)
    state  = SharedState()
    odo    = Odometry()
    vesc   = VESCInterface(state, config)
    servo  = ServoControl(config)

    if not args.dry_run:
        vesc.start()
        time.sleep(0.5)   # let VESC thread open the port

    print(f"\nOdometry calibration — {'DRY RUN' if args.dry_run else 'LIVE'}")
    print(f"Config: {config_path}")

    try:
        menu(state, vesc, odo, servo, config, args.dry_run)
    except KeyboardInterrupt:
        pass
    finally:
        if not args.dry_run:
            try:
                vesc.set_throttle(0.0)
                time.sleep(0.2)
                servo.stop()
            except Exception as e:
                log.warning("Shutdown error: %s", e)
        state.set(running=False)
        print("\nDone.")


if __name__ == "__main__":
    main()
