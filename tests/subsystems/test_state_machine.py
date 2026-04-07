"""
test_state_machine.py — Unit tests for state_machine.py

Tests cover all 5 states and their transitions:
  - Default state is LINE_FOLLOW
  - LINE_FOLLOW → DEAD_RECKON when confidence < threshold
  - DEAD_RECKON → REACQUIRE when confidence recovers
  - REACQUIRE → LINE_FOLLOW after hysteresis
  - REACQUIRE → DEAD_RECKON if confidence drops again
  - LINE_FOLLOW → GATE_APPROACH when odometry within gate radius
  - GATE_APPROACH → LINE_FOLLOW when past gate
  - LINE_FOLLOW → INTERSECTION when odometry within intersection radius
  - INTERSECTION turn direction: left=-0.8, right=+0.8
  - INTERSECTION ease-in ramp over 0.2s
  - INTERSECTION → LINE_FOLLOW after 1.2s
"""

import math
import sys
import os
import time
import pytest
from unittest.mock import MagicMock, patch

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from state_machine import (
    StateMachine,
    LINE_FOLLOW, DEAD_RECKON, REACQUIRE, GATE_APPROACH, INTERSECTION,
    REACQUIRE_HYSTERESIS_S, INTERSECTION_TURN_S,
)
from odometry import Odometry


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _make_sm(intersections=None, gates=None, odo=None):
    """Build a StateMachine with controllable config and route."""
    config = {
        "sensor": {"low_confidence_threshold": 30},
        "speed":  {"gate_ms": 0.3, "base_ms": 0.6},
    }
    route = {
        "intersections": intersections or [],
        "gates":         gates or [],
    }
    if odo is None:
        odo = Odometry()
    return StateMachine(config, route, odo), odo


def _step(sm, confidence=200, pid_steer=0.0, pid_throttle=0.6,
          heading=0.0, dt=0.01):
    """Run one update step, return (steering, throttle, state)."""
    return sm.update(confidence, pid_steer, pid_throttle, heading, dt)


# ─────────────────────────────────────────────────────────────────────────────
# Default state
# ─────────────────────────────────────────────────────────────────────────────

class TestDefaultState:

    def test_initial_state_is_line_follow(self):
        sm, _ = _make_sm()
        assert sm.state == LINE_FOLLOW

    def test_high_confidence_stays_line_follow(self):
        sm, _ = _make_sm()
        _, _, state = _step(sm, confidence=200)
        assert state == LINE_FOLLOW

    def test_pid_output_passed_through_in_line_follow(self):
        """In LINE_FOLLOW the PID outputs are passed through unchanged."""
        sm, _ = _make_sm()
        steering, throttle, state = _step(sm, pid_steer=0.42, pid_throttle=0.55)
        assert state == LINE_FOLLOW
        assert steering == pytest.approx(0.42)
        assert throttle == pytest.approx(0.55)


# ─────────────────────────────────────────────────────────────────────────────
# LINE_FOLLOW → DEAD_RECKON
# ─────────────────────────────────────────────────────────────────────────────

class TestDeadReckon:

    def test_low_confidence_triggers_dead_reckon(self):
        sm, _ = _make_sm()
        _, _, state = _step(sm, confidence=10)
        assert state == DEAD_RECKON

    def test_confidence_at_threshold_is_dead_reckon(self):
        """confidence == threshold - 1 → DEAD_RECKON."""
        sm, _ = _make_sm()
        _, _, state = _step(sm, confidence=29)
        assert state == DEAD_RECKON

    def test_confidence_exactly_at_threshold_is_line_follow(self):
        """confidence == threshold → stays in LINE_FOLLOW."""
        sm, _ = _make_sm()
        _, _, state = _step(sm, confidence=30)
        assert state == LINE_FOLLOW

    def test_dead_reckon_drives_straight(self):
        """In DEAD_RECKON: steering=0, throttle=gate_speed*0.8."""
        sm, _ = _make_sm()
        _step(sm, confidence=10)   # enter DEAD_RECKON
        steering, throttle, state = _step(sm, confidence=10)
        assert state == DEAD_RECKON
        assert steering == pytest.approx(0.0)
        assert throttle == pytest.approx(0.3 * 0.8)   # gate_ms=0.3, factor=0.8

    def test_dead_reckon_ignores_pid_output(self):
        """DEAD_RECKON output is fixed, not from PID."""
        sm, _ = _make_sm()
        _step(sm, confidence=10)
        steering, _, _ = _step(sm, confidence=10, pid_steer=0.99)
        assert steering == pytest.approx(0.0)   # PID ignored


# ─────────────────────────────────────────────────────────────────────────────
# DEAD_RECKON → REACQUIRE → LINE_FOLLOW
# ─────────────────────────────────────────────────────────────────────────────

class TestReacquire:

    def test_confidence_recovery_enters_reacquire(self):
        sm, _ = _make_sm()
        _step(sm, confidence=10)   # enter DEAD_RECKON
        _, _, state = _step(sm, confidence=100)   # should go to REACQUIRE
        assert state == REACQUIRE

    def test_reacquire_uses_pid_steering_at_gate_speed(self):
        """REACQUIRE: passes PID steering but uses gate speed."""
        sm, _ = _make_sm()
        _step(sm, confidence=10)
        steering, throttle, state = _step(sm, confidence=100, pid_steer=0.3)
        assert state == REACQUIRE
        assert steering == pytest.approx(0.3)
        assert throttle == pytest.approx(0.3)   # gate_ms=0.3

    def test_reacquire_to_line_follow_after_hysteresis(self):
        """REACQUIRE → LINE_FOLLOW after REACQUIRE_HYSTERESIS_S seconds."""
        t = [0.0]

        def fake_monotonic():
            return t[0]

        with patch("state_machine.time.monotonic", side_effect=fake_monotonic):
            sm, _ = _make_sm()
            _step(sm, confidence=10)    # enter DEAD_RECKON
            _step(sm, confidence=100)   # enter REACQUIRE

            steps_needed = int(REACQUIRE_HYSTERESIS_S / 0.01) + 5
            state = REACQUIRE
            for _ in range(steps_needed):
                t[0] += 0.01
                _, _, state = _step(sm, confidence=100, dt=0.01)

        assert state == LINE_FOLLOW

    def test_reacquire_back_to_dead_reckon_on_loss(self):
        """If confidence drops in REACQUIRE → back to DEAD_RECKON."""
        sm, _ = _make_sm()
        _step(sm, confidence=10)
        _step(sm, confidence=100)   # enter REACQUIRE
        _, _, state = _step(sm, confidence=5)   # lose line again
        assert state == DEAD_RECKON


# ─────────────────────────────────────────────────────────────────────────────
# GATE_APPROACH
# ─────────────────────────────────────────────────────────────────────────────

class TestGateApproach:

    def test_gate_approach_triggered_by_odometry(self):
        """Robot within gate radius → GATE_APPROACH."""
        odo = Odometry()
        odo.reset(0.0)
        gates = [{"id": 1, "x": 0.1, "y": 0.0, "radius_m": 0.5}]
        sm, _ = _make_sm(gates=gates, odo=odo)
        # Robot is at (0,0); gate at (0.1, 0); distance = 0.1 < 0.5
        _, _, state = _step(sm, confidence=200)
        assert state == GATE_APPROACH

    def test_gate_approach_slows_speed(self):
        """In GATE_APPROACH: throttle reduced to gate_ms."""
        odo = Odometry()
        odo.reset(0.0)
        gates = [{"id": 1, "x": 0.1, "y": 0.0, "radius_m": 0.5}]
        sm, _ = _make_sm(gates=gates, odo=odo)
        _, throttle, state = _step(sm, confidence=200)
        assert state == GATE_APPROACH
        assert throttle == pytest.approx(0.3)   # gate_ms

    def test_gate_approach_keeps_pid_steering(self):
        """In GATE_APPROACH: PID steering is preserved."""
        odo = Odometry()
        odo.reset(0.0)
        gates = [{"id": 1, "x": 0.1, "y": 0.0, "radius_m": 0.5}]
        sm, _ = _make_sm(gates=gates, odo=odo)
        steering, _, _ = _step(sm, confidence=200, pid_steer=0.35)
        assert steering == pytest.approx(0.35)

    def test_gate_not_triggered_when_far(self):
        """Robot far from gate → stays in LINE_FOLLOW."""
        odo = Odometry()
        odo.reset(0.0)
        gates = [{"id": 1, "x": 5.0, "y": 0.0, "radius_m": 0.5}]
        sm, _ = _make_sm(gates=gates, odo=odo)
        _, _, state = _step(sm, confidence=200)
        assert state == LINE_FOLLOW


# ─────────────────────────────────────────────────────────────────────────────
# INTERSECTION
# ─────────────────────────────────────────────────────────────────────────────

class TestIntersection:

    def test_intersection_triggered_by_odometry(self):
        """Robot within intersection radius → INTERSECTION state."""
        odo = Odometry()
        odo.reset(0.0)
        intersections = [{"id": 1, "x": 0.1, "y": 0.0,
                          "direction": "left", "radius_m": 0.3}]
        sm, _ = _make_sm(intersections=intersections, odo=odo)
        _, _, state = _step(sm, confidence=200)
        assert state == INTERSECTION

    def test_intersection_not_triggered_when_far(self):
        """Robot far from intersection → stays LINE_FOLLOW."""
        odo = Odometry()
        odo.reset(0.0)
        intersections = [{"id": 1, "x": 5.0, "y": 0.0,
                          "direction": "left", "radius_m": 0.3}]
        sm, _ = _make_sm(intersections=intersections, odo=odo)
        _, _, state = _step(sm, confidence=200)
        assert state == LINE_FOLLOW

    def test_left_intersection_negative_steering(self):
        """Left intersection → steering = -0.8 (after ease-in)."""
        odo = Odometry()
        odo.reset(0.0)
        intersections = [{"id": 1, "x": 0.0, "y": 0.0,
                          "direction": "left", "radius_m": 0.3}]
        sm, _ = _make_sm(intersections=intersections, odo=odo)
        _step(sm, confidence=200)   # enter INTERSECTION
        # After ease-in completes, steering should be -0.8
        steps_ease = int(0.25 / 0.01)   # past 0.2s ease-in
        for _ in range(steps_ease):
            steering, _, state = _step(sm, confidence=200, dt=0.01)
        assert state == INTERSECTION
        assert steering == pytest.approx(-0.8, abs=0.05)

    def test_right_intersection_positive_steering(self):
        """Right intersection → steering = +0.8 (after ease-in)."""
        odo = Odometry()
        odo.reset(0.0)
        intersections = [{"id": 1, "x": 0.0, "y": 0.0,
                          "direction": "right", "radius_m": 0.3}]
        sm, _ = _make_sm(intersections=intersections, odo=odo)
        _step(sm, confidence=200)
        steps_ease = int(0.25 / 0.01)
        for _ in range(steps_ease):
            steering, _, state = _step(sm, confidence=200, dt=0.01)
        assert state == INTERSECTION
        assert steering == pytest.approx(+0.8, abs=0.05)

    def test_intersection_ease_in_starts_small(self):
        """First step in INTERSECTION: ease=0, steering near 0."""
        odo = Odometry()
        odo.reset(0.0)
        intersections = [{"id": 1, "x": 0.0, "y": 0.0,
                          "direction": "left", "radius_m": 0.3}]
        sm, _ = _make_sm(intersections=intersections, odo=odo)
        # The very first step triggers INTERSECTION and outputs with ease
        # ease = min(1.0, elapsed/0.2); elapsed ≈ 0 so steering ≈ 0
        steering, _, state = _step(sm, confidence=200, dt=0.001)
        assert state == INTERSECTION
        assert abs(steering) < 0.1   # early ease-in

    def test_intersection_reduces_to_gate_speed(self):
        """INTERSECTION: throttle = gate_ms."""
        odo = Odometry()
        odo.reset(0.0)
        intersections = [{"id": 1, "x": 0.0, "y": 0.0,
                          "direction": "left", "radius_m": 0.3}]
        sm, _ = _make_sm(intersections=intersections, odo=odo)
        _, throttle, state = _step(sm, confidence=200)
        assert state == INTERSECTION
        assert throttle == pytest.approx(0.3)

    def test_intersection_completes_and_returns_to_line_follow(self):
        """INTERSECTION exits to LINE_FOLLOW after INTERSECTION_TURN_S seconds."""
        odo = Odometry()
        odo.reset(0.0)
        intersections = [{"id": 1, "x": 0.0, "y": 0.0,
                          "direction": "left", "radius_m": 0.3}]
        sm, _ = _make_sm(intersections=intersections, odo=odo)
        _step(sm, confidence=200)   # enter INTERSECTION

        steps = int(INTERSECTION_TURN_S / 0.01) + 10
        state = INTERSECTION
        for _ in range(steps):
            _, _, state = _step(sm, confidence=200, dt=0.01)

        assert state == LINE_FOLLOW

    def test_intersection_triggered_only_once(self):
        """Once an intersection is triggered, it is not re-triggered."""
        odo = Odometry()
        odo.reset(0.0)
        intersections = [{"id": 1, "x": 0.0, "y": 0.0,
                          "direction": "left", "radius_m": 0.5}]
        sm, _ = _make_sm(intersections=intersections, odo=odo)

        # Trigger intersection and complete it
        _step(sm, confidence=200)
        for _ in range(int(INTERSECTION_TURN_S / 0.01) + 10):
            _step(sm, confidence=200, dt=0.01)

        # Back to LINE_FOLLOW — intersection should now be "triggered=True"
        _, _, state = _step(sm, confidence=200)
        # Should NOT enter INTERSECTION again
        assert state != INTERSECTION

    def test_intersection_has_priority_over_dead_reckon(self):
        """INTERSECTION takes priority over DEAD_RECKON (checked first)."""
        odo = Odometry()
        odo.reset(0.0)
        intersections = [{"id": 1, "x": 0.0, "y": 0.0,
                          "direction": "right", "radius_m": 0.5}]
        sm, _ = _make_sm(intersections=intersections, odo=odo)
        # Low confidence AND within intersection radius
        _, _, state = _step(sm, confidence=5)   # threshold=30
        assert state == INTERSECTION   # intersection wins


# ─────────────────────────────────────────────────────────────────────────────
# Multiple intersections
# ─────────────────────────────────────────────────────────────────────────────

class TestMultipleIntersections:

    def test_first_intersection_triggered_first(self):
        """Only the closest (first eligible) intersection fires at a time."""
        odo = Odometry()
        odo.reset(0.0)
        intersections = [
            {"id": 1, "x": 0.0, "y": 0.0, "direction": "left",  "radius_m": 0.3},
            {"id": 2, "x": 5.0, "y": 0.0, "direction": "right", "radius_m": 0.3},
        ]
        sm, _ = _make_sm(intersections=intersections, odo=odo)
        _, _, state = _step(sm, confidence=200)
        assert state == INTERSECTION
        assert sm._current_turn_direction == "left"   # first intersection, not second
