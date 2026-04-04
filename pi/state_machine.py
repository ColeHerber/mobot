"""High-level state machine for Mobot course navigation.

States
------
LINE_FOLLOW   Default — PID on line_position, odometry accumulates.
GATE_APPROACH Near a known gate — slow to gate speed, align heading.
DEAD_RECKON   Sensor confidence too low — use odometry arc, ignore PID.
REACQUIRE     Line reappeared after dead reckon — brief hysteresis.
INTERSECTION  At a pre-loaded waypoint — execute pre-planned turn.

Transitions
-----------
LINE_FOLLOW   → DEAD_RECKON   if confidence < LOW_THRESHOLD
LINE_FOLLOW   → GATE_APPROACH if odometry within gate_radius of a gate wp
LINE_FOLLOW   → INTERSECTION  if odometry within intersection radius of an
                               intersection waypoint
DEAD_RECKON   → REACQUIRE     if confidence >= LOW_THRESHOLD
REACQUIRE     → LINE_FOLLOW   after REACQUIRE_HYSTERESIS_S seconds
GATE_APPROACH → LINE_FOLLOW   once past gate (heading stable)
INTERSECTION  → LINE_FOLLOW   once turn is complete

Route is loaded from config/route.yaml. Each intersection entry has:
  id, x, y, direction (left|right), radius_m
"""

import time
import math
import logging

log = logging.getLogger(__name__)

# State names
LINE_FOLLOW   = "LINE_FOLLOW"
GATE_APPROACH = "GATE_APPROACH"
DEAD_RECKON   = "DEAD_RECKON"
REACQUIRE     = "REACQUIRE"
INTERSECTION  = "INTERSECTION"

REACQUIRE_HYSTERESIS_S = 0.15   # seconds to stay in REACQUIRE before → LINE_FOLLOW
INTERSECTION_TURN_S    = 1.2    # seconds to execute a turn at an intersection


class StateMachine:
    def __init__(self, config: dict, route: dict, odometry):
        self._odo = odometry

        sensor_cfg = config.get("sensor", {})
        speed_cfg  = config.get("speed", {})

        self._low_threshold  = int(sensor_cfg.get("low_confidence_threshold", 30))
        self._gate_speed     = float(speed_cfg.get("gate_ms", 0.3))
        self._base_speed     = float(speed_cfg.get("base_ms", 0.6))

        # Parse intersections from route YAML
        self._intersections = []
        for wp in route.get("intersections", []):
            self._intersections.append({
                "id":        wp["id"],
                "x":         float(wp["x"]),
                "y":         float(wp.get("y", 0.0)),
                "direction": wp["direction"],   # "left" or "right"
                "radius_m":  float(wp.get("radius_m", 0.3)),
                "triggered": False,
            })

        # Parse gates from route YAML (optional)
        self._gates = []
        for g in route.get("gates", []):
            self._gates.append({
                "id":        g["id"],
                "x":         float(g["x"]),
                "y":         float(g.get("y", 0.0)),
                "radius_m":  float(g.get("radius_m", 0.5)),
                "passed":    False,
            })

        self._state = LINE_FOLLOW

        # Timing helpers
        self._reacquire_start: float = 0.0
        self._intersection_start: float = 0.0
        self._current_turn_direction: str = "left"

    @property
    def state(self) -> str:
        return self._state

    def _check_intersection(self) -> dict | None:
        """Return the first untriggered intersection within its trigger radius."""
        for wp in self._intersections:
            if not wp["triggered"]:
                dist = self._odo.distance_to(wp["x"], wp["y"])
                if dist <= wp["radius_m"]:
                    return wp
        return None

    def _check_gate(self) -> dict | None:
        """Return the first unpassed gate within its radius."""
        for g in self._gates:
            if not g["passed"]:
                dist = self._odo.distance_to(g["x"], g["y"])
                if dist <= g["radius_m"]:
                    return g
        return None

    def update(self, confidence: int, pid_steering: float,
               pid_throttle: float, heading_rad: float, dt: float):
        """Run one state machine step.

        Returns (steering, throttle, state_name) — values override PID when
        state is not LINE_FOLLOW.
        """
        now = time.monotonic()

        # ── Transitions ───────────────────────────────────────────────────────
        if self._state == LINE_FOLLOW:
            # Check intersection first (higher priority)
            wp = self._check_intersection()
            if wp is not None:
                wp["triggered"] = True
                self._current_turn_direction = wp["direction"]
                self._intersection_start = now
                log.info("→ INTERSECTION %s (%s)", wp["id"], wp["direction"])
                self._state = INTERSECTION

            elif confidence < self._low_threshold:
                log.info("→ DEAD_RECKON (confidence=%d)", confidence)
                self._state = DEAD_RECKON

            else:
                gate = self._check_gate()
                if gate is not None:
                    log.info("→ GATE_APPROACH gate %s", gate["id"])
                    self._state = GATE_APPROACH

        elif self._state == DEAD_RECKON:
            if confidence >= self._low_threshold:
                self._reacquire_start = now
                log.info("→ REACQUIRE")
                self._state = REACQUIRE

        elif self._state == REACQUIRE:
            if confidence < self._low_threshold:
                # Line lost again
                self._state = DEAD_RECKON
            elif (now - self._reacquire_start) >= REACQUIRE_HYSTERESIS_S:
                log.info("→ LINE_FOLLOW (reacquired)")
                self._state = LINE_FOLLOW

        elif self._state == GATE_APPROACH:
            # Exit when past gate radius
            gate_present = self._check_gate()
            if gate_present is None:
                # Moved past all active gate radii
                log.info("→ LINE_FOLLOW (past gate)")
                self._state = LINE_FOLLOW

        elif self._state == INTERSECTION:
            elapsed = now - self._intersection_start
            if elapsed >= INTERSECTION_TURN_S:
                log.info("→ LINE_FOLLOW (intersection complete)")
                self._state = LINE_FOLLOW

        # ── Output per state ──────────────────────────────────────────────────
        if self._state == LINE_FOLLOW:
            return pid_steering, pid_throttle, LINE_FOLLOW

        elif self._state == GATE_APPROACH:
            # Slow down, keep PID steering
            return pid_steering, self._gate_speed, GATE_APPROACH

        elif self._state == DEAD_RECKON:
            # Drive straight at reduced speed (line position unknown)
            return 0.0, self._gate_speed * 0.8, DEAD_RECKON

        elif self._state == REACQUIRE:
            # Hand back to PID but at reduced speed
            return pid_steering, self._gate_speed, REACQUIRE

        elif self._state == INTERSECTION:
            # Execute the pre-loaded turn
            turn_steer = -0.8 if self._current_turn_direction == "left" else 0.8
            elapsed = now - self._intersection_start
            # Ease in to the turn over first 0.2s
            ease = min(1.0, elapsed / 0.2)
            return turn_steer * ease, self._gate_speed, INTERSECTION

        return pid_steering, pid_throttle, self._state
