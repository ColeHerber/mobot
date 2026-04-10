"""High-level state machine for Mobot course navigation.

States
------
LINE_FOLLOW   Default — PID on line_position, odometry accumulates.
GATE_APPROACH Near a known gate — slow to gate speed, align heading.
DEAD_RECKON   Sensor confidence too low — use odometry arc, ignore PID.
REACQUIRE     Line reappeared after dead reckon — brief hysteresis.
INTERSECTION  At a pre-loaded waypoint — execute pre-planned turn.
CHORD         Dead-reckon toward a specific (x,y) using heading control,
              ignoring the white line. Activated by gates with type: chord.
DROP_WATCH    Armed near a DROP ramp — monitors pitch to detect ramp
              traversal; snaps odometry to known x when robot levels out.

Transitions
-----------
LINE_FOLLOW   → DROP_WATCH    if odometry within watch_radius_m of a DROP landmark
LINE_FOLLOW   → DEAD_RECKON   if confidence < LOW_THRESHOLD
LINE_FOLLOW   → CHORD         if within chord gate radius (checked before regular gates)
LINE_FOLLOW   → GATE_APPROACH if odometry within gate_radius of a gate wp
LINE_FOLLOW   → INTERSECTION  if odometry within intersection radius of an
                               intersection waypoint
DROP_WATCH    → LINE_FOLLOW   once |pitch| > entry_pitch then recovers to < level_pitch
                               (or after DROP_WATCH_TIMEOUT_S — snap either way)
DEAD_RECKON   → REACQUIRE     if confidence >= LOW_THRESHOLD
REACQUIRE     → LINE_FOLLOW   after REACQUIRE_HYSTERESIS_S seconds
GATE_APPROACH → LINE_FOLLOW   once past gate (heading stable)
INTERSECTION  → LINE_FOLLOW   once turn is complete
CHORD         → LINE_FOLLOW   once within arrival_radius_m of target (x,y)

Route is loaded from config/route.yaml. Each intersection entry has:
  id, x, y, direction (left|right), radius_m
Each gate entry has:
  id, x, y, radius_m, [type: gate|chord], [arrival_radius_m]
Each drop_landmark entry has:
  id, x, watch_radius_m, entry_pitch_deg, level_pitch_deg
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
CHORD         = "CHORD"
DROP_WATCH    = "DROP_WATCH"

REACQUIRE_HYSTERESIS_S = 0.15   # seconds to stay in REACQUIRE before → LINE_FOLLOW
INTERSECTION_TURN_S    = 1.2    # seconds to execute a turn at an intersection
CHORD_STEER_KP         = 1.2    # P-gain on heading error for chord navigation
DROP_WATCH_TIMEOUT_S   = 6.0    # snap odometry and exit DROP_WATCH after this long


class StateMachine:
    def __init__(self, config: dict, route: dict, odometry):
        self._odo = odometry

        sensor_cfg = config.get("sensor", {})
        speed_cfg  = config.get("speed", {})

        self._low_threshold      = int(sensor_cfg.get("low_confidence_threshold", 30))
        self._gate_speed         = float(speed_cfg.get("gate_ms", 0.3))
        self._base_speed         = float(speed_cfg.get("base_ms", 0.6))
        self._dead_reckon_steer  = float(sensor_cfg.get("dead_reckon_steer", -0.5))

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
        # Gates with type: chord go into _chord_gates; all others into _gates.
        self._gates = []
        self._chord_gates = []
        for g in route.get("gates", []):
            if g.get("type", "gate") == "chord":
                self._chord_gates.append({
                    "id":               g["id"],
                    "x":                float(g["x"]),
                    "y":                float(g.get("y", 0.0)),
                    "radius_m":         float(g.get("radius_m", 1.5)),
                    "arrival_radius_m": float(g.get("arrival_radius_m", 0.4)),
                    "triggered":        False,
                })
            else:
                self._gates.append({
                    "id":        g["id"],
                    "x":         float(g["x"]),
                    "y":         float(g.get("y", 0.0)),
                    "radius_m":  float(g.get("radius_m", 0.5)),
                    "passed":    False,
                })

        # Parse drop landmarks from route YAML
        self._drop_landmarks = []
        for d in route.get("drop_landmarks", []):
            self._drop_landmarks.append({
                "id":              d["id"],
                "x":               float(d["x"]),
                "watch_radius_m":  float(d.get("watch_radius_m", 2.5)),
                "entry_pitch_rad": math.radians(float(d.get("entry_pitch_deg", 6.0))),
                "level_pitch_rad": math.radians(float(d.get("level_pitch_deg", 3.0))),
                "triggered":       False,
            })

        self._state = LINE_FOLLOW
        self._active_chord: dict | None = None
        self._drop_active: dict | None = None
        self._drop_saw_ramp: bool = False

        # Timing helpers
        self._reacquire_start: float = 0.0
        self._intersection_start: float = 0.0
        self._drop_watch_start: float = 0.0
        self._current_turn_direction: str = "left"

    def reset(self):
        """Reset to LINE_FOLLOW — call on robot enable to avoid startup DEAD_RECKON."""
        self._state = LINE_FOLLOW
        self._active_chord = None
        self._drop_active  = None
        self._drop_saw_ramp = False
        self._reacquire_start    = 0.0
        self._intersection_start = 0.0
        self._drop_watch_start   = 0.0

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

    def _check_chord_gate(self) -> dict | None:
        """Return the first untriggered chord gate within its entry radius."""
        for g in self._chord_gates:
            if not g["triggered"]:
                dist = self._odo.distance_to(g["x"], g["y"])
                if dist <= g["radius_m"]:
                    return g
        return None

    def _check_drop_landmark(self) -> dict | None:
        """Return the first untriggered DROP landmark whose x-window contains odo.x."""
        for d in self._drop_landmarks:
            if not d["triggered"]:
                if abs(self._odo.x - d["x"]) <= d["watch_radius_m"]:
                    return d
        return None

    def update(self, confidence: int, pid_steering: float,
               pid_throttle: float, heading_rad: float, pitch_rad: float,
               dt: float):
        """Run one state machine step.

        Returns (steering, throttle, state_name) — values override PID when
        state is not LINE_FOLLOW.
        """
        now = time.monotonic()

        # ── Transitions ───────────────────────────────────────────────────────
        if self._state == LINE_FOLLOW:
            # Check intersection first (highest priority)
            wp = self._check_intersection()
            if wp is not None:
                wp["triggered"] = True
                self._current_turn_direction = wp["direction"]
                self._intersection_start = now
                log.info("→ INTERSECTION %s (%s)", wp["id"], wp["direction"])
                self._state = INTERSECTION

            else:
                # DROP landmarks checked before confidence test — arm even if
                # line sensor degrades on the raised section
                drop = self._check_drop_landmark()
                if drop is not None:
                    drop["triggered"] = True
                    self._drop_active = drop
                    self._drop_saw_ramp = False
                    self._drop_watch_start = now
                    log.info("→ DROP_WATCH landmark %s (x=%.2f)", drop["id"], drop["x"])
                    self._state = DROP_WATCH

                elif confidence < self._low_threshold:
                    log.info("→ DEAD_RECKON (confidence=%d)", confidence)
                    self._state = DEAD_RECKON

                else:
                    # Chord gates checked before regular gates
                    chord_gate = self._check_chord_gate()
                    if chord_gate is not None:
                        chord_gate["triggered"] = True
                        self._active_chord = chord_gate
                        log.info("→ CHORD gate %s → (%.2f, %.2f)",
                                 chord_gate["id"], chord_gate["x"], chord_gate["y"])
                        self._state = CHORD
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

        elif self._state == DROP_WATCH:
            d = self._drop_active
            pitch_abs = abs(pitch_rad)
            if pitch_abs >= d["entry_pitch_rad"]:
                self._drop_saw_ramp = True
            settled   = self._drop_saw_ramp and pitch_abs < d["level_pitch_rad"]
            timed_out = (now - self._drop_watch_start) >= DROP_WATCH_TIMEOUT_S
            if settled or timed_out:
                if timed_out and not settled:
                    log.warning("DROP_WATCH timeout on %s — snapping x to %.2f",
                                d["id"], d["x"])
                self._odo.reset_x(d["x"])
                log.info("→ LINE_FOLLOW (drop snap x=%.2f, landmark %s)",
                         d["x"], d["id"])
                self._drop_active = None
                self._state = LINE_FOLLOW

        elif self._state == CHORD:
            tgt = self._active_chord
            dist = self._odo.distance_to(tgt["x"], tgt["y"])
            if dist <= tgt["arrival_radius_m"]:
                log.info("→ LINE_FOLLOW (chord gate %s arrived)", tgt["id"])
                self._active_chord = None
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
            # Steer toward disabled-sensor edge (line assumed to have exited that side)
            return self._dead_reckon_steer, self._gate_speed * 0.8, DEAD_RECKON

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

        elif self._state == DROP_WATCH:
            # Robot still line-follows; state name signals the DROP detection window
            return pid_steering, pid_throttle, DROP_WATCH

        elif self._state == CHORD:
            # Dead-reckon toward target (x, y) using IMU heading
            tgt = self._active_chord
            dx = tgt["x"] - self._odo.x
            dy = tgt["y"] - self._odo.y
            desired_heading = math.atan2(dy, dx)
            heading_err = desired_heading - heading_rad
            # Wrap to [-π, π]
            while heading_err >  math.pi: heading_err -= 2 * math.pi
            while heading_err < -math.pi: heading_err += 2 * math.pi
            steer = max(-1.0, min(1.0, CHORD_STEER_KP * heading_err))
            return steer, self._gate_speed, CHORD

        return pid_steering, pid_throttle, self._state
