# Mobot — AI Development Guide

## Development Setup

**Claude Code runs on Cole's laptop (macOS), not on the Pi.**
The robot code is developed here and deployed to the Raspberry Pi 5 over the network.
- Do not expect hardware (lgpio, sensors, VESC) to be present on the local machine
- Flask and other Pi dependencies may not be installed locally — that is expected
- When diagnosing runtime issues, consider whether the problem is local (laptop) or on the Pi

---

## Competition Context

**CMU 32nd Annual Mobot Slalom Race, Spring 2026.**
This robot competes for a **significant cash prize** on a paved walkway in front of Wean Hall.
The race format is a timed slalom: the robot must follow a white line, navigate through gates,
and make correct left/right turns at intersections (directions announced by judges minutes before each heat).

**A broken robot on race day = loss of the prize. Treat every change with care.**

---

## Before Making ANY Change

Ask yourself these questions before touching a file:

1. Have I **read the existing file completely**?
2. Will this change **affect the 100 Hz main control loop**?
3. Is this change **tested** — did I run `bash scripts/run_tests.sh`?
4. Does this work on **Pi 5** (uses `lgpio`, NOT `pigpio`; gpiochip=4)?
5. Does this handle **hardware disconnects** gracefully (serial reconnect loops)?
6. Can I **undo** this quickly if it breaks on race morning?
7. Is this the **minimum change** needed, or am I adding unnecessary complexity?

---

## Hardware Summary

| Component | Interface | Device | Notes |
|-----------|-----------|--------|-------|
| Line sensor (QTRX-MD-16A) | USB serial | `/dev/ttyACM0` @ 115200 | 8-byte packets @ ~200 Hz |
| BNO085 IMU | I2C | `/dev/i2c-1` @ 0x4A | Quaternion → yaw, drift-free |
| VESC FlipSky 6.7 Pro | USB serial | `/dev/ttyACM1` @ 115200 | pyvesc protocol, 50 Hz |
| Steering servo | GPIO PWM | GPIO 18, gpiochip4 | lgpio, 50 Hz, 1000–2000 µs |
| Raspberry Pi 5 | — | — | **lgpio NOT pigpio**, gpiochip=4 |
| Teensy 4.x coprocessor | USB | `/dev/ttyACM0` | Line sensor firmware |

**Critical Pi 5 note:** Always use `lgpio`, never `pigpio`. The Pi 5 routes GPIO through the RP1
chip on `gpiochip4`. `pigpio` is incompatible. Imports: `import lgpio`.

---

## Architecture

```
main.py (100 Hz)
  ├── SensorReader [thread]   → SharedState (line_pos, confidence, heading)
  ├── VESCInterface [thread]  → SharedState (velocity, rpm, voltage)
  ├── Odometry [sync]         → SharedState (x, y)
  ├── PIDController [sync]    → (steering, throttle)
  ├── StateMachine [sync]     → overrides PID for gates/intersections
  ├── ServoControl [sync]     → GPIO PWM output
  └── WebServer [thread]      → http://0.0.0.0:5000
```

**Threading model:** All inter-thread data goes through `SharedState` with a single
`threading.Lock()`. Always use `state.get(...)` and `state.set(...)` / `state.update_*()`.
Never access `SharedState` fields directly from outside the main thread.

**100 Hz loop:** Any blocking call in the main loop adds jitter. Keep synchronous
code in `Odometry`, `PIDController`, `StateMachine`, `ServoControl` fast (< 1 ms).

---

## Module Responsibilities (one line each)

- `main.py` — Entry point, 100 Hz loop, CSV logging, curses display, hot-reload
- `shared_state.py` — Thread-safe state container (all inter-thread data)
- `sensor_reader.py` — Teensy USB packet parser + BNO085 I2C reader (daemon thread)
- `vesc_interface.py` — VESC UART bidirectional: throttle command + telemetry (daemon thread)
- `odometry.py` — Unicycle WIO: x += v·cos(θ)·dt, y += v·sin(θ)·dt
- `pid_controller.py` — PD steering (derivative on measurement) + P speed control
- `state_machine.py` — 5-state FSM: LINE_FOLLOW, DEAD_RECKON, REACQUIRE, GATE_APPROACH, INTERSECTION
- `servo_control.py` — lgpio PWM, maps [-1,+1] → [min_pw, max_pw] µs with center_offset_us trim
- `web_server.py` — Flask on 0.0.0.0:5000, SSE stream, params/route hot-reload
- `mapper.py` — Slow-speed mapping run, records path to JSON
- `tune_servo_center.py` — Interactive terminal servo center offset tuner

---

## Sub-Agent Usage

When implementing multiple **independent** features (e.g., new tests + new HTML page + a Python utility),
**spawn parallel sub-agents** — one per task. This prevents context bloat and speeds up work.

Guidelines for sub-agents:
- Each agent must **read the relevant existing files** before writing code
- Provide the agent with exact file paths, class names, and function signatures it needs
- Specify clearly: "write code" vs. "research only"
- Never have two agents edit the same file simultaneously
- After all agents complete, verify integration (run tests, check imports)

---

## Race-Day Critical Files

| File | What to update | When |
|------|----------------|------|
| `config/route.yaml` | Intersection `direction: left/right` | After judges announce, before heat |
| `config/params.yaml` | PID gains, servo trim | After tuning runs |

**Race-morning workflow (< 60 seconds):**
1. Get intersection directions from judges
2. Open `http://<pi-ip>:5000/route` on any browser
3. Click LEFT or RIGHT buttons for each intersection
4. Press SAVE ROUTE → confirm
5. Done — no SSH, no restart needed

---

## Testing Protocol

```bash
# Run all unit tests (no hardware):
bash scripts/run_tests.sh

# Safe bench test (sensors only, no actuators):
bash scripts/run.sh --dry-run

# Full live run:
bash scripts/run.sh

# Mapping pass:
bash scripts/run_mapper.sh

# Servo center tuner (terminal):
bash scripts/run_servo_tuner.sh

# Web server only (for UI development):
bash scripts/run_web.sh
```

**NEVER deploy code to the robot that hasn't passed `bash scripts/run_tests.sh`.**

---

## DO NOT

- Import `pigpio` — use `lgpio` (Pi 5 incompatibility)
- Block the main 100 Hz loop (no `time.sleep()` in synchronous modules)
- Bypass the `SharedState` lock (never set fields directly from a background thread)
- Change `center_pw` in params — change `center_offset_us` instead
- Deploy untested changes on race day
- Hard-code device paths — they come from config

---

## Known Risks

- **Teensy USB disconnect**: SensorReader reconnects automatically, but confidence drops
  and DEAD_RECKON activates. Watch for this during vibration/bumpy terrain.
- **Battery sag**: VESC reports voltage — warn if < 10.5V and adjust speed down.
- **Outdoor lighting**: Teensy calibration must be done in race-day lighting conditions.
- **IMU drift**: BNO085 is fusion-corrected but can drift if magnetic environment changes.
  Perform gyro reset if heading looks wrong before a run.
- **BEC current**: Pi 5 draws up to 5A. Ensure VESC BEC is rated ≥ 5A continuous.

---

## Questions to Always Ask

Before any code change, ask:

- What is the failure mode if this breaks? How bad is it?
- Does the existing test suite cover this path?
- Is there an existing utility or pattern I should reuse instead of writing new code?
- Does this change affect real-time performance?
- Should this be a config parameter rather than a hardcoded value?
