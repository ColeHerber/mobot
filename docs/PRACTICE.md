# Practice Run Guide

Follow these steps every practice session. They mirror competition day exactly —
same commands, same order, same web UI for route setting.

---

## Before You Leave Home

- [ ] LiPo charged to full (not storage voltage)
- [ ] Latest code on Pi: `git pull` over SSH if you made changes on your laptop
- [ ] Bring: laptop (SSH + params tuning), phone (web UI route), spare USB-A cables

---

## At the Course — Startup

**1. Power on the robot.**
The Pi boots automatically when the battery is connected.

**2. SSH in:**
```bash
ssh mobot@mobot.local
```

**3. Check battery voltage — must be > 11.0V before running:**
```bash
python3 debug/vesc_debug.py
```
If voltage is low, stop. A sagging battery causes erratic motor behavior.

**4. Verify all sensors are live:**
```bash
# Line sensor — Raw row should change as you tilt the robot:
python3 debug/sensor_debug.py

# IMU — yaw should change as you rotate the robot:
python3 debug/imu_debug.py
```

**5. Confirm servo is centered:**
```bash
python3 debug/servo_hwpwm_debug.py --pwmchip 0 --channel 0
```
Press `0` to center, `s` to sweep. Robot should steer left and right. Press `q` to exit.

---

## Calibration (do every session, outdoors in current lighting)

**6. Run drive-through calibration:**
```bash
python3 debug/calibrate_drive.py --drive --duty 0.04
```
- Robot drives itself slowly over the course
- Watch the Span row build — all 16 channels should reach > 200
- If robot stalls, increase duty: `--duty 0.06`
- Press Ctrl+C when the robot finishes, type `y` to save

A timestamped snapshot is automatically saved to `calibrations/` — no extra step needed.

**7. Verify calibration quality:**
```bash
python3 debug/sensor_debug.py
```
Place robot centered on the line. You should see:
- `Norm` row: center channels near 1000, outer channels near 0
- `Pos` near 0.0
- `Conf` > 100

If Conf < 100 or Norm never reaches 1000, re-calibrate.

---

## Set the Route (always use web UI — mirrors comp day)

**8. Open the route editor on your phone or laptop:**
```
http://mobot.local:5000/route
```

Pick LEFT or RIGHT for each intersection as if the judges just announced it.
Press **SAVE ROUTE**. No SSH, no restart needed.

---

## Run Sequence

**9. Dry run — sensors only, no motor or steering output:**
```bash
bash scripts/run.sh --dry-run
```
Walk beside the robot and push it along the course. Watch the curses display:
- Line position should track the line smoothly
- Confidence should stay > 30
- State should stay `LINE_FOLLOW` except near gates/intersections

**10. Slow live run — confirm steering and motor work:**

Edit `config/params.yaml` and set:
```yaml
speed:
  base_ms: 0.3
```
(Hot-reloads — no restart needed after saving the file.)

```bash
bash scripts/run.sh
```
Stand at the end of the course ready to catch the robot. Let it complete one pass.

**11. Full speed runs:**

Restore `base_ms` to your target (e.g. 0.6) and run repeatedly.

Each run, watch for:
- **Oscillation on straights** → lower `kp` or raise `kd`
- **Slow reaction to curves** → raise `kp`
- **Line lost at a gate** → lower `gate_ms` or adjust gate position in `route.yaml`
- **Wrong turn at intersection** → check route.yaml direction, adjust `radius_m`
- **Robot drifts left/right on straights** → run `bash scripts/run_servo_tuner.sh`

---

## Between Runs — Tuning

All params hot-reload when you save the file — no restart needed.

| Problem | Fix | File |
|---|---|---|
| Oscillates on straights | Lower `kp` (try −0.1 steps) | `config/params.yaml` |
| Cuts corners | Raise `kd` | `config/params.yaml` |
| Too slow / stalls | Raise `base_ms` | `config/params.yaml` |
| Drifts left or right | Run servo tuner | `bash scripts/run_servo_tuner.sh` |
| Misses intersection | Adjust `radius_m` or `x` position | `config/route.yaml` |
| Conf drops mid-run | Re-calibrate (lighting changed?) | `python3 debug/calibrate_drive.py --drive` |

---

## Tweaks To-Do — Fill In Each Session

```
Date: _______________   Location: _______________   Weather/lighting: _______________

Battery voltage at start: _______V   End: _______V

Calibration snapshot: calibrations/_______________

Params used:
  base_ms:  _______   gate_ms:  _______   min_ms:  _______
  kp:       _______   kd:       _______   speed_kp: _______
  center_offset_us: _______

Observed issues:
  [ ] Oscillation on straights
  [ ] Line lost at gate ___
  [ ] Wrong turn at intersection ___
  [ ] Drifts left / right
  [ ] Missed intersection trigger
  [ ] Sensor confidence drops
  [ ] Other: _______________________________________________

Changes made between runs:
  1. _______________________________________________
  2. _______________________________________________
  3. _______________________________________________

Best run time: _______s   Notes: _______________________________________________
```

---

## Post-Session

- Commit any param changes you want to keep:
  ```bash
  git add config/params.yaml config/route.yaml
  git commit -m "tune: <what you changed and why>"
  ```
- Check the latest run log for anomalies:
  ```bash
  ls -t logs/ | head -1   # most recent log
  ```
- Charge LiPo to **storage voltage** (3.8V/cell) if done for the day.

---

## Quick Reference — All Commands

```bash
# Startup checks
python3 debug/vesc_debug.py          # battery voltage + motor telemetry
python3 debug/sensor_debug.py        # line sensor live view
python3 debug/imu_debug.py           # IMU heading
python3 debug/servo_hwpwm_debug.py --pwmchip 0 --channel 0   # servo sweep

# Calibration
python3 debug/calibrate_drive.py --drive --duty 0.04          # motor-drive cal
python3 debug/calibrate_drive.py --push                       # push-by-hand cal
python3 debug/cal_manager.py list                             # view saved snapshots

# Running
bash scripts/run.sh --dry-run        # sensors only
bash scripts/run.sh                  # full live run

# Tuning
bash scripts/run_servo_tuner.sh      # servo center trim

# Route (use web UI instead when possible)
http://mobot.local:5000/route
```
