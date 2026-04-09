# Chord Navigation — Deploy & Test Checklist

## What was added
- **`type: chord` gates** in `config/route.yaml`: robot ignores white line and steers by
  IMU heading toward the gate's `(x, y)` coordinates, then resumes `LINE_FOLLOW` on arrival.
- **`CHORD` state** in `pi/state_machine.py`: P-controller on heading error (`CHORD_STEER_KP = 1.2`).
- **Route editor UI** (`/route`): each gate row now has a **● Gate / ⬦ Chord** toggle button.
  Chord gates render as orange diamonds on the course map.
- **`config/route.yaml`**: updated with estimated gate positions from satellite image
  and the **circles intersection** (gates 9–14, judge-announced direction).

---

## On Pi — steps before race

```bash
# 1. Pull latest code
cd ~/mobot && git pull

# 2. Run unit tests
bash scripts/run_tests.sh

# 3. Verify syntax (quick check if pytest unavailable)
python3 -c "from pi.state_machine import StateMachine; print('OK')"

# 4. Dry-run — push robot along course, watch curses display
bash scripts/run.sh --dry-run
```

**In the curses display, confirm:**
- State shows `CHORD` near any gate marked `type: chord` in route.yaml
- State shows `INTERSECTION` as robot enters the circles section (~38.5 m)
- State returns to `LINE_FOLLOW` after each

---

## Race morning — circles direction update (< 30 seconds)

1. Get direction from judges
2. Open `http://mobot.local:5000/route` on phone or laptop
3. Toggle **◀ LEFT** or **RIGHT ▶** for the `circles` intersection
4. Press **SAVE ROUTE** — done. No restart needed.

---

## Tuning chord gates

If the robot overshoots or oscillates during CHORD:
- Lower `CHORD_STEER_KP` in `pi/state_machine.py` (line ~48), default `1.2`
- Increase `arrival_radius_m` on the gate (default `0.4` m) to exit CHORD sooner
- Increase `radius_m` on the gate to trigger CHORD entry earlier

To disable a chord gate temporarily: toggle it back to `● Gate` in the route editor UI.

---

## Gate position confirmation

Gate x-positions in `route.yaml` are **estimates** from the satellite image.
The official positions will be announced 48 hrs before race (by Apr 8, 2026).

Once official positions are known:
- Edit `config/route.yaml` directly (SSH: `nano ~/mobot/config/route.yaml`)
- OR use the route editor UI (gate coords are not yet editable in the UI — edit YAML directly)
- Params hot-reload — no restart needed
