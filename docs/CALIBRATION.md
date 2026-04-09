# Sensor Calibration Guide

Calibration teaches the Teensy the min (pavement) and max (white line) ADC value
for each of the 16 sensor channels. Without calibration, all normalized values are
zero and the robot cannot detect the line.

Calibration is stored in Teensy EEPROM and survives power cycles.
Re-calibrate before each heat if lighting conditions have changed.

---

## When to Calibrate

| Situation | Action |
|---|---|
| First run of the day | Full drive-through calibration |
| Between heats, same lighting | Skip unless confidence looks low |
| Lighting changed (clouds, sun angle) | Re-calibrate |
| Sensor height or angle adjusted | Re-calibrate |
| Robot moved to a different surface | Re-calibrate |

---

## Quick Check Before Calibrating

Verify the sensor is reading live data:

```bash
python3 debug/sensor_debug.py --port /dev/serial/by-id/usb-Teensyduino_USB_Serial_14421340-if00
```

You should see the Raw row changing as you move the robot. If all values are 0
or frozen, check the Teensy USB connection.

---

## Method 1 — Stationary (bench, < 1 minute)

Best for quick sanity checks. Less accurate than drive-through.

```bash
python3 debug/calibrate_sensor.py --port /dev/serial/by-id/usb-Teensyduino_USB_Serial_14421340-if00
```

Steps the script walks you through:
1. Hold robot stationary over **bare pavement** for 3 seconds
2. Slide robot slowly **across the full line width** for 5 seconds
3. Script saves to EEPROM automatically

Limitation: only captures sensors that pass over the line during the slide.
If any channels show Span < 200, use Method 2.

---

## Method 2 — Push by Hand (recommended, ~3 minutes)

Push the robot slowly over the **entire course** by hand. Every channel sees
real line and real pavement from every course segment.

```bash
python3 debug/calibrate_drive.py --port /dev/serial/by-id/usb-Teensyduino_USB_Serial_14421340-if00 --push
```

What to do:
1. Start at the beginning of the course
2. Push the robot at walking pace along every segment, through all gates
3. Watch the Span row — all 16 channels should reach > 200
4. If any channels are low, push back over that segment again
5. Press **Enter** to save to EEPROM

The Span bar `[▁▂▃▄▅▆▇█]` shows contrast built up per channel.
A full bar `█` means that channel has seen clear line/pavement contrast.

---

## Method 3 — Motor Drive (geartrain not backdrivable)

Drives the robot at ~4% duty cycle (~creep speed) while recording calibration.
Does **not** auto-save — you review contrast and confirm before saving.

```bash
python3 debug/calibrate_drive.py \
  --port /dev/serial/by-id/usb-Teensyduino_USB_Serial_14421340-if00 \
  --vesc /dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00 \
  --drive --duty 0.04
```

- Increase `--duty` if the robot stalls (try 0.05–0.08)
- Keep duty low enough that the robot stays on the line — steering is not active
- Press **Ctrl+C** when the robot finishes the course
- Type **y** at the prompt to save, **n** to discard

---

## What Good Calibration Looks Like

After calibration, run `sensor_debug.py` and place the robot on the line:

```
Chan:    0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15
Raw:   110  120  890 3400 3750 3700 3200  780  130  115  108  112  119  110  122  115
Norm:    0    0  200  900 1000  980  750  150    0    0    0    0    0    0    0    0
[·············▃▇██▇▅▁·········]
Pos: +0.0123  Conf: 187
```

Good signs:
- Norm values hit ~1000 when sensor is over the line
- Norm values hit 0 when sensor is over bare pavement
- Conf > 100 with sensor centered on line
- Pos near 0.0 when robot is centered

Bad signs:
- Norm never exceeds 300 → re-calibrate, sensor may not have seen the line
- All channels show ~500 → min/max are too close, check sensor height (should be ~3mm)
- Conf < 50 on line → lighting changed, re-calibrate

---

## Saving Calibration Snapshots

After any successful calibration run, save a snapshot to the Pi immediately
while conditions are unchanged:

```bash
python3 debug/cal_manager.py save \
  --port /dev/serial/by-id/usb-Teensyduino_USB_Serial_14421340-if00 \
  --label "race_morning_heat1" --notes "overcast, 10am"

# List all saved snapshots:
python3 debug/cal_manager.py list

# Inspect a snapshot:
python3 debug/cal_manager.py show --label "race_morning_heat1"

# Delete a snapshot:
python3 debug/cal_manager.py delete --label "race_morning_heat1"
```

Snapshots are JSON files in `calibrations/`. They are records only — the
Teensy always uses whatever is in its EEPROM. If a heat goes badly, check
your snapshots to see which calibration run had the best contrast and
re-run calibrate_drive.py in those same conditions.

---

## Race Morning Checklist

- [ ] Robot is at the start line in race-day lighting (outdoors, ~same time of day)
- [ ] Run Method 2 (push) or Method 3 (drive) over the full course
- [ ] All 16 channels show Span > 200
- [ ] Run `sensor_debug.py` — confirm Conf > 100 with sensor on line
- [ ] Save a snapshot: `python3 debug/cal_manager.py save --label "race_morning"`
- [ ] If a heat goes badly, check `cal_manager.py list` and re-run calibration in the conditions that gave best contrast

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Span stays 0 on all channels | Teensy not in cal mode / not streaming | Check USB connection, rerun script |
| Some channels always 0 span | Sensor not wired or broken | Check ADC pin wiring on Teensy |
| Conf drops to 0 mid-run | Line lost or sensor too high | Check sensor height (3mm), re-calibrate |
| Robot drifts off line immediately | Bad calibration or wrong lighting | Re-calibrate in current conditions |
| "low-contrast channels" warning | Those sensors never saw the line | Push over more of the course |
