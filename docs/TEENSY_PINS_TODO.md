# TODO: Verify Teensy 4.1 Pin Wiring

Firmware was updated to fix A18/A19 (don't exist on Teensy 4.1).
Sensor channels 10–15 now use A10–A15. **Verify physical wiring matches.**

---

## Required Wiring

### Digital Control Pins (QTRX emitter enable + dimming)

| Firmware Pin | Teensy 4.1 Label | Physical Pin | Wire goes to        | Verified? |
|---|---|---|---|---|
| Pin 2 (LEDON_ODD)  | `2`  | Pin 4  | QTRX LEDON_ODD  | [ ] |
| Pin 3 (LEDON_EVEN) | `3`  | Pin 5  | QTRX LEDON_EVEN | [ ] |
| Pin 4 (CTRL_ODD)   | `4`  | Pin 6  | QTRX CTRL_ODD   | [ ] |
| Pin 5 (CTRL_EVEN)  | `5`  | Pin 7  | QTRX CTRL_EVEN  | [ ] |

### Analog Sensor Pins (QTRX sensor outputs → Teensy ADC)

Sensor order in firmware: Ch 0 = leftmost, Ch 15 = rightmost (verify orientation).

| Sensor Ch | Firmware | Teensy Label | Physical Pin | QTRX Out # | Verified? |
|---|---|---|---|---|---|
| 0  | A9  | `A9`  | Pin 23 | ___ | [ ] |
| 1  | A8  | `A8`  | Pin 22 | ___ | [ ] |
| 2  | A7  | `A7`  | Pin 21 | ___ | [ ] |
| 3  | A6  | `A6`  | Pin 20 | ___ | [ ] |
| 4  | A5  | `A5`  | Pin 19 | ___ | [ ] |
| 5  | A4  | `A4`  | Pin 18 | ___ | [ ] |
| 6  | A3  | `A3`  | Pin 17 | ___ | [ ] |
| 7  | A2  | `A2`  | Pin 16 | ___ | [ ] |
| 8  | A1  | `A1`  | Pin 15 | ___ | [ ] |
| 9  | A0  | `A0`  | Pin 14 | ___ | [ ] |
| 10 | A10 | `A10` | Pin 24 | ___ | [ ] |
| 11 | A11 | `A11` | Pin 25 | ___ | [ ] |
| 12 | A12 | `A12` | Pin 26 | ___ | [ ] |
| 13 | A13 | `A13` | Pin 27 | ___ | [ ] |
| 14 | A14 | `A14` | Pin 38 | ___ | [ ] |
| 15 | A15 | `A15` | Pin 39 | ___ | [ ] |

### Power

| Net    | Teensy Pin | Source      | Verified? |
|---|---|---|---|
| 3.3V   | 3.3V       | Teensy reg  | [ ] |
| GND    | GND        | Common GND  | [ ] |
| 5V/VIN | VIN        | VESC BEC or USB | [ ] |

---

## How to Verify

**Option A — visual check:**
Trace each wire from the QTRX connector to the Teensy pin and tick the table above.

**Option B — software check:**
Run sensor debug in raw mode and cover each sensor one at a time with your finger.
The channel that changes should match the channel number in the table.

```bash
python3 debug/sensor_debug.py
```

Cover sensor 0 (leftmost physical sensor) → Ch 0 Raw should change.
Cover sensor 15 (rightmost) → Ch 15 Raw should change.
If they're swapped or out of order, update `SENSOR_PINS[]` in `teensy/main.cpp`.

---

## Known Issue Fixed

The original firmware listed `A14–A19` for channels 10–15.
`A18` and `A19` do not exist on Teensy 4.1. They were replaced with `A10–A15`.
**If your QTRX was wired to the old A14–A17 physical pins, channels 10–13
will still work but channels 14–15 (old A18/A19) need to be re-wired to
Teensy pins A14 (pin 38) and A15 (pin 39).**
