# Mobot main board plan

Nov 15, 2026 · @Cole Herber

## Overview

Everything lives on one 4-layer PCB with two processors: an ESP32-S3 that steers and drives, and an ESP32-P4 that sees. The board sits on a Tamiya TT-02R with four-wheel steering (one servo per axle) and a Hobbywing QuicRun 10BL120 G2 ESC driving a Justock 3650 motor.

The target is the Mobot course record of 33.99 s through 14 gates over roughly 255 ft, about 2.3 m/s average, with 3–4 m/s peaks between forks. At those speeds the bottleneck is sensing and control, not motor power, so the board is built around a fast global-shutter camera and a precision gyro.

One board instead of a stack removes inter-board cables near the ESC, gives one continuous ground plane, and turns UART, sync and IMU buses into short traces. The S3/P4 split still keeps control independent: if vision crashes, the S3 can slow and stop on its own.

&#91;embedded content: board architecture · two processors, IMU island, power\]

The S3 owns the sensors and actuators and keeps driving if vision fails. The same sync pulse that links the two chips also triggers the camera and the SCH16T.

## Compute split

The ESP32-S3 owns everything safety-critical: steering and speed loops at 500 Hz–1 kHz, PWM to the ESC and both steering servos, the wheel encoder, all three IMUs, the heater loop and the watchdog. It is lightly loaded, so IMU sampling and control timing stay regular.

The ESP32-P4 (a Waveshare ESP32-P4-Core module, 360 MHz dual-core RISC-V, 32 MB PSRAM in package) runs the camera, line fitting, fork logic, map matching and SD logging. Vision load spikes and PSRAM traffic cannot delay steering because they run on a different chip.

The two talk over a 2–4 Mbaud UART with DMA, COBS framing and CRC-16. The P4 sends curve parameters, confidence and branch choice with each frame's capture timestamp; the S3 sends back calibrated attitude and rates at about 1 kHz plus telemetry. A single hardware sync pulse from the S3 goes to the camera trigger, the SCH16T SYNC input and a P4 timestamp pin, so every sample shares one clock.

The S3 propagates each curve forward by the \~10 ms vision delay using its own gyro and odometry before steering on it. If valid curve packets stop for about 50 ms, it slows down on the predicted curve, then stops. There is no SPI snoop: the UART stream already carries calibrated data with sub-millisecond delay.

## Vision

The camera is an OmniVision OV9281: 1 MP, monochrome, global shutter, on the P4's 2-lane MIPI CSI through a short FPC from a board-edge connector. Global shutter means yaw and bumps don't skew the frame, so each frame needs one timestamp and one attitude sample. Espressif's esp\_cam\_sensor component already includes an OV9281 MIPI mono driver.

The plan is a cropped, binned mode around 150–210 fps with a \~130° M12 lens. A near band (0.3–0.6 m ahead) steers; a far band (1–3 m) spots forks and curve entries about 0.5 s early at 4 m/s. Each detected line point is projected to the ground using a fisheye lens model and the S3's attitude, which removes the pitch error that dominates at long range.

Compute stays small because the P4 only searches a window around the curve predicted from the last frame plus gyro and odometry. A robust clothoid fit, fork handling and map matching come to an estimated 25–40% of one core, with about 10 ms from frame to curve. A pulsed 850 nm IR LED with an IR-pass filter keeps exposure short in sun and shade.

## IMU stack and why

The stack is one Murata SCH16T-K01 as the precision gyro plus two TDK ICM-45686s as wide-range backups, all raw sensors fused by the S3. It replaces the WitMotion WT901, whose raw gyro bias is ±0.5–1 °/s and whose heading relies on a magnetometer.

| Sensor | Role | Key spec | Interface |
| --- | --- | --- | --- |
| [SCH16T-K01](https://www.murata.com/products/sensor/gyro/overview/lineup/sch16t) | Primary gyro + accel | 0.5 °/h bias instability, 0.015 °/√h ARW, ±300 °/s | SPI A (SafeSPI with CRC) |
| [ICM-45686](https://adm.invensense.tdk.com/icm-45686) ×2 | High-rate backup, fault check | 3.8 mdps/√Hz noise, up to ±4000 °/s | SPI B, two chip-selects |

The SCH16T is the reason heading holds: its bias instability is thousands of times smaller than the WT901's raw bias, so gyro-only heading drifts well under a degree per run and odometry position error is set mostly by wheel slip. Murata also cites strong vibration rectification, which matters on a bumpy sidewalk.

The ICM-45686s exist because the K01 saturates at about 300 °/s, and four-wheel steering in counter-phase pushes cornering yaw to an estimated 330–390 °/s. When the SCH16T's per-axis saturation flag sets, the S3 switches that axis to the ICMs and switches back below \~250 °/s. Using a second vendor also decorrelates temperature and vibration errors, and three gyros let the S3 spot a faulty one.

There is no magnetometer. The brushless rotor's magnets, ESC current and rebar near the course would swamp a 0.05–0.3° sensor with multi-degree errors. Starting heading comes from the camera's first view of the line, and map matching holds it relative to the course after that. The footprint stays unpopulated for future outdoor robots.

## IMU island on the main board

The IMUs sit on the main PCB, not a daughterboard, in a thermal island in the quietest corner, far from the buck regulator and the P4. Routed slots on three sides cut heat flow and board stress; thin bridges carry the traces. The whole board mounts on soft dampers near the car's center of rotation, with flexible leads so cables don't carry vibration in.

- **Power:** a dedicated low-noise 3.3 V LDO inside the island, fed from the 5 V rail through a ferrite and capacitors. Gyro bias shifts slightly with supply voltage, so the IMUs never share a rail with the P4.
- **Heater:** resistors placed symmetrically around the SCH16T, switched by a logic-level MOSFET from the S3 and powered from the battery or 5 V, never through the LDO. A PI loop holds the die at about 50–60 °C, with a foam cap over the island and a firmware cutoff around 80 °C.
- **Ground:** an unbroken plane under the island; ESC, servo and P4 return currents are routed so they don't pass under it.

Calibration on the assembled board, with the heater at its setpoint: an overnight Allan variance log; a 12–24 position fit for accelerometer bias, scale and misalignment; gyro bias and g-sensitivity from the same data after subtracting Earth's rotation; gyro scale factor on a DIY turntable over whole revolutions; and a temperature sweep. Scale factor matters most for this car because it turns so much: 100–500 ppm of error is 0.7–3.6° over twenty full turns.

## Power and actuation

Power comes from a 2S hard-case shorty LiPo of 4000–5000 mAh in a TT-02 shorty holder. A race run uses well under 1 Wh, so the pack is chosen for weight and balance, not capacity.

```
2S battery ──┬── ESC → motor
             ├── front + rear HV servos (direct, bulk cap at the split)
             ├── IMU heater MOSFET
             └── 5 V buck ──┬── 3.3 V buck → S3, P4 Core, encoder
                            └── ferrite → low-noise LDO → IMU island
```

Both steering servos are HV-rated and run straight from the battery. The S3 sends interpolated, rate-limited position setpoints capped at the servos' worst-case speed at low voltage, so steering response stays the same from a full pack to a sagged one. A 1000–2200 µF low-ESR capacitor sits where the two servo leads split. The ESC's BEC (6/7.4 V, 4 A) is not used for servos, since two servos can exceed 4 A when stalled.

The 5 V buck must keep regulating down to about 5 V input so the processors don't reset during launch sag. The encoder (MT6701 or AS5047P) wires straight to the S3 over SPI or ABZ, and an unpowered fifth wheel or optical flow sensor is the next step for slip-free odometry. Servo, ESC and battery returns go straight back to the battery and meet logic ground at one point.

## Logging and open items

The P4 logs to a UHS-I A2/V30 microSD card. Every frame's curve, confidence, IMU state, odometry and timestamps are always logged (about 10 KB/s). The last \~2 s of full-rate frames stay in a PSRAM ring buffer and are saved only when something goes wrong, because sustained SD writes on the P4 are only around 5–15 MB/s. A low-priority writer drops log data rather than stalling vision.

Still to verify before layout:

- [ ] The Waveshare Core's castellated pinout for the CSI lanes and SD slot-0 pins, and how SD\_VREF is wired
- [ ] Which frame-rate and crop presets the OV9281 driver ships with, and whether it is Arducam-specific
- [ ] SCH16T-K01-10 suffix meaning, US stock and lead time; whether a K20 is available
- [ ] Rear-steering parts for the TT-02R (knuckles, driveshafts, servo mount)
- [ ] Servo stall current and measured worst-case speed under load
- [ ] SCH16T supply current, SPI mode and maximum SPI clock

GPIO assignments for both chips come next, once the Waveshare pinout is confirmed.
