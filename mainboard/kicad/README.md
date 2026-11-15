# Mobot main board — KiCad 10 project

Open `mobot_mainboard.kicad_pro` in KiCad 10. Everything is on **one large sheet** (custom 1220 x 850 mm page),
grouped into titled blocks:

- Row 1 (power): Battery In / ESC Out, 5V regulator (TPS54360B), 3.3V regulator (TPSM33620S3), battery voltage sense / LED / flags, 4 GND mounting holes + rail test pads
- Row 2: ESP32-S3 + buttons, USB, status LEDs, S3 test pads, IMU island (supply, ISM6HG256X, SCH16T-K01, heater)
- Row 3: ESP32-P4 module (C6 unused), microSD, camera FPC + trigger, PWM headers, encoder Qwiic

Style follows the ESP32 CAN+IMU board: blue block boxes with titles and power symbols for +3.3V/+5V/GND.
Regulators, the heater, every decoupling cap and the microSD pull-ups are **drawn with wires**; global labels
are used only for connections between blocks (block inputs/outputs).

Project libraries: `mobot.kicad_sym` (SCH16T-K01, Waveshare ESP32-P4-Module) and `mobot.pretty`
(TPS54360B DDA8, TPSM33620 RDN0011B, ISM6HG256X LGA14, SCH16T SOIC-24 *without* center pad,
Waveshare P4 module from Waveshare's DXF, G-Switch GT-TC048D-H025-L1 for RESET/BOOT; START uses the stock Cherry MX 1.00u footprint). STEP models in `3d/`.

`generator/` holds the scripts that produced it (`python3 -I generator/design.py <out_dir>`); they also report
estimated text/label overlaps. They overwrite the sheet, so **don't rerun them once the schematic has been edited by hand**.
