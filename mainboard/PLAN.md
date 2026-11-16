# Mobot main board — design plan

Nov 15, 2026 · companion to `../Mobot main board plan.md` (architecture) · reuse source: `~/Desktop/Rolly/Esp32 Can+IMU/` · **KiCad 10** · **fab: OSH Park only**

This document turns the architecture into a buildable board. It covers what carries over from the ESP32 CAN+IMU board, the decisions made so far, the open items with my recommendation on each, and the design phases. Datasheets go in `mainboard/datasheets/`, and its `README.md` is the checklist.

---

## 0. Decisions so far

| # | Topic | Decision |
|---|---|---|
| D1 | Backup IMU | SCH16T-K01 + **one ISM6HG256X**, no second footprint |
| D2 | Heater element | Symmetric discrete resistors, not a trace heater |
| D4 | SCH16T temperature output | Use it for compensation and heater feedback; safety is in firmware (no hardware cutoff) |
| D6 | CAN | Footprints only (DNP, bottom side) |
| D7 | P4 programming | **Through the S3**: a USB bridge plus store-and-forward. No P4 USB-C |
| D8 | Power out | Servos powered from VBAT through board headers (assumed 2S-safe). ESC powered through an **XT30 output on the board** |
| D9 | Camera | **Arducam OV9281 1 MP global-shutter NoIR mono, 130° M12, Raspberry Pi FPC** |
| D10 | Encoder | Adafruit AS5600 STEMMA QT on the CAN board's Qwiic circuit |
| D12 | Fab / copper | OSH Park 4-layer: 1 oz outer, ½ oz inner. No heavy copper |
| — | IMU LDO | **AP2112K-3.3** (yours, on hand), fed from 5 V through a ferrite |
| — | EDA | KiCad 10 |
| D11 | Servo | goBILDA 5-turn kept for now. **Must be reprogrammed** for steering (deferred) |
| D13 | USB ESD | USBLC6-2 footprint left **DNP** |
| D14 | Pogo pins | Removed. S3 BOOT/RESET buttons + bare test pads instead |
| — | SCH16T footprint | Center pad (pad 25) deleted, per Murata Fig. 6 |
| — | P4 module | **Waveshare ESP32-P4-Module**: ESP32-P4NRW32 (32 MB PSRAM) + 16 MB flash + **ESP32-C6 co-processor** for Wi-Fi 6/BT over SDIO. 88 pads, **3.3 V only** |
| — | Reflow | Available, so LGA/QFN/module parts are fine |
| — | S3 module | **ESP32-S3-WROOM-1-N8** (8 MB flash, no PSRAM). Fine with the pin plan |
| D15 | Role of the C6 | **Option A**: the S3 stays the control MCU, the C6 gives the P4 Wi-Fi, the S3's Wi-Fi is off. **The board must add the C6's antenna** (D16) |
| D5 | 5 V regulator | **Keep the TPS54360B** (parts on hand). Re-derive its UVLO/FB resistors for 2S |

Still open: D3 (heater at all?), D5 (power-topology details). Deferred: D11 (servo reprogramming, revisit after the board).

---

## 1. Reuse from the ESP32 CAN+IMU board

| Block | CAN+IMU part (ref) | Mobot use | Action |
|---|---|---|---|
| Control MCU | ESP32-S3-WROOM-1-**N8** (U1) | S3 control processor + P4 programmer | Keep. N8 has no PSRAM, so GPIO35–37 stay free for IMU SPI. |
| Backup IMU | ISM6HG256X (U2) on IO35/36/37/21/38/39 + C4/C5/C8/C9 | Wide-range backup gyro (D1) | Keep as-is: footprint, STEP, `imu.cpp`, SPI debug procedure. |
| 5 V buck | TPS54360B-Q1 (U4), L1, C6/C7/C11, Schottkys | 5 V rail (IMU LDO + heater) | **Keep** (parts on hand). **Re-derive the UVLO and FB resistors** (below). |
| 3.3 V buck | TPSM33620-Q1 (U5) | 3.3 V logic | Keep, but **feed it from VBAT**. Its 3–36 V input range means logic survives deep sag. 2.2 MHz fixed, syncable 1.9–2.5 MHz. |
| USB-C | JAE DX07S016 (J17), 5.1 k CC | The only USB port: S3, plus a bridge to the P4 (D7) | Keep the connector. **USBLC6-2 (U7) becomes a DNP footprint** (D13). |
| Battery in | XT30 (XT30F1/M1), F1, D_TVS1 | Battery input + **XT30 ESC output** | Keep the XT30s. **Swap the TVS to SMBJ12CA**: SMBJ58A clamps above 60 V and does nothing on 2S. **No fuse** (removed per your call). |
| PWM out | IO11–14 → RN1 33 Ω → 3-pin headers | ESC signal, front servo, rear servo, spare | Keep. Servo header pin 2 = **VBAT**. The ESC signal header pin 2 is **no-connect**, because the ESC BEC must not back-feed. |
| ADC dividers | RN4 3.9 k / RN3 2.2 k (9.1 V FS) | Battery voltage | Keep one or two channels. |
| Qwiic I²C | J11, R18/R19 4.7 k, IO8/9 | AS5600 encoder, alone on the bus (D10) | Keep as-is. Add a second Qwiic on I2C1 (IO16/17) for expansion. |
| UART1 | IO41/42 | S3↔P4 link **and** P4 flashing path (D7) | Keep. |
| Pogo pads | J3 (prog/reset/boot), J5 (IMU SPI tap) | — | **Removed** (D14). Replaced by S3 BOOT + RESET buttons and bare test pads. |
| LEDs | WS2812B-2020, PWR/LIFE/FAULT | Status | Keep. |
| Buttons | PTS526 | S3 BOOT, S3 RESET (PTS526); **START = Cherry MX keyboard switch** | BOOT + RESET replace the pogo programming pins. |
| CAN | SN65HVD232 (U3), JST-GH ×2 | U9, CAN0/CAN1, R36 + CAN_TERM terminator | **Copied, all DNP on B.Cu** (D6). |
| BOM fields | LCSC numbers | — | Replace with **DigiKey/Mouser** part numbers. OSH Park doesn't assemble (D12). |

**About copper weight:** you're right, and my "2 oz" was wrong. I took it from the header line in `CIRCUIT_DIAGRAM.md` ("4-layer, 2-oz Cu") without checking the PCB. The `.kicad_pcb` stackup shows about 1 oz outer. It's moot anyway, because OSH Park 4-layer is 1 oz outer / ½ oz inner with no heavy-copper option. See D8 for widths.

### Power warning: the TPS54360B resistor values are for a different pack

**Corrected after reading the CAN schematic:** the EN/UVLO divider is R1 = 174 k / **R2 = 35.3 k** (R4 13 k is compensation). Using the datasheet equations (I1 = 1.2 µA, IHYS = 3.4 µA), it starts at **≈ 6.9 V and stops at ≈ 6.3 V**. That's too high for 2S, because the IMU rail would drop out during sag. The new schematic uses **R2 = 43.2 k → start ≈ 5.8 V / stop ≈ 5.2 V**. The FB divider (59 k / 10.2 k → 5.43 V) is **intentional**: diode D5/D5 after the regulator brings +5V down to ≈ 5.0 V, so it stays as-is.

### Rail tree (updated)

```
2S LiPo ── XT30 in ──┬───────────────────────────── XT30 out → ESC (unfused on board; short wide pours)
                     ├── TVS (SMBJ10A/12A) + input bulk
                     └───────┬── servo VBAT bus ── 4×22µF MLCC ───┬── front servo header (pin 2)
                              │                                    └── rear servo header (pin 2)
                              ├── VBAT_SENSE divider → S3 ADC
                              ├── TPSM33620S3 → 3.3 V (2 A) ──┬── P4 module ESP_3V3 + VBAT pads (P4 + PSRAM + C6 Wi-Fi)
                              │                               ├── S3 module
                              │                               ├── camera FPC 3.3 V, encoder Qwiic, LEDs
                              │                               └── (P4 internal LDO VO4 → SD card I/O domain, GPIO39–48)
                              ├── IR LED driver (VBAT, pulsed)
                              └── TPS54360B → 5 V ──┬── heater (MOSFET, filtered PWM)
                                                    └── ferrite → AP2112K-3.3 → IMU island (SCH16T + ISM6HG256X)
```

**IMU LDO (the AP2112K-3.3 you have on hand):**
- Input range is 1.2–6 V (abs. max 7 V), so it **must come from 5 V, not VBAT**.
- Dropout is 250 mV at 600 mA. The island draws ~45 mA (the SCH16T alone is 36–47 mA), so headroom is large even with 5 V sagging to ~4.5 V.
- PSRR is ~75 dB at 1 kHz. It isn't an ultra-low-noise part, but that's fine here: the SCH16T regulates its own analog and digital cores internally (VREGA 2.5 V, VREGD 1.5 V), so the external LDO's job is isolating the island from P4/heater load steps on 5 V. It does that.
- Put a ferrite plus 10 µF before it and the datasheet's output capacitor after it, both inside the island.

---

## 2. Decisions and tradeoffs

### D1. Backup IMU — decided: SCH16T + one ISM6HG256X
The SCH16T saturates at ±300 °/s (dynamic range ±327.68 °/s per the datasheet), and counter-phase 4WS may reach 330–390 °/s, so the backup only covers those saturated slices. One well-calibrated backup is enough: a second identical part improves noise, not scale-factor residuals, and it shares temperature and vibration errors with the first. The ISM6HG256X is the lowest-effort choice: proven footprint and driver, ±4000 dps, 4.5 mdps/√Hz, 3 °/h bias instability, a second vendor, and it gets a ±256 g crash channel for free. Before layout, run the course map through a yaw-rate profile (v²/R). If the peak stays under about 280 °/s, saturation never happens.

### D2. Heater — decided: symmetric discrete resistors
Sizing: ~1–2.5 W to hold ~55 °C. Design for 2.5 W at 5 V → ~10 Ω (e.g. 8 × 82 Ω, 0.5 W-rated 1206/2010 in parallel), placed point-symmetric about the SCH16T on the bottom side. The island's solid ground pour spreads the heat.

Why not a trace heater:
- It needs ~1 m of 0.1 mm trace, a whole inner layer under the IMUs.
- Its resistance varies ±15–25 % board to board.
- It puts 0.5 A directly under the gyro.

Notes:
- The SCH16T dissipates ~135 mW by itself (41 mA × 3.3 V), which helps.
- PWM the heater at ≥ 20 kHz with filtering, or at a few Hz. Don't use 100 Hz–2 kHz.
- Route heater supply and return as a pair straight to the 5 V source.

### D3. Heater at all? (open)
A 34 s run sees little temperature drift. The heater's value is keeping bias the same from run to run and day to day, so indoor calibration still holds on race morning. **Rec:** populate it, but treat it as optional in firmware. The Phase 9 sweep decides whether temperature compensation alone is enough.

### D4. SCH16T temperature output — usable, now confirmed from the datasheet
The short datasheet, Table 8, gives:
- **100 LSB/°C** (°C = TEMP/100)
- range −50…135 °C
- **total error ±15 °C**
- **linearity ±1 °C**

That matches my expectation: fine resolution and good linearity, loose absolute accuracy. What matters for compensation and for a heater setpoint is repeatability, and the setpoint is "the die reading at calibration", not a true °C. So:
- **Use it** as the heater PI feedback (decimate to 10–50 Hz) and as the compensation variable. It measures the die, where the bias lives.
- **Don't trust ±15 °C absolute** when comparing against the ISM6HG256X die sensor (also ±15 °C) Learn each sensor's offset at calibration, then fault on changes.
- **No hardware cutoff (decision):** the comparator/NTC circuit was dropped. Protection is firmware: hardware watchdog (a reset releases HEATER_PWM and R21 turns the MOSFET off), a duty cap, and cross-checking the SCH16T die temperature against the ISM6HG256X's. A shorted MOSFET is not covered either way.
- The SCH16T's typical offset over temperature is already small (Z: ±0.01 °/s typ, ±0.1 °/s max over −40…110 °C; drift velocity ±0.01 (°/s)/min at 0.5 K/min). Compensation by itself may be enough (D3).

### D5. Power topology — settled by the module schematic
The Waveshare module runs entirely from **3.3 V**: ESP_3V3 pads 85/86, with its own MP1605 buck inside making the P4's 1.2 V core. Its VBAT pad (84) is the P4's low-power domain and gets tied to 3.3 V. So the 3.3 V rail carries both processors, and **5 V is left with only the heater (~0.5 A) and the IMU LDO (~45 mA)**.

**Decision: keep the TPS54360B for 5 V** (you have the parts). It's oversized for ~0.55 A, but proven. Its UVLO divider must drop from ~17 V to ~5.5–6 V start, and its FB divider must be corrected from 5.43 V to 5.0 V (Power warning above). In dropout it runs at 100 % duty, so 5 V tracks VBAT during a sag.

**3.3 V budget (to finalize in Phase 1):**

| Load | Estimate |
|---|---|
| P4 module (P4 at full load + PSRAM + C6 Wi-Fi TX peaks) | ~0.6–0.9 A peak |
| S3 | ~0.1 A with Wi-Fi off, ~0.45 A peak with it on |
| Camera | ~0.1 A |

That fits the TPSM33620's 2 A with margin. Turning the S3's Wi-Fi off (D15) keeps it comfortable.

### D6. CAN — footprints only, all DNP on the bottom side.

The Rolly CAN block is copied as-is: SN65HVD232 (U9) + 100 nF (C43), two JST-GH 4-pin ports in parallel (CAN0/CAN1, DroneCAN pinout: 1 +5V left open, 2 CAN+, 3 CAN-, 4 GND), and the 120 R terminator (R36) behind solder jumper CAN_TERM. TX = IO16 (same as Rolly); RX moves to **IO1** because IO17 is now the START LED. Rolly's schematic left R36/JP1 pin 1 unconnected; here they share net CAN_TERM so the terminator actually works when CAN_TERM is bridged.

### D7. Programming the P4 through the S3 — yes, and recommended

**Short answer: no Wi-Fi is needed to flash anything.** One USB-C cable into the S3 flashes **both** chips:
- the S3 directly, over its own USB
- the P4 through the S3's USB-to-UART bridge

Wi-Fi OTA is an optional convenience on top. **The P4 doesn't update the S3 by default.** It could (see 3 below), but you don't need it.

How it works: the S3 controls the P4's reset (EN) and boot strap, and the S3↔P4 UART is wired to the **P4's UART0**, which its ROM bootloader listens on. Workflows:

1. **USB bridge (daily development).** S3 firmware runs its USB in OTG/TinyUSB mode as a composite device with two serial ports: port 1 is the S3's own console, and port 2 bridges to the P4 UART, with DTR/RTS mapped to the P4's EN/BOOT. On the laptop you run `idf.py -p <second port> flash monitor` for the P4 exactly as if it had its own USB. Espressif's `esp-usb-bridge` project does exactly this on an S3 and is the reference. The **S3 is flashed through the same USB-C**. If the S3 firmware ever breaks, holding BOOT at plug-in drops it into the ROM's USB download mode, which always works.
2. **Optional: over-the-air.** This goes through the **P4's own Wi-Fi**: the module's C6 runs Espressif's `esp_hosted` firmware and gives the P4 Wi-Fi over SDIO, so the P4 can OTA itself. That replaces the earlier S3 store-and-forward idea, which is why the S3 no longer needs 16 MB of flash. The S3 can still reflash the P4 with `esp-serial-flasher` as a fallback, but it doesn't have to hold an image.

3. **Optional: the P4 updates the S3 (wireless updates for both chips).** The P4 receives an S3 image over Wi-Fi and streams it over the existing UART to the S3. The S3 writes it into its second OTA app slot (N8 has room for two ~3.5 MB slots) and reboots. This is pure firmware: no extra hardware, add it later if wanted. The S3 keeps its rollback slot, so a bad image can't brick it.

Costs and details:
- **Pins:** two S3 GPIOs (P4 EN, P4 BOOT).
- **Speed:** ROM download starts at 115200 baud, then the flasher raises it (≥ 921600 to 2 Mbaud). A 1–2 MB vision image takes ~10–30 s.
- **Strapping, confirmed (P4 datasheet Table 3-3 + Waveshare pinout):**
  - P4 **UART0 = GPIO37 TX (pad 64) / GPIO38 RX (pad 65)**.
  - **Boot strap = GPIO35 (pad 62)**: low at reset → download mode. Needs **GPIO36 (pad 63) pulled up with 10 k**.
  - **EN = pad 87**.
  - GPIO37/38 are "any value" for boot mode, so the S3's UART idle level can't mis-strap the P4.
  - Hold time is 3 ms after EN rises before the S3 releases BOOT.
- **Recovery:** with the pogo pads gone, the P4 is reachable only through the S3. That's acceptable, because the S3 itself can always be recovered with BOOT held at plug-in (ROM USB download), then the bridge firmware reflashed. The P4's native USB D± are left unconnected.
- **Bulk data:** frame dumps and logs come off the P4 over **Wi-Fi** (C6) or by pulling the **microSD card**. The S3's USB bridge is only for flashing and the console.

### D8. Servo and ESC power through the board — decided
- **Servos:** 2S VBAT goes straight to servo header pin 2 (no fuse), with C39–C42 (4×22 µF 25 V X5R 1210) at the split. No board electrolytic: the ESC's own input cap, on XT30 out, provides the bulk and hot-plug damping; on the bench without the ESC the TVS clamps. One caution: goBILDA rates this servo **4.8–7.4 V**, and a fresh 2S pack is 8.4 V. Treated as compatible per your call.
- **ESC:** XT30 out, placed **right next to** the XT30 in, joined by short, wide pours on L1 *and* L4, stitched with many vias. Keep the ESC loop inside the power corner, with the star ground at the input connector.
- **XT30 rating:** 15 A continuous, **30 A for 1 min (< 80 °C)** per the Amass datasheet. A 34 s run fits inside the 1-minute rating if launch peaks stay under ~30 A. **I need the motor turn/kV and gearing, or better, a logged/clamped current**, to confirm. If peaks are higher, move battery-in and ESC-out to XT60.
- **Trace widths (1 oz outer):** you're right that 0.6 mm works for each servo branch. 3 A stall is intermittent, and IPC-2221 puts 3 A continuous at ~0.7 mm for a 30 °C rise, or ~1.4 mm for 10 °C. Since board area is free there, I'll draw servo feeds at ≥ 1 mm and the shared servo trunk (two stalls = 6 A) as a pour. ESC current only flows in the short in→out pour.
- **No fuse on the board** (decision). Battery voltage is sensed directly from the battery input (VBAT_SENSE, S3 IO4) for battery health.
- **4 mounting holes**, all tied to GND, double as GND test points.

### D9. Camera — decided: Arducam OV9281 NoIR mono, 130° M12, Pi FPC
**Why this one:**
- Mono global shutter at 1280 × 800, up to 120 fps full frame (more when cropped).
- 2-lane MIPI on the Raspberry Pi 15-pin 1.0 mm FPC that P4 boards use.
- **NoIR**: no IR-cut filter, so the 850 nm illumination works; you add an IR-pass filter in front.
- The 130° M12 lens matches the architecture's field of view, and M12 lets you swap lenses.
- Espressif's `esp_cam_sensor` has shipped an OV9281 MIPI driver since v2.6.0 (listed as 1280 × 720, RAW8 mono).

**Caveats:**
- The driver's listed mode is 1280 × 720. The 150–210 fps cropped and binned modes will probably need a custom register table, which is firmware work, not a board change.
- **Trigger:** Arducam's Pi boards bring the trigger out on a solder pad (XVS on most revisions; their newer UC-788 board differs), with a minimum 2 µs pulse. **There's no STROBE output on the connector.** The board therefore provides a 2-pin JST-SH "CAM_TRIG" next to the FPC, with a level-shift option (the Leopard OV9281 board runs FSIN at 1.8 V). Populate the shifter once Arducam's board schematic confirms the pad's level.
- Some Arducam listings show 1-lane for the M12 variant. Confirm 2-lane on the exact SKU you order.

**IR LED timing without STROBE:** the **P4** fires the LED. It receives the SYNC edge on a GPIO and uses its event-task matrix (ETM) to start a hardware one-shot timer of the programmed exposure length. The pulse is jitter-free with no CPU involved, and the P4 already owns the exposure setting. Driver: a low-side constant-current MOSFET stage from VBAT on the main board, far from the island, feeding a small LED board at the lens over a JST-GH lead.

Alternative if you want a STROBE pin and a datasheet-backed FSIN: Leopard Imaging LI-OV9281-MIPI-RS-85H. But it has an 85° lens, a different connector and more cost. I wouldn't.

### D10. Encoder — decided: Adafruit AS5600 STEMMA QT on J11
- **Bus:** I2C0 dedicated to the encoder at 400 kHz, read at 1 kHz (~15 % bus load).
- **Magnet location:** on a wheel or the prop shaft, **not the motor**.
- **Filter:** set the AS5600's built-in filter to its fastest setting.
- **Protection:** an I²C timeout, bus recovery and a fallback in firmware. No ESD array; the same reasoning as D13 applies.
- **Breakout:** has its own regulator; per a reseller listing, also 10 k pull-ups, which in parallel with ours gives ≈ 3.2 k.

### D11. Servo — goBILDA 2000-0025-0502 (5-turn) — **deferred: reprogram later**

> **NOTE — revisit after the board is done:** the servo will have to be **reprogrammed** for steering before it's usable on Mobot. Out of the box it is a 5-turn servo (0.90°/µs), which is far too coarse for steering. goBILDA's servos are programmed with the **3102-0001-0001 Dual Mode Servo Programmer**. Check what it can change (travel range, deadband, mode), reprogram the servo to a short travel range, and re-measure deadband and update rate on the bench. If reprogramming can't get the resolution down, switch to the 300° version or a fast car servo. No board impact either way: same 3-pin header, same VBAT.

Why, from goBILDA's spec table:
From goBILDA's spec table:

| Spec | Value |
|---|---|
| Rotation | 1800° over 500–2500 µs, so **0.90°/µs** |
| Deadband | **4 µs**, which is **3.6° at the output shaft** |
| Speed | 0.17 s/60° @ 7.4 V |
| PWM frequency | 50 Hz |
| Stall current | 3.0 A @ 7.4 V |

For steering you only use a fraction of a turn: ±40° of servo travel is just ±44 µs of pulse, about **22 deadband steps across the whole steering range**. A line follower at 4 m/s will hunt inside that deadband. The 50 Hz update also adds up to 20 ms of steering latency to a 500 Hz–1 kHz loop.

Fallbacks if reprogramming isn't enough: goBILDA's standard-range (300°) version of the same servo, or a fast digital car servo that accepts 333 Hz.

### D12. Fab — decided: OSH Park 4-layer
- **Stackup:** 1 oz outer / ½ oz inner, FR408HR, 1.6 mm, ENIG, purple mask. Minimum 5/5 mil trace/space on 4-layer. Internal routed slots are fine for the IMU island.
- **Impedance:** no controlled-impedance service. Calculate MIPI (100 Ω diff) and USB (90 Ω diff) geometry from OSH Park's published stackup. Community sources disagree on the prepreg (~6.7–8 mil), so I'll use the **current official page**. Short MIPI runs at < 1 Gbps/lane tolerate a few ohms of error.
- **Cost:** priced per square inch for 3 boards, so the floorplan should stay compact.
- **No assembly**, so this becomes a hand-assembly board. Order an OSH Stencils stencil and reflow with a hot plate or oven. Parts that **can't** be done with an iron:
  - ISM6HG256X (LGA-14)
  - TPSM33620 (QFN module)
  - P4 module (88 pads at 0.7 mm; reflow if pads are bottom-only)
  - USB-C mid-mount
  - TPS54360B (exposed pad)

  The SCH16T (SOIC-24, 0.8 mm) is iron-friendly. **Do you have access to a reflow oven or hot plate?**

### D13. USB ESD chip (USBLC6-2) — DNP footprint
You're right that it isn't essential:
- The S3's USB pins have the chip's built-in ESD protection.
- This port only gets plugged in at the bench, never in the field.
- The USBLC6-2 has a real failure mode on a hand-built board: its VBUS pin (5) sits next to GND (2), and a bridge or a dead part shorts the USB 5 V rail.

The one argument for it: this USB-C is now the **only** way into both chips (D7), so a zapped S3 USB PHY means losing programming for both.

**Decision:** keep the footprint on the D± lines but **leave it unpopulated**. It's a shunt clamp, so the data lines run straight through with nothing fitted; no jumpers are needed, and populating it later is a rework, not a respin. The same logic dropped the Qwiic ESD array.

### D14. Pogo pins — removed
J3 and J5 are gone. Replacements:
- **S3 BOOT + RESET tactile buttons.** Needed for ROM USB recovery, which is the backstop for the whole S3-programs-P4 scheme.
- **Bare SMD test pads** (no part, no cost) on each rail, the S3 UART0 console, SYNC, and one pad per SPI bus line, for scope and logic-analyzer clips.

No P4 recovery header (see D7).

### D15. The module's built-in ESP32-C6 — do we still need the S3?
What the module schematic shows about the C6 (an ESP32-C6 with 8 MB flash in package):
- **Link to the P4:** SDIO on P4 GPIO14–19, which aren't on the pads.
- **P4 controls it:** P4 **GPIO54 drives C6 CHIP_PU** (its reset), and P4 **GPIO6 ↔ C6_IO2** is the host-wake/handshake line.
- **Exposed on pads:** only **9 C6 GPIOs** (IO4–9, IO13–15; IO8/IO9 are its boot straps) plus its UART0.

| Option | What it means | Verdict |
|---|---|---|
| **A. Keep the S3 for control; the C6 does the P4's Wi-Fi** (Rec) | The C6 runs `esp_hosted` as Waveshare intends. The P4 gets Wi-Fi for OTA, telemetry and fast log/frame download. **The S3's own Wi-Fi is switched off** (less RF, less 3.3 V current, no Wi-Fi interrupts near the 1 kHz loop). | Keeps the architecture's key property: a vision crash can't stop steering. Firmware for both chips stays standard. |
| B. Drop the S3; use the C6 as the control MCU | Steering, IMUs and actuators on the C6. | **No.** 9 pins isn't enough (the SCH16T SPI alone takes 6 with sync and reset, and we need ~17). It has one general-purpose SPI. It's a single high-performance core that would also be running the P4's Wi-Fi link. And **its reset is wired to the P4 (GPIO54)**, so a P4 crash or reboot can take steering down with it. |
| C. Drop the S3; the P4 does vision **and** control | Pin control to one P4 core and vision to the other. The P4 has ~40 free GPIOs, two general SPI buses, MCPWM, I²C and ADC. | Possible, and it removes a chip, the UART protocol and the S3↔P4 flashing scheme. But a vision-side panic reboots the whole chip, so steering is gone for hundreds of ms. Camera DMA and PSRAM traffic also share memory bandwidth with the control core, which is the timing jitter the two-chip split was designed to avoid. Worth it only if board space or cost forces it. |

**Rec: A.** The C6 isn't a substitute for the S3, but it changes three things in our favor:
1. The P4 can OTA itself, so the S3 doesn't need a big flash.
2. Big data (frame bursts) can leave over Wi-Fi.
3. The S3's Wi-Fi can be off.

**Module pins we must leave alone:**

| Pads | Reason |
|---|---|
| 21 (GPIO6), 82 (GPIO54) | C6 handshake and reset |
| 16 (GPIO1), 88 (GPIO0) | The module has a 32.768 kHz crystal on them |
| 34–39 (DSI) | No display |
| 48/49 (HS USB) | Unused |

**C6 firmware updates:** the P4 can update the C6's `esp_hosted` firmware over SDIO, which `esp_hosted` supports. For recovery, route C6_U0TXD/RXD (pads 5/4), C6_IO9 boot (pad 9) and C6_IO8 (pad 10, 10 k pull-up) to **bare test pads**. Same for P4 USB-Serial-JTAG (GPIO24/25, pads 50/51).

**SD card note:** the P4's SD slot-0 pins GPIO39–44 sit on the **VDD_IO_5 domain, powered by the P4's internal LDO VO4 (pad 66, 1.8 V by default)**. Firmware sets VO4 to 3.3 V for default-speed SD, or switches to 1.8 V for UHS-I. Anything else wired to GPIO39–48 shares that voltage, so I'll keep GPIO45–48 for SD card-detect or leave them unused.

### D16. C6 antenna — the board has to provide it
The module has **no antenna of its own**: the C6's RF output comes out on **LNA_OUT (pad 2)**, flanked by GND pads 1 and 3. Waveshare's schematic shows an IPEX option only behind an unpopulated 0 Ω on the module. Plan:
- **RF path on L1:** pad 2 → **π-match footprint** (0402: a series 0 Ω, and two shunt pads left DNP for tuning) → **U.FL / IPEX MHF1 connector** at the board edge → a 2.4 GHz dipole on a short pigtail, mounted outside the body shell or high on it.
- **Trace:** keep it short (< 10–15 mm) as a 50 Ω **grounded coplanar waveguide** over the solid L2 ground. Fence both sides with GND vias every ~1.5 mm. On OSH Park 4-layer (L1–L2 prepreg ≈ 6.7 mil), a tested 50 Ω CPWG recipe is ~13.4 mil trace with 6 mil gaps. I'll re-check against OSH Park's current stackup.
- **Keep-outs:** no copper pour or other traces near the RF section. Keep the module's pad 1–3 corner at the board edge, next to the connector.
- **Why not a chip antenna:** under a car body near the motor and ESC, an external antenna on a pigtail is far more predictable, and it costs nothing in board area.
- **S3 antenna:** the S3's Wi-Fi is off, so its PCB antenna doesn't strictly need Espressif's keep-out. I'll still place the WROOM-1 antenna at a board edge without copper under it, so S3 Wi-Fi/BLE stays usable for debugging.

---

## 3. Draft S3 pin plan (tentative — finalized in Phase 3)

| GPIO | Function | Note |
|---|---|---|
| 35/36/37/21 | ISM6HG256X MOSI/SCK/MISO/CS (SPI2, mode 3) | Unchanged from CAN board |
| 38/39 | ISM6HG256X INT1/INT2 | Unchanged |
| 5/6/7/10 | SCH16T SCK/MOSI/MISO/CS (SPI3, **mode 0**, ≤ 10.5 MHz; 25.5 MHz with MISO_HI_SPD) | Dedicated bus |
| 47 | SCH16T EXTRESN | |
| 48 | SYNC out → SCH16T **DRY_SYNC** (pin 20), camera trigger, P4 sync input | SCH16T pin 20 is **either** DRY out **or** SYNC in. We use SYNC, because the S3 is the time master and reads at a fixed offset after each edge. 0 Ω in series so it can be lifted for DRY experiments. |
| 11/12/13/14 | PWM: ESC, steer front, steer rear, spare | Reordered so S3 pins 19-22, RN2 and the J10 SIG column route as one uncrossed bus |
| 4 | Battery voltage ADC | |
| 41/42 | UART1 ↔ P4 UART0 (runtime 2–4 Mbaud + flashing) | |
| 3 | P4 EN | Strapping pin, used as output after boot; pull-up keeps the P4 running if the S3 is in reset |
| 18 | P4 BOOT strap | |
| 8/9 | I2C0 → J11 → AS5600 | |
| 16/17 | I2C1 → expansion Qwiic | |
| 40 | Heater PWM | |
| 1/2/15 | FAULT / LIFE / RGB | |
| 46 | Start button | Strapping, open at boot |
| 0, EN | BOOT / RESET buttons | |
| 43/44 | UART0 (S3 console) | Test pads only |
| 19/20 | USB D−/D+ | |

One spare: IO11.

---

## 4. Design phases

### Phase 0 — Inputs (you)
Fill `datasheets/` per its README, answer D3, and tell me about reflow access.

### Phase 1 — Architecture freeze
- Yaw-rate profile from the course map (D1).
- Power budget per rail, which sizes the TPS54360B UVLO/FB, the heater and the XT30 check.
- P4 module pin map and the S3↔P4 flashing strap check.
- Updated block diagram.

### Phase 2 — Schematic (KiCad 10, hierarchical) — **first pass done: `mainboard/kicad/`, ERC clean**
**Work on a copy.** Copy the CAN+IMU project into `mainboard/kicad/` and open the copy in KiCad 10 (the upgrade is one-way). Footprints, symbols and `Imports/` STEP models come along. The SnapEDA/UL "KiCADv6" libraries load fine.

Sheets:
1. `power`: XT30 in/out, TVS, TPSM33620S3 (3.3 V) from VBAT, TPS54360B (5 V, re-derived UVLO/FB), servo VBAT bus + bulk cap, star ground, test points.
2. `s3`: U1, USB-C (USBLC6-2 DNP), BOOT/RESET/start buttons, LEDs, test pads.
3. `imu_island`:
   - SCH16T, using the SnapEDA footprint with **pad 25 (center pad) deleted**, matching Murata's Fig. 6 and the full datasheet's reference layout (Fig. 18, no center pad).
   - SCH16T capacitors per the full datasheet, Table 80, all ESR < 100 mΩ @ 1 MHz (X7R ceramics):
     - C1 1 µF on VREGD/VREGD2 (pins 14/15)
     - C2 10 µF on VREGA/VREGA2 (pins 10/11)
     - C3 1 µF on V3P3 pin 17
     - C4 100 nF on V3P3 pin 8
     - C5 100 nF on VDDIO (pin 18)
   - SCH16T pins: TA8/TA9 → GND, pin 2 floating, pins 5/7 → GND, EXTRESN driven by the S3 (≤ 20 k pull-up to VDDIO if not), DRY_SYNC → SYNC net through the 0 Ω.
   - ISM6HG256X (copied).
   - Ferrite + AP2112K-3.3.
   - Heater resistors + MOSFET + R21 gate pull-down (no hardware cutoff).
4. `p4`: Waveshare ESP32-P4-Module (footprint from its DXF land pattern), VBAT pad tied to 3.3 V, GPIO36 10 k pull-up, EN/BOOT/UART0 to the S3, CSI pads 41–46 → FPC, microSD on GPIO39–44 (VO4 domain), sync input, IR LED one-shot output, LNA_OUT → π-match → U.FL (D16), test pads for C6 UART/boot and P4 USB-JTAG.
5. `vision_io`: CAM_TRIG connector + level-shift option, IR LED constant-current driver, LED-board connector.
6. `actuation`: PWM headers, battery ADC, J11 encoder Qwiic, expansion Qwiic.
7. ERC, then my datasheet cross-check per IC.

### Phase 3 — Pin assignment
Finalize the table above and assign P4 pins from the module pinout (CSI on dedicated pins; SD slot 0 on the P4's separately powered I/O domain, so check SD_VREF).

### Phase 4 — Floorplan and stackup
- Outline and holes from the TT-02R mount, with dampers.
- Power corner: XT30 in/out adjacent, bucks, servo and ESC headers.
- Island diagonally opposite.
- P4 + FPC on the camera edge.
- SD + USB-C on an accessible edge.
- Stackup: L1 signal / L2 solid GND / L3 power + slow signals / L4 signal + heater resistors.
- Island has routed slots on three sides and a single bridge.

### Phase 5 — Layout
Order: island → MIPI → power → rest. Nothing switching runs under the island. The ESC loop stays in the power corner. IMU SPI has series resistors at the source.

### Phase 6 — Review
DRC against **OSH Park's 4-layer DRU** (the rules file is imported into KiCad), a 3D check against the chassis, and my net-by-net review against the datasheets.

### Phase 7 — Fabrication and assembly
- OSH Park 4-layer, 3 boards.
- OSH Stencils stencil.
- DigiKey/Mouser BOM (SCH16T: order **-004**, the 4-piece bulk pack; -1 and -10 are 100/1000-piece reels).
- Hand assembly in this order: reflow the fine-pitch parts first, then hand-solder the THT parts and connectors.

### Phase 8 — Bring-up
1. Power only: rails, UVLO, ripple at the island.
2. S3: USB, LEDs, PWM.
3. ISM6HG256X (existing driver, WHO_AM_I 0x73). AS5600 at 0x36 with magnet status.
4. SCH16T: SafeSPI CRC, status, TEMP/100, SYNC.
5. Heater: confirm R21 holds it off through S3 reset/boot and that the watchdog trips it off, then tune the PI loop.
6. P4 flashing through the S3 bridge. Then camera, trigger and IR one-shot on a scope.
7. UART link and timestamp alignment (SYNC vs. trigger vs. P4 capture).

### Phase 9 — Calibration
As in the architecture doc, plus the ISM6HG256X's scale factor above 300 °/s.

---

## 5. Open risks
- **SCH16T datasheet versions:** the full datasheet you found is **Rev. 2 (Jan 2024)**, which is older than the short one (**Rev. 6**). Use Rev. 2 for the register map, SafeSPI frames and application circuit. Use Rev. 6 wherever specs differ, and watch for register changes when writing the driver. The full version is stamped **CONFIDENTIAL**, so keep it out of any public repo (add `mainboard/datasheets/02_imu/Datasheet-SCH16T*` to `.gitignore` if this repo is pushed anywhere public).
- **Murata APP 10871** (assembly + PCB design guidelines) is referenced by the full datasheet and is still missing. It matters for island stress and soldering, not for the schematic.
- **RF on a hand-built board:** the U.FL footprint and the 50 Ω trace need care. If Wi-Fi range is poor, the π-match pads allow tuning without a respin.
- **Servo must be reprogrammed** before steering use (D11, deferred).
- **XT30 at launch current** (D8).
- **OV9281 high-fps modes** need custom driver work.
- **Hand-assembling** LGA/QFN parts without a reflow setup.
- **Extended SCH16T saturation** in a crash spin: firmware must degrade gracefully.

---

## Update — Nov 15 (latest decisions, supersede anything above)
- **Wi-Fi comes from the ESP32-S3 only.** The P4 module's C6 is unused: no antenna, U.FL or π-match. LNA_OUT stays open, and firmware holds the C6 in reset through P4 GPIO54. The S3's Wi-Fi is switched off during runs. Consequences:
  - P4 over-the-air updates go through the S3, by store-and-forward over the S3↔P4 UART using `esp-serial-flasher`. USB flashing through the S3 bridge is unchanged.
  - Bulk P4 data (frame dumps) comes off the microSD card. The UART path to the S3's Wi-Fi is only ~250–500 KB/s.
  - Keep the WROOM-1 antenna at a board edge with no copper under it.
- **Expansion Qwiic removed.** Only the AS5600 encoder Qwiic (J11) remains; S3 IO16/17 are now unused.
- **IR illuminator removed** from the board (P4 GPIO3 now unused). If one is ever needed, it can run off the spare servo header J604 (PWM_SPARE + VBAT).
- **Camera I2C pull-ups** (R30/R31) are wired at the FPC pins.
- **Hardware heater cutoff removed.** Firmware handles safety (watchdog, duty cap, two-sensor temperature check). R21 gate pull-down kept.
- **No fuse.** Four GND mounting holes. START is a PC-case style front-panel header J4 (1x4: switch on pins 1-2, LED+/LED- on 3-4; LED+ driven straight from S3 IO17, LED− to GND; the LED/harness supplies its own series resistor); RESET/BOOT are G-Switch GT-TC048D-H025-L1 (4.2 × 3.2 mm SMD).
- **BOM trims:**
  - Heater is now 4 × 39 Ω 2010, one per side of the SCH16T (≈ 2.6 W at 5 V).
  - SPI series resistors are one 4×22 Ω array (RN1).
  - Removed pull-ups that duplicate internal ones: S3 IO0 R206 + C204, SCH16T EXTRESN R302, P4 GPIO35 R402.
  - FAULT LED removed; the NeoPixel shows status and faults, and the LIFE LED stays.
  - SD pull-ups stay discrete, because an array can't sit at the socket pins without crossing lines.
  - About 108 placed parts on 61 BOM lines, excluding test pads and mounting holes.
- **Servo connector:** the four 1×3 headers are now one 3×4 0.1" header, J10 (custom `mobot:PinHeader_3x04_P2.54mm_Servo`), sized for a stacked servo pin block.
  - Columns: ESC / STEER F / STEER R / SPARE.
  - Rows: SIG / V+ / GND.
  - Pads: (ch−1)·3 + row.
  - RN2 is wired straight into the SIG pins. V+ (except the ESC's) and GND are bussed, and C39–C42 sit on the V+ bus. RN2 pairs are 4-5 ESC, 3-6 STEER F, 2-7 STEER R, 1-8 SPARE (uncrossed bus).
- **Debug UART0 (DNP):** J5 (S3) and J7 (P4), separate 3-pin JST-GH (SM03B-GHS-TB): 1 GND / 2 RX / 3 TX, footprints only, meant for the bottom side. J7 shares the P4<->S3 UART link (hold the S3 in reset to use it). Camera trigger J9 is now the vertical JST-SH (BM02B-SRSS-TB).
