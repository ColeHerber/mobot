# Main board datasheets and inputs

✅ = in the folder · ⚠️ = in the folder but incomplete or needs a fix · ⬜ = still needed. **(blocking)** items gate the schematic phase.

## 01_processors
- ✅ `esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf`
- ✅ `ESP32-P4-Module-PCB-Land-Pattern.zip` (DWG/DXF) · ✅ `ESP32-P4-Module-3D.zip` (STEP)
- ✅ `waveshare-esp32-p4-module-schematic.pdf` (Waveshare ESP32-P4-Module V1.1: P4NRW32 + C6, 3.3 V only)
- ✅ `esp32-p4_datasheet_en.pdf` (strapping, UART0 = GPIO37/38, IO power domains)
- ⬜ ESP32-P4 *Hardware Design Guidelines* + Technical Reference Manual (I'll fetch these myself; public)
- ⬜ ESP32-S3 chip datasheet + *Hardware Design Guidelines* (I'll fetch these myself; public)
- ✅ S3 module: **ESP32-S3-WROOM-1-N8**
- ❓ `DS_stm32n657a0.pdf`, `DS_stm32h7a3li.pdf`, `STM32H7A3VGT6.zip` are still in `mainboard/` because I don't know their role. Are you considering an STM32N6 (it has MIPI CSI + NPU) for vision or an H7A3 for control? That would change the architecture, so tell me before I plan around them.

## 02_imu
- ✅ `Datasheet-SCH16T-K01_JAN2024.pdf`: **full datasheet** (Rev. 2, 53 pp). Register map, SafeSPI/CRC, SYNC/DRY, application circuit (Table 80) and reference layout. Marked **CONFIDENTIAL**, so don't publish it.
- ✅ `sch16t-k01-short-datasheet.pdf` (Rev. 6, newer specs)
- ⬜ Murata **APP 10871** (assembly + PCB design guidelines; referenced by the full datasheet). Not blocking the schematic.
- ⚠️ `SCH16T_K01_10.zip` (SnapEDA-style symbol/footprint/STEP). Pads 1–24 match Murata's Fig. 6 (0.8 mm pitch, 1.87 × 0.5 mm, 14.2 mm span). **Pad 25 (7.6 × 9.6 mm center pad) gets deleted** to match Murata's Fig. 6. "-10" is just the 1000-piece reel; order **SCH16T-K01-004**.
- ✅ `ism6hg256x.pdf`
- ⬜ NTC thermistor datasheet (I'll propose one)

## 03_power
- ✅ `tps54360b-q1.pdf` · ✅ `tpsm33620-q1.pdf` · ✅ `YJYCOIN YSMC0430H-8R2MB.pdf` (confirm it's L1)
- ✅ `AP2112K-3.3_MSKSEMI_ldo.pdf`: the IMU LDO you have on hand (Vin ≤ 6 V, so fed from 5 V)
- ✅ `AP2112.pdf` (Diodes original, for reference)
- ⬜ Heater MOSFET, heater resistors, hardware thermal-cutoff part (I'll propose; you confirm DigiKey stock)
- ⬜ **Tell me:** battery model and pack connector

## 04_camera_vision
- ⬜ Arducam **OV9281 1 MP NoIR mono, 130° M12, Raspberry Pi** (picked): https://www.arducam.com/product/arducam-ov9281-1mp-global-shutter-noir-mono-mipi-camera-with-130deg-m12-mount-for-raspberry-pi/. Once bought, a photo of the board's back (trigger pad) is enough; I'll fetch Arducam's docs. Not blocking for the outline: the FPC side is the standard Pi 15-pin pinout
- ⬜ 15-pin 1.0 mm FPC connector datasheet (I'll pick a DigiKey part)
- ⬜ IR-pass filter spec (850 nm) · 850 nm IR LED datasheet (peak/pulse current)
- ⬜ Link/notes: `esp_cam_sensor` version you'll use (OV9281 driver is in ≥ v2.6.0)

## 05_storage
- ⬜ microSD socket datasheet (push-push, DigiKey-stocked)

## 06_actuation_drivetrain
- ⬜ **(blocking)** Hobbywing **QuicRun 10BL120 G2** manual
- ⬜ **(blocking)** **Justock 3650 motor turns/kV + pinion/spur**, or better, measured launch current. Decides XT30 vs XT60.
- ✅ (spec read from web) goBILDA 2000-0025-0502 servo: 4.8–7.4 V, 3.0 A stall @ 7.4 V, 0.90°/µs, 4 µs deadband, 50 Hz. Must be **reprogrammed** before steering use (PLAN D11, deferred); save the 3102 programmer manual here when you get to it. ⬜ Save the spec page / STEP here, and tell me if you switch servos.
- ⬜ ams **AS5600** datasheet · Adafruit AS5600 STEMMA QT schematic + dimensions
- ⬜ **Tell me:** magnet location (wheel / prop shaft), magnet part, Qwiic cable length

## 07_connectors_protection
- ⬜ U.FL/IPEX MHF1 connector + 2.4 GHz pigtail dipole antenna (I'll pick DigiKey parts)
- ✅ `Amass XT30PW.pdf` (15 A cont., 30 A for 1 min), `eGH.pdf`, `2169900003.pdf`, `USBLC6-2.pdf` (DNP footprint only), `TVS-Diode-SMBJ-Datasheet.pdf`, `C41367225.pdf`, `1812L-Datasheet-Update.pdf`, `SK52A-L_SK5200A-L(SMA).pdf`

## 08_misc_ui
- ✅ `Littelfuse-CK-Tactile-PTS526-Series-Datasheet.pdf` · ✅ `sn65hvd230.pdf` (unused, CAN dropped)
- ⬜ WS2812B-2020 datasheet

## 09_mechanical
- ✅ `MobotFinalCourse26.pdf`, `Mobot Map.png`
- ⬜ **(blocking for layout)** TT-02R mounting area: outline, holes, height limit (measurements or STEP)
- ⬜ Damper/grommet part · camera mount position/height/tilt (FPC length) · rear-steering hardware

## 10_fab
- ⬜ OSH Park 4-layer **stackup page** (current official values) and their **KiCad DRU** file
- ✅ Reflow available
