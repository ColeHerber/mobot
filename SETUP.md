# Mobot — Pi Setup Guide

Complete setup for a fresh Raspberry Pi 5. Follow in order.

---

## 1. OS & Boot Config

Flash **Raspberry Pi OS Lite (64-bit, Bookworm)** to a microSD.

Edit `/boot/firmware/config.txt` and add/confirm these lines:

```ini
# Enable I2C for BNO085
dtparam=i2c_arm=on

# Enable hardware UART on GPIO 14/15 (not needed if VESC is USB-only)
# dtoverlay=uart0

# Allow Pi 5 to draw full current via GPIO 5V pins (suppresses undervoltage warnings)
usb_max_current_enable=1
```

On Pi 5, Bluetooth uses the RP1 southbridge and does NOT contend with the GPIO UART,
so no `disable-bt` overlay is needed.

---

## 2. System Packages

```bash
sudo apt update && sudo apt install -y \
    python3-pip python3-venv \
    python3-lgpio \
    python3-smbus i2c-tools \
    git
```

---

## 3. Python Dependencies

```bash
cd ~/mobot
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Key packages and why:

| Package | Purpose |
|---|---|
| `pyserial` | USB serial to Teensy and VESC |
| `pyvesc` | VESC packet framing + CRC |
| `adafruit-circuitpython-bno08x` | BNO085 IMU driver |
| `adafruit-blinka` | `board`/`busio` I2C abstractions |
| `lgpio` | GPIO PWM for steering servo (Pi 5 only — **not pigpio**) |
| `PyYAML` | Config file parsing |
| `flask` | Web UI / hot-reload server |

**Pi 5 critical:** Never install or import `pigpio`. The Pi 5 routes GPIO through the RP1
chip on `gpiochip4`. `pigpio` is incompatible. Always use `lgpio`.

---

## 4. USB Device Persistent Naming

USB device numbering (`ttyACM0`, `ttyACM1`, etc.) is assigned by the kernel in
**plug-in order**, not by physical port. This can swap across reboots.

**Use persistent symlinks instead.** With both the Teensy and VESC plugged in:

```bash
ls /dev/serial/by-id/
```

Expected output (names will include actual serial numbers):

```
usb-PJRC_Teensy_4.x_<serialno>-if00      # Teensy line sensor
usb-VESC_Project_VESC_<serialno>-if00    # VESC motor controller
```

Then set these in `config/params.yaml`:

```yaml
sensor:
  port: /dev/serial/by-id/usb-PJRC_Teensy_4.x_<serialno>-if00

vesc:
  port: /dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00
```

These symlinks are stable across reboots and won't swap even if plug order changes.

**To find serial numbers on first boot:**

```bash
udevadm info -a -n /dev/ttyACM0 | grep 'ATTRS{serial}'
udevadm info -a -n /dev/ttyACM1 | grep 'ATTRS{serial}'
```

---

## 5. I2C Verification

Confirm the BNO085 is visible at address `0x4A`:

```bash
i2cdetect -y 1
```

You should see `4a` in the output grid. If not:
- Check wiring (SDA → GPIO2, SCL → GPIO3)
- Confirm `dtparam=i2c_arm=on` is in `/boot/firmware/config.txt` and Pi has been rebooted

---

## 6. Permission: Serial Ports & GPIO

Add your user to the `dialout` and `gpio` groups so the service can access hardware
without running as root:

```bash
sudo usermod -aG dialout,gpio $USER
# Log out and back in (or reboot) for this to take effect
```

---

## 7. First-Boot Hardware Verification

Run these individually before attempting a full run:

```bash
# Teensy line sensor (should see binary packets scrolling):
python3 debug/sensor_debug.py

# VESC telemetry (should see RPM + voltage):
python3 debug/vesc_debug.py

# IMU heading (should see yaw in degrees):
python3 debug/imu_debug.py

# Servo sweep (visual check — robot should steer left/right):
python3 debug/servo_debug.py
```

---

## 8. Servo Center Tuning

After mounting the servo, find the exact center offset so the robot tracks straight:

```bash
bash scripts/run_servo_tuner.sh
```

Adjust with arrow keys, save result. The value writes to `center_offset_us` in
`config/params.yaml`. Do **not** change `center_pw` (nominal 1500 µs).

---

## 9. Sensor Calibration

Must be done in **race-day lighting conditions** (outdoors, same time of day).

1. Place robot over bare pavement and hold still for 2 seconds
2. Slide robot slowly left/right across the full line width
3. Place robot back on pavement — calibration complete

Calibration min/max values are stored in Teensy EEPROM and survive power cycles.
Recalibrate before each heat if lighting has changed.

---

## 10. Run Tests

```bash
bash scripts/run_tests.sh
```

All tests must pass before deploying any code change. Tests run without hardware.

---

## 11. Starting the Robot

```bash
# Dry run (sensors only, no actuators):
bash scripts/run.sh --dry-run

# Full live run:
bash scripts/run.sh

# Web UI only (route editing, param tuning):
bash scripts/run_web.sh
```

Web interface is available at `http://<pi-ip>:5000` from any browser on the same network.

---

## Race Morning (< 60 seconds)

1. Get intersection directions from judges
2. Open `http://<pi-ip>:5000/route` in any browser
3. Click LEFT or RIGHT for each intersection
4. Press **SAVE ROUTE** → confirm
5. Done — no SSH, no restart required

---

## Race Day Checklist

- [ ] Charge LiPo to full race morning (not storage voltage)
- [ ] SSH in, verify all processes start cleanly with `bash scripts/run.sh --dry-run`
- [ ] Recalibrate Teensy sensor in current lighting
- [ ] Confirm VESC is configured: correct motor poles, encoder PPR, current limits
- [ ] Get route from judges → update via web UI at `/route`
- [ ] One slow dry run before the heat
- [ ] Monitor VESC voltage — warn below 10.5V (3S) / 14V (4S), reduce `base_ms` in params

---

## Known Gotchas

| Issue | Fix |
|---|---|
| `/dev/ttyACM*` swaps after reboot | Use `/dev/serial/by-id/` paths in config |
| Pi 5 GPIO not working | Confirm `lgpio` installed, `gpiochip: 4` in params, never use `pigpio` |
| BNO085 not found at 0x4A | Check `dtparam=i2c_arm=on` in boot config, reboot |
| VESC port won't open | Check `dialout` group membership, confirm by-id path |
| Undervoltage warning at boot | Add `usb_max_current_enable=1` to boot config |
| Robot drifts after power cycle | Re-run servo center tuner (`run_servo_tuner.sh`) |
| Line lost during run | Check Teensy USB cable seating — vibration can cause disconnect |
