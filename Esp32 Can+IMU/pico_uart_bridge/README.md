# Pico 2 USB-UART Bridge

This turns a Raspberry Pi Pico 2 into a simple 3.3 V USB-to-UART bridge for
ESP32-S3 bring-up.

The bridge is fixed at 115200 baud. That is the ESP32-S3 ROM boot log speed and
is also usable for esptool uploads if PlatformIO is kept at 115200.

## Wiring

Use only 3.3 V logic. v2 of the ESP32 board has **no BOOT/RESET buttons** — J3
(9-pin pogo) is the only access to reset/boot/UART0/CAN, so the Pico wires to
all 9 J3 pins. Full physical pin numbers and the reasoning behind each wire
are in `PICO2_PINOUT.md` — short version:

```text
J3 pin 1 (GND)     -> Pico GND
J3 pin 2 (+5V)     -> Pico VBUS   (optional — see PICO2_PINOUT.md power caveat)
J3 pin 3 (3v3_EN)  -> Pico GP0    (open-drain-low: pull low to disable 3.3V)
J3 pin 4 (CAN_R)   -> Pico GP6    (input-only debug tap)
J3 pin 5 (CAN_D)   -> Pico GP7    (input-only debug tap)
J3 pin 6 (RESET)   -> Pico GP2    (open-drain-low: replaces the missing button)
J3 pin 7 (BOOT)    -> Pico GP3    (open-drain-low: replaces the missing button)
J3 pin 8 (UART0RX) -> Pico GP4
J3 pin 9 (UART0TX) -> Pico GP5
```

Cross the UART lines: Pico GP4 (TX) drives the board's UART0RX, Pico GP5 (RX)
reads the board's UART0TX.

Note: Pico GP4/GP5's UART alt-function is wired to the RP2350's **UART1**
peripheral, not UART0 — the firmware calls `uart_init(uart1, ...)` for that
reason, not a typo.

## Flashing The Pico 2 — native firmware (recommended)

The `pico_sdk/` directory contains a native C firmware that compiles to a UF2
you drag-and-drop onto the Pico 2.  No CircuitPython runtime required.

### Prerequisites

* Pico SDK 2.x installed and `PICO_SDK_PATH` set in your shell.
* CMake ≥ 3.13 and the ARM GCC toolchain (`arm-none-eabi-gcc`).

### Build

```bash
cd pico_uart_bridge/pico_sdk

# Copy the SDK import helper (only needed once)
cp "$PICO_SDK_PATH/external/pico_sdk_import.cmake" .

mkdir build && cd build
cmake .. -G Ninja -DPICO_BOARD=pico2
ninja
```

The output is `pico_uart_bridge.uf2` inside the `build/` directory.

### Flash

1. Hold **BOOTSEL** on the Pico 2 and plug it in — it mounts as a USB drive.
2. Copy `build/pico_uart_bridge.uf2` to the drive.
3. The Pico reboots automatically and appears as **"Pico UART Bridge"** on macOS.

macOS will show **two** serial ports (e.g. `/dev/cu.usbmodem*` and
`/dev/cu.usbmodem*1`): the first is the UART bridge (CDC0), the second is the
**control channel** (CDC1) described below.

## Control Channel (CDC1)

Since v2 has no BOOT/RESET buttons, the second CDC port lets you drive
3v3_EN/RESET/BOOT and watch CAN_R/CAN_D from the Pico instead of shorting
pogo pins by hand. Open it with any serial terminal (screen, minicom, etc.)
at any baud rate and send single ASCII characters:

| Command | Action |
|---|---|
| `r` | Release BOOT and pulse RESET (normal reboot) |
| `e` | Enter bootloader (assert BOOT, pulse RESET, keep BOOT held for esptool connect) |
| `b` | Hold BOOT low without pulsing RESET |
| `u` | Release BOOT |
| `o` | 3v3_EN off — disable the 3.3V rail (power-cycle IMU/CAN domain without touching 5V) |
| `n` | 3v3_EN on — release 3v3_EN, let the board's own pull-up re-enable the rail |
| `c` | Toggle a CAN_R/CAN_D logic-level monitor — prints edge transitions with timestamps. This is a coarse polling tap, not a bit-accurate CAN sniffer; good for "is CAN_D toggling when I send a frame" checks, not full decode |
| `l` | Print passive J3 GPIO levels without driving pins |
| `t` | Run a safe short test: release all control pins, then pull 3v3_EN/RESET/BOOT low one at a time and report J3 GPIO levels |
| `h` or `?` | Print the command list |

Example: `echo -n e > /dev/cu.usbmodemXXXX1` triggers bootloader entry before
an `esptool`/`pio run -t upload` — no need to hold BOOT and tap RESET by hand.

---

## Flashing The Pico 2 — CircuitPython (legacy)

1. Put CircuitPython for Raspberry Pi Pico 2 on the Pico 2.
2. Copy `circuitpython/boot.py` and `circuitpython/code.py` to the Pico's
   `CIRCUITPY` drive.
3. Unplug and replug the Pico.
4. macOS should show two Pico serial ports. Use the data CDC port for the bridge.

If you are unsure which `/dev/cu.*` entry is the bridge, run:

```bash
ls /dev/cu.*
```

Then unplug/replug the Pico and look for the new entries.

## ESP32-S3 Bootloader Test

Open a serial monitor at 115200 baud on the Pico bridge port (CDC0), then send
`r` on the control port (CDC1) to reset the ESP32. If it's alive and TXD0 is
connected, you should see ROM boot text.

For upload, enter download mode by sending `e` on the control port (CDC1) —
this automates the old manual procedure (hold BOOT, tap RESET, release BOOT
after esptool starts connecting), since there's no button to hold on v2.

Use a slow upload speed:

```bash
pio run -t upload --upload-port /dev/cu.usbmodem*
```

If upload fails after connecting, add `upload_speed = 115200` to the ESP32
environment or temporarily upload with a config that sets 115200.

The helper script wraps those steps and auto-detects the Pico's two CDC ports:

```bash
scripts/pico_flash_esp32.py --env esp32-s3-wroom-1-ism6hg256x-debug
```

It sends `e` on the Pico control port to strap BOOT and pulse RESET, runs
PlatformIO upload through the UART bridge port, sends `u` to release BOOT, then
sends `r` to reset the ESP32 normally after a successful upload. Use
`--dry-run --verbose` to confirm the detected ports and command without touching
BOOT/RESET or flashing.

To check for obvious J3 fixture shorts on the Pico-controlled pins:

```bash
scripts/pico_pin_debug.py --raw
```

For intermittent pogo/contact issues, use passive watch mode and gently move the fixture:

```bash
scripts/pico_pin_debug.py --watch
```
