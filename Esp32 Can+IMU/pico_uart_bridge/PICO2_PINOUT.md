# Raspberry Pi Pico 2 — Pin Map for the J3 Pogo Fixture

v2 has **no BOOT/RESET buttons** — confirmed against the schematic netlist, there are no
switch components on this board at all. J3 (9-pin pogo) is the only access to
RESET/BOOT/UART0/CAN, and it can also be used as an optional power-in for bench
bring-up: J3 pin 2 (+5V) sits directly on the board's main +5V rail (same net as U5's
VIN). Feeding 5V in at that pin powers the logic side of the board with no XT30 or USB-C
connected, but only use it when the Pico USB supply is the only board power source.

## J3 pogo → Pico 2

| J3 pin | Signal      | ESP32 net             | Pico 2 physical pin | Wired? |
|-------:|-------------|-------------------------|:--------------------:|:------:|
| 1      | GND         | GND                     | **8** (GND)           | Yes    |
| 2      | +5V         | +5V rail (board power)  | **40** (VBUS)         | Optional — powers the board from the Pico's USB 5V |
| 3      | 3v3_EN      | U5 EN (3.3V rail on/off)| **1** (GP0)           | Yes — open-drain-low only |
| 4      | CAN_R       | ESP32 IO17 (TWAI RX)    | **9** (GP6)           | Yes — input-only debug tap |
| 5      | CAN_D       | ESP32 IO16 (TWAI TX)    | **10** (GP7)          | Yes — input-only debug tap |
| 6      | RESET       | ESP32 EN (active-low)   | **4** (GP2)           | Yes — open-drain-low reset |
| 7      | BOOT        | ESP32 IO0               | **5** (GP3)           | Yes — open-drain-low boot strap |
| 8      | UART0RX     | ESP32 GPIO44 (U0RXD)    | **6** (GP4)           | Yes |
| 9      | UART0TX     | ESP32 GPIO43 (U0TXD)    | **7** (GP5)           | Yes |

3v3_EN is active-high with only a weak 100k pull-up (R16) to +5V — it floats/rests high
(rail enabled) on its own, and the pogo pin exists so the fixture can **actively pull it
low to force the 3.3V rail off** (e.g. to power-cycle the ESP32/IMU/CAN-transceiver
domain without touching 5V, or to hold the board's logic dead while probing). Treat GP0
as an open-drain output: drive it low to disable, then release it to input/high-Z to let
R16 re-enable the rail. Do not drive GP0 high; the board-side pull-up is to +5V.

RESET and BOOT should be handled the same way: drive GP2/GP3 low only when asserting the
signal, then release them to input/high-Z. The board already pulls RESET and BOOT up to
+3.3V.

CAN_R, CAN_D, and UART0TX are board-driven signals. Keep Pico GP6, GP7, and GP5 as inputs
with no internal pulls unless the ESP32 side is held in reset or the line is otherwise
isolated. This matters most on CAN_D: a pulldown can bias the CAN transceiver TX input
toward dominant while the ESP32 is Hi-Z. GP4 is the only line the bridge normally drives,
and it drives the ESP32 UART0RX input.

**9 wires total — every J3 pin is now wired:**

```
J3 pin 1 (GND)     ── Pico pin 8  (GND)
J3 pin 2 (+5V)     ── Pico pin 40 (VBUS)
J3 pin 3 (3v3_EN)  ── Pico pin 1  (GP0)   -- pull low to disable 3.3V
J3 pin 4 (CAN_R)   ── Pico pin 9  (GP6)
J3 pin 5 (CAN_D)   ── Pico pin 10 (GP7)
J3 pin 6 (RESET)   ── Pico pin 4  (GP2)
J3 pin 7 (BOOT)    ── Pico pin 5  (GP3)
J3 pin 8 (UART0RX) ── Pico pin 6  (GP4)
J3 pin 9 (UART0TX) ── Pico pin 7  (GP5)
```

Cross the UART lines as before: Pico **GP4** (pin 6) → board **UART0RX** (J3 pin 8), Pico
**GP5** (pin 7) → board **UART0TX** (J3 pin 9).

**Firmware status — implemented.** `pico_uart_bridge/pico_sdk/main.c` now exposes a second
USB-CDC interface (CDC1, alongside the CDC0 UART bridge) as a plain-text control channel:
`r`=release BOOT and pulse RESET, `e`=enter bootloader and keep BOOT asserted, `b`=hold
BOOT low, `u`=release BOOT, `o`/`n`=3v3_EN off/on, `c`=toggle a CAN_R/CAN_D logic-level
monitor (coarse edge-timestamp polling, not a bit-accurate CAN sniffer), `h`=help. See
`pico_uart_bridge/README.md`'s "Control Channel (CDC1)" section for details. GP0/GP2/GP3
are driven open-drain-low only (never forced high), matching the board's own pull-ups.

**Power caveat:** do not hardwire J3 pin 2 to Pico VBUS in a fixture that may also see
XT30 or board USB-C power. Pico pin 40 is the Pico USB VBUS node; tying it to the board
+5V rail can backfeed the Pico USB cable/host when the board is externally powered. For a
fixed fixture, make J3 pin 2 switched/removable or add proper power ORing/current limiting.
The Pico's VBUS is fine for MCU/IMU/CAN-transceiver bring-up, but not for driving servo
loads on the PWM headers.

## Full physical Pico 2 pinout (reference)

The Pico 2 (RP2350) uses the same 40-pin, 0.1" castellated-edge footprint as the
original Pico. USB connector at the top; pin 1 top-left, numbering runs down the
left side (1→20), then back up the right side (21→40).

```
                    ┌─────────────┐
                    │     USB     │
                    └─────────────┘
   GP0   ●1   → J3 pin 3 (3v3_EN) 40●  VBUS   ← J3 pin 2 (+5V)
   GP1   ●2                    39●  VSYS
   GND   ●3                    38●  GND
   GP2   ●4   → J3 pin 6 (RESET) 37●  3V3_EN
   GP3   ●5   → J3 pin 7 (BOOT)  36●  3V3(OUT)
   GP4   ●6   → J3 pin 8 (ESP32 RXD0) 35●  ADC_VREF
   GP5   ●7   ← J3 pin 9 (ESP32 TXD0) 34●  GP28 / ADC2
   GND   ●8   ← J3 pin 1 (GND)   33●  GND
   GP6   ●9   ← J3 pin 4 (CAN_R) 32●  GP27 / ADC1
   GP7   ●10  ← J3 pin 5 (CAN_D) 31●  GP26 / ADC0
   GP8   ●11                     30●  RUN
   GP9   ●12                     29●  GP22
   GND   ●13                     28●  GND
   GP10  ●14                     27●  GP21
   GP11  ●15                     26●  GP20
   GP12  ●16                     25●  GP19
   GP13  ●17                     24●  GP18
   GND   ●18                     23●  GND
   GP14  ●19                     22●  GP17
   GP15  ●20                     21●  GP16
```
