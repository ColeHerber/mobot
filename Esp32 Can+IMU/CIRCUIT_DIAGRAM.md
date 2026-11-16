# ESP32 CAN+IMU Board — Circuit Diagram
**Source: review_exports/schematic_netlist.xml + ipc_d_356.net (authoritative)**

Board: 4-layer, 2-oz Cu  
MCU: ESP32-S3-WROOM-1 (U1)  
IMU: ISM6HG256XTR (U2)

---

## Power Architecture

```
XT30F1 ──┐                           XT30M1 (passthrough)
XT30F2 ──┤─── VIN net ──────────────── XT30M1 pin2 (+)
         │                             XT30M1 pin1 (GND)
         ├── D_TVS1 (SMBJ58A) ──── GND
         │   TVS, 58 V clamp, anode→GND, cathode→VIN
         │
         F1 (C41367225 fuse, 1206 handsoldered)
         │
         └── VBUS net ─────────────────────────────────────┐
              C1, C2 (2.2 uF 100V × 2, input bulk filter)  │
              D7 (SS56, cathode→VBUS)                       │
              R1 (174 k, top of U4 voltage-sense divider)   │
                                                            │
         ┌──────────────────────────────────────────────────┘
         │
        [U4: TPS54360BQDDARQ1]  60 V-in, 3.5 A synchronous buck
         VIN (pin2) ← VBUS
         GND (pin7, EPAD) → GND
         FB  (pin5) ← R5 (59 k, from +5V) + R6 (10.2 k, to GND)
                       sets Vout = 5.0 V
         SW node → L1 (8.2 uH) → +5V output
         D5, D9, D10 (SS54 × 3) — bootstrap / body diodes
         D8 (SS56) — snubber / bootstrap
         C6, C7 (47 uF 10V × 2) — output filter
         C11 (4.7 uF X7R) — additional output filter
         +5V rail ─────────────────────────────────────────┐
                                                           │
        [U5: TPSM33620S3QRDNRQ1]   integrated 5V→3.3V buck│
         VIN  (pin3)  ← +5V ◄───────────────────────────── ┘
         VOUT (pin4)  → +3.3V
         EN   (pin2)  ← 3v3_EN net (R16 100 k pull-up to +5V; also J3 pin3)
         FB   (pin9)  → +3.3V (output sense)
         GND  (pin10) → GND
         MODE_SYNC (pin11) → GND (forced-PWM / CCM mode)
         C10 (22 uF, 0603) — output bulk
         C19, C20 (22 uF × 2, 0805) — additional output filter
         C3, C14, C15 (1 uF X5R × 3) — high-freq decoupling
         +3.3V rail ─────────────────────────────────────────→ U1, U2, U3

        USB backup power (D9 diode OR from USB VBUS):
         +USB_5V net: J17 VBUS pins → U7 pin5 → D9 anode
         D9 (SS54): anode ← +USB_5V, cathode → +5V rail
         → USB-C can power the 5 V rail when XT30 is unplugged

        +5V external output:
         J6 (2-pin 2.54 mm header): pin2 = +5V, pin1 = GND
```

---

## GPIO Map — ESP32-S3-WROOM-1 (U1)

All GPIOs derived from netlist U1 pin-function tags and confirmed against  
ESP32-S3-WROOM-1 module physical pinout.

```
Module  GPIO   Net / Signal          Peripheral    Connected to
pin
─────────────────────────────────────────────────────────────────────
 1      GND    GND
 2      3V3    +3.3V                 Power in
 3      EN     RESET                 Active-low     C3 (RC filter), R7 (10k PU to 3V3), J3 pin6
 4      IO4    ADC1_CH1 (= ADC1_CH3)ADC1 ch3       RN4→RN3 divider → J12 (ADC1) pin1
 5      IO5    ADC1_CH0 (= ADC1_CH4)ADC1 ch4       RN4→RN3 divider → J13 (ADC2) pin1
 6      IO6    ADC1_CH5              ADC1 ch5       RN4→RN3 divider → J14 (ADC3) pin1
 7      IO7    ADC1_CH6              ADC1 ch6       RN4→RN3 divider → J15 (ADC4) pin1
 8      IO15   RGB_1                 GPIO/RMT out   D2 WS2812B-2020 DIN
 9      IO16   CAN_D                 TWAI TX        U3 SN65HVD232DR D (driver input)
10      IO17   CAN_R                 TWAI RX        U3 SN65HVD232DR R (receiver output)
12      IO8    SDA0                  I2C-0 SDA      R19 (4.7k PU) → J11 Qwiic pin3
13      —      USB_D-                USB HS/FS      U7 USBLC6-2SC6 I/O1
14      —      USB_D+                USB HS/FS      U7 USBLC6-2SC6 I/O2
17      IO9    SCL0                  I2C-0 SCL      R18 (4.7k PU) → J11 Qwiic pin4
19      IO11   PWM_4                 LEDC ch3       RN1 (33Ω) → J10 (PWM4) pin1
20      IO12   PWM_3                 LEDC ch2       RN1 (33Ω) → J9  (PWM3) pin1
21      IO13   PWM_2                 LEDC ch1       RN1 (33Ω) → J8  (PWM2) pin1
22      IO14   PWM_1                 LEDC ch0       RN1 (33Ω) → J7  (PWM1) pin1
23      IO21   CS                    SPI CS         U2 IMU CS (active low); J5 pin6
27      IO0    BOOT                  GPIO/BOOT      C15 (RC), R8 (10k PU to 3V3), J3 pin7
28      IO35   SDA (= IMU MOSI)      SPI FSPI MOSI  U2 IMU SDA pin14; J5 pin4
29      IO36   SCL (= IMU SCK)       SPI FSPI SCK   U2 IMU SCL pin13; J5 pin3
30      IO37   SDO/TA0 (= IMU MISO)  SPI FSPI MISO  U2 IMU SDO pin1;  J5 pin7
31      IO38   INT1                  GPIO input     U2 IMU INT1 pin4; J5 pin8
32      IO39   INT2                  GPIO input     U2 IMU INT2 pin9; J5 pin9
34      IO41   UART1RX               UART1 RX       J4 (UART1) pin3
35      IO42   UART1TX               UART1 TX       J4 (UART1) pin1
36      GPIO44 UART0RX               UART0 RX       J3 pogo pin8 (console/prog)
37      GPIO43 UART0TX               UART0 TX       J3 pogo pin9 (console/prog)
38      IO2    STATUS                GPIO out       R23 (100Ω) → D4 (LIFE LED) anode
39      IO1    FAULT                 GPIO out       R24 (100Ω) → D6 (FAULT LED) anode
40,41   GND    GND
```

**Note on ADC net-label mismatch:** The schematic labels the GPIO4/5 nets
"ADC1_CH1" and "ADC1_CH0", but the ESP32-S3 peripheral map puts GPIO4 on
ADC1_CH**3** and GPIO5 on ADC1_CH**4**. GPIO6 = ADC1_CH5 and GPIO7 = ADC1_CH6
are correctly labelled. Use the actual channel numbers when calling
`analogRead()` or configuring the ADC peripheral.

---

## IMU — ISM6HG256XTR (U2)

Dual-channel IMU (±8g/±2000 dps low-g + ±256g high-g). SPI mode only
(U2 pins 2/3 = SDX/SCX tied to GND which forces SPI).

```
Signal         U2 pin   Net            ESP32 GPIO   Module pin
──────────────────────────────────────────────────────────────
SDO / MISO       1      SDO/TA0        IO37           30
SDX (SPI sel)    2      GND            —              —
SCX (SPI sel)    3      GND            —              —
INT1             4      INT1           IO38           31
VDDIO            5      +3.3V          —              —
GND              6      GND            —              —
GND              7      GND            —              —
VDD              8      +3.3V          —              —
INT2             9      INT2           IO39           32
CS (active low) 12      CS             IO21           23
SCL / SCK       13      SCL            IO36           29
SDA / MOSI      14      SDA            IO35           28

Decoupling: C4 (100nF), C9 (100nF), C5 (6.8nF), C8 (39pF on CS line)

INT1 = low-g accel + gyro DRDY (read on rising edge)
INT2 = high-g accel DRDY (independent ODR, ISM6HG256X-only)
SPI mode 3, 4-wire, up to 10 MHz (1 MHz used in current driver)
```

---

## CAN Bus — SN65HVD232DR (U3)

3.3V-native differential transceiver. Both CAN connectors wired in parallel
to the same CANH/CANL lines.

```
Signal      U3 pin   Net       ESP32 GPIO   Direction
──────────────────────────────────────────────────────
D (TX in)     1      CAN_D     IO16          ESP32 → U3
GND           2      GND
VCC           3      +3.3V
R (RX out)    4      CAN_R     IO17          U3 → ESP32
CANL          6      CAN-      —             bus differential
CANH          7      CAN+      —             bus differential

Bus connectors (JST-GH, 4-pin, 1.25mm):
  J1 (CAN0): pin1=GND, pin2=CAN+, pin3=CAN-, pin4=GND
  J2 (CAN1): pin1=GND, pin2=CAN+, pin3=CAN-, pin4=GND
  (J1/J2 wired in parallel — two device drop-in positions)
  Note: J1/J2 pin4 = GND (the +5V CAN bus power option is not connected
  to a net in this revision; RegPwr1 only places a marker on the +5V rail)

Termination: CAN_TERM1 (SolderJumper, 2-pin open)
  Jumper bridged → 120Ω (R9) inserted between CAN- and the jumper
  CAN_TERM1 pin2 → CAN+; CAN- → R9 (120Ω) → R9 pin1 (unnamed net)
  Close jumper to terminate this board as a bus endpoint.

R9 (120Ω): series on CAN- before the jumper
C12 (100nF): bus decoupling
RN3 (2.2k × 4): shares array with ADC voltage dividers
RN4 (3.9k × 4): shares array with ADC voltage dividers
```

---

## USB-C Interface — J17 + USBLC6-2SC6 (U7)

```
J17 (JAE DX07S016JA1R1500, USB-C receptacle)
  VBUS (A4,A9,B4,B9) → +USB_5V net → D9 (SS54) → +5V rail
  GND  (A1,A12,B1,B12,shield) → GND
  CC1 → R10 (5.1k) → GND    ← UFP/sink Ra configuration
  CC2 → R11 (5.1k) → GND
  D+  (A6/B6) → U7 I/O2 (pin4) → U1 USB_D+ (module pin14)
  D-  (A7/B7) → U7 I/O1 (pin6) → U1 USB_D- (module pin13)

U7 USBLC6-2SC6 (SOT-23-6, hand-soldered Rev B):
  pin2 = GND
  pin4 = I/O2 ↔ USB_D+
  pin5 = VBUS ← +USB_5V
  pin6 = I/O1 ↔ USB_D-
  (TVS-based ESD clamp on both data lines)

Programming/console: native USB-CDC via USB-D+/D- (no UART adapter needed)
UART0 is also exposed on J3 pogo pins 8/9 as backup.
```

---

## PWM Outputs (J7–J10)

3-pin 2.54mm headers: pin1=signal, pin2=+5V, pin3=GND.  
Signal passes through RN1 (33Ω array) for damping.

```
Connector   ESP32 GPIO   Net     RN1 resistor
──────────────────────────────────────────────
J7  PWM1      IO14       PWM_1   R2 (pin2↔pin7)
J8  PWM2      IO13       PWM_2   R1 (pin1↔pin8)
J9  PWM3      IO12       PWM_3   R3 (pin3↔pin6)
J10 PWM4      IO11       PWM_4   R4 (pin4↔pin5)

LEDC peripheral, 50 Hz default, 16-bit resolution, 1500 µs center pulse.
Clamp range: 500–2500 µs.
+5V on pin2 provides servo/ESC bus power.
```

---

## ADC Inputs (J12–J15)

3-pin 2.54mm headers: pin1=signal-in, pin2=+5V (sensor power), pin3=GND.  
Resistor voltage divider scales input before ADC pin.

```
Connector   ESP32 GPIO  ADC channel  Net label    Divider path
──────────────────────────────────────────────────────────────────
J12 ADC1      IO4       ADC1_CH3   "ADC1_CH1"  J12-p1→RN4-R4→mid→RN3-R4→GND
J13 ADC2      IO5       ADC1_CH4   "ADC1_CH0"  J13-p1→RN4-R3→mid→RN3-R3→GND
J14 ADC3      IO6       ADC1_CH5    ADC1_CH5   J14-p1→RN4-R2→mid→RN3-R2→GND
J15 ADC4      IO7       ADC1_CH6    ADC1_CH6   J15-p1→RN4-R1→mid→RN3-R1→GND

Divider values:  RN4 top = 3.9 kΩ,  RN3 bottom = 2.2 kΩ
Scale factor:    2.2 / (3.9 + 2.2) = 0.361
Max input for    3.3V ADC full-scale: 3.3 / 0.361 ≈ 9.1 V
At 5V input:     5 × 0.361 = 1.8 V at ADC pin

Note: schematic net labels "ADC1_CH0" and "ADC1_CH1" are annotation errors;
the true ESP32-S3 channel numbers are CH4 and CH3 respectively (GPIO5, GPIO4).
```

---

## I2C — Qwiic (J11)

JST-SH 4-pin 1mm (Qwiic standard). Second I2C bus (I2C-0).

```
J11 pin1 → GND
J11 pin2 → +3.3V
J11 pin3 → SDA0 net → R19 (4.7k pull-up to +3.3V) → ESP32 IO8  (GPIO8)
J11 pin4 → SCL0 net → R18 (4.7k pull-up to +3.3V) → ESP32 IO9  (GPIO9)
```

---

## UART1 — J4

JST-GH 3-pin 1.25mm.

```
J4 pin1 → UART1TX net → ESP32 IO42 (GPIO42)   board TX → external RX
J4 pin2 → GND
J4 pin3 → UART1RX net → ESP32 IO41 (GPIO41)   external TX → board RX
```

---

## Pogo Pads

### J3 (9-pin pogo) — Debug / programming / control
```
pin1  GND
pin2  +5V
pin3  3v3_EN   → U5 EN (pin2); R16 (100k) pulls to +5V (EN active-high)
                  Drive low to disable 3.3V rail from test fixture
pin4  CAN_R    → ESP32 IO17 (CAN TWAI RX)
pin5  CAN_D    → ESP32 IO16 (CAN TWAI TX)
pin6  RESET    → ESP32 EN (active-low reset), C3 RC filter
pin7  BOOT     → ESP32 IO0 (GPIO0), C15 RC filter
                  Hold low at reset to enter download mode
pin8  UART0RX  → ESP32 GPIO44 (UART0 RX — console / esptool)
pin9  UART0TX  → ESP32 GPIO43 (UART0 TX — console / esptool)
```

### J5 (9-pin pogo) — IMU SPI debug access
```
pin1  GND
pin2  +3.3V
pin3  SCL      → ESP32 IO36 (SPI FSPI SCK) + U2 IMU SCL
pin4  SDA      → ESP32 IO35 (SPI FSPI MOSI) + U2 IMU SDA
pin5  GND
pin6  CS       → ESP32 IO21 (SPI CS) + U2 IMU CS
pin7  SDO/TA0  → ESP32 IO37 (SPI FSPI MISO) + U2 IMU SDO
pin8  INT1     → ESP32 IO38 + U2 IMU INT1
pin9  INT2     → ESP32 IO39 + U2 IMU INT2
```

---

## Status LEDs

```
LED     Ref  Drive path                              Behavior
──────────────────────────────────────────────────────────────────────────
PWR     D3   +3.3V → R22 (1kΩ) → D3 anode → GND   always-on (power good)
LIFE    D4   GPIO2 → R23 (100Ω) → D4 anode → GND   firmware-driven STATUS
FAULT   D6   GPIO1 → R24 (100Ω) → D6 anode → GND   firmware-driven FAULT
RGB     D2   GPIO15 → D2 DIN (WS2812B-2020)         addressable NeoPixel
             D2 VDD → +3.3V;  D2 VSS → GND
```

---

## Complete GPIO Reference Table

| GPIO   | Direction | Function              | Peripheral | Net / Connector       |
|--------|-----------|-----------------------|------------|-----------------------|
| IO1    | Output    | FAULT LED             | GPIO       | R24 → D6 (FAULT)     |
| IO2    | Output    | LIFE/STATUS LED       | GPIO       | R23 → D4 (LIFE)      |
| IO4    | Input     | ADC in (0–9V scaled)  | ADC1_CH3   | RN4/RN3 → J12        |
| IO5    | Input     | ADC in (0–9V scaled)  | ADC1_CH4   | RN4/RN3 → J13        |
| IO6    | Input     | ADC in (0–9V scaled)  | ADC1_CH5   | RN4/RN3 → J14        |
| IO7    | Input     | ADC in (0–9V scaled)  | ADC1_CH6   | RN4/RN3 → J15        |
| IO8    | Bidir     | I2C-0 SDA             | I2C        | R19 PU → J11 Qwiic   |
| IO9    | Bidir     | I2C-0 SCL             | I2C        | R18 PU → J11 Qwiic   |
| IO11   | Output    | PWM ch3               | LEDC       | RN1 → J10            |
| IO12   | Output    | PWM ch2               | LEDC       | RN1 → J9             |
| IO13   | Output    | PWM ch1               | LEDC       | RN1 → J8             |
| IO14   | Output    | PWM ch0               | LEDC       | RN1 → J7             |
| IO15   | Output    | RGB LED data          | RMT/LEDC   | D2 WS2812B DIN        |
| IO16   | Output    | CAN TX                | TWAI       | U3 D pin → J1/J2     |
| IO17   | Input     | CAN RX                | TWAI       | U3 R pin ← J1/J2     |
| IO21   | Output    | IMU SPI CS            | FSPI       | U2 CS; J5 pin6        |
| IO35   | Output    | IMU SPI MOSI          | FSPI       | U2 SDA; J5 pin4       |
| IO36   | Output    | IMU SPI SCK           | FSPI       | U2 SCL; J5 pin3       |
| IO37   | Input     | IMU SPI MISO          | FSPI       | U2 SDO; J5 pin7       |
| IO38   | Input     | IMU INT1 (low-g DRDY) | GPIO IRQ   | U2 INT1; J5 pin8      |
| IO39   | Input     | IMU INT2 (high-g DRDY)| GPIO IRQ   | U2 INT2; J5 pin9      |
| IO41   | Input     | UART1 RX              | UART1      | J4 pin3               |
| IO42   | Output    | UART1 TX              | UART1      | J4 pin1               |
| GPIO43 | Output    | UART0 TX (console)    | UART0      | J3 pin9               |
| GPIO44 | Input     | UART0 RX (console)    | UART0      | J3 pin8               |
| IO0    | Input     | BOOT mode select      | Strapping  | J3 pin7, PU to 3V3    |
| EN     | Input     | Reset (active low)    | System     | J3 pin6, PU to 3V3    |
| USB_D+ | Bidir    | USB data+             | USB OTG    | U7 → J17              |
| USB_D- | Bidir    | USB data-             | USB OTG    | U7 → J17              |

---

## Passives Reference

| Ref          | Value       | Location / Function                                    |
|--------------|-------------|--------------------------------------------------------|
| C1, C2       | 2.2uF 100V  | VBUS input bulk (pre-buck)                             |
| C3           | 1uF 10V X5R | RESET RC filter (with R7)                              |
| C4           | 100nF       | IMU VDD decoupling                                     |
| C5           | 6.8nF       | IMU CS line filter                                     |
| C6, C7       | 47uF 10V    | U4 (TPS54360B) output filter                           |
| C8           | 39pF        | IMU CS ESD / slew cap                                  |
| C9           | 100nF       | IMU VDDIO decoupling                                   |
| C10          | 22uF        | U5 output bulk                                         |
| C11          | 4.7uF X7R   | +5V mid-freq decoupling                                |
| C12          | 100nF       | CAN bus decoupling                                     |
| C13          | 100nF       | 3.3V decoupling                                        |
| C14          | 1uF 10V X5R | 3.3V decoupling                                        |
| C15          | 1uF 10V X5R | BOOT RC filter (with R8)                               |
| C16–C18      | 100nF       | 3.3V high-freq decoupling                              |
| C19, C20     | 22uF        | U5 output additional bulk                              |
| D_TVS1       | SMBJ58A     | 58V input TVS clamp (cathode → VIN, anode → GND)      |
| D2           | WS2812B-2020| RGB status LED, addressable, 3.3V                     |
| D3           | PWR LED     | Always-on 3.3V power indicator (R22 1kΩ ballast)      |
| D4           | LIFE LED    | GPIO2-driven heartbeat/status indicator                |
| D5, D9, D10  | SS54        | 5A Schottky — buck diodes / USB-to-5V ORing           |
| D6           | FAULT LED   | GPIO1-driven fault indicator                           |
| D7, D8       | SS56        | 5A 60V Schottky — buck snubber / bootstrap             |
| F1           | C41367225   | Fuse, 1206, hand-solder                                |
| L1           | 8.2uH       | U4 TPS54360B switching inductor (Changjiang FNR4010S)  |
| R7           | 10k         | EN (RESET) pull-up to +3.3V                            |
| R8           | 10k         | BOOT (IO0) pull-up to +3.3V                            |
| R9           | 120Ω        | CAN termination resistor (via CAN_TERM1 jumper)        |
| R10, R11     | 5.1k        | USB-C CC1/CC2 pull-downs (UFP sink mode)               |
| R16          | 100k        | 3v3_EN pull-up to +5V (enables U5 by default)         |
| R18          | 4.7k        | SCL0 / Qwiic I2C pull-up to +3.3V                     |
| R19          | 4.7k        | SDA0 / Qwiic I2C pull-up to +3.3V                     |
| R22          | 1kΩ         | D3 (PWR LED) current-limit                             |
| R23          | 100Ω        | D4 (LIFE LED) series / GPIO damping                    |
| R24          | 100Ω        | D6 (FAULT LED) series / GPIO damping                   |
| RN1          | 33Ω × 4     | PWM output series damping (one per channel)            |
| RN3          | 2.2kΩ × 4   | ADC divider bottom (to GND)                            |
| RN4          | 3.9kΩ × 4   | ADC divider top (from connector signal pin)            |
| R1           | 174k        | U4 TPS54360B voltage-sense (top, on VBUS)              |
| R2, R3       | 35.3k, 162k | U4 TPS54360B compensation network                      |
| R5           | 59k         | U4 FB top (from +5V to FB pin)                         |
| R6           | 10.2k       | U4 FB bottom (FB pin to GND)                           |
| R4           | 13.0k       | U4 soft-start / UVLO setting                           |
| CAN_TERM1    | SolderJumper| Open by default; bridge to insert R9 (120Ω) on CAN-   |
