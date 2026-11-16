# CAN Communication Setup — v1 ↔ v2 Boards

This guide walks you through setting up bidirectional CAN communication between the v1 and v2 ESP32 CAN+IMU boards.

## Hardware Requirements

### CAN Bus Wiring
- Connect CAN+ (pin 2) on v1 J1/J2 to CAN+ (pin 2) on v2 J1/J2
- Connect CAN- (pin 3) on v1 J1/J2 to CAN- (pin 3) on v2 J1/J2
- Connect GND (pin 1) between both boards

### CAN Bus Termination
The CAN bus **requires** 120Ω termination at both ends:

**On both v1 and v2 boards:**
- Locate the **CAN_TERM1** solder jumper (2-pin, near the CAN connectors)
- Bridge the jumper with solder to insert the 120Ω termination resistor (R9)
- Verify with continuity tester: CAN+ to CAN- should read ~60Ω (two 120Ω resistors in parallel)

If only one board has termination enabled, the CAN bus will be unstable and frames may not transmit/receive properly.

## Firmware Protocol

Both boards run the same firmware code. The protocol works as follows:

### Message Format
IMU data is broadcast as two CAN frames (IMU_DATA message type):
- **Frame A (accel)**: Low-g accelerometer data (X, Y, Z in mg)
- **Frame B (gyro)**: Angular velocity data (X, Y, Z in dps×100)

Each frame is sent at **10 Hz** (100 ms interval).

### CAN IDs
```
CAN ID = [PRIO:2][TYPE:4][BOARD:3][FRAME_INDEX:3]
  PRIO = 1 (low priority)
  TYPE = 1 (IMU_DATA)
  BOARD = 1 (v1) or 2 (v2)
  FRAME_INDEX = 0 (accel) or 1 (gyro)
```

Example CAN IDs:
- v1 accel: 0x044 (0100 0100)
- v1 gyro:  0x045 (0100 0101)
- v2 accel: 0x04C (0100 1100)
- v2 gyro:  0x04D (0100 1101)

### Decoding
When a board receives frames from the remote board, it:
1. Decodes the IMU data (accel from frame A, gyro from frame B)
2. Stores it in `remoteBoardImuData` struct
3. Prints to serial: `[CAN] Received IMU: ax=... ay=... az=... gx=... gy=... gz=...`

## Flashing and Testing

### Step 1: Flash Both Boards
```bash
# Terminal 1: Flash v1 board
cd ~/Desktop/Rolly/Esp32\ Can+IMU
pio run -e v1-debug -t upload

# Terminal 2: Flash v2 board (after v1 is done)
pio run -e v2-debug -t upload
```

### Step 2: Monitor Serial Output
```bash
# Terminal 1: Monitor v1 (adjust port as needed)
pio device monitor -p /dev/cu.usbmodem* -b 115200

# Terminal 2: Monitor v2 (on a different USB port)
pio device monitor -p /dev/cu.usbmodem* -b 115200
```

### Step 3: Verify CAN Communication
Look for messages like:
```
[CAN] Received IMU: ax=0.04 ay=0.03 az=9.81 gx=0.12 gy=-0.05 gz=0.08
[CAN] Received IMU: ax=0.05 ay=0.02 az=9.80 gx=0.11 gy=-0.04 gz=0.09
...
```

These should appear on both boards at 10 Hz. If you don't see these messages:

1. **Check CAN bus wiring** — confirm CAN+, CAN-, and GND are connected
2. **Verify termination** — check that CAN_TERM1 is bridged on both boards
3. **Check baud rate** — should be 1 Mbps (default in firmware)
4. **Verify IMU is working** — check that local IMU initialization succeeded (no FAULT LED)

## Troubleshooting

### No "[CAN] Received IMU" messages appearing
- Verify CAN bus wiring and termination (see Hardware Requirements)
- Check that both boards' IMU are initialized (FAULT LED should not be solid)
- Inspect CAN connector pins for corrosion or poor contact

### Intermittent CAN communication
- Likely a termination or wiring issue
- Ensure both CAN_TERM1 jumpers are bridged
- Check cable length is <3m for 1 Mbps (longer cables may need lower baud rates)

### CAN TX/RX statistics not incrementing (visible in web UI)
- The board may not be transmitting or receiving
- Check the firmware build includes can_protocol.h (it should)
- Verify canInit() was called in setup() at line 273

## Code References

- **Protocol definition**: `src/can_protocol.h`
- **CAN bus driver**: `src/can_bus.h/cpp`
- **Communication logic**: `src/main.cpp` (lines 185–220 for CAN TX/RX functions)
- **Hardware pinout**: `CIRCUIT_DIAGRAM.md` → CAN Bus section

## Next Steps

Once communication is working, you can:
1. Add more message types (e.g., battery voltage, PWM commands)
2. Lower the transmit rate to save bandwidth
3. Add CAN filtering to respond only to specific boards/messages
4. Integrate remote IMU data into your control logic

