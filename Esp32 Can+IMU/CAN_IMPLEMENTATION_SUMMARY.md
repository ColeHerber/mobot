# CAN Communication Implementation Summary

## What Was Implemented

Added bidirectional CAN communication between v1 and v2 ESP32 boards so they can exchange IMU data via the CAN bus.

### Key Components

1. **can_protocol.h** (NEW)
   - Defines the CAN protocol message types and encoding/decoding functions
   - IMU data is sent as two CAN frames (accel + gyro)
   - Uses big-endian byte order for multi-byte values
   - CAN IDs encode: priority (2 bits) + message type (4 bits) + board ID (3 bits) + frame index (3 bits)

2. **main.cpp** (MODIFIED)
   - `canTxImuData()`: Encodes local IMU data into two CAN frames and transmits
   - `canProcessReceivedFrames()`: Receives CAN frames, decodes IMU data, logs to serial
   - Added periodic CAN transmission at 10 Hz (100 ms interval)
   - Integration with main loop to send local IMU and receive remote IMU

### Hardware Configuration

Both boards use:
- **CAN TX**: GPIO16 (TWAI TX)
- **CAN RX**: GPIO17 (TWAI RX)
- **Baud Rate**: 1 Mbps
- **Mode**: TWAI_MODE_NO_ACK (no ACK frames required)

Termination resistor (120Ω) must be enabled via CAN_TERM1 solder jumper on **both boards**.

## Data Flow

```
V1 Board                                V2 Board
┌─────────────────────────┐            ┌─────────────────────────┐
│ Local IMU (ax,ay,az,    │            │ Local IMU (ax,ay,az,    │
│ gx,gy,gz)               │            │ gx,gy,gz)               │
└─────┬───────────────────┘            └─────┬───────────────────┘
      │                                      │
      v (every 100ms)                        v (every 100ms)
  canTxImuData()                         canTxImuData()
      │                                      │
      ├─ Frame A (accel) ──────┐            │
      ├─ Frame B (gyro)  ──────┤            │
                               v            v
                          CAN Bus (1 Mbps)
                           ↑    ↓
      ┌──────────────────┤    ├──────────────────┐
      v                                         v
  canProcess...()                         canProcess...()
      │                                      │
      v                                      v
  remoteBoardIMU                          remoteBoardIMU
  Serial: "[CAN] Received IMU..."        Serial: "[CAN] Received IMU..."
```

## Message Format

### Frame A (Accelerometer)
```
Byte 0:   Message Type (0x01 = IMU_DATA)
Byte 1:   Sequence Number
Bytes 2-3: ax (int16, big-endian, units: mg)
Bytes 4-5: ay (int16, big-endian, units: mg)
Bytes 6-7: az (int16, big-endian, units: mg)
```

### Frame B (Gyroscope)
```
Byte 0:   Message Type (0x01 = IMU_DATA)
Byte 1:   Sequence Number (matches Frame A)
Bytes 2-3: gx (int16, big-endian, units: dps×100)
Bytes 4-5: gy (int16, big-endian, units: dps×100)
Bytes 6-7: gz (int16, big-endian, units: dps×100)
```

## Serial Output

When receiving IMU data from the remote board, you'll see:
```
[CAN] Received IMU: ax=0.04 ay=0.03 az=9.81 gx=0.12 gy=-0.05 gz=0.08
[CAN] Received IMU: ax=0.05 ay=0.02 az=9.80 gx=0.11 gy=-0.04 gz=0.09
```

This appears at 10 Hz (every 100ms) when both boards are healthy and CAN bus is connected.

## Build Status

✅ v1-debug: SUCCESS (912,069 bytes flash used)
✅ v2-debug: SUCCESS (912,077 bytes flash used)

Both boards use identical firmware. They differentiate themselves at runtime via the board ID in CAN messages (hard-coded to 1 for v1, 2 for v2).

## Testing Checklist

- [ ] Bridge CAN_TERM1 solder jumper on both boards
- [ ] Connect CAN+, CAN-, and GND between v1 J1/J2 and v2 J1/J2
- [ ] Flash v1 and v2 with latest firmware
- [ ] Open serial monitors for both boards (115200 baud)
- [ ] Power up both boards
- [ ] Verify "[CAN] Received IMU..." messages appear on both boards at 10 Hz
- [ ] Move/rotate boards and verify values change
- [ ] If no messages: check wiring, termination, and IMU health

## Files Changed/Created

- ✅ **NEW**: `src/can_protocol.h` — Protocol definitions and codec
- ✅ **MODIFIED**: `src/main.cpp` — Added CAN TX/RX logic
- 📄 **NEW**: `CAN_COMMUNICATION_SETUP.md` — Detailed setup guide
- 📄 **NEW**: `CAN_IMPLEMENTATION_SUMMARY.md` — This file

## Next Steps / Future Enhancements

1. **Display remote IMU on web UI**: Add `remoteBoardImuData` to the web UI state struct
2. **Add more message types**: Battery voltage, PWM commands, status flags
3. **Extend message protocol**: Use extended CAN IDs (29-bit) for more address space
4. **Redundancy**: Add message timeout detection and heartbeat messages
5. **Lower transmit rate**: Change CAN_TX_INTERVAL_MS to reduce bus load if needed
6. **Filtering**: Use CAN ID filtering to only accept messages from specific boards

