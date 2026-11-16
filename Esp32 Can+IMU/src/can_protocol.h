#pragma once
#include <stdint.h>
#include "imu.h"

// CAN message protocol for inter-board communication
// Both v1 and v2 boards use this protocol to exchange IMU data and status

namespace CanProtocol {

// Message type identifiers (bits 4-7 of CAN ID)
enum class MessageType : uint8_t {
    IMU_DATA    = 0x01,
    PING        = 0x02,
    STATUS      = 0x03,
};

// CAN frame structure for gyro data sharing
struct ImuDataFrame {
    uint8_t  msg_type;      // 0x01
    uint8_t  seq;           // Sequence number
    int16_t  gx;            // Gyro X (dps * 100)
    int16_t  gy;            // Gyro Y (dps * 100)
    int16_t  gz;            // Gyro Z (dps * 100)
};

struct PingFrame {
    uint8_t  msg_type;      // 0x02
    uint8_t  board_id;      // 1=v1, 2=v2
    uint32_t timestamp;     // millis()
};

// CAN IDs: 11-bit standard frames
// Format: [PRIO:2][TYPE:4][BOARD:3][FRAME:2]
// BOARD: 1=v1, 2=v2
// FRAME: for multi-frame messages (0=frameA, 1=frameB)
constexpr uint32_t canIdFor(uint8_t board_id, MessageType type, uint8_t frame_index = 0) {
    return (0x1 << 10) |  // Priority (low=1)
           ((uint8_t)type << 6) |
           (board_id << 3) |
           (frame_index & 0x3);
}

// Encode gyro data into CAN frame
inline void encodeGyroData(const ImuData& data, uint8_t seq, uint8_t* frame) {
    frame[0] = (uint8_t)MessageType::IMU_DATA;
    frame[1] = seq;
    frame[2] = (int16_t)(data.gx * 100) >> 8;
    frame[3] = (int16_t)(data.gx * 100) & 0xFF;
    frame[4] = (int16_t)(data.gy * 100) >> 8;
    frame[5] = (int16_t)(data.gy * 100) & 0xFF;
    frame[6] = (int16_t)(data.gz * 100) >> 8;
    frame[7] = (int16_t)(data.gz * 100) & 0xFF;
}

// Decode gyro data from CAN frame
inline void decodeGyroData(const uint8_t* data, ImuData& out) {
    int16_t gx = ((int16_t)data[2] << 8) | data[3];
    int16_t gy = ((int16_t)data[4] << 8) | data[5];
    int16_t gz = ((int16_t)data[6] << 8) | data[7];
    out.gx = gx / 100.0f;
    out.gy = gy / 100.0f;
    out.gz = gz / 100.0f;
}

}  // namespace CanProtocol
