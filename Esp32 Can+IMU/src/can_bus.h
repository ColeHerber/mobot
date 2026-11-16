#pragma once
#include <stdint.h>

struct CanFrame {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
    bool     extended;
};

// Initialize TWAI peripheral at given baud rate (default 1 Mbps for DroneCAN).
// CAN_REQUIRE_ACK build flag controls whether normal or no-ack mode is used.
bool canInit(uint32_t baudRate = 1000000);

// Send a frame. Returns false if TX queue full or driver not running.
bool canSend(uint32_t id, const uint8_t* data, uint8_t len, bool extended = false);

// Non-blocking receive. Returns false if no frame waiting.
bool canReceive(CanFrame& frame);

// Cumulative TX/RX ok and error counts.
void canGetStats(uint32_t& txOk, uint32_t& txErr, uint32_t& rxOk, uint32_t& rxErr);

bool canIsRunning();
