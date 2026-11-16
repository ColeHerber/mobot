#pragma once
#include "imu.h"
#include "can_bus.h"
#include <stdint.h>

struct WebUiState {
    ImuData  imu;
    float    adc[4];
    uint16_t pwm[4];
    uint32_t canTxOk;
    uint32_t canTxErr;
    uint32_t canRxOk;
    uint32_t canRxErr;
    bool     imuOk;
    int32_t  rssi;
    uint32_t freeHeap;
};

// Start the async web server (call once in setup, after WiFi begins).
void webUiInit();

// Push a data snapshot over SSE to all connected browsers.
// Pass any new CAN frames received since last call (up to 8).
void webUiPushSse(const WebUiState& state,
                  const CanFrame* newFrames = nullptr,
                  uint8_t frameCount = 0);

// Printf-style debug logger: writes to Serial and stores in ring buffer
// for the /api/log endpoint.
void dbgLog(const char* fmt, ...);
