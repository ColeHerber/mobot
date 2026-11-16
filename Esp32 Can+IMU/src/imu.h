#pragma once
#include <stdint.h>

struct ImuData {
    float    ax, ay, az;   // g
    float    gx, gy, gz;   // degrees/s
    uint32_t ts;            // millis() at read time

    // Raw register counts + high-resolution timestamp from the same burst
    // read as ax/ay/az/gx/gy/gz, for the MQTT raw-sample logger.
    int16_t  rawAx, rawAy, rawAz;
    int16_t  rawGx, rawGy, rawGz;
    uint32_t tsUs;          // micros() at low-g/gyro read time

    // High-g accelerometer channel (independent ODR/interrupt from the above).
    float    hgx, hgy, hgz;     // g
    int16_t  rawHgx, rawHgy, rawHgz;
    uint32_t tsHgUs;             // micros() at high-g read time
};

// Returns true on successful init (WHO_AM_I match + config write).
// Always call imuWhoAmI() and check Serial output if this returns false —
// the actual WHO_AM_I byte is printed regardless.
bool        imuInit();

// True if INT1 (data-ready) pin is high.
bool        imuAvailable();

// Read one low-g accel + gyro sample. Returns false if SPI fails.
bool        imuRead(ImuData& out);

// True if INT2 (high-g accel data-ready) pin is high.
bool        imuAvailableHg();

// Read one high-g accelerometer sample. Returns false if SPI fails.
bool        imuReadHg(ImuData& out);

// Human-readable status: "OK", "NOT_FOUND", or "SPI_ERROR"
const char* imuStatus();

// Raw WHO_AM_I byte read from register 0x0F (for debugging)
uint8_t     imuWhoAmI();

// IMU model name: ISM6HG256X
const char* imuModelName();
