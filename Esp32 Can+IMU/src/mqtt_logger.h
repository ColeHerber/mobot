#pragma once
#include "imu.h"

// Configure and connect the MQTT client (call once in setup(), after beginWifi()).
// No-ops at runtime if MQTT_BROKER_HOST is left at its placeholder in secrets.h.
void mqttLoggerInit();

// Service the MQTT connection (non-blocking reconnect) and drain buffered
// samples into fixed-size chunked publishes. Call once per loop() iteration.
void mqttLoggerService();

// Buffer one low-g accel + gyro sample. Call right after imuRead() succeeds.
void mqttLoggerPushLowG(const ImuData& d);

// Buffer one high-g accel sample. Call right after imuReadHg() succeeds.
void mqttLoggerPushHighG(const ImuData& d);
