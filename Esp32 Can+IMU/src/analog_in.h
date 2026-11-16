#pragma once
#include <stdint.h>

// 4-channel ADC on IO4/IO5/IO6/IO7 (ADC1 — safe while WiFi is active), J12–J15.
// Hardware: 3.9k (top) / 2.2k (bottom) voltage divider ahead of each pin.
// Scale factor 2.2/(3.9+2.2) = 0.361 -> ~9.1 V full scale at 3.3 V ADC FS.

void     analogInInit();

// Returns estimated voltage at the connector pin, in volts (0–~9.1 V).
float    analogInReadV(uint8_t ch);

// Raw 12-bit ADC count (averaged over 16 samples).
uint16_t analogInReadRaw(uint8_t ch);
