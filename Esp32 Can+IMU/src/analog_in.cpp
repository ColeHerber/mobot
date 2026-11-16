#include "analog_in.h"
#include "pins.h"
#include <Arduino.h>

static const uint8_t PINS[4] = { ADC_PIN_0, ADC_PIN_1, ADC_PIN_2, ADC_PIN_3 };

// v2 board divider: RN4 (3.9k, top, from signal) / RN3 (2.2k, bottom, to GND).
// Vpin = Vsig * 2.2/(3.9+2.2)  ->  Vsig = Vpin / DIVIDER_RATIO
static constexpr float DIVIDER_RATIO = 2.2f / (3.9f + 2.2f);  // 0.361
static constexpr float VREF = 3.3f;
static constexpr float ADC_FULL_SCALE = 4095.0f;

void analogInInit() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // 0–3.3V input range
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(PINS[i], INPUT);
    }
    Serial.println("[ADC] Init OK — 4 channels (12-bit, 16-sample average)");
    Serial0.println("[ADC] Init OK");
}

uint16_t analogInReadRaw(uint8_t ch) {
    if (ch >= 4) return 0;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 16; i++) sum += analogRead(PINS[ch]);
    return (uint16_t)(sum >> 4);  // divide by 16
}

float analogInReadV(uint8_t ch) {
    const uint16_t raw = analogInReadRaw(ch);
    return (raw / ADC_FULL_SCALE) * VREF / DIVIDER_RATIO;
}
