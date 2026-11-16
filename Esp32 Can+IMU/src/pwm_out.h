#pragma once
#include <stdint.h>

// 4-channel servo/ESC PWM on IO14/IO13/IO12/IO11 (J7–J10).
// Default: 50 Hz, 16-bit resolution, 1500 µs (center).

void     pwmInit();

// Set pulse width in microseconds (clamped to 500–2500 µs).
void     pwmSetUs(uint8_t ch, uint16_t us);

// Change output frequency. Resets duty to 1500 µs on channel.
void     pwmSetFreqHz(uint8_t ch, float hz);

// Returns last commanded pulse width.
uint16_t pwmGetUs(uint8_t ch);
