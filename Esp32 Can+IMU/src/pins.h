#pragma once

// Pin map for the v2 board. Source of truth: CIRCUIT_DIAGRAM.md (generated
// from review_exports/schematic_netlist.xml). If they disagree, trust that
// file over this one and fix this file.

// IMU - ISM6HG256X on SPI2 (FSPI)
#define IMU_MOSI   35
#define IMU_MISO   37
#define IMU_SCK    36
#define IMU_CS     21
#define IMU_INT1   38   // data-ready interrupt
#define IMU_INT2   39   // high-g accelerometer data-ready interrupt

// CAN — SN65HVD232 via ESP32 TWAI peripheral
#define CAN_TX     16
#define CAN_RX     17

// PWM outputs — 3.3V logic, servo/ESC capable (J7–J10)
#define PWM_PIN_0   14   // CH1 — J7
#define PWM_PIN_1   13   // CH2 — J8
#define PWM_PIN_2   12   // CH3 — J9
#define PWM_PIN_3   11   // CH4 — J10

// Analog inputs — 3.9k/2.2k voltage divider, ~0.361 scale (J12–J15)
#define ADC_PIN_0   4   // CH1 — J12
#define ADC_PIN_1   5   // CH2 — J13
#define ADC_PIN_2   6   // CH3 — J14
#define ADC_PIN_3   7   // CH4 — J15

// Status LEDs (active high). LED_LIFE = GPIO2, a normal I/O pin at module
// pin 38 — do not confuse with physical module pin 2, which is +3.3V power.
#define LED_LIFE   2    // heartbeat / status
#define LED_FAULT  1    // fault indicator
#define LED_RGB    15   // WS2812B-2020 addressable RGB (needs a NeoPixel-style driver)

// Qwiic I2C (J11) — bus brought up by qwiic_i2c stub, no device driver yet.
#define I2C0_SDA   8
#define I2C0_SCL   9

// UART1 (J4) — bus brought up by uart1 stub, no consumer yet.
#define UART1_RX   41
#define UART1_TX   42
