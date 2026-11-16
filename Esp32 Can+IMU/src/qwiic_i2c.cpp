#include "qwiic_i2c.h"
#include "pins.h"
#include <Arduino.h>
#include <Wire.h>

void qwiicI2cInit() {
    Wire.begin(I2C0_SDA, I2C0_SCL);
    Serial.println("[I2C0] Qwiic bus up on IO8/IO9 (no device driver attached yet)");
    Serial0.println("[I2C0] Qwiic bus up (no device driver attached yet)");
}
