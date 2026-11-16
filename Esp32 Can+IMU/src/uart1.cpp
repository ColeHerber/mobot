#include "uart1.h"
#include "pins.h"
#include <Arduino.h>

void uart1Init() {
    Serial1.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
    Serial.println("[UART1] Bus up at 115200 baud on IO41/IO42 (no consumer attached yet)");
    Serial0.println("[UART1] Bus up (no consumer attached yet)");
}
