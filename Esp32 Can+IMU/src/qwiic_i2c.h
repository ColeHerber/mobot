#pragma once

// Stub driver for the Qwiic I2C bus (J11, IO8 SDA0 / IO9 SCL0).
// Brings the bus up so a future Qwiic peripheral driver can call Wire.*
// directly, without anyone having to touch pins.h/main.cpp again. No device
// is attached yet.

void qwiicI2cInit();
