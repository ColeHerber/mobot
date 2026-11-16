#include "imu.h"
#include "pins.h"
#include "web_ui.h"
#include <Arduino.h>
#include <SPI.h>

// Shared primary-interface register map. The supported IMUs use the same
// control addresses, but have different identities and accelerometer ordering.
namespace reg {
    constexpr uint8_t INT1_CTRL     = 0x0D;  // interrupt routing
    constexpr uint8_t INT2_CTRL     = 0x0E;
    constexpr uint8_t WHO_AM_I      = 0x0F;
    constexpr uint8_t CTRL1         = 0x10;  // accel ODR / op-mode
    constexpr uint8_t CTRL2        = 0x11;  // gyro ODR / full-scale
    constexpr uint8_t CTRL3         = 0x12;  // BOOT (bit7), BDU, IF_INC, SW_RESET
    constexpr uint8_t CTRL4         = 0x13;  // DRDY_MASK (bit3): mask DRDY until filters settle
    constexpr uint8_t CTRL6         = 0x15;  // gyro full-scale + LPF1 bandwidth select
    constexpr uint8_t CTRL7         = 0x16;  // gyro LPF1 enable + high-g DRDY routing
    constexpr uint8_t CTRL8         = 0x17;  // accel full-scale + LPF2/HPF bandwidth select
    constexpr uint8_t CTRL9         = 0x18;  // accel LPF2/HPF enable + path select
    constexpr uint8_t STATUS_REG    = 0x1E;  // data-ready flags
    constexpr uint8_t OUTX_L_G      = 0x22;  // gyro XYZ (6 bytes)
    constexpr uint8_t OUTA_FIRST    = 0x28;  // six accel bytes; order is model-specific
    constexpr uint8_t OUTHG_FIRST   = 0x34;  // six high-g accel bytes, X,Y,Z order (ISM6HG256X only)
    constexpr uint8_t CTRL2_XL_HG   = 0x4D;  // high-g accel self-test (ISM6HG256X only)
    constexpr uint8_t CTRL1_XL_HG   = 0x4E;  // high-g accel enable/ODR/full-scale (ISM6HG256X only)
}

// ISM6HG256X: WHO_AM_I = 0x73
// Requires CTRL6 bit 3 set; FS_G[2:0]=100 selects +/-2000 dps
constexpr uint8_t WHO_AM_I_EXPECTED = 0x73;
constexpr uint8_t GYRO_CTRL6_VALUE = 0x0C;
constexpr char IMU_MODEL_NAME[] = "ISM6HG256X";

#ifndef IMU_SPI_DEBUG
#define IMU_SPI_DEBUG 0
#endif

#ifndef IMU_SPI_SCOPE_LOOP
#define IMU_SPI_SCOPE_LOOP 0
#endif

#ifndef IMU_SPI_SCOPE_LOOP_COUNT
#define IMU_SPI_SCOPE_LOOP_COUNT 300
#endif

// ===========================================================================
// BRING-UP PATCH NOTE - ISM6HG256XTR LOW-G ACCEL SCALE
// ===========================================================================
// Board under test: ISM6HG256XTR, WHO_AM_I = 0x73.
//
// IMPORTANT: keep this conversion datasheet-matched. A temporary experiment
// used 1/32768 g/count because the raw vector magnitude was ~31300 counts at
// rest, but that only forced |a| near 1g while hiding the real problem: when the
// PCB was flat on a countertop, raw X and raw Y were still both large positive
// values. That is not a scalar sensitivity error; it means the register data,
// axis mapping, or sensor/output-chain setup still needs investigation.
//
// Current diagnostic state:
//   CTRL8=0x03 -> low-g accel full scale ±16g
//   ACCEL_SCALE=0.000488 g/LSB per ISM6HG256X datasheet
//
// This wider range is temporary bring-up instrumentation: if the raw low-g
// accel registers still rail near +/-32767 at rest, the problem is not a
// legitimate +/-4g clipping event.
//
// Raw register counts are preserved untouched in ImuData::rawAx/rawAy/rawAz,
// UART JSON, and the MQTT raw stream so the axis bug can be debugged directly.
// ===========================================================================
constexpr uint8_t LOW_G_ACCEL_CTRL8_VALUE = 0x03;
constexpr float ACCEL_SCALE = 0.000488f;
// Gyro sensitivity for ±2000 dps: 70 mdps/LSB
constexpr float GYRO_SCALE = 0.070f;
// High-g accel sensitivity for ±256g: 10.4 mg/LSB (ISM6HG256X DS15034 Rev 2)
constexpr float HG_ACCEL_SCALE = 0.0104f;
constexpr uint32_t IMU_SPI_HZ = 1000000;
constexpr uint16_t BITBANG_SPI_EDGE_DELAY_US = 25;

static SPIClass imuSpi(FSPI);
static uint8_t  whoAmIByte = 0x00;
static bool     initOk = false;
static uint8_t  spiMode = SPI_MODE3;
static uint32_t lastAccelSatLogMs = 0;

static bool nearInt16Rail(int16_t value) {
    return value >= 32000 || value <= -32000;
}

static uint8_t spiReadWithSettings(uint8_t reg, uint8_t mode, uint32_t hz) {
    digitalWrite(IMU_CS, HIGH);
    delayMicroseconds(5);
    imuSpi.beginTransaction(SPISettings(hz, MSBFIRST, mode));
    digitalWrite(IMU_CS, LOW);
    delayMicroseconds(2);
    imuSpi.transfer(reg | 0x80);  // bit7=1 → read
    const uint8_t val = imuSpi.transfer(0x00);
    digitalWrite(IMU_CS, HIGH);
    imuSpi.endTransaction();
    return val;
}

static uint8_t spiReadWithMode(uint8_t reg, uint8_t mode) {
    return spiReadWithSettings(reg, mode, IMU_SPI_HZ);
}

static uint8_t spiRead(uint8_t reg) {
    return spiReadWithMode(reg, spiMode);
}

static void spiWrite(uint8_t reg, uint8_t val) {
    imuSpi.beginTransaction(SPISettings(IMU_SPI_HZ, MSBFIRST, spiMode));
    digitalWrite(IMU_CS, LOW);
    imuSpi.transfer(reg & 0x7F);  // bit7=0 → write
    imuSpi.transfer(val);
    digitalWrite(IMU_CS, HIGH);
    imuSpi.endTransaction();
}

// Read `len` bytes starting at `reg` using auto-increment (IF_INC must be set).
static void spiBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    imuSpi.beginTransaction(SPISettings(IMU_SPI_HZ, MSBFIRST, spiMode));
    digitalWrite(IMU_CS, LOW);
    imuSpi.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) buf[i] = imuSpi.transfer(0x00);
    digitalWrite(IMU_CS, HIGH);
    imuSpi.endTransaction();
}

#if IMU_SPI_DEBUG || IMU_SPI_SCOPE_LOOP
struct BitBangSpiResult {
    uint16_t raw;
    uint8_t  value;
};

static const char* spiModeLabel(uint8_t mode) {
    return mode == SPI_MODE0 ? "mode0" : "mode3";
}

static void logSpiGpioState(const char* label) {
    dbgLog("[IMU][SPIDBG] GPIO %s CS=%d SCK=%d MOSI=%d MISO=%d",
           label,
           digitalRead(IMU_CS),
           digitalRead(IMU_SCK),
           digitalRead(IMU_MOSI),
           digitalRead(IMU_MISO));
}

static void prepareBitBangSpi(uint8_t mode) {
    const uint8_t idleClock = (mode == SPI_MODE3) ? HIGH : LOW;

    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_SCK, idleClock);
    pinMode(IMU_SCK, OUTPUT);
    digitalWrite(IMU_MOSI, LOW);
    pinMode(IMU_MOSI, OUTPUT);
    pinMode(IMU_MISO, INPUT);
    delayMicroseconds(100);
}

static uint8_t clockBitBangSpiBit(uint8_t mode, bool txBit) {
    uint8_t rxBit = 0;
    digitalWrite(IMU_MOSI, txBit ? HIGH : LOW);
    delayMicroseconds(BITBANG_SPI_EDGE_DELAY_US);

    if (mode == SPI_MODE3) {
        digitalWrite(IMU_SCK, LOW);
        delayMicroseconds(BITBANG_SPI_EDGE_DELAY_US);
        digitalWrite(IMU_SCK, HIGH);
        delayMicroseconds(BITBANG_SPI_EDGE_DELAY_US);
        rxBit = digitalRead(IMU_MISO) ? 1 : 0;
    } else {
        digitalWrite(IMU_SCK, HIGH);
        delayMicroseconds(BITBANG_SPI_EDGE_DELAY_US);
        rxBit = digitalRead(IMU_MISO) ? 1 : 0;
        digitalWrite(IMU_SCK, LOW);
        delayMicroseconds(BITBANG_SPI_EDGE_DELAY_US);
    }

    return rxBit;
}

static BitBangSpiResult bitBangSpiReadReg(uint8_t reg, uint8_t mode) {
    prepareBitBangSpi(mode);

    const uint16_t tx = static_cast<uint16_t>(reg | 0x80) << 8;
    uint16_t raw = 0;

    digitalWrite(IMU_CS, LOW);
    delayMicroseconds(10);
    for (uint8_t bit = 0; bit < 16; bit++) {
        const bool txBit = (tx & (0x8000 >> bit)) != 0;
        raw = static_cast<uint16_t>((raw << 1) | clockBitBangSpiBit(mode, txBit));
    }
    delayMicroseconds(5);
    digitalWrite(IMU_CS, HIGH);

    const uint8_t idleClock = (mode == SPI_MODE3) ? HIGH : LOW;
    digitalWrite(IMU_SCK, idleClock);
    digitalWrite(IMU_MOSI, LOW);

    BitBangSpiResult result;
    result.raw = raw;
    result.value = raw & 0xFF;
    return result;
}

static void runHardwareSpiSweep(const char* label) {
    constexpr uint32_t speeds[] = {100000, 400000, 1000000};
    constexpr uint8_t modes[] = {SPI_MODE3, SPI_MODE0};

    dbgLog("[IMU][SPIDBG] HW SPI sweep: %s", label);
    for (uint8_t i = 0; i < sizeof(speeds) / sizeof(speeds[0]); i++) {
        for (uint8_t j = 0; j < sizeof(modes) / sizeof(modes[0]); j++) {
            const uint8_t who = spiReadWithSettings(reg::WHO_AM_I, modes[j], speeds[i]);
            dbgLog("[IMU][SPIDBG] HW %lu Hz %s WHO=0x%02X",
                   static_cast<unsigned long>(speeds[i]),
                   spiModeLabel(modes[j]),
                   who);
        }
    }
}

static void logSpiDebugHeader() {
    dbgLog("[IMU][SPIDBG] ==================================================");
    dbgLog("[IMU][SPIDBG] %s SPI debug sequence", IMU_MODEL_NAME);
    dbgLog("[IMU][SPIDBG] Order: HW SPI sweep -> bit-bang SPI sweep -> optional scope loop");
    dbgLog("[IMU][SPIDBG] Expected SPI WHO: CS low, MOSI=0x8F, 16 clocks, MISO byte2=0x%02X",
           WHO_AM_I_EXPECTED);
    dbgLog("[IMU][SPIDBG] ==================================================");
}

static void logSpiDebugPowerCyclePrompt() {
    dbgLog("[IMU][SPIDBG] Debug sequence complete.");
    dbgLog("[IMU][SPIDBG] Before rerunning after any interface-mode experiment: physically power-cycle the board.");
}

static void runBitBangSpiSweep(const char* label) {
    constexpr uint8_t modes[] = {SPI_MODE3, SPI_MODE0};

    dbgLog("[IMU][SPIDBG] bit-banged SPI sweep: %s", label);
    for (uint8_t i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
        const BitBangSpiResult who = bitBangSpiReadReg(reg::WHO_AM_I, modes[i]);
        dbgLog("[IMU][SPIDBG] BB %s raw=0x%04X WHO=0x%02X",
               spiModeLabel(modes[i]), who.raw, who.value);
    }
}

static void runSpiScopeLoop() {
    dbgLog("[IMU][SPIDBG] scope loop: repeating slow bit-banged mode3 WHO reads");
    dbgLog("[IMU][SPIDBG] scope target: CS low, 16 clocks, MOSI first byte 0x8F, MISO second byte 0x%02X",
           WHO_AM_I_EXPECTED);
    dbgLog("[IMU][SPIDBG] loop count=%u; set IMU_SPI_SCOPE_LOOP_COUNT=0 for endless UART-only probing",
           static_cast<unsigned>(IMU_SPI_SCOPE_LOOP_COUNT));

    uint32_t count = 0;
    while (IMU_SPI_SCOPE_LOOP_COUNT == 0 || count < IMU_SPI_SCOPE_LOOP_COUNT) {
        const BitBangSpiResult who = bitBangSpiReadReg(reg::WHO_AM_I, SPI_MODE3);
        if ((count % 10) == 0) {
            dbgLog("[IMU][SPIDBG] scope sample %lu raw=0x%04X WHO=0x%02X",
                   static_cast<unsigned long>(count), who.raw, who.value);
        }
        count++;
        delay(100);
    }
    dbgLog("[IMU][SPIDBG] scope loop complete; continuing boot so OTA can recover");
}
#endif

#if IMU_SPI_DEBUG
static void runSpiDebugPass() {
    logSpiDebugHeader();

    pinMode(IMU_MISO, INPUT);
    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_CS, OUTPUT);
    delay(2);
    logSpiGpioState("idle before sweep");

    runHardwareSpiSweep("initial");
    runBitBangSpiSweep("initial");

#if IMU_SPI_SCOPE_LOOP
    runSpiScopeLoop();
#endif

    logSpiDebugPowerCyclePrompt();

    // SPIClass::begin() is a no-op while its bus is already active. The
    // bit-bang tests changed the GPIO routing after the first begin(), so the
    // peripheral must be ended and started again to reattach SCK/MISO/MOSI.
    imuSpi.end();
    imuSpi.begin(IMU_SCK, IMU_MISO, IMU_MOSI, -1);
    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_CS, OUTPUT);
    delay(2);
    logSpiGpioState("after debug restore");
}
#endif

static uint8_t regRead(uint8_t reg) {
    return spiRead(reg);
}

static void regWrite(uint8_t reg, uint8_t val) {
    spiWrite(reg, val);
}

static void regBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    spiBurst(reg, buf, len);
}

bool imuInit() {
    dbgLog("[IMU] Build target: %s (WHO_AM_I 0x%02X)", IMU_MODEL_NAME, WHO_AM_I_EXPECTED);

    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);
    pinMode(IMU_INT1, INPUT);
    pinMode(IMU_INT2, INPUT);

    // Pass -1 for ss so the SPI driver never attaches IMU_CS to the hardware
    // peripheral's own CS line — CS is managed manually below via digitalWrite
    // across multiple transfer() calls, which requires it to stay a plain GPIO.
    imuSpi.begin(IMU_SCK, IMU_MISO, IMU_MOSI, -1);
    delay(10);

#if IMU_SPI_DEBUG
    runSpiDebugPass();
#endif

    const uint8_t whoMode3 = spiReadWithMode(reg::WHO_AM_I, SPI_MODE3);
    const uint8_t whoMode0 = spiReadWithMode(reg::WHO_AM_I, SPI_MODE0);
    dbgLog("[IMU] WHO probe: mode3=0x%02X mode0=0x%02X", whoMode3, whoMode0);
    if (whoMode3 == WHO_AM_I_EXPECTED) {
        spiMode = SPI_MODE3;
        dbgLog("[IMU] Using SPI mode 3");
    } else if (whoMode0 == WHO_AM_I_EXPECTED) {
        spiMode = SPI_MODE0;
        dbgLog("[IMU] Using SPI mode 0");
    } else {
        spiMode = SPI_MODE3;
        dbgLog("[IMU] SPI did not answer with the expected WHO_AM_I in mode 3 or mode 0");
    }

    // BOOT (CTRL3 bit7): reboot memory content to reload the factory calibration
    // trim. This is distinct from SW_RESET below — SW_RESET only returns the
    // control registers to their defaults and does NOT reload the trim. The
    // datasheet (DS15034 Rev 2, p.57) notes the boot-loaded registers "contain
    // the factory calibration values." Those load automatically at power-up, but
    // a marginal power ramp (hand-reworked / pogo-pin-powered board) can leave
    // that load incomplete — which shows up as a large accel DC offset on every
    // axis while WHO_AM_I (a ROM constant) still reads correctly. Forcing BOOT
    // makes the trim reload deterministic.
    regWrite(reg::CTRL3, 0x80);
    // BOOT self-clears when the reload finishes. Poll bit7 with a generous
    // timeout (gyro turn-on time is 40 ms per datasheet) rather than guessing a
    // fixed delay.
    for (uint8_t i = 0; i < 50; i++) {
        delay(1);
        if ((regRead(reg::CTRL3) & 0x80) == 0) break;
    }
    delay(1);

    // Software reset
    regWrite(reg::CTRL3, 0x01);
    delay(10);

    whoAmIByte = regRead(reg::WHO_AM_I);
    dbgLog("[IMU] WHO_AM_I = 0x%02X (expected 0x%02X)", whoAmIByte, WHO_AM_I_EXPECTED);

    if (whoAmIByte != WHO_AM_I_EXPECTED) {
        if (whoAmIByte == 0x00 || whoAmIByte == 0xFF) {
            dbgLog("[IMU] ERROR: bus fault (0x00 or 0xFF). Check wiring.");
        } else {
            dbgLog("[IMU] ERROR: wrong IMU model or unexpected WHO_AM_I.");
        }
        initOk = false;
        return false;
    }

    // CTRL3: BDU=1 (block data update), IF_INC=1 (auto-increment for burst reads)
    regWrite(reg::CTRL3, 0x44);

    // CTRL4: DRDY_MASK=1 (bit3). Masks the accel/gyro data-ready signal until the
    // digital filters have settled after this configuration, so the first samples
    // read out aren't a startup transient. Defense-in-depth; independent of the
    // BOOT trim reload above.
    regWrite(reg::CTRL4, 0x08);

    // CTRL1: accel @ 1.92 kHz, high-performance mode
    // bits[6:4] = 000 (high perf), bits[3:0] = 1010 (1.92 kHz)
    regWrite(reg::CTRL1, 0x0A);

    // CTRL2: gyro @ 1.92 kHz, high-performance mode
    // bits[7,5:4] = 000 (high perf), bits[3:0] = 1010 (1.92 kHz)
    regWrite(reg::CTRL2, 0x0A);

    // CTRL6: gyro full scale +/-2000 dps. Encoding differs between models.
    // (LPF1_G_BW_[2:0] occupies the same byte's bits[3:1] but is moot while
    // LPF1_G_EN=0 in CTRL7 below.)
    regWrite(reg::CTRL6, GYRO_CTRL6_VALUE);

    // CTRL7: gyro LPF1 filter enable (bit0) + high-g accel DRDY routing (bit6).
    //   LPF1_G_EN=0: gyro digital low-pass filter stage left OFF. Left off
    //     deliberately so the raw noise floor is captured for Allan-deviation /
    //     noise-density characterization — enabling it would smooth the gyro
    //     output (bandwidth chosen via CTRL6.LPF1_G_BW_[2:0], see datasheet
    //     Table 66) at the cost of added phase lag; fine for production use
    //     later, not for characterizing the sensor itself.
    //   INT2_DRDY_XL_HG=1: routes the high-g accelerometer's data-ready signal
    //     to the INT2 pin. It needs its own interrupt because the high-g ADC
    //     chain runs on an independent, unsynchronized clock from the
    //     low-g-accel/gyro DRDY already routed to INT1 below.
    regWrite(reg::CTRL7, 0x40);

    // CTRL8: accel full scale ±16g (bits[1:0] = 11). bits[7:5] (HP_LPF2_XL_BW)
    // select the optional second-stage filter's cutoff but are moot while
    // CTRL9.LPF2_XL_EN=0 below.
    regWrite(reg::CTRL8, LOW_G_ACCEL_CTRL8_VALUE);

    // CTRL9: accel second-stage filter selection — all left OFF, deliberately,
    // for the same raw-characterization reason as CTRL7.LPF1_G_EN above.
    //   LPF2_XL_EN (bit3)=0: don't route through the optional second low-pass
    //     stage. (The always-on *first* stage, fixed at ODR/2 cutoff in
    //     high-performance mode, has no enable bit and can't be disabled —
    //     this bit only controls the optional extra stage.) Bandwidth would be
    //     CTRL8.HP_LPF2_XL_BW_[2:0] (datasheet Table 71, ODR/4..ODR/800) if
    //     this were enabled.
    //   HP_SLOPE_XL_EN (bit4)=0: when the second stage *is* active, this picks
    //     which filter it is — 0 = low-pass (keeps the gravity/DC component,
    //     needed for tilt/orientation), 1 = high-pass/slope (removes DC/
    //     gravity, isolating dynamic motion/vibration but losing orientation
    //     info). Moot here since LPF2_XL_EN=0 means neither path is engaged.
    //   HP_REF_MODE_XL (bit6)=0: high-pass "reference mode" variant, only
    //     meaningful when HP_SLOPE_XL_EN=1.
    regWrite(reg::CTRL9, 0x00);

    // CTRL1_XL_HG: enable the high-g accelerometer channel's output registers
    // (XL_HG_REGOUT_EN), 1.92 kHz ODR (matches low-g accel/gyro so all three
    // channels share a timebase, even though the ADC chains aren't phase-
    // locked), ±256g full scale (avoids clipping on real shock events).
    regWrite(reg::CTRL1_XL_HG, 0xAC);
    // CTRL2_XL_HG: self-test bits only — leave off.
    regWrite(reg::CTRL2_XL_HG, 0x00);

    // INT1_CTRL: route DRDY_XL (bit0) and DRDY_G (bit1) to INT1 pin
    regWrite(reg::INT1_CTRL, 0x03);

    delay(5);

    // Verify register readbacks. If WHO_AM_I is good but these do not match,
    // the bus is writing poorly or the transaction format is wrong.
    const uint8_t ctrl3Check = regRead(reg::CTRL3);
    const uint8_t ctrl4Check = regRead(reg::CTRL4);
    const uint8_t ctrl1Check = regRead(reg::CTRL1);
    const uint8_t ctrl2Check = regRead(reg::CTRL2);
    const uint8_t ctrl6Check = regRead(reg::CTRL6);
    const uint8_t ctrl7Check = regRead(reg::CTRL7);
    const uint8_t ctrl8Check = regRead(reg::CTRL8);
    const uint8_t ctrl9Check = regRead(reg::CTRL9);
    dbgLog("[IMU] CTRL3 readback = 0x%02X (expected 0x44)", ctrl3Check);
    dbgLog("[IMU] CTRL4 readback = 0x%02X (expected 0x08)", ctrl4Check);
    dbgLog("[IMU] CTRL1 readback = 0x%02X (expected 0x0A)", ctrl1Check);
    dbgLog("[IMU] CTRL2 readback = 0x%02X (expected 0x0A)", ctrl2Check);
    dbgLog("[IMU] CTRL6 readback = 0x%02X (expected 0x%02X)", ctrl6Check, GYRO_CTRL6_VALUE);
    dbgLog("[IMU] CTRL7 readback = 0x%02X (expected 0x40)", ctrl7Check);
    dbgLog("[IMU] CTRL8 readback = 0x%02X (expected 0x%02X)",
           ctrl8Check, LOW_G_ACCEL_CTRL8_VALUE);
    dbgLog("[IMU] CTRL9 readback = 0x%02X (expected 0x00)", ctrl9Check);

    bool configOk = ctrl3Check == 0x44 && ctrl4Check == 0x08 &&
                     ctrl1Check == 0x0A && ctrl2Check == 0x0A &&
                     ctrl6Check == GYRO_CTRL6_VALUE && ctrl7Check == 0x40 &&
                     ctrl8Check == LOW_G_ACCEL_CTRL8_VALUE && ctrl9Check == 0x00;

    const uint8_t ctrl1HgCheck = regRead(reg::CTRL1_XL_HG);
    dbgLog("[IMU] CTRL1_XL_HG readback = 0x%02X (expected 0xAC)", ctrl1HgCheck);
    configOk = configOk && (ctrl1HgCheck == 0xAC);

    if (!configOk) {
        dbgLog("[IMU] ERROR: configuration readback failed");
        initOk = false;
        return false;
    }

    initOk = true;
    dbgLog("[IMU] Init OK");
    return true;
}

bool imuAvailable() {
    if (!initOk) return false;
    return digitalRead(IMU_INT1) == HIGH;
}

bool imuAvailableHg() {
    if (!initOk) return false;
    return digitalRead(IMU_INT2) == HIGH;
}

bool imuRead(ImuData& out) {
    if (!initOk) return false;

    // 0x22-0x2D is one contiguous block: gyro XYZ (0x22-0x27) then accel XYZ
    // (0x28-0x2D). IF_INC is set (CTRL3), so a single 12-byte burst replaces
    // what used to be two separate 6-byte bursts — halves the CS-toggle/SPI
    // transaction overhead per sample, which matters at 1.92kHz.
    uint8_t buf[12];
    regBurst(reg::OUTX_L_G, buf, 12);

    const int16_t rawGx = (int16_t)(buf[1] << 8 | buf[0]);
    const int16_t rawGy = (int16_t)(buf[3] << 8 | buf[2]);
    const int16_t rawGz = (int16_t)(buf[5] << 8 | buf[4]);
    out.gx = rawGx * GYRO_SCALE;
    out.gy = rawGy * GYRO_SCALE;
    out.gz = rawGz * GYRO_SCALE;
    out.rawGx = rawGx;
    out.rawGy = rawGy;
    out.rawGz = rawGz;

    int16_t rawAx, rawAy, rawAz;
    // ISM6HG256X registers 0x28..0x2D (buf[6..11]) are ordered X, Y, Z.
    rawAx = (int16_t)(buf[7] << 8 | buf[6]);
    rawAy = (int16_t)(buf[9] << 8 | buf[8]);
    rawAz = (int16_t)(buf[11] << 8 | buf[10]);
    out.ax = rawAx * ACCEL_SCALE;
    out.ay = rawAy * ACCEL_SCALE;
    out.az = rawAz * ACCEL_SCALE;
    out.rawAx = rawAx;
    out.rawAy = rawAy;
    out.rawAz = rawAz;

    const uint32_t now = millis();
    if ((nearInt16Rail(rawAx) || nearInt16Rail(rawAy) || nearInt16Rail(rawAz)) &&
        now - lastAccelSatLogMs >= 1000) {
        lastAccelSatLogMs = now;
        const uint8_t status = regRead(reg::STATUS_REG);
        dbgLog("[IMU] ACCEL SAT status=0x%02X rawA=(%d,%d,%d) rawG=(%d,%d,%d) "
               "a_bytes=%02X %02X %02X %02X %02X %02X",
               status,
               rawAx, rawAy, rawAz,
               rawGx, rawGy, rawGz,
               buf[6], buf[7], buf[8], buf[9], buf[10], buf[11]);
    }

    out.ts = millis();
    out.tsUs = micros();
    return true;
}

bool imuReadHg(ImuData& out) {
    if (!initOk) return false;

    uint8_t buf[6];
    regBurst(reg::OUTHG_FIRST, buf, 6);

    const int16_t rawHgx = (int16_t)(buf[1] << 8 | buf[0]);
    const int16_t rawHgy = (int16_t)(buf[3] << 8 | buf[2]);
    const int16_t rawHgz = (int16_t)(buf[5] << 8 | buf[4]);
    out.hgx = rawHgx * HG_ACCEL_SCALE;
    out.hgy = rawHgy * HG_ACCEL_SCALE;
    out.hgz = rawHgz * HG_ACCEL_SCALE;
    out.rawHgx = rawHgx;
    out.rawHgy = rawHgy;
    out.rawHgz = rawHgz;
    out.tsHgUs = micros();
    return true;
}

const char* imuStatus() {
    if (whoAmIByte == 0x00 || whoAmIByte == 0xFF) return "SPI_ERROR";
    if (!initOk) return "NOT_FOUND";
    return "OK";
}

uint8_t imuWhoAmI() {
    return whoAmIByte;
}

const char* imuModelName() {
    return IMU_MODEL_NAME;
}
