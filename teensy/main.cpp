// Teensy 4.x firmware — CMU Mobot 2026
// QTRX-MD-16A 16-channel line sensor coprocessor
//
// Pin assignments:
//   LEDON_ODD:    Pin 2  (digital out) — enables odd  IR LEDs on QTRX (sensors 1,3,5,…)
//   LEDON_EVEN:   Pin 3  (digital out) — enables even IR LEDs on QTRX (sensors 2,4,6,…)
//   CTRL_ODD:     Pin 4  (digital out) — dimming control for odd  emitters (pulse protocol)
//   CTRL_EVEN:    Pin 5  (digital out) — dimming control for even emitters (pulse protocol)
//   Sensor ADC:   A0–A9, A14–A19 (analog in)  — direct wiring from QTRX outputs
//
// Dimming protocol (CTRL pin):
//   - Pulse low for 0.5–300 µs, then high ≥0.5 µs → advance one dimming level (32 levels)
//   - Pulse low for >1 ms → reset to level 0 (100% current)
//   - Level 0=100%, level 31=1.67%; wraps back to 100% after level 31
//
// Serial protocol (USB, 115200 baud):
//   Commands from Pi:
//     'C' — enter calibration mode (stream raw values, update min/max)
//     'S' — save calibration to EEPROM (while in calibration mode)
//     'R' — enter run mode (stream 8-byte packets at ~200 Hz)
//
//   Packet format (run mode, 8 bytes):
//     [0xAA] [pos_hi] [pos_lo] [flags_hi] [flags_lo] [confidence] [checksum] [0x55]
//     line_pos = int16(pos_hi<<8 | pos_lo) / 10000.0  → range [-1.0, +1.0]
//     flags    = 16-bit bitmask, bit N set if sensor N above threshold
//     confidence = sum of normalized values, scaled 0–255
//     checksum = XOR of bytes 1–5 (pos_hi through confidence)

#include <Arduino.h>
#include <EEPROM.h>

// ─── Pin / hardware constants ─────────────────────────────────────────────────
static const int LEDON_ODD_PIN  = 2;   // digital — enables odd  emitters
static const int LEDON_EVEN_PIN = 3;   // digital — enables even emitters
static const int CTRL_ODD_PIN   = 4;   // dimming pulses — odd  emitters
static const int CTRL_EVEN_PIN  = 5;   // dimming pulses — even emitters

// Dimming level 0–31 (0 = 100% current, 31 = 1.67%)
// Set both odd/even to the same level for uniform illumination
static const uint8_t LEDON_DIM_LEVEL = 0;  // 0 = full brightness

static const int NUM_SENSORS = 16;
static const int SENSOR_PINS[NUM_SENSORS] = {
    A9, A8, A7,  A6,  A5,  A4,  A3,  A2,
    A1, A0, A19, A18, A17, A16, A15, A14
};

// ─── EEPROM layout ────────────────────────────────────────────────────────────
// 2 bytes per sensor × 16 sensors × 2 arrays (min/max) = 64 bytes
static const int EEPROM_MAGIC_ADDR = 0;       // 2 bytes: magic number 0xAB12
static const int EEPROM_MIN_ADDR   = 2;       // 32 bytes: uint16 min[16]
static const int EEPROM_MAX_ADDR   = 34;      // 32 bytes: uint16 max[16]
static const uint16_t EEPROM_MAGIC = 0xAB12;

// ─── Calibration state ────────────────────────────────────────────────────────
uint16_t cal_min[NUM_SENSORS];
uint16_t cal_max[NUM_SENSORS];

// ─── Mode ────────────────────────────────────────────────────────────────────
enum Mode { MODE_RUN, MODE_CALIBRATE };
Mode current_mode = MODE_RUN;

// ─── Timing ──────────────────────────────────────────────────────────────────
static const uint32_t RUN_INTERVAL_US    = 5000;  // 200 Hz
static const uint32_t CAL_INTERVAL_US    = 10000; // 100 Hz during calibration
static const uint32_t LEDON_SETTLE_US    = 50;    // µs for phototransistor settle

// ─── EEPROM helpers ──────────────────────────────────────────────────────────
void eeprom_write_u16(int addr, uint16_t val) {
    EEPROM.write(addr,     (val >> 8) & 0xFF);
    EEPROM.write(addr + 1, val & 0xFF);
}

uint16_t eeprom_read_u16(int addr) {
    return ((uint16_t)EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
}

void load_calibration() {
    uint16_t magic = eeprom_read_u16(EEPROM_MAGIC_ADDR);
    if (magic != EEPROM_MAGIC) {
        // No valid calibration — use full ADC range defaults
        for (int i = 0; i < NUM_SENSORS; i++) {
            cal_min[i] = 0;
            cal_max[i] = 4095;
        }
        return;
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
        cal_min[i] = eeprom_read_u16(EEPROM_MIN_ADDR + i * 2);
        cal_max[i] = eeprom_read_u16(EEPROM_MAX_ADDR + i * 2);
        // Guard against degenerate calibration
        if (cal_max[i] <= cal_min[i]) {
            cal_min[i] = 0;
            cal_max[i] = 4095;
        }
    }
}

void save_calibration() {
    eeprom_write_u16(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
    for (int i = 0; i < NUM_SENSORS; i++) {
        eeprom_write_u16(EEPROM_MIN_ADDR + i * 2, cal_min[i]);
        eeprom_write_u16(EEPROM_MAX_ADDR + i * 2, cal_max[i]);
    }
}

void reset_calibration() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        cal_min[i] = 4095;
        cal_max[i] = 0;
    }
}

// ─── CTRL dimming ────────────────────────────────────────────────────────────
// Reset CTRL pin to level 0 (100%) then advance to target level.
// Reset: hold low >1 ms. Each advance: pulse low 1 µs, high 1 µs.
void set_dim_level(int ctrl_pin, uint8_t level) {
    if (level > 31) level = 31;
    // Reset to level 0
    digitalWrite(ctrl_pin, LOW);
    delayMicroseconds(1500);  // >1 ms reset pulse
    digitalWrite(ctrl_pin, HIGH);
    delayMicroseconds(1);
    // Advance to target level
    for (uint8_t i = 0; i < level; i++) {
        digitalWrite(ctrl_pin, LOW);
        delayMicroseconds(1);   // 0.5–300 µs low
        digitalWrite(ctrl_pin, HIGH);
        delayMicroseconds(1);   // ≥0.5 µs high
    }
}

// ─── Sensor read ─────────────────────────────────────────────────────────────
void read_sensors(uint16_t raw[NUM_SENSORS]) {
    digitalWrite(LEDON_ODD_PIN,  HIGH);
    digitalWrite(LEDON_EVEN_PIN, HIGH);
    delayMicroseconds(LEDON_SETTLE_US);
    for (int i = 0; i < NUM_SENSORS; i++) {
        raw[i] = analogRead(SENSOR_PINS[i]);
    }
    digitalWrite(LEDON_ODD_PIN,  LOW);
    digitalWrite(LEDON_EVEN_PIN, LOW);
}

// Normalize raw ADC value to 0–1000 using per-channel calibration
uint16_t normalize(uint16_t raw, int ch) {
    if (cal_max[ch] <= cal_min[ch]) return 0;
    if (raw <= cal_min[ch]) return 0;
    if (raw >= cal_max[ch]) return 1000;
    return (uint16_t)(((uint32_t)(raw - cal_min[ch]) * 1000) / (cal_max[ch] - cal_min[ch]));
}

// ─── Packet build & send ─────────────────────────────────────────────────────
void send_packet(uint16_t raw[NUM_SENSORS]) {
    uint16_t norm[NUM_SENSORS];
    uint32_t weighted_sum = 0;
    uint32_t total = 0;
    uint16_t flags = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        norm[i] = normalize(raw[i], i);
        // Weighted centroid: sensor positions mapped to [-7500, +7500] in units of 1000/step
        // Position of sensor i (0-indexed): maps to -7.5 to +7.5 (in steps of 1.0)
        // Scaled by 1000 to keep integer math: -7500 to +7500
        int32_t pos_scaled = ((int32_t)i - 7) * 1000 + 500; // center of each sensor slot
        weighted_sum += (uint32_t)norm[i] * (uint32_t)(pos_scaled + 7500); // shift to positive
        total += norm[i];
        if (norm[i] > 500) flags |= (1 << i);
    }

    // line_pos in range [-1.0, +1.0]
    int16_t line_pos_i16;
    if (total < 100) {
        line_pos_i16 = 0; // no line detected
    } else {
        // centroid 0..15000, center at 7500
        int32_t centroid = (int32_t)(weighted_sum / total) - 7500;
        // centroid range: -7500 to +7500, map to -10000..+10000
        int32_t pos = (centroid * 10000) / 7500;
        if (pos > 10000)  pos = 10000;
        if (pos < -10000) pos = -10000;
        line_pos_i16 = (int16_t)pos;
    }

    // Confidence: total normalized energy, scaled to 0–255
    // Max possible total: 16 × 1000 = 16000
    uint8_t confidence = (uint8_t)((total > 16000 ? 16000 : total) * 255 / 16000);

    uint8_t pos_hi  = (uint8_t)((line_pos_i16 >> 8) & 0xFF);
    uint8_t pos_lo  = (uint8_t)(line_pos_i16 & 0xFF);
    uint8_t fl_hi   = (uint8_t)((flags >> 8) & 0xFF);
    uint8_t fl_lo   = (uint8_t)(flags & 0xFF);
    uint8_t chk     = pos_hi ^ pos_lo ^ fl_hi ^ fl_lo ^ confidence;

    uint8_t pkt[8] = {0xAA, pos_hi, pos_lo, fl_hi, fl_lo, confidence, chk, 0x55};
    Serial.write(pkt, 8);
}

// ─── Calibration stream ──────────────────────────────────────────────────────
void send_cal_packet(uint16_t raw[NUM_SENSORS]) {
    // Send CSV of raw values for monitoring
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(raw[i]);
        if (i < NUM_SENSORS - 1) Serial.print(',');
    }
    Serial.println();
}

void update_cal_minmax(uint16_t raw[NUM_SENSORS]) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (raw[i] < cal_min[i]) cal_min[i] = raw[i];
        if (raw[i] > cal_max[i]) cal_max[i] = raw[i];
    }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    analogReadResolution(12); // Teensy 4.x supports 12-bit ADC
    pinMode(LEDON_ODD_PIN,  OUTPUT);
    pinMode(LEDON_EVEN_PIN, OUTPUT);
    pinMode(CTRL_ODD_PIN,   OUTPUT);
    pinMode(CTRL_EVEN_PIN,  OUTPUT);
    digitalWrite(LEDON_ODD_PIN,  LOW);
    digitalWrite(LEDON_EVEN_PIN, LOW);
    digitalWrite(CTRL_ODD_PIN,   HIGH);  // CTRL idle-high
    digitalWrite(CTRL_EVEN_PIN,  HIGH);
    // Set dimming level (LEDON must be high during CTRL pulses per datasheet)
    digitalWrite(LEDON_ODD_PIN,  HIGH);
    digitalWrite(LEDON_EVEN_PIN, HIGH);
    set_dim_level(CTRL_ODD_PIN,  LEDON_DIM_LEVEL);
    set_dim_level(CTRL_EVEN_PIN, LEDON_DIM_LEVEL);
    digitalWrite(LEDON_ODD_PIN,  LOW);
    digitalWrite(LEDON_EVEN_PIN, LOW);
    load_calibration();
    // Default: run mode
    current_mode = MODE_RUN;
}

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
    static uint32_t last_tick_us = 0;

    // Handle incoming commands (non-blocking)
    if (Serial.available() > 0) {
        char cmd = (char)Serial.read();
        if (cmd == 'C' || cmd == 'c') {
            current_mode = MODE_CALIBRATE;
            reset_calibration();
            Serial.println("CAL_START");
        } else if (cmd == 'R' || cmd == 'r') {
            current_mode = MODE_RUN;
            Serial.println("RUN_START");
        } else if ((cmd == 'S' || cmd == 's') && current_mode == MODE_CALIBRATE) {
            save_calibration();
            Serial.println("CAL_SAVED");
        }
    }

    uint32_t now_us = micros();
    uint32_t interval = (current_mode == MODE_RUN) ? RUN_INTERVAL_US : CAL_INTERVAL_US;

    if ((now_us - last_tick_us) >= interval) {
        last_tick_us = now_us;

        uint16_t raw[NUM_SENSORS];
        read_sensors(raw);

        if (current_mode == MODE_RUN) {
            send_packet(raw);
        } else {
            update_cal_minmax(raw);
            send_cal_packet(raw);
        }
    }
}
