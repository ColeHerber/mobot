#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <math.h>

#include "secrets.h"
#include "pins.h"
#include "imu.h"
#include "can_bus.h"
#include "can_protocol.h"
#include "pwm_out.h"
#include "analog_in.h"
#include "web_ui.h"
#include "mqtt_logger.h"
#include "qwiic_i2c.h"
#include "uart1.h"

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

constexpr uint32_t SSE_INTERVAL_MS       = 50;    // 20 Hz SSE push
constexpr uint32_t SERIAL_DBG_INTERVAL_MS = 500;  // 2 Hz serial JSON dump
constexpr uint32_t HEAP_WARN_INTERVAL_MS  = 10000;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
constexpr uint32_t HEAP_WARN_THRESHOLD    = 50000; // bytes
constexpr uint8_t  RGB_GRAVITY_BRIGHTNESS = 48;
constexpr uint8_t  LEGACY_STATUS_LED      = 15;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static ImuData  imuData       = {};
static bool     imuOk         = false;
static bool     wifiDisabled  = false;
static bool     otaReady      = false;
static uint32_t lastWifiMs    = 0;
static uint32_t lastSseMs     = 0;
static uint32_t lastSerialMs  = 0;
static uint32_t lastHeapWarn  = 0;

// Pending CAN frames for the next SSE push (up to 8 buffered between pushes)
static CanFrame pendingCan[8];
static uint8_t  pendingCanCount = 0;

// Remote board IMU data received via CAN
static ImuData  remoteBoardImuData = {};
static bool     remoteBoardImuValid = false;
static uint32_t remoteBoardLastSeen = 0;

// CAN transmission state
static uint8_t  canTxSeq = 0;
static uint32_t lastCanTxMs = 0;
constexpr uint32_t CAN_TX_INTERVAL_MS = 100;  // 10 Hz IMU broadcast

// ---------------------------------------------------------------------------
// LED heartbeat
// ---------------------------------------------------------------------------

enum class LedMode { BLINK_FAST, BLINK_SLOW };
static LedMode  ledMode      = LedMode::BLINK_FAST;
static uint32_t lastLedMs    = 0;
static bool     ledState     = false;
static bool     faultLatched = false;  // set once, e.g. on IMU init failure

static bool hasFault() {
    return faultLatched;
}

static bool hasWifiFault() {
    return wifiDisabled || WiFi.status() != WL_CONNECTED;
}

static bool shouldLightFaultLed() {
    return hasFault() || hasWifiFault();
}

static void writeLifeLed(bool on) {
    digitalWrite(LED_LIFE, on ? HIGH : LOW);
    // Bring-up fallback: earlier firmware used GPIO15 as the only visible
    // status LED. Mirror LIFE there so old/reworked boards still show boot.
    digitalWrite(LEGACY_STATUS_LED, on ? HIGH : LOW);
}

static void updateLed() {
    // D6 FAULT is driven by the FAULT net on GPIO1 through its series resistor.
    digitalWrite(LED_FAULT, shouldLightFaultLed() ? HIGH : LOW);

    const uint32_t interval = (ledMode == LedMode::BLINK_FAST) ? 100 : 1000;
    const uint32_t now = millis();
    if (now - lastLedMs >= interval) {
        ledState = !ledState;
        writeLifeLed(ledState);
        lastLedMs = now;
    }
}

// ===========================================================================
// BRING-UP PATCH NOTE - STATUS LED BEHAVIOR
// ===========================================================================
// D6/FAULT is GPIO1. A failed IMU init latches FAULT on solid. WiFi not being
// connected also lights FAULT solid, but clears automatically once connected.
//
// D2/RGB is the WS2812B-2020 on GPIO15. When the IMU is healthy, RGB shows the
// strongest absolute low-g axis: |X|=red, |Y|=green, |Z|=blue. Absolute values
// are deliberate so flipping the board over does not make the active axis go
// dark. Faults are shown on D6/FAULT instead.
// ===========================================================================
static void updateRgb() {
    static uint8_t lastR = 255, lastG = 255, lastB = 255;  // force first write
    uint8_t r = 0, g = 0, b = 0;

    if (imuOk) {
        const float ax = fabsf(imuData.ax);
        const float ay = fabsf(imuData.ay);
        const float az = fabsf(imuData.az);

        if (ax > 0.05f || ay > 0.05f || az > 0.05f) {
            if (ax >= ay && ax >= az) {
                r = RGB_GRAVITY_BRIGHTNESS;
            } else if (ay >= az) {
                g = RGB_GRAVITY_BRIGHTNESS;
            } else {
                b = RGB_GRAVITY_BRIGHTNESS;
            }
        }
    }

    if (r != lastR || g != lastG || b != lastB) {
        neopixelWrite(LED_RGB, r, g, b);
        lastR = r; lastG = g; lastB = b;
    }
}

static void printSerialDebug(uint32_t now) {
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"t\":%lu,\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
        "\"rawAx\":%d,\"rawAy\":%d,\"rawAz\":%d,"
        "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
        "\"adc0\":%.3f,\"adc1\":%.3f,\"adc2\":%.3f,\"adc3\":%.3f,"
        "\"pwm0\":%u,\"pwm1\":%u,\"pwm2\":%u,\"pwm3\":%u,"
        "\"heap\":%lu,\"rssi\":%d}\n",
        now,
        imuData.ax, imuData.ay, imuData.az,
        imuData.rawAx, imuData.rawAy, imuData.rawAz,
        imuData.gx, imuData.gy, imuData.gz,
        analogInReadV(0), analogInReadV(1), analogInReadV(2), analogInReadV(3),
        pwmGetUs(0), pwmGetUs(1), pwmGetUs(2), pwmGetUs(3),
        ESP.getFreeHeap(),
        (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0);
    Serial.print(buf);
    Serial0.print(buf);
}

static const char* wifiStatusName(wl_status_t status) {
    switch (status) {
        case WL_IDLE_STATUS:     return "IDLE";
        case WL_NO_SSID_AVAIL:   return "NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED:  return "SCAN_COMPLETED";
        case WL_CONNECTED:       return "CONNECTED";
        case WL_CONNECT_FAILED:  return "CONNECT_FAILED";
        case WL_CONNECTION_LOST: return "CONNECTION_LOST";
        case WL_DISCONNECTED:    return "DISCONNECTED";
        default:                 return "UNKNOWN";
    }
}

static const char* wifiDisconnectReasonName(uint8_t reason) {
    switch (reason) {
        case WIFI_REASON_AUTH_EXPIRE:              return "AUTH_EXPIRE";
        case WIFI_REASON_AUTH_LEAVE:               return "AUTH_LEAVE";
        case WIFI_REASON_ASSOC_EXPIRE:             return "ASSOC_EXPIRE";
        case WIFI_REASON_ASSOC_TOOMANY:            return "ASSOC_TOOMANY";
        case WIFI_REASON_NOT_AUTHED:               return "NOT_AUTHED";
        case WIFI_REASON_NOT_ASSOCED:              return "NOT_ASSOCED";
        case WIFI_REASON_ASSOC_LEAVE:              return "ASSOC_LEAVE";
        case WIFI_REASON_ASSOC_NOT_AUTHED:         return "ASSOC_NOT_AUTHED";
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:   return "4WAY_HANDSHAKE_TIMEOUT";
        case WIFI_REASON_BEACON_TIMEOUT:           return "BEACON_TIMEOUT";
        case WIFI_REASON_NO_AP_FOUND:              return "NO_AP_FOUND";
        case WIFI_REASON_AUTH_FAIL:                return "AUTH_FAIL";
        case WIFI_REASON_ASSOC_FAIL:               return "ASSOC_FAIL";
        case WIFI_REASON_HANDSHAKE_TIMEOUT:        return "HANDSHAKE_TIMEOUT";
        case WIFI_REASON_CONNECTION_FAIL:          return "CONNECTION_FAIL";
        default:                                   return "UNMAPPED";
    }
}

// ---------------------------------------------------------------------------
// CAN Communication
// ---------------------------------------------------------------------------

static void canTxImuData(const ImuData& data) {
    uint8_t frame[8] = {0};
    CanProtocol::encodeGyroData(data, canTxSeq, frame);

    uint32_t can_id = CanProtocol::canIdFor(1, CanProtocol::MessageType::IMU_DATA, 0);
    canSend(can_id, frame, 8);

    canTxSeq++;
}

static void canProcessReceivedFrames() {
    CanFrame frame;
    while (canReceive(frame)) {
        // Buffer for web UI
        if (pendingCanCount < 8) {
            pendingCan[pendingCanCount++] = frame;
        }

        // Decode IMU data messages
        uint8_t msg_type = frame.data[0];

        if (msg_type == (uint8_t)CanProtocol::MessageType::IMU_DATA) {
            CanProtocol::decodeGyroData(frame.data, remoteBoardImuData);
            remoteBoardImuValid = true;
            remoteBoardLastSeen = millis();

            Serial.printf("[CAN] Received gyro: gx=%.2f gy=%.2f gz=%.2f\n",
                remoteBoardImuData.gx, remoteBoardImuData.gy, remoteBoardImuData.gz);
        }
    }
}

// ---------------------------------------------------------------------------
// WiFi / OTA (kept from original)
// ---------------------------------------------------------------------------

static void beginWifi() {
    if (strlen(WIFI_SSID) == 0 || strcmp(WIFI_SSID, "your-wifi-ssid") == 0) {
        wifiDisabled = true;
        dbgLog("[WiFi] Disabled — set WIFI_SSID in src/secrets.h");
        return;
    }
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
            dbgLog("[WiFi] Connected, IP=%s RSSI=%d dBm",
                   WiFi.localIP().toString().c_str(), WiFi.RSSI());
        } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
            const uint8_t reason = info.wifi_sta_disconnected.reason;
            dbgLog("[WiFi] Disconnected, reason=%u (%s), status=%d (%s)",
                   reason, wifiDisconnectReasonName(reason),
                   static_cast<int>(WiFi.status()), wifiStatusName(WiFi.status()));
        }
    });
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiMs = millis();
    // Do not emit credentials or network identifiers into serial/web logs.
    dbgLog("[WiFi] Connecting with configured credentials...");
}

static void beginOta() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        dbgLog("[OTA] Update started");
        writeLifeLed(true);
    });
    ArduinoOTA.onEnd([]()  { dbgLog("[OTA] Update finished"); });
    ArduinoOTA.onError([](ota_error_t err) { dbgLog("[OTA] Error %u", err); });
    ArduinoOTA.begin();
    otaReady = true;
    dbgLog("[OTA] Ready at %s (%s)", WiFi.localIP().toString().c_str(), OTA_HOSTNAME);
}

static void handleWifiAndOta() {
    if (wifiDisabled) return;

    if (WiFi.status() == WL_CONNECTED) {
        if (!otaReady) beginOta();
        ArduinoOTA.handle();
        return;
    }

    otaReady = false;
    const uint32_t now = millis();
    if (now - lastWifiMs >= WIFI_RETRY_INTERVAL_MS) {
        const wl_status_t status = WiFi.status();
        dbgLog("[WiFi] Retrying... status=%d (%s)",
               static_cast<int>(status), wifiStatusName(status));
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        lastWifiMs = now;
    }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup() {
    pinMode(LED_LIFE, OUTPUT);
    pinMode(LED_FAULT, OUTPUT);
    pinMode(LEGACY_STATUS_LED, OUTPUT);
    writeLifeLed(true);             // on during init
    digitalWrite(LED_FAULT, LOW);

    Serial.begin(115200);
    Serial0.begin(115200);
    delay(400);

    dbgLog("=== ESP32-S3 CAN+IMU board boot ===");
    dbgLog("Chip: %s rev%d cores=%d %dMHz",
        ESP.getChipModel(), ESP.getChipRevision(),
        ESP.getChipCores(), ESP.getCpuFreqMHz());

    // IMU
    imuOk = imuInit();
    if (!imuOk) {
        faultLatched = true;  // FAULT LED latches solid on; LIFE keeps blinking normally
        dbgLog("[IMU] INIT FAILED — check SPI wiring and WHO_AM_I above");
    }

    // CAN
    canInit(1000000);

    // PWM
    pwmInit();

    // ADC
    analogInInit();

    // Qwiic I2C / UART1 — stub drivers, bring the bus up, nothing attached yet
    qwiicI2cInit();
    uart1Init();

    // WiFi
    beginWifi();

    // MQTT raw IMU logger (no-ops if MQTT_BROKER_HOST is unconfigured)
    mqttLoggerInit();

    // Web server (starts even without WiFi — browser can't reach it yet)
    webUiInit();

    // Pass WHO_AM_I to web UI for display
    extern void webUiSetImuWhoAmI(uint8_t);
    webUiSetImuWhoAmI(imuWhoAmI());

    writeLifeLed(false);
    dbgLog("=== Setup complete. Free heap: %lu bytes ===", ESP.getFreeHeap());
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------

void loop() {
    handleWifiAndOta();

    // LIFE blink rate reflects WiFi connection state; FAULT is latched separately.
    ledMode = (WiFi.status() == WL_CONNECTED)
                ? LedMode::BLINK_SLOW   // nominal — slow blink
                : LedMode::BLINK_FAST;  // no wifi — fast blink
    updateLed();

    // Read low-g accel + gyro when data-ready (INT1 high)
    if (imuOk && imuAvailable()) {
        imuRead(imuData);
        mqttLoggerPushLowG(imuData);
    }

    // Read high-g accel when data-ready (INT2 high) — independent ODR/timing
    // from the low-g+gyro path above, so it's checked separately.
    if (imuOk && imuAvailableHg()) {
        imuReadHg(imuData);
        mqttLoggerPushHighG(imuData);
    }

    updateRgb();

    // Service the MQTT connection / drain buffered samples — non-blocking.
    mqttLoggerService();

    // Process incoming CAN frames: decode IMU data and buffer for web UI display
    canProcessReceivedFrames();

    const uint32_t now = millis();

    // Broadcast our IMU data via CAN at 10 Hz
    if (imuOk && now - lastCanTxMs >= CAN_TX_INTERVAL_MS) {
        lastCanTxMs = now;
        canTxImuData(imuData);
    }

    // Push SSE at 20 Hz
    if (now - lastSseMs >= SSE_INTERVAL_MS) {
        lastSseMs = now;

        WebUiState state;
        state.imu     = imuData;
        state.imuOk   = imuOk;
        state.adc[0]  = analogInReadV(0);
        state.adc[1]  = analogInReadV(1);
        state.adc[2]  = analogInReadV(2);
        state.adc[3]  = analogInReadV(3);
        state.pwm[0]  = pwmGetUs(0);
        state.pwm[1]  = pwmGetUs(1);
        state.pwm[2]  = pwmGetUs(2);
        state.pwm[3]  = pwmGetUs(3);
        state.rssi    = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
        state.freeHeap = ESP.getFreeHeap();
        canGetStats(state.canTxOk, state.canTxErr, state.canRxOk, state.canRxErr);

        webUiPushSse(state, pendingCan, pendingCanCount);
        pendingCanCount = 0;
    }

    // Serial JSON debug dump at 2 Hz
    if (now - lastSerialMs >= SERIAL_DBG_INTERVAL_MS) {
        lastSerialMs = now;
        printSerialDebug(now);
    }

    // Heap warning
    if (now - lastHeapWarn >= HEAP_WARN_INTERVAL_MS) {
        lastHeapWarn = now;
        const uint32_t heap = ESP.getFreeHeap();
        if (heap < HEAP_WARN_THRESHOLD) {
            dbgLog("[WARN] Low heap: %lu bytes", heap);
        }
    }
}
