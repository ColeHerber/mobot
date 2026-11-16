#include "can_bus.h"
#include "pins.h"
#include <Arduino.h>
#include "driver/twai.h"

static bool     running = false;
static uint32_t txOkCount  = 0;
static uint32_t txErrCount = 0;
static uint32_t rxOkCount  = 0;
static uint32_t rxErrCount = 0;

bool canInit(uint32_t baudRate) {
    twai_timing_config_t t_config;
    switch (baudRate) {
        case 125000:  t_config = TWAI_TIMING_CONFIG_125KBITS(); break;
        case 250000:  t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
        case 500000:  t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
        case 1000000: t_config = TWAI_TIMING_CONFIG_1MBITS();   break;
        default:
            Serial.printf("[CAN] Unsupported baud %lu, defaulting to 1Mbps\n", baudRate);
            Serial0.printf("[CAN] Unsupported baud %lu, defaulting to 1Mbps\n", baudRate);
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
    }

#if CAN_REQUIRE_ACK
    constexpr twai_mode_t mode = TWAI_MODE_NORMAL;
#else
    constexpr twai_mode_t mode = TWAI_MODE_NO_ACK;
#endif

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, mode);
    g_config.tx_queue_len = 8;
    g_config.rx_queue_len = 16;
    g_config.alerts_enabled = TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED;

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("[CAN] Install failed: %s\n", esp_err_to_name(err));
        Serial0.printf("[CAN] Install failed: %s\n", esp_err_to_name(err));
        return false;
    }

    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("[CAN] Start failed: %s\n", esp_err_to_name(err));
        Serial0.printf("[CAN] Start failed: %s\n", esp_err_to_name(err));
        twai_driver_uninstall();
        return false;
    }

    running = true;
    Serial.printf("[CAN] Running at %lu bps, mode=%s, TX=%d, RX=%d\n",
        baudRate,
        (mode == TWAI_MODE_NO_ACK) ? "NO_ACK" : "NORMAL",
        CAN_TX, CAN_RX);
    Serial0.printf("[CAN] Running at %lu bps\n", baudRate);
    return true;
}

bool canSend(uint32_t id, const uint8_t* data, uint8_t len, bool extended) {
    if (!running) return false;

    twai_message_t msg = {};
    msg.identifier   = id;
    msg.extd         = extended ? 1 : 0;
    msg.data_length_code = len > 8 ? 8 : len;
    memcpy(msg.data, data, msg.data_length_code);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (err == ESP_OK) {
        txOkCount++;
        return true;
    }
    txErrCount++;
    return false;
}

bool canReceive(CanFrame& frame) {
    if (!running) return false;

    twai_message_t msg;
    if (twai_receive(&msg, 0) != ESP_OK) return false;

    frame.id       = msg.identifier;
    frame.len      = msg.data_length_code;
    frame.extended = msg.extd;
    memcpy(frame.data, msg.data, frame.len);
    rxOkCount++;
    return true;
}

void canGetStats(uint32_t& txOk, uint32_t& txErr, uint32_t& rxOk, uint32_t& rxErr) {
    // Also pull hardware error counts from the driver
    twai_status_info_t info;
    if (running && twai_get_status_info(&info) == ESP_OK) {
        txErr = info.tx_error_counter + txErrCount;
        rxErr = info.rx_error_counter + rxErrCount;
    } else {
        txErr = txErrCount;
        rxErr = rxErrCount;
    }
    txOk = txOkCount;
    rxOk = rxOkCount;
}

bool canIsRunning() {
    return running;
}
