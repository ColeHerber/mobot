#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "tusb.h"

#define UART_ID      uart1
#define BAUD_RATE    115200
#define UART_TX_PIN  4u  // -> J3 pin 8 (UART0RX, ESP32 RX)
#define UART_RX_PIN  5u  // <- J3 pin 9 (UART0TX, ESP32 TX)

#define PIN_3V3_EN   0u  // -> J3 pin 3; low disables 3.3V rail
#define PIN_RESET    2u  // -> J3 pin 6; low resets ESP32
#define PIN_BOOT     3u  // -> J3 pin 7; low selects ESP32 download mode
#define PIN_CAN_R    6u  // <- J3 pin 4; input-only debug tap
#define PIN_CAN_D    7u  // <- J3 pin 5; input-only debug tap

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25u
#endif
#define LED_PIN PICO_DEFAULT_LED_PIN

#define CDC_UART       0u
#define CDC_CONTROL    1u
#define USB_BUF_SIZE   64u
#define UART_BUF_SIZE  64u
#define UART_RX_RING_SIZE 1024u

#define HEARTBEAT_US        1000000
#define ACTIVITY_PULSE_US     30000

typedef struct {
    uint pin;
    const char *name;
} named_pin_t;

static absolute_time_t activity_until;
static absolute_time_t last_heartbeat;
static bool heartbeat_state = false;
static bool can_monitor_enabled = false;
static bool uart_enabled = false;
static uint8_t uart_rx_ring[UART_RX_RING_SIZE];
static uint16_t uart_rx_head = 0;
static uint16_t uart_rx_tail = 0;

static const named_pin_t J3_PINS[] = {
    {PIN_3V3_EN, "3v3_EN"},
    {PIN_RESET, "RESET"},
    {PIN_BOOT, "BOOT"},
    {UART_TX_PIN, "UART0RX"},
    {UART_RX_PIN, "UART0TX"},
    {PIN_CAN_R, "CAN_R"},
    {PIN_CAN_D, "CAN_D"},
};

static void pulse_activity(void) {
    activity_until = make_timeout_time_us(ACTIVITY_PULSE_US);
    gpio_put(LED_PIN, 1);
}

static void update_led(void) {
    if (!time_reached(activity_until)) {
        gpio_put(LED_PIN, 1);
        return;
    }

    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(last_heartbeat, now) >= HEARTBEAT_US) {
        heartbeat_state = !heartbeat_state;
        gpio_put(LED_PIN, heartbeat_state);
        last_heartbeat = now;
    }
}

static void od_init(uint pin) {
    gpio_init(pin);
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

static void od_assert(uint pin) {
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
}

static void od_release(uint pin) {
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

static void uart_bridge_enable(void) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_enabled = true;
}

static void uart_bridge_disable(void) {
    uart_deinit(UART_ID);
    gpio_init(UART_TX_PIN);
    gpio_set_dir(UART_TX_PIN, GPIO_IN);
    gpio_disable_pulls(UART_TX_PIN);
    gpio_init(UART_RX_PIN);
    gpio_set_dir(UART_RX_PIN, GPIO_IN);
    gpio_disable_pulls(UART_RX_PIN);
    uart_enabled = false;
    uart_rx_head = 0;
    uart_rx_tail = 0;
}

static uint16_t ring_used(void) {
    return (uint16_t)((uart_rx_head - uart_rx_tail) & (UART_RX_RING_SIZE - 1u));
}

static uint16_t ring_free(void) {
    return (uint16_t)(UART_RX_RING_SIZE - 1u - ring_used());
}

static void ring_push(uint8_t byte) {
    uint16_t next = (uint16_t)((uart_rx_head + 1u) & (UART_RX_RING_SIZE - 1u));
    if (next == uart_rx_tail) return;
    uart_rx_ring[uart_rx_head] = byte;
    uart_rx_head = next;
}

static uint16_t ring_peek_chunk(uint8_t **ptr) {
    if (uart_rx_head == uart_rx_tail) {
        *ptr = NULL;
        return 0;
    }
    *ptr = &uart_rx_ring[uart_rx_tail];
    if (uart_rx_head > uart_rx_tail) {
        return (uint16_t)(uart_rx_head - uart_rx_tail);
    }
    return (uint16_t)(UART_RX_RING_SIZE - uart_rx_tail);
}

static void ring_drop(uint16_t n) {
    uart_rx_tail = (uint16_t)((uart_rx_tail + n) & (UART_RX_RING_SIZE - 1u));
}

static void pulse_reset_only(void) {
    od_assert(PIN_RESET);
    sleep_ms(200);
    od_release(PIN_RESET);
}

static void pulse_reset(void) {
    od_release(PIN_BOOT);
    sleep_ms(10);
    pulse_reset_only();
}

static void enter_bootloader(void) {
    od_assert(PIN_BOOT);
    sleep_ms(50);
    pulse_reset_only();
}

static void set_3v3_rail(bool on) {
    if (on) {
        od_release(PIN_3V3_EN);
        sleep_ms(50);
        if (!uart_enabled) uart_bridge_enable();
    } else {
        if (uart_enabled) uart_bridge_disable();
        od_assert(PIN_3V3_EN);
    }
}

static void cdc_write_str(uint8_t itf, const char *s) {
    if (!tud_cdc_n_connected(itf)) return;

    size_t len = strlen(s);
    size_t off = 0;
    absolute_time_t deadline = make_timeout_time_ms(250);

    while (off < len && !time_reached(deadline)) {
        tud_task();
        uint32_t room = tud_cdc_n_write_available(itf);
        if (!room) {
            sleep_ms(1);
            continue;
        }

        uint32_t n = (uint32_t)(len - off);
        if (n > room) n = room;
        tud_cdc_n_write(itf, s + off, n);
        tud_cdc_n_write_flush(itf);
        off += n;
    }
}

static void cdc_write_pin_sample(uint8_t itf, const char *tag) {
    char line[256];
    int used = snprintf(line, sizeof(line), "%s", tag);
    for (uint i = 0; i < sizeof(J3_PINS) / sizeof(J3_PINS[0]); i++) {
        if (used < 0 || used >= (int)sizeof(line)) break;
        used += snprintf(line + used, sizeof(line) - (size_t)used,
                         " GP%u/%s=%d",
                         J3_PINS[i].pin, J3_PINS[i].name,
                         gpio_get(J3_PINS[i].pin) ? 1 : 0);
    }
    if (used >= 0 && used < (int)sizeof(line)) {
        snprintf(line + used, sizeof(line) - (size_t)used, "\r\n");
    } else {
        line[sizeof(line) - 3] = '\r';
        line[sizeof(line) - 2] = '\n';
        line[sizeof(line) - 1] = '\0';
    }
    cdc_write_str(itf, line);
}

static void release_control_pins(void) {
    od_release(PIN_3V3_EN);
    od_release(PIN_RESET);
    od_release(PIN_BOOT);
}

static void prepare_j3_inputs(void) {
    if (uart_enabled) uart_bridge_disable();
    release_control_pins();

    for (uint i = 0; i < sizeof(J3_PINS) / sizeof(J3_PINS[0]); i++) {
        gpio_init(J3_PINS[i].pin);
        gpio_set_dir(J3_PINS[i].pin, GPIO_IN);
        gpio_disable_pulls(J3_PINS[i].pin);
    }
}

static void run_safe_short_test(void) {
    bool restore_uart = uart_enabled;

    cdc_write_str(CDC_CONTROL, "TEST safe-short begin\r\n");
    prepare_j3_inputs();
    sleep_ms(10);
    cdc_write_pin_sample(CDC_CONTROL, "BASE");

    od_assert(PIN_3V3_EN);
    sleep_ms(10);
    cdc_write_pin_sample(CDC_CONTROL, "DRIVE GP0/3v3_EN LOW");
    od_release(PIN_3V3_EN);
    sleep_ms(10);

    od_assert(PIN_RESET);
    sleep_ms(10);
    cdc_write_pin_sample(CDC_CONTROL, "DRIVE GP2/RESET LOW");
    od_release(PIN_RESET);
    sleep_ms(10);

    od_assert(PIN_BOOT);
    sleep_ms(10);
    cdc_write_pin_sample(CDC_CONTROL, "DRIVE GP3/BOOT LOW");
    od_release(PIN_BOOT);
    sleep_ms(10);

    cdc_write_pin_sample(CDC_CONTROL, "FINAL");
    cdc_write_str(CDC_CONTROL, "TEST safe-short end\r\n");

    if (restore_uart) uart_bridge_enable();
}

static const char HELP_TEXT[] =
    "commands: r=reset  e=enter-bootloader  o=3v3-off  n=3v3-on  "
    "b=boot-hold  u=boot-release  c=toggle-can-monitor  l=levels  "
    "t=safe-short-test  h=help\r\n";

static void poll_control_channel(void) {
    if (!tud_cdc_n_available(CDC_CONTROL)) return;

    uint8_t buf[USB_BUF_SIZE];
    uint32_t n = tud_cdc_n_read(CDC_CONTROL, buf, sizeof(buf));

    for (uint32_t i = 0; i < n; i++) {
        switch (buf[i]) {
            case 'r':
                pulse_reset();
                cdc_write_str(CDC_CONTROL, "OK reset\r\n");
                break;
            case 'e':
                enter_bootloader();
                cdc_write_str(CDC_CONTROL, "OK bootloader boot-held\r\n");
                break;
            case 'b':
                od_assert(PIN_BOOT);
                cdc_write_str(CDC_CONTROL, "OK boot held\r\n");
                break;
            case 'u':
                od_release(PIN_BOOT);
                cdc_write_str(CDC_CONTROL, "OK boot released\r\n");
                break;
            case 'o':
                set_3v3_rail(false);
                cdc_write_str(CDC_CONTROL, "OK 3v3 off\r\n");
                break;
            case 'n':
                set_3v3_rail(true);
                cdc_write_str(CDC_CONTROL, "OK 3v3 on\r\n");
                break;
            case 'c':
                can_monitor_enabled = !can_monitor_enabled;
                cdc_write_str(CDC_CONTROL,
                              can_monitor_enabled ? "OK can-monitor on\r\n"
                                                  : "OK can-monitor off\r\n");
                break;
            case 't':
                run_safe_short_test();
                break;
            case 'l':
                cdc_write_pin_sample(CDC_CONTROL, "LEVEL");
                break;
            case 'h':
            case '?':
                cdc_write_str(CDC_CONTROL, HELP_TEXT);
                break;
            default:
                break;
        }
    }
}

static void poll_uart_bridge(void) {
    if (!uart_enabled) return;

    if (tud_cdc_n_available(CDC_UART)) {
        uint8_t buf[USB_BUF_SIZE];
        uint32_t n = tud_cdc_n_read(CDC_UART, buf, sizeof(buf));
        for (uint32_t i = 0; i < n; i++) {
            uart_putc_raw(UART_ID, (char)buf[i]);
        }
        pulse_activity();
    }

    while (ring_free() && uart_is_readable(UART_ID)) {
        ring_push((uint8_t)uart_getc(UART_ID));
    }

    uint32_t room = tud_cdc_n_write_available(CDC_UART);
    uint8_t *chunk = NULL;
    uint16_t n = ring_peek_chunk(&chunk);
    if (room && n) {
        if (n > room) n = (uint16_t)room;
        if (n > UART_BUF_SIZE) n = UART_BUF_SIZE;
        tud_cdc_n_write(CDC_UART, chunk, n);
        tud_cdc_n_write_flush(CDC_UART);
        ring_drop(n);
        pulse_activity();
    }
}

static void poll_can_monitor(void) {
    static bool last_r = false;
    static bool last_d = false;
    static bool initialized = false;

    bool r = gpio_get(PIN_CAN_R);
    bool d = gpio_get(PIN_CAN_D);

    if (!initialized) {
        last_r = r;
        last_d = d;
        initialized = true;
        return;
    }

    if (!can_monitor_enabled) {
        last_r = r;
        last_d = d;
        return;
    }

    char line[48];
    if (r != last_r) {
        snprintf(line, sizeof(line), "CAN_R %d->%d @%lluus\r\n",
                 (int)last_r, (int)r, (unsigned long long)time_us_64());
        cdc_write_str(CDC_CONTROL, line);
        last_r = r;
    }
    if (d != last_d) {
        snprintf(line, sizeof(line), "CAN_D %d->%d @%lluus\r\n",
                 (int)last_d, (int)d, (unsigned long long)time_us_64());
        cdc_write_str(CDC_CONTROL, line);
        last_d = d;
    }
}

static void init_safe_pins(void) {
    od_init(PIN_3V3_EN);
    od_init(PIN_RESET);
    od_init(PIN_BOOT);

    gpio_init(PIN_CAN_R);
    gpio_set_dir(PIN_CAN_R, GPIO_IN);
    gpio_disable_pulls(PIN_CAN_R);

    gpio_init(PIN_CAN_D);
    gpio_set_dir(PIN_CAN_D, GPIO_IN);
    gpio_disable_pulls(PIN_CAN_D);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

int main(void) {
    board_init();
    init_safe_pins();
    uart_bridge_enable();

    tusb_init();

    activity_until = get_absolute_time();
    last_heartbeat = get_absolute_time();

    while (true) {
        tud_task();
        poll_uart_bridge();
        poll_control_channel();
        poll_can_monitor();
        update_led();
    }
}
