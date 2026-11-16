#include <string.h>
#include "pico/unique_id.h"
#include "tusb.h"

// ---- Device descriptor -------------------------------------------------- //

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    // IAD required for CDC composite device
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x2E8A,   // Raspberry Pi
    .idProduct          = 0x000A,   // CDC UART
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

// ---- Configuration descriptor ------------------------------------------- //

enum {
    ITF_NUM_CDC0_COMM,   // UART bridge (stdio, getchar/putchar)
    ITF_NUM_CDC0_DATA,
    ITF_NUM_CDC1_COMM,   // control channel (reset/bootloader/3v3/CAN-monitor)
    ITF_NUM_CDC1_DATA,
    ITF_NUM_TOTAL,
};

#define EPNUM_CDC0_NOTIF  0x81
#define EPNUM_CDC0_OUT    0x02
#define EPNUM_CDC0_IN     0x82

#define EPNUM_CDC1_NOTIF  0x83
#define EPNUM_CDC1_OUT    0x04
#define EPNUM_CDC1_IN     0x84

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 2 * TUD_CDC_DESC_LEN)

static uint8_t const desc_configuration[] = {
    // Config: bus-powered, 100 mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),
    // CDC0: UART bridge — comm iface, data iface, notif EP (8B), bulk (64B)
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC0_COMM, 4, EPNUM_CDC0_NOTIF, 8,
                       EPNUM_CDC0_OUT, EPNUM_CDC0_IN, 64),
    // CDC1: control channel — same shape, separate endpoints
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC1_COMM, 5, EPNUM_CDC1_NOTIF, 8,
                       EPNUM_CDC1_OUT, EPNUM_CDC1_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

// ---- String descriptors ------------------------------------------------- //

static char const *const string_desc_arr[] = {
    (const char[]){0x09, 0x04},  // 0: language (English)
    "Raspberry Pi",               // 1: manufacturer
    "Pico UART Bridge",           // 2: product
    NULL,                         // 3: serial (built from flash UID below)
    "CDC UART",                   // 4: CDC0 interface name (UART bridge)
    "Control",                    // 5: CDC1 interface name (reset/CAN/3v3)
};

static uint16_t desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else if (index == 3) {
        // Serial number: hex-encode the 8-byte flash unique ID
        pico_unique_board_id_t id;
        pico_get_unique_board_id(&id);
        chr_count = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES;
        for (uint8_t i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
            desc_str[1 + i * 2]     = "0123456789ABCDEF"[(id.id[i] >> 4) & 0xF];
            desc_str[1 + i * 2 + 1] = "0123456789ABCDEF"[(id.id[i])      & 0xF];
        }
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
            return NULL;
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++)
            desc_str[1 + i] = str[i];
    }

    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc_str;
}
