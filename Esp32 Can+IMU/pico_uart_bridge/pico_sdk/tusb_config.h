#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

// CFG_TUSB_MCU is injected by the Pico SDK build system — do not define here.
#define CFG_TUSB_OS           OPT_OS_PICO
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE

#define CFG_TUD_ENDPOINT0_SIZE 64

// CDC0 = UART bridge (via stdio, getchar/putchar); CDC1 = raw control channel
// (reset/bootloader/3v3/CAN-monitor commands, see main.c).
#define CFG_TUD_CDC    2
#define CFG_TUD_HID    0
#define CFG_TUD_MSC    0
#define CFG_TUD_MIDI   0
#define CFG_TUD_VENDOR 0
#define CFG_TUD_ECM_RNDIS 0
#define CFG_TUD_BTH    0

#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 512

#endif
