import time

import board
import busio
import digitalio
import usb_cdc


UART_BAUD = 115200
UART_TX = board.GP4
UART_RX = board.GP5
CHUNK_SIZE = 128
HEARTBEAT_INTERVAL_S = 1.0
ACTIVITY_PULSE_S = 0.03


uart = busio.UART(
    tx=UART_TX,
    rx=UART_RX,
    baudrate=UART_BAUD,
    timeout=0,
    receiver_buffer_size=4096,
)

usb = usb_cdc.data
usb.timeout = 0

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

last_heartbeat = time.monotonic()
activity_until = 0.0
heartbeat_state = False


def pulse_activity():
    global activity_until
    activity_until = time.monotonic() + ACTIVITY_PULSE_S
    led.value = True


while True:
    usb_waiting = usb.in_waiting
    if usb_waiting:
        data = usb.read(min(usb_waiting, CHUNK_SIZE))
        if data:
            uart.write(data)
            pulse_activity()

    uart_waiting = uart.in_waiting
    if uart_waiting:
        data = uart.read(min(uart_waiting, CHUNK_SIZE))
        if data:
            usb.write(data)
            pulse_activity()

    now = time.monotonic()
    if now < activity_until:
        led.value = True
    elif now - last_heartbeat >= HEARTBEAT_INTERVAL_S:
        heartbeat_state = not heartbeat_state
        led.value = heartbeat_state
        last_heartbeat = now

