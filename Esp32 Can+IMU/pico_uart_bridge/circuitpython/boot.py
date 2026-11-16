import usb_cdc

# Expose a second USB CDC serial port. The console stays available for normal
# CircuitPython access; the data port is used by code.py as the UART bridge.
usb_cdc.enable(console=True, data=True)

