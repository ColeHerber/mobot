# VESC Fix Verification (run on Pi)

## 1. Check USB device is present
```bash
ls /dev/serial/by-id/ | grep -i stm
# Should show: usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00
```

## 2. Check pyvesc is installed
```bash
python3 -c "import pyvesc; print('pyvesc OK')"
```

## 3. Run robot and watch VESC logs
```bash
bash scripts/run.sh 2>&1 | grep -E "VESC|vesc"
# Expect:
#   vesc_interface - INFO - VESC USB opened on /dev/serial/by-id/...
#   vesc_interface - INFO - VESC duty=0.XXXX raw=XXXXX   (when enabled)
```

## 4. Check web UI diagnostics
Visit `http://<pi-ip>:5000/diagnostics` — should show `vesc_connected: true`

## 5. If motor still doesn't spin after all above is OK
- Open VESC Tool on laptop, connect to VESC via USB
- Check: Motor Configuration → App Settings → App to use = **UART** (or PPM+UART)
- Verify no fault codes in VESC Tool's terminal
