"""VESC interface thread — bidirectional USB serial connection.

Sends throttle commands (duty cycle) to the VESC and polls GetValues
for motor RPM and input voltage. Uses the pyvesc library for packet
framing and CRC handling.

Runs as a daemon thread; main loop calls set_throttle() to update the
commanded duty cycle.
"""

import threading
import time
import logging
import serial

log = logging.getLogger(__name__)

try:
    import pyvesc
    from pyvesc import GetValues, SetDutyCycle
    _PYVESC_AVAILABLE = True
except ImportError:
    log.warning("pyvesc not installed — VESC interface will run in dummy mode")
    _PYVESC_AVAILABLE = False


class VESCInterface:
    """Background thread that manages the VESC UART connection.

    Thread safety: set_throttle() is called from the main loop;
    the thread reads throttle and writes velocity/voltage into shared state.
    """

    def __init__(self, shared_state, config: dict):
        self._state = shared_state
        cfg_vesc = config.get("vesc", {})
        self._port = cfg_vesc.get("port", "/dev/ttyACM1")
        self._baud = int(cfg_vesc.get("baud", 115200))
        self._wheel_circ = float(cfg_vesc.get("wheel_circumference_m", 0.204))
        self._gear_ratio = float(cfg_vesc.get("gear_ratio", 8.0))

        self._throttle = 0.0          # commanded duty cycle [-1.0, +1.0]
        self._throttle_lock = threading.Lock()

        self._thread = threading.Thread(target=self._run, name="vesc_interface",
                                        daemon=True)

    def start(self):
        self._thread.start()

    def set_throttle(self, duty: float):
        """Set commanded duty cycle. Thread-safe."""
        duty = max(-1.0, min(1.0, duty))
        with self._throttle_lock:
            self._throttle = duty

    def _rpm_to_ms(self, rpm: float) -> float:
        """Convert motor RPM to wheel linear velocity in m/s."""
        # wheel_rpm = motor_rpm / gear_ratio
        # velocity  = wheel_rpm / 60 * circumference
        return (rpm / self._gear_ratio / 60.0) * self._wheel_circ

    def _open_port(self) -> serial.Serial:
        while True:
            try:
                ser = serial.Serial(self._port, self._baud, timeout=0.05)
                log.info("VESC USB opened on %s", self._port)
                return ser
            except serial.SerialException as e:
                log.warning("Cannot open VESC port %s: %s — retrying in 2s",
                            self._port, e)
                time.sleep(2.0)

    def _run(self):
        if not _PYVESC_AVAILABLE:
            log.warning("pyvesc unavailable — VESC thread running as dummy")
            while self._state.running:
                time.sleep(0.1)
            return

        ser = self._open_port()

        SEND_INTERVAL  = 0.02   # 50 Hz throttle send
        POLL_INTERVAL  = 0.02   # 50 Hz GetValues poll
        last_send = 0.0
        last_poll = 0.0
        read_buf  = bytearray()

        while self._state.running:
            now = time.monotonic()

            # ── Send throttle command ─────────────────────────────────────────
            if now - last_send >= SEND_INTERVAL:
                with self._throttle_lock:
                    duty = self._throttle
                try:
                    msg = pyvesc.encode(SetDutyCycle(duty))
                    ser.write(msg)
                    last_send = now
                except Exception as e:
                    log.debug("VESC write error: %s", e)

            # ── Request telemetry ─────────────────────────────────────────────
            if now - last_poll >= POLL_INTERVAL:
                try:
                    req = pyvesc.encode_request(GetValues)
                    ser.write(req)
                    last_poll = now
                except Exception as e:
                    log.debug("VESC request error: %s", e)

            # ── Read and parse response ───────────────────────────────────────
            try:
                incoming = ser.read(ser.in_waiting or 0)
            except serial.SerialException as e:
                log.warning("VESC disconnect: %s — reconnecting", e)
                ser.close()
                ser = self._open_port()
                read_buf.clear()
                continue

            if incoming:
                read_buf.extend(incoming)
                while len(read_buf) > 0:
                    try:
                        msg, consumed = pyvesc.decode(bytes(read_buf))
                        del read_buf[:consumed]
                        if isinstance(msg, GetValues):
                            rpm = float(msg.rpm)
                            voltage = float(msg.input_voltage)
                            velocity = self._rpm_to_ms(rpm)
                            self._state.update_vesc(velocity, rpm, voltage)
                    except Exception:
                        # Discard one byte and retry on framing errors
                        if len(read_buf) > 0:
                            del read_buf[:1]
                        break

            time.sleep(0.001)  # 1ms yield to avoid busy-spinning

        # Ensure motor is stopped on exit
        try:
            ser.write(pyvesc.encode(SetDutyCycle(0.0)))
        except Exception:
            pass
        ser.close()
        log.info("vesc_interface stopped")
