"""VESC interface thread — bidirectional USB-CDC serial connection.

Implements the VESC wire protocol directly (no pyvesc dependency).
Protocol reference: debug/vesc_debug.py and VESC firmware comm_can.c.

Packet framing (short packets, payload ≤ 255 bytes):
    [0x02] [len:1] [payload:len] [crc_hi:1] [crc_lo:1] [0x03]
CRC: CRC-CCITT XModem (poly=0x1021, init=0x0000)

Commands used:
    COMM_GET_VALUES = 4   — poll motor telemetry
    COMM_SET_DUTY   = 5   — set duty cycle (int32, scaled ×100000)

Runs as a daemon thread; main loop calls set_throttle() to update the
commanded duty cycle.
"""

import struct
import threading
import time
import logging
import serial

log = logging.getLogger(__name__)

# ── VESC protocol constants ───────────────────────────────────────────────────

COMM_GET_VALUES = 4
COMM_SET_DUTY   = 5

# GetValues response struct (big-endian).
# Matches VESC firmware 3.x–6.x; firmware 6.x may append extra bytes which
# are safely ignored because we only unpack _GV_SIZE bytes.
_GV_FMT   = ">hh iiii h i h iiii ii b"
_GV_SCALE = (10, 10, 100, 100, 100, 100, 1000, 1, 10,
             10000, 10000, 10000, 10000, 1, 1, 1)
_GV_NAMES = (
    "temp_fet_filtered", "temp_motor_filtered",
    "avg_motor_current", "avg_input_current", "avg_id", "avg_iq",
    "duty_cycle_now", "rpm", "input_voltage",
    "amp_hours", "amp_hours_charged", "watt_hours", "watt_hours_charged",
    "tachometer", "tachometer_abs", "mc_fault_code",
)
_GV_SIZE = struct.calcsize(_GV_FMT)  # 55 bytes


# ── Wire-protocol helpers (proven against hardware in debug/vesc_debug.py) ────

def _crc_ccitt(data: bytes) -> int:
    """CRC-CCITT XModem: poly=0x1021, init=0x0000."""
    crc = 0
    for b in data:
        x = (crc >> 8) ^ b
        x ^= x >> 4
        crc = ((crc << 8) ^ (x << 12) ^ (x << 5) ^ x) & 0xFFFF
    return crc


def _make_packet(payload: bytes) -> bytes:
    """Wrap payload in VESC framing with CRC."""
    n = len(payload)
    if n <= 255:
        header = bytes([0x02, n])
    else:
        header = bytes([0x03, (n >> 8) & 0xFF, n & 0xFF])
    crc = _crc_ccitt(payload)
    return header + payload + bytes([(crc >> 8) & 0xFF, crc & 0xFF, 0x03])


def _make_set_duty_packet(duty: float) -> bytes:
    """Build a COMM_SET_DUTY packet. duty is clamped to [-1.0, +1.0]."""
    duty = max(-1.0, min(1.0, duty))
    raw = int(duty * -100000)
    payload = bytes([COMM_SET_DUTY]) + struct.pack(">i", raw)
    return _make_packet(payload)


def _make_get_values_packet() -> bytes:
    """Build a COMM_GET_VALUES request packet (constant, pre-built at import)."""
    return _make_packet(bytes([COMM_GET_VALUES]))


# Pre-built packets that never change
_GET_VALUES_PKT = _make_get_values_packet()
_STOP_PKT       = _make_set_duty_packet(0.0)


def _find_packet(buf: bytearray):
    """Scan buf for the first valid framed packet.

    Returns (payload: bytes, consumed: int) or (None, 0) if incomplete/invalid.
    """
    i = 0
    while i < len(buf):
        start = buf[i]
        if start == 0x02:
            if i + 4 > len(buf):
                break
            length = buf[i + 1]
            end = i + 2 + length + 3
            if end > len(buf):
                break
            payload  = bytes(buf[i + 2: i + 2 + length])
            crc_recv = (buf[i + 2 + length] << 8) | buf[i + 2 + length + 1]
            stop     = buf[end - 1]
        elif start == 0x03:
            if i + 5 > len(buf):
                break
            length = (buf[i + 1] << 8) | buf[i + 2]
            end = i + 3 + length + 3
            if end > len(buf):
                break
            payload  = bytes(buf[i + 3: i + 3 + length])
            crc_recv = (buf[i + 3 + length] << 8) | buf[i + 3 + length + 1]
            stop     = buf[end - 1]
        else:
            i += 1
            continue

        if stop != 0x03 or _crc_ccitt(payload) != crc_recv:
            i += 1
            continue

        return payload, end

    return None, 0


def _parse_get_values(payload: bytes) -> dict | None:
    """Decode a GetValues response payload. Returns field dict or None."""
    if not payload or payload[0] != COMM_GET_VALUES:
        return None
    body = payload[1:]
    if len(body) < _GV_SIZE:
        return None
    raw = struct.unpack(_GV_FMT, body[:_GV_SIZE])
    return {name: val / scale
            for name, scale, val in zip(_GV_NAMES, _GV_SCALE, raw)}


# ── VESCInterface class ───────────────────────────────────────────────────────

class VESCInterface:
    """Background thread that manages the VESC USB-CDC serial connection.

    Physical interface: USB (appears as ttyACM*). No pyvesc dependency.
    Thread safety: set_throttle() is called from the main loop;
    the thread reads throttle and writes velocity/voltage into shared state.
    """

    def __init__(self, shared_state, config: dict):
        self._state = shared_state
        cfg_vesc = config.get("vesc", {})
        self._port       = cfg_vesc.get("port", "/dev/ttyACM1")
        self._baud       = int(cfg_vesc.get("baud", 115200))
        self._wheel_circ = float(cfg_vesc.get("wheel_circumference_m", 0.204))
        self._gear_ratio = float(cfg_vesc.get("gear_ratio", 8.0))
        self._max_duty   = float(cfg_vesc.get("max_duty", 1.0))

        self._throttle      = 0.0   # commanded duty cycle [-1.0, +1.0]
        self._throttle_lock = threading.Lock()
        self._connected     = False

        self._thread = threading.Thread(target=self._run, name="vesc_interface",
                                        daemon=True)

    def start(self):
        self._thread.start()

    def set_throttle(self, duty: float):
        """Set commanded duty cycle. Thread-safe. duty in [-max_duty, +max_duty]."""
        duty = max(-self._max_duty, min(self._max_duty, duty))
        with self._throttle_lock:
            self._throttle = duty

    @property
    def connected(self) -> bool:
        return self._connected

    def _rpm_to_ms(self, rpm: float) -> float:
        return (rpm / self._gear_ratio / 60.0) * self._wheel_circ

    def _open_port(self) -> serial.Serial:
        self._connected = False
        self._state.set(vesc_connected=False)
        while True:
            try:
                ser = serial.Serial(self._port, self._baud, timeout=0.05)
                log.info("VESC USB opened on %s", self._port)
                self._connected = True
                self._state.set(vesc_connected=True)
                return ser
            except serial.SerialException as e:
                log.warning("Cannot open VESC port %s: %s — retrying in 2s",
                            self._port, e)
                time.sleep(2.0)

    def _run(self):
        ser = self._open_port()

        SEND_INTERVAL   = 0.02   # 50 Hz duty cycle commands
        POLL_INTERVAL   = 0.05   # 20 Hz GetValues polling
        MAX_WRITE_FAILS = 3

        last_send        = 0.0
        last_poll        = 0.0
        read_buf         = bytearray()
        write_fail_count = 0

        while self._state.running:
            now = time.monotonic()

            # ── Send duty cycle command ───────────────────────────────────────
            if now - last_send >= SEND_INTERVAL:
                with self._throttle_lock:
                    duty = self._throttle
                try:
                    pkt = _make_set_duty_packet(duty)
                    ser.write(pkt)
                    last_send = now
                    write_fail_count = 0
                    if not self._connected:
                        self._connected = True
                        self._state.set(vesc_connected=True)
                    log.debug("vesc duty=%.4f raw=%d", duty, int(duty * 100000))
                except Exception as e:
                    write_fail_count += 1
                    log.warning("VESC write error (%d/%d): %s",
                                write_fail_count, MAX_WRITE_FAILS, e)
                    last_send = now
                    if write_fail_count >= MAX_WRITE_FAILS:
                        log.warning("VESC: too many write failures — reconnecting")
                        self._connected = False
                        self._state.set(vesc_connected=False)
                        write_fail_count = 0
                        try:
                            ser.close()
                        except Exception:
                            pass
                        ser = self._open_port()
                        read_buf.clear()

            # ── Poll telemetry ────────────────────────────────────────────────
            if now - last_poll >= POLL_INTERVAL:
                try:
                    ser.write(_GET_VALUES_PKT)
                    last_poll = now
                except Exception as e:
                    log.debug("VESC poll error: %s", e)

            # ── Read and parse response ───────────────────────────────────────
            try:
                incoming = ser.read(ser.in_waiting or 0)
            except serial.SerialException as e:
                log.warning("VESC disconnect: %s — reconnecting", e)
                self._connected = False
                self._state.set(vesc_connected=False)
                write_fail_count = 0
                ser.close()
                ser = self._open_port()
                read_buf.clear()
                continue

            if incoming:
                read_buf.extend(incoming)
                while len(read_buf) > 0:
                    payload, consumed = _find_packet(read_buf)
                    if payload is None:
                        break
                    del read_buf[:consumed]
                    fields = _parse_get_values(payload)
                    if fields is not None:
                        rpm      = fields["rpm"]
                        voltage  = fields["input_voltage"]
                        velocity = self._rpm_to_ms(-rpm)
                        self._state.update_vesc(velocity, rpm, voltage)
                        log.debug("vesc telem rpm=%.0f vel=%.3f volt=%.1fV duty_live=%.3f",
                                  rpm, velocity, voltage, fields["duty_cycle_now"])

            time.sleep(0.001)  # 1ms yield

        # Ensure motor is stopped on exit
        try:
            ser.write(_STOP_PKT)
        except Exception:
            pass
        ser.close()
        log.info("vesc_interface stopped")
