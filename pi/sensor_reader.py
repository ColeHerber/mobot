"""Sensor reader thread — Teensy USB serial + BNO085 I2C.

Runs as a daemon thread. Continuously:
  - Reads 8-byte packets from Teensy on /dev/ttyACM0 (line sensor + flags + confidence)
  - Reads rotation vector from BNO085 over I2C and extracts yaw
  - Writes results to SharedState

Packet format from Teensy (8 bytes):
  [0xAA] [pos_hi] [pos_lo] [flags_hi] [flags_lo] [confidence] [checksum] [0x55]
  line_pos  = int16(pos_hi<<8|pos_lo) / 10000.0
  flags     = 16-bit bitmask of sensors above threshold
  confidence = 0–255
  checksum  = XOR of bytes 1–5
"""

import math
import struct
import threading
import time
import logging

import serial

log = logging.getLogger(__name__)

PACKET_LEN = 8
PACKET_START = 0xAA
PACKET_END   = 0x55


def _extract_yaw(quat_i, quat_j, quat_k, quat_real) -> float:
    """Extract yaw (rotation around Z) from BNO085 rotation vector quaternion.

    BNO085 reports (i, j, k, real) = (x, y, z, w).
    yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
    Returns yaw in radians in (-π, π].
    """
    w, x, y, z = quat_real, quat_i, quat_j, quat_k
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return yaw


def _open_teensy(port: str, baud: int) -> serial.Serial:
    """Open serial port, returning a Serial object. Blocks until successful."""
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=0.1)
            log.info("Opened Teensy on %s", port)
            return ser
        except serial.SerialException as e:
            log.warning("Cannot open %s: %s — retrying in 1s", port, e)
            time.sleep(1.0)


def _open_bno085(i2c_address: int):
    """Open BNO085 over I2C. Returns sensor object or None on failure."""
    try:
        import board
        import busio
        from adafruit_bno08x.i2c import BNO08X_I2C
        from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

        i2c = busio.I2C(board.SCL, board.SDA)
        bno = BNO08X_I2C(i2c, address=i2c_address)
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        log.info("BNO085 opened at I2C 0x%02X", i2c_address)
        return bno
    except Exception as e:
        log.warning("BNO085 not available: %s — heading will remain 0.0", e)
        return None


class SensorReader:
    """Reads Teensy packets and BNO085 in a background daemon thread."""

    def __init__(self, shared_state, config: dict):
        self._state = shared_state
        self._port  = config["sensor"]["port"]
        self._baud  = config["sensor"]["baud"]
        self._i2c_addr = int(config["imu"]["i2c_address"], 16) \
            if isinstance(config["imu"]["i2c_address"], str) \
            else config["imu"]["i2c_address"]

        self._thread = threading.Thread(target=self._run, name="sensor_reader",
                                        daemon=True)

    def start(self):
        self._thread.start()

    def _parse_packet(self, buf: bytes):
        """Parse a validated 8-byte packet into (line_pos, flags, confidence).

        Returns None if checksum fails.
        """
        if buf[0] != PACKET_START or buf[7] != PACKET_END:
            return None

        pos_hi, pos_lo, fl_hi, fl_lo, confidence, checksum = \
            buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]

        expected_chk = pos_hi ^ pos_lo ^ fl_hi ^ fl_lo ^ confidence
        if checksum != expected_chk:
            return None

        line_pos_i16 = struct.unpack('>h', bytes([pos_hi, pos_lo]))[0]
        line_pos = line_pos_i16 / 10000.0
        flags    = (fl_hi << 8) | fl_lo

        # Decode per-channel normalized values from flags bitmask
        # (flags give threshold-crossed status; full raw values not in packet)
        raw = [(1000 if (flags >> i) & 1 else 0) for i in range(16)]

        return line_pos, flags, confidence, raw

    def _run(self):
        ser = _open_teensy(self._port, self._baud)
        bno = _open_bno085(self._i2c_addr)

        buf = bytearray()

        while self._state.running:
            # ── IMU read ─────────────────────────────────────────────────────
            if bno is not None:
                try:
                    quat = bno.quaternion  # (i, j, k, real)
                    if quat is not None:
                        yaw = _extract_yaw(*quat)
                        self._state.update_imu(yaw)
                except Exception as e:
                    log.debug("BNO085 read error: %s", e)

            # ── Teensy packet read ────────────────────────────────────────────
            try:
                data = ser.read(ser.in_waiting or 1)
            except serial.SerialException as e:
                log.warning("Teensy disconnect: %s — reconnecting", e)
                ser.close()
                ser = _open_teensy(self._port, self._baud)
                buf.clear()
                continue

            buf.extend(data)

            # Scan buffer for complete packets
            while len(buf) >= PACKET_LEN:
                # Find start byte
                start_idx = buf.find(PACKET_START)
                if start_idx == -1:
                    buf.clear()
                    break
                if start_idx > 0:
                    del buf[:start_idx]  # discard garbage before start
                if len(buf) < PACKET_LEN:
                    break

                pkt = bytes(buf[:PACKET_LEN])
                result = self._parse_packet(pkt)
                if result is not None:
                    line_pos, flags, confidence, raw = result
                    self._state.update_sensor(line_pos, confidence, flags, raw)
                    del buf[:PACKET_LEN]
                else:
                    # Bad packet — discard just the start byte and re-sync
                    del buf[:1]

        ser.close()
        log.info("sensor_reader stopped")
