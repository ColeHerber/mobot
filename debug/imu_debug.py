#!/usr/bin/env python3
"""BNO085 IMU debug tool — print quaternion and yaw over I2C.

Usage:
    python3 imu_debug.py [--address 0x4A] [--i2c-bus 1]

Output (refreshed ~20 Hz):
    Yaw: 47.3°  | w: 0.923  x: 0.001  y: -0.002  z: 0.385

The BNO085 is read via the Adafruit CircuitPython library (same driver used
by the robot stack). Ensure adafruit-circuitpython-bno08x and adafruit-blinka
are installed:
    pip3 install adafruit-circuitpython-bno08x adafruit-blinka
    sudo apt install python3-smbus i2c-tools

I2C bus 1 is the standard Pi I2C bus (GPIO 2/3, pins 3/5).
"""

import argparse
import math
import sys
import time

try:
    import board
    import busio
except ImportError:
    print("ERROR: adafruit-blinka not installed.")
    print("       pip3 install adafruit-blinka")
    print("       sudo apt install python3-smbus i2c-tools")
    sys.exit(1)

try:
    import adafruit_bno08x
    from adafruit_bno08x.i2c import BNO08X_I2C
except ImportError:
    print("ERROR: adafruit-circuitpython-bno08x not installed.")
    print("       pip3 install adafruit-circuitpython-bno08x")
    sys.exit(1)

POLL_HZ = 20
POLL_DT = 1.0 / POLL_HZ


def quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    """Extract yaw (rotation about Z axis) from a quaternion, in degrees."""
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def main():
    parser = argparse.ArgumentParser(description="BNO085 IMU debug")
    parser.add_argument("--address", default="0x4A",
                        help="I2C address (default: 0x4A)")
    parser.add_argument("--i2c-bus", type=int, default=1,
                        help="I2C bus number (default: 1)")
    args = parser.parse_args()

    addr = int(args.address, 16) if isinstance(args.address, str) else args.address

    print(f"Connecting to BNO085 at I2C address 0x{addr:02X} on bus {args.i2c_bus}…")
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        bno = BNO08X_I2C(i2c, address=addr)
        bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        print("BNO085 connected — press Ctrl+C to exit\n")
    except Exception as e:
        print(f"ERROR: failed to init BNO085: {e}")
        sys.exit(1)

    try:
        while True:
            t0 = time.monotonic()
            try:
                quat = bno.quaternion   # (i, j, k, real) — note Adafruit order
                if quat is None:
                    raise ValueError("no data")
                qi, qj, qk, qr = quat
                # Adafruit returns (i, j, k, real) = (x, y, z, w)
                w, x, y, z = qr, qi, qj, qk
                yaw = quat_to_yaw(w, x, y, z)
                print(f"\rYaw: {yaw:+7.2f}°  | "
                      f"w: {w:+.4f}  x: {x:+.4f}  y: {y:+.4f}  z: {z:+.4f}   ",
                      end="", flush=True)
            except Exception as e:
                print(f"\r[error: {e}]   ", end="", flush=True)

            elapsed  = time.monotonic() - t0
            sleep_dt = POLL_DT - elapsed
            if sleep_dt > 0:
                time.sleep(sleep_dt)

    except KeyboardInterrupt:
        print("\n\nExiting.")


if __name__ == "__main__":
    main()
