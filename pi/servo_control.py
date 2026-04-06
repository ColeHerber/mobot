"""Servo controller using lgpio hardware-timed PWM.

Maps steering value [-1.0, +1.0] to pulse width [min_pw, max_pw] µs.
Positive steering = right, negative = left. Tune min_pw/max_pw against the
TT-02R steering geometry so that +1.0 and -1.0 are full lock.

Pi 5 note: The Pi 5 routes GPIO through the RP1 chip on gpiochip4.
pigpio is NOT compatible with Pi 5. This module uses lgpio instead, which
works on both Pi 4 (gpiochip0) and Pi 5 (gpiochip4).

Set servo.gpiochip in params.yaml:
  Pi 4: gpiochip: 0
  Pi 5: gpiochip: 4

Install: sudo apt install python3-lgpio
         pip3 install lgpio
"""

import logging
import time

log = logging.getLogger(__name__)

try:
    import lgpio
    _LGPIO_AVAILABLE = True
except ImportError:
    log.warning("lgpio not installed — servo will run in dummy mode")
    _LGPIO_AVAILABLE = False

_SERVO_FREQ_HZ = 50  # Standard RC servo: 50 Hz (20ms period)


class ServoControl:
    def __init__(self, config: dict):
        servo_cfg = config.get("servo", {})
        self._pin      = int(servo_cfg.get("gpio_pin",  18))
        self._min_pw   = int(servo_cfg.get("min_pw",  1000))
        self._max_pw   = int(servo_cfg.get("max_pw",  2000))
        self._center   = int(servo_cfg.get("center_pw", 1500))
        self._gpiochip = int(servo_cfg.get("gpiochip", 4))  # 4 for Pi 5, 0 for Pi 4

        # center_offset_us allows fine-tuning the servo center without changing
        # center_pw. Use tune_servo_center.py to adjust interactively.
        self._center_offset = int(servo_cfg.get("center_offset_us", 0))
        self._effective_center = self._center + self._center_offset

        self._h = None  # lgpio chip handle

        if _LGPIO_AVAILABLE:
            try:
                self._h = lgpio.gpiochip_open(self._gpiochip)
                lgpio.gpio_claim_output(self._h, self._pin)
                log.info("lgpio opened gpiochip%d, servo on GPIO %d",
                         self._gpiochip, self._pin)
                log.info("effective center = %d µs (center_pw=%d + offset=%d)",
                         self._effective_center, self._center, self._center_offset)
                # Brief calibration pulse: center → wait → ready
                lgpio.tx_servo(self._h, self._pin, self._effective_center, _SERVO_FREQ_HZ)
                time.sleep(0.3)
            except lgpio.error as e:
                log.error("lgpio init failed: %s — servo disabled", e)
                self._h = None
        else:
            log.warning("lgpio unavailable — servo in dummy mode")

    def set_steering(self, value: float):
        """Set steering position.

        Args:
            value: [-1.0, +1.0] — negative = left, positive = right
        """
        value = max(-1.0, min(1.0, value))
        ec = self._effective_center
        if value >= 0.0:
            pw = int(ec + value * (self._max_pw - ec))
        else:
            pw = int(ec + value * (ec - self._min_pw))

        pw = max(self._min_pw, min(self._max_pw, pw))

        if self._h is not None:
            lgpio.tx_servo(self._h, self._pin, pw, _SERVO_FREQ_HZ)

    def center(self):
        """Return servo to center position."""
        self.set_steering(0.0)

    def stop(self):
        """Return to center, then disable PWM output."""
        self.center()
        time.sleep(0.1)
        if self._h is not None:
            lgpio.tx_servo(self._h, self._pin, 0)  # 0 pulse width = disable
            lgpio.gpiochip_close(self._h)
            self._h = None
            log.info("lgpio closed")
