"""Servo controller using Linux hardware PWM via sysfs.

Maps steering value [-1.0, +1.0] to pulse width [min_pw, max_pw] µs.
Positive steering = right, negative = left.

Uses /sys/class/pwm/ (RP1 hardware PWM on Pi 5) — zero software jitter.

Pi 5 setup (one-time, requires reboot):
  echo "dtoverlay=pwm,pin=12,func=4" | sudo tee -a /boot/firmware/config.txt
  sudo reboot

Then verify: ls /sys/class/pwm/pwmchip*/

Set servo.pwmchip in params.yaml:
  Pi 5 with GPIO 12: pwmchip: 0   (RP1 — confirmed via ls /sys/class/pwm/)
  Pi 4 with GPIO 12: pwmchip: 0

params.yaml example:
  servo:
    gpio_pin: 12        # only used for reference/logging now
    pwmchip: 2          # /sys/class/pwm/pwmchip<N>
    pwm_channel: 0      # pwm0
    min_pw:   1000      # µs — full left
    max_pw:   2000      # µs — full right
    center_pw: 1500     # µs — straight ahead
    center_offset_us: 0 # µs — fine trim
"""

import logging
import os
import time

log = logging.getLogger(__name__)

_PERIOD_NS = 20_000_000  # 50 Hz = 20 ms period in nanoseconds


def _us_to_ns(us: int) -> int:
    return us * 1000


class ServoControl:
    def __init__(self, config: dict):
        servo_cfg = config.get("servo", {})
        self._pin        = int(servo_cfg.get("gpio_pin",     12))
        self._min_pw     = int(servo_cfg.get("min_pw",     1000))
        self._max_pw     = int(servo_cfg.get("max_pw",     2000))
        self._center     = int(servo_cfg.get("center_pw",  1500))
        self._pwmchip    = int(servo_cfg.get("pwmchip",       0))
        self._channel    = int(servo_cfg.get("pwm_channel",   0))
        self._center_offset = int(servo_cfg.get("center_offset_us", 0))
        self._effective_center = self._center + self._center_offset

        self._pwm_dir = f"/sys/class/pwm/pwmchip{self._pwmchip}/pwm{self._channel}"
        self._ready = False

        self._setup()

    def _write(self, filename: str, value):
        path = os.path.join(self._pwm_dir, filename)
        with open(path, "w") as f:
            f.write(str(value))

    def _setup(self):
        chip_dir = f"/sys/class/pwm/pwmchip{self._pwmchip}"
        if not os.path.exists(chip_dir):
            log.error("PWM chip not found: %s — is dtoverlay=pwm loaded?", chip_dir)
            return

        # Export the channel if not already exported
        if not os.path.exists(self._pwm_dir):
            try:
                with open(f"{chip_dir}/export", "w") as f:
                    f.write(str(self._channel))
                time.sleep(0.1)  # sysfs node takes a moment to appear
            except OSError as e:
                log.error("Failed to export PWM channel %d: %s", self._channel, e)
                return

        try:
            self._write("period", _PERIOD_NS)
            self._write("duty_cycle", _us_to_ns(self._effective_center))
            self._write("enable", 1)
            self._ready = True
            log.info("Hardware PWM ready: pwmchip%d/pwm%d, GPIO %d",
                     self._pwmchip, self._channel, self._pin)
            log.info("effective center = %d µs (center_pw=%d + offset=%d)",
                     self._effective_center, self._center, self._center_offset)
        except OSError as e:
            log.error("Hardware PWM init failed: %s", e)

    def set_steering(self, value: float):
        """Set steering position.

        Args:
            value: [-1.0, +1.0] — negative = left, positive = right
        """
        if not self._ready:
            return
        value = max(-1.0, min(1.0, value))
        ec = self._effective_center
        if value >= 0.0:
            pw = int(ec + value * (self._max_pw - ec))
        else:
            pw = int(ec + value * (ec - self._min_pw))

        pw = max(self._min_pw, min(self._max_pw, pw))

        try:
            self._write("duty_cycle", _us_to_ns(pw))
            log.debug("servo steer=%.3f pw=%d µs", value, pw)
        except OSError as e:
            log.error("PWM write failed: %s", e)
            self._ready = False

    def center(self):
        """Return servo to center position."""
        self.set_steering(0.0)

    def stop(self):
        """Return to center, then disable PWM output."""
        self.center()
        time.sleep(0.1)
        if self._ready:
            try:
                self._write("enable", 0)
            except OSError:
                pass
            self._ready = False
            log.info("Hardware PWM disabled")
