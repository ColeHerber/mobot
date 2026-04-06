# CMU Mobot — Build Documentation

**Competition:** 32nd Annual CMU Mobot Slalom Race, Spring 2026  
**Course:** Paved walk in front of Wean Hall — wavy white line connecting sequential gates  
**Key rule:** Lines split and merge at the end of the course. The correct path is told by judges immediately before each heat. The robot must support a race-morning route map.

---

## Hardware

| Component | Part | Role |
|---|---|---|
| Chassis | Tamiya TT-02R 4WD Touring Car | Drive platform |
| Compute | Raspberry Pi 5 (4GB or 8GB) | Main logic, all high-level control |
| Sensor MCU | Teensy 4.x | Real-time sensor sampling |
| Line sensor | QTRX-MD-16A (16-channel, 8mm pitch, analog) | Line detection |
| IMU | BNO085 | Heading + onboard sensor fusion |
| Motor controller | VESC FlipSky 6.7 Pro | Brushless motor drive + encoder reporting |
| Steering | GoBILDA Dual-Mode Torque Servo | Front steering |
| Power | Single LiPo (3S or 4S) | All power via VESC BEC |

---

## Communication Architecture

### Teensy ↔ Raspberry Pi: USB Serial

**Decision: USB Serial (not SPI, not UART GPIO)**

The Teensy connects to the Pi via a single USB cable, appearing as `/dev/ttyACM0`. This was chosen over SPI for three reasons. First, the Pi's hardware UART pins are fully occupied by the VESC connection — adding SPI would require careful GPIO management with no bandwidth benefit for this application. Second, SPI requires the Pi to be the bus master, meaning the Pi must poll constantly rather than the Teensy pushing data when ready; implementing a SPI slave state machine on the Teensy adds significant firmware complexity. Third, USB Full-Speed at 12 Mbps gives far more headroom than the sensor data rate requires, and a 1ms USB polling interval is well within the 5ms budget for a 200 Hz control loop. USB Serial also enables straightforward debugging with `screen /dev/ttyACM0 115200` during development.

### Pi ↔ VESC: Hardware UART

The VESC FlipSky 6.7 connects to the Pi via hardware UART (`/dev/serial0`, GPIO pins 14/15). On the Pi 5, the Bluetooth chip connects via the RP1 southbridge (not the GPIO UART), so there is no UART/BT contention — no `disable-bt` overlay or service disable is needed. If `/dev/serial0` is not present, add `dtoverlay=uart0` to `/boot/firmware/config.txt`. This is a **bidirectional** connection: the Pi sends throttle commands (duty cycle or current setpoint) and the VESC returns real-time telemetry including motor velocity (RPM), encoder position, input voltage, and motor temperature via the standard VESC UART protocol. Use `pyvesc` or a custom implementation of the VESC serial protocol. Baud rate: 115200.

The motor encoder is read **through the VESC** over this same UART link — no separate encoder wiring is needed. The VESC reads the encoder internally and reports it in the `GET_VALUES` response packet.

### BNO085 ↔ Pi: I2C

The BNO085 connects to the Pi's I2C bus (GPIO 2/3, `/dev/i2c-1`). Use the Adafruit `adafruit_bno08x` Python library. The BNO085 runs Hillcrest's SH-2 fusion firmware internally and outputs a pre-fused **rotation vector (quaternion)** — no custom attitude filter is needed. Request the `REPORT_ROTATION_VECTOR` report type. At 100 Hz this gives heading accurate to ~1° RMS.

### Pi → GoBILDA Servo: GPIO PWM

The GoBILDA dual-mode torque servo is controlled directly from a Pi GPIO pin using 50 Hz PWM (standard RC servo protocol: 1000–2000 µs pulse width). Use `pigpio` for hardware-timed PWM rather than `RPi.GPIO` software PWM — pigpio runs a daemon that generates accurate pulse widths without Linux scheduler jitter.

Servo center: ~1500 µs. Full lock: ~1000 / 2000 µs. Tune the exact range against the TT-02R steering geometry.

### Power: Single LiPo → VESC BEC

A single LiPo (3S or 4S) powers the entire system. The VESC's integrated BEC outputs regulated 5V which powers the Raspberry Pi 5 (via USB-C or GPIO 5V pins) and the Teensy (via USB or VIN). This eliminates the need for a separate logic battery and simplifies wiring. **Ensure the BEC is rated for at least 5A continuous** — the Pi 5 can draw up to 5A under load. Add `usb_max_current_enable=1` to `/boot/firmware/config.txt` when powering via GPIO 5V pins to suppress the low-voltage warning.

---

## Software Architecture

All control logic runs on the Raspberry Pi 5 in Python. The Teensy runs minimal C++ firmware for sensor sampling only. No ROS2 is used.

### Why no ROS2

ROS2 was considered and rejected for this build. The DDS middleware adds 5–20ms of jitter to the control loop, which is significant at competition speed. The sensor/actuator topology is simple enough (one sensor source, two actuator outputs) that ROS2's pub/sub modularity provides no architectural benefit. The tooling advantages (rqt_plot, rosbag) can be replaced by a simple `pyqtgraph` dashboard and CSV logging. Build time and debugging overhead for a 9-day timeline do not justify adoption.

### Teensy Firmware (`teensy/main.cpp`)

The Teensy runs a tight loop at ~1 kHz sampling all 16 analog channels of the QTRX-MD-16A. It computes a **weighted centroid** of the sensor readings to derive a single `line_position` value (range: −1.0 to +1.0, where 0 is centered). It also exposes raw per-channel values for calibration. Data is packed into a compact binary struct and streamed over USB Serial to the Pi at ~200 Hz.

Packet format (example, 8 bytes):
```
[0xAA] [line_pos_hi] [line_pos_lo] [raw_16bit_flags_hi] [raw_16bit_flags_lo] [confidence] [checksum] [0x55]
```

The Teensy **does not run any PID or steering logic**. It is a dedicated sensor coprocessor.


#### 1. Pulsed Multiplexing (Data Acquisition)
Instead of leaving all Infrared (IR) LEDs on continuously, the firmware pulses them sequentially. This drastically reduces power consumption, prevents localized thermal drift, and minimizes infrared crosstalk between adjacent sensors.

The hardware timer triggers the following sequence for each sensor in the array:
1. Turn ON the specific IR LED.
2. Wait a few microseconds for the phototransistor to stabilize.
3. Trigger the Analog-to-Digital Converter (ADC) to sample the reflection.
4. Turn OFF the IR LED.
5. Move to the next sensor via the multiplexer.

#### 2. Auto-Calibration and Normalization
Raw ADC values vary wildly depending on ambient room lighting, battery voltage, sensor height, and the specific track's paint. Before racing, the robot performs a calibration sweep to record the absolute minimum (darkest) and maximum (brightest) reflectance values for *each individual sensor*.

During the race, raw readings are dynamically mapped to a normalized scale (typically `0` to `1000`):

`Normalized_Value = ((Raw_Value - Min_Value) * 1000) / (Max_Value - Min_Value)`

* A value of `0` means the sensor sees pure background (e.g., black floor).
* A value of `1000` means the sensor is perfectly centered over the line (e.g., white line).


### Pi Software Stack

All Python modules run as threads sharing a common state object, protected by a threading lock. The main loop runs at 100–200 Hz.

#### `sensor_reader.py`

Reads the Teensy USB serial port (`/dev/ttyACM0`) and the BNO085 via I2C. Parses incoming sensor packets and writes `line_position`, `sensor_confidence`, and per-channel raw values to shared state. Reads BNO085 rotation vector and writes `heading_rad` (yaw extracted from quaternion) to shared state.

#### `vesc_interface.py`

Manages the VESC UART connection (`/dev/serial0`). Sends duty-cycle or current commands at the control loop rate. Polls `GET_VALUES` to retrieve motor RPM and converts to linear velocity (m/s) using wheel circumference and gear ratio. Writes `wheel_velocity_ms` to shared state.

#### `odometry.py`

Implements the **unicycle wheel-inertial odometry (WIO)** model. At each timestep `dt`:

```python
v     = shared.wheel_velocity_ms        # from VESC encoder
theta = shared.heading_rad              # from BNO085 directly (no integration needed)
x    += v * math.cos(theta) * dt
y    += v * math.sin(theta) * dt
```

The BNO085's onboard fusion means `theta` is already drift-corrected — no integration of gyro data is performed by the Pi. Position (`x`, `y`) accumulates over the run. The pose is reset to (0, 0, theta_0) at the start line.

This simple model is sufficient for the Mobot course. An EKF would add ~5% position accuracy on a ~30s run at the cost of a week of implementation and tuning time that does not exist in the competition timeline.

#### `pid_controller.py`

Computes steering output from `line_position`. Uses a standard PD controller (derivative on measurement, not error, to avoid derivative kick on setpoint changes):

```python
error      = 0.0 - line_position        # target is center (0)
derivative = (error - prev_error) / dt
steering   = Kp * error + Kd * derivative
prev_error = error
```

Speed control uses a separate P controller that reduces speed when `abs(line_position)` is large (i.e., on tight curves). All gains are tunable at runtime via a parameter file loaded at startup.

#### `state_machine.py`

The state machine governs high-level behavior:

| State | Trigger | Behavior |
|---|---|---|
| `LINE_FOLLOW` | Default | PID on line_position, odometry accumulates |
| `GATE_APPROACH` | Odometry within Xm of known gate | Slow to gate speed, align heading |
| `DEAD_RECKON` | Sensor confidence drops below threshold | Use odometry to drive expected arc, ignore PID |
| `REACQUIRE` | Line reappears after dead reckon | Snap back to LINE_FOLLOW |
| `INTERSECTION` | Odometry matches intersection waypoint | Execute pre-loaded turn direction |

The intersection turn directions (left/right at each split) are loaded from a YAML file at startup. **This file is edited race-morning** after the judges announce the route. The robot must be stopped, the YAML updated via SSH, and restarted — the whole process takes under 60 seconds.

#### `servo_control.py`

Uses `pigpio` to generate hardware-timed PWM on the designated GPIO pin. Maps steering output (−1.0 to +1.0) to pulse width (min_pw to max_pw µs). The servo is re-zeroed at startup with a brief calibration pulse.

#### `main.py`

Starts all threads, initializes the state machine, and runs the main control loop. Logs all state to a CSV file at 50 Hz for post-run analysis. Provides a simple `curses` terminal display showing live sensor position, heading, odometry pose, and current state.

---

## Repository Structure

```
mobot/
├── teensy/
│   └── main.cpp               # Teensy firmware (PlatformIO)
├── pi/
│   ├── main.py                # Entry point + main loop
│   ├── sensor_reader.py       # Teensy USB + BNO085 I2C
│   ├── vesc_interface.py      # UART throttle + encoder read
│   ├── odometry.py            # Unicycle WIO
│   ├── pid_controller.py      # Line PD + speed control
│   ├── state_machine.py       # State machine
│   ├── servo_control.py       # pigpio PWM output
│   └── shared_state.py        # Thread-safe state object
├── config/
│   ├── params.yaml            # PID gains, speed limits, servo limits
│   └── route.yaml             # Race-morning intersection map
├── logs/                      # Auto-generated run CSVs
└── README.md
```

---

## Sensor Array Setup

The QTRX-MD-16A should be mounted at the **front of the chassis**, centered laterally, at **3mm from the ground** (per the datasheet optimal range). At 8mm pitch × 16 channels, the sensing width is 128mm — ensure this exceeds the painted line width with margin on both sides.

**Calibration procedure:** Before each run, hold the robot stationary over the line (centered), then over bare pavement. The Teensy firmware should record min/max per-channel values and normalize to [0, 1]. Store calibration values in EEPROM so they persist between power cycles.

**Confidence metric:** Compute the sum of all channel readings. If the sum is below a threshold (line not detected by enough sensors), set `sensor_confidence = LOW` to trigger `DEAD_RECKON` state.

---

## Key Risks and Mitigations

**Outdoor lighting:** The Mobot course is outdoors. Sunlight angle changes between heats and can cause glare on the white line or reduce contrast. Recalibrate the sensor array before each heat if possible. The 2023 race notes specifically called out glare as a challenge.

**Weather:** The 2024 race ran in rain and wind. Ensure all electronics are adequately protected. The Pi and Teensy should be in an enclosure; the VESC is typically rated for some moisture but avoid direct water exposure.

**USB Serial disconnects:** Linux can occasionally re-enumerate USB devices. Add reconnection logic to `sensor_reader.py` that detects a disconnected port and reopens it without crashing the control loop.

**VESC UART framing errors:** Implement a checksum check on all VESC responses and discard malformed packets. The VESC protocol uses CRC16 for validation.

**Battery voltage sag:** As the LiPo discharges, the BEC output voltage drops. Monitor VESC-reported input voltage. Below ~10V (3S) or ~14V (4S), performance degrades — this is the "you can never have enough battery power" lesson from the 2025 race notes.

---

## Development Priority Order

Given 9 days to competition:

1. Flash Teensy firmware, verify sensor readings over USB serial (day 1)
2. Verify VESC UART connection, confirm encoder velocity readout (day 1–2)
3. Verify BNO085 heading readout over I2C (day 2)
4. Wire servo, confirm pigpio PWM control (day 2)
5. Implement sensor_reader + vesc_interface + servo_control (day 3)
6. Implement PID controller, tune Kp/Kd on straight course (day 3–4)
7. Implement odometry, validate against manual measurement (day 4–5)
8. Implement state machine with LINE_FOLLOW and DEAD_RECKON states (day 5–6)
9. Test full course pass — gates, curves (day 6–7)
10. Implement INTERSECTION state and route YAML (day 7–8)
11. Reliability testing, edge cases, battery endurance (day 8–9)
12. Race-day: recalibrate sensors, load route YAML, run heats

---

## Race Day Checklist

- [ ] Charge LiPo to storage voltage the night before, full charge race morning
- [ ] SSH into Pi, verify all processes start cleanly
- [ ] Run sensor calibration with robot on course line
- [ ] Confirm VESC is configured for correct motor poles, encoder PPR, and current limits
- [ ] Get intersection route from judges, update `config/route.yaml`, restart
- [ ] One dry run at reduced speed before the heat
- [ ] Have a team member follow the robot on course (permitted by rules) ready to grab if it goes off-course
