#!/bin/bash
# Enter Teensy calibration mode.
# Move the robot slowly over the full width of the line, then press Ctrl+C to save.
# Usage: bash scripts/calibrate.sh [/dev/ttyACM0]
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$SCRIPT_DIR/.."

PORT="${1:-/dev/ttyACM0}"
echo "[calibrate.sh] Connecting to Teensy on $PORT..."

python3 - "$PORT" <<'EOF'
import serial
import sys
import time

port = sys.argv[1]
try:
    s = serial.Serial(port, 115200, timeout=1)
except Exception as e:
    print(f"Cannot open {port}: {e}")
    sys.exit(1)

time.sleep(0.5)
s.write(b'C')
print(f"Calibration mode active on {port}.")
print("Move robot slowly over the full width of the line.")
print("Press Ctrl+C when done — calibration will be saved.")
print("")
print("Ch: 00   01   02   03   04   05   06   07   08   09   10   11   12   13   14   15")

try:
    while True:
        line = s.readline()
        if line:
            vals = line.decode(errors='replace').strip().split(',')
            if len(vals) == 16:
                row = "    " + "  ".join(f"{int(v):4d}" for v in vals)
                print(row, end='\r')
except KeyboardInterrupt:
    print("\nSaving calibration...")
    s.write(b'S')
    time.sleep(0.3)
    s.write(b'R')
    print("Saved. Teensy back in run mode.")
    s.close()
EOF
