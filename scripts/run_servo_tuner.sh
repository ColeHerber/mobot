#!/bin/bash
# Interactive terminal servo center tuner.
# Use arrow keys / A/D to nudge center_offset_us; S to save; Q to quit.
# Also accessible via http://<pi-ip>:5000/servo (requires web server running).
# Usage: bash scripts/run_servo_tuner.sh
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$SCRIPT_DIR/.."

cd "$ROOT/pi"
python3 tune_servo_center.py \
    --config "$ROOT/config/params.yaml"
