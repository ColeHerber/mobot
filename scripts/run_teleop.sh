#!/bin/bash
# Start teleop-only mode: VESC + servo + web server, no line sensor or PID.
# Open http://<pi-ip>:5000/teleop on your phone to drive.
# Usage: bash scripts/run_teleop.sh
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$SCRIPT_DIR/.."

cd "$ROOT/pi"
python3 main.py \
    --config "$ROOT/config/params.yaml" \
    --route  "$ROOT/config/route.yaml" \
    --teleop-only
