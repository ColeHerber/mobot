#!/bin/bash
# Start the web server in standalone mode (no robot running).
# Open http://<pi-ip>:5000 from any browser on the network.
# Usage: bash scripts/run_web.sh
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$SCRIPT_DIR/.."

cd "$ROOT/pi"
python3 web_server.py \
    --standalone \
    --config "$ROOT/config/params.yaml" \
    --route  "$ROOT/config/route.yaml" \
    --static "$ROOT/static"
