#!/bin/bash
# Start the main Mobot controller (live hardware by default).
# Usage: bash scripts/run.sh [--dry-run]
#   --dry-run   Sensor display only, no actuator output (safe for bench testing)
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$SCRIPT_DIR/.."

DRY_RUN=""
if [[ "$1" == "--dry-run" ]]; then
    DRY_RUN="--dry-run"
    echo "[run.sh] DRY RUN MODE — no actuator output"
fi

cd "$ROOT/pi"
python3 main.py \
    --config "$ROOT/config/params.yaml" \
    --route  "$ROOT/config/route.yaml" \
    $DRY_RUN
