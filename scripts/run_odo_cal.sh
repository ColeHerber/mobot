#!/bin/bash
# Run the interactive odometry calibration tool.
# Usage: bash scripts/run_odo_cal.sh [--dry-run]
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$SCRIPT_DIR/.."

DRY_RUN=""
for arg in "$@"; do
    if [[ "$arg" == "--dry-run" ]]; then
        DRY_RUN="--dry-run"
        echo "[run_odo_cal.sh] DRY RUN MODE — no actuator output"
    fi
done

cd "$ROOT/pi"
python3 odo_cal.py \
    --config "$ROOT/config/params.yaml" \
    $DRY_RUN
