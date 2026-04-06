#!/bin/bash
# Run the mapping pass at slow speed (max 0.15 m/s).
# Records path to logs/map_*.json and writes config/mapped_route.yaml on exit.
# Usage: bash scripts/run_mapper.sh [--dry-run] [--speed 0.10]
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$SCRIPT_DIR/.."

DRY_RUN=""
EXTRA_ARGS=()

for arg in "$@"; do
    if [[ "$arg" == "--dry-run" ]]; then
        DRY_RUN="--dry-run"
        echo "[run_mapper.sh] DRY RUN MODE — simulated sensors"
    else
        EXTRA_ARGS+=("$arg")
    fi
done

cd "$ROOT/pi"
python3 mapper.py \
    --config "$ROOT/config/params.yaml" \
    --route  "$ROOT/config/route.yaml" \
    $DRY_RUN \
    "${EXTRA_ARGS[@]}"
