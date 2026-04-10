#!/bin/bash
# Multi-surface sensor calibration — captures bare ground + line on each surface type.
# Usage: bash scripts/run_calibrate_multi.sh [--surfaces N]
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$SCRIPT_DIR/.."
cd "$ROOT"
source .venv/bin/activate 2>/dev/null || true
python3 debug/calibrate_multi.py "$@"
