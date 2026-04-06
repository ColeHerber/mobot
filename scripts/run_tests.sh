#!/bin/bash
# Run all unit tests — NO hardware required. Runs on any machine.
# Usage: bash scripts/run_tests.sh [pytest-args...]
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$SCRIPT_DIR/.."

cd "$ROOT"
python3 -m pytest tests/ -v --tb=short "$@"
