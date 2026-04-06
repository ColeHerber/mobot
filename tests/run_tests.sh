#!/bin/bash
# Run all Mobot unit tests — no hardware required.
# Usage: bash tests/run_tests.sh [pytest-args...]
set -e
cd "$(dirname "$0")/.."
python3 -m pytest tests/ -v --tb=short "$@"
