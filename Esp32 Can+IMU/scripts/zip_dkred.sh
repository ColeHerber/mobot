#!/usr/bin/env bash

set -euo pipefail

# Zip existing DKRed fabrication files without regenerating them.
# Usage:
#   ./scripts/zip_dkred.sh [input_directory] [output.zip]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

INPUT_DIR="${1:-$PROJECT_DIR/production/V2_DK_red}"
OUTPUT_ZIP="${2:-$PROJECT_DIR/production/V2_DK_red_clean.zip}"

if [[ ! -d "$INPUT_DIR" ]]; then
    echo "Error: input directory not found: $INPUT_DIR" >&2
    exit 1
fi

require_one() {
    local pattern="$1"
    local description="$2"
    local count
    count="$(find "$INPUT_DIR" -maxdepth 1 -type f -name "$pattern" | wc -l | tr -d '[:space:]')"
    if [[ "$count" != "1" ]]; then
        echo "Error: expected one $description file matching '$pattern'; found $count." >&2
        exit 1
    fi
}

require_one '*-F_Cu.gbr' 'top copper'
require_one '*-In1_Cu.gbr' 'inner copper layer 1'
require_one '*-In2_Cu.gbr' 'inner copper layer 2'
require_one '*-B_Cu.gbr' 'bottom copper'
require_one '*-F_Mask.gbr' 'top solder mask'
require_one '*-B_Mask.gbr' 'bottom solder mask'
require_one '*-F_Silkscreen.gbr' 'top silkscreen'
require_one '*-B_Silkscreen.gbr' 'bottom silkscreen'
require_one '*-Edge_Cuts.gbr' 'board outline'
require_one '*-PTH.drl' 'plated drill'
require_one '*-NPTH.drl' 'non-plated drill'

FILES=()
while IFS= read -r -d '' file; do
    FILES+=("$file")
done < <(
    find "$INPUT_DIR" -maxdepth 1 -type f \
        \( -name '*_Cu.gbr' -o -name '*_Mask.gbr' -o -name '*_Silkscreen.gbr' \
        -o -name '*-Edge_Cuts.gbr' -o -name '*.drl' \) -print0
)

TEMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/dkred-zip.XXXXXX")"
TEMP_ZIP="$TEMP_DIR/dkred.zip"
trap 'rm -rf "$TEMP_DIR"' EXIT

# Prevent Finder metadata/resource forks and store files at the ZIP root.
COPYFILE_DISABLE=1 zip -X -j -q "$TEMP_ZIP" "${FILES[@]}"
unzip -tqq "$TEMP_ZIP"

if unzip -Z1 "$TEMP_ZIP" | grep -Eq '(^|/)__MACOSX/|(^|/)\._|/'; then
    echo "Error: archive contains macOS metadata or nested directories." >&2
    exit 1
fi

mkdir -p "$(dirname "$OUTPUT_ZIP")"
mv -f "$TEMP_ZIP" "$OUTPUT_ZIP"

echo "Created: $OUTPUT_ZIP"
unzip -Z1 "$OUTPUT_ZIP" | sort
