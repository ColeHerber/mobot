#!/usr/bin/env bash

set -euo pipefail

# Export the current four-layer board for DigiKey Red and create a flat ZIP.
# Usage:
#   ./scripts/export_dkred.sh [board.kicad_pcb] [output.zip]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

BOARD_FILE="${1:-$PROJECT_DIR/Esp32 Can+IMU.kicad_pcb}"
OUTPUT_ZIP="${2:-$PROJECT_DIR/production/Esp32_Can_IMU_DKRed.zip}"

if [[ ! -f "$BOARD_FILE" ]]; then
    echo "Error: board file not found: $BOARD_FILE" >&2
    exit 1
fi

if [[ "${OUTPUT_ZIP##*.}" != "zip" ]]; then
    echo "Error: output filename must end in .zip: $OUTPUT_ZIP" >&2
    exit 1
fi

KICAD_CLI="${KICAD_CLI:-}"
if [[ -z "$KICAD_CLI" ]]; then
    KICAD_CLI="$(command -v kicad-cli || true)"
fi
if [[ -z "$KICAD_CLI" && -x /Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli ]]; then
    KICAD_CLI=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
fi
if [[ -z "$KICAD_CLI" || ! -x "$KICAD_CLI" ]]; then
    echo "Error: kicad-cli was not found. Install KiCad or set KICAD_CLI." >&2
    exit 1
fi

TEMP_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/dkred-export.XXXXXX")"
EXPORT_DIR="$TEMP_ROOT/files"
TEMP_ZIP="$TEMP_ROOT/dkred.zip"
trap 'rm -rf "$TEMP_ROOT"' EXIT
mkdir -p "$EXPORT_DIR"

echo "Exporting DKRed Gerbers from: $BOARD_FILE"
echo "Note: KiCad CLI exports the last saved board state. Save the board and refill zones first."

"$KICAD_CLI" pcb export gerbers \
    --output "$EXPORT_DIR/" \
    --layers F.Cu,In1.Cu,In2.Cu,B.Cu,F.Mask,B.Mask,F.SilkS,B.SilkS,Edge.Cuts \
    --subtract-soldermask \
    "$BOARD_FILE"

"$KICAD_CLI" pcb export drill \
    --output "$EXPORT_DIR/" \
    --format excellon \
    --drill-origin absolute \
    --excellon-zeros-format decimal \
    --excellon-oval-format route \
    --excellon-units mm \
    --excellon-separate-th \
    "$BOARD_FILE"

require_one() {
    local pattern="$1"
    local description="$2"
    local count
    count="$(find "$EXPORT_DIR" -maxdepth 1 -type f -name "$pattern" | wc -l | tr -d '[:space:]')"
    if [[ "$count" != "1" ]]; then
        echo "Error: expected one $description file matching '$pattern'; found $count." >&2
        exit 1
    fi
}

require_one '*.gtl' 'top copper'
require_one '*.g1' 'inner copper layer 1'
require_one '*.g2' 'inner copper layer 2'
require_one '*.gbl' 'bottom copper'
require_one '*.gts' 'top solder mask'
require_one '*.gbs' 'bottom solder mask'
require_one '*.gto' 'top silkscreen'
require_one '*.gbo' 'bottom silkscreen'
require_one '*.gm1' 'board outline'
require_one '*-PTH.drl' 'plated drill'
require_one '*-NPTH.drl' 'non-plated drill'

FILES=()
while IFS= read -r -d '' file; do
    FILES+=("$file")
done < <(
    find "$EXPORT_DIR" -maxdepth 1 -type f \
        \( -name '*.gtl' -o -name '*.g1' -o -name '*.g2' -o -name '*.gbl' \
        -o -name '*.gts' -o -name '*.gbs' -o -name '*.gto' -o -name '*.gbo' \
        -o -name '*.gm1' -o -name '*.drl' \) -print0
)

# COPYFILE_DISABLE prevents macOS resource forks. -X strips extra attributes,
# and -j stores every fabrication file at the root of the archive.
COPYFILE_DISABLE=1 zip -X -j -q "$TEMP_ZIP" "${FILES[@]}"
unzip -tqq "$TEMP_ZIP"

if unzip -Z1 "$TEMP_ZIP" | grep -Eq '(^|/)__MACOSX/|(^|/)\._'; then
    echo "Error: macOS resource-fork files were found in the archive." >&2
    exit 1
fi

if unzip -Z1 "$TEMP_ZIP" | grep -q '/'; then
    echo "Error: the archive contains nested directories." >&2
    exit 1
fi

mkdir -p "$(dirname "$OUTPUT_ZIP")"
mv -f "$TEMP_ZIP" "$OUTPUT_ZIP"

echo
echo "Created: $OUTPUT_ZIP"
echo "Archive contents:"
unzip -Z1 "$OUTPUT_ZIP" | sort
