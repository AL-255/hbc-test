#!/usr/bin/env bash
#
# flash.sh — turn-key build + flash for the 1 MHz complementary pulse firmware
# Board: Heltec WiFi LoRa 32 V3 (HTIT-WB32LA(F)_V3, ESP32-S3)
#
# Usage:
#   ./flash.sh                 # build, auto-detect port, flash
#   ./flash.sh -m              # ...and open the serial monitor afterwards
#   ./flash.sh -p /dev/ttyUSB0 # force a specific serial port
#   ./flash.sh -b              # build only, no flash
#   PORT=/dev/ttyACM0 ./flash.sh
#
set -euo pipefail

# ---- Configuration ------------------------------------------------------
IDF_PATH_DEFAULT="$HOME/.espressif/v6.0.1/esp-idf"
TARGET="esp32s3"
# ------------------------------------------------------------------------

# Always operate from the directory this script lives in (the project root).
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

PORT="${PORT:-}"
DO_FLASH=1
DO_MONITOR=0

while getopts "p:mbh" opt; do
  case "$opt" in
    p) PORT="$OPTARG" ;;
    m) DO_MONITOR=1 ;;
    b) DO_FLASH=0 ;;
    h)
      grep '^#' "$0" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    *) echo "Unknown option. Run '$0 -h' for help." >&2; exit 2 ;;
  esac
done

# ---- 1. Activate ESP-IDF -------------------------------------------------
IDF_PATH="${IDF_PATH:-$IDF_PATH_DEFAULT}"
if [ ! -f "$IDF_PATH/export.sh" ]; then
  echo "ERROR: ESP-IDF not found at '$IDF_PATH'." >&2
  echo "       Set IDF_PATH to your install, e.g. IDF_PATH=~/.espressif/v6.0.1/esp-idf $0" >&2
  exit 1
fi
echo ">> Activating ESP-IDF: $IDF_PATH"
# export.sh references unset vars; relax 'nounset' just for sourcing it.
set +u
# shellcheck disable=SC1091
source "$IDF_PATH/export.sh" >/dev/null
set -u

# ---- 2. Select target (only when it changes) -----------------------------
CURRENT_TARGET="$(sed -n 's/^CONFIG_IDF_TARGET="\(.*\)"/\1/p' sdkconfig 2>/dev/null || true)"
if [ "$CURRENT_TARGET" != "$TARGET" ]; then
  echo ">> Setting target: $TARGET"
  idf.py set-target "$TARGET"
fi

# ---- 3. Build ------------------------------------------------------------
echo ">> Building..."
idf.py build

if [ "$DO_FLASH" -eq 0 ]; then
  echo ">> Build complete (flash skipped)."
  exit 0
fi

# ---- 4. Auto-detect serial port if not given -----------------------------
if [ -z "$PORT" ]; then
  # Heltec V3 uses a CP2102 (ttyUSB*); ttyACM* covers native-USB fallbacks.
  mapfile -t PORTS < <(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true)
  case "${#PORTS[@]}" in
    0)
      echo "ERROR: no serial port found (looked for /dev/ttyUSB* and /dev/ttyACM*)." >&2
      echo "       Plug in the board, or pass one with: $0 -p /dev/ttyUSB0" >&2
      exit 1 ;;
    1)
      PORT="${PORTS[0]}" ;;
    *)
      PORT="${PORTS[0]}"
      echo ">> Multiple ports found: ${PORTS[*]}"
      echo ">> Using $PORT (override with -p if wrong)." ;;
  esac
fi
echo ">> Flashing to $PORT"

# ---- 5. Flash (+ optional monitor) ---------------------------------------
if [ "$DO_MONITOR" -eq 1 ]; then
  idf.py -p "$PORT" flash monitor
else
  idf.py -p "$PORT" flash
  echo ">> Done. Pulses should now be running on GPIO5 (OUT+) / GPIO6 (OUT-)."
fi
