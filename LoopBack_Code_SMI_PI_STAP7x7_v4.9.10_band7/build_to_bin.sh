#!/usr/bin/env bash
# build_to_bin.sh -- SMI-PI 4.9.10 STAP 7+7 (firmware b7): everything up to Build/Output/BOOT.BIN
#
#   ./build_to_bin.sh            full build (hardware + firmware), 16 jobs
#   ./build_to_bin.sh fw         firmware only (needs Build/Output/system.xsa from a full build)
#
# Optional: XILINX_DIR=/opt/Xilinx ./build_to_bin.sh   (default /tools/Xilinx)
#           ADI_LIB=/path/to/Vendor/ADI_hdl_2023_r2 ./build_to_bin.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XILINX_DIR="${XILINX_DIR:-/tools/Xilinx}"
MODE="${1:-full}"
cd "$ROOT"

# 1. ADI HDL library (not in the zip)
if [[ ! -d Vendor/ADI_hdl_2023_r2/library ]]; then
  SRC="${ADI_LIB:-}"
  if [[ -z "$SRC" ]]; then
    SRC=$(find "$HOME" -maxdepth 4 -type d -name ADI_hdl_2023_r2 -path "*/Vendor/*" 2>/dev/null \
          | grep -v "^$ROOT/" | head -1 || true)
  fi
  if [[ -z "$SRC" || ! -d "$SRC/library" ]]; then
    echo "BUILD_TO_BIN: FAIL - ADI library (Vendor/ADI_hdl_2023_r2) not found."
    echo "              Copy it from an earlier project folder, or run: ADI_LIB=/path/to/ADI_hdl_2023_r2 $0"
    exit 1
  fi
  echo "BUILD_TO_BIN: copying ADI library from $SRC"
  mkdir -p Vendor && cp -r "$SRC" Vendor/
fi

# 2. Xilinx tools
for s in "$XILINX_DIR/Vivado/2023.2/settings64.sh" "$XILINX_DIR/Vitis/2023.2/settings64.sh"; do
  [[ -f "$s" ]] || { echo "BUILD_TO_BIN: FAIL - $s not found (set XILINX_DIR)"; exit 1; }
  # shellcheck disable=SC1090
  source "$s"
done

# 3. build
chmod +x Automation/Linux/*.sh Automation/Linux/steps/*.sh 2>/dev/null || true
LOG="build_$(date +%Y%m%d_%H%M).log"
if [[ "$MODE" == "fw" ]]; then
  ./Automation/Linux/build_all.sh --from sw --to boot 2>&1 | tee "$LOG"
else
  ./Automation/Linux/build_all.sh --jobs 16 --to boot 2>&1 | tee "$LOG"
fi

# 4. result
if [[ -f Build/Output/BOOT.BIN ]]; then
  echo "BUILD_TO_BIN: PASS - $ROOT/Build/Output/BOOT.BIN  (log: $LOG)"
  ls -l Build/Output/BOOT.BIN
else
  echo "BUILD_TO_BIN: FAIL - no BOOT.BIN; see $LOG and Build/Logs/"
  exit 1
fi
