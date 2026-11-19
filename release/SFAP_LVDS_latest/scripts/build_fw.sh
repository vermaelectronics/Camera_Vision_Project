#!/usr/bin/env bash
# build_fw.sh - firmware + BOOT.bin from hw/PASS_SFAP.xsa
# Usage (package root):  ./scripts/build_fw.sh [path-to-Vitis-2023.2]   (default /tools/Xilinx/Vitis/2023.2)
set -e
VITIS_DIR="${1:-/tools/Xilinx/Vitis/2023.2}"
test -f "$VITIS_DIR/settings64.sh" || { echo "ERROR: $VITIS_DIR/settings64.sh not found"; exit 1; }
source "$VITIS_DIR/settings64.sh"
cd "$(dirname "$0")/.."
vitis -s scripts/build_fw.py
test -f output/BOOT.bin
echo; echo "BOOT.bin ready: $(pwd)/output/BOOT.bin"
