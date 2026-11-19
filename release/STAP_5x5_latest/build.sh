#!/usr/bin/env bash
# PASS_MVDR_STAP: build firmware + BOOT.bin from a Linux terminal.
# Usage:  ./build.sh [path-to-Vitis-2023.2]      (default /tools/Xilinx/Vitis/2023.2)
set -e
VITIS_DIR="${1:-/tools/Xilinx/Vitis/2023.2}"
if [ ! -f "$VITIS_DIR/settings64.sh" ]; then
  echo "ERROR: $VITIS_DIR/settings64.sh not found. Pass your Vitis 2023.2 folder: ./build.sh /opt/Xilinx/Vitis/2023.2"
  exit 1
fi
source "$VITIS_DIR/settings64.sh"
cd "$(dirname "$0")"
vitis -s scripts/build_fw.py
test -f output/BOOT.bin
echo
echo "BOOT.bin ready: $(pwd)/output/BOOT.bin"
