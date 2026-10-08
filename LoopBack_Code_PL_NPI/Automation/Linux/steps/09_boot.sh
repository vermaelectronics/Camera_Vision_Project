#!/usr/bin/env bash
# Step 9: BOOT.BIN = FSBL + bitstream + application
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from boot --to boot
echo "---- check ----"
ls -l Build/Output/BOOT.BIN && echo "Copy Build/Output/BOOT.BIN to the SD card."
