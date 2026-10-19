#!/usr/bin/env bash
# Step 2: package the ADI library IP
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from libip --to libip
echo "---- check ----"
n=$(ls Build/Logs/libip_*.log 2>/dev/null | wc -l); echo "STEP 2 libip: PASS ($n library logs in Build/Logs/libip_*.log)"
