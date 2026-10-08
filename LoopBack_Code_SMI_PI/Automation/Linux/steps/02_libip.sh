#!/usr/bin/env bash
# Step 2: package the ADI library IP
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from libip --to libip
echo "---- check ----"
tail -3 Build/Logs/libip.log
