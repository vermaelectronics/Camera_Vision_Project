#!/usr/bin/env bash
# Step 7: bare-metal application ELF
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from sw --to sw
echo "---- check ----"
grep -m1 "SW_RESULT" Build/Logs/sw.log
