#!/usr/bin/env bash
# Step 7: bare-metal application ELF
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from sw --to sw
echo "---- check ----"
grep -E "^SW_RESULT:" Build/Logs/sw.log | tail -1
