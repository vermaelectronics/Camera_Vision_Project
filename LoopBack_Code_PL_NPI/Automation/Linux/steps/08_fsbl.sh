#!/usr/bin/env bash
# Step 8: Zynq FSBL ELF
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from fsbl --to fsbl
echo "---- check ----"
grep -E "^FSBL_RESULT:" Build/Logs/fsbl.log | tail -1
