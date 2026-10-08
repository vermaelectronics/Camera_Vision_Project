#!/usr/bin/env bash
# Step 5: create the Vivado project
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from project --to project
echo "---- check ----"
grep -m1 "CREATE_RESULT" Build/Logs/project.log
