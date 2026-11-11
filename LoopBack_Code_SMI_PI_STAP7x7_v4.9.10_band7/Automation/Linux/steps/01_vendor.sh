#!/usr/bin/env bash
# Step 1: copy the ADI HDL library to Build/VendorWork
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from vendor --to vendor
echo "---- check ----"
ls -d Build/VendorWork/hdl && echo "STEP 1 vendor: PASS"
