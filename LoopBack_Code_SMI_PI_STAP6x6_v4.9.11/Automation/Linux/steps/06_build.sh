#!/usr/bin/env bash
# Step 6: synthesis, implementation, bitstream, XSA (long). Want BUILD_TIMING: MET.
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from build --to build
echo "---- check ----"
grep -E "^(BUILD_TIMING|BUILD_RESULT):" Build/Logs/build.log | tail -2; ls -l Build/Output/system_top.bit Build/Output/system.xsa
