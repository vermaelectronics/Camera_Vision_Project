#!/usr/bin/env bash
# Step 4: package the gnss_passthrough IP
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from ip --to ip
echo "---- check ----"
grep -E "^IP_PACKAGE_OK" Build/Logs/ip.log | tail -1
