#!/usr/bin/env bash
# Step 7: bare-metal application ELF
#   ./07_sw.sh                                   JTAG/console build (TX silent at boot)
#   ./07_sw.sh --defines GNSS_DEPLOY_AUTO_TX     SD-card build, TX1 starts at power-on
#   ./07_sw.sh --defines GNSS_BOOT_DIAG          adds the boot I/Q captures and band probe
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from sw --to sw "$@"
echo "---- check ----"
grep -E "^SW_RESULT:" Build/Logs/sw.log | tail -1
