#!/usr/bin/env bash
# Step 3: Vitis HLS PL-NPI core. Want clock <= 8 ns, II 2, PL_NPI_HLS: PASS.
# If the clock is above 8 ns: PL_NPI_EXTRA_DELAY=6 PL_NPI_STEP_SHIFT=2 ./03_npi.sh
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from npi --to npi
echo "---- check ----"
grep PL_NPI_HLS Build/Logs/npi.log
