#!/usr/bin/env bash
# Step 3: Vitis HLS PL-NPI v2.3 core (8 C-sim scenarios). Want clock <= 10 ns,
# II 1, PL_NPI_HLS: PASS.
# If it fails, send the PL_NPI_HLS lines and the critical path:
#   grep -A14 "HLS 200-1016" Build/Logs/npi.log
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from npi --to npi
echo "---- check ----"
grep -E "^PL_NPI_CSIM:|^PL_NPI_HLS: (estimated|IP|PASS|FAIL)" Build/Logs/npi.log
