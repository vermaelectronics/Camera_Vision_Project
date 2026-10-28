#!/usr/bin/env bash
# Step 3: Vitis HLS PL-NPI v2.5 core (11 C-sim scenarios incl. 13, IP 2.2; fallbacks PL_NPI_NOTCHES=1 | PL_NPI_NOTCH_IN=0 | PL_NPI_V25=0). Want clock <= 10 ns,
# II 1, PL_NPI_HLS: PASS.
# If it fails, send the PL_NPI_HLS lines and the critical path:
#   grep -A14 "HLS 200-1016" Build/Logs/npi.log
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from npi --to npi
echo "---- check ----"
grep -E "^PL_NPI_CSIM:|^PL_NPI_HLS: (estimated|IP|PASS|FAIL)" Build/Logs/npi.log
