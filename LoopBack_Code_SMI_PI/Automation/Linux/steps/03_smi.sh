#!/usr/bin/env bash
# Step 3: Vitis HLS SMI-PI core. Want clock <= 8 ns, II 2, SMI_PI_HLS: PASS.
set -e
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_all.sh --from smi --to smi
echo "---- check ----"
grep -E "^SMI_PI_HLS:" Build/Logs/smi.log
