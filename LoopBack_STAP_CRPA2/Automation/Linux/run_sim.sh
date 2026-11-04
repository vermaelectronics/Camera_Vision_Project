#!/usr/bin/env bash
# ============================================================================
#  run_sim.sh -- RTL simulation of the CRPA (gnss_stap) + loopback datapath.
#  No Xilinx tools needed: gcc, python3 + numpy, iverilog.
#
#    1. builds the host copy of the firmware weight solver (STAP_M = 2)
#    2. stap_golden.py: E310 scenario (2 elements, 30.72 MSPS complex baseband,
#       12-bit AD9361 samples, 1 wideband jammer, power inversion), bit-true
#       model, test vectors, performance report  -> Build/Sim/report.json
#    3. Source/Sim/tb_gnss_stap.v with Icarus Verilog
#  Extra arguments go to stap_golden.py, e.g.  --jam-db 60 --mode mvdr
# ============================================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
B="$ROOT/Build/Sim"
FW="$ROOT/Source/Firmware/app_gnss_e310"
mkdir -p "$B"

echo "RUN_SIM: building solver (host)"
gcc -O2 -std=c99 -Wall -Wextra -I "$FW" -o "$B/stap_solve_cli" \
    "$ROOT/Source/Sim/model/stap_solve_cli.c" "$FW/stap_solver.c" -lm

echo "RUN_SIM: golden model"
python3 "$ROOT/Source/Sim/model/stap_golden.py" --out "$B" --solver "$B/stap_solve_cli" \
    --M 2 --fs 30.72e6 --fif 0 --jam-theta 30 --adc-bits 12 --adc-align right \
    --ow 12 --mode pi --K 2000 "$@"

echo "RUN_SIM: iverilog"
iverilog -g2005 -Wall -I "$B" -o "$B/tb_gnss_stap.vvp" \
    "$ROOT/Source/Sim/tb_gnss_stap.v" "$ROOT/Source/HDL/gnss_stap.v" \
    "$ROOT/Source/HDL/gnss_passthrough.v" "$ROOT/Source/HDL/stap/"*.v 2>&1 | grep -v timescale || true
(cd "$B" && vvp -n tb_gnss_stap.vvp) | tee "$B/sim.log"
if grep -q "\*\*\* PASS" "$B/sim.log"; then echo "RUN_SIM: PASS"; else echo "RUN_SIM: FAIL"; exit 1; fi
