#!/usr/bin/env bash
# C simulation of the SMI-PI core + the firmware's weight solver, scenarios 1-10, with g++
# (no Vitis needed): HLS_INCLUDE=<dir that holds ap_int.h>  Tests/run_csim.sh
set -e
cd "$(dirname "$0")/.."
INC=${HLS_INCLUDE:-${XILINX_HLS:+$XILINX_HLS/include}}
[ -f "$INC/ap_int.h" ] || { echo "set HLS_INCLUDE (or XILINX_HLS) to the directory with ap_int.h"; exit 2; }
OUT=${TMPDIR:-/tmp}/smi_pi_tb
g++ -O2 -w -I"$INC" -ITests/shim -ISource/HLS/smi_pi -ISource/Firmware/app_gnss_e310 \
    Source/HLS/smi_pi/testbench.cpp Source/HLS/smi_pi/smi_pi.cpp Source/Firmware/app_gnss_e310/gnss_aj_math.c -o "$OUT"
rc=0
for i in 1 2 3 4 5 6 7 8 9 10; do "$OUT" $i | grep SMI_PI_CSIM || rc=1; done
exit $rc
