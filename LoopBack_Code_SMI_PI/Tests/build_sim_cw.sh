#!/usr/bin/env bash
# Builds Tests/sim_cw.cpp (needs the ap_int headers: HLS_INCLUDE=<dir with ap_int.h>).
set -e
cd "$(dirname "$0")/.."
INC=${HLS_INCLUDE:-${XILINX_HLS:+$XILINX_HLS/include}}
SHIM=${HLS_SHIM:-Tests/shim}
OUT=${1:-/tmp/sim_cw}
g++ -O2 -w -I"$INC" -I"$SHIM" -ISource/HLS/smi_pi -ISource/Firmware/app_gnss_e310 \
    Tests/sim_cw.cpp Source/HLS/smi_pi/smi_pi.cpp Source/Firmware/app_gnss_e310/gnss_aj_math.c -o "$OUT"
echo "built $OUT"
