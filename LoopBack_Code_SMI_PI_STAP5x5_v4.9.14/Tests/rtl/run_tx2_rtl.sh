#!/usr/bin/env bash
# 4.9.14 TX2 routing test: gnss_passthrough.v with a stub for smi_pi_0, Icarus Verilog.
#   Tests/rtl/run_tx2_rtl.sh      -> prints TX2_RTL: PASS / FAIL, exit status 0 / 1
set -e
cd "$(dirname "$0")/../.."
OUT=${TMPDIR:-/tmp}/tb_tx2_route.vvp
iverilog -g2005 -Wall -o "$OUT" Tests/rtl/tb_tx2_route.v \
    Source/HDL/gnss_passthrough.v Source/HDL/gnss_pulse_blank.v
vvp -n "$OUT" | tee "${OUT%.vvp}.log"
grep -q "TX2_RTL: PASS" "${OUT%.vvp}.log"
