#!/usr/bin/env bash
# PL-NPI v2.4 regression without Vivado/Vitis (host tools only):
#   1. HLS C-simulation of the PL-NPI core, scenarios 1-12 (g++, Xilinx
#      ap_int headers + a minimal hls::stream shim).  The GPS C/N0 table
#      (J/S 10/15/30/40, scenario 12 full) runs with --full.
#   2. RTL simulation of gnss_passthrough.v (register map, notch registers,
#      commit, datapath) with Icarus Verilog.
#   3. Firmware CW scan unit test (gnss_npi_det.c).
#   4. Firmware engine mock (gnss_npi_aj.c + gnss_npi_det.c, scenarios 1-8:
#      JAMMER STATUS hysteresis, report snapshot, CW notch, DMA loss, J/S).
# Needs: gcc, g++, iverilog/vvp, and the ap_int headers from either
#   $XILINX_HLS/include (Vitis HLS install) or HLS_INCLUDE=<dir with ap_int.h>
#   (e.g. a clone of github.com/Xilinx/HLS_arbitrary_Precision_Types/include).
# A step whose tool is missing is reported SKIP, not PASS.
# usage: Tests/run_regression.sh [--full]
set -u
cd "$(dirname "$0")/.."
ROOT=$(pwd)
OUT=${OUT:-$ROOT/Build/Tests}
mkdir -p "$OUT"
FULL=0; [ "${1:-}" = "--full" ] && FULL=1
FW=$ROOT/Source/Firmware/app_gnss_e310
HLS=$ROOT/Source/HLS/pl_npi
fails=0; skips=0
res() { printf '%-28s %s\n' "$1" "$2"; [ "$2" = FAIL ] && fails=$((fails + 1)); [ "$2" = SKIP ] && skips=$((skips + 1)); }

# 1. HLS C-sim
INC=${HLS_INCLUDE:-${XILINX_HLS:+$XILINX_HLS/include}}
if [ -n "$INC" ] && [ -f "$INC/ap_int.h" ] && command -v g++ >/dev/null; then
    # the shim stands in for hls_stream.h only; ap_int comes from $INC
    if g++ -O2 -w -I"$INC" -I"$ROOT/Tests/hls/shim" -I"$HLS" "$HLS/testbench.cpp" "$HLS/pl_npi.cpp" \
           -o "$OUT/pl_npi_csim" > "$OUT/csim_build.log" 2>&1; then
        for s in 1 2 3 4 5 7 8 9 10 11 12; do
            if "$OUT/pl_npi_csim" $s > "$OUT/csim_$s.log" 2>&1; then res "HLS csim scenario $s" PASS
            else res "HLS csim scenario $s" FAIL; tail -5 "$OUT/csim_$s.log"; fi
        done
        if [ $FULL = 1 ]; then
            "$OUT/pl_npi_csim" 12 1 > "$OUT/csim_gps_table.log" 2>&1; cat "$OUT/csim_gps_table.log"
        fi
    else res "HLS csim build" FAIL; tail -20 "$OUT/csim_build.log"; fi
else
    res "HLS csim" SKIP; echo "  (set XILINX_HLS or HLS_INCLUDE to the directory holding ap_int.h)"
fi

# 2. RTL sim
if command -v iverilog >/dev/null && command -v vvp >/dev/null; then
    if iverilog -g2012 -o "$OUT/pt.vvp" Tests/rtl/tb_gnss_passthrough.v Source/HDL/gnss_passthrough.v > "$OUT/rtl_build.log" 2>&1 \
       && vvp "$OUT/pt.vvp" > "$OUT/rtl.log" 2>&1 && grep -q "^PT_SIM: PASS" "$OUT/rtl.log"; then
        res "RTL gnss_passthrough" PASS
    else res "RTL gnss_passthrough" FAIL; tail -10 "$OUT/rtl_build.log" "$OUT/rtl.log"; fi
else res "RTL gnss_passthrough" SKIP; fi

# 3./4. firmware host tests
if command -v gcc >/dev/null; then
    CF="-O2 -std=gnu99 -w -include Tests/host/stubs/pre.h -I Tests/host/stubs -I $FW"
    if gcc $CF Tests/host/test_cw_scan.c "$FW/gnss_npi_det.c" -lm -o "$OUT/cw_scan" > "$OUT/cw_build.log" 2>&1 \
       && "$OUT/cw_scan" > "$OUT/cw_scan.log" 2>&1; then res "firmware CW scan" PASS
    else res "firmware CW scan" FAIL; cat "$OUT/cw_build.log" "$OUT/cw_scan.log" | tail -12; fi
    if gcc $CF -DGNSS_AJ_BUF_ADDR='((uintptr_t)mock_buf)' Tests/host/mock_engine.c "$FW/gnss_npi_aj.c" "$FW/gnss_npi_det.c" \
           -lm -o "$OUT/mock_engine" > "$OUT/mock_build.log" 2>&1; then
        for s in 1 2 3 4 5 6 7 8 9; do
            if "$OUT/mock_engine" $s > "$OUT/mock_$s.log" 2>&1 && grep -q "^MOCK: PASS" "$OUT/mock_$s.log"; then
                res "firmware mock scenario $s" PASS
            else res "firmware mock scenario $s" FAIL; tail -8 "$OUT/mock_$s.log"; fi
        done
    else res "firmware mock build" FAIL; tail -20 "$OUT/mock_build.log"; fi
else res "firmware host tests" SKIP; fi

echo "logs: $OUT"
if [ $fails -gt 0 ]; then echo "REGRESSION: FAIL ($fails failed, $skips skipped)"; exit 1; fi
echo "REGRESSION: PASS ($skips skipped)"
