#!/usr/bin/env bash
# Resource estimate with Yosys for a Xilinx 7-series / UltraScale-class target.
# usage: scripts/synth_yosys.sh [FOLD] [family]     (family: xc7 | xcu | xcup)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FOLD=${1:-4}; FAM=${2:-xc7}
OUT="$ROOT/build/synth_fold${FOLD}_${FAM}"
mkdir -p "$OUT"
yosys -q -l "$OUT/yosys.log" -p "
  read_verilog $ROOT/rtl/stap_fir.v $ROOT/rtl/stap_corr.v $ROOT/rtl/stap_top.v
  chparam -set FOLD $FOLD stap_top
  synth_xilinx -family $FAM -top stap_top
  tee -o $OUT/utilization.txt stat
"
grep -E "Number of cells|DSP48|RAMB|LUT[0-9]|FD[A-Z]+ |CARRY|SRL" "$OUT/utilization.txt" | sed 's/^ */  /'
