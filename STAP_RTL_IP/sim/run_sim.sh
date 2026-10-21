#!/usr/bin/env bash
# Run the STAP IP regression with Icarus Verilog.
#   ./sim/run_sim.sh [small|pi|full|fp|all]   (default: small)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RTL="$ROOT/rtl"
CFG="${1:-small}"

RTL_SRC="$RTL/stap_fp_add.v $RTL/stap_fp_mul.v $RTL/stap_fp_recip.v $RTL/stap_int2fp.v \
 $RTL/stap_fp2fix.v $RTL/stap_cdot_engine.v $RTL/stap_solver.v $RTL/stap_cov_est.v \
 $RTL/stap_sdp_ram.v $RTL/stap_beamformer.v $RTL/stap_wload.v $RTL/stap_top.v"

run_fp() {
  local W="$ROOT/sim/work_fp"; mkdir -p "$W"
  python3 -I "$ROOT/model/check_fp_units.py" "$W" --gen
  iverilog -g2005 -o "$W/tb.vvp" "$ROOT/tb/tb_fp_units.v" $RTL_SRC
  (cd "$W" && vvp -n tb.vvp > sim.log)
  python3 -I "$ROOT/model/check_fp_units.py" "$W"
}

run_cfg() {
  local c="$1" M N L
  case "$c" in
    small) M=4; N=4;  L=10 ;;
    pi)    M=4; N=8;  L=10 ;;
    full)  M=7; N=16; L=12 ;;
    *) echo "unknown config $c"; exit 2 ;;
  esac
  local W="$ROOT/sim/work_$c"; mkdir -p "$W"
  echo "=== config $c : M=$M N=$N LOG2L=$L"
  python3 -I "$ROOT/model/stap_model.py" gen --cfg "$c" --out "$W"
  iverilog -g2005 -o "$W/tb.vvp" -Ptb_stap_top.M=$M -Ptb_stap_top.N=$N \
      -Ptb_stap_top.LOG2L=$L "$ROOT/tb/tb_stap_top.v" $RTL_SRC
  (cd "$W" && vvp -n tb.vvp | tee sim.log)
  python3 -I "$ROOT/model/stap_model.py" check --cfg "$c" --out "$W" ${PLOTS:+--plots "$PLOTS"}
}

case "$CFG" in
  fp)  run_fp ;;
  all) run_fp; for c in small pi full; do run_cfg "$c"; done ;;
  *)   run_cfg "$CFG" ;;
esac
