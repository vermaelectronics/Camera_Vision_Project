#!/usr/bin/env bash
# Regression for the I-UESPRIT 2-antenna core (Icarus Verilog + Python model).
#   ./run_sim.sh            run all test cases
#   ./run_sim.sh <name>     run one case
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
IP="$(dirname "$HERE")"
OUT="${OUT:-$HERE/out}"
MODEL="$IP/model/i_uesprit_model.py"
mkdir -p "$OUT"

# name | generator options | tolerance [deg] | extra iverilog defines
CASES=(
  "cum_p4_snr10|--snr 10 --p_log2 2|0.05"
  "cum_p1_snr20|--snr 20 --p_log2 0|0.05"
  "cum_p8_snr0|--snr 0 --p_log2 3 --thetas=-50,-20,5,25,55|0.10"
  "cum_lowamp|--snr 15 --amp 0.01 --p_log2 2 --thetas=-45,-15,15,45|0.10"
  "cum_short_win|--snr 20 --log2_snapshots 8 --p_log2 2 --thetas=-30,0,30,-60,60,10,-10,40|0.10"
  "cum_min_win|--snr 30 --log2_snapshots 7 --p_log2 2 --thetas=-40,-20,0,20,40,60|0.10|-DNO_GAPS"
  "cov_p4|--use_cov --snr 10 --p_log2 2|0.05"
  "endfire|--snr 25 --p_log2 2 --thetas=-85,85,89|0.25"
)

fail=0
for c in "${CASES[@]}"; do
  IFS='|' read -r name opts tol defs <<< "$c"
  if [[ $# -gt 0 && "$1" != "$name" ]]; then continue; fi
  d="$OUT/$name"; mkdir -p "$d"
  echo "=== $name ==="
  # shellcheck disable=SC2086
  python3 -I "$MODEL" gen "$d" $opts
  # shellcheck disable=SC2086
  iverilog -g2005 -Wall -I "$d" -DSTIM_DIR="\"$d/\"" ${defs:-} -o "$d/sim.vvp" \
      "$HERE/tb_i_uesprit_2ant_core.v" "$IP"/rtl/*.v 2>&1 | grep -v "timescale" || true
  vvp -n "$d/sim.vvp" > "$d/sim.log"
  grep "TB:" "$d/sim.log" | tail -1
  if ! python3 -I "$MODEL" check "$d" --tol_deg "$tol"; then fail=1; fi
done

# IP top level (AXI4-Lite + AXI-Stream) smoke test
if [[ $# -eq 0 || "$1" == "axi" ]]; then
  d="$OUT/axi"; mkdir -p "$d"
  echo "=== axi (IP top) ==="
  python3 -I "$MODEL" gen "$d" --snr 12 --p_log2 2
  iverilog -g2005 -Wall -I "$d" -DSTIM_DIR="\"$d/\"" -o "$d/sim.vvp" \
      "$HERE/tb_i_uesprit_2ant_axi.v" "$IP"/rtl/*.v 2>&1 | grep -v "timescale" || true
  vvp -n "$d/sim.vvp" > "$d/sim.log"
  grep "AXI TB: done" "$d/sim.log"
  grep -q "AXI TB: done, .* 0 errors" "$d/sim.log" || fail=1
  if ! python3 -I "$MODEL" check "$d" --tol_deg 0.05 | tail -2; then fail=1; fi
fi
exit $fail
