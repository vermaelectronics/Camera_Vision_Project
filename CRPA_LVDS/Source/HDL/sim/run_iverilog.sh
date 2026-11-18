#!/usr/bin/env bash
# v7.0: register map / commit test of gnss_passthrough.v for each CRPA
# algorithm (Icarus Verilog 11+). Prints one HDL_TB line per algorithm.
#   ./run_iverilog.sh               all algorithms (crpa1 crpa27 .. crpa31)
#   ./run_iverilog.sh crpa27 crpa31 only these
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../../../Automation/Linux/crpa_algos.sh"
list=("$@"); [[ ${#list[@]} -eq 0 ]] && list=("${CRPA_ALGOS[@]}")
tmp="${TMPDIR:-/tmp}/tb_gnss_pt_$$"; mkdir -p "$tmp"
fail=0
for a in "${list[@]}"; do
  crpa_algo_params "$a" || { echo "HDL_TB: unknown algorithm $a"; fail=1; continue; }
  iverilog -g2005 -DALGO="$CRPA_NUM" -DNT="$CRPA_NT" -DREF="$CRPA_REF" -DNWTS="$CRPA_NWTS" \
           -DNW_USED="$CRPA_NW_USED" -DNWCJ="$CRPA_NWCJ" \
           -o "$tmp/tb" "$here/tb_gnss_passthrough_v7.v" "$here/../gnss_passthrough.v" || { fail=1; continue; }
  vvp -n "$tmp/tb" | grep -E "HDL_TB|FAIL" || fail=1
  vvp -n "$tmp/tb" | grep -q "HDL_TB: PASS" || fail=1
done
# the RX -> TX datapath itself (loop-back): matched rates, and TX 4x too fast
for dd in 4 1; do
  iverilog -g2005 -DDAC_DIV=$dd -o "$tmp/dp" "$here/tb_loopback_datapath.v" "$here/../gnss_passthrough.v" || { fail=1; continue; }
  vvp -n "$tmp/dp" | grep -E "^DATAPATH" || fail=1
  vvp -n "$tmp/dp" | grep -q "DATAPATH_TB: PASS" || fail=1
done
rm -rf "$tmp"
[[ $fail -eq 0 ]] && echo "HDL_TB: ALL PASS" || { echo "HDL_TB: FAIL"; exit 1; }
