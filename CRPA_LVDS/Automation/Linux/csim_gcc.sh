#!/usr/bin/env bash
# ============================================================================
#  csim_gcc.sh  --  fast C simulation of each CRPA algorithm's FPGA core with
#  its ARM weight solver, with plain g++ (no HLS project), using the ap_int /
#  hls_stream headers of the Vitis HLS install (or any folder holding them:
#  AP_INC=<dir>). Same testbench and pass criteria as build_hls.tcl.
#    ./csim_gcc.sh                       every algorithm, scenarios 1-6
#    ./csim_gcc.sh crpa27 crpa31         these algorithms
#    SCEN="2 6" ./csim_gcc.sh crpa1      only these scenarios
# ============================================================================
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/env.sh"
source "$here/crpa_algos.sh"
if [[ -z "${AP_INC:-}" ]]; then
  command -v vitis_hls >/dev/null 2>&1 || source "$XILINX_ROOT/Vitis_HLS/$XILINX_VER/settings64.sh" >/dev/null 2>&1
  AP_INC="${XILINX_HLS:-$XILINX_ROOT/Vitis_HLS/$XILINX_VER}/include"
fi
# no Vitis HLS: the open-source headers bundled with the package
[[ -f "$AP_INC/ap_int.h" ]] || AP_INC="$CRPA_ROOT/Vendor/hls_headers"
[[ -f "$AP_INC/ap_int.h" && -f "$AP_INC/hls_stream.h" ]] || { echo "ap_int.h / hls_stream.h not found in $AP_INC (set AP_INC)"; exit 2; }
S="$CRPA_ROOT/Source"
list=("$@"); [[ ${#list[@]} -eq 0 ]] && list=("${CRPA_ALGOS[@]}")
scen=(${SCEN:-1 2 3 4 5 6})
fail=0
for a in "${list[@]}"; do
  crpa_algo_params "$a" || { echo "unknown algorithm $a"; fail=1; continue; }
  out="$CRPA_BUILD_ROOT/$a/csim_gcc"; mkdir -p "$out"
  def=$([[ $CRPA_CORE == smi_pi ]] && echo -DCRPA_SMI || echo -DCRPA_STAP)
  echo "==== $a ($CRPA_CORE, $CRPA_NT + $CRPA_NT taps)"
  g++ -O2 -std=c++14 -w $def -I"$AP_INC" -I"$S/CRPA/$a/fpga" -I"$S/CRPA/$a/arm" \
      "$S/HLS/testbench.cpp" "$S/CRPA/$a/fpga/$CRPA_CORE.cpp" \
      -x c "$S/CRPA/$a/arm/gnss_aj_math.c" -o "$out/tb" || { fail=1; continue; }
  for s in "${scen[@]}"; do "$out/tb" "$s" | grep -v "^CRPA C simulation" || fail=1; done
done
[[ $fail -eq 0 ]] && echo "CSIM_GCC: PASS" || { echo "CSIM_GCC: FAIL"; exit 1; }
