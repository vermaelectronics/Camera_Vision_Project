#!/usr/bin/env bash
# ============================================================================
#  csim_gcc.sh  --  fast C simulation of sfap_pi + the firmware solver with
#  plain g++ (no HLS project), using the ap_int / hls_stream headers of the
#  Vitis HLS install. Same testbench, same pass criteria as build_hls.tcl.
#    ./csim_gcc.sh            all 9 scenarios
#    ./csim_gcc.sh 2 6 9      only these
# ============================================================================
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/env.sh"
command -v vitis_hls >/dev/null 2>&1 || source "$XILINX_ROOT/Vitis_HLS/$XILINX_VER/settings64.sh" >/dev/null 2>&1
inc="${XILINX_HLS:-$XILINX_ROOT/Vitis_HLS/$XILINX_VER}/include"
[[ -f "$inc/ap_int.h" ]] || { echo "ap_int.h not found in $inc"; exit 2; }
S="$SFAP_ROOT/Source"; out="$SFAP_BUILD/csim_gcc"; mkdir -p "$out"
g++ -O2 -std=c++14 -w -I"$inc" -I"$S/HLS/sfap_pi" -I"$S/Firmware/app_gnss_e310" \
    "$S/HLS/sfap_pi/testbench.cpp" "$S/HLS/sfap_pi/sfap_pi.cpp" \
    -x c "$S/Firmware/app_gnss_e310/gnss_aj_math.c" -o "$out/tb" || exit 1
list=("$@"); [[ ${#list[@]} -eq 0 ]] && list=(1 2 3 4 5 6 7 8 9)
fail=0
for s in "${list[@]}"; do "$out/tb" "$s" || fail=1; echo; done
[[ $fail -eq 0 ]] && echo "CSIM_GCC: PASS" || { echo "CSIM_GCC: FAIL"; exit 1; }
