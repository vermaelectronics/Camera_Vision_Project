#!/usr/bin/env bash
# Firmware/RTL co-simulation (Verilator). Linux host, verilator >= 4.2, g++.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$(cd "$HERE/../../.." && pwd)"
FW="$SRC/Firmware/app_gnss_e310"
ALGO="${GNSS_DOA_ALGO:-both}"          # both | iuesprit | mesprit
IMPL="${GNSS_DOA_IMPL:-rtl}"           # rtl | hls (HLS needs iuesprit or mesprit)
case "$ALGO" in both) GEN=();; iuesprit) GEN=(-GENABLE_MESPRIT=0);; mesprit) GEN=(-GENABLE_IUESPRIT=0);;
  *) echo "GNSS_DOA_ALGO=$ALGO - use both, iuesprit or mesprit"; exit 2;; esac
HLSF=(); HLSC=""
if [[ "$IMPL" == hls ]]; then
  [[ "$ALGO" == both ]] && { echo "GNSS_DOA_IMPL=hls needs GNSS_DOA_ALGO=iuesprit or mesprit"; exit 2; }
  # ap_int.h / hls_stream.h: the Vitis HLS include directory
  HLS_INC="${HLS_INCLUDE:-${XILINX_HLS:-}/include}"
  [[ -f "$HLS_INC/ap_int.h" ]] || { echo "set XILINX_HLS (source Vitis settings64.sh) or HLS_INCLUDE"; exit 2; }
  HD="$SRC/HLS/esprit_hls"
  if [[ "$ALGO" == iuesprit ]]; then HM=0; else HM=1; fi
  GEN+=(-GHLS_CORE=1)
  HLSC="-DHLS_MODEL=$HM -I$HD -I$HLS_INC ${HLS_EXTRA_INC:+-I$HLS_EXTRA_INC}"
  HLSF=("$HD/esprit_iu_hls.cpp" "$HD/esprit_m_hls.cpp")
fi
W="$SRC/../Build/fw_cosim"; [[ "$ALGO" != both ]] && W="${W}_$ALGO"; [[ "$IMPL" == hls ]] && W="${W}_hls"
mkdir -p "$W/stub"; echo "FW_COSIM: estimators: $ALGO ($IMPL)"
touch "$W/stub/xparameters.h"
# the firmware is C: compile it with gcc so it keeps C linkage
gcc -c -O2 -std=gnu99 -w -I"$FW" -I"$W/stub" "$FW/gnss_doa.c" -o "$W/gnss_doa.o"
verilator --cc --exe --build -O2 -j 4 --top-module gnss_passthrough "${GEN[@]}" \
  -Wno-fatal -Wno-lint -Wno-style -Wno-TIMESCALEMOD \
  -CFLAGS "-I$FW -I$W/stub -std=gnu++14 -w $HLSC" --Mdir "$W/obj" \
  "$SRC/HDL/gnss_passthrough.v" "$SRC/HDL/gnss_doa_esprit.v" "$SRC/HDL/doa_async_fifo.v" \
  "$SRC/HDL/doa_test_gen.v" "$SRC"/HDL/esprit/*.v \
  -LDFLAGS "$W/gnss_doa.o -lm" "$HERE/fw_cosim.cpp" "${HLSF[@]}" > "$W/build.log" 2>&1 || { tail -30 "$W/build.log"; exit 1; }
"$W/obj/Vgnss_passthrough" | tee "$W/fw_cosim.log"
grep -q "FW_COSIM_RESULT: PASS" "$W/fw_cosim.log"
