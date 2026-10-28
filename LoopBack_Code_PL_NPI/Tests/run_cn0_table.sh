#!/usr/bin/env bash
# C/N0 of one GPS satellite through the real fixed-point PL-NPI core (C model), v2.4 against v2.5:
# CW, wideband and CW + wideband jammers at J/N 10/20/30 dB, RX2 cable mismatch 0.15 / 0.5 / 1.0
# sample. Simulation only. Needs g++ and the ap_int headers ($XILINX_HLS/include or HLS_INCLUDE).
# usage: Tests/run_cn0_table.sh      (about 3 minutes on 4 cores)
set -u
cd "$(dirname "$0")/.."
ROOT=$(pwd); HLS=$ROOT/Source/HLS/pl_npi
INC=${HLS_INCLUDE:-${XILINX_HLS:+$XILINX_HLS/include}}
OUT=${OUT:-$ROOT/Build/Tests}; mkdir -p "$OUT"
if [ -z "$INC" ] || [ ! -f "$INC/ap_int.h" ]; then echo "set XILINX_HLS or HLS_INCLUDE (directory with ap_int.h)"; exit 2; fi
I="-I$INC -I$ROOT/Tests/hls/shim -I$HLS"
g++ -O2 -w $I "$ROOT/Tests/hls/cn0_array_sim.cpp" "$HLS/pl_npi.cpp" -o "$OUT/cn0_v25" &
g++ -O2 -w -DPL_NPI_V25=0 $I "$ROOT/Tests/hls/cn0_array_sim.cpp" "$HLS/pl_npi.cpp" -o "$OUT/cn0_v24" &
wait
echo "PRN1 C/N0 [dB-Hz] through the core C model; no jammer: $("$OUT/cn0_v25" -99 0 0 0)"
echo "raw = RX1 alone; v2.4 = previous core (notch on the output); v2.5 = notch in front of the loop, + RX2 equaliser"
for kind in 0 1 2; do
  names=("CW +2 kHz" "wideband 40 MHz" "CW + wideband (each -3 dB)")
  for tau in 0.15 0.5 1.0; do
    if [ $kind = 0 ] && [ "$tau" != 0.15 ]; then continue; fi
    echo; echo "== ${names[$kind]}$([ $kind != 0 ] && echo ", RX2 path difference $tau sample")"
    printf "%-5s %-7s %-8s %-11s %-9s\n" "J/N" "raw" "v2.4" "v2.5 notch" "v2.5 +eq"
    for jn in 10 20 30; do
      js=$((jn+32))
      "$OUT/cn0_v25" $js $kind 0 0 $tau > "$OUT/c1" &
      "$OUT/cn0_v24" $js $kind 2 1 $tau > "$OUT/c2" &
      "$OUT/cn0_v25" $js $kind 2 1 $tau > "$OUT/c3" &
      "$OUT/cn0_v25" $js $kind 2 3 $tau > "$OUT/c4" &
      wait
      printf "%-5s %-7s %-8s %-11s %-9s\n" $jn "$(cat "$OUT/c1")" "$(cat "$OUT/c2")" "$(cat "$OUT/c3")" "$(cat "$OUT/c4")"
    done
  done
done
