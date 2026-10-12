#!/usr/bin/env bash
# Firmware/RTL co-simulation (Verilator). Linux host, verilator >= 4.2, g++.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$(cd "$HERE/../../.." && pwd)"
FW="$SRC/Firmware/app_gnss_e310"
W="$SRC/../Build/fw_cosim"; mkdir -p "$W/stub"
touch "$W/stub/xparameters.h"
# the firmware is C: compile it with gcc so it keeps C linkage
gcc -c -O2 -std=gnu99 -w -I"$FW" -I"$W/stub" "$FW/gnss_doa.c" -o "$W/gnss_doa.o"
verilator --cc --exe --build -O2 -j 4 --top-module gnss_passthrough \
  -Wno-fatal -Wno-lint -Wno-style -Wno-TIMESCALEMOD \
  -CFLAGS "-I$FW -I$W/stub -std=gnu++14 -w" --Mdir "$W/obj" \
  "$SRC/HDL/gnss_passthrough.v" "$SRC/HDL/gnss_doa_esprit.v" "$SRC/HDL/doa_async_fifo.v" \
  "$SRC/HDL/doa_test_gen.v" "$SRC"/HDL/esprit/*.v \
  -LDFLAGS "$W/gnss_doa.o -lm" "$HERE/fw_cosim.cpp" > "$W/build.log" 2>&1 || { tail -30 "$W/build.log"; exit 1; }
"$W/obj/Vgnss_passthrough" | tee "$W/fw_cosim.log"
grep -q "FW_COSIM_RESULT: PASS" "$W/fw_cosim.log"
