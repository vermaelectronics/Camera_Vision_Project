#!/usr/bin/env bash
# Run the DOA system testbench with Icarus Verilog (no Vivado needed).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$(cd "$HERE/../.." && pwd)"
W="$SRC/../Build/sim_doa_iverilog"; mkdir -p "$W"
iverilog -g2005 -I "$HERE" -o "$W/tb.vvp" "$HERE/tb_gnss_doa.v" "$SRC/HDL/gnss_passthrough.v" \
  "$SRC/HDL/gnss_doa_esprit.v" "$SRC/HDL/doa_async_fifo.v" "$SRC/HDL/doa_test_gen.v" "$SRC"/HDL/esprit/*.v
vvp -n "$W/tb.vvp" +STIM="$HERE/doa_stim.hex" | tee "$W/sim.log" | grep "DOA_TB"
grep -q "DOA_TB_RESULT: PASS" "$W/sim.log"
