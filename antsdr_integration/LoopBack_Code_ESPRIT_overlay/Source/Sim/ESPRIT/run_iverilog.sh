#!/usr/bin/env bash
# Run the DOA system testbench with Icarus Verilog (no Vivado needed).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$(cd "$HERE/../.." && pwd)"
ALGO="${GNSS_DOA_ALGO:-both}"          # both | iuesprit | mesprit
case "$ALGO" in both) GEN=();; iuesprit) GEN=(-P tb_gnss_doa.EN_M=0);; mesprit) GEN=(-P tb_gnss_doa.EN_I=0);;
  *) echo "GNSS_DOA_ALGO=$ALGO - use both, iuesprit or mesprit"; exit 2;; esac
W="$SRC/../Build/sim_doa_iverilog"; [[ "$ALGO" != both ]] && W="${W}_$ALGO"
mkdir -p "$W"; echo "DOA_SIM: estimators: $ALGO"
iverilog -g2005 "${GEN[@]}" -I "$HERE" -o "$W/tb.vvp" "$HERE/tb_gnss_doa.v" "$SRC/HDL/gnss_passthrough.v" \
  "$SRC/HDL/gnss_doa_esprit.v" "$SRC/HDL/doa_async_fifo.v" "$SRC/HDL/doa_test_gen.v" "$SRC"/HDL/esprit/*.v
vvp -n "$W/tb.vvp" +STIM="$HERE/doa_stim.hex" | tee "$W/sim.log" | grep "DOA_TB"
grep -q "DOA_TB_RESULT: PASS" "$W/sim.log"
