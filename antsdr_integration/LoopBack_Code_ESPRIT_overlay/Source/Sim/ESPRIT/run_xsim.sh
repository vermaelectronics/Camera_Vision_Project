#!/usr/bin/env bash
# Run the DOA system testbench with Vivado xsim from a shell.
#   source <Vivado>/settings64.sh ; ./run_xsim.sh        (add "gui" for waveforms)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$(cd "$HERE/../.." && pwd)"
W="$SRC/../Build/sim_doa"; mkdir -p "$W"; cd "$W"
xvlog -i "$HERE" "$HERE/tb_gnss_doa.v" "$SRC/HDL/gnss_passthrough.v" "$SRC/HDL/gnss_doa_esprit.v" \
      "$SRC/HDL/doa_async_fifo.v" "$SRC/HDL/doa_test_gen.v" "$SRC"/HDL/esprit/*.v
xelab -debug typical -s tb_doa tb_gnss_doa
if [[ "${1:-}" == "gui" ]]; then
  xsim tb_doa -gui -testplusarg STIM="$HERE/doa_stim.hex" -tclbatch "$HERE/waves_doa.tcl"
else
  xsim tb_doa -R -testplusarg STIM="$HERE/doa_stim.hex" | tee xsim_run.out | grep "DOA_TB"
  grep -q "DOA_TB_RESULT: PASS" xsim_run.out
fi
