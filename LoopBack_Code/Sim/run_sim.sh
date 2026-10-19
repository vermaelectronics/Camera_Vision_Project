#!/usr/bin/env bash
# Runs the gnss_passthrough v4.0 + LCMV HLS IP (model) system testbench with Icarus Verilog.
# Usage:  cd LoopBack_Code/Sim && ./run_sim.sh
set -e
cd "$(dirname "$0")"
iverilog -g2012 -o tb_gnss_passthrough_lcmv.vvp -s tb_gnss_passthrough_lcmv \
  ../Source/HDL/gnss_passthrough.v \
  lcmv_crpa_model.v \
  tb_gnss_passthrough_lcmv.v
vvp -n tb_gnss_passthrough_lcmv.vvp | tee sim_results.txt
rm -f tb_gnss_passthrough_lcmv.vvp
