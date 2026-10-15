#!/usr/bin/env bash
# Runs the gnss_passthrough v2.0 (LCMV CRPA) system testbench with Icarus Verilog.
# Usage:  cd LoopBack_Code/Sim && ./run_sim.sh
set -e
cd "$(dirname "$0")"
iverilog -g2012 -o tb_gnss_passthrough_lcmv.vvp -s tb_gnss_passthrough_lcmv \
  ../Source/HDL/gnss_passthrough.v \
  ../Source/HDL/crpa/crpa_lcmv_2ch.v \
  ../Source/HDL/crpa/crpa_lcmv_weight_2ch.v \
  ../Source/HDL/crpa/crpa_sdiv_serial.v \
  ../Source/HDL/crpa/crpa_beamformer_2ch.v \
  ../Source/HDL/crpa/steering_vector_2x2.v \
  tb_gnss_passthrough_lcmv.v
vvp -n tb_gnss_passthrough_lcmv.vvp | tee sim_results.txt
rm -f tb_gnss_passthrough_lcmv.vvp
