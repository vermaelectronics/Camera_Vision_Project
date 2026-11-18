####################################################################################
## Copyright (c) 2018 - 2023 Analog Devices, Inc.
### SPDX short identifier: BSD-1-Clause
## ANTSDR E310 - LVDS passthrough + MVDR / SMI-STAP anti-jam (based on antsdr-fw-patch "ant")
####################################################################################
#vivado2023.2
PROJECT_NAME := passthrough_mvdr

M_DEPS += rtl/beamformer_2ch.v
M_DEPS += rtl/cmult.v
M_DEPS += rtl/cov2x2_accum.v
M_DEPS += rtl/cov_iir.v
M_DEPS += rtl/cov_prep.v
M_DEPS += rtl/gnss_passthrough.v
M_DEPS += rtl/gnss_passthrough_bd.v
M_DEPS += rtl/iq_corr.v
M_DEPS += rtl/iq_est.v
M_DEPS += rtl/jammer_est.v
M_DEPS += rtl/mvdr_ctrl_fsm.v
M_DEPS += rtl/mvdr_regs.v
M_DEPS += rtl/mvdr_top.v
M_DEPS += rtl/mvdr_weight_calc.v
M_DEPS += rtl/mvdr_xdom.v
M_DEPS += rtl/output_scaler.v
M_DEPS += rtl/real_divider.v
M_DEPS += rtl/stap_lms.v
M_DEPS += rtl/stap_smi.v
M_DEPS += ../../library/util_cdc/sync_bits.v
M_DEPS += ../../library/common/util_pulse_gen.v
M_DEPS += ../../library/common/ad_iobuf.v
M_DEPS += ../../library/common/ad_bus_mux.v
M_DEPS += ../../library/axi_ad9361/axi_ad9361_delay.tcl

LIB_DEPS += axi_ad9361
LIB_DEPS += axi_dmac
LIB_DEPS += util_pack/util_cpack2
LIB_DEPS += util_pack/util_upack2

include ../scripts/project-xilinx.mk
