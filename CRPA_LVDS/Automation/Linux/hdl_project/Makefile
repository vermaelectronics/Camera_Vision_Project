####################################################################################
## Copyright (c) 2018 - 2023 Analog Devices, Inc.
### SPDX short identifier: BSD-1-Clause
## ANTSDR E310 - Passthrough_LVDS + CRPA space-time anti-jam (crpa1, crpa27..31)
## Copied by build_all.sh into <hdl>/projects/passthrough_lvds_<algo> (next to
## passthrough_lvds) together with the sources from Source/. Vivado 2023.2.
## GNSS_CRPA_IP_REPO, CRPA_ALGO, CRPA_CORE, CRPA_CORE_VER, CRPA_HAS_GAIN must be
## set (build_all.sh does it); PROJECT_NAME is rewritten by build_all.sh.
####################################################################################
PROJECT_NAME := passthrough_lvds_crpa

M_DEPS += system_bd.tcl
M_DEPS += system_top.v
M_DEPS += system_constr.xdc
M_DEPS += gnss_passthrough_cdc.xdc
M_DEPS += ../../library/util_cdc/sync_bits.v
M_DEPS += ../../library/common/util_pulse_gen.v
M_DEPS += ../../library/common/ad_iobuf.v
M_DEPS += ../../library/common/ad_bus_mux.v
M_DEPS += ../../library/axi_tdd/scripts/axi_tdd.tcl
M_DEPS += ../../library/axi_ad9361/axi_ad9361_delay.tcl

LIB_DEPS += axi_ad9361
LIB_DEPS += axi_dmac
LIB_DEPS += axi_tdd
LIB_DEPS += util_pack/util_cpack2
LIB_DEPS += util_pack/util_upack2

include ../scripts/project-xilinx.mk
