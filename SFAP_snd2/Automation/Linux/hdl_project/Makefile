####################################################################################
## SFAP-PI E310 V1 project (copied into $ADI_HDL_DIR/projects/gnss_sfap/e310)
## `make` builds the ADI library IPs below, then the Vivado project.
####################################################################################

PROJECT_NAME := gnss_sfap_e310

M_DEPS += system_bd.tcl
M_DEPS += system_top.v
M_DEPS += system.xdc
M_DEPS += gnss_passthrough_cdc.xdc
M_DEPS += ../../scripts/adi_pd.tcl
M_DEPS += ../../../library/common/ad_iobuf.v

LIB_DEPS += axi_ad9361
LIB_DEPS += axi_dmac
LIB_DEPS += axi_gpreg
LIB_DEPS += axi_sysid
LIB_DEPS += sysid_rom
LIB_DEPS += util_pack/util_cpack2
LIB_DEPS += util_pack/util_upack2
LIB_DEPS += util_rfifo
LIB_DEPS += util_tdd_sync
LIB_DEPS += util_wfifo
LIB_DEPS += xilinx/util_clkdiv

include ../../scripts/project-xilinx.mk
