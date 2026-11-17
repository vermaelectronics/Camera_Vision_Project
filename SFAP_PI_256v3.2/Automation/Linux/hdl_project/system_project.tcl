# ANTSDR E310 - Passthrough_LVDS + SFAP-PI v6.3, 256 bins (Vivado 2023.2, ADI hdl of
# antsdr-fw-patch/plutosdr-fw). Same as passthrough_lvds/system_project.tcl,
# plus the gnss_passthrough CDC constraints.
source ../../scripts/adi_env.tcl
source $ad_hdl_dir/projects/scripts/adi_project_xilinx.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl

set p_name passthrough_lvds_sfap256
if {[info exists ::env(SFAP_HDL_PRJ_NAME)]} { set p_name $::env(SFAP_HDL_PRJ_NAME) }

adi_project_create $p_name 0 {} "xc7z020clg400-2"

adi_project_files $p_name [list \
  "system_top.v" \
  "system_constr.xdc" \
  "gnss_passthrough_cdc.xdc" \
  "$ad_hdl_dir/library/common/ad_iobuf.v"]

set_property is_enabled false [get_files  *system_sys_ps7_0.xdc]
adi_project_run $p_name
source $ad_hdl_dir/library/axi_ad9361/axi_ad9361_delay.tcl
