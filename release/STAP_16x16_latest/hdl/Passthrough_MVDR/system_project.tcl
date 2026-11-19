source ../../scripts/adi_env.tcl
source $ad_hdl_dir/projects/scripts/adi_project_xilinx.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl

adi_project_create passthrough_mvdr 0 {} "xc7z020clg400-2"

adi_project_files passthrough_mvdr [list \
  "system_top.v" \
  "system_constr.xdc" \
  "gnss_mvdr_cdc.xdc" \
  "$ad_hdl_dir/library/common/ad_iobuf.v"]

set_property is_enabled false [get_files  *system_sys_ps7_0.xdc]

# extra effort for the 4.0 ns rx_clk paths (adds post-route physical optimisation)
set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
adi_project_run passthrough_mvdr
source $ad_hdl_dir/library/axi_ad9361/axi_ad9361_delay.tcl
