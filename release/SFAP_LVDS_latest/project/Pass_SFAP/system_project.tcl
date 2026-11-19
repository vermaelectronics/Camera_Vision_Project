# system_project.tcl - ANTSDR E310 LVDS + SFAP-PI v6.3 (256 bins), Vivado 2023.2
#
# Same ADI project flow as Pass_MVDR_STAP (adi_project_create / adi_project_run),
# with the SFAP block design (system_bd.tcl, from Source/BlockDesign) and:
#   * power_opt_design ON for impl_1 (clock-enable gating, logic unchanged)
#   * ps7_ddr_drc.xdc: waiver for DRC IOSTDTYPE-1 on the PS7 DDR pins
#   * strategy Performance_ExplorePostRoutePhysOpt (as Pass_MVDR_STAP)
# finish_hw.tcl then waives the methodology warnings and writes PASS_SFAP.xsa.
#
# This folder must sit at <antsdr-fw-patch>/plutosdr-fw/hdl/projects/Pass_SFAP.
# Run:  vivado -mode batch -source system_project.tcl
# (scripts/build_hw.sh does the copy and both steps.)

source ../../scripts/adi_env.tcl
source $ad_hdl_dir/projects/scripts/adi_project_xilinx.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl

# custom IP (gnss_passthrough 3.0 + sfap_pi 5.0), see scripts/build_ip.sh
if {![info exists ::env(GNSS_CRPA_IP_REPO)]} {
  set ::env(GNSS_CRPA_IP_REPO) [file normalize ip_repo]
}
puts "SFAP_LVDS: custom IP repository $::env(GNSS_CRPA_IP_REPO)"

adi_project_create passthrough_sfap 0 {} "xc7z020clg400-2"

adi_project_files passthrough_sfap [list \
  "system_top.v" \
  "system_constr.xdc" \
  "gnss_passthrough_cdc.xdc" \
  "ps7_ddr_drc.xdc" \
  "$ad_hdl_dir/library/common/ad_iobuf.v"]

set_property is_enabled false [get_files -quiet *system_sys_ps7_0.xdc]
set_property USED_IN_SYNTHESIS false [get_files *ps7_ddr_drc.xdc]

set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
set_property STEPS.POWER_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]

adi_project_run passthrough_sfap
