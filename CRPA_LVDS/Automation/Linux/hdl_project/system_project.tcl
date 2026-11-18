# ANTSDR E310 - Passthrough_LVDS + CRPA space-time anti-jam (Vivado 2023.2, ADI
# hdl of antsdr-fw-patch/plutosdr-fw). Same as passthrough_lvds/system_project.tcl,
# plus the gnss_passthrough CDC constraints. One project per CRPA algorithm:
# projects/passthrough_lvds_<algo> (CRPA_HDL_PRJ_NAME, set by build_all.sh).
source ../../scripts/adi_env.tcl
source $ad_hdl_dir/projects/scripts/adi_project_xilinx.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl

set p_name passthrough_lvds_crpa
if {[info exists ::env(CRPA_HDL_PRJ_NAME)]} { set p_name $::env(CRPA_HDL_PRJ_NAME) }

adi_project_create $p_name 0 {} "xc7z020clg400-2"

adi_project_files $p_name [list \
  "system_top.v" \
  "system_constr.xdc" \
  "gnss_passthrough_cdc.xdc" \
  "$ad_hdl_dir/library/common/ad_iobuf.v"]

set_property is_enabled false [get_files  *system_sys_ps7_0.xdc]
# PS DDR pins: report DRC IOSTDTYPE-1 as a warning (see drc_severity.tcl). A
# constraint file can't do this (get_drc_checks is not an XDC command), so
# the implementation run sources the script before opt_design, in the same
# process that later runs the DRCs.
set_property STEPS.OPT_DESIGN.TCL.PRE [file normalize [file join [pwd] drc_severity.tcl]] [get_runs impl_1]
# Timing closure help (all builds): physical optimisation after placement with
# a stronger directive, and once more after routing. Functionally neutral;
# recovers the last ~0.1-0.3 ns on near-critical paths (the stap_pi builds
# missed by 0.1 ns without it). Each setting is skipped if this Vivado lacks it.
foreach {crpa_prop crpa_val} {
  STEPS.PHYS_OPT_DESIGN.IS_ENABLED            true
  STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE        AggressiveExplore
  STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true
  STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore
} {
  if {[catch {set_property $crpa_prop $crpa_val [get_runs impl_1]} crpa_err]} {
    puts "CRPA_LVDS: impl_1 $crpa_prop not set: $crpa_err"
  } else {
    puts "CRPA_LVDS: impl_1 $crpa_prop = $crpa_val"
  }
}
adi_project_run $p_name
source $ad_hdl_dir/library/axi_ad9361/axi_ad9361_delay.tcl
