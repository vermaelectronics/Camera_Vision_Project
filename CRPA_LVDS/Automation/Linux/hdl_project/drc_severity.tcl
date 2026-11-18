# ============================================================================
#  drc_severity.tcl  --  sourced by the implementation run before opt_design
#  (STEPS.OPT_DESIGN.TCL.PRE, set in system_project.tcl).
#
#  PS DDR3 pins (ddr_ck_p/n, ddr_dqs_p/n[3:0]): the ADI system_top.v declares
#  them as single-ended inout ports while the Zynq PS fixes them as differential
#  (DIFF_SSTL15 / DIFF_SSTL15_T_DCI), so the DRC reports 10 x IOSTDTYPE-1
#  critical warnings. They are hard-wired to the PS DDR controller and set up by
#  FSBL / ps7_init, not by the bitstream: harmless. Reported as warnings.
# ============================================================================
set_property SEVERITY {Warning} [get_drc_checks IOSTDTYPE-1]
puts "CRPA_LVDS: DRC IOSTDTYPE-1 (PS DDR pins) reported as a warning"
