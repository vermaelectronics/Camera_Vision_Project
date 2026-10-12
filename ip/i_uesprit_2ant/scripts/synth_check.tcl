# =============================================================================
# synth_check.tcl -- out-of-context synthesis + timing/utilisation report
#
#   vivado -mode batch -source scripts/synth_check.tcl [-tclargs <part> <period_ns>]
# =============================================================================
set ip_root [file normalize [file join [file dirname [info script]] ..]]
set part    [expr {[llength $argv] > 0 ? [lindex $argv 0] : "xczu4ev-sfvc784-1-i"}]
set period  [expr {[llength $argv] > 1 ? [lindex $argv 1] : 10.0}]
set out     [file join $ip_root build synth]
file mkdir $out

read_verilog [glob [file join $ip_root rtl *.v]]
synth_design -top i_uesprit_2ant_axi -part $part -mode out_of_context -retiming
create_clock -name aclk -period $period [get_ports aclk]
opt_design
place_design
phys_opt_design
route_design
report_utilization    -file [file join $out utilization.rpt]
report_timing_summary -file [file join $out timing.rpt] -max_paths 20
report_utilization -hierarchical -file [file join $out utilization_hier.rpt]
puts "INFO: reports in $out"
