# fix_timing.tcl - when finish_hw.tcl reports TIMING NOT MET (small negative WNS):
# runs the SAME synthesised design through 4 other implementation strategies
# IN PARALLEL, keeps the best run that meets timing, writes its reports and
# PASS_SFAP.xsa. No RTL, block design, clock or firmware change: the waivers
# and the DDR DRC waiver (constrs_1) and power_opt_design apply to every run.
#
# Usage (project folder, Vivado 2023.2 loaded), ~1-1.5 h with 24 cores:
#     vivado -mode batch -source fix_timing.tcl -log fix_timing.log -nojournal

open_project passthrough_sfap.xpr
set flow [get_property FLOW [get_runs impl_1]]

#        run      strategy                              extra
set cands {
  impl_2  Performance_ExtraTimingOpt            {}
  impl_3  Performance_NetDelay_high             {}
  impl_4  Performance_ExplorePostRoutePhysOpt   aggressive
  impl_5  Performance_Retiming                  {}
}
set runs {}
foreach {r strat extra} $cands {
  if {[llength [get_runs -quiet $r]]} { delete_runs $r }
  create_run $r -parent_run synth_1 -flow $flow -strategy $strat
  set_property STEPS.POWER_OPT_DESIGN.IS_ENABLED true [get_runs $r]
  if {$extra eq "aggressive"} {
    set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs $r]
    set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs $r]
    set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs $r]
  }
  lappend runs $r
}
launch_runs $runs -to_step write_bitstream -jobs 16
foreach r $runs { wait_on_run $r }

set best ""
set best_wns -999.0
puts "RESULTS (run, strategy, WNS, WHS):"
foreach {r strat extra} $cands {
  set ok [expr {[get_property PROGRESS [get_runs $r]] == "100%"}]
  set wns [get_property STATS.WNS [get_runs $r]]
  set whs [get_property STATS.WHS [get_runs $r]]
  puts "  $r  $strat $extra  WNS=$wns  WHS=$whs  [expr {$ok ? {done} : {FAILED}}]"
  if {$ok && $wns ne "" && $whs ne "" && $wns >= 0 && $whs >= 0 && $wns > $best_wns} {
    set best $r; set best_wns $wns
  }
}

if {$best eq ""} {
  puts "*** NO RUN MET TIMING - XSA NOT written. Send fix_timing.log and reports_<run>/1_timing_summary.txt"
  foreach r $runs {
    open_run $r -name tmp_$r
    file mkdir reports_$r
    report_timing_summary -max_paths 20 -file reports_$r/1_timing_summary.txt
    close_design
  }
  close_project
  return
}

puts "BEST: $best (WNS $best_wns ns)"
current_run [get_runs $best]
open_run $best
file mkdir reports
report_timing_summary -max_paths 20 -file reports/1_timing_summary.txt
report_utilization    -file reports/2_utilization.txt
report_utilization -hierarchical -file reports/2_utilization_hier.txt
report_power          -file reports/3_power_temperature.txt
report_clocks         -file reports/4_clocks.txt
report_methodology    -file reports/5_methodology.txt -name meth_after
report_drc            -file reports/6_drc.txt -name drc_after
set wns [get_property SLACK [get_timing_paths -delay_type max]]
set whs [get_property SLACK [get_timing_paths -delay_type min]]
set crit_m [get_methodology_violations -quiet -name meth_after -filter {SEVERITY == "Critical Warning"}]
set crit_d [get_drc_violations -quiet -name drc_after -filter {SEVERITY == "Critical Warning"}]
puts "TIMING: WNS = $wns ns, WHS = $whs ns  ($best)"
puts "CRITICAL WARNINGS LEFT: methodology [llength $crit_m], DRC [llength $crit_d]"
foreach v [concat $crit_m $crit_d] { puts "  LEFT: [get_property CHECK $v]  [get_property DESCRIPTION $v]" }
if {$wns < 0 || $whs < 0} {
  puts "*** TIMING NOT MET - XSA NOT written."
} else {
  write_hw_platform -fixed -force -include_bit -file PASS_SFAP.xsa
  puts "XSA written: [file normalize PASS_SFAP.xsa]"
}
close_project
