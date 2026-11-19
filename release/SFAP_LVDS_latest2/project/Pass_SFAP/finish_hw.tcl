# finish_hw.tcl - after system_project.tcl: methodology waivers, implementation
# again, reports, and PASS_SFAP.xsa (only if timing is met).
# Same corrections as Pass_MVDR_STAP; no RTL, block design or firmware change.
#
#   TIMING-54  scoped set_max_delay -datapath_only clk_fpga_0 <-> rx_clk
#              (ADI axi_dmac and Xilinx axis_data_fifo IP constraint files)
#   TIMING-9   CDC without a 2-flop synchroniser on the capture side
#              (gnss_passthrough capture-and-toggle handshake, data held stable)
#   TIMING-18  no set_input_delay on rx_data_in / rx_frame_in
#              (ADI axi_ad9361 LVDS capture aligned by IDELAY tuning at run time)
# Any OTHER critical warning is listed at the end, not waived.
#
# Usage (project folder, Vivado 2023.2 loaded):
#     vivado -mode batch -source finish_hw.tcl -log finish_hw.log -nojournal

open_project passthrough_sfap.xpr

# ---------- 1. waivers from the first implementation ----------
open_run impl_1 -name impl_before
report_methodology -name meth_before
set n_waived 0
foreach {id why} {
  TIMING-54 "ADI axi_dmac / Xilinx axis_data_fifo IP-scoped CDC max-delay constraints between clk_fpga_0 and rx_clk"
  TIMING-9  "gnss_passthrough CDC: capture-and-toggle handshake, data held stable for many destination clocks, 2-flop ASYNC_REG on the toggle"
  TIMING-18 "ADI axi_ad9361 LVDS RX: capture aligned by IDELAY tuning at run time, no static input delay by design"
} {
  set v [get_methodology_violations -quiet -name meth_before -filter "CHECK == $id"]
  if {[llength $v]} {
    create_waiver -of_objects $v -description $why
    incr n_waived [llength $v]
  }
}
puts "WAIVED: $n_waived methodology violations"
write_waivers -force waivers.xdc
close_design

if {[llength [get_files -quiet *waivers.xdc]] == 0} {
  add_files -fileset constrs_1 -norecurse waivers.xdc
}
set_property USED_IN_SYNTHESIS false [get_files *waivers.xdc]
set_property USED_IN_IMPLEMENTATION true [get_files *waivers.xdc]
set_property STEPS.POWER_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]

# ---------- 2. implementation again (synthesis is unchanged) ----------
reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} { error "implementation failed - see passthrough_sfap.runs/impl_1/runme.log" }

# ---------- 3. reports ----------
open_run impl_1
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
puts "TIMING: WNS = $wns ns, WHS = $whs ns"
puts "CRITICAL WARNINGS LEFT: methodology [llength $crit_m], DRC [llength $crit_d]"
foreach v [concat $crit_m $crit_d] { puts "  LEFT: [get_property CHECK $v]  [get_property DESCRIPTION $v]" }

if {$wns < 0 || $whs < 0} {
  puts "*** TIMING NOT MET - XSA NOT written. Send reports/1_timing_summary.txt."
} else {
  write_hw_platform -fixed -force -include_bit -file PASS_SFAP.xsa
  puts "XSA written: [file normalize PASS_SFAP.xsa]"
}
close_project
