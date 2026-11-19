# improve_hw.tcl - lower power/temperature and clear the methodology critical warnings
# WITHOUT changing the RTL, the block design or any firmware value.
#
#   1. Waivers (constraints only, no logic change) for:
#        TIMING-54  scoped set_max_delay -datapath_only clk_fpga_0 <-> rx_clk
#                   (written by the ADI axi_dmac IP's own constraint file)
#        TIMING-9   CDC without a 2-flop synchroniser on the capture side
#                   (toggle/ack handshake bundles in gnss_passthrough, data held stable)
#        TIMING-18  no set_input_delay on rx_data_in / rx_frame_in
#                   (ADI axi_ad9361 LVDS capture is aligned with IDELAY tuning at run time)
#      saved to waivers.xdc and added to the project (implementation only).
#   2. power_opt_design ON for impl_1 (clock-enable gating of BRAM/registers; the
#      logic function is unchanged), then implementation is run again.
#   3. Timing, power and methodology reports; the XSA is written ONLY if timing is met.
#
# Usage (in the project folder, Vivado 2023.2 loaded):
#     vivado -mode batch -source improve_hw.tcl -log improve_hw.log -nojournal

open_project passthrough_mvdr.xpr

# ---------- 1. waivers ----------
open_run impl_1 -name impl_before
report_methodology -name meth_before
set n_waived 0
foreach {id why} {
  TIMING-54 "ADI axi_dmac IP-scoped CDC max-delay constraints between clk_fpga_0 and rx_clk (vendor IP constraint file)"
  TIMING-9  "gnss_passthrough CDC: toggle/ack handshake, data bundle held stable for many destination clocks, 2-flop ASYNC_REG on the toggle"
  TIMING-18 "ADI axi_ad9361 LVDS RX interface: capture aligned by IDELAY tuning at run time, no static input delay by design"
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

if {[llength [get_files -quiet waivers.xdc]] == 0} {
  add_files -fileset constrs_1 -norecurse waivers.xdc
}
set_property USED_IN_SYNTHESIS false [get_files waivers.xdc]
set_property USED_IN_IMPLEMENTATION true [get_files waivers.xdc]

# ---------- 2. power optimisation + implementation ----------
if {[file exists PASS_MVDR_STAP.xsa]} { file copy -force PASS_MVDR_STAP.xsa PASS_MVDR_STAP_before_power_opt.xsa }
set_property STEPS.POWER_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} { error "implementation failed - see passthrough_mvdr.runs/impl_1/runme.log" }

# ---------- 3. reports ----------
open_run impl_1
report_timing_summary -file timing_report.txt
report_utilization    -file utilization_report.txt
report_power          -file power_report.txt
report_methodology    -file methodology_report.txt -name meth_after
set wns [get_property SLACK [get_timing_paths -delay_type max]]
set whs [get_property SLACK [get_timing_paths -delay_type min]]
set crit [llength [get_methodology_violations -quiet -name meth_after -filter {SEVERITY == "Critical Warning"}]]
puts "TIMING: WNS = $wns ns, WHS = $whs ns"
puts "METHODOLOGY critical warnings left: $crit"
puts "POWER: see power_report.txt (Total On-Chip Power / Junction Temperature)"

if {$wns < 0 || $whs < 0} {
  puts "*** TIMING NOT MET with power_opt_design - XSA NOT written."
  puts "*** Undo: set_property STEPS.POWER_OPT_DESIGN.IS_ENABLED false [get_runs impl_1]; reset_run impl_1; rerun run_to_xsa.tcl"
} else {
  write_hw_platform -fixed -force -include_bit -file PASS_MVDR_STAP.xsa
  puts "XSA written: [file normalize PASS_MVDR_STAP.xsa]"
}
close_project
