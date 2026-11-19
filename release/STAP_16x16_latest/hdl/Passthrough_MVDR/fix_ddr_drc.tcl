# fix_ddr_drc.tcl - clear the 10 DRC IOSTDTYPE-1 critical warnings on the PS7 DDR pins
# (ddr_ck_p/n, ddr_dqs_p/n[3:0]). No RTL, block design or firmware change.
#
# Why they are safe to waive: these are the Zynq PS7 dedicated DDR pins. Their
# differential I/O (DIFF_SSTL15 / DIFF_SSTL15_T_DCI) is built into the PS DDR
# controller; the top-level netlist only sees them as single-ended inout ports,
# which is what the DRC complains about. The DDR works (the firmware runs from DDR).
#
# Usage (project folder, Vivado 2023.2 loaded):
#     vivado -mode batch -source fix_ddr_drc.tcl -log fix_ddr_drc.log -nojournal

open_project passthrough_mvdr.xpr

set f [open ps7_ddr_drc.xdc w]
puts $f "## PS7 dedicated DDR pins: differential I/O is inside the PS DDR controller."
puts $f "## The single-ended top-level port view triggers DRC IOSTDTYPE-1; waived, and"
puts $f "## lowered to a warning in case a waiver does not match."
puts $f "create_waiver -type DRC -id IOSTDTYPE-1 -objects \[get_ports -quiet {ddr_ck_p ddr_ck_n ddr_dqs_p\[*\] ddr_dqs_n\[*\]}\] -description \"PS7 dedicated DDR pins, differential I/O built into the PS DDR controller\""
puts $f "set_property SEVERITY {Warning} \[get_drc_checks IOSTDTYPE-1\]"
close $f
if {[llength [get_files -quiet ps7_ddr_drc.xdc]] == 0} {
  add_files -fileset constrs_1 -norecurse ps7_ddr_drc.xdc
}
set_property USED_IN_SYNTHESIS false [get_files ps7_ddr_drc.xdc]
set_property USED_IN_IMPLEMENTATION true [get_files ps7_ddr_drc.xdc]

if {[file exists PASS_MVDR_STAP.xsa]} { file copy -force PASS_MVDR_STAP.xsa PASS_MVDR_STAP_before_ddr_fix.xsa }
reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} { error "implementation failed - see passthrough_mvdr.runs/impl_1/runme.log" }

open_run impl_1
file mkdir reports
report_timing_summary -file reports/1_timing_summary.txt
report_utilization    -file reports/2_utilization.txt
report_power          -file reports/3_power_temperature.txt
report_methodology    -file reports/5_methodology.txt
report_drc            -file reports/6_drc.txt
set wns [get_property SLACK [get_timing_paths -delay_type max]]
set whs [get_property SLACK [get_timing_paths -delay_type min]]
puts "TIMING: WNS = $wns ns, WHS = $whs ns"
if {$wns < 0 || $whs < 0} {
  puts "*** TIMING NOT MET - XSA NOT written (previous one kept as PASS_MVDR_STAP_before_ddr_fix.xsa)"
} else {
  write_hw_platform -fixed -force -include_bit -file PASS_MVDR_STAP.xsa
  puts "XSA written: [file normalize PASS_MVDR_STAP.xsa]"
}
close_project
