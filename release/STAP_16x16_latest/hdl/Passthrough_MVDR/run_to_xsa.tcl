# run_to_xsa.tcl - synthesis + implementation + bitstream + XSA for an existing project.
# Usage (in this folder):  vivado -mode batch -source run_to_xsa.tcl
open_project passthrough_mvdr.xpr
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} { error "synthesis failed - see passthrough_mvdr.runs/synth_1/runme.log" }
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} { error "implementation failed - see passthrough_mvdr.runs/impl_1/runme.log" }
open_run impl_1
report_timing_summary -file timing_report.txt
report_utilization -file utilization_report.txt
puts "TIMING: WNS = [get_property SLACK [get_timing_paths -delay_type max]] ns, WHS = [get_property SLACK [get_timing_paths -delay_type min]] ns"
write_hw_platform -fixed -force -include_bit -file PASS_MVDR_STAP.xsa
puts "XSA written: [file normalize PASS_MVDR_STAP.xsa]"
close_project
