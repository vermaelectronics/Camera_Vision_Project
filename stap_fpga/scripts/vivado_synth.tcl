# Out-of-context Vivado synthesis + implementation of stap_top (non-project mode).
#   vivado -mode batch -source scripts/vivado_synth.tcl -tclargs <part> <FOLD>
#   e.g. -tclargs xczu3eg-sbva484-1-e 4
# NOTE: written for convenience, not run in the environment this core was developed in.
set part [lindex $argv 0]
set fold [lindex $argv 1]
if {$part eq ""} { set part xczu3eg-sbva484-1-e }
if {$fold eq ""} { set fold 4 }
set root [file normalize [file dirname [info script]]/..]
set out  $root/build/vivado_${part}_fold${fold}
file mkdir $out
read_verilog [list $root/rtl/stap_fir.v $root/rtl/stap_corr.v $root/rtl/stap_top.v]
read_xdc $root/constraints/stap_top.xdc
synth_design -top stap_top -part $part -mode out_of_context -generic FOLD=$fold
opt_design
place_design
route_design
report_utilization    -file $out/utilization.rpt
report_timing_summary -file $out/timing.rpt
puts "reports in $out"
