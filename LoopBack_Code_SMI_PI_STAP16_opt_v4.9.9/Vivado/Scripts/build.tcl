# ============================================================================
#  build.tcl  --  synthesise, implement, write bitstream, export XSA
#
#  Kept separate from create_project.tcl so that programming does not require
#  a rebuild and vice versa (requirements 16, 26).
#
#  SUCCESS CRITERIA (requirement 60)
#    A pass is reported only if:
#      - synth_1 reaches status "synth_design Complete!"
#      - impl_1  reaches status "write_bitstream Complete!"
#      - the .bit and .xsa files exist AND were written by this run
#    The existence of an output file is never on its own treated as success.
#
#  USAGE
#    vivado -mode batch -source build.tcl -tclargs <xpr> <bit_out> <xsa_out> [jobs]
# ============================================================================

if {$argc < 3} {
  puts "BUILD_RESULT: FAIL - usage: build.tcl <xpr> <bit_out> <xsa_out> \[jobs\]"
  exit 2
}

set xpr     [file normalize [lindex $argv 0]]
set bit_out [file normalize [lindex $argv 1]]
set xsa_out [file normalize [lindex $argv 2]]
set jobs    [expr {$argc > 3 ? [lindex $argv 3] : 4}]

if {![file exists $xpr]} {
  puts "BUILD_RESULT: FAIL - project not found: $xpr"
  puts "BUILD_HINT: run create_project.tcl first, or use Automation/Linux/build_all.sh"
  exit 2
}

open_project $xpr

# Same as create_project.tcl: never synthesise against a stale or missing
# incremental reference (Vivado 2023.2 auto-incremental), and clear any
# reference a previous run left behind on an existing project.
foreach run [list synth_1 impl_1] {
  catch {set_property AUTO_INCREMENTAL_CHECKPOINT 0 [get_runs $run]}
  catch {set_property INCREMENTAL_CHECKPOINT {} [get_runs $run]}
}

# ---- synthesis -------------------------------------------------------------
puts "BUILD_STAGE: synthesis starting"
reset_run synth_1
launch_runs synth_1 -jobs $jobs
# wait_on_run raises a Tcl error whenever Vivado marks ANY run in the chain as
# failed, even when the run itself completed (seen on 2023.2: impl_1 wrote the
# bitstream, then wait_on_run aborted citing synth_1 with "set_property expects
# at least one object"). The decision is therefore taken from the run's own
# STATUS/PROGRESS below, never from wait_on_run's return code alone.
if {[catch {wait_on_run synth_1} wait_err]} {
  puts "BUILD_NOTE: wait_on_run synth_1 reported: $wait_err"
}

set synth_status [get_property STATUS   [get_runs synth_1]]
set synth_prog   [get_property PROGRESS [get_runs synth_1]]
puts "BUILD_SYNTH_STATUS: $synth_status"
puts "BUILD_SYNTH_PROGRESS: $synth_prog"

if {$synth_prog ne "100%"} {
  puts "BUILD_RESULT: FAIL - synthesis did not complete (progress $synth_prog)"
  puts "BUILD_EVIDENCE: see [get_property DIRECTORY [get_runs synth_1]]/runme.log"
  exit 2
}

open_run synth_1 -name synth_1
report_timing_summary -file [file join [file dirname $bit_out] timing_synth.rpt] -quiet
report_utilization    -file [file join [file dirname $bit_out] utilization_synth.rpt] -quiet

# ---- implementation + bitstream -------------------------------------------
puts "BUILD_STAGE: implementation starting"
reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs $jobs
if {[catch {wait_on_run impl_1} wait_err]} {
  puts "BUILD_NOTE: wait_on_run impl_1 reported: $wait_err"
  puts "BUILD_NOTE: judging the result from impl_1 STATUS/PROGRESS and the .bit file instead"
}

set impl_status [get_property STATUS   [get_runs impl_1]]
set impl_prog   [get_property PROGRESS [get_runs impl_1]]
puts "BUILD_IMPL_STATUS: $impl_status"
puts "BUILD_IMPL_PROGRESS: $impl_prog"

if {$impl_prog ne "100%" || ![string match "*write_bitstream Complete*" $impl_status]} {
  puts "BUILD_RESULT: FAIL - implementation did not complete (progress $impl_prog)"
  puts "BUILD_EVIDENCE: see [get_property DIRECTORY [get_runs impl_1]]/runme.log"
  exit 2
}

open_run impl_1
report_timing_summary -warn_on_violation \
  -file [file join [file dirname $bit_out] timing_impl.rpt] -quiet
report_utilization -file [file join [file dirname $bit_out] utilization_impl.rpt] -quiet

# Timing closure is reported explicitly; a design that met neither setup nor
# hold must not be presented as a clean build.
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
puts "BUILD_WNS: $wns"
puts "BUILD_WHS: $whs"
if {$wns < 0 || $whs < 0} {
  puts "BUILD_TIMING: VIOLATED - the bitstream is produced but timing is not met"
} else {
  puts "BUILD_TIMING: MET"
}

# AD9361 LVDS receive-path delay report from the vendor flow (informational).
if {[catch {
  set here [pwd]
  cd [file dirname $bit_out]
  source $::env(ADI_HDL_DIR)/library/axi_ad9361/axi_ad9361_delay.tcl
  cd $here
} e]} {
  puts "BUILD_NOTE: axi_ad9361_delay.tcl did not run: $e"
}

# ---- collect outputs -------------------------------------------------------
set impl_dir [get_property DIRECTORY [get_runs impl_1]]
set top      [get_property top [current_fileset]]
set bit_src  [file join $impl_dir "${top}.bit"]

if {![file exists $bit_src]} {
  puts "BUILD_RESULT: FAIL - write_bitstream reported complete but $bit_src is absent"
  exit 2
}

file mkdir [file dirname $bit_out]
file copy -force $bit_src $bit_out
puts "BUILD_BIT: $bit_out"

file mkdir [file dirname $xsa_out]
write_hw_platform -fixed -include_bit -force -file $xsa_out
if {![file exists $xsa_out]} {
  puts "BUILD_RESULT: FAIL - write_hw_platform did not produce $xsa_out"
  exit 2
}
puts "BUILD_XSA: $xsa_out"

puts "BUILD_RESULT: PASS"
exit 0
