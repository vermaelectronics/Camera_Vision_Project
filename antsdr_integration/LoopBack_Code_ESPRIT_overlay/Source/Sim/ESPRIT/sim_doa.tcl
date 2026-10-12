# ============================================================================
#  sim_doa.tcl -- run the ESPRIT DOA system testbench from the VIVADO TCL
#                 CONSOLE (or vivado -mode batch / -mode tcl).
#
#  USAGE (Vivado Tcl console, any project or none open):
#      source <LoopBack_Code>/Source/Sim/ESPRIT/sim_doa.tcl
#  Waveform GUI instead of batch:
#      set DOA_SIM_GUI 1
#      source <LoopBack_Code>/Source/Sim/ESPRIT/sim_doa.tcl
#  Batch from a shell:
#      vivado -mode batch -source Source/Sim/ESPRIT/sim_doa.tcl
#
#  What it does: xvlog -> xelab -> xsim -R in Build/sim_doa, then echoes every
#  "DOA_TB:" line into the Tcl console and finishes with
#      DOA_SIM_RESULT: PASS | FAIL
#  Full logs: Build/sim_doa/{xvlog,xelab,xsim}.log
# ============================================================================
set doa_here     [file dirname [file normalize [info script]]]
set doa_src_root [file normalize [file join $doa_here .. ..]]
set doa_root     [file normalize [file join $doa_src_root ..]]
source [file join $doa_here doa_files.tcl]

set doa_work [file join $doa_root Build sim_doa]
file mkdir $doa_work
set doa_old_pwd [pwd]
cd $doa_work

proc doa_run {tag args} {
  puts "DOA_SIM: $tag ..."
  if {[catch {exec {*}$args 2>@1} out]} {
    set fh [open "${tag}.out" w]; puts $fh $out; close $fh
    foreach l [split $out "\n"] {
      if {[regexp {ERROR|CRITICAL} $l]} { puts "DOA_SIM:   $l" }
    }
    return 0
  }
  return 1
}

set ok 1
if {![doa_run xvlog xvlog -i $doa_here {*}$doa_rtl_files {*}$doa_tb_files]} { set ok 0 }
if {$ok && ![doa_run xelab xelab -debug typical -s tb_doa tb_gnss_doa]} { set ok 0 }

if {!$ok} {
  puts "DOA_SIM_RESULT: FAIL - compile/elaborate error, see $doa_work/*.log"
} elseif {[info exists DOA_SIM_GUI] && $DOA_SIM_GUI} {
  set wv [file join $doa_here waves_doa.tcl]
  puts "DOA_SIM: opening xsim GUI with $wv"
  exec xsim tb_doa -gui -testplusarg STIM=[file join $doa_here doa_stim.hex] -tclbatch $wv &
  puts "DOA_SIM_RESULT: GUI launched (results print in the xsim Tcl console)"
} else {
  puts "DOA_SIM: xsim running (about 1-3 minutes) ..."
  catch {exec xsim tb_doa -R -testplusarg STIM=[file join $doa_here doa_stim.hex] 2>@1} out
  set fh [open "xsim_run.out" w]; puts $fh $out; close $fh
  set pass 0
  foreach l [split $out "\n"] {
    if {[string match "DOA_TB*" $l]} { puts $l }
    if {[string match "DOA_TB_RESULT: PASS*" $l]} { set pass 1 }
  }
  if {$pass} { puts "DOA_SIM_RESULT: PASS" } else { puts "DOA_SIM_RESULT: FAIL - see $doa_work/xsim_run.out" }
}
cd $doa_old_pwd
