# ============================================================================
#  add_doa_sim_to_project.tcl -- add the DOA system testbench to the OPEN
#  antsdr_e310_gnss project as a Vivado simulation set, then run it with the
#  normal Vivado simulator flow (Flow Navigator -> Run Simulation also works).
#
#  USAGE (Vivado Tcl console, project open):
#      source <LoopBack_Code>/Source/Sim/ESPRIT/add_doa_sim_to_project.tcl
#  Results print in the Tcl console and the waveform window opens.
# ============================================================================
set doa_here     [file dirname [file normalize [info script]]]
set doa_src_root [file normalize [file join $doa_here .. ..]]
source [file join $doa_here doa_files.tcl]

if {[llength [get_filesets -quiet sim_doa]] == 0} {
  create_fileset -simset sim_doa
}
set fs [get_filesets sim_doa]
# RTL is referenced, not copied; the block design is not needed for this test
add_files -norecurse -fileset $fs $doa_rtl_files
add_files -norecurse -fileset $fs $doa_tb_files
set_property include_dirs [list $doa_here] $fs
set_property top tb_gnss_doa $fs
set_property top_lib xil_defaultlib $fs
set_property -name {xsim.simulate.runtime} -value {all} -objects $fs
set_property -name {xsim.simulate.xsim.more_options} \
             -value "-testplusarg STIM=[file join $doa_here doa_stim.hex]" -objects $fs
current_fileset -simset $fs
puts "DOA_SIM: simulation set sim_doa ready (top tb_gnss_doa). Launching ..."
launch_simulation -simset sim_doa
source [file join $doa_here waves_doa.tcl]
puts "DOA_SIM: see the DOA_TB lines above; final line is DOA_TB_RESULT"
