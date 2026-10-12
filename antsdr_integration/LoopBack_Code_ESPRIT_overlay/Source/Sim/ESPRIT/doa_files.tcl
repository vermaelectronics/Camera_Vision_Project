# ============================================================================
#  doa_files.tcl -- single list of every source the ESPRIT DOA design needs.
#  Sourced by sim_doa.tcl, add_doa_sim_to_project.tcl and the IP packager so
#  the file list cannot drift between simulation and synthesis.
#  Expects:  $doa_src_root = <LoopBack_Code>/Source
#  Returns:  doa_rtl_files (synthesis), doa_tb_files (simulation only)
# ============================================================================
set doa_rtl_files [list \
  [file join $doa_src_root HDL gnss_passthrough.v] \
  [file join $doa_src_root HDL gnss_doa_esprit.v] \
  [file join $doa_src_root HDL doa_async_fifo.v] \
  [file join $doa_src_root HDL doa_test_gen.v] ]
foreach f [lsort [glob -nocomplain [file join $doa_src_root HDL esprit *.v]]] {
  lappend doa_rtl_files $f
}
set doa_tb_files [list [file join $doa_src_root Sim ESPRIT tb_gnss_doa.v]]
