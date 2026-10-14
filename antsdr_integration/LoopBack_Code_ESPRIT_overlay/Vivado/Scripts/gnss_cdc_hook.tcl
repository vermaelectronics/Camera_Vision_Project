# ============================================================================
#  gnss_cdc_hook.tcl -- applies the gnss_passthrough CDC constraints during
#                       IMPLEMENTATION (impl_1 STEPS.OPT_DESIGN.TCL.PRE)
#
#  Why a hook: in this ADI project flow Source/XDC/gnss_passthrough_cdc.xdc
#  was never applied in implementation (it did not appear in the impl log,
#  and every rx_clk <-> clk_fpga_0 path was timed synchronously, WNS -1.8 ns),
#  whatever its USED_IN settings. A step hook always runs, on the fully linked
#  design, before placement; constraints added here persist through
#  place/route and into the routed checkpoint.
#
#  Installed by Vivado/Scripts/create_project.tcl. Prints GNSS_CDC_HOOK lines
#  into antsdr_e310_gnss.runs/impl_1/runme.log.
# ============================================================================
# create_project.tcl registers a generated wrapper that sets gnss_hook_xdc to
# an absolute path (Vivado may run hooks from a copy in the run directory).
if {![info exists gnss_hook_xdc]} {
  set gnss_hook_root [file normalize [file join [file dirname [file normalize [info script]]] .. ..]]
  set gnss_hook_xdc  [file join $gnss_hook_root Source XDC gnss_passthrough_cdc.xdc]
}
if {![file exists $gnss_hook_xdc]} {
  puts "GNSS_CDC_HOOK: ERROR - $gnss_hook_xdc not found"
  error "gnss_cdc_hook: CDC constraint file missing"
}
puts "GNSS_CDC_HOOK: applying $gnss_hook_xdc"
source $gnss_hook_xdc

# verify on the worst rx_clk <-> clk_fpga_0 paths
foreach {a b} {rx_clk clk_fpga_0 clk_fpga_0 rx_clk} {
  set p [get_timing_paths -quiet -max_paths 1 -from [get_clocks -quiet $a] -to [get_clocks -quiet $b]]
  if {[llength $p]} {
    puts [format "GNSS_CDC_HOOK: %-10s -> %-10s exception '%s'" $a $b [get_property EXCEPTION $p]]
  } else {
    puts "GNSS_CDC_HOOK: $a -> $b: no paths"
  }
}
set p [get_timing_paths -quiet -max_paths 1 \
  -from [get_pins -quiet -hier -filter {NAME =~ *u_doa/u_afifo/mem_reg*/CLK}] \
  -to   [get_pins -quiet -hier -filter {NAME =~ *u_doa/u_afifo/rd_data_reg*/D}]]
if {[llength $p]} { puts "GNSS_CDC_HOOK: DOA FIFO storage exception '[get_property EXCEPTION $p]'" }
puts "GNSS_CDC_HOOK: done"
