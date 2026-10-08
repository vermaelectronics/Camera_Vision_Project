# ============================================================================
#  run_hls.tcl  --  build the SMI_PI IP on its own (Vitis HLS 2023.2)
#
#  USAGE (from this folder)
#    vitis_hls -f run_hls.tcl
#
#  Steps: C simulation (4 scenarios) -> C synthesis (8 ns, II=2) ->
#  check II and clock -> export Vivado IP to ./ip  (antsdr:gnss:smi_pi:1.0)
#
#  OPTIONS (environment)
#    SMI_PI_CLK_NS=<ns>   clock target, default 8
#    SMI_PI_K=<k>         averaging time constant 2^k samples, default 10
#    SMI_PI_SKIP_CSIM=1   skip C simulation
#    SMI_PI_COSIM=1       also run C/RTL co-simulation (slow)
#
#  In Vivado: Settings > IP > Repository > add this folder's ./ip, then
#  "Add IP" > "SMI power inversion".
# ============================================================================

set here [file normalize [file dirname [info script]]]
cd $here

set clk 8
if {[info exists ::env(SMI_PI_CLK_NS)]} { set clk $::env(SMI_PI_CLK_NS) }
set kdef ""
if {[info exists ::env(SMI_PI_K)]} { set kdef "-DSMI_PI_K=$::env(SMI_PI_K)" }

open_project -reset smi_pi_prj
set_top smi_pi
add_files smi_pi.cpp -cflags "-I$here $kdef"
add_files -tb testbench.cpp -cflags "-I$here $kdef -Wno-unknown-pragmas"
open_solution -reset sol -flow_target vivado
set_part {xc7z020clg400-2}
create_clock -period $clk -name default
set_clock_uncertainty 1.0

# ---- 1 C simulation (each scenario needs a freshly reset core) -------------
if {![info exists ::env(SMI_PI_SKIP_CSIM)] || $::env(SMI_PI_SKIP_CSIM) eq "0"} {
  foreach scen {1 2 3 4} {
    puts "SMI_PI: csim scenario $scen"
    if {[catch {csim_design -argv $scen} err]} {
      puts "SMI_PI: FAIL - C simulation scenario $scen"
      exit 1
    }
  }
}

# ---- 2 C synthesis -----------------------------------------------------------
csynth_design

set fh [open [file join smi_pi_prj sol syn report csynth.xml] r]
set xml [read $fh]
close $fh
set est ""; set ii ""
regexp {<EstimatedClockPeriod>([0-9.]+)</EstimatedClockPeriod>} $xml -> est
if {![regexp {<PipelineInitiationInterval>([0-9]+)</PipelineInitiationInterval>} $xml -> ii]} {
  regexp {<Interval-min>([0-9]+)</Interval-min>} $xml -> ii
}
puts "SMI_PI: estimated clock $est ns (target $clk ns), II $ii (target 2)"
if {$est eq "" || $est > $clk} { puts "SMI_PI: FAIL - clock estimate above target"; exit 1 }
if {$ii  eq "" || $ii  > 2}    { puts "SMI_PI: FAIL - II worse than 2"; exit 1 }

# ---- optional C/RTL co-simulation ------------------------------------------
if {[info exists ::env(SMI_PI_COSIM)] && $::env(SMI_PI_COSIM) eq "1"} {
  cosim_design -argv 1
}

# ---- 3 export the IP -----------------------------------------------------------
# (Plain-text description: braces or commas inside a {...} group break export.)
export_design -format ip_catalog -rtl verilog \
  -vendor antsdr -library gnss -version 1.0 -ipname smi_pi \
  -display_name "SMI power inversion" \
  -description "Two-element closed-form SMI power-inversion anti-jam core. Output is RX1 minus a times RX2."

set impl [file join smi_pi_prj sol impl]
file delete -force ip
if {[file exists [file join $impl ip component.xml]]} {
  file copy -force [file join $impl ip] ip
} else {
  set z [glob -nocomplain -directory $impl *.zip]
  if {[llength $z] == 0} { puts "SMI_PI: FAIL - no exported IP"; exit 1 }
  file mkdir ip
  exec unzip -q -o [lindex $z 0] -d ip
}
file copy -force [file join smi_pi_prj sol syn report smi_pi_csynth.rpt] ip
puts "SMI_PI: IP antsdr:gnss:smi_pi:1.0 -> [file normalize ip]"
puts "SMI_PI: PASS"
exit 0
