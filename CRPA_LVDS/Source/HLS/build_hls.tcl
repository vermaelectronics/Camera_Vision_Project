# ============================================================================
#  build_hls.tcl  --  Vitis HLS 2023.2: the FPGA filter core of ONE CRPA
#  algorithm -> block-design IP (instantiated as crpa_core_0).
#
#  The algorithm comes from the environment (build_all.sh sets it, see
#  Automation/Linux/crpa_algos.sh):
#    CRPA_ALGO      crpa1 | crpa27 | crpa28 | crpa29 | crpa30 | crpa31
#    CRPA_CORE      smi_pi (crpa1) | stap_pi (crpa27..31)
#    CRPA_CORE_VER  IP version (smi_pi 4.9, stap_pi 1.0)
#    GNSS_CRPA_IP_REPO  where the IP goes (Build/<algo>/ip_repo)
#  Sources: Source/CRPA/<algo>/fpga/<core>.{cpp,h} (exactly as delivered) and,
#  for the C simulation, Source/CRPA/<algo>/arm/gnss_aj_math.{c,h} (the
#  board's weight solver) with Source/HLS/testbench.cpp.
#
#  Steps
#    1. C simulation, scenarios 1-6 of testbench.cpp (core accuracy + bypass,
#       wideband, CW, 2-tap RX2 channel, no jammer, moving jammer), each a
#       separate csim_design run.
#    2. C synthesis at a 10 ns target (sys_cpu_clk, 100 MHz), II = 1.
#    3. Report checks: II must be 1 and the estimated clock within the target;
#       the generated RTL must have the ap_none ports the block design wires
#       (wts, wcj, cfg, nrm, and gain on stap_pi).
#    4. export_design -> $GNSS_CRPA_IP_REPO/<core>, VLNV antsdr:gnss:<core>:<ver>
#
#  OVERRIDES (environment)
#    CRPA_CLK_NS=<ns>        HLS clock target, default 10
#    CRPA_SKIP_CSIM=1        skip step 1
#    CRPA_CSIM_LIST="1 2"    only these scenarios
#
#  SUCCESS MARKER: "CRPA_HLS: PASS" only after the IP has been copied.
# ============================================================================

proc need_env {n} {
  if {![info exists ::env($n)] || $::env($n) eq ""} {
    puts "CRPA_HLS: FAIL - environment variable $n is not set (run through build_all.sh --algo <algo>)"
    exit 1
  }
  return $::env($n)
}
set algo    [need_env CRPA_ALGO]
set core    [need_env CRPA_CORE]
set corever [need_env CRPA_CORE_VER]

set here [file normalize [file dirname [info script]]]
set src  [file normalize [file join $here .. CRPA $algo fpga]]
set arm  [file normalize [file join $here .. CRPA $algo arm]]
if {[info exists ::env(GNSS_CRPA_IP_REPO)]} {
  set ip_repo [file normalize $::env(GNSS_CRPA_IP_REPO)]
} else {
  set ip_repo [file normalize [file join $here .. .. Build $algo ip_repo]]
}
set work [file join $ip_repo .. hls_work]
file mkdir $work
set work [file normalize $work]
set out_ip [file join $ip_repo $core]
foreach f [list [file join $src $core.cpp] [file join $src $core.h] \
                [file join $arm gnss_aj_math.c] [file join $arm gnss_aj_math.h] \
                [file join $here testbench.cpp]] {
  if {![file exists $f]} { puts "CRPA_HLS: FAIL - missing $f"; exit 1 }
}
set tbdef [expr {$core eq "smi_pi" ? "-DCRPA_SMI" : "-DCRPA_STAP"}]

set clk_period 10
if {[info exists ::env(CRPA_CLK_NS)]} { set clk_period $::env(CRPA_CLK_NS) }
set clk_uncertainty 1.0
set scenarios {1 2 3 4 5 6}
if {[info exists ::env(CRPA_CSIM_LIST)]} { set scenarios $::env(CRPA_CSIM_LIST) }
puts "CRPA_HLS: $algo, core $core $corever, sources $src"
puts "CRPA_HLS: HLS clock target $clk_period ns (uncertainty $clk_uncertainty ns), II=1"

cd $work
open_project -reset ${core}_hls
set_top $core
add_files [file join $src $core.cpp] -cflags "-I$src"
add_files [file join $src $core.h]
add_files -tb [file join $here testbench.cpp] -cflags "$tbdef -I$src -I$arm -Wno-unknown-pragmas"
add_files -tb [file join $arm gnss_aj_math.c] -cflags "-I$arm"
open_solution -reset sol -flow_target vivado
set_part {xc7z020clg400-2}
create_clock -period $clk_period -name default
set_clock_uncertainty $clk_uncertainty

# ---- 1 C simulation -------------------------------------------------------------
if {![info exists ::env(CRPA_SKIP_CSIM)] || $::env(CRPA_SKIP_CSIM) eq "0"} {
  foreach scen $scenarios {
    puts "CRPA_HLS: csim scenario $scen"
    if {[catch {csim_design -O -argv $scen} err]} {
      puts "CRPA_HLS: FAIL - C simulation scenario $scen: $err"
      exit 1
    }
  }
}

# ---- 2 C synthesis ---------------------------------------------------------------
csynth_design

# ---- 3 report checks -------------------------------------------------------------
set rpt_xml [file join $work ${core}_hls sol syn report csynth.xml]
set rpt_txt [file join $work ${core}_hls sol syn report ${core}_csynth.rpt]
if {![file exists $rpt_xml]} { puts "CRPA_HLS: FAIL - synthesis report $rpt_xml not found"; exit 1 }
set fh [open $rpt_xml r]; set xml [read $fh]; close $fh
set est ""
regexp {<EstimatedClockPeriod>([0-9.]+)</EstimatedClockPeriod>} $xml -> est
set ii ""
if {![regexp {<PipelineInitiationInterval>([0-9]+)</PipelineInitiationInterval>} $xml -> ii]} {
  regexp {<Interval-min>([0-9]+)</Interval-min>} $xml -> ii
}
foreach res {DSP BRAM_18K FF LUT} {
  set v ""
  regexp "<Resources>.*?<${res}>(\[0-9\]+)</${res}>" $xml -> v
  puts "CRPA_HLS: estimated $res = $v"
}
puts "CRPA_HLS: estimated clock period $est ns (target $clk_period ns), achieved II $ii (requested 1)"
if {$est eq "" || $est > $clk_period} {
  puts "CRPA_HLS: FAIL - estimated clock period '$est' ns exceeds the $clk_period ns target"; exit 1
}
if {$ii eq "" || $ii > 1} { puts "CRPA_HLS: FAIL - achieved II '$ii' is worse than 1"; exit 1 }

set vtop [file join $work ${core}_hls sol syn verilog ${core}.v]
if {[file exists $vtop]} {
  set fh [open $vtop r]; set vtxt [read $fh]; close $fh
  set ports {wts wcj cfg nrm}
  if {$core eq "stap_pi"} { lappend ports gain }
  foreach p $ports {
    if {![regexp "\\m${p}\\M" $vtxt]} {
      puts "CRPA_HLS: FAIL - port $p missing from the generated RTL"; exit 1
    }
  }
  puts "CRPA_HLS: ap_none weight / control ports OK ($ports)"
}

# ---- 4 package as a Vivado IP ------------------------------------------------------
if {$core eq "smi_pi"} {
  set dname "SMI-PI space-time power inversion ($algo)"
  set desc  "CRPA $algo: two-element space-time SMI power-inversion filter (Vitis HLS), RX1 + RX2 taps, 2 image taps, RX1 reference fixed at 1, weights from the ARM (gnss_passthrough AJ_W/AJ_WC, ap_none), weight scale, make-up shift, exact bypass."
} else {
  set dname "STAP-NLMS space-time power inversion ($algo)"
  set desc  "CRPA $algo: two-element space-time power-inversion filter (Vitis HLS), RX1 + RX2 taps, 3 + 3 image taps, RX1 reference fixed at 1, weights from the ARM (direct start + NLMS, gnss_passthrough AJ_W/AJ_WC, ap_none), weight scale, make-up shift, fine gain, exact bypass."
}
export_design -format ip_catalog -rtl verilog \
  -vendor antsdr -library gnss -version $corever -ipname $core \
  -display_name $dname -description $desc

set impl_dir [file join $work ${core}_hls sol impl]
set ip_src   [file join $impl_dir ip]
if {![file exists [file join $ip_src component.xml]]} {
  set zips [glob -nocomplain -directory $impl_dir *.zip]
  if {[llength $zips] == 0} { puts "CRPA_HLS: FAIL - export_design produced no IP in $impl_dir"; exit 1 }
  file delete -force $ip_src
  file mkdir $ip_src
  exec unzip -q -o [lindex $zips 0] -d $ip_src
}
if {![file exists [file join $ip_src component.xml]]} { puts "CRPA_HLS: FAIL - no component.xml in $ip_src"; exit 1 }

file mkdir $ip_repo
file delete -force $out_ip
file copy -force $ip_src $out_ip
if {[file exists $rpt_txt]} { file copy -force $rpt_txt $out_ip }

puts "CRPA_HLS: IP antsdr:gnss:${core}:${corever} ($algo) -> $out_ip"
puts "CRPA_HLS: PASS"
exit 0
