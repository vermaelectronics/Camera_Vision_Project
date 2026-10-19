# ============================================================================
#  build_hls.tcl  --  Vitis HLS 2023.2: lcmv_crpa.cpp -> block-design IP lcmv_crpa
#
#  Builds the LCMV CRPA core (beamformer + 2x2 covariance, see lcmv_crpa.cpp)
#  and packages it as a Vivado IP that Source/BlockDesign/system_bd.tcl
#  instantiates as lcmv_0, in the AD9361 sample clock domain (l_clk, 8 ns).
#
#  Steps
#    1. C simulation (testbench.cpp, using the firmware's LCMV solver
#       Source/Firmware/app_gnss_e310/lcmv_math.c).
#    2. C synthesis at an 8 ns target, II=2.
#    3. Checks the synthesis report: requested II reached and estimated clock
#       within the target, or the script fails here.
#    4. export_design -> $GNSS_CRPA_IP_REPO/lcmv_crpa (default ../../../Build/ip_repo)
#
#  USAGE (from this directory)
#    vitis_hls -f build_hls.tcl
#
#  OVERRIDES (environment)
#    LCMV_CLK_NS=<ns>    HLS clock target, default 8
#    LCMV_SKIP_CSIM=1    skip step 1
#
#  OUTPUT
#    $GNSS_CRPA_IP_REPO/lcmv_crpa/component.xml, VLNV antsdr:gnss:lcmv_crpa:1.0
#
#  SUCCESS MARKER
#    Prints "LCMV_HLS: PASS" only after the IP has been copied.
# ============================================================================

set here [file normalize [file dirname [info script]]]
set work [file join $here build]
set fw   [file normalize [file join $here .. .. Firmware app_gnss_e310]]
if {[info exists ::env(GNSS_CRPA_IP_REPO)]} {
  set ip_repo [file normalize $::env(GNSS_CRPA_IP_REPO)]
} else {
  set ip_repo [file normalize [file join $here .. .. .. Build ip_repo]]
}
set out_ip [file join $ip_repo lcmv_crpa]

set clk_period 8
if {[info exists ::env(LCMV_CLK_NS)]} { set clk_period $::env(LCMV_CLK_NS) }
set pipeline_ii 2
set clk_uncertainty 1.0
puts "LCMV_HLS: HLS clock target $clk_period ns (uncertainty $clk_uncertainty ns), II=$pipeline_ii"

foreach f [list [file join $here lcmv_crpa.cpp] [file join $here lcmv_crpa.h] \
                [file join $here testbench.cpp] [file join $fw lcmv_math.c] [file join $fw lcmv_math.h]] {
  if {![file exists $f]} {
    puts "LCMV_HLS: FAIL - missing source $f"
    exit 1
  }
}

file mkdir $work
cd $work

# ---- HLS project --------------------------------------------------------------
open_project -reset lcmv_hls
set_top lcmv_crpa
add_files [file join $here lcmv_crpa.cpp] -cflags "-I$here"
add_files -tb [file join $here testbench.cpp] -cflags "-I$here -I$fw -Wno-unknown-pragmas"
add_files -tb [file join $fw lcmv_math.c]     -cflags "-I$fw"
open_solution -reset sol -flow_target vivado
set_part {xc7z020clg400-2}
create_clock -period $clk_period -name default
set_clock_uncertainty $clk_uncertainty

# ---- 1 C simulation -------------------------------------------------------------
if {![info exists ::env(LCMV_SKIP_CSIM)] || $::env(LCMV_SKIP_CSIM) eq "0"} {
  puts "LCMV_HLS: csim"
  if {[catch {csim_design} err]} {
    puts "LCMV_HLS: FAIL - C simulation: $err"
    exit 1
  }
}

# ---- 2 C synthesis ---------------------------------------------------------------
csynth_design

# ---- 3 report checks -------------------------------------------------------------
set rpt_xml [file join $work lcmv_hls sol syn report csynth.xml]
set rpt_txt [file join $work lcmv_hls sol syn report lcmv_crpa_csynth.rpt]
if {![file exists $rpt_xml]} {
  puts "LCMV_HLS: FAIL - synthesis report $rpt_xml not found"
  exit 1
}
set fh [open $rpt_xml r]
set xml [read $fh]
close $fh

set est ""
regexp {<EstimatedClockPeriod>([0-9.]+)</EstimatedClockPeriod>} $xml -> est
set ii ""
if {![regexp {<PipelineInitiationInterval>([0-9]+)</PipelineInitiationInterval>} $xml -> ii]} {
  regexp {<Interval-min>([0-9]+)</Interval-min>} $xml -> ii
}
puts "LCMV_HLS: estimated clock period $est ns (target $clk_period ns), achieved II $ii (requested $pipeline_ii)"
if {$est eq "" || $est > $clk_period} {
  puts "LCMV_HLS: FAIL - estimated clock period '$est' ns exceeds the $clk_period ns target"
  exit 1
}
if {$ii eq "" || $ii > $pipeline_ii} {
  puts "LCMV_HLS: FAIL - achieved II '$ii' is worse than the requested $pipeline_ii"
  exit 1
}

# ---- 4 package as a Vivado IP ------------------------------------------------------
export_design -format ip_catalog -rtl verilog \
  -vendor antsdr -library gnss -version 1.0 -ipname lcmv_crpa \
  -display_name "LCMV CRPA (2-element)" \
  -description "Two-element LCMV anti-jam core (Vitis HLS): beamformer y = w1^H x1 + w2^H x2 and 2x2 covariance R for the PS solver w = R^-1 C (C^H R^-1 C)^-1 f."

set impl_dir [file join $work lcmv_hls sol impl]
set ip_src   [file join $impl_dir ip]
if {![file exists [file join $ip_src component.xml]]} {
  set zips [glob -nocomplain -directory $impl_dir *.zip]
  if {[llength $zips] == 0} {
    puts "LCMV_HLS: FAIL - export_design produced no IP in $impl_dir"
    exit 1
  }
  file delete -force $ip_src
  file mkdir $ip_src
  exec unzip -q -o [lindex $zips 0] -d $ip_src
}
if {![file exists [file join $ip_src component.xml]]} {
  puts "LCMV_HLS: FAIL - no component.xml in $ip_src"
  exit 1
}

file mkdir $ip_repo
file delete -force $out_ip
file copy -force $ip_src $out_ip
if {[file exists $rpt_txt]} { file copy -force $rpt_txt $out_ip }

puts "LCMV_HLS: IP antsdr:gnss:lcmv_crpa:1.0 -> $out_ip"
puts "LCMV_HLS: PASS"
exit 0
