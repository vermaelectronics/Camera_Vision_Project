# ============================================================================
#  build_hls.tcl  --  Vitis HLS 2021.1 / 2023.2: sfap_pi.cpp -> block-design IP sfap_pi
#
#  Builds the space-frequency power-inversion filter (SFAP-PI v6.0 core, see
#  sfap_pi.cpp: RX2 input normalisation, 256-bin windowed FFT filter bank with
#  50 % overlap, per-bin weight + 2 image taps read through memory ports,
#  exact bypass, output make-up gain) and packages it as a Vivado IP that
#  plutosdr-fw/hdl/projects/ant/system_bd.tcl (antsdr-fw-patch crpa15)
#  instantiates as sfap_pi_0, in the sys_cpu_clk domain (100 MHz), between
#  two clock-crossing FIFOs. The firmware v6.1 changes (weight hold, per-bin
#  detection, adaptive averaging) need no change to this core.
#
#  Steps
#    1. C simulation, scenarios 1 (core accuracy vs. a double WOLA model,
#       latency, exact bypass, saturation), 2 (wideband jammer), 3 (no jammer),
#       4 (CW), 5 (jammer + reflection), 6 (50 dB with I/Q imbalance), 7 (RX2
#       25 dB weaker), 8 (60 dB with I/Q imbalance), 9 (two CW jammers from
#       two directions), 10 (CW on the satellite, bad geometry), and the v6.1
#       capture sequences 11 (intermittent), 12 (hopping), 13 (narrowband
#       detector), 14 (drift-aware averaging), 15 (hold safety on a
#       direction change), and the v6.2 changing-channel cases 16 (obstacle
#       shadowing RX2), 17 (moving channel), 18 (multipath delay). The
#       weights come from the firmware's solver
#       (gnss_crpa/src/gnss_aj_math.c, the user's v6.2.1), compiled into the
#       testbench. Each scenario is a separate csim_design run.
#    2. C synthesis at a 10 ns target, II=1 (one sample per clock).
#    3. Checks the synthesis report: the pipeline must reach the requested II
#       and the estimated clock must not exceed the target.
#    4. export_design -> $GNSS_CRPA_IP_REPO/sfap_pi (default ../../plutosdr-fw/hdl/library/antsdr_ip)
#
#  USAGE (from this directory)
#    vitis_hls -f build_hls.tcl
#
#  OVERRIDES (environment)
#    SFAP_PI_CLK_NS=<ns>   HLS clock target, default 10 (sys_cpu_clk, 100 MHz)
#    SFAP_PI_SKIP_CSIM=1   skip step 1
#    SFAP_PI_CSIM_LIST="1 2 3"  run only these scenarios
#
#  OUTPUT
#    $GNSS_CRPA_IP_REPO/sfap_pi/component.xml, VLNV antsdr:gnss:sfap_pi:4.0
#    $GNSS_CRPA_IP_REPO/sfap_pi/sfap_pi_csynth.rpt
#
#  SUCCESS MARKER
#    Prints "SFAP_PI_HLS: PASS" only after the IP has been copied.
# ============================================================================

set here [file normalize [file dirname [info script]]]
set work [file join $here build]
if {[info exists ::env(GNSS_CRPA_IP_REPO)]} {
  set ip_repo [file normalize $::env(GNSS_CRPA_IP_REPO)]
} else {
  set ip_repo [file normalize [file join $here .. .. plutosdr-fw hdl library antsdr_ip]]
}
set out_ip [file join $ip_repo sfap_pi]
# The firmware's weight solver, compiled into the C simulation.
# antsdr-fw-patch crpa15: the solver is gnss_crpa/src (the Linux firmware, with the user's
# SFAP-PI v6.2.1 gnss_aj_math.c); build.sh passes it in SFAP_PI_FW_DIR.
if {[info exists ::env(SFAP_PI_FW_DIR)]} {
  set fw_dir [file normalize $::env(SFAP_PI_FW_DIR)]
} else {
  set fw_dir [file normalize [file join $here .. .. gnss_crpa src]]
}

set clk_period 10
if {[info exists ::env(SFAP_PI_CLK_NS)]} { set clk_period $::env(SFAP_PI_CLK_NS) }
set clk_uncertainty 1.0
set scenarios {1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18}
if {[info exists ::env(SFAP_PI_CSIM_LIST)]} { set scenarios $::env(SFAP_PI_CSIM_LIST) }
puts "SFAP_PI_HLS: HLS clock target $clk_period ns (uncertainty $clk_uncertainty ns), II=1"

file mkdir $work
cd $work
open_project -reset sfap_pi_hls
set_top sfap_pi
add_files [file join $here sfap_pi.cpp]   -cflags "-I$here"
add_files [file join $here sfap_pi.h]
add_files [file join $here sfap_tables.h]
add_files -tb [file join $here testbench.cpp] -cflags "-I$here -I$fw_dir -Wno-unknown-pragmas"
add_files -tb [file join $fw_dir gnss_aj_math.c] -cflags "-I$fw_dir"
open_solution -reset sol -flow_target vivado
set_part {xc7z020clg400-2}
create_clock -period $clk_period -name default
set_clock_uncertainty $clk_uncertainty

# ---- 1 C simulation -------------------------------------------------------------
if {![info exists ::env(SFAP_PI_SKIP_CSIM)] || $::env(SFAP_PI_SKIP_CSIM) eq "0"} {
  foreach scen $scenarios {
    puts "SFAP_PI_HLS: csim scenario $scen"
    if {[catch {csim_design -O -argv $scen} err]} {
      puts "SFAP_PI_HLS: FAIL - C simulation scenario $scen: $err"
      exit 1
    }
  }
}

# ---- 2 C synthesis ---------------------------------------------------------------
csynth_design

# ---- 3 report checks -------------------------------------------------------------
set rpt_xml [file join $work sfap_pi_hls sol syn report csynth.xml]
set rpt_txt [file join $work sfap_pi_hls sol syn report sfap_pi_csynth.rpt]
if {![file exists $rpt_xml]} {
  puts "SFAP_PI_HLS: FAIL - synthesis report $rpt_xml not found"
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
foreach res {DSP BRAM_18K FF LUT} {
  set v ""
  regexp "<Resources>.*?<${res}>(\[0-9\]+)</${res}>" $xml -> v
  puts "SFAP_PI_HLS: estimated $res = $v"
}
puts "SFAP_PI_HLS: estimated clock period $est ns (target $clk_period ns), achieved II $ii (requested 1)"
if {$est eq "" || $est > $clk_period} {
  puts "SFAP_PI_HLS: FAIL - estimated clock period '$est' ns exceeds the $clk_period ns target"
  exit 1
}
if {$ii eq "" || $ii > 1} {
  puts "SFAP_PI_HLS: FAIL - achieved II '$ii' is worse than 1"
  exit 1
}

# ---- 4 package as a Vivado IP ------------------------------------------------------
# The weights must come in through ap_memory ports (two per array); the
# block design connects them by name.
set vtop [file join $work sfap_pi_hls sol syn verilog sfap_pi.v]
if {[file exists $vtop]} {
  set fh [open $vtop r]; set vtxt [read $fh]; close $fh
  foreach arr {wb wc1 wc2} {
    foreach sig {address0 ce0 q0 address1 ce1 q1} {
      if {![regexp "\\m${arr}_${sig}\\M" $vtxt]} {
        puts "SFAP_PI_HLS: FAIL - port ${arr}_${sig} missing from the generated RTL (expected two ap_memory read ports per weight array)"
        exit 1
      }
    }
  }
  puts "SFAP_PI_HLS: weight memory ports OK (wb/wc1/wc2 x 2 read ports)"
}
export_design -format ip_catalog -rtl verilog \
  -vendor antsdr -library gnss -version 4.0 -ipname sfap_pi \
  -display_name "SFAP power inversion, 256 bins" \
  -description "Two-element space-frequency power-inversion anti-jam filter (Vitis HLS): RX2 normalised to RX1, 256-bin sqrt-Hann WOLA filter bank with 50 % overlap, per-bin weight on RX2 plus 2 image (conjugate) taps, RX1 fixed at 1, exact bypass, output make-up gain; weights from the ARM via gnss_passthrough's weight RAM (memory ports). s_axis_x = RX1 and RX2 IQ, m_axis_y = filtered RX1, latency 766 samples."

set impl_dir [file join $work sfap_pi_hls sol impl]
set ip_src   [file join $impl_dir ip]
if {![file exists [file join $ip_src component.xml]]} {
  set zips [glob -nocomplain -directory $impl_dir *.zip]
  if {[llength $zips] == 0} {
    puts "SFAP_PI_HLS: FAIL - export_design produced no IP in $impl_dir"
    exit 1
  }
  file delete -force $ip_src
  file mkdir $ip_src
  exec unzip -q -o [lindex $zips 0] -d $ip_src
}
if {![file exists [file join $ip_src component.xml]]} {
  puts "SFAP_PI_HLS: FAIL - no component.xml in $ip_src"
  exit 1
}

file mkdir $ip_repo
file delete -force $out_ip
file copy -force $ip_src $out_ip
if {[file exists $rpt_txt]} { file copy -force $rpt_txt $out_ip }

puts "SFAP_PI_HLS: IP antsdr:gnss:sfap_pi:4.0 -> $out_ip"
puts "SFAP_PI_HLS: PASS"
exit 0
