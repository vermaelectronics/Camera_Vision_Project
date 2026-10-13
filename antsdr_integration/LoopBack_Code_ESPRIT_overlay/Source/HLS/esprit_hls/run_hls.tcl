# ============================================================================
#  run_hls.tcl -- build one ESPRIT HLS IP with Vitis HLS (2023.2 or later)
#
#  USAGE (from any working directory; the HLS project is created there):
#      vitis_hls -f <LoopBack_Code>/Source/HLS/esprit_hls/run_hls.tcl \
#                -tclargs <mesprit|iuesprit> <hls_ip_repo_dir> [cosim]
#
#      mesprit   -> mesprit_hls   (M-ESPRIT)
#      iuesprit  -> iuesprit_hls  (I-UESPRIT)
#      (the short forms m / iu are accepted too)
#      <hls_ip_repo_dir>  e.g. <LoopBack_Code>/Build/hls_repo
#                         the IP is unpacked to <dir>/<algo>_hls/
#      cosim (optional)   also run RTL/C co-simulation (slower)
#
#  Steps: C simulation (tb_esprit_hls.cpp) -> C synthesis at 100 MHz for
#  xc7z020clg400-2 -> [co-simulation] -> export to the Vivado IP catalog
#  (VLNV antsdr:hls:mesprit_hls:1.0 / antsdr:hls:iuesprit_hls:1.0).
#
#  Console markers:
#      HLS_CSIM:   PASS | FAIL
#      HLS_SYNTH:  PASS | FAIL   (+ report path)
#      HLS_COSIM:  PASS | FAIL | skipped
#      HLS_EXPORT: <dir>/<algo>_hls/component.xml
#      HLS_RESULT: PASS | FAIL
# ============================================================================
set here [file dirname [file normalize [info script]]]

# vitis_hls puts its own options (-f <script> ...) into argv ahead of the
# -tclargs values on some releases, so locate the arguments instead of
# assuming positions: everything after -tclargs if present, then the first
# mesprit/iuesprit (or m/iu) token is the algorithm, the next the repository.
# Environment fallback: ESPRIT_HLS_ALGO, ESPRIT_HLS_REPO.
set args $argv
set ti [lsearch -exact $args -tclargs]
if {$ti >= 0} { set args [lrange $args [expr {$ti + 1}] end] }
set ai -1
for {set i 0} {$i < [llength $args]} {incr i} {
  if {[string tolower [lindex $args $i]] in {mesprit iuesprit m iu}} { set ai $i; break }
}
if {$ai >= 0 && [llength $args] > $ai + 1} {
  set algo [string tolower [lindex $args $ai]]
  set repo [file normalize [lindex $args [expr {$ai + 1}]]]
} elseif {[info exists ::env(ESPRIT_HLS_ALGO)] && [info exists ::env(ESPRIT_HLS_REPO)]} {
  set algo [string tolower $::env(ESPRIT_HLS_ALGO)]
  set repo [file normalize $::env(ESPRIT_HLS_REPO)]
} else {
  puts "HLS_RESULT: FAIL - usage: vitis_hls -f run_hls.tcl -tclargs <mesprit|iuesprit> <hls_ip_repo_dir> \[cosim\]"
  puts "HLS_RESULT: (or set ESPRIT_HLS_ALGO=mesprit|iuesprit and ESPRIT_HLS_REPO=<dir>)  argv was: $argv"
  exit 2
}
set cosim [expr {[lsearch -exact $args cosim] >= 0}]

switch -- $algo {
  iu - iuesprit { set algo iuesprit; set tb_algo 0; set disp "I-UESPRIT DOA (ESPRIT, 2 elements)" }
  m  - mesprit  { set algo mesprit;  set tb_algo 1; set disp "M-ESPRIT DOA (ESPRIT, 2 elements)" }
  default { puts "HLS_RESULT: FAIL - algorithm '$algo': use mesprit or iuesprit"; exit 2 }
}
set top "${algo}_hls"
puts "HLS_STAGE: building $top ($disp) into $repo"

open_project -reset "hls_${top}"
set_top $top
add_files "$here/${top}.cpp" -cflags "-I$here"
add_files "$here/esprit_core.h"
add_files "$here/esprit_hls.h"
add_files -tb "$here/tb_esprit_hls.cpp" -cflags "-I$here -DTB_ALGO=$tb_algo"

open_solution -reset "sol1" -flow_target vivado
set_part xc7z020clg400-2
create_clock -period 10 -name default
config_export -format ip_catalog -vendor antsdr -library hls -version 1.0 \
              -display_name $disp -ipname $top

# ---- C simulation ------------------------------------------------------------
if {[catch {csim_design} err]} {
  puts "HLS_CSIM: FAIL - $err"
  puts "HLS_RESULT: FAIL"
  exit 1
}
puts "HLS_CSIM: PASS"

# ---- C synthesis ------------------------------------------------------------
if {[catch {csynth_design} err]} {
  puts "HLS_SYNTH: FAIL - $err"
  puts "HLS_RESULT: FAIL"
  exit 1
}
set rpt [file normalize "hls_${top}/sol1/syn/report/${top}_csynth.rpt"]
puts "HLS_SYNTH: PASS - report $rpt"
if {[file exists $rpt]} {
  set fh [open $rpt r]; set txt [read $fh]; close $fh
  foreach l [split $txt "\n"] {
    if {[regexp {^\|\s*(Total|Utilization \(%\))\s*\|} $l]} { puts "HLS_UTIL: [string trim $l]" }
  }
}

# ---- co-simulation (optional) --------------------------------------------------
if {$cosim} {
  if {[catch {cosim_design -rtl verilog} err]} {
    puts "HLS_COSIM: FAIL - $err"
    puts "HLS_RESULT: FAIL"
    exit 1
  }
  puts "HLS_COSIM: PASS"
} else {
  puts "HLS_COSIM: skipped (add 'cosim' to run it)"
}

# ---- export to the IP catalog ------------------------------------------------
file mkdir $repo
set zip [file join $repo "${top}.zip"]
if {[catch {export_design -format ip_catalog -output $zip} err]} {
  puts "HLS_EXPORT: FAIL - $err"
  puts "HLS_RESULT: FAIL"
  exit 1
}
set ipdir [file join $repo $top]
file delete -force $ipdir
file mkdir $ipdir
if {[catch {exec unzip -o -q $zip -d $ipdir} err]} {
  puts "HLS_EXPORT: FAIL - could not unzip $zip: $err"
  puts "HLS_RESULT: FAIL"
  exit 1
}
if {![file exists [file join $ipdir component.xml]]} {
  puts "HLS_EXPORT: FAIL - $ipdir/component.xml missing after export"
  puts "HLS_RESULT: FAIL"
  exit 1
}
puts "HLS_EXPORT: [file join $ipdir component.xml]"
puts "HLS_RESULT: PASS"
exit 0
