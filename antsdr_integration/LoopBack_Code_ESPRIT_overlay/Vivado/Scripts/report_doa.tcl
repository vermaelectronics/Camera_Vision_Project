# ============================================================================
#  report_doa.tcl -- post-implementation checks of the ESPRIT DOA block,
#                    printed to the VIVADO TCL CONSOLE.
#
#  USAGE: open the project, implement it (or run build.tcl), then
#      open_run impl_1          ;# if not already open
#      source <LoopBack_Code>/Vivado/Scripts/report_doa.tcl
#  Also works after synthesis (open_run synth_1) for utilisation.
#
#  Prints DOA_IMPL: lines: utilisation of u_doa and of each estimator, the
#  worst setup/hold slack of paths into the DOA block, that the CDC
#  constraints of Source/XDC/gnss_passthrough_cdc.xdc section 4 found their
#  cells, and the overall WNS/WHS. Writes Build/reports/doa_*.rpt.
# ============================================================================
set root [file normalize [file join [file dirname [info script]] .. ..]]
set out  [file join $root Build reports]
# one report folder per project (Project, Project_IUESPRIT, Project_MESPRIT)
if {[llength [current_project -quiet]]} {
  set out [file join $out [file tail [get_property DIRECTORY [current_project]]]]
}
file mkdir $out

# Locate the DOA block. Try the expected path first, then any instance of the
# gnss_doa_esprit module (REF_NAME / ORIG_REF_NAME), so a different block-design
# hierarchy or cell name still works.
set doa [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough*/u_doa}]
if {[llength $doa] == 0} {
  set doa [get_cells -quiet -hier -filter {REF_NAME =~ *gnss_doa_esprit* || ORIG_REF_NAME =~ *gnss_doa_esprit*}]
}
if {[llength $doa] == 0} {
  set doa [get_cells -quiet -hier -filter {NAME =~ */u_doa && !IS_PRIMITIVE}]
}
if {[llength $doa] == 0} {
  puts "DOA_IMPL: FAIL - no gnss_doa_esprit (u_doa) instance in the open design."
  puts "DOA_IMPL: ---- diagnosis ----"
  if {[llength [current_project -quiet]]} {
    foreach r [get_runs -quiet] {
      if {[string match *gnss_passthrough* $r] || $r eq "synth_1" || $r eq "impl_1"} {
        puts [format "DOA_IMPL: run %-36s STATUS %s" $r [get_property STATUS $r]]
      }
    }
  }
  puts "DOA_IMPL: open design: [current_design -quiet]"
  set gp [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough* && !IS_PRIMITIVE}]
  puts "DOA_IMPL: hierarchical cells named *gnss_passthrough*: [llength $gp]"
  foreach c [lrange $gp 0 7] {
    puts "DOA_IMPL:   $c  (REF_NAME [get_property REF_NAME $c], ORIG_REF_NAME [get_property ORIG_REF_NAME $c])"
  }
  set prims [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough* && IS_PRIMITIVE}]
  set ndsp  [llength [filter -quiet $prims {REF_NAME =~ DSP48*}]]
  set ndoa  [llength [get_cells -quiet -hier -filter {NAME =~ *doa*}]]
  puts "DOA_IMPL: primitives under gnss_passthrough: [llength $prims], DSP48: $ndsp (v1.2: tens of DSP48 per estimator)"
  puts "DOA_IMPL: cells whose name contains 'doa': $ndoa"
  if {$ndoa > 0 && $ndsp > 20} {
    puts "DOA_IMPL: the DOA logic is present but its hierarchy was flattened."
  } elseif {[llength $gp] == 0} {
    puts "DOA_IMPL: gnss_passthrough itself is missing: open_run opened the wrong run, or synthesis failed."
  } else {
    puts "DOA_IMPL: gnss_passthrough has no DOA logic: the build used the old v1.1 IP (stale ip_repo or IP cache)."
  }
  return
}
set doa [lindex $doa 0]
puts "DOA_IMPL: ===== ESPRIT DOA block: [get_property NAME $doa] ====="
puts "DOA_IMPL: estimators built: [expr {[llength [get_cells -quiet "[get_property NAME $doa]/g_iuesprit.u_core"]] ? "I-UESPRIT " : ""}][expr {[llength [get_cells -quiet "[get_property NAME $doa]/g_mesprit.u_core"]] ? "M-ESPRIT" : ""}]"

report_utilization -hierarchical -hierarchical_depth 4 -cells $doa -file [file join $out doa_utilization.rpt]
foreach sub {"" "/g_iuesprit.u_core" "/g_mesprit.u_core" "/u_afifo" "/u_tgen"} {
  set c [get_cells -quiet "[get_property NAME $doa]$sub"]
  if {[llength $c] == 0} { continue }
  set prims [get_cells -quiet -hier -filter "NAME =~ [get_property NAME $c]/* && IS_PRIMITIVE"]
  set lut  [llength [filter -quiet $prims {REF_NAME =~ LUT*}]]
  set ff   [llength [filter -quiet $prims {REF_NAME =~ FD*}]]
  set dsp  [llength [filter -quiet $prims {REF_NAME =~ DSP48*}]]
  set bram [llength [filter -quiet $prims {REF_NAME =~ RAMB*}]]
  set lram [llength [filter -quiet $prims {REF_NAME =~ RAM32* || REF_NAME =~ RAM64*}]]
  puts [format "DOA_IMPL: %-28s LUT %6d  FF %6d  DSP48 %4d  RAMB %3d  LUTRAM %4d" \
        [expr {$sub eq "" ? "u_doa (total)" : [string trimleft $sub /]}] $lut $ff $dsp $bram $lram]
}

# HLS variant (GNSS_DOA_IMPL=hls): the estimator is its own block-design cell
set hls [get_cells -quiet -hier -filter {NAME =~ */*esprit_hls_0 && !IS_PRIMITIVE}]
foreach h $hls {
  set prims [get_cells -quiet -hier -filter "NAME =~ [get_property NAME $h]/* && IS_PRIMITIVE"]
  set lut  [llength [filter -quiet $prims {REF_NAME =~ LUT*}]]
  set ff   [llength [filter -quiet $prims {REF_NAME =~ FD*}]]
  set dsp  [llength [filter -quiet $prims {REF_NAME =~ DSP48*}]]
  set bram [llength [filter -quiet $prims {REF_NAME =~ RAMB*}]]
  set lram [llength [filter -quiet $prims {REF_NAME =~ RAM32* || REF_NAME =~ RAM64*}]]
  puts "DOA_IMPL: estimator is a Vitis HLS IP: [get_property NAME $h]"
  puts [format "DOA_IMPL: %-28s LUT %6d  FF %6d  DSP48 %4d  RAMB %3d  LUTRAM %4d" \
        "[file tail [get_property NAME $h]] (HLS)" $lut $ff $dsp $bram $lram]
}
if {[llength $hls] == 0 && [llength [get_cells -quiet "[get_property NAME $doa]/g_iuesprit.u_core"]] == 0 \
    && [llength [get_cells -quiet "[get_property NAME $doa]/g_mesprit.u_core"]] == 0} {
  puts "DOA_IMPL: WARNING - neither an RTL estimator core nor an *esprit_hls_0 cell was found"
}

# constraint coverage (empty pattern = constraint silently unused)
foreach {what pat} {
  "FIFO write gray pointer"    *u_doa/u_afifo/wgray_reg*
  "FIFO gray sync (w2r)"       *u_doa/u_afifo/gs_meta_w2r_reg*
  "FIFO read gray pointer"     *u_doa/u_afifo/rgray_reg*
  "FIFO gray sync (r2w)"       *u_doa/u_afifo/gs_meta_r2w_reg*
  "FIFO storage (LUTRAM)"      *u_doa/u_afifo/mem_reg*
  "FIFO read register"         *u_doa/u_afifo/rd_data_reg*
  "control synchroniser"       *u_doa/dsync_meta_reg*
} {
  set n [llength [get_cells -quiet -hier -filter "NAME =~ $pat"]]
  puts [format "DOA_IMPL: CDC cells %-26s %4d  %s" $what $n [expr {$n > 0 ? "OK" : "MISSING - constraint has no effect"}]]
}

# timing into the DOA block
set into [get_cells -quiet -hier -filter "NAME =~ [get_property NAME $doa]/* && IS_SEQUENTIAL"]
if {[llength $into] > 0} {
  set ps [get_timing_paths -quiet -to $into -setup -max_paths 1]
  set ph [get_timing_paths -quiet -to $into -hold  -max_paths 1]
  if {[llength $ps]} { puts "DOA_IMPL: worst setup slack into u_doa: [get_property SLACK $ps] ns ([get_property GROUP $ps])" }
  if {[llength $ph]} { puts "DOA_IMPL: worst hold  slack into u_doa: [get_property SLACK $ph] ns" }
  report_timing -to $into -max_paths 20 -file [file join $out doa_timing.rpt] -quiet
}
set wns [get_property SLACK [get_timing_paths -quiet -setup -max_paths 1]]
set whs [get_property SLACK [get_timing_paths -quiet -hold  -max_paths 1]]
puts "DOA_IMPL: design WNS $wns ns, WHS $whs ns -> [expr {($wns >= 0 && $whs >= 0) ? "TIMING MET" : "TIMING VIOLATED"}]"
report_cdc -details -file [file join $out doa_cdc.rpt] -quiet
puts "DOA_IMPL: reports written to $out (doa_utilization.rpt, doa_timing.rpt, doa_cdc.rpt)"
