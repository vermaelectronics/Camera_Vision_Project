# ============================================================================
#  export_bd_to_source.tcl
#
#  Pushes block-design edits made in the Vivado GUI back into the authoritative
#  Tcl, so that automation and the IDE never diverge (requirements 67, 10, 11).
#
#  WORKFLOW THIS SUPPORTS
#    1. Engineer opens Vivado/Project/antsdr_e310_gnss.xpr in the GUI.
#    2. Engineer edits the block design and saves it.
#    3. Engineer runs Automation/PowerShell/Sync-BlockDesign.ps1, which calls
#       this script.
#    4. The regenerated Tcl lands in Source/BlockDesign/system_bd.generated.tcl.
#    5. Engineer DIFFS it against Source/BlockDesign/system_bd.tcl and merges
#       the intended changes by hand, preserving the GNSS-CRPA MOD markers and
#       the provenance header.
#
#  Step 5 is deliberately manual.  write_bd_tcl emits a flat, machine-shaped
#  file that loses the MOD-1/MOD-2 comments and the upstream lineage, so
#  overwriting the authoritative file automatically would destroy the record of
#  what was changed from the MicroPhase baseline (requirements 34, 35).
#
#  USAGE
#    vivado -mode batch -source export_bd_to_source.tcl -tclargs <xpr> <out_tcl>
# ============================================================================

if {$argc < 2} {
  puts "EXPORT_BD_RESULT: FAIL - usage: export_bd_to_source.tcl <xpr> <out_tcl>"
  exit 2
}

set xpr [file normalize [lindex $argv 0]]
set out [file normalize [lindex $argv 1]]

if {![file exists $xpr]} {
  puts "EXPORT_BD_RESULT: FAIL - project not found: $xpr"
  exit 2
}

open_project $xpr

set bd [get_files -quiet *.bd]
if {[llength $bd] == 0} {
  puts "EXPORT_BD_RESULT: FAIL - no block design in $xpr"
  exit 2
}

open_bd_design [lindex $bd 0]

file mkdir [file dirname $out]
if {[file exists $out]} { file delete -force $out }

write_bd_tcl -force -no_ip_version $out

if {![file exists $out]} {
  puts "EXPORT_BD_RESULT: FAIL - write_bd_tcl produced no output"
  exit 2
}

puts "EXPORT_BD_OUT: $out"
puts "EXPORT_BD_NEXT_STEP: diff this against Source/BlockDesign/system_bd.tcl and merge by hand; do not overwrite."
puts "EXPORT_BD_RESULT: PASS"
exit 0
