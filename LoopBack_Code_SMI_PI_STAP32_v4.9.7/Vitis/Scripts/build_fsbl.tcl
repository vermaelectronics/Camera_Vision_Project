# ============================================================================
#  build_fsbl.tcl   (XSCT script)  --  DEFAULT FSBL flow for 2023.2
#
#  Builds the Zynq-7000 First Stage Boot Loader for the E310 V1 from the XSA
#  the FPGA build produced.  The FSBL brings up PLLs, MIO, clocks and DDR from
#  this design's ps7_init data, then loads the bitstream and the application
#  out of BOOT.BIN (see build_fsbl.py for the full background).
#
#  On 2023.2 the classic "Zynq FSBL" template is available directly, so unlike
#  the Python flow no separate boot domain has to be harvested.
#
#  USAGE
#    xsct build_fsbl.tcl <xsa> <workspace> <elf_out>
#
#  SUCCESS MARKER
#    Prints "FSBL_RESULT: PASS" only after an ELF actually exists on disk.
# ============================================================================

if {$argc < 3} {
  puts "FSBL_RESULT: FAIL - usage: build_fsbl.tcl <xsa> <workspace> <elf_out>"
  exit 2
}

set xsa     [file normalize [lindex $argv 0]]
set ws      [file normalize [lindex $argv 1]]
set elf_out [file normalize [lindex $argv 2]]
set app     "zynq_fsbl"

if {![file exists $xsa]} {
  puts "FSBL_RESULT: FAIL - XSA not found: $xsa"
  exit 2
}

if {[file exists $ws]} { file delete -force $ws }
file mkdir $ws
setws $ws

puts "FSBL_STAGE: creating FSBL application from [file tail $xsa]"
if {[catch {
  app create -name $app -hw $xsa -proc ps7_cortexa9_0 -os standalone \
             -template "Zynq FSBL"
} err]} {
  puts "FSBL_RESULT: FAIL - app create"
  puts "FSBL_ERROR: $err"
  exit 2
}

puts "FSBL_STAGE: building"
if {[catch {app build -name $app} err]} {
  puts "FSBL_RESULT: FAIL - app build"
  puts "FSBL_ERROR: $err"
  exit 2
}

set elf ""
foreach cand [list [file join $ws $app Debug "${app}.elf"] \
                   [file join $ws $app Release "${app}.elf"]] {
  if {[file exists $cand]} { set elf $cand; break }
}
if {$elf eq ""} {
  puts "FSBL_RESULT: FAIL - build reported success but no fsbl ELF was found"
  puts "FSBL_EVIDENCE: looked in [file join $ws $app]"
  exit 2
}

file mkdir [file dirname $elf_out]
file copy -force $elf $elf_out
puts "FSBL_ELF_SOURCE: $elf"
puts "FSBL_ELF: $elf_out ([file size $elf_out] bytes)"
puts "FSBL_RESULT: PASS"
exit 0
