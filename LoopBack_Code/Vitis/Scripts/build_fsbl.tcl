# ============================================================================
#  build_fsbl.tcl   (XSCT script, Vitis 2023.2 classic flow)
#
#  Builds the Zynq-7000 First Stage Boot Loader from the XSA the FPGA build
#  produced, so ps7_init (PLLs, MIO, DDR) always matches THIS hardware.
#  Vitis 2023.2 counterpart of build_fsbl.py (which needs the 2026.1 Python API).
#
#  USAGE
#    xsct build_fsbl.tcl <xsa> <work_dir> <elf_out>
#
#  SUCCESS MARKER
#    "FSBL_RESULT: PASS" only after the ELF exists on disk.
# ============================================================================

if {$argc < 3} {
  puts "FSBL_RESULT: FAIL - usage: build_fsbl.tcl <xsa> <work_dir> <elf_out>"
  exit 2
}

set xsa     [file normalize [lindex $argv 0]]
set work    [file normalize [lindex $argv 1]]
set elf_out [file normalize [lindex $argv 2]]

if {![file exists $xsa]} {
  puts "FSBL_RESULT: FAIL - XSA not found: $xsa"
  exit 2
}

# Regenerated every run so it can never drift from the XSA.
file delete -force $work
file mkdir $work

puts "FSBL_STAGE: opening [file tail $xsa]"
if {[catch {hsi::open_hw_design $xsa} err]} {
  puts "FSBL_RESULT: FAIL - open_hw_design"
  puts "FSBL_ERROR: $err"
  exit 2
}

puts "FSBL_STAGE: generating and compiling zynq_fsbl"
if {[catch {
  hsi::generate_app -hw [hsi::current_hw_design] -os standalone \
                    -proc ps7_cortexa9_0 -app zynq_fsbl -compile \
                    -sw fsbl -dir [file join $work fsbl]
} err]} {
  puts "FSBL_RESULT: FAIL - generate_app zynq_fsbl"
  puts "FSBL_ERROR: $err"
  exit 2
}
hsi::close_hw_design [hsi::current_hw_design]

set elf [file join $work fsbl executable.elf]
if {![file exists $elf]} {
  puts "FSBL_RESULT: FAIL - generate_app reported success but $elf is absent"
  exit 2
}

file mkdir [file dirname $elf_out]
file copy -force $elf $elf_out
puts "FSBL_ELF: $elf_out"
puts "FSBL_RESULT: PASS"
exit 0
