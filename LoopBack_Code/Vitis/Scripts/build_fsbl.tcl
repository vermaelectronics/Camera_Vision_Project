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

# FSBL boot messages on the console (default ON). They show "Xilinx First
# Stage Boot Loader", the boot mode and each partition it loads, which tells a
# boot failure apart from a firmware or console problem. FSBL_DEBUG_INFO=0 in
# the environment builds the silent FSBL instead.
set fsbl_dir   [file join $work fsbl]
set fsbl_debug [expr {![info exists ::env(FSBL_DEBUG_INFO)] || $::env(FSBL_DEBUG_INFO) ne "0"}]

puts "FSBL_STAGE: generating zynq_fsbl"
if {[catch {
  hsi::generate_app -hw [hsi::current_hw_design] -os standalone \
                    -proc ps7_cortexa9_0 -app zynq_fsbl \
                    -sw fsbl -dir $fsbl_dir
} err]} {
  puts "FSBL_RESULT: FAIL - generate_app zynq_fsbl"
  puts "FSBL_ERROR: $err"
  exit 2
}
hsi::close_hw_design [hsi::current_hw_design]

if {$fsbl_debug} {
  set dbg_h [file join $fsbl_dir fsbl_debug.h]
  if {[file exists $dbg_h]} {
    set fh [open $dbg_h r]; set txt [read $fh]; close $fh
    set fh [open $dbg_h w]
    puts $fh "#define FSBL_DEBUG_INFO   /* added by build_fsbl.tcl: boot messages on UART */"
    puts -nonewline $fh $txt
    close $fh
    puts "FSBL_DEBUG_INFO: enabled (boot messages on the console)"
  } else {
    puts "FSBL_DEBUG_INFO: WARNING fsbl_debug.h not found, FSBL stays silent"
  }
}

puts "FSBL_STAGE: compiling"
if {[catch {exec make -C $fsbl_dir >@ stdout 2>@ stderr} err]} {
  puts "FSBL_RESULT: FAIL - make zynq_fsbl"
  puts "FSBL_ERROR: $err"
  exit 2
}

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
