# ============================================================================
#  build_boot_2021.tcl   (XSCT, Vitis 2021.1)
#
#  Builds the Zynq-7000 FSBL from the XSA and packs FSBL + bitstream + the
#  application ELF into BOOT.BIN for SD-card boot. 2021.1 counterpart of
#  build_fsbl.py, which needs the Vitis 2026 Python API.
#
#  USAGE
#    xsct build_boot_2021.tcl <xsa> <bit> <app_elf> <out_dir>
#
#  OUTPUT
#    <out_dir>/fsbl/executable.elf, <out_dir>/boot.bif, <out_dir>/BOOT.BIN
#
#  SUCCESS MARKER
#    Prints "BOOT_RESULT: PASS" only after BOOT.BIN exists on disk.
# ============================================================================

if {$argc < 4} {
  puts "BOOT_RESULT: FAIL - usage: build_boot_2021.tcl <xsa> <bit> <app_elf> <out_dir>"
  exit 2
}
set xsa     [file normalize [lindex $argv 0]]
set bit     [file normalize [lindex $argv 1]]
set app_elf [file normalize [lindex $argv 2]]
set out_dir [file normalize [lindex $argv 3]]

foreach f [list $xsa $bit $app_elf] {
  if {![file exists $f]} {
    puts "BOOT_RESULT: FAIL - missing input: $f"
    exit 2
  }
}

file delete -force $out_dir
file mkdir $out_dir

# ---- FSBL ------------------------------------------------------------------
puts "BOOT_STAGE: building FSBL from [file tail $xsa]"
set fsbl_dir [file join $out_dir fsbl]
if {[catch {
  set hw [hsi::open_hw_design $xsa]
  hsi::generate_app -hw $hw -os standalone -proc ps7_cortexa9_0 \
                    -app zynq_fsbl -sw fsbl -dir $fsbl_dir -compile
  hsi::close_hw_design $hw
} err]} {
  puts "BOOT_RESULT: FAIL - FSBL generation"
  puts "BOOT_ERROR: $err"
  exit 2
}
set fsbl_elf [file join $fsbl_dir executable.elf]
if {![file exists $fsbl_elf]} {
  puts "BOOT_RESULT: FAIL - FSBL build reported success but $fsbl_elf is absent"
  exit 2
}

# ---- BOOT.BIN --------------------------------------------------------------
set bif [file join $out_dir boot.bif]
set fh [open $bif w]
puts $fh "the_ROM_image:"
puts $fh "\{"
puts $fh "  \[bootloader\] $fsbl_elf"
puts $fh "  $bit"
puts $fh "  $app_elf"
puts $fh "\}"
close $fh

set boot_bin [file join $out_dir BOOT.BIN]
puts "BOOT_STAGE: bootgen"
if {[catch {exec bootgen -arch zynq -image $bif -o $boot_bin -w on} out]} {
  # bootgen writes its banner to stderr; judge by the output file.
  puts $out
}
if {![file exists $boot_bin]} {
  puts "BOOT_RESULT: FAIL - bootgen did not produce $boot_bin"
  exit 2
}
puts "BOOT_BIN: $boot_bin"
puts "BOOT_RESULT: PASS"
exit 0
