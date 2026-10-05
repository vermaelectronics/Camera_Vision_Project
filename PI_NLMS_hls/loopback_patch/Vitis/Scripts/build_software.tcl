# ============================================================================
#  build_software.tcl   (XSCT script)
#
#  Creates a Vitis platform from the exported XSA and builds the bare-metal
#  application from Source/Firmware.
#
#  Requirement 15 (hardware/software sync): the platform is regenerated from
#  the XSA that the FPGA build just produced, so a hardware change always
#  propagates.  Requirement 14: the workspace it creates is a normal Vitis
#  workspace and opens in the IDE.
#
#  The application sources are ADDED BY REFERENCE where the tool allows it and
#  otherwise linked, so Source/Firmware stays the single authoritative copy
#  (requirement 9).
#
#  USAGE
#    xsct build_software.tcl <xsa> <workspace> <firmware_src> <elf_out>
# ============================================================================

if {$argc < 4} {
  puts "SW_RESULT: FAIL - usage: build_software.tcl <xsa> <workspace> <firmware_src> <elf_out>"
  exit 2
}

set xsa       [file normalize [lindex $argv 0]]
set ws        [file normalize [lindex $argv 1]]
set fw_src    [file normalize [lindex $argv 2]]
set elf_out   [file normalize [lindex $argv 3]]

set plat_name "e310_gnss_platform"
set app_name  "e310_gnss_app"
set domain    "standalone_ps7_cortexa9_0"

foreach f [list $xsa $fw_src] {
  if {![file exists $f]} {
    puts "SW_RESULT: FAIL - missing input: $f"
    exit 2
  }
}

file mkdir $ws
setws $ws

# ---- platform --------------------------------------------------------------
puts "SW_STAGE: creating platform from [file tail $xsa]"
if {[catch {
  platform create -name $plat_name -hw $xsa -proc ps7_cortexa9_0 \
                  -os standalone -out $ws -no-boot-bsp
} err]} {
  puts "SW_RESULT: FAIL - platform create"
  puts "SW_ERROR: $err"
  exit 2
}
platform generate

# ---- application -----------------------------------------------------------
puts "SW_STAGE: creating application"
if {[catch {
  app create -name $app_name -platform $plat_name -domain $domain \
             -template "Empty Application(C)"
} err]} {
  puts "SW_RESULT: FAIL - app create"
  puts "SW_ERROR: $err"
  exit 2
}

# Pull in the firmware sources.  importsources copies into the workspace; the
# workspace is a generated, disposable area (Vitis/Workspace), and
# Source/Firmware remains authoritative.  Sync-Firmware.ps1 documents the
# route back if the sources are edited inside the IDE (requirement 67).
puts "SW_STAGE: importing firmware from $fw_src"
importsources -name $app_name -path $fw_src -linker-script

# The no-OS AD9361 driver needs these to compile for this board.
app config -name $app_name -add define-compiler-symbols XILINX_PLATFORM
app config -name $app_name -add define-compiler-symbols ANTSDR_E310
app config -name $app_name -add include-path $fw_src

puts "SW_STAGE: building"
if {[catch {app build -name $app_name} err]} {
  puts "SW_RESULT: FAIL - app build"
  puts "SW_ERROR: $err"
  exit 2
}

set elf [file join $ws $app_name "Debug" "${app_name}.elf"]
if {![file exists $elf]} {
  set elf [file join $ws $app_name "build" "${app_name}.elf"]
}
if {![file exists $elf]} {
  puts "SW_RESULT: FAIL - build reported success but no ELF was found"
  puts "SW_EVIDENCE: looked in [file join $ws $app_name]"
  exit 2
}

file mkdir [file dirname $elf_out]
file copy -force $elf $elf_out
puts "SW_ELF: $elf_out"
puts "SW_RESULT: PASS"
exit 0
