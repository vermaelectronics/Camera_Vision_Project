# ============================================================================
#  package_library_ip.tcl
#
#  Packages one Analog Devices library IP core.
#
#  WHY THIS EXISTS
#    The upstream ADI flow packages its IP library with GNU make
#    (projects/scripts/project-xilinx.mk).  make is not part of a standard
#    Windows + Vivado installation, so this project drives the same per-library
#    "<lib>_ip.tcl" scripts directly through Vivado instead.  The packaging
#    logic is the vendor's own; only the driver is different.
#
#  WHERE IT WRITES
#    adi_ip_create does "create_project <name> ." -- it packages into the
#    current directory.  The caller therefore cd's into the library directory
#    of the DISPOSABLE working copy of the vendor tree
#    (Build/VendorWork/hdl/library/...), never into Vendor/, which stays
#    pristine and read-only (requirements 34, 71).
#
#  USAGE
#    vivado -mode batch -source package_library_ip.tcl -tclargs <lib_rel_path>
#    with the working directory set to the library folder itself.
# ============================================================================

if {$argc < 1} {
  puts "LIBIP_RESULT: FAIL - usage: package_library_ip.tcl <library_relative_path>"
  exit 2
}

set lib_rel  [lindex $argv 0]
set lib_name [file tail $lib_rel]
set ip_tcl   "${lib_name}_ip.tcl"

if {![file exists $ip_tcl]} {
  puts "LIBIP_RESULT: FAIL - $ip_tcl not found in [pwd]"
  exit 2
}

# The vendor _ip.tcl files do "source ../scripts/adi_env.tcl", which resolves
# relative to this library directory inside the working copy.
if {[catch {source $ip_tcl} err]} {
  puts "LIBIP_RESULT: FAIL - $lib_rel"
  puts "LIBIP_ERROR: $err"
  exit 2
}

if {![file exists "component.xml"]} {
  puts "LIBIP_RESULT: FAIL - $lib_rel produced no component.xml in [pwd]"
  exit 2
}

puts "LIBIP_OK: $lib_rel"
puts "LIBIP_RESULT: PASS"
exit 0
