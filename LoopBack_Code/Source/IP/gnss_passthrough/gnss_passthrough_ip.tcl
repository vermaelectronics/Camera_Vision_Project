# ============================================================================
#  gnss_passthrough_ip.tcl
#  Packages Source/HDL/gnss_passthrough.v as a Vivado IP so the block design
#  can instantiate it (requirement 41).
#
#  This script is deliberately self-contained: it uses only standard Vivado
#  IP-packager commands and does NOT depend on the Analog Devices library
#  scripts.  That keeps the custom IP buildable even if the vendor tree is
#  refreshed to a different upstream revision.
#
#  INVOCATION
#    vivado -mode batch -source gnss_passthrough_ip.tcl -tclargs <src> <out>
#      <src> absolute path to code_r1/Source
#      <out> absolute path to the IP repository to write into
#
#  OUTPUT
#    <out>/gnss_passthrough/component.xml  plus the packaged sources.
#
#  The resulting VLNV is:  antsdr:gnss:gnss_passthrough:1.0
# ============================================================================

if {$argc < 2} {
  puts "ERROR: expected 2 arguments: <source_dir> <ip_repo_dir>"
  exit 2
}

set src_dir    [file normalize [lindex $argv 0]]
set ip_repo_dir [file normalize [lindex $argv 1]]
set ip_name    "gnss_passthrough"
set ip_dir     [file join $ip_repo_dir $ip_name]
set rtl_file   [file join $src_dir "HDL" "${ip_name}.v"]
set xdc_file   [file join $src_dir "IP" $ip_name "${ip_name}_constr.xdc"]

foreach f [list $rtl_file $xdc_file] {
  if {![file exists $f]} {
    puts "ERROR: required source not found: $f"
    exit 2
  }
}

# A fresh edit-project guarantees the packaged result reflects the current RTL
# and never a stale cache (requirement 66).
file delete -force $ip_dir
file mkdir $ip_dir

create_project -force ${ip_name}_pkg [file join $ip_dir ".pkg_project"] -part xc7z020clg400-2
set_property target_language Verilog [current_project]

add_files -norecurse $rtl_file
set_property top $ip_name [current_fileset]
update_compile_order -fileset sources_1

# Scoped CDC constraints, packaged with the IP so they apply wherever it is
# instantiated. PROCESSING_ORDER LATE ensures they are applied after the
# design's own clock definitions exist.
add_files -norecurse -fileset constrs_1 $xdc_file
set_property USED_IN {synthesis implementation out_of_context} [get_files $xdc_file]
set_property PROCESSING_ORDER LATE [get_files $xdc_file]

ipx::package_project -root_dir $ip_dir -vendor antsdr -library gnss \
                     -taxonomy /UserIP -import_files -force

set core [ipx::current_core]

set_property name           $ip_name                                  $core
set_property version        1.0                                       $core
set_property display_name   "GNSS CRPA Passthrough"                   $core
set_property description    "Phase-1 transparent RX->TX I/Q passthrough and CRPA insertion point for ANTSDR E310 V1" $core
set_property vendor_display_name "ANTSDR GNSS CRPA project"           $core
set_property company_url    "https://github.com/MicroPhase"           $core

# ---------------------------------------------------------------------------
#  Clock / reset association.
#  ad_cpu_interconnect locates the AXI clock by matching CONFIG.ASSOCIATED_BUSIF
#  against the slave interface name, so this must be set explicitly rather than
#  left to inference.  A bus parameter has to be created before it can be given
#  a value, hence the helper below.
# ---------------------------------------------------------------------------
proc ensure_bus_if {core name vlnv} {
  if {[llength [ipx::get_bus_interfaces $name -of_objects $core]] == 0} {
    ipx::infer_bus_interface $name $vlnv $core
  }
  return [ipx::get_bus_interfaces $name -of_objects $core]
}

proc set_bus_param {bif name value} {
  if {[llength [ipx::get_bus_parameters $name -of_objects $bif]] == 0} {
    ipx::add_bus_parameter $name $bif
  }
  set_property value $value [ipx::get_bus_parameters $name -of_objects $bif]
}

set clk_if [ensure_bus_if $core s_axi_aclk    xilinx.com:signal:clock_rtl:1.0]
set rst_if [ensure_bus_if $core s_axi_aresetn xilinx.com:signal:reset_rtl:1.0]

set_bus_param $clk_if ASSOCIATED_BUSIF s_axi
set_bus_param $clk_if ASSOCIATED_RESET s_axi_aresetn
set_bus_param $rst_if POLARITY         ACTIVE_LOW

# The sample-domain clock is a separate, asynchronous clock and must NOT be
# associated with the AXI interface.
#
# Vivado's packager tends to auto-associate every clock it finds with the AXI
# bus interface. If that is left in place, ad_cpu_interconnect's lookup
#     get_bd_pins -filter "TYPE == clk && CONFIG.ASSOCIATED_BUSIF == s_axi"
# matches BOTH clocks, get_property then returns a two-element reset list, and
# the next filter becomes the malformed 'NAME == rst s_axi_aresetn'.
# Clearing ASSOCIATED_BUSIF here is what keeps that lookup unambiguous.
set sclk_if [ensure_bus_if $core clk xilinx.com:signal:clock_rtl:1.0]
set srst_if [ensure_bus_if $core rst xilinx.com:signal:reset_rtl:1.0]
set_bus_param $srst_if POLARITY         ACTIVE_HIGH
set_bus_param $sclk_if ASSOCIATED_RESET rst
set_bus_param $sclk_if ASSOCIATED_BUSIF ""

# ---------------------------------------------------------------------------
#  Memory map: 4 kB aperture.  The actual base address is assigned by the
#  block design (Source/BlockDesign/system_bd.tcl), not here.
# ---------------------------------------------------------------------------
ipx::add_memory_map s_axi $core
set mmap [ipx::get_memory_maps s_axi -of_objects $core]
set_property slave_memory_map_ref s_axi [ipx::get_bus_interfaces s_axi -of_objects $core]
ipx::add_address_block axi_lite $mmap
set ablk [ipx::get_address_blocks axi_lite -of_objects $mmap]
set_property base_address 0            $ablk
set_property range        4096         $ablk
set_property width        32           $ablk
set_property usage        register     $ablk

ipx::create_xgui_files $core
ipx::update_checksums  $core
ipx::check_integrity   $core
ipx::save_core         $core

# ---------------------------------------------------------------------------
#  Self-check: exactly one clock may claim the s_axi interface.
#  If this ever regresses, the build fails here with a clear message instead of
#  much later inside ad_cpu_interconnect with a malformed filter expression.
# ---------------------------------------------------------------------------
set claimants {}
foreach bif [ipx::get_bus_interfaces -of_objects $core] {
  set bname [get_property name $bif]
  set p [ipx::get_bus_parameters ASSOCIATED_BUSIF -of_objects $bif]
  if {[llength $p] == 0} { continue }
  if {[get_property value $p] eq "s_axi"} { lappend claimants $bname }
}
if {[llength $claimants] != 1 || [lindex $claimants 0] ne "s_axi_aclk"} {
  puts "IP_PACKAGE_SELFCHECK: FAIL - expected exactly s_axi_aclk to be associated\
        with the s_axi interface, got: '$claimants'"
  close_project
  exit 2
}
puts "IP_PACKAGE_SELFCHECK: OK - s_axi is claimed only by s_axi_aclk"

# The CDC constraints are what keep the design timing-clean. If they silently
# failed to package, the only symptom would be a setup violation much later.
set xdc_packaged 0
foreach fg [ipx::get_file_groups -of_objects $core] {
  foreach fl [ipx::get_files -of_objects $fg] {
    if {[string match "*${ip_name}_constr.xdc" [get_property name $fl]]} { set xdc_packaged 1 }
  }
}
if {!$xdc_packaged} {
  puts "IP_PACKAGE_SELFCHECK: FAIL - ${ip_name}_constr.xdc was not packaged into the IP"
  close_project
  exit 2
}
puts "IP_PACKAGE_SELFCHECK: OK - CDC constraints packaged with the IP"

close_project
file delete -force [file join $ip_dir ".pkg_project"]

puts "IP_PACKAGE_OK: $ip_dir/component.xml"
