# ============================================================================
#  pl_npi_ip.tcl
#  Packages the PL-NPI CRPA core (Source/HDL/pl_npi) as a Vivado IP so the
#  block design can instantiate it as its own cell, pl_npi_0, next to
#  gnss_passthrough.
#
#  INVOCATION
#    vivado -mode batch -source pl_npi_ip.tcl -tclargs <src> <out>
#      <src> absolute path to LoopBack_Code/Source
#      <out> absolute path to the IP repository to write into
#
#  OUTPUT
#    <out>/pl_npi/component.xml, VLNV antsdr:gnss:pl_npi:1.0
#
#  SUCCESS MARKER
#    Prints "IP_PACKAGE_OK" only after the self-checks pass.
# ============================================================================

if {$argc < 2} {
  puts "ERROR: expected 2 arguments: <source_dir> <ip_repo_dir>"
  exit 2
}

set src_dir     [file normalize [lindex $argv 0]]
set ip_repo_dir [file normalize [lindex $argv 1]]
set ip_name     "pl_npi"
set ip_dir      [file join $ip_repo_dir $ip_name]
set hdl_dir     [file join $src_dir "HDL" "pl_npi"]

# Wrapper first, then the core files used unchanged from the PL-NPI package.
set rtl_files [list \
  [file join $hdl_dir "pl_npi.v"] \
  [file join $hdl_dir "rtl" "pi_power_inversion_pl_npi.v"] \
  [file join $hdl_dir "rtl" "pi_reciprocal.v"] \
  [file join $hdl_dir "rtl" "pi_cmul.v"]]

foreach f $rtl_files {
  if {![file exists $f]} {
    puts "ERROR: required source not found: $f"
    exit 2
  }
}

file delete -force $ip_dir
file mkdir $ip_dir

create_project -force ${ip_name}_pkg [file join $ip_dir ".pkg_project"] -part xc7z020clg400-2
set_property target_language Verilog [current_project]
add_files -norecurse $rtl_files
set_property top $ip_name [current_fileset]
update_compile_order -fileset sources_1

ipx::package_project -root_dir $ip_dir -vendor antsdr -library gnss \
                     -taxonomy /UserIP -import_files -force

set core [ipx::current_core]
set_property name         $ip_name                                     $core
set_property version      1.0                                          $core
set_property display_name "PL-NPI power inversion"                     $core
set_property description  "Two-element PL-NPI (piecewise-linear normalized power inversion) CRPA core, Jia et al. IEEE Access 2023 Eq. 11" $core
set_property vendor_display_name "ANTSDR GNSS CRPA project"            $core

# ---------------------------------------------------------------------------
#  AXI4-Stream interfaces, created explicitly so the names and port maps are
#  exactly what system_bd.tcl connects. Only TDATA/TVALID/TREADY exist.
# ---------------------------------------------------------------------------
proc add_axis_if {core name mode} {
  foreach bif [ipx::get_bus_interfaces $name -of_objects $core] {
    ipx::remove_bus_interface $name $core
  }
  set bif [ipx::add_bus_interface $name $core]
  set_property abstraction_type_vlnv xilinx.com:interface:axis_rtl:1.0 $bif
  set_property bus_type_vlnv         xilinx.com:interface:axis:1.0     $bif
  set_property interface_mode        $mode                             $bif
  foreach sig {TDATA TVALID TREADY} {
    set pm [ipx::add_port_map $sig $bif]
    set_property physical_name "${name}_[string tolower $sig]" $pm
  }
}

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

add_axis_if $core s_axis_x slave
add_axis_if $core m_axis_y master

set clk_if [ensure_bus_if $core aclk    xilinx.com:signal:clock_rtl:1.0]
set rst_if [ensure_bus_if $core aresetn xilinx.com:signal:reset_rtl:1.0]
set_bus_param $clk_if ASSOCIATED_BUSIF "s_axis_x:m_axis_y"
set_bus_param $clk_if ASSOCIATED_RESET aresetn
set_bus_param $rst_if POLARITY         ACTIVE_LOW

ipx::create_xgui_files $core
ipx::update_checksums  $core
ipx::check_integrity   $core
ipx::save_core         $core

foreach n {s_axis_x m_axis_y} {
  if {[llength [ipx::get_bus_interfaces $n -of_objects $core]] != 1} {
    puts "IP_PACKAGE_SELFCHECK: FAIL - AXI-Stream interface $n missing"
    close_project
    exit 2
  }
}
puts "IP_PACKAGE_SELFCHECK: OK - AXI-Stream interfaces s_axis_x, m_axis_y"

close_project
file delete -force [file join $ip_dir ".pkg_project"]

puts "IP_PACKAGE_OK: $ip_dir/component.xml"
