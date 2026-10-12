# =============================================================================
# package_ip.tcl -- package the M-ESPRIT 2-antenna DOA estimator as a Vivado IP
#
# Usage (from any directory):
#   vivado -mode batch -source <path>/scripts/package_ip.tcl
#   vivado -mode batch -source <path>/scripts/package_ip.tcl -tclargs <part> <ip_repo_dir>
#
# Default part: xczu4ev-sfvc784-1-i (the Zynq UltraScale+ device used in the
# paper). Result: an IP repository at <ip>/ip_repo/m_esprit_2ant_1.0 that can
# be added to any project with
#   set_property ip_repo_paths <ip>/ip_repo [current_project]; update_ip_catalog
# and instantiated in a block design as "M-ESPRIT 2-Antenna DOA Estimator".
# =============================================================================

set ip_root [file normalize [file join [file dirname [info script]] ..]]
set part    [expr {[llength $argv] > 0 ? [lindex $argv 0] : "xczu4ev-sfvc784-1-i"}]
set repo    [expr {[llength $argv] > 1 ? [file normalize [lindex $argv 1]] : [file join $ip_root ip_repo]}]
set build   [file join $ip_root build pkg]
set ip_dir  [file join $repo m_esprit_2ant_1.0]

file delete -force $build $ip_dir
create_project -force pkg_m_esprit $build -part $part
add_files -norecurse [glob [file join $ip_root rtl *.v]]
set_property top m_esprit_2ant_axi [current_fileset]
update_compile_order -fileset sources_1

ipx::package_project -root_dir $ip_dir -vendor user.org -library dsp \
    -taxonomy /DSP -import_files -set_current true -force

set core [ipx::current_core]
set_property name             m_esprit_2ant                           $core
set_property version          1.0                                      $core
set_property core_revision    1                                        $core
set_property display_name     "M-ESPRIT 2-Antenna DOA Estimator"      $core
set_property description      "Modified ESPRIT (M-ESPRIT, Li et al., IEEE TIM 2025, Sec. III-A) for a 2-element CRPA baseline: 4th-order cumulant, C^p, regularised pseudo-inverse, closed-form complex eigenvalue with unit-circle test, CORDIC + arcsin LUT DOA recovery. AXI4-Stream in/out, AXI4-Lite control." $core
set_property company_url      ""                                       $core
set_property supported_families {zynquplus Production zynq Production artix7 Production kintex7 Production kintexuplus Production virtexuplus Production} $core

# --- bus interfaces (inferred from the s_axi_*, s_axis_*, m_axis_* names) ----
ipx::infer_bus_interfaces xilinx.com:interface:aximm_rtl:1.0 $core
ipx::infer_bus_interfaces xilinx.com:interface:axis_rtl:1.0  $core

foreach bi {s_axi s_axis m_axis} {
    if {[ipx::get_bus_interfaces $bi -of_objects $core] ne ""} {
        ipx::associate_bus_interfaces -busif $bi -clock aclk $core
    }
}

# clock / reset
if {[ipx::get_bus_interfaces aresetn -of_objects $core] eq ""} {
    ipx::infer_bus_interface aresetn xilinx.com:signal:reset_rtl:1.0 $core
}
set_property value ACTIVE_LOW [ipx::add_bus_parameter POLARITY \
    [ipx::get_bus_interfaces aresetn -of_objects $core]]
if {[ipx::get_bus_interfaces aclk -of_objects $core] eq ""} {
    ipx::infer_bus_interface aclk xilinx.com:signal:clock_rtl:1.0 $core
}
set clk_if [ipx::get_bus_interfaces aclk -of_objects $core]
if {[ipx::get_bus_parameters ASSOCIATED_RESET -of_objects $clk_if] eq ""} {
    ipx::add_bus_parameter ASSOCIATED_RESET $clk_if
}
set_property value aresetn [ipx::get_bus_parameters ASSOCIATED_RESET -of_objects $clk_if]

# interrupt
ipx::infer_bus_interface irq xilinx.com:signal:interrupt_rtl:1.0 $core
set_property value LEVEL_HIGH [ipx::add_bus_parameter SENSITIVITY \
    [ipx::get_bus_interfaces irq -of_objects $core]]

# --- AXI4-Lite memory map (64 bytes) -----------------------------------------
set mm [ipx::get_memory_maps s_axi -of_objects $core]
if {$mm eq ""} {
    set mm [ipx::add_memory_map s_axi $core]
    set_property slave_memory_map_ref s_axi [ipx::get_bus_interfaces s_axi -of_objects $core]
}
set ab [ipx::get_address_blocks reg0 -of_objects $mm]
if {$ab eq ""} { set ab [ipx::add_address_block reg0 $mm] }
set_property range 64     $ab
set_property width 32     $ab
set_property usage register $ab

# --- user parameter GUI ------------------------------------------------------
set p [ipx::get_user_parameters MAX_LOG2_SNAP -of_objects $core]
if {$p ne ""} {
    set_property display_name "Max log2(snapshots per window)" $p
    set_property value_validation_type range_long $p
    set_property value_validation_range_minimum 6 $p
    set_property value_validation_range_maximum 16 $p
}
foreach hp {C_S_AXI_ADDR_WIDTH C_S_AXI_DATA_WIDTH} {
    set p [ipx::get_user_parameters $hp -of_objects $core]
    if {$p ne ""} { set_property enablement_value false $p }
}

ipx::create_xgui_files $core
ipx::update_checksums  $core
ipx::check_integrity   $core
ipx::save_core         $core
close_project

puts "INFO: IP packaged to $ip_dir"
