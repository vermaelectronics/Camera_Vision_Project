# Vivado 2021.1: test project = Zynq PS7 + PI-NLMS HLS IP (xc7z020clg400-2)
#
# Usage:
#   /tools/Xilinx/Vivado/2021.1/bin/vivado -source create_prj.tcl -tclargs <ip_repo_dir>
# <ip_repo_dir> defaults to ~/PI_NLMS_2021/PI_NLMS_hls/solution1/impl/ip
#
# The three AXI-Stream ports are made external for validation only; for a
# bitstream, connect them to the AD9361/DMA path instead.

set ip_repo [expr {[llength $argv] > 0 ? [lindex $argv 0] \
                   : "$::env(HOME)/PI_NLMS_2021/PI_NLMS_hls/solution1/impl/ip"}]
set prj_dir [file normalize [file dirname [info script]]/pi_nlms_prj]

create_project pi_nlms_prj $prj_dir -part xc7z020clg400-2 -force
set_property ip_repo_paths $ip_repo [current_project]
update_ip_catalog -rebuild

create_bd_design system

# Zynq PS, FCLK_CLK0 = 100 MHz (HLS target clock)
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "0" Master "Disable" Slave "Disable"} \
    [get_bd_cells ps7]
set_property CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ 100 [get_bd_cells ps7]

# PI-NLMS core, control bus on M_AXI_GP0
create_bd_cell -type ip -vlnv Digvijay:hls:pi_nlms:1.0 pi_nlms_0
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config {Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/ps7/M_AXI_GP0} Slave {/pi_nlms_0/s_axi_CTRL_BUS} intc_ip {New AXI Interconnect} master_apm {0}} \
    [get_bd_intf_pins pi_nlms_0/s_axi_CTRL_BUS]

# Temporary external streams, associated with an exported 100 MHz clock
make_bd_intf_pins_external [get_bd_intf_pins pi_nlms_0/in1]
make_bd_intf_pins_external [get_bd_intf_pins pi_nlms_0/in2]
make_bd_intf_pins_external [get_bd_intf_pins pi_nlms_0/out]
create_bd_port -dir O -type clk stream_clk
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_ports stream_clk]
set_property CONFIG.ASSOCIATED_BUSIF {in1_0:in2_0:out_0} [get_bd_ports stream_clk]
set_property CONFIG.FREQ_HZ 100000000 [get_bd_intf_ports {in1_0 in2_0}]

assign_bd_address
regenerate_bd_layout
validate_bd_design
save_bd_design

make_wrapper -files [get_files system.bd] -top
add_files -norecurse [glob $prj_dir/pi_nlms_prj.gen/sources_1/bd/system/hdl/system_wrapper.v]
set_property top system_wrapper [current_fileset]
update_compile_order -fileset sources_1

puts "INFO: pi_nlms_0 control base address: [get_property OFFSET [get_bd_addr_segs -of_objects [get_bd_addr_spaces ps7/Data] -filter {NAME =~ *pi_nlms*}]]"
