# ============================================================================
#  gnss_stap_cdc.xdc
#  Clock-domain-crossing constraints for the CRPA anti-jam stage (gnss_stap).
#  Applied at project level with PROCESSING_ORDER LATE, exactly like
#  gnss_passthrough_cdc.xdc (see that file for why it is not packaged).
#
#  XDC IS NOT FULL TCL: straight-line constraint commands only.
#
#  gnss_stap spans two asynchronous clocks:
#    clk        - axi_ad9361/l_clk, sample domain    (rx_clk,     8 ns)
#    s_axi_aclk - sys_cpu_clk, register domain       (clk_fpga_0, 10 ns)
#
#  All crossings are safe by protocol (Source/HDL/stap/axil_cdc.v header and
#  the status section of Source/HDL/gnss_stap.v); the constraints keep Vivado
#  from timing them as synchronous paths. Payload registers get a bounded
#  delay (-datapath_only) rather than a false path, so the data is guaranteed
#  to arrive before the two-flop synchronised toggle that announces it.
#
#  Instance path (block design): i_system_wrapper/system_i/gnss_stap/inst/
# ============================================================================

# --- 1. CPU -> l_clk: request toggle and its payload ------------------------
set_false_path \
  -to [get_cells -quiet -hier -filter {NAME =~ *gnss_stap/inst/i_cdc/req_meta_reg*}]
set_max_delay -datapath_only \
  -from [get_cells -quiet -hier -filter {NAME =~ *gnss_stap/inst/i_cdc/req_addr_reg* || NAME =~ *gnss_stap/inst/i_cdc/req_wdata_reg* || NAME =~ *gnss_stap/inst/i_cdc/req_write_reg*}] \
  8.000

# --- 2. l_clk -> CPU: acknowledge toggle and read data -----------------------
set_false_path \
  -to [get_cells -quiet -hier -filter {NAME =~ *gnss_stap/inst/i_cdc/ack_meta_reg*}]
set_max_delay -datapath_only \
  -from [get_cells -quiet -hier -filter {NAME =~ *gnss_stap/inst/i_cdc/ack_rdata_reg*}] \
  8.000

# --- 3. status capture (sample counter, l_clk-alive toggle) ------------------
set_false_path \
  -to [get_cells -quiet -hier -filter {NAME =~ *gnss_stap/inst/tog_meta_reg*}]
set_max_delay -datapath_only \
  -from [get_cells -quiet -hier -filter {NAME =~ *gnss_stap/inst/cap_count_reg*}] \
  -to   [get_cells -quiet -hier -filter {NAME =~ *gnss_stap/inst/h_count_reg*}] \
  8.000
