## ===========================================================================
## gnss_passthrough clock-domain crossings: rx_clk (axi_ad9361 l_clk, 4 x fs) <->
## clk_fpga_0 (100 MHz AXI = MVDR maths clock)
##
## Every crossing inside gnss_passthrough / mvdr_top / mvdr_regs / stap_smi is a
## toggle (+ ack) handshake whose data bundle is held stable for many clocks
## of the destination domain, with a 2-flop ASYNC_REG synchroniser on the
## toggle. Only the data skew needs bounding: one fast clock period (4 ns)
## plus margin. A max delay (not a false path / clock group) keeps the
## buses bounded. ADI's own crossings inside axi_ad9361 are false paths in
## its scoped constraints, which take priority over these.
## ===========================================================================
set_max_delay -datapath_only 8.0 -from [get_clocks rx_clk]     -to [get_clocks clk_fpga_0]
set_max_delay -datapath_only 8.0 -from [get_clocks clk_fpga_0] -to [get_clocks rx_clk]
