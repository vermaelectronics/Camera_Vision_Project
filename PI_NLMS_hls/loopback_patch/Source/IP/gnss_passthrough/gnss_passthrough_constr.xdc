# ============================================================================
#  gnss_passthrough_constr.xdc
#  Clock-domain-crossing constraints, packaged WITH the IP so they travel with
#  it and apply wherever it is instantiated.
#
#  WHY THIS FILE EXISTS
#    gnss_passthrough spans two asynchronous clocks:
#      clk        - axi_ad9361/l_clk, the sample domain (rx_clk, 8 ns)
#      s_axi_aclk - sys_cpu_clk, the register domain (clk_fpga_0, 10 ns)
#
#    Without these constraints Vivado times the crossings as ordinary
#    synchronous paths between unrelated clocks and reports setup violations
#    that are not real. On the first build of this design that produced exactly
#    one violated path in the whole device:
#      cap_tx1_reg[5]/C (rx_clk) -> h_tx1_reg[5]/D (clk_fpga_0), slack -0.817 ns
#
#    The crossings are safe by protocol, not by timing:
#
#    1. Status, clk -> s_axi_aclk. The cap_* registers are loaded once every
#       256 clk cycles and then held. A toggle flips at the same moment, is
#       synchronised into s_axi_aclk by two flops, and only on its edge does the
#       AXI side copy cap_* into h_*. The data is therefore stable for far
#       longer than the transfer takes. What is required is a BOUNDED delay, not
#       a synchronous relationship - hence set_max_delay -datapath_only rather
#       than set_false_path, so the tool still keeps the path short enough that
#       the data cannot arrive later than the next capture.
#
#    2. Control, s_axi_aclk -> clk. Quasi-static software settings through
#       two-flop synchronisers. Genuinely asynchronous; false path is correct.
#
#  SCOPING
#    Packaged as a scoped IP constraint, so every get_cells below resolves
#    inside this IP instance only and cannot affect the rest of the design.
# ============================================================================

# --- 1. Status crossing: bounded delay, not a false path --------------------
# 8.000 ns is the shorter of the two clock periods (rx_clk). Keeping the
# datapath under one source-clock period guarantees the transfer completes long
# before cap_* is reloaded 256 cycles later.
set_max_delay -datapath_only \
  -from [get_cells -quiet -hier -filter {NAME =~ *cap_*_reg*}] \
  -to   [get_cells -quiet -hier -filter {NAME =~ *h_*_reg*}] \
  8.000

# --- 2. Toggle handshake into the AXI domain --------------------------------
# cap_toggle -> tog_meta is the classic single-bit asynchronous crossing that
# the two-flop synchroniser exists to handle.
set_false_path \
  -to [get_cells -quiet -hier -filter {NAME =~ *tog_meta_reg*}]

# --- 3. Control crossing into the sample domain -----------------------------
set_false_path \
  -to [get_cells -quiet -hier -filter {NAME =~ *ctrl_meta_reg*}]
