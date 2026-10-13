# ============================================================================
#  gnss_passthrough_cdc.xdc
#  Clock-domain-crossing constraints for the custom processing block.
#
#  !! XDC IS NOT FULL TCL !!
#    Vivado rejects control-flow commands in a constraint file:
#      CRITICAL WARNING: [Designutils 20-1307] Command 'if' is not supported in
#      the xdc constraint file.
#    An earlier version of this file wrapped each constraint in an
#    "if {[llength $cells] > 0}" guard. Vivado skipped every one of them and the
#    violation was unchanged, with only a CRITICAL WARNING in the run log to say
#    so. Keep this file to straight-line constraint commands only -- no if, no
#    loops, no puts.
#
#  WHY THESE CONSTRAINTS EXIST
#    gnss_passthrough spans two asynchronous clocks:
#      clk        - axi_ad9361/l_clk, sample domain   (rx_clk,     8 ns)
#      s_axi_aclk - sys_cpu_clk, register domain      (clk_fpga_0, 10 ns)
#
#    Vivado times the crossings between them as ordinary synchronous paths and
#    reports violations that are not real. Unconstrained, this produced the ONLY
#    failing timing in the whole device: WNS -0.817 ns with 312 failing
#    endpoints, every one of them a cap_* -> h_* crossing. Both intra-clock
#    groups were comfortably met (clk_fpga_0 +1.238 ns, rx_clk +1.420 ns).
#
#    The crossings are safe by protocol, not by timing. See the CDC section of
#    Source/HDL/gnss_passthrough.v and Docs/Architecture/DATAPATH.md.
#
#  WHY NOT PACKAGED WITH THE IP
#    Tried first. The XDC packaged into component.xml with userFileType "xdc",
#    but Vivado never scoped it to the instance -- the implementation log showed
#    "Parsing XDC File ... for cell ..." for every other IP and none for
#    gnss_passthrough. Applied at project level it is unambiguously read. This
#    project owns both the IP and the design that instantiates it.
#
#  HIERARCHY
#    Matched by wildcard so the constraints survive block-design regeneration.
#    The instance is currently at:
#      i_system_wrapper/system_i/gnss_passthrough/inst/
#
#    A pattern that matches nothing yields an empty object list and a warning,
#    not an error -- so check the run log if timing regresses after renaming
#    anything inside the IP.
# ============================================================================

# --- 1. Status capture: clk -> s_axi_aclk -----------------------------------
# cap_* is loaded once every 256 clk cycles and then held. A toggle flips at the
# same moment, crosses through a two-flop synchroniser, and only on its edge
# does the AXI side copy cap_* into h_*. The data is stable for far longer than
# the transfer takes.
#
# set_max_delay -datapath_only rather than set_false_path: the transfer must
# still finish before cap_* is reloaded, so a bound is wanted rather than a
# licence to route arbitrarily. 8.000 ns is the shorter of the two periods.
set_max_delay -datapath_only \
  -from [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/cap_*_reg*}] \
  -to   [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/h_*_reg*}] \
  8.000

# --- 2. Toggle handshake into the AXI domain --------------------------------
# cap_toggle -> tog_meta is the single-bit asynchronous crossing that the
# two-flop synchroniser exists to handle.
set_false_path \
  -to [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/tog_meta_reg*}]

# --- 3. Control crossing into the sample domain -----------------------------
# Quasi-static software settings through two-flop synchronisers.
set_false_path \
  -to [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/ctrl_meta_reg*}]

# ============================================================================
# --- 4. v1.2 ESPRIT DOA monitor (gnss_doa_esprit, instance u_doa) ----------
# ============================================================================
# RX snapshots cross from rx_clk (l_clk, 8 ns) into clk_fpga_0 (10 ns) through
# doa_async_fifo, a gray-pointer dual-clock FIFO. Control bits cross the other
# way through two-flop synchronisers (dsync_meta/dsync). See the CDC section
# of Source/HDL/gnss_doa_esprit.v.

# 4a. Gray-coded FIFO pointers into their two-flop synchronisers.
#     Bounded delay (not a false path): a gray pointer may change by only one
#     bit per source clock, so the skew across its bits must stay below one
#     source period. 8.000 ns = the shorter period (rx_clk).
set_max_delay -datapath_only \
  -from [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/u_doa/u_afifo/wgray_reg*}] \
  -to   [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/u_doa/u_afifo/gs_meta_w2r_reg*}] \
  8.000
set_max_delay -datapath_only \
  -from [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/u_doa/u_afifo/rgray_reg*}] \
  -to   [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/u_doa/u_afifo/gs_meta_r2w_reg*}] \
  8.000

# 4b. FIFO storage (distributed RAM written in rx_clk) to the read register
#     (clk_fpga_0). A word is only read after its write pointer has crossed
#     the synchroniser (>= 2 read clocks later), so a bounded delay suffices.
#     The storage is distributed RAM (RAM32M/RAM64M macros). get_cells on
#     "mem_reg*" also returns the hierarchical macro wrappers, which are not
#     valid timing startpoints, and the exception was observed NOT to apply
#     (paths timed rx_clk -> clk_fpga_0 with clock skew, WNS -1.8 ns). Name
#     the leaf RAM clock pins and the read-register D pins explicitly.
set_max_delay -datapath_only \
  -from [get_pins -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/u_doa/u_afifo/mem_reg*/CLK}] \
  -to   [get_pins -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/u_doa/u_afifo/rd_data_reg*/D}] \
  8.000

# 4c. Sticky overflow flag into the processing domain (level, two-flop).
set_false_path \
  -to [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/u_doa/u_afifo/ovf_meta_reg*}]

# 4d. Quasi-static DOA control (enable, reset, DECIM) into the sample domain.
set_false_path \
  -to [get_cells -quiet -hier -filter {NAME =~ *gnss_passthrough/inst/u_doa/dsync_meta_reg*}]
