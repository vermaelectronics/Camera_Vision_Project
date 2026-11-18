# ============================================================================
#  drc_severity.tcl  --  sourced by the implementation run before opt_design
#  (STEPS.OPT_DESIGN.TCL.PRE, set in system_project.tcl), so everything here
#  applies in the process that runs the DRCs and the methodology report.
#  Nothing here changes the design; it only records reviewed, intended
#  findings. Each step is in a catch: a step Vivado rejects is reported and
#  skipped, never stops the build.
# ============================================================================

proc crpa_try {what script} {
  if {[catch {uplevel 1 $script} err]} {
    puts "CRPA_LVDS: $what NOT applied: $err"
  } else {
    puts "CRPA_LVDS: $what"
  }
}

# ---- DRC: PS DDR3 pins ------------------------------------------------------------
# ddr_ck_p/n, ddr_dqs_p/n[3:0]: the ADI system_top.v declares them as
# single-ended inout ports while the Zynq PS fixes them as differential
# (DIFF_SSTL15 / DIFF_SSTL15_T_DCI) -> 10 x IOSTDTYPE-1. They are hard-wired to
# the PS DDR controller and set up by FSBL / ps7_init, not by the bitstream.
crpa_try "DRC IOSTDTYPE-1 (PS DDR pins) reported as a warning" {
  set_property SEVERITY {Warning} [get_drc_checks IOSTDTYPE-1]
}

# ---- methodology waivers -----------------------------------------------------------
# rx_clk = AD9361 LVDS data clock (l_clk), clk_fpga_0 = sys_cpu_clk (100 MHz).
set crpa_clk_rx  [get_clocks -quiet rx_clk]
set crpa_clk_cpu [get_clocks -quiet clk_fpga_0]

if {[llength $crpa_clk_rx] && [llength $crpa_clk_cpu]} {
  # TIMING-54: set_max_delay -datapath_only between the two clocks in IP-scoped
  # constraint files (ADI axi_dmac / axi_ad9361 clock crossings, and
  # gnss_passthrough's status / control crossings). Intended: these crossings
  # are protected by handshakes / synchronisers and only need a bounded delay.
  crpa_try "waiver TIMING-54 (scoped max-delay CDC constraints, rx_clk <-> clk_fpga_0)" {
    create_waiver -type METHODOLOGY -id TIMING-54 -from $crpa_clk_cpu -to $crpa_clk_rx \
      -description "Reviewed: ADI IP and gnss_passthrough CDC constraints, crossings protected by handshake/synchronisers"
    create_waiver -type METHODOLOGY -id TIMING-54 -from $crpa_clk_rx -to $crpa_clk_cpu \
      -description "Reviewed: ADI IP and gnss_passthrough CDC constraints, crossings protected by handshake/synchronisers"
  }
  # TIMING-9: gnss_passthrough status snapshot (cap_* -> h_*): the bus is held
  # stable for 256 rx_clk cycles and copied on a two-flop synchronised toggle,
  # so it needs no per-bit double register.
  crpa_try "waiver TIMING-9 (gnss_passthrough status snapshot, toggle handshake)" {
    create_waiver -type METHODOLOGY -id TIMING-9 -from $crpa_clk_rx -to $crpa_clk_cpu \
      -description "Reviewed: gnss_passthrough cap_* -> h_* bus held 256 cycles, copied on a synchronised toggle"
    create_waiver -type METHODOLOGY -id TIMING-9 -from $crpa_clk_cpu -to $crpa_clk_rx \
      -description "Reviewed: quasi-static control bits through two-flop synchronisers"
  }
} else {
  puts "CRPA_LVDS: clocks rx_clk / clk_fpga_0 not found, TIMING-54 / TIMING-9 waivers skipped"
}

# TIMING-18: no set_input_delay on the AD9361 LVDS inputs. As in every ADI AD9361
# design, the receive timing is set by the IDELAYs that the AD9361 driver tunes
# on the running board (digital interface tuning), not by static constraints.
set crpa_lvds_in [get_ports -quiet {rx_data_in_p* rx_data_in_n* rx_frame_in_p rx_frame_in_n}]
if {[llength $crpa_lvds_in]} {
  crpa_try "waiver TIMING-18 (AD9361 LVDS inputs, [llength $crpa_lvds_in] ports, delays tuned at run time)" {
    create_waiver -type METHODOLOGY -id TIMING-18 -objects $crpa_lvds_in \
      -description "Reviewed: AD9361 LVDS RX inputs, timing set by run-time IDELAY tuning (ADI ad9361 driver)"
  }
} else {
  puts "CRPA_LVDS: AD9361 LVDS input ports not found, TIMING-18 waiver skipped"
}

# TIMING-18 on the other PL inputs: spi_miso (AD9361 SPI through the PS EMIO, a
# few MHz, sampled with half a 40 ns spi0_clk period of margin), PPS_IN and
# CLKIN_10MHz (asynchronous inputs to axi_vcxo_ctrl, synchronised inside it) and
# the AD9361 control / status GPIOs (static or slow levels via the PS EMIO GPIO).
# Unconstrained in the ADI E310 design as well.
set crpa_slow_in [get_ports -quiet {spi_miso PPS_IN CLKIN_10MHz gpio_resetb gpio_en_agc gpio_ctl* gpio_status*}]
if {[llength $crpa_slow_in]} {
  crpa_try "waiver TIMING-18 (slow / asynchronous inputs, [llength $crpa_slow_in] ports)" {
    create_waiver -type METHODOLOGY -id TIMING-18 -objects $crpa_slow_in \
      -description "Reviewed: SPI MISO, PPS, 10 MHz reference and AD9361 GPIOs are slow or asynchronous inputs"
  }
}
