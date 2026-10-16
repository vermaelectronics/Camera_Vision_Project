// ============================================================================
//  gnss_passthrough.v
//  ANTSDR E310 V1 / GNSS CRPA project  --  Phase 1 custom processing block
// ----------------------------------------------------------------------------
//  PURPOSE
//    This module is the single, stable insertion point for custom FPGA
//    processing between the AD9361 RX datapath and the AD9361 TX datapath
//    (project requirements 41, 43, 44).
//
//    Phase 1 behaviour is transparent (requirement 42):
//        Iout = Iin
//        Qout = Qin
//    No scaling, no filtering, no rotation is applied.  The sample values that
//    leave this block are bit-for-bit the sample values that entered it.
//
//    A future CRPA (Controlled Reception Pattern Antenna) algorithm replaces
//    the body of the "PROCESSING CORE" section below.  Everything outside that
//    section -- the port list, the AXI4-Lite register map, the elastic buffer
//    and the status counters -- is intended to stay unchanged, so that CRPA can
//    be dropped in without redesigning the radio datapath (requirement 44).
//
//  DATAPATH POSITION  (requirement 45)
//    axi_ad9361 (ADC) --> gnss_passthrough --> axi_ad9361 (DAC)
//    The path is entirely in programmable logic.  The ARM PS is NOT in the
//    per-sample loop (requirement 46); the PS only reads/writes the registers
//    below.  The vendor AXI DMA capture path is preserved untouched and stays
//    available for diagnostics (requirements 47, 48).
//
//  CLOCK DOMAINS
//    clk          : axi_ad9361/l_clk   -- the AD9361 sample-interface clock.
//                                         All datapath logic lives here.
//    s_axi_aclk   : sys_cpu_clk        -- PS7 FCLK_CLK0, 100.0 MHz in the
//                                         official E310 V1 design.
//    These are asynchronous.  Status/counters cross clk -> s_axi_aclk through
//    a capture-and-toggle handshake; control bits cross s_axi_aclk -> clk
//    through two-flop synchronisers.  See the CDC section.
//
//  REGISTER MAP  (AXI4-Lite, 4 kB aperture)
//    Base address is assigned by the block design, not by this file.
//    This project assigns 0x43C0_0000 (see Source/Config/board_e310_v1.json).
//
//    Offset  Name            Access  Description
//    0x00    ID              RO      0x47435031 = "GCP1"
//    0x04    VERSION         RO      {16'h major, 16'h minor}
//    0x08    SCRATCH         RW      read/write test register
//    0x0C    CONTROL         RW      [0]  pass_en    1 = RX->TX passthrough
//                                         0 = vendor DMA/DDS drives the DAC
//                                    [1]  mute       1 = drive 0x0000 to DAC
//                                    [2]  swap_iq    1 = exchange I and Q
//                                    [3]  ch1_copy   1 = ch1 DAC fed from ch0
//                                    [4]  lcmv_en    1 = ch0 TX carries the
//                                                        2-element LCMV output
//                                                        (gain 1 to the GPS
//                                                        angle, null on the
//                                                        jammer angle)
//                                                    0 = ch0 TX carries RX1
//                                    [5]  lcmv_sw    weight source:
//                                                    1 = PS solves the paper's
//                                                        w = R^-1 C (C^H R^-1 C)^-1 f
//                                                        from the measured
//                                                        covariance (0x90..0xA0)
//                                                        and writes 0x80..0x8C
//                                                    0 = PL angle solver
//                                    [8]  cnt_clear  1 = hold counters cleared
//    0x10    STATUS          RO      [0]  adc_enable_i0   [1]  adc_enable_q0
//                                    [2]  dac_enable_i0   [3]  dac_enable_q0
//                                    [4]  fifo0_empty     [5]  fifo0_full
//                                    [6]  fifo1_empty     [7]  fifo1_full
//                                    [8]  overflow_sticky [9]  underflow_sticky
//                                    [16] pass_en (as seen in the clk domain)
//                                    [17] lcmv_en (as seen in the clk domain)
//                                    [19] lcmv_active: LCMV output is on TX
//                                         ch0 (lcmv_en & weights valid)
//                                    [22] lcmv_sat_sticky: an LCMV output
//                                         sample was clipped to 12 bits
//                                    [23] lcmv_sw (as seen in the clk domain)
//
//            READ [2]/[3] CAREFULLY. dac_enable_* is NOT an enable this block
//            drives, and it is NOT derived from the valid strobes. It is a
//            read-back of the AD9361 DAC channel's data-source select:
//                axi_ad9361_tx_channel.v:272
//                  dac_enable_int <= (dac_data_sel_s == 4'h2) ? 1'b1 : 1'b0;
//            and 4'h2 is the ONLY source select for which that channel's mux
//            takes dma_data -- the port this block drives:
//                axi_ad9361_tx_channel.v:297-305
//                  4'h2:    dac_data_out_int <= dma_data[15:4];
//                  default: dac_data_out_int <= dac_dds_data_s;   // DDS tone
//            So [2]/[3] reading 0 means the DAC core is IGNORING everything
//            this block outputs and emitting its DDS tone instead, even while
//            dac_valid keeps pulsing and TX_COUNT keeps advancing. Firmware
//            must write CHAN_CNTRL_7 (0x0418 + c*0x40) = 2 on the channels
//            before anything here can reach the AD9361. See
//            gnss_l1_set_dac_source() in Source/Firmware/app_gnss_e310.
//    0x14    RX_COUNT_CH0    RO      adc_valid_i0 pulses observed
//    0x18    TX_COUNT_CH0    RO      dac_valid_i0 pulses served
//    0x1C    OVERFLOW_COUNT  RO      writes attempted into a full buffer
//    0x20    UNDERFLOW_COUNT RO      reads attempted from an empty buffer
//    0x24    RX_SNAPSHOT_CH0 RO      {adc_data_q0, adc_data_i0} last accepted
//                                    RX format: 12-bit sample RIGHT-aligned in
//                                    16 bits, sign-extended into [15:12].
//    0x28    TX_SNAPSHOT_CH0 RO      {dac_data_q0, dac_data_i0} last driven
//                                    TX format: 12-bit sample LEFT-aligned,
//                                    [15:4]. In passthrough this reads 16x the
//                                    RX snapshot -- see OUTPUT SAMPLE ALIGNMENT
//                                    below. A TX snapshot equal to the RX one
//                                    means the alignment stage is missing.
//    0x2C    FIFO_LEVEL      RO      [5:0] ch0 occupancy, [13:8] ch1 occupancy
//    0x30    RX_COUNT_CH1    RO      adc_valid_i1 pulses observed
//    0x34    TX_COUNT_CH1    RO      dac_valid_i1 pulses served
//    0x38    RX_SNAPSHOT_CH1 RO      {adc_data_q1, adc_data_i1} last accepted
//    0x3C    RESERVED        RO      reads 0
//    0x40    LCMV_GPS_ANGLE  RW      [15:0] signed integer deg -90..90 from
//                                    broadside, look direction. Reset 30.
//    0x44    LCMV_JAM_ANGLE  RW      [15:0] signed integer deg -90..90 from
//                                    broadside, null direction. Reset 60.
//    0x48    CRPA_COEF[2]    RW      stored, unused (reserved)
//    0x4C    LCMV_EPS        RW      [15:0] Q0.16 LCMV gain toward the jammer,
//                                    0 = ideal null               (COEF[3])
//    0x50    LCMV_RX2_CAL    RW      {im[31:16], re[15:0]} Q2.14, multiplies
//                                    RX2 before the LCMV (RX2/RX1 gain and
//                                    phase match). Reset 0x00004000 = 1.0.
//    0x54    LCMV_W1_RE      RO      active LCMV weights, Q16.16
//    0x58    LCMV_W1_IM      RO
//    0x5C    LCMV_W2_RE      RO      (includes the RX2 calibration)
//    0x60    LCMV_W2_IM      RO
//    0x64    LCMV_STATUS     RO      [0] lcmv_en [1] weights_valid
//                                    [2] solver busy [3] sat_sticky
//                                    [4] lcmv_sw (sample domain)
//    0x68    LCMV_UPDATES    RO      LCMV weight sets computed
//    0x6C    LCMV_SAT_COUNT  RO      LCMV output samples clipped
//    0x70..  CRPA_COEF[12..15] RW    reserved; stored and returned, no logic
//                                    consumes them.
//    0x80    LCMV_SW_W1_RE   RW      PS weights, Q16.16 (bits [24:0] used,
//    0x84    LCMV_SW_W1_IM   RW      i.e. |w| < 128), final beamformer weights
//    0x88    LCMV_SW_W2_RE   RW      with the RX2 calibration included.
//    0x8C    LCMV_SW_W2_IM   RW      Writing 0x8C COMMITS all four atomically.
//    0x90    COV_R11         RO      covariance block average, Q.4 LSB^2
//    0x94    COV_R22         RO        (raw ADC samples, 4096-pair blocks,
//    0x98    COV_R12_RE      RO         R12 = E[x1 conj(x2)], signed)
//    0x9C    COV_R12_IM      RO
//    0xA0    COV_BLOCKS      RO      covariance blocks completed
//
//    LCMV (v3.0): R. Li, X. Zhao, X. W. Shi, "Derivative Constrained Robust
//    LCMV Beamforming Algorithm", PIER C vol. 4, 2008, eq. (9), look +
//    enforcement constraints for 2 elements (Source/HDL/crpa/). Angles come
//    from software; no DOA estimation in the FPGA.
//
//  LICENCE
//    Original work for this project.  Not derived from Analog Devices or
//    MicroPhase source.  See Docs/Architecture/MODIFICATIONS.md.
// ============================================================================

`timescale 1ns/100ps

module gnss_passthrough #(
  parameter FIFO_ADDR_WIDTH = 5   // elastic buffer depth = 2**FIFO_ADDR_WIDTH
) (
  // ---- sample-domain clock / reset (axi_ad9361 l_clk / rst) ----------------
  input               clk,
  input               rst,               // active high, from axi_ad9361/rst

  // ---- RX: from axi_ad9361 ADC interface ----------------------------------
  input               adc_enable_i0,
  input               adc_valid_i0,
  input      [15:0]   adc_data_i0,
  input               adc_enable_q0,
  input               adc_valid_q0,
  input      [15:0]   adc_data_q0,
  input               adc_enable_i1,
  input               adc_valid_i1,
  input      [15:0]   adc_data_i1,
  input               adc_enable_q1,
  input               adc_valid_q1,
  input      [15:0]   adc_data_q1,

  // ---- TX: request strobes from axi_ad9361 DAC interface ------------------
  input               dac_enable_i0,
  input               dac_valid_i0,
  input               dac_enable_q0,
  input               dac_valid_q0,
  input               dac_enable_i1,
  input               dac_valid_i1,
  input               dac_enable_q1,
  input               dac_valid_q1,

  // ---- TX: vendor DMA/DDS sourced data (from util_rfifo dout_data_*) ------
  input      [15:0]   dma_dac_data_i0,
  input      [15:0]   dma_dac_data_q0,
  input      [15:0]   dma_dac_data_i1,
  input      [15:0]   dma_dac_data_q1,

  // ---- TX: muxed data driven into axi_ad9361 ------------------------------
  output     [15:0]   dac_data_i0,
  output     [15:0]   dac_data_q0,
  output     [15:0]   dac_data_i1,
  output     [15:0]   dac_data_q1,

  // ---- AXI4-Lite slave (sys_cpu_clk domain) -------------------------------
  input               s_axi_aclk,
  input               s_axi_aresetn,
  input               s_axi_awvalid,
  input      [15:0]   s_axi_awaddr,
  input      [ 2:0]   s_axi_awprot,
  output              s_axi_awready,
  input               s_axi_wvalid,
  input      [31:0]   s_axi_wdata,
  input      [ 3:0]   s_axi_wstrb,
  output              s_axi_wready,
  output              s_axi_bvalid,
  output     [ 1:0]   s_axi_bresp,
  input               s_axi_bready,
  input               s_axi_arvalid,
  input      [15:0]   s_axi_araddr,
  input      [ 2:0]   s_axi_arprot,
  output              s_axi_arready,
  output              s_axi_rvalid,
  output     [ 1:0]   s_axi_rresp,
  output     [31:0]   s_axi_rdata,
  input               s_axi_rready
);

  localparam [31:0] CORE_ID      = 32'h47435031;   // "GCP1"
  // v1.1 -- adds the RX->TX sample alignment stage. Bumped so a running board
  // reports which of the two behaviours its bitstream actually has.
  // v3.0 -- LCMV CRPA build: 2-element LCMV beamformer (CONTROL[4],
  //         registers 0x40/0x44, 0x4C-0x6C) on TX ch0. No PL-NPI / PI core.
  // v3.1 -- adds the 2x2 covariance estimator (0x90..0xA0) and PS-computed
  //         weights (CONTROL[5], 0x80..0x8C): the full paper formula
  //         w = R^-1 C (C^H R^-1 C)^-1 f is solved on the ARM.
  localparam [31:0] CORE_VERSION = 32'h00030001;   // v3.1
  localparam        AW           = FIFO_ADDR_WIDTH;

  // ==========================================================================
  //  AXI4-Lite slave  (s_axi_aclk domain)
  // ==========================================================================
  reg         axi_awready = 1'b0;
  reg         axi_wready  = 1'b0;
  reg         axi_bvalid  = 1'b0;
  reg         axi_arready = 1'b0;
  reg         axi_rvalid  = 1'b0;
  reg  [31:0] axi_rdata   = 32'd0;
  reg  [13:0] axi_awaddr_r = 14'd0;
  reg  [13:0] axi_araddr_r = 14'd0;

  assign s_axi_awready = axi_awready;
  assign s_axi_wready  = axi_wready;
  assign s_axi_bvalid  = axi_bvalid;
  assign s_axi_bresp   = 2'b00;          // always OKAY
  assign s_axi_arready = axi_arready;
  assign s_axi_rvalid  = axi_rvalid;
  assign s_axi_rresp   = 2'b00;          // always OKAY
  assign s_axi_rdata   = axi_rdata;

  wire        wr_hs = s_axi_awvalid & s_axi_wvalid & ~axi_bvalid &
                      ~axi_awready & ~axi_wready;

  // Write address/data accepted together (single-beat AXI4-Lite).
  always @(posedge s_axi_aclk) begin
    if (s_axi_aresetn == 1'b0) begin
      axi_awready  <= 1'b0;
      axi_wready   <= 1'b0;
      axi_bvalid   <= 1'b0;
      axi_awaddr_r <= 14'd0;
    end else begin
      axi_awready <= wr_hs;
      axi_wready  <= wr_hs;
      if (wr_hs)
        axi_awaddr_r <= s_axi_awaddr[15:2];
      if (axi_awready)
        axi_bvalid <= 1'b1;
      else if (s_axi_bready && axi_bvalid)
        axi_bvalid <= 1'b0;
    end
  end

  always @(posedge s_axi_aclk) begin
    if (s_axi_aresetn == 1'b0) begin
      axi_arready  <= 1'b0;
      axi_rvalid   <= 1'b0;
      axi_araddr_r <= 14'd0;
    end else begin
      axi_arready <= s_axi_arvalid & ~axi_arready & ~axi_rvalid;
      if (s_axi_arvalid && axi_arready)
        axi_araddr_r <= s_axi_araddr[15:2];
      if (s_axi_arvalid && axi_arready)
        axi_rvalid <= 1'b1;
      else if (axi_rvalid && s_axi_rready)
        axi_rvalid <= 1'b0;
    end
  end

  wire        reg_wr    = axi_awready;             // one-cycle write strobe
  wire [11:0] reg_waddr = axi_awaddr_r[11:0];

  // ---- software-writable state --------------------------------------------
  reg  [31:0] reg_scratch = 32'd0;
  reg  [31:0] reg_control = 32'd0;
  reg  [31:0] crpa_coef [0:15];
  // LCMV settings, also kept in plain registers (synchronised below).
  localparam [31:0] LCMV_CAL_ONE = 32'h0000_4000;     // 1.0 + j0, Q2.14
  reg  [15:0] reg_lcmv_gps = 16'd30;                  // 0x40, reset 30 deg
  reg  [15:0] reg_lcmv_jam = 16'd60;                  // 0x44, reset 60 deg
  reg  [15:0] reg_lcmv_eps = 16'd0;                   // 0x4C
  reg  [31:0] reg_lcmv_cal = LCMV_CAL_ONE;            // 0x50
  // PS weights: staging registers, committed together by the 0x8C write.
  reg  [31:0] reg_sw_w1r = 32'h0001_0000, reg_sw_w1i = 32'd0;   // 0x80, 0x84
  reg  [31:0] reg_sw_w2r = 32'd0,         reg_sw_w2i = 32'd0;   // 0x88, 0x8C
  reg  [99:0] reg_sw_commit = {75'd0, 25'sd65536};             // {w2i,w2r,w1i,w1r}
  reg         reg_sw_seq    = 1'b0;

  integer ci;
  initial for (ci = 0; ci < 16; ci = ci + 1) crpa_coef[ci] = 32'd0;

  always @(posedge s_axi_aclk) begin
    if (s_axi_aresetn == 1'b0) begin
      reg_scratch <= 32'd0;
      reg_control <= 32'd0;
      reg_lcmv_gps  <= 16'd30;
      reg_lcmv_jam  <= 16'd60;
      reg_lcmv_eps  <= 16'd0;
      reg_lcmv_cal  <= LCMV_CAL_ONE;
      reg_sw_w1r <= 32'h0001_0000; reg_sw_w1i <= 32'd0;
      reg_sw_w2r <= 32'd0;         reg_sw_w2i <= 32'd0;
      reg_sw_commit <= {75'd0, 25'sd65536};
      reg_sw_seq    <= 1'b0;
    end else if (reg_wr) begin
      case (reg_waddr)
        12'h002: reg_scratch <= s_axi_wdata;   // 0x08
        12'h003: reg_control <= s_axi_wdata;   // 0x0C
        12'h010: reg_lcmv_gps  <= s_axi_wdata[15:0];  // 0x40
        12'h011: reg_lcmv_jam  <= s_axi_wdata[15:0];  // 0x44
        12'h013: reg_lcmv_eps  <= s_axi_wdata[15:0];  // 0x4C
        12'h014: reg_lcmv_cal  <= s_axi_wdata;        // 0x50
        12'h020: reg_sw_w1r    <= s_axi_wdata;        // 0x80
        12'h021: reg_sw_w1i    <= s_axi_wdata;        // 0x84
        12'h022: reg_sw_w2r    <= s_axi_wdata;        // 0x88
        12'h023: begin                                // 0x8C: commit
          reg_sw_w2i    <= s_axi_wdata;
          reg_sw_commit <= {s_axi_wdata[24:0], reg_sw_w2r[24:0],
                            reg_sw_w1i[24:0],  reg_sw_w1r[24:0]};
          reg_sw_seq    <= ~reg_sw_seq;
        end
        default: ;
      endcase
      if (reg_waddr >= 12'h010 && reg_waddr <= 12'h01F)   // 0x40..0x7C
        crpa_coef[reg_waddr[3:0]] <= s_axi_wdata;
    end
  end

  // Status view, refreshed from the clk domain (see CDC section).
  wire [31:0] v_status, v_rx_cnt0, v_tx_cnt0, v_ovf_cnt, v_unf_cnt;
  wire [31:0] v_rx_snap0, v_tx_snap0, v_level, v_rx_cnt1, v_tx_cnt1, v_rx_snap1;
  wire [31:0] v_lw1r, v_lw1i, v_lw2r, v_lw2i, v_lst, v_lupd, v_lsat;
  wire [31:0] v_c11, v_c22, v_c12r, v_c12i, v_cblk;

  // Read data is captured on the SAME edge that accepts the address, using the
  // live s_axi_araddr rather than the registered copy.
  //
  // The earlier version decoded axi_araddr_r, which is loaded on that very
  // edge. The case therefore saw the PREVIOUS address while rvalid was being
  // asserted for the current one, so every read returned the previous
  // register's value. On hardware this showed up as VERSION reading back
  // 0x47435031 -- the ID -- because the ID read had immediately preceded it.
  // The first read after reset happened to be correct only because
  // axi_araddr_r resets to 0, which is the ID offset.
  always @(posedge s_axi_aclk) begin
    if (s_axi_arvalid && axi_arready) begin
    case (s_axi_araddr[13:2])
      12'h000: axi_rdata <= CORE_ID;
      12'h001: axi_rdata <= CORE_VERSION;
      12'h002: axi_rdata <= reg_scratch;
      12'h003: axi_rdata <= reg_control;
      12'h004: axi_rdata <= v_status;
      12'h005: axi_rdata <= v_rx_cnt0;
      12'h006: axi_rdata <= v_tx_cnt0;
      12'h007: axi_rdata <= v_ovf_cnt;
      12'h008: axi_rdata <= v_unf_cnt;
      12'h009: axi_rdata <= v_rx_snap0;
      12'h00A: axi_rdata <= v_tx_snap0;
      12'h00B: axi_rdata <= v_level;
      12'h00C: axi_rdata <= v_rx_cnt1;
      12'h00D: axi_rdata <= v_tx_cnt1;
      12'h00E: axi_rdata <= v_rx_snap1;
      12'h010: axi_rdata <= {{16{reg_lcmv_gps[15]}}, reg_lcmv_gps};   // 0x40
      12'h011: axi_rdata <= {{16{reg_lcmv_jam[15]}}, reg_lcmv_jam};   // 0x44
      12'h012: axi_rdata <= crpa_coef[2];                             // 0x48
      12'h013: axi_rdata <= {16'd0, reg_lcmv_eps};                    // 0x4C
      12'h014: axi_rdata <= reg_lcmv_cal;                             // 0x50
      12'h015: axi_rdata <= v_lw1r;                                   // 0x54
      12'h016: axi_rdata <= v_lw1i;                                   // 0x58
      12'h017: axi_rdata <= v_lw2r;                                   // 0x5C
      12'h018: axi_rdata <= v_lw2i;                                   // 0x60
      12'h019: axi_rdata <= v_lst;                                    // 0x64
      12'h01A: axi_rdata <= v_lupd;                                   // 0x68
      12'h01B: axi_rdata <= v_lsat;                                   // 0x6C
      12'h020: axi_rdata <= reg_sw_w1r;                               // 0x80
      12'h021: axi_rdata <= reg_sw_w1i;                               // 0x84
      12'h022: axi_rdata <= reg_sw_w2r;                               // 0x88
      12'h023: axi_rdata <= reg_sw_w2i;                               // 0x8C
      12'h024: axi_rdata <= v_c11;                                    // 0x90
      12'h025: axi_rdata <= v_c22;                                    // 0x94
      12'h026: axi_rdata <= v_c12r;                                   // 0x98
      12'h027: axi_rdata <= v_c12i;                                   // 0x9C
      12'h028: axi_rdata <= v_cblk;                                   // 0xA0
      12'h01C, 12'h01D, 12'h01E, 12'h01F:
               axi_rdata <= crpa_coef[s_axi_araddr[5:2]];
      default: axi_rdata <= 32'd0;
    endcase
    end
  end

  // ==========================================================================
  //  CDC: control  s_axi_aclk -> clk   (two-flop synchronisers)
  //  These bits are quasi-static software settings, so bit-wise
  //  synchronisation is sufficient; no bus-coherency requirement.
  // ==========================================================================
  (* ASYNC_REG = "TRUE" *) reg [8:0] ctrl_meta = 9'd0;
  (* ASYNC_REG = "TRUE" *) reg [8:0] ctrl_sync = 9'd0;
  wire [8:0] ctrl_raw = {reg_control[8], 1'b0, reg_control[6:0]};

  always @(posedge clk) begin
    ctrl_meta <= ctrl_raw;
    ctrl_sync <= ctrl_meta;
  end

  wire pass_en   = ctrl_sync[0];
  wire mute      = ctrl_sync[1];
  wire swap_iq   = ctrl_sync[2];
  wire ch1_copy  = ctrl_sync[3];
  wire lcmv_en   = ctrl_sync[4];
  wire lcmv_sw   = ctrl_sync[5];
  wire cnt_clear = ctrl_sync[8];

  // LCMV settings, same "*ctrl_meta" synchroniser pattern (and therefore the
  // same existing false-path constraint). The four words are quasi-static,
  // but a write can land while the synchroniser samples, so the value is
  // only taken once it has been identical for two clocks: a torn mix of old
  // and new bits never reaches the solver.
  (* ASYNC_REG = "TRUE" *) reg [79:0] lcmv_ctrl_meta = {LCMV_CAL_ONE, 16'd0, 16'd60, 16'd30};
  (* ASYNC_REG = "TRUE" *) reg [79:0] lcmv_ctrl_sync = {LCMV_CAL_ONE, 16'd0, 16'd60, 16'd30};
  reg [79:0] lcmv_cfg_prev = {LCMV_CAL_ONE, 16'd0, 16'd60, 16'd30};
  reg [79:0] lcmv_cfg      = {LCMV_CAL_ONE, 16'd0, 16'd60, 16'd30};

  always @(posedge clk) begin
    lcmv_ctrl_meta <= {reg_lcmv_cal, reg_lcmv_eps, reg_lcmv_jam, reg_lcmv_gps};
    lcmv_ctrl_sync <= lcmv_ctrl_meta;
    lcmv_cfg_prev  <= lcmv_ctrl_sync;
    if (lcmv_ctrl_sync == lcmv_cfg_prev)
      lcmv_cfg <= lcmv_ctrl_sync;
  end

  wire signed [15:0] lcmv_gps   = lcmv_cfg[15:0];
  wire signed [15:0] lcmv_jam   = lcmv_cfg[31:16];
  wire        [15:0] lcmv_eps   = lcmv_cfg[47:32];
  wire signed [15:0] lcmv_cal_r = lcmv_cfg[63:48];
  wire signed [15:0] lcmv_cal_i = lcmv_cfg[79:64];

  // PS weights: same synchroniser + stability filter. The commit word and its
  // sequence bit change together on the AXI side; a set is only used once all
  // 101 bits have been stable for two clocks, so it is never torn.
  (* ASYNC_REG = "TRUE" *) reg [100:0] lcmv_sw_ctrl_meta = {1'b0, 75'd0, 25'sd65536};
  (* ASYNC_REG = "TRUE" *) reg [100:0] lcmv_sw_ctrl_sync = {1'b0, 75'd0, 25'sd65536};
  reg [100:0] lcmv_sw_prev = {1'b0, 75'd0, 25'sd65536};
  reg [100:0] lcmv_sw_cfg  = {1'b0, 75'd0, 25'sd65536};

  always @(posedge clk) begin
    lcmv_sw_ctrl_meta <= {reg_sw_seq, reg_sw_commit};
    lcmv_sw_ctrl_sync <= lcmv_sw_ctrl_meta;
    lcmv_sw_prev      <= lcmv_sw_ctrl_sync;
    if (lcmv_sw_ctrl_sync == lcmv_sw_prev)
      lcmv_sw_cfg <= lcmv_sw_ctrl_sync;
  end

  // ==========================================================================
  //  Elastic buffer (one per channel)
  //  RX and TX sample strobes are both in the clk domain and run at the same
  //  average rate, but their phase relationship is not guaranteed.  A shallow
  //  synchronous FIFO absorbs that phase difference.  Occupancy is steered
  //  toward half-full at start-up by holding reads off until the buffer has
  //  filled to the midpoint.
  // ==========================================================================
  // Distributed (LUT) RAM: at depth 32 a block RAM would be wasteful, and
  // block RAM is better kept free for the future CRPA implementation.
  (* ram_style = "distributed" *) reg [31:0] fifo0 [0:(1<<AW)-1];
  (* ram_style = "distributed" *) reg [31:0] fifo1 [0:(1<<AW)-1];
  reg  [AW:0] wptr0 = 0, rptr0 = 0;
  reg  [AW:0] wptr1 = 0, rptr1 = 0;

  wire [AW:0] level0 = wptr0 - rptr0;
  wire [AW:0] level1 = wptr1 - rptr1;
  wire        full0  = (level0 == (1<<AW));
  wire        empty0 = (level0 == 0);
  wire        full1  = (level1 == (1<<AW));
  wire        empty1 = (level1 == 0);

  // Prime the buffer to half depth before the first read, so that a small
  // phase error in either direction cannot immediately underflow.
  reg  primed0 = 1'b0, primed1 = 1'b0;
  always @(posedge clk) begin
    if (rst || !pass_en) begin
      primed0 <= 1'b0;
      primed1 <= 1'b0;
    end else begin
      if (level0 >= (1<<(AW-1))) primed0 <= 1'b1;
      if (level1 >= (1<<(AW-1))) primed1 <= 1'b1;
    end
  end

  // With the LCMV active the ch0 buffer is fed by the LCMV output strobe,
  // otherwise by the raw RX strobe. Both run at the RX sample rate.
  wire lcmv_out_valid;
  wire lcmv_use;                 // LCMV output selected for ch0 (see below)
  wire wr_en0 = pass_en & (lcmv_use ? lcmv_out_valid :
                                      (adc_valid_i0 & adc_enable_i0));
  wire rd_en0 = pass_en & dac_valid_i0 & primed0;
  wire wr_en1 = pass_en & adc_valid_i1 & adc_enable_i1;
  wire rd_en1 = pass_en & dac_valid_i1 & primed1;

  // ==========================================================================
  //  PROCESSING CORE
  //
  //  lcmv_en = 0 : identity. proc_i/proc_q are the raw RX samples.
  //  lcmv_en = 1 : channel 0 carries the 2-element LCMV output (RX1 and RX2
  //                combined: gain 1 toward the GPS angle, null toward the
  //                jammer angle). Channel 1 is left as raw RX2.
  // ==========================================================================
  // --------------------------------------------------------------------------
  //  LCMV, Source/HDL/crpa/. Runs only while pass_en & lcmv_en, and is
  //  held in reset otherwise so it restarts cleanly. Its weights come from the GPS / jammer angles, so
  //  nothing adapts: a new weight set is computed about every 70 clocks and
  //  swapped in atomically.
  //
  //  The beamformer is a fixed 5-clock pipeline that advances every clock;
  //  lcmv_out_valid is the RX pair strobe delayed by the same 5 clocks, and
  //  lcmv_use (which output ch0 takes) is delayed with it, so mode changes
  //  line up with the samples. The output is already saturated to 12 bits.
  // --------------------------------------------------------------------------
  localparam LCMV_LATENCY = 5;

  wire lcmv_run  = pass_en & lcmv_en;

  wire signed [11:0] lcmv_y_re, lcmv_y_im;
  wire               lcmv_out_sat;
  wire               lcmv_wvalid, lcmv_busy;
  wire [31:0]        lcmv_updates;
  wire signed [24:0] lcmv_w1_re, lcmv_w1_im, lcmv_w2_re, lcmv_w2_im;

  crpa_lcmv_2ch u_lcmv (
    .clk           (clk),
    .rst           (rst),
    .enable        (lcmv_run),
    .gps_angle     (lcmv_gps),
    .jam_angle     (lcmv_jam),
    .eps           (lcmv_eps),
    .cal_re        (lcmv_cal_r),
    .cal_im        (lcmv_cal_i),
    .sw_mode       (lcmv_sw),
    .sw_seq        (lcmv_sw_cfg[100]),
    .sw_w1_re      (lcmv_sw_cfg[24:0]),
    .sw_w1_im      (lcmv_sw_cfg[49:25]),
    .sw_w2_re      (lcmv_sw_cfg[74:50]),
    .sw_w2_im      (lcmv_sw_cfg[99:75]),
    .in_valid      (adc_valid_i0 & adc_valid_i1 & adc_enable_i0 & adc_enable_i1),
    .x1_re         (adc_data_i0[11:0]),
    .x1_im         (adc_data_q0[11:0]),
    .x2_re         (adc_data_i1[11:0]),
    .x2_im         (adc_data_q1[11:0]),
    .out_valid     (lcmv_out_valid),
    .y_re          (lcmv_y_re),
    .y_im          (lcmv_y_im),
    .out_sat       (lcmv_out_sat),
    .weights_valid (lcmv_wvalid),
    .busy          (lcmv_busy),
    .update_count  (lcmv_updates),
    .w1_re         (lcmv_w1_re),
    .w1_im         (lcmv_w1_im),
    .w2_re         (lcmv_w2_re),
    .w2_im         (lcmv_w2_im)
  );

  // ---- 2x2 covariance R = E[x x^H] of the raw array snapshots -------------
  //  Runs continuously (also with the LCMV off), so the PS always has the
  //  current R: for the LCMV weights and for jammer detection.
  wire [31:0] cov_r11, cov_r22, cov_r12r, cov_r12i, cov_blocks;

  crpa_cov_2ch #(.LOG2N(12)) u_cov (
    .clk      (clk),
    .rst      (rst),
    .in_valid (adc_valid_i0 & adc_valid_i1 & adc_enable_i0 & adc_enable_i1),
    .x1_re    (adc_data_i0[11:0]),
    .x1_im    (adc_data_q0[11:0]),
    .x2_re    (adc_data_i1[11:0]),
    .x2_im    (adc_data_q1[11:0]),
    .r11      (cov_r11),
    .r22      (cov_r22),
    .r12_re   (cov_r12r),
    .r12_im   (cov_r12i),
    .blocks   (cov_blocks)
  );

  reg [LCMV_LATENCY-1:0] lcmv_use_d = {LCMV_LATENCY{1'b0}};
  always @(posedge clk) begin
    if (rst) lcmv_use_d <= {LCMV_LATENCY{1'b0}};
    else     lcmv_use_d <= {lcmv_use_d[LCMV_LATENCY-2:0], lcmv_run & lcmv_wvalid};
  end
  assign lcmv_use = lcmv_use_d[LCMV_LATENCY-1];
  wire lcmv_active = lcmv_run & lcmv_wvalid;

  wire [15:0] lcmv_i = {{4{lcmv_y_re[11]}}, lcmv_y_re};
  wire [15:0] lcmv_q = {{4{lcmv_y_im[11]}}, lcmv_y_im};

  wire [15:0] proc_i0 = lcmv_use ? lcmv_i : adc_data_i0;
  wire [15:0] proc_q0 = lcmv_use ? lcmv_q : adc_data_q0;
  wire [15:0] proc_i1 = adc_data_i1;
  wire [15:0] proc_q1 = adc_data_q1;
  // ======================= END PROCESSING CORE ==============================

  reg  [31:0] rd_data0 = 32'd0;
  reg  [31:0] rd_data1 = 32'd0;

  always @(posedge clk) begin
    if (rst) begin
      wptr0 <= 0; rptr0 <= 0;
      wptr1 <= 0; rptr1 <= 0;
    end else begin
      if (wr_en0 && !full0) begin
        fifo0[wptr0[AW-1:0]] <= {proc_q0, proc_i0};
        wptr0 <= wptr0 + 1'b1;
      end
      if (rd_en0 && !empty0) begin
        rd_data0 <= fifo0[rptr0[AW-1:0]];
        rptr0 <= rptr0 + 1'b1;
      end
      if (wr_en1 && !full1) begin
        fifo1[wptr1[AW-1:0]] <= {proc_q1, proc_i1};
        wptr1 <= wptr1 + 1'b1;
      end
      if (rd_en1 && !empty1) begin
        rd_data1 <= fifo1[rptr1[AW-1:0]];
        rptr1 <= rptr1 + 1'b1;
      end
    end
  end

  // ==========================================================================
  //  Output mux  (requirement 51: processed I/Q feeds the normal TX path)
  //    pass_en = 0 -> vendor DMA / DDS data, i.e. the untouched E310 baseline
  //    pass_en = 1 -> samples that came from RX through this block
  // ==========================================================================
  wire [15:0] pt_i0 = swap_iq ? rd_data0[31:16] : rd_data0[15:0];
  wire [15:0] pt_q0 = swap_iq ? rd_data0[15:0]  : rd_data0[31:16];
  wire [31:0] src1  = ch1_copy ? rd_data0 : rd_data1;
  wire [15:0] pt_i1 = swap_iq ? src1[31:16] : src1[15:0];
  wire [15:0] pt_q1 = swap_iq ? src1[15:0]  : src1[31:16];

  // ==========================================================================
  //  OUTPUT SAMPLE ALIGNMENT
  //
  //  axi_ad9361 is ASYMMETRIC about where the 12 significant bits sit inside a
  //  16-bit word, and an identity passthrough has to convert between the two:
  //
  //    RX  axi_ad9361_rx_channel.v:144 instantiates
  //          ad_datafmt #(.DATA_WIDTH(12))            -> BITS_PER_SAMPLE 16
  //        whose output is RIGHT-aligned: ad_datafmt.v:100-101 place the
  //        sample in [11:0] and :96 fills [15:12] with sign extension.
  //
  //    TX  axi_ad9361_tx_channel.v:301 consumes
  //          dma_data[15:4]
  //        i.e. the 12 bits LEFT-aligned, discarding [3:0].
  //
  //  Copying RX straight into TX therefore hands the DAC sample>>4: 24 dB down
  //  with the four least significant bits thrown away. At the RX noise-floor
  //  magnitudes measured on this board (|sample| ~ 20 LSB) that leaves one or
  //  two bits of signal. Shifting left by 4 restores unity gain through the
  //  block.
  //
  //  The shift applies to the PASSTHROUGH branch only. The vendor DMA branch
  //  (dma_dac_data_*, from util_rfifo) is already in the left-aligned TX
  //  format, so pass_en = 0 must still deliver exact vendor behaviour.
  //
  //  Only the low 12 bits are taken, so this is correct whatever the ADC
  //  data-format control does with [15:12] (sign-extend or zero-fill).
  //
  //  FOR THE FUTURE CRPA: the processing core works in RX format. If it ever
  //  produces a value outside the 12-bit signed range -- a weighted sum of two
  //  elements easily can -- it must SATURATE before reaching this point.
  //  Truncating to [11:0] here would wrap, turning a large sample into an
  //  opposite-signed small one. That is a silent, hard-to-see failure.
  // ==========================================================================
  wire [15:0] al_i0 = {pt_i0[11:0], 4'b0000};
  wire [15:0] al_q0 = {pt_q0[11:0], 4'b0000};
  wire [15:0] al_i1 = {pt_i1[11:0], 4'b0000};
  wire [15:0] al_q1 = {pt_q1[11:0], 4'b0000};

  assign dac_data_i0 = mute ? 16'd0 : (pass_en ? al_i0 : dma_dac_data_i0);
  assign dac_data_q0 = mute ? 16'd0 : (pass_en ? al_q0 : dma_dac_data_q0);
  assign dac_data_i1 = mute ? 16'd0 : (pass_en ? al_i1 : dma_dac_data_i1);
  assign dac_data_q1 = mute ? 16'd0 : (pass_en ? al_q1 : dma_dac_data_q1);

  // ==========================================================================
  //  Counters and sticky flags  (requirement 59: prove data is flowing)
  // ==========================================================================
  reg [31:0] rx_cnt0 = 32'd0, tx_cnt0 = 32'd0;
  reg [31:0] rx_cnt1 = 32'd0, tx_cnt1 = 32'd0;
  reg [31:0] ovf_cnt = 32'd0, unf_cnt = 32'd0;
  reg [31:0] rx_snap0 = 32'd0, tx_snap0 = 32'd0, rx_snap1 = 32'd0;
  reg        ovf_sticky = 1'b0, unf_sticky = 1'b0;
  reg [31:0] lcmv_sat_cnt = 32'd0;
  reg        lcmv_sat_sticky = 1'b0;

  always @(posedge clk) begin
    if (rst || cnt_clear) begin
      rx_cnt0 <= 32'd0; tx_cnt0 <= 32'd0;
      rx_cnt1 <= 32'd0; tx_cnt1 <= 32'd0;
      ovf_cnt <= 32'd0; unf_cnt <= 32'd0;
      ovf_sticky <= 1'b0; unf_sticky <= 1'b0;
      lcmv_sat_cnt <= 32'd0; lcmv_sat_sticky <= 1'b0;
      rx_snap0 <= 32'd0; tx_snap0 <= 32'd0; rx_snap1 <= 32'd0;
    end else begin
      if (adc_valid_i0) begin
        rx_cnt0  <= rx_cnt0 + 1'b1;
        rx_snap0 <= {adc_data_q0, adc_data_i0};
      end
      if (adc_valid_i1) begin
        rx_cnt1  <= rx_cnt1 + 1'b1;
        rx_snap1 <= {adc_data_q1, adc_data_i1};
      end
      if (dac_valid_i0) begin
        tx_cnt0  <= tx_cnt0 + 1'b1;
        tx_snap0 <= {dac_data_q0, dac_data_i0};
      end
      if (dac_valid_i1)
        tx_cnt1 <= tx_cnt1 + 1'b1;

      if (wr_en0 && full0) begin
        ovf_cnt    <= ovf_cnt + 1'b1;
        ovf_sticky <= 1'b1;
      end
      if (rd_en0 && empty0) begin
        unf_cnt    <= unf_cnt + 1'b1;
        unf_sticky <= 1'b1;
      end
      if (lcmv_use && lcmv_out_valid && lcmv_out_sat) begin
        lcmv_sat_cnt    <= lcmv_sat_cnt + 1'b1;
        lcmv_sat_sticky <= 1'b1;
      end
    end
  end

  wire [31:0] status_w;
  assign status_w = { 8'd0, lcmv_sw, lcmv_sat_sticky, 2'd0, lcmv_active, 1'b0,
                      lcmv_en, pass_en,
                      6'd0, unf_sticky, ovf_sticky,
                      full1, empty1, full0, empty0,
                      dac_enable_q0, dac_enable_i0,
                      adc_enable_q0, adc_enable_i0 };

  // Occupancy is (AW+1) bits wide; zero-extend each to 6 bits for the register.
  // FIFO_ADDR_WIDTH must be <= 5 for the field to hold the full range.
  wire [5:0] level0_6 = level0;
  wire [5:0] level1_6 = level1;
  wire [31:0] level_w = { 18'd0, level1_6, 2'd0, level0_6 };

  // ==========================================================================
  //  CDC: status  clk -> s_axi_aclk
  //  Every 256 clk cycles the whole status set is latched into cap_* and a
  //  toggle is flipped.  cap_* is then stable for the following 256 cycles,
  //  which is far longer than the two-flop synchroniser latency, so the AXI
  //  side always copies a coherent snapshot.
  // ==========================================================================
  reg  [7:0]  cap_div    = 8'd0;
  reg         cap_toggle = 1'b0;
  reg  [31:0] cap_status, cap_rx0, cap_tx0, cap_ovf, cap_unf;
  reg  [31:0] cap_rxs0, cap_txs0, cap_lvl, cap_rx1, cap_tx1, cap_rxs1;
  reg  [31:0] cap_lw1r, cap_lw1i, cap_lw2r, cap_lw2i, cap_lst, cap_lupd, cap_lsat;
  reg  [31:0] cap_c11, cap_c22, cap_c12r, cap_c12i, cap_cblk;

  always @(posedge clk) begin
    cap_div <= cap_div + 1'b1;
    if (cap_div == 8'd0) begin
      cap_status <= status_w;
      cap_rx0    <= rx_cnt0;
      cap_tx0    <= tx_cnt0;
      cap_ovf    <= ovf_cnt;
      cap_unf    <= unf_cnt;
      cap_rxs0   <= rx_snap0;
      cap_txs0   <= tx_snap0;
      cap_lvl    <= level_w;
      cap_rx1    <= rx_cnt1;
      cap_tx1    <= tx_cnt1;
      cap_rxs1   <= rx_snap1;
      cap_lw1r   <= {{7{lcmv_w1_re[24]}}, lcmv_w1_re};
      cap_lw1i   <= {{7{lcmv_w1_im[24]}}, lcmv_w1_im};
      cap_lw2r   <= {{7{lcmv_w2_re[24]}}, lcmv_w2_re};
      cap_lw2i   <= {{7{lcmv_w2_im[24]}}, lcmv_w2_im};
      cap_lst    <= {27'd0, lcmv_sw, lcmv_sat_sticky, lcmv_busy, lcmv_wvalid, lcmv_en};
      cap_lupd   <= lcmv_updates;
      cap_lsat   <= lcmv_sat_cnt;
      cap_c11    <= cov_r11;      // the five covariance words change on the
      cap_c22    <= cov_r22;      // same clock, so this capture is coherent
      cap_c12r   <= cov_r12r;
      cap_c12i   <= cov_r12i;
      cap_cblk   <= cov_blocks;
      cap_toggle <= ~cap_toggle;
    end
  end

  (* ASYNC_REG = "TRUE" *) reg tog_meta = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg tog_sync = 1'b0;
  reg                         tog_prev = 1'b0;

  reg [31:0] h_status = 32'd0, h_rx0 = 32'd0, h_tx0 = 32'd0;
  reg [31:0] h_ovf = 32'd0, h_unf = 32'd0, h_rxs0 = 32'd0, h_txs0 = 32'd0;
  reg [31:0] h_lvl = 32'd0, h_rx1 = 32'd0, h_tx1 = 32'd0, h_rxs1 = 32'd0;
  reg [31:0] h_lw1r = 32'd0, h_lw1i = 32'd0, h_lw2r = 32'd0, h_lw2i = 32'd0;
  reg [31:0] h_lst = 32'd0, h_lupd = 32'd0, h_lsat = 32'd0;
  reg [31:0] h_c11 = 32'd0, h_c22 = 32'd0, h_c12r = 32'd0, h_c12i = 32'd0, h_cblk = 32'd0;

  always @(posedge s_axi_aclk) begin
    tog_meta <= cap_toggle;
    tog_sync <= tog_meta;
    tog_prev <= tog_sync;
    if (tog_sync != tog_prev) begin
      h_status <= cap_status;
      h_rx0    <= cap_rx0;
      h_tx0    <= cap_tx0;
      h_ovf    <= cap_ovf;
      h_unf    <= cap_unf;
      h_rxs0   <= cap_rxs0;
      h_txs0   <= cap_txs0;
      h_lvl    <= cap_lvl;
      h_rx1    <= cap_rx1;
      h_tx1    <= cap_tx1;
      h_rxs1   <= cap_rxs1;
      h_lw1r   <= cap_lw1r;
      h_lw1i   <= cap_lw1i;
      h_lw2r   <= cap_lw2r;
      h_lw2i   <= cap_lw2i;
      h_lst    <= cap_lst;
      h_lupd   <= cap_lupd;
      h_lsat   <= cap_lsat;
      h_c11    <= cap_c11;
      h_c22    <= cap_c22;
      h_c12r   <= cap_c12r;
      h_c12i   <= cap_c12i;
      h_cblk   <= cap_cblk;
    end
  end

  assign v_status   = h_status;
  assign v_rx_cnt0  = h_rx0;
  assign v_tx_cnt0  = h_tx0;
  assign v_ovf_cnt  = h_ovf;
  assign v_unf_cnt  = h_unf;
  assign v_rx_snap0 = h_rxs0;
  assign v_tx_snap0 = h_txs0;
  assign v_level    = h_lvl;
  assign v_rx_cnt1  = h_rx1;
  assign v_tx_cnt1  = h_tx1;
  assign v_rx_snap1 = h_rxs1;
  assign v_lw1r     = h_lw1r;
  assign v_lw1i     = h_lw1i;
  assign v_lw2r     = h_lw2r;
  assign v_lw2i     = h_lw2i;
  assign v_lst      = h_lst;
  assign v_lupd     = h_lupd;
  assign v_lsat     = h_lsat;
  assign v_c11      = h_c11;
  assign v_c22      = h_c22;
  assign v_c12r     = h_c12r;
  assign v_c12i     = h_c12i;
  assign v_cblk     = h_cblk;

endmodule
