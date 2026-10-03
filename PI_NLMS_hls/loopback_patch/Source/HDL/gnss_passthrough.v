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
//    v1.3: the PROCESSING CORE now holds two 2-element anti-jam (null-steering)
//    cores next to the Phase 1 identity path, chosen at run time by
//    CONTROL[5:4]:
//        0  bypass   -- Phase 1 identity, bit-for-bit as v1.1 (the default)
//        1  PI       -- pi_power_inversion.v, Compton power inversion (RTL)
//        2  PI-NLMS  -- pi_nlms, power-normalised block NLMS (Vitis HLS)
//    Both cores combine RX ch0 and ch1 into ONE nulled stream; set
//    CONTROL[3] ch1_copy to send it to both DACs.
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
//                                    [5:4] core_sel  0 = bypass (identity)
//                                                    1 = PI (RTL)
//                                                    2 = PI-NLMS (HLS)
//                                                    3 = reserved (bypass)
//                                    [8]  cnt_clear  1 = hold counters cleared
//    0x10    STATUS          RO      [0]  adc_enable_i0   [1]  adc_enable_q0
//                                    [2]  dac_enable_i0   [3]  dac_enable_q0
//                                    [4]  fifo0_empty     [5]  fifo0_full
//                                    [6]  fifo1_empty     [7]  fifo1_full
//                                    [8]  overflow_sticky [9]  underflow_sticky
//                                    [11:10] core_sel (as seen in the clk domain)
//                                    [12] nlms_drop_sticky  PI-NLMS refused a
//                                         sample (input stream not ready)
//                                    [13] nlms_cfg_done     PI-NLMS mu_shift
//                                         register programmed, core running
//                                    [16] pass_en (as seen in the clk domain)
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
//    0x40..  CRPA_COEF[0..15] RW     CRPA parameters:
//            CRPA_COEF[0] 0x40       PI alpha, signed Q8.8 [15:0]. Default
//                                    0x0100 = 1.0
//            CRPA_COEF[1] 0x44       PI-NLMS mu_shift_ctrl, signed [15:0].
//                                    Default -3 (0xFFFFFFFD). Lower = faster
//                                    convergence; values below -4 act as -4.
//            CRPA_COEF[2..15]        stored and returned, not consumed.
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
  // v1.3 -- adds the PI (RTL) and PI-NLMS (HLS) nulling cores, selected by
  // CONTROL[5:4]; the default remains the v1.1 identity path.
  localparam [31:0] CORE_VERSION = 32'h00010003;   // v1.3
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

  integer ci;
  initial begin
    for (ci = 0; ci < 16; ci = ci + 1) crpa_coef[ci] = 32'd0;
    crpa_coef[0] = 32'h0000_0100;   // PI alpha = 1.0 (Q8.8)
    crpa_coef[1] = 32'hFFFF_FFFD;   // PI-NLMS mu_shift_ctrl = -3
  end

  always @(posedge s_axi_aclk) begin
    if (s_axi_aresetn == 1'b0) begin
      reg_scratch <= 32'd0;
      reg_control <= 32'd0;
    end else if (reg_wr) begin
      case (reg_waddr)
        12'h002: reg_scratch <= s_axi_wdata;   // 0x08
        12'h003: reg_control <= s_axi_wdata;   // 0x0C
        default: ;
      endcase
      if (reg_waddr >= 12'h010 && reg_waddr <= 12'h01F)   // 0x40..0x7C
        crpa_coef[reg_waddr[3:0]] <= s_axi_wdata;
    end
  end

  // Status view, refreshed from the clk domain (see CDC section).
  wire [31:0] v_status, v_rx_cnt0, v_tx_cnt0, v_ovf_cnt, v_unf_cnt;
  wire [31:0] v_rx_snap0, v_tx_snap0, v_level, v_rx_cnt1, v_tx_cnt1, v_rx_snap1;

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
      12'h010, 12'h011, 12'h012, 12'h013,
      12'h014, 12'h015, 12'h016, 12'h017,
      12'h018, 12'h019, 12'h01A, 12'h01B,
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
  wire [8:0] ctrl_raw = {reg_control[8], 2'b00, reg_control[5:0]};

  always @(posedge clk) begin
    ctrl_meta <= ctrl_raw;
    ctrl_sync <= ctrl_meta;
  end

  wire pass_en   = ctrl_sync[0];
  wire mute      = ctrl_sync[1];
  wire swap_iq   = ctrl_sync[2];
  wire ch1_copy  = ctrl_sync[3];
  wire [1:0] core_sel = ctrl_sync[5:4];
  wire cnt_clear = ctrl_sync[8];

  wire use_pi   = (core_sel == 2'd1);
  wire use_nlms = (core_sel == 2'd2);
  wire use_core = use_pi | use_nlms;

  // ==========================================================================
  //  CDC: CRPA parameters  s_axi_aclk -> clk
  //  Quasi-static software settings, same two-flop pattern as CONTROL above.
  //  A multi-bit value can be caught mid-change for one clock; both consumers
  //  simply follow the settled value a clock later.
  // ==========================================================================
  (* ASYNC_REG = "TRUE" *) reg [15:0] crpa_alpha_meta = 16'h0100;
  (* ASYNC_REG = "TRUE" *) reg [15:0] crpa_alpha_sync = 16'h0100;
  (* ASYNC_REG = "TRUE" *) reg [15:0] nlms_mu_meta    = 16'hFFFD;
  (* ASYNC_REG = "TRUE" *) reg [15:0] nlms_mu_sync    = 16'hFFFD;

  always @(posedge clk) begin
    crpa_alpha_meta <= crpa_coef[0][15:0];
    crpa_alpha_sync <= crpa_alpha_meta;
    nlms_mu_meta    <= crpa_coef[1][15:0];
    nlms_mu_sync    <= nlms_mu_meta;
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

  wire rd_en0 = pass_en & dac_valid_i0 & primed0;
  wire rd_en1 = pass_en & dac_valid_i1 & primed1;

  // ==========================================================================
  //  PROCESSING CORE
  //  proc_i/proc_q are the values written into the elastic buffer, on
  //  wr_en0/wr_en1 (defined just after this section).
  //
  //  core_sel = 0 (bypass): Phase 1 identity -- the raw RX samples, written on
  //  each channel's own adc_valid, exactly as v1.1.
  //
  //  core_sel = 1 / 2: one of two M=2 null-steering cores combines element 0
  //  (adc_*_i0/q0) and element 1 (adc_*_i1/q1) into a single nulled stream.
  //  Both elements must be sampled in the same cycle, so the cores trigger on
  //  the AND of all four RX valid/enable strobes (true every second l_clk in
  //  AD9361 2R2T mode). The result is written to BOTH buffers when the core
  //  says it is ready, not when the raw sample arrived -- neither core is
  //  combinational. Only the selected core runs; the other is held in reset,
  //  so every enable restarts adaptation from its quiescent weights.
  //
  //  Both cores work on the RX format as plain signed integers (12-bit,
  //  right-aligned, sign-extended into [15:12]) and their outputs are
  //  SATURATED to the 12-bit signed range here, before the OUTPUT SAMPLE
  //  ALIGNMENT stage below takes [11:0] -- truncating there would wrap.
  // ==========================================================================
  wire crpa_sample_valid = adc_valid_i0 & adc_enable_i0
                         & adc_valid_q0 & adc_enable_q0
                         & adc_valid_i1 & adc_enable_i1
                         & adc_valid_q1 & adc_enable_q1;

  // --------------------------------------------------------------------------
  //  Core 1: PI -- pi_power_inversion.v (Compton power inversion, fixed alpha)
  // --------------------------------------------------------------------------
  localparam integer CRPA_DATA_W      = 16;
  localparam integer CRPA_WEIGHT_W    = 32;
  localparam integer CRPA_WEIGHT_FRAC = 20;
  localparam integer CRPA_ALPHA_W     = 16;
  localparam integer CRPA_ALPHA_FRAC  = 8;
  localparam integer CRPA_LPF_SHIFT   = 18;
  // Must match pi_power_inversion's own S_W for M=2: PROD1_W = DATA_W+WEIGHT_W+1,
  // SUM_GROWTH = clog2(2) = 1, S_W = PROD1_W + SUM_GROWTH.
  localparam integer CRPA_S_W         = CRPA_DATA_W + CRPA_WEIGHT_W + 2;

  wire signed [CRPA_S_W-1:0] crpa_s_re, crpa_s_im;
  wire                       crpa_s_valid;
  wire [2*CRPA_WEIGHT_W-1:0] crpa_w_re, crpa_w_im;
  wire                       crpa_weights_valid;
  wire                       crpa_aresetn = ~rst & pass_en & use_pi;

  pi_power_inversion #(
      .M           (2),
      .DATA_W      (CRPA_DATA_W),
      .WEIGHT_W    (CRPA_WEIGHT_W),
      .WEIGHT_FRAC (CRPA_WEIGHT_FRAC),
      .ALPHA_W     (CRPA_ALPHA_W),
      .ALPHA_FRAC  (CRPA_ALPHA_FRAC),
      .ALPHA_INIT  (1 << CRPA_ALPHA_FRAC),
      .LPF_SHIFT   (CRPA_LPF_SHIFT)
  ) u_crpa_core (
      .clk           (clk),
      .aresetn       (crpa_aresetn),
      .x_re          ({adc_data_i1[CRPA_DATA_W-1:0], adc_data_i0[CRPA_DATA_W-1:0]}),
      .x_im          ({adc_data_q1[CRPA_DATA_W-1:0], adc_data_q0[CRPA_DATA_W-1:0]}),
      .sample_valid  (crpa_sample_valid),
      .alpha_in      (crpa_alpha_sync),
      .alpha_wr      (1'b1),            // alpha follows CRPA_COEF[0]
      .adapt_en      (pass_en),
      .s_re          (crpa_s_re),
      .s_im          (crpa_s_im),
      .s_valid       (crpa_s_valid),
      .w_re          (crpa_w_re),
      .w_im          (crpa_w_im),
      .weights_valid (crpa_weights_valid)
  );

  // Round to nearest, saturate to 12-bit signed.
  function signed [11:0] crpa_sat12(input signed [CRPA_S_W-1:0] v);
    reg signed [CRPA_S_W-1:0] rounded;
    localparam signed [CRPA_S_W-1:0] SAT_MAX = 2047;
    localparam signed [CRPA_S_W-1:0] SAT_MIN = -2048;
    begin
      rounded = (v + (1 <<< (CRPA_WEIGHT_FRAC-1))) >>> CRPA_WEIGHT_FRAC;
      if (rounded > SAT_MAX)      crpa_sat12 = SAT_MAX[11:0];
      else if (rounded < SAT_MIN) crpa_sat12 = SAT_MIN[11:0];
      else                        crpa_sat12 = rounded[11:0];
    end
  endfunction

  wire signed [11:0] crpa_re12 = crpa_sat12(crpa_s_re);
  wire signed [11:0] crpa_im12 = crpa_sat12(crpa_s_im);

  // --------------------------------------------------------------------------
  //  Core 2: PI-NLMS -- Vitis HLS top function pi_nlms (Source/HLS/pi_nlms).
  //  out = 0.707*x0 + w*x1, w adapted once per 1024 samples with a step
  //  normalised to the element-1 input power. One output per input pair on
  //  out_r_TVALID, after the core's pipeline latency.
  //
  //  The HLS core keeps mu_shift_ctrl in its own AXI4-Lite register (0x10) on
  //  ap_clk = clk; the small writer below copies CRPA_COEF[1] into it after
  //  every core reset and whenever the value changes, and samples are offered
  //  only once that write has completed.
  // --------------------------------------------------------------------------
  // Registered synchronous reset. The power-up value of 1 guarantees a real
  // 1->0 edge on the first clock: the HLS RTL derives its internal reset with
  // always @(*), which a net already 0 at time 0 never triggers in simulation.
  reg         nlms_rst_n = 1'b1;
  always @(posedge clk)
    nlms_rst_n <= ~rst & pass_en & use_nlms;

  wire        nl_awready, nl_wready, nl_bvalid;
  reg         nl_awvalid  = 1'b0, nl_wvalid = 1'b0, nl_bready = 1'b0;
  reg  [15:0] nl_mu_wr    = 16'd0;
  reg         nl_cfg_done = 1'b0;
  reg  [1:0]  nl_st       = 2'd0;
  localparam [1:0] NL_IDLE = 2'd0, NL_REQ = 2'd1, NL_RESP = 2'd2;

  always @(posedge clk) begin
    if (!nlms_rst_n) begin
      nl_st       <= NL_IDLE;
      nl_awvalid  <= 1'b0;
      nl_wvalid   <= 1'b0;
      nl_bready   <= 1'b0;
      nl_cfg_done <= 1'b0;
    end else begin
      case (nl_st)
        NL_IDLE:
          if (!nl_cfg_done || nl_mu_wr != nlms_mu_sync) begin
            nl_mu_wr   <= nlms_mu_sync;
            nl_awvalid <= 1'b1;
            nl_wvalid  <= 1'b1;
            nl_st      <= NL_REQ;
          end
        NL_REQ: begin
          if (nl_awready) nl_awvalid <= 1'b0;
          if (nl_wready)  nl_wvalid  <= 1'b0;
          if ((!nl_awvalid || nl_awready) && (!nl_wvalid || nl_wready)) begin
            nl_bready <= 1'b1;
            nl_st     <= NL_RESP;
          end
        end
        NL_RESP:
          if (nl_bvalid) begin
            nl_bready   <= 1'b0;
            nl_cfg_done <= 1'b1;
            nl_st       <= NL_IDLE;
          end
        default: nl_st <= NL_IDLE;
      endcase
    end
  end

  wire        nlms_in_valid = crpa_sample_valid & nl_cfg_done;
  wire        nlms_in1_ready, nlms_in2_ready;
  wire [31:0] nlms_out_data;
  wire        nlms_out_valid;
  // A refused sample is lost; STATUS[12] latches it. Must never happen while
  // samples arrive no faster than the core's initiation interval.
  wire        nlms_drop = nlms_in_valid & ~(nlms_in1_ready & nlms_in2_ready);

  pi_nlms u_nlms_core (
      .ap_clk                 (clk),
      .ap_rst_n               (nlms_rst_n),
      .in1_TDATA              ({adc_data_q0, adc_data_i0}),
      .in1_TVALID             (nlms_in_valid),
      .in1_TREADY             (nlms_in1_ready),
      .in1_TKEEP              (4'hF),
      .in1_TSTRB              (4'hF),
      .in1_TLAST              (1'b0),
      .in2_TDATA              ({adc_data_q1, adc_data_i1}),
      .in2_TVALID             (nlms_in_valid),
      .in2_TREADY             (nlms_in2_ready),
      .in2_TKEEP              (4'hF),
      .in2_TSTRB              (4'hF),
      .in2_TLAST              (1'b0),
      .out_r_TDATA            (nlms_out_data),
      .out_r_TVALID           (nlms_out_valid),
      .out_r_TREADY           (1'b1),
      .out_r_TKEEP            (),
      .out_r_TSTRB            (),
      .out_r_TLAST            (),
      .s_axi_CTRL_BUS_AWVALID (nl_awvalid),
      .s_axi_CTRL_BUS_AWREADY (nl_awready),
      .s_axi_CTRL_BUS_AWADDR  (5'h10),
      .s_axi_CTRL_BUS_WVALID  (nl_wvalid),
      .s_axi_CTRL_BUS_WREADY  (nl_wready),
      .s_axi_CTRL_BUS_WDATA   ({{16{nl_mu_wr[15]}}, nl_mu_wr}),
      .s_axi_CTRL_BUS_WSTRB   (4'hF),
      .s_axi_CTRL_BUS_ARVALID (1'b0),
      .s_axi_CTRL_BUS_ARREADY (),
      .s_axi_CTRL_BUS_ARADDR  (5'h0),
      .s_axi_CTRL_BUS_RVALID  (),
      .s_axi_CTRL_BUS_RREADY  (1'b0),
      .s_axi_CTRL_BUS_RDATA   (),
      .s_axi_CTRL_BUS_RRESP   (),
      .s_axi_CTRL_BUS_BVALID  (nl_bvalid),
      .s_axi_CTRL_BUS_BREADY  (nl_bready),
      .s_axi_CTRL_BUS_BRESP   ()
  );

  // pi_nlms saturates to int16; this path carries 12-bit samples.
  function signed [11:0] sat12_16(input signed [15:0] v);
    begin
      if (v > 16'sd2047)       sat12_16 = 12'sd2047;
      else if (v < -16'sd2048) sat12_16 = -12'sd2048;
      else                     sat12_16 = v[11:0];
    end
  endfunction

  wire signed [11:0] nlms_re12 = sat12_16(nlms_out_data[15:0]);
  wire signed [11:0] nlms_im12 = sat12_16(nlms_out_data[31:16]);

  // --------------------------------------------------------------------------
  //  Selection
  // --------------------------------------------------------------------------
  wire signed [11:0] core_re12  = use_nlms ? nlms_re12 : crpa_re12;
  wire signed [11:0] core_im12  = use_nlms ? nlms_im12 : crpa_im12;
  wire               core_valid = use_nlms ? nlms_out_valid : crpa_s_valid;

  wire [15:0] core_i = {{4{core_re12[11]}}, core_re12};
  wire [15:0] core_q = {{4{core_im12[11]}}, core_im12};

  wire [15:0] proc_i0 = use_core ? core_i : adc_data_i0;
  wire [15:0] proc_q0 = use_core ? core_q : adc_data_q0;
  wire [15:0] proc_i1 = use_core ? core_i : adc_data_i1;   // one nulled stream:
  wire [15:0] proc_q1 = use_core ? core_q : adc_data_q1;   // ch1 mirrors ch0
  // ======================= END PROCESSING CORE ==============================

  wire wr_en0 = pass_en & (use_core ? core_valid : (adc_valid_i0 & adc_enable_i0));
  wire wr_en1 = pass_en & (use_core ? core_valid : (adc_valid_i1 & adc_enable_i1));

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
  reg        nlms_drop_sticky = 1'b0;

  always @(posedge clk) begin
    if (rst || cnt_clear) begin
      rx_cnt0 <= 32'd0; tx_cnt0 <= 32'd0;
      rx_cnt1 <= 32'd0; tx_cnt1 <= 32'd0;
      ovf_cnt <= 32'd0; unf_cnt <= 32'd0;
      ovf_sticky <= 1'b0; unf_sticky <= 1'b0;
      nlms_drop_sticky <= 1'b0;
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
      if (nlms_drop)
        nlms_drop_sticky <= 1'b1;
    end
  end

  wire [31:0] status_w;
  assign status_w = { 15'd0, pass_en,
                      2'd0, nl_cfg_done, nlms_drop_sticky, core_sel,
                      unf_sticky, ovf_sticky,
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
      cap_toggle <= ~cap_toggle;
    end
  end

  (* ASYNC_REG = "TRUE" *) reg tog_meta = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg tog_sync = 1'b0;
  reg                         tog_prev = 1'b0;

  reg [31:0] h_status = 32'd0, h_rx0 = 32'd0, h_tx0 = 32'd0;
  reg [31:0] h_ovf = 32'd0, h_unf = 32'd0, h_rxs0 = 32'd0, h_txs0 = 32'd0;
  reg [31:0] h_lvl = 32'd0, h_rx1 = 32'd0, h_tx1 = 32'd0, h_rxs1 = 32'd0;

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

endmodule
