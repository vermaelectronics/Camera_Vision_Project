// ============================================================================
//  gnss_stap.v
//  ANTSDR E310 V1  --  CRPA anti-jam stage (space-time adaptive processing)
// ----------------------------------------------------------------------------
//  DATAPATH POSITION
//      axi_ad9361 (ADC) --> gnss_stap --> gnss_passthrough --> axi_ad9361 (DAC)
//
//  gnss_stap sits between the AD9361 RX outputs and the RX inputs of the
//  existing gnss_passthrough block, and presents the SAME interface on both
//  sides (adc_enable/valid/data for I0,Q0,I1,Q1). gnss_passthrough, its
//  elastic buffers, the RX->TX alignment stage and the vendor DMA capture tap
//  are unchanged; the DMA capture still records the RAW antenna samples.
//
//  PROCESSING
//    RX1 and RX2 of the AD9361 are the two CRPA elements (M = 2). Both are
//    clocked by the same LO, so the array is phase coherent.
//      y[n] = sum_{m=0..1} sum_{k=0..15} conj(w[m*16+k]) * x_m[n+k]
//    with 32 complex weights computed by the ARM (Source/Firmware/
//    app_gnss_e310/gnss_stap.c) from the covariance the PL estimates:
//      power inversion (default, needs no array geometry) or MVDR.
//    A 2-element array can null ONE wideband jammer (M-1 degrees of freedom).
//
//    USER_CTRL (STAP register 0x0024):
//      [0] aj_en   0 = bypass: RX samples pass through unchanged (reset value)
//                  1 = channel 0 output = STAP beamformer output
//      [1] ch1_sel 0 = channel 1 output = raw RX2, 1 = copy of the STAP output
//
//  FORMATS
//    RX in : 12-bit sample right-aligned in 16 bits ([11:0] used, re-sign-
//            extended here, so the ADC data-format setting does not matter).
//    out   : same RX format; the STAP output is saturated to 12 bits inside
//            stap_fir (OW = 12), so gnss_passthrough's [11:0] -> [15:4]
//            alignment can never wrap.
//
//  CLOCKS
//    clk        : axi_ad9361/l_clk. adc_valid arrives at most every 2nd clock
//                 (axi_ad9361_lvds_if, 1R1T and 2R2T), which is exactly what
//                 FOLD = 2 requires. stap_top's overrun flag reports otherwise.
//    s_axi_aclk : sys_cpu_clk (100 MHz). Bridged with axil_cdc.
//
//  REGISTER MAP (AXI4-Lite, 64 kB aperture, base 0x43C1_0000)
//    0x0000-0xEFFF  stap_top registers, through the CDC bridge
//                   (see Source/HDL/stap/stap_top.v header):
//      0x0000 ID 0x53544150 "STAP", 0x0004 CONFIG, 0x0008 CONFIG2,
//      0x000C CONFIG3, 0x0010 CTRL, 0x0014 STATUS, 0x0018 CORR_LEN,
//      0x001C OUT_SHIFT, 0x0020 RAW_BASE, 0x0024 USER_CTRL,
//      0x0100.. raw window, 0x1000.. weights, 0x8000.. correlations
//    0xF000-0xFFFF  local registers, s_axi_aclk domain (always answer, even
//                   when l_clk is not running):
//      0xF000 WRAP_ID      RO 0x47535450 "GSTP"
//      0xF004 WRAP_VERSION RO 0x00010000 (1.0)
//      0xF008 WRAP_STATUS  RO [0] l_clk running  [31:16] bridge timeouts
//      0xF00C SAMPLE_COUNT RO input samples seen (sampled every 256 clk)
//
//  LICENCE
//    Original work for this project.
// ============================================================================
`timescale 1ns/100ps

module gnss_stap #(
  parameter N         = 16,     // taps per element
  parameter FOLD      = 2,      // clocks per sample available (l_clk = 2 x fs)
  parameter LAGP      = 8,      // correlator lags per pass
  parameter LMAX_LOG2 = 13      // covariance capture depth 8192
) (
  // ---- sample domain ----
  input               clk,
  input               rst,
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

  output              aj_enable_i0,
  output reg          aj_valid_i0,
  output reg [15:0]   aj_data_i0,
  output              aj_enable_q0,
  output              aj_valid_q0,
  output reg [15:0]   aj_data_q0,
  output              aj_enable_i1,
  output              aj_valid_i1,
  output reg [15:0]   aj_data_i1,
  output              aj_enable_q1,
  output              aj_valid_q1,
  output reg [15:0]   aj_data_q1,

  // ---- AXI4-Lite slave (sys_cpu_clk) ----
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

  localparam M  = 2;
  localparam DW = 16;
  localparam WW = 18;
  localparam OW = 12;
  localparam [31:0] WRAP_ID      = 32'h47535450;   // "GSTP"
  localparam [31:0] WRAP_VERSION = 32'h00010000;   // 1.0

  // ==========================================================================
  //  AXI split: 0xF000-0xFFFF local (s_axi_aclk), everything else -> bridge
  // ==========================================================================
  // Local registers are decoded with a tiny independent AXI-Lite slave; the
  // two slaves are selected per transfer by address bit pattern.
  wire aw_local = (s_axi_awaddr[15:12] == 4'hF);
  wire ar_local = (s_axi_araddr[15:12] == 4'hF);

  // bridge slave port
  wire        b_awready, b_wready, b_bvalid, b_arready, b_rvalid;
  wire [1:0]  b_bresp, b_rresp;
  wire [31:0] b_rdata;
  wire [15:0] bridge_timeouts;

  // local slave state
  reg         l_awready = 1'b0, l_wready = 1'b0, l_bvalid = 1'b0;
  reg         l_arready = 1'b0, l_rvalid = 1'b0;
  reg  [31:0] l_rdata   = 32'd0;

  // route: a write is local if its address is local. Writes to the local
  // space are accepted and ignored (all local registers are read-only).
  // W is steered by a flag latched with AW-or-W, whichever comes first, so
  // the slave whose AW was accepted also takes the W.
  reg         wr_sel_local = 1'b0, wr_sel_valid = 1'b0;
  reg         rd_sel_local = 1'b0, rd_busy = 1'b0;

  wire        wr_local = wr_sel_valid ? wr_sel_local : aw_local;
  wire        rd_local = rd_busy ? rd_sel_local : ar_local;

  always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
      wr_sel_valid <= 1'b0;
      rd_busy      <= 1'b0;
    end else begin
      if (s_axi_awvalid && !wr_sel_valid) begin
        wr_sel_valid <= 1'b1; wr_sel_local <= aw_local;
      end
      if (s_axi_bvalid && s_axi_bready) wr_sel_valid <= 1'b0;
      if (s_axi_arvalid && !rd_busy) begin
        rd_busy <= 1'b1; rd_sel_local <= ar_local;
      end
      if (s_axi_rvalid && s_axi_rready) rd_busy <= 1'b0;
    end
  end

  assign s_axi_awready = wr_local ? l_awready : b_awready;
  assign s_axi_wready  = wr_sel_valid & (wr_local ? l_wready : b_wready);   // W only after AW
  assign s_axi_bvalid  = wr_local ? l_bvalid  : b_bvalid;
  assign s_axi_bresp   = 2'b00;
  assign s_axi_arready = rd_local ? l_arready : b_arready;
  assign s_axi_rvalid  = rd_local ? l_rvalid  : b_rvalid;
  assign s_axi_rresp   = 2'b00;
  assign s_axi_rdata   = rd_local ? l_rdata   : b_rdata;

  // ---- local slave ----
  wire [31:0] v_status, v_count;
  reg         l_aw = 1'b0, l_w = 1'b0;
  always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
      l_awready <= 1'b0; l_wready <= 1'b0; l_bvalid <= 1'b0;
      l_arready <= 1'b0; l_rvalid <= 1'b0;
      l_aw <= 1'b0; l_w <= 1'b0;
    end else begin
      l_awready <= 1'b0; l_wready <= 1'b0; l_arready <= 1'b0;
      if (wr_sel_valid && wr_sel_local) begin
        if (s_axi_awvalid && !l_aw && !l_awready) begin l_awready <= 1'b1; l_aw <= 1'b1; end
        if (s_axi_wvalid  && !l_w  && !l_wready ) begin l_wready  <= 1'b1; l_w  <= 1'b1; end
        if (l_aw && l_w && !l_bvalid) l_bvalid <= 1'b1;
      end
      if (l_bvalid && s_axi_bready) begin l_bvalid <= 1'b0; l_aw <= 1'b0; l_w <= 1'b0; end

      if (rd_busy && rd_sel_local && s_axi_arvalid && !l_arready && !l_rvalid) begin
        l_arready <= 1'b1;
        l_rvalid  <= 1'b1;
        case (s_axi_araddr[11:2])
          10'h000: l_rdata <= WRAP_ID;
          10'h001: l_rdata <= WRAP_VERSION;
          10'h002: l_rdata <= v_status;
          10'h003: l_rdata <= v_count;
          default: l_rdata <= 32'd0;
        endcase
      end
      if (l_rvalid && s_axi_rready) l_rvalid <= 1'b0;
    end
  end

  // ==========================================================================
  //  CDC bridge to stap_top
  // ==========================================================================
  wire [15:0] m_awaddr, m_araddr;
  wire [31:0] m_wdata, m_rdata;
  wire        m_awvalid, m_awready, m_wvalid, m_wready, m_bvalid, m_bready;
  wire        m_arvalid, m_arready, m_rvalid, m_rready;
  wire [1:0]  m_bresp, m_rresp;

  axil_cdc #(.AW(16)) i_cdc (
    .s_clk     (s_axi_aclk),
    .s_resetn  (s_axi_aresetn),
    .s_awaddr  (s_axi_awaddr),
    .s_awvalid (s_axi_awvalid & ~wr_local),
    .s_awready (b_awready),
    .s_wdata   (s_axi_wdata),
    .s_wvalid  (s_axi_wvalid & wr_sel_valid & ~wr_sel_local),
    .s_wready  (b_wready),
    .s_bvalid  (b_bvalid),
    .s_bresp   (b_bresp),
    .s_bready  (s_axi_bready & ~wr_local),
    .s_araddr  (s_axi_araddr),
    .s_arvalid (s_axi_arvalid & ~rd_local),
    .s_arready (b_arready),
    .s_rdata   (b_rdata),
    .s_rvalid  (b_rvalid),
    .s_rresp   (b_rresp),
    .s_rready  (s_axi_rready & ~rd_local),
    .timeouts  (bridge_timeouts),
    .m_clk     (clk),
    .m_resetn  (~rst),
    .m_awaddr  (m_awaddr),
    .m_awvalid (m_awvalid),
    .m_awready (m_awready),
    .m_wdata   (m_wdata),
    .m_wvalid  (m_wvalid),
    .m_wready  (m_wready),
    .m_bvalid  (m_bvalid),
    .m_bready  (m_bready),
    .m_araddr  (m_araddr),
    .m_arvalid (m_arvalid),
    .m_arready (m_arready),
    .m_rdata   (m_rdata),
    .m_rvalid  (m_rvalid),
    .m_rready  (m_rready)
  );

  // ==========================================================================
  //  STAP core (sample domain)
  // ==========================================================================
  // 12-bit RX samples, sign-extended from bit 11 regardless of [15:12]
  function [15:0] sx12(input [15:0] d);
    sx12 = {{4{d[11]}}, d[11:0]};
  endfunction

  wire          in_valid = adc_valid_i0 & adc_enable_i0;
  wire [M*DW-1:0] s_re = {sx12(adc_data_i1), sx12(adc_data_i0)};
  wire [M*DW-1:0] s_im = {sx12(adc_data_q1), sx12(adc_data_q0)};

  wire          y_valid;
  wire [OW-1:0] y_re, y_im;
  wire [31:0]   user_ctrl;
  wire          irq_unused;

  stap_top #(.M(M), .N(N), .DW(DW), .WW(WW), .OW(OW), .FOLD(FOLD),
             .LMAX_LOG2(LMAX_LOG2), .LAGP(LAGP)) i_stap (
    .clk          (clk),
    .aresetn      (~rst),
    .s_valid      (in_valid),
    .s_re         (s_re),
    .s_im         (s_im),
    .y_valid      (y_valid),
    .y_re         (y_re),
    .y_im         (y_im),
    .irq          (irq_unused),
    .user_ctrl    (user_ctrl),
    .s_axi_awaddr (m_awaddr),
    .s_axi_awvalid(m_awvalid),
    .s_axi_awready(m_awready),
    .s_axi_wdata  (m_wdata),
    .s_axi_wstrb  (4'hF),
    .s_axi_wvalid (m_wvalid),
    .s_axi_wready (m_wready),
    .s_axi_bresp  (m_bresp),
    .s_axi_bvalid (m_bvalid),
    .s_axi_bready (m_bready),
    .s_axi_araddr (m_araddr),
    .s_axi_arvalid(m_arvalid),
    .s_axi_arready(m_arready),
    .s_axi_rdata  (m_rdata),
    .s_axi_rresp  (m_rresp),
    .s_axi_rvalid (m_rvalid),
    .s_axi_rready (m_rready)
  );

  // ==========================================================================
  //  Output selection (registered, sample domain)
  // ==========================================================================
  wire aj_en   = user_ctrl[0];
  wire ch1_sel = user_ctrl[1];

  reg  [15:0] raw_i1 = 16'd0, raw_q1 = 16'd0;    // last RX2 sample
  wire [15:0] yi16 = {{(16-OW){y_re[OW-1]}}, y_re};
  wire [15:0] yq16 = {{(16-OW){y_im[OW-1]}}, y_im};

  always @(posedge clk) begin
    if (rst) begin
      aj_valid_i0 <= 1'b0;
      aj_data_i0  <= 16'd0; aj_data_q0 <= 16'd0;
      aj_data_i1  <= 16'd0; aj_data_q1 <= 16'd0;
    end else begin
      if (in_valid) begin raw_i1 <= adc_data_i1; raw_q1 <= adc_data_q1; end
      if (!aj_en) begin
        // bypass: exactly the RX samples, one clock later
        aj_valid_i0 <= adc_valid_i0;
        aj_data_i0  <= adc_data_i0;
        aj_data_q0  <= adc_data_q0;
        aj_data_i1  <= adc_data_i1;
        aj_data_q1  <= adc_data_q1;
      end else begin
        aj_valid_i0 <= y_valid;
        if (y_valid) begin
          aj_data_i0 <= yi16;
          aj_data_q0 <= yq16;
          aj_data_i1 <= ch1_sel ? yi16 : raw_i1;
          aj_data_q1 <= ch1_sel ? yq16 : raw_q1;
        end
      end
    end
  end

  // all four channel strobes are identical in axi_ad9361; mirror that
  assign aj_valid_q0  = aj_valid_i0;
  assign aj_valid_i1  = aj_valid_i0;
  assign aj_valid_q1  = aj_valid_i0;
  assign aj_enable_i0 = adc_enable_i0;
  assign aj_enable_q0 = adc_enable_q0;
  assign aj_enable_i1 = adc_enable_i1;
  assign aj_enable_q1 = adc_enable_q1;

  // ==========================================================================
  //  Status into the AXI domain (same capture-and-toggle scheme as
  //  gnss_passthrough): cap_* loaded every 256 clk, h_* copied on the toggle.
  // ==========================================================================
  reg  [31:0] smp_cnt = 32'd0;
  reg  [7:0]  cap_div = 8'd0;
  reg         cap_toggle = 1'b0;
  reg  [31:0] cap_count = 32'd0;
  always @(posedge clk) begin
    if (in_valid) smp_cnt <= smp_cnt + 1'b1;
    cap_div <= cap_div + 1'b1;
    if (cap_div == 8'd0) begin
      cap_count  <= smp_cnt;
      cap_toggle <= ~cap_toggle;
    end
  end

  (* ASYNC_REG = "TRUE" *) reg tog_meta = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg tog_sync = 1'b0;
  reg         tog_prev = 1'b0;
  reg  [31:0] h_count  = 32'd0;
  reg  [15:0] alive_cnt = 16'd0;   // s_axi_aclk cycles since the last toggle
  always @(posedge s_axi_aclk) begin
    tog_meta <= cap_toggle;
    tog_sync <= tog_meta;
    tog_prev <= tog_sync;
    if (tog_sync != tog_prev) begin
      h_count   <= cap_count;
      alive_cnt <= 16'd0;
    end else if (alive_cnt != 16'hFFFF) begin
      alive_cnt <= alive_cnt + 1'b1;
    end
  end
  // l_clk >= 10 MHz toggles every <= 25.6 us = 2560 s_axi_aclk cycles
  wire lclk_alive = (alive_cnt < 16'd8192);

  assign v_status = {bridge_timeouts, 15'd0, lclk_alive};
  assign v_count  = h_count;

endmodule
