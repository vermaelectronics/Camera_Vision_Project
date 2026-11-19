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
//                                    [4]  mvdr_sel   1 = ch0 is fed by the MVDR beamformer
//                                                      output (needs BOTH RX channels enabled)
//                                                  0 = raw RX1, exactly the Phase 1 behaviour
//                                    [8]  cnt_clear  1 = hold counters cleared
//    0x10    STATUS          RO      [0]  adc_enable_i0   [1]  adc_enable_q0
//                                    [2]  dac_enable_i0   [3]  dac_enable_q0
//                                    [4]  fifo0_empty     [5]  fifo0_full
//                                    [6]  fifo1_empty     [7]  fifo1_full
//                                    [8]  overflow_sticky [9]  underflow_sticky
//                                    [16] pass_en (as seen in the clk domain)
//                                    [17] mvdr_sel (as seen in the clk domain)
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
//    0x40..  CRPA_COEF[0..15] RW     reserved for the future CRPA algorithm;
//                                    Phase 1 stores and returns them, and no
//                                    logic consumes them.
//    0x100.. MVDR window     RW/RO   adaptive 2-channel MVDR beamformer + jammer
//                                    estimator (mvdr_regs v1.11), 64 words
//    0x200.. MVDR page 2     RW/RO   (v1.11) jammer power per channel (average / peak)
//                                    and noise per channel: 0x200 JAM_PWR_CH,
//                                    0x204 JAM_PEAK, 0x208 NOISE_CH;
//                                    (v1.13) STAP: 0x20C STAP_CTRL (RW), 0x210 STAP_ST,
//                                    0x214 STAP_WNORM, 0x218 STAP_NWSAT
//                                    (0x100..0x1FC).  See mvdr_regs.v for the register map:
//                                    ID/VERSION, CTRL (mode, freeze, cnt_clear), COMMIT,
//                                    steering vector, diagonal loading, fixed weights, and
//                                    read-only status/counters/weights/covariance sums.
//                                    Reset behaviour is identical to Phase 1 until software
//                                    sets CONTROL[4] = 1 (mvdr_sel).
//    0x400.. SMI-MVDR STAP   RW/RO   (v1.14) stap_smi in mvdr_top, 768 words:
//            ..0xFFC                 0x400 + 16i  covariance entry 128 PAGE + i (i<=j order)
//                                    0xC00 PAGE, 0xC40 + 8k weight k re / im (Q4.20,
//                                    k 0..N-1: 2 x 16 space-time + 2 image taps),
//                                    0xEC0 CTRL, 0xEC4 STAT, 0xEC8 COUNT, 0xECC NOVR,
//                                    0xED0 INFO (layout), 0xED4 WLOAD, 0xED8 NLOAD.
//                                    See stap_smi.v
//
//  LICENCE
//    Original work for this project.  Not derived from Analog Devices or
//    MicroPhase source.  See Docs/Architecture/MODIFICATIONS.md.
// ============================================================================

`timescale 1ns/100ps

module gnss_passthrough #(
  parameter FIFO_ADDR_WIDTH = 5,  // elastic buffer depth = 2**FIFO_ADDR_WIDTH
  parameter STAP_LMS = 0          // v1.14: 1 = also build the NLMS STAP (does not fit an XC7Z020
                                  //   together with SMI; SMI = gnss_null=3 is the default core)
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
  // v1.2 -- adds the MVDR beamformer (0x100 window) and CONTROL[4] mvdr_sel.
  localparam [31:0] CORE_VERSION = 32'h00010002;   // v1.2
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

  // v1.14 timing: the write data is registered on the handshake-start cycle
  // (wr_hs: wvalid is high, so AXI holds wdata valid), the same edge that
  // raises awready. Every register write then takes wdata_r in the reg_wr
  // cycle -- no added latency -- and the PS7 WDATA pins drive one flop each
  // instead of ~70 loads spread over the die (Vivado: -1.0 ns, 9.5 ns route).
  reg  [31:0] wdata_r = 32'd0;
  always @(posedge s_axi_aclk)
    if (wr_hs) wdata_r <= s_axi_wdata;
  wire        reg_wr    = axi_awready;             // one-cycle write strobe
  wire [11:0] reg_waddr = axi_awaddr_r[11:0];

  // ---- software-writable state --------------------------------------------
  reg  [31:0] reg_scratch = 32'd0;
  reg  [31:0] reg_control = 32'd0;
  reg  [31:0] crpa_coef [0:15];

  integer ci;
  initial for (ci = 0; ci < 16; ci = ci + 1) crpa_coef[ci] = 32'd0;

  always @(posedge s_axi_aclk) begin
    if (s_axi_aresetn == 1'b0) begin
      reg_scratch <= 32'd0;
      reg_control <= 32'd0;
    end else if (reg_wr) begin
      case (reg_waddr)
        12'h002: reg_scratch <= wdata_r;       // 0x08
        12'h003: reg_control <= wdata_r;       // 0x0C
        default: ;
      endcase
      if (reg_waddr >= 12'h010 && reg_waddr <= 12'h01F)   // 0x40..0x7C
        crpa_coef[reg_waddr[3:0]] <= wdata_r;
    end
  end

  // Status view, refreshed from the clk domain (see CDC section).
  wire [31:0] v_status, v_rx_cnt0, v_tx_cnt0, v_ovf_cnt, v_unf_cnt;
  wire [31:0] v_rx_snap0, v_tx_snap0, v_level, v_rx_cnt1, v_tx_cnt1, v_rx_snap1;

  // MVDR register window (0x100..0x1FC): read data comes from mvdr_regs (declared here
  // because the read mux below uses it; the instance is further down)
  // pages 0x100..0x1FC and (v1.11) 0x200..0x2FC
  wire        mvdr_rd_win = (s_axi_araddr[13:8] == 6'd1) || (s_axi_araddr[13:8] == 6'd2);
  wire [31:0] mvdr_rd_data;
  // (v1.14) SMI-MVDR STAP window, bytes 0x400..0xFFC (words 0x100..0x3FF),
  // served by stap_smi inside mvdr_top
  wire        smi_rd_win  = (s_axi_araddr[13:12] == 2'd0) && (s_axi_araddr[11:10] != 2'd0);
  wire [31:0] smi_rd_data;

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
      default: axi_rdata <= mvdr_rd_win ? mvdr_rd_data :           // 0x100..0x2FC = MVDR window
                            smi_rd_win  ? smi_rd_data  : 32'd0;    // 0x400..0xFFC = SMI window
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
  wire [8:0] ctrl_raw = {reg_control[8], 3'b000, reg_control[4:0]};

  always @(posedge clk) begin
    ctrl_meta <= ctrl_raw;
    ctrl_sync <= ctrl_meta;
  end

  wire pass_en   = ctrl_sync[0];
  wire mute      = ctrl_sync[1];
  wire swap_iq   = ctrl_sync[2];
  wire ch1_copy  = ctrl_sync[3];
  wire mvdr_sel  = ctrl_sync[4];
  wire cnt_clear = ctrl_sync[8];

  // ==========================================================================
  //  MVDR adaptive beamformer
  //    mvdr_regs : AXI-visible registers (s_axi_aclk) with coherent CDC to clk
  //    mvdr_top  : cov accumulation -> loading/scaling -> weights -> beamformer
  //  CLOCKING (MVDR 1.5): the per-sample part of mvdr_top (covariance
  //  accumulators, beamformer, output scaler) runs on clk = l_clk = 4 x fs;
  //  the maths (cov_prep, weight calc, control FSM) and jammer_est run on
  //  s_axi_aclk (fixed 100 MHz), so they meet timing at any sample rate.
  //  It always runs on the ADC strobes (harmless while pass_en = 0).  Its output
  //  replaces raw RX1 in the ch0 elastic buffer only when mvdr_sel = 1.
  //  mvdr_top needs the RX1/RX2 samples of the same instant; both channels must
  //  be enabled.  mvdr_top raises its skew counter if the two valid strobes ever
  //  disagree.
  // ==========================================================================
  // Address window: word address bits [11:6] == 1  ->  byte 0x100..0x1FC
  wire        mvdr_wr_en  = reg_wr && ((reg_waddr[11:6] == 6'd1) || (reg_waddr[11:6] == 6'd2));
  wire        smi_wr_en   = reg_wr && (reg_waddr[11:10] == 2'd0) && (reg_waddr[9:8] != 2'd0);

  // --------------------------------------------------------------------------
  //  MVDR covariance window size -- the ONE place to change it.
  //    MV_LOG2_N : window = 2^MV_LOG2_N samples (12 = 4096, 13 = 8192)
  //    MV_ACC_W  : covariance-sum width, derived exactly as in cov2x2_accum
  //    MV_CLIP_W : clip-count width (0 .. 2^MV_LOG2_N)
  //  Every bus below that depends on these is declared from them, so mvdr_top,
  //  mvdr_regs and jammer_est can never disagree about the width again.
  //  (Before: 37 / [36:0] / [12:0] were literals and had to be edited by hand.)
  //  If you change MV_LOG2_N, also scale any absolute EPS_ABS value and any
  //  firmware constant that assumes N = 4096 (e.g. a /4096 in P_in/P_out dBFS).
  // --------------------------------------------------------------------------
  localparam MV_DATA_W  = 12;
  localparam MV_LOG2_N  = 13;                               // N = 8192
  localparam MV_IQF     = 2;                               // IQ corrector fraction bits
  localparam MV_DW      = MV_DATA_W + MV_IQF + 1;          // corrected sample width (15)
  localparam MV_ACC_W   = 2*MV_DW + MV_LOG2_N + 1;         // 44 (covariance sums)
  localparam MV_CLIP_W  = MV_LOG2_N + 1;                   // 14

  wire [1:0]        mvc_mode;
  wire              mvc_freeze, mvc_cnt_clear;
  wire signed [15:0] mvc_a1_re, mvc_a1_im, mvc_a2_re, mvc_a2_im;
  wire [MV_ACC_W-1:0] mvc_eps_abs;
  wire [5:0]        mvc_eps_rel_sh;
  wire signed [17:0] mvc_fw1_re, mvc_fw1_im, mvc_fw2_re, mvc_fw2_im;

  wire [1:0]        mv_st_mode;
  wire              mv_have_w, mv_last_sat, mv_last_skip, mv_skew, mv_wc_busy;
  wire [31:0]       mv_n_blocks, mv_n_upd, mv_n_skip, mv_n_sat, mv_n_drop, mv_n_zero, mv_n_skew, mv_n_osat;
  wire signed [17:0] mv_w1_re, mv_w1_im, mv_w2_re, mv_w2_im;
  wire signed [MV_ACC_W-1:0] mv_cov_r11, mv_cov_r22;
  wire signed [MV_ACC_W-1:0] mv_cov_r12_re, mv_cov_r12_im;   // MVDR window v1.1: R12 readback
  wire              mv_cov_blk;                      // 1-clk pulse per covariance window update
  wire [MV_CLIP_W-1:0] mv_cov_clip1, mv_cov_clip2;   // ADC-rail samples per window (MVDR v1.3)

  // jammer estimator (MVDR window v1.2)
  wire              jmc_en, jmc_inv;
  wire [3:0]        jmc_avg;
  wire signed [15:0] jmc_cal, jmc_on, jmc_off;
  wire [15:0]       jmc_k;
  wire [2:0]        mvc_out_gain;                    // MVDR output gain (MVDR window v1.4)
  wire              mvc_dc_rm;                       // mean-removed covariance (MVDR window v1.6)
  wire [MV_DATA_W-1:0] mv_cov_peak1, mv_cov_peak2;   // window peaks (MVDR window v1.6)
  wire              mv_cov_dc_rm;

  // committed configuration in the AXI (= maths) domain, MVDR 1.5
  wire [1:0]        mac_mode;
  wire              mac_freeze, mac_cnt_clear;
  wire signed [15:0] mac_a1_re, mac_a1_im, mac_a2_re, mac_a2_im;
  wire [MV_ACC_W-1:0] mac_eps_abs;
  wire [5:0]        mac_eps_rel_sh;
  wire [7:0]        mac_eps_nf_k;                    // noise-referenced loading (MVDR window v1.7)
  wire [15:0]       mac_hold;                        // weight hold/glide policy (MVDR window v1.8)
  wire [3:0]        mac_rel_sh, mac_coh_sh;
  wire [5:0]        mv_coh_l2;
  wire [31:0]       mv_n_jam, mv_n_hold;
  wire              mv_st_jam, mv_st_hold, mv_st_glide;
  wire              mac_iq_en, mac_iq_clear;           // IQ-image correction (MVDR window v1.9)
  wire [2:0]        mac_iq_mu;
  wire signed [17:0] mv_b1r, mv_b1i, mv_b2r, mv_b2i;
  wire [31:0]       mv_n_iqu;
  wire              mvc_agc_en;                      // AOLC (MVDR window v1.10)
  wire [3:0]        mvc_agc_sh;
  wire [7:0]        mvc_tdb;
  wire [31:0]       mv_out_lvl;
  wire signed [15:0] jm_j1, jm_j2, jm_n1, jm_n2, jm_pk1, jm_pk2;   // jammer_est v2
  wire [MV_ACC_W-1:0] mv_nf_est;
  wire              mvc_stap_en, mvc_stap_clr, mvc_stap_hoff;   // STAP (MVDR window v1.13)
  wire [3:0]        mvc_stap_mu;
  wire [4:0]        mvc_stap_leak;
  wire              mv_stap_act;
  wire [5:0]        mv_stap_pl;
  wire [31:0]       mv_stap_wn, mv_stap_ns;
  wire signed [17:0] mac_fw1_re, mac_fw1_im, mac_fw2_re, mac_fw2_im;
  wire              mac_jen, mac_jinv;
  wire [3:0]        mac_javg;
  wire signed [15:0] mac_jcal, mac_jon, mac_joff;
  wire [15:0]       mac_jk;
  wire              jm_present, jm_valid, jm_clip, jm_nosig, jm_noise0;
  wire signed [15:0] jm_theta, jm_psi, jm_psi_raw_bin, jm_psi_raw_cd;
  wire signed [15:0] jm_dbfs, jm_noise_dbfs, jm_jnr;
  wire [31:0]       jm_n_est, jm_n_drop;

  mvdr_regs #(.ACC_W(MV_ACC_W), .W_W(18), .A_W(16), .HAVE_STAP(STAP_LMS)) u_mvdr_regs (
    .s_axi_aclk(s_axi_aclk), .s_axi_aresetn(s_axi_aresetn),
    .wr_en(mvdr_wr_en), .wr_addr({1'b0, reg_waddr[7], reg_waddr[5:0]}), .wr_data(wdata_r),
    .rd_addr({1'b0, s_axi_araddr[9], s_axi_araddr[7:2]}), .rd_data(mvdr_rd_data),
    .clk(clk), .st_clk(s_axi_aclk),
    .cfg_mode(mvc_mode), .cfg_freeze(mvc_freeze), .cfg_cnt_clear(mvc_cnt_clear),
    .cfg_a1_re(mvc_a1_re), .cfg_a1_im(mvc_a1_im), .cfg_a2_re(mvc_a2_re), .cfg_a2_im(mvc_a2_im),
    .cfg_eps_abs(mvc_eps_abs), .cfg_eps_rel_sh(mvc_eps_rel_sh),
    .cfg_fix_w1_re(mvc_fw1_re), .cfg_fix_w1_im(mvc_fw1_im),
    .cfg_fix_w2_re(mvc_fw2_re), .cfg_fix_w2_im(mvc_fw2_im),
    .cfg_jam_en(jmc_en), .cfg_jam_inv(jmc_inv), .cfg_jam_avg(jmc_avg),
    .cfg_jam_cal(jmc_cal), .cfg_jam_k(jmc_k), .cfg_jam_on(jmc_on), .cfg_jam_off(jmc_off),
    .cfg_out_gain(mvc_out_gain), .cfg_dc_rm(mvc_dc_rm),
    .cfg_agc_en(mvc_agc_en), .cfg_agc_sh(mvc_agc_sh), .cfg_tdb(mvc_tdb),
    .cfg_stap_en(mvc_stap_en), .cfg_stap_mu(mvc_stap_mu),
    .cfg_stap_leak(mvc_stap_leak), .cfg_stap_clr(mvc_stap_clr), .cfg_stap_hoff(mvc_stap_hoff),
    .acfg_mode(mac_mode), .acfg_freeze(mac_freeze), .acfg_cnt_clear(mac_cnt_clear),
    .acfg_a1_re(mac_a1_re), .acfg_a1_im(mac_a1_im), .acfg_a2_re(mac_a2_re), .acfg_a2_im(mac_a2_im),
    .acfg_eps_abs(mac_eps_abs), .acfg_eps_rel_sh(mac_eps_rel_sh), .acfg_eps_nf_k(mac_eps_nf_k),
    .acfg_hold(mac_hold), .acfg_rel_sh(mac_rel_sh), .acfg_coh_sh(mac_coh_sh),
    .acfg_iq_en(mac_iq_en), .acfg_iq_clear(mac_iq_clear), .acfg_iq_mu(mac_iq_mu),
    .acfg_fix_w1_re(mac_fw1_re), .acfg_fix_w1_im(mac_fw1_im),
    .acfg_fix_w2_re(mac_fw2_re), .acfg_fix_w2_im(mac_fw2_im),
    .acfg_jam_en(mac_jen), .acfg_jam_inv(mac_jinv), .acfg_jam_avg(mac_javg),
    .acfg_jam_cal(mac_jcal), .acfg_jam_k(mac_jk), .acfg_jam_on(mac_jon), .acfg_jam_off(mac_joff),
    .st_mode(mv_st_mode), .st_have_w(mv_have_w), .st_last_sat(mv_last_sat),
    .st_last_skip(mv_last_skip), .st_skew(mv_skew), .st_wc_busy(mv_wc_busy),
    .n_blocks(mv_n_blocks), .n_upd(mv_n_upd), .n_skip(mv_n_skip), .n_sat(mv_n_sat),
    .n_drop(mv_n_drop), .n_zero(mv_n_zero), .n_skew(mv_n_skew), .n_osat(mv_n_osat),
    .w_cur_w1_re(mv_w1_re), .w_cur_w1_im(mv_w1_im), .w_cur_w2_re(mv_w2_re), .w_cur_w2_im(mv_w2_im),
    .cov_r11(mv_cov_r11), .cov_r22(mv_cov_r22),
    .cov_r12_re(mv_cov_r12_re), .cov_r12_im(mv_cov_r12_im),
    .jam_flags({jm_noise0, jm_nosig, jm_clip, jm_valid, jm_present}),
    .jam_theta(jm_theta), .jam_psi(jm_psi),
    .jam_psi_raw_bin(jm_psi_raw_bin), .jam_psi_raw_cd(jm_psi_raw_cd),
    .jam_dbfs(jm_dbfs), .jam_noise_dbfs(jm_noise_dbfs), .jam_jnr(jm_jnr),
    .jam_n_est(jm_n_est), .jam_n_drop(jm_n_drop),
    .cov_clip1({{(16-MV_CLIP_W){1'b0}}, mv_cov_clip1}),
    .cov_clip2({{(16-MV_CLIP_W){1'b0}}, mv_cov_clip2}),
    .cov_peak1({{(16-MV_DATA_W){1'b0}}, mv_cov_peak1}),
    .cov_peak2({{(16-MV_DATA_W){1'b0}}, mv_cov_peak2}),
    .cov_dc_rm(mv_cov_dc_rm), .nf_est(mv_nf_est),
    .coh_l2(mv_coh_l2), .st_jam(mv_st_jam), .st_hold(mv_st_hold), .st_glide(mv_st_glide),
    .n_jam(mv_n_jam), .n_hold(mv_n_hold),
    .iq_b1_re(mv_b1r), .iq_b1_im(mv_b1i), .iq_b2_re(mv_b2r), .iq_b2_im(mv_b2i), .n_iq_upd(mv_n_iqu),
    .out_lvl(mv_out_lvl),
    .jam_j1(jm_j1), .jam_j2(jm_j2), .jam_n1(jm_n1), .jam_n2(jm_n2), .jam_pk1(jm_pk1), .jam_pk2(jm_pk2),
    .stap_active(mv_stap_act), .stap_pl(mv_stap_pl),
    .stap_wnorm(mv_stap_wn), .stap_nwsat(mv_stap_ns));

  wire              mvdr_out_valid, mvdr_out_sat;
  wire signed [11:0] mvdr_out_i, mvdr_out_q;

  // ==========================================================================
  //  Local reset (added for 200 MHz l_clk)
  //  axi_ad9361's rst is ALSO the asynchronous clear of its own register
  //  banks. Using it directly here put ~3000 loads on that net (fo=2955 in
  //  the 5 ns timing report) and its route to axi_ad9361's CLR pins took
  //  4.3 ns -> 637 recovery failures. Two local flops isolate it: the
  //  vendor net goes back to its own few loads, and this copy may be
  //  replicated freely. Reset is held for many clocks, so 2 extra cycles of
  //  latency are irrelevant.
  // ==========================================================================
  (* max_fanout = 64 *) reg rst_l  = 1'b1;
                        reg rst_q1 = 1'b1;
  always @(posedge clk) begin
    rst_q1 <= rst;
    rst_l  <= rst_q1;
  end

  // Maths-domain reset (s_axi_aclk): registered from the AXI reset.
  (* max_fanout = 64 *) reg mrst  = 1'b1;
                        reg mrst_q1 = 1'b1;
  always @(posedge s_axi_aclk) begin
    mrst_q1 <= ~s_axi_aresetn;
    mrst    <= mrst_q1;
  end

  // LOG2_SUB = 3: the 2^MV_LOG2_N-sample covariance window slides every
  // 2^(MV_LOG2_N-3) samples (N = 8192: window 149 us, slides every 1024
  // samples = 18.6 us at 55 MSPS). Same null depth as LOG2_SUB = 2, half the
  // reaction time to a new jammer.
  mvdr_top #(.DATA_W(MV_DATA_W), .LOG2_N(MV_LOG2_N), .LOG2_SUB(3), .W_W(18), .W_FRAC(15), .OUT_W(12),
             .SMI(1), .STAP_LMS(STAP_LMS)) u_mvdr (
    .clk(clk), .rst(rst_l), .mclk(s_axi_aclk), .mrst(mrst),
    .in_valid0(adc_valid_i0 & adc_enable_i0),
    .in_valid1(adc_valid_i1 & adc_enable_i1),
    .x1_i(adc_data_i0[11:0]), .x1_q(adc_data_q0[11:0]),
    .x2_i(adc_data_i1[11:0]), .x2_q(adc_data_q1[11:0]),
    .out_gain_sh(mvc_out_gain), .s_cnt_clear(mvc_cnt_clear | cnt_clear), .dc_rm(mvc_dc_rm),
    .out_agc_en(mvc_agc_en), .out_agc_sh(mvc_agc_sh), .out_target_db(mvc_tdb),
    .mode(mac_mode), .freeze(mac_freeze), .cnt_clear(mac_cnt_clear | reg_control[8]),
    .a1_re(mac_a1_re), .a1_im(mac_a1_im), .a2_re(mac_a2_re), .a2_im(mac_a2_im),
    .eps_abs(mac_eps_abs), .eps_rel_sh(mac_eps_rel_sh), .eps_nf_k(mac_eps_nf_k),
    .coh_sh(mac_coh_sh), .hold_units(mac_hold), .rel_sh(mac_rel_sh),
    .iq_en(mac_iq_en), .iq_clear(mac_iq_clear), .iq_mu_sh(mac_iq_mu),
    .stap_en(mvc_stap_en), .stap_mu_sh(mvc_stap_mu), .stap_leak_sh(mvc_stap_leak),
    .stap_clear(mvc_stap_clr), .stap_hold_off(mvc_stap_hoff),
    .smi_wr_en(smi_wr_en), .smi_wr_addr(reg_waddr[9:0]), .smi_wr_data(wdata_r),
    .smi_rd_addr(s_axi_araddr[11:2]), .smi_rd_data(smi_rd_data),
    .fix_w1_re(mac_fw1_re), .fix_w1_im(mac_fw1_im), .fix_w2_re(mac_fw2_re), .fix_w2_im(mac_fw2_im),
    .out_valid(mvdr_out_valid), .out_i(mvdr_out_i), .out_q(mvdr_out_q), .out_sat(mvdr_out_sat),
    .w_cur_w1_re(mv_w1_re), .w_cur_w1_im(mv_w1_im), .w_cur_w2_re(mv_w2_re), .w_cur_w2_im(mv_w2_im),
    .cov_blk(mv_cov_blk), .cov_r11(mv_cov_r11), .cov_r22(mv_cov_r22),
    .cov_r12_re(mv_cov_r12_re), .cov_r12_im(mv_cov_r12_im),
    .cov_clip1(mv_cov_clip1), .cov_clip2(mv_cov_clip2),
    .cov_peak1(mv_cov_peak1), .cov_peak2(mv_cov_peak2), .cov_dc_rm(mv_cov_dc_rm),
    .wc_busy(mv_wc_busy), .nf_est(mv_nf_est), .out_lvl(mv_out_lvl),
    .coh_l2(mv_coh_l2), .n_jam(mv_n_jam), .n_hold(mv_n_hold),
    .st_jam(mv_st_jam), .st_hold(mv_st_hold), .st_glide(mv_st_glide),
    .iq_b1_re(mv_b1r), .iq_b1_im(mv_b1i), .iq_b2_re(mv_b2r), .iq_b2_im(mv_b2i), .n_iq_upd(mv_n_iqu),
    .n_blocks(mv_n_blocks), .n_upd(mv_n_upd), .n_skip(mv_n_skip), .n_sat(mv_n_sat),
    .n_drop(mv_n_drop), .n_zero(mv_n_zero), .n_skew(mv_n_skew), .n_osat(mv_n_osat),
    .st_skew(mv_skew), .st_have_w(mv_have_w), .st_last_sat(mv_last_sat),
    .st_last_skip(mv_last_skip), .st_mode(mv_st_mode),
    .stap_active(mv_stap_act), .stap_pl(mv_stap_pl),
    .stap_wnorm(mv_stap_wn), .stap_nwsat(mv_stap_ns));

  // ==========================================================================
  //  Jammer estimator: detection, power (dBFS, JNR) and angle of arrival from
  //  the same covariance block sums the MVDR uses.  Runs in every MVDR mode.
  //  Results are read back through the MVDR window (0x194..0x1AC), config at
  //  0x1C0..0x1CC (applied by MVDR COMMIT).
  // ==========================================================================
  // MVDR 1.5: on s_axi_aclk, fed by the window sums mvdr_top has already
  // carried into that domain (mv_cov_blk is a 1-aclk pulse).
  // DATA_W = raw width + IQF: the window sums are of samples scaled by 2^IQF, so
  // full scale is 2^(MV_DATA_W-1+MV_IQF) and the dBFS outputs stay referenced to the ADC.
  jammer_est #(.DATA_W(MV_DATA_W + MV_IQF), .LOG2_N(MV_LOG2_N), .ACC_W(MV_ACC_W), .AVG_MAX(8)) u_jam (
    .clk(s_axi_aclk), .rst(mrst),
    .cov_valid(mv_cov_blk),
    .r11(mv_cov_r11), .r22(mv_cov_r22), .r12_re(mv_cov_r12_re), .r12_im(mv_cov_r12_im),
    .cfg_enable(mac_jen), .cfg_phase_inv(mac_jinv), .cfg_avg_log2(mac_javg),
    .cfg_phase_cal(mac_jcal), .cfg_k_q14(mac_jk),
    .cfg_jnr_on_q8(mac_jon), .cfg_jnr_off_q8(mac_joff),
    .est_done(),
    .jam_present(jm_present), .st_valid(jm_valid), .st_sin_clip(jm_clip),
    .st_no_signal(jm_nosig), .st_noise_zero(jm_noise0),
    .theta_cdeg(jm_theta), .psi_cdeg(jm_psi),
    .psi_raw_bin(jm_psi_raw_bin), .psi_raw_cdeg(jm_psi_raw_cd),
    .jam_dbfs_q8(jm_dbfs), .noise_dbfs_q8(jm_noise_dbfs), .jnr_db_q8(jm_jnr),
    .j1_dbfs_q8(jm_j1), .j2_dbfs_q8(jm_j2), .n1_dbfs_q8(jm_n1), .n2_dbfs_q8(jm_n2),
    .pk1_dbfs_q8(jm_pk1), .pk2_dbfs_q8(jm_pk2),
    .n_est(jm_n_est), .n_drop(jm_n_drop));

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

  // Occupancy (= wptr - rptr) and the full / empty flags are kept in their own
  // registers, updated with the pointers, so every FIFO decision starts from
  // flip-flops: at l_clk = 250 MHz the old "subtract, compare, then gate 32
  // data enables" path was 6 logic levels.
  reg  [AW:0] level0 = 0, level1 = 0;
  reg         full0 = 1'b0, empty0 = 1'b1, full1 = 1'b0, empty1 = 1'b1;

  // FIFO clear: reset, or pass_en = 0 (registered, aligned with rst_l;
  // max_fanout lets synthesis replicate it).
  (* max_fanout = 32 *) reg fclr = 1'b1;
  always @(posedge clk) fclr <= rst_q1 | ~pass_en;

  // Prime the buffer to half depth before the first read, so that a small
  // phase error in either direction cannot immediately underflow.
  //
  // SELF-RECENTRING (added for 50 MSPS operation). Before this, priming only
  // happened once. A sample-rate change (the AD9361 BBPLL re-locks, RX and
  // TX strobes stop at different moments) could leave the buffer pinned at
  // FULL for ever: seen on hardware as fifo level 31/32 with OVERFLOW_COUNT
  // still creeping up long after the change -- every small strobe jitter
  // dropped another sample, i.e. a glitch on TX1. Now:
  //   overflow  (write while full) -> drop half the buffer (read pointer
  //                                   jumps forward by HALF): one glitch,
  //                                   then back at half depth;
  //   underflow (read while empty) -> un-prime: reads pause until half full.
  // Pointers are also cleared while pass_en = 0, so gnss_tx=0/1 always
  // restarts from a clean, half-full buffer.
  localparam [AW:0] HALF = (1 << (AW-1));
  reg  primed0 = 1'b0, primed1 = 1'b0;
  // DAC read strobes registered once: the read happens one clock after
  // dac_valid, still >= 1 clock before the DAC's next dac_valid, so TX1 sees
  // the same continuous stream. REQUIRES dac_valid never high on two
  // consecutive l_clk cycles: true for the LVDS interface (E310), where it is
  // every 4th l_clk at 2R2T and every 2nd at 1R1T.
  reg  rd_req0 = 1'b0, rd_req1 = 1'b0;
  always @(posedge clk) begin
    rd_req0 <= pass_en & dac_valid_i0;
    rd_req1 <= pass_en & dac_valid_i1;
  end
  always @(posedge clk) begin
    if (fclr) begin
      primed0 <= 1'b0;
      primed1 <= 1'b0;
    end else begin
      if (primed0 && rd_req0 && empty0) primed0 <= 1'b0;
      else if (level0 >= HALF)          primed0 <= 1'b1;
      if (primed1 && rd_req1 && empty1) primed1 <= 1'b0;
      else if (level1 >= HALF)          primed1 <= 1'b1;
    end
  end

  // ==========================================================================
  //  PROCESSING CORE  --  replace this section with the CRPA algorithm.
  //  Phase 1: identity.  proc_i/proc_q are the values written into the
  //  elastic buffer; they are the raw RX samples, unmodified.
  // ==========================================================================
  // ch0: MVDR output (12-bit, already saturated by mvdr_top's output scaler, sign-
  // extended to 16 bits like the RX format) or raw RX1.  The alignment stage below
  // takes [11:0], so the MVDR path can never wrap.
  wire [15:0] proc_i0 = mvdr_sel ? {{4{mvdr_out_i[11]}}, mvdr_out_i} : adc_data_i0;
  wire [15:0] proc_q0 = mvdr_sel ? {{4{mvdr_out_q[11]}}, mvdr_out_q} : adc_data_q0;
  wire [15:0] proc_i1 = adc_data_i1;
  wire [15:0] proc_q1 = adc_data_q1;
  // ======================= END PROCESSING CORE ==============================

  // Write side registered once (one clock of extra latency, invisible): the
  // write strobe then comes straight from a flop.
  // ch0: raw RX1 strobe (Phase 1) or the MVDR output strobe (which arrives a few
  // clocks after the sample).  ch1 is unchanged.
  reg         wr_en0 = 1'b0, wr_en1 = 1'b0;
  reg  [31:0] wd0 = 32'd0, wd1 = 32'd0;
  always @(posedge clk) begin
    wr_en0 <= pass_en & (mvdr_sel ? mvdr_out_valid : (adc_valid_i0 & adc_enable_i0));
    wr_en1 <= pass_en & adc_valid_i1 & adc_enable_i1;
    wd0    <= {proc_q0, proc_i0};
    wd1    <= {proc_q1, proc_i1};
  end
  wire rd_en0 = rd_req0 & primed0;
  wire rd_en1 = rd_req1 & primed1;

  // Every FIFO decision is a function of 5 flops (wr_en, rd_req, primed,
  // full, empty), written out so each maps to a single LUT:
  //   w_ok = write accepted, w_ov = write while full (recentre),
  //   r_ok = read (an overflow write takes priority over a read),
  //   inc  = level +1 (write, no read), dec = level -1 (read, no write)
  wire w_ok0 = wr_en0 & ~full0,  w_ov0 = wr_en0 & full0;
  wire r_ok0 = rd_req0 & primed0 & ~empty0 & ~(wr_en0 & full0);
  wire inc0  = wr_en0 & ~full0 & ~(rd_req0 & primed0 & ~empty0);
  wire dec0  = rd_req0 & primed0 & ~empty0 & ~wr_en0;
  wire w_ok1 = wr_en1 & ~full1,  w_ov1 = wr_en1 & full1;
  wire r_ok1 = rd_req1 & primed1 & ~empty1 & ~(wr_en1 & full1);
  wire inc1  = wr_en1 & ~full1 & ~(rd_req1 & primed1 & ~empty1);
  wire dec1  = rd_req1 & primed1 & ~empty1 & ~wr_en1;

  reg  [31:0] rd_data0 = 32'd0;
  reg  [31:0] rd_data1 = 32'd0;

  // The RAM write is kept OUT of the reset branch so the distributed RAM
  // still infers cleanly; only the pointers are reset.
  always @(posedge clk) begin
    if (w_ok0) fifo0[wptr0[AW-1:0]] <= wd0;
    if (w_ok1) fifo1[wptr1[AW-1:0]] <= wd1;
  end
  always @(posedge clk) begin
    if (r_ok0) rd_data0 <= fifo0[rptr0[AW-1:0]];
    if (r_ok1) rd_data1 <= fifo1[rptr1[AW-1:0]];
  end

  // level change: +1 (write only), -1 (read only), overflow (only possible
  // when full): level = TOP - HALF = HALF, else unchanged. level+1 and level-1
  // are kept in their own registers, so the next level is a mux of flops
  // controlled by single-LUT decisions (all inputs are flops).
  localparam [AW:0] TOP = (1 << AW), TOP1 = (1 << AW) - 1;
  reg  [AW:0] lvp0 = 1, lvm0 = {(AW+1){1'b1}}, lvp1 = 1, lvm1 = {(AW+1){1'b1}};   // level +1, level -1
  always @(posedge clk) begin
    if (fclr) begin
      wptr0 <= 0; rptr0 <= 0; level0 <= 0; lvp0 <= 1; lvm0 <= {(AW+1){1'b1}}; full0 <= 1'b0; empty0 <= 1'b1;
      wptr1 <= 0; rptr1 <= 0; level1 <= 0; lvp1 <= 1; lvm1 <= {(AW+1){1'b1}}; full1 <= 1'b0; empty1 <= 1'b1;
    end else begin
      if (w_ok0) wptr0 <= wptr0 + 1'b1;
      if (w_ov0)      rptr0 <= rptr0 + HALF;      // overflow: recentre
      else if (r_ok0) rptr0 <= rptr0 + 1'b1;
      if (w_ov0)     begin level0 <= HALF;  lvp0 <= HALF + 1'b1;  lvm0 <= HALF - 1'b1; end
      else if (inc0) begin level0 <= lvp0;  lvp0 <= lvp0 + 1'b1;  lvm0 <= level0;      end
      else if (dec0) begin level0 <= lvm0;  lvp0 <= level0;       lvm0 <= lvm0 - 1'b1; end
      full0  <= ((level0 == TOP1) & inc0) | ((level0 == TOP) & ~dec0 & ~w_ov0);
      empty0 <= ((level0 == 1) & dec0) | ((level0 == 0) & ~inc0);

      if (w_ok1) wptr1 <= wptr1 + 1'b1;
      if (w_ov1)      rptr1 <= rptr1 + HALF;
      else if (r_ok1) rptr1 <= rptr1 + 1'b1;
      if (w_ov1)     begin level1 <= HALF;  lvp1 <= HALF + 1'b1;  lvm1 <= HALF - 1'b1; end
      else if (inc1) begin level1 <= lvp1;  lvp1 <= lvp1 + 1'b1;  lvm1 <= level1;      end
      else if (dec1) begin level1 <= lvm1;  lvp1 <= level1;       lvm1 <= lvm1 - 1'b1; end
      full1  <= ((level1 == TOP1) & inc1) | ((level1 == TOP) & ~dec1 & ~w_ov1);
      empty1 <= ((level1 == 1) & dec1) | ((level1 == 0) & ~inc1);
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

  (* max_fanout = 32 *) reg cclr = 1'b1;
  always @(posedge clk) cclr <= rst_q1 | cnt_clear;     // aligned with rst_l

  always @(posedge clk) begin
    if (cclr) begin
      rx_cnt0 <= 32'd0; tx_cnt0 <= 32'd0;
      rx_cnt1 <= 32'd0; tx_cnt1 <= 32'd0;
      ovf_cnt <= 32'd0; unf_cnt <= 32'd0;
      ovf_sticky <= 1'b0; unf_sticky <= 1'b0;
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
    end
  end

  wire [31:0] status_w;
  assign status_w = { 14'd0, mvdr_sel, pass_en,
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

  reg cap_now = 1'b0;                     // registered: cap_div == 0 this clock
  always @(posedge clk) begin
    cap_div <= cap_div + 1'b1;
    cap_now <= (cap_div == 8'd255);
    if (cap_now) begin
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
