// ============================================================================
//  gnss_doa_esprit.v
//  ANTSDR E310 V1 / GNSS CRPA project -- ESPRIT direction-of-arrival monitor
// ----------------------------------------------------------------------------
//  PURPOSE
//    Estimates the direction of arrival (DOA) of the dominant RX source from
//    the two AD9361 receive channels (RX1 = element 0, RX2 = element 1), using
//    the algorithms of
//
//      J. Li, G. Yang, X. Ren, Z. Fu, S. Qiu, "Modified ESPRIT and
//      Unitary-Transformation Form for Computationally Efficient DOA
//      Estimation on FPGA", IEEE Trans. Instrum. Meas., vol. 74, 2025,
//      Art. 2003209.
//
//    Two independent estimators run side by side on the same snapshots:
//      * I-UESPRIT  (improved unitary ESPRIT, Sec. III-B, real-valued)
//      * M-ESPRIT   (modified ESPRIT, Sec. III-A, complex eigenvalue with
//                    unit-circle source test, Remark A)
//    Either can be removed at build time with ENABLE_IUESPRIT / ENABLE_MESPRIT.
//
//    The block is RECEIVE-ONLY. It taps the RX samples and never touches the
//    RX->TX passthrough, the elastic buffers or the DAC mux of
//    gnss_passthrough. With DOA_CTRL.enable = 0 (reset value) it is held in
//    reset and the design behaves exactly like gnss_passthrough v1.1.
//
//  DATAPATH
//
//   l_clk (sample domain)          | sys_cpu_clk (100 MHz, AXI domain)
//                                  |
//   adc_data_{i,q}{0,1} --decim--> async --> [source mux] --> cal --> I-UESPRIT --> regs
//   (12b right-aligned)    <<4     FIFO  |       ^          rotate    M-ESPRIT  --> regs
//                                  |     |   doa_test_gen   (RX2)     averaging
//
//   * RX samples are 12-bit right-aligned (ad_datafmt, DATA_WIDTH 12); they
//     are left-aligned to Q1.15 for the estimators.
//   * DECIM keeps 1 of every (DECIM+1) snapshots (0 = all).
//   * doa_test_gen: built-in synthetic two-element source with programmable
//     inter-element phase, for hardware verification with a KNOWN angle.
//   * CAL: complex rotation applied to element 1,  x1' = x1 * (c + j s),
//     to remove the fixed RX1/RX2 phase offset of the board and cables.
//   * Averaging: every 2^AVG_LOG2 results the mean angle and the mean square
//     angle are latched, so firmware can report mean and standard deviation.
//
//  REGISTERS  (offsets from the gnss_passthrough base, 0x43C0_0000)
//    0x100 DOA_ID          RO  0x444F4131 "DOA1"
//    0x104 DOA_VERSION     RO  0x00010000 (1.0)
//    0x108 DOA_CTRL        RW  [0] enable       [1] src_test (1 = test gen)
//                              [2] use_cov      [3] tg_noise_en
//                              [5:4] p_log2 (C^p, p = 2^n)
//                              [12:8] log2_snap (window 2^n, n = 7..14)
//                              [19:16] avg_log2 (average 2^n results, 0..10)
//                              [24] soft_reset  [25] clear_stats
//                              reset 0x00040A20 (disabled, p=4, 1024, avg 16)
//    0x10C DOA_MU_Q30      RW  regulariser mu, Q30           (1024)
//    0x110 DOA_K_Q30       RW  k = lambda/(2 pi d), Q2.30    (341782638 = 1/pi)
//    0x114 DOA_UCTOL_Q24   RW  M-ESPRIT unit-circle tolerance (0x00400000 = 0.25)
//    0x118 DOA_DECIM       RW  [15:0] keep 1 of (N+1) snapshots (0)
//    0x11C DOA_CAL         RW  {sin_q15[31:16], cos_q15[15:0]} (0x00007FFF)
//    0x120 DOA_TG_FTW      RW  test tone phase increment per sample (0x0147AE14)
//    0x124 DOA_TG_DPHASE   RW  test inter-element phase, 2^32 = 360 deg (0)
//    0x128 DOA_TG_CFG      RW  [15:0] clocks per sample (>= 8) [19:16] amp shift
//                              [23:20] noise shift            (0x00400064)
//    0x12C DOA_STATUS      RO  [0] cdc_overflow (sticky) [1] I present [2] M present
//                              [3] enable [4] src_test [11:8] I flags [15:12] M flags
//    0x130 DOA_IN_COUNT    RO  snapshots delivered to the estimators
//    0x134 DOA_PEAK        RO  {peak |x1|, peak |x0|} over the last window (Q1.15)
//    0x138 I_SEQ           RO  I-UESPRIT result counter
//    0x13C I_PHI_Q28       RO  electrical phase arg(x1/x0), rad Q28
//    0x140 I_PSI_Q24       RO  eigenvalue psi = tan(phi/2), Q8.24
//    0x144 I_THETA_Q28     RO  DOA, rad Q28
//    0x148 I_DEG_Q16       RO  DOA, degrees Q16
//    0x14C I_AVG_SEQ       RO  averaged-result counter
//    0x150 I_AVG_DEG_Q16   RO  mean DOA over the last 2^avg_log2 results
//    0x154 I_AVG_SQ_LO     RO  mean of DOA^2, deg^2 Q32, bits [31:0]
//    0x158 I_AVG_SQ_HI     RO  bits [63:32]
//    0x15C..0x17C          M_SEQ, M_PHI_Q28, M_PSIMAG_Q24, M_THETA_Q28, M_DEG_Q16,
//                          M_AVG_SEQ, M_AVG_DEG_Q16, M_AVG_SQ_LO, M_AVG_SQ_HI
//                          (same layout; PSIMAG = |psi|, ~1.0 for a real source)
//    0x180 M_AVG_OFFCIRC   RO  off-circle windows in the last average block
//    0x184 DOA_C00         RO  normalised cumulant matrix (debug, Q30)
//    0x188 DOA_C11         RO
//    0x18C DOA_C01_RE      RO
//    0x190 DOA_C01_IM      RO
//    0x194 DOA_CAPS        RO  [0] I-UESPRIT built [1] M-ESPRIT built
//                              [2] estimator is an external HLS IP [15:8] MAX_LOG2_SNAP
//
//  FLAGS (per estimator, 4 bits)
//    I-UESPRIT: {0,          overrun, ambiguous, no_signal}
//    M-ESPRIT : {off_circle, overrun, ambiguous, no_signal}
//
//  CLOCKS AND CDC
//    clk   : axi_ad9361/l_clk  (rx_clk, 8 ns)  -- capture + decimation only
//    aclk  : sys_cpu_clk       (clk_fpga_0, 10 ns) -- everything else
//    Crossings: doa_async_fifo (gray pointers), and quasi-static control bits
//    through two-flop synchronisers named dsync_meta/dsync. Constraints are in
//    Source/XDC/gnss_passthrough_cdc.xdc, section 4.
//
//  Original work for this project.
// ============================================================================

`timescale 1ns/100ps

module gnss_doa_esprit #(
  parameter ENABLE_IUESPRIT = 1,
  parameter ENABLE_MESPRIT  = 1,
  parameter MAX_LOG2_SNAP   = 14,
  // 0: estimators are the RTL cores below.  1: the estimator is an external
  // Vitis HLS IP (iuesprit_hls / mesprit_hls) in the block design, fed by
  // m_axis_doa and answering on s_axis_res. Exactly one of ENABLE_IUESPRIT /
  // ENABLE_MESPRIT selects which result registers the HLS IP fills.
  parameter HLS_CORE        = 0
) (
  // ---- sample domain -------------------------------------------------------
  input               clk,
  input               rst,
  input               adc_valid,
  input               adc_enable_0,
  input               adc_enable_1,
  input      [15:0]   adc_data_i0,
  input      [15:0]   adc_data_q0,
  input      [15:0]   adc_data_i1,
  input      [15:0]   adc_data_q1,

  // ---- register domain -----------------------------------------------------
  input               aclk,
  input               aresetn,
  input               reg_wr,          // one-cycle write strobe
  input      [5:0]    reg_widx,        // word index inside the 0x100 window
  input      [31:0]   reg_wdata,
  input      [5:0]    reg_ridx,
  output reg [31:0]   reg_rdata,       // combinational read data

  // ---- external HLS estimator (aclk domain, used when HLS_CORE = 1) ----------
  output     [63:0]   m_axis_doa_tdata,   // {q1, i1, q0, i0}, Q1.15, after CAL
  output              m_axis_doa_tvalid,
  input               m_axis_doa_tready,
  input      [31:0]   s_axis_res_tdata,   // 9-word result packet (esprit_hls.h)
  input               s_axis_res_tvalid,
  output              s_axis_res_tready,
  output     [4:0]    doa_cfg_log2_snap,
  output     [1:0]    doa_cfg_p_log2,
  output              doa_cfg_use_cov,
  output     [31:0]   doa_cfg_mu_q30,
  output     [31:0]   doa_cfg_k_q30,
  output     [31:0]   doa_cfg_uctol_q24,
  output              doa_hls_rst_n       // HLS ap_rst_n: low while disabled
);

  localparam [31:0] DOA_ID      = 32'h444F4131;   // "DOA1"
  localparam [31:0] DOA_VERSION = 32'h00010000;   // 1.0
  localparam [7:0]  MLS         = MAX_LOG2_SNAP;

  // ==========================================================================
  //  Registers (aclk)
  // ==========================================================================
  reg [31:0] r_ctrl   = 32'h00040A20;
  reg [31:0] r_mu     = 32'd1024;
  reg [31:0] r_k      = 32'd341782638;
  reg [31:0] r_uctol  = 32'h00400000;
  reg [31:0] r_decim  = 32'd0;
  reg [31:0] r_cal    = 32'h00007FFF;
  reg [31:0] r_ftw    = 32'h0147AE14;
  reg [31:0] r_dphase = 32'd0;
  reg [31:0] r_tgcfg  = 32'h00400064;

  always @(posedge aclk) begin
    if (!aresetn) begin
      r_ctrl   <= 32'h00040A20;
      r_mu     <= 32'd1024;
      r_k      <= 32'd341782638;
      r_uctol  <= 32'h00400000;
      r_decim  <= 32'd0;
      r_cal    <= 32'h00007FFF;
      r_ftw    <= 32'h0147AE14;
      r_dphase <= 32'd0;
      r_tgcfg  <= 32'h00400064;
    end else if (reg_wr) begin
      case (reg_widx)
        6'd2:  r_ctrl   <= reg_wdata;
        6'd3:  r_mu     <= reg_wdata;
        6'd4:  r_k      <= reg_wdata;
        6'd5:  r_uctol  <= reg_wdata;
        6'd6:  r_decim  <= reg_wdata;
        6'd7:  r_cal    <= reg_wdata;
        6'd8:  r_ftw    <= reg_wdata;
        6'd9:  r_dphase <= reg_wdata;
        6'd10: r_tgcfg  <= reg_wdata;
        default: ;
      endcase
    end
  end

  wire       c_enable    = r_ctrl[0];
  wire       c_src_test  = r_ctrl[1];
  wire       c_use_cov   = r_ctrl[2];
  wire       c_tg_noise  = r_ctrl[3];
  wire [1:0] c_p_log2    = r_ctrl[5:4];
  wire [4:0] c_log2_snap = r_ctrl[12:8];
  wire [3:0] c_avg_raw   = r_ctrl[19:16];
  wire [3:0] c_avg_log2  = (c_avg_raw > 4'd10) ? 4'd10 : c_avg_raw;
  wire       c_soft_rst  = r_ctrl[24];
  wire       c_clr_stats = r_ctrl[25];

  // processing-domain reset: held while disabled or soft-reset
  reg proc_rst = 1'b1;
  always @(posedge aclk) proc_rst <= ~aresetn | c_soft_rst | ~c_enable;

  // ==========================================================================
  //  Sample domain: capture, decimate, left-align, push into the FIFO
  // ==========================================================================
  // quasi-static control into clk: {capture enable, reset, decim[15:0]}
  (* ASYNC_REG = "TRUE" *) reg [17:0] dsync_meta = 18'd0;
  (* ASYNC_REG = "TRUE" *) reg [17:0] dsync      = 18'd0;
  wire [17:0] dsync_src = {c_enable & ~c_src_test,
                           ~aresetn | c_soft_rst | ~c_enable,
                           r_decim[15:0]};
  always @(posedge clk) begin
    dsync_meta <= dsync_src;
    dsync      <= dsync_meta;
  end
  wire        s_capture = dsync[17];
  wire        s_rst    = dsync[16] | rst;
  wire [15:0] s_decim  = dsync[15:0];

  reg  [15:0] s_dcnt = 16'd0;
  reg         s_push = 1'b0;
  reg  [63:0] s_word = 64'd0;

  always @(posedge clk) begin
    if (s_rst) begin
      s_dcnt <= 16'd0;
      s_push <= 1'b0;
    end else begin
      s_push <= 1'b0;
      if (s_capture && adc_valid && adc_enable_0 && adc_enable_1) begin
        if (s_dcnt >= s_decim) begin
          s_dcnt <= 16'd0;
          s_push <= 1'b1;
          // {Q1, I1, Q0, I0}, 12-bit right-aligned -> Q1.15
          s_word <= {adc_data_q1[11:0], 4'b0000, adc_data_i1[11:0], 4'b0000,
                     adc_data_q0[11:0], 4'b0000, adc_data_i0[11:0], 4'b0000};
        end else begin
          s_dcnt <= s_dcnt + 1'b1;
        end
      end
    end
  end

  wire        f_valid;
  wire [63:0] f_data;
  wire        f_full, f_wovf, f_rovf;

  doa_async_fifo #(.DW(64), .AW(4)) u_afifo (
    .wclk (clk),  .wrst (s_rst), .wr_en (s_push), .wr_data (s_word),
    .wr_full (f_full), .wr_overflow (f_wovf),
    .rclk (aclk), .rrst (proc_rst), .rd_valid (f_valid), .rd_data (f_data),
    .rd_overflow (f_rovf)
  );

  // ==========================================================================
  //  Processing domain: source mux
  // ==========================================================================
  wire               tg_valid;
  wire signed [15:0] tg_x0i, tg_x0q, tg_x1i, tg_x1q;

  doa_test_gen u_tgen (
    .clk (aclk), .rst (proc_rst), .en (c_src_test),
    .div (r_tgcfg[15:0]), .ftw (r_ftw), .dphase (r_dphase),
    .amp_shift (r_tgcfg[19:16]), .noise_en (c_tg_noise), .noise_shift (r_tgcfg[23:20]),
    .valid (tg_valid), .x0_i (tg_x0i), .x0_q (tg_x0q), .x1_i (tg_x1i), .x1_q (tg_x1q)
  );

  reg               m_valid = 1'b0;
  reg signed [15:0] m_x0i, m_x0q, m_x1i, m_x1q;

  always @(posedge aclk) begin
    if (proc_rst) begin
      m_valid <= 1'b0;
    end else begin
      m_valid <= c_src_test ? tg_valid : f_valid;
      if (c_src_test) begin
        m_x0i <= tg_x0i; m_x0q <= tg_x0q; m_x1i <= tg_x1i; m_x1q <= tg_x1q;
      end else begin
        m_x0i <= f_data[15:0];  m_x0q <= f_data[31:16];
        m_x1i <= f_data[47:32]; m_x1q <= f_data[63:48];
      end
    end
  end

  // ==========================================================================
  //  Calibration rotation of element 1:  x1' = x1 * (c + j s) / 2^15
  // ==========================================================================
  wire signed [15:0] cal_c = r_cal[15:0];
  wire signed [15:0] cal_s = r_cal[31:16];

  reg               r1_valid = 1'b0;
  reg signed [31:0] p_ic, p_qs, p_is, p_qc;
  reg signed [15:0] r1_x0i, r1_x0q;

  reg               r2_valid = 1'b0;
  reg signed [15:0] r2_x0i, r2_x0q, r2_x1i, r2_x1q;

  function signed [15:0] sat_q15;
    input signed [32:0] v;          // Q30 sum
    reg   signed [32:0] s;
    begin
      s = v >>> 15;
      if (s > 33'sd32767)       sat_q15 = 16'sd32767;
      else if (s < -33'sd32768) sat_q15 = -16'sd32768;
      else                      sat_q15 = s[15:0];
    end
  endfunction

  always @(posedge aclk) begin
    if (proc_rst) begin
      r1_valid <= 1'b0;
      r2_valid <= 1'b0;
    end else begin
      // stage 1: products
      r1_valid <= m_valid;
      p_ic <= m_x1i * cal_c;
      p_qs <= m_x1q * cal_s;
      p_is <= m_x1i * cal_s;
      p_qc <= m_x1q * cal_c;
      r1_x0i <= m_x0i; r1_x0q <= m_x0q;
      // stage 2: sums, saturate
      r2_valid <= r1_valid;
      r2_x1i <= sat_q15($signed({p_ic[31], p_ic}) - $signed({p_qs[31], p_qs}));
      r2_x1q <= sat_q15($signed({p_is[31], p_is}) + $signed({p_qc[31], p_qc}));
      r2_x0i <= r1_x0i; r2_x0q <= r1_x0q;
    end
  end

  // input statistics: snapshot count and per-window peak magnitude
  reg  [31:0] in_count = 32'd0;
  reg  [15:0] pk0_run = 16'd0, pk1_run = 16'd0, pk0 = 16'd0, pk1 = 16'd0;

  function [15:0] mag16;
    input signed [15:0] a;
    input signed [15:0] b;
    reg   [15:0] ua, ub;
    begin
      ua = a[15] ? (~a + 1'b1) : a;
      ub = b[15] ? (~b + 1'b1) : b;
      mag16 = (ua > ub) ? ua : ub;
    end
  endfunction

  wire [15:0] mag0 = mag16(r2_x0i, r2_x0q);
  wire [15:0] mag1 = mag16(r2_x1i, r2_x1q);

  // ==========================================================================
  //  External HLS estimator interface (HLS_CORE = 1)
  //    * 1024-snapshot stream buffer: holds the samples that arrive while the
  //      HLS kernel computes the previous window (its input loop is II = 1,
  //      so it drains the buffer far faster than samples arrive);
  //    * result parser: 9-word packet -> the same valid/data signals the RTL
  //      cores produce, so registers, averaging and firmware are unchanged.
  // ==========================================================================
  reg hls_rst_n_r = 1'b0;
  always @(posedge aclk) hls_rst_n_r <= ~proc_rst;

  assign doa_cfg_log2_snap = c_log2_snap;
  assign doa_cfg_p_log2    = c_p_log2;
  assign doa_cfg_use_cov   = c_use_cov;
  assign doa_cfg_mu_q30    = r_mu;
  assign doa_cfg_k_q30     = r_k;
  assign doa_cfg_uctol_q24 = r_uctol;
  assign doa_hls_rst_n     = hls_rst_n_r;

  wire               h_valid;
  wire signed [31:0] h_phi, h_psi, h_th, h_deg;
  wire [3:0]         h_flags;
  wire signed [31:0] h_c00, h_c11, h_c01r, h_c01i;

  generate
    if (HLS_CORE != 0) begin : g_hls
      // ---- stream buffer -------------------------------------------------
      (* ram_style = "block" *) reg [63:0] sb_mem [0:1023];
      reg  [10:0] sb_wp = 11'd0, sb_rp = 11'd0;
      reg  [63:0] sb_out = 64'd0;
      reg         sb_out_v = 1'b0;
      reg         sb_drop = 1'b0;          // sticky until the next result
      wire        sb_full  = (sb_wp[10] != sb_rp[10]) && (sb_wp[9:0] == sb_rp[9:0]);
      wire        sb_empty = (sb_wp == sb_rp);
      wire        sb_take  = !sb_empty && (!sb_out_v || m_axis_doa_tready);
      reg         hp_done = 1'b0;

      always @(posedge aclk) begin
        if (r2_valid && !sb_full) sb_mem[sb_wp[9:0]] <= {r2_x1q, r2_x1i, r2_x0q, r2_x0i};
        if (sb_take) sb_out <= sb_mem[sb_rp[9:0]];
      end
      always @(posedge aclk) begin
        if (proc_rst) begin
          sb_wp <= 11'd0; sb_rp <= 11'd0; sb_out_v <= 1'b0; sb_drop <= 1'b0;
        end else begin
          if (r2_valid && !sb_full) sb_wp <= sb_wp + 1'b1;
          if (r2_valid && sb_full)  sb_drop <= 1'b1;
          else if (hp_done)         sb_drop <= 1'b0;
          if (sb_take)                            begin sb_rp <= sb_rp + 1'b1; sb_out_v <= 1'b1; end
          else if (sb_out_v && m_axis_doa_tready) sb_out_v <= 1'b0;
        end
      end
      assign m_axis_doa_tdata  = sb_out;
      assign m_axis_doa_tvalid = sb_out_v;

      // ---- result parser -------------------------------------------------
      reg  [3:0]  hp_idx = 4'd0;
      reg  [31:0] hp_w [1:8];
      reg  [3:0]  hp_fl = 4'd0;
      reg         hp_ovr = 1'b0;
      always @(posedge aclk) begin
        hp_done <= 1'b0;
        if (proc_rst) begin
          hp_idx <= 4'd0;
        end else if (s_axis_res_tvalid) begin
          if (hp_idx == 4'd0) begin
            if (s_axis_res_tdata[31:16] == 16'hE5A1) begin    // magic: start of packet
              hp_fl  <= s_axis_res_tdata[11:8];
              hp_ovr <= sb_drop;
              hp_idx <= 4'd1;
            end
          end else begin
            hp_w[hp_idx] <= s_axis_res_tdata;
            if (hp_idx == 4'd8) begin hp_idx <= 4'd0; hp_done <= 1'b1; end
            else                       hp_idx <= hp_idx + 1'b1;
          end
        end
      end
      assign s_axis_res_tready = 1'b1;
      assign h_valid = hp_done;
      assign h_phi   = hp_w[1];  assign h_psi  = hp_w[2];
      assign h_th    = hp_w[3];  assign h_deg  = hp_w[4];
      assign h_c00   = hp_w[5];  assign h_c11  = hp_w[6];
      assign h_c01r  = hp_w[7];  assign h_c01i = hp_w[8];
      assign h_flags = {hp_fl[3], hp_ovr, hp_fl[1:0]};
    end else begin : g_no_hls
      assign m_axis_doa_tdata  = 64'd0;
      assign m_axis_doa_tvalid = 1'b0;
      assign s_axis_res_tready = 1'b1;
      assign h_valid = 1'b0;
      assign h_phi = 32'sd0; assign h_psi = 32'sd0; assign h_th = 32'sd0; assign h_deg = 32'sd0;
      assign h_c00 = 32'sd0; assign h_c11 = 32'sd0; assign h_c01r = 32'sd0; assign h_c01i = 32'sd0;
      assign h_flags = 4'd0;
    end
  endgenerate

  // ==========================================================================
  //  Estimators
  // ==========================================================================
  wire               i_res_valid, m_res_valid;
  wire signed [31:0] i_phi, i_psi, i_th, i_deg;
  wire signed [31:0] m_phi, m_psi, m_th, m_deg;
  wire [3:0]         i_flags, m_flags;
  wire signed [31:0] i_c00, i_c11, i_c01r, i_c01i;
  wire signed [31:0] m_c00, m_c11, m_c01r, m_c01i;

  generate
    if (ENABLE_IUESPRIT && HLS_CORE == 0) begin : g_iuesprit
      i_uesprit_2ant_core #(.MAX_LOG2_SNAP(MAX_LOG2_SNAP)) u_core (
        .clk (aclk), .rst (proc_rst),
        .cfg_log2_snap (c_log2_snap), .cfg_p_log2 (c_p_log2),
        .cfg_mu_q30 (r_mu), .cfg_k_q30 (r_k), .cfg_use_cov (c_use_cov),
        .in_valid (r2_valid),
        .x0_i (r2_x0i), .x0_q (r2_x0q), .x1_i (r2_x1i), .x1_q (r2_x1q),
        .res_valid (i_res_valid),
        .res_phi_q28 (i_phi), .res_psi_q24 (i_psi),
        .res_theta_q28 (i_th), .res_theta_deg_q16 (i_deg),
        .res_flags (i_flags),
        .dbg_c00 (i_c00), .dbg_c11 (i_c11), .dbg_c01_re (i_c01r), .dbg_c01_im (i_c01i)
      );
    end else if (ENABLE_IUESPRIT) begin : g_hls_iuesprit   // external HLS IP
      assign i_res_valid = h_valid;
      assign i_phi = h_phi; assign i_psi = h_psi; assign i_th = h_th; assign i_deg = h_deg;
      assign i_flags = h_flags;
      assign i_c00 = h_c00; assign i_c11 = h_c11; assign i_c01r = h_c01r; assign i_c01i = h_c01i;
    end else begin : g_no_iuesprit
      assign i_res_valid = 1'b0;
      assign i_phi = 32'sd0; assign i_psi = 32'sd0; assign i_th = 32'sd0; assign i_deg = 32'sd0;
      assign i_flags = 4'd0;
      assign i_c00 = 32'sd0; assign i_c11 = 32'sd0; assign i_c01r = 32'sd0; assign i_c01i = 32'sd0;
    end

    if (ENABLE_MESPRIT && HLS_CORE == 0) begin : g_mesprit
      m_esprit_2ant_core #(.MAX_LOG2_SNAP(MAX_LOG2_SNAP)) u_core (
        .clk (aclk), .rst (proc_rst),
        .cfg_log2_snap (c_log2_snap), .cfg_p_log2 (c_p_log2),
        .cfg_mu_q30 (r_mu), .cfg_k_q30 (r_k), .cfg_use_cov (c_use_cov),
        .cfg_uc_tol_q24 (r_uctol),
        .in_valid (r2_valid),
        .x0_i (r2_x0i), .x0_q (r2_x0q), .x1_i (r2_x1i), .x1_q (r2_x1q),
        .res_valid (m_res_valid),
        .res_phi_q28 (m_phi), .res_psi_mag_q24 (m_psi),
        .res_theta_q28 (m_th), .res_theta_deg_q16 (m_deg),
        .res_flags (m_flags),
        .dbg_c00 (m_c00), .dbg_c11 (m_c11), .dbg_c01_re (m_c01r), .dbg_c01_im (m_c01i)
      );
    end else if (ENABLE_MESPRIT && !ENABLE_IUESPRIT) begin : g_hls_mesprit   // external HLS IP
      assign m_res_valid = h_valid;
      assign m_phi = h_phi; assign m_psi = h_psi; assign m_th = h_th; assign m_deg = h_deg;
      assign m_flags = h_flags;
      assign m_c00 = h_c00; assign m_c11 = h_c11; assign m_c01r = h_c01r; assign m_c01i = h_c01i;
    end else begin : g_no_mesprit
      assign m_res_valid = 1'b0;
      assign m_phi = 32'sd0; assign m_psi = 32'sd0; assign m_th = 32'sd0; assign m_deg = 32'sd0;
      assign m_flags = 4'd0;
      assign m_c00 = 32'sd0; assign m_c11 = 32'sd0; assign m_c01r = 32'sd0; assign m_c01i = 32'sd0;
    end
  endgenerate

  // window boundary marker for the peak detector: first estimator present
  wire win_done = (ENABLE_IUESPRIT != 0) ? i_res_valid : m_res_valid;

  // ==========================================================================
  //  Result registers and averaging
  // ==========================================================================
  reg [31:0] i_seq = 0, i_phi_r = 0, i_psi_r = 0, i_th_r = 0, i_deg_r = 0;
  reg [31:0] m_seq = 0, m_phi_r = 0, m_psi_r = 0, m_th_r = 0, m_deg_r = 0;
  reg [3:0]  i_flags_r = 0, m_flags_r = 0;
  reg [31:0] dbg00 = 0, dbg11 = 0, dbg01r = 0, dbg01i = 0;
  reg        ovf_sticky = 1'b0;

  // one averaging unit per estimator
  reg               ia_v1 = 0, ia_v2 = 0, ma_v1 = 0, ma_v2 = 0;
  reg signed [23:0] ia_d = 0, ma_d = 0;            // |deg_q16| < 2^23 (90 deg)
  reg signed [47:0] ia_sq = 0, ma_sq = 0;
  reg signed [47:0] ia_sum = 0, ma_sum = 0;         // 24 + 10 bits + sign margin
  reg        [57:0] ia_ssum = 0, ma_ssum = 0;       // 46 + 10 bits
  reg        [10:0] ia_cnt = 0, ma_cnt = 0;
  reg        [10:0] ma_offc = 0;
  reg               ia_d_off = 0, ma_d_off = 0;
  reg        [31:0] i_avg_seq = 0, i_avg_deg = 0, m_avg_seq = 0, m_avg_deg = 0;
  reg        [63:0] i_avg_sq = 0, m_avg_sq = 0;
  reg        [31:0] m_avg_offc = 0;

  wire [10:0] avg_n = 11'd1 << c_avg_log2;

  function signed [23:0] deg24;
    input signed [31:0] d;
    begin
      if (d > 32'sd8388607)       deg24 = 24'sd8388607;
      else if (d < -32'sd8388608) deg24 = -24'sd8388608;
      else                        deg24 = d[23:0];
    end
  endfunction

  always @(posedge aclk) begin
    if (proc_rst || c_clr_stats) begin
      in_count <= 32'd0;
      pk0_run <= 16'd0; pk1_run <= 16'd0;
      if (proc_rst) begin
        // a disabled / reset estimator reports zeros, never a stale angle
        i_seq <= 0; m_seq <= 0; i_avg_seq <= 0; m_avg_seq <= 0;
        i_phi_r <= 0; i_psi_r <= 0; i_th_r <= 0; i_deg_r <= 0; i_flags_r <= 0;
        m_phi_r <= 0; m_psi_r <= 0; m_th_r <= 0; m_deg_r <= 0; m_flags_r <= 0;
        i_avg_deg <= 0; i_avg_sq <= 0; m_avg_deg <= 0; m_avg_sq <= 0; m_avg_offc <= 0;
        pk0 <= 16'd0; pk1 <= 16'd0;
      end
      ovf_sticky <= 1'b0;
      ia_v1 <= 0; ia_v2 <= 0; ma_v1 <= 0; ma_v2 <= 0;
      ia_sum <= 0; ia_ssum <= 0; ia_cnt <= 0;
      ma_sum <= 0; ma_ssum <= 0; ma_cnt <= 0; ma_offc <= 0;
    end else begin
      if (f_rovf) ovf_sticky <= 1'b1;

      if (r2_valid) begin
        in_count <= in_count + 1'b1;
        if (mag0 > pk0_run) pk0_run <= mag0;
        if (mag1 > pk1_run) pk1_run <= mag1;
      end
      if (win_done) begin
        pk0 <= pk0_run; pk1 <= pk1_run;
        pk0_run <= 16'd0; pk1_run <= 16'd0;
      end

      // ---------------- I-UESPRIT ----------------
      if (i_res_valid) begin
        i_seq   <= i_seq + 1'b1;
        i_phi_r <= i_phi; i_psi_r <= i_psi; i_th_r <= i_th; i_deg_r <= i_deg;
        i_flags_r <= i_flags;
        if (ENABLE_MESPRIT == 0) begin
          dbg00 <= i_c00; dbg11 <= i_c11; dbg01r <= i_c01r; dbg01i <= i_c01i;
        end
      end
      ia_v1 <= i_res_valid;
      if (i_res_valid) ia_d <= deg24(i_deg);
      ia_v2 <= ia_v1;
      if (ia_v1) ia_sq <= ia_d * ia_d;
      if (ia_v2) begin
        if (ia_cnt + 1'b1 >= avg_n) begin
          i_avg_deg <= (ia_sum + ia_d) >>> c_avg_log2;
          i_avg_sq  <= (ia_ssum + ia_sq[45:0]) >> c_avg_log2;
          i_avg_seq <= i_avg_seq + 1'b1;
          ia_sum <= 0; ia_ssum <= 0; ia_cnt <= 0;
        end else begin
          ia_sum  <= ia_sum + ia_d;
          ia_ssum <= ia_ssum + ia_sq[45:0];
          ia_cnt  <= ia_cnt + 1'b1;
        end
      end

      // ---------------- M-ESPRIT -----------------
      if (m_res_valid) begin
        m_seq   <= m_seq + 1'b1;
        m_phi_r <= m_phi; m_psi_r <= m_psi; m_th_r <= m_th; m_deg_r <= m_deg;
        m_flags_r <= m_flags;
        dbg00 <= m_c00; dbg11 <= m_c11; dbg01r <= m_c01r; dbg01i <= m_c01i;
      end
      ma_v1 <= m_res_valid;
      if (m_res_valid) begin
        ma_d     <= deg24(m_deg);
        ma_d_off <= m_flags[3];
      end
      ma_v2 <= ma_v1;
      if (ma_v1) ma_sq <= ma_d * ma_d;
      if (ma_v2) begin
        if (ma_cnt + 1'b1 >= avg_n) begin
          m_avg_deg  <= (ma_sum + ma_d) >>> c_avg_log2;
          m_avg_sq   <= (ma_ssum + ma_sq[45:0]) >> c_avg_log2;
          m_avg_offc <= ma_offc + ma_d_off;
          m_avg_seq  <= m_avg_seq + 1'b1;
          ma_sum <= 0; ma_ssum <= 0; ma_cnt <= 0; ma_offc <= 0;
        end else begin
          ma_sum  <= ma_sum + ma_d;
          ma_ssum <= ma_ssum + ma_sq[45:0];
          ma_cnt  <= ma_cnt + 1'b1;
          ma_offc <= ma_offc + ma_d_off;
        end
      end
    end
  end

  // ==========================================================================
  //  Read mux (combinational; gnss_passthrough registers it)
  // ==========================================================================
  wire [31:0] status_w = {16'd0, m_flags_r, i_flags_r, 3'd0,
                          c_src_test, c_enable,
                          (ENABLE_MESPRIT != 0), (ENABLE_IUESPRIT != 0), ovf_sticky};
  wire [31:0] caps_w   = {16'd0, MLS, 5'd0, (HLS_CORE != 0),
                          (ENABLE_MESPRIT != 0) && !(HLS_CORE != 0 && ENABLE_IUESPRIT != 0),
                          (ENABLE_IUESPRIT != 0)};

  always @(*) begin
    case (reg_ridx)
      6'd0:  reg_rdata = DOA_ID;
      6'd1:  reg_rdata = DOA_VERSION;
      6'd2:  reg_rdata = r_ctrl;
      6'd3:  reg_rdata = r_mu;
      6'd4:  reg_rdata = r_k;
      6'd5:  reg_rdata = r_uctol;
      6'd6:  reg_rdata = r_decim;
      6'd7:  reg_rdata = r_cal;
      6'd8:  reg_rdata = r_ftw;
      6'd9:  reg_rdata = r_dphase;
      6'd10: reg_rdata = r_tgcfg;
      6'd11: reg_rdata = status_w;
      6'd12: reg_rdata = in_count;
      6'd13: reg_rdata = {pk1, pk0};
      6'd14: reg_rdata = i_seq;
      6'd15: reg_rdata = i_phi_r;
      6'd16: reg_rdata = i_psi_r;
      6'd17: reg_rdata = i_th_r;
      6'd18: reg_rdata = i_deg_r;
      6'd19: reg_rdata = i_avg_seq;
      6'd20: reg_rdata = i_avg_deg;
      6'd21: reg_rdata = i_avg_sq[31:0];
      6'd22: reg_rdata = i_avg_sq[63:32];
      6'd23: reg_rdata = m_seq;
      6'd24: reg_rdata = m_phi_r;
      6'd25: reg_rdata = m_psi_r;
      6'd26: reg_rdata = m_th_r;
      6'd27: reg_rdata = m_deg_r;
      6'd28: reg_rdata = m_avg_seq;
      6'd29: reg_rdata = m_avg_deg;
      6'd30: reg_rdata = m_avg_sq[31:0];
      6'd31: reg_rdata = m_avg_sq[63:32];
      6'd32: reg_rdata = m_avg_offc;
      6'd33: reg_rdata = dbg00;
      6'd34: reg_rdata = dbg11;
      6'd35: reg_rdata = dbg01r;
      6'd36: reg_rdata = dbg01i;
      6'd37: reg_rdata = caps_w;
      default: reg_rdata = 32'd0;
    endcase
  end

endmodule
