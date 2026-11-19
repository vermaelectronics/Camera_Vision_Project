// -----------------------------------------------------------------------------
// mvdr_top.v  (v3, MVDR window 1.5)  --  2-channel adaptive MVDR beamformer
//
//   SAMPLE DOMAIN  clk  (axi_ad9361 l_clk = 4 x fs)
//     x1,x2 --+--> cov2x2_accum ------------------+  (window sums every 2^(LOG2_N-LOG2_SUB) samples)
//             |                                    |
//             +--> beamformer_2ch --> output_scaler --> out_i / out_q
//                        ^                         |
//   ---------------------|--- mvdr_xdom -----------|-------------------------
//                        |                         v
//   MATHS DOMAIN  mclk  (s_axi_aclk, fixed 100 MHz)
//                 weights <-- mvdr_ctrl_fsm <-- mvdr_weight_calc <-- cov_prep
//                                                  (also -> jammer_est, outside)
//
// The maths (cov_prep, weight calculation with its 52-bit dividers, control)
// runs once per covariance window and has long carry chains; it is clocked by
// the fixed 100 MHz AXI clock so it meets timing at ANY sample rate. Only the
// per-sample logic (accumulators, beamformer, scaler) runs at l_clk.
// mvdr_xdom carries the window sums to mclk and the weight sets back to clk.
// One weight set per window hop; the extra latency is ~1 us (weight calc at
// 100 MHz) + ~0.1 us (two crossings), against a hop of 2^(LOG2_N-LOG2_SUB)
// samples (LOG2_N=13, LOG2_SUB=3: 1024 samples = 18.6 us at 55 MSPS).
// v4 (covariance v3): mean-removed covariance (dc_rm), window peaks.
// v5 (cov_prep v2): noise-referenced diagonal loading, noise-floor readback.
// v6 (ctrl_fsm v2): coherence jammer detector, weight hold + glide.
// v7: receiver IQ-image correction (iq_corr in the sample path, iq_est in the
//     maths domain). The covariance and beamformer now see the corrected
//     stream, DW = DATA_W + IQF + 1 bits (x 2^IQF); covariance / NF_EST values
//     are therefore in units of (raw LSB * 2^IQF)^2. Output scale unchanged.
//     Output latency +3 clocks (9 clocks).
//
// CONFIGURATION
//   sample domain (synchronous to clk) : out_gain_sh, s_cnt_clear, dc_rm
//   maths domain  (synchronous to mclk): mode, freeze, cnt_clear, a1/a2,
//                                        eps_abs, eps_rel_sh, eps_nf_k, fix_w*,
//                                        coh_sh, hold_units, rel_sh
// STATUS: every status output is in the MATHS domain (mclk), including the
//   covariance sums, clip counts and the sample-domain counters n_skew /
//   n_osat / st_skew (sampled once per window by mvdr_xdom).
//   cov_blk is a 1-mclk pulse when a new window arrives in the maths domain.
//
// OUTPUT LATENCY (samples): 18 clocks after the sample's in_valid
//   (iq_corr 5 + beamformer 4 + output_scaler 9).
// v8: output_scaler v3 -- automatic output level control (AOLC).
// v9 (block 1.12): timing closure at l_clk = 250 MHz (ADI's 4.0 ns rx_clk
//     constraint): iq_corr input register + DSP stage (+2), output_scaler v4 (+4),
//     cov2x2_accum v5 (commit sequence +3 clocks, values bit-exact), sample-
//     domain counters start from registered events. No functional change.
// v10 (block 1.13): STAP -- space-time power inversion (stap_lms, 2 antennas x
//     5 taps, NLMS) in parallel with beamformer_2ch; stap_en selects which one
//     feeds output_scaler. One complex weight per antenna nulls a CW jammer
//     but only ~3..20 dB of a wideband barrage, because RX1/RX2 differ across
//     the band (filters, cables, array delay); the 5-tap FIRs equalise that.
//     Output latency +1 clock (select register): MVDR 19 clocks; STAP 34 at
//     LVDS 2R2T (+6 pipeline clocks, +2 samples: the reference is RX1's centre tap).
// v11 (block 1.14): SMI-MVDR STAP (stap_smi, 2 antennas x 8 taps). The FPGA
//     accumulates the 16x16 space-time covariance over 2^KEXP snapshots and
//     publishes it; the ARM solves the loaded MVDR/LCMV weights (power
//     inversion by default, or steered to a direction) and writes them back.
//     Its CTRL.EN takes the output ahead of stap_en. Latency 44 clocks at
//     LVDS 2R2T (+13 pipeline clocks vs MVDR, +3 samples: reference tap 3).
//     2 widely-linear image taps (conj RX1/RX2 at the reference delay) cancel
//     the AD9361 IQ image of a barrage; DC removed at the SMI input.
//     Needs >= 4 clocks per sample (always true at 2R2T, l_clk = 4 x rate).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module mvdr_top #(
    parameter DATA_W = 12,
    parameter LOG2_N = 13,      // window = 2^LOG2_N samples (13 = 8192; was 12 = 4096)
    parameter LOG2_SUB = 3,     // covariance window slides by 2^(LOG2_N-LOG2_SUB) samples (0 = disjoint blocks)
    parameter R_W    = 32,      // scaled covariance width (cov_prep TARGET_W)
    parameter A_W    = 16,
    parameter A_FRAC = 14,
    parameter W_W    = 18,
    parameter W_FRAC = 15,
    parameter OUT_W  = 12,
    parameter IQF    = 2,       // fraction bits added by the IQ-image corrector
    parameter DW     = DATA_W + IQF + 1,        // derived: corrected sample width (+1 guard)
    parameter ACC_W  = 2*DW + LOG2_N + 1,       // derived - do not override
    parameter BF_W   = DW + W_W + 2,            // derived - do not override
    parameter SMI    = 1,                       // v11: 1 = build stap_smi (SMI-MVDR STAP)
    parameter SMI_L  = 16,                      // SMI taps per antenna (8 = the first 1.14)
    parameter SMI_REF = 8,                      // SMI reference tap (RX1 delay)
    parameter STAP_LMS = 0                      // v11: 1 = also build stap_lms (NLMS STAP, ~6k LUT,
                                                //   78 DSP). Off by default: SMI + NLMS do not
                                                //   both fit an XC7Z020 with the rest (54.5k LUT)
)(
    input  wire                     clk,       // sample domain (l_clk)
    input  wire                     rst,       // sample-domain reset, sync to clk
    input  wire                     mclk,      // maths domain (100 MHz AXI clock)
    input  wire                     mrst,      // maths-domain reset, sync to mclk

    // ---- sample stream ----
    input  wire                     in_valid0,
    input  wire                     in_valid1,
    input  wire signed [DATA_W-1:0] x1_i,
    input  wire signed [DATA_W-1:0] x1_q,
    input  wire signed [DATA_W-1:0] x2_i,
    input  wire signed [DATA_W-1:0] x2_q,

    // ---- sample-domain configuration (synchronous to clk) ----
    input  wire        [2:0]        out_gain_sh,   // manual output gain 2^n (6.02 dB steps), 0 = unity
    input  wire                     out_agc_en,    // automatic output level control (AOLC)
    input  wire        [3:0]        out_agc_sh,    // AOLC loop speed
    input  wire        [7:0]        out_target_db, // AOLC output noise floor, dB below FS
    input  wire                     s_cnt_clear,   // clears n_skew / n_osat / st_skew
    input  wire                     dc_rm,         // 1 = covariance with the mean (DC) removed

    // ---- maths-domain configuration (synchronous to mclk) ----
    input  wire [1:0]               mode,
    input  wire                     freeze,
    input  wire                     cnt_clear,
    input  wire signed [A_W-1:0]    a1_re,
    input  wire signed [A_W-1:0]    a1_im,
    input  wire signed [A_W-1:0]    a2_re,
    input  wire signed [A_W-1:0]    a2_im,
    input  wire        [ACC_W-1:0]  eps_abs,
    input  wire        [5:0]        eps_rel_sh,
    input  wire        [7:0]        eps_nf_k,      // noise-floor loading k/8 (8 = x1, 0 = off)
    input  wire        [3:0]        coh_sh,        // jammer detect: coherence > 2^-coh_sh
    input  wire        [15:0]       hold_units,    // null hold after the jammer goes: x64 hops
    input  wire        [3:0]        rel_sh,        // glide time constant 2^rel_sh hops
    input  wire signed [W_W-1:0]    fix_w1_re,
    input  wire signed [W_W-1:0]    fix_w1_im,
    input  wire signed [W_W-1:0]    fix_w2_re,
    input  wire signed [W_W-1:0]    fix_w2_im,
    input  wire                     iq_en,         // IQ-image correction enable
    input  wire                     iq_clear,      // force the image coefficients to 0
    input  wire        [2:0]        iq_mu_sh,      // refinement step 2^-n (default 2)
    // ---- v10: space-time power inversion (stap_lms), sample domain (clk) ----
    input  wire                     stap_en,       // 1 = output from STAP instead of beamformer_2ch
    input  wire        [3:0]        stap_mu_sh,    // NLMS step 2^-n (default 5, min 5)
    input  wire        [4:0]        stap_leak_sh,  // weight leakage 2^-n per sample (default 18, 0 = off)
    input  wire                     stap_clear,    // weights to 0 while set
    input  wire                     stap_hold_off, // 1 = no normalisation hold (pulsed jammers)
    // ---- v11: SMI-MVDR STAP (stap_smi) register window, maths domain (mclk) ----
    //   word addresses 0x100..0x3FF (AXI bytes 0x400..0xFFC); see stap_smi.v
    input  wire                     smi_wr_en,
    input  wire        [9:0]        smi_wr_addr,
    input  wire        [31:0]       smi_wr_data,
    input  wire        [9:0]        smi_rd_addr,
    output wire        [31:0]       smi_rd_data,

    // ---- beamformed output ----
    output wire                     out_valid,
    output wire signed [OUT_W-1:0]  out_i,
    output wire signed [OUT_W-1:0]  out_q,
    output wire                     out_sat,

    // ---- status / debug  (ALL in the maths domain, mclk) ----
    output wire signed [W_W-1:0]    w_cur_w1_re,   // weights last handed to the beamformer
    output wire signed [W_W-1:0]    w_cur_w1_im,
    output wire signed [W_W-1:0]    w_cur_w2_re,
    output wire signed [W_W-1:0]    w_cur_w2_im,
    output wire                     cov_blk,       // 1-mclk pulse per covariance window update
    output wire signed [ACC_W-1:0]  cov_r11,       // latest raw block sums (use to set eps_abs)
    output wire signed [ACC_W-1:0]  cov_r22,
    output wire signed [ACC_W-1:0]  cov_r12_re,    // latest raw cross-covariance sum x1*conj(x2)
    output wire signed [ACC_W-1:0]  cov_r12_im,    //   (same block as cov_r11/cov_r22)
    output wire        [LOG2_N:0]   cov_clip1,     // RX1 samples on an ADC rail in that window
    output wire        [LOG2_N:0]   cov_clip2,     // RX2 samples on an ADC rail in that window
    output wire        [DATA_W-1:0] cov_peak1,     // max(|I|,|Q|) of RX1 in that window
    output wire        [DATA_W-1:0] cov_peak2,     // max(|I|,|Q|) of RX2 in that window
    output wire                     cov_dc_rm,     // that window's r** are mean-removed
    output wire                     wc_busy,
    output wire        [ACC_W-1:0]  nf_est,        // noise-floor estimate, window-sum units
    output wire        [31:0]       out_lvl,       // {gain_l Q8 octaves, out_lg Q4 log2 power}
    output wire        [5:0]        coh_l2,        // ~ -log2(RX1/RX2 coherence), 63 = none
    output wire [31:0]              n_jam,         // runs with the jammer flag
    output wire [31:0]              n_hold,        // runs absorbed by a hold
    output wire                     st_jam,
    output wire signed [17:0]       iq_b1_re,      // IQ image coefficients in use (Q20)
    output wire signed [17:0]       iq_b1_im,
    output wire signed [17:0]       iq_b2_re,
    output wire signed [17:0]       iq_b2_im,
    output wire [31:0]              n_iq_upd,
    output wire                     st_hold,
    output wire                     st_glide,
    output wire [31:0]              n_blocks,
    output wire [31:0]              n_upd,
    output wire [31:0]              n_skip,
    output wire [31:0]              n_sat,
    output wire [31:0]              n_drop,
    output wire [31:0]              n_zero,
    output wire [31:0]              n_skew,        // valid0 != valid1 events
    output wire [31:0]              n_osat,        // output samples that were clamped
    output wire                     st_skew,       // sticky
    output wire                     st_have_w,
    output wire                     st_last_sat,
    output wire                     st_last_skip,
    output wire [1:0]               st_mode,
    // v10: STAP status, maths domain (mclk), refreshed once per window
    output wire                     stap_active,   // STAP is driving the output
    output wire        [5:0]        stap_pl,       // normalisation exponent: input power ~ 2^(pl-2)
    output wire        [31:0]       stap_wnorm,    // sum |w|^2 of the 9 auxiliary taps, Q8.24
    output wire        [31:0]       stap_nwsat     // weight saturation events
);

    // =====================================================================
    //  SAMPLE DOMAIN (clk)
    // =====================================================================
    wire pair_valid = in_valid0 & in_valid1;
    wire skew_now   = in_valid0 ^ in_valid1;

    // ---- IQ-image correction (v7): everything downstream sees the corrected
    //      stream, scaled by 2^IQF (DW bits) ----
    localparam B_W = 18, B_FRAC = 20;
    wire                     xc_valid, xc_clip1, xc_clip2;
    wire signed [DW-1:0]     xc1_i, xc1_q, xc2_i, xc2_q;
    wire                     iq_b_load;
    wire signed [B_W-1:0]    s_b1_re, s_b1_im, s_b2_re, s_b2_im;

    iq_corr #(.DATA_W(DATA_W), .IQF(IQF), .B_W(B_W), .B_FRAC(B_FRAC)) u_iqc (
        .clk(clk), .rst(rst),
        .in_valid(pair_valid),
        .x1_i(x1_i), .x1_q(x1_q), .x2_i(x2_i), .x2_q(x2_q),
        .b_load(iq_b_load),
        .b1_re(s_b1_re), .b1_im(s_b1_im), .b2_re(s_b2_re), .b2_im(s_b2_im),
        .out_valid(xc_valid),
        .y1_i(xc1_i), .y1_q(xc1_q), .y2_i(xc2_i), .y2_q(xc2_q),
        .clip1(xc_clip1), .clip2(xc_clip2));

    wire                     cov_valid;
    wire signed [ACC_W-1:0]  r11, r22, r12_re, r12_im;
    wire signed [ACC_W-1:0]  q11_re, q11_im, q22_re, q22_im;
    wire        [LOG2_N:0]   s_clip1, s_clip2;
    wire        [DW-1:0]     s_peak1, s_peak2;
    wire                     s_dc_done;

    cov2x2_accum #(.DATA_W(DW), .LOG2_N(LOG2_N), .LOG2_SUB(LOG2_SUB),
                   .EXT_CLIP(1), .PK_SH(IQF)) u_cov (
        .clk(clk), .rst(rst),
        .in_valid(xc_valid),
        .x1_i(xc1_i), .x1_q(xc1_q), .x2_i(xc2_i), .x2_q(xc2_q),
        .clip_in1(xc_clip1), .clip_in2(xc_clip2),
        .cfg_dc_rm(dc_rm),
        .cov_valid(cov_valid),
        .r11(r11), .r22(r22), .r12_re(r12_re), .r12_im(r12_im),
        .clip1(s_clip1), .clip2(s_clip2),
        .s1_i(), .s1_q(), .s2_i(), .s2_q(),
        .peak1(s_peak1), .peak2(s_peak2), .dc_rm_done(s_dc_done),
        .q11_re(q11_re), .q11_im(q11_im), .q22_re(q22_re), .q22_im(q22_im),
        .q12_re(), .q12_im());

    wire                     bf_w_load, bf_swap;
    wire signed [W_W-1:0]    bf_w1_re, bf_w1_im, bf_w2_re, bf_w2_im;
    wire                     bf_valid;
    wire signed [BF_W-1:0]   bf_re, bf_im;

    beamformer_2ch #(.DATA_W(DW), .W_W(W_W), .W_FRAC(W_FRAC)) u_bf (
        .clk(clk), .rst(rst),
        .in_valid(xc_valid),
        .x1_i(xc1_i), .x1_q(xc1_q), .x2_i(xc2_i), .x2_q(xc2_q),
        .w_load(bf_w_load),
        .w1_re(bf_w1_re), .w1_im(bf_w1_im), .w2_re(bf_w2_re), .w2_im(bf_w2_im),
        .swap(bf_swap),
        .out_valid(bf_valid), .y_re(bf_re), .y_im(bf_im));

    wire signed [15:0] s_gain_l, s_out_lg;     // output_scaler level readback

    // ---- v10: STAP (space-time power inversion, 2 antennas x 5 taps) ----
    // Runs in parallel with beamformer_2ch on the same corrected samples and
    // produces the same scaling (y = x * 2^W_FRAC for a unity reference), so
    // output_scaler / AOLC take either one unchanged. The covariance, the
    // jammer estimator, NF_EST and the MVDR weight calculation keep running
    // in both modes (the firmware reads them for its reports).
    // stap_en = 0 holds the STAP weights at 0 (cfg_en low).
    localparam ST_L = 5;
    wire                   st_valid;
    wire signed [BF_W-1:0] st_re, st_im;
    wire [5:0]             s_st_pl;
    wire [31:0]            s_st_wnorm, s_st_nwsat;
    reg                    stap_sel = 1'b0;
    always @(posedge clk) stap_sel <= stap_en & (STAP_LMS != 0);

    generate if (STAP_LMS) begin : g_stap
        stap_lms #(.DW(DW), .L(ST_L), .OUT_W(BF_W)) u_stap (
            .clk(clk), .rst(rst),
            .cfg_en(stap_sel), .cfg_mu_sh(stap_mu_sh), .cfg_leak_sh(stap_leak_sh),
            .cfg_clear(stap_clear), .cfg_hold_off(stap_hold_off),
            .in_valid(xc_valid),
            .x1_i(xc1_i), .x1_q(xc1_q), .x2_i(xc2_i), .x2_q(xc2_q),
            .out_valid(st_valid), .y_re(st_re), .y_im(st_im),
            .st_pl(s_st_pl), .st_wnorm(s_st_wnorm), .n_wsat(s_st_nwsat));
    end else begin : g_nostap                  // STAP_ST[31] (mvdr_regs HAVE_STAP=0) tells the firmware
        assign st_valid = 1'b0;  assign st_re = 0;  assign st_im = 0;
        assign s_st_pl = 6'd0;   assign s_st_wnorm = 32'd0;  assign s_st_nwsat = 32'd0;
    end endgenerate

    // ---- v11: SMI-MVDR STAP (2 antennas x 8 taps, sample-matrix inversion) ----
    // The FPGA accumulates R = sum x x^H over 2^KEXP snapshots and publishes
    // it; the ARM solves w = (R + lambda I)^-1 a / (a^H (R + lambda I)^-1 a)
    // and writes w back; the FPGA filters y = w^H x with the same Q15 output
    // scaling as beamformer_2ch / stap_lms. Its own CTRL.EN (smi_act) takes
    // the output, ahead of stap_en. SMI = 0 leaves it out (reads return 0).
    wire                   sm_valid, smi_act;
    wire signed [BF_W-1:0] sm_re, sm_im;
    generate if (SMI) begin : g_smi
        stap_smi #(.DW(DW), .OUT_W(BF_W), .L(SMI_L), .REF(SMI_REF)) u_smi (
            .clk(clk), .rst(rst),
            .in_valid(xc_valid),
            .x1_i(xc1_i), .x1_q(xc1_q), .x2_i(xc2_i), .x2_q(xc2_q),
            .out_valid(sm_valid), .y_re(sm_re), .y_im(sm_im), .active(smi_act),
            .aclk(mclk), .wr_en(smi_wr_en), .wr_addr(smi_wr_addr), .wr_data(smi_wr_data),
            .rd_addr(smi_rd_addr), .rd_data(smi_rd_data));
    end else begin : g_nosmi
        assign sm_valid = 1'b0;  assign sm_re = 0;  assign sm_im = 0;
        assign smi_act = 1'b0;   assign smi_rd_data = 32'd0;
    end endgenerate

    // output select, registered: +1 clock in every mode. Priority SMI >
    // STAP (stap_lms) > beamformer_2ch. Path lengths: beamformer_2ch 4,
    // stap_lms 10, stap_smi 17 clocks (input to y).
    reg                    sc_valid = 1'b0;
    reg signed [BF_W-1:0]  sc_re = 0, sc_im = 0;
    always @(posedge clk) begin
        sc_valid <= smi_act ? sm_valid : stap_sel ? st_valid : bf_valid;
        sc_re    <= smi_act ? sm_re    : stap_sel ? st_re    : bf_re;
        sc_im    <= smi_act ? sm_im    : stap_sel ? st_im    : bf_im;
    end

    output_scaler #(.IN_W(BF_W), .SHIFT(W_FRAC + IQF), .OUT_W(OUT_W), .LSB_PAD(0)) u_sc (
        .clk(clk), .rst(rst),
        .in_valid(sc_valid), .in_re(sc_re), .in_im(sc_im), .gain_sh(out_gain_sh),
        .agc_en(out_agc_en), .agc_sh(out_agc_sh), .target_db(out_target_db),
        .out_valid(out_valid), .out_re(out_i), .out_im(out_q), .sat(out_sat),
        .gain_l(s_gain_l), .out_lg(s_out_lg));
    reg [31:0] s_n_skew  = 32'd0;
    reg [31:0] s_n_osat  = 32'd0;
    reg        s_st_skew = 1'b0;
    // events and clear registered first (250 MHz: counters start from flops)
    reg        e_skew = 1'b0, e_osat = 1'b0, s_clr = 1'b1;
    always @(posedge clk) begin
        e_skew <= skew_now;
        e_osat <= out_valid & out_sat;
        s_clr  <= rst | s_cnt_clear;
    end
    always @(posedge clk) begin
        if (s_clr) begin
            s_n_skew  <= 32'd0;
            s_n_osat  <= 32'd0;
            s_st_skew <= 1'b0;
        end else begin
            if (e_skew) s_n_skew  <= s_n_skew + 32'd1;
            if (e_osat) s_n_osat  <= s_n_osat + 32'd1;
            if (e_skew) s_st_skew <= 1'b1;
        end
    end

    // =====================================================================
    //  CROSSING
    // =====================================================================
    wire signed [ACC_W-1:0]  m_r11, m_r22, m_r12_re, m_r12_im;
    wire                     m_cov_valid;
    wire                     fsm_w_load;
    wire signed [W_W-1:0]    fsm_w1_re, fsm_w1_im, fsm_w2_re, fsm_w2_im;

    // Resend the current weight set once per window as well: if the sample
    // domain was reset (axi_ad9361 resets it on a sample-rate change) the
    // beamformer falls back to [1,0]; the resend restores fixed/adaptive
    // weights within one window. Re-loading identical weights is glitch-free.
    reg m_resync = 1'b0;
    always @(posedge mclk) m_resync <= m_cov_valid;

    wire                     m_iq_upd;
    wire signed [B_W-1:0]    m_b1_re, m_b1_im, m_b2_re, m_b2_im;
    wire signed [ACC_W-1:0]  m_q11_re, m_q11_im, m_q22_re, m_q22_im;

    mvdr_xdom #(.ACC_W(ACC_W), .CLIP_W(LOG2_N+1), .W_W(W_W), .PK_W(DATA_W), .B_W(B_W)) u_x (
        .clk(clk),
        .cov_valid(cov_valid),
        .r11(r11), .r22(r22), .r12_re(r12_re), .r12_im(r12_im),
        .clip1(s_clip1), .clip2(s_clip2),
        .peak1(s_peak1[DATA_W-1:0]), .peak2(s_peak2[DATA_W-1:0]), .dc_rm_done(s_dc_done),
        .q11_re(q11_re), .q11_im(q11_im), .q22_re(q22_re), .q22_im(q22_im),
        .s_out_lvl({s_gain_l, s_out_lg}),
        .s_n_skew(s_n_skew), .s_n_osat(s_n_osat), .s_st_skew(s_st_skew),
        .s_stap({stap_sel, s_st_pl, s_st_nwsat, s_st_wnorm}),
        .bf_w_load(bf_w_load), .bf_swap(bf_swap),
        .bf_w1_re(bf_w1_re), .bf_w1_im(bf_w1_im), .bf_w2_re(bf_w2_re), .bf_w2_im(bf_w2_im),
        .iq_b1_re(s_b1_re), .iq_b1_im(s_b1_im), .iq_b2_re(s_b2_re), .iq_b2_im(s_b2_im),
        .mclk(mclk),
        .m_cov_valid(m_cov_valid),
        .m_r11(m_r11), .m_r22(m_r22), .m_r12_re(m_r12_re), .m_r12_im(m_r12_im),
        .m_clip1(cov_clip1), .m_clip2(cov_clip2),
        .m_peak1(cov_peak1), .m_peak2(cov_peak2), .m_dc_rm_done(cov_dc_rm),
        .m_q11_re(m_q11_re), .m_q11_im(m_q11_im), .m_q22_re(m_q22_re), .m_q22_im(m_q22_im),
        .m_out_lvl(out_lvl),
        .m_n_skew(n_skew), .m_n_osat(n_osat), .m_st_skew(st_skew),
        .m_stap({stap_active, stap_pl, stap_nwsat, stap_wnorm}),
        .m_w_load(fsm_w_load | m_resync | m_iq_upd),
        .m_w1_re(fsm_w1_re), .m_w1_im(fsm_w1_im), .m_w2_re(fsm_w2_re), .m_w2_im(fsm_w2_im),
        .m_b1_re(m_b1_re), .m_b1_im(m_b1_im), .m_b2_re(m_b2_re), .m_b2_im(m_b2_im),
        .m_w_busy());

    assign iq_b_load  = bf_w_load;            // b travels with every weight set

    assign cov_blk    = m_cov_valid;
    assign cov_r11    = m_r11;
    assign cov_r22    = m_r22;
    assign cov_r12_re = m_r12_re;
    assign cov_r12_im = m_r12_im;

    // =====================================================================
    //  MATHS DOMAIN (mclk)
    // =====================================================================
    wire                     prep_valid, prep_zero, prep_jam, prep_strong;
    wire signed [R_W-1:0]    p11, p22, p12_re, p12_im;
    wire        [7:0]        prep_shift;

    cov_prep #(.ACC_W(ACC_W), .TARGET_W(R_W)) u_prep (
        .clk(mclk), .rst(mrst),
        .in_valid(m_cov_valid),
        .r11(m_r11), .r22(m_r22), .r12_re(m_r12_re), .r12_im(m_r12_im),
        .eps_abs(eps_abs), .eps_rel_sh(eps_rel_sh), .eps_nf_k(eps_nf_k),
        .out_valid(prep_valid),
        .o11(p11), .o22(p22), .o12_re(p12_re), .o12_im(p12_im),
        .shift(prep_shift), .r_zero(prep_zero), .nf_est(nf_est), .busy(),
        .coh_sh(coh_sh), .coh_jam(prep_jam), .coh_l2(coh_l2), .strong_jam(prep_strong));

    wire                     wc_start, wc_valid, wc_skip, wc_sat, wc_dropped;
    wire signed [W_W-1:0]    wc_w1_re, wc_w1_im, wc_w2_re, wc_w2_im;

    mvdr_ctrl_fsm #(.W_W(W_W), .W_FRAC(W_FRAC)) u_fsm (
        .clk(mclk), .rst(mrst),
        .mode(mode), .freeze(freeze),
        .fix_w1_re(fix_w1_re), .fix_w1_im(fix_w1_im),
        .fix_w2_re(fix_w2_re), .fix_w2_im(fix_w2_im),
        .cnt_clear(cnt_clear), .hold_units(hold_units), .rel_sh(rel_sh),
        .prep_valid(prep_valid), .prep_zero(prep_zero), .prep_jam(prep_jam),
        .wc_start(wc_start),
        .wc_valid(wc_valid), .wc_skip(wc_skip), .wc_sat(wc_sat), .wc_dropped(wc_dropped),
        .wc_w1_re(wc_w1_re), .wc_w1_im(wc_w1_im), .wc_w2_re(wc_w2_re), .wc_w2_im(wc_w2_im),
        .bf_w_load(fsm_w_load), .bf_swap(),
        .bf_w1_re(fsm_w1_re), .bf_w1_im(fsm_w1_im), .bf_w2_re(fsm_w2_re), .bf_w2_im(fsm_w2_im),
        .n_blocks(n_blocks), .n_upd(n_upd), .n_skip(n_skip), .n_sat(n_sat),
        .n_drop(n_drop), .n_zero(n_zero), .n_jam(n_jam), .n_hold(n_hold),
        .st_have_w(st_have_w), .st_last_sat(st_last_sat), .st_last_skip(st_last_skip),
        .st_jam(st_jam), .st_hold(st_hold), .st_glide(st_glide),
        .st_mode(st_mode));

    iq_est #(.ACC_W(ACC_W), .B_W(B_W), .B_FRAC(B_FRAC), .NSUB(1 << LOG2_SUB)) u_iqe (
        .clk(mclk), .rst(mrst),
        .cfg_en(iq_en), .cfg_clear(iq_clear), .cfg_mu_sh(iq_mu_sh),
        .trig(prep_valid), .jam(prep_strong),
        .r11(m_r11), .r22(m_r22),
        .q11_re(m_q11_re), .q11_im(m_q11_im), .q22_re(m_q22_re), .q22_im(m_q22_im),
        .b_upd(m_iq_upd),
        .b1_re(m_b1_re), .b1_im(m_b1_im), .b2_re(m_b2_re), .b2_im(m_b2_im),
        .n_upd(n_iq_upd), .n_drop(), .busy());

    assign iq_b1_re = m_b1_re;  assign iq_b1_im = m_b1_im;
    assign iq_b2_re = m_b2_re;  assign iq_b2_im = m_b2_im;

    assign w_cur_w1_re = fsm_w1_re;
    assign w_cur_w1_im = fsm_w1_im;
    assign w_cur_w2_re = fsm_w2_re;
    assign w_cur_w2_im = fsm_w2_im;

    mvdr_weight_calc #(.R_W(R_W), .A_W(A_W), .A_FRAC(A_FRAC), .W_W(W_W), .W_FRAC(W_FRAC)) u_wc (
        .clk(mclk), .rst(mrst),
        .start(wc_start),
        .r11(p11), .r22(p22), .r12_re(p12_re), .r12_im(p12_im),
        .a1_re(a1_re), .a1_im(a1_im), .a2_re(a2_re), .a2_im(a2_im),
        .busy(wc_busy), .w_valid(wc_valid), .w_skip(wc_skip), .w_sat(wc_sat), .dropped(wc_dropped),
        .w1_re(wc_w1_re), .w1_im(wc_w1_im), .w2_re(wc_w2_re), .w2_im(wc_w2_im));

endmodule
