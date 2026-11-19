// -----------------------------------------------------------------------------
// mvdr_xdom.v  --  clock-domain bridge between the MVDR sample datapath and the
//                  MVDR maths (MVDR window v1.5)
//
//   sample domain (clk = axi_ad9361 l_clk, 4 x fs: 200 MHz at 50 MSPS)
//       cov2x2_accum, beamformer_2ch, output_scaler, elastic FIFO
//   maths domain  (mclk = s_axi_aclk, fixed 100 MHz, independent of fs)
//       cov_prep, mvdr_ctrl_fsm, mvdr_weight_calc/real_divider, jammer_est
//
// Why: the maths runs once per covariance window (every 1024 samples, i.e.
// >= 1600 mclk cycles up to 61.44 MSPS) but has long carry chains (52-bit
// divide, normalisation, CORDIC). Clocked at l_clk it failed timing at
// 200 MHz; on the fixed 100 MHz clock it has ~3 ns of margin, at any fs.
//
// 1) COVARIANCE  clk -> mclk   (toggle, no ack needed)
//    On every cov_valid the window sums, clip counts, window peaks, the
//    dc_rm flag and the sample-domain status counters are copied into c_hold
//    and c_tog flips. c_hold then
//    stays stable until the next window (>= 2048 clk cycles), far longer
//    than the 2-flop synchroniser + copy (~4 mclk). m_cov_valid is a 1-mclk
//    pulse with the copied data, which then holds until the next one.
//
// 2) WEIGHTS  mclk -> clk   (toggle + ack, latest-value-wins)
//    m_w_load presents a new weight set. If the previous set is still in
//    flight (ack not back), the new set is parked and sent as soon as the
//    ack returns; a newer set overwrites a parked one. On the clk side a
//    The IQ image coefficients b (iq_est -> iq_corr) travel in the same bundle
//    and are valid with bf_w_load (re-sending an unchanged b is harmless).
//    toggle edge copies the set and issues bf_w_load, then bf_swap one clock
//    later -- exactly the beamformer_2ch load/swap protocol, so the whole
//    set changes on one sample edge. w_hold is stable from the toggle until
//    the ack returns, so the copy can never see a mixed set.
//
//    The synchronisers and ack have no reset, so a reset of either domain
//    (e.g. axi_ad9361 resetting during a sample-rate change) can never leave
//    the handshake stuck.
//
// v3 (STAP, block 1.13): the STAP status (normalisation exponent, weight
//    norm, saturation count, active flag) rides in the covariance bundle.
// Latency added: about 3 clk + 3 mclk each way (~50 ns at 200/100 MHz).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module mvdr_xdom #(
    parameter ACC_W  = 37,
    parameter CLIP_W = 13,
    parameter W_W    = 18,
    parameter PK_W   = 12,         // window peak width (= DATA_W)
    parameter B_W    = 18          // IQ image coefficient width (iq_est / iq_corr)
)(
    // ---------------- sample domain ----------------
    input  wire                     clk,
    input  wire                     cov_valid,
    input  wire signed [ACC_W-1:0]  r11,
    input  wire signed [ACC_W-1:0]  r22,
    input  wire signed [ACC_W-1:0]  r12_re,
    input  wire signed [ACC_W-1:0]  r12_im,
    input  wire        [CLIP_W-1:0] clip1,
    input  wire        [CLIP_W-1:0] clip2,
    input  wire        [PK_W-1:0]   peak1,
    input  wire        [PK_W-1:0]   peak2,
    input  wire                     dc_rm_done,
    input  wire signed [ACC_W-1:0]  q11_re,         // pseudo-covariance sums (iq_est)
    input  wire signed [ACC_W-1:0]  q11_im,
    input  wire signed [ACC_W-1:0]  q22_re,
    input  wire signed [ACC_W-1:0]  q22_im,
    input  wire        [31:0]       s_out_lvl,      // output_scaler {gain_l, out_lg}
    input  wire        [31:0]       s_n_skew,
    input  wire        [31:0]       s_n_osat,
    input  wire                     s_st_skew,
    input  wire        [70:0]       s_stap,         // v3: {active, pl[5:0], n_wsat, wnorm}

    output reg                      bf_w_load = 1'b0,
    output reg                      bf_swap   = 1'b0,
    output reg  signed [W_W-1:0]    bf_w1_re  = 0,
    output reg  signed [W_W-1:0]    bf_w1_im  = 0,
    output reg  signed [W_W-1:0]    bf_w2_re  = 0,
    output reg  signed [W_W-1:0]    bf_w2_im  = 0,
    output reg  signed [B_W-1:0]    iq_b1_re  = 0,  // valid with bf_w_load
    output reg  signed [B_W-1:0]    iq_b1_im  = 0,
    output reg  signed [B_W-1:0]    iq_b2_re  = 0,
    output reg  signed [B_W-1:0]    iq_b2_im  = 0,

    // ---------------- maths domain ----------------
    input  wire                     mclk,
    output reg                      m_cov_valid = 1'b0,
    output reg  signed [ACC_W-1:0]  m_r11    = 0,
    output reg  signed [ACC_W-1:0]  m_r22    = 0,
    output reg  signed [ACC_W-1:0]  m_r12_re = 0,
    output reg  signed [ACC_W-1:0]  m_r12_im = 0,
    output reg         [CLIP_W-1:0] m_clip1  = 0,
    output reg         [CLIP_W-1:0] m_clip2  = 0,
    output reg         [PK_W-1:0]   m_peak1  = 0,
    output reg         [PK_W-1:0]   m_peak2  = 0,
    output reg                      m_dc_rm_done = 1'b0,
    output reg  signed [ACC_W-1:0]  m_q11_re = 0,
    output reg  signed [ACC_W-1:0]  m_q11_im = 0,
    output reg  signed [ACC_W-1:0]  m_q22_re = 0,
    output reg  signed [ACC_W-1:0]  m_q22_im = 0,
    output reg         [31:0]       m_out_lvl = 0,
    output reg         [31:0]       m_n_skew = 0,
    output reg         [31:0]       m_n_osat = 0,
    output reg                      m_st_skew = 1'b0,
    output reg         [70:0]       m_stap    = 71'd0,

    input  wire                     m_w_load,       // send this weight set
    input  wire signed [W_W-1:0]    m_w1_re,
    input  wire signed [W_W-1:0]    m_w1_im,
    input  wire signed [W_W-1:0]    m_w2_re,
    input  wire signed [W_W-1:0]    m_w2_im,
    input  wire signed [B_W-1:0]    m_b1_re,        // sent with every weight set
    input  wire signed [B_W-1:0]    m_b1_im,
    input  wire signed [B_W-1:0]    m_b2_re,
    input  wire signed [B_W-1:0]    m_b2_im,
    output wire                     m_w_busy        // a set is in flight (debug)
);

    localparam CB_W = 8*ACC_W + 2*CLIP_W + 2*PK_W + 1 + 32 + 32 + 32 + 1 + 71;
    localparam WB_W = 4*W_W + 4*B_W;

    // =====================================================================
    //  1) covariance bundle, clk -> mclk
    // =====================================================================
    reg  [CB_W-1:0] c_hold = {CB_W{1'b0}};
    reg             c_tog  = 1'b0;

    always @(posedge clk) begin
        if (cov_valid) begin
            c_hold <= {s_stap, s_out_lvl, q22_im, q22_re, q11_im, q11_re,
                       s_st_skew, s_n_osat, s_n_skew, dc_rm_done, peak2, peak1,
                       clip2, clip1, r12_im, r12_re, r22, r11};
            c_tog  <= ~c_tog;
        end
    end

    (* ASYNC_REG = "TRUE" *) reg c_m = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg c_s = 1'b0;
    reg c_p = 1'b0;

    always @(posedge mclk) begin
        c_m <= c_tog;
        c_s <= c_m;
        c_p <= c_s;
        m_cov_valid <= (c_s != c_p);
        if (c_s != c_p)
            {m_stap, m_out_lvl, m_q22_im, m_q22_re, m_q11_im, m_q11_re,
             m_st_skew, m_n_osat, m_n_skew, m_dc_rm_done, m_peak2, m_peak1,
             m_clip2, m_clip1, m_r12_im, m_r12_re, m_r22, m_r11} <= c_hold;
    end

    // =====================================================================
    //  2) weight set, mclk -> clk
    // =====================================================================
    reg  [WB_W-1:0] w_hold = {WB_W{1'b0}};
    reg  [WB_W-1:0] w_next = {WB_W{1'b0}};
    reg             w_tog  = 1'b0;
    reg             w_pend = 1'b0;
    reg             w_ack  = 1'b0;                    // clk domain

    (* ASYNC_REG = "TRUE" *) reg wa_m = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg wa_s = 1'b0;

    wire            busy  = (w_tog != wa_s);
    wire [WB_W-1:0] w_new = {m_b2_im, m_b2_re, m_b1_im, m_b1_re,
                             m_w2_im, m_w2_re, m_w1_im, m_w1_re};
    assign m_w_busy = busy | w_pend;

    // v2: as soon as the previous set has been acknowledged, send the newest one
    // (a set arriving on this clock beats a parked one). v1 sent a parked set
    // only on a clock WITHOUT a new load, so a load held high would starve it.
    always @(posedge mclk) begin
        wa_m <= w_ack;
        wa_s <= wa_m;
        if (!busy && (m_w_load || w_pend)) begin
            w_hold <= m_w_load ? w_new : w_next;
            w_tog  <= ~w_tog;
            w_pend <= 1'b0;
        end else if (m_w_load) begin
            w_next <= w_new;                          // latest set wins
            w_pend <= 1'b1;
        end
    end

    (* ASYNC_REG = "TRUE" *) reg wt_m = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg wt_s = 1'b0;
    reg wt_p = 1'b0;

    always @(posedge clk) begin
        wt_m      <= w_tog;
        wt_s      <= wt_m;
        wt_p      <= wt_s;
        bf_w_load <= 1'b0;
        bf_swap   <= bf_w_load;                       // swap follows load by one clock
        if (wt_s != wt_p) begin
            {iq_b2_im, iq_b2_re, iq_b1_im, iq_b1_re,
             bf_w2_im, bf_w2_re, bf_w1_im, bf_w1_re} <= w_hold;
            bf_w_load <= 1'b1;
            w_ack     <= wt_s;
        end
    end

endmodule
