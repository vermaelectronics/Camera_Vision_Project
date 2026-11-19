// -----------------------------------------------------------------------------
// cov2x2_accum.v  --  MVDR step 1: 2-channel sample-covariance estimator
//                     v3: sliding window + exact MEAN (DC) REMOVAL + window peak
//                     v4: + PSEUDO-COVARIANCE sums (x x^T) for IQ-image estimation,
//                         external clip flags (input already IQ-corrected)
//                     v5: restructured for l_clk = 250 MHz (4.0 ns). Same published
//                         values (bit-exact); the commit sequence is 2 clocks longer.
//
// For complex samples x1 = x1_i + j*x1_q (RX1) and x2 = x2_i + j*x2_q (RX2),
// over a window of N = 2^LOG2_N valid sample pairs:
//
//     raw sums       Q11 = sum |x1|^2         Q22 = sum |x2|^2
//                    Q12 = sum x1 * conj(x2)
//                    S1  = sum x1             S2  = sum x2
//
//     published      r11 = Q11 - |S1|^2 / N                  (cfg_dc_rm = 1)
//                    r22 = Q22 - |S2|^2 / N
//                    r12 = Q12 - S1 * conj(S2) / N
//                    i.e. N x the CENTRED sample covariance (mean removed).
//                    With cfg_dc_rm = 0 the raw Q sums are published (= v2).
//
//     r12_re = sum (x1_i*x2_i + x1_q*x2_q)    r12_im = sum (x1_q*x2_i - x1_i*x2_q)
//
// WHY MEAN REMOVAL (new in v3)
//   Each AD9361 channel carries a residual DC / LO-leakage term. Two constant
//   terms add a rank-1, perfectly coherent component d*d^H to R: it grows like
//   N in r12, while the noise part grows only like sqrt(N). With two elements
//   the array has ONE degree of freedom, and with no jammer present the power-
//   inversion/MVDR solution spends it on that DC "interferer". Model
//   (Docs: model/dc_effect.py), no jammer, N = 8192:
//         DC vs noise   |w2| raw   output noise rise   |w2| centred
//            -6 dB        0.20          +0.2 dB            0.01
//             0 dB        0.50          +0.9 dB            0.01
//            +6 dB        0.80          +2.1 dB            0.01
//   i.e. up to 2.5 dB of satellite C/N0 lost for nothing. The centred estimate
//   removes it exactly (per window, integer arithmetic, rounding 0.5 LSB).
//   Side effect: a jammer EXACTLY at the LO frequency (0 Hz baseband) is also
//   removed from R and will not be nulled. The AD9361 DC-tracking loops already
//   suppress 0 Hz, so this costs nothing in practice; set cfg_dc_rm = 0 to
//   test such a case.
//
// SLIDING WINDOW (as v2)
//   Sums are accumulated over sub-blocks of N/2^LOG2_SUB samples; the window is
//   the last 2^LOG2_SUB sub-blocks, maintained exactly as W += newest - oldest.
//   A new window is published every sub-block (the "hop"):
//       LOG2_N = 13, LOG2_SUB = 3 : window 8192, hop 1024 = 18.6 us @ 55 MSPS
//       (v2 used LOG2_SUB = 2: hop 2048 = 37 us)
//   Estimation noise, null depth and the SCALE of all outputs depend only on
//   N, not on LOG2_SUB: halving the hop halves the reaction time to a new
//   jammer for free (one more history entry per channel).
//   LOG2_SUB = 0 gives disjoint blocks (v1 behaviour).
//
// PSEUDO-COVARIANCE (v4), same window and same mean removal:
//     q11 = sum x1^2 - S1^2/N      q22 = sum x2^2 - S2^2/N      q12 = sum x1 x2 - S1 S2/N
//   A proper (circular) signal -- noise, GNSS, a CW tone away from 0 Hz -- has
//   E[x^2] = 0. A receiver IQ imbalance x = s + g s* makes q_kk = 2 g * r_kk: iq_est
//   uses this to estimate and remove the image.
//   Costs 2 more sample multipliers (x_i * x_q per channel; the x1 x2 products are
//   shared) and 2 more products in the commit sequence.
//
// EXTRA OUTPUTS (same window as r**)
//   s1_*, s2_*   : window sums S1, S2 (mean = S/N; DC level diagnostics)
//   clip1/clip2  : samples whose I or Q sat on an ADC rail (+2^(W-1)-1 / -2^(W-1)),
//                  or, with EXT_CLIP = 1, samples whose clip_in flag was set.
//                  A clipped jammer is non-linear and cannot be nulled: non-zero
//                  means "reduce RX gain".
//   peak1/peak2  : max(|I|,|Q|) over the window, per channel, >> PK_SH. Headroom to full
//                  scale = 20*log10(2^(DATA_W-1)/peak) dB. Lets firmware set the
//                  RX gain BEFORE clipping starts (clip counts only tell you after).
//   dc_rm_done   : the published r** are centred (cfg_dc_rm was 1 for this window)
//
// TIMING
//   Fully pipelined, one sample per clock maximum. After the last sample of a
//   sub-block, the window update + mean correction take PH_PUB+5 clocks
//   (~25 clocks), then cov_valid pulses for one clock; outputs hold until the
//   next pulse.
//   v5 (250 MHz): every wide clear/enable is a flip-flop output driving the
//   flops' R / CE pins directly (no decode after the fan-out); datapath flops
//   carry no reset; the history is one distributed RAM per term (8 deep);
//   the commit sequence is a one-hot shift register; the mean-correction
//   multiplier is two DSP products (25x18, 25x8) with M and P registers and
//   one fabric add; rounding by N is done as ((c >> (L-1)) + 1) >> 1; the
//   positive clamp of r11/r22 and the peak scan have their own stages. The first cov_valid comes only once a full window has been
//   seen, so a partially filled window is never published.
//   Mean correction: 8 products of SX_W x SX_W bits through ONE pipelined
//   multiplier (2 DSP48 for SX_W = 25), one product per clock.
//   Requirement: LOG2_N - LOG2_SUB >= 5 (a sub-block must outlast the ~20-clock
//   commit sequence; at 4 clocks/sample it outlasts it by 200x).
//   Widest single-cycle operations: one 2*SX_W+1-bit add, one ACC_W-bit add.
//
// WIDTHS
//   |I|,|Q| <= 2^(DATA_W-1): one |x|^2 term <= 2^(2*DATA_W-1), a window sum
//   <= 2^(2*DATA_W-1+LOG2_N)  -> ACC_W = 2*DATA_W + LOG2_N + 1 (signed).
//   |S| <= 2^(DATA_W-1+LOG2_N) -> SX_W = DATA_W + LOG2_N (signed).
//   DATA_W = 12, LOG2_N = 13 -> ACC_W = 38, SX_W = 25.
//
// History storage is written without reset (distributed-RAM friendly); entries
// not yet written since reset are never used (the `fill` count guards them).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module cov2x2_accum #(
    parameter DATA_W   = 12,
    parameter LOG2_N   = 13,                    // window = 2^LOG2_N samples
    parameter LOG2_SUB = 3,                     // 2^LOG2_SUB sub-blocks per window (0 = disjoint)
    parameter EXT_CLIP = 0,                     // 1: clip counts from clip_in1/2, not rails
    parameter PK_SH    = 0,                     // peak output = max(|I|,|Q|) >> PK_SH
    parameter ACC_W    = 2*DATA_W + LOG2_N + 1, // derived
    parameter CLIP_W   = LOG2_N + 1,            // derived: 0 .. 2^LOG2_N
    parameter SX_W     = DATA_W + LOG2_N        // derived: window sum of x
)(
    input  wire                      clk,
    input  wire                      rst,        // synchronous, active high

    input  wire                      in_valid,
    input  wire signed [DATA_W-1:0]  x1_i,
    input  wire signed [DATA_W-1:0]  x1_q,
    input  wire signed [DATA_W-1:0]  x2_i,
    input  wire signed [DATA_W-1:0]  x2_q,
    input  wire                      clip_in1,   // used when EXT_CLIP = 1
    input  wire                      clip_in2,

    input  wire                      cfg_dc_rm,  // 1 = publish the mean-removed covariance

    output reg                       cov_valid,  // 1-clk pulse: new window
    output reg  signed [ACC_W-1:0]   r11,
    output reg  signed [ACC_W-1:0]   r22,
    output reg  signed [ACC_W-1:0]   r12_re,
    output reg  signed [ACC_W-1:0]   r12_im,
    output reg         [CLIP_W-1:0]  clip1,
    output reg         [CLIP_W-1:0]  clip2,
    output reg  signed [SX_W-1:0]    s1_i,
    output reg  signed [SX_W-1:0]    s1_q,
    output reg  signed [SX_W-1:0]    s2_i,
    output reg  signed [SX_W-1:0]    s2_q,
    output reg         [DATA_W-1:0]  peak1,
    output reg         [DATA_W-1:0]  peak2,
    output reg                       dc_rm_done,
    output reg  signed [ACC_W-1:0]   q11_re,     // pseudo-covariance sum x1^2 (centred)
    output reg  signed [ACC_W-1:0]   q11_im,
    output reg  signed [ACC_W-1:0]   q22_re,     // sum x2^2
    output reg  signed [ACC_W-1:0]   q22_im,
    output reg  signed [ACC_W-1:0]   q12_re,     // sum x1 x2
    output reg  signed [ACC_W-1:0]   q12_im
);

    localparam PW     = 2*DATA_W;               // one product
    localparam SW     = 2*DATA_W + 1;           // sum of two products
    localparam LOG2_S = LOG2_N - LOG2_SUB;      // samples per sub-block = 2^LOG2_S
    localparam NSUB   = 1 << LOG2_SUB;
    localparam PTR_W  = (LOG2_SUB > 0) ? LOG2_SUB : 1;
    localparam CS_W   = LOG2_S + 1;             // clip count width per sub-block
    localparam MP_W   = 2*SX_W;                 // one mean-correction product
    localparam MC_W   = 2*SX_W + 1;             // sum of two of them
    localparam HB_W   = SX_W - 17;              // upper part of the second operand

    // commit sequence (one-hot pv[k] = "phase k"):
    //   1..10 issue product k, 5..14 collect it, 15..16 sums (two halves),
    //   17 /N, 18 subtract, 19 clamp, 1..NSUB+3 peak scan,
    //   PH_PUB-1 publish decision, PH_PUB publish
    localparam PH_PUB = (NSUB + 5 > 21) ? (NSUB + 5) : 21;
    localparam CH     = (MC_W + 1) / 2;         // low half of a correction sum

    localparam signed [DATA_W-1:0] RAIL_P = {1'b0, {(DATA_W-1){1'b1}}};
    localparam signed [DATA_W-1:0] RAIL_N = {1'b1, {(DATA_W-1){1'b0}}};

    // synthesis-time sanity
    initial begin
        if (LOG2_S < 5) begin
            $display("cov2x2_accum: LOG2_N - LOG2_SUB must be >= 5");
            $finish;
        end
    end

    // ======================= stage 0: register, clip, |.| ====================
    reg                     v0;
    reg signed [DATA_W-1:0] a_i, a_q, b_i, b_q;
    reg                     c1_0, c2_0;
    reg        [DATA_W-1:0] ab_ai, ab_aq, ab_bi, ab_bq;     // |x|, unsigned (2^(W-1) fits)

    always @(posedge clk) begin
        if (rst) v0 <= 1'b0; else v0 <= in_valid;
    end
    always @(posedge clk) begin
        if (in_valid) begin
            a_i <= x1_i;  a_q <= x1_q;
            b_i <= x2_i;  b_q <= x2_q;
            c1_0 <= (EXT_CLIP != 0) ? clip_in1 :
                    ((x1_i == RAIL_P) || (x1_i == RAIL_N) ||
                     (x1_q == RAIL_P) || (x1_q == RAIL_N));
            c2_0 <= (EXT_CLIP != 0) ? clip_in2 :
                    ((x2_i == RAIL_P) || (x2_i == RAIL_N) ||
                     (x2_q == RAIL_P) || (x2_q == RAIL_N));
            ab_ai <= x1_i[DATA_W-1] ? (~x1_i + 1'b1) : x1_i;
            ab_aq <= x1_q[DATA_W-1] ? (~x1_q + 1'b1) : x1_q;
            ab_bi <= x2_i[DATA_W-1] ? (~x2_i + 1'b1) : x2_i;
            ab_bq <= x2_q[DATA_W-1] ? (~x2_q + 1'b1) : x2_q;
        end
    end

    // ======================= stage 1: multipliers, peak per sample, count =====
    reg                     v1, l1;             // l1: this is the last sample of a sub-block
    reg                     c1_1, c2_1;
    reg signed [PW-1:0]     m_aa_i, m_aa_q, m_bb_i, m_bb_q;
    reg signed [PW-1:0]     m_ab_ii, m_ab_qq, m_ab_qi, m_ab_iq;
    reg signed [PW-1:0]     m_aa_iq, m_bb_iq;
    reg signed [DATA_W-1:0] a_i1, a_q1, b_i1, b_q1;
    reg        [DATA_W-1:0] pk1_1, pk2_1;
    reg [LOG2_S-1:0]        cnt;

    always @(posedge clk) begin
        if (rst) begin
            v1 <= 1'b0;  l1 <= 1'b0;  cnt <= {LOG2_S{1'b0}};
        end else begin
            v1 <= v0;
            l1 <= v0 && (cnt == {LOG2_S{1'b1}});
            if (v0) cnt <= cnt + 1'b1;
        end
    end
    always @(posedge clk) begin
        c1_1    <= c1_0;
        c2_1    <= c2_0;
        m_aa_i  <= a_i * a_i;
        m_aa_q  <= a_q * a_q;
        m_bb_i  <= b_i * b_i;
        m_bb_q  <= b_q * b_q;
        m_ab_ii <= a_i * b_i;
        m_ab_qq <= a_q * b_q;
        m_ab_qi <= a_q * b_i;
        m_ab_iq <= a_i * b_q;
        m_aa_iq <= a_i * a_q;
        m_bb_iq <= b_i * b_q;
        a_i1 <= a_i;  a_q1 <= a_q;  b_i1 <= b_i;  b_q1 <= b_q;
        pk1_1 <= (ab_ai > ab_aq) ? ab_ai : ab_aq;
        pk2_1 <= (ab_bi > ab_bq) ? ab_bi : ab_bq;
    end

    // ======================= stage 2: pair sums ==============================
    (* max_fanout = 64 *) reg v2;               // accumulator enable
    (* max_fanout = 64 *) reg l2;               // last sample: publish the sub-block
    (* max_fanout = 64 *) reg z2;               // clear the accumulators (last sample or reset)
    reg                     c1_2, c2_2;
    reg signed [SW-1:0]     p11, p22, p12_re, p12_im;
    reg signed [SW-1:0]     u11r, u11i, u22r, u22i, u12r, u12i;    // pseudo terms
    reg signed [DATA_W-1:0] a_i2, a_q2, b_i2, b_q2;
    reg        [DATA_W-1:0] pk1_2, pk2_2;

    always @(posedge clk) begin
        if (rst) begin v2 <= 1'b0;  l2 <= 1'b0; end
        else     begin v2 <= v1;    l2 <= l1;   end
        z2 <= l1 | rst;
    end
    always @(posedge clk) begin
        c1_2   <= c1_1;
        c2_2   <= c2_1;
        p11    <= m_aa_i  + m_aa_q;
        p22    <= m_bb_i  + m_bb_q;
        p12_re <= m_ab_ii + m_ab_qq;
        p12_im <= m_ab_qi - m_ab_iq;
        u11r   <= m_aa_i  - m_aa_q;
        u11i   <= {m_aa_iq, 1'b0};
        u22r   <= m_bb_i  - m_bb_q;
        u22i   <= {m_bb_iq, 1'b0};
        u12r   <= m_ab_ii - m_ab_qq;
        u12i   <= m_ab_iq + m_ab_qi;
        a_i2 <= a_i1;  a_q2 <= a_q1;  b_i2 <= b_i1;  b_q2 <= b_q1;
        pk1_2 <= pk1_1;  pk2_2 <= pk2_1;
    end

    // ======================= stage 3: sub-block accumulate ===================
    //   acc <= 0 on z2 (R pin), else acc + new on v2 (CE pin); the sub-block
    //   result sb = acc + new is captured on l2 (CE pin).
    reg signed [ACC_W-1:0]  acc11, acc22, acc12_re, acc12_im;
    reg signed [SX_W-1:0]   accx1i, accx1q, accx2i, accx2q;
    reg        [CS_W-1:0]   accc1, accc2;
    reg        [DATA_W-1:0] accp1, accp2;
    reg signed [ACC_W-1:0]  accu0, accu1, accu2, accu3, accu4, accu5;

    reg signed [ACC_W-1:0]  sb11, sb22, sb12_re, sb12_im;
    reg signed [SX_W-1:0]   sbx1i, sbx1q, sbx2i, sbx2q;
    reg        [CS_W-1:0]   sbc1, sbc2;
    reg        [DATA_W-1:0] sbp1, sbp2;
    reg signed [ACC_W-1:0]  sbu0, sbu1, sbu2, sbu3, sbu4, sbu5;
    (* max_fanout = 64 *) reg sb_v;

    wire signed [ACC_W-1:0] n11  = acc11    + p11;
    wire signed [ACC_W-1:0] n22  = acc22    + p22;
    wire signed [ACC_W-1:0] n12r = acc12_re + p12_re;
    wire signed [ACC_W-1:0] n12i = acc12_im + p12_im;
    wire signed [SX_W-1:0]  nx1i = accx1i + a_i2;
    wire signed [SX_W-1:0]  nx1q = accx1q + a_q2;
    wire signed [SX_W-1:0]  nx2i = accx2i + b_i2;
    wire signed [SX_W-1:0]  nx2q = accx2q + b_q2;
    wire        [CS_W-1:0]  nc1  = accc1 + c1_2;
    wire        [CS_W-1:0]  nc2  = accc2 + c2_2;
    wire        [DATA_W-1:0] np1 = (pk1_2 > accp1) ? pk1_2 : accp1;
    wire        [DATA_W-1:0] np2 = (pk2_2 > accp2) ? pk2_2 : accp2;
    wire signed [ACC_W-1:0] nu0 = accu0 + u11r;
    wire signed [ACC_W-1:0] nu1 = accu1 + u11i;
    wire signed [ACC_W-1:0] nu2 = accu2 + u22r;
    wire signed [ACC_W-1:0] nu3 = accu3 + u22i;
    wire signed [ACC_W-1:0] nu4 = accu4 + u12r;
    wire signed [ACC_W-1:0] nu5 = accu5 + u12i;

    always @(posedge clk) begin
        if (z2) begin
            acc11 <= 0;  acc22 <= 0;  acc12_re <= 0;  acc12_im <= 0;
            accx1i <= 0; accx1q <= 0; accx2i <= 0;    accx2q <= 0;
            accc1 <= 0;  accc2 <= 0;  accp1 <= 0;     accp2 <= 0;
            accu0 <= 0;  accu1 <= 0;  accu2 <= 0;  accu3 <= 0;  accu4 <= 0;  accu5 <= 0;
        end else if (v2) begin
            acc11 <= n11;   acc22 <= n22;   acc12_re <= n12r;  acc12_im <= n12i;
            accx1i <= nx1i; accx1q <= nx1q; accx2i <= nx2i;    accx2q <= nx2q;
            accc1 <= nc1;   accc2 <= nc2;   accp1 <= np1;      accp2 <= np2;
            accu0 <= nu0;   accu1 <= nu1;   accu2 <= nu2;  accu3 <= nu3;  accu4 <= nu4;  accu5 <= nu5;
        end
    end
    always @(posedge clk) begin
        if (l2) begin
            sb11 <= n11;    sb22 <= n22;    sb12_re <= n12r;   sb12_im <= n12i;
            sbx1i <= nx1i;  sbx1q <= nx1q;  sbx2i <= nx2i;     sbx2q <= nx2q;
            sbc1 <= nc1;    sbc2 <= nc2;    sbp1 <= np1;       sbp2 <= np2;
            sbu0 <= nu0;    sbu1 <= nu1;    sbu2 <= nu2;  sbu3 <= nu3;  sbu4 <= nu4;  sbu5 <= nu5;
        end
    end
    always @(posedge clk) begin
        if (rst) sb_v <= 1'b0; else sb_v <= l2;
    end

    // ======================= stage 4: newest - oldest (history ring) =========
    // One distributed RAM per term (NSUB deep). No reset on the contents: until
    // `fill` reaches NSUB the entry being replaced has never been written and
    // its contribution is taken as zero (o_* forced to 0 through their R pin).
    reg signed [ACC_W-1:0]  h11 [0:NSUB-1];
    reg signed [ACC_W-1:0]  h22 [0:NSUB-1];
    reg signed [ACC_W-1:0]  h12r[0:NSUB-1];
    reg signed [ACC_W-1:0]  h12i[0:NSUB-1];
    reg signed [SX_W-1:0]   hx1i[0:NSUB-1];
    reg signed [SX_W-1:0]   hx1q[0:NSUB-1];
    reg signed [SX_W-1:0]   hx2i[0:NSUB-1];
    reg signed [SX_W-1:0]   hx2q[0:NSUB-1];
    reg        [CS_W-1:0]   hc1 [0:NSUB-1];
    reg        [CS_W-1:0]   hc2 [0:NSUB-1];
    reg        [DATA_W-1:0] hp1 [0:NSUB-1];
    reg        [DATA_W-1:0] hp2 [0:NSUB-1];
    reg signed [ACC_W-1:0]  hu0 [0:NSUB-1];
    reg signed [ACC_W-1:0]  hu1 [0:NSUB-1];
    reg signed [ACC_W-1:0]  hu2 [0:NSUB-1];
    reg signed [ACC_W-1:0]  hu3 [0:NSUB-1];
    reg signed [ACC_W-1:0]  hu4 [0:NSUB-1];
    reg signed [ACC_W-1:0]  hu5 [0:NSUB-1];
    reg        [PTR_W-1:0]  ptr;
    reg        [LOG2_SUB:0] fill;               // sub-blocks seen, saturates at NSUB
    (* max_fanout = 64 *) reg oz;               // 1 = window not yet full: oldest entry is 0

    always @(posedge clk) begin
        if (sb_v) begin
            h11[ptr]  <= sb11;     h22[ptr]  <= sb22;
            h12r[ptr] <= sb12_re;  h12i[ptr] <= sb12_im;
            hx1i[ptr] <= sbx1i;    hx1q[ptr] <= sbx1q;
            hx2i[ptr] <= sbx2i;    hx2q[ptr] <= sbx2q;
            hc1[ptr]  <= sbc1;     hc2[ptr]  <= sbc2;
            hp1[ptr]  <= sbp1;     hp2[ptr]  <= sbp2;
            hu0[ptr]  <= sbu0;     hu1[ptr]  <= sbu1;     hu2[ptr] <= sbu2;
            hu3[ptr]  <= sbu3;     hu4[ptr]  <= sbu4;     hu5[ptr] <= sbu5;
        end
    end

    // The entry being replaced (h*[ptr]) is fixed for the whole sub-block, so it
    // is read into registers every clock; ptr / fill change only on sb_v,
    // >= 2^LOG2_S clocks before the next use.
    reg signed [ACC_W-1:0]  o_11, o_22, o_12r, o_12i;
    reg signed [SX_W-1:0]   o_x1i, o_x1q, o_x2i, o_x2q;
    reg        [CS_W-1:0]   o_c1, o_c2;
    reg signed [ACC_W-1:0]  o_u0, o_u1, o_u2, o_u3, o_u4, o_u5;

    always @(posedge clk) begin
        oz <= (fill != NSUB);
        if (oz) begin
            o_11 <= 0;  o_22 <= 0;  o_12r <= 0;  o_12i <= 0;
            o_x1i <= 0; o_x1q <= 0; o_x2i <= 0;  o_x2q <= 0;
            o_c1 <= 0;  o_c2 <= 0;
            o_u0 <= 0;  o_u1 <= 0;  o_u2 <= 0;  o_u3 <= 0;  o_u4 <= 0;  o_u5 <= 0;
        end else begin
            o_11 <= h11[ptr];    o_22 <= h22[ptr];    o_12r <= h12r[ptr];  o_12i <= h12i[ptr];
            o_x1i <= hx1i[ptr];  o_x1q <= hx1q[ptr];  o_x2i <= hx2i[ptr];  o_x2q <= hx2q[ptr];
            o_c1 <= hc1[ptr];    o_c2 <= hc2[ptr];
            o_u0 <= hu0[ptr];    o_u1 <= hu1[ptr];    o_u2 <= hu2[ptr];
            o_u3 <= hu3[ptr];    o_u4 <= hu4[ptr];    o_u5 <= hu5[ptr];
        end
    end

    (* max_fanout = 64 *) reg d_v;
    reg                     d_full;
    // v6 (self-healing window): the newest sub-block, and the hop at which a
    // fresh sum of the last NSUB sub-blocks replaces the running window
    reg signed [ACC_W-1:0]  h_n11, h_n22, h_n12_re, h_n12_im;
    reg signed [SX_W-1:0]   h_nx1i, h_nx1q, h_nx2i, h_nx2q;
    reg        [CS_W-1:0]   h_nc1, h_nc2;
    reg signed [ACC_W-1:0]  h_n_u0, h_n_u1, h_n_u2, h_n_u3, h_n_u4, h_n_u5;
    (* max_fanout = 64 *) reg d_wrap;
    (* max_fanout = 32 *) reg d_usef;   // wrap AND F complete: replace W (registered)
    reg                     f_ok;       // F started at a wrap: complete at the next
    reg signed [ACC_W-1:0]  d11, d22, d12_re, d12_im;
    reg signed [SX_W-1:0]   dx1i, dx1q, dx2i, dx2q;
    reg signed [CLIP_W:0]   dc1, dc2;
    reg signed [ACC_W-1:0]  d_u0, d_u1, d_u2, d_u3, d_u4, d_u5;

    always @(posedge clk) begin
        if (rst) begin
            ptr  <= {PTR_W{1'b0}};
            fill <= {(LOG2_SUB+1){1'b0}};
            d_v  <= 1'b0;
        end else begin
            d_v <= sb_v;
            if (sb_v) begin
                ptr    <= (ptr == NSUB-1) ? {PTR_W{1'b0}} : ptr + 1'b1;
                if (fill != NSUB) fill <= fill + 1'b1;
            end
        end
    end
    always @(posedge clk) begin
        if (sb_v) begin
            d11    <= sb11    - o_11;
            d22    <= sb22    - o_22;
            d12_re <= sb12_re - o_12r;
            d12_im <= sb12_im - o_12i;
            dx1i   <= sbx1i   - o_x1i;
            dx1q   <= sbx1q   - o_x1q;
            dx2i   <= sbx2i   - o_x2i;
            dx2q   <= sbx2q   - o_x2q;
            d_u0 <= sbu0 - o_u0;  d_u1 <= sbu1 - o_u1;  d_u2 <= sbu2 - o_u2;
            d_u3 <= sbu3 - o_u3;  d_u4 <= sbu4 - o_u4;  d_u5 <= sbu5 - o_u5;
            dc1    <= $signed({1'b0, {(CLIP_W-CS_W){1'b0}}, sbc1})
                    - $signed({1'b0, {(CLIP_W-CS_W){1'b0}}, o_c1});
            dc2    <= $signed({1'b0, {(CLIP_W-CS_W){1'b0}}, sbc2})
                    - $signed({1'b0, {(CLIP_W-CS_W){1'b0}}, o_c2});
            d_full <= (fill >= NSUB-1);     // this sub-block completes a full window
            h_n11  <= sb11;   h_n22  <= sb22;   h_n12_re <= sb12_re;  h_n12_im <= sb12_im;
            h_nx1i <= sbx1i;  h_nx1q <= sbx1q;  h_nx2i   <= sbx2i;    h_nx2q   <= sbx2q;
            h_nc1  <= sbc1;   h_nc2  <= sbc2;
            h_n_u0 <= sbu0;   h_n_u1 <= sbu1;   h_n_u2 <= sbu2;
            h_n_u3 <= sbu3;   h_n_u4 <= sbu4;   h_n_u5 <= sbu5;
            d_wrap <= (ptr == NSUB-1);
            d_usef <= (ptr == NSUB-1) & f_ok;
        end
    end

    // ======================= stage 5: window update  W += newest - oldest ====
    // v6 SELF-HEALING (field, 4 Nov: R22 read 0 / +11 dBFS for a whole session
    // after a sample-rate change, while RX1, R12 and the ADC peaks were fine).
    // A running sum W += newest - oldest never forgets a disturbance: one bad
    // update (a clock glitch while the AD9361 re-tunes its clock chain at a
    // rate change, which does not reset this core) offsets W for good. Now a
    // second accumulator F sums the newest sub-blocks from one ring wrap to
    // the next; at each wrap W is REPLACED by F + newest = the exact sum of
    // the last NSUB sub-blocks. Same numbers when nothing went wrong
    // (bit-exact), and any upset is gone after at most NSUB hops (one window).
    reg signed [ACC_W-1:0]  w11, w22, w12_re, w12_im;
    reg signed [SX_W-1:0]   wx1i, wx1q, wx2i, wx2q;
    reg signed [CLIP_W:0]   wc1, wc2;
    reg signed [ACC_W-1:0]  w_u0, w_u1, w_u2, w_u3, w_u4, w_u5;
    reg signed [ACC_W-1:0]  f11, f22, f12_re, f12_im;
    reg signed [SX_W-1:0]   fx1i, fx1q, fx2i, fx2q;
    reg signed [CLIP_W:0]   fc1, fc2;
    reg signed [ACC_W-1:0]  f_u0, f_u1, f_u2, f_u3, f_u4, f_u5;
    wire                    use_f = d_usef;
    // v6: the window update takes two clocks (operands, then one adder)
    reg signed [ACC_W-1:0]  a11, a22, a12_re, a12_im, b11, b22, b12_re, b12_im;
    reg signed [SX_W-1:0]   ax1i, ax1q, ax2i, ax2q, bx1i, bx1q, bx2i, bx2q;
    reg signed [CLIP_W:0]   ac1, ac2, bc1, bc2;
    reg signed [ACC_W-1:0]  a_u0, a_u1, a_u2, a_u3, a_u4, a_u5, b_u0, b_u1, b_u2, b_u3, b_u4, b_u5;
    (* max_fanout = 64 *) reg d_vw;      // d_v one clock later: W and the commit sequence
    wire signed [CLIP_W:0]  h_nc1x = $signed({1'b0, {(CLIP_W-CS_W){1'b0}}, h_nc1});
    wire signed [CLIP_W:0]  h_nc2x = $signed({1'b0, {(CLIP_W-CS_W){1'b0}}, h_nc2});

    always @(posedge clk) begin
        if (rst) begin
            f_ok <= 1'b0;
        end else if (d_v) begin
            if (d_wrap) f_ok <= 1'b1;
        end
    end

    // v6 stage 5a (d_v): operands -- the exact fresh window at a wrap
    // (F + newest), else W and newest - oldest
    always @(posedge clk) begin
        d_vw <= d_v & ~rst;
        if (d_v) begin
            a11 <= use_f ? f11 : w11;  b11 <= use_f ? h_n11 : d11;
            a22 <= use_f ? f22 : w22;  b22 <= use_f ? h_n22 : d22;
            a12_re <= use_f ? f12_re : w12_re;  b12_re <= use_f ? h_n12_re : d12_re;
            a12_im <= use_f ? f12_im : w12_im;  b12_im <= use_f ? h_n12_im : d12_im;
            ax1i <= use_f ? fx1i : wx1i;  bx1i <= use_f ? h_nx1i : dx1i;
            ax1q <= use_f ? fx1q : wx1q;  bx1q <= use_f ? h_nx1q : dx1q;
            ax2i <= use_f ? fx2i : wx2i;  bx2i <= use_f ? h_nx2i : dx2i;
            ax2q <= use_f ? fx2q : wx2q;  bx2q <= use_f ? h_nx2q : dx2q;
            ac1 <= use_f ? fc1 : wc1;  bc1 <= use_f ? h_nc1x : dc1;
            ac2 <= use_f ? fc2 : wc2;  bc2 <= use_f ? h_nc2x : dc2;
            a_u0 <= use_f ? f_u0 : w_u0;  b_u0 <= use_f ? h_n_u0 : d_u0;
            a_u1 <= use_f ? f_u1 : w_u1;  b_u1 <= use_f ? h_n_u1 : d_u1;
            a_u2 <= use_f ? f_u2 : w_u2;  b_u2 <= use_f ? h_n_u2 : d_u2;
            a_u3 <= use_f ? f_u3 : w_u3;  b_u3 <= use_f ? h_n_u3 : d_u3;
            a_u4 <= use_f ? f_u4 : w_u4;  b_u4 <= use_f ? h_n_u4 : d_u4;
            a_u5 <= use_f ? f_u5 : w_u5;  b_u5 <= use_f ? h_n_u5 : d_u5;
        end
    end
    // v6 stage 5b (d_vw): one adder per term
    always @(posedge clk) begin
        if (rst) begin
            w11  <= 0;  w22  <= 0;  w12_re <= 0;  w12_im <= 0;
            wx1i <= 0;  wx1q <= 0;  wx2i <= 0;    wx2q <= 0;
            wc1  <= 0;  wc2  <= 0;
            w_u0 <= 0;  w_u1 <= 0;  w_u2 <= 0;  w_u3 <= 0;  w_u4 <= 0;  w_u5 <= 0;
        end else if (d_vw) begin
            w11 <= a11 + b11;
            w22 <= a22 + b22;
            w12_re <= a12_re + b12_re;
            w12_im <= a12_im + b12_im;
            wx1i <= ax1i + bx1i;
            wx1q <= ax1q + bx1q;
            wx2i <= ax2i + bx2i;
            wx2q <= ax2q + bx2q;
            wc1 <= ac1 + bc1;
            wc2 <= ac2 + bc2;
            w_u0 <= a_u0 + b_u0;
            w_u1 <= a_u1 + b_u1;
            w_u2 <= a_u2 + b_u2;
            w_u3 <= a_u3 + b_u3;
            w_u4 <= a_u4 + b_u4;
            w_u5 <= a_u5 + b_u5;
        end
    end
    // fresh sums: restart at every wrap (datapath, no reset needed: f_ok guards)
    always @(posedge clk) begin
        if (d_v) begin
            if (d_wrap) begin
                f11 <= 0;  f22 <= 0;  f12_re <= 0;  f12_im <= 0;
                fx1i <= 0; fx1q <= 0; fx2i <= 0;    fx2q <= 0;
                fc1 <= 0;  fc2 <= 0;
                f_u0 <= 0; f_u1 <= 0; f_u2 <= 0; f_u3 <= 0; f_u4 <= 0; f_u5 <= 0;
            end else begin
                f11    <= f11    + h_n11;
                f22    <= f22    + h_n22;
                f12_re <= f12_re + h_n12_re;
                f12_im <= f12_im + h_n12_im;
                fx1i <= fx1i + h_nx1i;  fx1q <= fx1q + h_nx1q;
                fx2i <= fx2i + h_nx2i;  fx2q <= fx2q + h_nx2q;
                fc1  <= fc1 + h_nc1x;   fc2  <= fc2 + h_nc2x;
                f_u0 <= f_u0 + h_n_u0;  f_u1 <= f_u1 + h_n_u1;  f_u2 <= f_u2 + h_n_u2;
                f_u3 <= f_u3 + h_n_u3;  f_u4 <= f_u4 + h_n_u4;  f_u5 <= f_u5 + h_n_u5;
            end
        end
    end

    // ======================= commit sequence: mean correction + peak scan ====
    //   products: 1 s1i^2  2 s1q^2  3 s2i^2  4 s2q^2  5 s1i s2i  6 s1q s2q
    //             7 s1q s2i  8 s1i s2q  9 s1i s1q  10 s2i s2q
    //   phases 15-16 combine:  R:  c11 = 1+2, c22 = 3+4, cre = 5+6, cim = 7-8
    //                      Q:  q11 = (1-2) + j 2*9,  q22 = (3-4) + j 2*10,
    //                          q12 = (5-6) + j (8+7)
    (* max_fanout = 64 *) reg [PH_PUB:1] pv;     // one-hot phase
    reg                     cm_full, cm_dc;
    (* max_fanout = 64 *) reg kz;               // phase 16 with mean removal off: corrections = 0
    (* max_fanout = 64 *) reg pub;              // phase PH_PUB with a full window

    always @(posedge clk) begin
        if (rst) begin
            pv <= {PH_PUB{1'b0}};
            kz <= 1'b0;  pub <= 1'b0;
        end else begin
            pv  <= {pv[PH_PUB-1:1], d_vw};
            kz  <= pv[16] & ~cm_dc;
            pub <= pv[PH_PUB-1] & cm_full;
        end
        if (d_vw) begin
            cm_full <= d_full;
            cm_dc   <= cfg_dc_rm;
        end
    end

    // ---- mean-correction products: opa * opb = opa*lo + (opa*hi) << 17 ----
    reg signed [SX_W-1:0]   opa, opb;
    reg signed [SX_W+17:0]  mpl1, mpl2;         // opa * {0, opb[16:0]}
    reg signed [SX_W+HB_W-1:0] mph1, mph2;      // opa * opb[SX_W-1:17]
    reg signed [MP_W-1:0]   mp3;
    wire signed [17:0]      opb_lo = {1'b0, opb[16:0]};
    wire signed [HB_W-1:0]  opb_hi = opb[SX_W-1:17];

    // v5.1 (Vivado 2026-11-02, -0.06 ns): the operand choice is decoded one
    // clock EARLIER into 2-bit codes (0 = s1i, 1 = s1q, 2 = s2i, 3 = s2q) from
    // the NEXT phase vector, so loading opa/opb is one LUT (4:1 mux) and the
    // registers stay inside the DSP (A/B). Was: a 10-way one-hot select on the
    // fan-out-63 phase bits, 3 LUT levels, and Vivado moved B out of the DSP.
    //   phase : 1  2  3  4  5  6  7  8  9  10
    //   opa   : 0  1  2  3  0  1  1  0  0  2
    //   opb   : 0  1  2  3  2  3  2  3  1  3
    wire [10:1] pnx = {pv[9:1], d_vw};         // phase vector of the next clock
    reg  [1:0]  sela = 2'd0, selb = 2'd0;
    reg         opld = 1'b0;
    always @(posedge clk) begin
        sela <= {pnx[3] | pnx[4] | pnx[10],
                 pnx[2] | pnx[4] | pnx[6] | pnx[7]};
        selb <= {pnx[3] | pnx[4] | pnx[5] | pnx[6] | pnx[7] | pnx[8] | pnx[10],
                 pnx[2] | pnx[4] | pnx[6] | pnx[8] | pnx[9] | pnx[10]};
        opld <= ~rst & (|pnx);
    end

    always @(posedge clk) begin
        if (opld) begin
            case (sela)
                2'd0: opa <= wx1i;  2'd1: opa <= wx1q;
                2'd2: opa <= wx2i;  default: opa <= wx2q;
            endcase
            case (selb)
                2'd0: opb <= wx1i;  2'd1: opb <= wx1q;
                2'd2: opb <= wx2i;  default: opb <= wx2q;
            endcase
        end
        mpl1 <= opa * opb_lo;   mpl2 <= mpl1;
        mph1 <= opa * opb_hi;   mph2 <= mph1;
        mp3  <= $signed({mph2, 17'd0}) + $signed(mpl2);
    end

    reg signed [MP_W-1:0]   pr1, pr2, pr3, pr4, pr5, pr6, pr7, pr8, pr9, pr10;
    always @(posedge clk) begin                 // product k is in mp3 in phase k+4
        if (pv[5])  pr1  <= mp3;
        if (pv[6])  pr2  <= mp3;
        if (pv[7])  pr3  <= mp3;
        if (pv[8])  pr4  <= mp3;
        if (pv[9])  pr5  <= mp3;
        if (pv[10]) pr6  <= mp3;
        if (pv[11]) pr7  <= mp3;
        if (pv[12]) pr8  <= mp3;
        if (pv[13]) pr9  <= mp3;
        if (pv[14]) pr10 <= mp3;
    end

    function signed [MC_W:0] sx;                // sign-extend a product
        input signed [MP_W-1:0] v;
        begin
            sx = {{2{v[MP_W-1]}}, v};
        end
    endfunction

    // round-half-up division by N:  (c + N/2) >> L  =  ((c >> (L-1)) + 1) >> 1
    localparam RD_W = MC_W + 2 - (LOG2_N - 1);
    function signed [ACC_W-1:0] rdiv;
        input signed [MC_W:0] c;
        reg   signed [RD_W-1:0] t;
        begin
            t    = (c >>> (LOG2_N - 1)) + 1;
            rdiv = t[ACC_W:1];
        end
    endfunction

    // ---- correction sums c = a +- b, in two halves (carry registered) ----
    wire signed [MC_W:0] ca [0:9];
    wire signed [MC_W:0] cb [0:9];
    wire        [9:0]    csub = 10'b0101011000;     // terms 3, 4, 6, 8 subtract
    assign ca[0] = sx(pr1);   assign cb[0] = sx(pr2);
    assign ca[1] = sx(pr3);   assign cb[1] = sx(pr4);
    assign ca[2] = sx(pr5);   assign cb[2] = sx(pr6);
    assign ca[3] = sx(pr7);   assign cb[3] = sx(pr8);
    assign ca[4] = sx(pr1);   assign cb[4] = sx(pr2);
    assign ca[5] = sx(pr9);   assign cb[5] = sx(pr9);
    assign ca[6] = sx(pr3);   assign cb[6] = sx(pr4);
    assign ca[7] = sx(pr10);  assign cb[7] = sx(pr10);
    assign ca[8] = sx(pr5);   assign cb[8] = sx(pr6);
    assign ca[9] = sx(pr8);   assign cb[9] = sx(pr7);

    genvar gc;
    generate
        for (gc = 0; gc < 10; gc = gc + 1) begin : G_CS
            wire [MC_W:0]      bx = csub[gc] ? ~cb[gc] : cb[gc];
            reg  [CH:0]        lo;
            reg  [MC_W-CH:0]   ah, bh;
            reg  signed [MC_W:0] c;
            always @(posedge clk) begin
                if (pv[15]) begin
                    lo <= {1'b0, ca[gc][CH-1:0]} + {1'b0, bx[CH-1:0]} + csub[gc];
                    ah <= ca[gc][MC_W:CH];
                    bh <= bx[MC_W:CH];
                end
                if (pv[16])
                    c <= {ah + bh + lo[CH], lo[CH-1:0]};
            end
        end
    endgenerate
    wire signed [MC_W:0] cc0 = G_CS[0].c, cc1 = G_CS[1].c, cc2 = G_CS[2].c, cc3 = G_CS[3].c,
                         cc4 = G_CS[4].c, cc5 = G_CS[5].c, cc6 = G_CS[6].c, cc7 = G_CS[7].c,
                         cc8 = G_CS[8].c, cc9 = G_CS[9].c;

    reg signed [ACC_W-1:0]  kk0, kk1, kk2, kk3, kk4, kk5, kk6, kk7, kk8, kk9;
    reg signed [ACC_W-1:0]  d11c, d22c, ore, oim;
    reg signed [ACC_W-1:0]  oq0, oq1, oq2, oq3, oq4, oq5;
    reg signed [ACC_W-1:0]  o11, o22;

    always @(posedge clk) begin
        if (kz) begin                           // mean removal off: publish the raw sums
            kk0 <= 0; kk1 <= 0; kk2 <= 0; kk3 <= 0; kk4 <= 0;
            kk5 <= 0; kk6 <= 0; kk7 <= 0; kk8 <= 0; kk9 <= 0;
        end else if (pv[17]) begin
            kk0 <= rdiv(cc0); kk1 <= rdiv(cc1); kk2 <= rdiv(cc2); kk3 <= rdiv(cc3); kk4 <= rdiv(cc4);
            kk5 <= rdiv(cc5); kk6 <= rdiv(cc6); kk7 <= rdiv(cc7); kk8 <= rdiv(cc8); kk9 <= rdiv(cc9);
        end
        if (pv[18]) begin
            d11c <= w11 - kk0;                  // r11 = max(w11 - k, 0) (>= 0 by Cauchy-Schwarz)
            d22c <= w22 - kk1;
            ore  <= w12_re - kk2;
            oim  <= w12_im - kk3;
            oq0  <= w_u0 - kk4;  oq1 <= w_u1 - kk5;  oq2 <= w_u2 - kk6;
            oq3  <= w_u3 - kk7;  oq4 <= w_u4 - kk8;  oq5 <= w_u5 - kk9;
        end
        if (pv[19]) begin
            o11 <= d11c[ACC_W-1] ? {ACC_W{1'b0}} : d11c;
            o22 <= d22c[ACC_W-1] ? {ACC_W{1'b0}} : d22c;
        end
    end

    // ---- peak scan over the NSUB history entries ----
    reg [PTR_W-1:0]         si;
    reg [LOG2_SUB+1:0]      scn;
    reg                     rdv;
    reg [DATA_W-1:0]        rd1, rd2, scp1, scp2;
    always @(posedge clk) begin
        if (pv[1]) begin
            si <= {PTR_W{1'b0}};  scn <= NSUB;  rdv <= 1'b0;
            scp1 <= {DATA_W{1'b0}};  scp2 <= {DATA_W{1'b0}};
        end else begin
            rdv <= (scn != 0);
            if (scn != 0) begin
                rd1 <= hp1[si];  rd2 <= hp2[si];
                si  <= si + 1'b1;  scn <= scn - 1'b1;
            end
            if (rdv) begin
                scp1 <= (rd1 > scp1) ? rd1 : scp1;
                scp2 <= (rd2 > scp2) ? rd2 : scp2;
            end
        end
    end

    // ---- publish ----
    always @(posedge clk) begin
        if (rst) cov_valid <= 1'b0; else cov_valid <= pub;
    end
    always @(posedge clk) begin
        if (rst) begin
            r11 <= 0;  r22 <= 0;  r12_re <= 0;  r12_im <= 0;
            q11_re <= 0;  q11_im <= 0;  q22_re <= 0;  q22_im <= 0;  q12_re <= 0;  q12_im <= 0;
            clip1 <= 0;  clip2 <= 0;
            s1_i <= 0;  s1_q <= 0;  s2_i <= 0;  s2_q <= 0;
            peak1 <= 0;  peak2 <= 0;
            dc_rm_done <= 1'b0;
        end else if (pub) begin
            r11 <= o11;  r22 <= o22;  r12_re <= ore;  r12_im <= oim;
            q11_re <= oq0;  q11_im <= oq1;
            q22_re <= oq2;  q22_im <= oq3;
            q12_re <= oq4;  q12_im <= oq5;
            clip1 <= wc1[CLIP_W-1:0];
            clip2 <= wc2[CLIP_W-1:0];
            s1_i <= wx1i;  s1_q <= wx1q;  s2_i <= wx2i;  s2_q <= wx2q;
            peak1 <= scp1 >> PK_SH;  peak2 <= scp2 >> PK_SH;
            dc_rm_done <= cm_dc;
        end
    end

endmodule
