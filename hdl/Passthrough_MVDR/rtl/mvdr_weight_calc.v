// -----------------------------------------------------------------------------
// mvdr_weight_calc.v  --  2-element MVDR weights from the scaled covariance
//                         v2: 4-lane parallel datapath + ratio-preserving scaling
//
//   R = [ r11  r12 ; conj(r12)  r22 ],  r12 = p + j*q      (from cov_prep)
//   a = [ a1 ; a2 ]  complex steering vector, Q(A_FRAC)     (1.0 = 1<<A_FRAC)
//
//   u = adj(R) * a         u1 = r22*a1 - r12*a2
//                          u2 = -conj(r12)*a1 + r11*a2
//   d = a^H u              = a1r*u1r + a1i*u1i + a2r*u2r + a2i*u2i   (real, > 0)
//   w = u / d              == R^-1 a / (a^H R^-1 a)  (the 1/det factor cancels)
//
// Weights come out in Q(W_FRAC), W_W bits, ready to drop into beamformer_2ch
// (w_load = w_valid).  beamformer_2ch then applies conj(w), i.e. y = w^H x.
//
// WHAT CHANGED IN v2
//   1. SPEED.  v1 ran one shift-add multiplier and one divider for everything:
//      16 products one after another (256 clocks) and 4 divisions one after
//      another (220 clocks) -> 546 clocks per weight set.  v2 has four lanes,
//      one per weight component (u1r, u1i, u2r, u2i):
//        * the 3 products of each u component run in parallel lanes,
//        * the 4 products of d = a^H u run in parallel and are added in a tree,
//        * the 4 divisions run in 4 parallel real_dividers.
//      Same integer arithmetic, same rounding, same widths -> with
//      SAT_SCALE = 0 the weights are BIT-EXACT with v1 (checked in the tb),
//      in about 150 clocks instead of 546.  Still no DSP slices.
//
//   2. STABILITY when a weight does not fit in W_W bits (|w| >= 4.0 in Q15).
//      v1 CLAMPED each component separately.  That changes the RATIO w2/w1,
//      i.e. it changes where the null points -- exactly the case (a strong
//      jammer, unequal channel gains) where the null matters most.
//      With SAT_SCALE = 1 (default) all four components are instead shifted
//      right by the SAME number of bits, so the ratio, and therefore the null
//      and the SINR, are preserved; only the output level drops by 2^k (the
//      GNSS receiver's AGC absorbs that).  MVDR's SINR does not depend on a
//      common scale of w, so this is the mathematically correct response.
//      w_sat then means "the set was scaled down", not "the null is broken".
//      SAT_SCALE = 0 keeps the v1 clamp (for A/B comparison).
//
// v3: (a) DT / NUM_W scale with W_FRAC (W_FRAC = 15 is bit-identical to v2);
//     (b) SCALE_HYST: 6 dB hysteresis on the common scale so it cannot flip on
//         and off between updates (see S_Q4B).
//
// Arithmetic (unchanged from v1): exact integer products, so the cancellation
// inside u and d is exact.  d is shifted right by s2 so it keeps 24 significant
// bits, u is shifted by (s2 - A_FRAC) to keep the same precision, the numerator
// u' << W_FRAC is clamped to the divider width, and real_divider rounds to
// nearest.
//
// Handshake (unchanged):
//   start     1-clk pulse (cov_prep out_valid & ~r_zero); inputs are latched on
//             that clock.  A start while busy is ignored and flags `dropped`.
//   w_valid   1-clk pulse: new weights are on w1_re..w2_im (they then hold)
//   w_skip    1-clk pulse: run finished but weights NOT updated because d <= 0
//             (degenerate R or steering vector) -> keep previous weights
//   w_sat     valid with w_valid: the set was scaled down (SAT_SCALE=1) or a
//             component was clamped (SAT_SCALE=0)
//   busy      high from the start clock until w_valid / w_skip
//
// Timing: every state does at most ONE wide operation per lane (one add, one
// barrel shift, one compare), as in v1; the lanes are copies of v1's datapath.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module mvdr_weight_calc #(
    parameter R_W       = 32,     // cov_prep TARGET_W
    parameter A_W       = 16,     // steering vector width (signed)
    parameter A_FRAC    = 14,     // steering fractional bits (1.0 = 1<<14)
    parameter W_W       = 18,     // weight width (signed) = beamformer_2ch W_W
    parameter W_FRAC    = 15,     // weight fractional bits = beamformer_2ch W_FRAC
    parameter SAT_SCALE = 1,      // 1: scale whole set on overflow, 0: clamp (v1)
    parameter SCALE_HYST = 1      // 1: 6 dB hysteresis on the set scale (v3), 0: v2
)(
    input  wire                     clk,
    input  wire                     rst,          // synchronous, active high

    input  wire                     start,
    input  wire signed [R_W-1:0]    r11,
    input  wire signed [R_W-1:0]    r22,
    input  wire signed [R_W-1:0]    r12_re,
    input  wire signed [R_W-1:0]    r12_im,

    input  wire signed [A_W-1:0]    a1_re,
    input  wire signed [A_W-1:0]    a1_im,
    input  wire signed [A_W-1:0]    a2_re,
    input  wire signed [A_W-1:0]    a2_im,

    output reg                      busy,
    output reg                      w_valid,
    output reg                      w_skip,
    output reg                      w_sat,
    output reg                      dropped,
    output reg  signed [W_W-1:0]    w1_re,
    output reg  signed [W_W-1:0]    w1_im,
    output reg  signed [W_W-1:0]    w2_re,
    output reg  signed [W_W-1:0]    w2_im
);

    // ---------------- widths (as v1) ----------------
    localparam U_W   = R_W + A_W + 2;          // |u|  < 2^(U_W-1)
    localparam ACC_W = U_W + A_W + 2;          // |d|  < 2^(ACC_W-1)
    // d keeps DT significant bits; the numerator u' << W_FRAC is clamped so that
    // |w| <= ~2^11. Both scale with W_FRAC so the result is always within ~0.7
    // LSB of the exact MVDR weight. W_FRAC = 15 gives DT = 24, NUM_W = 52:
    // bit-identical to v2.
    localparam DT    = W_FRAC + 9;             // significant bits kept in d
    localparam NUM_W = DT + W_FRAC + 13;       // divider numerator
    localparam DEN_W = DT + 1;                 // (rounding can give 2^24)

    localparam signed [ACC_W-1:0] UCL  = ({{(ACC_W-1){1'b0}}, 1'b1} << (NUM_W-1-W_FRAC)) - 1;
    localparam signed [NUM_W-1:0] WMAX = ({{(NUM_W-1){1'b0}}, 1'b1} << (W_W-1)) - 1;
    localparam signed [NUM_W-1:0] WMIN = -({{(NUM_W-1){1'b0}}, 1'b1} << (W_W-1));

    // ---------------- states ----------------
    localparam [4:0] S_IDLE  = 5'd0,
                     S_LOAD  = 5'd1,    // load the 4 lanes' operands for product j
                     S_MAC   = 5'd2,    // 16 shift-add steps (all lanes at once)
                     S_NEXT  = 5'd3,    // next product / store u or d terms
                     S_DSUM1 = 5'd4,    // d = (t0 + t1) + (t2 + t3)
                     S_DSUM2 = 5'd5,
                     S_SC1   = 5'd6,    // bitlen(d)
                     S_SC2   = 5'd7,    // s2
                     S_SC3   = 5'd8,    // u shifts, d rounding constant
                     S_SC4   = 5'd9,    // d + rounding, u rounding constant
                     S_SC5   = 5'd10,   // d'
                     S_DP1   = 5'd11,   // u + rounding        (4 lanes)
                     S_DP2   = 5'd12,   // right shift         (4 lanes)
                     S_DP3   = 5'd13,   // left shift          (4 lanes)
                     S_DP4   = 5'd14,   // clamp compares      (4 lanes)
                     S_DP5   = 5'd15,   // divider inputs      (4 lanes)
                     S_DSTART= 5'd16,
                     S_DWAIT = 5'd17,
                     S_Q1    = 5'd18,   // |q|                 (4 lanes)
                     S_Q2    = 5'd19,   // pairwise max / clamp flags
                     S_Q3    = 5'd20,   // overall max
                     S_Q4    = 5'd21,   // bitlen -> common shift
                     S_Q5    = 5'd22,   // rounding constant
                     S_Q6    = 5'd23,   // q + rounding
                     S_Q7    = 5'd24,   // shift
                     S_Q8    = 5'd25,   // final 1-LSB guard compares
                     S_Q9    = 5'd26,   // select + store
                     S_FIN   = 5'd27,
                     S_Q4B   = 5'd28;   // scale hysteresis

    reg [4:0]                st;
    reg [1:0]                j;              // product index within a phase (0..2)
    reg                      dph;            // 0 = u phase, 1 = d phase
    reg [7:0]                bit_i;

    reg signed [R_W-1:0]     q11, q22, qp, qq;
    reg signed [A_W-1:0]     qa1r, qa1i, qa2r, qa2i;

    // ---- 4 lanes: lane 0 = u1r, 1 = u1i, 2 = u2r, 3 = u2i ----
    reg signed [ACC_W-1:0]   acc   [0:3];
    reg signed [ACC_W-1:0]   mcand [0:3];
    reg        [A_W-1:0]     ymul  [0:3];
    reg                      sneg  [0:3];
    reg signed [U_W-1:0]     u     [0:3];

    reg signed [ACC_W-1:0]   dpart01, dpart23, dsum;
    reg                      bad, satacc;
    reg        [7:0]         bl, s2, u_rsh, u_lsh;
    reg signed [ACC_W-1:0]   rnd_dr, rnd_ur, dsum_r;
    reg        [DEN_W-1:0]   dprime;

    reg signed [ACC_W-1:0]   usum  [0:3];
    reg signed [ACC_W-1:0]   uq_rr [0:3];
    reg signed [ACC_W-1:0]   uq_fr [0:3];
    reg                      hi_f  [0:3];
    reg                      lo_f  [0:3];

    reg                      dv_start;
    reg signed [NUM_W-1:0]   dv_num [0:3];
    reg        [DEN_W-1:0]   dv_den;
    wire       [3:0]         dv_done, dv_div0;
    wire signed [NUM_W-1:0]  dv_quo [0:3];

    reg        [NUM_W-1:0]   qabs  [0:3];
    reg        [NUM_W-1:0]   qmax01, qmax23, qmax;
    reg                      qhi   [0:3];
    reg                      qlo   [0:3];
    reg        [7:0]         qsh;
    reg        [7:0]         qsh_prev;       // scale applied to the last good set
    reg        [7:0]         q_need, q_need_m;
    reg signed [NUM_W-1:0]   qrnd;
    reg signed [NUM_W-1:0]   qr    [0:3];
    reg signed [NUM_W-1:0]   qs    [0:3];
    reg                      ghi   [0:3];
    reg                      glo   [0:3];
    reg signed [W_W-1:0]     tw1r, tw1i, tw2r, tw2i;

    genvar g;
    generate for (g = 0; g < 4; g = g + 1) begin : DIV
        real_divider #(.NUM_W(NUM_W), .DEN_W(DEN_W), .ROUND(1)) u_div (
            .clk(clk), .rst(rst), .start(dv_start), .num(dv_num[g]), .den(dv_den),
            .busy(), .done(dv_done[g]), .quo(dv_quo[g]), .div0(dv_div0[g]));
    end endgenerate

    // ---------------- product schedule ----------------
    //  u phase, product j = 0,1,2 (lane: x * y, sign)
    //   lane0 u1r = +r22*a1r - p*a2r + q*a2i
    //   lane1 u1i = +r22*a1i - p*a2i - q*a2r
    //   lane2 u2r = -p*a1r   - q*a1i + r11*a2r
    //   lane3 u2i = -p*a1i   + q*a1r + r11*a2i
    //  d phase (j = 0): lane k: u_k * a_k  (a = a1r, a1i, a2r, a2i), all +
    reg signed [ACC_W-1:0] xs [0:3];
    reg signed [A_W-1:0]   ys [0:3];
    reg                    ss [0:3];

    always @* begin
        if (!dph) begin
            case (j)
            2'd0: begin
                xs[0] = q22; ys[0] = qa1r; ss[0] = 1'b0;
                xs[1] = q22; ys[1] = qa1i; ss[1] = 1'b0;
                xs[2] = qp;  ys[2] = qa1r; ss[2] = 1'b1;
                xs[3] = qp;  ys[3] = qa1i; ss[3] = 1'b1;
            end
            2'd1: begin
                xs[0] = qp;  ys[0] = qa2r; ss[0] = 1'b1;
                xs[1] = qp;  ys[1] = qa2i; ss[1] = 1'b1;
                xs[2] = qq;  ys[2] = qa1i; ss[2] = 1'b1;
                xs[3] = qq;  ys[3] = qa1r; ss[3] = 1'b0;
            end
            default: begin
                xs[0] = qq;  ys[0] = qa2i; ss[0] = 1'b0;
                xs[1] = qq;  ys[1] = qa2r; ss[1] = 1'b1;
                xs[2] = q11; ys[2] = qa2r; ss[2] = 1'b0;
                xs[3] = q11; ys[3] = qa2i; ss[3] = 1'b0;
            end
            endcase
        end else begin
            xs[0] = u[0]; ys[0] = qa1r; ss[0] = 1'b0;
            xs[1] = u[1]; ys[1] = qa1i; ss[1] = 1'b0;
            xs[2] = u[2]; ys[2] = qa2r; ss[2] = 1'b0;
            xs[3] = u[3]; ys[3] = qa2i; ss[3] = 1'b0;
        end
    end

    // bit length of a positive value
    function [7:0] bitlen;
        input [ACC_W-1:0] v;
        integer i;
        begin
            bitlen = 8'd0;
            for (i = 0; i < ACC_W; i = i + 1)
                if (v[i]) bitlen = i + 1;
        end
    endfunction

    function [7:0] bitlen_q;
        input [NUM_W-1:0] v;
        integer i;
        begin
            bitlen_q = 8'd0;
            for (i = 0; i < NUM_W; i = i + 1)
                if (v[i]) bitlen_q = i + 1;
        end
    endfunction

    wire signed [ACC_W-1:0] dp_shift = dsum_r >>> s2;

    integer k;
    always @(posedge clk) begin
        if (rst) begin
            st       <= S_IDLE;
            busy     <= 1'b0;
            w_valid  <= 1'b0;
            w_skip   <= 1'b0;
            w_sat    <= 1'b0;
            dropped  <= 1'b0;
            dv_start <= 1'b0;
            qsh_prev <= 8'd0;
            w1_re <= {W_W{1'b0}};  w1_im <= {W_W{1'b0}};
            w2_re <= {W_W{1'b0}};  w2_im <= {W_W{1'b0}};
        end else begin
            w_valid  <= 1'b0;
            w_skip   <= 1'b0;
            dv_start <= 1'b0;
            dropped  <= start && (st != S_IDLE);

            case (st)
            S_IDLE: if (start) begin
                q11 <= r11;   q22 <= r22;   qp <= r12_re;  qq <= r12_im;
                qa1r <= a1_re; qa1i <= a1_im; qa2r <= a2_re; qa2i <= a2_im;
                j <= 2'd0;  dph <= 1'b0;
                for (k = 0; k < 4; k = k + 1) acc[k] <= {ACC_W{1'b0}};
                bad <= 1'b0;  satacc <= 1'b0;
                busy <= 1'b1;
                st <= S_LOAD;
            end

            S_LOAD: begin
                for (k = 0; k < 4; k = k + 1) begin
                    mcand[k] <= xs[k];  ymul[k] <= ys[k];  sneg[k] <= ss[k];
                end
                bit_i <= 8'd0;
                st <= S_MAC;
            end

            // two's-complement shift-add multiply, all 4 lanes in lockstep
            S_MAC: begin
                for (k = 0; k < 4; k = k + 1) begin
                    if (ymul[k][0])
                        acc[k] <= (sneg[k] ^ (bit_i == (A_W - 1))) ? (acc[k] - mcand[k])
                                                                    : (acc[k] + mcand[k]);
                    mcand[k] <= mcand[k] <<< 1;
                    ymul[k]  <= ymul[k] >> 1;
                end
                if (bit_i == (A_W - 1)) st <= S_NEXT;
                else                    bit_i <= bit_i + 8'd1;
            end

            S_NEXT: begin
                if (!dph) begin
                    if (j == 2'd2) begin                     // u complete
                        for (k = 0; k < 4; k = k + 1) begin
                            u[k]   <= acc[k][U_W-1:0];
                            acc[k] <= {ACC_W{1'b0}};
                        end
                        dph <= 1'b1;  j <= 2'd0;
                    end else begin
                        j <= j + 2'd1;
                    end
                    st <= S_LOAD;
                end else begin
                    st <= S_DSUM1;                           // 4 d terms in acc[]
                end
            end

            S_DSUM1: begin
                dpart01 <= acc[0] + acc[1];
                dpart23 <= acc[2] + acc[3];
                st <= S_DSUM2;
            end
            S_DSUM2: begin
                dsum <= dpart01 + dpart23;
                st <= S_SC1;
            end

            S_SC1: begin                                     // d must be positive
                if (dsum <= 0) begin
                    bad <= 1'b1;
                    st  <= S_FIN;
                end else begin
                    bl <= bitlen(dsum);
                    st <= S_SC2;
                end
            end
            S_SC2: begin
                s2 <= (bl > DT) ? (bl - DT) : 8'd0;
                st <= S_SC3;
            end
            S_SC3: begin
                u_rsh  <= (s2 >= A_FRAC) ? (s2 - A_FRAC) : 8'd0;
                u_lsh  <= (s2 >= A_FRAC) ? 8'd0 : (A_FRAC - s2);
                rnd_dr <= (s2 == 8'd0) ? {ACC_W{1'b0}}
                          : ({{(ACC_W-1){1'b0}}, 1'b1} << (s2 - 8'd1));
                st <= S_SC4;
            end
            S_SC4: begin
                dsum_r <= dsum + rnd_dr;
                rnd_ur <= (u_rsh == 8'd0) ? {ACC_W{1'b0}}
                          : ({{(ACC_W-1){1'b0}}, 1'b1} << (u_rsh - 8'd1));
                st <= S_SC5;
            end
            S_SC5: begin
                dprime <= dp_shift[DEN_W-1:0];
                st <= S_DP1;
            end

            S_DP1: begin
                for (k = 0; k < 4; k = k + 1) usum[k] <= $signed(u[k]) + rnd_ur;
                st <= S_DP2;
            end
            S_DP2: begin
                for (k = 0; k < 4; k = k + 1) uq_rr[k] <= usum[k] >>> u_rsh;
                st <= S_DP3;
            end
            S_DP3: begin
                for (k = 0; k < 4; k = k + 1) uq_fr[k] <= uq_rr[k] <<< u_lsh;
                st <= S_DP4;
            end
            S_DP4: begin
                for (k = 0; k < 4; k = k + 1) begin
                    hi_f[k] <= (uq_fr[k] >  UCL);
                    lo_f[k] <= (uq_fr[k] < -UCL);
                end
                st <= S_DP5;
            end
            S_DP5: begin
                for (k = 0; k < 4; k = k + 1) begin
                    dv_num[k] <= (hi_f[k] ? UCL[NUM_W-1:0] :
                                 (lo_f[k] ? -UCL[NUM_W-1:0] : uq_fr[k][NUM_W-1:0])) <<< W_FRAC;
                    if (hi_f[k] || lo_f[k]) satacc <= 1'b1;   // numerator clamp (|w| > ~4096)
                end
                dv_den <= dprime;
                st <= S_DSTART;
            end
            S_DSTART: begin
                dv_start <= 1'b1;
                st <= S_DWAIT;
            end
            S_DWAIT: begin
                // all four dividers start together and have the same latency
                if (dv_done[0]) begin
                    if (|dv_div0) bad <= 1'b1;
                    st <= S_Q1;
                end
            end

            // ---------- quotient -> W_W bits ----------
            S_Q1: begin
                for (k = 0; k < 4; k = k + 1)
                    qabs[k] <= dv_quo[k][NUM_W-1] ? (~dv_quo[k] + 1'b1) : dv_quo[k];
                st <= S_Q2;
            end
            S_Q2: begin
                qmax01 <= (qabs[0] > qabs[1]) ? qabs[0] : qabs[1];
                qmax23 <= (qabs[2] > qabs[3]) ? qabs[2] : qabs[3];
                for (k = 0; k < 4; k = k + 1) begin
                    qhi[k] <= (dv_quo[k] > WMAX);
                    qlo[k] <= (dv_quo[k] < WMIN);
                end
                st <= S_Q3;
            end
            S_Q3: begin
                qmax <= (qmax01 > qmax23) ? qmax01 : qmax23;
                st <= S_Q4;
            end
            S_Q4: begin
                // shift needed so the largest |q| fits W_W bits, and the shift that
                // would leave 1 bit (6 dB) of headroom
                q_need   <= (bitlen_q(qmax) > (W_W-1)) ? (bitlen_q(qmax) - (W_W-1)) : 8'd0;
                q_need_m <= (bitlen_q(qmax) > (W_W-2)) ? (bitlen_q(qmax) - (W_W-2)) : 8'd0;
                st <= S_Q4B;
            end
            S_Q4B: begin
                // SCALE_HYST: grow the scale at once, release it only when the set
                // fits with 6 dB to spare. Without this, a channel-gain imbalance that
                // puts |w| right at the W_W limit (~12 dB) makes the scale flip every
                // update, stepping the output level by 6 dB at the window-hop rate.
                if (SAT_SCALE == 0)
                    qsh <= 8'd0;
                else if (SCALE_HYST == 0 || q_need >= qsh_prev)
                    qsh <= q_need;
                else
                    qsh <= (q_need_m < qsh_prev) ? q_need_m : qsh_prev;
                st <= S_Q5;
            end
            S_Q5: begin
                qrnd <= (qsh == 8'd0) ? {NUM_W{1'b0}}
                        : ({{(NUM_W-1){1'b0}}, 1'b1} << (qsh - 8'd1));
                if (qsh != 8'd0) satacc <= 1'b1;
                st <= S_Q6;
            end
            S_Q6: begin
                for (k = 0; k < 4; k = k + 1) qr[k] <= dv_quo[k] + qrnd;
                st <= S_Q7;
            end
            S_Q7: begin
                for (k = 0; k < 4; k = k + 1) qs[k] <= qr[k] >>> qsh;
                st <= S_Q8;
            end
            S_Q8: begin
                // SAT_SCALE=1: only a 1-LSB rounding carry can still exceed WMAX
                // SAT_SCALE=0: the v1 per-component clamp
                for (k = 0; k < 4; k = k + 1) begin
                    ghi[k] <= (SAT_SCALE != 0) ? (qs[k] > WMAX) : qhi[k];
                    glo[k] <= (SAT_SCALE != 0) ? (qs[k] < WMIN) : qlo[k];
                end
                st <= S_Q9;
            end
            S_Q9: begin
                for (k = 0; k < 4; k = k + 1)
                    if (SAT_SCALE == 0 && (qhi[k] || qlo[k])) satacc <= 1'b1;
                tw1r <= ghi[0] ? WMAX[W_W-1:0] : (glo[0] ? WMIN[W_W-1:0] : qs[0][W_W-1:0]);
                tw1i <= ghi[1] ? WMAX[W_W-1:0] : (glo[1] ? WMIN[W_W-1:0] : qs[1][W_W-1:0]);
                tw2r <= ghi[2] ? WMAX[W_W-1:0] : (glo[2] ? WMIN[W_W-1:0] : qs[2][W_W-1:0]);
                tw2i <= ghi[3] ? WMAX[W_W-1:0] : (glo[3] ? WMIN[W_W-1:0] : qs[3][W_W-1:0]);
                st <= S_FIN;
            end

            S_FIN: begin
                if (bad) begin
                    w_skip <= 1'b1;
                end else begin
                    // outputs change only on a good run (a skipped run keeps
                    // the previous weights, as in v1)
                    w1_re <= tw1r;  w1_im <= tw1i;
                    w2_re <= tw2r;  w2_im <= tw2i;
                    w_valid <= 1'b1;
                    w_sat   <= satacc;
                    qsh_prev <= qsh;
                end
                busy <= 1'b0;
                st   <= S_IDLE;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
