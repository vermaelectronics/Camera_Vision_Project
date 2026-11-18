// -----------------------------------------------------------------------------
// cov_prep.v  --  MVDR step 2: diagonal loading + block-floating-point scaling
//                 v2: NOISE-REFERENCED loading (self-estimated noise floor)
//
// Input : window sums from cov2x2_accum (r11, r22 >= 0, r12 complex), mean-
//         removed when dc_rm is on. Any common scale (1/N) is irrelevant.
// Output: R' = R + eps*I, scaled so the largest element fits TARGET_W signed
//         bits, for mvdr_weight_calc (MVDR weights do not depend on a common
//         scale of R).
//
//   eps = eps_nf + eps_rel + eps_abs
//
//   eps_nf  = (k/8) * L          L = det(R) / trace(R)      (NEW in v2, default k = 8)
//             For the 2x2 Hermitian R with eigenvalues l1 >= l2:
//                 det/trace = l1*l2/(l1+l2)  in  [l2/2, l2]
//             i.e. L tracks the NOISE eigenvalue: = l2 with a jammer present,
//             = l2/2 (half the noise) with none. So eps_nf is loading at the
//             noise floor whatever the jammer power. k = 8 -> x1 (0 dB above
//             noise), 80 -> x10 (+10 dB, Carlson's classic rule), 0 = off.
//   eps_rel = mean diagonal >> eps_rel_sh            (v1 term; 63 = off, now default off)
//   eps_abs = absolute, window-sum units             (v1 term; 0 = off)
//
// WHY (model/loading_study.py, N = 8192, PI with a = e1):
//   v1's default eps_rel_sh = 10 loads at 1/1024 of the TOTAL power, which
//   grows with the jammer. The jammer residual is then ~(1/1024)^2 of the
//   jammer power, a fixed -60 dB, so for JNR > 60 dB the jammer leaks back
//   ABOVE the noise floor:
//          JNR      v1 (rel 2^-10)    v2 (noise x1)      unloaded
//          40 dB       -22 dB           -37 dB            -41 dB   (residual vs noise)
//          60 dB        -3 dB           -40 dB            -43 dB
//          70 dB        +7 dB           -42 dB            -43 dB
//   Noise-referenced loading keeps the residual ~40 dB under the noise floor
//   at every JNR (the floor is the N = 8192 estimation limit) and halves the
//   weight jitter with no jammer (0.65 -> 0.32 deg rms) at no SNR cost.
//
// ARITHMETIC (exact integers; every rounding is round-half-up / to-nearest)
//   1. pre-scale : s0 so that max(r11, r22, |re|, |im|) < 2^PRE_B
//                  a,b,c,d = round(R / 2^s0)
//   2. L         : det = a*b - c^2 - d^2 (exact, 2*PRE_B+2 bits, clamped >= 0),
//                  T = a + b, L = round(det / T) (real_divider)
//   3. eps       : eps_nf  = round(L * k / 8)
//                  eps_rel = (T >> 1) >> eps_rel_sh
//                  eps_abs = round(eps_abs / 2^s0)
//   4. load      : a' = a + eps, b' = b + eps
//   5. scale     : s1 so that max(a', b', |c|, |d|) fits TARGET_W signed bits,
//                  outputs = round(. / 2^s1), saturated (only +1 LSB is possible)
//      shift  = s0 + s1 (total right shift applied to R)
//      nf_est = L * 2^s0 : noise-floor estimate in window-sum units (readback)
//      r_zero = the loaded matrix is all zero (keep the previous weights)
//   6. jammer detector (v2.1, for mvdr_ctrl_fsm's hold/glide):
//      coherence  g2 = |r12|^2 / (r11*r22) = (c^2+d^2) / (a*b), before loading
//      coh_jam = g2 > 2^-coh_sh        (exact: (c^2+d^2) << coh_sh > a*b)
//      coh_l2  = bitlen(a*b) - bitlen(c^2+d^2) ~ -log2(g2), +-1 (63 = none)
//      Noise only: g2 ~ 1/N (1.2e-4 for N = 8192). One jammer with per-element
//      JNR J: g2 = (J/(J+1))^2 -> -12 dB JNR gives 2^-8. Invariant to the RX1/RX2
//      gain ratio (an eigenvalue ratio is not). Default coh_sh = 8: no false
//      alarm in 10^14 windows, detects JNR >~ -12 dB.
//      strong  = det * 64 < a*b  (1 - g2 < 1/64: one dominant interferer, JNR
//                >~ 21 dB). Gates iq_est: only then is a single-window image
//                estimate (error ~ 1/(2 sqrt(N JNR))) better than the AD9361's
//                own ~ -50 dBc.
//
// HANDSHAKE / TIMING
//   Sequential, one operation per state at mclk (100 MHz): ~100 clocks
//   (~1 us) per window; a window arrives every 1024 samples (>= 1600 mclk), so
//   there is a >10x margin. Outputs hold until the next out_valid pulse.
//   A window arriving while busy is ignored (cannot happen at these rates;
//   `busy` is exported for a sticky check). One 32x32 pipelined multiplier
//   (4 DSP48), one real_divider (no DSP).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module cov_prep #(
    parameter ACC_W    = 38,     // cov2x2_accum ACC_W (2*DATA_W + LOG2_N + 1)
    parameter TARGET_W = 32      // width of scaled outputs (signed)
)(
    input  wire                        clk,
    input  wire                        rst,        // synchronous, active high

    input  wire                        in_valid,
    input  wire signed [ACC_W-1:0]     r11,
    input  wire signed [ACC_W-1:0]     r22,
    input  wire signed [ACC_W-1:0]     r12_re,
    input  wire signed [ACC_W-1:0]     r12_im,

    input  wire        [ACC_W-1:0]     eps_abs,    // unsigned, window-sum units (0 = off)
    input  wire        [5:0]           eps_rel_sh, // relative loading shift (63 = off)
    input  wire        [7:0]           eps_nf_k,   // noise loading = k/8 x noise floor (0 = off)
    input  wire        [3:0]           coh_sh,     // jammer detect: coherence > 2^-coh_sh

    output reg                         out_valid,
    output reg  signed [TARGET_W-1:0]  o11,
    output reg  signed [TARGET_W-1:0]  o22,
    output reg  signed [TARGET_W-1:0]  o12_re,
    output reg  signed [TARGET_W-1:0]  o12_im,
    output reg         [7:0]           shift,
    output reg                         r_zero,
    output reg         [ACC_W-1:0]     nf_est,     // noise floor estimate, window-sum units
    output reg                         coh_jam,    // correlated interference present
    output reg                         strong_jam, // one dominant interferer (JNR >~ 21 dB)
    output reg         [5:0]           coh_l2,     // ~ -log2(coherence), 63 = none
    output wire                        busy
);

    localparam PRE_B = TARGET_W - 2;           // pre-scaled magnitude < 2^PRE_B
    localparam XW    = ACC_W + 2;              // |input| incl. -2^(ACC_W-1)
    localparam DW    = 2*PRE_B + 3;            // det / product width (signed)
    localparam EW    = ACC_W + 4;              // eps / loaded-diagonal width (signed)
    localparam NUM_W = DW;                     // divider numerator
    localparam DEN_W = PRE_B + 2;              // T = a + b < 2^(PRE_B+1)

    // ---------------- states ----------------
    localparam [4:0] S_IDLE = 5'd0,  S_MX1 = 5'd1,  S_MX2 = 5'd2,  S_MX3 = 5'd3,
                     S_BL0  = 5'd4,  S_RND0 = 5'd5, S_ADD0 = 5'd6, S_SH0 = 5'd7,
                     S_P0   = 5'd8,  S_P1  = 5'd9,  S_P2  = 5'd10, S_P3  = 5'd11,
                     S_P4   = 5'd12, S_P5  = 5'd13, S_P6  = 5'd14, S_DET = 5'd15,
                     S_DIVW = 5'd16, S_NF0 = 5'd17, S_NFW = 5'd18, S_EPS1 = 5'd19,
                     S_EPS2 = 5'd20, S_LD  = 5'd21, S_MX4 = 5'd22, S_MX5 = 5'd23,
                     S_BL1  = 5'd24, S_RND1 = 5'd25, S_ADD1 = 5'd26, S_SH1 = 5'd27,
                     S_SAT  = 5'd28, S_OUT = 5'd29;

    reg [4:0] st;
    assign busy = (st != S_IDLE);

    function [7:0] bitlen_d;                   // bit length, product width
        input [DW-1:0] v;
        integer i;
        begin
            bitlen_d = 8'd0;
            for (i = 0; i < DW; i = i + 1)
                if (v[i]) bitlen_d = i + 1;
        end
    endfunction

    function [7:0] bitlen;                     // bit length of a non-negative value
        input [EW-1:0] v;
        integer i;
        begin
            bitlen = 8'd0;
            for (i = 0; i < EW; i = i + 1)
                if (v[i]) bitlen = i + 1;
        end
    endfunction

    // ---------------- latched inputs ----------------
    reg signed [XW-1:0]     x11, x22, xre, xim;      // sign-extended copies
    reg        [ACC_W-1:0]  q_eabs;
    reg        [5:0]        q_rsh;
    reg        [7:0]        q_k;
    reg        [3:0]        q_coh;

    reg        [XW-1:0]     mxa, mxb, mx;            // non-negative
    reg        [7:0]        s0, s1, bl;
    reg signed [XW-1:0]     rnd0;
    reg signed [XW-1:0]     t11, t22, tre, tim;      // + rounding
    reg signed [PRE_B+1:0]  a, b, c, d;              // pre-scaled (|.| <= 2^PRE_B)

    // ---------------- 32x32 multiplier, product in mp3 four clocks after issue
    reg signed [PRE_B+1:0]  ma, mb;
    reg signed [2*PRE_B+3:0] mp1, mp2, mp3;
    always @(posedge clk) begin
        mp1 <= ma * mb;
        mp2 <= mp1;
        mp3 <= mp2;
    end

    reg signed [DW-1:0]     det;
    reg        [DW-1:0]     pab, pcd;                // a*b, c^2+d^2 (both >= 0)
    reg                     cj, sj;
    reg        [5:0]        cl2;
    reg        [DEN_W-1:0]  tr;
    reg        [NUM_W-1:0]  L;                       // noise-floor estimate (scaled)

    // ---------------- divider: L = round(det / T) ----------------
    reg                     dv_start;
    wire                    dv_done, dv_div0;
    wire signed [NUM_W-1:0] dv_quo;
    real_divider #(.NUM_W(NUM_W), .DEN_W(DEN_W), .ROUND(1)) u_div (
        .clk(clk), .rst(rst), .start(dv_start), .num(det), .den(tr),
        .busy(), .done(dv_done), .quo(dv_quo), .div0(dv_div0));

    reg [2:0]               wcnt;
    reg signed [EW-1:0]     e_nf, e_rel, e_abs, e_sum;
    reg signed [EW-1:0]     ld11, ld22;
    reg signed [EW-1:0]     cx, dx;                  // c, d sign-extended to EW
    reg        [EW-1:0]     m2a, m2b, m2;
    reg signed [EW-1:0]     rnd1;
    reg signed [EW-1:0]     u11, u22, ure, uim;
    reg signed [EW-1:0]     v11, v22, vre, vim;
    reg                     zf;

    localparam signed [EW-1:0] MAXT =  ({{(EW-1){1'b0}}, 1'b1} << (TARGET_W-1)) - 1;
    localparam signed [EW-1:0] MINT = -({{(EW-1){1'b0}}, 1'b1} << (TARGET_W-1));

    wire [XW-1:0] ab_re = xre[XW-1] ? -xre : xre;
    wire [XW-1:0] ab_im = xim[XW-1] ? -xim : xim;
    wire [EW-1:0] ab_c  = cx[EW-1] ? -cx : cx;
    wire [EW-1:0] ab_d  = dx[EW-1] ? -dx : dx;

    // eps_abs rounded to the pre-scaled units (s0 is known when this is used)
    wire [ACC_W:0]  eabs_rnd = {1'b0, q_eabs} + ((s0 == 8'd0) ? {(ACC_W+1){1'b0}}
                               : ({{ACC_W{1'b0}}, 1'b1} << (s0 - 8'd1)));

    always @(posedge clk) begin
        dv_start  <= 1'b0;
        out_valid <= 1'b0;
        if (rst) begin
            st     <= S_IDLE;
            o11 <= 0; o22 <= 0; o12_re <= 0; o12_im <= 0;
            shift  <= 8'd0;
            r_zero <= 1'b1;
            nf_est <= {ACC_W{1'b0}};
            coh_jam <= 1'b0;
            strong_jam <= 1'b0;
            coh_l2  <= 6'd63;
            ma <= 0; mb <= 0;
        end else begin
            case (st)
            S_IDLE: if (in_valid) begin
                x11 <= {{2{r11[ACC_W-1]}},    r11};
                x22 <= {{2{r22[ACC_W-1]}},    r22};
                xre <= {{2{r12_re[ACC_W-1]}}, r12_re};
                xim <= {{2{r12_im[ACC_W-1]}}, r12_im};
                q_eabs <= eps_abs;
                q_rsh  <= eps_rel_sh;
                q_k    <= eps_nf_k;
                q_coh  <= coh_sh;
                st <= S_MX1;
            end

            // ---- 1. pre-scale ----
            S_MX1: begin
                mxa <= (x11 > x22) ? x11 : x22;           // diagonals are >= 0
                mxb <= (ab_re > ab_im) ? ab_re : ab_im;
                st  <= S_MX2;
            end
            S_MX2: begin mx <= (mxa > mxb) ? mxa : mxb;  st <= S_MX3; end
            S_MX3: begin bl <= bitlen(mx);               st <= S_BL0; end
            S_BL0: begin
                s0 <= (bl > PRE_B) ? (bl - PRE_B) : 8'd0;
                st <= S_RND0;
            end
            S_RND0: begin
                rnd0 <= (s0 == 8'd0) ? {XW{1'b0}} : ({{(XW-1){1'b0}}, 1'b1} << (s0 - 8'd1));
                st   <= S_ADD0;
            end
            S_ADD0: begin
                t11 <= x11 + rnd0;  t22 <= x22 + rnd0;
                tre <= xre + rnd0;  tim <= xim + rnd0;
                st  <= S_SH0;
            end
            S_SH0: begin
                a <= t11 >>> s0;  b <= t22 >>> s0;
                c <= tre >>> s0;  d <= tim >>> s0;
                st <= S_P0;
            end

            // ---- 2. det = a*b - c^2 - d^2 ; T = a + b ----
            S_P0: begin ma <= a; mb <= b; tr <= a + b; st <= S_P1; end
            S_P1: begin ma <= c; mb <= c; st <= S_P2; end
            S_P2: begin ma <= d; mb <= d; st <= S_P3; end
            S_P3: st <= S_P4;
            S_P4: begin det <= mp3;        pab <= mp3;        st <= S_P5; end  // a*b
            S_P5: begin det <= det - mp3;  pcd <= mp3;        st <= S_P6; end  // - c^2
            S_P6: begin det <= det - mp3;  pcd <= pcd + mp3;  st <= S_DET; end // - d^2
            S_DET: begin
                cj  <= ({pcd, 15'd0} >> (5'd15 - q_coh)) > {15'd0, pab};
                sj  <= (pab != {DW{1'b0}}) &&
                       (det[DW-1] || ({det, 6'd0} < {6'd0, pab}));
                cl2 <= (pcd == {DW{1'b0}} || pab == {DW{1'b0}}) ? 6'd63
                       : (bitlen_d(pab) > bitlen_d(pcd) ? bitlen_d(pab) - bitlen_d(pcd) : 8'd0);
                if (det[DW-1]) det <= {DW{1'b0}};             // rounding can make it -0
                dv_start <= 1'b1;
                st <= S_DIVW;
            end
            S_DIVW: if (dv_done) begin
                L  <= dv_div0 ? {NUM_W{1'b0}} : dv_quo;
                st <= S_NF0;
            end

            // ---- 3. eps ----
            S_NF0: begin
                ma   <= $signed({1'b0, L[PRE_B:0]});           // L <= T/2 < 2^PRE_B
                mb   <= $signed({{(PRE_B-7){1'b0}}, q_k});
                wcnt <= 3'd0;
                e_rel <= $signed({{(EW-DEN_W){1'b0}}, tr >> 1}) >>> q_rsh;
                e_abs <= $signed({{(EW-ACC_W-1){1'b0}}, eabs_rnd >> s0});
                st   <= S_NFW;
            end
            S_NFW: begin
                wcnt <= wcnt + 3'd1;
                if (wcnt == 3'd3) begin                          // product in mp3
                    e_nf <= (mp3 + 4) >>> 3;                     // round(L*k/8)
                    st   <= S_EPS1;
                end
            end
            S_EPS1: begin e_sum <= e_nf + e_rel;  st <= S_EPS2; end
            S_EPS2: begin e_sum <= e_sum + e_abs; st <= S_LD;   end

            // ---- 4. load ----
            S_LD: begin
                ld11 <= {{(EW-PRE_B-2){a[PRE_B+1]}}, a} + e_sum;
                ld22 <= {{(EW-PRE_B-2){b[PRE_B+1]}}, b} + e_sum;
                cx   <= {{(EW-PRE_B-2){c[PRE_B+1]}}, c};
                dx   <= {{(EW-PRE_B-2){d[PRE_B+1]}}, d};
                st   <= S_MX4;
            end

            // ---- 5. final scale ----
            S_MX4: begin
                m2a <= (ld11 > ld22) ? ld11 : ld22;
                m2b <= (ab_c > ab_d) ? ab_c : ab_d;
                st  <= S_MX5;
            end
            S_MX5: begin m2 <= (m2a > m2b) ? m2a : m2b;  st <= S_BL1; end
            S_BL1: begin
                bl <= bitlen(m2);
                zf <= (m2 == {EW{1'b0}});
                st <= S_RND1;
            end
            S_RND1: begin
                s1   <= (bl > TARGET_W-1) ? (bl - (TARGET_W-1)) : 8'd0;
                st   <= S_ADD1;
            end
            S_ADD1: begin
                rnd1 = (s1 == 8'd0) ? {EW{1'b0}} : ({{(EW-1){1'b0}}, 1'b1} << (s1 - 8'd1));
                u11 <= ld11 + rnd1;  u22 <= ld22 + rnd1;
                ure <= cx   + rnd1;  uim <= dx   + rnd1;
                st  <= S_SH1;
            end
            S_SH1: begin
                v11 <= u11 >>> s1;  v22 <= u22 >>> s1;
                vre <= ure >>> s1;  vim <= uim >>> s1;
                st  <= S_SAT;
            end
            S_SAT: begin
                v11 <= (v11 > MAXT) ? MAXT : ((v11 < MINT) ? MINT : v11);
                v22 <= (v22 > MAXT) ? MAXT : ((v22 < MINT) ? MINT : v22);
                vre <= (vre > MAXT) ? MAXT : ((vre < MINT) ? MINT : vre);
                vim <= (vim > MAXT) ? MAXT : ((vim < MINT) ? MINT : vim);
                st  <= S_OUT;
            end
            S_OUT: begin
                o11    <= v11[TARGET_W-1:0];
                o22    <= v22[TARGET_W-1:0];
                o12_re <= vre[TARGET_W-1:0];
                o12_im <= vim[TARGET_W-1:0];
                shift  <= s0 + s1;
                r_zero <= zf;
                coh_jam <= cj;
                strong_jam <= sj;
                coh_l2  <= cl2;
                nf_est <= (L[NUM_W-1:0] << s0) > {{(NUM_W){1'b0}}, {ACC_W{1'b1}}} ? {ACC_W{1'b1}}
                          : (L << s0);
                out_valid <= 1'b1;
                st     <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
