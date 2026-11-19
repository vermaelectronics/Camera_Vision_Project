// -----------------------------------------------------------------------------
// stap_lms.v  (v2)  --  space-time power inversion (2 antennas x L taps), NLMS
//
// WHY: one complex weight per antenna (beamformer_2ch) nulls a jammer only as
//   well as RX1 and RX2 see it as an exact scaled copy of each other. A CW
//   jammer is (one frequency); a WIDEBAND / barrage jammer is not: antenna
//   SAW/LNA, cable and AD9361 filter differences, and the array delay itself,
//   change the RX1/RX2 relation across the band. Field result: 56 MHz barrage,
//   coherence 0.47 -> 2.8 dB of null; 40 MHz -> 27 dB. A short FIR per antenna
//   (here L = 5 taps) equalises those differences, so the null holds over the
//   whole band, for CW and barrage alike.
//
// WHAT: generalised-sidelobe-canceller form of power inversion:
//     reference r(n)  = x1(n - C)                    (RX1 centre tap, C = (L-1)/2)
//     auxiliary u(n)  = { x1(n-k), k != C ;  x2(n-k), k = 0..L-1 }   (M = 2L-1)
//     output    y(n)  = r(n) - sum_m w_m u_m(n)
//     update    w_m  += mu * y(n) * conj(u_m(n)) / 2^pl          (normalised LMS)
//                       - w_m * 2^-LEAK                          (leakage)
//   Minimising E|y|^2 with the reference fixed is exactly power inversion,
//   now over space AND time. With no jammer the weights stay ~0 and y ~ r:
//   the satellites pass as on one antenna.
//   Adaptation is continuous (every sample): a barrage that fluctuates, a
//   swept or a pulsed jammer is tracked within ~2^MU_SH samples.
//
// FIXED POINT
//   x  : signed DW (= 15: the IQ-corrected 12-bit samples x4, as everywhere)
//   w  : accumulator WA = 36 bits Q8.27; filter operand Q4.20 (25 bits,
//        saturated at +/-16; the accumulator never wraps, see anti-windup)
//   y  : Q15 scaling, y = r*2^15 - sum(w*u)  -- the same format as
//        beamformer_2ch (y = x * 2^15 for unity weight), so output_scaler
//        takes it unchanged.
//   normalisation: 2^pl ~ ||u||^2, pl = floor(log2(max(P_avg, P_now, P_hold/8))) + 2,
//        P = |x1|^2 + |x2|^2 (the +2: 9 taps ~ 4.5 P), P_avg = IIR 2^-4,
//        P_hold = peak-hold of P_avg released 2^-17 per sample (keeps the
//        null through the OFF time of a pulsed jammer; cfg_hold_off = 1 off).
//        pl = h + h' with h = floor(pl/2). The two operands are normalised
//        separately, so the 9 complex products need only a FIXED shift:
//          un = sat17( u * 2^(15-h) )             (|un| < 2^15 at the power in use)
//          e  = sat25( y * 2^(24-h'-MU) )
//          dw = e * conj(un) >>> 12   = mu y conj(u) 2^(27-pl)   in Q27
//        |e| < 2^24 and |un| < 2^16 bound |dw| < 2^29: no saturation needed.
//        The input samples are normalised ONCE (4 shifters) into a second
//        tap line, 3 clocks behind the raw one, instead of 18 per-tap shifts.
//
// PIPELINE (every stage advances every clock; vf[j] = stage j holds a sample)
//   filter : 1 operands (w sat to Q4.20, u)  2 M  3 P  4 combine >>>5
//            5..8 adder tree  9 y = r<<15 - sum  10 output (sat to OUT_W)
//   update : 10..12 e (coarse shift, fine shift, sat)   13 operands (e, un)
//            14 M  15 P  16 combine = dw  17 anti-windup  18 dw - leak
//            accumulate on vf[18]
//   Delayed LMS: weights used by sample n are ~19 clocks old: <= 19 samples
//   at one sample per clock, ~5 at LVDS 2R2T (a sample every 4 clocks).
//   MU_SH = 5 is stable at both rates (CW and barrage simulated); 4 diverges
//   on a CW at one sample per clock, so the step is clamped to <= 2^-5.
//   Every product has its operands, M and P registered (DSP48E1 AREG/BREG,
//   MREG, PREG) for the 250 MHz l_clk the design is closed at. The slowly
//   varying normalisation (log2 of the power) has its own pipeline.
//
// COST (L = 5): 78 DSP48 (36 filter + 36 update + 4 power + 2 weight norm).
// L is fixed at 5 by the adder tree below.
// v2: normalisation split onto the operands (v1: 18 x 57-bit variable shifts
//     + saturation per tap: -5.8 ns at 250 MHz, ~7k LUT); accumulator 36 bits
//     with anti-windup instead of a saturating add; leak shifter on 20 bits;
//     LEAK fix: both ?: arms signed (an unsigned arm made >>> logical and
//     drove every negative weight further negative).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module stap_lms #(
    parameter DW    = 15,
    parameter L     = 5,
    parameter OUT_W = 35
)(
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     cfg_en,        // 1 = adapt; 0 = weights held at 0
    input  wire        [3:0]        cfg_mu_sh,     // step 2^-mu, 5..15 (default 5; < 5 acts as 5)
    input  wire        [4:0]        cfg_leak_sh,   // leakage 2^-n per update, 16..31 (1..15 act as 16), 0 = off
    input  wire                     cfg_clear,     // weights to 0 while set
    input  wire                     cfg_hold_off,  // 1 = no normalisation hold (see below)
    input  wire                     in_valid,
    input  wire signed [DW-1:0]     x1_i,
    input  wire signed [DW-1:0]     x1_q,
    input  wire signed [DW-1:0]     x2_i,
    input  wire signed [DW-1:0]     x2_q,
    output reg                      out_valid = 1'b0,
    output reg  signed [OUT_W-1:0]  y_re = 0,
    output reg  signed [OUT_W-1:0]  y_im = 0,
    output reg         [5:0]        st_pl = 6'd2,      // normalisation exponent in use
    output reg         [31:0]       st_wnorm = 32'd0,  // sum |w_m|^2, Q8.24 (unsigned)
    output reg         [31:0]       n_wsat = 32'd0     // updates limited by the anti-windup
);
    localparam M   = 2*L - 1;          // 9 auxiliary taps
    localparam C   = (L - 1) / 2;      // centre (reference) tap of RX1
    localparam WA  = 36;               // weight accumulator, Q8.27
    localparam WM  = 25;               // filter weight operand, Q4.20
    localparam PW  = WM + DW;          // 40-bit filter products (Q20)
    localparam CW  = PW + 1 - 5;       // combined product >>> 5 (Q15), 36 bits
    localparam SW  = CW + 4;           // tree sums, 40 bits
    localparam YW  = SW + 1;           // y (Q15), 41 bits
    localparam UN  = 17;               // normalised aux sample
    localparam EW  = 25;               // normalised error
    localparam GW  = EW + UN + 1;      // 43-bit update products sum

    integer k, j;
    genvar  g;

    // ------------------------------------------------------------------
    // raw taps (shift on in_valid); v0 marks the clock after a shift
    // ------------------------------------------------------------------
    reg signed [DW-1:0] t1r [0:L-1];
    reg signed [DW-1:0] t1i [0:L-1];
    reg signed [DW-1:0] t2r [0:L-1];
    reg signed [DW-1:0] t2i [0:L-1];
    reg                 v0 = 1'b0;
    integer ik1;
    initial for (ik1 = 0; ik1 < L; ik1 = ik1 + 1) begin
        t1r[ik1] = 0; t1i[ik1] = 0; t2r[ik1] = 0; t2i[ik1] = 0;
    end
    always @(posedge clk) begin
        v0 <= in_valid & ~rst;
        if (in_valid) begin
            t1r[0] <= x1_i;  t1i[0] <= x1_q;
            t2r[0] <= x2_i;  t2i[0] <= x2_q;
            for (k = 1; k < L; k = k + 1) begin
                t1r[k] <= t1r[k-1];  t1i[k] <= t1i[k-1];
                t2r[k] <= t2r[k-1];  t2i[k] <= t2i[k-1];
            end
        end
    end

    // auxiliary vector (wiring only)
    wire signed [DW-1:0] ur [0:M-1];
    wire signed [DW-1:0] ui [0:M-1];
    generate
        for (g = 0; g < M; g = g + 1) begin : g_aux
            if (g < L - 1) begin : g_x1
                localparam integer TI = (g < C) ? g : g + 1;
                assign ur[g] = t1r[TI];
                assign ui[g] = t1i[TI];
            end else begin : g_x2
                assign ur[g] = t2r[g - (L - 1)];
                assign ui[g] = t2i[g - (L - 1)];
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // normalisation exponent: pl = floor(log2(max(P_avg, P_now))) + 2 (via OR),
    // h = floor(pl/2), h' = pl - h, se = h' + MU - 9 (clamped at 0).
    // All slowly varying; every step registered.
    // ------------------------------------------------------------------
    localparam PWR = 2*DW + 2;     // 32 bits
    reg  [2*DW-1:0]     sq0 = 0, sq1 = 0, sq2 = 0, sq3 = 0;
    reg  [2*DW:0]       pa = 0, pb = 0;
    localparam HOLD_SH = 17;
    reg  [PWR-1:0]      p_now = 0, p_avg = 0, p_use = 0;   // p_use = p_now | p_avg | hold/8
    reg  [PWR-1:0]      p_hold = 0;
    reg                 ph_gt = 1'b0, ph_on = 1'b1;
    reg  signed [PWR:0] p_dlt = 0;
    reg                 pv1 = 1'b0, pv2 = 1'b0, pv3 = 1'b0, pv4 = 1'b0;
    reg  [3:0]          nzb = 4'd0;            // byte b of p_use non-zero
    reg  [2:0]          msb_b [0:3];           // msb position inside byte b
    reg  [4:0]          lz = 5'd0;             // floor(log2(p_use))
    (* max_fanout = 40 *) reg [5:0] h = 6'd1;
    reg  [5:0]          hp = 6'd1;
    (* max_fanout = 40 *) reg [5:0] se = 6'd0;   // error shift
    reg  [6:0]          se_raw = 7'd0;
    reg  [3:0]          mu_c = 4'd5;
    integer ib;
    initial for (ib = 0; ib < 4; ib = ib + 1) msb_b[ib] = 3'd0;

    function [2:0] msb8;                        // msb index of a byte (0 if zero)
        input [7:0] v;
        begin
            msb8 = v[7] ? 3'd7 : v[6] ? 3'd6 : v[5] ? 3'd5 : v[4] ? 3'd4 :
                   v[3] ? 3'd3 : v[2] ? 3'd2 : v[1] ? 3'd1 : 3'd0;
        end
    endfunction

    always @(posedge clk) begin
        pv1 <= in_valid;  pv2 <= pv1;  pv3 <= pv2;  pv4 <= pv3;
        sq0 <= x1_i * x1_i;  sq1 <= x1_q * x1_q;
        sq2 <= x2_i * x2_i;  sq3 <= x2_q * x2_q;
        pa  <= sq0 + sq1;    pb  <= sq2 + sq3;
        if (pv3) p_now <= pa + pb;
        // IIR 2^-4 in two steps (the step uses the p_avg of one clock earlier)
        p_dlt <= pv4 ? ($signed({1'b0, p_now >> 4}) - $signed({1'b0, p_avg >> 4}))
                     : {(PWR+1){1'b0}};
        p_avg <= p_avg + p_dlt[PWR-1:0];
        // HOLD: peak-hold of P_avg, released by 2^-HOLD_SH per sample. While a
        // jammer is OFF (pulsed / intermittent) the step is normalised by
        // >= its recent power / 8, so the weights barely move on noise alone
        // and the null is still there when it comes back. After the hold has
        // decayed (35 dB jammer: ~6 x 2^17 samples = 14 ms at 56 MSPS) STAP
        // adapts normally again. The compare uses the 24 MSBs, 1 clock late.
        ph_gt <= (p_avg[PWR-1:8] > p_hold[PWR-1:8]);
        if (ph_gt)    p_hold <= p_avg;
        else if (pv4) p_hold <= p_hold - (p_hold >> HOLD_SH);
        ph_on <= ~cfg_hold_off;
        // floor(log2(a | b | c)) == max of the three floor(log2): no compare
        p_use <= p_now | p_avg | (ph_on ? (p_hold >> 3) : {PWR{1'b0}});
        for (ib = 0; ib < 4; ib = ib + 1) begin
            nzb[ib]   <= |p_use[8*ib +: 8];
            msb_b[ib] <= msb8(p_use[8*ib +: 8]);
        end
        lz <= nzb[3] ? {2'd3, msb_b[3]} : nzb[2] ? {2'd2, msb_b[2]} :
              nzb[1] ? {2'd1, msb_b[1]} : {2'd0, msb_b[0]};
        st_pl  <= {1'b0, lz} + 6'd2;
        h      <= ({1'b0, lz} + 6'd2) >> 1;
        hp     <= st_pl - h;
        // step clamped to 2^-5 or smaller: with the ~19-sample update delay at
        // one sample per clock, 2^-4 is unstable on a CW jammer (simulated)
        mu_c   <= (cfg_mu_sh < 4'd5) ? 4'd5 : cfg_mu_sh;
        se_raw <= {1'b0, hp} + {3'd0, mu_c};
        se     <= (se_raw < 7'd9) ? 6'd0 : se_raw[5:0] - 6'd9;
    end

    // ------------------------------------------------------------------
    // normalised samples: xn = sat17( x * 2^(15-h) ), 3 stages, then a
    // second tap line (3 clocks behind the raw one)
    // ------------------------------------------------------------------
    localparam XS = DW + 15;                        // x << 15, 30 bits
    reg signed [XS-1:0] na [0:3];                   // coarse (multiples of 8)
    reg signed [XS-1:0] nb [0:3];                   // fine
    reg signed [UN-1:0] nc [0:3];                   // saturated
    reg  [2:0]          h_f = 3'd0;
    reg                 nva = 1'b0, nvb = 1'b0, nvc = 1'b0;
    integer ik2;
    initial for (ik2 = 0; ik2 < 4; ik2 = ik2 + 1) begin na[ik2] = 0; nb[ik2] = 0; nc[ik2] = 0; end

    function signed [UN-1:0] sat_un;
        input signed [XS-1:0] v;
        begin
            if (v[XS-1:UN-1] == {(XS-UN+1){1'b0}} || v[XS-1:UN-1] == {(XS-UN+1){1'b1}})
                sat_un = v[UN-1:0];
            else
                sat_un = v[XS-1] ? {1'b1, {(UN-2){1'b0}}, 1'b1} : {1'b0, {(UN-1){1'b1}}};
        end
    endfunction

    wire signed [DW-1:0] xin [0:3];
    assign xin[0] = x1_i;  assign xin[1] = x1_q;
    assign xin[2] = x2_i;  assign xin[3] = x2_q;
    // h_f is loaded on the same clock as na, so nb completes the shift na
    // started with the same h
    always @(posedge clk) begin
        nva <= in_valid;  nvb <= nva;  nvc <= nvb;
        h_f <= h[2:0];
        for (k = 0; k < 4; k = k + 1) begin
            na[k] <= $signed({xin[k], 15'd0}) >>> {h[4:3], 3'b000};
            nb[k] <= na[k] >>> h_f;
            nc[k] <= sat_un(nb[k]);
        end
    end

    reg signed [UN-1:0] n1r [0:L-1];
    reg signed [UN-1:0] n1i [0:L-1];
    reg signed [UN-1:0] n2r [0:L-1];
    reg signed [UN-1:0] n2i [0:L-1];
    integer ik3;
    initial for (ik3 = 0; ik3 < L; ik3 = ik3 + 1) begin
        n1r[ik3] = 0; n1i[ik3] = 0; n2r[ik3] = 0; n2i[ik3] = 0;
    end
    always @(posedge clk) begin
        if (nvc) begin
            n1r[0] <= nc[0];  n1i[0] <= nc[1];
            n2r[0] <= nc[2];  n2i[0] <= nc[3];
            for (k = 1; k < L; k = k + 1) begin
                n1r[k] <= n1r[k-1];  n1i[k] <= n1i[k-1];
                n2r[k] <= n2r[k-1];  n2i[k] <= n2i[k-1];
            end
        end
    end
    wire signed [UN-1:0] nur [0:M-1];
    wire signed [UN-1:0] nui [0:M-1];
    generate
        for (g = 0; g < M; g = g + 1) begin : g_naux
            if (g < L - 1) begin : g_x1
                localparam integer TI = (g < C) ? g : g + 1;
                assign nur[g] = n1r[TI];
                assign nui[g] = n1i[TI];
            end else begin : g_x2
                assign nur[g] = n2r[g - (L - 1)];
                assign nui[g] = n2i[g - (L - 1)];
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // weights (accumulators)
    // ------------------------------------------------------------------
    reg signed [WA-1:0] war [0:M-1];
    reg signed [WA-1:0] wai [0:M-1];
    integer ik4;
    initial for (ik4 = 0; ik4 < M; ik4 = ik4 + 1) begin war[ik4] = 0; wai[ik4] = 0; end

    // Q8.27 (36 bits) -> Q4.20 (25 bits), saturated at +/-16
    function signed [WM-1:0] sat_wm;
        input signed [WA-1:0] v;
        begin
            if (v[WA-1:31] == 5'b00000 || v[WA-1:31] == 5'b11111)
                sat_wm = v[31:7];
            else
                sat_wm = v[WA-1] ? {1'b1, {(WM-2){1'b0}}, 1'b1} : {1'b0, {(WM-1){1'b1}}};
        end
    endfunction

    // ------------------------------------------------------------------
    // FILTER
    // ------------------------------------------------------------------
    reg signed [WM-1:0] fa_r [0:M-1];
    reg signed [WM-1:0] fa_i [0:M-1];
    reg signed [DW-1:0] fb_r [0:M-1];
    reg signed [DW-1:0] fb_i [0:M-1];
    reg signed [PW-1:0] m_rr [0:M-1], m_ii [0:M-1], m_ri [0:M-1], m_ir [0:M-1];
    reg signed [PW-1:0] p_rr [0:M-1], p_ii [0:M-1], p_ri [0:M-1], p_ir [0:M-1];
    reg signed [CW-1:0] c_r [0:M-1];
    reg signed [CW-1:0] c_i [0:M-1];
    // (shreg_extract = "no": an SRL's clock-to-out is ~1.3 ns vs 0.4 ns for a
    //  flop; these registers feed long adders or have a high fan-out)
    (* shreg_extract = "no" *) reg signed [SW-1:0] s1r [0:4], s1i [0:4];
    (* shreg_extract = "no" *) reg signed [SW-1:0] s2r [0:2], s2i [0:2];
    (* shreg_extract = "no" *) reg signed [SW-1:0] s3r [0:1], s3i [0:1];
    reg signed [SW-1:0] s4r = 0, s4i = 0;
    reg signed [YW-1:0] yfr = 0, yfi = 0;
    (* shreg_extract = "no" *) reg signed [DW-1:0] rr_d [1:8];
    (* shreg_extract = "no" *) reg signed [DW-1:0] ri_d [1:8];
    (* shreg_extract = "no" *) reg [18:1] vf = 18'd0;
    reg signed [PW:0]   c_r_full, c_i_full;
    integer ik5;
    initial begin
        for (ik5 = 0; ik5 < M; ik5 = ik5 + 1) begin
            fa_r[ik5] = 0; fa_i[ik5] = 0; fb_r[ik5] = 0; fb_i[ik5] = 0;
            m_rr[ik5] = 0; m_ii[ik5] = 0; m_ri[ik5] = 0; m_ir[ik5] = 0;
            p_rr[ik5] = 0; p_ii[ik5] = 0; p_ri[ik5] = 0; p_ir[ik5] = 0;
            c_r[ik5] = 0; c_i[ik5] = 0;
        end
        for (ik5 = 0; ik5 < 5; ik5 = ik5 + 1) begin s1r[ik5] = 0; s1i[ik5] = 0; end
        for (ik5 = 0; ik5 < 3; ik5 = ik5 + 1) begin s2r[ik5] = 0; s2i[ik5] = 0; end
        for (ik5 = 0; ik5 < 2; ik5 = ik5 + 1) begin s3r[ik5] = 0; s3i[ik5] = 0; end
        for (ik5 = 1; ik5 <= 8; ik5 = ik5 + 1) begin rr_d[ik5] = 0; ri_d[ik5] = 0; end
    end

    always @(posedge clk) begin
        for (k = 0; k < M; k = k + 1) begin
            // f1: operands
            fa_r[k] <= sat_wm(war[k]);
            fa_i[k] <= sat_wm(wai[k]);
            fb_r[k] <= ur[k];
            fb_i[k] <= ui[k];
            // f2 M, f3 P
            m_rr[k] <= fa_r[k] * fb_r[k];
            m_ii[k] <= fa_i[k] * fb_i[k];
            m_ri[k] <= fa_r[k] * fb_i[k];
            m_ir[k] <= fa_i[k] * fb_r[k];
            p_rr[k] <= m_rr[k];  p_ii[k] <= m_ii[k];
            p_ri[k] <= m_ri[k];  p_ir[k] <= m_ir[k];
            // f4: complex combine, Q20 -> Q15
            c_r_full = p_rr[k] - p_ii[k];
            c_i_full = p_ri[k] + p_ir[k];
            c_r[k]  <= c_r_full >>> 5;
            c_i[k]  <= c_i_full >>> 5;
        end
        s1r[0] <= c_r[0] + c_r[1];  s1i[0] <= c_i[0] + c_i[1];
        s1r[1] <= c_r[2] + c_r[3];  s1i[1] <= c_i[2] + c_i[3];
        s1r[2] <= c_r[4] + c_r[5];  s1i[2] <= c_i[4] + c_i[5];
        s1r[3] <= c_r[6] + c_r[7];  s1i[3] <= c_i[6] + c_i[7];
        s1r[4] <= c_r[8];           s1i[4] <= c_i[8];
        s2r[0] <= s1r[0] + s1r[1];  s2i[0] <= s1i[0] + s1i[1];
        s2r[1] <= s1r[2] + s1r[3];  s2i[1] <= s1i[2] + s1i[3];
        s2r[2] <= s1r[4];           s2i[2] <= s1i[4];
        s3r[0] <= s2r[0] + s2r[1];  s3i[0] <= s2i[0] + s2i[1];
        s3r[1] <= s2r[2];           s3i[1] <= s2i[2];
        s4r    <= s3r[0] + s3r[1];  s4i    <= s3i[0] + s3i[1];

        rr_d[1] <= t1r[C];  ri_d[1] <= t1i[C];
        for (k = 2; k <= 8; k = k + 1) begin rr_d[k] <= rr_d[k-1]; ri_d[k] <= ri_d[k-1]; end
        // f9: y = r * 2^15 - sum
        yfr <= ($signed({{(YW-DW){rr_d[8][DW-1]}}, rr_d[8]}) <<< 15) - s4r;
        yfi <= ($signed({{(YW-DW){ri_d[8][DW-1]}}, ri_d[8]}) <<< 15) - s4i;

        vf <= {vf[17:1], v0};
    end

    // f10: output, saturated to OUT_W
    function signed [OUT_W-1:0] sat_out;
        input signed [YW-1:0] v;
        begin
            if (v[YW-1:OUT_W-1] == {(YW-OUT_W+1){1'b0}} || v[YW-1:OUT_W-1] == {(YW-OUT_W+1){1'b1}})
                sat_out = v[OUT_W-1:0];
            else
                sat_out = v[YW-1] ? {1'b1, {(OUT_W-2){1'b0}}, 1'b1} : {1'b0, {(OUT_W-1){1'b1}}};
        end
    endfunction

    always @(posedge clk) begin
        out_valid <= vf[9] & ~rst;
        y_re      <= sat_out(yfr);
        y_im      <= sat_out(yfi);
    end

    // ------------------------------------------------------------------
    // UPDATE
    // ------------------------------------------------------------------
    // normalised aux vector carried alongside: udr[j] = the un of the sample
    // in stage j. The normalised taps are 3 clocks behind the raw ones, so
    // they enter at stage 4 and reach stage 12, from where the update
    // operands (stage 13) are loaded together with that sample's e.
    // No reset on this pipeline: Vivado maps it to SRLs.
    (* shreg_extract = "yes" *) reg signed [UN-1:0] udr [4:12][0:M-1];
    (* shreg_extract = "yes" *) reg signed [UN-1:0] udi [4:12][0:M-1];
    integer ik6, ij6;
    initial for (ij6 = 4; ij6 <= 12; ij6 = ij6 + 1) for (ik6 = 0; ik6 < M; ik6 = ik6 + 1) begin
        udr[ij6][ik6] = 0; udi[ij6][ik6] = 0;
    end
    always @(posedge clk) begin
        for (k = 0; k < M; k = k + 1) begin
            udr[4][k] <= nur[k];
            udi[4][k] <= nui[k];
            for (j = 5; j <= 12; j = j + 1) begin
                udr[j][k] <= udr[j-1][k];
                udi[j][k] <= udi[j-1][k];
            end
        end
    end

    // stages 10..12: e = sat25( y >>> se )   (y in Q15: e = y * 2^(24-h'-mu))
    reg signed [YW-1:0] e1r = 0, e1i = 0, e2r = 0, e2i = 0;
    reg signed [EW-1:0] er = 0, ei = 0;
    reg  [2:0]          se_f = 3'd0;
    function signed [EW-1:0] sat_e;
        input signed [YW-1:0] v;
        begin
            if (v[YW-1:EW-1] == {(YW-EW+1){1'b0}} || v[YW-1:EW-1] == {(YW-EW+1){1'b1}})
                sat_e = v[EW-1:0];
            else
                sat_e = v[YW-1] ? {1'b1, {(EW-2){1'b0}}, 1'b1} : {1'b0, {(EW-1){1'b1}}};
        end
    endfunction
    always @(posedge clk) begin
        se_f <= se[2:0];
        e1r  <= yfr >>> {se[5:3], 3'b000};
        e1i  <= yfi >>> {se[5:3], 3'b000};
        e2r  <= e1r >>> se_f;
        e2i  <= e1i >>> se_f;
        er   <= sat_e(e2r);
        ei   <= sat_e(e2i);
    end

    // stage 13 operands, 14 M, 15 P, 16 combine: g = e * conj(un)
    reg signed [EW-1:0]    ga_r [0:M-1], ga_i [0:M-1];
    reg signed [UN-1:0]    gb_r [0:M-1], gb_i [0:M-1];
    reg signed [EW+UN-1:0] n_rr [0:M-1], n_ii [0:M-1], n_ri [0:M-1], n_ir [0:M-1];
    reg signed [EW+UN-1:0] q_rr [0:M-1], q_ii [0:M-1], q_ri [0:M-1], q_ir [0:M-1];
    reg signed [GW-1:0]    gr [0:M-1], gi [0:M-1];
    integer ik7;
    initial for (ik7 = 0; ik7 < M; ik7 = ik7 + 1) begin
        ga_r[ik7] = 0; ga_i[ik7] = 0; gb_r[ik7] = 0; gb_i[ik7] = 0;
        n_rr[ik7] = 0; n_ii[ik7] = 0; n_ri[ik7] = 0; n_ir[ik7] = 0;
        q_rr[ik7] = 0; q_ii[ik7] = 0; q_ri[ik7] = 0; q_ir[ik7] = 0;
        gr[ik7] = 0; gi[ik7] = 0;
    end
    always @(posedge clk) begin
        for (k = 0; k < M; k = k + 1) begin
            ga_r[k] <= er;            ga_i[k] <= ei;
            gb_r[k] <= udr[12][k];    gb_i[k] <= udi[12][k];
            n_rr[k] <= ga_r[k] * gb_r[k];
            n_ii[k] <= ga_i[k] * gb_i[k];
            n_ri[k] <= ga_r[k] * gb_i[k];
            n_ir[k] <= ga_i[k] * gb_r[k];
            q_rr[k] <= n_rr[k];  q_ii[k] <= n_ii[k];
            q_ri[k] <= n_ri[k];  q_ir[k] <= n_ir[k];
            // e * conj(un) = (er ur + ei ui) + j (ei ur - er ui)
            gr[k]   <= q_rr[k] + q_ii[k];
            gi[k]   <= q_ir[k] - q_ri[k];
        end
    end

    // ------------------------------------------------------------------
    // stages 17/18: d = (g >>> 12) with anti-windup, minus leak; accumulate on vf[18]
    //   leak = w >>> LEAK (LEAK 16..31) from the weight 3 clocks earlier.
    //   anti-windup: a weight beyond the filter operand range (+/-16) takes
    //   no step that would move it further out (flag 2 clocks old: at most 3
    //   steps of < 4.0 overshoot, so the 36-bit (+/-256) accumulator can never
    //   wrap); the leak still pulls it back.
    // ------------------------------------------------------------------
    localparam LKW = WA - 16;                   // leak operand, 20 bits
    localparam DGW = GW - 12;                   // g >>> 12, 31 bits
    reg signed [LKW-1:0] lwr [0:M-1], lwi [0:M-1];
    reg signed [LKW-1:0] lcr [0:M-1], lci [0:M-1];   // coarse (x4) leak shift
    reg signed [LKW-1:0] lkr [0:M-1], lki [0:M-1];
    reg  [M-1:0]         bigr = {M{1'b0}}, bigi = {M{1'b0}};
    reg  [M-1:0]         sgr = {M{1'b0}}, sgi = {M{1'b0}};
    reg signed [DGW-1:0] dsr [0:M-1], dsi [0:M-1];   // stage 17: step after anti-windup
    reg signed [WA-1:0]  ddr [0:M-1], ddi [0:M-1];   // stage 18: step - leak
    reg  [M-1:0]         awr = {M{1'b0}}, awi = {M{1'b0}};
    (* max_fanout = 40 *) reg       leak_on = 1'b0;
    (* max_fanout = 40 *) reg [3:0] leak_lo = 4'd2;
    integer ik8;
    initial for (ik8 = 0; ik8 < M; ik8 = ik8 + 1) begin
        lwr[ik8] = 0; lwi[ik8] = 0; lkr[ik8] = 0; lki[ik8] = 0; ddr[ik8] = 0; ddi[ik8] = 0;
        lcr[ik8] = 0; lci[ik8] = 0;
        dsr[ik8] = 0; dsi[ik8] = 0;
    end
    wire signed [DGW-1:0] dgr [0:M-1];
    wire signed [DGW-1:0] dgi [0:M-1];
    wire        [M-1:0]   stopr, stopi;
    generate
        for (g = 0; g < M; g = g + 1) begin : g_dg
            assign dgr[g]   = gr[g][GW-1:12];
            assign dgi[g]   = gi[g][GW-1:12];
            assign stopr[g] = bigr[g] && (dgr[g][DGW-1] == sgr[g]);
            assign stopi[g] = bigi[g] && (dgi[g][DGW-1] == sgi[g]);
        end
    endgenerate

    always @(posedge clk) begin
        leak_on <= (cfg_leak_sh != 5'd0);
        leak_lo <= cfg_leak_sh[4] ? cfg_leak_sh[3:0] : 4'd0;
        for (k = 0; k < M; k = k + 1) begin
            lwr[k]  <= war[k][WA-1:16];          // w >>> 16 (wiring)
            lwi[k]  <= wai[k][WA-1:16];
            // two stages (multiples of 4, then 0..3); both ?: arms signed, or
            // >>> becomes a logical shift
            lcr[k]  <= lwr[k] >>> {leak_lo[3:2], 2'b00};
            lci[k]  <= lwi[k] >>> {leak_lo[3:2], 2'b00};
            lkr[k]  <= leak_on ? (lcr[k] >>> leak_lo[1:0]) : $signed({LKW{1'b0}});
            lki[k]  <= leak_on ? (lci[k] >>> leak_lo[1:0]) : $signed({LKW{1'b0}});
            bigr[k] <= ~(war[k][WA-1:31] == 5'b00000 || war[k][WA-1:31] == 5'b11111);
            bigi[k] <= ~(wai[k][WA-1:31] == 5'b00000 || wai[k][WA-1:31] == 5'b11111);
            sgr[k]  <= war[k][WA-1];
            sgi[k]  <= wai[k][WA-1];
            awr[k]  <= stopr[k] & vf[16];        // only for a real sample
            awi[k]  <= stopi[k] & vf[16];
            dsr[k]  <= stopr[k] ? $signed({DGW{1'b0}}) : dgr[k];
            dsi[k]  <= stopi[k] ? $signed({DGW{1'b0}}) : dgi[k];
            ddr[k]  <= {{(WA-DGW){dsr[k][DGW-1]}}, dsr[k]} - {{(WA-LKW){lkr[k][LKW-1]}}, lkr[k]};
            ddi[k]  <= {{(WA-DGW){dsi[k][DGW-1]}}, dsi[k]} - {{(WA-LKW){lki[k][LKW-1]}}, lki[k]};
        end
    end

    // update strobe and clear, one register copy per tap (fan-out 72 each)
    (* shreg_extract = "no", max_fanout = 80 *) reg [M-1:0] upd  = {M{1'b0}};
    (* shreg_extract = "no", max_fanout = 80 *) reg [M-1:0] wclr = {M{1'b1}};
    reg wclr0 = 1'b1;
    always @(posedge clk) begin
        upd   <= {M{vf[17]}};                // = vf[18]: ddr holds a sample's step
        wclr0 <= rst | cfg_clear | ~cfg_en;
        wclr  <= {M{wclr0}};
    end
    always @(posedge clk) begin
        for (k = 0; k < M; k = k + 1) begin
            if (wclr[k]) begin
                war[k] <= {WA{1'b0}};
                wai[k] <= {WA{1'b0}};
            end else if (upd[k]) begin
                war[k] <= war[k] + ddr[k];
                wai[k] <= wai[k] + ddi[k];
            end
        end
    end

    // anti-windup events (one count per update that was limited)
    reg aw_ev = 1'b0;
    always @(posedge clk) begin
        aw_ev <= |{awr, awi};                // stage 18
        if (rst) n_wsat <= 32'd0;
        else if (aw_ev) n_wsat <= n_wsat + 32'd1;
    end

    // ------------------------------------------------------------------
    // weight norm sum |w|^2 (status): one weight per clock, Q4.13 operands
    // taken from the weight accumulators (Q8.27 -> Q4.13, saturated), NOT
    // from fa_r: fa_r then feeds only its two filter DSPs (Vivado: -0.001 ns
    // with the extra load).
    // v2.1 (Vivado 2026-11-02: -1.5 ns, the worst path of the design): every
    // step has its own register -- index mux, DSP square (M and P registers),
    // two-term add, accumulate, capture, saturate. Status only: latency free.
    // ------------------------------------------------------------------
    function signed [17:0] sat_wq;                // Q8.27 -> Q4.13, +/-16
        input signed [WA-1:0] v;
        begin
            if (v[WA-1:31] == 5'b00000 || v[WA-1:31] == 5'b11111)
                sat_wq = v[31:14];
            else
                sat_wq = v[WA-1] ? 18'sh20001 : 18'sh1FFFF;
        end
    endfunction
    reg [3:0]          wn_idx = 4'd0;
    reg signed [WA-1:0] wsel_r = 0, wsel_i = 0;    // 9:1 mux of the accumulators
    reg signed [17:0]  wq_r = 0, wq_i = 0;
    reg [35:0]         wsq_r = 0, wsq_i = 0;       // DSP M
    reg [35:0]         wsp_r = 0, wsp_i = 0;       // DSP P
    reg [34:0]         wsum = 0;                   // Q8.24, one weight
    reg [38:0]         wacc = 0;                   // Q8.24, sum of 9
    reg [38:0]         wn_tot = 0;
    reg                wn_big = 1'b0;
    reg [5:1]          wn_first = 5'd0;            // weight 0 at stage n
    reg                wn_cap = 1'b0;
    always @(posedge clk) begin
        wn_idx   <= (wn_idx == M - 1) ? 4'd0 : wn_idx + 4'd1;
        wsel_r   <= war[wn_idx];                              // 1
        wsel_i   <= wai[wn_idx];
        wq_r     <= sat_wq(wsel_r);                           // 2: Q4.13
        wq_i     <= sat_wq(wsel_i);
        wsq_r    <= wq_r * wq_r;                              // 3: Q8.26
        wsq_i    <= wq_i * wq_i;
        wsp_r    <= wsq_r;                                    // 4
        wsp_i    <= wsq_i;
        wsum     <= wsp_r[35:2] + wsp_i[35:2];                // 5: Q8.24
        wn_first <= {wn_first[4:1], (wn_idx == 4'd0)};
        // 6: wsum holds weight 0 -> wacc holds the complete previous set
        wacc     <= (wn_first[5] ? 39'd0 : wacc) + {4'd0, wsum};
        wn_cap   <= wn_first[5];
        if (wn_first[5]) wn_tot <= wacc;
        // 6/7: saturate to 32 bits in two steps
        wn_big   <= |wn_tot[38:32];
        if (wn_cap) st_wnorm <= st_wnorm;                     // (hold while wn_tot loads)
        else        st_wnorm <= wn_big ? 32'hFFFFFFFF : wn_tot[31:0];
    end

endmodule
