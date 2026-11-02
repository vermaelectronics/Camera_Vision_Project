// ============================================================================
// pi_aj_notch.v  --  PI-AJ v5 adaptive complex notch, one stage, 2 channels
//
//   y[n] = x[n] - a*x[n-1] + rho*a*y[n-1]          (direct form I)
//
//   zero on the unit circle at angle(a), pole at radius rho = 1-2^-rho_sh.
//   3 dB width ~ fs*2^-rho_sh/pi  (90 kHz at 18 MS/s, rho_sh=6 default;
//   wide enough to also take the phase-noise skirt of a cheap CW jammer).
//
//   ONE coefficient a is applied to BOTH channels. The notch is the same
//   linear filter on RX1 and RX2, so the inter-channel (spatial) relationship
//   that the following power-inversion stage relies on is preserved.
//
//   MODE 0 ("estimator"): a tracks the dominant spectral line of channel 0
//     r1 = < x0[n] x0*[n-1] >      (lag-1 autocorrelation, 2^-r_sh smoothing)
//     a  <- a + norm(r1)*2^-a_sh   (norm = power-of-two, no divider)
//     a  <- a*(1.5-0.5|a|^2)       (Newton step back to |a|=1, no sqrt)
//     The stage switches itself in only when it removes power:
//       ON  if P_in > 1.25 P_out,  OFF if P_in < 1.125 P_out
//     ext_act = inhibit (used to keep notch #2 off notch #1's frequency).
//   MODE 1 ("image"): a = conj(ext_a), active = ext_act. Placed after a mode-0
//     stage it removes the I/Q-imbalance image of the CW jammer (mirror freq).
//
//   Timing (rev 3): every multiplier has registered operands and a registered
//   product, and each carry chain has its own clock cycle. The coefficient
//   and rho*coefficient are registered copies (a moves by ~1e-5 per sample,
//   so using it a sample later changes nothing). a*x[n] is formed as soon as
//   x[n] arrives and is used by sample n+1, so the per-sample recursion is
//   only  y[n-1] -> rho*a*y[n-1] (DSP48) -> add + saturate -> y[n].
//
//   Needs >= 2 clock cycles between in_valid pulses.
// ============================================================================
`timescale 1ns / 1ps
module pi_aj_notch #(
    parameter DW   = 18,          // data width
    parameter MODE = 0            // 0 = self-estimating, 1 = conj(ext_a)
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 clear,      // re-acquire (a=1, r1=0)
    input  wire                 en,
    input  wire                 force_on,
    input  wire                 freeze,
    input  wire [3:0]           rho_sh,
    input  wire [3:0]           r_sh,
    input  wire [3:0]           a_sh,
    input  wire signed [17:0]   ext_a_re, ext_a_im,
    input  wire                 ext_act,
    input  wire                 in_valid,
    input  wire signed [DW-1:0] x0_re, x0_im, x1_re, x1_im,
    output reg                  out_valid,
    output reg  signed [DW-1:0] y0_re, y0_im, y1_re, y1_im,
    output wire signed [17:0]   a_re_o, a_im_o,
    output wire                 active,
    output reg  [31:0]          pin_o, pout_o
);
    localparam YW = DW + 2;                      // internal recursion width
    localparam RF = 16;                          // r1 fraction bits
    localparam RW = 2*DW + 2 + RF;               // r1 width
    localparam PF = 12;                          // power fraction bits
    localparam PW = 2*DW + 2 + PF;
    localparam SW = 2*DW + 9;                    // recursion sum width

    // saturation by checking that the dropped bits all equal the sign
    function signed [YW-1:0] sat_y;
        input signed [SW-1:0] v;
        if ((v[SW-1:YW-1] == 0) || (&v[SW-1:YW-1])) sat_y = v[YW-1:0];
        else if (v[SW-1])                           sat_y = {1'b1, {(YW-1){1'b0}}};
        else                                        sat_y = {1'b0, {(YW-1){1'b1}}};
    endfunction
    // straight to DW bits (same result as saturating to YW, then to DW)
    function signed [DW-1:0] sat_dw;
        input signed [SW-1:0] v;
        if ((v[SW-1:DW-1] == 0) || (&v[SW-1:DW-1])) sat_dw = v[DW-1:0];
        else if (v[SW-1])                           sat_dw = {1'b1, {(DW-1){1'b0}}};
        else                                        sat_dw = {1'b0, {(DW-1){1'b1}}};
    endfunction

    // leading one in two clocked steps (same result as a priority encoder,
    // 0 when v == 0):  therm(): per 8-bit group, "non-zero" + index of its
    // top 1;  th_pos(): top non-zero group, then that group's index.
    // Both are about 2 LUT levels; pure AND/OR logic.
    function [2:0] top8;                     // index of the top 1 of 8 bits
        input [7:0] b;
        reg h2, h1;
        begin
            h2 = |b[7:4];
            h1 = h2 ? |b[7:6] : |b[3:2];
            top8 = {h2, h1, h2 ? (h1 ? b[7] : b[5]) : (h1 ? b[3] : b[1])};
        end
    endfunction
    function [31:0] therm;                   // {idx7..idx0, nz7..nz0}
        input [RW-1:0] v;
        reg [63:0] x;
        integer j;
        begin
            x = v;
            for (j = 0; j < 8; j = j + 1) begin
                therm[j]           = |x[8*j +: 8];
                therm[8+3*j +: 3]  = top8(x[8*j +: 8]);
            end
        end
    endfunction
    function [6:0] th_pos;
        input [31:0] t;
        reg [2:0] q;
        begin
            q = top8(t[7:0]);
            th_pos = {q, t[8+3*q +: 3]};
        end
    endfunction

    // ---------------------------------------------------------------- coef
    // the estimator loop runs in Q2.24 with round-to-nearest (a truncating
    // Q16 loop is biased by ~-0.5 LSB/step, i.e. several kHz of notch error)
    localparam signed [26:0] ONE24 = 27'sd16777216;
    reg  signed [26:0] a24_re, a24_im;
    wire signed [26:0] a24r_re = a24_re + 27'sd128;
    wire signed [26:0] a24r_im = a24_im + 27'sd128;
    // registered Q16 coefficient and rho*coefficient
    reg  signed [17:0] a_re, a_im, ra_re, ra_im;
    assign a_re_o = a_re;
    assign a_im_o = a_im;
    always @(posedge clk) begin
        if (rst) begin
            a_re <= 18'sd65536; a_im <= 0; ra_re <= 0; ra_im <= 0;
        end else begin
            a_re  <= (MODE == 1) ? ext_a_re  : a24r_re[25:8];
            a_im  <= (MODE == 1) ? -ext_a_im : a24r_im[25:8];
            ra_re <= a_re - (a_re >>> rho_sh);
            ra_im <= a_im - (a_im >>> rho_sh);
        end
    end

    // ---------------------------------------------------------------- state
    reg signed [DW-1:0] xr0_re, xr0_im, xr1_re, xr1_im;        // x[n]
    reg signed [DW-1:0] xp0_re, xp0_im;                        // x0[n-1]
    reg signed [YW-1:0] yp0_re, yp0_im, yp1_re, yp1_im;        // y[n-1]
    reg v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14;

    // a*x[n]: formed in S1 of sample n, used in S1 of sample n+1
    reg signed [2*DW+2:0] ma0_re, ma0_im, ma1_re, ma1_im;
    reg signed [2*DW+6:0] mr0_re, mr0_im, mr1_re, mr1_im;      // rho*a*y[n-1]
    reg signed [SW-1:0]   xm0_re, xm0_im, xm1_re, xm1_im;      // x[n]<<16 + rnd - a x[n-1]
    reg signed [2*DW:0]   pr_re, pr_im;                        // x0[n] x0*[n-1]
    reg signed [DW-1:0]   xd0_re, xd0_im, xd1_re, xd1_im;

    localparam signed [SW-1:0] RND = 1 <<< 15;
    wire signed [SW-1:0] yn0_re = (xm0_re + mr0_re) >>> 16;
    wire signed [SW-1:0] yn0_im = (xm0_im + mr0_im) >>> 16;
    wire signed [SW-1:0] yn1_re = (xm1_re + mr1_re) >>> 16;
    wire signed [SW-1:0] yn1_im = (xm1_im + mr1_im) >>> 16;

    // r1 / power estimators
    reg signed [RW-1:0] r1_re, r1_im, dr_re, dr_im;
    reg signed [PW:0]   pin, pout;          // always >= 0
    reg signed [PW:0]   px_q, py_q;
    reg signed [PW:0]   pin_q, th_on, th_off;
    reg                 act;
    reg signed [DW-1:0] xs_re, xs_im;      // x0[n] for power
    reg signed [DW-1:0] ys_re, ys_im;      // y0[n] for power (18 bit: one DSP48)

    // estimator, feed-forward part
    reg [RW-1:0]        r1a_re, r1a_im;    // |r1|
    reg signed [RW-1:0] r1s_re, r1s_im, r1q_re, r1q_im, r1n_re, r1n_im;
    reg [RW-1:0]        r1_mx;
    reg [31:0]          r1_th;
    reg signed [RW-1:0] r1t_re, r1t_im;
    reg [6:0]           r1_pos, pos_n, pos_r, pos_s, pos_d, pos_t, pos_m, pos_e;
    reg signed [27:0]   nr8_re, nr8_im;    // norm(r1), Q24
    reg signed [27:0]   nrr_re, nrr_im;    // + rounding constant
    reg signed [27:0]   ds_re, ds_im;      // shifted
    reg signed [27:0]   d_re, d_im, d128_re, d128_im;
    // estimator loop (4 clocks, as in rev 1/2): a+d -> |a|^2 -> eps -> a(1+eps)
    reg signed [27:0]   at_re, at_im, atm_re, atm_im, ate_re, ate_im;
    reg signed [17:0]   at_hi_re, at_hi_im, ath_re, ath_im, ate_hi_re, ate_hi_im;
    reg signed [37:0]   at_mag;            // |a|^2, Q32 (from an 18-bit a)
    reg signed [17:0]   t_eps;             // (1 - |a|^2)/2, Q24, clamped
    wire signed [38:0]  eps_w = ($signed(39'sd4294967296) - at_mag) >>> 9;
    wire signed [27:0]  at_s_re = a24_re + d_re;
    wire signed [27:0]  at_s_im = a24_im + d_im;
    wire signed [27:0]  at_r_re = a24_re + d128_re;   // +128: rounding to Q16
    wire signed [27:0]  at_r_im = a24_im + d128_im;

    // normalised r1 in Q16 with its largest component in [1,2)
    wire signed [RW-1:0] nr_re = (r1_pos >= 16) ? (r1n_re >>> (r1_pos - 16)) : (r1n_re <<< (16 - r1_pos));
    wire signed [RW-1:0] nr_im = (r1_pos >= 16) ? (r1n_im >>> (r1_pos - 16)) : (r1n_im <<< (16 - r1_pos));
    wire signed [27:0]   d_rnd = (a_sh == 0) ? 28'sd0 : (28'sd1 <<< (a_sh - 1));

    // Newton product a*eps, rounded (18x18 + constant: one DSP48)
    wire signed [35:0]   ne_re = ate_hi_re * t_eps + 36'sd32768;
    wire signed [35:0]   ne_im = ate_hi_im * t_eps + 36'sd32768;

    assign active = act;

    always @(posedge clk) begin
        if (rst) begin
            v0 <= 0; v1 <= 0; v2 <= 0; v3 <= 0; v4 <= 0; v5 <= 0;
            v6 <= 0; v7 <= 0; v8 <= 0; v9 <= 0; v10 <= 0; v11 <= 0;
            v12 <= 0; v13 <= 0; v14 <= 0;
            out_valid <= 0;
            xp0_re <= 0; xp0_im <= 0;
            ma0_re <= 0; ma0_im <= 0; ma1_re <= 0; ma1_im <= 0;
            yp0_re <= 0; yp0_im <= 0; yp1_re <= 0; yp1_im <= 0;
            y0_re <= 0; y0_im <= 0; y1_re <= 0; y1_im <= 0;
            a24_re <= ONE24; a24_im <= 0;
            r1_re <= 0; r1_im <= 0; dr_re <= 0; dr_im <= 0;
            pin <= 0; pout <= 0; act <= 0;
            pin_o <= 0; pout_o <= 0;
        end else begin
            // ---------------- S0: capture
            v0 <= in_valid;
            if (in_valid) begin
                xr0_re <= x0_re; xr0_im <= x0_im; xr1_re <= x1_re; xr1_im <= x1_im;
            end
            // ---------------- S1: products (registered operands)
            v1 <= v0;
            if (v0) begin
                // x[n]<<16 + rnd - a x[n-1]   (ma still holds a x[n-1])
                xm0_re <= $signed({xr0_re, 16'd0}) + RND - ma0_re;
                xm0_im <= $signed({xr0_im, 16'd0}) + RND - ma0_im;
                xm1_re <= $signed({xr1_re, 16'd0}) + RND - ma1_re;
                xm1_im <= $signed({xr1_im, 16'd0}) + RND - ma1_im;
                // a x[n], for the next sample
                ma0_re <= a_re * xr0_re - a_im * xr0_im;
                ma0_im <= a_re * xr0_im + a_im * xr0_re;
                ma1_re <= a_re * xr1_re - a_im * xr1_im;
                ma1_im <= a_re * xr1_im + a_im * xr1_re;
                mr0_re <= ra_re * yp0_re - ra_im * yp0_im;
                mr0_im <= ra_re * yp0_im + ra_im * yp0_re;
                mr1_re <= ra_re * yp1_re - ra_im * yp1_im;
                mr1_im <= ra_re * yp1_im + ra_im * yp1_re;
                pr_re  <= xr0_re * xp0_re + xr0_im * xp0_im;
                pr_im  <= xr0_im * xp0_re - xr0_re * xp0_im;
                xp0_re <= xr0_re; xp0_im <= xr0_im;
                xd0_re <= xr0_re; xd0_im <= xr0_im; xd1_re <= xr1_re; xd1_im <= xr1_im;
            end
            // ---------------- S2: recursion, output
            v2 <= v1;
            out_valid <= v1;
            if (v1) begin
                yp0_re <= sat_y(yn0_re); yp0_im <= sat_y(yn0_im);
                yp1_re <= sat_y(yn1_re); yp1_im <= sat_y(yn1_im);
                if (act) begin
                    y0_re <= sat_dw(yn0_re); y0_im <= sat_dw(yn0_im);
                    y1_re <= sat_dw(yn1_re); y1_im <= sat_dw(yn1_im);
                end else begin
                    y0_re <= xd0_re; y0_im <= xd0_im; y1_re <= xd1_re; y1_im <= xd1_im;
                end
                xs_re <= xd0_re; xs_im <= xd0_im;
                ys_re <= sat_dw(yn0_re); ys_im <= sat_dw(yn0_im);
                dr_re <= ($signed(pr_re) <<< RF) - r1_re;
                dr_im <= ($signed(pr_im) <<< RF) - r1_im;
            end
            // ---------------- S3: r1 update, squares
            v3 <= v2;
            if (v2) begin
                if (clear) begin
                    r1_re <= 0; r1_im <= 0;
                end else begin
                    r1_re <= r1_re + (dr_re >>> r_sh);
                    r1_im <= r1_im + (dr_im >>> r_sh);
                end
                px_q <= xs_re * xs_re + xs_im * xs_im;
                py_q <= ys_re * ys_re + ys_im * ys_im;
            end
            // ---------------- S4: power smoothing; |r1|
            v4 <= v3;
            if (v3) begin
                pin  <= pin  + (((px_q <<< PF) - pin ) >>> 12);
                pout <= pout + (((py_q <<< PF) - pout) >>> 12);
                r1s_re <= r1_re; r1s_im <= r1_im;
                r1a_re <= r1_re[RW-1] ? -r1_re : r1_re;
                r1a_im <= r1_im[RW-1] ? -r1_im : r1_im;
            end
            // ---------------- S5: gating thresholds; leading-one input
            v5 <= v4;
            if (v4) begin
                pin_q  <= pin;
                th_on  <= pout + (pout >>> 2);
                th_off <= pout + (pout >>> 3);
                pin_o  <= (|pin [PW:PF+32]) ? 32'hFFFFFFFF : pin [PF+31:PF];
                pout_o <= (|pout[PW:PF+32]) ? 32'hFFFFFFFF : pout[PF+31:PF];
                r1q_re <= r1s_re; r1q_im <= r1s_im;
                r1_mx  <= r1a_re | r1a_im;      // same leading one as max(|re|,|im|)
            end
            // ---------------- S6: gating; leading one of |r1| (thermometer)
            v6 <= v5;
            if (v5) begin
                if (MODE == 1)
                    act <= en & (ext_act | force_on);
                else if (!en || ext_act)
                    act <= 1'b0;
                else if (force_on)
                    act <= 1'b1;
                else if (pin_q > th_on)
                    act <= 1'b1;
                else if (pin_q < th_off)
                    act <= 1'b0;
                r1t_re <= r1q_re; r1t_im <= r1q_im;
                r1_th  <= therm(r1_mx);
            end
            // ---------------- S7: leading one of |r1| (index)
            v7 <= v6;
            if (v6) begin
                r1n_re <= r1t_re; r1n_im <= r1t_im;
                r1_pos <= th_pos(r1_th);
            end
            // ---------------- S8: normalise r1 (power-of-two shift)
            v8 <= v7;
            if (v7) begin
                nr8_re <= $signed(nr_re[18:0]) <<< 8;
                nr8_im <= $signed(nr_im[18:0]) <<< 8;
                pos_n  <= r1_pos;
            end
            // ---------------- S9..S11: increment norm(r1)*2^-a_sh, rounded
            v9 <= v8;
            if (v8) begin
                nrr_re <= nr8_re + d_rnd;
                nrr_im <= nr8_im + d_rnd;
                pos_r  <= pos_n;
            end
            v10 <= v9;
            if (v9) begin
                ds_re <= nrr_re >>> a_sh;
                ds_im <= nrr_im >>> a_sh;
                pos_s <= pos_r;
            end
            v11 <= v10;
            if (v10) begin
                d_re    <= ds_re;
                d_im    <= ds_im;
                d128_re <= ds_re + 28'sd128;
                d128_im <= ds_im + 28'sd128;
                pos_d   <= pos_s;
            end
            // ---------------- S12: (loop) a + increment, and its Q16 rounding
            v12 <= v11;
            if (v11) begin
                at_re    <= at_s_re;        at_im    <= at_s_im;
                at_hi_re <= at_r_re >>> 8;  at_hi_im <= at_r_im >>> 8;
                pos_t    <= pos_d;
            end
            // ---------------- S13: (loop) |a|^2
            v13 <= v12;
            if (v12) begin
                at_mag <= at_hi_re * at_hi_re + at_hi_im * at_hi_im;
                atm_re <= at_re;    atm_im <= at_im;
                ath_re <= at_hi_re; ath_im <= at_hi_im;
                pos_m  <= pos_t;
            end
            // ---------------- S14: (loop) eps = (1 - |a|^2)/2  (Q24), clamped
            v14 <= v13;
            if (v13) begin
                if ((eps_w[38:17] == 0) || (&eps_w[38:17])) t_eps <= eps_w[17:0];
                else if (eps_w[38])                        t_eps <= -18'sd131071;
                else                                       t_eps <=  18'sd131071;
                ate_re    <= atm_re; ate_im    <= atm_im;
                ate_hi_re <= ath_re; ate_hi_im <= ath_im;
                pos_e     <= pos_m;
            end
            // ---------------- S15: (loop) a = a + a*eps
            if (MODE == 0 && v14) begin
                if (clear) begin
                    a24_re <= ONE24; a24_im <= 0;
                end else if (!freeze && pos_e > 7) begin
                    a24_re <= ate_re + (ne_re >>> 16);
                    a24_im <= ate_im + (ne_im >>> 16);
                end
            end
        end
    end
endmodule
