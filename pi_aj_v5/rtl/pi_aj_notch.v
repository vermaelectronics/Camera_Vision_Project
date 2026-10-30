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
    output wire [31:0]          pin_o, pout_o
);
    localparam signed [17:0] ONE = 18'sd65536;   // Q2.16
    localparam YW = DW + 2;                      // internal recursion width
    localparam RF = 16;                          // r1 fraction bits
    localparam RW = 2*DW + 2 + RF;               // r1 width
    localparam PF = 12;                          // power fraction bits
    localparam PW = 2*DW + 2 + PF;

    function signed [YW-1:0] sat_y;
        input signed [2*DW+8:0] v;
        if (v > $signed({{(2*DW+9-YW+1){1'b0}}, {(YW-1){1'b1}}}))
            sat_y = {1'b0, {(YW-1){1'b1}}};
        else if (v < $signed({{(2*DW+9-YW+1){1'b1}}, {(YW-1){1'b0}}}))
            sat_y = {1'b1, {(YW-1){1'b0}}};
        else
            sat_y = v[YW-1:0];
    endfunction

    function signed [DW-1:0] sat_d;
        input signed [YW-1:0] v;
        if (v > $signed({2'b00, {(DW-1){1'b1}}}))
            sat_d = {1'b0, {(DW-1){1'b1}}};
        else if (v < $signed({2'b11, {(DW-1){1'b0}}}))
            sat_d = {1'b1, {(DW-1){1'b0}}};
        else
            sat_d = v[DW-1:0];
    endfunction

    // highest set bit of a positive value (0 when v==0)
    function [6:0] msb;
        input [RW-1:0] v;
        integer k;
        begin
            msb = 0;
            for (k = 0; k < RW; k = k + 1)
                if (v[k]) msb = k;
        end
    endfunction

    // ---------------------------------------------------------------- coef
    // the estimator loop runs in Q2.24 with round-to-nearest (a truncating
    // Q16 loop is biased by ~-0.5 LSB/step, i.e. several kHz of notch error)
    localparam AF = 24;
    localparam signed [26:0] ONE24 = 27'sd16777216;
    reg  signed [26:0] a24_re, a24_im;
    wire signed [17:0] a_re_r = (a24_re + 27'sd128) >>> 8;
    wire signed [17:0] a_im_r = (a24_im + 27'sd128) >>> 8;
    wire signed [17:0] a_re = (MODE == 1) ? ext_a_re  : a_re_r;
    wire signed [17:0] a_im = (MODE == 1) ? -ext_a_im : a_im_r;
    wire signed [17:0] ra_re = a_re - (a_re >>> rho_sh);
    wire signed [17:0] ra_im = a_im - (a_im >>> rho_sh);
    assign a_re_o = a_re;
    assign a_im_o = a_im;

    // ---------------------------------------------------------------- state
    reg signed [DW-1:0] xr0_re, xr0_im, xr1_re, xr1_im;        // x[n]
    reg signed [DW-1:0] xp0_re, xp0_im, xp1_re, xp1_im;        // x[n-1]
    reg signed [YW-1:0] yp0_re, yp0_im, yp1_re, yp1_im;        // y[n-1]
    reg v0, v1, v2, v3, v4, v5, v6;

    // stage 1 products
    reg signed [2*DW+2:0] ma0_re, ma0_im, ma1_re, ma1_im;      // a*x[n-1]
    reg signed [2*DW+6:0] mr0_re, mr0_im, mr1_re, mr1_im;      // rho*a*y[n-1]
    reg signed [2*DW:0]   pr_re, pr_im;                        // x0[n] x0*[n-1]
    reg signed [DW-1:0]   xd0_re, xd0_im, xd1_re, xd1_im;

    localparam signed [2*DW+8:0] RND = 1 <<< 15;
    wire signed [2*DW+8:0] xs0_re = $signed({xd0_re, 16'd0});
    wire signed [2*DW+8:0] xs0_im = $signed({xd0_im, 16'd0});
    wire signed [2*DW+8:0] xs1_re = $signed({xd1_re, 16'd0});
    wire signed [2*DW+8:0] xs1_im = $signed({xd1_im, 16'd0});
    wire signed [2*DW+8:0] yn0_re = (xs0_re + RND - ma0_re + mr0_re) >>> 16;
    wire signed [2*DW+8:0] yn0_im = (xs0_im + RND - ma0_im + mr0_im) >>> 16;
    wire signed [2*DW+8:0] yn1_re = (xs1_re + RND - ma1_re + mr1_re) >>> 16;
    wire signed [2*DW+8:0] yn1_im = (xs1_im + RND - ma1_im + mr1_im) >>> 16;

    // r1 / power estimators
    reg signed [RW-1:0] r1_re, r1_im;
    reg signed [PW:0]   pin, pout;          // always >= 0
    reg                 act;
    reg signed [DW-1:0] xs_re, xs_im;      // x0[n] for power
    reg signed [YW-1:0] ys_re, ys_im;      // y0[n] for power

    // estimator pipeline
    reg signed [RW-1:0] r1s_re, r1s_im;
    reg [6:0]           r1_pos;
    reg signed [27:0]   at_re, at_im;
    reg signed [56:0]   at_mag;
    reg signed [28:0]   t_nr;

    wire [RW-1:0] r1_abs_re = r1_re[RW-1] ? -r1_re : r1_re;
    wire [RW-1:0] r1_abs_im = r1_im[RW-1] ? -r1_im : r1_im;
    wire [RW-1:0] r1_max    = (r1_abs_re > r1_abs_im) ? r1_abs_re : r1_abs_im;

    // normalised r1 in Q16 with its largest component in [1,2)
    wire signed [RW-1:0] nr_re = (r1_pos >= 16) ? (r1s_re >>> (r1_pos - 16)) : (r1s_re <<< (16 - r1_pos));
    wire signed [RW-1:0] nr_im = (r1_pos >= 16) ? (r1s_im >>> (r1_pos - 16)) : (r1s_im <<< (16 - r1_pos));
    // increment in Q24, rounded:  norm(r1) * 2^-a_sh
    wire signed [27:0]   nr8_re = $signed(nr_re[18:0]) <<< 8;
    wire signed [27:0]   nr8_im = $signed(nr_im[18:0]) <<< 8;
    wire signed [27:0]   d_rnd  = (a_sh == 0) ? 28'sd0 : (28'sd1 <<< (a_sh - 1));
    wire signed [27:0]   d_re   = (nr8_re + d_rnd) >>> a_sh;
    wire signed [27:0]   d_im   = (nr8_im + d_rnd) >>> a_sh;

    wire signed [PW:0] px = xs_re * xs_re + xs_im * xs_im;
    wire signed [PW:0] py = ys_re * ys_re + ys_im * ys_im;

    assign active = act;
    assign pin_o  = (pin  >>> PF) > 33'sh0FFFFFFFF ? 32'hFFFFFFFF : pin [PF+31:PF];
    assign pout_o = (pout >>> PF) > 33'sh0FFFFFFFF ? 32'hFFFFFFFF : pout[PF+31:PF];

    always @(posedge clk) begin
        if (rst) begin
            v0 <= 0; v1 <= 0; v2 <= 0; v3 <= 0; v4 <= 0; v5 <= 0; v6 <= 0;
            out_valid <= 0;
            xp0_re <= 0; xp0_im <= 0; xp1_re <= 0; xp1_im <= 0;
            yp0_re <= 0; yp0_im <= 0; yp1_re <= 0; yp1_im <= 0;
            y0_re <= 0; y0_im <= 0; y1_re <= 0; y1_im <= 0;
            a24_re <= ONE24; a24_im <= 0;
            r1_re <= 0; r1_im <= 0;
            pin <= 0; pout <= 0; act <= 0;
        end else begin
            // ---------------- stage 0: capture
            v0 <= in_valid;
            if (in_valid) begin
                xr0_re <= x0_re; xr0_im <= x0_im; xr1_re <= x1_re; xr1_im <= x1_im;
            end
            // ---------------- stage 1: products
            v1 <= v0;
            if (v0) begin
                ma0_re <= a_re * xp0_re - a_im * xp0_im;
                ma0_im <= a_re * xp0_im + a_im * xp0_re;
                ma1_re <= a_re * xp1_re - a_im * xp1_im;
                ma1_im <= a_re * xp1_im + a_im * xp1_re;
                mr0_re <= ra_re * yp0_re - ra_im * yp0_im;
                mr0_im <= ra_re * yp0_im + ra_im * yp0_re;
                mr1_re <= ra_re * yp1_re - ra_im * yp1_im;
                mr1_im <= ra_re * yp1_im + ra_im * yp1_re;
                pr_re  <= xr0_re * xp0_re + xr0_im * xp0_im;
                pr_im  <= xr0_im * xp0_re - xr0_re * xp0_im;
                xd0_re <= xr0_re; xd0_im <= xr0_im; xd1_re <= xr1_re; xd1_im <= xr1_im;
            end
            // ---------------- stage 2: recursion, output
            v2 <= v1;
            out_valid <= v1;
            if (v1) begin
                yp0_re <= sat_y(yn0_re); yp0_im <= sat_y(yn0_im);
                yp1_re <= sat_y(yn1_re); yp1_im <= sat_y(yn1_im);
                xp0_re <= xd0_re; xp0_im <= xd0_im; xp1_re <= xd1_re; xp1_im <= xd1_im;
                if (act) begin
                    y0_re <= sat_d(sat_y(yn0_re)); y0_im <= sat_d(sat_y(yn0_im));
                    y1_re <= sat_d(sat_y(yn1_re)); y1_im <= sat_d(sat_y(yn1_im));
                end else begin
                    y0_re <= xd0_re; y0_im <= xd0_im; y1_re <= xd1_re; y1_im <= xd1_im;
                end
                xs_re <= xd0_re; xs_im <= xd0_im;
                ys_re <= sat_y(yn0_re); ys_im <= sat_y(yn0_im);
                if (clear) begin
                    r1_re <= 0; r1_im <= 0;
                end else begin
                    r1_re <= r1_re + ((($signed(pr_re) <<< RF) - r1_re) >>> r_sh);
                    r1_im <= r1_im + ((($signed(pr_im) <<< RF) - r1_im) >>> r_sh);
                end
            end
            // ---------------- stage 3: power, gating, estimator step 1
            v3 <= v2;
            if (v2) begin
                pin  <= pin  + (((px <<< PF) - pin ) >>> 12);
                pout <= pout + (((py <<< PF) - pout) >>> 12);
                if (MODE == 1)
                    act <= en & (ext_act | force_on);
                else if (!en || ext_act)
                    act <= 1'b0;
                else if (force_on)
                    act <= 1'b1;
                else if (pin > pout + (pout >> 2))
                    act <= 1'b1;
                else if (pin < pout + (pout >> 3))
                    act <= 1'b0;
                r1s_re <= r1_re; r1s_im <= r1_im;
                r1_pos <= msb(r1_max);
            end
            // ---------------- estimator step 2: a + norm(r1)*2^-a_sh
            v4 <= v3;
            if (v3) begin
                at_re <= a24_re + d_re;
                at_im <= a24_im + d_im;
            end
            // ---------------- estimator step 3: |a|^2
            v5 <= v4;
            if (v4)
                at_mag <= at_re * at_re + at_im * at_im;
            // ---------------- estimator step 4: t = 1.5 - |a|^2/2   (Q24)
            v6 <= v5;
            if (v5)
                t_nr <= 29'sd25165824 - (at_mag >>> 25);
            // ---------------- estimator step 5: a = a*t
            if (MODE == 0 && v6) begin
                if (clear) begin
                    a24_re <= ONE24; a24_im <= 0;
                end else if (!freeze && r1_pos > 7) begin
                    a24_re <= (at_re * t_nr + 57'sd8388608) >>> AF;
                    a24_im <= (at_im * t_nr + 57'sd8388608) >>> AF;
                end
            end
        end
    end
endmodule
