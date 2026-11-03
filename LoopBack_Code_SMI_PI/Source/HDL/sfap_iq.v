// ============================================================================
//  sfap_iq.v  --  blind I/Q-imbalance corrector for one receiver (SMI-PI 5.1)
//
//  An AD9361 receiver with I/Q gain/phase error turns a signal a into
//      x = a + k a*          (|k| = 10^(-IRR/20): 0.8 % for 42 dB image rejection)
//  so every jammer also has an image at the mirror frequency, with a spatial signature of its
//  own. Two antennas have one spatial null per frequency; the image takes it away, and the
//  jammer residual is stuck at (J/N - image rejection). WL-STAP-PI handles this with conjugate
//  taps; here the cheaper blind correction
//      y = x - k x*,        k = E[x^2] / (2 E|x|^2)
//  (a circular jammer or noise has E[a^2] = 0, so E[x^2] = 2 k E|a|^2) is applied per receiver.
//  The simulation (Tests/sfap/iq_study.py) shows the same gain as per-bin widely-linear weights
//  at a fraction of the cost.
//
//  This module only applies k and measures the three sums over 2^16 samples:
//      S2r = sum (xr^2 - xi^2),  S2i = sum 2 xr xi,  P = sum (xr^2 + xi^2)
//  latched (>> 8) at the end of each window. The ARM divides (k = S2 / 2P), averages and writes k
//  back: it is a slow loop, and it only runs while a jammer is there.
//  k is Q1.15 (kr, ki). k = 0 is a plain delay by 7 clocks.
// ============================================================================
`timescale 1ns/1ps

module sfap_iq (
    input  wire               clk,
    input  wire               v,
    input  wire signed [15:0] xi, xq,           // 12-bit samples, right aligned
    input  wire signed [15:0] kr, ki,           // Q1.15
    output reg                ov,
    output reg  signed [15:0] yi, yq,
    output reg  [31:0]        s2r_l, s2i_l, p_l
);
    // stage 1: register
    reg signed [11:0] a1, b1; reg signed [15:0] kr1, ki1; reg v1;
    always @(posedge clk) begin a1 <= xi[11:0]; b1 <= xq[11:0]; kr1 <= kr; ki1 <= ki; v1 <= v; end
    // stage 2: products (DSP48, M register)
    reg signed [23:0] aa2, bb2, ab2; reg signed [27:0] kra2, kib2, kia2, krb2;
    reg signed [11:0] a2, b2; reg v2;
    always @(posedge clk) begin
        aa2 <= a1 * a1; bb2 <= b1 * b1; ab2 <= a1 * b1;
        kra2 <= kr1 * a1; kib2 <= ki1 * b1; kia2 <= ki1 * a1; krb2 <= kr1 * b1;
        a2 <= a1; b2 <= b1; v2 <= v1;
    end
    // stage 3: P register
    reg signed [23:0] aa3, bb3, ab3; reg signed [27:0] kra3, kib3, kia3, krb3;
    reg signed [11:0] a3, b3; reg v3;
    always @(posedge clk) begin
        aa3 <= aa2; bb3 <= bb2; ab3 <= ab2; kra3 <= kra2; kib3 <= kib2; kia3 <= kia2; krb3 <= krb2;
        a3 <= a2; b3 <= b2; v3 <= v2;
    end
    // stage 4: sums
    reg signed [28:0] cr4, ci4;                      // k x*  (Q1.15 x LSB)
    reg signed [24:0] d4, e4, p4;                    // xr^2 - xi^2, 2 xr xi, xr^2 + xi^2
    reg signed [11:0] a4, b4; reg v4;
    always @(posedge clk) begin
        cr4 <= kra3 + kib3;                          //  kr xr + ki xi
        ci4 <= kia3 - krb3;                          //  ki xr - kr xi
        d4 <= aa3 - bb3; e4 <= {ab3, 1'b0}; p4 <= aa3 + bb3;
        a4 <= a3; b4 <= b3; v4 <= v3;
    end
    // stage 5: rounding offset
    reg signed [28:0] cr5, ci5; reg signed [11:0] a5, b5; reg v5;
    always @(posedge clk) begin cr5 <= cr4 + 29'sd16384; ci5 <= ci4 + 29'sd16384; a5 <= a4; b5 <= b4; v5 <= v4; end
    // stage 6: x - k x*
    always @(posedge clk) begin
        yi <= $signed({{4{a5[11]}}, a5}) - (cr5 >>> 15);
        yq <= $signed({{4{b5[11]}}, b5}) - (ci5 >>> 15);
        ov <= v5;
    end
    // statistics: accumulate at stage 4 -> 5 (one add per clock), window of 2^16 valid samples
    reg signed [47:0] sr = 48'sd0, si = 48'sd0, sp = 48'sd0;
    reg [15:0] cnt = 16'd0;
    always @(posedge clk) if (v4) begin
        cnt <= cnt + 16'd1;
        if (cnt == 16'hFFFF) begin
            s2r_l <= sr >>> 8; s2i_l <= si >>> 8; p_l <= sp >>> 8;
            sr <= 48'd0; si <= 48'd0; sp <= 48'd0;
        end else begin
            sr <= sr + {{23{d4[24]}}, d4};
            si <= si + {{23{e4[24]}}, e4};
            sp <= sp + {{23{p4[24]}}, p4};
        end
    end
endmodule
