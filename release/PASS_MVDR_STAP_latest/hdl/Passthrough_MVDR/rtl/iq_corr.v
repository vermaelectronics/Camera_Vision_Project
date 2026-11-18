// -----------------------------------------------------------------------------
// iq_corr.v  --  per-channel receiver IQ-imbalance (image) correction
//
//     xc_k = x_k * 2^IQF  -  round( b_k * conj(x_k) * 2^IQF )        k = 1, 2
//
// WHY
//   The AD9361 receiver leaves an image of ~ -50 dBc after its own quadrature
//   calibration: every signal at +f also appears at -f, ~50 dB down, with a
//   DIFFERENT RX1/RX2 phase relation from the original (the image of channel k
//   carries conj of that channel's phase). For a strong CW jammer this mirror
//   is a second interferer; a 2-element array has one null, so with linear
//   weights the null stops at ~ -50 dB relative to the jammer: above ~50 dB
//   JNR the jammer leaks above the noise (model/image_study.py: +1 dB at
//   50 dB JNR, +16 dB at 65 dB). Removing the image per channel restores the
//   ideal null (-36 dB vs noise at 50..65 dB JNR).
//   b_k is estimated by iq_est.v (blind: from the pseudo-covariance sum x x^T
//   of the corrected stream, which must be zero for a proper signal).
//
// FORMAT
//   x   : signed DATA_W (raw ADC, 12)
//   b   : signed B_W, B_FRAC fraction bits (Q20 in 18 bits: |b| < 1/8, LSB 1e-6)
//   xc  : signed DATA_W + IQF + 1: IQF fraction bits (no new quantization noise
//         worth mentioning: 1/4 LSB rounding, 24 dB under the ADC's own), plus
//         one guard bit for |b x| (full-scale x with |b| near 1/8)
//   clip1/clip2 : raw input on an ADC rail, delayed with the sample (the corrected
//         value no longer sits on a fixed rail)
//   b_load: new b set takes effect on the next sample (whole set at once).
//   With b = 0 the output is exactly x * 2^IQF.
//
// Latency 5 clocks (v1: 3; v2 for 250 MHz: input register, and the rounding
// constant added in its own DSP stage), one sample per clock. Cost: 8 multipliers (DATA_W x B_W).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module iq_corr #(
    parameter DATA_W = 12,
    parameter IQF    = 2,
    parameter B_W    = 18,
    parameter B_FRAC = 20,
    parameter OW     = DATA_W + IQF + 1          // derived
)(
    input  wire                      clk,
    input  wire                      rst,

    input  wire                      in_valid,
    input  wire signed [DATA_W-1:0]  x1_i,
    input  wire signed [DATA_W-1:0]  x1_q,
    input  wire signed [DATA_W-1:0]  x2_i,
    input  wire signed [DATA_W-1:0]  x2_q,

    input  wire                      b_load,
    input  wire signed [B_W-1:0]     b1_re,
    input  wire signed [B_W-1:0]     b1_im,
    input  wire signed [B_W-1:0]     b2_re,
    input  wire signed [B_W-1:0]     b2_im,

    output reg                       out_valid,
    (* keep = "true" *) output reg  signed [OW-1:0]      y1_i,
    (* keep = "true" *) output reg  signed [OW-1:0]      y1_q,
    (* keep = "true" *) output reg  signed [OW-1:0]      y2_i,
    (* keep = "true" *) output reg  signed [OW-1:0]      y2_q,
    output reg                       clip1,
    output reg                       clip2
);

    localparam PW = DATA_W + B_W;                 // one product
    localparam SH = B_FRAC - IQF;                 // product -> output units
    localparam signed [DATA_W-1:0] RAIL_P = {1'b0, {(DATA_W-1){1'b1}}};
    localparam signed [DATA_W-1:0] RAIL_N = {1'b1, {(DATA_W-1){1'b0}}};
    localparam signed [PW:0] HALF = {{PW{1'b0}}, 1'b1} << (SH - 1);

    reg signed [B_W-1:0] c1r = 0, c1i = 0, c2r = 0, c2i = 0;    // b in use
    always @(posedge clk) begin
        if (rst) begin
            c1r <= 0; c1i <= 0; c2r <= 0; c2i <= 0;
        end else if (b_load) begin
            c1r <= b1_re; c1i <= b1_im; c2r <= b2_re; c2i <= b2_im;
        end
    end

    // stage 0: input register (the samples arrive from axi_ad9361 flops in
    // another part of the die; the multipliers then start from local flops)
    reg                     v0, k1_0, k2_0;
    reg signed [DATA_W-1:0] a_i, a_q, d_i, d_q;
    always @(posedge clk) begin
        if (rst) v0 <= 1'b0; else v0 <= in_valid;
        a_i <= x1_i;  a_q <= x1_q;  d_i <= x2_i;  d_q <= x2_q;
        k1_0 <= (x1_i == RAIL_P) || (x1_i == RAIL_N) || (x1_q == RAIL_P) || (x1_q == RAIL_N);
        k2_0 <= (x2_i == RAIL_P) || (x2_i == RAIL_N) || (x2_q == RAIL_P) || (x2_q == RAIL_N);
    end

    // stage 1: products of b * conj(x) = (br xr + bi xi) + j (bi xr - br xi)
    //   (DSP M register)
    reg                     v1, k1_1, k2_1;
    reg signed [DATA_W-1:0] a_i1, a_q1, d_i1, d_q1;
    reg signed [PW-1:0]     m1a, m1b, m1c, m1d, m2a, m2b, m2c, m2d;
    always @(posedge clk) begin
        if (rst) v1 <= 1'b0; else v1 <= v0;
        k1_1 <= k1_0;  k2_1 <= k2_0;
        a_i1 <= a_i;  a_q1 <= a_q;  d_i1 <= d_i;  d_q1 <= d_q;
        m1a <= c1r * a_i;  m1b <= c1i * a_q;  m1c <= c1i * a_i;  m1d <= c1r * a_q;
        m2a <= c2r * d_i;  m2b <= c2i * d_q;  m2c <= c2i * d_i;  m2d <= c2r * d_q;
    end

    // stage 1b: the rounding constant rides on one product of each pair
    //   (DSP post-adder, P register: multiply and add never share a clock)
    reg                     v1b, k1_1b, k2_1b;
    reg signed [DATA_W-1:0] a_i1b, a_q1b, d_i1b, d_q1b;
    reg signed [PW:0]       p1a, p1c, p2a, p2c;
    reg signed [PW-1:0]     p1b, p1d, p2b, p2d;
    always @(posedge clk) begin
        if (rst) v1b <= 1'b0; else v1b <= v1;
        k1_1b <= k1_1;  k2_1b <= k2_1;
        a_i1b <= a_i1;  a_q1b <= a_q1;  d_i1b <= d_i1;  d_q1b <= d_q1;
        p1a <= m1a + HALF;  p1b <= m1b;  p1c <= m1c + HALF;  p1d <= m1d;
        p2a <= m2a + HALF;  p2b <= m2b;  p2c <= m2c + HALF;  p2d <= m2d;
    end

    // stage 2: sums
    reg                     v2, k1_2, k2_2;
    reg signed [DATA_W-1:0] a_i2, a_q2, d_i2, d_q2;
    reg signed [PW:0]       s1r, s1i, s2r, s2i;
    always @(posedge clk) begin
        if (rst) v2 <= 1'b0; else v2 <= v1b;
        k1_2 <= k1_1b;  k2_2 <= k2_1b;
        a_i2 <= a_i1b;  a_q2 <= a_q1b;  d_i2 <= d_i1b;  d_q2 <= d_q1b;
        s1r <= p1a + p1b;  s1i <= p1c - p1d;
        s2r <= p2a + p2b;  s2i <= p2c - p2d;
    end

    // stage 3: scale, subtract
    wire signed [PW:0] t1r = s1r >>> SH, t1i = s1i >>> SH, t2r = s2r >>> SH, t2i = s2i >>> SH;
    // v1.14 timing: the outputs stay fabric flops (KEEP) -- Vivado otherwise
    // folds them into the covariance DSP's B register and the DSP P -> carry
    // -> DSP B path missed 4 ns by 40 ps.
    always @(posedge clk) begin
        if (rst) out_valid <= 1'b0; else out_valid <= v2;
        clip1 <= k1_2;  clip2 <= k2_2;
        y1_i <= ($signed({a_i2, {IQF{1'b0}}}) - t1r);
        y1_q <= ($signed({a_q2, {IQF{1'b0}}}) - t1i);
        y2_i <= ($signed({d_i2, {IQF{1'b0}}}) - t2r);
        y2_q <= ($signed({d_q2, {IQF{1'b0}}}) - t2i);
    end

endmodule
