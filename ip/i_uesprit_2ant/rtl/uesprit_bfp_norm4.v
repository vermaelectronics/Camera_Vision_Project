`timescale 1ns/1ps
// ============================================================================
// uesprit_bfp_norm4  --  block-floating-point normaliser (combinational)
//
// Scales four signed W-bit values by one common power of two and returns
// 32-bit mantissas in which the largest magnitude lies in [2^29, 2^30)
// (i.e. [0.5, 1) in Q30, with one guard bit).
//
// All ESPRIT quantities used after the CMC stage are invariant to a common
// positive scale factor (C, C^p, C_fb, C_r, and the ratio psi = N / D), so
// the exponent is simply discarded. This keeps every later multiplier at
// 32-34 bits regardless of the received signal power.
//
// Requires |v| < 2^(W-2) for all inputs (headroom guaranteed by callers).
// ============================================================================
module uesprit_bfp_norm4 #(
    parameter integer W = 72
)(
    input  wire signed [W-1:0] v0,
    input  wire signed [W-1:0] v1,
    input  wire signed [W-1:0] v2,
    input  wire signed [W-1:0] v3,
    output wire signed [31:0]  m0,
    output wire signed [31:0]  m1,
    output wire signed [31:0]  m2,
    output wire signed [31:0]  m3,
    output wire                zero      // all inputs are zero
);

    function [W-1:0] absv;
        input signed [W-1:0] v;
        begin
            absv = v[W-1] ? (~v + 1'b1) : v;
        end
    endfunction

    wire [W-1:0] mag = absv(v0) | absv(v1) | absv(v2) | absv(v3);

    // leading-zero count of the OR of magnitudes = that of the maximum
    function integer lzc;
        input [W-1:0] x;
        integer k;
        reg     found;
        begin
            lzc   = W;
            found = 1'b0;
            for (k = W - 1; k >= 0; k = k - 1)
                if (!found && x[k]) begin
                    lzc   = W - 1 - k;
                    found = 1'b1;
                end
        end
    endfunction

    // place the magnitude MSB at bit W-3, then keep the top 32 bits
    wire [7:0] lz = lzc(mag);
    wire [7:0] sh = (lz >= 8'd2 && lz < W) ? lz - 8'd2 : 8'd0;

    wire signed [W-1:0] s0 = v0 <<< sh;
    wire signed [W-1:0] s1 = v1 <<< sh;
    wire signed [W-1:0] s2 = v2 <<< sh;
    wire signed [W-1:0] s3 = v3 <<< sh;

    assign m0   = s0[W-1 -: 32];
    assign m1   = s1[W-1 -: 32];
    assign m2   = s2[W-1 -: 32];
    assign m3   = s3[W-1 -: 32];
    assign zero = (mag == {W{1'b0}});

endmodule
