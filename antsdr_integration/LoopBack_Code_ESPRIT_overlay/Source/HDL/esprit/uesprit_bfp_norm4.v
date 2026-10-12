`timescale 1ns/1ps
// ============================================================================
// uesprit_bfp_norm4  --  block-floating-point normaliser (2-stage pipeline)
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
// Stage 1: magnitude OR + leading-zero count (registered with the inputs)
// Stage 2: common left shift, keep the top 32 bits (registered)
// Latency: 2 cycles, fully pipelined.
//
// Requires |v| < 2^(W-2) for all inputs (headroom guaranteed by callers).
// ============================================================================
module uesprit_bfp_norm4 #(
    parameter integer W = 72
)(
    input  wire                clk,
    input  wire                rst,
    input  wire                in_valid,
    input  wire signed [W-1:0] v0,
    input  wire signed [W-1:0] v1,
    input  wire signed [W-1:0] v2,
    input  wire signed [W-1:0] v3,
    output reg                 out_valid,
    output reg  signed [31:0]  m0,
    output reg  signed [31:0]  m1,
    output reg  signed [31:0]  m2,
    output reg  signed [31:0]  m3,
    output reg                 zero      // all inputs are zero
);

    function [W-1:0] absv;
        input signed [W-1:0] v;
        begin
            absv = v[W-1] ? (~v + 1'b1) : v;
        end
    endfunction

    // leading-zero count of the OR of magnitudes = that of the maximum
    function [7:0] lzc;
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

    wire [W-1:0] mag = absv(v0) | absv(v1) | absv(v2) | absv(v3);
    wire [7:0]   lz  = lzc(mag);

    // ---------------- stage 1 ----------------
    reg signed [W-1:0] r0, r1, r2, r3;
    reg [7:0]          sh;
    reg                z1, v1r;

    always @(posedge clk) begin
        if (rst) begin
            v1r <= 1'b0; z1 <= 1'b0; sh <= 8'd0;
            r0 <= 0; r1 <= 0; r2 <= 0; r3 <= 0;
        end else begin
            v1r <= in_valid;
            if (in_valid) begin
                r0 <= v0; r1 <= v1; r2 <= v2; r3 <= v3;
                // place the magnitude MSB at bit W-3
                sh <= (lz >= 8'd2 && lz < W) ? lz - 8'd2 : 8'd0;
                z1 <= (mag == {W{1'b0}});
            end
        end
    end

    // ---------------- stage 2 ----------------
    wire signed [W-1:0] s0 = r0 <<< sh;
    wire signed [W-1:0] s1 = r1 <<< sh;
    wire signed [W-1:0] s2 = r2 <<< sh;
    wire signed [W-1:0] s3 = r3 <<< sh;

    always @(posedge clk) begin
        if (rst) begin
            out_valid <= 1'b0; zero <= 1'b0;
            m0 <= 0; m1 <= 0; m2 <= 0; m3 <= 0;
        end else begin
            out_valid <= v1r;
            if (v1r) begin
                m0   <= s0[W-1 -: 32];
                m1   <= s1[W-1 -: 32];
                m2   <= s2[W-1 -: 32];
                m3   <= s3[W-1 -: 32];
                zero <= z1;
            end
        end
    end

endmodule
