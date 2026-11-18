// -----------------------------------------------------------------------------
// beamformer_2ch.v  --  MVDR step: apply weights to the live sample stream
//
//     y = w1* . x1  +  w2* . x2          (wH x, complex)
//
//     yr = (w1r*x1r + w1i*x1i) + (w2r*x2r + w2i*x2i)
//     yi = (w1r*x1i - w1i*x1r) + (w2r*x2i - w2i*x2r)
//
// Weights are double-buffered:
//   * w_load : pulse -> capture w1_*/w2_* into the PENDING registers
//   * swap   : pulse -> copy PENDING into ACTIVE registers
// If both pulse on the same clock, swap uses the OLD pending value (the new
// load lands in pending for the next swap).
//
// Swap timing: a sample presented on the same clock edge that samples `swap`
// still uses the OLD weights.  The first sample presented on the following
// edge uses the NEW weights.  The whole weight set changes on one edge, so a
// sample never sees a mix of old and new weights.
//
// After reset both ACTIVE and PENDING hold w1 = 1.0, w2 = 0, i.e. the output
// is RX1 alone (scaled by 2**W_FRAC) until the first real weight set arrives.
//
// Number format:
//   x  : signed DATA_W-bit I/Q integers
//   w  : signed W_W-bit I/Q, W_FRAC fractional bits  (1.0 == 1 << W_FRAC)
//   y  : signed OUT_W = DATA_W + W_W + 2 bits, W_FRAC fractional bits
//        -> full precision, no overflow possible.  output_scaler.v later
//        shifts right by W_FRAC and rounds/saturates to the TX width.
//
// Latency: 4 clocks (3 in cmult incl. its v2 operand register + 1 for the
//          channel sum), 1 sample per clock. The swap rule above is unchanged:
//          the weights are captured together with the sample in cmult's
//          operand register.
// Cost   : 8 real multipliers (2 x cmult).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module beamformer_2ch #(
    parameter DATA_W = 16,
    parameter W_W    = 18,
    parameter W_FRAC = 15,                   // must be <= W_W-2 so 1.0 fits
    parameter OUT_W  = DATA_W + W_W + 2
)(
    input  wire                      clk,
    input  wire                      rst,        // synchronous, active high

    // sample stream
    input  wire                      in_valid,
    input  wire signed [DATA_W-1:0]  x1_i,
    input  wire signed [DATA_W-1:0]  x1_q,
    input  wire signed [DATA_W-1:0]  x2_i,
    input  wire signed [DATA_W-1:0]  x2_q,

    // weight update
    input  wire                      w_load,
    input  wire signed [W_W-1:0]     w1_re,
    input  wire signed [W_W-1:0]     w1_im,
    input  wire signed [W_W-1:0]     w2_re,
    input  wire signed [W_W-1:0]     w2_im,
    input  wire                      swap,

    // beamformed output
    output reg                       out_valid,
    output reg  signed [OUT_W-1:0]   y_re,
    output reg  signed [OUT_W-1:0]   y_im
);

    localparam signed [W_W-1:0] W_ONE = (1 <<< W_FRAC);

    // ---------------- weight registers ----------------
    reg signed [W_W-1:0] pend_w1r, pend_w1i, pend_w2r, pend_w2i;
    reg signed [W_W-1:0] act_w1r,  act_w1i,  act_w2r,  act_w2i;

    always @(posedge clk) begin
        if (rst) begin
            pend_w1r <= W_ONE;  pend_w1i <= {W_W{1'b0}};
            pend_w2r <= {W_W{1'b0}};  pend_w2i <= {W_W{1'b0}};
            act_w1r  <= W_ONE;  act_w1i  <= {W_W{1'b0}};
            act_w2r  <= {W_W{1'b0}};  act_w2i  <= {W_W{1'b0}};
        end else begin
            if (swap) begin
                act_w1r <= pend_w1r;  act_w1i <= pend_w1i;
                act_w2r <= pend_w2r;  act_w2i <= pend_w2i;
            end
            if (w_load) begin
                pend_w1r <= w1_re;  pend_w1i <= w1_im;
                pend_w2r <= w2_re;  pend_w2i <= w2_im;
            end
        end
    end

    // ---------------- per-channel complex multiply: x * conj(w) ----------------
    localparam CM_W = DATA_W + W_W + 1;

    wire                     v_ch1, v_ch2;
    wire signed [CM_W-1:0]   p1_re, p1_im, p2_re, p2_im;

    cmult #(.A_W(DATA_W), .B_W(W_W), .CONJ_B(1)) u_ch1 (
        .clk(clk), .rst(rst), .in_valid(in_valid),
        .ar(x1_i), .ai(x1_q), .br(act_w1r), .bi(act_w1i),
        .out_valid(v_ch1), .yr(p1_re), .yi(p1_im));

    cmult #(.A_W(DATA_W), .B_W(W_W), .CONJ_B(1)) u_ch2 (
        .clk(clk), .rst(rst), .in_valid(in_valid),
        .ar(x2_i), .ai(x2_q), .br(act_w2r), .bi(act_w2i),
        .out_valid(v_ch2), .yr(p2_re), .yi(p2_im));

    // ---------------- channel sum ----------------
    always @(posedge clk) begin
        if (rst) out_valid <= 1'b0;
        else     out_valid <= v_ch1;          // v_ch1 == v_ch2 by construction
        y_re <= p1_re + p2_re;
        y_im <= p1_im + p2_im;
    end

endmodule
