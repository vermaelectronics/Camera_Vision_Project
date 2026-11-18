// -----------------------------------------------------------------------------
// cov_iir.v  --  exponential (IIR) smoothing of the covariance across windows
//
//   y[n] = y[n-1] + ( (x[n] - y[n-1]) >>> sh )
//
// sh=0 is an exact bypass (y + (x-y)>>>0 = x). Larger sh averages over more
// consecutive published windows, pushing the null deeper than the base
// N=4096 statistical floor allows -- WITHOUT lengthening the base window,
// so the first reaction to a new jammer is unaffected (it's still driven
// by the freshest window on arrival). Only the time to reach the DEEPEST
// null grows, roughly proportional to 2^sh window-updates.
//
// On reset, or right after sh_reset pulses (assert whenever firmware changes
// sh), the next sample passes straight through instead of ramping up from
// zero.
//
// Update rate: once per covariance window (~1024 samples apart) -- no
// timing pressure, 1-clock latency.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module cov_iir #(
    parameter ACC_W = 37
)(
    input  wire                     clk,
    input  wire                     rst,        // synchronous, active high

    input  wire                     in_valid,   // = m_cov_valid
    input  wire signed [ACC_W-1:0]  r11_in,
    input  wire signed [ACC_W-1:0]  r22_in,
    input  wire signed [ACC_W-1:0]  r12_re_in,
    input  wire signed [ACC_W-1:0]  r12_im_in,

    input  wire        [3:0]        sh,         // 0 = bypass (default), 1..8 = EMA shift
    input  wire                     sh_reset,   // pulse: re-prime on the next sample

    output reg                      out_valid,
    output reg  signed [ACC_W-1:0]  r11_out,
    output reg  signed [ACC_W-1:0]  r22_out,
    output reg  signed [ACC_W-1:0]  r12_re_out,
    output reg  signed [ACC_W-1:0]  r12_im_out
);

    reg                     primed;
    reg signed [ACC_W-1:0]  y11, y22, y12_re, y12_im;

    wire signed [ACC_W-1:0] d11    = r11_in    - y11;
    wire signed [ACC_W-1:0] d22    = r22_in    - y22;
    wire signed [ACC_W-1:0] d12_re = r12_re_in - y12_re;
    wire signed [ACC_W-1:0] d12_im = r12_im_in - y12_im;

    always @(posedge clk) begin
        if (rst) begin
            out_valid <= 1'b0;
            primed    <= 1'b0;
            y11 <= {ACC_W{1'b0}}; y22 <= {ACC_W{1'b0}};
            y12_re <= {ACC_W{1'b0}}; y12_im <= {ACC_W{1'b0}};
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                if (!primed || sh_reset) begin
                    y11 <= r11_in;  y22 <= r22_in;
                    y12_re <= r12_re_in;  y12_im <= r12_im_in;
                    primed <= 1'b1;
                end else begin
                    y11    <= y11    + (d11    >>> sh);
                    y22    <= y22    + (d22    >>> sh);
                    y12_re <= y12_re + (d12_re >>> sh);
                    y12_im <= y12_im + (d12_im >>> sh);
                end
            end
        end
        r11_out    <= y11;
        r22_out    <= y22;
        r12_re_out <= y12_re;
        r12_im_out <= y12_im;
    end

endmodule
