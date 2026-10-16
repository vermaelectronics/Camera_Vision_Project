/*******************************************************************************
 * pi_cmul.v
 *
 * Combinational complex multiplier, used as the building block for the
 * power-inversion (PI) beamformer core (pi_power_inversion.v).
 *
 *   CONJ_A = 0 :  p = a * b            = (a_re*b_re - a_im*b_im)
 *                                      + j(a_re*b_im + a_im*b_re)
 *   CONJ_A = 1 :  p = conj(a) * b      = (a_re*b_re + a_im*b_im)
 *                                      + j(a_re*b_im - a_im*b_re)
 *
 * No rounding and no truncation here -- the output is the full-width raw
 * product (A_W + B_W + 1 bits per rail). The caller decides how to
 * truncate/round down to whatever fixed-point format it needs; keeping
 * that decision out of this module keeps it reusable for any Q-format.
 ******************************************************************************/
`timescale 1ns/1ps

module pi_cmul #(
    parameter A_W    = 18,
    parameter B_W    = 18,
    parameter CONJ_A = 0          // 0: a*b   1: conj(a)*b
) (
    input  wire signed [A_W-1:0]     a_re,
    input  wire signed [A_W-1:0]     a_im,
    input  wire signed [B_W-1:0]     b_re,
    input  wire signed [B_W-1:0]     b_im,
    output wire signed [A_W+B_W:0]   p_re,   // A_W+B_W+1 bits: sum of two (A_W+B_W)-bit products
    output wire signed [A_W+B_W:0]   p_im
);

    wire signed [A_W+B_W-1:0] m_re_re = a_re * b_re;
    wire signed [A_W+B_W-1:0] m_im_im = a_im * b_im;
    wire signed [A_W+B_W-1:0] m_re_im = a_re * b_im;
    wire signed [A_W+B_W-1:0] m_im_re = a_im * b_re;

    assign p_re = CONJ_A ? (m_re_re + m_im_im) : (m_re_re - m_im_im);
    assign p_im = CONJ_A ? (m_re_im - m_im_re) : (m_re_im + m_im_re);

endmodule
