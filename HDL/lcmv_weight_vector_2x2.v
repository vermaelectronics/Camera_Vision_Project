`timescale 1ns/1ps

// ============================================================================
// lcmv_weight_vector_2x2.v
//
// Pipelined LCMV weight generation:
//
//                  W = F * f
//
// F = | F11 F12 |
//     | F21 F22 |
//
// f = | fGPS    |
//     | fJAMMER |
//
// w1 = F11*fGPS    + F12*fJAMMER
// w2 = F21*fGPS    + F22*fJAMMER
//
// Q16.16 complex arithmetic.
//
// PIPELINE:
//   Stage 0 : complex multiplications
//   Stage 1 : addition + Q32.32 -> Q16.16 + output register
//
// valid_out occurs one clock after valid input.
// ============================================================================

module lcmv_weight_vector_2x2 #(
    parameter W = 32
)(
    input  wire                   clk,
    input  wire                   rst,

    input  wire signed [W-1:0]    F11_re,
    input  wire signed [W-1:0]    F11_im,

    input  wire signed [W-1:0]    F12_re,
    input  wire signed [W-1:0]    F12_im,

    input  wire signed [W-1:0]    F21_re,
    input  wire signed [W-1:0]    F21_im,

    input  wire signed [W-1:0]    F22_re,
    input  wire signed [W-1:0]    F22_im,

    input  wire signed [W-1:0]    fGPS_re,
    input  wire signed [W-1:0]    fGPS_im,

    input  wire signed [W-1:0]    fJAMMER_re,
    input  wire signed [W-1:0]    fJAMMER_im,

    output reg signed [W-1:0]     w1_re,
    output reg signed [W-1:0]     w1_im,

    output reg signed [W-1:0]     w2_re,
    output reg signed [W-1:0]     w2_im,

    output reg                    valid
);

    // ========================================================================
    // Stage-0 complex multiplication results
    // ========================================================================

    reg signed [2*W-1:0] m11_re;
    reg signed [2*W-1:0] m11_im;

    reg signed [2*W-1:0] m12_re;
    reg signed [2*W-1:0] m12_im;

    reg signed [2*W-1:0] m21_re;
    reg signed [2*W-1:0] m21_im;

    reg signed [2*W-1:0] m22_re;
    reg signed [2*W-1:0] m22_im;

    reg valid_d;

    // ========================================================================
    // Stage-1 accumulators
    // ========================================================================

    reg signed [2*W:0] sum_w1_re;
    reg signed [2*W:0] sum_w1_im;

    reg signed [2*W:0] sum_w2_re;
    reg signed [2*W:0] sum_w2_im;

    // ========================================================================
    // PIPELINE
    // ========================================================================

    always @(posedge clk) begin

        if (rst) begin

            m11_re <= 0;
            m11_im <= 0;
            m12_re <= 0;
            m12_im <= 0;

            m21_re <= 0;
            m21_im <= 0;
            m22_re <= 0;
            m22_im <= 0;

            valid_d <= 1'b0;

            sum_w1_re <= 0;
            sum_w1_im <= 0;
            sum_w2_re <= 0;
            sum_w2_im <= 0;

            w1_re <= 0;
            w1_im <= 0;
            w2_re <= 0;
            w2_im <= 0;

            valid <= 1'b0;

        end
        else begin

            // ================================================================
            // STAGE 0
            // Complex multiplications
            // ================================================================

            m11_re <=
                  ($signed(F11_re) * $signed(fGPS_re))
                - ($signed(F11_im) * $signed(fGPS_im));

            m11_im <=
                  ($signed(F11_re) * $signed(fGPS_im))
                + ($signed(F11_im) * $signed(fGPS_re));

            m12_re <=
                  ($signed(F12_re) * $signed(fJAMMER_re))
                - ($signed(F12_im) * $signed(fJAMMER_im));

            m12_im <=
                  ($signed(F12_re) * $signed(fJAMMER_im))
                + ($signed(F12_im) * $signed(fJAMMER_re));

            m21_re <=
                  ($signed(F21_re) * $signed(fGPS_re))
                - ($signed(F21_im) * $signed(fGPS_im));

            m21_im <=
                  ($signed(F21_re) * $signed(fGPS_im))
                + ($signed(F21_im) * $signed(fGPS_re));

            m22_re <=
                  ($signed(F22_re) * $signed(fJAMMER_re))
                - ($signed(F22_im) * $signed(fJAMMER_im));

            m22_im <=
                  ($signed(F22_re) * $signed(fJAMMER_im))
                + ($signed(F22_im) * $signed(fJAMMER_re));

            valid_d <= 1'b1;

            // ================================================================
            // STAGE 1
            // Add the two products
            // ================================================================

            sum_w1_re <= $signed(m11_re) + $signed(m12_re);
            sum_w1_im <= $signed(m11_im) + $signed(m12_im);

            sum_w2_re <= $signed(m21_re) + $signed(m22_re);
            sum_w2_im <= $signed(m21_im) + $signed(m22_im);

            // ================================================================
            // OUTPUT REGISTER
            // Q32.32 -> Q16.16
            // ================================================================

            w1_re <= $signed(sum_w1_re >>> 16);
            w1_im <= $signed(sum_w1_im >>> 16);

            w2_re <= $signed(sum_w2_re >>> 16);
            w2_im <= $signed(sum_w2_im >>> 16);

            valid <= valid_d;

        end

    end

endmodule