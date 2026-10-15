`timescale 1ns/1ps

// ============================================================================
// lcmv_beamformer_2ch.v
//
// RESOURCE / TIMING OPTIMIZED 2-CHANNEL LCMV BEAMFORMER
//
//                         Y = w^H * X
//
//       Y = conj(w1)*X1 + conj(w2)*X2
//
// Complex arithmetic:
//
//       conj(w)*X:
//
//       real = w_re*X_re + w_im*X_im
//       imag = w_re*X_im - w_im*X_re
//
// DATA FORMAT:
//
//       w1/w2 : Q16.16 complex, 32 bits
//       X1/X2 : Q16.16 complex, 32 bits
//       products : Q32.32
//       output   : Q16.16
//
// ============================================================================
//
// PIPELINE:
//
//   STAGE 1
//   --------
//       8 real multiplications
//       REGISTER
//
//   STAGE 2
//   --------
//       Complex multiplication additions/subtractions
//       REGISTER
//
//   STAGE 3
//   --------
//       Antenna-1 + antenna-2
//       REGISTER
//
//   STAGE 4
//   --------
//       Q32.32 -> Q16.16
//       REGISTER
//
// Therefore:
//
//       rx_valid -> valid = 4 clock cycles
//
// IMPORTANT:
//   Weights and RX samples are used from the same input cycle.
//   Weight values are pipelined together with the corresponding sample
//   through the arithmetic pipeline.
//
// ============================================================================

module lcmv_beamformer_2ch #(
    parameter W = 32
)(
    input  wire                   clk,
    input  wire                   rst,

    input  wire signed [W-1:0]    w1_re,
    input  wire signed [W-1:0]    w1_im,

    input  wire signed [W-1:0]    w2_re,
    input  wire signed [W-1:0]    w2_im,

    input  wire signed [W-1:0]    X1_re,
    input  wire signed [W-1:0]    X1_im,

    input  wire signed [W-1:0]    X2_re,
    input  wire signed [W-1:0]    X2_im,

    input  wire                   rx_valid,

    output reg signed [W-1:0]     Y_re,
    output reg signed [W-1:0]     Y_im,

    output reg                    valid
);

    // =========================================================================
    // WIDTH DEFINITIONS
    // =========================================================================

    localparam PW = 2*W;      // Q32.32 product width
    localparam AW = PW + 1;   // extra bit for addition

    // =========================================================================
    // STAGE 1
    //
    // ONLY MULTIPLICATIONS.
    //
    // IMPORTANT:
    // No addition is performed in this stage.
    //
    // This breaks the critical path:
    //
    //       weight register
    //              |
    //          multiplier
    //              |
    //          REGISTER
    //
    // =========================================================================

    reg signed [PW-1:0] p11_rr;
    reg signed [PW-1:0] p11_ii;
    reg signed [PW-1:0] p11_ri;
    reg signed [PW-1:0] p11_ir;

    reg signed [PW-1:0] p22_rr;
    reg signed [PW-1:0] p22_ii;
    reg signed [PW-1:0] p22_ri;
    reg signed [PW-1:0] p22_ir;

    reg valid_s1;


    // =========================================================================
    // STAGE 2
    //
    // Form the two complex products.
    //
    // antenna 1:
    //
    //       A1_re = w1_re*X1_re + w1_im*X1_im
    //       A1_im = w1_re*X1_im - w1_im*X1_re
    //
    // antenna 2:
    //
    //       A2_re = w2_re*X2_re + w2_im*X2_im
    //       A2_im = w2_re*X2_im - w2_im*X2_re
    //
    // =========================================================================

    reg signed [AW-1:0] a1_re;
    reg signed [AW-1:0] a1_im;

    reg signed [AW-1:0] a2_re;
    reg signed [AW-1:0] a2_im;

    reg valid_s2;


    // =========================================================================
    // STAGE 3
    //
    // Add the two antenna contributions.
    //
    //       Y_re = A1_re + A2_re
    //       Y_im = A1_im + A2_im
    //
    // =========================================================================

    reg signed [AW:0] sum_re;
    reg signed [AW:0] sum_im;

    reg valid_s3;


    // =========================================================================
    // STAGE 4
    //
    // Convert Q32.32 to Q16.16.
    //
    // Arithmetic right shift by 16.
    //
    // =========================================================================

    always @(posedge clk) begin

        if (rst) begin

            // -----------------------------------------------------------------
            // STAGE 1 RESET
            // -----------------------------------------------------------------

            p11_rr <= 0;
            p11_ii <= 0;
            p11_ri <= 0;
            p11_ir <= 0;

            p22_rr <= 0;
            p22_ii <= 0;
            p22_ri <= 0;
            p22_ir <= 0;

            valid_s1 <= 1'b0;

            // -----------------------------------------------------------------
            // STAGE 2 RESET
            // -----------------------------------------------------------------

            a1_re <= 0;
            a1_im <= 0;

            a2_re <= 0;
            a2_im <= 0;

            valid_s2 <= 1'b0;

            // -----------------------------------------------------------------
            // STAGE 3 RESET
            // -----------------------------------------------------------------

            sum_re <= 0;
            sum_im <= 0;

            valid_s3 <= 1'b0;

            // -----------------------------------------------------------------
            // STAGE 4 RESET
            // -----------------------------------------------------------------

            Y_re <= 0;
            Y_im <= 0;

            valid <= 1'b0;

        end
        else begin

            // =================================================================
            // STAGE 1
            //
            // Eight independent real multiplications.
            //
            // NO ADDITION HERE.
            //
            // =================================================================

            // Antenna 1

            p11_rr <= $signed(w1_re) * $signed(X1_re);
            p11_ii <= $signed(w1_im) * $signed(X1_im);

            p11_ri <= $signed(w1_re) * $signed(X1_im);
            p11_ir <= $signed(w1_im) * $signed(X1_re);

            // Antenna 2

            p22_rr <= $signed(w2_re) * $signed(X2_re);
            p22_ii <= $signed(w2_im) * $signed(X2_im);

            p22_ri <= $signed(w2_re) * $signed(X2_im);
            p22_ir <= $signed(w2_im) * $signed(X2_re);

            valid_s1 <= rx_valid;


            // =================================================================
            // STAGE 2
            //
            // Complex multiplication addition/subtraction.
            //
            // The multipliers are now BEFORE a register, so the critical
            // multiplier + adder path is divided into two timing stages.
            //
            // =================================================================

            a1_re <= $signed(p11_rr) + $signed(p11_ii);
            a1_im <= $signed(p11_ri) - $signed(p11_ir);

            a2_re <= $signed(p22_rr) + $signed(p22_ii);
            a2_im <= $signed(p22_ri) - $signed(p22_ir);

            valid_s2 <= valid_s1;


            // =================================================================
            // STAGE 3
            //
            // Add antenna contributions.
            //
            // =================================================================

            sum_re <= $signed(a1_re) + $signed(a2_re);
            sum_im <= $signed(a1_im) + $signed(a2_im);

            valid_s3 <= valid_s2;


            // =================================================================
            // STAGE 4
            //
            // Q32.32 -> Q16.16
            //
            // Arithmetic shift preserves the sign.
            //
            // =================================================================

            Y_re <= $signed(sum_re >>> 16);
            Y_im <= $signed(sum_im >>> 16);

            valid <= valid_s3;

        end

    end

endmodule