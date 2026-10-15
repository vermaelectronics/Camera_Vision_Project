
`timescale 1ns/1ps

// ============================================================================
// lcmv_r_inverse_c_2x2.v
//
// TIMING-OPTIMIZED:
//
//              B = R^-1 * C
//
// R^-1 : Q16.16, 32-bit complex
// C    : Q1.15,  16-bit complex
// B    : Q16.16, 32-bit complex
//
// Matrix:
//
//              | RI11 RI12 |   | C11 C12 |
//       B =    |            | * |         |
//              | RI21 RI22 |   | C21 C22 |
//
// Therefore:
//
// B11 = RI11*C11 + RI12*C21
// B12 = RI11*C12 + RI12*C22
// B21 = RI21*C11 + RI22*C21
// B22 = RI21*C12 + RI22*C22
//
// Complex multiplication:
//
// (a + jb)(c + jd)
//
// real = ac - bd
// imag = ad + bc
//
// Scaling:
//
// R^-1 = Q16.16
// C    = Q1.15
// product = Q17.31
// >>> 15
// result  = Q16.16
//
// ============================================================================
// PIPELINE
//
// Stage 1:
//     32 real multiplications
//     registered
//
// Stage 2:
//     8 real/imag additions
//     >>> 15
//     output registered
//
// valid_in -> Stage 1 -> Stage 2 -> valid_out
//
// ============================================================================

module lcmv_r_inverse_c_2x2 #(
    parameter RW = 32,
    parameter CW = 16,
    parameter BW = 32
)(
    input wire clk,
    input wire rst,

    // ------------------------------------------------------------------------
    // R^-1
    // ------------------------------------------------------------------------

    input wire signed [RW-1:0] ri11_re,
    input wire signed [RW-1:0] ri11_im,

    input wire signed [RW-1:0] ri12_re,
    input wire signed [RW-1:0] ri12_im,

    input wire signed [RW-1:0] ri21_re,
    input wire signed [RW-1:0] ri21_im,

    input wire signed [RW-1:0] ri22_re,
    input wire signed [RW-1:0] ri22_im,

    // ------------------------------------------------------------------------
    // C steering matrix
    // ------------------------------------------------------------------------

    input wire signed [CW-1:0] c11_re,
    input wire signed [CW-1:0] c11_im,

    input wire signed [CW-1:0] c12_re,
    input wire signed [CW-1:0] c12_im,

    input wire signed [CW-1:0] c21_re,
    input wire signed [CW-1:0] c21_im,

    input wire signed [CW-1:0] c22_re,
    input wire signed [CW-1:0] c22_im,

    input wire valid_in,

    // ------------------------------------------------------------------------
    // B = R^-1 * C
    // ------------------------------------------------------------------------

    output reg signed [BW-1:0] b11_re,
    output reg signed [BW-1:0] b11_im,

    output reg signed [BW-1:0] b12_re,
    output reg signed [BW-1:0] b12_im,

    output reg signed [BW-1:0] b21_re,
    output reg signed [BW-1:0] b21_im,

    output reg signed [BW-1:0] b22_re,
    output reg signed [BW-1:0] b22_im,

    output reg valid_out
);

    // ========================================================================
    // PRODUCT WIDTH
    // ========================================================================

    localparam PW = RW + CW;

    // ========================================================================
    // STAGE 1 PRODUCT REGISTERS
    // ========================================================================
    //
    // Every complex multiplication is:
    //
    // real = ar*br - ai*bi
    // imag = ar*bi + ai*br
    //
    // 8 complex products x 4 real multiplications = 32 products.
    //
    // The important optimization is that ALL multiplication results are
    // registered before the additions.
    //
    // ========================================================================

    reg signed [PW-1:0] p11_ac;
    reg signed [PW-1:0] p11_bd;
    reg signed [PW-1:0] p11_ad;
    reg signed [PW-1:0] p11_bc;

    reg signed [PW-1:0] p12_ac;
    reg signed [PW-1:0] p12_bd;
    reg signed [PW-1:0] p12_ad;
    reg signed [PW-1:0] p12_bc;

    reg signed [PW-1:0] p13_ac;
    reg signed [PW-1:0] p13_bd;
    reg signed [PW-1:0] p13_ad;
    reg signed [PW-1:0] p13_bc;

    reg signed [PW-1:0] p14_ac;
    reg signed [PW-1:0] p14_bd;
    reg signed [PW-1:0] p14_ad;
    reg signed [PW-1:0] p14_bc;

    reg signed [PW-1:0] p21_ac;
    reg signed [PW-1:0] p21_bd;
    reg signed [PW-1:0] p21_ad;
    reg signed [PW-1:0] p21_bc;

    reg signed [PW-1:0] p22_ac;
    reg signed [PW-1:0] p22_bd;
    reg signed [PW-1:0] p22_ad;
    reg signed [PW-1:0] p22_bc;

    reg signed [PW-1:0] p23_ac;
    reg signed [PW-1:0] p23_bd;
    reg signed [PW-1:0] p23_ad;
    reg signed [PW-1:0] p23_bc;

    reg signed [PW-1:0] p24_ac;
    reg signed [PW-1:0] p24_bd;
    reg signed [PW-1:0] p24_ad;
    reg signed [PW-1:0] p24_bc;

    // ========================================================================
    // VALID PIPELINE
    // ========================================================================

    reg valid_stage1;

    // ========================================================================
    // STAGE 2 SUM WIRES
    // ========================================================================
    //
    // 48-bit product + 48-bit product = 49 bits.
    //
    // ========================================================================

    reg signed [PW:0] s11_re;
    reg signed [PW:0] s11_im;

    reg signed [PW:0] s12_re;
    reg signed [PW:0] s12_im;

    reg signed [PW:0] s21_re;
    reg signed [PW:0] s21_im;

    reg signed [PW:0] s22_re;
    reg signed [PW:0] s22_im;

    // ========================================================================
    // MAIN PIPELINE
    // ========================================================================

    always @(posedge clk) begin

        if (rst) begin

            // ---------------------------------------------------------------
            // Product registers
            // ---------------------------------------------------------------

            p11_ac <= 0;
            p11_bd <= 0;
            p11_ad <= 0;
            p11_bc <= 0;

            p12_ac <= 0;
            p12_bd <= 0;
            p12_ad <= 0;
            p12_bc <= 0;

            p13_ac <= 0;
            p13_bd <= 0;
            p13_ad <= 0;
            p13_bc <= 0;

            p14_ac <= 0;
            p14_bd <= 0;
            p14_ad <= 0;
            p14_bc <= 0;

            p21_ac <= 0;
            p21_bd <= 0;
            p21_ad <= 0;
            p21_bc <= 0;

            p22_ac <= 0;
            p22_bd <= 0;
            p22_ad <= 0;
            p22_bc <= 0;

            p23_ac <= 0;
            p23_bd <= 0;
            p23_ad <= 0;
            p23_bc <= 0;

            p24_ac <= 0;
            p24_bd <= 0;
            p24_ad <= 0;
            p24_bc <= 0;

            valid_stage1 <= 1'b0;

            // ---------------------------------------------------------------
            // Outputs
            // ---------------------------------------------------------------

            b11_re <= 0;
            b11_im <= 0;

            b12_re <= 0;
            b12_im <= 0;

            b21_re <= 0;
            b21_im <= 0;

            b22_re <= 0;
            b22_im <= 0;

            valid_out <= 1'b0;

        end
        else begin

            // ================================================================
            // VALID PIPELINE
            // ================================================================

            valid_out <= valid_stage1;

            valid_stage1 <= valid_in;

            // ================================================================
            // STAGE 1
            //
            // Register all multiplication results.
            // ================================================================

            if (valid_in) begin

                // ------------------------------------------------------------
                // B11
                //
                // RI11 * C11
                // ------------------------------------------------------------

                p11_ac <= $signed(ri11_re) * $signed(c11_re);
                p11_bd <= $signed(ri11_im) * $signed(c11_im);
                p11_ad <= $signed(ri11_re) * $signed(c11_im);
                p11_bc <= $signed(ri11_im) * $signed(c11_re);

                // ------------------------------------------------------------
                // RI12 * C21
                // ------------------------------------------------------------

                p12_ac <= $signed(ri12_re) * $signed(c21_re);
                p12_bd <= $signed(ri12_im) * $signed(c21_im);
                p12_ad <= $signed(ri12_re) * $signed(c21_im);
                p12_bc <= $signed(ri12_im) * $signed(c21_re);

                // ------------------------------------------------------------
                // B12
                //
                // RI11 * C12
                // ------------------------------------------------------------

                p13_ac <= $signed(ri11_re) * $signed(c12_re);
                p13_bd <= $signed(ri11_im) * $signed(c12_im);
                p13_ad <= $signed(ri11_re) * $signed(c12_im);
                p13_bc <= $signed(ri11_im) * $signed(c12_re);

                // ------------------------------------------------------------
                // RI12 * C22
                // ------------------------------------------------------------

                p14_ac <= $signed(ri12_re) * $signed(c22_re);
                p14_bd <= $signed(ri12_im) * $signed(c22_im);
                p14_ad <= $signed(ri12_re) * $signed(c22_im);
                p14_bc <= $signed(ri12_im) * $signed(c22_re);

                // ------------------------------------------------------------
                // B21
                //
                // RI21 * C11
                // ------------------------------------------------------------

                p21_ac <= $signed(ri21_re) * $signed(c11_re);
                p21_bd <= $signed(ri21_im) * $signed(c11_im);
                p21_ad <= $signed(ri21_re) * $signed(c11_im);
                p21_bc <= $signed(ri21_im) * $signed(c11_re);

                // ------------------------------------------------------------
                // RI22 * C21
                // ------------------------------------------------------------

                p22_ac <= $signed(ri22_re) * $signed(c21_re);
                p22_bd <= $signed(ri22_im) * $signed(c21_im);
                p22_ad <= $signed(ri22_re) * $signed(c21_im);
                p22_bc <= $signed(ri22_im) * $signed(c21_re);

                // ------------------------------------------------------------
                // B22
                //
                // RI21 * C12
                // ------------------------------------------------------------

                p23_ac <= $signed(ri21_re) * $signed(c12_re);
                p23_bd <= $signed(ri21_im) * $signed(c12_im);
                p23_ad <= $signed(ri21_re) * $signed(c12_im);
                p23_bc <= $signed(ri21_im) * $signed(c12_re);

                // ------------------------------------------------------------
                // RI22 * C22
                // ------------------------------------------------------------

                p24_ac <= $signed(ri22_re) * $signed(c22_re);
                p24_bd <= $signed(ri22_im) * $signed(c22_im);
                p24_ad <= $signed(ri22_re) * $signed(c22_im);
                p24_bc <= $signed(ri22_im) * $signed(c22_re);

            end

            // ================================================================
            // STAGE 2
            //
            // Complex additions and scaling.
            // ================================================================

            if (valid_stage1) begin

                // ------------------------------------------------------------
                // B11
                //
                // real = RI11*C11(real) + RI12*C21(real)
                // imag = RI11*C11(imag) + RI12*C21(imag)
                // ------------------------------------------------------------

                s11_re =
                    $signed(p11_ac) -
                    $signed(p11_bd) +
                    $signed(p12_ac) -
                    $signed(p12_bd);

                s11_im =
                    $signed(p11_ad) +
                    $signed(p11_bc) +
                    $signed(p12_ad) +
                    $signed(p12_bc);

                // ------------------------------------------------------------
                // B12
                // ------------------------------------------------------------

                s12_re =
                    $signed(p13_ac) -
                    $signed(p13_bd) +
                    $signed(p14_ac) -
                    $signed(p14_bd);

                s12_im =
                    $signed(p13_ad) +
                    $signed(p13_bc) +
                    $signed(p14_ad) +
                    $signed(p14_bc);

                // ------------------------------------------------------------
                // B21
                // ------------------------------------------------------------

                s21_re =
                    $signed(p21_ac) -
                    $signed(p21_bd) +
                    $signed(p22_ac) -
                    $signed(p22_bd);

                s21_im =
                    $signed(p21_ad) +
                    $signed(p21_bc) +
                    $signed(p22_ad) +
                    $signed(p22_bc);

                // ------------------------------------------------------------
                // B22
                // ------------------------------------------------------------

                s22_re =
                    $signed(p23_ac) -
                    $signed(p23_bd) +
                    $signed(p24_ac) -
                    $signed(p24_bd);

                s22_im =
                    $signed(p23_ad) +
                    $signed(p23_bc) +
                    $signed(p24_ad) +
                    $signed(p24_bc);

                // ------------------------------------------------------------
                // Q17.31 -> Q16.16
                // ------------------------------------------------------------

                b11_re <= $signed(s11_re) >>> 16;
                b11_im <= $signed(s11_im) >>> 16;

                b12_re <= $signed(s12_re) >>> 16;
                b12_im <= $signed(s12_im) >>> 16;

                b21_re <= $signed(s21_re) >>> 16;
                b21_im <= $signed(s21_im) >>> 16;

                b22_re <= $signed(s22_re) >>> 16;
                b22_im <= $signed(s22_im) >>> 16;

            end

        end

    end

endmodule








