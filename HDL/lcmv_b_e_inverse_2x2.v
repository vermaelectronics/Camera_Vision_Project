`timescale 1ns/1ps

// ============================================================================
// lcmv_b_e_inverse_2x2.v
//
// F = B * E^-1
//
// B    : 32-bit signed complex Q16.16
// E^-1 : 32-bit signed complex Q16.16
// F    : 32-bit signed complex Q16.16
//
// Complex multiplication:
//
// (a + jb)(c + jd)
//   real = ac - bd
//   imag = ad + bc
//
// Since both operands are Q16.16:
//
//   32-bit x 32-bit = 64-bit Q32.32
//
// After accumulation:
//
//   result >>> 16
//
// returns Q16.16.
//
// Matrix:
//
//       [B11 B12]   [EI11 EI12]
// F  =  [B21 B22] * [EI21 EI22]
//
// ============================================================================

module lcmv_b_e_inverse_2x2 #(
    parameter W = 32
)(
    input  wire                 clk,
    input  wire                 rst,

    // ------------------------------------------------------------------------
    // B matrix
    // ------------------------------------------------------------------------
    input  wire signed [W-1:0] b11_re,
    input  wire signed [W-1:0] b11_im,

    input  wire signed [W-1:0] b12_re,
    input  wire signed [W-1:0] b12_im,

    input  wire signed [W-1:0] b21_re,
    input  wire signed [W-1:0] b21_im,

    input  wire signed [W-1:0] b22_re,
    input  wire signed [W-1:0] b22_im,

    // ------------------------------------------------------------------------
    // E^-1 matrix
    // ------------------------------------------------------------------------
    input  wire signed [W-1:0] ei11_re,
    input  wire signed [W-1:0] ei11_im,

    input  wire signed [W-1:0] ei12_re,
    input  wire signed [W-1:0] ei12_im,

    input  wire signed [W-1:0] ei21_re,
    input  wire signed [W-1:0] ei21_im,

    input  wire signed [W-1:0] ei22_re,
    input  wire signed [W-1:0] ei22_im,

    input  wire                 valid_in,

    // ------------------------------------------------------------------------
    // F = B * E^-1
    // ------------------------------------------------------------------------
    output reg signed [W-1:0] f11_re,
    output reg signed [W-1:0] f11_im,

    output reg signed [W-1:0] f12_re,
    output reg signed [W-1:0] f12_im,

    output reg signed [W-1:0] f21_re,
    output reg signed [W-1:0] f21_im,

    output reg signed [W-1:0] f22_re,
    output reg signed [W-1:0] f22_im,

    output reg                 valid_out
);

    // ------------------------------------------------------------------------
    // 32 x 32 -> 64 bit products
    // ------------------------------------------------------------------------

    reg signed [63:0] p1;
    reg signed [63:0] p2;
    reg signed [63:0] p3;
    reg signed [63:0] p4;

    reg signed [63:0] q1;
    reg signed [63:0] q2;
    reg signed [63:0] q3;
    reg signed [63:0] q4;

    reg signed [64:0] sum1;
    reg signed [64:0] sum2;
    reg signed [64:0] sum3;
    reg signed [64:0] sum4;

    reg signed [64:0] isum1;
    reg signed [64:0] isum2;
    reg signed [64:0] isum3;
    reg signed [64:0] isum4;

    // ------------------------------------------------------------------------
    // Intermediate scaled results
    // ------------------------------------------------------------------------

    reg signed [64:0] f11_re_calc;
    reg signed [64:0] f11_im_calc;

    reg signed [64:0] f12_re_calc;
    reg signed [64:0] f12_im_calc;

    reg signed [64:0] f21_re_calc;
    reg signed [64:0] f21_im_calc;

    reg signed [64:0] f22_re_calc;
    reg signed [64:0] f22_im_calc;

    // ------------------------------------------------------------------------
    // Combinational complex matrix multiplication
    // ------------------------------------------------------------------------

    always @(*) begin

        // ====================================================================
        // F11
        //
        // F11 = B11*EI11 + B12*EI21
        // ====================================================================

        p1 = b11_re * ei11_re;
        p2 = b11_im * ei11_im;

        p3 = b12_re * ei21_re;
        p4 = b12_im * ei21_im;

        sum1 = {p1[63],p1} - {p2[63],p2}
             + {p3[63],p3} - {p4[63],p4};

        q1 = b11_re * ei11_im;
        q2 = b11_im * ei11_re;

        q3 = b12_re * ei21_im;
        q4 = b12_im * ei21_re;

        isum1 = {q1[63],q1} + {q2[63],q2}
              + {q3[63],q3} + {q4[63],q4};


        // ====================================================================
        // F12
        //
        // F12 = B11*EI12 + B12*EI22
        // ====================================================================

        p1 = b11_re * ei12_re;
        p2 = b11_im * ei12_im;

        p3 = b12_re * ei22_re;
        p4 = b12_im * ei22_im;

        sum2 = {p1[63],p1} - {p2[63],p2}
             + {p3[63],p3} - {p4[63],p4};

        q1 = b11_re * ei12_im;
        q2 = b11_im * ei12_re;

        q3 = b12_re * ei22_im;
        q4 = b12_im * ei22_re;

        isum2 = {q1[63],q1} + {q2[63],q2}
              + {q3[63],q3} + {q4[63],q4};


        // ====================================================================
        // F21
        //
        // F21 = B21*EI11 + B22*EI21
        // ====================================================================

        p1 = b21_re * ei11_re;
        p2 = b21_im * ei11_im;

        p3 = b22_re * ei21_re;
        p4 = b22_im * ei21_im;

        sum3 = {p1[63],p1} - {p2[63],p2}
             + {p3[63],p3} - {p4[63],p4};

        q1 = b21_re * ei11_im;
        q2 = b21_im * ei11_re;

        q3 = b22_re * ei21_im;
        q4 = b22_im * ei21_re;

        isum3 = {q1[63],q1} + {q2[63],q2}
              + {q3[63],q3} + {q4[63],q4};


        // ====================================================================
        // F22
        //
        // F22 = B21*EI12 + B22*EI22
        // ====================================================================

        p1 = b21_re * ei12_re;
        p2 = b21_im * ei12_im;

        p3 = b22_re * ei22_re;
        p4 = b22_im * ei22_im;

        sum4 = {p1[63],p1} - {p2[63],p2}
             + {p3[63],p3} - {p4[63],p4};

        q1 = b21_re * ei12_im;
        q2 = b21_im * ei12_re;

        q3 = b22_re * ei22_im;
        q4 = b22_im * ei22_re;

        isum4 = {q1[63],q1} + {q2[63],q2}
              + {q3[63],q3} + {q4[63],q4};


        // ====================================================================
        // Q32.32 -> Q16.16
        // ====================================================================

        f11_re_calc = sum1  >>> 16;
        f11_im_calc = isum1 >>> 16;

        f12_re_calc = sum2  >>> 16;
        f12_im_calc = isum2 >>> 16;

        f21_re_calc = sum3  >>> 16;
        f21_im_calc = isum3 >>> 16;

        f22_re_calc = sum4  >>> 16;
        f22_im_calc = isum4 >>> 16;

    end

    // ------------------------------------------------------------------------
    // Register output
    // ------------------------------------------------------------------------

    always @(posedge clk) begin

        if (rst) begin

            f11_re <= 0;
            f11_im <= 0;
            f12_re <= 0;
            f12_im <= 0;
            f21_re <= 0;
            f21_im <= 0;
            f22_re <= 0;
            f22_im <= 0;

            valid_out <= 1'b0;

        end
        else begin

            valid_out <= valid_in;

            if (valid_in) begin

                f11_re <= f11_re_calc[31:0];
                f11_im <= f11_im_calc[31:0];

                f12_re <= f12_re_calc[31:0];
                f12_im <= f12_im_calc[31:0];

                f21_re <= f21_re_calc[31:0];
                f21_im <= f21_im_calc[31:0];

                f22_re <= f22_re_calc[31:0];
                f22_im <= f22_im_calc[31:0];

            end

        end

    end

endmodule