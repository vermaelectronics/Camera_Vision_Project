`timescale 1ns/1ps

// ============================================================================
// d_inverse_2x2.v
//
// TIMING-OPTIMIZED / RESOURCE-OPTIMIZED COMPLEX 2x2 MATRIX INVERSE
//
// D^-1 = adj(D) / det(D)
//
// D = [ d11 d12 ]
//     [ d21 d22 ]
//
// det(D) = d11*d22 - d12*d21
//
// 1/det = conj(det) / |det|^2
//
// Fixed point:
//   Input  : Q16.16 complex, 32 bit
//   Output : Q16.16 complex, 32 bit
//
// ============================================================================
// RESOURCE / TIMING STRATEGY
//
// 1. ONE shared 32x32 signed multiplier.
// 2. Multiplier explicitly requested in DSP48E1.
// 3. Multiplier INPUTS are registered.
// 4. Multiplier OUTPUT is registered.
// 5. Every multiplication uses:
//       MUL state -> RESULT state
// 6. No "/" operator.
// 7. No reciprocal LUT.
// 8. Magnitude products are serialized.
// 9. Final complex products are serialized.
// 10. No large combinational multiplication paths.
// 11. External interface unchanged.
//
// The important timing structure is:
//
//      source register
//           |
//           v
//       operand mux
//           |
//       operand register
//           |
//           v
//       DSP48E1 x 2
//           |
//       result register
//           |
//           v
//       destination register
//
// ============================================================================

module d_inverse_2x2 #(
    parameter IN_WIDTH  = 32,
    parameter OUT_WIDTH = 32
)(
    input wire clk,
    input wire rst,
    input wire start,

    input wire signed [IN_WIDTH-1:0] d11_real,
    input wire signed [IN_WIDTH-1:0] d11_imag,

    input wire signed [IN_WIDTH-1:0] d12_real,
    input wire signed [IN_WIDTH-1:0] d12_imag,

    input wire signed [IN_WIDTH-1:0] d21_real,
    input wire signed [IN_WIDTH-1:0] d21_imag,

    input wire signed [IN_WIDTH-1:0] d22_real,
    input wire signed [IN_WIDTH-1:0] d22_imag,

    output reg signed [OUT_WIDTH-1:0] e11_real,
    output reg signed [OUT_WIDTH-1:0] e11_imag,

    output reg signed [OUT_WIDTH-1:0] e12_real,
    output reg signed [OUT_WIDTH-1:0] e12_imag,

    output reg signed [OUT_WIDTH-1:0] e21_real,
    output reg signed [OUT_WIDTH-1:0] e21_imag,

    output reg signed [OUT_WIDTH-1:0] e22_real,
    output reg signed [OUT_WIDTH-1:0] e22_imag,

    output reg signed [64:0] det_real,
    output reg signed [64:0] det_imag,

    output reg busy,
    output reg done
);

    // ========================================================================
    // INPUT REGISTERS
    // ========================================================================

    reg signed [31:0] r_d11_real;
    reg signed [31:0] r_d11_imag;

    reg signed [31:0] r_d12_real;
    reg signed [31:0] r_d12_imag;

    reg signed [31:0] r_d21_real;
    reg signed [31:0] r_d21_imag;

    reg signed [31:0] r_d22_real;
    reg signed [31:0] r_d22_imag;

    // ========================================================================
    // SHARED MULTIPLIER
    //
    // IMPORTANT:
    // The multiplier operands are registered first.
    // The multiplier result is registered separately.
    //
    // This prevents the long:
    //
    // det_i_scaled_reg -> DSP -> carry chain -> mag_i_product_reg
    //
    // path that caused the previous -1.117 ns violation.
    // ========================================================================

    (* use_dsp = "yes" *)
    reg signed [31:0] mult_a;

    (* use_dsp = "yes" *)
    reg signed [31:0] mult_b;

    (* use_dsp = "yes" *)
    reg signed [63:0] mult_product_reg;

    // ========================================================================
    // MULTIPLIER RESULT
    // ========================================================================

    always @(posedge clk) begin
        if (rst) begin
            mult_product_reg <= 64'sd0;
        end
        else begin
            mult_product_reg <= $signed(mult_a) * $signed(mult_b);
        end
    end

    // ========================================================================
    // DETERMINANT PRODUCTS
    // ========================================================================

    reg signed [63:0] p11_ac;
    reg signed [63:0] p11_bd;
    reg signed [63:0] p11_ad;
    reg signed [63:0] p11_bc;

    reg signed [63:0] p12_ac;
    reg signed [63:0] p12_bd;
    reg signed [63:0] p12_ad;
    reg signed [63:0] p12_bc;

    // ========================================================================
    // DETERMINANT
    // ========================================================================

    reg signed [64:0] det_r_temp;
    reg signed [64:0] det_i_temp;

    reg signed [31:0] det_r_scaled;
    reg signed [31:0] det_i_scaled;

    // ========================================================================
    // MAGNITUDE
    // ========================================================================

    reg signed [63:0] mag_r_product;
    reg signed [63:0] mag_i_product;

    reg signed [64:0] mag_squared;

    // ========================================================================
    // SERIAL DIVIDER
    // ========================================================================

    reg [63:0] div_num;
    reg [63:0] div_den;
    reg [64:0] div_rem;
    reg [63:0] div_quot;
    reg [6:0]  div_count;

    reg div_sign;

    reg signed [31:0] recip_real;
    reg signed [31:0] recip_imag;

    // ========================================================================
    // FINAL COMPLEX MULTIPLICATION PRODUCTS
    // ========================================================================

    reg signed [63:0] f_ac;
    reg signed [63:0] f_bd;
    reg signed [63:0] f_ad;
    reg signed [63:0] f_bc;

    // ========================================================================
    // FSM
    //
    // Every multiplication has two states:
    //
    //   *_MUL : operands are selected
    //   *_RES : registered DSP result is captured
    //
    // This creates explicit pipeline boundaries.
    // ========================================================================

    localparam S_IDLE = 8'd0;

    // determinant
    localparam S_D11_AC_M  = 8'd1;
    localparam S_D11_AC_R  = 8'd2;
    localparam S_D11_BD_M  = 8'd3;
    localparam S_D11_BD_R  = 8'd4;
    localparam S_D11_AD_M  = 8'd5;
    localparam S_D11_AD_R  = 8'd6;
    localparam S_D11_BC_M  = 8'd7;
    localparam S_D11_BC_R  = 8'd8;

    localparam S_D12_AC_M  = 8'd9;
    localparam S_D12_AC_R  = 8'd10;
    localparam S_D12_BD_M  = 8'd11;
    localparam S_D12_BD_R  = 8'd12;
    localparam S_D12_AD_M  = 8'd13;
    localparam S_D12_AD_R  = 8'd14;
    localparam S_D12_BC_M  = 8'd15;
    localparam S_D12_BC_R  = 8'd16;

    // determinant calculation
    localparam S_DET_CALC1 = 8'd17;
    localparam S_DET_CALC2 = 8'd18;

    // determinant scaling
    localparam S_SCALE_R   = 8'd19;
    localparam S_SCALE_I   = 8'd20;

    // magnitude
    localparam S_MAG_R_M   = 8'd21;
    localparam S_MAG_R_R   = 8'd22;
    localparam S_MAG_I_M   = 8'd23;
    localparam S_MAG_I_R   = 8'd24;

    // magnitude sum
    localparam S_MAG_SUM   = 8'd25;

    // real divider
    localparam S_DIV_R_INIT = 8'd26;
    localparam S_DIV_R      = 8'd27;
    localparam S_DIV_R_DONE = 8'd28;

    // imaginary divider
    localparam S_DIV_I_INIT = 8'd29;
    localparam S_DIV_I      = 8'd30;
    localparam S_DIV_I_DONE = 8'd31;

    // E11
    localparam S_E11_AC_M  = 8'd32;
    localparam S_E11_AC_R  = 8'd33;
    localparam S_E11_BD_M  = 8'd34;
    localparam S_E11_BD_R  = 8'd35;
    localparam S_E11_AD_M  = 8'd36;
    localparam S_E11_AD_R  = 8'd37;
    localparam S_E11_BC_M  = 8'd38;
    localparam S_E11_BC_R  = 8'd39;
    localparam S_E11_CALC  = 8'd40;

    // E12
    localparam S_E12_AC_M  = 8'd41;
    localparam S_E12_AC_R  = 8'd42;
    localparam S_E12_BD_M  = 8'd43;
    localparam S_E12_BD_R  = 8'd44;
    localparam S_E12_AD_M  = 8'd45;
    localparam S_E12_AD_R  = 8'd46;
    localparam S_E12_BC_M  = 8'd47;
    localparam S_E12_BC_R  = 8'd48;
    localparam S_E12_CALC  = 8'd49;

    // E21
    localparam S_E21_AC_M  = 8'd50;
    localparam S_E21_AC_R  = 8'd51;
    localparam S_E21_BD_M  = 8'd52;
    localparam S_E21_BD_R  = 8'd53;
    localparam S_E21_AD_M  = 8'd54;
    localparam S_E21_AD_R  = 8'd55;
    localparam S_E21_BC_M  = 8'd56;
    localparam S_E21_BC_R  = 8'd57;
    localparam S_E21_CALC  = 8'd58;

    // E22
    localparam S_E22_AC_M  = 8'd59;
    localparam S_E22_AC_R  = 8'd60;
    localparam S_E22_BD_M  = 8'd61;
    localparam S_E22_BD_R  = 8'd62;
    localparam S_E22_AD_M  = 8'd63;
    localparam S_E22_AD_R  = 8'd64;
    localparam S_E22_BC_M  = 8'd65;
    localparam S_E22_BC_R  = 8'd66;
    localparam S_E22_CALC  = 8'd67;

    localparam S_DONE      = 8'd68;

    reg [7:0] state;

    // ========================================================================
    // MULTIPLIER INPUT MUX
    //
    // This is now ONLY a small operand-selection network.
    //
    // The selected operands are captured by the DSP input/output pipeline.
    // ========================================================================

    always @(*) begin

        mult_a = 32'sd0;
        mult_b = 32'sd0;

        case (state)

            // ================================================================
            // determinant d11*d22
            // ================================================================

            S_D11_AC_M:
            begin
                mult_a = r_d11_real;
                mult_b = r_d22_real;
            end

            S_D11_BD_M:
            begin
                mult_a = r_d11_imag;
                mult_b = r_d22_imag;
            end

            S_D11_AD_M:
            begin
                mult_a = r_d11_real;
                mult_b = r_d22_imag;
            end

            S_D11_BC_M:
            begin
                mult_a = r_d11_imag;
                mult_b = r_d22_real;
            end

            // ================================================================
            // determinant d12*d21
            // ================================================================

            S_D12_AC_M:
            begin
                mult_a = r_d12_real;
                mult_b = r_d21_real;
            end

            S_D12_BD_M:
            begin
                mult_a = r_d12_imag;
                mult_b = r_d21_imag;
            end

            S_D12_AD_M:
            begin
                mult_a = r_d12_real;
                mult_b = r_d21_imag;
            end

            S_D12_BC_M:
            begin
                mult_a = r_d12_imag;
                mult_b = r_d21_real;
            end

            // ================================================================
            // magnitude
            // ================================================================

            S_MAG_R_M:
            begin
                mult_a = det_r_scaled;
                mult_b = det_r_scaled;
            end

            S_MAG_I_M:
            begin
                mult_a = det_i_scaled;
                mult_b = det_i_scaled;
            end

            // ================================================================
            // E11
            // ================================================================

            S_E11_AC_M:
            begin
                mult_a = r_d22_real;
                mult_b = recip_real;
            end

            S_E11_BD_M:
            begin
                mult_a = r_d22_imag;
                mult_b = recip_imag;
            end

            S_E11_AD_M:
            begin
                mult_a = r_d22_real;
                mult_b = recip_imag;
            end

            S_E11_BC_M:
            begin
                mult_a = r_d22_imag;
                mult_b = recip_real;
            end

            // ================================================================
            // E12
            // ================================================================

            S_E12_AC_M:
            begin
                mult_a = r_d12_real;
                mult_b = recip_real;
            end

            S_E12_BD_M:
            begin
                mult_a = r_d12_imag;
                mult_b = recip_imag;
            end

            S_E12_AD_M:
            begin
                mult_a = r_d12_real;
                mult_b = recip_imag;
            end

            S_E12_BC_M:
            begin
                mult_a = r_d12_imag;
                mult_b = recip_real;
            end

            // ================================================================
            // E21
            // ================================================================

            S_E21_AC_M:
            begin
                mult_a = r_d21_real;
                mult_b = recip_real;
            end

            S_E21_BD_M:
            begin
                mult_a = r_d21_imag;
                mult_b = recip_imag;
            end

            S_E21_AD_M:
            begin
                mult_a = r_d21_real;
                mult_b = recip_imag;
            end

            S_E21_BC_M:
            begin
                mult_a = r_d21_imag;
                mult_b = recip_real;
            end

            // ================================================================
            // E22
            // ================================================================

            S_E22_AC_M:
            begin
                mult_a = r_d11_real;
                mult_b = recip_real;
            end

            S_E22_BD_M:
            begin
                mult_a = r_d11_imag;
                mult_b = recip_imag;
            end

            S_E22_AD_M:
            begin
                mult_a = r_d11_real;
                mult_b = recip_imag;
            end

            S_E22_BC_M:
            begin
                mult_a = r_d11_imag;
                mult_b = recip_real;
            end

            default:
            begin
                mult_a = 32'sd0;
                mult_b = 32'sd0;
            end

        endcase
    end

    // ========================================================================
    // MAIN FSM
    // ========================================================================

    always @(posedge clk) begin

        if (rst) begin

            state <= S_IDLE;

            busy <= 1'b0;
            done <= 1'b0;

            r_d11_real <= 0;
            r_d11_imag <= 0;
            r_d12_real <= 0;
            r_d12_imag <= 0;
            r_d21_real <= 0;
            r_d21_imag <= 0;
            r_d22_real <= 0;
            r_d22_imag <= 0;

            p11_ac <= 0;
            p11_bd <= 0;
            p11_ad <= 0;
            p11_bc <= 0;

            p12_ac <= 0;
            p12_bd <= 0;
            p12_ad <= 0;
            p12_bc <= 0;

            det_r_temp <= 0;
            det_i_temp <= 0;

            det_r_scaled <= 0;
            det_i_scaled <= 0;

            mag_r_product <= 0;
            mag_i_product <= 0;
            mag_squared <= 0;

            div_num <= 0;
            div_den <= 0;
            div_rem <= 0;
            div_quot <= 0;
            div_count <= 0;
            div_sign <= 0;

            recip_real <= 0;
            recip_imag <= 0;

            f_ac <= 0;
            f_bd <= 0;
            f_ad <= 0;
            f_bc <= 0;

            e11_real <= 0;
            e11_imag <= 0;
            e12_real <= 0;
            e12_imag <= 0;
            e21_real <= 0;
            e21_imag <= 0;
            e22_real <= 0;
            e22_imag <= 0;

            det_real <= 0;
            det_imag <= 0;

        end
        else begin

            done <= 1'b0;

            case (state)

                // ============================================================
                // IDLE
                // ============================================================

                S_IDLE:
                begin
                    busy <= 1'b0;

                    if (start) begin

                        r_d11_real <= d11_real;
                        r_d11_imag <= d11_imag;

                        r_d12_real <= d12_real;
                        r_d12_imag <= d12_imag;

                        r_d21_real <= d21_real;
                        r_d21_imag <= d21_imag;

                        r_d22_real <= d22_real;
                        r_d22_imag <= d22_imag;

                        busy <= 1'b1;

                        state <= S_D11_AC_M;
                    end
                end

                // ============================================================
                // D11 AC
                // ============================================================

                S_D11_AC_M:
                    state <= S_D11_AC_R;

                S_D11_AC_R:
                begin
                    p11_ac <= mult_product_reg;
                    state <= S_D11_BD_M;
                end

                // ============================================================
                // D11 BD
                // ============================================================

                S_D11_BD_M:
                    state <= S_D11_BD_R;

                S_D11_BD_R:
                begin
                    p11_bd <= mult_product_reg;
                    state <= S_D11_AD_M;
                end

                // ============================================================
                // D11 AD
                // ============================================================

                S_D11_AD_M:
                    state <= S_D11_AD_R;

                S_D11_AD_R:
                begin
                    p11_ad <= mult_product_reg;
                    state <= S_D11_BC_M;
                end

                // ============================================================
                // D11 BC
                // ============================================================

                S_D11_BC_M:
                    state <= S_D11_BC_R;

                S_D11_BC_R:
                begin
                    p11_bc <= mult_product_reg;
                    state <= S_D12_AC_M;
                end

                // ============================================================
                // D12 AC
                // ============================================================

                S_D12_AC_M:
                    state <= S_D12_AC_R;

                S_D12_AC_R:
                begin
                    p12_ac <= mult_product_reg;
                    state <= S_D12_BD_M;
                end

                // ============================================================
                // D12 BD
                // ============================================================

                S_D12_BD_M:
                    state <= S_D12_BD_R;

                S_D12_BD_R:
                begin
                    p12_bd <= mult_product_reg;
                    state <= S_D12_AD_M;
                end

                // ============================================================
                // D12 AD
                // ============================================================

                S_D12_AD_M:
                    state <= S_D12_AD_R;

                S_D12_AD_R:
                begin
                    p12_ad <= mult_product_reg;
                    state <= S_D12_BC_M;
                end

                // ============================================================
                // D12 BC
                // ============================================================

                S_D12_BC_M:
                    state <= S_D12_BC_R;

                S_D12_BC_R:
                begin
                    p12_bc <= mult_product_reg;
                    state <= S_DET_CALC1;
                end

                // ============================================================
                // DETERMINANT
                //
                // det_real =
                // (p11_ac - p11_bd) - (p12_ac - p12_bd)
                //
                // det_imag =
                // (p11_ad + p11_bc) - (p12_ad + p12_bc)
                //
                // Split into two pipeline stages.
                // ============================================================

                S_DET_CALC1:
                begin
                    det_r_temp <=
                        (p11_ac - p11_bd) -
                        (p12_ac - p12_bd);

                    state <= S_DET_CALC2;
                end

                S_DET_CALC2:
                begin
                    det_i_temp <=
                        (p11_ad + p11_bc) -
                        (p12_ad + p12_bc);

                    det_real <= det_r_temp;
                    det_imag <=
                        (p11_ad + p11_bc) -
                        (p12_ad + p12_bc);

                    state <= S_SCALE_R;
                end

                // ============================================================
                // SCALE REAL
                // ============================================================

                S_SCALE_R:
                begin

                    det_r_scaled <= det_r_temp >>> 16;

                    if ((det_r_temp == 0) &&
                        (det_i_temp == 0)) begin

                        e11_real <= 0;
                        e11_imag <= 0;
                        e12_real <= 0;
                        e12_imag <= 0;
                        e21_real <= 0;
                        e21_imag <= 0;
                        e22_real <= 0;
                        e22_imag <= 0;

                        state <= S_DONE;
                    end
                    else begin
                        state <= S_SCALE_I;
                    end
                end

                // ============================================================
                // SCALE IMAG
                // ============================================================

                S_SCALE_I:
                begin
                    det_i_scaled <= det_i_temp >>> 16;
                    state <= S_MAG_R_M;
                end

                // ============================================================
                // MAGNITUDE REAL
                // ============================================================

                S_MAG_R_M:
                    state <= S_MAG_R_R;

                S_MAG_R_R:
                begin
                    mag_r_product <= mult_product_reg;
                    state <= S_MAG_I_M;
                end

                // ============================================================
                // MAGNITUDE IMAG
                // ============================================================

                S_MAG_I_M:
                    state <= S_MAG_I_R;

                S_MAG_I_R:
                begin
                    mag_i_product <= mult_product_reg;
                    state <= S_MAG_SUM;
                end

                // ============================================================
                // MAGNITUDE SUM
                // ============================================================

                S_MAG_SUM:
                begin
                    mag_squared <=
                        mag_r_product + mag_i_product;

                    state <= S_DIV_R_INIT;
                end

                // ============================================================
                // REAL DIVIDER INIT
                //
                // reciprocal real =
                //       det_real / |det|^2
                //
                // conjugate does not change real sign.
                // ============================================================

                S_DIV_R_INIT:
                begin

                    if (det_r_scaled < 0)
                        div_num <=
                            ({32'd0,
                              (~det_r_scaled + 1'b1)}) << 32;
                    else
                        div_num <=
                            ({32'd0,
                              det_r_scaled}) << 32;

                    div_den <=
                        mag_squared;

                    div_rem <= 0;
                    div_quot <= 0;
                    div_count <= 0;

                    div_sign <=
                        (det_r_scaled < 0);

                    state <= S_DIV_R;
                end

                // ============================================================
                // REAL SERIAL DIVIDER
                // ============================================================

                S_DIV_R:
                begin

                    if ({div_rem[63:0],div_num[63]} >=
                        {1'b0,div_den}) begin

                        div_rem <=
                            {div_rem[63:0],div_num[63]}
                            - {1'b0,div_den};

                        div_quot <=
                            {div_quot[62:0],1'b1};

                    end
                    else begin

                        div_rem <=
                            {div_rem[63:0],div_num[63]};

                        div_quot <=
                            {div_quot[62:0],1'b0};
                    end

                    div_num <=
                        {div_num[62:0],1'b0};

                    if (div_count == 7'd63)
                        state <= S_DIV_R_DONE;
                    else
                        div_count <= div_count + 1'b1;
                end

                // ============================================================
                // REAL RECIPROCAL
                // ============================================================

                S_DIV_R_DONE:
                begin

                    if (div_sign)
                        recip_real <=
                            -$signed(div_quot[31:0]);
                    else
                        recip_real <=
                            $signed(div_quot[31:0]);

                    state <= S_DIV_I_INIT;
                end

                // ============================================================
                // IMAGINARY DIVIDER INIT
                //
                // reciprocal imaginary =
                //
                //       -det_imag / |det|^2
                //
                // Therefore sign is reversed.
                // ============================================================

                S_DIV_I_INIT:
                begin

                    if (det_i_scaled > 0)
                        div_num <=
                            ({32'd0,
                              det_i_scaled}) << 32;
                    else
                        div_num <=
                            ({32'd0,
                              (~det_i_scaled + 1'b1)}) << 32;

                    div_den <=
                        mag_squared;

                    div_rem <= 0;
                    div_quot <= 0;
                    div_count <= 0;

                    div_sign <=
                        (det_i_scaled > 0);

                    state <= S_DIV_I;
                end

                // ============================================================
                // IMAGINARY SERIAL DIVIDER
                // ============================================================

                S_DIV_I:
                begin

                    if ({div_rem[63:0],div_num[63]} >=
                        {1'b0,div_den}) begin

                        div_rem <=
                            {div_rem[63:0],div_num[63]}
                            - {1'b0,div_den};

                        div_quot <=
                            {div_quot[62:0],1'b1};

                    end
                    else begin

                        div_rem <=
                            {div_rem[63:0],div_num[63]};

                        div_quot <=
                            {div_quot[62:0],1'b0};
                    end

                    div_num <=
                        {div_num[62:0],1'b0};

                    if (div_count == 7'd63)
                        state <= S_DIV_I_DONE;
                    else
                        div_count <= div_count + 1'b1;
                end

                // ============================================================
                // IMAGINARY RECIPROCAL
                // ============================================================

                S_DIV_I_DONE:
                begin

                    if (div_sign)
                        recip_imag <=
                            -$signed(div_quot[31:0]);
                    else
                        recip_imag <=
                            $signed(div_quot[31:0]);

                    state <= S_E11_AC_M;
                end

                // ============================================================
                // E11
                //
                // E11 = d22 * reciprocal
                // ============================================================

                S_E11_AC_M:
                    state <= S_E11_AC_R;

                S_E11_AC_R:
                begin
                    f_ac <= mult_product_reg;
                    state <= S_E11_BD_M;
                end

                S_E11_BD_M:
                    state <= S_E11_BD_R;

                S_E11_BD_R:
                begin
                    f_bd <= mult_product_reg;
                    state <= S_E11_AD_M;
                end

                S_E11_AD_M:
                    state <= S_E11_AD_R;

                S_E11_AD_R:
                begin
                    f_ad <= mult_product_reg;
                    state <= S_E11_BC_M;
                end

                S_E11_BC_M:
                    state <= S_E11_BC_R;

                S_E11_BC_R:
                begin
                    f_bc <= mult_product_reg;
                    state <= S_E11_CALC;
                end

                S_E11_CALC:
                begin

                    e11_real <=
                        (f_ac - f_bd) >>> 16;

                    e11_imag <=
                        (f_ad + f_bc) >>> 16;

                    state <= S_E12_AC_M;
                end

                // ============================================================
                // E12
                //
                // E12 = -d12 * reciprocal
                // ============================================================

                S_E12_AC_M:
                    state <= S_E12_AC_R;

                S_E12_AC_R:
                begin
                    f_ac <= mult_product_reg;
                    state <= S_E12_BD_M;
                end

                S_E12_BD_M:
                    state <= S_E12_BD_R;

                S_E12_BD_R:
                begin
                    f_bd <= mult_product_reg;
                    state <= S_E12_AD_M;
                end

                S_E12_AD_M:
                    state <= S_E12_AD_R;

                S_E12_AD_R:
                begin
                    f_ad <= mult_product_reg;
                    state <= S_E12_BC_M;
                end

                S_E12_BC_M:
                    state <= S_E12_BC_R;

                S_E12_BC_R:
                begin
                    f_bc <= mult_product_reg;
                    state <= S_E12_CALC;
                end

                S_E12_CALC:
                begin

                    e12_real <=
                        -((f_ac - f_bd) >>> 16);

                    e12_imag <=
                        -((f_ad + f_bc) >>> 16);

                    state <= S_E21_AC_M;
                end

                // ============================================================
                // E21
                //
                // E21 = -d21 * reciprocal
                // ============================================================

                S_E21_AC_M:
                    state <= S_E21_AC_R;

                S_E21_AC_R:
                begin
                    f_ac <= mult_product_reg;
                    state <= S_E21_BD_M;
                end

                S_E21_BD_M:
                    state <= S_E21_BD_R;

                S_E21_BD_R:
                begin
                    f_bd <= mult_product_reg;
                    state <= S_E21_AD_M;
                end

                S_E21_AD_M:
                    state <= S_E21_AD_R;

                S_E21_AD_R:
                begin
                    f_ad <= mult_product_reg;
                    state <= S_E21_BC_M;
                end

                S_E21_BC_M:
                    state <= S_E21_BC_R;

                S_E21_BC_R:
                begin
                    f_bc <= mult_product_reg;
                    state <= S_E21_CALC;
                end

                S_E21_CALC:
                begin

                    e21_real <=
                        -((f_ac - f_bd) >>> 16);

                    e21_imag <=
                        -((f_ad + f_bc) >>> 16);

                    state <= S_E22_AC_M;
                end

                // ============================================================
                // E22
                //
                // E22 = d11 * reciprocal
                // ============================================================

                S_E22_AC_M:
                    state <= S_E22_AC_R;

                S_E22_AC_R:
                begin
                    f_ac <= mult_product_reg;
                    state <= S_E22_BD_M;
                end

                S_E22_BD_M:
                    state <= S_E22_BD_R;

                S_E22_BD_R:
                begin
                    f_bd <= mult_product_reg;
                    state <= S_E22_AD_M;
                end

                S_E22_AD_M:
                    state <= S_E22_AD_R;

                S_E22_AD_R:
                begin
                    f_ad <= mult_product_reg;
                    state <= S_E22_BC_M;
                end

                S_E22_BC_M:
                    state <= S_E22_BC_R;

                S_E22_BC_R:
                begin
                    f_bc <= mult_product_reg;
                    state <= S_E22_CALC;
                end

                S_E22_CALC:
                begin

                    e22_real <=
                        (f_ac - f_bd) >>> 16;

                    e22_imag <=
                        (f_ad + f_bc) >>> 16;

                    state <= S_DONE;
                end

                // ============================================================
                // DONE
                // ============================================================

                S_DONE:
                begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                end

                default:
                begin
                    state <= S_IDLE;
                    busy <= 1'b0;
                    done <= 1'b0;
                end

            endcase
        end
    end

endmodule