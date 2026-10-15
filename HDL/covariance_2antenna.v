
`timescale 1ns/1ps

// ============================================================================
// covariance_2antenna.v
//
// TIMING-OPTIMIZED 2-ANTENNA COVARIANCE CALCULATOR
//
// Input:
//
//   X1 = I1 + jQ1
//   X2 = I2 + jQ2
//
// Covariance:
//
//        | R11              R12 |
//   R =  |                    |
//        | conj(R12)        R22 |
//
//   R11        = E[I1*I1 + Q1*Q1]
//   R22        = E[I2*I2 + Q2*Q2]
//
//   Re(R12)   = E[I1*I2 + Q1*Q2]
//   Im(R12)   = E[Q1*I2 - I1*Q2]
//
// ============================================================================
// TIMING OPTIMIZATION
//
// OLD:
//
//   ADC -> 16x16 multipliers -> add/sub -> 64-bit accumulator
//
// NEW:
//
//   Stage 1:
//      six 16x16 multiplications
//      registered
//
//   Stage 2:
//      covariance additions/subtractions
//      registered
//
//   Stage 3:
//      64-bit accumulation
//
// This removes the long ADC -> multiplier -> arithmetic -> accumulator path.
//
// ============================================================================
// RESOURCE INTENT
//
//   6 x 16x16 multipliers
//   Expected implementation: DSP48E1
//
//   No division operator.
//   No LUT reciprocal.
//   No large combinational multiplier feeding the accumulator.
//
// ============================================================================

module covariance_2antenna #(
    parameter IQ_WIDTH    = 16,
    parameter ACC_WIDTH   = 64,
    parameter COV_SAMPLES = 1024,
    parameter COV_SHIFT   = 10
)(
    input wire clk,
    input wire rst,

    // ------------------------------------------------------------
    // Start covariance calculation
    // ------------------------------------------------------------

    input wire start,

    // ------------------------------------------------------------
    // RX sample input
    // ------------------------------------------------------------

    input wire rx_valid,

    input wire signed [IQ_WIDTH-1:0] rx1_i,
    input wire signed [IQ_WIDTH-1:0] rx1_q,

    input wire signed [IQ_WIDTH-1:0] rx2_i,
    input wire signed [IQ_WIDTH-1:0] rx2_q,

    // ------------------------------------------------------------
    // Status
    // ------------------------------------------------------------

    output reg busy,
    output reg done,

    // ------------------------------------------------------------
    // Covariance outputs
    // ------------------------------------------------------------

    output reg signed [ACC_WIDTH-1:0] cov_r11,
    output reg signed [ACC_WIDTH-1:0] cov_r22,

    output reg signed [ACC_WIDTH-1:0] cov_r12_real,
    output reg signed [ACC_WIDTH-1:0] cov_r12_imag
);

    // ========================================================================
    // STATES
    // ========================================================================

    localparam ST_IDLE   = 3'd0;
    localparam ST_CAPTURE = 3'd1;
    localparam ST_FLUSH  = 3'd2;
    localparam ST_DIVIDE = 3'd3;
    localparam ST_DONE   = 3'd4;

    reg [2:0] state;

    // ========================================================================
    // SAMPLE COUNTER
    // ========================================================================

    localparam SAMPLE_CNT_WIDTH =
                    (COV_SAMPLES <= 2)    ? 1  :
                    (COV_SAMPLES <= 4)    ? 2  :
                    (COV_SAMPLES <= 8)    ? 3  :
                    (COV_SAMPLES <= 16)   ? 4  :
                    (COV_SAMPLES <= 32)   ? 5  :
                    (COV_SAMPLES <= 64)   ? 6  :
                    (COV_SAMPLES <= 128)  ? 7  :
                    (COV_SAMPLES <= 256)  ? 8  :
                    (COV_SAMPLES <= 512)  ? 9  :
                    (COV_SAMPLES <= 1024) ? 10 :
                    (COV_SAMPLES <= 2048) ? 11 :
                    (COV_SAMPLES <= 4096) ? 12 :
                    16;

    reg [SAMPLE_CNT_WIDTH-1:0] sample_count;

    // ========================================================================
    // ACCUMULATORS
    // ========================================================================

    reg signed [ACC_WIDTH-1:0] acc_r11;
    reg signed [ACC_WIDTH-1:0] acc_r22;
    reg signed [ACC_WIDTH-1:0] acc_r12_real;
    reg signed [ACC_WIDTH-1:0] acc_r12_imag;

    // ========================================================================
    // ========================================================================
    // PIPELINE STAGE 1
    // SIX 16x16 MULTIPLIERS
    // ========================================================================
    // ========================================================================

    reg signed [31:0] p_i1_i1;
    reg signed [31:0] p_q1_q1;

    reg signed [31:0] p_i2_i2;
    reg signed [31:0] p_q2_q2;

    reg signed [31:0] p_i1_i2;
    reg signed [31:0] p_q1_q2;

    reg signed [31:0] p_q1_i2;
    reg signed [31:0] p_i1_q2;

    // Valid pipeline
    reg pipe_valid0;
    reg pipe_valid1;

    // Last-sample marker
    reg pipe_last0;
    reg pipe_last1;

    // ========================================================================
    // STAGE 2
    // COVARIANCE TERMS
    // ========================================================================

    reg signed [32:0] term_r11;
    reg signed [32:0] term_r22;
    reg signed [32:0] term_r12_real;
    reg signed [32:0] term_r12_imag;

    // ========================================================================
    // SIGN EXTENSION HELPER
    // ========================================================================

    wire signed [ACC_WIDTH-1:0] ext_r11;
    wire signed [ACC_WIDTH-1:0] ext_r22;
    wire signed [ACC_WIDTH-1:0] ext_r12_real;
    wire signed [ACC_WIDTH-1:0] ext_r12_imag;

    assign ext_r11 =
        {{(ACC_WIDTH-33){term_r11[32]}}, term_r11};

    assign ext_r22 =
        {{(ACC_WIDTH-33){term_r22[32]}}, term_r22};

    assign ext_r12_real =
        {{(ACC_WIDTH-33){term_r12_real[32]}}, term_r12_real};

    assign ext_r12_imag =
        {{(ACC_WIDTH-33){term_r12_imag[32]}}, term_r12_imag};

    // ========================================================================
    // MAIN SEQUENTIAL LOGIC
    // ========================================================================

    always @(posedge clk) begin

        if (rst) begin

            state <= ST_IDLE;

            sample_count <= 0;

            acc_r11      <= 0;
            acc_r22      <= 0;
            acc_r12_real <= 0;
            acc_r12_imag <= 0;

            cov_r11      <= 0;
            cov_r22      <= 0;
            cov_r12_real <= 0;
            cov_r12_imag <= 0;

            busy <= 1'b0;
            done <= 1'b0;

            // Stage 1
            p_i1_i1 <= 0;
            p_q1_q1 <= 0;
            p_i2_i2 <= 0;
            p_q2_q2 <= 0;
            p_i1_i2 <= 0;
            p_q1_q2 <= 0;
            p_q1_i2 <= 0;
            p_i1_q2 <= 0;

            pipe_valid0 <= 1'b0;
            pipe_valid1 <= 1'b0;

            pipe_last0 <= 1'b0;
            pipe_last1 <= 1'b0;

            // Stage 2
            term_r11      <= 0;
            term_r22      <= 0;
            term_r12_real <= 0;
            term_r12_imag <= 0;

        end
        else begin

            // ------------------------------------------------------------
            // done = one-clock pulse
            // ------------------------------------------------------------

            done <= 1'b0;

            // ------------------------------------------------------------
            // PIPELINE VALID / LAST
            // ------------------------------------------------------------

            pipe_valid1 <= pipe_valid0;
            pipe_last1  <= pipe_last0;

            pipe_valid0 <= 1'b0;
            pipe_last0  <= 1'b0;

            // ============================================================
            // STAGE 1
            //
            // Input samples -> six registered multipliers
            //
            // ============================================================

            if (state == ST_CAPTURE) begin

                if (rx_valid) begin

                    // ----------------------------------------------------
                    // R11
                    // ----------------------------------------------------

                    p_i1_i1 <= $signed(rx1_i) * $signed(rx1_i);
                    p_q1_q1 <= $signed(rx1_q) * $signed(rx1_q);

                    // ----------------------------------------------------
                    // R22
                    // ----------------------------------------------------

                    p_i2_i2 <= $signed(rx2_i) * $signed(rx2_i);
                    p_q2_q2 <= $signed(rx2_q) * $signed(rx2_q);

                    // ----------------------------------------------------
                    // R12 real
                    // ----------------------------------------------------

                    p_i1_i2 <= $signed(rx1_i) * $signed(rx2_i);
                    p_q1_q2 <= $signed(rx1_q) * $signed(rx2_q);

                    // ----------------------------------------------------
                    // R12 imaginary
                    // ----------------------------------------------------

                    p_q1_i2 <= $signed(rx1_q) * $signed(rx2_i);
                    p_i1_q2 <= $signed(rx1_i) * $signed(rx2_q);

                    // Pipeline valid
                    pipe_valid0 <= 1'b1;

                    // Last accepted sample
                    if (sample_count == COV_SAMPLES-1) begin
                        pipe_last0 <= 1'b1;
                    end

                end

            end

            // ============================================================
            // STAGE 2
            //
            // Registered products -> covariance terms
            //
            // ============================================================

            if (pipe_valid0) begin

                // --------------------------------------------------------
                // R11 = I1^2 + Q1^2
                // --------------------------------------------------------

                term_r11 <=
                    $signed(p_i1_i1) +
                    $signed(p_q1_q1);

                // --------------------------------------------------------
                // R22 = I2^2 + Q2^2
                // --------------------------------------------------------

                term_r22 <=
                    $signed(p_i2_i2) +
                    $signed(p_q2_q2);

                // --------------------------------------------------------
                // Re(R12)
                //
                // = I1*I2 + Q1*Q2
                // --------------------------------------------------------

                term_r12_real <=
                    $signed(p_i1_i2) +
                    $signed(p_q1_q2);

                // --------------------------------------------------------
                // Im(R12)
                //
                // = Q1*I2 - I1*Q2
                // --------------------------------------------------------

                term_r12_imag <=
                    $signed(p_q1_i2) -
                    $signed(p_i1_q2);

            end

            // ============================================================
            // STAGE 3
            //
            // Covariance terms -> 64-bit accumulators
            //
            // ============================================================

            if (pipe_valid1) begin

                acc_r11 <=
                    acc_r11 + ext_r11;

                acc_r22 <=
                    acc_r22 + ext_r22;

                acc_r12_real <=
                    acc_r12_real + ext_r12_real;

                acc_r12_imag <=
                    acc_r12_imag + ext_r12_imag;

            end

            // ============================================================
            // FSM
            // ============================================================

            case (state)

                // ========================================================
                // IDLE
                // ========================================================

                ST_IDLE: begin

                    busy <= 1'b0;

                    if (start) begin

                        sample_count <= 0;

                        acc_r11      <= 0;
                        acc_r22      <= 0;
                        acc_r12_real <= 0;
                        acc_r12_imag <= 0;

                        busy <= 1'b1;

                        state <= ST_CAPTURE;

                    end

                end

                // ========================================================
                // CAPTURE
                // ========================================================

                ST_CAPTURE: begin

                    busy <= 1'b1;

                    if (rx_valid) begin

                        if (sample_count == COV_SAMPLES-1) begin

                            // Last input sample accepted.
                            //
                            // Products and covariance terms still need
                            // to travel through the pipeline.

                            state <= ST_FLUSH;

                        end
                        else begin

                            sample_count <= sample_count + 1'b1;

                        end

                    end

                end

                // ========================================================
                // FLUSH
                //
                // Wait for the final pipelined sample to reach the
                // accumulator.
                // ========================================================

                ST_FLUSH: begin

                    busy <= 1'b1;

                    if (pipe_valid1 && pipe_last1) begin

                        state <= ST_DIVIDE;

                    end

                end

                // ========================================================
                // DIVIDE
                //
                // Power-of-two division implemented as arithmetic shift.
                //
                // covariance = accumulator / COV_SAMPLES
                //
                // For 1024 samples:
                //
                // covariance = accumulator >>> 10
                //
                // ========================================================

                ST_DIVIDE: begin

                    busy <= 1'b1;

                    cov_r11 <=
                        acc_r11 >>> COV_SHIFT;

                    cov_r22 <=
                        acc_r22 >>> COV_SHIFT;

                    cov_r12_real <=
                        acc_r12_real >>> COV_SHIFT;

                    cov_r12_imag <=
                        acc_r12_imag >>> COV_SHIFT;

                    state <= ST_DONE;

                end

                // ========================================================
                // DONE
                // ========================================================

                ST_DONE: begin

                    busy <= 1'b0;

                    done <= 1'b1;

                    state <= ST_IDLE;

                end

                // ========================================================
                // DEFAULT
                // ========================================================

                default: begin

                    state <= ST_IDLE;

                    busy <= 1'b0;

                end

            endcase

        end

    end

endmodule
