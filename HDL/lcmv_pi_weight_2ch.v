`timescale 1ns/1ps

// ============================================================================
// lcmv_pi_weight_2ch.v
//
// LCMV POWER-INVERSION (LCMV-PI) WEIGHT SOLVER, 2 ANTENNAS
//
// Reference:
//   F. Wang, H. Liu, C. Zhang, "Anti-Jamming Front-End Design of Satellite
//   Navigation Receiver", ACES Journal, vol. 34, no. 11, Nov. 2019,
//   Section II-B, eq. (5)-(9).
//
// LCMV criterion (eq. 5):
//
//      min  w^H R w     s.t.  w^H C = f
//
// Power inversion (eq. 8): C = s0 = [1 0]^T, f = 1
//
//      w = R^-1 s0 / (s0^H R^-1 s0)                                (eq. 9)
//
// For a 2x2 Hermitian R this reduces exactly to:
//
//      w1 = 1
//      w2 = -conj(R12) / R22
//
// so no matrix inverse and no steering vector / angle is needed. Every
// signal far above the noise floor (the jammer) is driven into a null whose
// depth follows the jammer-to-noise ratio, as in the paper's Fig. 3-5.
//
// DIAGONAL LOADING:
//
//      R22' = R22 + (R11 + R22) >>> DL_SHIFT + 1
//
//   Keeps the division well conditioned when no jammer is present and
//   limits the null depth to about 20*log10(2^-DL_SHIFT * 2) dB
//   (DL_SHIFT = 16 -> about -90 dB, below the Q16.16 weight resolution).
//
// FIXED POINT:
//   R inputs : integer covariance from covariance_2antenna (ACC_WIDTH bits)
//   weights  : Q16.16 complex, 32 bit, saturated
//
// TIMING:
//   One 65-bit compare/subtract per clock per divider, two dividers
//   (real, imag) in parallel. start -> done = 68 clocks.
//
// ============================================================================

module lcmv_pi_weight_2ch #(
    parameter ACC_WIDTH = 64,
    parameter DL_SHIFT  = 16
)(
    input wire clk,
    input wire rst,
    input wire start,

    input wire signed [ACC_WIDTH-1:0] r11,
    input wire signed [ACC_WIDTH-1:0] r22,
    input wire signed [ACC_WIDTH-1:0] r12_re,
    input wire signed [ACC_WIDTH-1:0] r12_im,

    output reg busy,
    output reg done,

    output reg signed [31:0] w1_re,
    output reg signed [31:0] w1_im,
    output reg signed [31:0] w2_re,
    output reg signed [31:0] w2_im
);

    // ========================================================================
    // CONSTANTS
    // ========================================================================

    localparam signed [31:0] ONE_Q16 = 32'sd65536;
    localparam signed [31:0] SAT_POS = 32'sh7FFF_FFFF;
    localparam signed [31:0] SAT_NEG = -32'sh7FFF_FFFF;

    localparam S_IDLE = 2'd0;
    localparam S_PREP = 2'd1;
    localparam S_DIV  = 2'd2;
    localparam S_OUT  = 2'd3;

    reg [1:0] state;

    // ========================================================================
    // LATCHED INPUTS
    //
    // Numerator = -conj(R12) = -Re(R12) + j Im(R12)
    // ========================================================================

    reg signed [63:0] num_re;
    reg signed [63:0] num_im;
    reg        [63:0] den;

    // ========================================================================
    // SERIAL RESTORING DIVIDERS  |num| << 16 / den
    // ========================================================================

    reg [63:0] dvd_re;
    reg [63:0] dvd_im;

    reg [64:0] rem_re;
    reg [64:0] rem_im;

    reg [63:0] quo_re;
    reg [63:0] quo_im;

    reg neg_re;
    reg neg_im;

    reg [6:0] count;

    wire [64:0] trial_re = {rem_re[63:0], dvd_re[63]};
    wire [64:0] trial_im = {rem_im[63:0], dvd_im[63]};

    wire ge_re = (trial_re >= {1'b0, den});
    wire ge_im = (trial_im >= {1'b0, den});

    // ========================================================================
    // SIGN + SATURATION
    // ========================================================================

    function signed [31:0] to_q16;
        input [63:0] mag;
        input        neg;
        begin
            if (mag > 64'h0000_0000_7FFF_FFFF)
                to_q16 = neg ? SAT_NEG : SAT_POS;
            else if (neg)
                to_q16 = -$signed({1'b0, mag[30:0]});
            else
                to_q16 = $signed({1'b0, mag[30:0]});
        end
    endfunction

    // Sign-extended copies of the inputs
    wire signed [63:0] r11_x    = r11;
    wire signed [63:0] r22_x    = r22;
    wire signed [63:0] r12_re_x = r12_re;
    wire signed [63:0] r12_im_x = r12_im;

    wire signed [63:0] trace    = r11_x + r22_x;
    wire signed [63:0] loading  = (trace >>> DL_SHIFT) + 64'sd1;

    // ========================================================================
    // FSM
    // ========================================================================

    always @(posedge clk) begin

        if (rst) begin

            state  <= S_IDLE;
            busy   <= 1'b0;
            done   <= 1'b0;

            num_re <= 64'sd0;
            num_im <= 64'sd0;
            den    <= 64'd1;

            dvd_re <= 64'd0;
            dvd_im <= 64'd0;
            rem_re <= 65'd0;
            rem_im <= 65'd0;
            quo_re <= 64'd0;
            quo_im <= 64'd0;
            neg_re <= 1'b0;
            neg_im <= 1'b0;
            count  <= 7'd0;

            w1_re  <= ONE_Q16;
            w1_im  <= 32'sd0;
            w2_re  <= 32'sd0;
            w2_im  <= 32'sd0;

        end
        else begin

            done <= 1'b0;

            case (state)

                // -----------------------------------------------------------
                // IDLE: latch R on start
                // -----------------------------------------------------------

                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy   <= 1'b1;
                        num_re <= -r12_re_x;
                        num_im <=  r12_im_x;
                        den    <= (r22_x < 0) ? loading : (r22_x + loading);
                        state  <= S_PREP;
                    end
                end

                // -----------------------------------------------------------
                // PREP: magnitudes and signs, Q16.16 scaling of dividend
                // -----------------------------------------------------------

                S_PREP: begin
                    neg_re <= num_re[63];
                    neg_im <= num_im[63];

                    dvd_re <= (num_re[63] ? -num_re : num_re) << 16;
                    dvd_im <= (num_im[63] ? -num_im : num_im) << 16;

                    rem_re <= 65'd0;
                    rem_im <= 65'd0;
                    quo_re <= 64'd0;
                    quo_im <= 64'd0;

                    count  <= 7'd0;
                    state  <= S_DIV;
                end

                // -----------------------------------------------------------
                // DIV: one quotient bit per clock for each divider
                // -----------------------------------------------------------

                S_DIV: begin
                    if (ge_re) begin
                        rem_re <= trial_re - {1'b0, den};
                        quo_re <= {quo_re[62:0], 1'b1};
                    end
                    else begin
                        rem_re <= trial_re;
                        quo_re <= {quo_re[62:0], 1'b0};
                    end

                    if (ge_im) begin
                        rem_im <= trial_im - {1'b0, den};
                        quo_im <= {quo_im[62:0], 1'b1};
                    end
                    else begin
                        rem_im <= trial_im;
                        quo_im <= {quo_im[62:0], 1'b0};
                    end

                    dvd_re <= dvd_re << 1;
                    dvd_im <= dvd_im << 1;

                    count <= count + 1'b1;
                    if (count == 7'd63)
                        state <= S_OUT;
                end

                // -----------------------------------------------------------
                // OUT: w1 = 1, w2 = -conj(R12)/R22'
                // -----------------------------------------------------------

                S_OUT: begin
                    w1_re <= ONE_Q16;
                    w1_im <= 32'sd0;
                    w2_re <= to_q16(quo_re, neg_re);
                    w2_im <= to_q16(quo_im, neg_im);

                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

            endcase

        end

    end

endmodule
