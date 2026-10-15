`timescale 1ns/1ps

// ============================================================================
// lcmv_crpa_core.v
//
// 2-ANTENNA CONTINUOUS LCMV-PI (POWER-INVERSION) CRPA CORE
//
// Algorithm reference:
//   F. Wang, H. Liu, C. Zhang, "Anti-Jamming Front-End Design of Satellite
//   Navigation Receiver", ACES Journal, vol. 34, no. 11, Nov. 2019.
//
//   LCMV criterion with C = s0 = [1 0]^T, f = 1 (paper eq. 8-9):
//
//       w = R^-1 s0 / (s0^H R^-1 s0)  ->  w1 = 1, w2 = -conj(R12)/R22
//
//   The null is placed on the jammer's actual direction, measured from the
//   data, instead of on an angle written by the PS. A wrong jammer angle
//   no longer makes the null shallow, and the null follows a moving jammer
//   every covariance block.
//
// PROCESSING (Fig. 20 of the paper):
//
//   covariance estimation -> weight calculation -> weighted summation
//
//   The next covariance block starts as soon as the previous one is done,
//   so the weights are refreshed every COV_SAMPLES valid samples
//   (1024 samples + 68 clocks).
//
// USER CONTROL:
//
//      enable = 0
//          -> LCMV disabled, covariance and solver held in reset
//          -> weights_valid = 0, beam_valid = 0
//
//      enable = 1
//          -> continuous covariance / weight update
//          -> beamformer runs with the newest weights
//
// PORT COMPATIBILITY:
//   The port list is unchanged so gnss_passthrough.v needs no edits.
//   gps_angle / jam_angle are NOT used by power inversion and are ignored.
//   COV_WIDTH is kept only for parameter compatibility.
//
// LIMITATION:
//   2 antennas = 1 degree of freedom = 1 null. Its angular width is fixed
//   by the array geometry. Wider nulls (CMT / virtual jammers) need
//   3 or more antennas.
//
// ============================================================================

module lcmv_crpa_core #(
    parameter IQ_WIDTH    = 16,
    parameter ACC_WIDTH   = 64,
    parameter COV_WIDTH   = 32,
    parameter COV_SAMPLES = 1024,
    parameter COV_SHIFT   = 10,
    parameter DL_SHIFT    = 16
)(
    input wire clk,
    input wire rst,

    input wire enable,

    input wire rx_valid,

    input wire signed [IQ_WIDTH-1:0] rx1_i,
    input wire signed [IQ_WIDTH-1:0] rx1_q,

    input wire signed [IQ_WIDTH-1:0] rx2_i,
    input wire signed [IQ_WIDTH-1:0] rx2_q,

    input wire signed [15:0] gps_angle,     // unused (power inversion)
    input wire signed [15:0] jam_angle,     // unused (power inversion)

    output reg busy,
    output reg weights_valid,

    output reg signed [31:0] w1_re,
    output reg signed [31:0] w1_im,

    output reg signed [31:0] w2_re,
    output reg signed [31:0] w2_im,

    output reg beam_valid,

    output reg signed [31:0] y_re,
    output reg signed [31:0] y_im
);

    // ========================================================================
    // FSM
    // ========================================================================

    localparam S_IDLE = 2'd0;
    localparam S_COV  = 2'd1;
    localparam S_WGT  = 2'd2;

    reg [1:0] state;

    reg cov_start;
    reg wgt_start;

    // Covariance and solver are held in reset while LCMV is disabled so that
    // every enable starts from a fresh covariance block.
    wire blk_rst = rst | ~enable;

    // ========================================================================
    // COVARIANCE  (full ACC_WIDTH: no truncation of strong-jammer power)
    // ========================================================================

    wire cov_busy;
    wire cov_done;

    wire signed [ACC_WIDTH-1:0] cov_r11;
    wire signed [ACC_WIDTH-1:0] cov_r22;
    wire signed [ACC_WIDTH-1:0] cov_r12_real;
    wire signed [ACC_WIDTH-1:0] cov_r12_imag;

    covariance_2antenna #(
        .IQ_WIDTH    (IQ_WIDTH),
        .ACC_WIDTH   (ACC_WIDTH),
        .COV_SAMPLES (COV_SAMPLES),
        .COV_SHIFT   (COV_SHIFT)
    ) u_covariance (
        .clk          (clk),
        .rst          (blk_rst),
        .start        (cov_start),

        .rx_valid     (rx_valid),

        .rx1_i        (rx1_i),
        .rx1_q        (rx1_q),
        .rx2_i        (rx2_i),
        .rx2_q        (rx2_q),

        .busy         (cov_busy),
        .done         (cov_done),

        .cov_r11      (cov_r11),
        .cov_r12_real (cov_r12_real),
        .cov_r12_imag (cov_r12_imag),
        .cov_r22      (cov_r22)
    );

    // ========================================================================
    // LCMV-PI WEIGHTS
    // ========================================================================

    wire wgt_busy;
    wire wgt_done;

    wire signed [31:0] weight1_re;
    wire signed [31:0] weight1_im;
    wire signed [31:0] weight2_re;
    wire signed [31:0] weight2_im;

    lcmv_pi_weight_2ch #(
        .ACC_WIDTH (ACC_WIDTH),
        .DL_SHIFT  (DL_SHIFT)
    ) u_pi_weight (
        .clk    (clk),
        .rst    (blk_rst),
        .start  (wgt_start),

        .r11    (cov_r11),
        .r22    (cov_r22),
        .r12_re (cov_r12_real),
        .r12_im (cov_r12_imag),

        .busy   (wgt_busy),
        .done   (wgt_done),

        .w1_re  (weight1_re),
        .w1_im  (weight1_im),
        .w2_re  (weight2_re),
        .w2_im  (weight2_im)
    );

    // ========================================================================
    // BEAMFORMER   y = w^H x
    //
    // Old weights stay active while the next block is being calculated.
    // ========================================================================

    wire signed [31:0] beam_y_re;
    wire signed [31:0] beam_y_im;
    wire beam_y_valid;

    lcmv_beamformer_2ch #(
        .W(32)
    ) u_beamformer (
        .clk      (clk),
        .rst      (rst),

        .w1_re    (w1_re),
        .w1_im    (w1_im),
        .w2_re    (w2_re),
        .w2_im    (w2_im),

        .X1_re    ({{(32-IQ_WIDTH){rx1_i[IQ_WIDTH-1]}}, rx1_i}),
        .X1_im    ({{(32-IQ_WIDTH){rx1_q[IQ_WIDTH-1]}}, rx1_q}),

        .X2_re    ({{(32-IQ_WIDTH){rx2_i[IQ_WIDTH-1]}}, rx2_i}),
        .X2_im    ({{(32-IQ_WIDTH){rx2_q[IQ_WIDTH-1]}}, rx2_q}),

        .rx_valid (rx_valid),

        .Y_re     (beam_y_re),
        .Y_im     (beam_y_im),

        .valid    (beam_y_valid)
    );

    // ========================================================================
    // MAIN FSM
    // ========================================================================

    always @(posedge clk) begin

        if (rst) begin

            state <= S_IDLE;

            cov_start <= 1'b0;
            wgt_start <= 1'b0;

            busy          <= 1'b0;
            weights_valid <= 1'b0;

            w1_re <= 32'sd0;
            w1_im <= 32'sd0;
            w2_re <= 32'sd0;
            w2_im <= 32'sd0;

            beam_valid <= 1'b0;
            y_re       <= 32'sd0;
            y_im       <= 32'sd0;

        end
        else begin

            cov_start  <= 1'b0;
            wgt_start  <= 1'b0;
            beam_valid <= 1'b0;

            if (!enable) begin

                state         <= S_IDLE;
                busy          <= 1'b0;
                weights_valid <= 1'b0;

                y_re <= 32'sd0;
                y_im <= 32'sd0;

            end
            else begin

                // -----------------------------------------------------------
                // BEAMFORMER OUTPUT
                // -----------------------------------------------------------

                if (weights_valid && beam_y_valid) begin
                    y_re       <= beam_y_re;
                    y_im       <= beam_y_im;
                    beam_valid <= 1'b1;
                end

                case (state)

                    // -------------------------------------------------------
                    // IDLE: start the first covariance block
                    // -------------------------------------------------------

                    S_IDLE: begin
                        busy      <= 1'b1;
                        cov_start <= 1'b1;
                        state     <= S_COV;
                    end

                    // -------------------------------------------------------
                    // COV: wait for R, then solve weights and immediately
                    // start capturing the next block
                    // -------------------------------------------------------

                    S_COV: begin
                        busy <= 1'b1;
                        if (cov_done) begin
                            wgt_start <= 1'b1;
                            cov_start <= 1'b1;
                            state     <= S_WGT;
                        end
                    end

                    // -------------------------------------------------------
                    // WGT: atomic weight update
                    // -------------------------------------------------------

                    S_WGT: begin
                        busy <= 1'b1;
                        if (wgt_done) begin
                            w1_re <= weight1_re;
                            w1_im <= weight1_im;
                            w2_re <= weight2_re;
                            w2_im <= weight2_im;

                            weights_valid <= 1'b1;
                            busy          <= 1'b0;

                            state <= S_COV;
                        end
                    end

                    default: begin
                        state         <= S_IDLE;
                        busy          <= 1'b0;
                        weights_valid <= 1'b0;
                    end

                endcase

            end

        end

    end

endmodule
