`timescale 1ns/1ps

// ============================================================================
// lcmv_crpa_core.v
//
// 2-ANTENNA ROBUST LCMV CRPA CORE  (jammer angle supplied by the PS)
//
// Algorithm reference:
//   R. Li, X. Zhao, X. W. Shi, "Derivative Constrained Robust LCMV
//   Beamforming Algorithm", PIER C, vol. 4, pp. 43-52, 2008, eq. (9).
//
//   min w^H R w   s.t.  S^H w = 1 (GPS look),  w^H a_J = eps (jammer)
//
//   See lcmv_dc_weight_2ch.v for the derivation. With two antennas the
//   derivative (null-widening) row would force w2 = 0, so only the look and
//   enforcement rows are used, and the LCMV optimum is w = C^-H f.
//
// DATA FLOW:
//
//   gps_angle, jam_angle (PS, integer deg)
//        -> steering_vector_2x2      a_s, a_J
//        -> lcmv_dc_weight_2ch       w = C^-H [1 eps]^T
//        -> lcmv_beamformer_2ch      y = w^H x
//
//   The jammer direction must come from outside this core (PS software,
//   external DOA, or a known jammer position).
//
// USER CONTROL:
//
//      enable = 0  -> weights_valid = 0, beam_valid = 0, FSM in IDLE
//      enable = 1  -> weights recomputed continuously from the latest
//                     gps_angle / jam_angle (about 80 clocks per update);
//                     beamformer runs with the newest weights
//
// PORT COMPATIBILITY:
//   Port list unchanged. COV_* parameters are kept only so that
//   gnss_passthrough.v needs no edits; no covariance is needed because all
//   degrees of freedom are used by the constraints.
// ============================================================================

module lcmv_crpa_core #(
    parameter IQ_WIDTH    = 16,
    parameter ACC_WIDTH   = 64,                  // unused (compatibility)
    parameter COV_WIDTH   = 32,                  // unused (compatibility)
    parameter COV_SAMPLES = 1024,                // unused (compatibility)
    parameter COV_SHIFT   = 10,                  // unused (compatibility)
    parameter signed [31:0] EPS_Q16     = 32'sd0,   // paper eps, Q16.16
    parameter        [31:0] DEN_MIN_Q16 = 32'd655   // jammer-in-mainbeam guard
)(
    input wire clk,
    input wire rst,

    input wire enable,

    input wire rx_valid,

    input wire signed [IQ_WIDTH-1:0] rx1_i,
    input wire signed [IQ_WIDTH-1:0] rx1_q,

    input wire signed [IQ_WIDTH-1:0] rx2_i,
    input wire signed [IQ_WIDTH-1:0] rx2_q,

    input wire signed [15:0] gps_angle,
    input wire signed [15:0] jam_angle,

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

    localparam S_IDLE   = 2'd0;
    localparam S_STEER  = 2'd1;
    localparam S_WGT    = 2'd2;

    reg [1:0] state;

    reg steering_start;
    reg wgt_start;

    reg signed [15:0] gps_angle_work;
    reg signed [15:0] jam_angle_work;

    // ========================================================================
    // STEERING VECTORS  a = [1, exp(j*pi*sin(theta))], Q1.15
    // ========================================================================

    wire signed [15:0] gps_a1_i, gps_a1_q, gps_a2_i, gps_a2_q;
    wire signed [15:0] jam_a1_i, jam_a1_q, jam_a2_i, jam_a2_q;
    wire steering_valid;

    steering_vector_2x2 #(
        .W(16)
    ) u_steering (
        .clk       (clk),
        .rst       (rst),
        .valid_in  (steering_start),

        .gps_angle (gps_angle_work),
        .jam_angle (jam_angle_work),

        .gps_a1_i  (gps_a1_i),
        .gps_a1_q  (gps_a1_q),
        .gps_a2_i  (gps_a2_i),
        .gps_a2_q  (gps_a2_q),

        .jam_a1_i  (jam_a1_i),
        .jam_a1_q  (jam_a1_q),
        .jam_a2_i  (jam_a2_i),
        .jam_a2_q  (jam_a2_q),

        .valid_out (steering_valid)
    );

    // ========================================================================
    // LCMV WEIGHTS  (PIER 2008 eq. 9, look + enforcement constraints)
    // ========================================================================

    wire wgt_busy;
    wire wgt_done;

    wire signed [31:0] weight1_re, weight1_im;
    wire signed [31:0] weight2_re, weight2_im;

    lcmv_dc_weight_2ch #(
        .EPS_Q16     (EPS_Q16),
        .DEN_MIN_Q16 (DEN_MIN_Q16)
    ) u_lcmv_weight (
        .clk   (clk),
        .rst   (rst | ~enable),
        .start (wgt_start),

        .s_re  (gps_a2_i),
        .s_im  (gps_a2_q),
        .j_re  (jam_a2_i),
        .j_im  (jam_a2_q),

        .busy  (wgt_busy),
        .done  (wgt_done),

        .w1_re (weight1_re),
        .w1_im (weight1_im),
        .w2_re (weight2_re),
        .w2_im (weight2_im)
    );

    // ========================================================================
    // BEAMFORMER   y = w^H x
    //
    // Old weights stay active while the next set is calculated.
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

            steering_start <= 1'b0;
            wgt_start      <= 1'b0;

            busy          <= 1'b0;
            weights_valid <= 1'b0;

            gps_angle_work <= 16'sd30;
            jam_angle_work <= 16'sd60;

            w1_re <= 32'sd0;
            w1_im <= 32'sd0;
            w2_re <= 32'sd0;
            w2_im <= 32'sd0;

            beam_valid <= 1'b0;
            y_re       <= 32'sd0;
            y_im       <= 32'sd0;

        end
        else begin

            steering_start <= 1'b0;
            wgt_start      <= 1'b0;
            beam_valid     <= 1'b0;

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
                    // IDLE: capture angles for one calculation
                    // -------------------------------------------------------

                    S_IDLE: begin
                        busy           <= 1'b1;
                        gps_angle_work <= gps_angle;
                        jam_angle_work <= jam_angle;
                        steering_start <= 1'b1;
                        state          <= S_STEER;
                    end

                    // -------------------------------------------------------
                    // STEER: wait for a_s, a_J
                    // -------------------------------------------------------

                    S_STEER: begin
                        busy <= 1'b1;
                        if (steering_valid) begin
                            wgt_start <= 1'b1;
                            state     <= S_WGT;
                        end
                    end

                    // -------------------------------------------------------
                    // WGT: atomic weight update, then recalculate
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

                            state <= S_IDLE;
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
