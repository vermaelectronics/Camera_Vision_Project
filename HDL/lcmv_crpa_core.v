`timescale 1ns/1ps

// ============================================================================
// lcmv_crpa_core.v
//
// 2-ANTENNA CONTINUOUS LCMV CRPA CORE
//
// USER CONTROL:
//
//      enable = 0
//          -> LCMV disabled
//          -> weights_valid = 0
//          -> beam_valid = 0
//          -> FSM returned to IDLE
//
//      enable = 1
//          -> automatically start LCMV calculation
//          -> calculate covariance
//          -> calculate LCMV weights
//          -> latch weights
//          -> beamformer continuously operates
//          -> automatically calculate NEW weights
//          -> repeat continuously while enable = 1
//
// NO EXTERNAL START SIGNAL IS REQUIRED.
//
// ============================================================================

module lcmv_crpa_core #(
    parameter IQ_WIDTH    = 16,
    parameter ACC_WIDTH   = 64,
    parameter COV_WIDTH   = 32,
    parameter COV_SAMPLES = 1024,
    parameter COV_SHIFT   = 10
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

    localparam S_IDLE       = 4'd0;
    localparam S_PREP       = 4'd1;
    localparam S_STEER      = 4'd2;
    localparam S_STEER_WAIT = 4'd3;
    localparam S_COV        = 4'd4;
    localparam S_RINV_WAIT  = 4'd5;
    localparam S_B_WAIT     = 4'd6;
    localparam S_E_WAIT     = 4'd7;
    localparam S_EINV_WAIT  = 4'd8;
    localparam S_F_WAIT     = 4'd9;
    localparam S_W_WAIT     = 4'd10;
    localparam S_RESTART    = 4'd11;

    reg [3:0] state;

    // ========================================================================
    // ANGLES
    // ========================================================================

    reg signed [15:0] gps_angle_work;
    reg signed [15:0] jam_angle_work;

    // ========================================================================
    // INTERNAL START PULSES
    // ========================================================================

    reg cov_start;
    reg rinv_start;
    reg einv_start;
    reg steering_start;

    // ========================================================================
    // COVARIANCE
    // ========================================================================

    wire cov_busy;
    wire cov_done;

    wire signed [COV_WIDTH-1:0] cov_r11;
    wire signed [COV_WIDTH-1:0] cov_r12_real;
    wire signed [COV_WIDTH-1:0] cov_r12_imag;
    wire signed [COV_WIDTH-1:0] cov_r22;

    wire signed [COV_WIDTH-1:0] cov_r21_real;
    wire signed [COV_WIDTH-1:0] cov_r21_imag;

    assign cov_r21_real = cov_r12_real;
    assign cov_r21_imag = -cov_r12_imag;

    // ========================================================================
    // STEERING
    // ========================================================================

    wire signed [15:0] gps_a1_i;
    wire signed [15:0] gps_a1_q;

    wire signed [15:0] gps_a2_i;
    wire signed [15:0] gps_a2_q;

    wire signed [15:0] jam_a1_i;
    wire signed [15:0] jam_a1_q;

    wire signed [15:0] jam_a2_i;
    wire signed [15:0] jam_a2_q;

    wire steering_valid;

    // ========================================================================
    // R^-1
    // ========================================================================

    wire signed [31:0] ri11_re;
    wire signed [31:0] ri11_im;

    wire signed [31:0] ri12_re;
    wire signed [31:0] ri12_im;

    wire signed [31:0] ri21_re;
    wire signed [31:0] ri21_im;

    wire signed [31:0] ri22_re;
    wire signed [31:0] ri22_im;

    wire signed [64:0] rdet_re;
    wire signed [64:0] rdet_im;

    wire rinv_busy;
    wire rinv_done;

    // ========================================================================
    // B = R^-1 C
    // ========================================================================

    wire signed [31:0] b11_re;
    wire signed [31:0] b11_im;

    wire signed [31:0] b12_re;
    wire signed [31:0] b12_im;

    wire signed [31:0] b21_re;
    wire signed [31:0] b21_im;

    wire signed [31:0] b22_re;
    wire signed [31:0] b22_im;

    wire b_valid;

    // ========================================================================
    // C^H
    // ========================================================================

    wire signed [15:0] ch11_i;
    wire signed [15:0] ch11_q;

    wire signed [15:0] ch12_i;
    wire signed [15:0] ch12_q;

    wire signed [15:0] ch21_i;
    wire signed [15:0] ch21_q;

    wire signed [15:0] ch22_i;
    wire signed [15:0] ch22_q;

    assign ch11_i = gps_a1_i;
    assign ch11_q = -gps_a1_q;

    assign ch12_i = gps_a2_i;
    assign ch12_q = -gps_a2_q;

    assign ch21_i = jam_a1_i;
    assign ch21_q = -jam_a1_q;

    assign ch22_i = jam_a2_i;
    assign ch22_q = -jam_a2_q;

    // ========================================================================
    // E = C^H B
    // ========================================================================

    wire signed [31:0] e11_i;
    wire signed [31:0] e11_q;

    wire signed [31:0] e12_i;
    wire signed [31:0] e12_q;

    wire signed [31:0] e21_i;
    wire signed [31:0] e21_q;

    wire signed [31:0] e22_i;
    wire signed [31:0] e22_q;

    wire e_valid;

    // ========================================================================
    // E^-1
    // ========================================================================

    wire signed [31:0] ei11_re;
    wire signed [31:0] ei11_im;

    wire signed [31:0] ei12_re;
    wire signed [31:0] ei12_im;

    wire signed [31:0] ei21_re;
    wire signed [31:0] ei21_im;

    wire signed [31:0] ei22_re;
    wire signed [31:0] ei22_im;

    wire signed [64:0] edet_re;
    wire signed [64:0] edet_im;

    wire einv_busy;
    wire einv_done;

    // ========================================================================
    // F = B E^-1
    // ========================================================================

    wire signed [31:0] f11_re;
    wire signed [31:0] f11_im;

    wire signed [31:0] f12_re;
    wire signed [31:0] f12_im;

    wire signed [31:0] f21_re;
    wire signed [31:0] f21_im;

    wire signed [31:0] f22_re;
    wire signed [31:0] f22_im;

    wire f_valid;

    // ========================================================================
    // WEIGHTS
    // ========================================================================

    wire signed [31:0] weight1_re;
    wire signed [31:0] weight1_im;

    wire signed [31:0] weight2_re;
    wire signed [31:0] weight2_im;

    wire weight_valid;

    // ========================================================================
    // BEAMFORMER
    // ========================================================================

    wire signed [31:0] beam_y_re;
    wire signed [31:0] beam_y_im;

    wire beam_y_valid;

    // ========================================================================
    // COVARIANCE
    // ========================================================================

    covariance_2antenna #(
        .IQ_WIDTH    (IQ_WIDTH),
        .ACC_WIDTH   (ACC_WIDTH),
        .COV_SAMPLES (COV_SAMPLES),
        .COV_SHIFT   (COV_SHIFT)
    ) u_covariance (
        .clk          (clk),
        .rst          (rst),
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
    // STEERING
    // ========================================================================

    steering_vector_2x2 #(
        .W(16)
    ) u_steering (
        .clk        (clk),
        .rst        (rst),

        .valid_in   (steering_start),

        .gps_angle  (gps_angle_work),
        .jam_angle  (jam_angle_work),

        .gps_a1_i   (gps_a1_i),
        .gps_a1_q   (gps_a1_q),
        .gps_a2_i   (gps_a2_i),
        .gps_a2_q   (gps_a2_q),

        .jam_a1_i   (jam_a1_i),
        .jam_a1_q   (jam_a1_q),
        .jam_a2_i   (jam_a2_i),
        .jam_a2_q   (jam_a2_q),

        .valid_out  (steering_valid)
    );

    // ========================================================================
    // R^-1
    // ========================================================================

    d_inverse_2x2 #(
        .IN_WIDTH  (32),
        .OUT_WIDTH (32)
    ) u_r_inverse (
        .clk       (clk),
        .rst       (rst),
        .start     (rinv_start),

        .d11_real  (cov_r11),
        .d11_imag  (32'sd0),

        .d12_real  (cov_r12_real),
        .d12_imag  (cov_r12_imag),

        .d21_real  (cov_r21_real),
        .d21_imag  (cov_r21_imag),

        .d22_real  (cov_r22),
        .d22_imag  (32'sd0),

        .e11_real  (ri11_re),
        .e11_imag  (ri11_im),

        .e12_real  (ri12_re),
        .e12_imag  (ri12_im),

        .e21_real  (ri21_re),
        .e21_imag  (ri21_im),

        .e22_real  (ri22_re),
        .e22_imag  (ri22_im),

        .det_real   (rdet_re),
        .det_imag   (rdet_im),

        .busy       (rinv_busy),
        .done       (rinv_done)
    );

    // ========================================================================
    // B = R^-1 C
    // ========================================================================

    lcmv_r_inverse_c_2x2 #(
        .RW(32),
        .CW(16),
        .BW(32)
    ) u_rinv_c (
        .clk       (clk),
        .rst       (rst),

        .ri11_re   (ri11_re),
        .ri11_im   (ri11_im),
        .ri12_re   (ri12_re),
        .ri12_im   (ri12_im),
        .ri21_re   (ri21_re),
        .ri21_im   (ri21_im),
        .ri22_re   (ri22_re),
        .ri22_im   (ri22_im),

        .c11_re    (gps_a1_i),
        .c11_im    (gps_a1_q),
        .c12_re    (jam_a1_i),
        .c12_im    (jam_a1_q),
        .c21_re    (gps_a2_i),
        .c21_im    (gps_a2_q),
        .c22_re    (jam_a2_i),
        .c22_im    (jam_a2_q),

        .valid_in  (rinv_done),

        .b11_re    (b11_re),
        .b11_im    (b11_im),
        .b12_re    (b12_re),
        .b12_im    (b12_im),
        .b21_re    (b21_re),
        .b21_im    (b21_im),
        .b22_re    (b22_re),
        .b22_im    (b22_im),

        .valid_out (b_valid)
    );

    // ========================================================================
    // E = C^H B
    // ========================================================================

    c_h_b_matrix_2x2 #(
        .CW(16),
        .BW(32),
        .P (32)
    ) u_ch_b (
        .clk      (clk),
        .rst      (rst),

        .ch11_i   (ch11_i),
        .ch11_q   (ch11_q),
        .ch12_i   (ch12_i),
        .ch12_q   (ch12_q),
        .ch21_i   (ch21_i),
        .ch21_q   (ch21_q),
        .ch22_i   (ch22_i),
        .ch22_q   (ch22_q),

        .b11_i    (b11_re),
        .b11_q    (b11_im),
        .b12_i    (b12_re),
        .b12_q    (b12_im),
        .b21_i    (b21_re),
        .b21_q    (b21_im),
        .b22_i    (b22_re),
        .b22_q    (b22_im),

        .valid_in (b_valid),

        .e11_i    (e11_i),
        .e11_q    (e11_q),
        .e12_i    (e12_i),
        .e12_q    (e12_q),
        .e21_i    (e21_i),
        .e21_q    (e21_q),
        .e22_i    (e22_i),
        .e22_q    (e22_q),

        .valid_out(e_valid)
    );

    // ========================================================================
    // E^-1
    // ========================================================================

    d_inverse_2x2 #(
        .IN_WIDTH  (32),
        .OUT_WIDTH (32)
    ) u_e_inverse (
        .clk       (clk),
        .rst       (rst),
        .start     (einv_start),

        .d11_real  (e11_i),
        .d11_imag  (e11_q),

        .d12_real  (e12_i),
        .d12_imag  (e12_q),

        .d21_real  (e21_i),
        .d21_imag  (e21_q),

        .d22_real  (e22_i),
        .d22_imag  (e22_q),

        .e11_real  (ei11_re),
        .e11_imag  (ei11_im),

        .e12_real  (ei12_re),
        .e12_imag  (ei12_im),

        .e21_real  (ei21_re),
        .e21_imag  (ei21_im),

        .e22_real  (ei22_re),
        .e22_imag  (ei22_im),

        .det_real  (edet_re),
        .det_imag  (edet_im),

        .busy      (einv_busy),
        .done      (einv_done)
    );

    // ========================================================================
    // F = B E^-1
    // ========================================================================

    lcmv_b_e_inverse_2x2 #(
        .W(32)
    ) u_b_e_inverse (
        .clk      (clk),
        .rst      (rst),

        .b11_re   (b11_re),
        .b11_im   (b11_im),
        .b12_re   (b12_re),
        .b12_im   (b12_im),
        .b21_re   (b21_re),
        .b21_im   (b21_im),
        .b22_re   (b22_re),
        .b22_im   (b22_im),

        .ei11_re  (ei11_re),
        .ei11_im  (ei11_im),
        .ei12_re  (ei12_re),
        .ei12_im  (ei12_im),
        .ei21_re  (ei21_re),
        .ei21_im  (ei21_im),
        .ei22_re  (ei22_re),
        .ei22_im  (ei22_im),

        .valid_in (einv_done),

        .f11_re   (f11_re),
        .f11_im   (f11_im),
        .f12_re   (f12_re),
        .f12_im   (f12_im),
        .f21_re   (f21_re),
        .f21_im   (f21_im),
        .f22_re   (f22_re),
        .f22_im   (f22_im),

        .valid_out(f_valid)
    );

    // ========================================================================
    // CONSTRAINT f = [1 0]
    // ========================================================================

    localparam signed [31:0] FGPS_RE = 32'sd65536;
    localparam signed [31:0] FGPS_IM = 32'sd0;

    localparam signed [31:0] FJAM_RE = 32'sd0;
    localparam signed [31:0] FJAM_IM = 32'sd0;

    // ========================================================================
    // WEIGHT VECTOR
    // ========================================================================

    lcmv_weight_vector_2x2 #(
        .W(32)
    ) u_weight (
        .clk        (clk),
        .rst        (rst),

        .F11_re     (f11_re),
        .F11_im     (f11_im),
        .F12_re     (f12_re),
        .F12_im     (f12_im),
        .F21_re     (f21_re),
        .F21_im     (f21_im),
        .F22_re     (f22_re),
        .F22_im     (f22_im),

        .fGPS_re    (FGPS_RE),
        .fGPS_im    (FGPS_IM),

        .fJAMMER_re (FJAM_RE),
        .fJAMMER_im (FJAM_IM),

        .w1_re      (weight1_re),
        .w1_im      (weight1_im),
        .w2_re      (weight2_re),
        .w2_im      (weight2_im),

        .valid      (weight_valid)
    );

    // ========================================================================
    // BEAMFORMER
    //
    // Always receives RX samples.
    //
    // During calculation, old weights remain active.
    // New weights are atomically loaded after weight_valid.
    // ========================================================================

    lcmv_beamformer_2ch #(
        .W(32)
    ) u_beamformer (
        .clk      (clk),
        .rst      (rst),

        .w1_re    (w1_re),
        .w1_im    (w1_im),
        .w2_re    (w2_re),
        .w2_im    (w2_im),

        .X1_re    ({{16{rx1_i[15]}},rx1_i}),
        .X1_im    ({{16{rx1_q[15]}},rx1_q}),

        .X2_re    ({{16{rx2_i[15]}},rx2_i}),
        .X2_im    ({{16{rx2_q[15]}},rx2_q}),

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

            cov_start      <= 1'b0;
            rinv_start     <= 1'b0;
            einv_start     <= 1'b0;
            steering_start <= 1'b0;

            busy          <= 1'b0;
            weights_valid <= 1'b0;

            w1_re <= 32'sd0;
            w1_im <= 32'sd0;
            w2_re <= 32'sd0;
            w2_im <= 32'sd0;

            gps_angle_work <= 16'sd30;
            jam_angle_work <= 16'sd60;

            beam_valid <= 1'b0;
            y_re       <= 32'sd0;
            y_im       <= 32'sd0;

        end
        else begin

            // ---------------------------------------------------------------
            // All internal starts are one-cycle pulses.
            // ---------------------------------------------------------------

            cov_start      <= 1'b0;
            rinv_start     <= 1'b0;
            einv_start     <= 1'b0;
            steering_start <= 1'b0;

            beam_valid <= 1'b0;

            // ===============================================================
            // LCMV DISABLED
            //
            // This has absolute priority.
            //
            // 1 -> 0:
            //      immediately leave LCMV
            //      invalidate weights
            //      return FSM to IDLE
            //
            // The old weight registers themselves do not matter because
            // weights_valid=0 prevents them from being selected.
            // ===============================================================

            if (!enable) begin

                state         <= S_IDLE;
                busy          <= 1'b0;
                weights_valid <= 1'b0;

                y_re <= 32'sd0;
                y_im <= 32'sd0;

            end
            else begin

                // ===========================================================
                // BEAMFORMER OUTPUT
                //
                // Once weights are valid, this remains active continuously.
                // ===========================================================

                if (weights_valid && beam_y_valid) begin

                    y_re       <= beam_y_re;
                    y_im       <= beam_y_im;
                    beam_valid <= 1'b1;

                end

                // ===========================================================
                // FSM
                // ===========================================================

                case (state)

                    // -------------------------------------------------------
                    // IDLE
                    //
                    // enable=1 automatically starts.
                    // -------------------------------------------------------

                    S_IDLE: begin

                        busy  <= 1'b1;
                        state <= S_PREP;

                    end

                    // -------------------------------------------------------
                    // PREP
                    //
                    // Capture angles for one complete calculation.
                    // -------------------------------------------------------

                    S_PREP: begin

                        busy <= 1'b1;

                        gps_angle_work <= gps_angle;
                        jam_angle_work <= jam_angle;

                        state <= S_STEER;

                    end

                    // -------------------------------------------------------
                    // START STEERING
                    // -------------------------------------------------------

                    S_STEER: begin

                        busy <= 1'b1;

                        steering_start <= 1'b1;

                        state <= S_STEER_WAIT;

                    end

                    // -------------------------------------------------------
                    // WAIT STEERING
                    // -------------------------------------------------------

                    S_STEER_WAIT: begin

                        busy <= 1'b1;

                        if (steering_valid) begin

                            cov_start <= 1'b1;
                            state <= S_COV;

                        end

                    end

                    // -------------------------------------------------------
                    // COVARIANCE
                    // -------------------------------------------------------

                    S_COV: begin

                        busy <= 1'b1;

                        if (cov_done) begin

                            rinv_start <= 1'b1;
                            state <= S_RINV_WAIT;

                        end

                    end

                    // -------------------------------------------------------
                    // R^-1
                    // -------------------------------------------------------

                    S_RINV_WAIT: begin

                        busy <= 1'b1;

                        if (rinv_done)
                            state <= S_B_WAIT;

                    end

                    // -------------------------------------------------------
                    // B = R^-1 C
                    // -------------------------------------------------------

                    S_B_WAIT: begin

                        busy <= 1'b1;

                        if (b_valid)
                            state <= S_E_WAIT;

                    end

                    // -------------------------------------------------------
                    // E = C^H B
                    // -------------------------------------------------------

                    S_E_WAIT: begin

                        busy <= 1'b1;

                        if (e_valid) begin

                            einv_start <= 1'b1;
                            state <= S_EINV_WAIT;

                        end

                    end

                    // -------------------------------------------------------
                    // E^-1
                    // -------------------------------------------------------

                    S_EINV_WAIT: begin

                        busy <= 1'b1;

                        if (einv_done)
                            state <= S_F_WAIT;

                    end

                    // -------------------------------------------------------
                    // F = B E^-1
                    // -------------------------------------------------------

                    S_F_WAIT: begin

                        busy <= 1'b1;

                        if (f_valid)
                            state <= S_W_WAIT;

                    end

                    // -------------------------------------------------------
                    // W = F f
                    // -------------------------------------------------------

                    S_W_WAIT: begin

                        busy <= 1'b1;

                        if (weight_valid) begin

                            // ------------------------------------------------
                            // ATOMIC WEIGHT UPDATE
                            // ------------------------------------------------

                            w1_re <= weight1_re;
                            w1_im <= weight1_im;

                            w2_re <= weight2_re;
                            w2_im <= weight2_im;

                            // ------------------------------------------------
                            // From this point forward the beamformer is
                            // considered active.
                            // ------------------------------------------------

                            weights_valid <= 1'b1;

                            busy <= 1'b0;

                            // Automatically begin another calculation.
                            state <= S_RESTART;

                        end

                    end

                    // -------------------------------------------------------
                    // RESTART
                    //
                    // No external start required.
                    // -------------------------------------------------------

                    S_RESTART: begin

                        busy <= 1'b1;
                        state <= S_PREP;

                    end

                    // -------------------------------------------------------
                    // DEFAULT
                    // -------------------------------------------------------

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