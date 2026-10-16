// ============================================================================
//  crpa_lcmv_2ch.v
//  2-element robust LCMV CRPA core (sample-clock domain).
//
//  Algorithm: R. Li, X. Zhao, X. W. Shi, PIER C vol. 4, 2008, eq. (9),
//  look + enforcement constraints (see crpa_lcmv_weight_2ch.v).
//
//      gps_angle, jam_angle  ->  steering_vector_2x2   a_s, a_J
//                            ->  crpa_lcmv_weight_2ch  w = C^-H [1 eps]^T
//                            ->  crpa_beamformer_2ch   y = w^H x
//
//  The jammer direction is an INPUT (from the PS through gnss_passthrough
//  registers). This core does no DOA estimation.
//
//  STEERING CONVENTION
//    a(theta) = [1, exp(j*pi*sin(theta))], antenna spacing lambda/2.
//    x1 = RX1 (adc ch0) is the reference element; for theta > 0 the wave
//    reaches RX2 first (RX2 leads RX1 by pi*sin(theta)).
//
//  WEIGHT SOURCE
//    sw_mode = 0 : angle solver in the PL (look + enforcement, w = C^-H f).
//    sw_mode = 1 : weights computed by the PS from the measured covariance
//                  (full paper formula w = R^-1 C (C^H R^-1 C)^-1 f, see
//                  crpa_cov_2ch.v) and loaded through sw_w*; a new set is
//                  taken atomically whenever sw_seq toggles.
//
//  OPERATION
//    enable = 0 : weights_valid = 0, solver idle.
//    enable = 1 : weights recomputed continuously from the latest angles
//                 (~70 clocks per update); the beamformer always runs with
//                 the last complete weight set (atomic update).
//
//  Original work for the ANTSDR E310 V1 GNSS CRPA project.
// ============================================================================

`timescale 1ns/100ps

module crpa_lcmv_2ch (
  input                    clk,
  input                    rst,
  input                    enable,

  input  signed [15:0]     gps_angle,    // integer degrees, -90..90
  input  signed [15:0]     jam_angle,    // integer degrees, -90..90
  input         [15:0]     eps,          // Q0.16
  input  signed [15:0]     cal_re,       // RX2 calibration, Q2.14 (1.0 = 16384)
  input  signed [15:0]     cal_im,

  input                    sw_mode,      // 1 = weights from the PS
  input                    sw_seq,       // toggles on every new PS weight set
  input  signed [24:0]     sw_w1_re,     // Q8.16, final beamformer weights
  input  signed [24:0]     sw_w1_im,     //        (RX2 calibration included)
  input  signed [24:0]     sw_w2_re,
  input  signed [24:0]     sw_w2_im,

  input                    in_valid,
  input  signed [11:0]     x1_re,
  input  signed [11:0]     x1_im,
  input  signed [11:0]     x2_re,
  input  signed [11:0]     x2_im,

  output                   out_valid,
  output signed [11:0]     y_re,
  output signed [11:0]     y_im,
  output                   out_sat,

  output reg               weights_valid,
  output reg               busy,
  output reg [31:0]        update_count,
  output reg signed [24:0] w1_re,        // Q8.16, active weights
  output reg signed [24:0] w1_im,
  output reg signed [24:0] w2_re,
  output reg signed [24:0] w2_im
);

  localparam LATENCY = 5;               // crpa_beamformer_2ch

  localparam S_IDLE  = 2'd0;
  localparam S_STEER = 2'd1;
  localparam S_WGT   = 2'd2;

  reg [1:0] state;
  reg       sw_seq_d;
  reg       sw_have;                    // a PS weight set has been committed
  reg       steer_start;
  reg       wgt_start;

  reg signed [15:0] gps_work, jam_work;
  reg        [15:0] eps_work;
  reg signed [15:0] cal_re_work, cal_im_work;

  // ---- steering vectors, Q1.15 -----------------------------------------------
  wire signed [15:0] gps_a1_i, gps_a1_q, gps_a2_i, gps_a2_q;
  wire signed [15:0] jam_a1_i, jam_a1_q, jam_a2_i, jam_a2_q;
  wire               steer_valid;

  steering_vector_2x2 #(.W(16)) u_steering (
    .clk       (clk),
    .rst       (rst),
    .valid_in  (steer_start),
    .gps_angle (gps_work),
    .jam_angle (jam_work),
    .gps_a1_i  (gps_a1_i),  .gps_a1_q (gps_a1_q),
    .gps_a2_i  (gps_a2_i),  .gps_a2_q (gps_a2_q),
    .jam_a1_i  (jam_a1_i),  .jam_a1_q (jam_a1_q),
    .jam_a2_i  (jam_a2_i),  .jam_a2_q (jam_a2_q),
    .valid_out (steer_valid)
  );

  // ---- LCMV weights -----------------------------------------------------------
  wire               wgt_busy, wgt_done;
  wire signed [24:0] nw1_re, nw1_im, nw2_re, nw2_im;

  crpa_lcmv_weight_2ch u_weight (
    .clk   (clk),
    .rst   (rst | ~enable | sw_mode),
    .start (wgt_start),
    .s_re  (gps_a2_i),
    .s_im  (gps_a2_q),
    .j_re  (jam_a2_i),
    .j_im  (jam_a2_q),
    .eps   (eps_work),
    .cal_re(cal_re_work),
    .cal_im(cal_im_work),
    .busy  (wgt_busy),
    .done  (wgt_done),
    .w1_re (nw1_re),
    .w1_im (nw1_im),
    .w2_re (nw2_re),
    .w2_im (nw2_im)
  );

  // ---- beamformer -------------------------------------------------------------
  crpa_beamformer_2ch u_beam (
    .clk       (clk),
    .rst       (rst),
    .in_valid  (in_valid),
    .x1_re     (x1_re),  .x1_im (x1_im),
    .x2_re     (x2_re),  .x2_im (x2_im),
    .w1_re     (w1_re),  .w1_im (w1_im),
    .w2_re     (w2_re),  .w2_im (w2_im),
    .out_valid (out_valid),
    .y_re      (y_re),
    .y_im      (y_im),
    .out_sat   (out_sat)
  );

  // ---- control ----------------------------------------------------------------
  always @(posedge clk) begin
    if (rst) begin
      state         <= S_IDLE;
      steer_start   <= 1'b0;
      wgt_start     <= 1'b0;
      busy          <= 1'b0;
      weights_valid <= 1'b0;
      update_count  <= 32'd0;
      gps_work      <= 16'sd0;
      jam_work      <= 16'sd0;
      eps_work      <= 16'd0;
      cal_re_work   <= 16'sd16384;
      cal_im_work   <= 16'sd0;
      w1_re <= 25'sd65536; w1_im <= 25'sd0;
      w2_re <= 25'sd0;     w2_im <= 25'sd0;
      sw_seq_d      <= 1'b0;
      sw_have       <= 1'b0;
    end else begin
      steer_start <= 1'b0;
      wgt_start   <= 1'b0;
      sw_seq_d    <= sw_seq;
      if (sw_seq != sw_seq_d) sw_have <= 1'b1;

      if (!enable) begin
        state         <= S_IDLE;
        busy          <= 1'b0;
        weights_valid <= 1'b0;
      end else if (sw_mode) begin
        // PS-computed (covariance) weights: atomic load on every new set,
        // and of the last committed set right after enable (the PS commits
        // before it sets the enable).
        state <= S_IDLE;
        busy  <= 1'b0;
        if (sw_seq != sw_seq_d || (sw_have && !weights_valid)) begin
          w1_re <= sw_w1_re;  w1_im <= sw_w1_im;
          w2_re <= sw_w2_re;  w2_im <= sw_w2_im;
          weights_valid <= 1'b1;
          update_count  <= update_count + 1'b1;
        end
      end else begin
        case (state)
          // capture one consistent set of inputs per calculation
          S_IDLE: begin
            busy        <= 1'b1;
            gps_work    <= gps_angle;
            jam_work    <= jam_angle;
            eps_work    <= eps;
            cal_re_work <= cal_re;
            cal_im_work <= cal_im;
            steer_start <= 1'b1;
            state       <= S_STEER;
          end

          S_STEER: begin
            if (steer_valid) begin
              wgt_start <= 1'b1;
              state     <= S_WGT;
            end
          end

          // atomic weight update, then recalculate
          S_WGT: begin
            if (wgt_done) begin
              w1_re <= nw1_re;  w1_im <= nw1_im;
              w2_re <= nw2_re;  w2_im <= nw2_im;
              weights_valid <= 1'b1;
              update_count  <= update_count + 1'b1;
              busy          <= 1'b0;
              state         <= S_IDLE;
            end
          end

          default: state <= S_IDLE;
        endcase
      end
    end
  end

endmodule
