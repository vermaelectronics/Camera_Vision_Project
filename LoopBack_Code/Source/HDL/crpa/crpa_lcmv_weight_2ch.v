// ============================================================================
//  crpa_lcmv_weight_2ch.v
//  Robust LCMV weight solver for a 2-element array.
//
//  Reference:
//    R. Li, X. Zhao, X. W. Shi, "Derivative Constrained Robust LCMV
//    Beamforming Algorithm", Progress In Electromagnetics Research C,
//    vol. 4, pp. 43-52, 2008.  Eq. (6)-(9).
//
//  Paper problem, eq. (9):
//      min  w^H R w
//      s.t. S^H w       = 1      look direction (GPS)
//           w^H B^p A_J = 0      derivative (null widening)
//           w^H A_J     = eps    enforcement (null deepening)
//      w_opt = R^-1 C (C^H R^-1 C)^-1 f
//
//  TWO ANTENNAS (M = 2):
//    * The derivative row w^H B a_J = conj(w2) e^{j psi} = 0 forces w2 = 0
//      (switches antenna 2 off), so it cannot be used and is omitted.
//    * C = [a_s a_J], f = [1 eps]^T is square, so the paper's solution
//      reduces exactly to w = C^-H f:
//
//          w2 = (1 - eps) (s - j) / |s - j|^2      s = a_s(2), j = a_J(2)
//          w1 = 1 - conj(s) w2
//
//      R cancels because the two constraints use both degrees of freedom.
//
//  RX2 CHANNEL CALIBRATION:
//    The constraints hold for the corrected array x' = [x1, c*x2], where c
//    (Q2.14 complex, |c| < 2, 1.0 = no correction) removes the RX2/RX1 gain and phase
//    mismatch. Applied to the raw samples this is
//          y = conj(w1) x1 + conj(w2) (c x2) = conj(w1) x1 + conj(w2 conj(c)) x2
//    so the solver outputs w2_out = w2 * conj(c).
//
//  FIXED POINT (sized so every multiply maps to one DSP48E1 at 8 ns):
//    s, j       Q1.15  16 bit           (steering LUT)
//    eps        Q0.16  16 bit unsigned  (0 = ideal null)
//    d = s - j  Q2.15  17 bit
//    |d|^2      Q.30   35 bit           clamped to DEN_MIN_Q30
//    (1-eps) d  Q.30   35 bit
//    w1, w2     Q8.16  25 bit signed, saturated   (|w| <= 1/|d|min + 1)
//
//  Latency: start -> done = 60 clocks.
//
//  Original work for the ANTSDR E310 V1 GNSS CRPA project.
// ============================================================================

`timescale 1ns/100ps

module crpa_lcmv_weight_2ch #(
  parameter [34:0] DEN_MIN_Q30 = 35'd10737418     // 0.01  (~2 deg at broadside)
) (
  input                    clk,
  input                    rst,
  input                    start,

  input  signed [15:0]     s_re,      // a_s(2), Q1.15
  input  signed [15:0]     s_im,
  input  signed [15:0]     j_re,      // a_J(2), Q1.15
  input  signed [15:0]     j_im,
  input         [15:0]     eps,       // Q0.16
  input  signed [15:0]     cal_re,    // RX2 calibration c, Q2.14
  input  signed [15:0]     cal_im,

  output reg               busy,
  output reg               done,
  output reg signed [24:0] w1_re,     // Q8.16
  output reg signed [24:0] w1_im,
  output reg signed [24:0] w2_re,
  output reg signed [24:0] w2_im
);

  localparam signed [24:0] ONE_Q16 = 25'sd65536;
  localparam signed [24:0] SAT_POS = 25'sh0FF_FFFF;
  localparam signed [24:0] SAT_NEG = -25'sh0FF_FFFF;

  localparam S_IDLE = 3'd0;
  localparam S_D    = 3'd1;
  localparam S_MUL  = 3'd2;
  localparam S_DEN  = 3'd3;
  localparam S_DIVW = 3'd4;
  localparam S_W1A  = 3'd5;
  localparam S_W1B  = 3'd6;
  localparam S_C1   = 3'd7;

  reg [2:0] state;
  reg       c_phase;                     // 0: products, 1: sums (in S_C1)

  reg signed [15:0] sr, si, jr, ji, cr, ci;
  reg signed [17:0] ome;                 // 1 - eps, Q1.16 (0 .. 65536)
  reg signed [16:0] d_re, d_im;          // Q2.15

  reg signed [33:0] p_rr, p_ii;          // d^2 terms, Q.30
  reg signed [34:0] n_re, n_im;          // (1-eps) d, Q.31

  reg        [47:0] den;                 // |d|^2, Q.30
  reg signed [47:0] num_re, num_im;      // (1-eps) d, Q.30

  reg signed [40:0] q_rr, q_ii, q_ri, q_ir;   // s * w2, Q.31

  // ---- dividers: w2 = num / den -------------------------------------------
  reg                div_start;
  wire signed [24:0] qd_re, qd_im;
  wire               div_busy_re, div_busy_im, div_done_re, div_done_im;
  reg                got_re, got_im;

  crpa_sdiv_serial #(.W(48), .QW(25)) u_div_re (
    .clk(clk), .rst(rst), .start(div_start),
    .num(num_re), .den(den),
    .q(qd_re), .busy(div_busy_re), .done(div_done_re)
  );

  crpa_sdiv_serial #(.W(48), .QW(25)) u_div_im (
    .clk(clk), .rst(rst), .start(div_start),
    .num(num_im), .den(den),
    .q(qd_im), .busy(div_busy_im), .done(div_done_im)
  );

  wire signed [34:0] den_sum = p_rr + p_ii;

  // w1 = 1 - conj(s) w2 ;  conj(s) w2 = (sr w2r + si w2i) + j (sr w2i - si w2r)
  wire signed [41:0] cw_re = q_rr + q_ii;
  wire signed [41:0] cw_im = q_ri - q_ir;
  wire signed [42:0] w1r_full = $signed({ONE_Q16[24], ONE_Q16}) - (cw_re >>> 15);
  wire signed [42:0] w1i_full = -(cw_im >>> 15);

  // w2 * conj(c) = (w2r cr + w2i ci) + j (w2i cr - w2r ci)   (reuses q_*)
  wire signed [41:0] cc_re = q_rr + q_ii;
  wire signed [41:0] cc_im = q_ri - q_ir;

  function signed [24:0] sat25;
    input signed [42:0] v;
    begin
      if (v > $signed({{18{1'b0}}, SAT_POS}))      sat25 = SAT_POS;
      else if (v < $signed({{18{1'b1}}, SAT_NEG})) sat25 = SAT_NEG;
      else                                         sat25 = v[24:0];
    end
  endfunction

  always @(posedge clk) begin
    if (rst) begin
      state     <= S_IDLE;
      busy      <= 1'b0;
      done      <= 1'b0;
      div_start <= 1'b0;
      got_re    <= 1'b0;
      got_im    <= 1'b0;
      sr <= 16'sd0; si <= 16'sd0; jr <= 16'sd0; ji <= 16'sd0;
      cr <= 16'sd16384; ci <= 16'sd0;
      c_phase <= 1'b0;
      ome  <= 18'sd65536;
      d_re <= 17'sd0; d_im <= 17'sd0;
      p_rr <= 34'sd0; p_ii <= 34'sd0;
      n_re <= 35'sd0; n_im <= 35'sd0;
      den  <= 48'd1;
      num_re <= 48'sd0; num_im <= 48'sd0;
      q_rr <= 41'sd0; q_ii <= 41'sd0; q_ri <= 41'sd0; q_ir <= 41'sd0;
      w1_re <= ONE_Q16; w1_im <= 25'sd0;
      w2_re <= 25'sd0;  w2_im <= 25'sd0;
    end else begin
      done      <= 1'b0;
      div_start <= 1'b0;

      case (state)
        S_IDLE: begin
          busy <= 1'b0;
          if (start) begin
            busy  <= 1'b1;
            sr <= s_re;  si <= s_im;
            jr <= j_re;  ji <= j_im;
            cr <= cal_re; ci <= cal_im;
            ome   <= 18'sd65536 - $signed({2'b00, eps});
            state <= S_D;
          end
        end

        // d = s - j
        S_D: begin
          d_re  <= sr - jr;
          d_im  <= si - ji;
          state <= S_MUL;
        end

        // one 17x17 or 18x17 multiply per DSP, registered
        S_MUL: begin
          p_rr  <= d_re * d_re;
          p_ii  <= d_im * d_im;
          n_re  <= ome * d_re;
          n_im  <= ome * d_im;
          state <= S_DEN;
        end

        // |d|^2 with guard, numerators to Q.30, start both dividers
        S_DEN: begin
          if ($unsigned(den_sum) < DEN_MIN_Q30)
            den <= {13'd0, DEN_MIN_Q30};
          else
            den <= {13'd0, den_sum};
          num_re    <= {{14{n_re[34]}}, n_re[34:1]};
          num_im    <= {{14{n_im[34]}}, n_im[34:1]};
          div_start <= 1'b1;
          got_re    <= 1'b0;
          got_im    <= 1'b0;
          state     <= S_DIVW;
        end

        S_DIVW: begin
          if (div_done_re) begin w2_re <= qd_re; got_re <= 1'b1; end
          if (div_done_im) begin w2_im <= qd_im; got_im <= 1'b1; end
          if ((got_re || div_done_re) && (got_im || div_done_im))
            state <= S_W1A;
        end

        // s * w2 : 16x25 multiplies, one DSP each
        S_W1A: begin
          q_rr  <= sr * w2_re;
          q_ii  <= si * w2_im;
          q_ri  <= sr * w2_im;
          q_ir  <= si * w2_re;
          state <= S_W1B;
        end

        S_W1B: begin
          w1_re   <= sat25(w1r_full);
          w1_im   <= sat25(w1i_full);
          c_phase <= 1'b0;
          state   <= S_C1;
        end

        // w2_out = w2 * conj(c): products, then sums
        S_C1: begin
          if (!c_phase) begin
            q_rr    <= cr * w2_re;
            q_ii    <= ci * w2_im;
            q_ri    <= cr * w2_im;
            q_ir    <= ci * w2_re;
            c_phase <= 1'b1;
          end else begin
            w2_re <= sat25(cc_re >>> 14);     // c is Q2.14
            w2_im <= sat25(cc_im >>> 14);
            busy  <= 1'b0;
            done  <= 1'b1;
            state <= S_IDLE;
          end
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
