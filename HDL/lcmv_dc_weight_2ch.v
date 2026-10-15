`timescale 1ns/1ps

// ============================================================================
// lcmv_dc_weight_2ch.v
//
// ROBUST LCMV WEIGHT SOLVER, 2 ANTENNAS
//
// Reference:
//   R. Li, X. Zhao, X. W. Shi, "Derivative Constrained Robust LCMV
//   Beamforming Algorithm", Progress In Electromagnetics Research C,
//   vol. 4, pp. 43-52, 2008.  Eq. (6)-(9).
//
// Paper problem, eq. (9):
//
//      min  w^H R w
//      s.t. S^H w        = 1        look direction (GPS)
//           w^H B^p A_J  = 0        derivative (null widening)
//           w^H A_J      = eps      enforcement (null deepening)
//
//      w_opt = R^-1 C (C^H R^-1 C)^-1 f
//
// TWO ANTENNAS (M = 2, two degrees of freedom):
//
//   * Derivative row: B = diag(0,1), so w^H B a_J = conj(w2) * e^{j psi_J} = 0
//     forces w2 = 0, which switches the second antenna off and removes the
//     look and null constraints. It cannot be used with M = 2 and is omitted.
//
//   * The remaining constraints are look + enforcement:
//
//         C = [a_s  a_J],   f = [1  eps]^T,   a = [1, exp(j*pi*sin(theta))]^T
//
//     C is 2x2 and invertible, so the paper's solution reduces exactly to
//
//         w = R^-1 C (C^H R^-1 C)^-1 f = R^-1 C C^-1 R C^-H f = C^-H f
//
//     i.e. the solution of a_s^H w = 1, a_J^H w = eps:
//
//         w2 = (1 - eps) * (s - j) / |s - j|^2        s = a_s(2), j = a_J(2)
//         w1 = 1 - conj(s) * w2
//
//     This is the exact LCMV optimum for this constraint set. R cancels
//     (all degrees of freedom are used by constraints), so no covariance
//     or matrix inverse is required.
//
// eps (EPS_Q16, Q16.16):
//   0       -> ideal null at jam_angle
//   > 0     -> residual gain eps toward the jammer (paper: 0.001-0.006).
//              With M = 2 this makes the null shallower; it does not widen it.
//
// GUARD:
//   When the jammer is close to the GPS direction, |s - j|^2 -> 0 and the
//   weights grow without bound. |s - j|^2 is clamped to DEN_MIN_Q16.
//
// Fixed point: steering Q1.15 in, weights Q16.16 out.
// Latency: start -> done = 72 clocks.
// ============================================================================

module lcmv_dc_weight_2ch #(
    parameter signed [31:0] EPS_Q16     = 32'sd0,
    parameter        [31:0] DEN_MIN_Q16 = 32'd655      // 0.01
)(
    input wire clk,
    input wire rst,
    input wire start,

    // a_s = [1, s], a_J = [1, j]   (Q1.15, element 1 is 1)
    input wire signed [15:0] s_re,
    input wire signed [15:0] s_im,
    input wire signed [15:0] j_re,
    input wire signed [15:0] j_im,

    output reg busy,
    output reg done,

    output reg signed [31:0] w1_re,
    output reg signed [31:0] w1_im,
    output reg signed [31:0] w2_re,
    output reg signed [31:0] w2_im
);

    localparam signed [31:0] ONE_Q16 = 32'sd65536;

    localparam S_IDLE = 3'd0;
    localparam S_D    = 3'd1;
    localparam S_DEN  = 3'd2;
    localparam S_NUM  = 3'd3;
    localparam S_DIVW = 3'd4;
    localparam S_W1A  = 3'd5;
    localparam S_W1B  = 3'd6;

    reg [2:0] state;

    // Steering element 2, Q16.16
    reg signed [31:0] sr, si, jr, ji;

    // d = s - j, Q16.16 (|d| <= 2)
    reg signed [31:0] d_re, d_im;

    // |d|^2, Q16.16
    reg        [63:0] den;

    // (1 - eps) * d, Q16.16
    reg signed [63:0] num_re, num_im;

    // conj(s) * w2 products, Q32.32
    reg signed [63:0] p_rr, p_ii, p_ri, p_ir;

    // Dividers
    reg  div_start;
    wire signed [31:0] q_re, q_im;
    wire div_busy_re, div_busy_im;
    wire div_done_re, div_done_im;
    reg  got_re, got_im;

    sdiv_q16_serial u_div_re (
        .clk(clk), .rst(rst), .start(div_start),
        .num(num_re), .den(den),
        .q(q_re), .busy(div_busy_re), .done(div_done_re)
    );

    sdiv_q16_serial u_div_im (
        .clk(clk), .rst(rst), .start(div_start),
        .num(num_im), .den(den),
        .q(q_im), .busy(div_busy_im), .done(div_done_im)
    );

    wire signed [63:0] one_minus_eps = ONE_Q16 - EPS_Q16;
    wire signed [63:0] d2 = $signed(d_re) * $signed(d_re)
                          + $signed(d_im) * $signed(d_im);

    always @(posedge clk) begin

        if (rst) begin
            state     <= S_IDLE;
            busy      <= 1'b0;
            done      <= 1'b0;
            div_start <= 1'b0;
            got_re    <= 1'b0;
            got_im    <= 1'b0;
            sr <= 0; si <= 0; jr <= 0; ji <= 0;
            d_re <= 0; d_im <= 0;
            den <= 64'd1;
            num_re <= 0; num_im <= 0;
            p_rr <= 0; p_ii <= 0; p_ri <= 0; p_ir <= 0;
            w1_re <= ONE_Q16; w1_im <= 32'sd0;
            w2_re <= 32'sd0;  w2_im <= 32'sd0;
        end
        else begin

            done      <= 1'b0;
            div_start <= 1'b0;

            case (state)

                // Q1.15 -> Q16.16
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        sr <= {{15{s_re[15]}}, s_re, 1'b0};
                        si <= {{15{s_im[15]}}, s_im, 1'b0};
                        jr <= {{15{j_re[15]}}, j_re, 1'b0};
                        ji <= {{15{j_im[15]}}, j_im, 1'b0};
                        state <= S_D;
                    end
                end

                // d = s - j
                S_D: begin
                    d_re  <= sr - jr;
                    d_im  <= si - ji;
                    state <= S_DEN;
                end

                // |d|^2 (Q32.32 -> Q16.16), clamped
                S_DEN: begin
                    if ((d2 >>> 16) < $signed({32'd0, DEN_MIN_Q16}))
                        den <= {32'd0, DEN_MIN_Q16};
                    else
                        den <= d2 >>> 16;
                    state <= S_NUM;
                end

                // (1 - eps) * d  (Q16.16)
                S_NUM: begin
                    num_re    <= (one_minus_eps * $signed(d_re)) >>> 16;
                    num_im    <= (one_minus_eps * $signed(d_im)) >>> 16;
                    div_start <= 1'b1;
                    got_re    <= 1'b0;
                    got_im    <= 1'b0;
                    state     <= S_DIVW;
                end

                // w2 = num / |d|^2
                S_DIVW: begin
                    if (div_done_re) begin
                        w2_re  <= q_re;
                        got_re <= 1'b1;
                    end
                    if (div_done_im) begin
                        w2_im  <= q_im;
                        got_im <= 1'b1;
                    end
                    if ((got_re || div_done_re) && (got_im || div_done_im))
                        state <= S_W1A;
                end

                // conj(s) * w2
                S_W1A: begin
                    p_rr  <= $signed(sr) * $signed(w2_re);
                    p_ii  <= $signed(si) * $signed(w2_im);
                    p_ri  <= $signed(sr) * $signed(w2_im);
                    p_ir  <= $signed(si) * $signed(w2_re);
                    state <= S_W1B;
                end

                // w1 = 1 - conj(s) * w2
                //   conj(s)*w2 = (sr*w2r + si*w2i) + j(sr*w2i - si*w2r)
                S_W1B: begin
                    w1_re <= ONE_Q16 - ((p_rr + p_ii) >>> 16);
                    w1_im <= -((p_ri - p_ir) >>> 16);
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;

            endcase
        end
    end

endmodule
