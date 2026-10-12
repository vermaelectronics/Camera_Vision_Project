`timescale 1ns/1ps
// ============================================================================
// mesprit_cmc_2x2  --  CMC block (cumulant matrix creation), Fig. 3 of
//                      Li et al., IEEE TIM vol. 74, 2025, Art. 2003209.
//
// Fourth-order cumulant matrix, Eq. (7), for a 2-element array:
//
//   C(m,n) = cum{x_m, x0*, x0, x_n*}
//          = E[x_m |x0|^2 x_n*] - E[x_m x0*]E[x0 x_n*]
//            - E[x_m x0]E[x0* x_n*] - E[x_m x_n*]E[|x0|^2]
//
// With R(m,n) = E[x_m x_n*], P(m) = E[x_m x0], M4(m,n) = E[|x0|^2 x_m x_n*]:
//
//   c00 = M4_00 - 2 R00^2      - |P0|^2
//   c11 = M4_11 - |R01|^2      - |P1|^2 - R11 R00
//   c01 = M4_01 - 2 R00 R01    - P0 P1*
//   c10 = c01*                         (C is Hermitian)
//
// The signal subspace is built from C without any knowledge of the number
// of sources (Section III-A); a Gaussian (thermal) noise floor has zero
// fourth-order cumulant and is suppressed.
//
// use_cov = 1 bypasses the cumulant and outputs the covariance R instead
// (useful for Gaussian-like noise jammers, whose 4th-order cumulant is 0).
//
// Streaming: one snapshot per clock (valid_in may be gapped). Windows of
// 2^log2_snap snapshots run back to back; the per-window statistics are
// divided by the window length (serial shift, log2_snap cycles) and combined
// by a small sequential FSM (1 shared multiplier, ~17 cycles) while the next
// window accumulates. log2_snap is sampled at each window start.
//
// Number formats (inputs are Q1.15):
//   second-order terms Q2.30 (33 b),  fourth-order terms Q4.60 (66 b)
//   outputs c00, c11, c01_re, c01_im : signed Q4.60 in CW bits
// ============================================================================
module mesprit_cmc_2x2 #(
    parameter integer MAX_LOG2_SNAP = 14,     // largest supported window 2^14
    parameter integer CW            = 72      // output width
)(
    input  wire                  clk,
    input  wire                  rst,

    input  wire [4:0]            log2_snap,   // clamped to [7, MAX_LOG2_SNAP]
    input  wire                  use_cov,

    input  wire                  valid_in,
    input  wire signed [15:0]    x0_i,
    input  wire signed [15:0]    x0_q,
    input  wire signed [15:0]    x1_i,
    input  wire signed [15:0]    x1_q,

    output reg  signed [CW-1:0]  c00,
    output reg  signed [CW-1:0]  c11,
    output reg  signed [CW-1:0]  c01_re,
    output reg  signed [CW-1:0]  c01_im,
    output reg                   c_valid,
    output reg                   overrun      // window finished while FSM busy
);

    localparam integer A2 = 33 + MAX_LOG2_SNAP;   // 2nd-order accumulator width
    localparam integer A4 = 66 + MAX_LOG2_SNAP;   // 4th-order accumulator width

    // ------------------------------------------------------------------
    // Window length control
    // ------------------------------------------------------------------
    wire [4:0] l_req = (log2_snap < 7)             ? 5'd7 :
                       (log2_snap > MAX_LOG2_SNAP) ? MAX_LOG2_SNAP[4:0] : log2_snap;

    reg  [4:0]               l_cur;
    reg  [MAX_LOG2_SNAP-1:0] cnt;
    wire [MAX_LOG2_SNAP:0]   win_last_cnt = ({{MAX_LOG2_SNAP{1'b0}}, 1'b1} << l_cur) - 1'b1;
    wire                     last_in = ({1'b0, cnt} == win_last_cnt);

    always @(posedge clk) begin
        if (rst) begin
            cnt   <= {MAX_LOG2_SNAP{1'b0}};
            l_cur <= l_req;
        end else if (valid_in) begin
            if (last_in) begin
                cnt   <= {MAX_LOG2_SNAP{1'b0}};
                l_cur <= l_req;                    // takes effect next window
            end else begin
                cnt   <= cnt + 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Stage 1 : 16 x 16 products
    // ------------------------------------------------------------------
    reg signed [31:0] i0i0, q0q0, i1i1, q1q1, i0i1, q0q1, q0i1, i0q1, i0q0;
    reg               v1, l1;
    reg [4:0]         lsh1;

    always @(posedge clk) begin
        if (rst) begin
            v1 <= 1'b0; l1 <= 1'b0; lsh1 <= 5'd0;
            i0i0 <= 0; q0q0 <= 0; i1i1 <= 0; q1q1 <= 0;
            i0i1 <= 0; q0q1 <= 0; q0i1 <= 0; i0q1 <= 0; i0q0 <= 0;
        end else begin
            v1   <= valid_in;
            l1   <= valid_in & last_in;
            lsh1 <= l_cur;
            if (valid_in) begin
                i0i0 <= x0_i * x0_i;  q0q0 <= x0_q * x0_q;
                i1i1 <= x1_i * x1_i;  q1q1 <= x1_q * x1_q;
                i0i1 <= x0_i * x1_i;  q0q1 <= x0_q * x1_q;
                q0i1 <= x0_q * x1_i;  i0q1 <= x0_i * x1_q;
                i0q0 <= x0_i * x0_q;
            end
        end
    end

    // ------------------------------------------------------------------
    // Stage 2 : second-order instantaneous terms (Q2.30, 33 bit)
    //   e00 = |x0|^2, e11 = |x1|^2, e01 = x0 x1*, f0 = x0 x0, f1 = x1 x0
    // ------------------------------------------------------------------
    reg signed [32:0] e00, e11, e01r, e01i, f0r, f0i, f1r, f1i;
    reg               v2, l2;
    reg [4:0]         lsh2;

    always @(posedge clk) begin
        if (rst) begin
            v2 <= 1'b0; l2 <= 1'b0; lsh2 <= 5'd0;
            e00 <= 0; e11 <= 0; e01r <= 0; e01i <= 0;
            f0r <= 0; f0i <= 0; f1r <= 0; f1i <= 0;
        end else begin
            v2   <= v1;
            l2   <= l1;
            lsh2 <= lsh1;
            if (v1) begin
                e00  <= $signed({i0i0[31], i0i0}) + $signed({q0q0[31], q0q0});
                e11  <= $signed({i1i1[31], i1i1}) + $signed({q1q1[31], q1q1});
                e01r <= $signed({i0i1[31], i0i1}) + $signed({q0q1[31], q0q1});
                e01i <= $signed({q0i1[31], q0i1}) - $signed({i0q1[31], i0q1});
                f0r  <= $signed({i0i0[31], i0i0}) - $signed({q0q0[31], q0q0});
                f0i  <= $signed({i0q0, 1'b0});
                f1r  <= $signed({i0i1[31], i0i1}) - $signed({q0q1[31], q0q1});
                f1i  <= $signed({q0i1[31], q0i1}) + $signed({i0q1[31], i0q1});
            end
        end
    end

    // ------------------------------------------------------------------
    // Stage 3 : fourth-order instantaneous terms (Q4.60, 66 bit)
    //   g00 = |x0|^4, g11 = |x0|^2|x1|^2, g01 = |x0|^2 x0 x1*
    // Two register levels after the 33 x 33 multipliers (3a, 3b) so that
    // synthesis can use the DSP48 M and P pipeline registers.
    // ------------------------------------------------------------------
    reg signed [65:0] g00_a, g11_a, g01r_a, g01i_a;
    reg signed [32:0] e00_a, e11_a, e01r_a, e01i_a, f0r_a, f0i_a, f1r_a, f1i_a;
    reg               v3a, l3a;
    reg [4:0]         lsh3a;

    reg signed [65:0] g00, g11, g01r, g01i;
    reg signed [32:0] e00_3, e11_3, e01r_3, e01i_3, f0r_3, f0i_3, f1r_3, f1i_3;
    reg               v3, l3;
    reg [4:0]         lsh3;

    always @(posedge clk) begin
        if (rst) begin
            v3a <= 1'b0; l3a <= 1'b0; lsh3a <= 5'd0;
            g00_a <= 0; g11_a <= 0; g01r_a <= 0; g01i_a <= 0;
            e00_a <= 0; e11_a <= 0; e01r_a <= 0; e01i_a <= 0;
            f0r_a <= 0; f0i_a <= 0; f1r_a <= 0; f1i_a <= 0;
            v3 <= 1'b0; l3 <= 1'b0; lsh3 <= 5'd0;
            g00 <= 0; g11 <= 0; g01r <= 0; g01i <= 0;
            e00_3 <= 0; e11_3 <= 0; e01r_3 <= 0; e01i_3 <= 0;
            f0r_3 <= 0; f0i_3 <= 0; f1r_3 <= 0; f1i_3 <= 0;
        end else begin
            // 3a : multiply
            v3a   <= v2;
            l3a   <= l2;
            lsh3a <= lsh2;
            if (v2) begin
                g00_a  <= e00 * e00;
                g11_a  <= e00 * e11;
                g01r_a <= e00 * e01r;
                g01i_a <= e00 * e01i;
                e00_a <= e00;  e11_a <= e11;  e01r_a <= e01r; e01i_a <= e01i;
                f0r_a <= f0r;  f0i_a <= f0i;  f1r_a  <= f1r;  f1i_a  <= f1i;
            end
            // 3b : register
            v3   <= v3a;
            l3   <= l3a;
            lsh3 <= lsh3a;
            if (v3a) begin
                g00 <= g00_a; g11 <= g11_a; g01r <= g01r_a; g01i <= g01i_a;
                e00_3 <= e00_a; e11_3 <= e11_a; e01r_3 <= e01r_a; e01i_3 <= e01i_a;
                f0r_3 <= f0r_a; f0i_3 <= f0i_a; f1r_3  <= f1r_a;  f1i_3  <= f1i_a;
            end
        end
    end

    // ------------------------------------------------------------------
    // Stage 4 : accumulation over the window
    // ------------------------------------------------------------------
    reg signed [A2-1:0] s_e00, s_e11, s_e01r, s_e01i, s_f0r, s_f0i, s_f1r, s_f1i;
    reg signed [A4-1:0] s_g00, s_g11, s_g01r, s_g01i;

    wire signed [A2-1:0] t_e00  = s_e00  + e00_3;
    wire signed [A2-1:0] t_e11  = s_e11  + e11_3;
    wire signed [A2-1:0] t_e01r = s_e01r + e01r_3;
    wire signed [A2-1:0] t_e01i = s_e01i + e01i_3;
    wire signed [A2-1:0] t_f0r  = s_f0r  + f0r_3;
    wire signed [A2-1:0] t_f0i  = s_f0i  + f0i_3;
    wire signed [A2-1:0] t_f1r  = s_f1r  + f1r_3;
    wire signed [A2-1:0] t_f1i  = s_f1i  + f1i_3;
    wire signed [A4-1:0] t_g00  = s_g00  + g00;
    wire signed [A4-1:0] t_g11  = s_g11  + g11;
    wire signed [A4-1:0] t_g01r = s_g01r + g01r;
    wire signed [A4-1:0] t_g01i = s_g01i + g01i;

    // Window sums are latched and then divided by 2^log2_snap with a serial
    // arithmetic shift (one bit per clock, <= MAX_LOG2_SNAP cycles), which
    // avoids twelve wide barrel shifters. A mean of an n-bit quantity fits
    // in n bits, so the combine FSM uses the low 34 / 67 bits.
    reg signed [A2-1:0] w_e00, w_e11, w_e01r, w_e01i, w_f0r, w_f0i, w_f1r, w_f1i;
    reg signed [A4-1:0] w_g00, w_g11, w_g01r, w_g01i;
    reg [4:0]           shift_left;
    reg                 shifting;
    reg                 stats_valid;

    wire signed [33:0] R00  = w_e00[33:0];     // Q2.30 means
    wire signed [33:0] R11  = w_e11[33:0];
    wire signed [33:0] R01r = w_e01r[33:0];
    wire signed [33:0] R01i = w_e01i[33:0];
    wire signed [33:0] P0r  = w_f0r[33:0];
    wire signed [33:0] P0i  = w_f0i[33:0];
    wire signed [33:0] P1r  = w_f1r[33:0];
    wire signed [33:0] P1i  = w_f1i[33:0];
    wire signed [66:0] M00  = w_g00[66:0];     // Q4.60 means
    wire signed [66:0] M11  = w_g11[66:0];
    wire signed [66:0] M01r = w_g01r[66:0];
    wire signed [66:0] M01i = w_g01i[66:0];

    always @(posedge clk) begin
        if (rst) begin
            s_e00 <= 0; s_e11 <= 0; s_e01r <= 0; s_e01i <= 0;
            s_f0r <= 0; s_f0i <= 0; s_f1r  <= 0; s_f1i  <= 0;
            s_g00 <= 0; s_g11 <= 0; s_g01r <= 0; s_g01i <= 0;
            w_e00 <= 0; w_e11 <= 0; w_e01r <= 0; w_e01i <= 0;
            w_f0r <= 0; w_f0i <= 0; w_f1r  <= 0; w_f1i  <= 0;
            w_g00 <= 0; w_g11 <= 0; w_g01r <= 0; w_g01i <= 0;
            shift_left <= 5'd0; shifting <= 1'b0;
            stats_valid <= 1'b0;
        end else begin
            stats_valid <= 1'b0;

            if (shifting) begin
                if (shift_left == 5'd0) begin
                    shifting    <= 1'b0;
                    stats_valid <= 1'b1;
                end else begin
                    shift_left <= shift_left - 1'b1;
                    w_e00 <= w_e00 >>> 1; w_e11 <= w_e11 >>> 1;
                    w_e01r <= w_e01r >>> 1; w_e01i <= w_e01i >>> 1;
                    w_f0r <= w_f0r >>> 1; w_f0i <= w_f0i >>> 1;
                    w_f1r <= w_f1r >>> 1; w_f1i <= w_f1i >>> 1;
                    w_g00 <= w_g00 >>> 1; w_g11 <= w_g11 >>> 1;
                    w_g01r <= w_g01r >>> 1; w_g01i <= w_g01i >>> 1;
                end
            end

            if (v3) begin
                if (l3) begin
                    w_e00 <= t_e00;  w_e11 <= t_e11;  w_e01r <= t_e01r; w_e01i <= t_e01i;
                    w_f0r <= t_f0r;  w_f0i <= t_f0i;  w_f1r  <= t_f1r;  w_f1i  <= t_f1i;
                    w_g00 <= t_g00;  w_g11 <= t_g11;  w_g01r <= t_g01r; w_g01i <= t_g01i;
                    shift_left <= lsh3;
                    shifting   <= 1'b1;
                    s_e00 <= 0; s_e11 <= 0; s_e01r <= 0; s_e01i <= 0;
                    s_f0r <= 0; s_f0i <= 0; s_f1r  <= 0; s_f1i  <= 0;
                    s_g00 <= 0; s_g11 <= 0; s_g01r <= 0; s_g01i <= 0;
                end else begin
                    s_e00 <= t_e00; s_e11 <= t_e11; s_e01r <= t_e01r; s_e01i <= t_e01i;
                    s_f0r <= t_f0r; s_f0i <= t_f0i; s_f1r  <= t_f1r;  s_f1i  <= t_f1i;
                    s_g00 <= t_g00; s_g11 <= t_g11; s_g01r <= t_g01r; s_g01i <= t_g01i;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Combine FSM : C = M4 - (products of second-order moments)
    // One shared 34 x 34 multiplier, three-cycle latency, 14 products.
    //
    //  step  product        contribution
    //   0    R00*R00        c00  -= 2x
    //   1    P0r*P0r        c00  -= x
    //   2    P0i*P0i        c00  -= x
    //   3    R01r*R01r      c11  -= x
    //   4    R01i*R01i      c11  -= x
    //   5    P1r*P1r        c11  -= x
    //   6    P1i*P1i        c11  -= x
    //   7    R11*R00        c11  -= x
    //   8    R00*R01r       c01r -= 2x
    //   9    R00*R01i       c01i -= 2x
    //  10    P0r*P1r        c01r -= x      (P0 P1* real part)
    //  11    P0i*P1i        c01r -= x
    //  12    P0i*P1r        c01i -= x      (P0 P1* imag part)
    //  13    P0r*P1i        c01i += x
    // ------------------------------------------------------------------
    localparam integer NSTEP = 14;

    reg               busy;
    reg [4:0]         step;           // operand issue index
    reg signed [33:0] op_a, op_b;
    reg signed [33:0] op_a_n, op_b_n;
    reg               op_v;
    reg [3:0]         op_idx;
    reg signed [67:0] prod_m, prod;   // two registers after the multiplier
    reg               v_m, prod_v;
    reg [3:0]         idx_m, prod_idx;
    reg               cov_mode;
    reg signed [CW-1:0] acc00, acc11, acc01r, acc01i;

    wire signed [CW-1:0] px  = {{(CW-68){prod[67]}}, prod};
    wire signed [CW-1:0] px2 = {{(CW-69){prod[67]}}, prod, 1'b0};

    always @(*) begin
        case (step[3:0])
            4'd0:    begin op_a_n = R00;  op_b_n = R00;  end
            4'd1:    begin op_a_n = P0r;  op_b_n = P0r;  end
            4'd2:    begin op_a_n = P0i;  op_b_n = P0i;  end
            4'd3:    begin op_a_n = R01r; op_b_n = R01r; end
            4'd4:    begin op_a_n = R01i; op_b_n = R01i; end
            4'd5:    begin op_a_n = P1r;  op_b_n = P1r;  end
            4'd6:    begin op_a_n = P1i;  op_b_n = P1i;  end
            4'd7:    begin op_a_n = R11;  op_b_n = R00;  end
            4'd8:    begin op_a_n = R00;  op_b_n = R01r; end
            4'd9:    begin op_a_n = R00;  op_b_n = R01i; end
            4'd10:   begin op_a_n = P0r;  op_b_n = P1r;  end
            4'd11:   begin op_a_n = P0i;  op_b_n = P1i;  end
            4'd12:   begin op_a_n = P0i;  op_b_n = P1r;  end
            default: begin op_a_n = P0r;  op_b_n = P1i;  end
        endcase
    end
    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; step <= 5'd0;
            op_a <= 0; op_b <= 0; op_v <= 1'b0; op_idx <= 4'd0;
            prod_m <= 0; prod <= 0; v_m <= 1'b0; prod_v <= 1'b0;
            idx_m <= 4'd0; prod_idx <= 4'd0;
            cov_mode <= 1'b0;
            acc00 <= 0; acc11 <= 0; acc01r <= 0; acc01i <= 0;
            c00 <= 0; c11 <= 0; c01_re <= 0; c01_im <= 0;
            c_valid <= 1'b0; overrun <= 1'b0;
        end else begin
            c_valid <= 1'b0;
            overrun <= 1'b0;

            // pipeline: operands -> product
            op_v     <= 1'b0;
            v_m      <= op_v;
            prod_v   <= v_m;
            idx_m    <= op_idx;
            prod_idx <= idx_m;
            prod_m   <= op_a * op_b;
            prod     <= prod_m;

            if ((v3 && l3 && (shifting || busy)) || (stats_valid && busy))
                overrun <= 1'b1;               // new window while previous one is still in use

            if (stats_valid && !busy) begin
                busy     <= 1'b1;
                step     <= 5'd0;
                cov_mode <= use_cov;
                acc00    <= {{(CW-67){M00[66]}},  M00};
                acc11    <= {{(CW-67){M11[66]}},  M11};
                acc01r   <= {{(CW-67){M01r[66]}}, M01r};
                acc01i   <= {{(CW-67){M01i[66]}}, M01i};
            end else if (busy) begin
                if (step < NSTEP) begin
                    op_a   <= op_a_n;
                    op_b   <= op_b_n;
                    op_v   <= 1'b1;
                    op_idx <= step[3:0];
                    step   <= step + 1'b1;
                end

                if (prod_v) begin
                    case (prod_idx)
                        4'd0:    acc00  <= acc00  - px2;
                        4'd1,
                        4'd2:    acc00  <= acc00  - px;
                        4'd3, 4'd4, 4'd5, 4'd6,
                        4'd7:    acc11  <= acc11  - px;
                        4'd8:    acc01r <= acc01r - px2;
                        4'd9:    acc01i <= acc01i - px2;
                        4'd10,
                        4'd11:   acc01r <= acc01r - px;
                        4'd12:   acc01i <= acc01i - px;
                        default: acc01i <= acc01i + px;
                    endcase

                    if (prod_idx == NSTEP - 1) begin
                        busy    <= 1'b0;
                        c_valid <= 1'b1;
                        if (cov_mode) begin
                            // covariance bypass, scaled to the same Q4.60 grid
                            c00    <= {{(CW-64){R00[33]}},  R00,  30'd0};
                            c11    <= {{(CW-64){R11[33]}},  R11,  30'd0};
                            c01_re <= {{(CW-64){R01r[33]}}, R01r, 30'd0};
                            c01_im <= {{(CW-64){R01i[33]}}, R01i, 30'd0};
                        end else begin
                            c00    <= acc00;
                            c11    <= acc11;
                            c01_re <= acc01r;
                            c01_im <= (prod_idx == 4'd13) ? acc01i + px : acc01i;
                        end
                    end
                end
            end
        end
    end

endmodule
