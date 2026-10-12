`timescale 1ns/1ps
// ============================================================================
// uesprit_rma_2x2  --  RMA block (rotation matrix approximation), Fig. 3.
//
// Input : normalised cumulant matrix C = [[a, b], [b*, d]]  (a, d real)
//         as 32-bit mantissas (|.| < 2^30, Q30).
//
// 1) C^p, Eq. (38), p = 2^p_log2 (1, 2, 4, 8) by repeated squaring of a
//    2x2 Hermitian matrix:
//        [[a,b],[b*,d]]^2 = [[a^2+|b|^2, (a+d) b], [., d^2+|b|^2]]
//    re-normalised (block floating point) after every squaring.
//
// 2) Forward-backward average, Eq. (26), with Pi = [[0,1],[1,0]]:
//        C_fb = [[A, b], [b*, A]],   A = (a + d) / 2
//
// 3) Unitary transform, Eq. (25), Q = 1/sqrt(2) [[1, j], [1, -j]]:
//        C_r = Q^H C_fb Q = [[A + Re b, Im b], [Im b, A - Re b]]   (real)
//
// 4) Selection matrices, Eqs. (30)-(31), for M = 2 (Q_{M-1} = 1):
//        K1 = sqrt(2) [1, 0],   K2 = sqrt(2) [0, -1]
//    C_r1 = K1 C_r = sqrt(2) r,  r = [cr00, cr01]           Eq. (33)
//    C_r2 = K2 C_r = sqrt(2) s,  s = -[cr10, cr11]          Eq. (34)
//
// 5) Psi_r = (C_r1^T C_r1 + mu I)^-1 C_r1^T C_r2, Eqs. (36)-(37).
//    C_r1 is 1x2, so Psi_r is rank one and its single non-zero eigenvalue
//    is its trace:
//        psi = (r . s) / (|r|^2 + mu')  =  N / D,   mu' = mu / 2
//    (the PIE stage of the paper therefore reduces to one division; no
//    iterative EVD is needed for a two-element array).
//
// Output: N, D (signed Q60, NW bits), D > 0 for mu > 0.
// Resources: one shared 34 x 34 multiplier (4 DSP48E2 on UltraScale+).
// Latency : ~ 10*p_log2 + 9 clock cycles.
// ============================================================================
module uesprit_rma_2x2 #(
    parameter integer NW = 70
)(
    input  wire                 clk,
    input  wire                 rst,

    input  wire [1:0]           p_log2,      // p = 1, 2, 4, 8
    input  wire [31:0]          mu_q30,      // regulariser mu' (unsigned Q30)

    input  wire                 in_valid,
    input  wire signed [31:0]   in_a,        // c00
    input  wire signed [31:0]   in_d,        // c11
    input  wire signed [31:0]   in_br,       // Re c01
    input  wire signed [31:0]   in_bi,       // Im c01
    output wire                 in_ready,

    output reg                  out_valid,
    output reg  signed [NW-1:0] num,         // N = r . s
    output reg  signed [NW-1:0] den,         // D = |r|^2 + mu'
    output reg  signed [31:0]   cp_a,        // normalised C^p (debug)
    output reg  signed [31:0]   cp_d,
    output reg  signed [31:0]   cp_br,
    output reg  signed [31:0]   cp_bi
);

    localparam [2:0] S_IDLE = 3'd0,
                     S_SQ   = 3'd1,
                     S_SQN  = 3'd2,
                     S_FB   = 3'd3,
                     S_PSI  = 3'd4;

    reg [2:0]  state;
    reg [1:0]  iter, iter_n;
    reg [3:0]  step;
    reg [31:0] mu_r;

    reg signed [31:0] a, d, br, bi;
    reg signed [33:0] cr00, cr01, cr10, cr11;

    // shared multiplier, operands registered, product registered
    reg signed [33:0] op_a, op_b;
    reg               op_v;
    reg [3:0]         op_idx;
    reg signed [67:0] prod;
    reg               prod_v;
    reg [3:0]         prod_idx;

    wire signed [65:0] p66 = prod[65:0];

    // squaring accumulators (|.| < 3 * 2^60)
    reg signed [65:0] na, nd, nbr, nbi;

    wire signed [32:0] s_ad = a + d;        // a + d (33 bit)

    // re-normalise C^2
    wire signed [31:0] mn0, mn1, mn2, mn3;
    wire               mzero;

    uesprit_bfp_norm4 #(.W(66)) u_norm (
        .v0 (na), .v1 (nd), .v2 (nbr), .v3 (nbi),
        .m0 (mn0), .m1 (mn1), .m2 (mn2), .m3 (mn3),
        .zero (mzero)
    );

    assign in_ready = (state == S_IDLE);

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            iter <= 2'd0; iter_n <= 2'd0; step <= 4'd0; mu_r <= 32'd0;
            a <= 0; d <= 0; br <= 0; bi <= 0;
            cr00 <= 0; cr01 <= 0; cr10 <= 0; cr11 <= 0;
            op_a <= 0; op_b <= 0; op_v <= 1'b0; op_idx <= 4'd0;
            prod <= 0; prod_v <= 1'b0; prod_idx <= 4'd0;
            na <= 0; nd <= 0; nbr <= 0; nbi <= 0;
            out_valid <= 1'b0; num <= 0; den <= 0;
            cp_a <= 0; cp_d <= 0; cp_br <= 0; cp_bi <= 0;
        end else begin
            out_valid <= 1'b0;

            op_v     <= 1'b0;
            prod     <= op_a * op_b;
            prod_v   <= op_v;
            prod_idx <= op_idx;

            case (state)
            // ---------------------------------------------------------
            S_IDLE: begin
                if (in_valid) begin
                    a  <= in_a;  d  <= in_d;
                    br <= in_br; bi <= in_bi;
                    mu_r   <= mu_q30;
                    iter   <= 2'd0;
                    iter_n <= p_log2;
                    step   <= 4'd0;
                    na <= 0; nd <= 0; nbr <= 0; nbi <= 0;
                    state  <= (p_log2 == 2'd0) ? S_FB : S_SQ;
                end
            end
            // ---------------------------------------------------------
            // C <- C^2 : products a*a, d*d, br*br, bi*bi, (a+d)*br, (a+d)*bi
            S_SQ: begin
                if (step < 4'd6) begin
                    op_v   <= 1'b1;
                    op_idx <= step;
                    step   <= step + 1'b1;
                    case (step)
                        4'd0:    begin op_a <= a;    op_b <= a;  end
                        4'd1:    begin op_a <= d;    op_b <= d;  end
                        4'd2:    begin op_a <= br;   op_b <= br; end
                        4'd3:    begin op_a <= bi;   op_b <= bi; end
                        4'd4:    begin op_a <= s_ad; op_b <= br; end
                        default: begin op_a <= s_ad; op_b <= bi; end
                    endcase
                end
                if (prod_v) begin
                    case (prod_idx)
                        4'd0: na <= na + p66;
                        4'd1: nd <= nd + p66;
                        4'd2, 4'd3: begin
                            na <= na + p66;
                            nd <= nd + p66;
                        end
                        4'd4: nbr <= p66;
                        default: begin
                            nbi   <= p66;
                            state <= S_SQN;
                        end
                    endcase
                end
            end
            // ---------------------------------------------------------
            S_SQN: begin
                a  <= mn0; d  <= mn1;
                br <= mn2; bi <= mn3;
                na <= 0; nd <= 0; nbr <= 0; nbi <= 0;
                step <= 4'd0;
                iter <= iter + 1'b1;
                state <= (iter + 1'b1 == iter_n) ? S_FB : S_SQ;
            end
            // ---------------------------------------------------------
            // forward-backward average + unitary transform -> C_r
            S_FB: begin
                cp_a <= a; cp_d <= d; cp_br <= br; cp_bi <= bi;
                cr00 <= (s_ad >>> 1) + br;
                cr01 <= bi;
                cr10 <= bi;
                cr11 <= (s_ad >>> 1) - br;
                num  <= {NW{1'b0}};
                den  <= {{(NW-62){1'b0}}, mu_r, 30'd0};   // mu' in Q60
                step <= 4'd0;
                state <= S_PSI;
            end
            // ---------------------------------------------------------
            // N = r.s = -(cr00*cr10 + cr01*cr11),  D = cr00^2 + cr01^2 + mu'
            S_PSI: begin
                if (step < 4'd4) begin
                    op_v   <= 1'b1;
                    op_idx <= step;
                    step   <= step + 1'b1;
                    case (step)
                        4'd0:    begin op_a <= cr00; op_b <= cr10; end
                        4'd1:    begin op_a <= cr01; op_b <= cr11; end
                        4'd2:    begin op_a <= cr00; op_b <= cr00; end
                        default: begin op_a <= cr01; op_b <= cr01; end
                    endcase
                end
                if (prod_v) begin
                    case (prod_idx)
                        4'd0, 4'd1: num <= num - {{(NW-68){prod[67]}}, prod};
                        4'd2:       den <= den + {{(NW-68){prod[67]}}, prod};
                        default: begin
                            den       <= den + {{(NW-68){prod[67]}}, prod};
                            out_valid <= 1'b1;
                            state     <= S_IDLE;
                        end
                    endcase
                end
            end
            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
