`timescale 1ns/1ps
// ============================================================================
// mesprit_rma_2x2  --  rotation-matrix stage of M-ESPRIT (Section III-A of
//                      Li et al., IEEE TIM vol. 74, 2025, Art. 2003209).
//
// Input : normalised cumulant matrix C = [[a, b], [b*, d]]  (a, d real)
//         as 32-bit mantissas (|.| < 2^30, Q30).
//
// 1) C^p, Eq. (38) / Remark D, p = 2^p_log2 (1, 2, 4, 8), repeated squaring:
//        [[a,b],[b*,d]]^2 = [[a^2+|b|^2, (a+d) b], [., d^2+|b|^2]]
//    re-normalised (block floating point) after every squaring.
//
// 2) Selection, Eqs. (11)-(12), M = 2:
//        C1 = J1 C = [a, b]        (1x2, complex)
//        C2 = J2 C = [b*, d]
//
// 3) Regularised pseudo-inverse and Psi, Eqs. (17), (19):
//        P_C1 = (C1^H C1 + mu I)^-1 C1^H = C1^H / (|C1|^2 + mu)
//        Psi  = P_C1 C2 = C1^H C2 / (|C1|^2 + mu)          (2x2, rank one)
//    Its only non-zero eigenvalue (= trace) is
//        psi = C2 C1^H / (|C1|^2 + mu) = (a + d) b* / (a^2 + |b|^2 + mu)
//    so no EVD is required for a two-element array. For an ideal single
//    source psi = exp(j phi) lies on the unit circle (Remark A).
//
// Output: psi = (num_re + j num_im) / den, all signed Q60 in NW bits.
//         num = (a+d) b*,  den = a^2 + |b|^2 + mu  (den > 0 for mu > 0).
// Resources: one shared 34 x 34 multiplier.
// Latency : ~ 13*p_log2 + 9 clock cycles.
// ============================================================================
module mesprit_rma_2x2 #(
    parameter integer NW = 70
)(
    input  wire                 clk,
    input  wire                 rst,

    input  wire [1:0]           p_log2,      // p = 1, 2, 4, 8
    input  wire [31:0]          mu_q30,      // regulariser mu (unsigned Q30)

    input  wire                 in_valid,
    input  wire signed [31:0]   in_a,        // c00
    input  wire signed [31:0]   in_d,        // c11
    input  wire signed [31:0]   in_br,       // Re c01
    input  wire signed [31:0]   in_bi,       // Im c01
    output wire                 in_ready,

    output reg                  out_valid,
    output reg  signed [NW-1:0] num_re,      // Re{(a+d) b*}
    output reg  signed [NW-1:0] num_im,      // Im{(a+d) b*}
    output reg  signed [NW-1:0] den,         // a^2 + |b|^2 + mu
    output reg  signed [31:0]   cp_a,        // normalised C^p (debug)
    output reg  signed [31:0]   cp_d,
    output reg  signed [31:0]   cp_br,
    output reg  signed [31:0]   cp_bi
);

    localparam [2:0] S_IDLE = 3'd0,
                     S_SQ   = 3'd1,
                     S_SQN  = 3'd2,
                     S_PSI  = 3'd3;

    reg [2:0]  state;
    reg [1:0]  iter, iter_n;
    reg [3:0]  step;
    reg [31:0] mu_r;

    reg signed [31:0] a, d, br, bi;

    // shared multiplier, operands registered, product registered
    reg signed [33:0] op_a, op_b;
    reg               op_v;
    reg [3:0]         op_idx;
    reg signed [67:0] prod_m, prod;      // two registers after the multiplier
    reg               v_m, prod_v;       // (DSP MREG + PREG)
    reg [3:0]         idx_m, prod_idx;

    wire signed [65:0]   p66 = prod[65:0];
    wire signed [NW-1:0] pnw = {{(NW-68){prod[67]}}, prod};

    // squaring accumulators (|.| < 3 * 2^60)
    reg signed [65:0] na, nd, nbr, nbi;

    wire signed [32:0] s_ad = a + d;

    // re-normalise C^2 (2-cycle pipelined normaliser)
    wire signed [31:0] mn0, mn1, mn2, mn3;
    wire               mzero, mn_valid;
    reg                norm_go, norm_wait;

    mesprit_bfp_norm4 #(.W(66)) u_norm (
        .clk (clk), .rst (rst), .in_valid (norm_go),
        .v0 (na), .v1 (nd), .v2 (nbr), .v3 (nbi),
        .out_valid (mn_valid),
        .m0 (mn0), .m1 (mn1), .m2 (mn2), .m3 (mn3),
        .zero (mzero)
    );

    assign in_ready = (state == S_IDLE);

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            iter <= 2'd0; iter_n <= 2'd0; step <= 4'd0; mu_r <= 32'd0;
            a <= 0; d <= 0; br <= 0; bi <= 0;
            op_a <= 0; op_b <= 0; op_v <= 1'b0; op_idx <= 4'd0;
            prod_m <= 0; prod <= 0; v_m <= 1'b0; prod_v <= 1'b0;
            idx_m <= 4'd0; prod_idx <= 4'd0;
            norm_go <= 1'b0; norm_wait <= 1'b0;
            na <= 0; nd <= 0; nbr <= 0; nbi <= 0;
            out_valid <= 1'b0; num_re <= 0; num_im <= 0; den <= 0;
            cp_a <= 0; cp_d <= 0; cp_br <= 0; cp_bi <= 0;
        end else begin
            out_valid <= 1'b0;

            op_v     <= 1'b0;
            norm_go  <= 1'b0;
            prod_m   <= op_a * op_b;
            prod     <= prod_m;
            v_m      <= op_v;
            prod_v   <= v_m;
            idx_m    <= op_idx;
            prod_idx <= idx_m;

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
                    state  <= (p_log2 == 2'd0) ? S_PSI : S_SQ;
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
                if (!norm_wait) begin
                    norm_go   <= 1'b1;            // na..nbi are final now
                    norm_wait <= 1'b1;
                end else if (mn_valid) begin
                    norm_wait <= 1'b0;
                    a  <= mn0; d  <= mn1;
                    br <= mn2; bi <= mn3;
                    na <= 0; nd <= 0; nbr <= 0; nbi <= 0;
                    step <= 4'd0;
                    iter <= iter + 1'b1;
                    state <= (iter + 1'b1 == iter_n) ? S_PSI : S_SQ;
                end
            end
            // ---------------------------------------------------------
            // num = (a+d) b* = (a+d) br - j (a+d) bi
            // den = a^2 + br^2 + bi^2 + mu
            S_PSI: begin
                if (step == 4'd0) begin
                    cp_a <= a; cp_d <= d; cp_br <= br; cp_bi <= bi;
                    num_re <= {NW{1'b0}};
                    num_im <= {NW{1'b0}};
                    den    <= {{(NW-62){1'b0}}, mu_r, 30'd0};   // mu in Q60
                end
                if (step < 4'd5) begin
                    op_v   <= 1'b1;
                    op_idx <= step;
                    step   <= step + 1'b1;
                    case (step)
                        4'd0:    begin op_a <= s_ad; op_b <= br; end
                        4'd1:    begin op_a <= s_ad; op_b <= bi; end
                        4'd2:    begin op_a <= a;    op_b <= a;  end
                        4'd3:    begin op_a <= br;   op_b <= br; end
                        default: begin op_a <= bi;   op_b <= bi; end
                    endcase
                end
                if (prod_v) begin
                    case (prod_idx)
                        4'd0:       num_re <= pnw;
                        4'd1:       num_im <= -pnw;
                        4'd2, 4'd3: den    <= den + pnw;
                        default: begin
                            den       <= den + pnw;
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
