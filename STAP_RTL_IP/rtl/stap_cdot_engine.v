//-----------------------------------------------------------------------------
// stap_cdot_engine : interleaved complex floating point dot-product engine
//
//   For every slot s the engine evaluates
//        acc[s] = (first ? 0 : acc[s]) - a * conj(b)
//   one term per clock. Consecutive terms of the same slot must be issued at
//   least G (>=3) cycles apart; the sequencer hides this by interleaving G
//   independent dot products (round robin over the slots).
//
//   Pipeline (6 cycles, input -> output):
//     t0..t1  4x fp_mul        ar*br, ai*bi, ai*br, ar*bi
//     t2..t3  2x fp_add        pr = ar*br + ai*bi ; pi = ai*br - ar*bi
//     t4..t5  2x fp_add        acc - p
//     t6      result valid, acc[slot] updated, out_valid on last term
//-----------------------------------------------------------------------------
module stap_cdot_engine #(
    parameter integer G    = 4,              // number of interleaved slots
    parameter integer TAGW = 16              // user tag carried to the output
) (
    input  wire                  clk,
    input  wire                  rst_n,
    // term input
    input  wire                  in_valid,
    input  wire [$clog2(G)-1:0]  in_slot,
    input  wire                  in_first,
    input  wire                  in_last,
    input  wire [TAGW-1:0]       in_tag,
    input  wire [31:0]           ar, ai, br, bi,
    // result output (one per dot product, on its last term)
    output reg                   out_valid,
    output reg  [TAGW-1:0]       out_tag,
    output wire [31:0]           out_re,
    output wire [31:0]           out_im,
    output wire                  busy
);

    localparam integer SW  = (G > 1) ? $clog2(G) : 1;
    localparam integer LAT = 6;

    //--------------------------------------------------------- products
    wire [31:0] p_rr, p_ii, p_ir, p_ri;
    stap_fp_mul u_m0 (.clk(clk), .a(ar), .b(br), .y(p_rr));
    stap_fp_mul u_m1 (.clk(clk), .a(ai), .b(bi), .y(p_ii));
    stap_fp_mul u_m2 (.clk(clk), .a(ai), .b(br), .y(p_ir));
    stap_fp_mul u_m3 (.clk(clk), .a(ar), .b(bi), .y(p_ri));

    // a * conj(b) = (ar*br + ai*bi) + j(ai*br - ar*bi)
    wire [31:0] pr, pi;
    stap_fp_add u_a0 (.clk(clk), .a(p_rr), .b(p_ii), .y(pr));
    stap_fp_add u_a1 (.clk(clk), .a(p_ir), .b({~p_ri[31], p_ri[30:0]}), .y(pi));

    //--------------------------------------------------------- control pipe
    reg [LAT:1]    v_d;
    reg [SW-1:0]   s_d   [1:LAT];
    reg            f_d   [1:LAT];
    reg            l_d   [1:LAT];
    reg [TAGW-1:0] t_d   [1:LAT];
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            v_d <= {LAT{1'b0}};
        else
            v_d <= {v_d[LAT-1:1], in_valid};
    end
    always @(posedge clk) begin
        s_d[1] <= in_slot;
        f_d[1] <= in_first;
        l_d[1] <= in_last;
        t_d[1] <= in_tag;
        for (k = 2; k <= LAT; k = k + 1) begin
            s_d[k] <= s_d[k-1];
            f_d[k] <= f_d[k-1];
            l_d[k] <= l_d[k-1];
            t_d[k] <= t_d[k-1];
        end
    end

    //--------------------------------------------------------- accumulate
    reg  [31:0] acc_re [0:G-1];
    reg  [31:0] acc_im [0:G-1];
    wire [31:0] op_re = f_d[4] ? 32'd0 : acc_re[s_d[4]];
    wire [31:0] op_im = f_d[4] ? 32'd0 : acc_im[s_d[4]];
    stap_fp_add u_c0 (.clk(clk), .a(op_re), .b({~pr[31], pr[30:0]}), .y(out_re));
    stap_fp_add u_c1 (.clk(clk), .a(op_im), .b({~pi[31], pi[30:0]}), .y(out_im));

    always @(posedge clk) begin
        if (v_d[LAT]) begin
            acc_re[s_d[LAT]] <= out_re;
            acc_im[s_d[LAT]] <= out_im;
        end
    end

    always @(*) begin
        out_valid = v_d[LAT] & l_d[LAT];
        out_tag   = t_d[LAT];
    end

    assign busy = in_valid | (|v_d);

endmodule
