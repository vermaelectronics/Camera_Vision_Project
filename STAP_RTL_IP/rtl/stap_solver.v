//-----------------------------------------------------------------------------
// stap_solver : MVDR / power-inversion weight solver (fp32)
//
//   Solves   w = R^-1 s / (s^H R^-1 s)       (STAP_constraint.m : w_opt)
//
//   R    : NDIM x NDIM Hermitian covariance, lower triangle packed row-wise
//          in an external RAM, addr(i,j) = i*(i+1)/2 + j  (i >= j),
//          64-bit words {im[31:0], re[31:0]}
//   s    : space-time steering vector (MVDR) or e1 (power inversion)
//
//   Algorithm (all arithmetic through one interleaved complex MAC engine):
//     1. LDL^H factorisation in place    R = L D L^H
//          for j = 0..NDIM-1
//            V : v_k  = L_jk * D_k                       k < j
//            T : t_i  = R_ij - sum_k L_ik * conj(v_k)    i >= j
//                D_j  = Re(t_j)
//            DIV: Dinv_j = 1 / D_j
//            SC: L_ij = t_i * Dinv_j                      i > j
//     2. FW : z = L^-1 s                 (forward substitution)
//     3. DG : y = D^-1 z
//     4. BW : x = L^-H y                 (backward substitution)
//     5. AL : alpha = s^H x , ainv = 1/alpha
//     6. NM : w = x * ainv
//
//   The resulting weight vector stays in the X register file and can be
//   read through x_raddr / x_rdata while the solver is idle.
//-----------------------------------------------------------------------------
module stap_solver #(
    parameter integer NDIM = 112,                      // M * N
    parameter integer G    = 4,                        // interleave factor (>=3)
    // derived - do not override
    parameter integer RAW  = $clog2(NDIM * (NDIM + 1) / 2),
    parameter integer IW   = $clog2(NDIM + G + 1)
) (
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     start,
    input  wire                     mode_mvdr,         // 1: MVDR (s from S RAM), 0: PI (s = e1)
    output reg                      busy,
    output reg                      done,              // 1-cycle pulse
    output reg                      err,               // sticky until next start
    output reg  [1:0]               err_code,          // 1: pivot <= 0, 2: alpha <= 0
    output reg  [IW-1:0]            err_col,
    output reg  [31:0]              alpha,
    // covariance / factor RAM (1R1W, synchronous read, 1-cycle latency)
    output reg  [RAW-1:0]           ram_raddr,
    input  wire [63:0]              ram_rdata,
    output reg                      ram_we,
    output reg  [RAW-1:0]           ram_waddr,
    output reg  [63:0]              ram_wdata,
    // steering vector write port (fp32)
    input  wire                     s_we_re,
    input  wire                     s_we_im,
    input  wire [IW-1:0]            s_waddr,
    input  wire [31:0]              s_wdata,
    // weight vector read port (valid when !busy, 1-cycle latency)
    input  wire [IW-1:0]            x_raddr,
    output wire [63:0]              x_rdata
);

    localparam integer SW   = (G > 1) ? $clog2(G) : 1;
    localparam integer TAGW = 3 + 1 + RAW;

    localparam [31:0] FP_ONE  = 32'h3F80_0000;
    localparam [31:0] FP_MONE = 32'hBF80_0000;

    // operand / destination selectors
    localparam [1:0] A_RAM = 2'd0, A_X = 2'd1, A_S = 2'd2;
    localparam [2:0] B_RAM = 3'd0, B_X = 3'd1, B_S = 3'd2, B_V = 3'd3,
                     B_D   = 3'd4, B_DI = 3'd5, B_M1 = 3'd6, B_AI = 3'd7;
    localparam [2:0] D_NONE = 3'd0, D_RAM = 3'd1, D_X = 3'd2, D_V = 3'd3, D_AL = 3'd4;

    // sequencer states
    localparam [4:0] S_IDLE  = 5'd0,  S_V     = 5'd1,  S_T     = 5'd2,
                     S_DIVS  = 5'd3,  S_DIVW  = 5'd4,  S_SC    = 5'd5,
                     S_FW    = 5'd6,  S_DG    = 5'd7,  S_BW    = 5'd8,
                     S_AL    = 5'd9,  S_ADIVS = 5'd10, S_ADIVW = 5'd11,
                     S_NM    = 5'd12, S_DRAIN = 5'd13, S_DONE  = 5'd14;

    //=========================================================================
    // register files (synchronous read)
    //=========================================================================
    reg [31:0] x_re [0:NDIM-1];
    reg [31:0] x_im [0:NDIM-1];
    reg [31:0] s_re [0:NDIM-1];
    reg [31:0] s_im [0:NDIM-1];
    reg [31:0] v_re [0:NDIM-1];
    reg [31:0] v_im [0:NDIM-1];
    reg [31:0] d_rf [0:NDIM-1];
    reg [31:0] di_rf[0:NDIM-1];

    reg [IW-1:0] x_ra, s_ra, v_ra, d_ra, di_ra;        // sequencer addresses (c0)
    reg [63:0]   x_rd, v_rd;
    reg [31:0]   s_rd_re, s_rd_im, d_rd, di_rd;
    reg [IW-1:0] s_ra_q;

    always @(posedge clk) begin
        x_rd    <= {x_im[busy ? x_ra : x_raddr], x_re[busy ? x_ra : x_raddr]};
        v_rd    <= {v_im[v_ra], v_re[v_ra]};
        s_rd_re <= s_re[s_ra];
        s_rd_im <= s_im[s_ra];
        s_ra_q  <= s_ra;
        d_rd    <= d_rf[d_ra];
        di_rd   <= di_rf[di_ra];
        if (s_we_re) s_re[s_waddr] <= s_wdata;
        if (s_we_im) s_im[s_waddr] <= s_wdata;
    end
    assign x_rdata = x_rd;

    // power inversion: s = e1
    wire [63:0] s_rd = mode_mvdr ? {s_rd_im, s_rd_re}
                                 : ((s_ra_q == {IW{1'b0}}) ? {32'd0, FP_ONE} : 64'd0);

    //=========================================================================
    // sequencer
    //=========================================================================
    reg [4:0]     state, nxt;
    reg [IW-1:0]  j, r0, q;
    reg [SW-1:0]  p;
    reg [31:0]    dj, ainv;
    reg           rcp_start;
    wire          rcp_done;
    wire [31:0]   rcp_y;
    wire          eng_busy;

    wire serial = (state == S_FW) || (state == S_BW) || (state == S_AL);
    wire [IW-1:0] row = serial ? r0 : (r0 + p);

    reg  [IW-1:0] qlast;
    always @(*) begin
        case (state)
            S_T:     qlast = j;
            S_FW:    qlast = r0;
            S_BW:    qlast = NDIM - 1 - r0;
            S_AL:    qlast = NDIM - 1;
            default: qlast = {IW{1'b0}};
        endcase
    end

    // triangular row base  tri_base(x) = x(x+1)/2
    reg  [IW-1:0]   tri_in;
    wire [2*IW:0]   tri_full = tri_in * (tri_in + 1'b1);
    wire [RAW-1:0]  tri_base = tri_full[RAW:1];
    always @(*) begin
        case (state)
            S_V:     tri_in = j;
            S_BW:    tri_in = row + q;
            default: tri_in = row;
        endcase
    end

    //----------------------------------------------------------- term (c0)
    reg            t_valid, t_diag, b_neg, b_conj;
    reg [1:0]      a_sel;
    reg [2:0]      b_sel, dst;
    reg [RAW-1:0]  t_ra, dst_addr;
    always @(*) begin
        t_valid  = 1'b0;
        t_diag   = 1'b0;
        b_neg    = 1'b0;
        b_conj   = 1'b0;
        a_sel    = A_RAM;
        b_sel    = B_M1;
        dst      = D_NONE;
        dst_addr = {RAW{1'b0}};
        t_ra     = {RAW{1'b0}};
        x_ra     = row;
        s_ra     = row;
        v_ra     = row;
        d_ra     = row;
        di_ra    = row;
        case (state)
            S_V: begin                       // v_k = L_jk * D_k
                t_valid  = (row < j);
                t_ra     = tri_base + row;
                b_sel    = B_D;
                b_neg    = 1'b1;
                dst      = D_V;
                dst_addr = row;
            end
            S_T: begin                       // t_i = R_ij - sum L_ik conj(v_k)
                t_valid  = (row < NDIM);
                if (q == {IW{1'b0}}) begin
                    t_ra  = tri_base + j;
                    b_sel = B_M1;
                end else begin
                    t_ra  = tri_base + (q - 1'b1);
                    b_sel = B_V;
                    v_ra  = q - 1'b1;
                end
                dst      = D_RAM;
                dst_addr = tri_base + j;
                t_diag   = (row == j);
            end
            S_SC: begin                      // L_ij = t_i / D_j
                t_valid  = (row < NDIM);
                t_ra     = tri_base + j;
                b_sel    = B_DI;
                b_neg    = 1'b1;
                di_ra    = j;
                dst      = D_RAM;
                dst_addr = tri_base + j;
            end
            S_FW: begin                      // z_i = s_i - sum L_ik z_k
                t_valid  = (p == {SW{1'b0}});
                if (q == {IW{1'b0}}) begin
                    a_sel = A_S;
                    b_sel = B_M1;
                end else begin
                    t_ra   = tri_base + (q - 1'b1);
                    b_sel  = B_X;
                    b_conj = 1'b1;
                    x_ra   = q - 1'b1;
                end
                dst      = D_X;
                dst_addr = row;
            end
            S_DG: begin                      // y_i = z_i / D_i
                t_valid  = (row < NDIM);
                a_sel    = A_X;
                b_sel    = B_DI;
                b_neg    = 1'b1;
                dst      = D_X;
                dst_addr = row;
            end
            S_BW: begin                      // x_i = y_i - sum conj(L_ki) x_k
                t_valid  = (p == {SW{1'b0}});
                a_sel    = A_X;
                if (q == {IW{1'b0}}) begin
                    b_sel = B_M1;
                end else begin
                    x_ra  = row + q;
                    t_ra  = tri_base + row;
                    b_sel = B_RAM;
                end
                dst      = D_X;
                dst_addr = row;
            end
            S_AL: begin                      // alpha = sum conj(s_i) x_i
                t_valid  = (p == {SW{1'b0}});
                a_sel    = A_X;
                x_ra     = q;
                b_sel    = B_S;
                s_ra     = q;
                b_neg    = 1'b1;
                dst      = D_AL;
            end
            S_NM: begin                      // w_i = x_i / alpha
                t_valid  = (row < NDIM);
                a_sel    = A_X;
                b_sel    = B_AI;
                b_neg    = 1'b1;
                dst      = D_X;
                dst_addr = row;
            end
            default: ;
        endcase
        ram_raddr = t_ra;
    end

    //----------------------------------------------------------- c1 stage
    reg            c1_valid, c1_first, c1_last, c1_bneg, c1_bconj;
    reg [SW-1:0]   c1_slot;
    reg [1:0]      c1_asel;
    reg [2:0]      c1_bsel;
    reg [TAGW-1:0] c1_tag;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            c1_valid <= 1'b0;
        else
            c1_valid <= t_valid;
    end
    always @(posedge clk) begin
        c1_first <= (q == {IW{1'b0}});
        c1_last  <= (q == qlast);
        c1_slot  <= p;
        c1_asel  <= a_sel;
        c1_bsel  <= b_sel;
        c1_bneg  <= b_neg;
        c1_bconj <= b_conj;
        c1_tag   <= {dst, t_diag, dst_addr};
    end

    reg [63:0] op_a, op_b;
    always @(*) begin
        case (c1_asel)
            A_X:     op_a = x_rd;
            A_S:     op_a = s_rd;
            default: op_a = ram_rdata;
        endcase
        case (c1_bsel)
            B_RAM:   op_b = ram_rdata;
            B_X:     op_b = x_rd;
            B_S:     op_b = s_rd;
            B_V:     op_b = v_rd;
            B_D:     op_b = {32'd0, d_rd};
            B_DI:    op_b = {32'd0, di_rd};
            B_AI:    op_b = {32'd0, ainv};
            default: op_b = {32'd0, FP_MONE};
        endcase
        if (c1_bneg)  op_b = op_b ^ {1'b1, 31'd0, 1'b1, 31'd0};
        if (c1_bconj) op_b = op_b ^ {1'b1, 63'd0};
    end

    //----------------------------------------------------------- engine
    wire            e_valid;
    wire [TAGW-1:0] e_tag;
    wire [31:0]     e_re, e_im;
    stap_cdot_engine #(.G(G), .TAGW(TAGW)) u_eng (
        .clk(clk), .rst_n(rst_n),
        .in_valid(c1_valid), .in_slot(c1_slot), .in_first(c1_first),
        .in_last(c1_last), .in_tag(c1_tag),
        .ar(op_a[31:0]), .ai(op_a[63:32]), .br(op_b[31:0]), .bi(op_b[63:32]),
        .out_valid(e_valid), .out_tag(e_tag), .out_re(e_re), .out_im(e_im),
        .busy(eng_busy)
    );

    wire [2:0]     e_dst  = e_tag[TAGW-1 -: 3];
    wire           e_diag = e_tag[RAW];
    wire [RAW-1:0] e_addr = e_tag[RAW-1:0];

    // result write back
    always @(*) begin
        ram_we    = e_valid && (e_dst == D_RAM);
        ram_waddr = e_addr;
        ram_wdata = {e_im, e_re};
    end
    always @(posedge clk) begin
        if (e_valid && e_dst == D_X) begin
            x_re[e_addr[IW-1:0]] <= e_re;
            x_im[e_addr[IW-1:0]] <= e_im;
        end
        if (e_valid && e_dst == D_V) begin
            v_re[e_addr[IW-1:0]] <= e_re;
            v_im[e_addr[IW-1:0]] <= e_im;
        end
        if (e_valid && e_diag)
            d_rf[j] <= e_re;
        if (state == S_DIVW && rcp_done)
            di_rf[j] <= rcp_y;
    end

    //----------------------------------------------------------- reciprocal
    stap_fp_recip u_rcp (
        .clk(clk), .rst_n(rst_n), .start(rcp_start),
        .x((state == S_ADIVS || state == S_ADIVW) ? alpha : dj),
        .done(rcp_done), .y(rcp_y)
    );

    //----------------------------------------------------------- FSM
    wire last_slot = (p == G - 1);
    wire grp_end   = last_slot && (q == qlast);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            nxt       <= S_IDLE;
            j         <= {IW{1'b0}};
            r0        <= {IW{1'b0}};
            q         <= {IW{1'b0}};
            p         <= {SW{1'b0}};
            busy      <= 1'b0;
            done      <= 1'b0;
            err       <= 1'b0;
            err_code  <= 2'd0;
            err_col   <= {IW{1'b0}};
            alpha     <= 32'd0;
            dj        <= 32'd0;
            ainv      <= 32'd0;
            rcp_start <= 1'b0;
        end else begin
            done      <= 1'b0;
            rcp_start <= 1'b0;

            if (e_valid && e_diag)     dj    <= e_re;
            if (e_valid && e_dst == D_AL) alpha <= e_re;

            // slot / term counters advance in every issuing state
            if (state == S_V || state == S_T || state == S_SC || state == S_FW ||
                state == S_DG || state == S_BW || state == S_AL || state == S_NM) begin
                if (last_slot) begin
                    p <= {SW{1'b0}};
                    q <= (q == qlast) ? {IW{1'b0}} : q + 1'b1;
                end else begin
                    p <= p + 1'b1;
                end
            end

            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy     <= 1'b1;
                        err      <= 1'b0;
                        err_code <= 2'd0;
                        j        <= {IW{1'b0}};
                        r0       <= {IW{1'b0}};
                        q        <= {IW{1'b0}};
                        p        <= {SW{1'b0}};
                        state    <= S_T;
                    end
                end

                S_V: if (grp_end) begin
                    if (r0 + G >= j) begin
                        r0    <= j;
                        nxt   <= S_T;
                        state <= S_DRAIN;
                    end else
                        r0 <= r0 + G;
                end

                S_T: if (grp_end) begin
                    if (r0 + G >= NDIM) begin
                        nxt   <= S_DIVS;
                        state <= S_DRAIN;
                    end else
                        r0 <= r0 + G;
                end

                S_DIVS: begin
                    if (dj[31] || dj[30:23] == 8'd0) begin
                        err      <= 1'b1;
                        err_code <= 2'd1;
                        err_col  <= j;
                        state    <= S_DONE;
                    end else begin
                        rcp_start <= 1'b1;
                        state     <= S_DIVW;
                    end
                end

                S_DIVW: if (rcp_done) begin
                    if (j == NDIM - 1) begin
                        r0    <= {IW{1'b0}};
                        state <= S_FW;
                    end else begin
                        r0    <= j + 1'b1;
                        state <= S_SC;
                    end
                end

                S_SC: if (grp_end) begin
                    if (r0 + G >= NDIM) begin
                        j     <= j + 1'b1;
                        r0    <= {IW{1'b0}};
                        nxt   <= S_V;
                        state <= S_DRAIN;
                    end else
                        r0 <= r0 + G;
                end

                S_FW: if (grp_end) begin
                    state <= S_DRAIN;
                    if (r0 == NDIM - 1) begin
                        r0  <= {IW{1'b0}};
                        nxt <= S_DG;
                    end else begin
                        r0  <= r0 + 1'b1;
                        nxt <= S_FW;
                    end
                end

                S_DG: if (grp_end) begin
                    if (r0 + G >= NDIM) begin
                        r0    <= NDIM - 1;
                        nxt   <= S_BW;
                        state <= S_DRAIN;
                    end else
                        r0 <= r0 + G;
                end

                S_BW: if (grp_end) begin
                    state <= S_DRAIN;
                    if (r0 == {IW{1'b0}}) begin
                        nxt <= S_AL;
                    end else begin
                        r0  <= r0 - 1'b1;
                        nxt <= S_BW;
                    end
                end

                S_AL: if (grp_end) begin
                    nxt   <= S_ADIVS;
                    state <= S_DRAIN;
                end

                S_ADIVS: begin
                    if (alpha[31] || alpha[30:23] == 8'd0) begin
                        err      <= 1'b1;
                        err_code <= 2'd2;
                        state    <= S_DONE;
                    end else begin
                        rcp_start <= 1'b1;
                        state     <= S_ADIVW;
                    end
                end

                S_ADIVW: if (rcp_done) begin
                    ainv  <= rcp_y;
                    r0    <= {IW{1'b0}};
                    state <= S_NM;
                end

                S_NM: if (grp_end) begin
                    if (r0 + G >= NDIM) begin
                        nxt   <= S_DONE;
                        state <= S_DRAIN;
                    end else
                        r0 <= r0 + G;
                end

                S_DRAIN: begin
                    if (!eng_busy) state <= nxt;
                end

                S_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
