//-----------------------------------------------------------------------------
// stap_cov_est : space-time sample covariance estimator
//
//   R = (1/L) * sum_l x(l) x(l)^H  + DL * I                (Rx = Xm*Xm'/L)
//
//   x(l) is the M*N space-time snapshot, element i = m*N + t holds
//   y_m(l + t)  (channel m, tap t), exactly as built by the circshift loop
//   in STAP_constraint.m.
//
//   The captured block (L + N - 1 samples of all M channels) is streamed
//   once per (mi, mj, ti) triple. Two N-deep sliding windows (channel mi and
//   channel mj) feed N complex integer MACs in parallel, producing the N
//   entries R(mi*N+ti, mj*N+0..N-1) per pass. Only the lower triangle is
//   written, as fp32, with the 1/L scaling folded into the exponent.
//
//   Passes : M(M+1)/2 * N        cycles/pass ~ L + 2N + 6
//-----------------------------------------------------------------------------
module stap_cov_est #(
    parameter integer M     = 7,
    parameter integer N     = 16,
    parameter integer DW    = 16,
    parameter integer LOG2L = 12,
    // derived - do not override
    parameter integer CAW   = $clog2((1 << LOG2L) + N - 1),
    parameter integer RAW   = $clog2(M * N * (M * N + 1) / 2)
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 start,
    output reg                  busy,
    output reg                  done,
    input  wire [31:0]          diag_load,     // added to R(i,i), covariance units
    // capture buffer read port (1-cycle latency)
    output wire [CAW-1:0]       cap_raddr,
    input  wire [M*2*DW-1:0]    cap_rdata,
    // covariance RAM write port
    output reg                  ram_we,
    output reg  [RAW-1:0]       ram_waddr,
    output wire [63:0]          ram_wdata
);

    localparam integer L    = 1 << LOG2L;
    localparam integer CAPD = L + N - 1;
    localparam integer NDIM = M * N;
    localparam integer IW   = $clog2(NDIM + 1);
    localparam integer MW   = (M > 1) ? $clog2(M) : 1;
    localparam integer TW   = (N > 1) ? $clog2(N) : 1;
    localparam integer ACCW = 2 * DW + LOG2L + 3;
    localparam integer PW   = 2 * DW;

    localparam [2:0] S_IDLE = 3'd0, S_RUN = 3'd1, S_DRAIN = 3'd2,
                     S_WB   = 3'd3, S_NEXT = 3'd4;

    reg [2:0]      state;
    reg [MW-1:0]   mi, mj;
    reg [TW-1:0]   ti;
    reg [CAW:0]    a;               // read address counter
    reg [3:0]      dcnt;
    reg [TW:0]     wb;              // write back index

    //------------------------------------------------------- read / windows
    reg                  rd_v;
    reg [CAW:0]          rd_a;
    reg signed [DW-1:0]  wi_re [0:N-1];
    reg signed [DW-1:0]  wi_im [0:N-1];
    reg signed [DW-1:0]  wj_re [0:N-1];
    reg signed [DW-1:0]  wj_im [0:N-1];
    reg                  mac_v1, mac_v2, clr;

    assign cap_raddr = a[CAW-1:0];

    wire [2*DW-1:0] smp_i = cap_rdata[mi*2*DW +: 2*DW];
    wire [2*DW-1:0] smp_j = cap_rdata[mj*2*DW +: 2*DW];

    integer t;
    always @(posedge clk) begin
        if (rd_v) begin
            for (t = 0; t < N - 1; t = t + 1) begin
                wi_re[t] <= wi_re[t+1];
                wi_im[t] <= wi_im[t+1];
                wj_re[t] <= wj_re[t+1];
                wj_im[t] <= wj_im[t+1];
            end
            wi_re[N-1] <= smp_i[DW-1:0];
            wi_im[N-1] <= smp_i[2*DW-1:DW];
            wj_re[N-1] <= smp_j[DW-1:0];
            wj_im[N-1] <= smp_j[2*DW-1:DW];
        end
    end

    //------------------------------------------------------- N parallel MACs
    reg signed [PW-1:0]   p_rr [0:N-1];
    reg signed [PW-1:0]   p_ii [0:N-1];
    reg signed [PW-1:0]   p_ir [0:N-1];
    reg signed [PW-1:0]   p_ri [0:N-1];
    reg signed [ACCW-1:0] acc_re [0:N-1];
    reg signed [ACCW-1:0] acc_im [0:N-1];

    wire signed [DW-1:0] xr = wi_re[ti];
    wire signed [DW-1:0] xi = wi_im[ti];

    // x_i * conj(x_j) = (xr*yr + xi*yi) + j(xi*yr - xr*yi)
    always @(posedge clk) begin
        for (t = 0; t < N; t = t + 1) begin
            p_rr[t] <= xr * wj_re[t];
            p_ii[t] <= xi * wj_im[t];
            p_ir[t] <= xi * wj_re[t];
            p_ri[t] <= xr * wj_im[t];
        end
        for (t = 0; t < N; t = t + 1) begin
            if (clr) begin
                acc_re[t] <= {ACCW{1'b0}};
                acc_im[t] <= {ACCW{1'b0}};
            end else if (mac_v2) begin
                acc_re[t] <= acc_re[t] + p_rr[t] + p_ii[t];
                acc_im[t] <= acc_im[t] + p_ir[t] - p_ri[t];
            end
        end
    end

    //------------------------------------------------------- write back
    wire [IW-1:0]   row_i  = mi * N + ti;
    wire [IW-1:0]   col_j  = mj * N + wb[TW-1:0];
    wire [2*IW:0]   tri_f  = row_i * (row_i + 1'b1);
    wire [RAW-1:0]  waddr  = tri_f[RAW:1] + col_j;
    wire            wr_ok  = (state == S_WB) && (wb < N) && (row_i >= col_j);
    wire            is_dg  = (row_i == col_j);

    wire [ACCW-1:0]        dl_ext = diag_load << LOG2L;
    wire signed [ACCW-1:0] wb_re  = acc_re[wb[TW-1:0]] + (is_dg ? dl_ext : {ACCW{1'b0}});
    wire signed [ACCW-1:0] wb_im  = acc_im[wb[TW-1:0]];

    wire [31:0] f_re, f_im;
    stap_int2fp #(.W(ACCW), .EXP_OFS(-LOG2L)) u_cre (.clk(clk), .x(wb_re), .y(f_re));
    stap_int2fp #(.W(ACCW), .EXP_OFS(-LOG2L)) u_cim (.clk(clk), .x(wb_im), .y(f_im));
    assign ram_wdata = {f_im, f_re};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ram_we    <= 1'b0;
            ram_waddr <= {RAW{1'b0}};
        end else begin
            ram_we    <= wr_ok;
            ram_waddr <= waddr;
        end
    end

    //------------------------------------------------------- control
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            busy      <= 1'b0;
            done      <= 1'b0;
            mi        <= {MW{1'b0}};
            mj        <= {MW{1'b0}};
            ti        <= {TW{1'b0}};
            a         <= {(CAW+1){1'b0}};
            rd_v      <= 1'b0;
            rd_a      <= {(CAW+1){1'b0}};
            mac_v1    <= 1'b0;
            mac_v2    <= 1'b0;
            clr       <= 1'b0;
            dcnt      <= 4'd0;
            wb        <= {(TW+1){1'b0}};
        end else begin
            done   <= 1'b0;
            clr    <= 1'b0;
            rd_v   <= 1'b0;
            mac_v1 <= rd_v && (rd_a >= N - 1);
            mac_v2 <= mac_v1;

            case (state)
                S_IDLE: if (start) begin
                    busy  <= 1'b1;
                    mi    <= {MW{1'b0}};
                    mj    <= {MW{1'b0}};
                    ti    <= {TW{1'b0}};
                    a     <= {(CAW+1){1'b0}};
                    clr   <= 1'b1;
                    state <= S_RUN;
                end

                S_RUN: begin
                    rd_v      <= 1'b1;
                    rd_a      <= a;
                    if (a == CAPD - 1) begin
                        dcnt  <= 4'd0;
                        state <= S_DRAIN;
                    end
                    a <= a + 1'b1;
                end

                S_DRAIN: begin                 // read -> window -> product -> acc
                    dcnt <= dcnt + 1'b1;
                    if (dcnt == 4'd4) begin
                        wb    <= {(TW+1){1'b0}};
                        state <= S_WB;
                    end
                end

                S_WB: begin
                    wb <= wb + 1'b1;
                    if (wb == N)
                        state <= S_NEXT;
                end

                S_NEXT: begin
                    a     <= {(CAW+1){1'b0}};
                    clr   <= 1'b1;
                    state <= S_RUN;
                    if (ti != N - 1) begin
                        ti <= ti + 1'b1;
                    end else begin
                        ti <= {TW{1'b0}};
                        if (mj != mi) begin
                            mj <= mj + 1'b1;
                        end else begin
                            mj <= {MW{1'b0}};
                            if (mi != M - 1) begin
                                mi <= mi + 1'b1;
                            end else begin
                                busy  <= 1'b0;
                                done  <= 1'b1;
                                state <= S_IDLE;
                            end
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
