// -----------------------------------------------------------------------------
// stap_corr.v  --  Space-time covariance estimator (the "R = Xm*Xm'/L" part)
//
// Captures a block of SL = L + 2(N-1) samples of all M channels into BRAM and
// then computes, for every channel pair (a <= b) and every lag
// tau = -(N-1) .. +(N-1):
//
//   c_ab[tau] = sum_{l=0}^{L-1}  x_a[l + (N-1) + tau] * conj( x_b[l + (N-1)] )
//
// The exact MN x MN covariance matrix of STAP_constraint.m,
//
//   R[(a,k),(b,k')] = 1/L * sum_{l=0}^{L-1} x_a[l+k] conj(x_b[l+k'])     (a <= b)
//
// differs from c_ab[k-k'] only by <= N-1 products at each end of the block:
//
//   L*R = c_ab[k-k'] + sum_{j=k'}^{N-2}   x_a[j+k-k'] conj(x_b[j])
//                    - sum_{j=k'+L}^{N-2+L} x_a[j+k-k'] conj(x_b[j])
//
// The CPU reads those 2*(2N-2) edge samples per channel back through the raw
// sample port (raw_addr/raw_ch) and applies the correction, so it gets the
// exact (positive definite) R from only NP*NL = 868 hardware results instead
// of 112*112 = 12544.
//
// Result entry index:  e = pair*NL + q,  q = tau + (N-1),
// pairs enumerated as  for a in 0..M-1: for b in a..M-1.
//
// Architecture: one pass per channel pair, NL complex MACs (4*NL multipliers)
// working in parallel on a 2N-1 deep shift register -> one snapshot per clock.
// Cycles per update ~= NP * (L + 2N-2 + NL + 8)   (28*5099 = 143k for L=5000,
// i.e. 0.7 ms at 200 MHz).  The filter keeps running with the old weights
// while this runs.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module stap_corr #(
    parameter M         = 7,
    parameter N         = 16,
    parameter DW        = 16,
    parameter LMAX_LOG2 = 13,                          // capture buffer depth 2^LMAX_LOG2
    parameter CW        = 2*DW + 2 + LMAX_LOG2         // result width per component
)(
    input  wire                  clk,
    input  wire                  rst,
    // sample stream (same bus as the filter)
    input  wire                  in_valid,
    input  wire [M*DW-1:0]       in_re,
    input  wire [M*DW-1:0]       in_im,
    // control
    input  wire                  start,      // capture next L+2N-2 samples, then compute
    input  wire [LMAX_LOG2:0]    len,        // L, number of snapshots (clamped)
    output wire                  busy,
    output reg                   done,       // 1-clock pulse when results are ready
    // result read port (1 clock latency)
    input  wire [9:0]            res_addr,
    output reg  signed [CW-1:0]  res_re,
    output reg  signed [CW-1:0]  res_im,
    // raw capture-buffer read port, valid in IDLE (1 clock latency)
    input  wire [LMAX_LOG2-1:0]  raw_addr,
    input  wire [2:0]            raw_ch,
    output wire [2*DW-1:0]       raw_sample   // {re, im}
);
    localparam NL   = 2*N - 1;
    localparam NP   = M*(M+1)/2;
    localparam NRES = NP*NL;
    localparam AWID = LMAX_LOG2;
    localparam DEPTH= 1 << LMAX_LOG2;
    localparam PW   = 2*DW;           // real product width
    localparam SW   = 2*DW + 1;       // complex product component width
    localparam [LMAX_LOG2:0] MAXL = DEPTH - (2*N - 2);

    localparam S_IDLE = 3'd0, S_CAP = 3'd1, S_RUN = 3'd2, S_DRAIN = 3'd3, S_DUMP = 3'd4;

    reg  [2:0]      state;
    reg  [AWID:0]   sl;              // stream length L + 2N-2
    reg  [AWID:0]   cnt;             // capture address / stream index
    reg  [2:0]      ch_a, ch_b;
    reg  [9:0]      res_base;
    reg  [4:0]      dcnt;            // drain counter
    reg  [5:0]      qcnt;            // dump counter

    assign busy = (state != S_IDLE);

    integer i, q;

    // ------------------------------------------------------------------ sample buffers
    wire             cap_we = (state == S_CAP) & in_valid;
    wire [AWID-1:0]  addr_a = (state == S_IDLE) ? raw_addr : cnt[AWID-1:0];
    wire [AWID-1:0]  addr_b = cnt[AWID-1:0] - (N-1);   // only used when cnt >= N-1
    wire [M*PW-1:0]  qa_flat, qb_flat;

    genvar g;
    generate
        for (g = 0; g < M; g = g + 1) begin : g_buf
            reg [PW-1:0] mem [0:DEPTH-1];
            reg [PW-1:0] qa, qb;
            always @(posedge clk) begin                 // port A: write / read
                if (cap_we) mem[addr_a] <= {in_re[g*DW +: DW], in_im[g*DW +: DW]};
                qa <= mem[addr_a];
            end
            always @(posedge clk) begin                 // port B: read
                qb <= mem[addr_b];
            end
            assign qa_flat[g*PW +: PW] = qa;
            assign qb_flat[g*PW +: PW] = qb;
        end
    endgenerate

    assign raw_sample = qa_flat[raw_ch*PW +: PW];

    // ------------------------------------------------------------------ control FSM
    reg run_issue;                                       // a read is issued this cycle
    always @(*) run_issue = (state == S_RUN);

    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            cnt      <= 0;
            sl       <= 0;
            ch_a     <= 0;
            ch_b     <= 0;
            res_base <= 0;
            dcnt     <= 0;
            qcnt     <= 0;
            done     <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
            S_IDLE: if (start) begin
                        sl    <= ((len > MAXL) ? MAXL : (len == 0) ? 1 : len) + (2*N - 2);
                        cnt   <= 0;
                        state <= S_CAP;
                    end
            S_CAP:  if (in_valid) begin
                        if (cnt == sl - 1) begin
                            cnt      <= 0;
                            ch_a     <= 0;
                            ch_b     <= 0;
                            res_base <= 0;
                            state    <= S_RUN;
                        end else cnt <= cnt + 1'b1;
                    end
            S_RUN:  if (cnt == sl - 1) begin
                        dcnt  <= 0;
                        state <= S_DRAIN;
                    end else cnt <= cnt + 1'b1;
            S_DRAIN: if (dcnt == 5'd8) begin
                        qcnt  <= 0;
                        state <= S_DUMP;
                    end else dcnt <= dcnt + 1'b1;
            S_DUMP: if (qcnt == NL-1) begin
                        res_base <= res_base + NL;
                        cnt      <= 0;
                        if (ch_b == M-1) begin
                            if (ch_a == M-1) begin
                                state <= S_IDLE;
                                done  <= 1'b1;
                            end else begin
                                ch_a  <= ch_a + 1'b1;
                                ch_b  <= ch_a + 1'b1;
                                state <= S_RUN;
                            end
                        end else begin
                            ch_b  <= ch_b + 1'b1;
                            state <= S_RUN;
                        end
                    end else qcnt <= qcnt + 1'b1;
            default: state <= S_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------ data pipeline
    // s1: RAM output valid   s2: channel mux   s3: shift register   s4: products
    // s5: complex sums       s6: accumulators
    reg              v1, v2, en3, en4, en5;
    reg  [AWID:0]    i1, i2;
    reg  signed [DW-1:0] xa_re, xa_im, xb_re, xb_im;
    reg  signed [DW-1:0] sr_re [0:NL-1];
    reg  signed [DW-1:0] sr_im [0:NL-1];
    reg  signed [DW-1:0] rf_re, rf_im;

    always @(posedge clk) begin
        if (rst) begin
            v1 <= 1'b0; v2 <= 1'b0; en3 <= 1'b0; en4 <= 1'b0; en5 <= 1'b0;
        end else begin
            v1  <= run_issue;
            i1  <= cnt;
            v2  <= v1;
            i2  <= i1;
            xa_re <= qa_flat[ch_a*PW + DW +: DW];
            xa_im <= qa_flat[ch_a*PW      +: DW];
            xb_re <= qb_flat[ch_b*PW + DW +: DW];
            xb_im <= qb_flat[ch_b*PW      +: DW];
            en3 <= v2 & (i2 >= (2*N - 2));
            en4 <= en3;
            en5 <= en4;
        end
    end

    always @(posedge clk) begin
        if (v2) begin
            sr_re[0] <= xa_re;
            sr_im[0] <= xa_im;
            for (i = 1; i < NL; i = i + 1) begin
                sr_re[i] <= sr_re[i-1];
                sr_im[i] <= sr_im[i-1];
            end
            rf_re <= xb_re;
            rf_im <= xb_im;
        end
    end

    // NL parallel complex multipliers: lag q uses sr[NL-1-q] * conj(ref)
    wire [NL*SW-1:0] s_re_flat, s_im_flat;
    generate
        for (g = 0; g < NL; g = g + 1) begin : g_lag
            reg signed [PW-1:0] p_rr, p_ii, p_ir, p_ri;
            reg signed [SW-1:0] c_re, c_im;
            always @(posedge clk) begin
                p_rr <= sr_re[NL-1-g] * rf_re;
                p_ii <= sr_im[NL-1-g] * rf_im;
                p_ir <= sr_im[NL-1-g] * rf_re;
                p_ri <= sr_re[NL-1-g] * rf_im;
                c_re <= p_rr + p_ii;                 // Re{a * conj(b)}
                c_im <= p_ir - p_ri;                 // Im{a * conj(b)}
            end
            assign s_re_flat[g*SW +: SW] = c_re;
            assign s_im_flat[g*SW +: SW] = c_im;
        end
    endgenerate

    // accumulators: accumulate during RUN/DRAIN, shift out (zero fill) during DUMP
    reg signed [CW-1:0] acc_re [0:NL-1];
    reg signed [CW-1:0] acc_im [0:NL-1];
    wire dump = (state == S_DUMP);

    always @(posedge clk) begin
        if (rst || (state == S_IDLE)) begin
            for (q = 0; q < NL; q = q + 1) begin
                acc_re[q] <= {CW{1'b0}};
                acc_im[q] <= {CW{1'b0}};
            end
        end else if (dump) begin
            for (q = 0; q < NL-1; q = q + 1) begin
                acc_re[q] <= acc_re[q+1];
                acc_im[q] <= acc_im[q+1];
            end
            acc_re[NL-1] <= {CW{1'b0}};
            acc_im[NL-1] <= {CW{1'b0}};
        end else if (en5) begin
            for (q = 0; q < NL; q = q + 1) begin
                acc_re[q] <= acc_re[q] + $signed(s_re_flat[q*SW +: SW]);
                acc_im[q] <= acc_im[q] + $signed(s_im_flat[q*SW +: SW]);
            end
        end
    end

    // ------------------------------------------------------------------ result RAM
    reg [2*CW-1:0] res_mem [0:1023];
    always @(posedge clk) begin
        if (dump) res_mem[res_base + qcnt] <= {acc_re[0], acc_im[0]};
        {res_re, res_im} <= res_mem[res_addr];
    end

endmodule
