//-----------------------------------------------------------------------------
// stap_top : Space-Time Adaptive Processing (STAP) anti-jamming IP
//
//   RTL implementation of STAP_constraint.m / STAP_unconstraint.m
//
//     x(n)  M-element array x N-tap space-time snapshot
//     R     = (1/L) sum x x^H + DL*I                 (stap_cov_est)
//     w     = R^-1 s / (s^H R^-1 s)                  (stap_solver)
//             s = steering vector (MVDR)  or  e1 (power inversion)
//     y(n)  = w^H x(n)                               (stap_beamformer)
//
//   Adaptation is block based (sample matrix inversion):
//     CAPTURE (L+N-1 samples) -> COVARIANCE -> SOLVE -> LOAD WEIGHTS
//   while the beamformer keeps filtering the live stream with the previous
//   weight set (double buffered).
//
//   Control/status through an APB3 slave, see README.md for the map.
//-----------------------------------------------------------------------------
module stap_top #(
    parameter integer M     = 7,        // antenna elements (RecNum)
    parameter integer N     = 16,       // taps per element (orders)
    parameter integer DW    = 16,       // input I/Q width
    parameter integer LOG2L = 12,       // snapshots per estimate L = 2^LOG2L
    parameter integer WW    = 18,       // beamformer weight width
    parameter integer WFRAC = 16,       // weight fraction bits
    parameter integer OW    = 16,       // output I/Q width
    parameter integer G     = 4         // solver interleave factor
) (
    input  wire                clk,
    input  wire                rst_n,
    // APB3 slave
    input  wire                psel,
    input  wire                penable,
    input  wire                pwrite,
    input  wire [15:0]         paddr,
    input  wire [31:0]         pwdata,
    output reg  [31:0]         prdata,
    output wire                pready,
    output wire                pslverr,
    // array sample stream, channel m = s_data[m*2*DW +: 2*DW] = {Q, I}
    input  wire                s_valid,
    input  wire [M*2*DW-1:0]   s_data,
    // adapted output stream {Q, I}
    output wire                m_valid,
    output wire [2*OW-1:0]     m_data,
    // interrupt (weights updated / error)
    output wire                irq
);

    localparam integer NDIM = M * N;
    localparam integer NTRI = NDIM * (NDIM + 1) / 2;
    localparam integer RAW  = $clog2(NTRI);
    localparam integer IW   = $clog2(NDIM + 1);
    localparam integer SIW  = $clog2(NDIM + G + 1);       // solver index width
    localparam integer L    = 1 << LOG2L;
    localparam integer CAPD = L + N - 1;
    localparam integer CAW  = $clog2(CAPD);
    localparam [7:0]   VERSION = 8'h10;

    localparam [2:0] C_IDLE = 3'd0, C_CAP = 3'd1, C_COV = 3'd2,
                     C_SOL  = 3'd3, C_LOAD = 3'd4;

    //=========================================================================
    // registers
    //=========================================================================
    reg         r_cont, r_mvdr, r_irq_en;
    reg  [31:0] r_diag;
    reg  [5:0]  r_oshift;
    reg  [31:0] r_updates, r_cycles, r_cyc_cnt;
    reg  [1:0]  r_irq_st;
    reg         r_wvalid;
    reg         start_req;

    wire        apb_wr = psel & penable & pwrite;
    wire        apb_rd = psel & ~pwrite;
    wire        win_s  = (paddr[15:12] == 4'h1);
    wire        win_w  = (paddr[15:12] == 4'h2);
    wire [IW-1:0] widx = paddr[IW+2:3];

    // weight readback needs one wait state (synchronous register file)
    reg         rd_wait;
    assign pready  = ~(psel & penable & ~pwrite & win_w & ~rd_wait);
    assign pslverr = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rd_wait <= 1'b0;
        else        rd_wait <= psel & penable & ~pwrite & win_w & ~rd_wait;
    end

    //=========================================================================
    // sub-blocks
    //=========================================================================
    reg  [2:0]  cstate;

    // capture buffer
    reg  [CAW:0]       cap_cnt;
    wire               cap_we = (cstate == C_CAP) && s_valid;
    wire [CAW-1:0]     cap_raddr;
    wire [M*2*DW-1:0]  cap_rdata;
    stap_sdp_ram #(.DW(M*2*DW), .DEPTH(CAPD)) u_cap (
        .clk(clk), .we(cap_we), .waddr(cap_cnt[CAW-1:0]), .wdata(s_data),
        .raddr(cap_raddr), .rdata(cap_rdata)
    );

    // covariance estimator
    reg              cov_start;
    wire             cov_busy, cov_done, cov_we;
    wire [RAW-1:0]   cov_waddr;
    wire [63:0]      cov_wdata;
    stap_cov_est #(.M(M), .N(N), .DW(DW), .LOG2L(LOG2L)) u_cov (
        .clk(clk), .rst_n(rst_n), .start(cov_start), .busy(cov_busy),
        .done(cov_done), .diag_load(r_diag),
        .cap_raddr(cap_raddr), .cap_rdata(cap_rdata),
        .ram_we(cov_we), .ram_waddr(cov_waddr), .ram_wdata(cov_wdata)
    );

    // covariance / LDL RAM
    wire             sol_we;
    wire [RAW-1:0]   sol_waddr, sol_raddr;
    wire [63:0]      sol_wdata, ram_rdata;
    wire             ram_we    = (cstate == C_COV) ? cov_we    : sol_we;
    wire [RAW-1:0]   ram_waddr = (cstate == C_COV) ? cov_waddr : sol_waddr;
    wire [63:0]      ram_wdata = (cstate == C_COV) ? cov_wdata : sol_wdata;
    stap_sdp_ram #(.DW(64), .DEPTH(NTRI)) u_rram (
        .clk(clk), .we(ram_we), .waddr(ram_waddr), .wdata(ram_wdata),
        .raddr(sol_raddr), .rdata(ram_rdata)
    );

    // solver
    reg              sol_start;
    wire             sol_busy, sol_done, sol_err;
    wire [1:0]       sol_err_code;
    wire [SIW-1:0]   sol_err_col;
    wire [31:0]      sol_alpha;
    wire [SIW-1:0]   x_raddr;
    wire [63:0]      x_rdata;
    wire             s_we_re = apb_wr & win_s & ~paddr[2];
    wire             s_we_im = apb_wr & win_s &  paddr[2];
    stap_solver #(.NDIM(NDIM), .G(G)) u_sol (
        .clk(clk), .rst_n(rst_n), .start(sol_start), .mode_mvdr(r_mvdr),
        .busy(sol_busy), .done(sol_done), .err(sol_err), .err_code(sol_err_code),
        .err_col(sol_err_col), .alpha(sol_alpha),
        .ram_raddr(sol_raddr), .ram_rdata(ram_rdata),
        .ram_we(sol_we), .ram_waddr(sol_waddr), .ram_wdata(sol_wdata),
        .s_we_re(s_we_re), .s_we_im(s_we_im), .s_waddr(widx), .s_wdata(pwdata),
        .x_raddr(x_raddr), .x_rdata(x_rdata)
    );

    // weight loader
    reg              wl_start;
    wire             wl_busy, wl_done;
    wire [IW-1:0]    wl_raddr;
    wire             w_we, w_swap;
    wire [IW-1:0]    w_addr;
    wire [WW-1:0]    w_re, w_im;
    wire signed [9:0] wexp;
    stap_wload #(.NDIM(NDIM), .WW(WW), .WFRAC(WFRAC)) u_wl (
        .clk(clk), .rst_n(rst_n), .start(wl_start), .busy(wl_busy), .done(wl_done),
        .x_raddr(wl_raddr), .x_rdata(x_rdata),
        .w_we(w_we), .w_addr(w_addr), .w_re(w_re), .w_im(w_im),
        .w_swap(w_swap), .wexp(wexp)
    );
    assign x_raddr = (cstate == C_LOAD) ? wl_raddr : widx;

    // adaptive filter
    stap_beamformer #(.M(M), .N(N), .DW(DW), .WW(WW), .WFRAC(WFRAC), .OW(OW)) u_bf (
        .clk(clk), .rst_n(rst_n), .s_valid(s_valid), .s_data(s_data),
        .m_valid(m_valid), .m_data(m_data), .out_shift(r_oshift),
        .w_we(w_we), .w_addr(w_addr), .w_re(w_re), .w_im(w_im), .w_swap(w_swap)
    );

    //=========================================================================
    // adaptation controller
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cstate    <= C_IDLE;
            cap_cnt   <= {(CAW+1){1'b0}};
            cov_start <= 1'b0;
            sol_start <= 1'b0;
            wl_start  <= 1'b0;
            r_updates <= 32'd0;
            r_cycles  <= 32'd0;
            r_cyc_cnt <= 32'd0;
            r_wvalid  <= 1'b0;
            r_irq_st  <= 2'b00;
        end else begin
            cov_start <= 1'b0;
            sol_start <= 1'b0;
            wl_start  <= 1'b0;
            if (cstate != C_IDLE) r_cyc_cnt <= r_cyc_cnt + 1'b1;

            // IRQ status, write-one-to-clear
            if (apb_wr && !win_s && !win_w && paddr[11:0] == 12'h020)
                r_irq_st <= r_irq_st & ~pwdata[1:0];

            case (cstate)
                C_IDLE: if (start_req || r_cont) begin
                    cap_cnt   <= {(CAW+1){1'b0}};
                    r_cyc_cnt <= 32'd0;
                    cstate    <= C_CAP;
                end

                C_CAP: if (s_valid) begin
                    cap_cnt <= cap_cnt + 1'b1;
                    if (cap_cnt == CAPD - 1) begin
                        cov_start <= 1'b1;
                        cstate    <= C_COV;
                    end
                end

                C_COV: if (cov_done) begin
                    sol_start <= 1'b1;
                    cstate    <= C_SOL;
                end

                C_SOL: if (sol_done) begin
                    if (sol_err) begin
                        r_irq_st[1] <= 1'b1;
                        r_cycles    <= r_cyc_cnt;
                        cstate      <= C_IDLE;
                    end else begin
                        wl_start <= 1'b1;
                        cstate   <= C_LOAD;
                    end
                end

                C_LOAD: if (wl_done) begin
                    r_updates   <= r_updates + 1'b1;
                    r_cycles    <= r_cyc_cnt;
                    r_wvalid    <= 1'b1;
                    r_irq_st[0] <= 1'b1;
                    cstate      <= C_IDLE;
                end

                default: cstate <= C_IDLE;
            endcase
        end
    end

    assign irq = r_irq_en & (|r_irq_st);

    //=========================================================================
    // APB register file
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_cont    <= 1'b0;
            r_mvdr    <= 1'b0;
            r_irq_en  <= 1'b0;
            r_diag    <= 32'd0;
            r_oshift  <= WFRAC;
            start_req <= 1'b0;
        end else begin
            if (cstate != C_IDLE) start_req <= 1'b0;
            if (apb_wr && !win_s && !win_w) begin
                case (paddr[11:0])
                    12'h000: begin
                        if (pwdata[0]) start_req <= 1'b1;
                        r_cont   <= pwdata[1];
                        r_mvdr   <= pwdata[2];
                        r_irq_en <= pwdata[8];
                    end
                    12'h008: r_diag   <= pwdata;
                    12'h00C: r_oshift <= pwdata[5:0];
                    default: ;
                endcase
            end
        end
    end

    always @(*) begin
        prdata = 32'd0;
        if (apb_rd) begin
            if (win_w) begin
                prdata = paddr[2] ? x_rdata[63:32] : x_rdata[31:0];
            end else if (!win_s) begin
                case (paddr[11:0])
                    12'h000: prdata = {23'd0, r_irq_en, 5'd0, r_mvdr, r_cont, 1'b0};
                    12'h004: prdata = {{(16-SIW){1'b0}}, sol_err_col, 5'd0, cstate,
                                       2'd0, sol_err_code, 1'b0, sol_err, r_wvalid,
                                       (cstate != C_IDLE)};
                    12'h008: prdata = r_diag;
                    12'h00C: prdata = {26'd0, r_oshift};
                    12'h010: prdata = r_updates;
                    12'h014: prdata = {{22{wexp[9]}}, wexp};
                    12'h018: prdata = sol_alpha;
                    12'h01C: prdata = {VERSION, LOG2L[7:0], N[7:0], M[7:0]};
                    12'h020: prdata = {30'd0, r_irq_st};
                    12'h024: prdata = r_cycles;
                    default: prdata = 32'd0;
                endcase
            end
        end
    end

endmodule
