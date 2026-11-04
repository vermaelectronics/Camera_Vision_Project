// -----------------------------------------------------------------------------
// stap_top.v  --  STAP anti-jamming core with AXI4-Lite control interface
//
//   ADC/DDC samples ──► stap_fir  (space-time filter, weights applied every sample) ──► y
//                   └─► stap_corr (block covariance estimate, on request)
//                                    │  AXI4-Lite
//                                    ▼
//                   CPU (Zynq PS / MicroBlaze / RISC-V):  sw/stap_solver.c
//                   builds R, Cholesky solve of the MVDR / power-inversion
//                   weights, writes them back, commits.
//
// Register map (byte addresses, 32-bit registers)
//   0x0000  ID          RO  0x53544150 ("STAP")
//   0x0004  CONFIG      RO  [7:0] M  [15:8] N  [23:16] DW  [31:24] WW
//   0x0008  CONFIG2     RO  [7:0] FOLD  [15:8] OW  [23:16] CW  [31:24] LMAX_LOG2
//   0x000C  CONFIG3     RO  [7:0] LAGP (correlator lags per pass)
//   0x0010  CTRL        WO  bit0 CORR_START, bit1 W_COMMIT, bit2 CLR_OVERRUN (self clearing)
//   0x0014  STATUS      RO  bit0 corr_busy, bit1 corr_done (sticky, cleared by CORR_START),
//                           bit2 commit_pending, bit3 overrun
//   0x0018  CORR_LEN    RW  number of snapshots L
//   0x001C  OUT_SHIFT   RW  [5:0] output right shift
//   0x0020  RAW_BASE    RW  first sample index for the raw-sample window
//   0x0100 + 4*(64*m+i) RO  captured sample RAW_BASE+i of channel m (i < 64),
//                           {re[15:0], im[15:0]}  (valid when corr engine idle)
//   0x1000 + 8*t        WO  weight t real part  (WW LSBs, two's complement)
//   0x1004 + 8*t        WO  weight t imag part
//   0x8000 + 16*e       RO  correlation e real [31:0]
//   0x8004 + 16*e       RO  correlation e real [63:32] (sign extended)
//   0x8008 + 16*e       RO  correlation e imag [31:0]
//   0x800C + 16*e       RO  correlation e imag [63:32] (sign extended)
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module stap_top #(
    parameter M         = 7,
    parameter N         = 16,
    parameter DW        = 16,
    parameter WW        = 18,
    parameter OW        = 16,
    // Defaults = Zynq-7020 configuration: 120 MHz clock, 60 MSPS,
    // 168 (filter) + 24 (correlator) = 192 of 220 DSP48E1.
    parameter FOLD      = 2,
    parameter LMAX_LOG2 = 13,
    parameter LAGP      = 8
)(
    input  wire              clk,
    input  wire              aresetn,
    // sample input (M channels, complex)
    input  wire              s_valid,
    input  wire [M*DW-1:0]   s_re,
    input  wire [M*DW-1:0]   s_im,
    // filtered output
    output wire              y_valid,
    output wire [OW-1:0]     y_re,
    output wire [OW-1:0]     y_im,
    output wire              irq,          // level: corr_done
    // AXI4-Lite slave
    input  wire [15:0]       s_axi_awaddr,
    input  wire              s_axi_awvalid,
    output reg               s_axi_awready,
    input  wire [31:0]       s_axi_wdata,
    input  wire [3:0]        s_axi_wstrb,
    input  wire              s_axi_wvalid,
    output reg               s_axi_wready,
    output reg  [1:0]        s_axi_bresp,
    output reg               s_axi_bvalid,
    input  wire              s_axi_bready,
    input  wire [15:0]       s_axi_araddr,
    input  wire              s_axi_arvalid,
    output reg               s_axi_arready,
    output reg  [31:0]       s_axi_rdata,
    output reg  [1:0]        s_axi_rresp,
    output reg               s_axi_rvalid,
    input  wire              s_axi_rready
);
    localparam CW = 2*DW + 2 + LMAX_LOG2;
    localparam [7:0] P_M = M, P_N = N, P_DW = DW, P_WW = WW, P_FOLD = FOLD,
                     P_OW = OW, P_CW = CW, P_LM = LMAX_LOG2, P_LAGP = LAGP;
    wire rst = ~aresetn;

    // ------------------------------------------------------------------ registers
    reg  [LMAX_LOG2:0] corr_len;
    reg  [5:0]         out_shift;
    reg                corr_start, w_commit, corr_done;
    reg                w_we, w_sel_im;
    reg  [7:0]         w_addr;
    reg  [WW-1:0]      w_data;
    wire               corr_busy, corr_done_p, commit_pending, overrun_raw;
    reg                ovr_clr_mask;      // CTRL.CLR_OVERRUN pulse
    reg  [9:0]         res_addr;
    reg  [LMAX_LOG2-1:0] raw_base, raw_addr;
    reg  [2:0]         raw_ch;
    wire [2*DW-1:0]    raw_sample;
    wire signed [CW-1:0] res_re, res_im;

    assign irq = corr_done;

    // ------------------------------------------------------------------ cores
    stap_fir #(.M(M), .N(N), .DW(DW), .WW(WW), .OW(OW), .FOLD(FOLD)) u_fir (
        .clk(clk), .rst(rst),
        .in_valid(s_valid), .in_re(s_re), .in_im(s_im),
        .w_we(w_we), .w_addr(w_addr), .w_sel_im(w_sel_im), .w_data(w_data),
        .w_commit(w_commit), .w_commit_pending(commit_pending),
        .out_shift(out_shift),
        .out_valid(y_valid), .out_re(y_re), .out_im(y_im),
        .overrun(overrun_raw), .overrun_clr(ovr_clr_mask)
    );

    stap_corr #(.M(M), .N(N), .DW(DW), .LMAX_LOG2(LMAX_LOG2), .CW(CW), .LAGP(LAGP)) u_corr (
        .clk(clk), .rst(rst),
        .in_valid(s_valid), .in_re(s_re), .in_im(s_im),
        .start(corr_start), .len(corr_len),
        .busy(corr_busy), .done(corr_done_p),
        .res_addr(res_addr), .res_re(res_re), .res_im(res_im),
        .raw_addr(raw_addr), .raw_ch(raw_ch), .raw_sample(raw_sample)
    );

    // ------------------------------------------------------------------ AXI write channel
    reg        aw_have, w_have;
    reg [15:0] aw_addr;
    reg [31:0] w_wdata;

    always @(posedge clk) begin
        if (rst) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            aw_have       <= 1'b0;
            w_have        <= 1'b0;
            corr_len      <= 5000;
            raw_base      <= {LMAX_LOG2{1'b0}};
            out_shift     <= 6'd0;
            corr_start    <= 1'b0;
            w_commit      <= 1'b0;
            ovr_clr_mask  <= 1'b0;
            w_we          <= 1'b0;
            w_sel_im      <= 1'b0;
            w_addr        <= 8'd0;
            w_data        <= {WW{1'b0}};
            corr_done     <= 1'b0;
        end else begin
            corr_start   <= 1'b0;
            w_commit     <= 1'b0;
            ovr_clr_mask <= 1'b0;
            w_we         <= 1'b0;

            if (corr_done_p) corr_done <= 1'b1;

            s_axi_awready <= ~aw_have & ~s_axi_awready;
            s_axi_wready  <= ~w_have  & ~s_axi_wready;
            if (s_axi_awvalid & s_axi_awready) begin aw_have <= 1'b1; aw_addr <= s_axi_awaddr; s_axi_awready <= 1'b0; end
            if (s_axi_wvalid  & s_axi_wready ) begin w_have  <= 1'b1; w_wdata <= s_axi_wdata;  s_axi_wready  <= 1'b0; end

            if (aw_have & w_have & ~s_axi_bvalid) begin
                aw_have      <= 1'b0;
                w_have       <= 1'b0;
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
                if (aw_addr[15:12] == 4'h1) begin                  // weights
                    w_we     <= 1'b1;
                    w_addr   <= aw_addr[10:3];
                    w_sel_im <= aw_addr[2];
                    w_data   <= w_wdata[WW-1:0];
                end else begin
                    case (aw_addr[11:0])
                    12'h010: begin
                                 corr_start   <= w_wdata[0];
                                 w_commit     <= w_wdata[1];
                                 ovr_clr_mask <= w_wdata[2];
                                 if (w_wdata[0]) corr_done <= 1'b0;
                             end
                    12'h018: corr_len  <= w_wdata[LMAX_LOG2:0];
                    12'h01C: out_shift <= w_wdata[5:0];
                    12'h020: raw_base  <= w_wdata[LMAX_LOG2-1:0];
                    default: ;
                    endcase
                end
            end
            if (s_axi_bvalid & s_axi_bready) s_axi_bvalid <= 1'b0;
        end
    end

    // ------------------------------------------------------------------ AXI read channel
    localparam R_IDLE = 2'd0, R_W1 = 2'd1, R_W2 = 2'd2, R_RESP = 2'd3;
    reg [1:0]  rstate;
    reg [15:0] ar_addr;

    wire signed [15:0] raw_re16 = $signed(raw_sample[2*DW-1:DW]);   // DW <= 16
    wire signed [15:0] raw_im16 = $signed(raw_sample[DW-1:0]);
    wire [63:0] res_re64 = {{(64-CW){res_re[CW-1]}}, res_re};
    wire [63:0] res_im64 = {{(64-CW){res_im[CW-1]}}, res_im};

    always @(posedge clk) begin
        if (rst) begin
            rstate        <= R_IDLE;
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rdata   <= 32'd0;
            res_addr      <= 10'd0;
            ar_addr       <= 16'd0;
            raw_addr      <= {LMAX_LOG2{1'b0}};
            raw_ch        <= 3'd0;
        end else begin
            case (rstate)
            R_IDLE: begin
                s_axi_arready <= 1'b1;
                if (s_axi_arvalid & s_axi_arready) begin
                    s_axi_arready <= 1'b0;
                    ar_addr       <= s_axi_araddr;
                    res_addr      <= (s_axi_araddr - 16'h8000) >> 4;
                    raw_ch        <= s_axi_araddr[10:8] - 3'd1;       // (addr-0x100)>>8
                    raw_addr      <= raw_base + s_axi_araddr[7:2];
                    rstate        <= R_W1;
                end
            end
            R_W1: rstate <= R_W2;                  // result RAM read latency
            R_W2: begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= 2'b00;
                if (ar_addr[15]) begin
                    case (ar_addr[3:2])
                    2'd0: s_axi_rdata <= res_re64[31:0];
                    2'd1: s_axi_rdata <= res_re64[63:32];
                    2'd2: s_axi_rdata <= res_im64[31:0];
                    2'd3: s_axi_rdata <= res_im64[63:32];
                    endcase
                end else if (ar_addr[15:11] == 5'd0 && ar_addr[10:8] != 3'd0) begin
                    s_axi_rdata <= {raw_re16, raw_im16};
                end else begin
                    case (ar_addr[11:0])
                    12'h000: s_axi_rdata <= 32'h53544150;
                    12'h004: s_axi_rdata <= {P_WW, P_DW, P_N, P_M};
                    12'h008: s_axi_rdata <= {P_LM, P_CW, P_OW, P_FOLD};
                    12'h00C: s_axi_rdata <= {24'd0, P_LAGP};
                    12'h014: s_axi_rdata <= {28'd0, overrun_raw, commit_pending, corr_done, corr_busy};
                    12'h018: s_axi_rdata <= corr_len;
                    12'h01C: s_axi_rdata <= {26'd0, out_shift};
                    12'h020: s_axi_rdata <= raw_base;
                    default: s_axi_rdata <= 32'd0;
                    endcase
                end
                rstate <= R_RESP;
            end
            R_RESP: if (s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
                rstate       <= R_IDLE;
            end
            endcase
        end
    end

endmodule
