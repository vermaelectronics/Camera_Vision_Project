// -----------------------------------------------------------------------------
// tb_stap_top.v -- end-to-end, bit-exact testbench for stap_top (AXI4-Lite)
//
//  1. CORR_LEN = L, CORR_START, stream L+2N-2 samples         (training block)
//  2. poll STATUS.corr_done, read all 868 correlations, compare with golden
//  3. write the 112 weights computed by the C solver, OUT_SHIFT, W_COMMIT
//  4. stream K new samples, compare every filter output bit-exactly
//
// Vectors come from model/stap_golden.py (build/ directory).
// Compile with -DFOLD_VAL=<n> to test the time-multiplexed filter.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`include "tb_params.vh"

`ifndef FOLD_VAL
`define FOLD_VAL 2
`endif
`ifndef LAGP_VAL
`define LAGP_VAL 8
`endif

module tb_stap_top;
    localparam M = 7, N = 16;
    localparam DW = `TB_DW, WW = `TB_WW, OW = `TB_OW, LM = `TB_LMAX_LOG2;
    localparam FOLD = `FOLD_VAL;
    localparam LAGP = `LAGP_VAL;
    localparam NS = `TB_SL + `TB_K;

    reg clk = 0, aresetn = 0;
    always #2.5 clk = ~clk;                         // 200 MHz

    reg              s_valid = 0;
    reg [M*DW-1:0]   s_re = 0, s_im = 0;
    wire             y_valid;
    wire [OW-1:0]    y_re, y_im;
    wire             irq;

    reg  [15:0] awaddr = 0, araddr = 0;
    reg         awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
    reg  [31:0] wdata = 0;
    wire        awready, wready, bvalid, arready, rvalid;
    wire [1:0]  bresp, rresp;
    wire [31:0] rdata;

    stap_top #(.M(M), .N(N), .DW(DW), .WW(WW), .OW(OW), .FOLD(FOLD), .LMAX_LOG2(LM), .LAGP(LAGP)) dut (
        .clk(clk), .aresetn(aresetn),
        .s_valid(s_valid), .s_re(s_re), .s_im(s_im),
        .y_valid(y_valid), .y_re(y_re), .y_im(y_im), .irq(irq),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready)
    );

    // ------------------------------------------------------------------ vectors
    reg [2*M*DW-1:0] stim  [0:NS-1];
    reg [127:0]      cexp  [0:`TB_NRES-1];
    reg [63:0]       wts   [0:`TB_T-1];
    reg [2*OW-1:0]   yexp  [0:`TB_K-1];
    localparam HT = 2*N-2;
    reg [31:0]       edg   [0:2*M*HT-1];

    initial begin
        $readmemh("stim.hex",     stim);
        $readmemh("corr_exp.hex", cexp);
        $readmemh("weights.hex",  wts);
        $readmemh("yexp.hex",     yexp);
        $readmemh("edges.hex",    edg);
    end

    // ------------------------------------------------------------------ AXI tasks
    task axi_write(input [15:0] a, input [31:0] d);
        begin
            @(posedge clk);
            awaddr <= a; awvalid <= 1; wdata <= d; wvalid <= 1; bready <= 1;
            @(posedge clk);                  // valids now visible
            while (awvalid || wvalid) begin
                if (awready && awvalid) awvalid <= 0;   // handshake at this edge
                if (wready  && wvalid)  wvalid  <= 0;
                @(posedge clk);
            end
            while (!bvalid) @(posedge clk);
            @(posedge clk); bready <= 0;
        end
    endtask

    task axi_read(input [15:0] a, output [31:0] d);
        begin
            @(posedge clk);
            araddr <= a; arvalid <= 1; rready <= 1;
            @(posedge clk);
            while (!arready) @(posedge clk);
            arvalid <= 0;
            while (!rvalid) @(posedge clk);
            d = rdata;
            @(posedge clk); rready <= 0;
        end
    endtask

    // ------------------------------------------------------------------ sample driver
    task send_sample(input integer idx);           // back-to-back capable: valid for 1 clk,
        integer g;                                 // then FOLD-1 idle clocks
        begin
            @(posedge clk);
            s_valid <= 1;
            s_re    <= stim[idx][2*M*DW-1:M*DW];
            s_im    <= stim[idx][M*DW-1:0];
            for (g = 1; g < FOLD; g = g + 1) begin
                @(posedge clk);
                s_valid <= 0;
            end
        end
    endtask

    task idle(input integer n);
        integer g;
        begin
            for (g = 0; g < n; g = g + 1) begin
                @(posedge clk);
                s_valid <= 0;
            end
        end
    endtask

    // ------------------------------------------------------------------ output monitor
    integer out_cnt = 0, out_err = 0;
    reg     checking = 0;
    always @(posedge clk) begin
        if (y_valid && checking) begin
            if ({y_re, y_im} !== yexp[out_cnt]) begin
                out_err = out_err + 1;
                if (out_err <= 10)
                    $display("  OUT MISMATCH #%0d: got %h_%h exp %h", out_cnt, y_re, y_im, yexp[out_cnt]);
            end
            out_cnt = out_cnt + 1;
        end
    end

    // ------------------------------------------------------------------ test sequence
    integer i, m, p, corr_err, raw_err, t0, t1;
    reg [31:0] r0, r1, r2, r3;
    initial begin
        corr_err = 0;
        repeat (10) @(posedge clk);
        aresetn <= 1;
        repeat (5) @(posedge clk);

        axi_read(16'h0000, r0);
        axi_read(16'h0004, r1);
        axi_read(16'h0008, r2);
        axi_read(16'h000C, r3);
        $display("ID=%h CONFIG=%h CONFIG2=%h CONFIG3=%h (FOLD=%0d LAGP=%0d)", r0, r1, r2, r3, FOLD, LAGP);
        if (r0 !== 32'h53544150) begin $display("FAIL: bad ID"); $finish; end

        // ---- 1. training capture
        axi_write(16'h0018, `TB_L);
        axi_write(16'h0010, 32'h1);
        for (i = 0; i < `TB_SL; i = i + 1) send_sample(i);
        idle(1);
        t0 = $time;

        // ---- 2. wait for correlation engine
        r0 = 0;
        while (!(r0 & 32'h2)) begin
            repeat (200) @(posedge clk);
            axi_read(16'h0014, r0);
        end
        t1 = $time;
        $display("correlation engine finished: %0d clocks after capture", (t1 - t0) / 5);

        for (i = 0; i < `TB_NRES; i = i + 1) begin
            axi_read(16'h8000 + 16*i + 0,  r0);
            axi_read(16'h8000 + 16*i + 4,  r1);
            axi_read(16'h8000 + 16*i + 8,  r2);
            axi_read(16'h8000 + 16*i + 12, r3);
            if ({r1, r0, r3, r2} !== cexp[i]) begin
                corr_err = corr_err + 1;
                if (corr_err <= 10)
                    $display("  CORR MISMATCH e=%0d: got %h%h %h%h exp %h", i, r1, r0, r3, r2, cexp[i]);
            end
        end
        $display("correlation results: %0d/%0d mismatches", corr_err, `TB_NRES);

        // raw edge-sample window (used by the CPU for the exact covariance)
        raw_err = 0;
        for (p = 0; p < 2; p = p + 1) begin
            axi_write(16'h0020, p ? `TB_L : 0);
            for (m = 0; m < M; m = m + 1)
                for (i = 0; i < HT; i = i + 1) begin
                    axi_read(16'h0100 + 4*(64*m + i), r0);
                    if (r0 !== edg[p*M*HT + m*HT + i]) raw_err = raw_err + 1;
                end
        end
        $display("raw edge samples   : %0d/%0d mismatches", raw_err, 2*M*HT);

        // ---- 3. load weights
        for (i = 0; i < `TB_T; i = i + 1) begin
            axi_write(16'h1000 + 8*i,     wts[i][63:32]);
            axi_write(16'h1000 + 8*i + 4, wts[i][31:0]);
        end
        axi_write(16'h001C, `TB_SHIFT);
        axi_write(16'h0010, 32'h2);
        repeat (50) @(posedge clk);
        axi_read(16'h0014, r0);
        if (r0 & 32'h4) $display("FAIL: commit still pending");

        // ---- 4. stream out-of-sample data, check filter output
        checking = 1;
        for (i = `TB_SL; i < NS; i = i + 1) begin
            send_sample(i);
            if ((i % 13) == 0) idle(3);                         // irregular gaps
        end
        idle(100);
        axi_read(16'h0014, r0);

        $display("filter outputs     : %0d checked, %0d mismatches", out_cnt, out_err);
        $display("overrun flag       : %0d", r0[3]);
        if (corr_err == 0 && raw_err == 0 && out_err == 0 && out_cnt == `TB_K && !r0[3])
            $display("*** PASS (FOLD=%0d LAGP=%0d) ***", FOLD, LAGP);
        else
            $display("*** FAIL (FOLD=%0d LAGP=%0d) ***", FOLD, LAGP);
        $finish;
    end

    initial begin
        #2_000_000_000;
        $display("*** FAIL: timeout ***");
        $finish;
    end
endmodule
