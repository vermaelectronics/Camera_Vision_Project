// Testbench for gnss_pulse_blank / gnss_peak_hold: reads blank_in.hex (one 32-bit word
// {q[15:0], i[15:0]} per sample), drives it with a strobe every 2nd clock, writes blank_out.hex
// and the counters. Compared with Tests/rtl/blank_model.py by Tests/rtl/run_blank_rtl.sh.
`timescale 1ns/100ps
module tb_pulse_blank;
    parameter NS = 20000;
    reg clk = 0, rst_n = 0, clr = 0, en = 1;
    reg in_valid = 0;
    reg [15:0] in_i = 0, in_q = 0;
    reg [31:0] thr;
    wire out_valid; wire [15:0] out_i, out_q; wire [31:0] blk_cnt, burst_cnt;
    reg [31:0] mem [0:NS-1];
    integer n, fo, nout;
    wire [15:0] peak, pidx;
    gnss_pulse_blank dut(.clk(clk), .rst_n(rst_n), .clr(clr), .in_valid(in_valid), .in_i(in_i), .in_q(in_q),
        .en(en), .thr(thr), .out_valid(out_valid), .out_i(out_i), .out_q(out_q), .blk_cnt(blk_cnt), .burst_cnt(burst_cnt));
    gnss_peak_hold #(.WIN_BITS(10)) ph(.clk(clk), .rst_n(rst_n), .valid(in_valid), .a0(in_i), .b0(in_q), .a1(16'd0), .b1(16'd0), .peak_win(peak), .win_idx(pidx));
    always #4 clk = ~clk;
    initial begin
        if (!$value$plusargs("thr=%d", thr)) thr = 32'd20000;
        if (!$value$plusargs("en=%d", n)) n = 1;
        en = n[0];
        $readmemh("blank_in.hex", mem);
        fo = $fopen("blank_out.hex", "w");
        nout = 0;
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);
        for (n = 0; n < NS + 40; n = n + 1) begin
            @(posedge clk);
            in_valid <= (n < NS);
            if (n < NS) begin in_i <= mem[n][15:0]; in_q <= mem[n][31:16]; end
            @(posedge clk);
            in_valid <= 0;
        end
        repeat (10) @(posedge clk);
        $fclose(fo);
        $display("BLK_CNT %0d BURSTS %0d NOUT %0d PEAK %0d WIN %0d", blk_cnt, burst_cnt, nout, peak, pidx);
        $finish;
    end
    always @(posedge clk) if (out_valid) begin
        $fwrite(fo, "%08x\n", {out_q, out_i});
        nout = nout + 1;
    end
endmodule
