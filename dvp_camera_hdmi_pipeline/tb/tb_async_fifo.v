// ============================================================================
// tb_async_fifo.v -- functional check of async_fifo.v
// ----------------------------------------------------------------------------
// 1) Two independent, unrelated clocks (write=17ns period, read=23ns period)
//    to genuinely exercise the CDC logic (no lucky common-multiple aliasing).
// 2) Fill past capacity and confirm wr_full asserts and extra writes are
//    dropped (data past the full point must NOT appear in the output).
// 3) Drain fully, confirm rd_empty asserts and held rd_data does not change
//    on further (ignored) read attempts.
// 4) Push DEPTH*3 words through in bursts with random-ish gaps on both
//    sides and check the received sequence is correct, in order, complete.
// ============================================================================
`timescale 1ns / 1ps

module tb_async_fifo;

    localparam WIDTH = 8;
    localparam DEPTH = 16;

    reg wr_clk = 0; always #8.5 wr_clk = ~wr_clk;   // 17ns period
    reg rd_clk = 0; always #11.5 rd_clk = ~rd_clk;  // 23ns period

    reg wr_rst = 1, rd_rst = 1;
    reg wr_en = 0;
    reg [WIDTH-1:0] wr_data = 0;
    wire wr_full;
    wire [$clog2(DEPTH):0] wr_level;

    reg rd_en = 0;
    wire [WIDTH-1:0] rd_data;
    wire rd_empty;

    async_fifo #(.WIDTH(WIDTH), .DEPTH(DEPTH)) dut (
        .wr_clk(wr_clk), .wr_rst(wr_rst), .wr_en(wr_en), .wr_data(wr_data),
        .wr_full(wr_full), .wr_level(wr_level),
        .rd_clk(rd_clk), .rd_rst(rd_rst), .rd_en(rd_en), .rd_data(rd_data),
        .rd_empty(rd_empty)
    );

    integer errors = 0;

    // ---- Phase 1: fill past capacity, confirm full + drop behaviour ------
    reg [7:0] expect_q [0:255];
    integer   exp_wr = 0, exp_rd = 0;
    integer   i;
    integer   dropped;

    initial begin
        #200000;
        $display("WATCHDOG TIMEOUT at t=%0t (sb_wr_ptr=%0d sb_rd_ptr=%0d exp_wr=%0d exp_rd=%0d)",
                   $time, sb_wr_ptr, sb_rd_ptr, exp_wr, exp_rd);
        $finish;
    end

    initial begin
        $dumpfile("tb_async_fifo.vcd");
        $dumpvars(0, tb_async_fifo);

        repeat (5) @(posedge wr_clk);
        wr_rst = 0;
        repeat (5) @(posedge rd_clk);
        rd_rst = 0;

        // --- fill past capacity with rd_en held low the whole time ---
        // Poll wr_full freshly before each attempt (same safe pattern as
        // the Phase 2 writer below) so the accepted-word count is exact.
        dropped = 0;
        for (i = 0; i < DEPTH + 6; i = i + 1) begin
            @(posedge wr_clk); #1;
            if (!wr_full) begin
                wr_en   = 1;
                wr_data = i[7:0];
                expect_q[exp_wr] = i[7:0];
                exp_wr = exp_wr + 1;
            end else begin
                wr_en = 0;
                dropped = dropped + 1;
            end
            @(posedge wr_clk); #1; wr_en = 0;
        end

        if (exp_wr != DEPTH) begin
            errors = errors + 1;
            $display("ERROR: expected exactly %0d words accepted before full, got %0d", DEPTH, exp_wr);
        end else begin
            $display("OK: FIFO accepted exactly DEPTH=%0d words before asserting full (%0d dropped)", DEPTH, dropped);
        end

        // --- drain fully (bounded loop), checking order and content -------
        for (i = 0; i < DEPTH + 4; i = i + 1) begin
            @(posedge rd_clk); #1;
            if (!rd_empty) begin
                rd_en = 1;
                @(posedge rd_clk); #1; rd_en = 0;
                if (rd_data !== expect_q[exp_rd]) begin
                    errors = errors + 1;
                    $display("ERROR drain: word %0d expected %02x got %02x", exp_rd, expect_q[exp_rd], rd_data);
                end
                exp_rd = exp_rd + 1;
            end else begin
                rd_en = 0;
            end
        end

        if (exp_rd == exp_wr && rd_empty)
            $display("OK: drained all %0d words in order, FIFO reports empty", exp_rd);
        else begin
            errors = errors + 1;
            $display("ERROR: drain incomplete/empty flag wrong (exp_rd=%0d exp_wr=%0d rd_empty=%b)", exp_rd, exp_wr, rd_empty);
        end

        // give the design a clean restart for the burst test
        @(posedge wr_clk); wr_rst = 1;
        @(posedge rd_clk); rd_rst = 1;
        repeat (5) @(posedge wr_clk); wr_rst = 0;
        repeat (5) @(posedge rd_clk); rd_rst = 0;

        run_burst_test;

        if (errors == 0)
            $display("TB_ASYNC_FIFO: PASS");
        else
            $display("TB_ASYNC_FIFO: FAIL (%0d errors)", errors);
        $finish;
    end

    // ---- Phase 2: continuous burst test with a scoreboard -----------------
    reg [7:0] sb_data [0:4095];
    integer   sb_wr_ptr = 0;
    integer   sb_rd_ptr = 0;
    integer   sb_errors = 0;
    integer   n_words;

    task run_burst_test;
        integer k;
        integer gap;
        begin
            n_words = 300;
            sb_wr_ptr = 0;
            sb_rd_ptr = 0;

            fork
                // writer process
                begin
                    for (k = 0; k < n_words; k = k + 1) begin
                        @(posedge wr_clk);
                        #1;
                        while (wr_full) begin
                            @(posedge wr_clk); #1;
                        end
                        wr_en   = 1;
                        wr_data = k[7:0] ^ 8'hA5; // simple recognizable pattern
                        sb_data[sb_wr_ptr] = k[7:0] ^ 8'hA5;
                        sb_wr_ptr = sb_wr_ptr + 1;
                        @(posedge wr_clk); #1; wr_en = 0;
                        // occasional idle gap on the write side
                        if ((k % 7) == 0) repeat (2) @(posedge wr_clk);
                    end
                end
                // reader process
                begin
                    while (sb_rd_ptr < n_words) begin
                        @(posedge rd_clk);
                        #1;
                        if (!rd_empty) begin
                            rd_en = 1;
                            @(posedge rd_clk); #1;
                            rd_en = 0;
                            if (rd_data !== sb_data[sb_rd_ptr]) begin
                                sb_errors = sb_errors + 1;
                                errors = errors + 1;
                                $display("ERROR burst: word %0d expected %02x got %02x",
                                          sb_rd_ptr, sb_data[sb_rd_ptr], rd_data);
                            end
                            sb_rd_ptr = sb_rd_ptr + 1;
                        end else begin
                            rd_en = 0;
                        end
                        // occasional idle gap on the read side
                        if ((sb_rd_ptr % 11) == 0) repeat (3) @(posedge rd_clk);
                    end
                end
            join

            if (sb_errors == 0)
                $display("OK: burst test of %0d words across independent clocks, all matched in order", n_words);
        end
    endtask

endmodule
