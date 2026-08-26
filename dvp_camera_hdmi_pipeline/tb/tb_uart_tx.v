// ============================================================================
// tb_uart_tx.v -- checks uart_tx.v's bit framing/timing against an
// independently-written behavioral UART receiver
// ----------------------------------------------------------------------------
// Uses small overridden CLK_FREQ_HZ/BAUD (fast, but same DIV-based logic
// path as the real 50MHz/115200 config) so this runs quickly in simulation.
// ============================================================================
`timescale 1ns / 1ps

module tb_uart_tx;

    localparam CLK_PERIOD_NS = 20;      // 50MHz sim clock
    localparam CLK_FREQ_HZ   = 50_000_000;
    localparam BAUD          = 5_000_000; // DIV = 10 -> fast bit periods for sim
    localparam integer DIV   = CLK_FREQ_HZ / BAUD;
    localparam BIT_TIME_NS   = DIV * CLK_PERIOD_NS;

    reg clk = 0;
    always #(CLK_PERIOD_NS/2) clk = ~clk;

    reg rst = 1;
    reg [7:0] data = 8'h00;
    reg start = 0;
    wire tx, busy;

    uart_tx #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD(BAUD)) dut (
        .clk(clk), .rst(rst), .data(data), .start(start), .tx(tx), .busy(busy)
    );

    integer errors = 0;

    // Independent behavioral UART receiver: waits for the start bit's
    // falling edge, samples each bit at mid-period, checks the stop bit.
    task recv_and_check;
        input [7:0] expected;
        reg   [7:0] got;
        integer i;
        begin
            @(negedge tx); // start bit begins
            #(BIT_TIME_NS/2); // move to the middle of the start bit
            if (tx !== 1'b0) begin
                errors = errors + 1;
                $display("ERROR: expected start bit low at mid-bit sample");
            end
            got = 8'h00;
            for (i = 0; i < 8; i = i + 1) begin
                #(BIT_TIME_NS);
                got[i] = tx;
            end
            #(BIT_TIME_NS); // move to middle of stop bit
            if (tx !== 1'b1) begin
                errors = errors + 1;
                $display("ERROR: expected stop bit high at mid-bit sample");
            end
            if (got !== expected) begin
                errors = errors + 1;
                $display("ERROR: got byte %02h, expected %02h", got, expected);
            end else begin
                $display("OK: received byte %02h matches", got);
            end
        end
    endtask

    initial begin
        $dumpfile("tb_uart_tx.vcd");
        $dumpvars(0, tb_uart_tx);

        repeat (3) @(posedge clk);
        rst = 0;
        repeat (3) @(posedge clk);

        if (tx !== 1'b1) begin
            errors = errors + 1;
            $display("ERROR: tx not idle-high after reset");
        end
        if (busy !== 1'b0) begin
            errors = errors + 1;
            $display("ERROR: busy asserted before any transmission started");
        end

        // Send three test bytes back-to-back (each must wait for !busy).
        //
        // IMPORTANT: recv_and_check's `@(negedge tx)` must be armed BEFORE
        // `start` is pulsed, not after -- the start-bit transition happens
        // in the same simulation timestep as the clock edge that samples
        // `start`, via a non-blocking-assignment update. A sequential
        // "pulse start, THEN call recv_and_check" ordering arms the wait
        // one delta/timestep too late, silently missing that transition
        // and instead catching the NEXT negedge (partway into the byte).
        // Running the stimulus and the receiver concurrently via fork/join
        // avoids this race.
        data = 8'hA5; // 10100101 -- alternating pattern, catches bit-order bugs
        fork
            begin
                @(posedge clk); #1; start = 1;
                @(posedge clk); #1; start = 0;
            end
            recv_and_check(8'hA5);
        join
        wait (busy === 1'b0);

        data = 8'h00;
        fork
            begin
                @(posedge clk); #1; start = 1;
                @(posedge clk); #1; start = 0;
            end
            recv_and_check(8'h00);
        join
        wait (busy === 1'b0);

        data = 8'hFF;
        fork
            begin
                @(posedge clk); #1; start = 1;
                @(posedge clk); #1; start = 0;
            end
            recv_and_check(8'hFF);
        join
        wait (busy === 1'b0);

        repeat (5) @(posedge clk);
        if (tx !== 1'b1) begin
            errors = errors + 1;
            $display("ERROR: tx not idle-high after all transmissions complete");
        end

        if (errors == 0)
            $display("TB_UART_TX: PASS");
        else
            $display("TB_UART_TX: FAIL (%0d errors)", errors);

        $finish;
    end

    initial begin
        #200000;
        $display("TB_UART_TX: WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
