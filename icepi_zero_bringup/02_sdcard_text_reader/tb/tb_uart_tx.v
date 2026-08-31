// ============================================================================
// tb_uart_tx.v -- decodes uart_tx's serial output bit-by-bit and checks it
// against the bytes sent in, including start/stop bit framing.
// ============================================================================
`timescale 1ns / 1ps

module tb_uart_tx;

    localparam integer CLK_FREQ_HZ = 1_000_000; // scaled down for fast sim
    localparam integer BAUD        = 100_000;
    localparam integer BIT_NS      = 1_000_000_000 / BAUD;

    reg clk = 0;
    always #500 clk = ~clk; // 1MHz

    reg rst = 1;
    reg [7:0] data;
    reg       valid;
    wire      ready;
    wire      tx;

    uart_tx #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD(BAUD)) dut (
        .clk(clk), .rst(rst), .data(data), .valid(valid), .ready(ready), .tx(tx)
    );

    integer errors = 0;

    task send_and_check(input [7:0] b);
        integer i;
        reg [7:0] got;
        begin
            @(posedge clk);
            while (!ready) @(posedge clk);
            data  = b;
            valid = 1'b1;
            @(posedge clk);
            valid = 1'b0;

            // start bit
            wait (tx === 1'b0);
            #(BIT_NS/2); // sample mid-bit
            if (tx !== 1'b0) begin
                $display("ERROR: start bit not 0"); errors = errors + 1;
            end

            got = 8'h00;
            for (i = 0; i < 8; i = i + 1) begin
                #(BIT_NS);
                got[i] = tx;
            end

            #(BIT_NS); // stop bit
            if (tx !== 1'b1) begin
                $display("ERROR: stop bit not 1"); errors = errors + 1;
            end

            if (got !== b) begin
                $display("ERROR: sent 0x%02x, decoded 0x%02x", b, got);
                errors = errors + 1;
            end else begin
                $display("OK: sent/decoded 0x%02x", b);
            end
        end
    endtask

    initial begin
        $dumpfile("tb_uart_tx.vcd");
        $dumpvars(0, tb_uart_tx);

        data = 8'h00; valid = 1'b0;
        repeat (5) @(posedge clk);
        rst = 0;
        @(posedge clk);

        if (tx !== 1'b1) begin
            $display("ERROR: tx not idle-high after reset"); errors = errors + 1;
        end

        send_and_check(8'h55);
        send_and_check(8'h00);
        send_and_check(8'hFF);
        send_and_check(8'hA5);

        if (errors == 0)
            $display("TB_UART_TX: PASS");
        else
            $display("TB_UART_TX: FAIL (%0d errors)", errors);

        $finish;
    end

    initial begin
        #2_000_000;
        $display("TB_UART_TX: WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
