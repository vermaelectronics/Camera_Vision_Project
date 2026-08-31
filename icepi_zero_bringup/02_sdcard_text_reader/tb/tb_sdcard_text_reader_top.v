// ============================================================================
// tb_sdcard_text_reader_top.v -- full top-level integration test: mounts the
// synthetic FAT16 volume through the real sdcard_text_reader_top (SD SPI +
// FAT16 + byte FIFO + UART TX all wired exactly as they will be on real
// hardware), auto-starts on sd_ready, and decodes the UART serial output
// bit-by-bit to confirm every byte of HELLO.TXT arrives correctly -- this
// is the one test that actually exercises the FIFO backpressure path
// (fifo_ready_for_block) that tb_sdcard_text_reader.v bypasses.
// ============================================================================
`timescale 1ns / 1ps

module tb_sdcard_text_reader_top;

    localparam integer CLK_FREQ_HZ = 50_000_000;
    localparam integer BAUD        = 2_000_000; // fast baud, testbench-only, to keep sim time down
    localparam integer BIT_NS      = 1_000_000_000 / BAUD;

    reg clk = 0;
    always #10 clk = ~clk; // 50MHz

    reg [1:0] button = 2'b11; // both released (active-low)
    wire [4:0] led;
    wire uart_txd;
    wire sd_sclk, sd_mosi, sd_miso, sd_cs_n;

    sdcard_text_reader_top #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD(BAUD), .FILENAME("HELLO   TXT")
    ) dut (
        .clk(clk), .button(button), .led(led), .uart_txd(uart_txd),
        .sd_sclk(sd_sclk), .sd_mosi(sd_mosi), .sd_miso(sd_miso), .sd_cs_n(sd_cs_n)
    );

    sd_card_model #(.NUM_BLOCKS(8)) card (
        .sclk(sd_sclk), .mosi(sd_mosi), .miso(sd_miso), .cs_n(sd_cs_n)
    );

    initial $readmemh("sd_image.hex", card.blocks);

    reg [7:0] expected [0:1023];
    integer   expected_len;
    integer   errors;
    integer   rx_i;

    initial begin
        expected_len = 600;
        $readmemh("expected_content.hex", expected, 0, expected_len - 1);
    end

    // ---- decode the UART line continuously ----
    reg [7:0] got_byte;
    reg [7:0] got_stream [0:1023];
    integer   bi;
    initial begin
        errors = 0;
        rx_i   = 0;
        forever begin
            @(negedge uart_txd); // start bit begins
            #(BIT_NS/2);
            if (uart_txd !== 1'b0) begin
                $display("ERROR: expected start bit low, got 1 (t=%0t)", $time);
                errors = errors + 1;
            end
            got_byte = 8'h00;
            for (bi = 0; bi < 8; bi = bi + 1) begin
                #(BIT_NS);
                got_byte[bi] = uart_txd;
            end
            #(BIT_NS); // stop bit
            if (uart_txd !== 1'b1) begin
                $display("ERROR: expected stop bit high, got 0 (t=%0t)", $time);
                errors = errors + 1;
            end

            if (rx_i < 1024) got_stream[rx_i] = got_byte;
            if (rx_i >= expected_len) begin
                $display("ERROR: received more bytes than expected: 0x%02x", got_byte);
                errors = errors + 1;
            end else if (got_byte !== expected[rx_i]) begin
                errors = errors + 1;
            end
            rx_i = rx_i + 1;
        end
    end

    initial begin
        $dumpfile("tb_sdcard_text_reader_top.vcd");
        $dumpvars(0, tb_sdcard_text_reader_top);

        // hold reset briefly
        button[0] = 1'b0;
        repeat (5) @(posedge clk);
        button[0] = 1'b1;

        // wait until the whole file has been decoded off the UART line
        wait (rx_i >= expected_len);
        # (BIT_NS * 12); // let the last stop bit settle

        if (led[2]) begin
            $display("ERROR: error LED is lit");
            errors = errors + 1;
        end
        if (!led[3]) begin
            $display("ERROR: done LED not lit after the read completed");
            errors = errors + 1;
        end

        $display("Decoded %0d bytes over UART", rx_i);
        begin : dump
            integer d;
            $write("GOT[200:330]: ");
            for (d = 200; d < 330 && d < rx_i; d = d + 1) $write("%c", got_stream[d]);
            $write("\n");
            $write("EXP[200:330]: ");
            for (d = 200; d < 330; d = d + 1) $write("%c", expected[d]);
            $write("\n");
        end

        if (errors == 0)
            $display("TB_SDCARD_TEXT_READER_TOP: PASS");
        else
            $display("TB_SDCARD_TEXT_READER_TOP: FAIL (%0d errors)", errors);

        $finish;
    end

    initial begin
        #20_000_000;
        $display("TB_SDCARD_TEXT_READER_TOP: WATCHDOG TIMEOUT (rx_i=%0d)", rx_i);
        begin : dump2
            integer d;
            $write("GOT[200:330]: ");
            for (d = 200; d < 330 && d < rx_i; d = d + 1) $write("%c", got_stream[d]);
            $write("\n");
            $write("EXP[200:330]: ");
            for (d = 200; d < 330; d = d + 1) $write("%c", expected[d]);
            $write("\n");
        end
        $finish;
    end

endmodule
