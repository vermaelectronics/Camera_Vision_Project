// ============================================================================
// tb_uart_debug.v -- checks the startup banner and one status line against
// independently-written expected strings, and exercises the frame-counter
// and NACK-counter cross-domain logic with real pulses.
// ----------------------------------------------------------------------------
// Uses small overridden CLK_FREQ_HZ/BAUD/TICK_HZ (fast, but the same logic
// path as the real 50MHz/115200/1Hz config) so this runs quickly in
// simulation.
// ============================================================================
`timescale 1ns / 1ps

module tb_uart_debug;

    localparam CLK_PERIOD_NS = 20;        // 50MHz sim clock
    localparam CLK_FREQ_HZ   = 50_000_000;
    localparam BAUD          = 5_000_000; // DIV = 10 -> fast bit periods
    localparam integer DIV   = CLK_FREQ_HZ / BAUD;
    localparam BIT_TIME_NS   = DIV * CLK_PERIOD_NS;
    localparam TICK_HZ       = 100_000;   // TICK_DIV = 500 cycles -> fast first tick

    reg clk = 0;
    always #(CLK_PERIOD_NS/2) clk = ~clk;

    reg rst = 1;
    reg pll_locked = 0, mclk_locked = 0, cam_seq_done = 0, cfg_done = 0;
    reg i2c_nack = 0, buf_ready = 0, pattern_sel = 0;
    reg cam_vsync = 0;
    reg [31:0] raw_bytes = 32'h00000000;
    wire tx;

    uart_debug #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD(BAUD), .TICK_HZ(TICK_HZ)
    ) dut (
        .clk(clk), .rst(rst),
        .pll_locked(pll_locked), .mclk_locked(mclk_locked),
        .cam_seq_done(cam_seq_done), .cfg_done(cfg_done),
        .i2c_nack(i2c_nack), .buf_ready(buf_ready), .pattern_sel(pattern_sel),
        .cam_vsync(cam_vsync), .raw_bytes(raw_bytes),
        .tx(tx)
    );

    integer errors = 0;

    // Independent behavioral UART receiver -- same technique as tb_uart_tx.v.
    task recv_byte;
        output [7:0] got;
        integer i;
        begin
            @(negedge tx);
            #(BIT_TIME_NS/2);
            got = 8'h00;
            for (i = 0; i < 8; i = i + 1) begin
                #(BIT_TIME_NS);
                got[i] = tx;
            end
            #(BIT_TIME_NS); // stop bit
        end
    endtask

    localparam BANNER_LEN = 68;
    // Independently re-typed expected banner (not read from the DUT's own
    // constant) -- if this and the DUT's copy ever disagree, the test fails.
    reg [8*BANNER_LEN-1:0] EXPECTED_BANNER =
        "\r\n=== DVP Camera->HDMI Pipeline (720p60, OV5640) -- UART Debug ===\r\n";

    localparam STATUS_LEN = 71;
    reg [8*STATUS_LEN-1:0] EXPECTED_STATUS =
        "PLL=1 MCLK=1 SEQ=1 CFG=1 NACK=1 BUF=1 MODE=C FRAMES=0x05 RAW=A5C3F02D\r\n";

    reg [7:0] got_byte;
    reg [7:0] exp_byte;
    integer i;

    // Continuously capture every byte the DUT transmits, from the moment
    // reset deasserts, into a growing buffer -- run concurrently with the
    // stimulus below via fork/join. This avoids the same class of race
    // tb_uart_tx.v hit: the DUT starts transmitting the banner within a
    // cycle or two of reset deasserting, so a receiver that only starts
    // listening *after* driving all the stimulus would miss the first
    // several banner bytes entirely, not just one bit-transition.
    localparam TOTAL_LEN = BANNER_LEN + STATUS_LEN;
    reg [7:0] rx_buf [0:TOTAL_LEN-1];
    integer   rx_count;

    task capture_forever;
        reg [7:0] b;
        begin
            rx_count = 0;
            forever begin
                recv_byte(b);
                if (rx_count < TOTAL_LEN) rx_buf[rx_count] = b;
                rx_count = rx_count + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_uart_debug.vcd");
        $dumpvars(0, tb_uart_debug);

        repeat (3) @(posedge clk);

        fork
            capture_forever;
            begin
                rst = 0;

                // Drive 5 camera VSYNC pulses early -- well before the
                // first status tick -- so the frame counter should read
                // exactly 5 (0x05) at snapshot time. cam_vsync is
                // deliberately toggled on a schedule unrelated to `clk`'s
                // period, exercising the real CDC path. (The banner alone
                // takes ~136000ns to transmit, so there's ample margin
                // for all of this setup to complete well before the first
                // status-line tick fires.)
                for (i = 0; i < 5; i = i + 1) begin
                    #530 cam_vsync = 1;
                    #470 cam_vsync = 0;
                end

                // Set the rest of the status fields to a known pattern,
                // with time to settle through the 2FF synchronizers.
                #1000;
                pll_locked   = 1;
                mclk_locked  = 1;
                cam_seq_done = 1;
                cfg_done     = 1;
                buf_ready    = 1;
                pattern_sel  = 0; // camera mode -> 'C'
                raw_bytes    = 32'hA5C3F02D;

                // One NACKed transaction: pulse i2c_nack high then low,
                // mirroring i2c_master.v's real behavior (set during a
                // transaction, cleared only when the next one starts).
                #200 i2c_nack = 1;
                #200 i2c_nack = 0;

                // Wait for the banner (68 bytes) + first status line (58
                // bytes) to be fully captured.
                wait (rx_count >= TOTAL_LEN);

                // ---- check the startup banner ----
                for (i = 0; i < BANNER_LEN; i = i + 1) begin
                    got_byte = rx_buf[i];
                    exp_byte = EXPECTED_BANNER[8*(BANNER_LEN-1-i) +: 8];
                    if (got_byte !== exp_byte) begin
                        errors = errors + 1;
                        $display("ERROR: banner byte %0d = %02h, expected %02h", i, got_byte, exp_byte);
                    end
                end
                $display("banner check done (%0d errors so far)", errors);

                // ---- check the first status line ----
                for (i = 0; i < STATUS_LEN; i = i + 1) begin
                    got_byte = rx_buf[BANNER_LEN + i];
                    exp_byte = EXPECTED_STATUS[8*(STATUS_LEN-1-i) +: 8];
                    if (got_byte !== exp_byte) begin
                        errors = errors + 1;
                        $display("ERROR: status byte %0d = %02h ('%c'), expected %02h ('%c')",
                                  i, got_byte, got_byte, exp_byte, exp_byte);
                    end
                end
                $display("status line check done (%0d errors total)", errors);

                if (errors == 0)
                    $display("TB_UART_DEBUG: PASS");
                else
                    $display("TB_UART_DEBUG: FAIL (%0d errors)", errors);

                $finish;
            end
        join
    end

    initial begin
        #5_000_000;
        $display("TB_UART_DEBUG: WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
