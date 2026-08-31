// ============================================================================
// tb_led_blink.v -- checks the blink period is exact and button[1] correctly
// switches led[4:1] between "held at 0" and "counting."
// ============================================================================
`timescale 1ns / 1ps

module tb_led_blink;

    // Small scaled-down clock/blink rate so this runs quickly in simulation.
    localparam integer CLK_FREQ_HZ = 1000; // 1kHz "clock" -- just scales the math
    localparam integer BLINK_HZ    = 10;   // 10Hz blink at this scaled clock
    localparam CLK_PERIOD_NS = 1000; // arbitrary time unit, 1000ns = one "cycle"

    reg clk = 0;
    always #(CLK_PERIOD_NS/2) clk = ~clk;

    reg [1:0] button = 2'b11; // both released (active-low)
    wire [4:0] led;

    led_blink_top #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BLINK_HZ(BLINK_HZ)) dut (
        .clk(clk), .button(button), .led(led)
    );

    integer errors = 0;
    integer half_period_cycles = CLK_FREQ_HZ / (BLINK_HZ * 2); // = 50

    integer i;
    integer waited;
    reg last_led0;

    initial begin
        $dumpfile("tb_led_blink.vcd");
        $dumpvars(0, tb_led_blink);

        // ---- hold reset (button[0]=0) briefly ----
        button[0] = 0;
        repeat (5) @(posedge clk);
        button[0] = 1;
        @(posedge clk);

        if (led[0] !== 1'b0) begin
            errors = errors + 1;
            $display("ERROR: led[0] not 0 immediately after reset release");
        end
        if (led[4:1] !== 4'h0) begin
            errors = errors + 1;
            $display("ERROR: led[4:1] not 0 immediately after reset (button[1] released)");
        end

        // ---- check the blink period is exactly half_period_cycles ----
        last_led0 = led[0];
        for (i = 0; i < 4; i = i + 1) begin
            waited = 0;
            while (led[0] === last_led0 && waited <= half_period_cycles + 5) begin
                @(posedge clk);
                waited = waited + 1;
            end
            if (led[0] === last_led0) begin
                errors = errors + 1;
                $display("ERROR: led[0] failed to toggle within expected window (waited %0d)", waited);
            end else if (waited !== half_period_cycles) begin
                errors = errors + 1;
                $display("ERROR: led[0] toggled after %0d cycles, expected %0d", waited, half_period_cycles);
            end
            last_led0 = led[0];
        end

        // ---- button[1] held: led[4:1] should start counting ----
        button[1] = 0;
        repeat (3) @(posedge clk);
        if (led[4:1] === 4'h0) begin
            // could coincidentally still be 0 this early; wait a bit more and recheck trend
        end
        repeat (200) @(posedge clk); // several count periods at this scaled rate
        if (led[4:1] === 4'h0) begin
            errors = errors + 1;
            $display("ERROR: led[4:1] never counted while button[1] held");
        end

        // ---- button[1] released: led[4:1] should return to 0 ----
        button[1] = 1;
        repeat (3) @(posedge clk);
        if (led[4:1] !== 4'h0) begin
            errors = errors + 1;
            $display("ERROR: led[4:1] not held at 0 after button[1] released");
        end

        if (errors == 0)
            $display("TB_LED_BLINK: PASS");
        else
            $display("TB_LED_BLINK: FAIL (%0d errors)", errors);

        $finish;
    end

    initial begin
        #2_000_000;
        $display("TB_LED_BLINK: WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
