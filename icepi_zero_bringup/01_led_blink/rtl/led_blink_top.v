// ============================================================================
// led_blink_top.v -- IcePi-Zero bring-up #1: LED blink / button-controlled
// chase, the simplest possible "does my toolchain + board + flashing work"
// smoke test.
// ----------------------------------------------------------------------------
// The board has 5 onboard status LEDs (led[4:0], real LEDs already
// populated on the board itself -- no external wiring needed) and 2
// buttons (active-low, external pull-ups per the board's own design).
//
// Default behavior: led[0] blinks at 1Hz (proves the 50MHz oscillator and
// basic synthesis/P&R/flash flow all work). button[0] (active-low) resets
// the design. Holding button[1] switches led[1..4] into a simple 4-bit
// binary counter (visually distinguishable from the 1Hz blink), so you can
// also confirm button input works without needing UART/PC tooling at all.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module led_blink_top #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BLINK_HZ    = 1
) (
    input  wire        clk,          // 50MHz board reference oscillator (site M1)
    input  wire  [1:0] button,       // active-low; button[0]=reset, button[1]=mode select
    output wire  [4:0] led           // active-high (board's own onboard LEDs)
);

    wire rst = ~button[0];

    // ---- 1Hz blink on led[0] -----------------------------------------
    localparam integer HALF_PERIOD_CYCLES = CLK_FREQ_HZ / (BLINK_HZ * 2);
    localparam integer CNT_W = $clog2(HALF_PERIOD_CYCLES);

    reg [CNT_W-1:0] blink_cnt;
    reg             blink_r;

    always @(posedge clk) begin
        if (rst) begin
            blink_cnt <= 0;
            blink_r   <= 1'b0;
        end else if (blink_cnt == HALF_PERIOD_CYCLES - 1) begin
            blink_cnt <= 0;
            blink_r   <= ~blink_r;
        end else begin
            blink_cnt <= blink_cnt + 1'b1;
        end
    end

    // ---- led[1..4]: while button[1] held, a free-running ~4Hz 4-bit
    // counter (visually distinct "chase" pattern); released, held low. ---
    localparam integer COUNT_HALF_PERIOD = CLK_FREQ_HZ / (4 * 2);
    localparam integer CCNT_W = $clog2(COUNT_HALF_PERIOD);

    reg [CCNT_W-1:0] count_cnt;
    reg [3:0]        chase_r;
    wire             mode_chase = ~button[1];

    always @(posedge clk) begin
        if (rst) begin
            count_cnt <= 0;
            chase_r   <= 4'h0;
        end else if (mode_chase) begin
            if (count_cnt == COUNT_HALF_PERIOD - 1) begin
                count_cnt <= 0;
                chase_r   <= chase_r + 1'b1;
            end else begin
                count_cnt <= count_cnt + 1'b1;
            end
        end else begin
            count_cnt <= 0; // hold at 0 so the counter always starts fresh next press
        end
    end

    assign led[0]   = blink_r;
    assign led[4:1] = mode_chase ? chase_r : 4'h0;

endmodule

`default_nettype wire
