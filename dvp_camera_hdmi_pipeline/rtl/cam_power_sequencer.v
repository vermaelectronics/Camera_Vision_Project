// ============================================================================
// cam_power_sequencer.v -- camera PWDN/RESET power-up sequencing
// ----------------------------------------------------------------------------
// Many DVP camera modules (including the Waveshare OV5640 board this
// project targets) break PWDN and RESET out as pins the host must actively
// drive -- there's no onboard "just works" default. This module implements
// the standard, generously-timed sequence used across most sensor app
// notes/reference designs:
//
//   1. Hold PWDN high (powered DOWN) and RESET low (asserted) until MCLK
//      is confirmed running (mclk_locked).
//   2. Once MCLK is stable: drive PWDN low (powered UP), keep RESET
//      asserted for RST_CYCLES more clocks (>= ~1ms of margin).
//   3. Release RESET (drive high), then wait SETTLE_CYCLES more clocks
//      (>= ~20ms of margin) for internal regulators/PLL to stabilize
//      before I2C communication is attempted.
//   4. Assert `seq_done` (stays high) -- safe to start cam_config_rom.
//
// Default cycle counts assume a 24MHz clock (this project's MCLK) -- see
// the CLK_FREQ_HZ parameter if you drive this from a different clock.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module cam_power_sequencer #(
    parameter integer CLK_FREQ_HZ  = 24_000_000,
    parameter integer RST_MS       = 2,    // reset pulse width, in ms (datasheet minimums are usually << 1ms; this is a generous margin)
    parameter integer SETTLE_MS    = 20    // post-reset settle time, in ms, before I2C starts
) (
    input  wire       clk,          // camera MCLK domain
    input  wire        mclk_locked,  // from clk_gen_mclk
    input  wire        rst_async,    // system reset (async assert)

    output reg          cam_pwdn,     // to sensor PWDN pin (active high = powered down)
    output reg          cam_rst_n,    // to sensor RESET pin (active low)
    output reg          seq_done      // safe to start I2C configuration
);

    localparam integer RST_CYCLES    = (CLK_FREQ_HZ / 1000) * RST_MS;
    localparam integer SETTLE_CYCLES = (CLK_FREQ_HZ / 1000) * SETTLE_MS;
    localparam integer CNT_W = $clog2(SETTLE_CYCLES > RST_CYCLES ? SETTLE_CYCLES : RST_CYCLES) + 1;

    localparam S_WAIT_MCLK = 0,
               S_RESET     = 1,
               S_SETTLE    = 2,
               S_DONE      = 3;

    reg [1:0]        state;
    reg [CNT_W-1:0]  cnt;

    // reset synchronizer into this (mclk) domain
    reg [1:0] rst_sr = 2'b11;
    always @(posedge clk or posedge rst_async)
        if (rst_async) rst_sr <= 2'b11;
        else           rst_sr <= {rst_sr[0], 1'b0};
    wire rst = rst_sr[1];

    always @(posedge clk) begin
        if (rst) begin
            state     <= S_WAIT_MCLK;
            cnt       <= 0;
            cam_pwdn  <= 1'b1; // powered down
            cam_rst_n <= 1'b0; // asserted
            seq_done  <= 1'b0;
        end else begin
            case (state)
                S_WAIT_MCLK: begin
                    cam_pwdn  <= 1'b1;
                    cam_rst_n <= 1'b0;
                    if (mclk_locked) begin
                        cnt   <= 0;
                        state <= S_RESET;
                    end
                end

                S_RESET: begin
                    cam_pwdn  <= 1'b0; // power up
                    cam_rst_n <= 1'b0; // keep reset asserted
                    if (cnt == RST_CYCLES - 1) begin
                        cnt   <= 0;
                        state <= S_SETTLE;
                    end else begin
                        cnt <= cnt + 1;
                    end
                end

                S_SETTLE: begin
                    cam_rst_n <= 1'b1; // release reset
                    if (cnt == SETTLE_CYCLES - 1) begin
                        state <= S_DONE;
                    end else begin
                        cnt <= cnt + 1;
                    end
                end

                S_DONE: begin
                    seq_done <= 1'b1;
                end

                default: state <= S_WAIT_MCLK;
            endcase
        end
    end

endmodule

`default_nettype wire
