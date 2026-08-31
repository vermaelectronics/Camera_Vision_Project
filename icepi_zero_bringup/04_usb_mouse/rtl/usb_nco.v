// ============================================================================
// usb_nco.v -- fractional-N bit-rate tick generator (a simple DDA/Bresenham
// accumulator, not a PLL) for low-speed USB's 1.5MHz bit rate from a 50MHz
// reference. 50,000,000 / 1,500,000 = 33.33.., not an integer divide, so a
// fixed clock-cycle counter would drift; this accumulator produces ticks at
// exactly the right *average* rate with at most one 20ns system-clock
// period of jitter on any single tick -- comfortably inside low-speed
// USB's timing tolerance.
// ----------------------------------------------------------------------------
// `resync` is for the RX side: pulsing it re-arms the accumulator so the
// *next* tick lands roughly half a bit period later, i.e. at the center of
// the bit that just started -- used to re-lock bit timing on every line
// transition seen (see usb_packet_rx.v), which bounds any drift between
// resyncs to at most the 7-bit-period gap bit-stuffing guarantees.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module usb_nco #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BIT_RATE_HZ = 1_500_000
) (
    input  wire clk,
    input  wire rst,
    input  wire resync,
    output reg  tick
);

    localparam integer STEP = BIT_RATE_HZ;
    localparam integer WRAP = CLK_FREQ_HZ;

    reg [31:0] acc;

    always @(posedge clk) begin
        tick <= 1'b0;
        if (rst) begin
            acc <= 32'd0;
        end else if (resync) begin
            acc  <= WRAP / 2; // next tick lands ~half a bit period from now
            tick <= 1'b0;
        end else if (acc + STEP >= WRAP) begin
            acc  <= acc + STEP - WRAP;
            tick <= 1'b1;
        end else begin
            acc <= acc + STEP;
        end
    end

endmodule

`default_nettype wire
