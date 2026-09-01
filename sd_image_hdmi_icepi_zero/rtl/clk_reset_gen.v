`timescale 1ns/1ps
// clk_reset_gen : clean reset for the 20 MHz sys_clk domain.
//
// Holds rst_sys asserted asynchronously whenever the PLL is not locked
// (or the external reset button is pressed), and releases it
// synchronously to sys_clk through a multi-stage shift register once
// both conditions clear, so every sys_clk flop in the design comes out
// of reset on the same edge with no recovery/removal violation.
module clk_reset_gen (
    input  wire clk,        // 20 MHz sys_clk (PLL CLKOP)
    input  wire pll_locked, // PLL LOCK, asynchronous
    input  wire ext_rst_n,  // external reset request, asynchronous, active-low
    output reg  rst_sys,    // synchronous active-high reset, sys_clk domain
    output wire rst_sys_n
);
    localparam SYNC_STAGES = 4;

    reg [SYNC_STAGES-1:0] sync_chain;
    wire async_rst_n = pll_locked & ext_rst_n;

    always @(posedge clk or negedge async_rst_n) begin
        if (!async_rst_n) begin
            sync_chain <= {SYNC_STAGES{1'b0}};
            rst_sys    <= 1'b1;
        end else begin
            sync_chain <= {sync_chain[SYNC_STAGES-2:0], 1'b1};
            rst_sys    <= ~sync_chain[SYNC_STAGES-1];
        end
    end

    assign rst_sys_n = ~rst_sys;

endmodule
