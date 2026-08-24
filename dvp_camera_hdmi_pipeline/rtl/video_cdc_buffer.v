// ============================================================================
// video_cdc_buffer.v -- camera-clock -> video-clock pixel stream bridge
// ----------------------------------------------------------------------------
// The camera's PCLK and the FPGA-generated HDMI pixel clock are two
// genuinely independent oscillators (no PLL relationship between them). This
// module is the ONLY place the two domains meet, and it does so through the
// generic async_fifo primitive (verified separately) -- no assumptions about
// phase or frequency relationship are made anywhere in this design.
//
// Because both sides run at (nominally) the same average pixel rate -- the
// camera is configured, via I2C, to output exactly WIDTH x HEIGHT active
// pixels per frame at the same 60 Hz the HDMI timing generator uses -- the
// FIFO fill level stays roughly constant in steady state and only needs
// modest depth to absorb short-term jitter (blanking-interval mismatches,
// clock ppm differences). Sustained frequency mismatch beyond that degrades
// gracefully rather than corrupting the picture:
//   - if the FIFO runs empty on the read (HDMI) side, the last valid pixel
//     is held (a visible line "freezes" rather than showing garbage/noise);
//   - if the FIFO fills up on the write (camera) side, new incoming pixels
//     are simply dropped (async_fifo's existing wr_full behaviour) instead
//     of overwriting/corrupting the buffer.
// Output stays blanked (black) until the buffer has pre-filled past
// PREFILL_WORDS, so the very first frame after power-up/reset never shows
// stale/garbage FIFO content.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module video_cdc_buffer #(
    parameter DEPTH         = 4096,        // FIFO depth in pixels (power of 2)
    parameter PREFILL_WORDS = 2048         // hold output blanked until this many pixels buffered
) (
    // camera (write) side
    input  wire        cam_pclk,
    input  wire         cam_rst,
    input  wire         cam_pixel_valid,
    input  wire [23:0]  cam_rgb,

    // video (read) side
    input  wire         out_pclk,
    input  wire         out_rst,
    input  wire         out_de,           // read one pixel every cycle this is high
    output wire [23:0]  out_rgb,
    output reg          out_ready         // 1 once the buffer has pre-filled
);

    wire [$clog2(DEPTH):0] wr_level;
    wire                    fifo_full;
    wire                    fifo_empty;

    async_fifo #(
        .WIDTH(24),
        .DEPTH(DEPTH)
    ) u_fifo (
        .wr_clk   (cam_pclk),
        .wr_rst   (cam_rst),
        .wr_en    (cam_pixel_valid),
        .wr_data  (cam_rgb),
        .wr_full  (fifo_full),
        .wr_level (wr_level),

        .rd_clk   (out_pclk),
        .rd_rst   (out_rst),
        .rd_en    (out_de & out_ready & ~fifo_empty),
        .rd_data  (out_rgb),
        .rd_empty (fifo_empty)
    );

    // Pre-fill gate, evaluated in the write-clock domain (wr_level is a
    // write-clock-domain estimate as documented in async_fifo.v) and
    // synchronized into the read-clock domain with a simple 2-flop
    // synchronizer, since it only needs to be a clean single-bit level, not
    // a bus (no multi-bit CDC hazard).
    wire prefilled = (wr_level >= PREFILL_WORDS[$clog2(DEPTH):0]);
    reg  prefilled_s1, prefilled_s2;

    always @(posedge out_pclk) begin
        if (out_rst) begin
            prefilled_s1 <= 1'b0;
            prefilled_s2 <= 1'b0;
            out_ready    <= 1'b0;
        end else begin
            prefilled_s1 <= prefilled;
            prefilled_s2 <= prefilled_s1;
            // Latching (sticky) once ready: once the pipeline is primed we
            // do not want brief dips below PREFILL_WORDS to re-blank the
            // picture -- only a genuine empty condition should freeze a line.
            out_ready    <= out_ready | prefilled_s2;
        end
    end

endmodule

`default_nettype wire
