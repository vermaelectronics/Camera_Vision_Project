// ============================================================================
// dvp_capture.v -- generic parallel (DVP) camera bus front end
// ----------------------------------------------------------------------------
// Samples a standard 8-bit DVP camera interface (PCLK / HREF / VSYNC / D[7:0],
// as found on OV7670/OV2640/OV5647-parallel/OV5640-DVP/GC0308-class sensors)
// entirely in the sensor's own PCLK clock domain and reduces it to a simple
// byte stream with line/frame boundary markers. No clock-domain crossing
// happens in this module -- PCLK IS this module's clock, exactly as the DVP
// protocol requires (all of HREF/VSYNC/D[] are specified relative to PCLK by
// the sensor itself).
//
// HREF_POL / VSYNC_POL let you match whichever active polarity your sensor
// datasheet specifies (this varies between sensor families).
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module dvp_capture #(
    parameter HREF_POL  = 1'b1,   // 1 = HREF/HSYNC line-valid is active-high
    parameter VSYNC_POL = 1'b1    // 1 = VSYNC frame-valid is active-high
) (
    input  wire       pclk,        // camera pixel clock (this module's clock)
    input  wire        rst_async,   // async system reset, synchronized internally
    input  wire        href,        // line valid (raw from sensor)
    input  wire        vsync,       // frame valid (raw from sensor)
    input  wire [7:0]  d,           // 8-bit parallel data (raw from sensor)

    output reg          byte_valid,  // one pulse per captured byte
    output reg  [7:0]   byte_data,
    output reg          line_start,  // pulses on the first captured byte of a line
    output reg          frame_start  // pulses on the first captured byte of a frame
);

    // ---- reset synchronizer (async assert, sync de-assert, into pclk) ----
    reg [1:0] rst_sync = 2'b11;
    always @(posedge pclk or posedge rst_async)
        if (rst_async) rst_sync <= 2'b11;
        else           rst_sync <= {rst_sync[0], 1'b0};
    wire rst = rst_sync[1];

    // ---- normalize polarities --------------------------------------------
    wire href_active  = HREF_POL  ? href  : ~href;
    wire vsync_active = VSYNC_POL ? vsync : ~vsync;

    // ---- edge detection (all in the pclk domain -- no CDC needed) --------
    reg href_d, vsync_d, vsync_active_d;

    always @(posedge pclk) begin
        if (rst) begin
            href_d         <= 1'b0;
            vsync_d        <= 1'b0;
            vsync_active_d <= 1'b0;
        end else begin
            href_d         <= href_active;
            vsync_d        <= vsync_active;
            vsync_active_d <= vsync_active;
        end
    end

    wire href_rising  = href_active  && !href_d;
    // A new frame begins on the first line (HREF rising) that follows a
    // completed VSYNC pulse -- i.e. once vsync_active has been seen and has
    // since gone low again, catch the next href_rising.
    reg vsync_seen;
    always @(posedge pclk) begin
        if (rst) begin
            vsync_seen <= 1'b0;
        end else begin
            if (vsync_active)
                vsync_seen <= 1'b1;
            else if (href_rising && vsync_seen)
                vsync_seen <= 1'b0; // consumed -> arms for the next frame
        end
    end

    wire frame_start_pulse = href_rising && vsync_seen;

    always @(posedge pclk) begin
        if (rst) begin
            byte_valid  <= 1'b0;
            byte_data   <= 8'h00;
            line_start  <= 1'b0;
            frame_start <= 1'b0;
        end else begin
            byte_valid  <= href_active;
            byte_data   <= d;
            line_start  <= href_rising;
            frame_start <= frame_start_pulse;
        end
    end

endmodule

`default_nettype wire
