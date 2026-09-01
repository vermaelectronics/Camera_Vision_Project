`timescale 1ns/1ps
// framebuffer : 160x120 RGB565 true dual-port, dual-clock block RAM.
//
// Sized to fit entirely in the LFE5U-25F's on-chip EBR (56 x 18Kbit =
// ~1008Kbit total): 160*120*16 = 307,200 bits (~300Kbit), leaving ample
// margin for the rest of the design. This is why the display resolution
// is 160x120 rather than a typical HDMI resolution directly - there is
// no external SDRAM in this design, so the whole frame must fit on-chip.
// hdmi_out.v upscales it to 640x480 by 4x nearest-neighbor pixel
// replication (160*4=640, 120*4=480 - an exact integer scale, so no
// filtering/interpolation logic is needed).
//
// Write port: sys_clk domain, driven by img_loader while it streams the
// image in from the SD card (one-time load, not a per-frame refresh).
// Read port: pix_clk domain, driven by hdmi_out's scan-out address
// every pixel clock. No read/write collision handling is implemented
// because none is needed: hdmi_out only starts reading once
// img_loader's `loaded` flag (synchronized into pix_clk domain) is set,
// by which point all writes have already finished.
module framebuffer (
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [14:0] wr_addr,   // 0..19199
    input  wire [15:0] wr_data,

    input  wire        rd_clk,
    input  wire [14:0] rd_addr,   // 0..19199
    output reg  [15:0] rd_data
);
    reg [15:0] mem [0:19199];

    always @(posedge wr_clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    always @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
    end

endmodule
