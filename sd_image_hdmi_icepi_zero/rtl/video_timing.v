`timescale 1ns/1ps
// video_timing : VESA DMT 640x480@60Hz timing generator, pix_clk domain.
//
// Standard timing (VESA DMT 0x04, "640x480@60"):
//   H: active=640  front=16  sync=96  back=48   total=800
//   V: active=480  front=10  sync=2   back=33   total=525
//   HSYNC/VSYNC polarity: both negative (active-low)
//
// The nominal pixel clock for this mode is 25.175 MHz; this design uses
// a clean 25.000 MHz (see icepi_clk_wiz_video.v) for exact-integer PLL
// dividers. That is a 0.7% low pixel clock, giving ~59.5 Hz refresh
// instead of 60.0 Hz - within the tolerance essentially all HDMI/DVI
// sinks accept for this mode (many cheap "640x480@60" sources already
// run at similarly rounded clocks).
//
// x/y are only meaningful while de=1; both hold their last active-video
// value during blanking (harmless, since downstream logic must gate on
// de anyway).
module video_timing (
    input  wire        pix_clk,
    input  wire        rst,

    output reg          hsync,   // active-low
    output reg          vsync,   // active-low
    output reg          de,      // 1 during active video
    output reg  [9:0]   x,       // 0..639 while de=1
    output reg  [9:0]   y        // 0..479 while de=1
);
    localparam H_ACTIVE = 10'd640;
    localparam H_FRONT  = 10'd16;
    localparam H_SYNC   = 10'd96;
    localparam H_BACK   = 10'd48;
    localparam H_TOTAL  = H_ACTIVE + H_FRONT + H_SYNC + H_BACK; // 800

    localparam V_ACTIVE = 10'd480;
    localparam V_FRONT  = 10'd10;
    localparam V_SYNC   = 10'd2;
    localparam V_BACK   = 10'd33;
    localparam V_TOTAL  = V_ACTIVE + V_FRONT + V_SYNC + V_BACK; // 525

    reg [9:0] hcnt;
    reg [9:0] vcnt;

    wire h_sync_active = (hcnt >= H_ACTIVE + H_FRONT) && (hcnt < H_ACTIVE + H_FRONT + H_SYNC);
    wire v_sync_active = (vcnt >= V_ACTIVE + V_FRONT) && (vcnt < V_ACTIVE + V_FRONT + V_SYNC);

    always @(posedge pix_clk) begin
        if (rst) begin
            hcnt  <= 10'd0;
            vcnt  <= 10'd0;
            hsync <= 1'b1;
            vsync <= 1'b1;
            de    <= 1'b0;
            x     <= 10'd0;
            y     <= 10'd0;
        end else begin
            if (hcnt == H_TOTAL - 1) begin
                hcnt <= 10'd0;
                vcnt <= (vcnt == V_TOTAL - 1) ? 10'd0 : vcnt + 10'd1;
            end else begin
                hcnt <= hcnt + 10'd1;
            end

            hsync <= ~h_sync_active;
            vsync <= ~v_sync_active;
            de    <= (hcnt < H_ACTIVE) && (vcnt < V_ACTIVE);

            if (hcnt < H_ACTIVE) x <= hcnt;
            if (vcnt < V_ACTIVE) y <= vcnt;
        end
    end

endmodule
