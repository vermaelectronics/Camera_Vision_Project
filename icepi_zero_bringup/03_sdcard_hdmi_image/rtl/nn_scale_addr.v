// ============================================================================
// nn_scale_addr.v -- nearest-neighbor upscale address generator: converts a
// 1280x720 output pixel position (x,y) into the corresponding source-image
// pixel position (small_x, small_y) for a fixed-size source image.
// ----------------------------------------------------------------------------
// Horizontal: X_SHIFT is chosen so 2**X_SHIFT == 1280/IMG_W exactly (a plain
// bit-slice, no logic at all). Vertical: 720/IMG_H isn't generally a power
// of 2 (it isn't for the 160x120 default: 720/120 = 6), so small_y is
// tracked with an explicit sub-row counter that advances once every
// Y_SCALE output rows, rather than a runtime divide.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module nn_scale_addr #(
    parameter integer X_SHIFT = 3,  // 1280/IMG_W == 2**X_SHIFT
    parameter integer Y_SCALE = 6   // 720/IMG_H
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        de,
    input  wire [15:0] x,
    input  wire [15:0] y,

    output wire [7:0] small_x,
    output reg  [7:0] small_y
);

    assign small_x = x[X_SHIFT+7 : X_SHIFT];

    reg [7:0] y_sub; // counts 0 .. Y_SCALE-1

    always @(posedge clk) begin
        if (rst) begin
            y_sub   <= 8'd0;
            small_y <= 8'd0;
        end else if (de && x == 16'd0 && y == 16'd0) begin
            y_sub   <= 8'd0;   // first active row of a new frame
            small_y <= 8'd0;
        end else if (de && x == 16'd0) begin
            if (y_sub == Y_SCALE - 1) begin
                y_sub   <= 8'd0;
                small_y <= small_y + 1'b1;
            end else begin
                y_sub <= y_sub + 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
