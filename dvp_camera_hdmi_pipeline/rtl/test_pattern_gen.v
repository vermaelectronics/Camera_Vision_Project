// ============================================================================
// test_pattern_gen.v -- 8-bar colour bar generator + bottom-strip ramp
// ----------------------------------------------------------------------------
// Purely combinational function of (x, y) from video_timing_gen.v. Useful
// for bringing up and verifying the HDMI/DVI output chain (PLL, TMDS
// encoder, serializer, GPDI wiring) completely independently of whether a
// camera is attached/working yet -- wire this in place of the camera pixel
// stream via CAM_SOURCE_SEL in the top level.
//
// Plain Verilog-2001 only (no SystemVerilog casts/division of variable
// operands) so it maps to simple combinational logic, no hardware divider.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module test_pattern_gen #(
    parameter H_ACTIVE  = 1280,
    parameter V_ACTIVE  = 720,
    parameter BAR_W_LOG2 = 7      // 2^BAR_W_LOG2 =~ H_ACTIVE/8; tune per resolution
                                    // (1280/8=160~=2^7=128; 1920/8=240~=2^8=256 -> pass 8 for 1080p)
) (
    input  wire [15:0] x,
    input  wire [15:0] y,
    output reg  [23:0] rgb
);

    wire [15:0] bar_idx = x >> BAR_W_LOG2; // 0..7ish depending on resolution

    always @(*) begin
        case (bar_idx[2:0])
            3'd0: rgb = 24'hFFFFFF; // white
            3'd1: rgb = 24'hFFFF00; // yellow
            3'd2: rgb = 24'h00FFFF; // cyan
            3'd3: rgb = 24'h00FF00; // green
            3'd4: rgb = 24'hFF00FF; // magenta
            3'd5: rgb = 24'hFF0000; // red
            3'd6: rgb = 24'h0000FF; // blue
            default: rgb = 24'h000000; // black
        endcase

        // Bottom 1/8th of the frame: a horizontal grayscale ramp (coarse bit
        // truncation, not exact 0..255 across the full width -- good enough
        // to spot banding/format issues visually during bring-up).
        if (y >= (V_ACTIVE - (V_ACTIVE >> 3))) begin
            rgb = {x[10:3], x[10:3], x[10:3]};
        end
    end

endmodule

`default_nettype wire
