// ============================================================================
// pixel_formatter.v -- byte stream -> RGB888 pixel assembler
// ----------------------------------------------------------------------------
// Consumes the byte stream produced by dvp_capture.v and reassembles full
// pixels, converting to 24-bit RGB888. Two common DVP sensor output formats
// are supported, selected with the FORMAT parameter:
//
//   "RGB565"  : 2 bytes/pixel, {R[4:0],G[5:3]} then {G[2:0],B[4:0]}
//               (set BYTE_SWAP=1 if your sensor sends the low byte first)
//   "YUYV422" : 4 bytes / 2 pixels, sequence Y0 U0 Y1 V0 (BT.601 -> RGB)
//
// All logic stays in the pclk (camera) clock domain -- this module simply
// widens the byte stream to a pixel stream at the same clock.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module pixel_formatter #(
    parameter FORMAT     = "RGB565", // "RGB565" or "YUYV422"
    parameter BYTE_SWAP  = 1'b0
) (
    input  wire        pclk,
    input  wire         rst,

    input  wire         byte_valid,
    input  wire [7:0]   byte_data,
    input  wire         line_start,
    input  wire         frame_start,

    output reg          pixel_valid,
    output reg  [23:0]  rgb,          // {R,G,B}, 8 bits each
    output reg          pixel_line_start,
    output reg          pixel_frame_start
);

    generate
    if (FORMAT == "RGB565") begin : G_RGB565
        reg        byte_phase; // 0 = expect high byte, 1 = expect low byte
        reg [7:0]  byte_hi;
        reg        pend_line_start, pend_frame_start;

        wire [7:0] b0 = BYTE_SWAP ? byte_data : byte_hi;   // captured first
        wire [7:0] b1 = BYTE_SWAP ? byte_hi   : byte_data; // captured second
        // b0 = {R[4:0],G[5:3]}, b1 = {G[2:0],B[4:0]} (standard RGB565 order)
        wire [4:0] r5 = b0[7:3];
        wire [5:0] g6 = {b0[2:0], b1[7:5]};
        wire [4:0] b5 = b1[4:0];
        wire [7:0] r8 = {r5, r5[4:2]};
        wire [7:0] g8 = {g6, g6[5:4]};
        wire [7:0] b8 = {b5, b5[4:2]};

        always @(posedge pclk) begin
            if (rst) begin
                byte_phase        <= 1'b0;
                pixel_valid       <= 1'b0;
                pend_line_start   <= 1'b0;
                pend_frame_start  <= 1'b0;
                pixel_line_start  <= 1'b0;
                pixel_frame_start <= 1'b0;
                rgb               <= 24'h0;
            end else begin
                pixel_valid <= 1'b0;
                if (line_start || frame_start) begin
                    // resynchronize to the start of every line: first byte
                    // after HREF rises is always the high byte.
                    byte_phase       <= 1'b0;
                    pend_line_start  <= pend_line_start | line_start;
                    pend_frame_start <= pend_frame_start | frame_start;
                end

                if (byte_valid) begin
                    if (byte_phase == 1'b0) begin
                        byte_hi    <= byte_data;
                        byte_phase <= 1'b1;
                    end else begin
                        byte_phase        <= 1'b0;
                        rgb               <= {r8, g8, b8};
                        pixel_valid       <= 1'b1;
                        pixel_line_start  <= pend_line_start;
                        pixel_frame_start <= pend_frame_start;
                        pend_line_start   <= 1'b0;
                        pend_frame_start  <= 1'b0;
                    end
                end
            end
        end
    end // G_RGB565
    endgenerate

    generate
    if (FORMAT == "YUYV422") begin : G_YUYV422
        // Byte sequence per 2-pixel group: Y0 U Y1 V (standard "YUYV" order).
        // Pixel0 = (Y0,U,V), Pixel1 = (Y1,U,V) -- both chroma-shared.
        // Only one pixel can be emitted per byte cycle, so pixel0 is emitted
        // the moment V arrives (end of its group) and pixel1 is emitted one
        // byte-cycle later, piggy-backed on the Y0 capture cycle of the
        // *next* group (bytes arrive back-to-back with no idle cycles, so
        // there is no free cycle to emit both at once).
        reg [1:0]  phase; // 0:Y0 1:U 2:Y1 3:V
        reg [7:0]  y0_r, u_r, y1_r, v_r;
        reg        pend_line_start, pend_frame_start;
        reg        group_pending;   // a completed (y1,u,v) group awaits pixel1 emission
        reg        pix1_line_start, pix1_frame_start;

        wire signed [9:0] cb_s = $signed({2'b00, u_r}) - 10'sd128;
        wire signed [9:0] cr_s = $signed({2'b00, v_r}) - 10'sd128;

        // BT.601 fixed-point (Q8) coefficients: 1.402*256=359, 0.344*256=88,
        // 0.714*256=183, 1.772*256=454 (standard integer approximation).
        wire signed [18:0] r_term =  359 * cr_s;
        wire signed [18:0] g_term = -(88  * cb_s) - (183 * cr_s);
        wire signed [18:0] b_term =  454 * cb_s;

        // Pixel0 is emitted the same cycle V arrives on byte_data, *before*
        // v_r <= byte_data has taken effect (NBA), so pixel0's chroma terms
        // must be computed directly from byte_data rather than from v_r.
        wire signed [9:0]  cr_s_now = $signed({2'b00, byte_data}) - 10'sd128;
        wire signed [18:0] r_term_now =  359 * cr_s_now;
        wire signed [18:0] g_term_now = -(88  * cb_s) - (183 * cr_s_now);
        wire signed [18:0] b_term_now =  454 * cb_s;

        function [7:0] add_clamp;
            input [7:0]         y_in;
            input signed [18:0] term;
            reg   signed [26:0] sum;
            begin
                sum = $signed({1'b0, y_in}) + (term >>> 8);
                if (sum < 0)          add_clamp = 8'h00;
                else if (sum > 255)   add_clamp = 8'hFF;
                else                  add_clamp = sum[7:0];
            end
        endfunction

        always @(posedge pclk) begin
            if (rst) begin
                phase             <= 2'd0;
                pixel_valid       <= 1'b0;
                pend_line_start   <= 1'b0;
                pend_frame_start  <= 1'b0;
                pixel_line_start  <= 1'b0;
                pixel_frame_start <= 1'b0;
                group_pending     <= 1'b0;
                rgb               <= 24'h0;
            end else begin
                pixel_valid <= 1'b0;

                if (line_start || frame_start) begin
                    phase            <= 2'd0;
                    group_pending    <= 1'b0; // discard any half-finished group at a line boundary
                    pend_line_start  <= pend_line_start | line_start;
                    pend_frame_start <= pend_frame_start | frame_start;
                end

                if (byte_valid) begin
                    case (phase)
                        2'd0: begin
                            y0_r  <= byte_data;
                            phase <= 2'd1;
                            // Piggy-back: emit pixel1 of the *previous* group.
                            if (group_pending) begin
                                rgb               <= {add_clamp(y1_r, r_term),
                                                       add_clamp(y1_r, g_term),
                                                       add_clamp(y1_r, b_term)};
                                pixel_valid       <= 1'b1;
                                pixel_line_start  <= pix1_line_start;
                                pixel_frame_start <= pix1_frame_start;
                                group_pending     <= 1'b0;
                            end
                        end
                        2'd1: begin u_r  <= byte_data; phase <= 2'd2; end
                        2'd2: begin y1_r <= byte_data; phase <= 2'd3; end
                        2'd3: begin
                            v_r   <= byte_data;
                            phase <= 2'd0;
                            // Emit pixel0 now (chroma from byte_data directly,
                            // see r_term_now/g_term_now/b_term_now above);
                            // remember to emit pixel1 next.
                            rgb               <= {add_clamp(y0_r, r_term_now),
                                                   add_clamp(y0_r, g_term_now),
                                                   add_clamp(y0_r, b_term_now)};
                            pixel_valid       <= 1'b1;
                            pixel_line_start  <= pend_line_start;
                            pixel_frame_start <= pend_frame_start;
                            pix1_line_start   <= 1'b0; // pixel1 is never itself a line/frame start
                            pix1_frame_start  <= 1'b0;
                            pend_line_start   <= 1'b0;
                            pend_frame_start  <= 1'b0;
                            group_pending     <= 1'b1;
                        end
                    endcase
                end
            end
        end
    end // G_YUYV422
    endgenerate

endmodule

`default_nettype wire
