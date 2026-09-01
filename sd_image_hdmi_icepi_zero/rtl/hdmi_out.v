`timescale 1ns/1ps
// hdmi_out : ties together video timing, framebuffer scan-out, RGB565
// expansion, TMDS encoding and serialization, and the differential
// output buffers - the whole pix_clk/shift_clk side of the design.
//
// Pipeline / latency bookkeeping (all pix_clk cycles unless noted):
//   stage 0 : video_timing's registered hsync/vsync/de/x/y ("now")
//   stage 1 : framebuffer read data becomes valid 1 cycle after its
//             address is presented, so hsync/vsync/de are re-registered
//             once here to stay paired with fb_data
//   stage 2 : tmds_encoder registers its own output 1 cycle after its
//             inputs (already stage-1-aligned) are presented
// tmds_serializer (shift_clk domain) then reads stage-2 tmds_q directly
// - no further pix_clk-domain delay needed.
//
// 640x480 output is produced from the 160x120 framebuffer by exact 4x
// nearest-neighbor pixel replication (fb address = (y>>2)*160+(x>>2)),
// chosen because it's an exact integer scale factor - no interpolation
// logic needed, and no fractional-pixel artifacts.
module hdmi_out (
    input  wire        pix_clk,
    input  wire        shift_clk,
    input  wire        rst,

    input  wire        fb_loaded,     // synchronized into pix_clk domain by the caller

    // Framebuffer read port - the caller wires framebuffer.rd_clk to
    // pix_clk directly; only the address/data pair is threaded through
    // here.
    output wire [14:0]  fb_rd_addr,
    input  wire [15:0]  fb_rd_data,

    output wire hdmi_clk_p,   hdmi_clk_n,
    output wire hdmi_d0_p,    hdmi_d0_n,   // blue
    output wire hdmi_d1_p,    hdmi_d1_n,   // green
    output wire hdmi_d2_p,    hdmi_d2_n    // red
);

    // ---- stage 0: timing ---------------------------------------------
    wire        hsync0, vsync0, de0;
    wire [9:0]  x0, y0;

    video_timing u_timing (
        .pix_clk (pix_clk),
        .rst     (rst),
        .hsync   (hsync0),
        .vsync   (vsync0),
        .de      (de0),
        .x       (x0),
        .y       (y0)
    );

    assign fb_rd_addr = {1'b0, y0[8:2]} * 15'd160 + {7'd0, x0[9:2]};

    // ---- stage 1: re-align with framebuffer's 1-cycle read latency ---
    reg hsync1, vsync1, de1;
    always @(posedge pix_clk) begin
        hsync1 <= hsync0;
        vsync1 <= vsync0;
        de1    <= de0 && fb_loaded; // blank (all-black) until the image has finished loading
    end

    // RGB565 -> RGB888 by MSB-replication (0->0x00, max->0xFF exactly).
    wire [4:0] r5 = fb_rd_data[15:11];
    wire [5:0] g6 = fb_rd_data[10:5];
    wire [4:0] b5 = fb_rd_data[4:0];
    wire [7:0] r8 = {r5, r5[4:2]};
    wire [7:0] g8 = {g6, g6[5:4]};
    wire [7:0] b8 = {b5, b5[4:2]};

    // ---- stage 2: TMDS encode (only blue carries hsync/vsync) --------
    wire [9:0] tmds_r, tmds_g, tmds_b;

    tmds_encoder u_enc_r (.pix_clk(pix_clk), .rst(rst), .data(r8), .ctrl(2'b00),            .de(de1), .tmds_q(tmds_r));
    tmds_encoder u_enc_g (.pix_clk(pix_clk), .rst(rst), .data(g8), .ctrl(2'b00),            .de(de1), .tmds_q(tmds_g));
    tmds_encoder u_enc_b (.pix_clk(pix_clk), .rst(rst), .data(b8), .ctrl({vsync1, hsync1}), .de(de1), .tmds_q(tmds_b));

    // TMDS clock channel: the fixed "5 zeros then 5 ones" pattern every
    // pixel clock, per the DVI/HDMI spec - not run through the encoder,
    // it's a constant, not encoded pixel/control data. tmds_serializer
    // transmits bit 0 (LSB) first, so bit0..bit4 must be the zeros and
    // bit5..bit9 the ones - written MSB-first as a Verilog literal,
    // that's 1's in bits 9-5 and 0's in bits 4-0.
    localparam [9:0] TMDS_CLOCK_PATTERN = 10'b1111100000;

    wire serial_clk, serial_d0, serial_d1, serial_d2;

    tmds_serializer u_ser_clk (.shift_clk(shift_clk), .rst(rst), .tmds_word(TMDS_CLOCK_PATTERN), .serial_out(serial_clk));
    tmds_serializer u_ser_d0  (.shift_clk(shift_clk), .rst(rst), .tmds_word(tmds_b),              .serial_out(serial_d0));
    tmds_serializer u_ser_d1  (.shift_clk(shift_clk), .rst(rst), .tmds_word(tmds_g),              .serial_out(serial_d1));
    tmds_serializer u_ser_d2  (.shift_clk(shift_clk), .rst(rst), .tmds_word(tmds_r),              .serial_out(serial_d2));

`ifdef SIMULATION
    // No true differential buffer model in open simulation - expose a
    // simple single-ended/inverted pair purely so a testbench can probe
    // the bitstream. Not representative of real TMDS electrical levels.
    assign hdmi_clk_p = serial_clk; assign hdmi_clk_n = ~serial_clk;
    assign hdmi_d0_p  = serial_d0;  assign hdmi_d0_n  = ~serial_d0;
    assign hdmi_d1_p  = serial_d1;  assign hdmi_d1_n  = ~serial_d1;
    assign hdmi_d2_p  = serial_d2;  assign hdmi_d2_n  = ~serial_d2;
`else
    // OLVDS: ECP5 true-LVDS differential output buffer, the standard
    // way to drive TMDS-style signaling from ECP5 general-purpose I/O
    // (there is no dedicated HDMI PHY on this device). Verify this is
    // the exact primitive/library name your Diamond version expects -
    // some library versions/board reference designs use ELVDS_OBUF
    // instead; check Diamond's IP/primitive browser if OLVDS doesn't
    // elaborate.
    OLVDS obuf_clk (.A(serial_clk), .Z(hdmi_clk_p), .ZN(hdmi_clk_n));
    OLVDS obuf_d0  (.A(serial_d0),  .Z(hdmi_d0_p),  .ZN(hdmi_d0_n));
    OLVDS obuf_d1  (.A(serial_d1),  .Z(hdmi_d1_p),  .ZN(hdmi_d1_n));
    OLVDS obuf_d2  (.A(serial_d2),  .Z(hdmi_d2_p),  .ZN(hdmi_d2_n));
`endif

endmodule
