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

    // Single-ended from RTL's perspective: these drive gpdi_dp[] pads
    // constrained IO_TYPE=LVCMOS33D in the LPF, which makes the ECP5 I/O
    // hardware generate the complementary signal on the paired physical
    // pad by itself (pseudo-differential) - see the "gpdi_dn[]" note
    // below the OLVDS block that used to be here.
    output wire hdmi_clk,
    output wire hdmi_d0,   // blue
    output wire hdmi_d1,   // green
    output wire hdmi_d2    // red
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

    // Pseudo-differential output: drive one net per channel and let the
    // LPF's IO_TYPE=LVCMOS33D constraint (see constraints/icepi_zero_hdmi.lpf)
    // make the ECP5 I/O hardware generate the complementary signal on the
    // paired physical pad automatically. No OLVDS/true-LVDS primitive
    // needed - and no separate simulation-only shim either, since there's
    // now only one real net per channel in both sim and synthesis.
    //
    // This replaced an earlier true-differential OLVDS implementation
    // (two explicit RTL ports per channel, Z/ZN) after real nextpnr-ecp5
    // place-and-route runs failed with "cannot place differential IO at
    // location PIOB"/"PIOD": this board's gpdi_dn[0]/gpdi_dn[1] pads sit
    // on Bel types Trellis's ECP5-25F database doesn't support true-LVDS
    // output on. This repo's other two HDMI projects
    // (dvp_camera_hdmi_pipeline, icepi_zero_bringup/03_sdcard_hdmi_image)
    // already use this same single-port LVCMOS33D approach successfully.
    assign hdmi_clk = serial_clk;
    assign hdmi_d0  = serial_d0;
    assign hdmi_d1  = serial_d1;
    assign hdmi_d2  = serial_d2;

endmodule
