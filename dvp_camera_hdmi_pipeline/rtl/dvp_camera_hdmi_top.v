// ============================================================================
// dvp_camera_hdmi_top.v -- top level: DVP camera -> native GPDI/TMDS output
// ----------------------------------------------------------------------------
// Targets the IcePi-Zero board (Lattice ECP5 LFE5U-25F-6BG256C). Produces
// native TMDS on the board's GPDI connector at a fixed 74.25MHz-class pixel
// clock, covering:
//   RESOLUTION = "720P60"  -> 1280x720 @ 60Hz  (CEA-861 VIC 4)
//   RESOLUTION = "1080P30" -> 1920x1080 @ 30Hz (same H/V blanking totals as
//                             1080p60, half the pixel clock -- see
//                             clk_gen_dvi.v and README.md for why 1080p60
//                             itself needs the *external-transmitter* top
//                             level, dvp_camera_hdmi_top_ext.v, instead)
//
// Set CAMERA_FORMAT to match your sensor's DVP output ("RGB565" or
// "YUYV422"), and edit cam_config_rom.v's register table for your specific
// sensor part number.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module dvp_camera_hdmi_top #(
    parameter RESOLUTION    = "720P60",   // "720P60" or "1080P30"
    parameter CAMERA_FORMAT = "RGB565",   // "RGB565" or "YUYV422"
    parameter BYTE_SWAP     = 1'b0,
    parameter HREF_POL      = 1'b1,
    parameter VSYNC_POL     = 1'b1,
    parameter I2C_DEV_ADDR7 = 7'h21
) (
    input  wire        clk,          // 50MHz board reference oscillator (site M1)
    input  wire  [1:0] button,       // active-low buttons; button[0]=reset, button[1]=pattern select

    // DVP camera interface
    input  wire         cam_pclk,
    input  wire         cam_href,
    input  wire         cam_vsync,
    input  wire [7:0]   cam_d,
    inout  wire          cam_scl,
    inout  wire          cam_sda,

    // native GPDI/TMDS output
    output wire [3:0]   gpdi_dp,      // [0]=Blue [1]=Green [2]=Red [3]=Clock

    output wire [4:0]   led
);

    // ------------------------------------------------------------------
    // Video mode selection
    // ------------------------------------------------------------------
    localparam IS_1080 = (RESOLUTION == "1080P30");
    localparam integer H_ACTIVE = IS_1080 ? 1920 : 1280;
    localparam integer H_FRONT  = IS_1080 ? 88   : 110;
    localparam integer H_SYNC   = IS_1080 ? 44   : 40;
    localparam integer H_BACK   = IS_1080 ? 148  : 220;
    localparam integer V_ACTIVE = IS_1080 ? 1080 : 720;
    localparam integer V_FRONT  = IS_1080 ? 4    : 5;
    localparam integer V_SYNC   = IS_1080 ? 5    : 5;
    localparam integer V_BACK   = IS_1080 ? 36   : 20;
    localparam integer BAR_LOG2 = IS_1080 ? 8    : 7;

    // ------------------------------------------------------------------
    // Reset + PLL
    // ------------------------------------------------------------------
    wire rst_btn = ~button[0]; // active-low button -> active-high reset request
    wire pll_locked;
    wire clk_pixel, clk_eclk;

    clk_gen_dvi u_pll (
        .clk_in    (clk),
        .clk_pixel (clk_pixel),
        .clk_eclk  (clk_eclk),
        .locked    (pll_locked)
    );

    reg [3:0] rst_pixel_sr = 4'hF;
    always @(posedge clk_pixel or negedge pll_locked)
        if (!pll_locked) rst_pixel_sr <= 4'hF;
        else             rst_pixel_sr <= {rst_pixel_sr[2:0], rst_btn};
    wire rst_pixel = rst_pixel_sr[3];

    // ------------------------------------------------------------------
    // Camera capture chain (cam_pclk domain)
    // ------------------------------------------------------------------
    wire byte_valid, line_start_cap, frame_start_cap;
    wire [7:0] byte_data;

    // camera-domain reset: level-synchronized from the (async, cross-domain)
    // pixel-domain reset request via a simple 2FF synchronizer into cam_pclk.
    reg [1:0] rst_cam_sr = 2'b11;
    always @(posedge cam_pclk or posedge rst_pixel)
        if (rst_pixel) rst_cam_sr <= 2'b11;
        else           rst_cam_sr <= {rst_cam_sr[0], 1'b0};
    wire rst_cam = rst_cam_sr[1];

    dvp_capture #(.HREF_POL(HREF_POL), .VSYNC_POL(VSYNC_POL)) u_capture (
        .pclk(cam_pclk), .rst_async(rst_pixel),
        .href(cam_href), .vsync(cam_vsync), .d(cam_d),
        .byte_valid(byte_valid), .byte_data(byte_data),
        .line_start(line_start_cap), .frame_start(frame_start_cap)
    );

    wire cam_pixel_valid;
    wire [23:0] cam_rgb;

    pixel_formatter #(.FORMAT(CAMERA_FORMAT), .BYTE_SWAP(BYTE_SWAP)) u_formatter (
        .pclk(cam_pclk), .rst(rst_cam),
        .byte_valid(byte_valid), .byte_data(byte_data),
        .line_start(line_start_cap), .frame_start(frame_start_cap),
        .pixel_valid(cam_pixel_valid), .rgb(cam_rgb),
        .pixel_line_start(), .pixel_frame_start()
    );

    // ------------------------------------------------------------------
    // Camera -> pixel-clock domain CDC buffer
    // ------------------------------------------------------------------
    wire [23:0] buf_rgb;
    wire        buf_ready;
    wire        out_de;

    video_cdc_buffer #(
        .DEPTH(IS_1080 ? 8192 : 4096),
        .PREFILL_WORDS(IS_1080 ? 4096 : 2048)
    ) u_cdc (
        .cam_pclk(cam_pclk), .cam_rst(rst_cam),
        .cam_pixel_valid(cam_pixel_valid), .cam_rgb(cam_rgb),
        .out_pclk(clk_pixel), .out_rst(rst_pixel),
        .out_de(out_de), .out_rgb(buf_rgb), .out_ready(buf_ready)
    );

    // ------------------------------------------------------------------
    // Video timing generator (pixel-clock domain)
    // ------------------------------------------------------------------
    wire hsync, vsync;
    wire [15:0] x, y;

    video_timing_gen #(
        .H_ACTIVE(H_ACTIVE), .H_FRONT(H_FRONT), .H_SYNC(H_SYNC), .H_BACK(H_BACK),
        .V_ACTIVE(V_ACTIVE), .V_FRONT(V_FRONT), .V_SYNC(V_SYNC), .V_BACK(V_BACK),
        .HS_POL(1'b1), .VS_POL(1'b1)
    ) u_timing (
        .clk(clk_pixel), .rst(rst_pixel),
        .hsync(hsync), .vsync(vsync), .de(out_de), .frame_start(),
        .x(x), .y(y)
    );

    // ------------------------------------------------------------------
    // Pixel source: on-chip test pattern for bring-up (button[1] held), or
    // the live camera feed via the CDC buffer (button[1] released).
    // ------------------------------------------------------------------
    wire [23:0] pattern_rgb;
    test_pattern_gen #(.H_ACTIVE(H_ACTIVE), .V_ACTIVE(V_ACTIVE), .BAR_W_LOG2(BAR_LOG2))
        u_pattern (.x(x), .y(y), .rgb(pattern_rgb));

    wire pattern_sel = ~button[1];

    // Pipeline register: this is REQUIRED for two independent reasons, not
    // just timing closure --
    //  (1) async_fifo's rd_data (and hence buf_rgb) is itself a *registered*
    //      read output with one cycle of latency from rd_en (=out_de) to
    //      valid data, so buf_rgb for a given pixel only becomes valid on
    //      the clock edge *after* the timing generator has already moved on
    //      to the next pixel's (x,y,de). Registering hsync/vsync/de and
    //      pattern_rgb by one cycle here re-aligns everything (registered
    //      sync + registered pattern, vs. already-delayed-by-construction
    //      buf_rgb) onto the same pixel.
    //  (2) it also breaks what would otherwise be one long combinational
    //      path from the timing generator's counters, through
    //      test_pattern_gen's comparator, straight into tmds_encoder's own
    //      internal 8b/10b logic -- verified against real STA (nextpnr-ecp5)
    //      to blow the 74.25MHz timing budget without this register.
    reg        hsync_r, vsync_r, de_r;
    reg [23:0] pattern_rgb_r;
    always @(posedge clk_pixel) begin
        hsync_r       <= hsync;
        vsync_r       <= vsync;
        de_r          <= out_de;
        pattern_rgb_r <= pattern_rgb;
    end

    wire [23:0] pixel_rgb = pattern_sel ? pattern_rgb_r : buf_rgb;

    // ------------------------------------------------------------------
    // TMDS encode + native GPDI serialization
    // ------------------------------------------------------------------
    wire [9:0] tmds_r, tmds_g, tmds_b;

    tmds_encoder u_enc_r (.clk(clk_pixel), .rst(rst_pixel), .din(pixel_rgb[23:16]), .ctrl(2'b00),            .de(de_r), .tmds(tmds_r));
    tmds_encoder u_enc_g (.clk(clk_pixel), .rst(rst_pixel), .din(pixel_rgb[15:8]),  .ctrl(2'b00),            .de(de_r), .tmds(tmds_g));
    tmds_encoder u_enc_b (.clk(clk_pixel), .rst(rst_pixel), .din(pixel_rgb[7:0]),   .ctrl({vsync_r,hsync_r}),.de(de_r), .tmds(tmds_b));

    tmds_serial_gearbox u_ser (
        .clk_pixel(clk_pixel), .clk_eclk(clk_eclk), .rst_pixel(rst_pixel),
        .tmds_r(tmds_r), .tmds_g(tmds_g), .tmds_b(tmds_b),
        .gpdi_dp(gpdi_dp)
    );

    // ------------------------------------------------------------------
    // Camera I2C configuration sequencer (runs once after PLL lock)
    // ------------------------------------------------------------------
    wire i2c_start, i2c_busy, i2c_done, i2c_nack;
    wire [7:0] i2c_reg_addr, i2c_reg_data;
    wire cfg_done;

    reg go_d1, go_d2;
    always @(posedge clk_pixel) begin
        go_d1 <= pll_locked;
        go_d2 <= go_d1;
    end
    wire cfg_go = go_d1 & ~go_d2; // one-shot pulse on PLL lock

    i2c_master #(.CLK_FREQ_HZ(74_286_000), .I2C_FREQ_HZ(100_000), .DEV_ADDR7(I2C_DEV_ADDR7)) u_i2c (
        .clk(clk_pixel), .rst(rst_pixel),
        .start(i2c_start), .reg_addr(i2c_reg_addr), .reg_data(i2c_reg_data),
        .busy(i2c_busy), .done(i2c_done), .nack_error(i2c_nack),
        .scl(cam_scl), .sda(cam_sda)
    );

    cam_config_rom u_cfg (
        .clk(clk_pixel), .rst(rst_pixel), .go(cfg_go), .config_done(cfg_done),
        .i2c_start(i2c_start), .i2c_reg_addr(i2c_reg_addr), .i2c_reg_data(i2c_reg_data),
        .i2c_busy(i2c_busy), .i2c_done(i2c_done), .i2c_nack_error(i2c_nack)
    );

    // ------------------------------------------------------------------
    // Status LEDs
    // ------------------------------------------------------------------
    assign led[0] = pll_locked;
    assign led[1] = cfg_done;
    assign led[2] = buf_ready;
    assign led[3] = i2c_nack;      // lit = at least one NACK seen during config
    assign led[4] = pattern_sel;

endmodule

`default_nettype wire
