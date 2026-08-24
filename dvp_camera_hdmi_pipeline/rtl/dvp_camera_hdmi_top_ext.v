// ============================================================================
// dvp_camera_hdmi_top_ext.v -- top level: DVP camera -> true 1080p60 via an
//                               external HDMI/DVI transmitter IC
// ----------------------------------------------------------------------------
// The LFE5U-25F (a non-"5G" ECP5 part, no built-in SERDES/DCU) cannot
// serialize a full-rate 1080p60 TMDS bit stream (1.485 Gbps/lane) through
// its fabric DDR gearbox -- see clk_gen_1080p60.v and README.md "Why two
// different 1080p60 delivery paths" for the exact numbers. This top level
// sidesteps that ceiling entirely: it runs the whole pixel pipeline at the
// real 148.5MHz-class 1080p60 pixel clock and hands the result to an
// external dedicated HDMI/DVI transmitter chip (ADV7511/ADV7513, ITE
// IT6613/IT66121, SiI9022-class parts all accept this) over a standard
// 24-bit parallel-RGB + HSYNC/VSYNC/DE + pixel-clock video bus -- no
// on-chip TMDS serialization needed, so no ECLK-ceiling issue at all.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module dvp_camera_hdmi_top_ext #(
    parameter CAMERA_FORMAT = "RGB565",
    parameter BYTE_SWAP     = 1'b0,
    parameter HREF_POL      = 1'b1,
    parameter VSYNC_POL     = 1'b1,
    parameter I2C_DEV_ADDR7 = 7'h21,
    parameter PLL_OPT       = "A"     // see clk_gen_1080p60.v
) (
    input  wire        clk,          // 50MHz board reference oscillator
    input  wire  [1:0] button,       // button[0]=reset, button[1]=pattern select

    input  wire         cam_pclk,
    input  wire         cam_href,
    input  wire         cam_vsync,
    input  wire [7:0]   cam_d,
    inout  wire          cam_scl,
    inout  wire          cam_sda,

    // external transmitter parallel-RGB video bus
    output wire         hdmi_pclk,
    output wire         hdmi_de,
    output wire         hdmi_hsync,
    output wire         hdmi_vsync,
    output wire [23:0]  hdmi_d,       // {R[7:0],G[7:0],B[7:0]}

    output wire [4:0]   led
);

    localparam integer H_ACTIVE = 1920, H_FRONT = 88, H_SYNC = 44, H_BACK = 148;
    localparam integer V_ACTIVE = 1080, V_FRONT = 4,  V_SYNC = 5,  V_BACK = 36;

    // ------------------------------------------------------------------
    // Reset + PLL
    // ------------------------------------------------------------------
    wire rst_btn = ~button[0];
    wire pll_locked;
    wire clk_pixel;

    clk_gen_1080p60 #(.PLL_OPT(PLL_OPT)) u_pll (
        .clk_in(clk), .clk_pixel(clk_pixel), .locked(pll_locked)
    );

    reg [3:0] rst_pixel_sr = 4'hF;
    always @(posedge clk_pixel or negedge pll_locked)
        if (!pll_locked) rst_pixel_sr <= 4'hF;
        else             rst_pixel_sr <= {rst_pixel_sr[2:0], rst_btn};
    wire rst_pixel = rst_pixel_sr[3];

    // ------------------------------------------------------------------
    // Camera capture chain
    // ------------------------------------------------------------------
    wire byte_valid, line_start_cap, frame_start_cap;
    wire [7:0] byte_data;

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

    wire [23:0] buf_rgb;
    wire        buf_ready;
    wire        out_de;

    video_cdc_buffer #(.DEPTH(8192), .PREFILL_WORDS(4096)) u_cdc (
        .cam_pclk(cam_pclk), .cam_rst(rst_cam),
        .cam_pixel_valid(cam_pixel_valid), .cam_rgb(cam_rgb),
        .out_pclk(clk_pixel), .out_rst(rst_pixel),
        .out_de(out_de), .out_rgb(buf_rgb), .out_ready(buf_ready)
    );

    // ------------------------------------------------------------------
    // Video timing generator + pixel source mux
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

    wire [23:0] pattern_rgb;
    test_pattern_gen #(.H_ACTIVE(H_ACTIVE), .V_ACTIVE(V_ACTIVE), .BAR_W_LOG2(8))
        u_pattern (.x(x), .y(y), .rgb(pattern_rgb));

    wire pattern_sel = ~button[1];

    // Same 1-cycle re-alignment rationale as dvp_camera_hdmi_top.v: buf_rgb
    // trails out_de/x/y by one cycle by construction of the CDC buffer's
    // registered FIFO read.
    reg        hsync_r, vsync_r, de_r;
    reg [23:0] pattern_rgb_r;
    always @(posedge clk_pixel) begin
        hsync_r       <= hsync;
        vsync_r       <= vsync;
        de_r          <= out_de;
        pattern_rgb_r <= pattern_rgb;
    end

    wire [23:0] pixel_rgb = pattern_sel ? pattern_rgb_r : buf_rgb;

    assign hdmi_pclk  = clk_pixel;
    assign hdmi_de    = de_r;
    assign hdmi_hsync = hsync_r;
    assign hdmi_vsync = vsync_r;
    assign hdmi_d     = pixel_rgb;

    // ------------------------------------------------------------------
    // Camera I2C configuration sequencer
    // ------------------------------------------------------------------
    wire i2c_start, i2c_busy, i2c_done, i2c_nack;
    wire [7:0] i2c_reg_addr, i2c_reg_data;
    wire cfg_done;

    reg go_d1, go_d2;
    always @(posedge clk_pixel) begin
        go_d1 <= pll_locked;
        go_d2 <= go_d1;
    end
    wire cfg_go = go_d1 & ~go_d2;

    i2c_master #(.CLK_FREQ_HZ(148_500_000), .I2C_FREQ_HZ(100_000), .DEV_ADDR7(I2C_DEV_ADDR7)) u_i2c (
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

    assign led[0] = pll_locked;
    assign led[1] = cfg_done;
    assign led[2] = buf_ready;
    assign led[3] = i2c_nack;
    assign led[4] = pattern_sel;

endmodule

`default_nettype wire
