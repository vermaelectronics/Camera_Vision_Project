// ============================================================================
// dvp_camera_hdmi_top.v -- top level: DVP camera -> native GPDI/TMDS output
// ----------------------------------------------------------------------------
// Targets the IcePi-Zero board (Lattice ECP5 LFE5U-25F-6BG256C). Produces
// native TMDS on the board's GPDI connector at a fixed 74.25MHz pixel clock,
// 1280x720 @ 60Hz (CEA-861 VIC 4) -- this module is 720p60-only. (An
// earlier revision also supported 1920x1080@30 via a RESOLUTION parameter;
// that was dropped to simplify bring-up/debugging to a single fixed
// configuration. 1080p60 itself needs the *external-transmitter* top
// level, dvp_camera_hdmi_top_ext.v, instead -- see README.md.)
//
// Set CAMERA_FORMAT to match your sensor's DVP output ("RGB565" or
// "YUYV422"), and edit cam_config_rom.v's register table for your specific
// sensor part number.
//
// Also includes a UART debug output (`uart_tx`, 115200 8N1) that streams a
// live human-readable hardware status line -- PLL/MCLK lock, I2C config
// progress, NACK count, buffer-ready, camera mode, and a live camera-frame
// counter -- to a terminal (PuTTY/minicom) on your PC. See uart_debug.v
// and README.md "UART debug interface".
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module dvp_camera_hdmi_top #(
    parameter CAMERA_FORMAT = "RGB565",   // "RGB565" or "YUYV422"
    parameter BYTE_SWAP     = 1'b0,
    parameter RB_SWAP       = 1'b0,       // swap R/B *channel content* (not byte-transmission order --
                                           // see BYTE_SWAP for that). Set to 1 if the image is otherwise
                                           // correct (framing, motion, brightness) but red/blue are
                                           // swapped -- e.g. warm scene content renders blue/purple while
                                           // white/bright content stays roughly neutral. RGB565 path only.
    parameter HREF_POL      = 1'b0,       // confirmed on real hardware (Waveshare OV5640 DVP module +
                                           // IcePi-Zero): this polarity, not the originally-assumed 1'b1,
                                           // is what gets the frame counter incrementing. VSYNC_POL=1'b1
                                           // was already correct.
    parameter VSYNC_POL     = 1'b1,
    parameter I2C_DEV_ADDR7 = 7'h3C,      // default = OV5640 (Waveshare DVP module); 0x21 was the earlier generic placeholder
    parameter ADDR_BYTES    = 2           // 2 = 16-bit reg addressing (OV5640/OV5647-class); 1 = 8-bit (OV7670/OV2640-class)
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

    // Camera power-up control -- required by modules with no onboard
    // oscillator/reset-default (e.g. the Waveshare OV5640 module this
    // project targets): cam_mclk is the sensor's master clock input
    // (24MHz, generated on-chip -- see clk_gen_mclk.v), cam_rst_n is its
    // active-low RESET pin, cam_pwdn is its active-high PWDN pin. If your
    // camera module ties these off on its own PCB (onboard crystal +
    // pull resistors), these outputs are harmless to leave unconnected.
    output wire         cam_mclk,
    output wire         cam_rst_n,
    output wire         cam_pwdn,

    // native GPDI/TMDS output
    output wire [3:0]   gpdi_dp,      // [0]=Blue [1]=Green [2]=Red [3]=Clock

    // UART hardware-status debug output (115200 8N1) -- see uart_debug.v
    output wire         uart_tx,

    // Real "image data is being captured" indicator -- lights up while
    // pixel capture is actively happening (stretched to stay visibly lit
    // during continuous capture, see uart_debug.v). Wire an external LED
    // (+ series resistor) here if you want a physical indicator; the
    // OV5640 module's own onboard LEDs next to the lens are NOT usable
    // for this -- they're fixed power-on indicators on essentially every
    // OV5640 breakout, with no GPIO/register connection at all.
    output wire         cap_led,

    output wire [4:0]   led
);

    // ------------------------------------------------------------------
    // Video mode: fixed 1280x720 @ 60Hz (CEA-861 VIC 4)
    // ------------------------------------------------------------------
    localparam integer H_ACTIVE = 1280;
    localparam integer H_FRONT  = 110;
    localparam integer H_SYNC   = 40;
    localparam integer H_BACK   = 220;
    localparam integer V_ACTIVE = 720;
    localparam integer V_FRONT  = 5;
    localparam integer V_SYNC   = 5;
    localparam integer V_BACK   = 20;
    localparam integer BAR_LOG2 = 7;

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
    // Camera MCLK generation + power-up/reset sequencing
    // ------------------------------------------------------------------
    wire mclk_locked;

    clk_gen_mclk u_mclk_pll (
        .clk_in(clk), .cam_mclk(cam_mclk), .locked(mclk_locked)
    );

    wire cam_seq_done;

    cam_power_sequencer u_cam_seq (
        .clk(cam_mclk), .mclk_locked(mclk_locked), .rst_async(rst_btn),
        .cam_pwdn(cam_pwdn), .cam_rst_n(cam_rst_n), .seq_done(cam_seq_done)
    );

    // synchronize seq_done (cam_mclk domain) into clk_pixel domain, where
    // the I2C config sequencer below actually runs
    reg [1:0] seq_done_sync;
    always @(posedge clk_pixel or posedge rst_pixel)
        if (rst_pixel) seq_done_sync <= 2'b00;
        else           seq_done_sync <= {seq_done_sync[0], cam_seq_done};
    wire cam_seq_done_px = seq_done_sync[1];

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

    // Rolling window of the last 4 bytes actually captured off cam_d[7:0]
    // (oldest byte in the high 8 bits), fed to uart_debug.v's RAW field --
    // direct visibility into real sensor data content for hardware
    // bring-up. See uart_debug.v's header comment for how to read it.
    reg [31:0] raw_byte_window;
    always @(posedge cam_pclk) begin
        if (byte_valid) raw_byte_window <= {raw_byte_window[23:0], byte_data};
    end

    wire cam_pixel_valid;
    wire [23:0] cam_rgb;

    pixel_formatter #(.FORMAT(CAMERA_FORMAT), .BYTE_SWAP(BYTE_SWAP), .RB_SWAP(RB_SWAP)) u_formatter (
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
        .DEPTH(4096),
        .PREFILL_WORDS(2048)
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
    // Camera I2C configuration sequencer -- starts once BOTH the video
    // pixel-clock PLL has locked AND the camera's own power-up/reset
    // sequence has completed (cam_seq_done_px). Starting I2C before the
    // sensor is out of reset/settled is exactly the kind of thing that
    // silently produces all-NACK behaviour on modules like the OV5640.
    // ------------------------------------------------------------------
    wire i2c_start, i2c_busy, i2c_done, i2c_nack;
    wire [15:0] i2c_reg_addr;
    wire [7:0]  i2c_reg_data;
    wire cfg_done;

    wire both_ready = pll_locked & cam_seq_done_px;
    reg  both_ready_d1, both_ready_d2;
    always @(posedge clk_pixel) begin
        both_ready_d1 <= both_ready;
        both_ready_d2 <= both_ready_d1;
    end
    wire cfg_go = both_ready_d1 & ~both_ready_d2; // one-shot pulse

    i2c_master #(
        .CLK_FREQ_HZ(74_286_000), .I2C_FREQ_HZ(100_000),
        .DEV_ADDR7(I2C_DEV_ADDR7), .ADDR_BYTES(ADDR_BYTES)
    ) u_i2c (
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

    // ------------------------------------------------------------------
    // UART hardware-status debug output. Driven from `clk` (the always-
    // running 50MHz board oscillator) rather than clk_pixel, so status
    // reporting -- including the startup banner -- works even before the
    // pixel PLL locks; every status input is resynchronized internally.
    // ------------------------------------------------------------------
    uart_debug #(
        .CLK_FREQ_HZ(50_000_000), .BAUD(115200), .TICK_HZ(1)
    ) u_uart_debug (
        .clk(clk), .rst(rst_btn),
        .pll_locked(pll_locked), .mclk_locked(mclk_locked),
        .cam_seq_done(cam_seq_done), .cfg_done(cfg_done),
        .i2c_nack(i2c_nack), .buf_ready(buf_ready), .pattern_sel(pattern_sel),
        .cam_vsync(cam_vsync), .cam_pixel_valid(cam_pixel_valid),
        .raw_bytes(raw_byte_window),
        .tx(uart_tx), .cap_led(cap_led)
    );

endmodule

`default_nettype wire
