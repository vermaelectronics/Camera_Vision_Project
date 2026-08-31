// ============================================================================
// image_display_top.v -- IcePi-Zero bring-up #3: load a 160x120 24bpp BMP
// off a FAT16 SD card and display it over native GPDI/HDMI, letterbox-free,
// nearest-neighbor upscaled to fill a real 1280x720p60 signal (8x horizontal,
// 6x vertical -- both exact integer factors, chosen specifically so IMG_W
// and IMG_H divide 1280/720 cleanly, avoiding any fractional-scaling logic).
// ----------------------------------------------------------------------------
// Storage stack (sdcard_spi.v, fat16_reader.v) is reused byte-for-byte from
// the sibling 02_sdcard_text_reader sub-project -- a FAT16 file's contents
// are just bytes in order, regardless of whether a UART or a BMP parser is
// the one making sense of them.
//
// Reused, unmodified, from dvp_camera_hdmi_pipeline (the camera project):
// clk_gen_dvi.v (50MHz -> 74.286MHz pixel clock + 5x SERDES clock),
// video_timing_gen.v (1280x720p60 timing), tmds_encoder.v,
// tmds_serial_gearbox.v (native GPDI serialization), test_pattern_gen.v
// (shown until an image has actually loaded, or if loading fails), and
// dp_line_ram.v (here sized as a 160x120 RGB565 frame buffer instead of one
// video line -- same true-dual-port, independent-clocks building block).
//
// LEDs: [0]=sd_ready [1]=frame busy loading [2]=error (sticky) [3]=image
// displayed (sticky, set once the frame buffer is fully loaded) [4]=1Hz
// heartbeat. button[0]=reset, button[1]=reload the image from the SD card.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module image_display_top #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter [8*11-1:0] FILENAME = "IMAGE   BMP",
    parameter integer IMG_W = 160,
    parameter integer IMG_H = 120
) (
    input  wire        clk,        // 50MHz board oscillator (site M1)
    input  wire  [1:0] button,     // active-low; [0]=reset, [1]=reload image
    output wire  [4:0] led,

    output wire  [3:0] gpdi_dp,    // native GPDI/TMDS: [0]=B [1]=G [2]=R [3]=clock

    output wire         sd_sclk,
    output wire         sd_mosi,
    input  wire          sd_miso,
    output wire          sd_cs_n
);

    wire rst = ~button[0];

    // ---- button[1] debounce + rising-edge detect (active-low: press = 0) --
    localparam integer DEBOUNCE_CYCLES = CLK_FREQ_HZ / 1000; // ~1ms
    reg [1:0] btn_sync;
    reg       btn_stable;
    reg [$clog2(DEBOUNCE_CYCLES):0] debounce_cnt;
    reg       btn_stable_prev;
    wire      btn1_pressed_edge;

    always @(posedge clk) begin
        btn_sync <= {btn_sync[0], button[1]};
        if (btn_sync[1] == btn_stable) begin
            debounce_cnt <= 0;
        end else if (debounce_cnt == DEBOUNCE_CYCLES) begin
            btn_stable   <= btn_sync[1];
            debounce_cnt <= 0;
        end else begin
            debounce_cnt <= debounce_cnt + 1'b1;
        end
        btn_stable_prev <= btn_stable;
    end
    assign btn1_pressed_edge = btn_stable_prev && !btn_stable;

    // ---- SD SPI controller ----
    wire sd_ready_w, sd_err_w;
    wire [1:0] sd_card_type;
    wire sd_read_req_w;
    wire [31:0] sd_block_addr_w;
    wire sd_rd_busy_w, sd_rd_done_w, sd_rd_error_w;
    wire sd_byte_valid_w;
    wire [7:0] sd_byte_data_w;
    wire [8:0] sd_byte_index_w;

    sdcard_spi #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) sd (
        .clk(clk), .rst(rst),
        .sd_sclk(sd_sclk), .sd_mosi(sd_mosi), .sd_miso(sd_miso), .sd_cs_n(sd_cs_n),
        .ready(sd_ready_w), .error(sd_err_w), .card_type(sd_card_type),
        .read_req(sd_read_req_w), .block_addr(sd_block_addr_w),
        .rd_busy(sd_rd_busy_w), .rd_done(sd_rd_done_w), .rd_error(sd_rd_error_w),
        .byte_valid(sd_byte_valid_w), .byte_data(sd_byte_data_w), .byte_index(sd_byte_index_w)
    );

    reg sd_ready_prev;
    wire auto_start = sd_ready_w && !sd_ready_prev;
    always @(posedge clk) sd_ready_prev <= rst ? 1'b0 : sd_ready_w;

    wire fat_start_w = auto_start || btn1_pressed_edge;

    // ---- FAT16 reader (no FIFO needed -- bmp_frame_loader below always
    // accepts a byte the same cycle it's offered, no backpressure required) --
    wire fat_busy_w, fat_done_w, fat_error_w;
    wire [31:0] fat_file_size_w;
    wire fat_data_valid_w;
    wire [7:0] fat_data_byte_w;

    fat16_reader #(.FILENAME(FILENAME)) fat (
        .clk(clk), .rst(rst),
        .sd_read_req(sd_read_req_w), .sd_block_addr(sd_block_addr_w),
        .sd_rd_busy(sd_rd_busy_w), .sd_rd_done(sd_rd_done_w), .sd_rd_error(sd_rd_error_w),
        .sd_byte_valid(sd_byte_valid_w), .sd_byte_data(sd_byte_data_w), .sd_byte_index(sd_byte_index_w),
        .sd_ready(sd_ready_w),
        .start(fat_start_w), .fifo_ready_for_block(1'b1),
        .busy(fat_busy_w), .done(fat_done_w), .error(fat_error_w),
        .file_size(fat_file_size_w),
        .data_valid(fat_data_valid_w), .data_byte(fat_data_byte_w)
    );

    // ---- BMP parse + frame buffer write ----
    wire bmp_busy_w, bmp_done_w, bmp_error_w;
    wire fb_wr_en_w;
    wire [14:0] fb_wr_addr_w;
    wire [15:0] fb_wr_data_w;

    bmp_frame_loader #(.IMG_W(IMG_W), .IMG_H(IMG_H)) bmp (
        .clk(clk), .rst(rst),
        .data_valid(fat_data_valid_w), .data_byte(fat_data_byte_w),
        .file_done(fat_done_w), .file_error(fat_error_w),
        .busy(bmp_busy_w), .done(bmp_done_w), .error(bmp_error_w),
        .fb_wr_en(fb_wr_en_w), .fb_wr_addr(fb_wr_addr_w), .fb_wr_data(fb_wr_data_w)
    );

    // ---- sticky status ----
    reg error_latched, img_ready;
    always @(posedge clk) begin
        if (rst || fat_start_w) begin
            error_latched <= 1'b0;
            img_ready     <= 1'b0;
        end else begin
            if (fat_error_w || bmp_error_w) error_latched <= 1'b1;
            if (bmp_done_w)                 img_ready     <= 1'b1;
        end
    end

    localparam integer HALF_PERIOD = CLK_FREQ_HZ / 2;
    reg [$clog2(HALF_PERIOD)-1:0] hb_cnt;
    reg hb_r;
    always @(posedge clk) begin
        if (rst) begin
            hb_cnt <= 0; hb_r <= 1'b0;
        end else if (hb_cnt == HALF_PERIOD - 1) begin
            hb_cnt <= 0; hb_r <= ~hb_r;
        end else begin
            hb_cnt <= hb_cnt + 1'b1;
        end
    end

    assign led[0] = sd_ready_w;
    assign led[1] = fat_busy_w || bmp_busy_w;
    assign led[2] = error_latched || sd_err_w;
    assign led[3] = img_ready;
    assign led[4] = hb_r;

    // ------------------------------------------------------------------
    // Frame buffer: 160x120 RGB565, true dual-port -- write side above
    // (clk domain, one load per file read), read side below (clk_pixel
    // domain, every frame, forever). No CDC hazard: reads never begin
    // showing the frame until img_ready (synced below) goes high, by
    // which point the write side is idle and stays idle until the next
    // reload, so there's no read-chases-write race to handle.
    // ------------------------------------------------------------------
    wire clk_pixel, clk_eclk, pll_locked;

    clk_gen_dvi u_pll (
        .clk_in(clk), .clk_pixel(clk_pixel), .clk_eclk(clk_eclk), .locked(pll_locked)
    );

    reg [3:0] rst_pixel_sr = 4'hF;
    always @(posedge clk_pixel or negedge pll_locked)
        if (!pll_locked) rst_pixel_sr <= 4'hF;
        else             rst_pixel_sr <= {rst_pixel_sr[2:0], rst};
    wire rst_pixel = rst_pixel_sr[3];

    // sync the (slow-changing, level) img_ready flag into clk_pixel
    reg [1:0] img_ready_sync;
    always @(posedge clk_pixel or posedge rst_pixel)
        if (rst_pixel) img_ready_sync <= 2'b00;
        else           img_ready_sync <= {img_ready_sync[0], img_ready};
    wire img_ready_px = img_ready_sync[1];

    wire hsync, vsync, out_de;
    wire [15:0] x, y;

    video_timing_gen #(
        .H_ACTIVE(1280), .H_FRONT(110), .H_SYNC(40), .H_BACK(220),
        .V_ACTIVE(720),  .V_FRONT(5),   .V_SYNC(5),  .V_BACK(20),
        .HS_POL(1'b1), .VS_POL(1'b1)
    ) u_timing (
        .clk(clk_pixel), .rst(rst_pixel),
        .hsync(hsync), .vsync(vsync), .de(out_de), .frame_start(),
        .x(x), .y(y)
    );

    // ---- nearest-neighbor downscale address (see nn_scale_addr.v) ----
    wire [7:0] small_x, small_y;

    nn_scale_addr #(.X_SHIFT(3), .Y_SCALE(720 / IMG_H)) u_scale (
        .clk(clk_pixel), .rst(rst_pixel), .de(out_de), .x(x), .y(y),
        .small_x(small_x), .small_y(small_y)
    );

    wire [14:0] fb_rd_addr = small_y * IMG_W + small_x;
    wire [15:0] fb_rd_data;

    dp_line_ram #(.DEPTH(IMG_W*IMG_H), .DATA_W(16)) frame_buf (
        .wr_clk(clk), .wr_en(fb_wr_en_w), .wr_addr(fb_wr_addr_w), .wr_data(fb_wr_data_w),
        .rd_clk(clk_pixel), .rd_en(1'b1), .rd_addr(fb_rd_addr), .rd_data(fb_rd_data)
    );

    // RGB565 -> RGB888: replicate each field's top bits into its own low
    // bits (standard bit-replication expansion, no clipping/rounding error
    // at either end -- 0x00->0x00, max->0xFF).
    wire [4:0] fb_r5 = fb_rd_data[15:11];
    wire [5:0] fb_g6 = fb_rd_data[10:5];
    wire [4:0] fb_b5 = fb_rd_data[4:0];
    wire [23:0] fb_rgb = { fb_r5, fb_r5[4:2], fb_g6, fb_g6[5:4], fb_b5, fb_b5[4:2] };

    // Registered one more cycle below (fb_rgb_r) -- real STA (nextpnr-ecp5)
    // showed the raw combinational chain from dp_line_ram's own registered
    // output, through this 565->888 expansion, through the pixel_rgb mux,
    // straight into tmds_encoder's internal XOR-chain logic, missed the
    // 74.29MHz budget outright (dp_line_ram's block-RAM clock-to-Q delay
    // alone eats a large slice of one period). Same fix, same reasoning,
    // as dvp_camera_hdmi_pipeline's own top module: widen the realignment
    // from one pipeline stage to two.
    reg [23:0] fb_rgb_r;
    always @(posedge clk_pixel) fb_rgb_r <= fb_rgb;

    wire [23:0] pattern_rgb;
    test_pattern_gen #(.H_ACTIVE(1280), .V_ACTIVE(720), .BAR_W_LOG2(7))
        u_pattern (.x(x), .y(y), .rgb(pattern_rgb));

    // ---- two-cycle realign: dp_line_ram's registered read (1 cycle) plus
    // fb_rgb_r above (1 more cycle) = 2 cycles from (x,y) to a usable pixel
    // -- hsync/vsync/de/pattern_rgb need the same two stages to land back
    // on the same pixel (mirrors dvp_camera_hdmi_top.v's own two-stage
    // realignment, added there for the identical reason). ----
    reg        hsync_r1, vsync_r1, de_r1;
    reg [23:0] pattern_rgb_r1;
    reg        img_ready_px_r1;
    reg        hsync_r, vsync_r, de_r;
    reg [23:0] pattern_rgb_r;
    reg        img_ready_px_r;
    always @(posedge clk_pixel) begin
        hsync_r1        <= hsync;
        vsync_r1        <= vsync;
        de_r1           <= out_de;
        pattern_rgb_r1  <= pattern_rgb;
        img_ready_px_r1 <= img_ready_px;

        hsync_r        <= hsync_r1;
        vsync_r        <= vsync_r1;
        de_r           <= de_r1;
        pattern_rgb_r  <= pattern_rgb_r1;
        img_ready_px_r <= img_ready_px_r1;
    end

    wire [23:0] pixel_rgb = img_ready_px_r ? fb_rgb_r : pattern_rgb_r;

    wire [9:0] tmds_r, tmds_g, tmds_b;

    tmds_encoder u_enc_r (.clk(clk_pixel), .rst(rst_pixel), .din(pixel_rgb[23:16]), .ctrl(2'b00),             .de(de_r), .tmds(tmds_r));
    tmds_encoder u_enc_g (.clk(clk_pixel), .rst(rst_pixel), .din(pixel_rgb[15:8]),  .ctrl(2'b00),             .de(de_r), .tmds(tmds_g));
    tmds_encoder u_enc_b (.clk(clk_pixel), .rst(rst_pixel), .din(pixel_rgb[7:0]),   .ctrl({vsync_r,hsync_r}), .de(de_r), .tmds(tmds_b));

    tmds_serial_gearbox u_ser (
        .clk_pixel(clk_pixel), .clk_eclk(clk_eclk), .rst_pixel(rst_pixel),
        .tmds_r(tmds_r), .tmds_g(tmds_g), .tmds_b(tmds_b),
        .gpdi_dp(gpdi_dp)
    );

endmodule

`default_nettype wire
