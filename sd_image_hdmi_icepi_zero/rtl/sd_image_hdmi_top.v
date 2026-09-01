`timescale 1ns/1ps
// sd_image_hdmi_top : top level for the IcePi Zero (LFE5U-25F-6BG256C).
//
// Reads IMAGE.RAW from an SD card (FAT16/32, SPI mode) into a 160x120
// RGB565 on-chip framebuffer once at power-on, then continuously scans
// it out over the board's HDMI port as 640x480@~59.5Hz (4x pixel
// replication), TMDS-encoded on general-purpose ECP5 I/O (this device
// has no dedicated HDMI PHY).
//
// Three clock domains:
//   - clk       (50 MHz, direct from the board oscillator)  : unused directly by any output in this design, only feeds the two PLLs below
//   - sys_clk   (20 MHz, from icepi_clk_wiz_sys)             : SD/SPI/FAT/image-load/status-LED logic
//   - pix_clk   (25 MHz) + shift_clk (125 MHz), both from icepi_clk_wiz_video, same PLL : HDMI video pipeline
//
// sd_cd_n / card-detect does not exist in this design, matching the
// sd_uart_top project: SD init starts unconditionally on reset release.
//
// Port names below (button[], gpdi_dp[]/gpdi_dn[]) match the board's
// real master LPF exactly (board-owner-provided in this session) -
// LOCATE COMP entries in an LPF must match actual net/port names in
// the design, so these are no longer this project's own naming choice.
// button[0] is the reset button (button[1] is a second board button,
// unused by this design, left as an unread input). gpdi_dp/dn[3] is
// the TMDS clock pair; gpdi_dp/dn[0..2] are the TMDS data pairs
// (blue/green/red respectively) - both per the master LPF's own inline
// comments and confirmed by its IO_TYPE=LVCMOS33D (Lattice's
// explicit-both-pins differential I/O standard, matching this design's
// OLVDS-based hdmi_out.v exactly).
module sd_image_hdmi_top (
    input  wire       clk,         // 50 MHz board oscillator
    input  wire [1:0] button,      // button[0] = reset, active-low (pulled up); button[1] unused

    output wire       sd_clk,
    output wire       sd_mosi,
    input  wire       sd_miso,
    output wire       sd_csn,

    output wire [3:0] led,

    output wire [3:0] gpdi_dp, gpdi_dn   // [3]=TMDS clock, [0]=blue, [1]=green, [2]=red
);

    wire button0_n = button[0];

    // -----------------------------------------------------------------
    // Clock generation: two independent PLLs from the one oscillator.
    // -----------------------------------------------------------------
    wire sys_clk, sys_pll_locked;
    icepi_clk_wiz_sys u_clk_wiz_sys (
        .CLKI  (clk),
        .CLKOP (sys_clk),
        .LOCK  (sys_pll_locked)
    );

    wire pix_clk, shift_clk, vid_pll_locked;
    icepi_clk_wiz_video u_clk_wiz_video (
        .CLKI  (clk),
        .CLKOP (pix_clk),
        .CLKOS (shift_clk),
        .LOCK  (vid_pll_locked)
    );

    wire rst_sys, rst_sys_n;
    clk_reset_gen u_clk_reset_sys (
        .clk        (sys_clk),
        .pll_locked (sys_pll_locked),
        .ext_rst_n  (button0_n),
        .rst_sys    (rst_sys),
        .rst_sys_n  (rst_sys_n)
    );

    wire rst_pix, rst_pix_n;
    clk_reset_gen u_clk_reset_pix (
        .clk        (pix_clk),
        .pll_locked (vid_pll_locked),
        .ext_rst_n  (button0_n),
        .rst_sys    (rst_pix),
        .rst_sys_n  (rst_pix_n)
    );

    // -----------------------------------------------------------------
    // Shared physical SPI bus - identical init_done-selected mux
    // arbitration as the sd_uart_top project (sd_spi_init and
    // sd_block_read never run concurrently).
    // -----------------------------------------------------------------
    wire        init_spi_start;
    wire [7:0]  init_spi_tx;
    wire [15:0] init_spi_div;
    wire        init_cs_n;

    wire        blk_spi_start;
    wire [7:0]  blk_spi_tx;
    wire [15:0] blk_spi_div;
    wire        blk_cs_n;

    wire        spi_busy, spi_done;
    wire [7:0]  spi_rx;

    wire        init_done_w;

    wire        spi_start_mux = init_done_w ? blk_spi_start : init_spi_start;
    wire [7:0]  spi_tx_mux    = init_done_w ? blk_spi_tx    : init_spi_tx;
    wire [15:0] spi_div_mux   = init_done_w ? blk_spi_div   : init_spi_div;
    wire        cs_n_mux      = init_done_w ? blk_cs_n      : init_cs_n;

    assign sd_csn = cs_n_mux;

    spi_master #(.CLK_FREQ_HZ(20_000_000)) u_spi_master (
        .clk     (sys_clk),
        .rst     (rst_sys),
        .clk_div (spi_div_mux),
        .start   (spi_start_mux),
        .tx_data (spi_tx_mux),
        .rx_data (spi_rx),
        .busy    (spi_busy),
        .done    (spi_done),
        .sclk    (sd_clk),
        .mosi    (sd_mosi),
        .miso    (sd_miso)
    );

    // -----------------------------------------------------------------
    // SD bring-up and block read
    // -----------------------------------------------------------------
    wire init_ok_w, sdhc_w, init_error_w, init_busy_w;

    sd_spi_init u_sd_spi_init (
        .clk         (sys_clk),
        .rst         (rst_sys),
        .spi_start   (init_spi_start),
        .spi_tx_data (init_spi_tx),
        .spi_clk_div (init_spi_div),
        .spi_busy    (spi_busy),
        .spi_done    (spi_done),
        .spi_rx_data (spi_rx),
        .cs_n        (init_cs_n),
        .init_done   (init_done_w),
        .init_ok     (init_ok_w),
        .sdhc        (sdhc_w),
        .error       (init_error_w),
        .busy        (init_busy_w)
    );

    wire        blk_req_start;
    wire [31:0] blk_req_lba;
    wire        blk_busy_w, blk_done_w, blk_error_w;
    wire        blk_data_valid_w;
    wire [7:0]  blk_data_w;

    sd_block_read u_sd_block_read (
        .clk         (sys_clk),
        .rst         (rst_sys),
        .sdhc        (sdhc_w),
        .start       (blk_req_start),
        .lba         (blk_req_lba),
        .busy        (blk_busy_w),
        .done        (blk_done_w),
        .error       (blk_error_w),
        .data_valid  (blk_data_valid_w),
        .data_out    (blk_data_w),
        .spi_start   (blk_spi_start),
        .spi_tx_data (blk_spi_tx),
        .spi_clk_div (blk_spi_div),
        .spi_busy    (spi_busy),
        .spi_done    (spi_done),
        .spi_rx_data (spi_rx),
        .cs_n        (blk_cs_n)
    );

    // -----------------------------------------------------------------
    // FAT mount / search / IMAGE.RAW read
    // -----------------------------------------------------------------
    wire        fat_start_w;
    wire        mount_done_w, mount_ok_w, file_found_w, file_done_w, fat_error_w;
    wire        fat_data_valid_w;
    wire [7:0]  fat_data_w;

    fat_reader u_fat_reader (
        .clk             (sys_clk),
        .rst             (rst_sys),
        .start           (fat_start_w),
        .blk_start       (blk_req_start),
        .blk_lba         (blk_req_lba),
        .blk_done        (blk_done_w),
        .blk_error       (blk_error_w),
        .blk_data_valid  (blk_data_valid_w),
        .blk_data_in     (blk_data_w),
        .mount_done      (mount_done_w),
        .mount_ok        (mount_ok_w),
        .file_found      (file_found_w),
        .file_done       (file_done_w),
        .error           (fat_error_w),
        .data_valid      (fat_data_valid_w),
        .data_out        (fat_data_w)
    );

    // -----------------------------------------------------------------
    // Image loader -> framebuffer
    // -----------------------------------------------------------------
    wire         img_loaded_w, img_error_w, img_busy_w;
    wire         fb_wr_en_w;
    wire [14:0]  fb_wr_addr_w;
    wire [15:0]  fb_wr_data_w;

    img_loader u_img_loader (
        .clk             (sys_clk),
        .rst             (rst_sys),
        .init_done       (init_done_w),
        .init_ok         (init_ok_w),
        .fat_start       (fat_start_w),
        .mount_done      (mount_done_w),
        .mount_ok        (mount_ok_w),
        .file_found      (file_found_w),
        .file_done       (file_done_w),
        .fat_error       (fat_error_w),
        .file_data       (fat_data_w),
        .file_data_valid (fat_data_valid_w),
        .fb_wr_en        (fb_wr_en_w),
        .fb_wr_addr      (fb_wr_addr_w),
        .fb_wr_data      (fb_wr_data_w),
        .loaded          (img_loaded_w),
        .error           (img_error_w),
        .busy            (img_busy_w)
    );

    wire [14:0] fb_rd_addr_w;
    wire [15:0] fb_rd_data_w;

    framebuffer u_framebuffer (
        .wr_clk   (sys_clk),
        .wr_en    (fb_wr_en_w),
        .wr_addr  (fb_wr_addr_w),
        .wr_data  (fb_wr_data_w),
        .rd_clk   (pix_clk),
        .rd_addr  (fb_rd_addr_w),
        .rd_data  (fb_rd_data_w)
    );

    // -----------------------------------------------------------------
    // sys_clk -> pix_clk CDC for the "image loaded" status flag: a
    // plain 2-flop synchronizer is sufficient (not a toggle handshake)
    // because this is a level, not a pulse - img_loaded_w rises exactly
    // once per power-on session and then stays high, so a synchronizer
    // only needs to resolve metastability on that single transition,
    // with no risk of ever missing a pulse.
    // -----------------------------------------------------------------
    reg img_loaded_s0, img_loaded_s1;
    always @(posedge pix_clk or posedge rst_pix) begin
        if (rst_pix) begin
            img_loaded_s0 <= 1'b0;
            img_loaded_s1 <= 1'b0;
        end else begin
            img_loaded_s0 <= img_loaded_w;
            img_loaded_s1 <= img_loaded_s0;
        end
    end

    // -----------------------------------------------------------------
    // HDMI output
    // -----------------------------------------------------------------
    hdmi_out u_hdmi_out (
        .pix_clk     (pix_clk),
        .shift_clk   (shift_clk),
        .rst         (rst_pix),
        .fb_loaded   (img_loaded_s1),
        .fb_rd_addr  (fb_rd_addr_w),
        .fb_rd_data  (fb_rd_data_w),
        .hdmi_clk_p  (gpdi_dp[3]), .hdmi_clk_n (gpdi_dn[3]),
        .hdmi_d0_p   (gpdi_dp[0]), .hdmi_d0_n  (gpdi_dn[0]),
        .hdmi_d1_p   (gpdi_dp[1]), .hdmi_d1_n  (gpdi_dn[1]),
        .hdmi_d2_p   (gpdi_dp[2]), .hdmi_d2_n  (gpdi_dn[2])
    );

    // -----------------------------------------------------------------
    // LED diagnostics (sys_clk domain)
    // -----------------------------------------------------------------
    wire spi_activity_w = init_busy_w || blk_busy_w;
    wire error_w         = init_error_w || img_error_w;

    status_led u_status_led (
        .clk          (sys_clk),
        .rst          (rst_sys),
        .sd_init_ok   (init_ok_w),
        .spi_activity (spi_activity_w),
        .img_loaded   (img_loaded_w),
        .error_in     (error_w),
        .led          (led)
    );

endmodule
