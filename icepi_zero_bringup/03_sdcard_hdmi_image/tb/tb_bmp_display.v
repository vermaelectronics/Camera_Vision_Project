// ============================================================================
// tb_bmp_display.v -- mounts the synthetic FAT16 volume built by
// gen_bmp_fat16_image.py, reads IMAGE.BMP through sdcard_spi + fat16_reader
// (same modules as the sibling 02_sdcard_text_reader sub-project) +
// bmp_frame_loader, and checks every one of the 19200 pixels written into
// the frame buffer against the golden RGB565 values computed independently
// in Python -- including the 113-cluster FAT chain follow this file's
// size requires.
// ============================================================================
`timescale 1ns / 1ps

module tb_bmp_display;

    localparam integer IMG_W = 160;
    localparam integer IMG_H = 120;

    reg clk = 0;
    always #10 clk = ~clk; // 50MHz

    reg rst = 1;

    wire sclk, mosi, miso, cs_n;

    sd_card_model #(.NUM_BLOCKS(120)) card (
        .sclk(sclk), .mosi(mosi), .miso(miso), .cs_n(cs_n)
    );
    initial $readmemh("sd_image_bmp.hex", card.blocks);

    wire ready_w, err_w;
    wire [1:0] card_type_w;
    wire read_req_w;
    wire [31:0] block_addr_w;
    wire rd_busy_w, rd_done_w, rd_error_w;
    wire byte_valid_w;
    wire [7:0] byte_data_w;
    wire [8:0] byte_index_w;

    sdcard_spi #(.CLK_FREQ_HZ(50_000_000)) sd (
        .clk(clk), .rst(rst),
        .sd_sclk(sclk), .sd_mosi(mosi), .sd_miso(miso), .sd_cs_n(cs_n),
        .ready(ready_w), .error(err_w), .card_type(card_type_w),
        .read_req(read_req_w), .block_addr(block_addr_w),
        .rd_busy(rd_busy_w), .rd_done(rd_done_w), .rd_error(rd_error_w),
        .byte_valid(byte_valid_w), .byte_data(byte_data_w), .byte_index(byte_index_w)
    );

    reg fat_start_r;
    wire fat_busy_w, fat_done_w, fat_error_w;
    wire [31:0] fat_file_size_w;
    wire fat_data_valid_w;
    wire [7:0] fat_data_byte_w;

    fat16_reader #(.FILENAME("IMAGE   BMP")) fat (
        .clk(clk), .rst(rst),
        .sd_read_req(read_req_w), .sd_block_addr(block_addr_w),
        .sd_rd_busy(rd_busy_w), .sd_rd_done(rd_done_w), .sd_rd_error(rd_error_w),
        .sd_byte_valid(byte_valid_w), .sd_byte_data(byte_data_w), .sd_byte_index(byte_index_w),
        .sd_ready(ready_w),
        .start(fat_start_r), .fifo_ready_for_block(1'b1),
        .busy(fat_busy_w), .done(fat_done_w), .error(fat_error_w),
        .file_size(fat_file_size_w),
        .data_valid(fat_data_valid_w), .data_byte(fat_data_byte_w)
    );

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

    wire [15:0] fb_rd_data_unused;
    dp_line_ram #(.DEPTH(IMG_W*IMG_H), .DATA_W(16)) frame_buf (
        .wr_clk(clk), .wr_en(fb_wr_en_w), .wr_addr(fb_wr_addr_w), .wr_data(fb_wr_data_w),
        .rd_clk(clk), .rd_en(1'b0), .rd_addr(15'd0), .rd_data(fb_rd_data_unused)
    );

    reg [15:0] golden [0:IMG_W*IMG_H-1];
    initial $readmemh("golden_frame565.hex", golden);

    integer errors;
    integer i;

    initial begin
        $dumpfile("tb_bmp_display.vcd");
        $dumpvars(0, tb_bmp_display);

        errors = 0;
        fat_start_r = 1'b0;

        repeat (5) @(posedge clk);
        rst = 0;

        wait (ready_w || err_w);
        if (err_w) begin
            $display("ERROR: sdcard_spi init failed");
            errors = errors + 1;
        end

        @(posedge clk);
        fat_start_r = 1'b1;
        @(posedge clk);
        fat_start_r = 1'b0;

        wait (bmp_done_w || bmp_error_w || fat_error_w);
        @(posedge clk);

        if (bmp_error_w || fat_error_w) begin
            $display("ERROR: bmp_done_w=%b bmp_error_w=%b fat_error_w=%b", bmp_done_w, bmp_error_w, fat_error_w);
            errors = errors + 1;
        end else begin
            $display("BMP load complete, file_size=%0d", fat_file_size_w);
            for (i = 0; i < IMG_W*IMG_H; i = i + 1) begin
                if (frame_buf.mem[i] !== golden[i]) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("ERROR: pixel %0d mismatch: got %04h expected %04h",
                                  i, frame_buf.mem[i], golden[i]);
                end
            end
        end

        if (errors == 0)
            $display("TB_BMP_DISPLAY: PASS");
        else
            $display("TB_BMP_DISPLAY: FAIL (%0d errors)", errors);

        $finish;
    end

    initial begin
        #200_000_000;
        $display("TB_BMP_DISPLAY: WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
