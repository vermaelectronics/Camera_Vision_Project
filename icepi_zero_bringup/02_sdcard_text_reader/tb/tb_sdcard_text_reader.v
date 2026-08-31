// ============================================================================
// tb_sdcard_text_reader.v -- integration test: sdcard_spi + fat16_reader
// against the behavioral sd_card_model, mounting a synthetic FAT16 volume
// (built by gen_fat16_image.py) and reading HELLO.TXT off it. Checks the
// full init handshake (idle -> CMD8 -> ACMD41 loop -> ready, card_type
// detected as SDHC), the FAT16 mount (boot sector parsed, file found by
// name in the root directory, correct size), and that every one of the
// streamed data bytes exactly matches the source file, including the
// cluster-chain follow from cluster 2 to cluster 3 partway through.
// ============================================================================
`timescale 1ns / 1ps

module tb_sdcard_text_reader;

    reg clk = 0;
    always #10 clk = ~clk; // 50MHz

    reg rst = 1;

    // ---- SPI bus between sdcard_spi (master) and sd_card_model (slave) ----
    wire sclk, mosi, miso, cs_n;

    sd_card_model #(.NUM_BLOCKS(8)) card (
        .sclk(sclk), .mosi(mosi), .miso(miso), .cs_n(cs_n)
    );

    initial $readmemh("sd_image.hex", card.blocks);

    wire ready_w, err_w;
    wire [1:0] card_type_w;
    wire read_req_w;
    wire [31:0] block_addr_w;
    wire rd_busy_w, rd_done_w, rd_error_w;
    wire byte_valid_w;
    wire [7:0] byte_data_w;
    wire [8:0] byte_index_w;

    sdcard_spi #(.CLK_FREQ_HZ(50_000_000)) dut_sd (
        .clk(clk), .rst(rst),
        .sd_sclk(sclk), .sd_mosi(mosi), .sd_miso(miso), .sd_cs_n(cs_n),
        .ready(ready_w), .error(err_w), .card_type(card_type_w),
        .read_req(read_req_w), .block_addr(block_addr_w),
        .rd_busy(rd_busy_w), .rd_done(rd_done_w), .rd_error(rd_error_w),
        .byte_valid(byte_valid_w), .byte_data(byte_data_w), .byte_index(byte_index_w)
    );

    // fat16_reader talks to sdcard_spi directly (fifo_ready always granted --
    // this testbench isn't exercising the FIFO backpressure path).
    reg fat_start_r;
    wire fat_busy_w, fat_done_w, fat_error_w;
    wire [31:0] fat_file_size_w;
    wire fat_data_valid_w;
    wire [7:0] fat_data_byte_w;

    fat16_reader #(.FILENAME("HELLO   TXT")) dut_fat (
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

    reg [7:0] expected [0:1023];
    integer   expected_len;
    integer   rx_i;
    integer   errors;
    integer   fi;

    initial begin
        // sized generously; gen_fat16_image.py currently produces 600 bytes
        expected_len = 600;
        $readmemh("expected_content.hex", expected, 0, expected_len - 1);
    end

    always @(posedge clk) begin
        if (fat_data_valid_w) begin
            if (rx_i >= expected_len) begin
                $display("ERROR: received more bytes than expected (byte %0d = 0x%02x)", rx_i, fat_data_byte_w);
                errors = errors + 1;
            end else if (fat_data_byte_w !== expected[rx_i]) begin
                $display("ERROR: byte %0d mismatch: got 0x%02x expected 0x%02x",
                          rx_i, fat_data_byte_w, expected[rx_i]);
                errors = errors + 1;
            end
            rx_i = rx_i + 1;
        end
    end

    initial begin
        $dumpfile("tb_sdcard_text_reader.vcd");
        $dumpvars(0, tb_sdcard_text_reader);

        errors = 0;
        rx_i   = 0;
        fat_start_r = 1'b0;

        repeat (5) @(posedge clk);
        rst = 0;

        // ---- wait for sdcard_spi to finish its init sequence ----
        wait (ready_w || err_w);
        if (err_w) begin
            $display("ERROR: sdcard_spi reported init error");
            errors = errors + 1;
        end else begin
            $display("sdcard_spi ready, card_type=%0d (2=SDHC/SDXC expected)", card_type_w);
            if (card_type_w !== 2'd2) begin
                $display("ERROR: expected card_type=2 (SDHC), got %0d", card_type_w);
                errors = errors + 1;
            end
        end

        // ---- kick off the FAT16 mount + file read ----
        @(posedge clk);
        fat_start_r = 1'b1;
        @(posedge clk);
        fat_start_r = 1'b0;

        wait (fat_done_w || fat_error_w);
        @(posedge clk);

        if (fat_error_w) begin
            $display("ERROR: fat16_reader reported an error");
            errors = errors + 1;
        end else begin
            $display("fat16_reader done: file_size=%0d, bytes streamed=%0d", fat_file_size_w, rx_i);
            if (fat_file_size_w !== expected_len) begin
                $display("ERROR: file_size mismatch: got %0d expected %0d", fat_file_size_w, expected_len);
                errors = errors + 1;
            end
            if (rx_i !== expected_len) begin
                $display("ERROR: streamed byte count mismatch: got %0d expected %0d", rx_i, expected_len);
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("TB_SDCARD_TEXT_READER: PASS");
        else
            $display("TB_SDCARD_TEXT_READER: FAIL (%0d errors)", errors);

        $finish;
    end

    initial begin
        #50_000_000;
        $display("TB_SDCARD_TEXT_READER: WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
