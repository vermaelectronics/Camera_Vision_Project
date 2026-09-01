`timescale 1ns/1ps
// tb_framebuffer_img_loader : exercises the real sys_clk-domain half of
// the design (spi_master, sd_spi_init, sd_block_read, fat_reader,
// img_loader, framebuffer) against a behavioral fake SD card holding a
// real FAT16 superfloppy volume with a real IMAGE.RAW file, using the
// same byte-level SPI slave model technique proven in the sd_uart_top
// project's demo_full_system.v testbench.
//
// The test image's pixel data is a simple, exactly-verifiable pattern:
// pixel n (0..19199) = n itself (as a 16-bit RGB565-shaped word, though
// its actual color-channel interpretation doesn't matter for this
// test) - after loading, framebuffer.mem[n] is checked against n
// directly, byte for byte, for every one of the 19200 pixels.
`timescale 1ns/1ps
`define SIMULATION

module tb_framebuffer_img_loader;

    initial begin
        $dumpfile("tb_framebuffer_img_loader.vcd");
        $dumpvars(0, tb_framebuffer_img_loader);
    end

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #25.0 clk = ~clk; // 20 MHz sys_clk

    // ---- DUT wiring (mirrors sd_image_hdmi_top's sys_clk half) --------
    wire        init_spi_start, blk_spi_start;
    wire [7:0]  init_spi_tx, blk_spi_tx;
    wire [15:0] init_spi_div, blk_spi_div;
    wire        init_cs_n, blk_cs_n;
    wire        spi_busy, spi_done;
    wire [7:0]  spi_rx;
    wire        init_done_w;

    wire spi_start_mux = init_done_w ? blk_spi_start : init_spi_start;
    wire [7:0]  spi_tx_mux  = init_done_w ? blk_spi_tx  : init_spi_tx;
    wire [15:0] spi_div_mux = init_done_w ? blk_spi_div : init_spi_div;
    wire        cs_n_mux    = init_done_w ? blk_cs_n    : init_cs_n;

    wire sd_clk, sd_mosi;
    wire sd_miso;

    spi_master #(.CLK_FREQ_HZ(20_000_000)) u_spi_master (
        .clk(clk), .rst(rst), .clk_div(spi_div_mux), .start(spi_start_mux),
        .tx_data(spi_tx_mux), .rx_data(spi_rx), .busy(spi_busy), .done(spi_done),
        .sclk(sd_clk), .mosi(sd_mosi), .miso(sd_miso)
    );

    wire init_ok_w, sdhc_w, init_error_w, init_busy_w;
    sd_spi_init u_sd_spi_init (
        .clk(clk), .rst(rst), .spi_start(init_spi_start), .spi_tx_data(init_spi_tx),
        .spi_clk_div(init_spi_div), .spi_busy(spi_busy), .spi_done(spi_done),
        .spi_rx_data(spi_rx), .cs_n(init_cs_n), .init_done(init_done_w),
        .init_ok(init_ok_w), .sdhc(sdhc_w), .error(init_error_w), .busy(init_busy_w)
    );

    wire        blk_req_start;
    wire [31:0] blk_req_lba;
    wire        blk_busy_w, blk_done_w, blk_error_w, blk_data_valid_w;
    wire [7:0]  blk_data_w;
    sd_block_read u_sd_block_read (
        .clk(clk), .rst(rst), .sdhc(sdhc_w), .start(blk_req_start), .lba(blk_req_lba),
        .busy(blk_busy_w), .done(blk_done_w), .error(blk_error_w),
        .data_valid(blk_data_valid_w), .data_out(blk_data_w),
        .spi_start(blk_spi_start), .spi_tx_data(blk_spi_tx), .spi_clk_div(blk_spi_div),
        .spi_busy(spi_busy), .spi_done(spi_done), .spi_rx_data(spi_rx), .cs_n(blk_cs_n)
    );

    wire fat_start_w, mount_done_w, mount_ok_w, file_found_w, file_done_w, fat_error_w;
    wire fat_data_valid_w;
    wire [7:0] fat_data_w;
    fat_reader u_fat_reader (
        .clk(clk), .rst(rst), .start(fat_start_w),
        .blk_start(blk_req_start), .blk_lba(blk_req_lba), .blk_done(blk_done_w),
        .blk_error(blk_error_w), .blk_data_valid(blk_data_valid_w), .blk_data_in(blk_data_w),
        .mount_done(mount_done_w), .mount_ok(mount_ok_w), .file_found(file_found_w),
        .file_done(file_done_w), .error(fat_error_w),
        .data_valid(fat_data_valid_w), .data_out(fat_data_w)
    );

    wire img_loaded_w, img_error_w, img_busy_w;
    wire fb_wr_en_w;
    wire [14:0] fb_wr_addr_w;
    wire [15:0] fb_wr_data_w;
    img_loader u_img_loader (
        .clk(clk), .rst(rst), .init_done(init_done_w), .init_ok(init_ok_w),
        .fat_start(fat_start_w), .mount_done(mount_done_w), .mount_ok(mount_ok_w),
        .file_found(file_found_w), .file_done(file_done_w), .fat_error(fat_error_w),
        .file_data(fat_data_w), .file_data_valid(fat_data_valid_w),
        .fb_wr_en(fb_wr_en_w), .fb_wr_addr(fb_wr_addr_w), .fb_wr_data(fb_wr_data_w),
        .loaded(img_loaded_w), .error(img_error_w), .busy(img_busy_w)
    );

    wire [14:0] fb_rd_addr_probe = 15'd0;
    wire [15:0] fb_rd_data_probe;
    framebuffer u_framebuffer (
        .wr_clk(clk), .wr_en(fb_wr_en_w), .wr_addr(fb_wr_addr_w), .wr_data(fb_wr_data_w),
        .rd_clk(clk), .rd_addr(fb_rd_addr_probe), .rd_data(fb_rd_data_probe)
    );

    // -----------------------------------------------------------------
    // Fake SD card: FAT16 superfloppy, one file IMAGE.RAW, single
    // 76-sector cluster (SecPerClus=76 covers the whole 38408-byte
    // file in one cluster, so no FAT chain-walk is needed).
    // -----------------------------------------------------------------
    localparam NUM_PIXELS  = 19200;
    localparam FILE_BYTES  = 8 + NUM_PIXELS*2; // 38408
    localparam DATA_SECTORS = 76;              // ceil(38408/512) = 76
    localparam CARD_SECTORS = 3 + DATA_SECTORS; // LBA0..2 metadata + data

    reg [7:0] card_mem [0:CARD_SECTORS*512-1];
    integer   ci, px;

    initial begin
        for (ci = 0; ci < CARD_SECTORS*512; ci = ci + 1) card_mem[ci] = 8'h00;

        // ---- LBA0: BPB ----
        card_mem[11] = 8'h00; card_mem[12] = 8'h02; // BytesPerSec=512
        card_mem[13] = 76;                          // SecPerClus=76
        card_mem[14] = 8'h01; card_mem[15] = 8'h00; // RsvdSecCnt=1
        card_mem[16] = 8'h01;                       // NumFATs=1
        card_mem[17] = 8'h10; card_mem[18] = 8'h00; // RootEntCnt=16
        card_mem[22] = 8'h01; card_mem[23] = 8'h00; // FATSz16=1
        card_mem['h1C2] = 8'h00;                    // superfloppy (no MBR)

        // ---- LBA1: FAT table, FAT[2] = EOC ----
        card_mem[512+4] = 8'hFF; card_mem[512+5] = 8'hFF;

        // ---- LBA2: root dir, entry0 = IMAGE.RAW ----
        card_mem[1024+0]="I"; card_mem[1024+1]="M"; card_mem[1024+2]="A"; card_mem[1024+3]="G";
        card_mem[1024+4]="E"; card_mem[1024+5]=" "; card_mem[1024+6]=" "; card_mem[1024+7]=" ";
        card_mem[1024+8]="R"; card_mem[1024+9]="A"; card_mem[1024+10]="W";
        card_mem[1024+11] = 8'h20; // ATTR_ARCHIVE
        card_mem[1024+26] = 8'h02; card_mem[1024+27] = 8'h00; // first cluster = 2
        card_mem[1024+28] = FILE_BYTES[7:0];
        card_mem[1024+29] = FILE_BYTES[15:8];
        card_mem[1024+30] = 8'h00; card_mem[1024+31] = 8'h00;

        // ---- LBA3.. : cluster 2 = IMAGE.RAW contents ----
        // header: magic "RIMG", width=160 LE, height=120 LE
        card_mem[1536+0]="R"; card_mem[1536+1]="I"; card_mem[1536+2]="M"; card_mem[1536+3]="G";
        card_mem[1536+4]=8'd160; card_mem[1536+5]=8'd0;
        card_mem[1536+6]=8'd120; card_mem[1536+7]=8'd0;
        // pixel data: pixel n = n (16-bit LE), starting right after the 8-byte header
        for (px = 0; px < NUM_PIXELS; px = px + 1) begin
            card_mem[1536 + 8 + px*2]     = px[7:0];
            card_mem[1536 + 8 + px*2 + 1] = px[15:8];
        end
    end

    // -----------------------------------------------------------------
    // Byte-level SPI slave (mode 0, MSB first) - identical technique to
    // the sd_uart_top project's demo_full_system.v.
    // -----------------------------------------------------------------
    reg [7:0] rx_shift;
    reg [2:0] in_bit_cnt;
    reg [7:0] rx_byte;
    reg       byte_in_rdy;
    wire      sd_csn_probe = cs_n_mux;

    always @(posedge sd_clk or posedge sd_csn_probe) begin
        if (sd_csn_probe) begin
            in_bit_cnt <= 3'd0;
        end else begin
            rx_shift    <= {rx_shift[6:0], sd_mosi};
            in_bit_cnt  <= in_bit_cnt + 3'd1;
            byte_in_rdy <= (in_bit_cnt == 3'd7);
            if (in_bit_cnt == 3'd7)
                rx_byte <= {rx_shift[6:0], sd_mosi};
        end
    end

    localparam C_IDLE = 0, C_CMD = 1, C_RESP = 2;
    reg [1:0]  cstate = C_IDLE;
    reg [2:0]  cmd_byte_idx;
    reg [5:0]  cmd_idx;
    reg [31:0] cmd_arg;
    reg [15:0] resp_idx;
    integer    acmd41_tries;
    reg [31:0] cmd17_lba;

    reg [7:0] next_tx;
    always @(*) begin
        next_tx = 8'hFF;
        if (cstate == C_RESP) begin
            case (cmd_idx)
                6'd0: next_tx = (resp_idx == 0) ? 8'h01 : 8'hFF;
                6'd8: case (resp_idx)
                        16'd0: next_tx = 8'h01;
                        16'd1: next_tx = cmd_arg[31:24];
                        16'd2: next_tx = cmd_arg[23:16];
                        16'd3: next_tx = cmd_arg[15:8];
                        16'd4: next_tx = cmd_arg[7:0];
                        default: next_tx = 8'hFF;
                      endcase
                6'd55: next_tx = (resp_idx == 0) ? 8'h00 : 8'hFF;
                6'd41: next_tx = (resp_idx == 0) ? ((acmd41_tries < 1) ? 8'h01 : 8'h00) : 8'hFF;
                6'd58: case (resp_idx)
                        16'd0: next_tx = 8'h00;
                        16'd1: next_tx = 8'h40;
                        16'd2: next_tx = 8'h00;
                        16'd3: next_tx = 8'hFF;
                        16'd4: next_tx = 8'h80;
                        default: next_tx = 8'hFF;
                      endcase
                6'd17: begin
                    if (resp_idx == 0) next_tx = 8'h00;
                    else if (resp_idx == 1) next_tx = 8'hFE;
                    else if (resp_idx >= 2 && resp_idx <= 513)
                        next_tx = card_mem[cmd17_lba*512 + (resp_idx - 2)];
                    else
                        next_tx = 8'hFF;
                end
                default: next_tx = 8'hFF;
            endcase
        end
    end

    reg [7:0] tx_shift = 8'hFF;
    reg [2:0] out_bit_cnt;
    assign sd_miso = tx_shift[7];

    always @(negedge sd_clk or posedge sd_csn_probe) begin
        if (sd_csn_probe) begin
            out_bit_cnt <= 3'd0;
            tx_shift    <= 8'hFF;
        end else begin
            if (out_bit_cnt == 3'd0)
                tx_shift <= next_tx;
            else
                tx_shift <= {tx_shift[6:0], 1'b0};
            out_bit_cnt <= out_bit_cnt + 3'd1;
        end
    end

    always @(posedge sd_csn_probe) cstate <= C_IDLE;

    always @(posedge byte_in_rdy) begin
        case (cstate)
            C_IDLE: begin
                cmd_byte_idx <= 3'd1;
                cmd_idx      <= rx_byte[5:0];
                cstate       <= C_CMD;
            end
            C_CMD: begin
                case (cmd_byte_idx)
                    3'd1: cmd_arg[31:24] <= rx_byte;
                    3'd2: cmd_arg[23:16] <= rx_byte;
                    3'd3: cmd_arg[15:8]  <= rx_byte;
                    3'd4: cmd_arg[7:0]   <= rx_byte;
                    default: ;
                endcase
                if (cmd_byte_idx == 3'd5) begin
                    resp_idx    <= 16'd0;
                    out_bit_cnt <= 3'd0;
                    if (cmd_idx == 6'd17) cmd17_lba <= cmd_arg;
                    if (cmd_idx == 6'd41) acmd41_tries <= acmd41_tries + 1;
                    cstate <= C_RESP;
                end else begin
                    cmd_byte_idx <= cmd_byte_idx + 3'd1;
                end
            end
            C_RESP: resp_idx <= resp_idx + 16'd1;
        endcase
    end

    initial begin
        cstate = C_IDLE;
        acmd41_tries = 0;
    end

    // -----------------------------------------------------------------
    // Run + verify
    // -----------------------------------------------------------------
    integer errors;
    integer n;
    initial begin
        errors = 0;
        repeat (4) @(posedge clk);
        rst = 1'b0;

        wait (img_loaded_w || img_error_w);
        @(posedge clk);

        if (img_error_w) begin
            $display("FAIL: img_loader reported error instead of loaded");
            errors = errors + 1;
        end else begin
            $display("PASS: img_loader reports the image fully loaded");
            for (n = 0; n < NUM_PIXELS; n = n + 1) begin
                if (u_framebuffer.mem[n] !== n[15:0]) begin
                    $display("FAIL: framebuffer[%0d] = %04h, expected %04h", n, u_framebuffer.mem[n], n[15:0]);
                    errors = errors + 1;
                end
            end
            if (errors == 0)
                $display("PASS: all %0d framebuffer pixels match the expected test pattern exactly", NUM_PIXELS);
        end

        if (errors == 0)
            $display("PASS: tb_framebuffer_img_loader - SD init, FAT mount, IMAGE.RAW search/read and framebuffer write all correct");
        else
            $display("FAIL: %0d check(s) failed", errors);
        $finish;
    end

    initial begin
        // 76 data sectors + 2 metadata sector reads at ~915us/sector
        // (post-init 5 MHz SPI) plus ~4ms of init overhead totals
        // ~75ms - give it a healthy margin above that.
        #150_000_000; // 150 ms cap
        $display("[tb_framebuffer_img_loader] TIMEOUT waiting for img_loaded/img_error");
        $finish;
    end

endmodule
