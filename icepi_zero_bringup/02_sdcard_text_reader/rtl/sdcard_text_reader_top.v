// ============================================================================
// sdcard_text_reader_top.v -- IcePi-Zero bring-up #2: read one text file off
// a FAT16-formatted SD card (over SPI) and stream its contents out UART.
// ----------------------------------------------------------------------------
// Flow: sdcard_spi mounts the card automatically after reset. Once mounted
// (sd_ready), fat16_reader is kicked off automatically once, and again on
// every button[1] press, to look up FILENAME in the root directory and
// stream its bytes out. A 1024-byte sync_fifo absorbs the rate mismatch
// between the SD card's fast per-block bursts and the much slower UART, with
// fat16_reader backpressured (fifo_ready_for_block) so no byte is ever
// dropped regardless of file size.
//
// LEDs (all real, onboard, no external wiring):
//   led[0] sd_ready       -- SD card detected, initialized, ready
//   led[1] read busy      -- a file read is in progress
//   led[2] error (sticky) -- last operation (SD init or FAT16 read) failed
//   led[3] done (sticky)  -- last file read completed successfully
//   led[4] heartbeat      -- 1Hz blink; proves the clock/bitstream is alive
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module sdcard_text_reader_top #(
    parameter integer CLK_FREQ_HZ    = 50_000_000,
    parameter integer BAUD           = 115200,
    parameter [8*11-1:0] FILENAME    = "HELLO   TXT"
) (
    input  wire        clk,          // 50MHz board oscillator (site M1)
    input  wire  [1:0] button,       // active-low; [0]=reset, [1]=re-read file
    output wire  [4:0] led,
    output wire        uart_txd,     // to the board's FTDI UART header

    output wire         sd_sclk,
    output wire         sd_mosi,
    input  wire          sd_miso,
    output wire          sd_cs_n
);

    wire rst = ~button[0];

    // ---- button[1] debounce + rising-edge detect (active-low: press = 0) ----
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
    assign btn1_pressed_edge = btn_stable_prev && !btn_stable; // active-low: 1->0 = press

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

    // ---- auto-start once, right after the SD card becomes ready ----
    reg sd_ready_prev;
    wire auto_start = sd_ready_w && !sd_ready_prev;
    always @(posedge clk) sd_ready_prev <= rst ? 1'b0 : sd_ready_w;

    wire fat_start_w = auto_start || btn1_pressed_edge;

    // ---- FAT16 reader ----
    wire fat_busy_w, fat_done_w, fat_error_w;
    wire [31:0] fat_file_size_w;
    wire fat_data_valid_w;
    wire [7:0] fat_data_byte_w;
    wire fifo_ready_for_block_w;

    fat16_reader #(.FILENAME(FILENAME)) fat (
        .clk(clk), .rst(rst),
        .sd_read_req(sd_read_req_w), .sd_block_addr(sd_block_addr_w),
        .sd_rd_busy(sd_rd_busy_w), .sd_rd_done(sd_rd_done_w), .sd_rd_error(sd_rd_error_w),
        .sd_byte_valid(sd_byte_valid_w), .sd_byte_data(sd_byte_data_w), .sd_byte_index(sd_byte_index_w),
        .sd_ready(sd_ready_w),
        .start(fat_start_w), .fifo_ready_for_block(fifo_ready_for_block_w),
        .busy(fat_busy_w), .done(fat_done_w), .error(fat_error_w),
        .file_size(fat_file_size_w),
        .data_valid(fat_data_valid_w), .data_byte(fat_data_byte_w)
    );

    // ---- byte FIFO between the (fast, bursty) SD stream and the (slow) UART ----
    localparam integer FIFO_DEPTH = 1024;
    wire fifo_full_w, fifo_empty_w;
    wire [$clog2(FIFO_DEPTH):0] fifo_count_w;
    wire [7:0] fifo_rd_data_w;
    reg  fifo_rd_en;

    sync_fifo #(.WIDTH(8), .DEPTH(FIFO_DEPTH)) fifo (
        .clk(clk), .rst(rst),
        .wr_en(fat_data_valid_w), .wr_data(fat_data_byte_w), .full(fifo_full_w),
        .rd_en(fifo_rd_en), .rd_data(fifo_rd_data_w), .empty(fifo_empty_w),
        .count(fifo_count_w)
    );

    assign fifo_ready_for_block_w = ((FIFO_DEPTH - fifo_count_w) >= 512);

    // ---- UART transmitter, fed from the FIFO ----
    //
    // sync_fifo's rd_data is a registered (synchronous) read: asserting
    // rd_en during cycle C makes the popped byte appear on rd_data starting
    // cycle C+1, one cycle later. uart_valid_r must therefore lag fifo_rd_en
    // by exactly that same one cycle -- if it were driven straight off
    // fifo_rd_en (one stage instead of two), valid would fire a cycle
    // before the corresponding data is actually fresh, silently pairing
    // every "new" byte with the *previous* pop's stale data and corrupting
    // the stream by one byte per pop. fifo_rd_en_d1 supplies that missing
    // second stage so both the valid and data paths carry the same delay.
    wire uart_ready_w;
    reg  fifo_rd_en_d1;
    reg  uart_valid_r;
    reg  [7:0] uart_data_r;

    always @(posedge clk) begin
        fifo_rd_en    <= 1'b0;
        fifo_rd_en_d1 <= fifo_rd_en;
        uart_valid_r  <= 1'b0;
        if (rst) begin
            fifo_rd_en_d1 <= 1'b0;
            uart_data_r   <= 8'h00;
        end else begin
            // uart_ready_w only drops to 0 the cycle *after* uart_tx accepts a
            // valid pulse (its own registered busy/ready latency) -- gating
            // solely on fifo_rd_en/fifo_rd_en_d1 (the FIFO's read latency)
            // leaves a one-cycle window, right after uart_valid_r fires, where
            // uart_ready_w still reads stale-high and a second pop sneaks in.
            // That extra byte gets popped from the FIFO but arrives while
            // uart_tx is already busy, so it's silently dropped -- shifting
            // every byte after it. !uart_valid_r closes that window.
            if (uart_ready_w && !fifo_empty_w && !fifo_rd_en && !fifo_rd_en_d1 && !uart_valid_r) begin
                fifo_rd_en <= 1'b1; // pop this cycle
            end
            uart_valid_r <= fifo_rd_en_d1; // rd_data is fresh exactly when this fires
            uart_data_r  <= fifo_rd_data_w;
        end
    end

    uart_tx #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD(BAUD)) utx (
        .clk(clk), .rst(rst),
        .data(uart_data_r), .valid(uart_valid_r), .ready(uart_ready_w),
        .tx(uart_txd)
    );

    // ---- sticky status latches, cleared at the start of each new read ----
    reg error_latched, done_latched;
    always @(posedge clk) begin
        if (rst || fat_start_w) begin
            error_latched <= 1'b0;
            done_latched  <= 1'b0;
        end else begin
            if (fat_error_w) error_latched <= 1'b1;
            if (fat_done_w)  done_latched  <= 1'b1;
        end
    end

    // ---- 1Hz heartbeat ----
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
    assign led[1] = fat_busy_w;
    assign led[2] = error_latched || sd_err_w;
    assign led[3] = done_latched;
    assign led[4] = hb_r;

endmodule

`default_nettype wire
