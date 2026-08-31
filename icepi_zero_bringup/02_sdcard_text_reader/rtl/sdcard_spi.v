// ============================================================================
// sdcard_spi.v -- SD/SDHC card SPI-mode initialization + single-block reader.
// ----------------------------------------------------------------------------
// Implements the standard SD SPI-mode power-up sequence (SanDisk/SD Assoc.
// "SD Specifications Part 1", section 7 "SPI Mode"):
//   1. >=74 clock cycles with CS high, MOSI high (card power-up)
//   2. CMD0  (GO_IDLE_STATE)              -> expect R1 = 0x01 (idle)
//   3. CMD8  (SEND_IF_COND, 0x1AA)        -> distinguishes SD v2 (SDHC/SDXC
//      capable) from SD v1/MMC via an illegal-command R1 vs. an echoed
//      check pattern
//   4. CMD55+ACMD41 (SD_SEND_OP_COND) looped until the card leaves idle
//      (HCS bit set in the ACMD41 argument when CMD8 succeeded, to allow
//      SDHC/SDXC negotiation)
//   5. CMD58 (READ_OCR) to read the CCS bit -> tells us whether block
//      addresses are byte-addressed (SDSC) or block-addressed (SDHC/SDXC)
//   6. CMD16 (SET_BLOCKLEN, 512) -- a no-op/ignored on SDHC/SDXC (block
//      length is fixed at 512), required on SDSC
//
// After that, CMD17 (READ_SINGLE_BLOCK) reads one 512-byte block: send the
// command, wait for R1=0x00, wait for the 0xFE data start token, clock out
// 512 data bytes + 2 CRC bytes (CRC is not checked -- SPI-mode CRC checking
// is off by default unless CMD59 enables it, which this driver never sends).
//
// A single generic "send 6 command bytes, wait for R1, optionally read N
// trailing bytes" engine (states S_SEND_BYTES/S_WAIT_R1/S_TRAIL, dispatched
// via cmd_id in S_DISPATCH) drives every command above -- avoids repeating
// the same byte-shifting bookkeeping seven times.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module sdcard_spi #(
    parameter integer CLK_FREQ_HZ = 50_000_000
) (
    input  wire        clk,
    input  wire        rst,

    // physical SD SPI pins
    output wire         sd_sclk,
    output wire         sd_mosi,
    input  wire          sd_miso,
    output reg           sd_cs_n,

    // status
    output reg          ready,       // init complete; read_req now accepted
    output reg          error,       // init failed, sticky until rst
    output reg  [1:0]   card_type,   // 0=unknown, 1=SDSC (byte addr), 2=SDHC/SDXC (block addr)

    // single-block read interface
    input  wire         read_req,    // pulse (only while ready && !rd_busy)
    input  wire [31:0]  block_addr,  // 512-byte block number
    output reg          rd_busy,
    output reg          rd_done,     // one-cycle pulse: block fully streamed
    output reg          rd_error,    // one-cycle pulse: this read failed

    output reg          byte_valid,  // one-cycle pulse, byte_data valid
    output reg  [7:0]   byte_data,
    output reg  [8:0]   byte_index   // 0..511, position matching byte_data/byte_valid
);

    // ---- clock divider: slow (<=400kHz) during init, faster once ready ----
    localparam integer DIV_INIT = (CLK_FREQ_HZ / (2 * 200_000)) - 1; // ~200kHz
    localparam integer DIV_FAST = (CLK_FREQ_HZ / (2 * 6_250_000)) - 1; // ~6.25MHz
    wire [15:0] spi_div = ready ? DIV_FAST[15:0] : DIV_INIT[15:0];

    reg         spi_start;
    reg  [7:0]  spi_tx;
    wire        spi_busy, spi_done;
    wire [7:0]  spi_rx;

    spi_master spi (
        .clk(clk), .rst(rst),
        .div(spi_div),
        .start(spi_start), .tx_byte(spi_tx),
        .busy(spi_busy), .done(spi_done), .rx_byte(spi_rx),
        .sclk(sd_sclk), .mosi(sd_mosi), .miso(sd_miso)
    );

    // ---- top-level FSM ----
    localparam
        S_PWRUP      = 5'd0,
        S_SEND_BYTES = 5'd1,
        S_WAIT_R1    = 5'd2,
        S_TRAIL      = 5'd3,
        S_DISPATCH   = 5'd4,
        S_READY      = 5'd5,
        S_WAIT_TOKEN = 5'd6,
        S_READ_DATA  = 5'd7,
        S_READ_CRC0  = 5'd8,
        S_READ_CRC1  = 5'd9,
        S_ERROR      = 5'd10;

    reg [4:0] state;

    localparam
        CMD_0  = 3'd0,
        CMD_8  = 3'd1,
        CMD_55 = 3'd2,
        CMD_41 = 3'd3,
        CMD_58 = 3'd4,
        CMD_16 = 3'd5,
        CMD_17 = 3'd6;

    reg [2:0]  cmd_id;
    reg [7:0]  cmd_buf [0:5];
    reg [2:0]  send_idx;
    reg [2:0]  trail_count;
    reg [2:0]  trail_idx;
    reg        byte_sub;      // 0 = issue spi start, 1 = wait for spi_done
    reg [15:0] r1_retry;
    reg [15:0] acmd41_retry;
    reg [7:0]  last_r1;
    reg [31:0] last_r7;
    reg        hcs;           // CMD8 succeeded -> negotiate HCS in ACMD41
    reg [8:0]  byte_pos;      // internal loop counter for S_READ_DATA (see byte_index note above)
    reg [3:0]  pwrup_idx;
    reg [31:0] token_retry;

    localparam [15:0] R1_MAX_RETRY     = 16'd8;
    localparam [15:0] ACMD41_MAX_RETRY = 16'd20000; // generous: real cards can take up to ~1s
    localparam [31:0] TOKEN_MAX_RETRY  = 32'd200000;

    always @(posedge clk) begin
        spi_start  <= 1'b0;
        rd_done    <= 1'b0;
        rd_error   <= 1'b0;
        byte_valid <= 1'b0;

        if (rst) begin
            state        <= S_PWRUP;
            sd_cs_n      <= 1'b1;
            ready        <= 1'b0;
            error        <= 1'b0;
            card_type    <= 2'd0;
            rd_busy      <= 1'b0;
            byte_index   <= 9'd0;
            byte_pos     <= 9'd0;
            pwrup_idx    <= 4'd0;
            byte_sub     <= 1'b0;
            r1_retry     <= 16'd0;
            acmd41_retry <= 16'd0;
            token_retry  <= 32'd0;
            hcs          <= 1'b0;
        end else begin
            case (state)

                // ---- 80 dummy clocks with CS deasserted, MOSI high ----
                S_PWRUP: begin
                    sd_cs_n <= 1'b1;
                    case (byte_sub)
                        1'b0: begin
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                            byte_sub  <= 1'b1;
                        end
                        1'b1: begin
                            spi_start <= 1'b0;
                            if (spi_done) begin
                                if (pwrup_idx == 4'd9) begin
                                    sd_cs_n     <= 1'b0;
                                    cmd_id      <= CMD_0;
                                    cmd_buf[0]  <= 8'h40; cmd_buf[1] <= 8'h00; cmd_buf[2] <= 8'h00;
                                    cmd_buf[3]  <= 8'h00; cmd_buf[4] <= 8'h00; cmd_buf[5] <= 8'h95;
                                    trail_count <= 3'd0;
                                    send_idx    <= 3'd0;
                                    byte_sub    <= 1'b0;
                                    state       <= S_SEND_BYTES;
                                end else begin
                                    pwrup_idx <= pwrup_idx + 1'b1;
                                    byte_sub  <= 1'b0;
                                end
                            end
                        end
                    endcase
                end

                // ---- send the 6 command bytes (0x40|idx, 4 arg bytes, crc) ----
                S_SEND_BYTES: begin
                    case (byte_sub)
                        1'b0: begin
                            spi_tx    <= cmd_buf[send_idx];
                            spi_start <= 1'b1;
                            byte_sub  <= 1'b1;
                        end
                        1'b1: begin
                            spi_start <= 1'b0;
                            if (spi_done) begin
                                if (send_idx == 3'd5) begin
                                    r1_retry <= 16'd0;
                                    byte_sub <= 1'b0;
                                    state    <= S_WAIT_R1;
                                end else begin
                                    send_idx <= send_idx + 1'b1;
                                    byte_sub <= 1'b0;
                                end
                            end
                        end
                    endcase
                end

                // ---- poll 0xFF until the R1 response byte (MSB=0) arrives ----
                S_WAIT_R1: begin
                    case (byte_sub)
                        1'b0: begin
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                            byte_sub  <= 1'b1;
                        end
                        1'b1: begin
                            spi_start <= 1'b0;
                            if (spi_done) begin
                                if (spi_rx[7] == 1'b0) begin
                                    last_r1 <= spi_rx;
                                    if (trail_count != 3'd0) begin
                                        trail_idx <= 3'd0;
                                        byte_sub  <= 1'b0;
                                        state     <= S_TRAIL;
                                    end else begin
                                        byte_sub <= 1'b0;
                                        state    <= S_DISPATCH;
                                    end
                                end else if (r1_retry >= R1_MAX_RETRY) begin
                                    state <= S_ERROR;
                                end else begin
                                    r1_retry <= r1_retry + 1'b1;
                                    byte_sub <= 1'b0; // stay, poll again
                                end
                            end
                        end
                    endcase
                end

                // ---- read `trail_count` extra bytes (R3/R7 payload) ----
                S_TRAIL: begin
                    case (byte_sub)
                        1'b0: begin
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                            byte_sub  <= 1'b1;
                        end
                        1'b1: begin
                            spi_start <= 1'b0;
                            if (spi_done) begin
                                last_r7 <= {last_r7[23:0], spi_rx};
                                if (trail_idx == trail_count - 1'b1) begin
                                    byte_sub <= 1'b0;
                                    state    <= S_DISPATCH;
                                end else begin
                                    trail_idx <= trail_idx + 1'b1;
                                    byte_sub  <= 1'b0;
                                end
                            end
                        end
                    endcase
                end

                // ---- one command just completed: decide what happens next ----
                S_DISPATCH: begin
                    case (cmd_id)
                        CMD_0: begin
                            if (last_r1 == 8'h01) begin
                                cmd_id      <= CMD_8;
                                cmd_buf[0]  <= 8'h48; cmd_buf[1] <= 8'h00; cmd_buf[2] <= 8'h00;
                                cmd_buf[3]  <= 8'h01; cmd_buf[4] <= 8'hAA; cmd_buf[5] <= 8'h87;
                                trail_count <= 3'd4;
                                send_idx    <= 3'd0;
                                state       <= S_SEND_BYTES;
                            end else begin
                                state <= S_ERROR; // card never went idle
                            end
                        end

                        CMD_8: begin
                            if (last_r1[2]) begin
                                hcs <= 1'b0; // illegal command -> SD v1 / MMC
                            end else if (last_r1 == 8'h01 && last_r7[7:0] == 8'hAA) begin
                                hcs <= 1'b1; // SD v2, check pattern echoed correctly
                            end
                            if (last_r1[2] || (last_r1 == 8'h01 && last_r7[7:0] == 8'hAA)) begin
                                cmd_id      <= CMD_55;
                                cmd_buf[0]  <= 8'h77; cmd_buf[1] <= 8'h00; cmd_buf[2] <= 8'h00;
                                cmd_buf[3]  <= 8'h00; cmd_buf[4] <= 8'h00; cmd_buf[5] <= 8'h01;
                                trail_count <= 3'd0;
                                send_idx    <= 3'd0;
                                acmd41_retry<= 16'd0;
                                state       <= S_SEND_BYTES;
                            end else begin
                                state <= S_ERROR; // unexpected CMD8 response
                            end
                        end

                        CMD_55: begin
                            if (last_r1[2]) begin
                                state <= S_ERROR; // not an SD card
                            end else begin
                                cmd_id      <= CMD_41;
                                cmd_buf[0]  <= 8'h69;
                                cmd_buf[1]  <= hcs ? 8'h40 : 8'h00;
                                cmd_buf[2]  <= 8'h00; cmd_buf[3] <= 8'h00; cmd_buf[4] <= 8'h00;
                                cmd_buf[5]  <= 8'h01;
                                trail_count <= 3'd0;
                                send_idx    <= 3'd0;
                                state       <= S_SEND_BYTES;
                            end
                        end

                        CMD_41: begin
                            if (last_r1 == 8'h00) begin
                                cmd_id      <= CMD_58;
                                cmd_buf[0]  <= 8'h7A; cmd_buf[1] <= 8'h00; cmd_buf[2] <= 8'h00;
                                cmd_buf[3]  <= 8'h00; cmd_buf[4] <= 8'h00; cmd_buf[5] <= 8'h01;
                                trail_count <= 3'd4;
                                send_idx    <= 3'd0;
                                state       <= S_SEND_BYTES;
                            end else if (last_r1 == 8'h01) begin
                                if (acmd41_retry >= ACMD41_MAX_RETRY) begin
                                    state <= S_ERROR; // card never left idle
                                end else begin
                                    acmd41_retry <= acmd41_retry + 1'b1;
                                    cmd_id       <= CMD_55;
                                    cmd_buf[0]   <= 8'h77; cmd_buf[1] <= 8'h00; cmd_buf[2] <= 8'h00;
                                    cmd_buf[3]   <= 8'h00; cmd_buf[4] <= 8'h00; cmd_buf[5] <= 8'h01;
                                    trail_count  <= 3'd0;
                                    send_idx     <= 3'd0;
                                    state        <= S_SEND_BYTES;
                                end
                            end else begin
                                state <= S_ERROR; // unexpected ACMD41 response
                            end
                        end

                        CMD_58: begin
                            card_type   <= last_r7[30] ? 2'd2 : 2'd1; // CCS bit
                            cmd_id      <= CMD_16;
                            cmd_buf[0]  <= 8'h50; cmd_buf[1] <= 8'h00; cmd_buf[2] <= 8'h00;
                            cmd_buf[3]  <= 8'h02; cmd_buf[4] <= 8'h00; cmd_buf[5] <= 8'h01; // arg=512
                            trail_count <= 3'd0;
                            send_idx    <= 3'd0;
                            state       <= S_SEND_BYTES;
                        end

                        CMD_16: begin
                            // Ignore the result: SDHC/SDXC cards may error here since
                            // their block length is always fixed at 512 -- harmless.
                            ready <= 1'b1;
                            state <= S_READY;
                        end

                        CMD_17: begin
                            if (last_r1 == 8'h00) begin
                                token_retry <= 32'd0;
                                state       <= S_WAIT_TOKEN;
                            end else begin
                                rd_error <= 1'b1;
                                rd_busy  <= 1'b0;
                                state    <= S_READY;
                            end
                        end

                        default: state <= S_ERROR;
                    endcase
                end

                // ---- idle: accept one-block read requests ----
                S_READY: begin
                    if (read_req && !rd_busy) begin
                        rd_busy     <= 1'b1;
                        cmd_id      <= CMD_17;
                        cmd_buf[0]  <= 8'h51;
                        if (card_type == 2'd2) begin
                            // SDHC/SDXC: argument is the block number directly
                            cmd_buf[1] <= block_addr[31:24];
                            cmd_buf[2] <= block_addr[23:16];
                            cmd_buf[3] <= block_addr[15:8];
                            cmd_buf[4] <= block_addr[7:0];
                        end else begin
                            // SDSC: argument is a byte address (block * 512)
                            cmd_buf[1] <= block_addr[23:16];
                            cmd_buf[2] <= block_addr[15:8];
                            cmd_buf[3] <= block_addr[7:0];
                            cmd_buf[4] <= 8'h00;
                        end
                        cmd_buf[5]  <= 8'h01;
                        trail_count <= 3'd0;
                        send_idx    <= 3'd0;
                        state       <= S_SEND_BYTES;
                    end
                end

                // ---- poll for the 0xFE data-start token ----
                S_WAIT_TOKEN: begin
                    case (byte_sub)
                        1'b0: begin
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                            byte_sub  <= 1'b1;
                        end
                        1'b1: begin
                            spi_start <= 1'b0;
                            if (spi_done) begin
                                if (spi_rx == 8'hFE) begin
                                    byte_index <= 9'd0;
                                    byte_pos   <= 9'd0;
                                    byte_sub   <= 1'b0;
                                    state      <= S_READ_DATA;
                                end else if (spi_rx != 8'hFF) begin
                                    // data-error token (0000xxxx with a bit set)
                                    rd_error <= 1'b1;
                                    rd_busy  <= 1'b0;
                                    state    <= S_READY;
                                end else if (token_retry >= TOKEN_MAX_RETRY) begin
                                    rd_error <= 1'b1;
                                    rd_busy  <= 1'b0;
                                    state    <= S_READY;
                                end else begin
                                    token_retry <= token_retry + 1'b1;
                                    byte_sub    <= 1'b0;
                                end
                            end
                        end
                    endcase
                end

                // ---- stream 512 data bytes ----
                S_READ_DATA: begin
                    case (byte_sub)
                        1'b0: begin
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                            byte_sub  <= 1'b1;
                        end
                        1'b1: begin
                            spi_start <= 1'b0;
                            if (spi_done) begin
                                byte_data  <= spi_rx;
                                // Report THIS byte's position on the same edge that
                                // byte_data/byte_valid change, using the pre-increment
                                // value of the internal loop counter (byte_pos) --
                                // bumping byte_index itself here would advance it one
                                // edge too early relative to byte_data as seen by any
                                // consumer registered on the same clock (both update
                                // via nonblocking assignment "together," so a consumer
                                // sampling next edge would see the *next* index paired
                                // with *this* byte's data).
                                byte_index <= byte_pos;
                                byte_valid <= 1'b1;
                                byte_sub   <= 1'b0;
                                if (byte_pos == 9'd511) begin
                                    state <= S_READ_CRC0;
                                end else begin
                                    byte_pos <= byte_pos + 1'b1;
                                end
                            end
                        end
                    endcase
                end

                // ---- read + discard the 2 trailing CRC bytes ----
                S_READ_CRC0: begin
                    case (byte_sub)
                        1'b0: begin spi_tx <= 8'hFF; spi_start <= 1'b1; byte_sub <= 1'b1; end
                        1'b1: begin
                            spi_start <= 1'b0;
                            if (spi_done) begin byte_sub <= 1'b0; state <= S_READ_CRC1; end
                        end
                    endcase
                end
                S_READ_CRC1: begin
                    case (byte_sub)
                        1'b0: begin spi_tx <= 8'hFF; spi_start <= 1'b1; byte_sub <= 1'b1; end
                        1'b1: begin
                            spi_start <= 1'b0;
                            if (spi_done) begin
                                rd_done  <= 1'b1;
                                rd_busy  <= 1'b0;
                                byte_sub <= 1'b0;
                                state    <= S_READY;
                            end
                        end
                    endcase
                end

                S_ERROR: begin
                    error <= 1'b1;
                    ready <= 1'b0;
                end

                default: state <= S_ERROR;
            endcase
        end
    end

endmodule

`default_nettype wire
