`timescale 1ns/1ps
// sd_block_read : single 512-byte sector read via CMD17.
//
// Takes over the shared spi_master bus from sd_spi_init once
// init_done is asserted (arbitration is done in sd_uart_top). Runs at
// SPI_DIV_FAST since the card is fully initialized by this point.
//
// `sdhc` selects the CMD17 argument format: SDHC/SDXC cards address
// blocks directly (argument = LBA); SDSC cards use byte addressing
// (argument = LBA * 512).
//
// error is always accompanied by done, and busy always returns to 0
// on error or completion - no transaction can leave busy asserted.
module sd_block_read (
    input  wire        clk,        // sys_clk, 20 MHz
    input  wire        rst,
    input  wire        sdhc,

    input  wire        start,
    input  wire [31:0] lba,
    output reg         busy,
    output reg         done,
    output reg         error,

    output reg         data_valid,
    output reg  [7:0]  data_out,

    // shared spi_master interface (granted to this module post-init_done)
    output reg         spi_start,
    output reg  [7:0]  spi_tx_data,
    output wire [15:0] spi_clk_div,
    input  wire        spi_busy,
    input  wire        spi_done,
    input  wire [7:0]  spi_rx_data,
    output reg         cs_n
);
    localparam [5:0] SD_CMD17 = 6'd17; // READ_SINGLE_BLOCK

    localparam [7:0] SD_TOKEN_START_BLOCK = 8'hFE;
    localparam [7:0] SD_TOKEN_DUMMY       = 8'hFF;

    // SPI clock divider for spi_master (clk_div input), derived from
    // sys_clk = 20 MHz : sclk = sys_clk / (2*(clk_div+1))
    localparam [15:0] SPI_DIV_FAST = 16'd1; // ~5 MHz, used after init completes

    assign spi_clk_div = SPI_DIV_FAST;

    wire [31:0] cmd_arg = sdhc ? lba : {lba[22:0], 9'd0};

    localparam S_IDLE        = 4'd0,
               S_SEND         = 4'd1,
               S_SEND_WAIT    = 4'd2,
               S_POLL_R1      = 4'd3,
               S_POLL_R1_WAIT = 4'd4,
               S_WAIT_TOKEN   = 4'd5,
               S_WAIT_TOKEN_WAIT = 4'd6,
               S_READ_DATA    = 4'd7,
               S_READ_DATA_WAIT  = 4'd8,
               S_READ_CRC     = 4'd9,
               S_READ_CRC_WAIT   = 4'd10,
               S_CS_HIGH      = 4'd11,
               S_DONE         = 4'd12,
               S_ERROR        = 4'd13,
               S_CS_HIGH_WAIT = 4'd14;

    localparam NCR_MAX      = 4'd8;
    localparam TOKEN_TIMEOUT = 21'd2_000_000; // 100 ms @ 20 MHz (Nac)

    reg [3:0]  state;
    reg [2:0]  byte_idx;    // command byte index 0..5
    reg [3:0]  poll_cnt;
    reg [20:0] tmo_cnt;
    reg [9:0]  data_cnt;    // 0..511
    reg [0:0]  crc_idx;

    function [7:0] cmd_byte;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: cmd_byte = {2'b01, SD_CMD17};
                3'd1: cmd_byte = cmd_arg[31:24];
                3'd2: cmd_byte = cmd_arg[23:16];
                3'd3: cmd_byte = cmd_arg[15:8];
                3'd4: cmd_byte = cmd_arg[7:0];
                default: cmd_byte = 8'hFF;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            spi_start   <= 1'b0;
            spi_tx_data <= 8'hFF;
            cs_n        <= 1'b1;
            busy        <= 1'b0;
            done        <= 1'b0;
            error       <= 1'b0;
            data_valid  <= 1'b0;
            data_out    <= 8'd0;
            byte_idx    <= 3'd0;
            poll_cnt    <= 4'd0;
            tmo_cnt     <= 21'd0;
            data_cnt    <= 10'd0;
            crc_idx     <= 1'b0;
        end else begin
            spi_start  <= 1'b0;
            done       <= 1'b0;
            data_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    error <= 1'b0;
                    if (start) begin
                        busy     <= 1'b1;
                        cs_n     <= 1'b0;
                        byte_idx <= 3'd0;
                        state    <= S_SEND;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                S_SEND: begin
                    if (!spi_busy) begin
                        spi_tx_data <= cmd_byte(byte_idx);
                        spi_start   <= 1'b1;
                        state       <= S_SEND_WAIT;
                    end
                end
                S_SEND_WAIT: begin
                    if (spi_done) begin
                        if (byte_idx == 3'd5) begin
                            poll_cnt <= 4'd0;
                            state    <= S_POLL_R1;
                        end else begin
                            byte_idx <= byte_idx + 3'd1;
                            state    <= S_SEND;
                        end
                    end
                end

                S_POLL_R1: begin
                    if (!spi_busy) begin
                        spi_tx_data <= SD_TOKEN_DUMMY;
                        spi_start   <= 1'b1;
                        state       <= S_POLL_R1_WAIT;
                    end
                end
                S_POLL_R1_WAIT: begin
                    if (spi_done) begin
                        if (spi_rx_data == 8'h00) begin
                            tmo_cnt <= 21'd0;
                            state   <= S_WAIT_TOKEN;
                        end else if (spi_rx_data[7] == 1'b0) begin
                            // valid R1 but nonzero -> card reported an error
                            state <= S_ERROR;
                        end else if (poll_cnt == NCR_MAX - 1) begin
                            state <= S_ERROR;
                        end else begin
                            poll_cnt <= poll_cnt + 4'd1;
                            state    <= S_POLL_R1;
                        end
                    end
                end

                S_WAIT_TOKEN: begin
                    if (!spi_busy) begin
                        spi_tx_data <= SD_TOKEN_DUMMY;
                        spi_start   <= 1'b1;
                        state       <= S_WAIT_TOKEN_WAIT;
                    end
                end
                S_WAIT_TOKEN_WAIT: begin
                    if (spi_done) begin
                        if (spi_rx_data == SD_TOKEN_START_BLOCK) begin
                            data_cnt <= 10'd0;
                            state    <= S_READ_DATA;
                        end else if (tmo_cnt == TOKEN_TIMEOUT) begin
                            state <= S_ERROR;
                        end else begin
                            tmo_cnt <= tmo_cnt + 21'd1;
                            state   <= S_WAIT_TOKEN;
                        end
                    end
                end

                S_READ_DATA: begin
                    if (!spi_busy) begin
                        spi_tx_data <= SD_TOKEN_DUMMY;
                        spi_start   <= 1'b1;
                        state       <= S_READ_DATA_WAIT;
                    end
                end
                S_READ_DATA_WAIT: begin
                    if (spi_done) begin
                        data_out   <= spi_rx_data;
                        data_valid <= 1'b1;
                        if (data_cnt == 10'd511) begin
                            crc_idx <= 1'b0;
                            state   <= S_READ_CRC;
                        end else begin
                            data_cnt <= data_cnt + 10'd1;
                            state    <= S_READ_DATA;
                        end
                    end
                end

                S_READ_CRC: begin
                    if (!spi_busy) begin
                        spi_tx_data <= SD_TOKEN_DUMMY;
                        spi_start   <= 1'b1;
                        state       <= S_READ_CRC_WAIT;
                    end
                end
                S_READ_CRC_WAIT: begin
                    if (spi_done) begin
                        if (crc_idx == 1'b1) begin
                            state <= S_CS_HIGH;
                        end else begin
                            crc_idx <= 1'b1;
                            state   <= S_READ_CRC;
                        end
                    end
                end

                // Deassert CS and clock out one trailing dummy byte,
                // then WAIT for that transfer to finish before
                // reporting done - fat_reader reacts to done within a
                // cycle by requesting the next sector, whose S_IDLE
                // branch reasserts CS unconditionally. If done fired
                // here immediately (not waiting for spi_done), that
                // reassert could land while spi_master was still
                // mid-transfer on this trailing byte, corrupting the
                // next sector request's framing.
                S_CS_HIGH: begin
                    if (!spi_busy) begin
                        cs_n        <= 1'b1;
                        spi_tx_data <= SD_TOKEN_DUMMY;
                        spi_start   <= 1'b1;
                        state       <= S_CS_HIGH_WAIT;
                    end
                end
                S_CS_HIGH_WAIT: begin
                    if (spi_done)
                        state <= S_DONE;
                end

                S_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                S_ERROR: begin
                    cs_n  <= 1'b1;
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    error <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
