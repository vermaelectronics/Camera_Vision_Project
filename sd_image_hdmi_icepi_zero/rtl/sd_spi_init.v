`timescale 1ns/1ps
// sd_spi_init : SD-card SPI-mode bring-up (CMD0 -> CMD8 -> ACMD41 -> CMD58).
//
// No card-detect input exists on this board, so initialization is
// started unconditionally on reset release and success/failure is
// determined purely from SPI responses (init_ok / error).
//
// Owns the physical SPI bus (via the shared spi_master core, arbitrated
// in sd_uart_top) until init_done is asserted; sd_block_read takes over
// after that.
module sd_spi_init (
    input  wire        clk,          // sys_clk, 20 MHz
    input  wire        rst,

    // shared spi_master interface (granted to this module pre-init_done)
    output reg         spi_start,
    output reg  [7:0]  spi_tx_data,
    output wire [15:0] spi_clk_div,
    input  wire        spi_busy,
    input  wire        spi_done,
    input  wire [7:0]  spi_rx_data,
    output reg         cs_n,

    output reg         init_done,
    output reg         init_ok,
    output reg         sdhc,         // 1 = block addressing (SDHC/SDXC)
    output reg         error,
    output wire        busy
);
    // SD command indices (SPI mode)
    localparam [5:0] SD_CMD0  = 6'd0;   // GO_IDLE_STATE
    localparam [5:0] SD_CMD8  = 6'd8;   // SEND_IF_COND
    localparam [5:0] SD_CMD55 = 6'd55;  // APP_CMD
    localparam [5:0] SD_CMD41 = 6'd41;  // SD_SEND_OP_COND (used as ACMD41)
    localparam [5:0] SD_CMD58 = 6'd58;  // READ_OCR

    // Fixed CRC7+end-bit bytes mandated by the SD spec for the two
    // commands that are always CRC-checked in SPI mode with these
    // exact arguments.
    localparam [7:0] SD_CRC_CMD0 = 8'h95; // CMD0, arg = 32'h0000_0000
    localparam [7:0] SD_CRC_CMD8 = 8'h87; // CMD8, arg = 32'h0000_01AA

    // R1 response bit masks
    localparam [7:0] SD_R1_IDLE        = 8'h01;
    localparam [7:0] SD_R1_ERROR_BITS  = 8'h7E; // any bit other than IDLE set => error

    localparam [7:0] SD_TOKEN_DUMMY = 8'hFF;

    // SPI clock divider for spi_master (clk_div input), derived from
    // sys_clk = 20 MHz : sclk = sys_clk / (2*(clk_div+1))
    localparam [15:0] SPI_DIV_INIT = 16'd49; // ~200 kHz, required during card bring-up

    assign spi_clk_div = SPI_DIV_INIT;

    // ---------------------------------------------------------------
    // Command engine: the only block driving spi_start/spi_tx_data/cs_n.
    // Handles three kinds of requests from the top sequencer below:
    //   dummy_go  - clock out dummy_n bytes of 0xFF with CS high
    //   cmd_go    - send a 6-byte command frame, poll for R1 (<=NCR_MAX
    //               byte-times), then clock in extra_bytes trailer bytes
    // ---------------------------------------------------------------
    localparam CE_IDLE       = 4'd0,
               CE_SEND       = 4'd1,
               CE_SEND_WAIT  = 4'd2,
               CE_POLL       = 4'd3,
               CE_POLL_WAIT  = 4'd4,
               CE_EXTRA      = 4'd5,
               CE_EXTRA_WAIT = 4'd6,
               CE_CS_HIGH    = 4'd7,
               CE_DUMMY      = 4'd8,
               CE_DUMMY_WAIT = 4'd9,
               CE_CS_HIGH_WAIT = 4'd10;

    localparam NCR_MAX = 4'd8;

    reg [3:0]  ce_state;
    reg [5:0]  cmd_index;
    reg [31:0] cmd_arg;
    reg [7:0]  cmd_crc;
    reg [2:0]  extra_bytes;      // 0 or 4
    reg        cmd_go;
    reg        dummy_go;
    reg [4:0]  dummy_n;
    reg [4:0]  byte_idx;
    reg [3:0]  poll_cnt;
    reg [7:0]  r1_val;
    reg [31:0] resp_extra;
    reg        cmd_done, cmd_timeout, dummy_done;

    wire [7:0] cmd_byte0 = {2'b01, cmd_index};

    function [7:0] cmd_byte;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: cmd_byte = cmd_byte0;
                3'd1: cmd_byte = cmd_arg[31:24];
                3'd2: cmd_byte = cmd_arg[23:16];
                3'd3: cmd_byte = cmd_arg[15:8];
                3'd4: cmd_byte = cmd_arg[7:0];
                default: cmd_byte = cmd_crc;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            ce_state    <= CE_IDLE;
            spi_start   <= 1'b0;
            spi_tx_data <= 8'hFF;
            cs_n        <= 1'b1;
            byte_idx    <= 5'd0;
            poll_cnt    <= 4'd0;
            r1_val      <= 8'hFF;
            resp_extra  <= 32'd0;
            cmd_done    <= 1'b0;
            cmd_timeout <= 1'b0;
            dummy_done  <= 1'b0;
        end else begin
            spi_start  <= 1'b0;
            cmd_done   <= 1'b0;
            dummy_done <= 1'b0;

            case (ce_state)
                CE_IDLE: begin
                    cmd_timeout <= 1'b0;
                    if (dummy_go) begin
                        cs_n     <= 1'b1;
                        byte_idx <= 5'd0;
                        ce_state <= CE_DUMMY;
                    end else if (cmd_go) begin
                        cs_n     <= 1'b0;
                        byte_idx <= 5'd0;
                        ce_state <= CE_SEND;
                    end
                end

                CE_DUMMY: begin
                    if (!spi_busy) begin
                        spi_tx_data <= SD_TOKEN_DUMMY;
                        spi_start   <= 1'b1;
                        ce_state    <= CE_DUMMY_WAIT;
                    end
                end
                CE_DUMMY_WAIT: begin
                    if (spi_done) begin
                        if (byte_idx == dummy_n - 1) begin
                            dummy_done <= 1'b1;
                            ce_state   <= CE_IDLE;
                        end else begin
                            byte_idx <= byte_idx + 5'd1;
                            ce_state <= CE_DUMMY;
                        end
                    end
                end

                CE_SEND: begin
                    if (!spi_busy) begin
                        spi_tx_data <= cmd_byte(byte_idx[2:0]);
                        spi_start   <= 1'b1;
                        ce_state    <= CE_SEND_WAIT;
                    end
                end

                CE_SEND_WAIT: begin
                    if (spi_done) begin
                        if (byte_idx == 5'd5) begin
                            poll_cnt <= 4'd0;
                            ce_state <= CE_POLL;
                        end else begin
                            byte_idx <= byte_idx + 5'd1;
                            ce_state <= CE_SEND;
                        end
                    end
                end

                CE_POLL: begin
                    if (!spi_busy) begin
                        spi_tx_data <= SD_TOKEN_DUMMY;
                        spi_start   <= 1'b1;
                        ce_state    <= CE_POLL_WAIT;
                    end
                end

                CE_POLL_WAIT: begin
                    if (spi_done) begin
                        if (spi_rx_data[7] == 1'b0) begin
                            // valid R1 (MSB of a real R1 is always 0)
                            r1_val   <= spi_rx_data;
                            byte_idx <= 5'd0;
                            if (extra_bytes != 3'd0)
                                ce_state <= CE_EXTRA;
                            else
                                ce_state <= CE_CS_HIGH;
                        end else if (poll_cnt == NCR_MAX - 1) begin
                            cmd_timeout <= 1'b1;
                            ce_state    <= CE_CS_HIGH;
                        end else begin
                            poll_cnt <= poll_cnt + 4'd1;
                            ce_state <= CE_POLL;
                        end
                    end
                end

                CE_EXTRA: begin
                    if (!spi_busy) begin
                        spi_tx_data <= SD_TOKEN_DUMMY;
                        spi_start   <= 1'b1;
                        ce_state    <= CE_EXTRA_WAIT;
                    end
                end

                CE_EXTRA_WAIT: begin
                    if (spi_done) begin
                        resp_extra <= {resp_extra[23:0], spi_rx_data};
                        if (byte_idx == extra_bytes - 1) begin
                            ce_state <= CE_CS_HIGH;
                        end else begin
                            byte_idx <= byte_idx + 5'd1;
                            ce_state <= CE_EXTRA;
                        end
                    end
                end

                // Deassert CS and clock out one trailing dummy byte, then
                // WAIT for that transfer to actually finish before
                // reporting cmd_done - the top sequencer reacts to
                // cmd_done within a cycle by requesting the next
                // command, whose CE_IDLE branch reasserts CS
                // unconditionally. If cmd_done fired here immediately
                // (not waiting for spi_done), that reassert could land
                // while spi_master was still mid-transfer on this
                // trailing byte, corrupting the next command's framing.
                CE_CS_HIGH: begin
                    if (!spi_busy) begin
                        cs_n        <= 1'b1;
                        spi_tx_data <= SD_TOKEN_DUMMY;
                        spi_start   <= 1'b1;
                        ce_state    <= CE_CS_HIGH_WAIT;
                    end
                end
                CE_CS_HIGH_WAIT: begin
                    if (spi_done) begin
                        ce_state <= CE_IDLE;
                        cmd_done <= 1'b1;
                    end
                end

                default: ce_state <= CE_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------
    // Top-level init sequencer (only ever pulses dummy_go/cmd_go and
    // reads back cmd_done/dummy_done/r1_val/resp_extra - never touches
    // spi_start/spi_tx_data/cs_n directly).
    // ---------------------------------------------------------------
    localparam S_PWRUP_WAIT = 4'd0,
               S_DUMMY_CLK  = 4'd1,
               S_CMD0       = 4'd2,
               S_CMD0_CHK   = 4'd3,
               S_CMD8       = 4'd4,
               S_CMD8_CHK   = 4'd5,
               S_CMD55      = 4'd6,
               S_CMD55_CHK  = 4'd7,
               S_ACMD41     = 4'd8,
               S_ACMD41_CHK = 4'd9,
               S_CMD58      = 4'd10,
               S_CMD58_CHK  = 4'd11,
               S_DONE       = 4'd12,
               S_ERROR      = 4'd13;

    localparam PWRUP_CYCLES   = 20_000;          // 1 ms @ 20 MHz
    localparam DUMMY_BYTES    = 5'd20;           // 160 clocks, > required 74
    localparam ACMD41_TIMEOUT = 25'd20_000_000;  // 1 s @ 20 MHz

    reg [3:0]  state;
    reg [24:0] wait_cnt;
    reg        sd_ver2;             // CMD8 accepted -> spec ver >= 2.0

    assign busy = (state != S_DONE) && !error;

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_PWRUP_WAIT;
            wait_cnt    <= 25'd0;
            init_done   <= 1'b0;
            init_ok     <= 1'b0;
            sdhc        <= 1'b0;
            error       <= 1'b0;
            sd_ver2     <= 1'b0;
            cmd_go      <= 1'b0;
            dummy_go    <= 1'b0;
            dummy_n     <= 5'd0;
            cmd_index   <= 6'd0;
            cmd_arg     <= 32'd0;
            cmd_crc     <= 8'd0;
            extra_bytes <= 3'd0;
        end else begin
            cmd_go   <= 1'b0;
            dummy_go <= 1'b0;

            case (state)
                S_PWRUP_WAIT: begin
                    if (wait_cnt == PWRUP_CYCLES - 1) begin
                        wait_cnt <= 25'd0;
                        dummy_n  <= DUMMY_BYTES;
                        dummy_go <= 1'b1;
                        state    <= S_DUMMY_CLK;
                    end else begin
                        wait_cnt <= wait_cnt + 25'd1;
                    end
                end

                S_DUMMY_CLK: begin
                    if (dummy_done)
                        state <= S_CMD0;
                end

                S_CMD0: begin
                    cmd_index   <= SD_CMD0;
                    cmd_arg     <= 32'h0000_0000;
                    cmd_crc     <= SD_CRC_CMD0;
                    extra_bytes <= 3'd0;
                    cmd_go      <= 1'b1;
                    state       <= S_CMD0_CHK;
                end
                S_CMD0_CHK: begin
                    if (cmd_done) begin
                        if (cmd_timeout || r1_val != SD_R1_IDLE)
                            state <= S_ERROR;
                        else
                            state <= S_CMD8;
                    end
                end

                S_CMD8: begin
                    cmd_index   <= SD_CMD8;
                    cmd_arg     <= 32'h0000_01AA; // check pattern 0xAA, 2.7-3.6V
                    cmd_crc     <= SD_CRC_CMD8;
                    extra_bytes <= 3'd4;
                    cmd_go      <= 1'b1;
                    state       <= S_CMD8_CHK;
                end
                S_CMD8_CHK: begin
                    if (cmd_done) begin
                        if (cmd_timeout) begin
                            state <= S_ERROR;
                        end else if (r1_val[2]) begin
                            // illegal command -> SD ver1.x / MMC, no HCS
                            sd_ver2 <= 1'b0;
                            state   <= S_CMD55;
                        end else if (r1_val == SD_R1_IDLE &&
                                     resp_extra[7:0] == 8'hAA) begin
                            sd_ver2 <= 1'b1;
                            state   <= S_CMD55;
                        end else begin
                            state <= S_ERROR;
                        end
                    end
                end

                S_CMD55: begin
                    cmd_index   <= SD_CMD55;
                    cmd_arg     <= 32'h0000_0000;
                    cmd_crc     <= 8'hFF;
                    extra_bytes <= 3'd0;
                    cmd_go      <= 1'b1;
                    state       <= S_CMD55_CHK;
                end
                S_CMD55_CHK: begin
                    if (cmd_done) begin
                        if (cmd_timeout || (r1_val & SD_R1_ERROR_BITS) != 8'h00)
                            state <= S_ERROR;
                        else
                            state <= S_ACMD41;
                    end
                end

                S_ACMD41: begin
                    cmd_index   <= SD_CMD41;
                    cmd_arg     <= sd_ver2 ? 32'h4000_0000 : 32'h0000_0000; // HCS
                    cmd_crc     <= 8'hFF;
                    extra_bytes <= 3'd0;
                    cmd_go      <= 1'b1;
                    state       <= S_ACMD41_CHK;
                end
                S_ACMD41_CHK: begin
                    if (wait_cnt == ACMD41_TIMEOUT) begin
                        state <= S_ERROR;
                    end else if (cmd_done) begin
                        if (cmd_timeout || (r1_val & SD_R1_ERROR_BITS) != 8'h00) begin
                            state <= S_ERROR;
                        end else if (r1_val == 8'h00) begin
                            wait_cnt <= 25'd0;
                            state    <= S_CMD58;
                        end else begin
                            // still idle -> card busy powering up, retry
                            state <= S_CMD55;
                        end
                    end else begin
                        wait_cnt <= wait_cnt + 25'd1;
                    end
                end

                S_CMD58: begin
                    cmd_index   <= SD_CMD58;
                    cmd_arg     <= 32'h0000_0000;
                    cmd_crc     <= 8'hFF;
                    extra_bytes <= 3'd4;
                    cmd_go      <= 1'b1;
                    state       <= S_CMD58_CHK;
                end
                S_CMD58_CHK: begin
                    if (cmd_done) begin
                        if (cmd_timeout || r1_val != 8'h00) begin
                            state <= S_ERROR;
                        end else begin
                            sdhc  <= resp_extra[30]; // OCR bit30 = CCS
                            state <= S_DONE;
                        end
                    end
                end

                S_DONE: begin
                    init_done <= 1'b1;
                    init_ok   <= 1'b1;
                end

                S_ERROR: begin
                    init_done <= 1'b1;
                    init_ok   <= 1'b0;
                    error     <= 1'b1;
                end

                default: state <= S_ERROR;
            endcase
        end
    end

endmodule
