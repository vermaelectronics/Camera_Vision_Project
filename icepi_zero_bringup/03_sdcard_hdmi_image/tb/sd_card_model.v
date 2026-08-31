// ============================================================================
// sd_card_model.v -- SIMULATION-ONLY behavioral SD card SPI slave.
// ----------------------------------------------------------------------------
// Not synthesizable, not used anywhere in the real build -- this exists
// purely so tb_sdcard_text_reader.v can exercise sdcard_spi.v and
// fat16_reader.v against protocol-correct command/response traffic without
// real hardware. It understands exactly the commands sdcard_spi.v sends
// (CMD0, CMD8, CMD55, ACMD41, CMD58, CMD16, CMD17) and nothing else.
//
// Simplifications explicitly accepted here (real cards vary; this model
// picks the simplest legal behavior in every case):
//   - Always responds with Ncr=0 (the very next byte-time), never the
//     small random response delay real cards insert.
//   - Emulates an SD v2 SDHC/SDXC card (CCS=1): CMD8 succeeds, ACMD41 goes
//     ready after a couple of idle polls (exercises sdcard_spi's retry
//     loop), CMD17's argument is a block number (not a byte address) --
//     the byte-address (SDSC) path in sdcard_spi is NOT exercised by this
//     model (see the sub-project README's "Known limitations").
//   - CMD17 always succeeds with a fixed, short token-wait delay; it does
//     not model read errors or out-of-range addresses.
//
// Response bytes for one command are fully precomputed into resp_queue[]
// the moment the 6th command byte is received (using the already-loaded
// `blocks` memory for CMD17), then shifted out one byte-time per call --
// this makes commands with wildly different response shapes (R1-only vs.
// R1+512 data bytes) share one uniform output loop.
// ============================================================================
`timescale 1ns / 1ps

module sd_card_model #(
    parameter integer NUM_BLOCKS = 8
) (
    input  wire sclk,
    input  wire mosi,
    output reg  miso,
    input  wire cs_n
);

    reg [7:0] blocks [0:NUM_BLOCKS*512-1];

    // ---- incoming byte assembly ----
    reg [7:0] rx_shift;
    reg [2:0] bit_cnt;

    localparam MODE_WAIT_CMD = 1'b0, MODE_BUSY = 1'b1;
    reg mode;

    reg [7:0] cmd_buf [0:5];
    reg [2:0] cmd_byte_idx;

    // ---- outgoing response queue ----
    reg [7:0] resp_queue [0:519]; // worst case: R1 + wait + token + 512 data + 2 CRC
    integer   resp_len;
    integer   resp_ptr;
    reg [7:0] resp_shift;

    reg       last_was_cmd55;
    integer   acmd41_polls;

    integer i;

    initial begin
        miso            = 1'b1;
        bit_cnt          = 3'd0;
        mode             = MODE_WAIT_CMD;
        cmd_byte_idx     = 3'd0;
        resp_len         = 0;
        resp_ptr         = 0;
        resp_shift       = 8'hFF;
        last_was_cmd55   = 1'b0;
        acmd41_polls     = 0;
    end

    task decode_command;
        reg [5:0] idx;
        reg [31:0] arg;
        reg [31:0] blk;
        integer    j;
        begin
            idx = cmd_buf[0][5:0];
            arg = {cmd_buf[1], cmd_buf[2], cmd_buf[3], cmd_buf[4]};

            case (idx)
                6'd0: begin // CMD0 -> GO_IDLE_STATE
                    resp_queue[0] = 8'h01;
                    resp_len = 1;
                    last_was_cmd55 = 1'b0;
                end

                6'd8: begin // CMD8 -> SEND_IF_COND (echo the check pattern)
                    resp_queue[0] = 8'h01;
                    resp_queue[1] = 8'h00;
                    resp_queue[2] = 8'h00;
                    resp_queue[3] = arg[11:8]; // voltage nibble echoed back
                    resp_queue[4] = arg[7:0];  // check pattern echoed back
                    resp_len = 5;
                    last_was_cmd55 = 1'b0;
                end

                6'd55: begin // CMD55 -> APP_CMD (next CMD41 means ACMD41)
                    resp_queue[0] = 8'h01;
                    resp_len = 1;
                    last_was_cmd55 = 1'b1;
                end

                6'd41: begin // ACMD41 -> SD_SEND_OP_COND
                    if (last_was_cmd55) begin
                        if (acmd41_polls < 2) begin
                            acmd41_polls = acmd41_polls + 1;
                            resp_queue[0] = 8'h01; // still idle -- exercise the retry loop
                        end else begin
                            resp_queue[0] = 8'h00; // ready
                        end
                    end else begin
                        resp_queue[0] = 8'h05; // illegal command (not an ACMD)
                    end
                    resp_len = 1;
                    last_was_cmd55 = 1'b0;
                end

                6'd58: begin // CMD58 -> READ_OCR (CCS=1: emulate SDHC/SDXC)
                    resp_queue[0] = 8'h00;
                    resp_queue[1] = 8'hC0; // busy=1, CCS=1
                    resp_queue[2] = 8'hFF;
                    resp_queue[3] = 8'h80;
                    resp_queue[4] = 8'h00;
                    resp_len = 5;
                    last_was_cmd55 = 1'b0;
                end

                6'd16: begin // CMD16 -> SET_BLOCKLEN (ignored/harmless on SDHC)
                    resp_queue[0] = 8'h00;
                    resp_len = 1;
                    last_was_cmd55 = 1'b0;
                end

                6'd17: begin // CMD17 -> READ_SINGLE_BLOCK (block-addressed, SDHC-style)
                    resp_queue[0] = 8'h00; // R1: accepted
                    // a short, fixed "thinking" delay before the data token
                    resp_queue[1] = 8'hFF;
                    resp_queue[2] = 8'hFF;
                    resp_queue[3] = 8'hFF;
                    resp_queue[4] = 8'hFF;
                    resp_queue[5] = 8'hFE; // data start token
                    blk = arg;
                    for (j = 0; j < 512; j = j + 1) begin
                        if (blk < NUM_BLOCKS)
                            resp_queue[6 + j] = blocks[blk * 512 + j];
                        else
                            resp_queue[6 + j] = 8'h00;
                    end
                    resp_queue[6 + 512]     = 8'h00; // CRC (unchecked)
                    resp_queue[6 + 512 + 1] = 8'h00;
                    resp_len = 6 + 512 + 2;
                    last_was_cmd55 = 1'b0;
                end

                default: begin
                    resp_queue[0] = 8'h05; // illegal command
                    resp_len = 1;
                    last_was_cmd55 = 1'b0;
                end
            endcase
        end
    endtask

    // ---- drive MISO on falling edges (mode 0: slave outputs change on the
    // falling edge, sampled by the master on the next rising edge) ----
    always @(negedge sclk or posedge cs_n) begin
        if (cs_n) begin
            miso <= 1'b1;
        end else begin
            miso       <= resp_shift[7];
            resp_shift <= {resp_shift[6:0], 1'b0};
        end
    end

    // ---- sample MOSI on rising edges; once per full byte, advance the
    // command-collection or response-emission state ----
    always @(posedge sclk) begin
        if (!cs_n) begin
            rx_shift <= {rx_shift[6:0], mosi};
            if (bit_cnt == 3'd7) begin
                bit_cnt <= 3'd0;

                if (mode == MODE_WAIT_CMD) begin
                    cmd_buf[cmd_byte_idx] <= {rx_shift[6:0], mosi};
                    if (cmd_byte_idx == 3'd5) begin
                        cmd_buf[5] <= {rx_shift[6:0], mosi};
                        decode_command; // uses cmd_buf[0..4] already latched; cmd_buf[5]=CRC unused
                        resp_shift   <= resp_queue[0];
                        resp_ptr     <= 1;
                        mode         <= MODE_BUSY;
                        cmd_byte_idx <= 3'd0;
                    end else begin
                        cmd_byte_idx <= cmd_byte_idx + 1'b1;
                    end
                end else begin // MODE_BUSY: emit the next queued response byte
                    if (resp_ptr < resp_len) begin
                        resp_shift <= resp_queue[resp_ptr];
                        resp_ptr   <= resp_ptr + 1'b1;
                    end else begin
                        mode <= MODE_WAIT_CMD; // transaction complete
                    end
                end
            end else begin
                bit_cnt <= bit_cnt + 1'b1;
            end
        end
    end

endmodule
