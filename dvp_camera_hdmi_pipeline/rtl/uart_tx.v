// ============================================================================
// uart_tx.v -- generic 8N1 UART transmitter
// ----------------------------------------------------------------------------
// Standard asynchronous serial framing: 1 start bit (low), 8 data bits
// (LSB first), 1 stop bit (high), idle high. Pulse `start` for one clock
// while `busy` is low to send `data`; `busy` stays high for the whole
// 10-bit-period frame. Used by uart_debug.v to report live hardware status
// to a terminal (PuTTY, minicom, etc.) over a USB-TTL adapter.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module uart_tx #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD        = 115200
) (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data,
    input  wire       start,   // pulse 1 cycle to send `data`; ignored while busy
    output reg         tx,      // idles high
    output reg         busy
);

    localparam integer DIV   = CLK_FREQ_HZ / BAUD;
    localparam integer DIV_W = (DIV <= 1) ? 1 : $clog2(DIV);

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0]       state;
    reg [DIV_W-1:0] baud_cnt;
    reg [2:0]       bit_idx;
    reg [7:0]       shift;

    wire baud_tick = (baud_cnt == DIV-1);

    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            tx       <= 1'b1;
            busy     <= 1'b0;
            baud_cnt <= 0;
            bit_idx  <= 0;
            shift    <= 8'h00;
        end else begin
            case (state)
                S_IDLE: begin
                    tx       <= 1'b1;
                    busy     <= 1'b0;
                    baud_cnt <= 0;
                    if (start) begin
                        shift    <= data;
                        busy     <= 1'b1;
                        tx       <= 1'b0;   // start bit
                        baud_cnt <= 0;
                        state    <= S_START;
                    end
                end

                S_START: begin
                    if (baud_tick) begin
                        baud_cnt <= 0;
                        bit_idx  <= 0;
                        tx       <= shift[0];
                        state    <= S_DATA;
                    end else begin
                        baud_cnt <= baud_cnt + 1'b1;
                    end
                end

                S_DATA: begin
                    if (baud_tick) begin
                        baud_cnt <= 0;
                        if (bit_idx == 3'd7) begin
                            tx    <= 1'b1;  // stop bit
                            state <= S_STOP;
                        end else begin
                            shift   <= {1'b0, shift[7:1]};
                            bit_idx <= bit_idx + 1'b1;
                            tx      <= shift[1];
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 1'b1;
                    end
                end

                S_STOP: begin
                    if (baud_tick) begin
                        baud_cnt <= 0;
                        busy     <= 1'b0;
                        state    <= S_IDLE;
                    end else begin
                        baud_cnt <= baud_cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
