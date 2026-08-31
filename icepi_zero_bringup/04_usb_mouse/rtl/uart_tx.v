// ============================================================================
// uart_tx.v -- minimal 8N1 UART transmitter.
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
    input  wire       valid,   // pulse while `ready`, to send one byte
    output reg         ready,   // high while idle, safe to pulse `valid`
    output wire        tx
);

    localparam integer DIV = CLK_FREQ_HZ / BAUD;
    localparam integer CW  = $clog2(DIV);

    reg [CW-1:0] cnt;
    reg [3:0]    bit_idx;   // 0 = start bit .. 9 = stop bit
    reg [9:0]    shreg;     // {stop, data[7:0], start}, shifted out LSB-first
    reg          busy;

    assign tx = shreg[0];

    always @(posedge clk) begin
        if (rst) begin
            shreg   <= 10'b1111111111; // idle-high line
            busy    <= 1'b0;
            ready   <= 1'b1;
            cnt     <= 0;
            bit_idx <= 0;
        end else if (!busy) begin
            ready <= 1'b1;
            if (valid) begin
                shreg   <= {1'b1, data, 1'b0};
                busy    <= 1'b1;
                ready   <= 1'b0;
                cnt     <= 0;
                bit_idx <= 0;
            end
        end else begin
            if (cnt == DIV - 1) begin
                cnt   <= 0;
                shreg <= {1'b1, shreg[9:1]};
                if (bit_idx == 4'd9)
                    busy <= 1'b0;
                else
                    bit_idx <= bit_idx + 1'b1;
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
