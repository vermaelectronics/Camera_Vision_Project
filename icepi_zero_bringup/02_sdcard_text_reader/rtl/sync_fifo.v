// ============================================================================
// sync_fifo.v -- plain single-clock-domain FIFO, one clock, no CDC needed.
// Used here to absorb the rate mismatch between the SD card's burst-y
// per-block byte stream and the much slower UART transmitter.
// DEPTH must be a power of 2.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module sync_fifo #(
    parameter integer WIDTH = 8,
    parameter integer DEPTH = 1024
) (
    input  wire             clk,
    input  wire             rst,

    input  wire             wr_en,
    input  wire [WIDTH-1:0] wr_data,
    output wire              full,

    input  wire             rd_en,
    output reg  [WIDTH-1:0] rd_data,
    output wire              empty,

    output wire [$clog2(DEPTH):0] count
);

    localparam AW = $clog2(DEPTH);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [AW:0]      wr_ptr;
    reg [AW:0]      rd_ptr;

    wire [AW-1:0] wr_addr = wr_ptr[AW-1:0];
    wire [AW-1:0] rd_addr = rd_ptr[AW-1:0];

    assign full  = (wr_ptr[AW] != rd_ptr[AW]) && (wr_addr == rd_addr);
    assign empty = (wr_ptr == rd_ptr);
    assign count = wr_ptr - rd_ptr;

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr <= {(AW+1){1'b0}};
        end else if (wr_en && !full) begin
            mem[wr_addr] <= wr_data;
            wr_ptr       <= wr_ptr + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            rd_ptr <= {(AW+1){1'b0}};
        end else if (rd_en && !empty) begin
            rd_data <= mem[rd_addr];
            rd_ptr  <= rd_ptr + 1'b1;
        end
    end

endmodule

`default_nettype wire
