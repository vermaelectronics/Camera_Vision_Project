// ============================================================================
// dp_line_ram.v -- one video line's worth of pixel storage, true dual-port,
// fully independent read/write clocks
// ----------------------------------------------------------------------------
// A plain `reg [...] mem [0:DEPTH-1]` with one clocked write port and one
// clocked read port, the same style already used by async_fifo.v -- Yosys's
// synth_ecp5 reliably infers this as a real ECP5 DP16KD true-dual-port block
// RAM. Used by video_line_buffer.v, instantiated N_LINES times.
//
// No read-during-write-same-address collision handling beyond whatever the
// real DP16KD primitive itself does (undefined result) -- never exercised
// here, since the two ports are never addressed at the same real-world
// (row, column) at the same instant by construction: see video_line_buffer.v
// for why the write side is always at least one whole line ahead of
// whichever line the read side is currently consuming.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module dp_line_ram #(
    parameter DEPTH  = 1280,
    parameter DATA_W = 24,
    parameter ADDR_W = $clog2(DEPTH)
) (
    input  wire                  wr_clk,
    input  wire                  wr_en,
    input  wire [ADDR_W-1:0]     wr_addr,
    input  wire [DATA_W-1:0]     wr_data,

    input  wire                  rd_clk,
    input  wire                  rd_en,
    input  wire [ADDR_W-1:0]     rd_addr,
    output reg  [DATA_W-1:0]     rd_data
);

    reg [DATA_W-1:0] mem [0:DEPTH-1];

    always @(posedge wr_clk)
        if (wr_en) mem[wr_addr] <= wr_data;

    always @(posedge rd_clk)
        if (rd_en) rd_data <= mem[rd_addr];

endmodule

`default_nettype wire
