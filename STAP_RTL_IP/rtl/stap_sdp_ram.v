//-----------------------------------------------------------------------------
// stap_sdp_ram : simple dual-port RAM (1 write, 1 synchronous read port)
//   Infers block RAM on FPGA; replace by a memory compiler macro for ASIC.
//   Read-during-write to the same address returns the old data.
//-----------------------------------------------------------------------------
module stap_sdp_ram #(
    parameter integer DW    = 64,
    parameter integer DEPTH = 1024,
    parameter integer AW    = $clog2(DEPTH)
) (
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);
    reg [DW-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end
endmodule
