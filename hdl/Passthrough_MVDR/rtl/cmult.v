// -----------------------------------------------------------------------------
// cmult.v  --  pipelined complex multiplier, full-precision output
//
//   CONJ_B = 0 :  y = a * b
//   CONJ_B = 1 :  y = a * conj(b)
//
//   a = ar + j*ai   (A_W bits each, signed)
//   b = br + j*bi   (B_W bits each, signed)
//   y = yr + j*yi   (A_W + B_W + 1 bits each, signed)  -> no overflow possible
//
//   CONJ_B = 0:  yr = ar*br - ai*bi ;  yi = ar*bi + ai*br
//   CONJ_B = 1:  yr = ar*br + ai*bi ;  yi = ai*br - ar*bi
//
// Timing: 2 + IN_REG clock latency, one result per clock (valid follows data).
//   IN_REG = 1 (v2 default) registers a and b before the multipliers, so each
//   DSP48 sees flop -> multiply -> flop (AREG/BREG/MREG): the v1 path was
//   fabric route + 15x18 multiply in one clock, marginal at 222 MHz on a -1
//   Zynq-7020. Both operands go through the same stage, so a sample and the
//   weight it is multiplied by stay paired.
// Cost:   4 real multipliers (maps to 4 DSP slices).  A 3-multiplier version
//         exists but needs extra adders and one more pipeline stage; not worth
//         it at this size.
// Reset only clears the valid pipeline; data registers are not reset (saves
// routing, and outputs are qualified by out_valid anyway).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module cmult #(
    parameter A_W    = 16,
    parameter B_W    = 18,
    parameter CONJ_B = 0,
    parameter IN_REG = 1,
    parameter OUT_W  = A_W + B_W + 1
)(
    input  wire                     clk,
    input  wire                     rst,       // synchronous, active high

    input  wire                     in_valid,
    input  wire signed [A_W-1:0]    ar,
    input  wire signed [A_W-1:0]    ai,
    input  wire signed [B_W-1:0]    br,
    input  wire signed [B_W-1:0]    bi,

    output reg                      out_valid,
    output reg  signed [OUT_W-1:0]  yr,
    output reg  signed [OUT_W-1:0]  yi
);

    localparam PW = A_W + B_W;

    // ---------------- stage 0 (IN_REG): operand registers ----------------
    reg                   v0;
    reg signed [A_W-1:0]  ar0, ai0;
    reg signed [B_W-1:0]  br0, bi0;
    always @(posedge clk) begin
        if (rst) v0 <= 1'b0;
        else     v0 <= in_valid;
        ar0 <= ar;  ai0 <= ai;  br0 <= br;  bi0 <= bi;
    end
    wire                  vm  = (IN_REG != 0) ? v0  : in_valid;
    wire signed [A_W-1:0] xar = (IN_REG != 0) ? ar0 : ar;
    wire signed [A_W-1:0] xai = (IN_REG != 0) ? ai0 : ai;
    wire signed [B_W-1:0] xbr = (IN_REG != 0) ? br0 : br;
    wire signed [B_W-1:0] xbi = (IN_REG != 0) ? bi0 : bi;

    // ---------------- stage 1: four real products ----------------
    reg                   v1;
    reg signed [PW-1:0]   m_rr, m_ii, m_ri, m_ir;

    always @(posedge clk) begin
        if (rst) v1 <= 1'b0;
        else     v1 <= vm;
        m_rr <= xar * xbr;
        m_ii <= xai * xbi;
        m_ri <= xar * xbi;
        m_ir <= xai * xbr;
    end

    // ---------------- stage 2: combine ----------------
    always @(posedge clk) begin
        if (rst) out_valid <= 1'b0;
        else     out_valid <= v1;

        if (CONJ_B == 0) begin
            yr <= m_rr - m_ii;
            yi <= m_ri + m_ir;
        end else begin
            yr <= m_rr + m_ii;
            yi <= m_ir - m_ri;
        end
    end

endmodule
