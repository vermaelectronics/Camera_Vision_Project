// ============================================================================
// async_fifo.v  --  Generic dual-clock (asynchronous) FIFO
// ----------------------------------------------------------------------------
// Standard Gray-code pointer / 2-flop synchronizer design (Cummings, SNUG2002
// "Simulation and Synthesis Techniques for Asynchronous FIFO Design").
// Used everywhere in this project that two genuinely independent clocks must
// exchange data safely:
//   - camera PCLK domain  -> video pixel-clock domain (line buffering)
//   - video pixel-clock domain -> TMDS bit-clock (SCLK) domain (serializer)
//
// Fully synthesizable. DEPTH must be a power of two.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module async_fifo #(
    parameter WIDTH = 8,
    parameter DEPTH = 16,                 // must be power of 2
    parameter ADDR_W = $clog2(DEPTH)
) (
    // write side
    input  wire               wr_clk,
    input  wire               wr_rst,     // synchronous to wr_clk, active high
    input  wire                wr_en,
    input  wire [WIDTH-1:0]   wr_data,
    output wire                wr_full,
    output wire [ADDR_W:0]    wr_level,   // words currently buffered (wr_clk domain estimate)

    // read side
    input  wire                rd_clk,
    input  wire                rd_rst,    // synchronous to rd_clk, active high
    input  wire                rd_en,
    output reg  [WIDTH-1:0]   rd_data,
    output wire                rd_empty
);

    // Memory: inferred as ECP5 DP16KD true-dual-port block RAM by Yosys/nextpnr.
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    // ------------------------------------------------------------------
    // Write-side pointer (binary + Gray) and full flag
    // ------------------------------------------------------------------
    reg [ADDR_W:0] wr_bin = {(ADDR_W+1){1'b0}};
    reg [ADDR_W:0] wr_gray = {(ADDR_W+1){1'b0}};

    // "if we were to write this cycle" pointer/Gray value, computed
    // UNCONDITIONALLY from the current (registered) pointer only -- this is
    // what wr_full is predicted from. Note this deliberately does NOT depend
    // on wr_full itself: gating it by wr_full here would create a
    // combinational loop (wr_full -> wr_bin_p1 -> wr_gray_p1 -> wr_full).
    // The actual conditional pointer update happens only in the clocked
    // always block below, as a register-enable, not in this expression.
    wire [ADDR_W:0] wr_bin_p1  = wr_bin + 1'b1;
    wire [ADDR_W:0] wr_gray_p1 = (wr_bin_p1 >> 1) ^ wr_bin_p1;

    // Read pointer synchronized into write clock domain
    reg [ADDR_W:0] rd_gray_s1 = 0, rd_gray_s2 = 0;

    // rd_gray_ptr itself is the read-side pointer register, declared and
    // driven in the "Read-side pointer" section further down -- declared
    // here (ahead of its first use, in the write-domain synchronizer
    // immediately below) purely for declare-before-use portability. Some
    // Icarus Verilog builds (this was found against an older bundled
    // release) reject a forward reference to a reg declared later in the
    // same module, even though later-declared-is-fine is standard Verilog;
    // declaring it here costs nothing and avoids the whole class of issue.
    reg [ADDR_W:0] rd_bin = {(ADDR_W+1){1'b0}};
    reg [ADDR_W:0] rd_gray_ptr = {(ADDR_W+1){1'b0}};

    // "Already full" -- built from the CURRENT (already registered) wr_gray,
    // not the predictive wr_gray_p1. Using the predictive value here (the
    // textbook Cummings formulation, meant for an *external* controller to
    // poll one cycle ahead of asserting its own write-enable) would, if used
    // to gate the write in the very same cycle as this module does, reject
    // the write that legitimately fills the last free slot -- capping usable
    // capacity at DEPTH-1 instead of DEPTH. Comparing the current pointer
    // avoids that off-by-one and keeps the write gate a simple, safe,
    // same-cycle self-contained check (verified exhaustively in
    // tb/tb_async_fifo.v: exactly DEPTH words are accepted before wr_full).
    assign wr_full = (wr_gray == {~rd_gray_s2[ADDR_W:ADDR_W-1], rd_gray_s2[ADDR_W-2:0]});

    always @(posedge wr_clk) begin
        if (wr_rst) begin
            wr_bin  <= 0;
            wr_gray <= 0;
        end else if (wr_en && !wr_full) begin
            wr_bin  <= wr_bin_p1;
            wr_gray <= wr_gray_p1;
        end
    end

    always @(posedge wr_clk) begin
        if (wr_rst) begin
            rd_gray_s1 <= 0;
            rd_gray_s2 <= 0;
        end else begin
            rd_gray_s1 <= rd_gray_ptr;
            rd_gray_s2 <= rd_gray_s1;
        end
    end

    always @(posedge wr_clk) begin
        if (wr_en && !wr_full)
            mem[wr_bin[ADDR_W-1:0]] <= wr_data;
    end

    // Approximate fill level in the write-clock domain (for margin checks /
    // almost-full logic upstream). Not required for correctness.
    reg [ADDR_W:0] rd_bin_wrdom = 0;
    integer gi;
    always @(*) begin
        // Gray -> binary of the synchronized read pointer, purely combinational
        rd_bin_wrdom = rd_gray_s2;
        for (gi = 1; gi <= ADDR_W; gi = gi + 1)
            rd_bin_wrdom = rd_bin_wrdom ^ (rd_gray_s2 >> gi);
    end
    assign wr_level = wr_bin - rd_bin_wrdom;

    // ------------------------------------------------------------------
    // Read-side pointer (binary + Gray) and empty flag
    // ------------------------------------------------------------------
    // (rd_bin / rd_gray_ptr themselves are declared up top, near the
    // write-domain synchronizer that needs to see them first -- see the
    // comment there.)

    // Same construction as the write side: "if we were to read this cycle"
    // pointer, computed unconditionally so it never feeds back into
    // rd_empty combinationally (see wr_bin_p1 comment above).
    wire [ADDR_W:0] rd_bin_p1  = rd_bin + 1'b1;
    wire [ADDR_W:0] rd_gray_p1 = (rd_bin_p1 >> 1) ^ rd_bin_p1;

    // Write pointer synchronized into read clock domain
    reg [ADDR_W:0] wr_gray_s1 = 0, wr_gray_s2 = 0;

    always @(posedge rd_clk) begin
        if (rd_rst) begin
            rd_bin      <= 0;
            rd_gray_ptr <= 0;
        end else if (rd_en && !rd_empty) begin
            rd_bin      <= rd_bin_p1;
            rd_gray_ptr <= rd_gray_p1;
        end
    end

    always @(posedge rd_clk) begin
        if (rd_rst) begin
            wr_gray_s1 <= 0;
            wr_gray_s2 <= 0;
        end else begin
            wr_gray_s1 <= wr_gray;
            wr_gray_s2 <= wr_gray_s1;
        end
    end

    // Empty when the *current* (already registered) read pointer has caught
    // up with the synchronized write pointer -- a pure register-to-register
    // comparison, so (unlike wr_full's "would writing overflow?" check)
    // there is no need for a speculative "next pointer" term here.
    assign rd_empty = (rd_gray_ptr == wr_gray_s2);

    always @(posedge rd_clk) begin
        if (rd_rst)
            rd_data <= {WIDTH{1'b0}};
        else if (rd_en && !rd_empty)
            rd_data <= mem[rd_bin[ADDR_W-1:0]];
    end

endmodule

`default_nettype wire
