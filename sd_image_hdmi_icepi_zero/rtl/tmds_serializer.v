`timescale 1ns/1ps
// tmds_serializer : 10:1 gearbox, shift_clk domain (125 MHz = 5 x
// pix_clk). Takes the 10-bit TMDS symbol tmds_word (registered once per
// pix_clk cycle by tmds_encoder, and therefore stable for the entire
// pix_clk period) and serializes it onto `serial_out`, LSB (bit 0)
// first, using the real ECP5 ODDRX1F DDR output primitive: 2 bits per
// shift_clk cycle (one on the rising edge, one on the falling edge) x 5
// shift_clk cycles per pix_clk period = 10 bits/pix_clk, matching the
// TMDS symbol rate exactly.
//
// pair_idx cycles 0..4, sending bits {2*pair_idx, 2*pair_idx+1} each
// shift_clk cycle. The NEXT symbol is captured into word_r while
// pair_idx==4 (the last cycle of the current group) rather than at
// pair_idx==0, so the nonblocking update lands in time to be visible
// combinationally the instant pair_idx wraps back to 0 - capturing at
// pair_idx==0 instead would combinationally read the just-finished old
// word for one extra cycle (bits 0/1 of the new symbol would be sent a
// cycle late, using stale data).
//
// *** shift_clk and pix_clk MUST come from the same PLL instance with a
// fixed, static phase relationship (see icepi_clk_wiz_video.v) - this
// module has no CDC synchronizer between the two, relying entirely on
// that fixed relationship. Which of the 5 shift_clk edges per pix_clk
// period this module treats as "capture a new symbol" is arbitrary
// (fixed at whatever phase the free-running counter below happens to
// power up in) and NOT re-aligned to pix_clk at runtime. This is safe
// from a protocol standpoint (tmds_word is stable for the whole
// pix_clk period, so any of the 5 phases reads a valid value most of
// the time), but the actual setup/hold margin for the specific cycle
// that captures right before tmds_word changes depends on the real
// EHXPLLL CPHASE relationship between CLKOP and CLKOS, which can only
// be checked with Diamond's static timing analysis (Trace report) on
// real hardware - not something verifiable from RTL simulation alone.
// If the real display is unstable/glitchy after bringing this up on
// hardware, check Trace's cross-clock timing between CLKOP and CLKOS
// first; the fix is normally adjusting CLKOP_CPHASE/CLKOS_CPHASE via
// Clarity Designer or adding a pipeline stage, not an RTL rewrite. ***
module tmds_serializer (
    input  wire shift_clk,   // 125 MHz, 5x pix_clk, same PLL/VCO as pix_clk
    input  wire rst,
    input  wire [9:0] tmds_word,
    output wire serial_out
);
    reg [9:0] word_r;
    reg [2:0] pair_idx; // 0..4

    wire       wrap          = (pair_idx == 3'd4);
    wire [2:0] next_pair_idx = wrap ? 3'd0 : pair_idx + 3'd1;
    wire [9:0] next_word_r   = wrap ? tmds_word : word_r;

    always @(posedge shift_clk or posedge rst) begin
        if (rst) begin
            pair_idx <= 3'd0;
            word_r   <= 10'd0;
        end else begin
            word_r   <= next_word_r;
            pair_idx <= next_pair_idx;
        end
    end

    // d0 (this edge's rising-edge bit) must reflect the bit at the
    // pair position pair_idx is ABOUT TO become, so it's derived from
    // next_pair_idx/next_word_r rather than the pre-edge pair_idx/
    // word_r. d1 (the falling-edge bit, used a half-cycle later) is
    // safe to read directly off the already-updated, now-stable
    // pair_idx/word_r.
    wire d0 = next_word_r[{next_pair_idx, 1'b0}];
    wire d1 = word_r[{pair_idx, 1'b1}];

`ifdef SIMULATION
    // Behavioral DDR model. d0/d1 above are fine as combinational
    // feeds for a real ODDRX1F (which samples them with its own
    // internal register, immune to pre-edge settling), but a plain
    // level-sensitive `assign serial_out = shift_clk ? d0 : d1` here
    // would glitch: d0/d1 depend on pair_idx/word_r, which update at
    // this exact edge, so the continuous assign re-evaluates multiple
    // times within the same simulation instant as those dependencies
    // settle, and simulation-only edge counting (e.g. a testbench
    // measuring this clock's frequency) can see a stray extra
    // transition. Capturing d0/d1 into a register at the exact edge
    // that defines each of them - posedge for d0, negedge for d1 -
    // instead of continuously re-deriving the output from live wires,
    // gives a clean, glitch-free step function matching how a real
    // DDR flip-flop's Q output actually behaves (Q only changes AT
    // clock edges, it doesn't "watch" its D input in between).
    reg serial_out_r;
    always @(posedge shift_clk or posedge rst) begin
        if (rst) serial_out_r <= 1'b0;
        else     serial_out_r <= d0;
    end
    always @(negedge shift_clk or posedge rst) begin
        if (rst) serial_out_r <= 1'b0;
        else     serial_out_r <= d1;
    end
    assign serial_out = serial_out_r;
`else
    ODDRX1F oddr_inst (
        .D0   (d0),
        .D1   (d1),
        .SCLK (shift_clk),
        .RST  (rst),
        .Q    (serial_out)
    );
`endif
endmodule
