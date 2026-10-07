/*******************************************************************************
 * pi_reciprocal.v
 *
 * Fully pipelined unsigned fixed-point reciprocal: q_out(k) = floor(2^Q_FRAC
 * / d_in(k)), i.e. q_out(k)/2^Q_FRAC is a UQ0.Q_FRAC approximation of
 * 1/d_in(k).
 *
 * Built specifically for pi_power_inversion_normalized.v's
 * alpha(k) = 1/(gamma + 2*sum_i|x_i(k)|^2) (Eq. 13 of Meng/Feng/Lu,
 * "Anti-Jamming with Adaptive Arrays Utilizing Power Inversion Algorithm",
 * Tsinghua Sci. & Tech. 13(6), 2008) -- but this module itself is a generic
 * pipelined reciprocal, not specific to that use.
 *
 * ALGORITHM -- standard radix-2 restoring division, unrolled into one
 * pipeline stage per output bit (a systolic/array divider):
 *
 *   remainder = 0
 *   for i = Q_FRAC downto 0:
 *       remainder = (remainder << 1) | dividend_bit[i]   -- dividend = 2^Q_FRAC,
 *                                                            so this bit is 1
 *                                                            only when i==Q_FRAC
 *       if remainder >= D:
 *           remainder = remainder - D
 *           quotient[i] = 1
 *       else:
 *           quotient[i] = 0
 *
 * D is fixed for the whole computation of one result (carried unchanged
 * through every stage), so this is exactly that loop, unrolled in SPACE
 * instead of iterated in TIME: stage i performs iteration (Q_FRAC-i) [stage
 * 0 = i=Q_FRAC, the MSB], then passes its remainder, D, quotient-so-far and
 * a valid bit to the next stage through a register. That costs Q_FRAC+1
 * cycles of LATENCY but accepts a NEW d_in every single clock (THROUGHPUT =
 * 1/cycle) -- required here because sample_valid in the caller pulses
 * roughly every 4 clk cycles, far faster than a Q_FRAC+1-cycle SERIAL
 * divider (one bit per cycle, blocking) could keep up with one at a time.
 *
 * WHY THE DIVIDEND IS A CONSTANT, NOT A PORT
 *   This core only ever computes 1/D, never a general A/D -- that is all
 *   pi_power_inversion_normalized.v needs (Eq. 13's numerator is the literal
 *   constant 1, not a signal). Hard-coding the dividend as 2^Q_FRAC (a
 *   single set bit) instead of taking it as a runtime input is what makes
 *   each stage just "shift in dividend_bit[i], compare, maybe subtract"
 *   instead of a full general-purpose divider -- simpler, and it is
 *   genuinely all this needs.
 *
 * SATURATION IS AUTOMATIC, NOT EXTRA LOGIC
 *   The quotient register is exactly Q_FRAC+1 bits wide, so a result that
 *   would need more bits (d_in < 1, i.e. d_in == 0 for an integer) simply
 *   cannot be represented -- and that is the ONE input value this structure
 *   cannot produce a meaningful reciprocal of anyway. The caller's GAMMA
 *   regularisation (Eq. 13's gamma > 0) exists precisely to guarantee d_in
 *   is never 0; this module trusts that and does not re-check it.
 ******************************************************************************/
`timescale 1ns/1ps

module pi_reciprocal #(
    parameter integer D_W    = 36,   // width of the unsigned denominator input
    parameter integer Q_FRAC = 32    // fractional bits of the UQ0.Q_FRAC quotient;
                                      // pipeline depth = Q_FRAC+1 cycles
) (
    input  wire               clk,
    input  wire               aresetn,       // synchronous, active low

    input  wire                d_valid,
    input  wire [D_W-1:0]      d_in,          // D(k), must be >= 1 (caller's job)

    output wire                q_valid,
    output wire [Q_FRAC:0]     q_out          // UQ0.Q_FRAC: q_out / 2^Q_FRAC ~= 1/d_in
);

    // Remainder must hold a D_W-bit value plus the one shifted-in dividend
    // bit, so D_W+1 bits is enough for every stage's compare/subtract.
    localparam integer R_W = D_W + 1;

    // One pipeline register set per stage: stage i's inputs are stage
    // (i-1)'s registered outputs. Stage 0's inputs are the module inputs
    // (remainder = 0, quotient-so-far = 0).
    reg [R_W-1:0]  rem_pipe   [0:Q_FRAC];
    reg [D_W-1:0]  d_pipe     [0:Q_FRAC];
    reg [Q_FRAC:0] q_pipe     [0:Q_FRAC];
    reg            valid_pipe [0:Q_FRAC];

    // ---- stage 0: the ONE iteration where the dividend bit is 1 (bit
    //      Q_FRAC of 2^Q_FRAC) -- remainder starts at 0, so this reduces to
    //      "remainder = 1, then compare/subtract against d_in" -----------
    wire [R_W-1:0] rem0_shifted = {{(R_W-1){1'b0}}, 1'b1};
    wire           rem0_ge_d    = (rem0_shifted >= {1'b0, d_in});
    wire [R_W-1:0] rem0_next    = rem0_ge_d ? (rem0_shifted - {1'b0, d_in}) : rem0_shifted;

    always @(posedge clk) begin
        if (!aresetn) begin
            rem_pipe[0]   <= {R_W{1'b0}};
            d_pipe[0]     <= {D_W{1'b0}};
            q_pipe[0]     <= {(Q_FRAC+1){1'b0}};
            valid_pipe[0] <= 1'b0;
        end else begin
            rem_pipe[0]   <= rem0_next;
            d_pipe[0]     <= d_in;
            // bit Q_FRAC (the MSB of the Q_FRAC+1-wide quotient) is this
            // stage's result; every other bit is still 0 at this point.
            q_pipe[0]     <= {rem0_ge_d, {Q_FRAC{1'b0}}};
            valid_pipe[0] <= d_valid;
        end
    end

    // ---- stages 1..Q_FRAC: shift in a 0 dividend bit (every remaining
    //      iteration of 2^Q_FRAC's binary expansion is 0 below bit
    //      Q_FRAC), compare/subtract against the SAME d_pipe carried
    //      along, record the next quotient bit down -----------------------
    genvar g;
    generate
        for (g = 1; g <= Q_FRAC; g = g + 1) begin : STAGES
            wire [R_W-1:0] rem_shifted = {rem_pipe[g-1][R_W-2:0], 1'b0};
            wire           rem_ge_d    = (rem_shifted >= {1'b0, d_pipe[g-1]});
            wire [R_W-1:0] rem_next    = rem_ge_d ? (rem_shifted - {1'b0, d_pipe[g-1]})
                                                   : rem_shifted;
            // this stage's bit lands at position (Q_FRAC - g), zero-
            // extended to the full quotient width before OR-ing in so the
            // shift can't spill into a neighbouring bit.
            wire [Q_FRAC:0] bit_this_stage = ({{Q_FRAC{1'b0}}, rem_ge_d} << (Q_FRAC - g));

            always @(posedge clk) begin
                if (!aresetn) begin
                    rem_pipe[g]   <= {R_W{1'b0}};
                    d_pipe[g]     <= {D_W{1'b0}};
                    q_pipe[g]     <= {(Q_FRAC+1){1'b0}};
                    valid_pipe[g] <= 1'b0;
                end else begin
                    rem_pipe[g]   <= rem_next;
                    d_pipe[g]     <= d_pipe[g-1];
                    q_pipe[g]     <= q_pipe[g-1] | bit_this_stage;
                    valid_pipe[g] <= valid_pipe[g-1];
                end
            end
        end
    endgenerate

    assign q_out   = q_pipe[Q_FRAC];
    assign q_valid = valid_pipe[Q_FRAC];

endmodule
