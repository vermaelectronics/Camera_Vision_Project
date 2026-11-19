// -----------------------------------------------------------------------------
// real_divider.v  --  bit-serial restoring divider (small, fixed latency)
//
//     quo = round( num / den )
//
//   num : signed  NUM_W bits
//   den : UNSIGNED DEN_W bits   (a real divisor: the MVDR normaliser aH.R^-1.a
//                                is a positive real, so no signed divisor needed)
//   quo : signed  NUM_W bits
//
//   ROUND = 1 : round to nearest, ties away from zero
//   ROUND = 0 : truncate toward zero
//   den == 0  : div0 = 1 and quo = 0 (caller should keep its previous result)
//
// Handshake (one operation at a time):
//   - pulse `start` for one clock while `busy` = 0; num/den are sampled on that
//     clock edge.
//   - `busy` is 1 until the result is ready.
//   - `done` is a 1-clock pulse; quo / div0 are valid on that clock and stay
//     stable until the next start.
//
// Latency: NUM_W + 2 clocks from the edge that samples `start` to the clock on
//          which `done` is high.  Cost: one DEN_W-bit subtractor, a NUM_W-bit
//          shift register, no multipliers / DSPs.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module real_divider #(
    parameter NUM_W = 64,
    parameter DEN_W = 32,
    parameter ROUND = 1
)(
    input  wire                      clk,
    input  wire                      rst,       // synchronous, active high

    input  wire                      start,
    input  wire signed [NUM_W-1:0]   num,
    input  wire        [DEN_W-1:0]   den,

    output reg                       busy,
    output reg                       done,
    output reg  signed [NUM_W-1:0]   quo,
    output reg                       div0
);

    localparam [1:0] S_IDLE = 2'd0,
                     S_RUN  = 2'd1,
                     S_FIN  = 2'd2;

    reg [1:0]          st;
    reg [15:0]         cnt;
    reg                neg;
    reg                dz;
    reg [DEN_W-1:0]    den_r;
    reg [DEN_W-1:0]    rem;
    reg [NUM_W-1:0]    qreg;      // holds remaining dividend bits, fills with quotient bits

    // one restoring-division step
    wire [DEN_W:0] shifted = {rem, qreg[NUM_W-1]};
    wire           ge      = (shifted >= {1'b0, den_r});
    wire [DEN_W:0] diff    = shifted - {1'b0, den_r};

    // final rounding / sign (used in S_FIN)
    wire [DEN_W:0]   rem2   = {rem, 1'b0};
    wire             rnd_up = (ROUND != 0) && (rem2 >= {1'b0, den_r});
    wire [NUM_W-1:0] qmag   = qreg + rnd_up;
    wire [NUM_W-1:0] qsgn   = neg ? (~qmag + 1'b1) : qmag;

    always @(posedge clk) begin
        if (rst) begin
            st   <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            div0 <= 1'b0;
            quo  <= {NUM_W{1'b0}};
        end else begin
            done <= 1'b0;
            case (st)
                S_IDLE: begin
                    if (start) begin
                        neg   <= num[NUM_W-1];
                        qreg  <= num[NUM_W-1] ? (~num + 1'b1) : num;   // |num|
                        den_r <= den;
                        dz    <= (den == {DEN_W{1'b0}});
                        rem   <= {DEN_W{1'b0}};
                        cnt   <= NUM_W - 1;
                        busy  <= 1'b1;
                        st    <= S_RUN;
                    end
                end

                S_RUN: begin
                    rem  <= ge ? diff[DEN_W-1:0] : shifted[DEN_W-1:0];
                    qreg <= {qreg[NUM_W-2:0], ge};
                    if (cnt == 16'd0) st <= S_FIN;
                    else              cnt <= cnt - 16'd1;
                end

                S_FIN: begin
                    quo  <= dz ? {NUM_W{1'b0}} : qsgn;
                    div0 <= dz;
                    done <= 1'b1;
                    busy <= 1'b0;
                    st   <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
