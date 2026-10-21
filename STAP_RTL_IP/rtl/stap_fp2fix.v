//-----------------------------------------------------------------------------
// stap_fp2fix : IEEE-754 single -> signed fixed point (1 cycle registered)
//   y = sat( round( f * 2^(k + FRAC) ) )   on W bits
//   k is a run-time block-exponent shift (signed), FRAC the fixed fraction
//   bits. Rounding is half away from zero, the result saturates.
//-----------------------------------------------------------------------------
module stap_fp2fix #(
    parameter integer W    = 18,
    parameter integer FRAC = 16
) (
    input  wire              clk,
    input  wire [31:0]       f,
    input  wire signed [9:0] k,
    output reg  [W-1:0]      y
);

    localparam [63:0] MAXMAG = (64'd1 << (W - 1)) - 64'd1;

    wire [23:0]        m  = {1'b1, f[22:0]};
    wire signed [11:0] s  = $signed({4'd0, f[30:23]}) - 12'sd150
                          + {{2{k[9]}}, k} + FRAC;

    reg  [63:0] mag;
    reg  [6:0]  sh;
    always @(*) begin
        sh = 7'd0;
        if (f[30:23] == 8'd0) begin
            mag = 64'd0;
        end else if (s >= 0) begin
            if (s >= (W - 24))
                mag = MAXMAG;
            else
                mag = {40'd0, m} << s;
        end else begin
            sh = -s;
            if (sh > 7'd25)
                mag = 64'd0;
            else
                mag = ({40'd0, m} + (64'd1 << (sh - 7'd1))) >> sh;
        end
        if (mag > MAXMAG)
            mag = MAXMAG;
    end

    always @(posedge clk)
        y <= f[31] ? (~mag[W-1:0] + 1'b1) : mag[W-1:0];

endmodule
