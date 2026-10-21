//-----------------------------------------------------------------------------
// stap_int2fp : signed integer -> IEEE-754 single, scaled by 2^EXP_OFS
//   y = float(x) * 2^EXP_OFS     (1 cycle registered, round to nearest even)
//   Used by the covariance engine to apply the 1/L averaging for free.
//-----------------------------------------------------------------------------
module stap_int2fp #(
    parameter integer W       = 48,     // input width (>= 26)
    parameter integer EXP_OFS = 0       // power-of-two scale (may be negative)
) (
    input  wire         clk,
    input  wire [W-1:0] x,
    output reg  [31:0]  y
);

    localparam integer LZW = $clog2(W + 1);

    function [LZW-1:0] lzc;
        input [W-1:0] v;
        integer k;
        reg found;
        begin
            lzc   = W;
            found = 1'b0;
            for (k = W - 1; k >= 0; k = k - 1) begin
                if (!found && v[k]) begin
                    lzc   = W - 1 - k;
                    found = 1'b1;
                end
            end
        end
    endfunction

    wire          sgn  = x[W-1];
    wire [W-1:0]  mag  = sgn ? (~x + 1'b1) : x;    // |x| (most negative ok)
    wire [LZW-1:0] lz  = lzc(mag);
    wire [W-1:0]  nrm  = mag << lz;                // leading one at bit W-1

    wire [22:0]   frac = nrm[W-2 -: 23];
    wire          g    = nrm[W-25];
    wire          s    = |(nrm & ({W{1'b1}} >> 25));
    wire          rup  = g & (s | frac[0]);
    wire [23:0]   fr   = {1'b0, frac} + {23'd0, rup};

    reg  signed [12:0] e;
    always @(*) begin
        e = (W - 1) - lz + EXP_OFS + 127 + fr[23];
    end

    always @(posedge clk) begin
        if (mag == {W{1'b0}} || e <= 0)
            y <= 32'd0;
        else if (e >= 255)
            y <= {sgn, 8'hFF, 23'd0};
        else
            y <= {sgn, e[7:0], fr[22:0]};
    end

endmodule
