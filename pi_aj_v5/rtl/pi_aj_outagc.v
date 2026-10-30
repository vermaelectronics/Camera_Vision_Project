// ============================================================================
// pi_aj_outagc.v  --  PI-AJ v5 output scaling for TX1 (DAC)
//
// After a deep null the array output is only thermal noise + GNSS, i.e.
// 50..70 dB below the jammer that set the ADC level. Re-transmitting it at
// that level would bury it in DAC quantisation, so the output is re-scaled
// in 6 dB steps (power-of-two shifts) to put its RMS at 2^target LSB of the
// 16-bit DAC word. One step at most every 2^14 samples (~0.9 ms @18 MS/s), so
// the GNSS receiver's own AGC sees only slow, small level changes.
// agc_en = 0 -> fixed shift man_shift (signed, + = gain).
// ============================================================================
`timescale 1ns / 1ps
module pi_aj_outagc #(
    parameter IW = 20,
    parameter PF = 10
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 agc_en,
    input  wire signed [4:0]    man_shift,
    input  wire [3:0]           target,       // log2 of output RMS (16-bit LSB)
    input  wire                 in_valid,
    input  wire signed [IW-1:0] in_re, in_im,
    output reg                  out_valid,
    output reg  signed [15:0]   out_i, out_q,
    output wire signed [4:0]    shift_o,
    output wire [31:0]          pow_o
);
    localparam PWD = 2*IW + PF + 1;

    // index of the highest set bit (0 when v == 0); log-depth search
    function [6:0] msb;
        input [PWD-1:0] v;
        reg [63:0] x;
        begin
            x = v;
            msb = 0;
            if (x[63:32] != 0) begin msb = msb + 32; x = x >> 32; end
            if (x[31:16] != 0) begin msb = msb + 16; x = x >> 16; end
            if (x[15:8]  != 0) begin msb = msb + 8;  x = x >> 8;  end
            if (x[7:4]   != 0) begin msb = msb + 4;  x = x >> 4;  end
            if (x[3:2]   != 0) begin msb = msb + 2;  x = x >> 2;  end
            if (x[1]     != 0) begin msb = msb + 1;               end
        end
    endfunction

    function signed [15:0] sat16;
        input signed [IW+16:0] v;
        if (v > 32767)       sat16 = 16'sh7FFF;
        else if (v < -32768) sat16 = -16'sh8000;
        else                 sat16 = v[15:0];
    endfunction

    reg signed [PWD:0]  po;
    reg [13:0]          tick;
    reg signed [4:0]    sh_agc;
    wire signed [4:0]   sh = agc_en ? sh_agc : man_shift;
    assign shift_o = sh;
    assign pow_o   = (po >>> PF) > 33'sh0FFFFFFFF ? 32'hFFFFFFFF : po[PF+31:PF];

    // power from the top 18 bits (one DSP48 per square), rescaled
    wire signed [IW-3:0] pr_re = in_re >>> 2;
    wire signed [IW-3:0] pr_im = in_im >>> 2;
    wire signed [2*IW:0] pinst = (pr_re * pr_re + pr_im * pr_im) <<< 4;
    reg  [6:0]  lp;                         // registered: log2 of the power
    wire signed [7:0] s_want = $signed({4'd0, target}) - $signed({2'b00, lp[6:1]});
    wire signed [7:0] s_cl   = (s_want > 15) ? 8'sd15 : (s_want < -8) ? -8'sd8 : s_want;

    wire [4:0] nsh = -sh;
    wire signed [IW+16:0] up_re = $signed(in_re) <<< sh;
    wire signed [IW+16:0] up_im = $signed(in_im) <<< sh;
    wire signed [IW+16:0] rn    = (sh < 0) ? ($signed(1) <<< (nsh - 1)) : 0;
    wire signed [IW+16:0] dn_re = ($signed(in_re) + rn) >>> nsh;
    wire signed [IW+16:0] dn_im = ($signed(in_im) + rn) >>> nsh;

    always @(posedge clk) begin
        if (rst) begin
            po <= 0; tick <= 0; sh_agc <= 0; lp <= 0;
            out_valid <= 0; out_i <= 0; out_q <= 0;
        end else begin
            out_valid <= in_valid;
            lp <= (msb(po) > PF) ? (msb(po) - PF) : 7'd0;
            if (in_valid) begin
                po   <= po + (((pinst <<< PF) - po) >>> 10);
                tick <= tick + 14'd1;
                if (tick == 0 && po != 0) begin
                    if (s_cl > sh_agc)      sh_agc <= sh_agc + 5'sd1;
                    else if (s_cl < sh_agc) sh_agc <= sh_agc - 5'sd1;
                end
                out_i <= sat16(sh >= 0 ? up_re : dn_re);
                out_q <= sat16(sh >= 0 ? up_im : dn_im);
            end
        end
    end
endmodule
