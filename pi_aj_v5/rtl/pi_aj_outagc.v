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
//
// Timing (rev 3): the output is shift -> register -> saturate -> register
// (2 clocks); the power estimate is square (registered DSP48) -> difference
// -> accumulate, one carry chain per clock. Needs >= 3 clocks per sample.
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
    output reg  [31:0]          pow_o
);
    localparam PWD = 2*IW + PF + 1;
    localparam XW  = IW + 17;

    // leading one in two clocked steps (same result as a priority encoder,
    // 0 when v == 0):  therm(): per 8-bit group, "non-zero" + index of its
    // top 1;  th_pos(): top non-zero group, then that group's index.
    // Both are about 2 LUT levels; pure AND/OR logic.
    function [2:0] top8;                     // index of the top 1 of 8 bits
        input [7:0] b;
        reg h2, h1;
        begin
            h2 = |b[7:4];
            h1 = h2 ? |b[7:6] : |b[3:2];
            top8 = {h2, h1, h2 ? (h1 ? b[7] : b[5]) : (h1 ? b[3] : b[1])};
        end
    endfunction
    function [31:0] therm;                   // {idx7..idx0, nz7..nz0}
        input [PWD-1:0] v;
        reg [63:0] x;
        integer j;
        begin
            x = v;
            for (j = 0; j < 8; j = j + 1) begin
                therm[j]           = |x[8*j +: 8];
                therm[8+3*j +: 3]  = top8(x[8*j +: 8]);
            end
        end
    endfunction
    function [6:0] th_pos;
        input [31:0] t;
        reg [2:0] q;
        begin
            q = top8(t[7:0]);
            th_pos = {q, t[8+3*q +: 3]};
        end
    endfunction

    // saturation by checking that the dropped bits all equal the sign
    function signed [15:0] sat16;
        input signed [XW-1:0] v;
        if ((v[XW-1:15] == 0) || (&v[XW-1:15])) sat16 = v[15:0];
        else if (v[XW-1])                       sat16 = 16'sh8000;
        else                                    sat16 = 16'sh7FFF;
    endfunction

    reg signed [PWD:0]  po, dpo;
    reg [13:0]          tick;
    reg signed [4:0]    sh_agc;
    reg signed [4:0]    sh;                 // registered: agc_en ? sh_agc : man_shift
    reg        [4:0]    nsh;                // registered -sh
    reg  [31:0]      po_th;
    assign shift_o = sh;

    // power from the top 18 bits (one DSP48 per square, registered operands)
    reg  signed [IW-3:0] pr_re, pr_im;
    reg  signed [2*IW:0] pinst;
    reg                  pv0, pv1, pv2;
    reg  [6:0]  lp_m, lp;                   // log2 of the power
    wire signed [7:0] s_want = $signed({4'd0, target}) - $signed({2'b00, lp[6:1]});
    wire signed [7:0] s_cl   = (s_want > 15) ? 8'sd15 : (s_want < -8) ? -8'sd8 : s_want;

    reg  signed [XW-1:0] rn;                // registered rounding constant
    wire signed [XW-1:0] up_re = $signed(in_re) <<< sh;
    wire signed [XW-1:0] up_im = $signed(in_im) <<< sh;
    wire signed [XW-1:0] dn_re = ($signed(in_re) + rn) >>> nsh;
    wire signed [XW-1:0] dn_im = ($signed(in_im) + rn) >>> nsh;
    reg  signed [XW-1:0] sc_re, sc_im;
    reg                  scv;

    always @(posedge clk) begin
        if (rst) begin
            po <= 0; dpo <= 0; tick <= 0; sh_agc <= 0; sh <= 0; nsh <= 0; rn <= 0; po_th <= 0; lp_m <= 0; lp <= 0;
            pv0 <= 0; pv1 <= 0; pv2 <= 0; scv <= 0; pow_o <= 0;
            out_valid <= 0; out_i <= 0; out_q <= 0;
        end else begin
            // quasi-static: the shift settles 2 clocks after a change
            sh   <= agc_en ? sh_agc : man_shift;
            nsh  <= -sh;
            rn   <= (sh < 0) ? ($signed(1) <<< (nsh - 1)) : 0;
            po_th <= therm(po[PWD-1:0]);
            lp_m <= th_pos(po_th);
            lp   <= (lp_m > PF) ? (lp_m - PF) : 7'd0;
            pow_o <= (|po[PWD:PF+32]) ? 32'hFFFFFFFF : po[PF+31:PF];
            // power: square, difference, accumulate
            pv0 <= in_valid;
            if (in_valid) begin
                pr_re <= in_re >>> 2;
                pr_im <= in_im >>> 2;
            end
            pv1 <= pv0;
            if (pv0)
                pinst <= (pr_re * pr_re + pr_im * pr_im) <<< 4;
            if (pv1)
                dpo <= ($signed(pinst) <<< PF) - po;
            pv2 <= pv1;
            if (pv2)
                po <= po + (dpo >>> 10);
            if (in_valid) begin
                tick <= tick + 14'd1;
                if (tick == 0 && po != 0) begin
                    if (s_cl > sh_agc)      sh_agc <= sh_agc + 5'sd1;
                    else if (s_cl < sh_agc) sh_agc <= sh_agc - 5'sd1;
                end
            end
            // output: shift, then saturate
            scv <= in_valid;
            if (in_valid) begin
                sc_re <= (sh >= 0) ? up_re : dn_re;
                sc_im <= (sh >= 0) ? up_im : dn_im;
            end
            out_valid <= scv;
            if (scv) begin
                out_i <= sat16(sc_re);
                out_q <= sat16(sc_im);
            end
        end
    end
endmodule
