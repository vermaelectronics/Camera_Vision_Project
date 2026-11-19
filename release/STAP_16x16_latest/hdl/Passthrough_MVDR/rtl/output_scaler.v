// -----------------------------------------------------------------------------
// output_scaler.v  --  gain, round, shift and saturate the beamformer output
//                      v4: pipelined for l_clk = 250 MHz (4.0 ns), same arithmetic
//
//   out = clamp( round( in * G ) )  to OUT_W bits,   G = 2^(L/256) / 2^SHIFT
//   L   : gain in 1/256-octave steps (0.0235 dB), signed, L = 0 is unity
//   implemented as  out = (in * m + 2^(sh-1)) >>> sh,
//         m  = round(2^15 * 2^(frac(L)/256))   (32768 .. 65447, LUT)
//         sh = SHIFT + 15 - floor(L/256)
//
// v4 (timing): bit-identical output samples to v3, only the latency changes.
//   * product split into two 18x17 DSP products (registered in, M, P) and one
//     fabric add;
//   * rounding done after the shift:  (p + 2^(sh-1)) >> sh
//        = ((p >> (sh-1)) + 1) >> 1     (exact for floor shifts)
//     so the variable 53-bit rounding constant and add are gone;
//   * variable shift in two registered steps (by 4s, then 0..3);
//   * saturation: overflow flags registered, then the clamp (own stage);
//   * AOLC squares from a private copy of the output, fully pipelined DSP
//     (A, M, P registers; 4 clocks);
//   * AOLC loop: min-of-4 split into compare/select steps, log2 by a
//     normalising shift (8 or 1 bit per step, <= 12 steps of 2 clocks) instead
//     of a 45-bit priority encoder, L update split into add / candidates /
//     select steps.
//   The gain (m, shift) is registered three times before use; it changes only on a
//   register write or once per AOLC block, so this is invisible.
//
// MANUAL (agc_en = 0): L = gain_sh * 256 -> 2^gain_sh, 6.02 dB steps.
// AOLC (agc_en = 1): keeps the OUTPUT noise floor at `target_db` dB below the
//   power of a full-scale complex tone (default -18 dBFS). Block power over
//   2^BLK_LOG2 output samples, minimum of the last 4 blocks, log2 (Q4), dead
//   zone +-1/16 octave, L += (err*8) >>> agc_sh per block (at least +-1),
//   L limited to [-8, +15) octaves; entering AOLC starts from the manual L.
//   ENABLING AOLC RAISES THE TX1 DIGITAL LEVEL: set the TX attenuation first.
//
// Status: gain_l = L in use (Q8 octaves), out_lg = log2 of the measured output
//   power per sample (Q4), updated once per block.
//
// Latency: 9 clocks (v3: 5). sat: 1 if either I or Q was clamped.
// Requires SHIFT >= 1, IN_W > 17, BLK_LOG2 >= 2.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module output_scaler #(
    parameter IN_W     = 36,    // beamformer_2ch OUT_W
    parameter SHIFT    = 15,    // unity-gain shift
    parameter OUT_W    = 16,
    parameter LSB_PAD  = 0,
    parameter MAX_GAIN = 7,     // manual gain_sh range 0..MAX_GAIN
    parameter BLK_LOG2 = 12     // AOLC measurement block = 2^BLK_LOG2 samples
)(
    input  wire                             clk,
    input  wire                             rst,        // synchronous, active high

    input  wire                             in_valid,
    input  wire signed [IN_W-1:0]           in_re,
    input  wire signed [IN_W-1:0]           in_im,
    input  wire        [2:0]                gain_sh,    // manual: 2^gain_sh

    input  wire                             agc_en,     // 1 = automatic output level
    input  wire        [3:0]                agc_sh,     // loop speed (8: tau ~ 256 blocks)
    input  wire        [7:0]                target_db,  // output noise floor, dB below FS

    output reg                              out_valid,
    output reg  signed [OUT_W+LSB_PAD-1:0]  out_re,
    output reg  signed [OUT_W+LSB_PAD-1:0]  out_im,
    output reg                              sat,
    output reg  signed [15:0]               gain_l,     // L in use, Q8 octaves
    output reg  signed [15:0]               out_lg      // measured log2 power/sample, Q4
);

    localparam PW   = IN_W + 17;                         // in * m
    localparam QW   = PW - SHIFT;                        // after the fixed part of the shift
    localparam HW   = IN_W - 17;                         // upper operand part
    localparam signed [15:0] L_MIN = -16'sd2048;         // -8 octaves
    localparam signed [15:0] L_MAX =  16'sd3839;         // < +15 octaves
    localparam signed [OUT_W-1:0] MAXV = {1'b0, {(OUT_W-1){1'b1}}};
    localparam signed [OUT_W-1:0] MINV = {1'b1, {(OUT_W-1){1'b0}}};
    // log2 of the power of a full-scale complex tone, Q4 (2 * (2^(OUT_W-1)-1)^2)
    localparam integer FS_Q4 = 16 * (2 * (OUT_W - 1) + 1);

    function [15:0] mant;
        input [7:0] f;
        begin
            case (f)
            8'd0  : mant = 16'd32768;
            8'd1  : mant = 16'd32857;
            8'd2  : mant = 16'd32946;
            8'd3  : mant = 16'd33035;
            8'd4  : mant = 16'd33125;
            8'd5  : mant = 16'd33215;
            8'd6  : mant = 16'd33305;
            8'd7  : mant = 16'd33395;
            8'd8  : mant = 16'd33486;
            8'd9  : mant = 16'd33576;
            8'd10 : mant = 16'd33667;
            8'd11 : mant = 16'd33759;
            8'd12 : mant = 16'd33850;
            8'd13 : mant = 16'd33942;
            8'd14 : mant = 16'd34034;
            8'd15 : mant = 16'd34126;
            8'd16 : mant = 16'd34219;
            8'd17 : mant = 16'd34312;
            8'd18 : mant = 16'd34405;
            8'd19 : mant = 16'd34498;
            8'd20 : mant = 16'd34591;
            8'd21 : mant = 16'd34685;
            8'd22 : mant = 16'd34779;
            8'd23 : mant = 16'd34874;
            8'd24 : mant = 16'd34968;
            8'd25 : mant = 16'd35063;
            8'd26 : mant = 16'd35158;
            8'd27 : mant = 16'd35253;
            8'd28 : mant = 16'd35349;
            8'd29 : mant = 16'd35445;
            8'd30 : mant = 16'd35541;
            8'd31 : mant = 16'd35637;
            8'd32 : mant = 16'd35734;
            8'd33 : mant = 16'd35831;
            8'd34 : mant = 16'd35928;
            8'd35 : mant = 16'd36025;
            8'd36 : mant = 16'd36123;
            8'd37 : mant = 16'd36221;
            8'd38 : mant = 16'd36319;
            8'd39 : mant = 16'd36417;
            8'd40 : mant = 16'd36516;
            8'd41 : mant = 16'd36615;
            8'd42 : mant = 16'd36715;
            8'd43 : mant = 16'd36814;
            8'd44 : mant = 16'd36914;
            8'd45 : mant = 16'd37014;
            8'd46 : mant = 16'd37114;
            8'd47 : mant = 16'd37215;
            8'd48 : mant = 16'd37316;
            8'd49 : mant = 16'd37417;
            8'd50 : mant = 16'd37518;
            8'd51 : mant = 16'd37620;
            8'd52 : mant = 16'd37722;
            8'd53 : mant = 16'd37824;
            8'd54 : mant = 16'd37927;
            8'd55 : mant = 16'd38030;
            8'd56 : mant = 16'd38133;
            8'd57 : mant = 16'd38236;
            8'd58 : mant = 16'd38340;
            8'd59 : mant = 16'd38444;
            8'd60 : mant = 16'd38548;
            8'd61 : mant = 16'd38653;
            8'd62 : mant = 16'd38757;
            8'd63 : mant = 16'd38863;
            8'd64 : mant = 16'd38968;
            8'd65 : mant = 16'd39074;
            8'd66 : mant = 16'd39180;
            8'd67 : mant = 16'd39286;
            8'd68 : mant = 16'd39392;
            8'd69 : mant = 16'd39499;
            8'd70 : mant = 16'd39606;
            8'd71 : mant = 16'd39714;
            8'd72 : mant = 16'd39821;
            8'd73 : mant = 16'd39929;
            8'd74 : mant = 16'd40037;
            8'd75 : mant = 16'd40146;
            8'd76 : mant = 16'd40255;
            8'd77 : mant = 16'd40364;
            8'd78 : mant = 16'd40473;
            8'd79 : mant = 16'd40583;
            8'd80 : mant = 16'd40693;
            8'd81 : mant = 16'd40804;
            8'd82 : mant = 16'd40914;
            8'd83 : mant = 16'd41025;
            8'd84 : mant = 16'd41136;
            8'd85 : mant = 16'd41248;
            8'd86 : mant = 16'd41360;
            8'd87 : mant = 16'd41472;
            8'd88 : mant = 16'd41584;
            8'd89 : mant = 16'd41697;
            8'd90 : mant = 16'd41810;
            8'd91 : mant = 16'd41923;
            8'd92 : mant = 16'd42037;
            8'd93 : mant = 16'd42151;
            8'd94 : mant = 16'd42265;
            8'd95 : mant = 16'd42380;
            8'd96 : mant = 16'd42495;
            8'd97 : mant = 16'd42610;
            8'd98 : mant = 16'd42726;
            8'd99 : mant = 16'd42841;
            8'd100: mant = 16'd42958;
            8'd101: mant = 16'd43074;
            8'd102: mant = 16'd43191;
            8'd103: mant = 16'd43308;
            8'd104: mant = 16'd43425;
            8'd105: mant = 16'd43543;
            8'd106: mant = 16'd43661;
            8'd107: mant = 16'd43780;
            8'd108: mant = 16'd43898;
            8'd109: mant = 16'd44017;
            8'd110: mant = 16'd44137;
            8'd111: mant = 16'd44256;
            8'd112: mant = 16'd44376;
            8'd113: mant = 16'd44497;
            8'd114: mant = 16'd44617;
            8'd115: mant = 16'd44738;
            8'd116: mant = 16'd44859;
            8'd117: mant = 16'd44981;
            8'd118: mant = 16'd45103;
            8'd119: mant = 16'd45225;
            8'd120: mant = 16'd45348;
            8'd121: mant = 16'd45471;
            8'd122: mant = 16'd45594;
            8'd123: mant = 16'd45718;
            8'd124: mant = 16'd45842;
            8'd125: mant = 16'd45966;
            8'd126: mant = 16'd46091;
            8'd127: mant = 16'd46216;
            8'd128: mant = 16'd46341;
            8'd129: mant = 16'd46467;
            8'd130: mant = 16'd46593;
            8'd131: mant = 16'd46719;
            8'd132: mant = 16'd46846;
            8'd133: mant = 16'd46973;
            8'd134: mant = 16'd47100;
            8'd135: mant = 16'd47228;
            8'd136: mant = 16'd47356;
            8'd137: mant = 16'd47484;
            8'd138: mant = 16'd47613;
            8'd139: mant = 16'd47742;
            8'd140: mant = 16'd47871;
            8'd141: mant = 16'd48001;
            8'd142: mant = 16'd48131;
            8'd143: mant = 16'd48262;
            8'd144: mant = 16'd48393;
            8'd145: mant = 16'd48524;
            8'd146: mant = 16'd48655;
            8'd147: mant = 16'd48787;
            8'd148: mant = 16'd48920;
            8'd149: mant = 16'd49052;
            8'd150: mant = 16'd49185;
            8'd151: mant = 16'd49319;
            8'd152: mant = 16'd49452;
            8'd153: mant = 16'd49586;
            8'd154: mant = 16'd49721;
            8'd155: mant = 16'd49856;
            8'd156: mant = 16'd49991;
            8'd157: mant = 16'd50126;
            8'd158: mant = 16'd50262;
            8'd159: mant = 16'd50399;
            8'd160: mant = 16'd50535;
            8'd161: mant = 16'd50672;
            8'd162: mant = 16'd50810;
            8'd163: mant = 16'd50947;
            8'd164: mant = 16'd51085;
            8'd165: mant = 16'd51224;
            8'd166: mant = 16'd51363;
            8'd167: mant = 16'd51502;
            8'd168: mant = 16'd51642;
            8'd169: mant = 16'd51782;
            8'd170: mant = 16'd51922;
            8'd171: mant = 16'd52063;
            8'd172: mant = 16'd52204;
            8'd173: mant = 16'd52346;
            8'd174: mant = 16'd52488;
            8'd175: mant = 16'd52630;
            8'd176: mant = 16'd52773;
            8'd177: mant = 16'd52916;
            8'd178: mant = 16'd53059;
            8'd179: mant = 16'd53203;
            8'd180: mant = 16'd53347;
            8'd181: mant = 16'd53492;
            8'd182: mant = 16'd53637;
            8'd183: mant = 16'd53782;
            8'd184: mant = 16'd53928;
            8'd185: mant = 16'd54074;
            8'd186: mant = 16'd54221;
            8'd187: mant = 16'd54368;
            8'd188: mant = 16'd54515;
            8'd189: mant = 16'd54663;
            8'd190: mant = 16'd54811;
            8'd191: mant = 16'd54960;
            8'd192: mant = 16'd55109;
            8'd193: mant = 16'd55258;
            8'd194: mant = 16'd55408;
            8'd195: mant = 16'd55558;
            8'd196: mant = 16'd55709;
            8'd197: mant = 16'd55860;
            8'd198: mant = 16'd56012;
            8'd199: mant = 16'd56163;
            8'd200: mant = 16'd56316;
            8'd201: mant = 16'd56468;
            8'd202: mant = 16'd56622;
            8'd203: mant = 16'd56775;
            8'd204: mant = 16'd56929;
            8'd205: mant = 16'd57083;
            8'd206: mant = 16'd57238;
            8'd207: mant = 16'd57393;
            8'd208: mant = 16'd57549;
            8'd209: mant = 16'd57705;
            8'd210: mant = 16'd57861;
            8'd211: mant = 16'd58018;
            8'd212: mant = 16'd58176;
            8'd213: mant = 16'd58333;
            8'd214: mant = 16'd58491;
            8'd215: mant = 16'd58650;
            8'd216: mant = 16'd58809;
            8'd217: mant = 16'd58968;
            8'd218: mant = 16'd59128;
            8'd219: mant = 16'd59289;
            8'd220: mant = 16'd59449;
            8'd221: mant = 16'd59611;
            8'd222: mant = 16'd59772;
            8'd223: mant = 16'd59934;
            8'd224: mant = 16'd60097;
            8'd225: mant = 16'd60260;
            8'd226: mant = 16'd60423;
            8'd227: mant = 16'd60587;
            8'd228: mant = 16'd60751;
            8'd229: mant = 16'd60916;
            8'd230: mant = 16'd61081;
            8'd231: mant = 16'd61247;
            8'd232: mant = 16'd61413;
            8'd233: mant = 16'd61579;
            8'd234: mant = 16'd61746;
            8'd235: mant = 16'd61914;
            8'd236: mant = 16'd62081;
            8'd237: mant = 16'd62250;
            8'd238: mant = 16'd62419;
            8'd239: mant = 16'd62588;
            8'd240: mant = 16'd62757;
            8'd241: mant = 16'd62928;
            8'd242: mant = 16'd63098;
            8'd243: mant = 16'd63269;
            8'd244: mant = 16'd63441;
            8'd245: mant = 16'd63613;
            8'd246: mant = 16'd63785;
            8'd247: mant = 16'd63958;
            8'd248: mant = 16'd64132;
            8'd249: mant = 16'd64306;
            8'd250: mant = 16'd64480;
            8'd251: mant = 16'd64655;
            8'd252: mant = 16'd64830;
            8'd253: mant = 16'd65006;
            8'd254: mant = 16'd65182;
            8'd255: mant = 16'd65359;
            default: mant = 16'd32768;
            endcase
        end
    endfunction

    // ---------------- gain path (registered three times) ----------------
    reg  signed [15:0] L = 16'sd0;
    wire [2:0]  g    = (gain_sh > MAX_GAIN) ? MAX_GAIN[2:0] : gain_sh;
    wire signed [15:0] L_man = $signed({8'd0, 5'd0, g}) <<< 8;
    reg  signed [15:0] Lu = 16'sd0;                      // L in use
    (* shreg_extract = "no" *) reg signed [16:0] m_r = 17'sd32768, m_g = 17'sd32768;
    (* shreg_extract = "no" *) reg        [4:0]  s_r = 5'd14,     s_g = 5'd14;                      // sh - 1 - SHIFT = 14 - floor(L/256), 0..22
    wire signed [7:0]  e_u = Lu[15:8];                 // floor(L / 256)
    always @(posedge clk) begin
        Lu  <= agc_en ? L : L_man;
        m_r <= {1'b0, mant(Lu[7:0])};                    // ROM, then one more register
        s_r <= 5'd14 - e_u[4:0];
        m_g <= m_r;
        s_g <= s_r;
    end

    // ---------------- S1: operands ----------------
    reg                   v1;
    reg signed [IN_W-1:0] a_re, a_im;
    (* shreg_extract = "no" *) reg signed [16:0] m1;
    (* shreg_extract = "no" *) reg        [4:0]  s1;
    always @(posedge clk) begin
        if (rst) v1 <= 1'b0; else v1 <= in_valid;
        a_re <= in_re;  a_im <= in_im;
        m1   <= m_g;    s1   <= s_g;
    end

    // ---------------- S2, S3: two DSP products per part (M, P registers) ----------------
    wire signed [17:0]    al_re = {1'b0, a_re[16:0]}, al_im = {1'b0, a_im[16:0]};
    wire signed [HW-1:0]  ah_re = a_re[IN_W-1:17],    ah_im = a_im[IN_W-1:17];
    reg                   v2, v3;
    reg signed [34:0]     pl_re2, pl_im2, pl_re3, pl_im3;
    reg signed [HW+16:0]  ph_re2, ph_im2, ph_re3, ph_im3;
    (* shreg_extract = "no" *) reg [4:0] s2, s3;
    always @(posedge clk) begin
        if (rst) begin v2 <= 1'b0; v3 <= 1'b0; end else begin v2 <= v1; v3 <= v2; end
        pl_re2 <= al_re * m1;   pl_im2 <= al_im * m1;
        ph_re2 <= ah_re * m1;   ph_im2 <= ah_im * m1;
        pl_re3 <= pl_re2;       pl_im3 <= pl_im2;
        ph_re3 <= ph_re2;       ph_im3 <= ph_im2;
        s2 <= s1;  s3 <= s2;
    end

    // ---------------- S4: recombine ----------------
    reg                   v4;
    reg signed [PW-1:0]   p_re, p_im;
    (* shreg_extract = "no" *) reg [4:0] s4;
    always @(posedge clk) begin
        if (rst) v4 <= 1'b0; else v4 <= v3;
        p_re <= ($signed({ph_re3, 17'd0})) + $signed({{(PW-35){1'b0}}, pl_re3});
        p_im <= ($signed({ph_im3, 17'd0})) + $signed({{(PW-35){1'b0}}, pl_im3});
        s4 <= s3;
    end

    // ---------------- S5, S6: shift right by SHIFT + s (s = sh - 1 - SHIFT) ----------------
    //   by 4 * s[4:2] (6-way mux), then by s[1:0] (4-way mux)
    wire signed [QW-1:0] p0_re = p_re >>> SHIFT;
    wire signed [QW-1:0] p0_im = p_im >>> SHIFT;
    reg                  v5, v6;
    reg signed [QW-1:0]  q1_re, q1_im, q2_re, q2_im;
    (* shreg_extract = "no" *) reg [1:0] s5;
    always @(posedge clk) begin
        if (rst) begin v5 <= 1'b0; v6 <= 1'b0; end else begin v5 <= v4; v6 <= v5; end
        q1_re <= p0_re >>> {s4[4:2], 2'b00};
        q1_im <= p0_im >>> {s4[4:2], 2'b00};
        s5    <= s4[1:0];
        q2_re <= q1_re >>> s5;
        q2_im <= q1_im >>> s5;
    end

    // ---------------- S7: round ----------------
    reg                  v7;
    reg signed [QW-1:0]  t_re, t_im;
    wire signed [QW:0]   u_re = {q2_re[QW-1], q2_re} + 1'b1;
    wire signed [QW:0]   u_im = {q2_im[QW-1], q2_im} + 1'b1;
    always @(posedge clk) begin
        if (rst) v7 <= 1'b0; else v7 <= v6;
        t_re <= u_re[QW:1];
        t_im <= u_im[QW:1];
    end

    // ---------------- S8: overflow flags ----------------
    wire [QW-OUT_W:0] tu_re = t_re[QW-1:OUT_W-1];
    wire [QW-OUT_W:0] tu_im = t_im[QW-1:OUT_W-1];
    reg                     v8, hi_re, lo_re, hi_im, lo_im;
    reg  signed [OUT_W-1:0] t8_re, t8_im;
    always @(posedge clk) begin
        if (rst) v8 <= 1'b0; else v8 <= v7;
        hi_re <= ~t_re[QW-1] & (|tu_re);  lo_re <= t_re[QW-1] & ~(&tu_re);
        hi_im <= ~t_im[QW-1] & (|tu_im);  lo_im <= t_im[QW-1] & ~(&tu_im);
        t8_re <= t_re[OUT_W-1:0];          t8_im <= t_im[OUT_W-1:0];
    end

    // ---------------- S9: saturate, pad, output ----------------
    wire signed [OUT_W-1:0] c_re = hi_re ? MAXV : (lo_re ? MINV : t8_re);
    wire signed [OUT_W-1:0] c_im = hi_im ? MAXV : (lo_im ? MINV : t8_im);
    reg  signed [OUT_W-1:0] o_re, o_im;                  // unpadded copy for the AOLC
    always @(posedge clk) begin
        if (rst) out_valid <= 1'b0; else out_valid <= v8;
        out_re <= c_re <<< LSB_PAD;
        out_im <= c_im <<< LSB_PAD;
        o_re   <= c_re;  o_im <= c_im;
        sat    <= hi_re | lo_re | hi_im | lo_im;
    end

    // =====================================================================
    //  AOLC: block power measurement and gain loop
    // =====================================================================
    localparam AW = 2*OUT_W + BLK_LOG2 + 1;              // block power width
    // squarers: own input register (DSP AREG), multiply register (MREG), output
    // register (PREG). The input copy must NOT be merged into out_re/out_im
    // (Vivado did that in v1.12-a: the DSP then had no A/M registers and a
    // 3.1 ns A->P setup), hence the keep attributes.
    (* keep = "true" *) reg signed [OUT_W-1:0] sa_re, sa_im;
    reg  [2*OUT_W-1:0] sqm_a, sqm_b;
    reg  [2*OUT_W-1:0] sqa, sqb;
    reg                sv1, sv2;
    reg  [2*OUT_W:0]   sq;
    reg                sqa_v, sq_v;
    reg  [BLK_LOG2-1:0] bcnt;
    reg                bl;                               // bcnt == all ones
    reg  [AW-1:0]      acc, pb0, pb1, pb2, pb3;
    reg  [2:0]         nblk;                             // blocks seen (saturates at 4)
    reg  [12:1]        ps;                               // control sequence phase, one-hot
    reg                bend;                             // next sq_v ends a block
    reg                bcl;                              // bcnt == BC_LAST1 (kept with bcnt)
    reg                ez0, ezm;                         // err[15:1] == 0, err == -1
    reg                c01, c23, c0;
    reg  [AW-1:0]      pm01, pm23, nrm;
    reg  [7:0]         blen;
    reg  [3:0]         frac4;
    reg  signed [15:0] lg, err;
    reg  signed [23:0] dl, ls;
    reg                upd;
    reg                nst, fd, f8;                      // normalise: flags / act
    reg                a8, a1;                           // normalise: shift by 8 / by 1 next clock
    reg                bz, bg8;                          // blen == 0, blen > 8 (kept with blen)
    reg                idle;                             // ps == 0
    reg                dlz, epos, gtmax, ltmin;
    reg  signed [15:0] lp1, lm1;
    reg                agc_q;
    reg  [15:0]        tm, tma, tmb;
    reg  signed [15:0] tq;                               // target, Q4 log2 power / sample
    wire [AW-1:0]      acc_sum = acc + {{(AW-2*OUT_W-1){1'b0}}, sq};
    localparam [BLK_LOG2-1:0] BC_LAST1 = {{(BLK_LOG2-1){1'b1}}, 1'b0};
    localparam [BLK_LOG2-1:0] BC_LAST2 = BC_LAST1 - 1'b1;

    // target in Q4 log2 units: FS - t * 16 / 3.0103  (16/3.0103 ~ 85/16)
    always @(posedge clk) begin
        // 85 t + 8 = (64 t + 16 t) + (4 t + t + 8): shift-adds over two clocks
        tma  <= {2'd0, target_db, 6'd0} + {4'd0, target_db, 4'd0};
        tmb  <= {6'd0, target_db, 2'd0} + {8'd0, target_db} + 16'd8;
        tm   <= tma + tmb;
        tq   <= FS_Q4 - $signed({4'd0, tm[15:4]});
        sa_re <= o_re;           sa_im <= o_im;
        sqm_a <= sa_re * sa_re;  sqm_b <= sa_im * sa_im;
        sqa   <= sqm_a;          sqb   <= sqm_b;
        sq   <= {1'b0, sqa} + {1'b0, sqb};
    end

    // ---- control state (reset) ----
    //   bend = sq_v & bl one clock ahead (sq_v is sqa_v delayed; bl changes
    //   only on sq_v), so the block-end actions start from one flop.
    wire nact = ps[5] & nst & fd;                        // normalisation finished
    wire held = ps[5] & ~(nst & fd);                     // phase 5 not leaving
    always @(posedge clk) begin
        if (rst) begin
            sv1 <= 1'b0;  sv2 <= 1'b0;  sqa_v <= 1'b0;  sq_v <= 1'b0;  bend <= 1'b0;
            L <= 16'sd0;  agc_q <= 1'b0;
            bcnt <= 0;  bl <= 1'b0;  bcl <= (BC_LAST1 == 0);  nblk <= 3'd0;  ps <= 12'd0;  idle <= 1'b1;
            gain_l <= 16'sd0;  out_lg <= -16'sd32768;
        end else begin
            sv1   <= out_valid;
            sv2   <= sv1;
            sqa_v <= sv2;
            sq_v  <= sqa_v;
            bend  <= sqa_v & (sq_v ? bcl : bl);
            agc_q <= agc_en;
            if (agc_en && !agc_q) L <= L_man;                // enter AOLC from the manual gain
            gain_l <= Lu;

            if (sq_v) begin
                bcnt <= bcnt + 1'b1;
                bl   <= bcl;
                bcl  <= (bcnt == BC_LAST2);              // bcnt + 1 == BC_LAST1
            end
            if (bend && nblk != 3'd4) nblk <= nblk + 3'd1;

            // phase sequence (a block end restarts it from idle or from a
            // waiting phase 5, as the binary v3/v4 sequencer did)
            ps[1]  <= bend & (idle | held);
            idle   <= ~((bend & (idle | held)) | (|ps[11:1]));
            ps[2]  <= ps[1];   ps[3] <= ps[2];   ps[4] <= ps[3];
            ps[5]  <= ps[4] | (held & ~bend);
            ps[6]  <= nact;
            ps[7]  <= ps[6];   ps[8] <= ps[7];   ps[9] <= ps[8];
            ps[10] <= ps[9];   ps[11] <= ps[10]; ps[12] <= ps[11];

            if (ps[8] && !bz) out_lg <= lg;                  // bz: measured power was 0
            if (ps[12] && upd && agc_en && agc_q)
                L <= dlz   ? (epos ? lp1 : lm1) :
                     gtmax ? L_MAX :
                     ltmin ? L_MIN : ls[15:0];
        end
    end

    // ---- block power: acc cleared through its R pin, pb shift on the block end ----
    always @(posedge clk) begin
        if (rst || bend)  acc <= 0;
        else if (sq_v)    acc <= acc_sum;
        if (rst) begin
            pb0 <= 0; pb1 <= 0; pb2 <= 0; pb3 <= 0;
        end else if (bend) begin
            pb3 <= pb2;  pb2 <= pb1;  pb1 <= pb0;  pb0 <= acc_sum;
        end
    end

    // ---- sequencer datapath (no reset; steered by the one-hot phase) ----
    //   normalise: on the even clock of phase 5 the flags and the shift
    //   decision (a8 / a1) are registered; on the odd clock the shift happens.
    wire fd_n = nrm[AW-1] | bz;
    wire f8_n = (nrm[AW-1 -: 8] == 8'd0) & bg8;
    always @(posedge clk) begin
        a8 <= ps[5] & ~nst & ~fd_n &  f8_n;
        a1 <= ps[5] & ~nst & ~fd_n & ~f8_n;
    end
    always @(posedge clk) begin
        if (ps[1]) begin c01 <= (pb0 < pb1);  c23 <= (pb2 < pb3); end
        if (ps[2]) begin pm01 <= c01 ? pb0 : pb1;  pm23 <= c23 ? pb2 : pb3; end
        if (ps[3]) c0 <= (pm01 < pm23);
        // normalise: blen = bit length of pmin (even clocks: flags, odd: act)
        if (ps[4]) begin nrm <= c0 ? pm01 : pm23;  blen <= AW[7:0];  nst <= 1'b0;
                         bz <= (AW == 0);  bg8 <= (AW > 8); end
        if (ps[5]) begin
            nst <= ~nst;
            if (!nst) begin fd <= fd_n;  f8 <= f8_n; end
        end
        if (a8) begin nrm <= nrm << 8;  blen <= blen - 8'd8;  bz <= (blen == 8'd8);  bg8 <= (blen > 8'd16); end
        if (a1) begin nrm <= nrm << 1;  blen <= blen - 8'd1;  bz <= (blen == 8'd1);  bg8 <= (blen > 8'd9);  end
        if (ps[6]) frac4 <= nrm[AW-2 -: 4];              // 4 bits below the leading one
        if (ps[7]) lg  <= $signed({8'd0, blen - 8'd1}) * 16'sd16 + $signed({12'd0, frac4})
                          - 16'sd16 * BLK_LOG2;          // log2(power per sample), Q4
        if (ps[8]) err <= tq - lg;
        if (ps[9]) begin
            dl  <= ($signed({{8{err[15]}}, err}) * 24'sd8) >>> agc_sh;
            ez0 <= (err[15:1] == 15'd0);                 // err in {0, 1}
            ezm <= (err == 16'hFFFF);                    // err == -1
        end
        if (ps[10]) begin
            ls  <= $signed({{8{L[15]}}, L}) + dl;
            upd <= ~(ez0 | ezm) & agc_en & agc_q & (nblk == 3'd4) & ~bz;   // |err| > 1, ...
        end
        if (ps[11]) begin                                // candidates
            dlz   <= (dl == 24'sd0);
            epos  <= (err > 16'sd0);
            lp1   <= (L < L_MAX) ? L + 16'sd1 : L;
            lm1   <= (L > L_MIN) ? L - 16'sd1 : L;
            gtmax <= (ls > $signed({{8{L_MAX[15]}}, L_MAX}));
            ltmin <= (ls < $signed({{8{L_MIN[15]}}, L_MIN}));
        end
    end

endmodule
