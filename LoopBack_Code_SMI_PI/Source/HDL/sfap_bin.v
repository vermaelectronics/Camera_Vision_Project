// ============================================================================
//  sfap_bin.v  --  per-bin space-frequency adaptive power inversion
//
//  For every FFT bin k of every frame, with X1 = antenna 1 and X2 = antenna 2:
//      R22[k] <- R22[k] + (|X2|^2    - R22[k]) >> LAM        (recursive average over frames)
//      R12[k] <- R12[k] + (X1 X2^*   - R12[k]) >> LAM
//      c[k]    = R12[k] / (R22[k] + LOAD)
//      Y[k]    = X1[k] - c[k] X2[k]
//  i.e. the minimum-output-power solution with antenna 1 fixed at weight 1
//  (power inversion), solved in closed form for each frequency independently.
//  The current frame is part of the average, so the weights are ready for the
//  frame they are applied to: nothing waits for the ARM.
//
//  One bin per clock, latency 32. Every stage is ONE operator (an add, a barrel
//  shift, a compare or a multiply with its M and P registers), because the
//  4.7.0 build failed 245.76 MHz timing by packing several operators into one
//  step. Operands of every multiplier are registered, and every product has two
//  register stages behind it, so the DSP48 can use AREG/BREG/MREG/PREG.
//
//  Division without a divider: normalise D = R22 + LOAD to m * 2^E with
//  m in [1,2), look up 1/m (512 entries, Q1.17) and refine with one
//  Newton-Raphson step (about 17 bits).  c is Q3.20.
// ============================================================================
`timescale 1ns/1ps

module sfap_dly #(parameter W = 8, parameter N = 4) (
    input  wire         clk,
    input  wire [W-1:0] d,
    output wire [W-1:0] q
);
    reg [W-1:0] s [0:N-1];
    integer i;
    always @(posedge clk) begin
        s[0] <= d;
        for (i = 1; i < N; i = i + 1) s[i] <= s[i-1];
    end
    assign q = s[N-1];
endmodule

module sfap_bin #(
    parameter WD = 25,
    parameter RW = 52,
    parameter LN = 10               // log2 of the transform size
) (
    input  wire                 clk,
    input  wire                 v0,
    input  wire [LN-1:0]        t0,          // bin position (any order, visited once per frame)
    input  wire signed [WD-1:0] x1r0, x1i0, x2r0, x2i0,
    input  wire                 init0,       // 1: R <- instantaneous value (first frame)
    input  wire [2:0]           sh0,         // averaging shift (1 .. 7)
    input  wire [31:0]          load0,       // diagonal load, same units as |X2|^2
    output reg                  ov,
    output reg  [LN-1:0]        ot,
    output reg  signed [WD-1:0] yr, yi
);
    localparam XW = RW + 20;

    // ---- stage 1: registers, covariance memory read ------------------------
    reg signed [WD-1:0] x1r1, x1i1, x2r1, x2i1;
    reg [3*RW-1:0] rmem [0:(1<<LN)-1];
    reg [3*RW-1:0] rd;
    reg [4:0] ctl1; reg [31:0] ld1; reg [LN:0] meta1;
    always @(posedge clk) begin
        x1r1 <= x1r0; x1i1 <= x1i0; x2r1 <= x2r0; x2i1 <= x2i0;
        ctl1 <= {init0, sh0, 1'b0}; ld1 <= load0; meta1 <= {v0, t0};
        rd <= rmem[t0];
    end

    // alignments from stage 1
    wire [3*RW-1:0] rd5;     sfap_dly #(.W(3*RW), .N(4))  u_drd (.clk(clk), .d(rd),    .q(rd5));
    wire [4:0]      ctl6;    sfap_dly #(.W(5),    .N(5))  u_dct (.clk(clk), .d(ctl1),  .q(ctl6));
    wire [31:0]     ld8;     sfap_dly #(.W(32),   .N(7))  u_dld (.clk(clk), .d(ld1),   .q(ld8));
    wire [LN:0]      meta8;   sfap_dly #(.W(LN+1),   .N(7))  u_dm8 (.clk(clk), .d(meta1), .q(meta8));
    wire [LN:0]      meta30;  sfap_dly #(.W(LN+1),   .N(30)) u_dm30(.clk(clk), .d(meta1), .q(meta30));
    wire [2*WD-1:0] x2_25;   sfap_dly #(.W(2*WD), .N(25)) u_dx2 (.clk(clk), .d({x2i1, x2r1}), .q(x2_25));
    wire [2*WD-1:0] x1_29;   sfap_dly #(.W(2*WD), .N(29)) u_dx1 (.clk(clk), .d({x1i1, x1r1}), .q(x1_29));

    // ---- stages 2..5: products and sums -------------------------------------
    reg signed [WD-1:0] a2, b2, c2, d2;
    always @(posedge clk) begin a2 <= x2r1; b2 <= x2i1; c2 <= x1r1; d2 <= x1i1; end
    reg signed [2*WD-1:0] maa3, mbb3, mca3, mdb3, mda3, mcb3;
    always @(posedge clk) begin
        maa3 <= a2 * a2; mbb3 <= b2 * b2; mca3 <= c2 * a2; mdb3 <= d2 * b2; mda3 <= d2 * a2; mcb3 <= c2 * b2;
    end
    reg signed [2*WD-1:0] maa4, mbb4, mca4, mdb4, mda4, mcb4;
    always @(posedge clk) begin
        maa4 <= maa3; mbb4 <= mbb3; mca4 <= mca3; mdb4 <= mdb3; mda4 <= mda3; mcb4 <= mcb3;
    end
    reg signed [RW-1:0] p22_5, p12r_5, p12i_5;
    always @(posedge clk) begin
        p22_5  <= maa4 + mbb4;
        p12r_5 <= mca4 + mdb4;
        p12i_5 <= mda4 - mcb4;
    end
    wire signed [RW-1:0] r22_5  = rd5[RW-1:0];
    wire signed [RW-1:0] r12r_5 = rd5[2*RW-1:RW];
    wire signed [RW-1:0] r12i_5 = rd5[3*RW-1:2*RW];

    // ---- stage 6: differences ---------------------------------------------
    reg signed [RW:0]   dd22_6, dd12r_6, dd12i_6;
    reg signed [RW-1:0] p22_6, p12r_6, p12i_6, r22_6, r12r_6, r12i_6;
    always @(posedge clk) begin
        dd22_6  <= p22_5  - r22_5;
        dd12r_6 <= p12r_5 - r12r_5;
        dd12i_6 <= p12i_5 - r12i_5;
        p22_6 <= p22_5; p12r_6 <= p12r_5; p12i_6 <= p12i_5;
        r22_6 <= r22_5; r12r_6 <= r12r_5; r12i_6 <= r12i_5;
    end

    // ---- stage 7: divide the difference by 2^sh -------------------------------
    reg signed [RW:0]   sd22_7, sd12r_7, sd12i_7;
    reg signed [RW-1:0] p22_7, p12r_7, p12i_7, r22_7, r12r_7, r12i_7;
    reg init7;
    wire [2:0] sh6 = ctl6[3:1];
    always @(posedge clk) begin
        sd22_7  <= dd22_6  >>> sh6;
        sd12r_7 <= dd12r_6 >>> sh6;
        sd12i_7 <= dd12i_6 >>> sh6;
        p22_7 <= p22_6; p12r_7 <= p12r_6; p12i_7 <= p12i_6;
        r22_7 <= r22_6; r12r_7 <= r12r_6; r12i_7 <= r12i_6;
        init7 <= ctl6[4];
    end

    // ---- stage 8: averaged covariance --------------------------------------
    reg signed [RW-1:0] r22_8, r12r_8, r12i_8;
    always @(posedge clk) begin
        r22_8  <= init7 ? p22_7  : r22_7  + sd22_7;
        r12r_8 <= init7 ? p12r_7 : r12r_7 + sd12r_7;
        r12i_8 <= init7 ? p12i_7 : r12i_7 + sd12i_7;
    end

    // ---- stage 9: write back, add the diagonal load -------------------------
    reg we9; reg [LN-1:0] wa9; reg [3*RW-1:0] wd9;
    reg [RW-1:0] d9; reg signed [RW-1:0] r12r_9, r12i_9;
    always @(posedge clk) begin
        we9 <= meta8[LN]; wa9 <= meta8[LN-1:0]; wd9 <= {r12i_8, r12r_8, r22_8};
        d9 <= r22_8 + {{(RW-32){1'b0}}, ld8};
        r12r_9 <= r12r_8; r12i_9 <= r12i_8;
    end
    always @(posedge clk) if (we9) rmem[wa9] <= wd9;

    // ---- stage 10: leading zeros ---------------------------------------------
    function [5:0] clz52(input [51:0] d);
        integer i; reg f;
        begin
            clz52 = 6'd52; f = 1'b0;
            for (i = 51; i >= 0; i = i - 1)
                if (!f && d[i]) begin clz52 = 51 - i; f = 1'b1; end
        end
    endfunction
    // stage 10a: per nibble (13 of them) non-zero flag and leading zeros inside the nibble
    reg [12:0] nz10a; reg [25:0] nl10a; reg [RW-1:0] d10a; reg signed [RW-1:0] r12r_10a, r12i_10a;
    integer ni;
    always @(posedge clk) begin
        for (ni = 0; ni < 13; ni = ni + 1) begin
            nz10a[ni] <= |d9[4*ni +: 4];
            nl10a[2*ni +: 2] <= d9[4*ni+3] ? 2'd0 : d9[4*ni+2] ? 2'd1 : d9[4*ni+1] ? 2'd2 : 2'd3;
        end
        d10a <= d9; r12r_10a <= r12r_9; r12i_10a <= r12i_9;
    end
    // stage 10b: the highest non-zero nibble gives the leading zero count
    function [5:0] top_nib(input [12:0] nz, input [25:0] nl);
        integer k; reg f;
        begin
            top_nib = 6'd52; f = 1'b0;
            for (k = 12; k >= 0; k = k - 1)
                if (!f && nz[k]) begin top_nib = (12 - k) * 4 + nl[2*k +: 2]; f = 1'b1; end
        end
    endfunction
    reg [5:0] lz10; reg [RW-1:0] d10; reg signed [RW-1:0] r12r_10, r12i_10;
    always @(posedge clk) begin lz10 <= top_nib(nz10a, nl10a); d10 <= d10a; r12r_10 <= r12r_10a; r12i_10 <= r12i_10a; end

    // ---- stages 11, 12, 13: normalise D, scale R12 -----------------------------
    reg [RW-1:0] dn11; reg [5:0] lz11, sr11; reg signed [XW-1:0] x11r, x11i;
    always @(posedge clk) begin
        dn11 <= d10 << {lz10[5:3], 3'b000};
        lz11 <= lz10;
        sr11 <= (lz10 > 6'd51) ? 6'd0 : (6'd51 - lz10);
        x11r <= {r12r_10, 20'b0}; x11i <= {r12i_10, 20'b0};
    end
    reg [RW-1:0] dn12; reg [5:0] sr12; reg signed [XW-1:0] x12r, x12i;
    always @(posedge clk) begin
        dn12 <= dn11 << lz11[2:0];
        sr12 <= sr11;
        x12r <= x11r >>> {sr11[5:3], 3'b000}; x12i <= x11i >>> {sr11[5:3], 3'b000};
    end
    reg [17:0] m13; reg signed [XW-1:0] x13r, x13i;
    always @(posedge clk) begin
        m13 <= dn12[RW-1:RW-18];
        x13r <= x12r >>> sr12[2:0]; x13i <= x12i >>> sr12[2:0];
    end

    // ---- stage 14: reciprocal seed ---------------------------------------------
    wire [17:0] r0_14;
    sfap_recip_rom u_rom (.clk(clk), .a(m13[16:8]), .d(r0_14));
    reg [17:0] m14; reg signed [XW-1:0] x14r, x14i;
    always @(posedge clk) begin m14 <= m13; x14r <= x13r; x14i <= x13i; end

    // ---- stages 15..21: Newton-Raphson  r1 = r0 (2 - m r0) ------------------------
    reg signed [37:0] e15;
    always @(posedge clk) e15 <= $signed({1'b0, m14}) * $signed({1'b0, r0_14});
    reg signed [37:0] e16; reg [17:0] r0_15, r0_16;
    always @(posedge clk) begin e16 <= e15; r0_15 <= r0_14; r0_16 <= r0_15; end
    reg signed [19:0] tq17; reg [17:0] r0_17;
    always @(posedge clk) begin tq17 <= (20'sd1 <<< 18) - (e16 >>> 17); r0_17 <= r0_16; end
    reg signed [19:0] tq18; reg [17:0] r0_18;
    always @(posedge clk) begin tq18 <= tq17; r0_18 <= r0_17; end
    reg signed [39:0] rr19;
    always @(posedge clk) rr19 <= $signed({1'b0, r0_18}) * tq18;
    reg signed [39:0] rr20;
    always @(posedge clk) rr20 <= rr19;

    wire [2*XW-1:0] x20;      // scaled R12 aligned to stage 20 (from stage 14: 6 stages)
    sfap_dly #(.W(2*XW), .N(6)) u_dx (.clk(clk), .d({x14i, x14r}), .q(x20));
    localparam signed [WD-1:0] TMAX = {1'b0, {(WD-1){1'b1}}};
    localparam signed [WD-1:0] TMIN = {1'b1, {(WD-1){1'b0}}};
    reg [17:0] r1_21; reg signed [WD-1:0] tr21, ti21;
    wire signed [39:0] rr_s = rr20 >>> 17;
    wire signed [XW-1:0] x20r = x20[XW-1:0];
    wire signed [XW-1:0] x20i = x20[2*XW-1:XW];
    always @(posedge clk) begin
        r1_21 <= (rr_s > 40'sd131071) ? 18'd131071 : (rr_s < 0 ? 18'd0 : rr_s[17:0]);
        tr21  <= (x20r > TMAX) ? TMAX : (x20r < TMIN) ? TMIN : x20r[WD-1:0];
        ti21  <= (x20i > TMAX) ? TMAX : (x20i < TMIN) ? TMIN : x20i[WD-1:0];
    end

    // ---- stages 22..25: c = T * r1 (Q3.20) ------------------------------------
    reg signed [WD-1:0] tr22, ti22; reg signed [18:0] r22;
    always @(posedge clk) begin tr22 <= tr21; ti22 <= ti21; r22 <= $signed({1'b0, r1_21}); end
    reg signed [WD+18:0] cf23r, cf23i;
    always @(posedge clk) begin cf23r <= tr22 * r22; cf23i <= ti22 * r22; end
    reg signed [WD+18:0] cf24r, cf24i;
    always @(posedge clk) begin cf24r <= cf23r; cf24i <= cf23i; end
    localparam signed [23:0] CMAX = 24'sh7FFFFF;
    localparam signed [23:0] CMIN = 24'sh800000;
    reg signed [23:0] cr25, ci25;
    wire signed [WD+18:0] cs_r = cf24r >>> 17;
    wire signed [WD+18:0] cs_i = cf24i >>> 17;
    always @(posedge clk) begin
        cr25 <= (cs_r > CMAX) ? CMAX : (cs_r < CMIN) ? CMIN : cs_r[23:0];
        ci25 <= (cs_i > CMAX) ? CMAX : (cs_i < CMIN) ? CMIN : cs_i[23:0];
    end

    // ---- stages 26..29: c * X2 ------------------------------------------------
    reg signed [23:0] cr26, ci26; reg signed [WD-1:0] x2r26, x2i26;
    always @(posedge clk) begin
        cr26 <= cr25; ci26 <= ci25; x2r26 <= x2_25[WD-1:0]; x2i26 <= x2_25[2*WD-1:WD];
    end
    reg signed [WD+23:0] q1_27, q2_27, q3_27, q4_27;
    always @(posedge clk) begin
        q1_27 <= cr26 * x2r26; q2_27 <= ci26 * x2i26; q3_27 <= cr26 * x2i26; q4_27 <= ci26 * x2r26;
    end
    reg signed [WD+23:0] q1_28, q2_28, q3_28, q4_28;
    always @(posedge clk) begin q1_28 <= q1_27; q2_28 <= q2_27; q3_28 <= q3_27; q4_28 <= q4_27; end
    reg signed [WD+24:0] qr29, qi29;
    always @(posedge clk) begin qr29 <= q1_28 - q2_28; qi29 <= q3_28 + q4_28; end

    // ---- stages 30, 31: Y = X1 - c X2 ----------------------------------------
    reg signed [WD+4:0] y30r, y30i;
    wire signed [WD+24:0] qsr = qr29 >>> 20;
    wire signed [WD+24:0] qsi = qi29 >>> 20;
    always @(posedge clk) begin
        y30r <= $signed(x1_29[WD-1:0])    - qsr;
        y30i <= $signed(x1_29[2*WD-1:WD]) - qsi;
    end
    always @(posedge clk) begin
        yr <= (y30r > TMAX) ? TMAX : (y30r < TMIN) ? TMIN : y30r[WD-1:0];
        yi <= (y30i > TMAX) ? TMAX : (y30i < TMIN) ? TMIN : y30i[WD-1:0];
        ov <= meta30[LN]; ot <= meta30[LN-1:0];
    end
endmodule
