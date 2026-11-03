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
//  One bin per clock, latency 27. Every stage is one operator (add, shift or
//  multiply with the DSP registers), because the 4.7.0 build failed 245.76 MHz
//  timing by packing several operators into one step.
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
    parameter RW = 52
) (
    input  wire                 clk,
    input  wire                 v0,
    input  wire [9:0]           t0,          // bin position (any order, visited once per frame)
    input  wire signed [WD-1:0] x1r0, x1i0, x2r0, x2i0,
    input  wire                 init0,       // 1: R <- instantaneous value (first frame)
    input  wire [2:0]           sh0,         // averaging shift (1 .. 7)
    input  wire [31:0]          load0,       // diagonal load, same units as |X2|^2
    output reg                  ov,
    output reg  [9:0]           ot,
    output reg  signed [WD-1:0] yr, yi
);
    // ---- stage 1: registers, covariance memory read ------------------------
    reg v1; reg [9:0] t1; reg signed [WD-1:0] x1r1, x1i1, x2r1, x2i1; reg init1; reg [2:0] sh1; reg [31:0] ld1;
    reg [3*RW-1:0] rmem [0:1023];
    reg [3*RW-1:0] rd;
    always @(posedge clk) begin
        v1 <= v0; t1 <= t0; x1r1 <= x1r0; x1i1 <= x1i0; x2r1 <= x2r0; x2i1 <= x2i0;
        init1 <= init0; sh1 <= sh0; ld1 <= load0;
        rd <= rmem[t0];
    end

    // ---- stage 2..5: products and sums -------------------------------------
    reg signed [WD-1:0] a2, b2, c2, d2; reg v2; reg [9:0] t2;
    always @(posedge clk) begin a2 <= x2r1; b2 <= x2i1; c2 <= x1r1; d2 <= x1i1; v2 <= v1; t2 <= t1; end
    reg signed [2*WD-1:0] maa3, mbb3, mca3, mdb3, mda3, mcb3; reg v3; reg [9:0] t3;
    always @(posedge clk) begin
        maa3 <= a2 * a2; mbb3 <= b2 * b2; mca3 <= c2 * a2; mdb3 <= d2 * b2; mda3 <= d2 * a2; mcb3 <= c2 * b2;
        v3 <= v2; t3 <= t2;
    end
    reg signed [2*WD-1:0] maa4, mbb4, mca4, mdb4, mda4, mcb4; reg v4; reg [9:0] t4;
    always @(posedge clk) begin
        maa4 <= maa3; mbb4 <= mbb3; mca4 <= mca3; mdb4 <= mdb3; mda4 <= mda3; mcb4 <= mcb3; v4 <= v3; t4 <= t3;
    end
    reg signed [RW-1:0] p22_5, p12r_5, p12i_5; reg v5; reg [9:0] t5;
    always @(posedge clk) begin
        p22_5  <= maa4 + mbb4;
        p12r_5 <= mca4 + mdb4;
        p12i_5 <= mda4 - mcb4;
        v5 <= v4; t5 <= t4;
    end

    // covariance read data, and the controls, aligned to stage 5
    wire [3*RW-1:0] rd5;  wire init5_w; wire [2:0] sh5_w;
    sfap_dly #(.W(3*RW), .N(4)) u_drd (.clk(clk), .d(rd), .q(rd5));
    wire [3:0] ctl5;
    sfap_dly #(.W(4), .N(4)) u_dctl (.clk(clk), .d({init1, sh1}), .q(ctl5));
    wire [31:0] ld7;
    sfap_dly #(.W(32), .N(6)) u_dld (.clk(clk), .d(ld1), .q(ld7));   // aligned to stage 7

    // ---- stage 6: differences ---------------------------------------------
    reg signed [RW:0] dd22_6, dd12r_6, dd12i_6;
    reg signed [RW-1:0] p22_6, p12r_6, p12i_6, r22_6, r12r_6, r12i_6;
    reg v6; reg [9:0] t6; reg [3:0] ctl6;
    wire signed [RW-1:0] r22_5  = rd5[RW-1:0];
    wire signed [RW-1:0] r12r_5 = rd5[2*RW-1:RW];
    wire signed [RW-1:0] r12i_5 = rd5[3*RW-1:2*RW];
    always @(posedge clk) begin
        dd22_6  <= p22_5  - r22_5;
        dd12r_6 <= p12r_5 - r12r_5;
        dd12i_6 <= p12i_5 - r12i_5;
        p22_6 <= p22_5; p12r_6 <= p12r_5; p12i_6 <= p12i_5;
        r22_6 <= r22_5; r12r_6 <= r12r_5; r12i_6 <= r12i_5;
        v6 <= v5; t6 <= t5; ctl6 <= ctl5;
    end

    // ---- stage 7: averaged covariance --------------------------------------
    reg signed [RW-1:0] r22_7, r12r_7, r12i_7; reg v7; reg [9:0] t7;
    wire init6 = ctl6[3]; wire [2:0] sh6 = ctl6[2:0];
    always @(posedge clk) begin
        r22_7  <= init6 ? p22_6  : r22_6  + (dd22_6  >>> sh6);
        r12r_7 <= init6 ? p12r_6 : r12r_6 + (dd12r_6 >>> sh6);
        r12i_7 <= init6 ? p12i_6 : r12i_6 + (dd12i_6 >>> sh6);
        v7 <= v6; t7 <= t6;
    end

    // ---- stage 8: write back, add the diagonal load -------------------------
    reg we8; reg [9:0] wa8; reg [3*RW-1:0] wd8;
    reg [RW-1:0] d8; reg signed [RW-1:0] r12r_8, r12i_8; reg v8; reg [9:0] t8;
    always @(posedge clk) begin
        we8 <= v7; wa8 <= t7; wd8 <= {r12i_7, r12r_7, r22_7};
        d8 <= r22_7 + {{(RW-32){1'b0}}, ld7};
        r12r_8 <= r12r_7; r12i_8 <= r12i_7; v8 <= v7; t8 <= t7;
    end
    always @(posedge clk) if (we8) rmem[wa8] <= wd8;

    // ---- stage 9: leading zeros --------------------------------------------
    function [5:0] clz52(input [51:0] d);
        integer i; reg f;
        begin
            clz52 = 6'd52; f = 1'b0;
            for (i = 51; i >= 0; i = i - 1)
                if (!f && d[i]) begin clz52 = 51 - i; f = 1'b1; end
        end
    endfunction
    reg [5:0] lz9; reg [RW-1:0] d9; reg signed [RW-1:0] r12r_9, r12i_9; reg v9; reg [9:0] t9;
    always @(posedge clk) begin
        lz9 <= clz52(d8); d9 <= d8; r12r_9 <= r12r_8; r12i_9 <= r12i_8; v9 <= v8; t9 <= t8;
    end

    // ---- stages 10, 11, 12: normalise D, scale R12 -----------------------------
    reg [RW-1:0] dn10; reg [5:0] lz10, sr10; reg signed [RW+19:0] x10r, x10i; reg v10; reg [9:0] t10;
    always @(posedge clk) begin
        dn10 <= d9 << {lz9[5:3], 3'b000};
        lz10 <= lz9;
        sr10 <= (lz9 > 6'd51) ? 6'd0 : (6'd51 - lz9);
        x10r <= {r12r_9, 20'b0}; x10i <= {r12i_9, 20'b0};
        v10 <= v9; t10 <= t9;
    end
    reg [RW-1:0] dn11; reg [5:0] sr11; reg signed [RW+19:0] x11r, x11i; reg v11; reg [9:0] t11;
    always @(posedge clk) begin
        dn11 <= dn10 << lz10[2:0];
        sr11 <= sr10;
        x11r <= x10r >>> {sr10[5:3], 3'b000}; x11i <= x10i >>> {sr10[5:3], 3'b000};
        v11 <= v10; t11 <= t10;
    end
    reg [17:0] m12; reg signed [RW+19:0] x12r, x12i; reg v12; reg [9:0] t12;
    always @(posedge clk) begin
        m12 <= dn11[RW-1:RW-18];
        x12r <= x11r >>> sr11[2:0]; x12i <= x11i >>> sr11[2:0];
        v12 <= v11; t12 <= t11;
    end

    // ---- stage 13: reciprocal seed ------------------------------------------
    wire [17:0] r0_13;
    sfap_recip_rom u_rom (.clk(clk), .a(m12[16:8]), .d(r0_13));
    reg [17:0] m13; reg signed [RW+19:0] x13r, x13i; reg v13; reg [9:0] t13;
    always @(posedge clk) begin m13 <= m12; x13r <= x12r; x13i <= x12i; v13 <= v12; t13 <= t12; end

    // ---- stages 14..18: Newton-Raphson  r1 = r0 (2 - m r0) --------------------
    reg signed [37:0] e14; reg [17:0] r0_14; reg signed [RW+19:0] x14r, x14i; reg v14; reg [9:0] t14;
    always @(posedge clk) begin
        e14 <= $signed({1'b0, m13}) * $signed({1'b0, r0_13});
        r0_14 <= r0_13; x14r <= x13r; x14i <= x13i; v14 <= v13; t14 <= t13;
    end
    reg signed [19:0] tq15; reg [17:0] r0_15; reg signed [RW+19:0] x15r, x15i; reg v15; reg [9:0] t15;
    always @(posedge clk) begin
        tq15 <= (20'sd1 <<< 18) - (e14 >>> 17);
        r0_15 <= r0_14; x15r <= x14r; x15i <= x14i; v15 <= v14; t15 <= t14;
    end
    reg signed [19:0] tq16; reg [17:0] r0_16; reg signed [RW+19:0] x16r, x16i; reg v16; reg [9:0] t16;
    always @(posedge clk) begin
        tq16 <= tq15; r0_16 <= r0_15; x16r <= x15r; x16i <= x15i; v16 <= v15; t16 <= t15;
    end
    reg signed [39:0] rr17; reg signed [RW+19:0] x17r, x17i; reg v17; reg [9:0] t17;
    always @(posedge clk) begin
        rr17 <= $signed({1'b0, r0_16}) * tq16;
        x17r <= x16r; x17i <= x16i; v17 <= v16; t17 <= t16;
    end
    // r1 (Q1.17) and the scaled R12 clamped to 25 bits
    localparam signed [WD-1:0] TMAX = {1'b0, {(WD-1){1'b1}}};
    localparam signed [WD-1:0] TMIN = {1'b1, {(WD-1){1'b0}}};
    reg [17:0] r1_18; reg signed [WD-1:0] tr18, ti18; reg v18; reg [9:0] t18;
    wire signed [39:0] rr_s = rr17 >>> 17;
    always @(posedge clk) begin
        r1_18 <= (rr_s > 40'sd131071) ? 18'd131071 : (rr_s < 0 ? 18'd0 : rr_s[17:0]);
        tr18  <= (x17r > TMAX) ? TMAX : (x17r < TMIN) ? TMIN : x17r[WD-1:0];
        ti18  <= (x17i > TMAX) ? TMAX : (x17i < TMIN) ? TMIN : x17i[WD-1:0];
        v18 <= v17; t18 <= t17;
    end

    // ---- stages 19..21: c = T * r1 (Q3.20) ------------------------------------
    reg signed [WD-1:0] tr19, ti19; reg signed [18:0] r19; reg v19; reg [9:0] t19;
    always @(posedge clk) begin tr19 <= tr18; ti19 <= ti18; r19 <= $signed({1'b0, r1_18}); v19 <= v18; t19 <= t18; end
    reg signed [WD+18:0] cf20r, cf20i; reg v20; reg [9:0] t20;
    always @(posedge clk) begin cf20r <= tr19 * r19; cf20i <= ti19 * r19; v20 <= v19; t20 <= t19; end
    localparam signed [23:0] CMAX = 24'sh7FFFFF;
    localparam signed [23:0] CMIN = 24'sh800000;
    reg signed [23:0] cr21, ci21; reg v21; reg [9:0] t21;
    wire signed [WD+18:0] cs_r = cf20r >>> 17;
    wire signed [WD+18:0] cs_i = cf20i >>> 17;
    always @(posedge clk) begin
        cr21 <= (cs_r > CMAX) ? CMAX : (cs_r < CMIN) ? CMIN : cs_r[23:0];
        ci21 <= (cs_i > CMAX) ? CMAX : (cs_i < CMIN) ? CMIN : cs_i[23:0];
        v21 <= v20; t21 <= t20;
    end

    // ---- stages 22..25: c * X2 ------------------------------------------------
    wire [2*WD-1:0] x2_21;       // X2 aligned to stage 21
    sfap_dly #(.W(2*WD), .N(20)) u_dx2 (.clk(clk), .d({x2i1, x2r1}), .q(x2_21));
    reg signed [23:0] cr22, ci22; reg signed [WD-1:0] x2r22, x2i22; reg v22; reg [9:0] t22;
    always @(posedge clk) begin
        cr22 <= cr21; ci22 <= ci21; x2r22 <= x2_21[WD-1:0]; x2i22 <= x2_21[2*WD-1:WD]; v22 <= v21; t22 <= t21;
    end
    reg signed [WD+23:0] q1_23, q2_23, q3_23, q4_23; reg v23; reg [9:0] t23;
    always @(posedge clk) begin
        q1_23 <= cr22 * x2r22; q2_23 <= ci22 * x2i22; q3_23 <= cr22 * x2i22; q4_23 <= ci22 * x2r22;
        v23 <= v22; t23 <= t22;
    end
    reg signed [WD+23:0] q1_24, q2_24, q3_24, q4_24; reg v24; reg [9:0] t24;
    always @(posedge clk) begin q1_24 <= q1_23; q2_24 <= q2_23; q3_24 <= q3_23; q4_24 <= q4_23; v24 <= v23; t24 <= t23; end
    reg signed [WD+24:0] qr25, qi25; reg v25; reg [9:0] t25;
    always @(posedge clk) begin qr25 <= q1_24 - q2_24; qi25 <= q3_24 + q4_24; v25 <= v24; t25 <= t24; end

    // ---- stages 26, 27: Y = X1 - c X2 ----------------------------------------
    wire [2*WD-1:0] x1_25;
    sfap_dly #(.W(2*WD), .N(24)) u_dx1 (.clk(clk), .d({x1i1, x1r1}), .q(x1_25));
    reg signed [WD+4:0] y26r, y26i; reg v26; reg [9:0] t26;
    wire signed [WD+24:0] qsr = qr25 >>> 20;
    wire signed [WD+24:0] qsi = qi25 >>> 20;
    always @(posedge clk) begin
        y26r <= $signed(x1_25[WD-1:0])      - qsr;
        y26i <= $signed(x1_25[2*WD-1:WD])   - qsi;
        v26 <= v25; t26 <= t25;
    end
    always @(posedge clk) begin
        yr <= (y26r > TMAX) ? TMAX : (y26r < TMIN) ? TMIN : y26r[WD-1:0];
        yi <= (y26i > TMAX) ? TMAX : (y26i < TMIN) ? TMIN : y26i[WD-1:0];
        ov <= v26; ot <= t26;
    end
endmodule
