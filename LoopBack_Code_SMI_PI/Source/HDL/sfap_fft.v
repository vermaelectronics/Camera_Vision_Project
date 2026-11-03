// ============================================================================
//  sfap_fft.v  --  1024-point radix-2^2 single-path-delay-feedback FFT (DIF)
//
//  One complex sample per clock, natural order in, BIT-REVERSED order out.
//
//  Continuous operation: the caller feeds 1024 data samples and then zeros (so
//  the feedback delay lines flush) and tags every sample with a free-running
//  10-bit slot counter `in_idx`. The tag travels with the data, so no stage has
//  its own counter and nothing needs resetting between frames.
//
//  Every stage does one operator per clock (an add, or a 25x18 multiply with
//  the DSP registers on). The 4.7.0 build failed 245.76 MHz timing by putting
//  several operators in one step; this file is registered at every operator.
//
//  One radix-4 pair (a block of M samples, M = 1024, 256, 64, 16, 4):
//     BF1  delay M/2   a +- b                            (sums first, then differences)
//     BF2  delay M/4   same, with -j applied to b while the difference half goes by
//     TW   multiply by W_M^(r*j)   (not for M = 4)
//  Time order of the four quarters at the pair's output: r = 2, 1, 3, 0, each
//  with j = 0 .. M/4-1; the whole transform comes out in bit-reversed order.
//
//  Fixed point: WD-bit signed data (25 = the DSP48E1 A port), 18-bit Q1.16
//  twiddles. SCALE[s] = 1 halves the output of butterfly s (s = 0: first BF1,
//  s = 9: last BF2), with rounding.
// ============================================================================
`timescale 1ns/1ps

// ---- one radix-2 SDF butterfly stage ----------------------------------------
//  D >= 8 (delay line in RAM): latency 4, every operator in its own clock
//     edge 1  register the input, read the delay RAM
//     edge 2  RAM output register (so the BRAM clock-to-out is not in the add)
//     edge 3  sum and difference (one adder; -j on b is an add/sub select)
//     edge 4  halve (wiring only), choose, register the output and the write data
//     edge 5  write the RAM from registers
//  The loop through the RAM is D slots long and the pipeline in it is 4, so D >= 8 is enough.
//  D < 8 (1, 2, 4: shift register): latency 2, one add/sub and a mux in the loop (it cannot be pipelined).
//  Scaling truncates (no +1): one adder less in the loop; the bias is 0.5 LSB per scaled stage.
module sfap_bf #(
    parameter WD    = 25,
    parameter LOGD  = 9,            // delay D = 2^LOGD
    parameter ROT   = 0,            // 1: -j on b while idx[LOGD+1] == 0 (BF2 of a pair)
    parameter SCALE = 0
) (
    input  wire                 clk,
    input  wire signed [WD-1:0] in_re, in_im,
    input  wire [9:0]           in_idx,
    output reg  signed [WD-1:0] out_re, out_im,
    output reg  [9:0]           out_idx
);
    localparam D = (1 << LOGD);

    function signed [WD-1:0] fit(input signed [WD:0] v);
        begin
            if (SCALE != 0) fit = v >>> 1;
            else            fit = v[WD-1:0];
        end
    endfunction

    generate
    if (LOGD >= 3) begin : g_ram
        if (LOGD >= 7) begin : g_b
            (* ram_style = "block" *) reg [2*WD-1:0] mem [0:D-1];
            reg [2*WD-1:0] rd;
            always @(posedge clk) rd <= mem[in_idx[LOGD-1:0]];
            reg [LOGD-1:0] wa;  reg [2*WD-1:0] wd;  reg we;
            always @(posedge clk) if (we) mem[wa] <= wd;
            // pipeline (shared code below drives wa/wd/we through these nets)
            reg signed [WD-1:0] x1_re, x1_im, x_re, x_im; reg [9:0] x_idx, i1;
            reg [2*WD-1:0] rd2;
            always @(posedge clk) begin
                x_re <= in_re; x_im <= in_im; x_idx <= in_idx;
                x1_re <= x_re; x1_im <= x_im; i1 <= x_idx;
                rd2 <= rd;
            end
            wire signed [WD-1:0] d_re = rd2[WD-1:0];
            wire signed [WD-1:0] d_im = rd2[2*WD-1:WD];
            wire bf1  = i1[LOGD];
            wire rot1 = (ROT != 0) && !i1[LOGD+1];
            reg signed [WD:0] s_re, s_im, f_re, f_im;
            reg signed [WD-1:0] d3_re, d3_im, x3_re, x3_im; reg bf3; reg [9:0] i3; reg [LOGD-1:0] a3;
            always @(posedge clk) begin
                s_re <= rot1 ? ({d_re[WD-1], d_re} + {x1_im[WD-1], x1_im}) : ({d_re[WD-1], d_re} + {x1_re[WD-1], x1_re});
                s_im <= rot1 ? ({d_im[WD-1], d_im} - {x1_re[WD-1], x1_re}) : ({d_im[WD-1], d_im} + {x1_im[WD-1], x1_im});
                f_re <= rot1 ? ({d_re[WD-1], d_re} - {x1_im[WD-1], x1_im}) : ({d_re[WD-1], d_re} - {x1_re[WD-1], x1_re});
                f_im <= rot1 ? ({d_im[WD-1], d_im} + {x1_re[WD-1], x1_re}) : ({d_im[WD-1], d_im} - {x1_im[WD-1], x1_im});
                d3_re <= d_re; d3_im <= d_im; x3_re <= x1_re; x3_im <= x1_im; bf3 <= bf1; i3 <= i1; a3 <= i1[LOGD-1:0];
            end
            always @(posedge clk) begin
                out_idx <= i3;
                if (bf3) begin out_re <= fit(s_re); out_im <= fit(s_im); wd <= {fit(f_im), fit(f_re)}; end
                else     begin out_re <= d3_re;     out_im <= d3_im;     wd <= {x3_im, x3_re}; end
                wa <= a3; we <= 1'b1;
            end
        end else begin : g_l
            (* ram_style = "distributed" *) reg [2*WD-1:0] mem [0:D-1];
            reg [2*WD-1:0] rd;
            always @(posedge clk) rd <= mem[in_idx[LOGD-1:0]];
            reg [LOGD-1:0] wa;  reg [2*WD-1:0] wd;  reg we;
            always @(posedge clk) if (we) mem[wa] <= wd;
            reg signed [WD-1:0] x1_re, x1_im, x_re, x_im; reg [9:0] x_idx, i1;
            reg [2*WD-1:0] rd2;
            always @(posedge clk) begin
                x_re <= in_re; x_im <= in_im; x_idx <= in_idx;
                x1_re <= x_re; x1_im <= x_im; i1 <= x_idx;
                rd2 <= rd;
            end
            wire signed [WD-1:0] d_re = rd2[WD-1:0];
            wire signed [WD-1:0] d_im = rd2[2*WD-1:WD];
            wire bf1  = i1[LOGD];
            wire rot1 = (ROT != 0) && !i1[LOGD+1];
            reg signed [WD:0] s_re, s_im, f_re, f_im;
            reg signed [WD-1:0] d3_re, d3_im, x3_re, x3_im; reg bf3; reg [9:0] i3; reg [LOGD-1:0] a3;
            always @(posedge clk) begin
                s_re <= rot1 ? ({d_re[WD-1], d_re} + {x1_im[WD-1], x1_im}) : ({d_re[WD-1], d_re} + {x1_re[WD-1], x1_re});
                s_im <= rot1 ? ({d_im[WD-1], d_im} - {x1_re[WD-1], x1_re}) : ({d_im[WD-1], d_im} + {x1_im[WD-1], x1_im});
                f_re <= rot1 ? ({d_re[WD-1], d_re} - {x1_im[WD-1], x1_im}) : ({d_re[WD-1], d_re} - {x1_re[WD-1], x1_re});
                f_im <= rot1 ? ({d_im[WD-1], d_im} + {x1_re[WD-1], x1_re}) : ({d_im[WD-1], d_im} - {x1_im[WD-1], x1_im});
                d3_re <= d_re; d3_im <= d_im; x3_re <= x1_re; x3_im <= x1_im; bf3 <= bf1; i3 <= i1; a3 <= i1[LOGD-1:0];
            end
            always @(posedge clk) begin
                out_idx <= i3;
                if (bf3) begin out_re <= fit(s_re); out_im <= fit(s_im); wd <= {fit(f_im), fit(f_re)}; end
                else     begin out_re <= d3_re;     out_im <= d3_im;     wd <= {x3_im, x3_re}; end
                wa <= a3; we <= 1'b1;
            end
        end
    end else begin : g_sr
        // tiny delays (1, 2, 4): shift register of D entries, add/sub and mux in the loop
        reg signed [WD-1:0] x_re, x_im; reg [9:0] x_idx;
        always @(posedge clk) begin x_re <= in_re; x_im <= in_im; x_idx <= in_idx; end
        reg signed [WD-1:0] sr_re [0:D-1];
        reg signed [WD-1:0] sr_im [0:D-1];
        wire signed [WD-1:0] d_re = sr_re[D-1];
        wire signed [WD-1:0] d_im = sr_im[D-1];
        wire bf  = x_idx[LOGD];
        wire rot = (ROT != 0) && !x_idx[LOGD+1];
        wire signed [WD:0] sum_re = rot ? ({d_re[WD-1], d_re} + {x_im[WD-1], x_im}) : ({d_re[WD-1], d_re} + {x_re[WD-1], x_re});
        wire signed [WD:0] sum_im = rot ? ({d_im[WD-1], d_im} - {x_re[WD-1], x_re}) : ({d_im[WD-1], d_im} + {x_im[WD-1], x_im});
        wire signed [WD:0] dif_re = rot ? ({d_re[WD-1], d_re} - {x_im[WD-1], x_im}) : ({d_re[WD-1], d_re} - {x_re[WD-1], x_re});
        wire signed [WD:0] dif_im = rot ? ({d_im[WD-1], d_im} + {x_re[WD-1], x_re}) : ({d_im[WD-1], d_im} - {x_im[WD-1], x_im});
        integer i;
        always @(posedge clk) begin
            out_idx <= x_idx;
            for (i = D-1; i > 0; i = i - 1) begin sr_re[i] <= sr_re[i-1]; sr_im[i] <= sr_im[i-1]; end
            if (bf) begin
                out_re <= fit(sum_re); out_im <= fit(sum_im);
                sr_re[0] <= fit(dif_re); sr_im[0] <= fit(dif_im);
            end else begin
                out_re <= d_re; out_im <= d_im;
                sr_re[0] <= x_re; sr_im[0] <= x_im;
            end
        end
    end
    endgenerate
endmodule

// ---- twiddle multiplier W_M^(r*j) (latency TWLAT = 7) ----------------------
module sfap_tw #(
    parameter WD = 25,
    parameter LM = 10                 // M = 2^LM
) (
    input  wire                 clk,
    input  wire signed [WD-1:0] in_re, in_im,
    input  wire [9:0]           in_idx,
    output reg  signed [WD-1:0] out_re, out_im,
    output reg  [9:0]           out_idx
);
    localparam M = (1 << LM);
    // stage 0: quarter -> r, exponent e = r*j
    wire [1:0]    q = in_idx[LM-1:LM-2];
    wire [LM-3:0] j = in_idx[LM-3:0];
    reg  [LM-1:0] e0;
    reg signed [WD-1:0] a0_re, a0_im; reg [9:0] i0;
    always @(posedge clk) begin
        case (q)
            2'd0: e0 <= {1'b0, j, 1'b0};            // r = 2
            2'd1: e0 <= {2'b00, j};                  // r = 1
            2'd2: e0 <= {1'b0, j, 1'b0} + {2'b00, j}; // r = 3
            default: e0 <= {LM{1'b0}};              // r = 0
        endcase
        a0_re <= in_re; a0_im <= in_im; i0 <= in_idx;
    end
    // stage 1: ROM read
    wire [35:0] tw;
    generate
        if (LM == 10)      sfap_tw_rom_1024 u_rom (.clk(clk), .a(e0), .d(tw));
        else if (LM == 8)  sfap_tw_rom_256  u_rom (.clk(clk), .a(e0), .d(tw));
        else if (LM == 6)  sfap_tw_rom_64   u_rom (.clk(clk), .a(e0), .d(tw));
        else               sfap_tw_rom_16   u_rom (.clk(clk), .a(e0), .d(tw));
    endgenerate
    reg signed [WD-1:0] a1_re, a1_im; reg [9:0] i1;
    always @(posedge clk) begin a1_re <= a0_re; a1_im <= a0_im; i1 <= i0; end
    // stage 2: operand registers
    reg signed [WD-1:0] a2_re, a2_im; reg signed [17:0] c2, s2; reg [9:0] i2;
    always @(posedge clk) begin
        a2_re <= a1_re; a2_im <= a1_im; c2 <= tw[35:18]; s2 <= tw[17:0]; i2 <= i1;
    end
    // stage 3: four products (DSP48E1, 25 x 18)
    reg signed [WD+17:0] m_rc, m_is, m_rs, m_ic; reg [9:0] i3;
    always @(posedge clk) begin
        m_rc <= a2_re * c2; m_is <= a2_im * s2;
        m_rs <= a2_re * s2; m_ic <= a2_im * c2;
        i3 <= i2;
    end
    // stage 3b: second register behind the multipliers (MREG and PREG)
    reg signed [WD+17:0] n_rc, n_is, n_rs, n_ic; reg [9:0] i3b;
    always @(posedge clk) begin n_rc <= m_rc; n_is <= m_is; n_rs <= m_rs; n_ic <= m_ic; i3b <= i3; end
    // stage 4: combine
    reg signed [WD+18:0] p_re, p_im; reg [9:0] i4;
    always @(posedge clk) begin
        p_re <= n_rc - n_is; p_im <= n_rs + n_ic; i4 <= i3b;
    end
    // stage 5: Q1.16 -> data format, saturate
    localparam signed [WD-1:0] MAXV = {1'b0, {(WD-1){1'b1}}};
    localparam signed [WD-1:0] MINV = {1'b1, {(WD-1){1'b0}}};
    wire signed [WD+2:0] t_re = p_re[WD+18:16];
    wire signed [WD+2:0] t_im = p_im[WD+18:16];
    always @(posedge clk) begin
        out_re <= (t_re > MAXV) ? MAXV : (t_re < MINV) ? MINV : t_re[WD-1:0];
        out_im <= (t_im > MAXV) ? MAXV : (t_im < MINV) ? MINV : t_im[WD-1:0];
        out_idx <= i4;
    end
endmodule

// ---- the transform ----------------------------------------------------------
module sfap_fft #(
    parameter WD    = 25,
    parameter SCALE = 10'b0000011111    // halve after the first 5 butterflies (bit s = butterfly s)
) (
    input  wire                 clk,
    input  wire signed [WD-1:0] in_re, in_im,
    input  wire [9:0]           in_idx,      // free-running slot counter, 0 .. 1023
    output wire signed [WD-1:0] out_re, out_im,
    output wire [9:0]           out_idx
);
    // pair p: M = 1024 >> (2p); BF1 delay M/2 = 2^(9-2p), BF2 delay M/4 = 2^(8-2p)
    wire signed [WD-1:0] r0 [0:5], i0 [0:5];
    wire [9:0]           t0 [0:5];
    assign r0[0] = in_re; assign i0[0] = in_im; assign t0[0] = in_idx;
    genvar p;
    generate
        for (p = 0; p < 5; p = p + 1) begin : g_pair
            wire signed [WD-1:0] ar, ai, br, bi;
            wire [9:0] at, bt;
            sfap_bf #(.WD(WD), .LOGD(9-2*p), .ROT(0), .SCALE(SCALE[2*p]))
                u_bf1 (.clk(clk), .in_re(r0[p]), .in_im(i0[p]), .in_idx(t0[p]),
                       .out_re(ar), .out_im(ai), .out_idx(at));
            sfap_bf #(.WD(WD), .LOGD(8-2*p), .ROT(1), .SCALE(SCALE[2*p+1]))
                u_bf2 (.clk(clk), .in_re(ar), .in_im(ai), .in_idx(at),
                       .out_re(br), .out_im(bi), .out_idx(bt));
            if (p < 4) begin : g_tw
                sfap_tw #(.WD(WD), .LM(10-2*p))
                    u_tw (.clk(clk), .in_re(br), .in_im(bi), .in_idx(bt),
                          .out_re(r0[p+1]), .out_im(i0[p+1]), .out_idx(t0[p+1]));
            end else begin : g_nt
                assign r0[p+1] = br; assign i0[p+1] = bi; assign t0[p+1] = bt;
            end
        end
    endgenerate
    assign out_re = r0[5]; assign out_im = i0[5]; assign out_idx = t0[5];
endmodule
