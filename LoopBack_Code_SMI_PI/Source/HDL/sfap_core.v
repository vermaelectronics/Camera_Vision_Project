// ============================================================================
//  sfap_core.v  --  SFAP: space-frequency adaptive processing (SMI-PI 5.0)
//
//  Two antennas, 1024-point sqrt-Hann STFT with 50 % overlap, one 2x2
//  closed-form power-inversion weight per FFT bin (sfap_bin.v), inverse FFT,
//  overlap-add. Everything runs in the sample clock domain (245.76 MHz) with
//  one sample per `in_v` strobe; nothing waits for the ARM.
//
//     x1,x2 (12 bit) -> buffer 2048 -> window -> FFT (shared by both antennas)
//        -> X1 buffer -> sfap_bin (X1 - c X2) -> reorder RAM -> window -> FFT
//        -> frame RAM (bit-reversed write) -> window, overlap-add, make-up
//        -> output FIFO -> out_v/out_i/out_q at the input strobe rate
//
//  Schedule (slot = 1024 clocks, hop = 2 slots = 512 samples at 61.44 MS/s):
//     slot A : antenna 1 frame through the forward FFT
//     slot B : antenna 2 frame through the forward FFT (X1 is read back as X2 arrives)
//  The forward FFT is therefore busy 100 % at 61.44 MS/s; the inverse FFT 50 %.
//  Latency from input to output is about 2600 sample strobes (42 us).
//
//  Registers (all quasi-static, from the AXI side through synchronisers):
//     en      run (0 resets the whole chain)
//     clr     re-initialise the covariance (level; the next frame after it falls starts afresh)
//     sh_lam  averaging 1 - 2^-sh_lam per frame (3..6; 5 = 32 frames = 270 us)
//     load    diagonal load added to |X2|^2 (about the noise power per bin)
//     mk      make-up gain 2^mk (0..7), undoes the RX gain cut
// ============================================================================
`timescale 1ns/1ps

module sfap_core (
    input  wire         clk,
    input  wire         en,
    input  wire         clr,
    input  wire [2:0]   sh_lam,
    input  wire [31:0]  load,
    input  wire [2:0]   mk,
    input  wire         in_v,
    input  wire signed [15:0] x1i, x1q, x2i, x2q,   // 12-bit samples, right aligned
    input  wire signed [15:0] k1r, k1i, k2r, k2i,   // I/Q-image corrector coefficients, Q1.15 (0 = off)
    output wire [191:0] iq_stat,                    // {P2,S2i2,S2r2,P1,S2i1,S2r1}: sums over 2^16 samples (>> 8)
    output reg          out_v,
    output reg  signed [15:0] out_i, out_q,
    output reg          ready,          // first frames delivered, output is live
    output reg          underrun,       // sticky: the output FIFO ran dry
    output reg          overrun,        // sticky: frames fell behind the input
    output reg  [15:0]  frames,
    output reg  [31:0]  in_mag, out_mag // mean |I|+|Q| x 2^15 samples, input and output
);
    localparam WD = 25;

    function [9:0] brev(input [9:0] a);
        brev = {a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9]};
    endfunction

    // en edge-safe copy
    reg en_s = 1'b0;
    always @(posedge clk) en_s <= en;

    // ------------------------------------------------------------------ input capture
    // blind I/Q-image corrector in front of the buffer (one per receiver), 6 clocks
    wire iq_v1, iq_v2;
    wire signed [15:0] c1i, c1q, c2i, c2q;
    wire [31:0] s2r1, s2i1, p1, s2r2, s2i2, p2;
    sfap_iq u_iq1 (.clk(clk), .v(in_v), .xi(x1i), .xq(x1q), .kr(k1r), .ki(k1i),
                   .ov(iq_v1), .yi(c1i), .yq(c1q), .s2r_l(s2r1), .s2i_l(s2i1), .p_l(p1));
    sfap_iq u_iq2 (.clk(clk), .v(in_v), .xi(x2i), .xq(x2q), .kr(k2r), .ki(k2i),
                   .ov(iq_v2), .yi(c2i), .yq(c2q), .s2r_l(s2r2), .s2i_l(s2i2), .p_l(p2));
    assign iq_stat = {p2, s2i2, s2r2, p1, s2i1, s2r1};
    reg [10:0] wp = 11'd0;
    reg [47:0] ibuf [0:2047];
    always @(posedge clk) if (iq_v1) ibuf[wp] <= {c2q[11:0], c2i[11:0], c1q[11:0], c1i[11:0]};

    // samples that arrived beyond what the started frames consumed: frame f needs
    // 512 f + 1024 samples, i.e. acc >= 1024
    reg [12:0] acc = 13'd0;
    reg [1:0]  base_n = 2'd0;          // frame start (sample 512 f) / 512 mod 4
    reg [1:0]  fbase = 2'd0;           // base of the frame being fed
    reg [9:0]  aidx = 10'd0;           // slot counter (issue side)
    reg [1:0]  kind_cur = 2'd0, kind_prev = 2'd0;   // 0 zeros, 1 antenna 1, 2 antenna 2
    reg        ph = 1'b0;
    wire       slot_end = (aidx == 10'd1023);
    reg        start_frame;
    always @(*) start_frame = slot_end && en_s && !ph && (acc >= 13'd1024);

    always @(posedge clk) begin
        aidx <= aidx + 10'd1;
        if (!en_s) begin
            wp <= 11'd0; acc <= 13'd0; base_n <= 2'd0; ph <= 1'b0; kind_cur <= 2'd0; kind_prev <= 2'd0;
            frames <= 16'd0; overrun <= 1'b0;
        end else begin
            if (iq_v1) wp <= wp + 11'd1;
            if (in_v) begin if (acc != 13'h1FFF) acc <= acc + 13'd1; end
            if (in_v && acc > 13'd1792) overrun <= 1'b1;
            if (slot_end) begin
                kind_prev <= kind_cur;
                if (ph) begin kind_cur <= 2'd2; ph <= 1'b0; end
                else if (start_frame) begin
                    kind_cur <= 2'd1; ph <= 1'b1; fbase <= base_n; base_n <= base_n + 2'd1;
                    frames <= frames + 16'd1;
                    acc <= acc - 13'd512 + (in_v ? 13'd1 : 13'd0);
                end else kind_cur <= 2'd0;
            end
        end
    end

    // ------------------------------------------------------------------ analysis feed
    // a0: issue the read
    wire [10:0] raddr = {fbase, 9'd0} + {1'b0, aidx};
    wire [15:0] win_a, win_s;
    reg  [9:0]  on = 10'd0;             // overlap-add pass counter (below)
    reg  [9:0]  onw = 10'd0;            // own address register for the synthesis window (cuts the fan-out of 'on')
    always @(posedge clk) onw <= on;
    sfap_win_rom u_win (.clk(clk), .a(aidx), .d(win_a), .b(onw), .e(win_s));   // port b: 2 clocks after onw
    reg [47:0] rd48;
    reg [1:0]  k1; reg [9:0] g1;
    always @(posedge clk) begin
        rd48 <= ibuf[raddr];
        k1 <= kind_cur; g1 <= aidx;
    end
    // a2: channel select, operand registers
    reg signed [12:0] si2, sq2; reg [16:0] w2; reg [9:0] g2;
    always @(posedge clk) begin
        case (k1)
            2'd1:    begin si2 <= {rd48[11], rd48[11:0]};  sq2 <= {rd48[23], rd48[23:12]}; end
            2'd2:    begin si2 <= {rd48[35], rd48[35:24]}; sq2 <= {rd48[47], rd48[47:36]}; end
            default: begin si2 <= 13'sd0; sq2 <= 13'sd0; end
        endcase
        w2 <= {1'b0, win_a}; g2 <= g1;
    end
    // a3: products
    reg signed [29:0] pi3, pq3; reg [9:0] g3;
    always @(posedge clk) begin pi3 <= si2 * $signed(w2); pq3 <= sq2 * $signed(w2); g3 <= g2; end
    // a3b: second register behind the multipliers (MREG and PREG)
    reg signed [29:0] pi3b, pq3b; reg [9:0] g3b;
    always @(posedge clk) begin pi3b <= pi3; pq3b <= pq3; g3b <= g3; end
    // a4: round, drop 12 bits -> FFT input
    reg signed [WD-1:0] fa_re, fa_im; reg [9:0] fa_idx;
    wire signed [29:0] pi3r = pi3b + 30'sd2048;
    wire signed [29:0] pq3r = pq3b + 30'sd2048;
    always @(posedge clk) begin
        fa_re <= pi3r >>> 12; fa_im <= pq3r >>> 12; fa_idx <= g3b;
    end

    // The FFT returns bin number t = out_idx + 1 (its latency is 1068 clocks, tag latency 1067 + 1):
    // one register stage turns the tag into the bin position.
    wire signed [WD-1:0] xa_re0, xa_im0; wire [9:0] xa_tag;
    sfap_fft #(.WD(WD), .SCALE(10'b1000000000)) u_fftA
        (.clk(clk), .in_re(fa_re), .in_im(fa_im), .in_idx(fa_idx),
         .out_re(xa_re0), .out_im(xa_im0), .out_idx(xa_tag));
    reg signed [WD-1:0] xa_re, xa_im; reg [9:0] xa_idx;
    always @(posedge clk) begin xa_re <= xa_re0; xa_im <= xa_im0; xa_idx <= xa_tag + 10'd1; end

    // ------------------------------------------------------------------ X1 buffer, bin processor
    reg [1:0] okind = 2'd0;
    wire [1:0] okind_now = (xa_idx == 10'd0) ? kind_prev : okind;
    always @(posedge clk) okind <= okind_now;

    reg [2*WD-1:0] x1buf [0:1023];
    always @(posedge clk) if (okind_now == 2'd1) x1buf[xa_idx] <= {xa_im, xa_re};
    reg [2*WD-1:0] x1rd;
    always @(posedge clk) x1rd <= x1buf[xa_idx];
    reg bv; reg [9:0] bt; reg signed [WD-1:0] b2r, b2i;
    always @(posedge clk) begin
        bv <= (okind_now == 2'd2) && en_s; bt <= xa_idx; b2r <= xa_re; b2i <= xa_im;
    end

    // covariance initialisation: the first frame after enable / clear starts the averages
    reg rinit = 1'b1;
    reg clr_d = 1'b0;
    always @(posedge clk) begin
        clr_d <= clr;
        if (!en_s || (clr && !clr_d)) rinit <= 1'b1;
        else if (bv && bt == 10'd1023) rinit <= 1'b0;
    end

    wire yv; wire [9:0] yt; wire signed [WD-1:0] yr, yi;
    sfap_bin #(.WD(WD)) u_bin
        (.clk(clk), .v0(bv), .t0(bt),
         .x1r0($signed(x1rd[WD-1:0])), .x1i0($signed(x1rd[2*WD-1:WD])), .x2r0(b2r), .x2i0(b2i),
         .init0(rinit), .sh0(sh_lam), .load0(load),
         .ov(yv), .ot(yt), .yr(yr), .yi(yi));

    // ------------------------------------------------------------------ reorder RAM (bit-reversed write, natural read)
    reg [2*WD-1:0] rbuf [0:2047];
    reg wb = 1'b0;                      // bank being written
    reg y_pend = 1'b0, y_bank = 1'b0;   // a complete block waits for the inverse FFT
    reg yw_v = 1'b0; reg [10:0] yw_a; reg [2*WD-1:0] yw_d;                 // registered write (the negation is its own stage)
    always @(posedge clk) begin yw_v <= yv; yw_a <= {wb, brev(yt)}; yw_d <= {-yi, yr}; end   // conjugate
    always @(posedge clk) if (yw_v) rbuf[yw_a] <= yw_d;
    // inverse-side slot counter and read control
    reg [9:0] iidx = 10'd0;
    reg       rd_cur = 1'b0, rd_prev = 1'b0, rd_bank = 1'b0;
    always @(posedge clk) begin
        iidx <= iidx + 10'd1;
        if (!en_s) begin y_pend <= 1'b0; wb <= 1'b0; rd_cur <= 1'b0; rd_prev <= 1'b0; end
        else begin
            if (yv && yt == 10'd1023) begin y_pend <= 1'b1; y_bank <= wb; wb <= ~wb; end
            if (iidx == 10'd1023) begin
                rd_prev <= rd_cur;
                if (y_pend && !(yv && yt == 10'd1023)) begin rd_cur <= 1'b1; rd_bank <= y_bank; y_pend <= 1'b0; end
                else rd_cur <= 1'b0;
            end
        end
    end
    reg [2*WD-1:0] rbq; reg rdv1; reg [9:0] ig1;
    always @(posedge clk) begin
        rbq <= rbuf[{rd_bank, iidx}];
        rdv1 <= rd_cur; ig1 <= iidx;
    end
    wire signed [WD-1:0] fb_re = rdv1 ? $signed(rbq[WD-1:0])    : {WD{1'b0}};
    wire signed [WD-1:0] fb_im = rdv1 ? $signed(rbq[2*WD-1:WD]) : {WD{1'b0}};

    wire signed [WD-1:0] xb_re0, xb_im0; wire [9:0] xb_tag;
    sfap_fft #(.WD(WD), .SCALE(10'b1111111111)) u_fftB
        (.clk(clk), .in_re(fb_re), .in_im(fb_im), .in_idx(ig1),
         .out_re(xb_re0), .out_im(xb_im0), .out_idx(xb_tag));
    reg signed [WD-1:0] xb_re, xb_im; reg [9:0] xb_idx;
    always @(posedge clk) begin xb_re <= xb_re0; xb_im <= xb_im0; xb_idx <= xb_tag + 10'd1; end

    // ------------------------------------------------------------------ frame RAM (time domain, bit-reversed write)
    reg ik_valid = 1'b0;
    wire ik_now = (xb_idx == 10'd0) ? rd_prev : ik_valid;
    always @(posedge clk) ik_valid <= ik_now;
    localparam [17:0] FMAX = 18'h1FFFF;
    localparam [17:0] FMIN = 18'h20000;
    reg [35:0] fram [0:2047];
    reg fb = 1'b0;                       // bank being written
    reg f_pend = 1'b0, f_bank = 1'b0;
    // three registered stages between the inverse FFT and the frame RAM (5.1.2: the negate, the
    // range test and the clamp each get their own clock; one operator per stage)
    // c1: conjugate back (negate in WD+1 bits so -FMIN-like values cannot wrap)
    reg signed [WD:0] c1_re, c1_im; reg c1_v = 1'b0; reg [10:0] c1_a;
    always @(posedge clk) begin
        c1_re <= {xb_re[WD-1], xb_re};
        c1_im <= -{xb_im[WD-1], xb_im};
        c1_v  <= ik_now; c1_a <= {fb, brev(xb_idx)};
    end
    // c2: does the value fit 18 bits? (upper bits all equal to the sign)
    wire [WD-17:0] c1r_up = c1_re[WD:17];
    wire [WD-17:0] c1i_up = c1_im[WD:17];
    reg c2_ore, c2_oim, c2_sre, c2_sim; reg [17:0] c2_re, c2_im; reg c2_v = 1'b0; reg [10:0] c2_a;
    always @(posedge clk) begin
        c2_ore <= !((&c1r_up) || !(|c1r_up)); c2_sre <= c1_re[WD];
        c2_oim <= !((&c1i_up) || !(|c1i_up)); c2_sim <= c1_im[WD];
        c2_re <= c1_re[17:0]; c2_im <= c1_im[17:0];
        c2_v <= c1_v; c2_a <= c1_a;
    end
    // c3: clamp and register as the RAM write port
    reg fw_v = 1'b0; reg [10:0] fw_a; reg [35:0] fw_d;
    always @(posedge clk) begin
        fw_v <= c2_v; fw_a <= c2_a;
        fw_d <= { c2_oim ? (c2_sim ? FMIN : FMAX) : c2_im,
                  c2_ore ? (c2_sre ? FMIN : FMAX) : c2_re };
    end
    always @(posedge clk) if (fw_v) fram[fw_a] <= fw_d;
    // OLA pass
    reg        ola = 1'b0; reg ob = 1'b0;
    reg        tail_ok = 1'b0, pass1 = 1'b1;
    always @(posedge clk) begin
        if (!en_s) begin f_pend <= 1'b0; fb <= 1'b0; ola <= 1'b0; on <= 10'd0; tail_ok <= 1'b0; pass1 <= 1'b1; end
        else begin
            if (ik_now && xb_idx == 10'd1023) begin f_pend <= 1'b1; f_bank <= fb; fb <= ~fb; end
            if (!ola) begin
                if (f_pend && !(ik_now && xb_idx == 10'd1023)) begin ola <= 1'b1; ob <= f_bank; on <= 10'd0; f_pend <= 1'b0; end
            end else begin
                on <= on + 10'd1;
                if (on == 10'd1023) begin ola <= 1'b0; tail_ok <= 1'b1; pass1 <= 1'b0; end
            end
        end
    end
    // o0 -> o1: read frame RAM, tail RAM and the synthesis window
    reg [35:0] fq; reg [39:0] tq; reg ov1; reg [9:0] on1; reg tok1, p1_1;
    reg [39:0] tail [0:511];
    always @(posedge clk) begin
        fq <= fram[{ob, on}];
        tq <= tail[on[8:0]];
        ov1 <= ola; on1 <= on; tok1 <= tail_ok; p1_1 <= pass1;
    end
    // o1b, o1c: two more clocks, the window arrives 2 clocks later than the RAM data (onw register + ROM output register)
    reg [35:0] fqb, fqc; reg [39:0] tqb, tqc; reg ov1b, ov1c; reg [9:0] on1b, on1c; reg tok1b, tok1c, p1_1b, p1_1c;
    always @(posedge clk) begin
        fqb <= fq; tqb <= tq; ov1b <= ov1; on1b <= on1; tok1b <= tok1; p1_1b <= p1_1;
        fqc <= fqb; tqc <= tqb; ov1c <= ov1b; on1c <= on1b; tok1c <= tok1b; p1_1c <= p1_1b;
    end
    // o2: operands
    reg signed [17:0] cr2, ci2; reg [16:0] sw2; reg signed [19:0] tr2, ti2; reg ov2, p1_2; reg [9:0] on2;
    always @(posedge clk) begin
        cr2 <= fqc[17:0]; ci2 <= fqc[35:18]; sw2 <= {1'b0, win_s};
        tr2 <= tok1c ? tqc[19:0] : 20'sd0; ti2 <= tok1c ? tqc[39:20] : 20'sd0; ov2 <= ov1c; on2 <= on1c; p1_2 <= p1_1c;
    end
    // o3: products
    reg signed [34:0] pr3, pi3o; reg signed [19:0] tr3, ti3; reg ov3, p1_3; reg [9:0] on3;
    always @(posedge clk) begin
        pr3 <= cr2 * $signed(sw2); pi3o <= ci2 * $signed(sw2);
        tr3 <= tr2; ti3 <= ti2; ov3 <= ov2; on3 <= on2; p1_3 <= p1_2;
    end
    // o3b: second register behind the multipliers (MREG and PREG)
    reg signed [34:0] pr3b, pi3ob; reg signed [19:0] tr3b, ti3b; reg ov3b, p1_3b; reg [9:0] on3b;
    always @(posedge clk) begin
        pr3b <= pr3; pi3ob <= pi3o; tr3b <= tr3; ti3b <= ti3; ov3b <= ov3; on3b <= on3; p1_3b <= p1_3;
    end
    // o4: window result, overlap-add with the stored tail
    reg signed [19:0] wr4, wi4, sr4, si4; reg ov4, p1_4; reg [9:0] on4;
    wire signed [19:0] wrs = pr3b[34:15];      // x w / 2^15 -> one extra fractional bit kept
    wire signed [19:0] wis = pi3ob[34:15];
    always @(posedge clk) begin
        wr4 <= wrs; wi4 <= wis;
        sr4 <= wrs + tr3b; si4 <= wis + ti3b;   // tail is stored in the same format
        ov4 <= ov3b; on4 <= on3b; p1_4 <= p1_3b;
    end
    always @(posedge clk) if (ov4 && on4[9]) tail[on4[8:0]] <= {wi4, wr4};
    // o5: make-up gain: (v << mk) >> 4  (v = 2 x 8 x x12 because one fractional bit is kept)
    reg signed [27:0] mr5, mi5; reg ov5; reg [9:0] on5;
    always @(posedge clk) begin
        mr5 <= ($signed({{8{sr4[19]}}, sr4}) <<< mk) >>> 4;
        mi5 <= ($signed({{8{si4[19]}}, si4}) <<< mk) >>> 4;
        ov5 <= ov4 && !on4[9] && !p1_4; on5 <= on4;
    end
    // o6a: does the value fit 16 bits? (bits 27:15 all equal to the sign); o6: clamp, write the output FIFO
    reg [27:0] mr6a, mi6a; reg ovr6a, ovi6a, ov6a;
    always @(posedge clk) begin
        mr6a <= mr5; mi6a <= mi5;
        ovr6a <= !((&mr5[27:15]) || !(|mr5[27:15]));
        ovi6a <= !((&mi5[27:15]) || !(|mi5[27:15]));
        ov6a <= ov5;
    end
    reg [31:0] ofifo [0:1023];
    reg [9:0] ofw = 10'd0, ofr = 10'd0;
    reg signed [15:0] sat_r, sat_i; reg ov6;
    always @(posedge clk) begin
        sat_r <= ovr6a ? (mr6a[27] ? 16'sh8000 : 16'sh7FFF) : mr6a[15:0];
        sat_i <= ovi6a ? (mi6a[27] ? 16'sh8000 : 16'sh7FFF) : mi6a[15:0];
        ov6 <= ov6a;
    end
    always @(posedge clk) if (ov6) ofifo[ofw] <= {sat_i, sat_r};
    wire [9:0] level = ofw - ofr;
    reg primed = 1'b0;
    reg [31:0] fq32;
    always @(posedge clk) fq32 <= ofifo[ofr];
    always @(posedge clk) begin
        if (!en_s) begin ofw <= 10'd0; ofr <= 10'd0; primed <= 1'b0; ready <= 1'b0; underrun <= 1'b0; out_v <= 1'b0; end
        else begin
            if (ov6) ofw <= ofw + 10'd1;
            if (!primed && level >= 10'd256) begin primed <= 1'b1; ready <= 1'b1; end
            out_v <= 1'b0;
            if (in_v && primed) begin
                out_v <= 1'b1;
                if (level != 10'd0) begin out_i <= fq32[15:0]; out_q <= fq32[31:16]; ofr <= ofr + 10'd1; end
                else begin out_i <= 16'sd0; out_q <= 16'sd0; underrun <= 1'b1; end
            end
        end
    end

    // ------------------------------------------------------------------ power statistics
    function [12:0] ab12(input signed [11:0] a);
        ab12 = a[11] ? (~{1'b0, a} + 13'd1) : {1'b0, a};
    endfunction
    function [16:0] ab16(input signed [15:0] a);
        ab16 = a[15] ? (~{1'b0, a} + 17'd1) : {1'b0, a};
    endfunction
    reg [14:0] scnt = 15'd0; reg [31:0] ia = 32'd0, oa = 32'd0;
    reg [12:0] am1, am2; reg am_v;
    always @(posedge clk) begin
        am1 <= ab12(x1i[11:0]); am2 <= ab12(x1q[11:0]); am_v <= in_v;
        if (am_v) begin
            ia <= ia + am1 + am2;
            oa <= oa + ab16(out_i) + ab16(out_q);       // out_v follows in_v by one clock
            scnt <= scnt + 15'd1;
            if (scnt == 15'h7FFF) begin in_mag <= ia; out_mag <= oa; ia <= 32'd0; oa <= 32'd0; end
        end
    end
endmodule
