// ============================================================================
// pi_aj_wlpi.v  --  PI-AJ v5 Widely-Linear Space-Time Power Inversion
//
// Power inversion (PI): minimise the array output power with the reference
// element (RX1, centre tap) weight fixed to 1. Written in its generalised
// sidelobe canceller form, it is adapted sample by sample (NLMS):
//
//     d[n] = x0[n-D]                         D = (L-1)/2  (RX1 centre tap)
//     u[n] = [ x0 taps except centre , x1 taps ,          <- linear part
//              conj(x0 taps) , conj(x1 taps) ]            <- widely-linear part
//     y[n] = d[n] - w^H u[n]                 (array output = error)
//     w   += mu * u[n-Q] * conj(y[n-Q]) / 2^(log2 P + C)       (Q = DELAY)
//
// * Space-time (L taps per channel): frequency-dependent weights absorb the
//   RX1/RX2 analog filter mismatch and the jammer's own multipath, so a
//   wideband (barrage) jammer is nulled across the whole band, not only at
//   the centre frequency.
// * Widely-linear (conjugate taps): each AD9361 channel has its own I/Q
//   imbalance, so every jammer has an "image" whose spatial signature differs
//   from the jammer's. A plain 2-element PI has 1 degree of freedom and cannot
//   null both; the conjugate taps give the extra degrees of freedom to do it.
// * Power-of-two normalisation (P = smoothed |x0|^2+|x1|^2, fast attack):
//   no divider, convergence speed independent of the jammer level.
// * Delayed update (Q samples): the error of sample n-Q is used, so the
//   multiplier lanes can be time-shared without a long combinational loop.
//
// Time-multiplexed: NL complex lanes handle NW weights in K = ceil(NW/NL)
// clocks per sample. Requirements:   K >= 2,   clocks/sample >= K+1,
//                                    DELAY*(clocks/sample) >= K + 7
// (defaults NW=19, NL=5 -> K=4: 5 clocks/sample, e.g. 18 MS/s @ >= 90 MHz;
//  NL=10 -> K=2: 3 clocks/sample.)  'overrun' / 'late' flag violations.
// ============================================================================
`timescale 1ns / 1ps
module pi_aj_wlpi #(
    parameter DW    = 18,        // input data width
    parameter L     = 5,         // taps per channel (odd)
    parameter DELAY = 3,         // update delay Q (samples), 1..7
    parameter NL    = 5,         // parallel complex lanes
    parameter WL    = 1,         // 1 = widely linear (conjugate taps)
    parameter WW    = 32,        // weight accumulator width
    parameter WF    = 28,        // weight fraction bits
    parameter WM    = 25,        // weight bits used in the filter multiplier
    parameter OW    = 20         // output width
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 wreset,     // zero all weights (pulse)
    input  wire                 en,         // 0: y = d (reference only)
    input  wire                 freeze,
    input  wire [3:0]           mu_sh,
    input  wire                 in_valid,
    input  wire signed [DW-1:0] x0_re, x0_im, x1_re, x1_im,
    output reg                  out_valid,
    output reg  signed [OW-1:0] y_re, y_im,
    // monitoring
    input  wire [5:0]           rd_sel,
    output reg  signed [WW-1:0] rd_w_re, rd_w_im,
    output reg                  overrun,    // sticky: sample arrived while busy
    output reg                  late        // sticky: DELAY too small for clock
);
    // ------------------------------------------------------------ constants
    localparam NW   = WL ? (4*L - 1) : (2*L - 1);
    localparam K    = (NW + NL - 1) / NL;
    localparam NWP  = K * NL;               // padded weight count
    localparam D    = (L - 1) / 2;
    localparam LB   = L + DELAY;
    localparam OPW  = DW + 1;               // regressor width (conj headroom)
    localparam PFR  = 5;                    // power smoother fraction bits
    localparam PWID = 2*DW + 3 + PFR;
    localparam WFM  = WF - (WW - WM);       // fraction bits of filter weight
    localparam HB   = 3;                    // history buffer index bits

    function integer clog2;
        input integer v;
        integer t;
        begin
            clog2 = 0;
            for (t = v - 1; t > 0; t = t >> 1)
                clog2 = clog2 + 1;
        end
    endfunction
    localparam C = clog2(8*L);              // normaliser constant 2^C ~ 8L

    // weight index -> (channel, tap, conjugate)
    function integer f_ch;
        input integer i;
        f_ch = (i < L-1) ? 0 : (i < 2*L-1) ? 1 : (i < 3*L-1) ? 0 : 1;
    endfunction
    function integer f_tap;
        input integer i;
        f_tap = (i < L-1)   ? ((i < D) ? i : i + 1) :
                (i < 2*L-1) ? i - (L-1) :
                (i < 3*L-1) ? i - (2*L-1) : i - (3*L-1);
    endfunction
    function integer f_cj;
        input integer i;
        f_cj = (i >= 2*L-1) ? 1 : 0;
    endfunction

    function [6:0] msb;
        input [PWID-1:0] v;
        integer k;
        begin
            msb = 0;
            for (k = 0; k < PWID; k = k + 1)
                if (v[k]) msb = k;
        end
    endfunction

    // ------------------------------------------------------------ delay lines
    reg signed [DW-1:0] b0_re [0:LB-1];
    reg signed [DW-1:0] b0_im [0:LB-1];
    reg signed [DW-1:0] b1_re [0:LB-1];
    reg signed [DW-1:0] b1_im [0:LB-1];

    // flattened regressor vectors (filter: taps 0..L-1, update: +DELAY)
    wire [NWP*OPW-1:0] UF_RE, UF_IM, UU_RE, UU_IM;
    genvar gi;
    generate
        for (gi = 0; gi < NWP; gi = gi + 1) begin : g_reg
            if (gi < NW) begin : g_v
                localparam CH = f_ch(gi);
                localparam TP = f_tap(gi);
                localparam CJ = f_cj(gi);
                wire signed [OPW-1:0] fr = CH ? b1_re[TP] : b0_re[TP];
                wire signed [OPW-1:0] fi = CH ? b1_im[TP] : b0_im[TP];
                wire signed [OPW-1:0] ur = CH ? b1_re[TP+DELAY] : b0_re[TP+DELAY];
                wire signed [OPW-1:0] ui = CH ? b1_im[TP+DELAY] : b0_im[TP+DELAY];
                assign UF_RE[gi*OPW +: OPW] = fr;
                assign UF_IM[gi*OPW +: OPW] = CJ ? -fi : fi;
                assign UU_RE[gi*OPW +: OPW] = ur;
                assign UU_IM[gi*OPW +: OPW] = CJ ? -ui : ui;
            end else begin : g_pad
                assign UF_RE[gi*OPW +: OPW] = 0;
                assign UF_IM[gi*OPW +: OPW] = 0;
                assign UU_RE[gi*OPW +: OPW] = 0;
                assign UU_IM[gi*OPW +: OPW] = 0;
            end
        end
    endgenerate

    // ------------------------------------------------------------ weights
    reg signed [WW-1:0] w_re [0:NWP-1];
    reg signed [WW-1:0] w_im [0:NWP-1];

    // ------------------------------------------------------------ control
    reg              start;          // pass begins next cycle
    reg              issuing;
    reg [3:0]        cyc;
    reg [7:0]        sid_ctr;        // id of the sample being issued
    reg [7:0]        sid;            // id of current pass
    reg [7:0]        done_ctr;
    reg signed [DW-1:0] d_buf_re [0:(1<<HB)-1];
    reg signed [DW-1:0] d_buf_im [0:(1<<HB)-1];
    reg signed [DW-1:0] e_buf_re [0:(1<<HB)-1];
    reg signed [DW-1:0] e_buf_im [0:(1<<HB)-1];
    reg [6:0]        lp_buf [0:(1<<HB)-1];

    reg signed [DW-1:0] e_upd_re, e_upd_im;
    reg signed [8:0]    sh_upd;

    // power normaliser pipeline
    reg              pv0, pv1;
    reg [7:0]        psid0, psid1;
    reg [2*DW+1:0]   ip;
    reg [PWID-1:0]   pacc;

    // a new sample may arrive in the last issue cycle of the current pass
    wire busy = start | (issuing & (cyc != K-1));
    wire [7:0] lag_n = sid_ctr - done_ctr;   // samples whose error is pending

    // per-lane pipeline registers
    reg signed [WW-1:0]   p0_w_re [0:NL-1], p0_w_im [0:NL-1];
    reg signed [OPW-1:0]  p0_uf_re[0:NL-1], p0_uf_im[0:NL-1];
    reg signed [OPW-1:0]  p0_uu_re[0:NL-1], p0_uu_im[0:NL-1];
    reg                   p0_val  [0:NL-1];

    reg signed [2*DW+1:0] p1_up_re[0:NL-1], p1_up_im[0:NL-1];
    reg signed [WW-1:0]   p1_w_re [0:NL-1], p1_w_im [0:NL-1];
    reg signed [OPW-1:0]  p1_uf_re[0:NL-1], p1_uf_im[0:NL-1];
    reg                   p1_val  [0:NL-1];

    reg signed [WW-1:0]   p2_w_re [0:NL-1], p2_w_im [0:NL-1];
    reg signed [OPW-1:0]  p2_uf_re[0:NL-1], p2_uf_im[0:NL-1];
    reg                   p2_val  [0:NL-1];

    reg signed [WM+OPW:0] p3_f_re [0:NL-1], p3_f_im [0:NL-1];
    reg                   p3_val  [0:NL-1];

    reg signed [WM+OPW+4:0] acc_re [0:NL-1], acc_im [0:NL-1];

    // stage control (common to all lanes)
    reg s0, s1, s2, s3, s4, s5;
    reg f0, f1, f2, f3;            // first cycle of a pass
    reg l0, l1, l2, l3, l4;        // last cycle of a pass
    reg [7:0] q0, q1, q2, q3, q4, q5;   // sample id
    reg [3:0] c0, c1;                   // issue cycle of the P0/P1 data
    reg signed [WM+OPW+8:0] sum5_re, sum5_im;

    // ------------------------------------------------------------ lane logic
    // P2: weight update  w += round(up * 2^-(sh_upd))  (sh_upd may be < 0)
    wire signed [WW-1:0] wn_re [0:NL-1];
    wire signed [WW-1:0] wn_im [0:NL-1];
    // P3: conj(w) * u
    wire signed [WM-1:0] wm_re [0:NL-1];
    wire signed [WM-1:0] wm_im [0:NL-1];

    // shift amount for P2 (shared): s = sh_upd + 16, clamp [0,63]
    reg  [5:0] sh_p1;

    genvar gl;
    generate
        for (gl = 0; gl < NL; gl = gl + 1) begin : g_lane
            wire signed [63:0] t_re = $signed(p1_up_re[gl]) <<< 16;
            wire signed [63:0] t_im = $signed(p1_up_im[gl]) <<< 16;
            wire signed [63:0] rnd  = (sh_p1 == 0) ? 64'sd0 : (64'sd1 <<< (sh_p1 - 1));
            wire signed [63:0] dl_re = (t_re + rnd) >>> sh_p1;
            wire signed [63:0] dl_im = (t_im + rnd) >>> sh_p1;
            wire signed [WW+1:0] s_re = $signed(p1_w_re[gl]) + dl_re;
            wire signed [WW+1:0] s_im = $signed(p1_w_im[gl]) + dl_im;
            // saturate (dl is clamped to the weight range first)
            wire ovr_re = (dl_re > $signed({1'b0, {(WW-1){1'b1}}})) | (s_re > $signed({1'b0, {(WW-1){1'b1}}}));
            wire unr_re = (dl_re < $signed({1'b1, {(WW-1){1'b0}}})) | (s_re < $signed({1'b1, {(WW-1){1'b0}}}));
            wire ovr_im = (dl_im > $signed({1'b0, {(WW-1){1'b1}}})) | (s_im > $signed({1'b0, {(WW-1){1'b1}}}));
            wire unr_im = (dl_im < $signed({1'b1, {(WW-1){1'b0}}})) | (s_im < $signed({1'b1, {(WW-1){1'b0}}}));
            wire signed [WW-1:0] su_re = ovr_re ? {1'b0, {(WW-1){1'b1}}} : unr_re ? {1'b1, {(WW-1){1'b0}}} : s_re[WW-1:0];
            wire signed [WW-1:0] su_im = ovr_im ? {1'b0, {(WW-1){1'b1}}} : unr_im ? {1'b1, {(WW-1){1'b0}}} : s_im[WW-1:0];
            assign wn_re[gl] = (freeze | ~en) ? p1_w_re[gl] : su_re;
            assign wn_im[gl] = (freeze | ~en) ? p1_w_im[gl] : su_im;
            assign wm_re[gl] = p2_w_re[gl] >>> (WW - WM);
            assign wm_im[gl] = p2_w_im[gl] >>> (WW - WM);
        end
    endgenerate

    // lane operand select: lane ln only ever handles weights c*NL+ln,
    // so each operand is a K-input mux (not an NW-input one)
    reg signed [WW-1:0]  lw_re [0:NL-1], lw_im [0:NL-1];
    reg signed [OPW-1:0] lf_re [0:NL-1], lf_im [0:NL-1];
    reg signed [OPW-1:0] lu_re [0:NL-1], lu_im [0:NL-1];
    integer ml, mc;
    always @* begin
        for (ml = 0; ml < NL; ml = ml + 1) begin
            lw_re[ml] = 0; lw_im[ml] = 0;
            lf_re[ml] = 0; lf_im[ml] = 0; lu_re[ml] = 0; lu_im[ml] = 0;
            for (mc = 0; mc < K; mc = mc + 1) begin
                if (cyc == mc) begin
                    lw_re[ml] = w_re[mc*NL + ml];
                    lw_im[ml] = w_im[mc*NL + ml];
                    lf_re[ml] = UF_RE[(mc*NL + ml)*OPW +: OPW];
                    lf_im[ml] = UF_IM[(mc*NL + ml)*OPW +: OPW];
                    lu_re[ml] = UU_RE[(mc*NL + ml)*OPW +: OPW];
                    lu_im[ml] = UU_IM[(mc*NL + ml)*OPW +: OPW];
                end
            end
        end
    end

    // lane adder tree (combinational, used once per pass)
    reg signed [WM+OPW+8:0] tree_re, tree_im;
    integer t;
    always @* begin
        tree_re = 0; tree_im = 0;
        for (t = 0; t < NL; t = t + 1) begin
            tree_re = tree_re + acc_re[t];
            tree_im = tree_im + acc_im[t];
        end
    end

    // P6: e = d - round(sum >>> WFM)
    localparam signed [WM+OPW+8:0] RND6 = 1 <<< (WFM - 1);
    wire signed [WM+OPW+8:0] fs_re = (sum5_re + RND6) >>> WFM;
    wire signed [WM+OPW+8:0] fs_im = (sum5_im + RND6) >>> WFM;
    wire signed [DW-1:0]     d5_re = d_buf_re[q5[HB-1:0]];
    wire signed [DW-1:0]     d5_im = d_buf_im[q5[HB-1:0]];
    wire signed [WM+OPW+8:0] e_re  = en ? (d5_re - fs_re) : d5_re;
    wire signed [WM+OPW+8:0] e_im  = en ? (d5_im - fs_im) : d5_im;

    function signed [OW-1:0] sat_o;
        input signed [WM+OPW+8:0] v;
        if (v > $signed({1'b0, {(OW-1){1'b1}}}))      sat_o = {1'b0, {(OW-1){1'b1}}};
        else if (v < $signed({1'b1, {(OW-1){1'b0}}})) sat_o = {1'b1, {(OW-1){1'b0}}};
        else                                           sat_o = v[OW-1:0];
    endfunction
    function signed [DW-1:0] sat_e;
        input signed [WM+OPW+8:0] v;
        if (v > $signed({1'b0, {(DW-1){1'b1}}}))      sat_e = {1'b0, {(DW-1){1'b1}}};
        else if (v < $signed({1'b1, {(DW-1){1'b0}}})) sat_e = {1'b1, {(DW-1){1'b0}}};
        else                                           sat_e = v[DW-1:0];
    endfunction

    integer j, ln;
    always @(posedge clk) begin
        if (rst) begin
            start <= 0; issuing <= 0; cyc <= 0;
            sid_ctr <= 0; sid <= 0; done_ctr <= 0;
            overrun <= 0; late <= 0;
            out_valid <= 0; y_re <= 0; y_im <= 0;
            pv0 <= 0; pv1 <= 0; pacc <= 0;
            s0 <= 0; s1 <= 0; s2 <= 0; s3 <= 0; s4 <= 0; s5 <= 0;
            f0 <= 0; f1 <= 0; f2 <= 0; f3 <= 0;
            l0 <= 0; l1 <= 0; l2 <= 0; l3 <= 0; l4 <= 0;
            for (j = 0; j < LB; j = j + 1) begin
                b0_re[j] <= 0; b0_im[j] <= 0; b1_re[j] <= 0; b1_im[j] <= 0;
            end
            for (j = 0; j < NWP; j = j + 1) begin
                w_re[j] <= 0; w_im[j] <= 0;
            end
            for (j = 0; j < (1<<HB); j = j + 1) begin
                e_buf_re[j] <= 0; e_buf_im[j] <= 0; lp_buf[j] <= 0;
                d_buf_re[j] <= 0; d_buf_im[j] <= 0;
            end
            for (j = 0; j < NL; j = j + 1) begin
                acc_re[j] <= 0; acc_im[j] <= 0;
                p0_val[j] <= 0; p1_val[j] <= 0; p2_val[j] <= 0; p3_val[j] <= 0;
            end
        end else begin
            // ================= input / pass control
            start <= 1'b0;
            if (in_valid) begin
                if (busy) begin
                    overrun <= 1'b1;                    // sample dropped
                end else begin
                    for (j = LB-1; j > 0; j = j - 1) begin
                        b0_re[j] <= b0_re[j-1]; b0_im[j] <= b0_im[j-1];
                        b1_re[j] <= b1_re[j-1]; b1_im[j] <= b1_im[j-1];
                    end
                    b0_re[0] <= x0_re; b0_im[0] <= x0_im;
                    b1_re[0] <= x1_re; b1_im[0] <= x1_im;
                    start <= 1'b1;
                end
            end
            if (start) begin
                // delay lines now hold sample n at index 0
                issuing <= 1'b1;
                cyc     <= 0;
                sid     <= sid_ctr;
                sid_ctr <= sid_ctr + 8'd1;
                d_buf_re[sid_ctr[HB-1:0]] <= b0_re[D];
                d_buf_im[sid_ctr[HB-1:0]] <= b0_im[D];
                e_upd_re <= e_buf_re[(sid_ctr - DELAY) & ((1<<HB)-1)];
                e_upd_im <= e_buf_im[(sid_ctr - DELAY) & ((1<<HB)-1)];
                sh_upd   <= $signed({5'd0, mu_sh}) + $signed({2'b00, lp_buf[(sid_ctr - DELAY) & ((1<<HB)-1)]})
                            + C - WF;
                if (lag_n > DELAY - 1)
                    late <= 1'b1;
            end else if (issuing) begin
                if (cyc == K - 1)
                    issuing <= 1'b0;
                else
                    cyc <= cyc + 4'd1;
            end

            // ================= power normaliser (per sample)
            pv0 <= start;
            psid0 <= sid_ctr;
            if (start)
                ip <= b0_re[0]*b0_re[0] + b0_im[0]*b0_im[0] + b1_re[0]*b1_re[0] + b1_im[0]*b1_im[0];
            pv1 <= pv0;
            psid1 <= psid0;
            if (pv0) begin
                if ({3'b000, ip, {PFR{1'b0}}} > {pacc, 3'b000})
                    pacc <= {ip, {PFR{1'b0}}};
                else if ({ip, {PFR{1'b0}}} >= pacc)
                    pacc <= pacc + (({ip, {PFR{1'b0}}} - pacc) >> PFR);
                else
                    pacc <= pacc - ((pacc - {ip, {PFR{1'b0}}}) >> PFR);
            end
            if (pv1)
                lp_buf[psid1[HB-1:0]] <= (msb(pacc) > PFR) ? msb(pacc) - PFR : 7'd0;

            // ================= P0: operand fetch
            s0 <= issuing;
            f0 <= issuing & (cyc == 0);
            l0 <= issuing & (cyc == K-1);
            q0 <= sid;
            sh_p1 <= ($signed(sh_upd) + 16 < 0) ? 6'd0 :
                     ($signed(sh_upd) + 16 > 63) ? 6'd63 : sh_upd + 16;
            c0 <= cyc;
            for (ln = 0; ln < NL; ln = ln + 1) begin
                p0_val[ln]   <= issuing & ((cyc * NL + ln) < NW);
                p0_w_re[ln]  <= lw_re[ln];
                p0_w_im[ln]  <= lw_im[ln];
                p0_uf_re[ln] <= lf_re[ln];
                p0_uf_im[ln] <= lf_im[ln];
                p0_uu_re[ln] <= lu_re[ln];
                p0_uu_im[ln] <= lu_im[ln];
            end

            // ================= P1: update product  u_old * conj(e_old)
            s1 <= s0; f1 <= f0; l1 <= l0; q1 <= q0; c1 <= c0;
            for (ln = 0; ln < NL; ln = ln + 1) begin
                p1_up_re[ln] <= p0_uu_re[ln] * e_upd_re + p0_uu_im[ln] * e_upd_im;
                p1_up_im[ln] <= p0_uu_im[ln] * e_upd_re - p0_uu_re[ln] * e_upd_im;
                p1_w_re[ln]  <= p0_w_re[ln];  p1_w_im[ln]  <= p0_w_im[ln];
                p1_uf_re[ln] <= p0_uf_re[ln]; p1_uf_im[ln] <= p0_uf_im[ln];
                p1_val[ln]   <= p0_val[ln] & s0;
            end

            // ================= P2: weight update + write back
            s2 <= s1; f2 <= f1; l2 <= l1; q2 <= q1;
            for (ln = 0; ln < NL; ln = ln + 1) begin
                p2_w_re[ln]  <= wn_re[ln];   p2_w_im[ln]  <= wn_im[ln];
                p2_uf_re[ln] <= p1_uf_re[ln]; p2_uf_im[ln] <= p1_uf_im[ln];
                p2_val[ln]   <= p1_val[ln];
            end
            // write back: weight j belongs to lane j%NL, issue cycle j/NL
            for (j = 0; j < NW; j = j + 1) begin
                if (p1_val[j % NL] && c1 == j / NL) begin
                    w_re[j] <= wn_re[j % NL];
                    w_im[j] <= wn_im[j % NL];
                end
            end

            // ================= P3: filter product  conj(w) * u
            s3 <= s2; f3 <= f2; l3 <= l2; q3 <= q2;
            for (ln = 0; ln < NL; ln = ln + 1) begin
                p3_f_re[ln] <= wm_re[ln] * p2_uf_re[ln] + wm_im[ln] * p2_uf_im[ln];
                p3_f_im[ln] <= wm_re[ln] * p2_uf_im[ln] - wm_im[ln] * p2_uf_re[ln];
                p3_val[ln]  <= p2_val[ln];
            end

            // ================= P4: per-lane accumulation over the pass
            s4 <= s3 & l3; q4 <= q3;
            for (ln = 0; ln < NL; ln = ln + 1) begin
                if (s3) begin
                    if (f3)
                        acc_re[ln] <= p3_val[ln] ? p3_f_re[ln] : 0;
                    else if (p3_val[ln])
                        acc_re[ln] <= acc_re[ln] + p3_f_re[ln];
                    if (f3)
                        acc_im[ln] <= p3_val[ln] ? p3_f_im[ln] : 0;
                    else if (p3_val[ln])
                        acc_im[ln] <= acc_im[ln] + p3_f_im[ln];
                end
            end

            // ================= P5: lane sum
            s5 <= s4; q5 <= q4;
            if (s4) begin
                sum5_re <= tree_re;
                sum5_im <= tree_im;
            end

            // ================= P6: error = array output
            out_valid <= s5;
            if (s5) begin
                y_re <= sat_o(e_re);
                y_im <= sat_o(e_im);
                e_buf_re[q5[HB-1:0]] <= en ? sat_e(e_re) : 0;
                e_buf_im[q5[HB-1:0]] <= en ? sat_e(e_im) : 0;
                done_ctr <= q5 + 8'd1;
            end

            // ================= weight reset
            if (wreset) begin
                for (j = 0; j < NWP; j = j + 1) begin
                    w_re[j] <= 0; w_im[j] <= 0;
                end
            end

            rd_w_re <= (rd_sel < NWP) ? w_re[rd_sel] : 0;
            rd_w_im <= (rd_sel < NWP) ? w_im[rd_sel] : 0;
        end
    end
endmodule
