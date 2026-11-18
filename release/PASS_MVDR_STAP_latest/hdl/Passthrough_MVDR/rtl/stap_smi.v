// -----------------------------------------------------------------------------
// stap_smi.v  --  SMI space-time adaptive processing (2 antennas x L taps
//                 + 2 widely-linear image taps), MVDR / LCMV with a
//                 programmable space-time constraint vector
//
// v4 (1.14b): L and the reference tap REF are parameters (mvdr_top: L = 16,
//   REF = 8, N = 34). With REF in the middle the RX2 taps reach 8 samples
//   before and 7 after the reference: RX1/RX2 differences that last longer
//   than a few samples (filters, LNAs, cables, multipath) are equalised.
//   The other team's SMI-PI (same board, >50 dB) runs 31 space-time taps;
//   its largest RX2 weight sits at delay 9, out of reach of 8 taps.
//   The covariance (595 entries at L = 16) is read in PAGES of 128 entries
//   (PAGE register); the weights moved to off 0x210. INFO = {3, REF, L, 'S'}.
//
// v3 (1.14): two WIDELY-LINEAR taps conj(x1(n-REF)), conj(x2(n-REF)) join the
//   space-time vector (N = 18). The AD9361 is zero-IF with the LO at the band
//   centre, so each channel carries an IQ image eps_k * conj(jammer). For a
//   barrage centred on the LO the image overlaps the jammer in frequency: two
//   sources at every frequency, which 2 antennas cannot null with linear taps
//   (model: -25 dBc image -> 18.7 dB residual, the field ceiling). The image is
//   a linear function of conj(x), so the conj taps cancel it (3.5 dB).
//   DC REMOVAL at the input (CTRL[2] = 1 turns it off): per channel and
//   component e(n) = x(n) - mm, x' = sat(e(n)), m += e(n-1), mm = m >>> 14
//   (registered; the one-sample lag in the loop is immaterial at 2^-14):
//   a notch ~fs/100k wide. The
//   AD9361 DC / LO leakage is a separate constant "source" in each channel
//   (the other team logs ~530 LSB under a jammer); removed, it neither uses a
//   degree of freedom of the null nor reaches TX1 at the band centre.
//
// WHAT
//   Sample-matrix-inversion STAP. The FPGA does the per-sample work, the ARM
//   does the matrix algebra (double precision):
//     1. FPGA  : estimates the N x N space-time covariance (N = 2L + 2 = 34)
//                  R = sum_snapshots  x x^H,
//                  x = [x1(n..n-L+1), x2(n..n-L+1), conj x1(n-REF), conj x2(n-REF)]
//                over a window of 2^KEXP snapshots, in two ping-pong banks.
//     2. ARM   : LOCKs, reads the published bank (N(N+1)/2 entries, i <= j),
//                  w = (R + lambda I)^-1 a / (a^H (R + lambda I)^-1 a)
//                (MVDR; a = reference tap -> SMI power inversion, or a
//                 satellite space-time steering vector), writes the N complex
//                weights and pulses WLOAD.
//     3. FPGA  : y(n) = w^H x(n), the new weight set taking effect on one
//                sample boundary (coherent swap).
//   Default weights (reset): w = e_REF (RX1 tap REF = 1): y = x1(n - REF),
//   i.e. the raw RX1 signal, delayed REF samples.
//
// FORMATS
//   x : signed DW (= 15: IQ-corrected 12-bit x4, as everywhere)
//   w : Q4.20 in 25 bits (|w| < 16; firmware scales a larger set by 2^-k)
//   y : Q15, y = x * 2^15 for w = e_REF -- the same as beamformer_2ch and
//       stap_lms, so output_scaler / AOLC take it unchanged.
//   R : AW = 48-bit sums of x_i conj(x_j) (x in DW units), up to 2^16
//       snapshots without overflow.
//
// SAMPLE RATE: the N-tap filter is time-shared over 4 clocks per sample
//   (N DSP48 instead of 4N). LVDS 2R2T delivers one sample every 4 clocks
//   (l_clk = 4 x fs), which is the design's operating point. A sample arriving
//   sooner is counted in NOVR (status) and restarts the filter.
//
// COVARIANCE ENGINE: 2 complex multiply lanes (8 DSP48) sweep the NE unique
//   entries of one snapshot in NE/2 clocks (+ pipeline drain): L 16 -> 595
//   entries, 298 clocks, a snapshot every ~75 samples (L 8: 171, ~24).
//   Snapshot decimation costs nothing in accuracy that matters: 2^8
//   snapshots = 7.5 N (firmware default KEXP 8 at L 16, 0.43 ms). Entries are accumulated in block RAM
//   (read-modify-write, block RAM) with ping-pong banks:
//     - the ACTIVE bank accumulates; after 2^KEXP snapshots it is PUBLISHED
//       (SEQ+1) and the other bank becomes active (first sweep overwrites)
//     - while LOCK is set the published bank is never switched, so the ARM
//       reads one consistent window; the active bank keeps accumulating and
//       COUNT reports how many snapshots it really holds.
//   Entry order e = 0..NE-1: i = 0..N-1 outer, j = i..N-1 inner (R_ij, i <= j);
//   lane = e[0], slot = e >> 1. Index k: 0..L-1 = RX1 delay 0..L-1 (0 newest),
//   L..2L-1 = RX2 delay 0..L-1, 2L = conj RX1(n-REF), 2L+1 = conj RX2(n-REF).
//
// AXI WINDOW (aclk = s_axi_aclk), word offset off = rd/wr word address - 0x100
//   (gnss_passthrough maps byte 0x400..0xFFF):
//   off 0x000..0x1FF  COV  PAGE of 128 entries: entry e = 128 PAGE + i at 4i:
//                          +0 re[31:0] +1 im[31:0] +2 {im[47:32], re[47:32]} +3 0
//                          (published bank)
//   off 0x200  PAGE   RW [2:0] covariance page
//   off 0x210+2k / +2k+1  W_k re / im  RW  Q4.20, 25 bits (sign-extended), k 0..N-1
//   off 0x2B0  CTRL   RW [0] EN (output from SMI) [1] LOCK [2] DC removal OFF
//                        [11:8] KEXP (def 9)
//   off 0x2B1  STAT   RO [0] LOCK_ACK [1] published bank [2] ACTIVE
//                        [31:16] SEQ (published windows)
//   off 0x2B2  COUNT  RO snapshots in the published window
//   off 0x2B3  NOVR   RO samples closer than 4 clocks (filter restarted)
//   off 0x2B4  INFO   RO {8'd3 version (3 = paged, + 2 image taps), 8'd REF, 8'd L, 8'h53}
//   off 0x2B5  WLOAD  W  any value: apply the weight shadow; R [0] pending
//   off 0x2B6  NLOAD  RO weight sets applied
//
// ARM PROTOCOL: poll STAT until SEQ changes; set LOCK; wait for LOCK_ACK;
//   re-read STAT (SEQ, bank) and COUNT, read the entries page by page; clear LOCK;
//   solve; write the 2N weight words; write WLOAD; (WLOAD reads 1 until the
//   set is applied -- don't write the shadow while it does). LOCK_ACK comes
//   two clocks after LOCK is seen, so no publish can follow the acknowledge.
//
// TIMING (l_clk up to 250 MHz): every per-sample load enable is an explicit
//   KEEP copy of v0 (ld[], one per half filter lane; snap_ld); the filter
//   operands come out of a per-lane 4-deep shift chain (no phase mux); the
//   pair sums run every clock (no clock enables), only tagged samples are
//   picked up; the window-full / publish decision and the overrun event are
//   registered; the accumulator read is zeroed by synchronous reset for the
//   first sweep; the slot counter, the (i, j) entry walkers and slot_last are
//   held by the registered iss alone. Yosys estimate (v5, L 16): WNS +0.19 ns
//   at 4.0 ns. Resources (v5, L 16): 42 DSP48, 11 RAMB36 (two 1024 x 192
//   copies: accumulate + ARM read-out), ~9.7k LUTs, ~15.6k FFs before SRL
//   packing; whole gnss_passthrough ~37k LUT (70 % of the XC7Z020).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module stap_smi #(
    parameter DW    = 15,
    parameter OUT_W = 35,
    parameter L     = 16,               // taps per antenna (8..16)
    parameter REF   = 8                 // reference tap (RX1 delay REF)
)(
    // ---------------- sample domain ----------------
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     in_valid,
    input  wire signed [DW-1:0]     x1_i,
    input  wire signed [DW-1:0]     x1_q,
    input  wire signed [DW-1:0]     x2_i,
    input  wire signed [DW-1:0]     x2_q,
    output reg                      out_valid = 1'b0,
    output reg  signed [OUT_W-1:0]  y_re = 0,
    output reg  signed [OUT_W-1:0]  y_im = 0,
    output reg                      active = 1'b0,     // EN applied (clk)
    // ---------------- AXI side (aclk) ----------------
    input  wire                     aclk,
    input  wire                     wr_en,
    input  wire        [9:0]        wr_addr,           // word address (0x100..0x3FF)
    input  wire        [31:0]       wr_data,
    input  wire        [9:0]        rd_addr,
    output reg         [31:0]       rd_data
);
    localparam NI  = 2;                     // widely-linear image taps
    localparam N   = 2 * L + NI;            // 34 taps at L = 16
    localparam NE  = N * (N + 1) / 2;       // 595 unique covariance entries
    localparam NS  = (NE + 1) / 2;          // 298 slots per lane (lane 1 of the last is unused)
    localparam SW  = 9;                     // slot counter width (NS <= 512)
    localparam IW  = 6;                     // tap index width (N <= 64)
    localparam AW  = 48;                    // covariance accumulator
    localparam WM  = 25;                    // weight, Q4.20
    localparam PW  = WM + DW;               // 40-bit filter products
    localparam CP  = 2 * DW;                // 30-bit covariance products
    localparam [WM-1:0] W_ONE = 25'sd1048576;

    localparam [9:0] OFF_CTRL  = 10'h2B0, OFF_STAT = 10'h2B1, OFF_COUNT = 10'h2B2,
                     OFF_NOVR  = 10'h2B3, OFF_INFO = 10'h2B4, OFF_WLOAD = 10'h2B5,
                     OFF_NLOAD = 10'h2B6, OFF_W    = 10'h210, OFF_PAGE = 10'h200;

    integer k, j;
    genvar  g;

    // =====================================================================
    //  AXI side: control, weight shadow, read mux
    // =====================================================================
    reg  [31:0]         ctrl = 32'h0000_0900;              // EN 0, LOCK 0, KEXP 9
    reg  signed [WM-1:0] wsh_r [0:N-1];
    reg  signed [WM-1:0] wsh_i [0:N-1];
    reg                 wtog = 1'b0;
    reg  [2:0]          page = 3'd0;                       // covariance page (128 entries)
    integer ia;
    initial for (ia = 0; ia < N; ia = ia + 1) begin
        wsh_r[ia] = (ia == REF) ? W_ONE : {WM{1'b0}};
        wsh_i[ia] = {WM{1'b0}};
    end

    wire [9:0] woff = wr_addr - 10'h100;
    wire [9:0] roff = rd_addr - 10'h100;
    (* ASYNC_REG = "TRUE" *) reg wa_m = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg wa_s = 1'b0;
    reg  wack = 1'b0;                                       // clk domain
    wire wpend = (wtog != wa_s);

    always @(posedge aclk) begin
        wa_m <= wack;
        wa_s <= wa_m;
        if (wr_en) begin
            if (woff == OFF_CTRL) ctrl <= wr_data;
            if (woff == OFF_PAGE) page <= wr_data[2:0];
            if (woff == OFF_WLOAD && !wpend) wtog <= ~wtog;
            if (woff >= OFF_W && woff < OFF_W + 2*N) begin
                if (woff[0]) wsh_i[(woff - OFF_W) >> 1] <= wr_data[WM-1:0];
                else         wsh_r[(woff - OFF_W) >> 1] <= wr_data[WM-1:0];
            end
        end
    end

    // =====================================================================
    //  clk side: control synchronisers, weight load
    // =====================================================================
    (* ASYNC_REG = "TRUE" *) reg [6:0] c_m = 7'd0;
    (* ASYNC_REG = "TRUE" *) reg [6:0] c_s = 7'd0;
    wire en_s    = c_s[0];
    wire lock_s  = c_s[1];
    wire [3:0] kexp_s = c_s[5:2];
    wire dcoff_s = c_s[6];
    reg  lock_ack = 1'b0;

    (* ASYNC_REG = "TRUE" *) reg wt_m = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg wt_s = 1'b0;
    reg  wt_p = 1'b0;
    (* max_fanout = 64 *) reg wld = 1'b0;          // registered load strobe
    reg  signed [WM-1:0] wcf_r [0:N-1];
    reg  signed [WM-1:0] wcf_i [0:N-1];
    reg  [15:0] nload = 16'd0;
    integer ib;
    initial for (ib = 0; ib < N; ib = ib + 1) begin
        wcf_r[ib] = (ib == REF) ? W_ONE : {WM{1'b0}};
        wcf_i[ib] = {WM{1'b0}};
    end

    always @(posedge clk) begin
        c_m    <= {ctrl[2], ctrl[11:8], ctrl[1], ctrl[0]};
        c_s    <= c_m;
        active <= en_s;
        wt_m   <= wtog;
        wt_s   <= wt_m;
        wt_p   <= wt_s;
        wld    <= (wt_s != wt_p);
        if (wld) begin                              // shadow stable until wack returns
            for (k = 0; k < N; k = k + 1) begin
                wcf_r[k] <= wsh_r[k];
                wcf_i[k] <= wsh_i[k];
            end
            wack  <= wt_p;
            nload <= nload + 16'd1;
        end
    end

    // =====================================================================
    //  DC removal (one clock): m += d, d = x - m >>> DCK, x' = sat(d)
    // =====================================================================
    localparam DCK = 14;
    localparam MW_ = DW + DCK + 1;
    reg signed [MW_-1:0] m1i = 0, m1q = 0, m2i = 0, m2q = 0;
    reg signed [DW:0]    mm1i = 0, mm1q = 0, mm2i = 0, mm2q = 0;   // m >>> DCK, registered
    reg signed [DW:0]    r1i = 0, r1q = 0, r2i = 0, r2q = 0;       // e of the previous sample
    reg signed [DW-1:0]  d1i = 0, d1q = 0, d2i = 0, d2q = 0;
    reg                  dv = 1'b0;
    wire signed [DW:0]   e1i = $signed({x1_i[DW-1], x1_i}) - mm1i;
    wire signed [DW:0]   e1q = $signed({x1_q[DW-1], x1_q}) - mm1q;
    wire signed [DW:0]   e2i = $signed({x2_i[DW-1], x2_i}) - mm2i;
    wire signed [DW:0]   e2q = $signed({x2_q[DW-1], x2_q}) - mm2q;
    function signed [DW-1:0] sat_dw;
        input signed [DW:0] v;
        begin
            if (v[DW] != v[DW-1]) sat_dw = v[DW] ? {1'b1, {(DW-1){1'b0}}} : {1'b0, {(DW-1){1'b1}}};
            else                  sat_dw = v[DW-1:0];
        end
    endfunction
    always @(posedge clk) begin
        dv <= in_valid & ~rst;
        mm1i <= m1i >>> DCK;  mm1q <= m1q >>> DCK;  mm2i <= m2i >>> DCK;  mm2q <= m2q >>> DCK;
        if (rst || dcoff_s) begin
            m1i <= 0;  m1q <= 0;  m2i <= 0;  m2q <= 0;
            r1i <= 0;  r1q <= 0;  r2i <= 0;  r2q <= 0;
        end else if (in_valid) begin
            m1i <= m1i + r1i;  m1q <= m1q + r1q;  m2i <= m2i + r2i;  m2q <= m2q + r2q;
            r1i <= e1i;        r1q <= e1q;        r2i <= e2i;        r2q <= e2q;
        end
        if (in_valid) begin
            d1i <= sat_dw(e1i);  d1q <= sat_dw(e1q);  d2i <= sat_dw(e2i);  d2q <= sat_dw(e2q);
        end
    end

    // =====================================================================
    //  taps (shift on dv); v0 = clock after a shift
    // =====================================================================
    reg signed [DW-1:0] t1r [0:L-1], t1i [0:L-1], t2r [0:L-1], t2i [0:L-1];
    reg                 v0 = 1'b0;
    integer ic;
    initial for (ic = 0; ic < L; ic = ic + 1) begin
        t1r[ic] = 0; t1i[ic] = 0; t2r[ic] = 0; t2i[ic] = 0;
    end
    always @(posedge clk) begin
        v0 <= dv & ~rst;
        if (dv) begin
            t1r[0] <= d1i;  t1i[0] <= d1q;
            t2r[0] <= d2i;  t2i[0] <= d2q;
            for (k = 1; k < L; k = k + 1) begin
                t1r[k] <= t1r[k-1];  t1i[k] <= t1i[k-1];
                t2r[k] <= t2r[k-1];  t2i[k] <= t2i[k-1];
            end
        end
    end
    wire signed [DW-1:0] xr [0:N-1];
    wire signed [DW-1:0] xi [0:N-1];
    generate
        for (g = 0; g < N; g = g + 1) begin : g_x
            if (g < L) begin : g1
                assign xr[g] = t1r[g];  assign xi[g] = t1i[g];
            end else if (g < 2*L) begin : g2
                assign xr[g] = t2r[g-L];  assign xi[g] = t2i[g-L];
            end else if (g == 2*L) begin : g3          // image tap: conj x1(n-REF)
                assign xr[g] = t1r[REF];  assign xi[g] = -t1i[REF];
            end else begin : g4                        // image tap: conj x2(n-REF)
                assign xr[g] = t2r[REF];  assign xi[g] = -t2i[REF];
            end
        end
    endgenerate

    // =====================================================================
    //  FILTER  y = w^H x, 16 lanes, one DSP each, 4 phases per sample
    //    ph0: wr*xr  ph1: wi*xi  -> re = sum      (conj(w) x)
    //    ph2: wr*xi  ph3: -wi*xr -> im = sum
    // =====================================================================
    // Per lane: weights and taps are loaded once per sample (ld[] = KEEP
    // copies of v0, one per lane), a per-lane copy of the phase (phl, KEEP)
    // picks the operands -- no shared high-fanout select:
    //   ph0 wr*xr  ph1 wi*xi  -> re = p0 + p1      (conj(w) x)
    //   ph2 wr*xi  ph3 wi*xr  -> im = p2 - p3
    (* keep = "true", shreg_extract = "no" *) reg [N-1:0] ld = {N{1'b0}};
    reg signed [DW-1:0] fxr [0:N-1], fxi [0:N-1];
    reg signed [WM-1:0] fwr [0:N-1], fwi [0:N-1];
    (* keep = "true", shreg_extract = "no" *) reg [1:0] phl [0:N-1];
    (* shreg_extract = "no" *) reg [1:0] pd1 [0:N-1], pd2 [0:N-1], pd3 [0:N-1];
    reg        [1:0]    ph = 2'd0;
    reg                 run = 1'b0;
    reg        [15:0]   novr = 16'd0;
    reg                 ovr_ev = 1'b0;
    integer id;
    initial for (id = 0; id < N; id = id + 1) begin
        fxr[id] = 0; fxi[id] = 0; fwr[id] = 0; fwi[id] = 0;
        phl[id] = 2'd0; pd1[id] = 2'd0; pd2[id] = 2'd0; pd3[id] = 2'd0;
    end
    always @(posedge clk) begin
        ld <= {N{dv & ~rst}};                       // = v0
        for (k = 0; k < N; k = k + 1) begin
            if (ld[k]) begin
                fxr[k] <= xr[k];     fxi[k] <= xi[k];
                fwr[k] <= wcf_r[k];  fwi[k] <= wcf_i[k];
                phl[k] <= 2'd0;
            end else begin
                phl[k] <= phl[k] + 2'd1;
            end
        end
        ovr_ev <= v0 & run & (ph != 2'd3);          // sample arrived mid-filter
        if (ovr_ev) novr <= novr + 16'd1;
        if (rst) begin
            run <= 1'b0;
        end else if (v0) begin
            ph  <= 2'd0;
            run <= 1'b1;
        end else if (run) begin
            ph <= ph + 2'd1;
            if (ph == 2'd3) run <= 1'b0;
        end
    end

    // operand (DSP A/B reg) / M / P / pair stages. The pair sum runs every
    // clock; only the samples tagged valid are picked up at the output.
    reg signed [WM-1:0] opa [0:N-1];
    reg signed [DW-1:0] opb [0:N-1];
    reg signed [PW-1:0] fm  [0:N-1];
    reg signed [PW-1:0] fp  [0:N-1];
    reg signed [PW-1:0] fh  [0:N-1];
    reg signed [PW:0]   fs  [0:N-1];
    reg        [2:0]    otag = 3'd0, mtag = 3'd0, ptag = 3'd0;  // {valid, ph}
    reg                 sv = 1'b0, sim = 1'b0;
    integer ie;
    initial for (ie = 0; ie < N; ie = ie + 1) begin
        opa[ie] = 0; opb[ie] = 0; fm[ie] = 0; fp[ie] = 0; fh[ie] = 0; fs[ie] = 0;
    end
    always @(posedge clk) begin
        otag <= {run, ph};
        for (k = 0; k < N; k = k + 1) begin
            opa[k] <= phl[k][0] ? fwi[k] : fwr[k];
            opb[k] <= (phl[k] == 2'd1 || phl[k] == 2'd2) ? fxi[k] : fxr[k];
            pd1[k] <= phl[k];
            fm[k]  <= opa[k] * opb[k];
            pd2[k] <= pd1[k];
            fp[k]  <= fm[k];
            pd3[k] <= pd2[k];
            fh[k]  <= fp[k];
            fs[k]  <= (pd3[k] == 2'd3) ? (fh[k] - fp[k]) : (fh[k] + fp[k]);
        end
        mtag <= otag;
        ptag <= mtag;
        sv   <= ptag[2] & ptag[0];
        sim  <= ptag[1];
    end

    // adder tree N -> 1 in 6 levels (N <= 64; an odd element passes through),
    // re and im 2 clocks apart, then a saturation stage
    localparam N1 = (N  + 1) / 2, N2 = (N1 + 1) / 2, N3 = (N2 + 1) / 2,
               N4 = (N3 + 1) / 2, N5 = (N4 + 1) / 2, N6 = (N5 + 1) / 2;
    (* shreg_extract = "no" *) reg signed [PW+1:0] a1 [0:N1-1];
    (* shreg_extract = "no" *) reg signed [PW+2:0] a2 [0:N2-1];
    (* shreg_extract = "no" *) reg signed [PW+3:0] a3 [0:N3-1];
    (* shreg_extract = "no" *) reg signed [PW+4:0] a4 [0:N4-1];
    (* shreg_extract = "no" *) reg signed [PW+5:0] a5 [0:N5-1];
    reg signed [PW+6:0] a6 = 0;
    reg signed [OUT_W-1:0] a7 = 0;                 // saturated, own stage (timing)
    reg [7:1] tv = 7'd0, ti = 7'd0;
    reg signed [OUT_W-1:0] yre_h = 0;
    integer ig;
    initial begin
        for (ig = 0; ig < N1; ig = ig + 1) a1[ig] = 0;
        for (ig = 0; ig < N2; ig = ig + 1) a2[ig] = 0;
        for (ig = 0; ig < N3; ig = ig + 1) a3[ig] = 0;
        for (ig = 0; ig < N4; ig = ig + 1) a4[ig] = 0;
        for (ig = 0; ig < N5; ig = ig + 1) a5[ig] = 0;
    end
    function signed [OUT_W-1:0] sat_out;
        input signed [PW+6:0] v;
        reg   signed [PW+6:0] s;
        begin
            s = v >>> 5;                                     // Q20 -> Q15
            if (s[PW+6:OUT_W-1] == {(PW+8-OUT_W){1'b0}} || s[PW+6:OUT_W-1] == {(PW+8-OUT_W){1'b1}})
                sat_out = s[OUT_W-1:0];
            else
                sat_out = s[PW+6] ? {1'b1, {(OUT_W-2){1'b0}}, 1'b1} : {1'b0, {(OUT_W-1){1'b1}}};
        end
    endfunction
    always @(posedge clk) begin
        for (k = 0; k < N1; k = k + 1) a1[k] <= (2*k+1 < N)  ? fs[2*k] + fs[2*k+1] : fs[2*k];
        for (k = 0; k < N2; k = k + 1) a2[k] <= (2*k+1 < N1) ? a1[2*k] + a1[2*k+1] : a1[2*k];
        for (k = 0; k < N3; k = k + 1) a3[k] <= (2*k+1 < N2) ? a2[2*k] + a2[2*k+1] : a2[2*k];
        for (k = 0; k < N4; k = k + 1) a4[k] <= (2*k+1 < N3) ? a3[2*k] + a3[2*k+1] : a3[2*k];
        for (k = 0; k < N5; k = k + 1) a5[k] <= (2*k+1 < N4) ? a4[2*k] + a4[2*k+1] : a4[2*k];
        a6 <= (N5 > 1) ? a5[0] + a5[N5 > 1 ? 1 : 0] : a5[0];
        a7 <= sat_out(a6);
        tv <= {tv[6:1], sv};
        ti <= {ti[6:1], sim};
        out_valid <= 1'b0;
        if (tv[7]) begin
            if (!ti[7]) begin
                yre_h <= a7;
            end else begin
                y_re      <= yre_h;
                y_im      <= a7;
                out_valid <= ~rst;
            end
        end
    end

    // =====================================================================
    //  COVARIANCE ENGINE
    // =====================================================================
    // entry walk: e = 0..NE-1 (i <= j, row by row), lane = e[0], slot = e >> 1.
    // v5: each lane steps its (i, j) two entries per clock (the 298-deep index
    // ROM of L = 16 was 4 LUT levels from the slot counter: -0.6 ns at 4 ns).
    // Past the last entry it parks on (N-1, N-1): the unused lane-1 half of the
    // last slot (NE odd) is never read.
    localparam [IW-1:0] NM1 = N - 1;
    function [2*IW-1:0] step2(input [IW-1:0] i, input [IW-1:0] j);
        begin
            if (j <= N - 3)          step2 = {i, j + 2'd2};
            else if (j == N - 2)     step2 = {i + 1'b1, i + 1'b1};
            else if (i >= N - 2)     step2 = {NM1, NM1};
            else                     step2 = {i + 1'b1, i + 2'd2};
        end
    endfunction

    // snapshot + sweep control
    reg signed [DW-1:0] snr [0:N-1], sni [0:N-1];
    reg        busy = 1'b0, iss = 1'b0;
    reg        slot_last = 1'b0;        // slot == NS-1 next clock (registered compare)
    reg [SW-1:0] slot = {SW{1'b0}};
    reg [IW-1:0] wi0 = 0, wj0 = 0, wi1 = 0, wj1 = 1;   // entries of `slot`, lane 0 / 1
    reg [3:0]  drain = 4'd0;
    reg        wbank = 1'b0;            // bank being accumulated
    reg        pub_bank = 1'b1;         // bank published to the ARM
    reg [15:0] cnt = 16'd0;             // snapshots in the active bank
    reg [15:0] pub_cnt = 16'd0;
    reg [15:0] seq = 16'd0;
    reg        first = 1'b1;            // this sweep overwrites (new window)
    (* max_fanout = 64 *) reg snap_ld = 1'b0;   // load the snapshot (clock after v0)
    reg        full = 1'b0;             // cnt >= 2^KEXP (registered; cnt changes
                                        //   only at a snapshot, >= 78 clocks apart)
    reg        pub_ok = 1'b0;           // full && !LOCK, registered
    reg        lock_d = 1'b0;           // LOCK_ACK = LOCK two clocks late: no
                                        //   publish can follow the acknowledge
    wire [16:0] kval = 17'd1 << kexp_s;
    integer ih;
    initial for (ih = 0; ih < N; ih = ih + 1) begin snr[ih] = 0; sni[ih] = 0; end

    // slot counter and entry walk: held at the start (slot 0, e = 0 / 1)
    // whenever no sweep is issuing, so a new sweep starts from a single
    // registered condition (iss) instead of v0 && !busy && !rst (v5 timing)
    always @(posedge clk) begin
        slot_last <= iss & (slot == NS - 2);   // 0 again at slot NS-1
        if (!iss) begin
            slot <= {SW{1'b0}};
            wi0 <= 0;  wj0 <= 0;  wi1 <= 0;  wj1 <= 1;
        end else begin
            slot <= slot + 1'b1;
            {wi0, wj0} <= step2(wi0, wj0);
            {wi1, wj1} <= step2(wi1, wj1);
        end
    end

    always @(posedge clk) begin
        lock_d   <= lock_s;
        lock_ack <= lock_d;             // after any publish decision (pub_ok lags lock_s by 1)
        full     <= ({1'b0, cnt} >= kval);
        pub_ok   <= full & ~lock_s;
        snap_ld  <= 1'b0;
        if (snap_ld)                    // taps are stable >= 3 clocks after v0
            for (k = 0; k < N; k = k + 1) begin
                snr[k] <= xr[k];  sni[k] <= xi[k];
            end
        if (rst) begin
            busy <= 1'b0;  iss <= 1'b0;  drain <= 4'd0;
            wbank <= 1'b0; pub_bank <= 1'b1; cnt <= 16'd0; pub_cnt <= 16'd0;
            seq <= 16'd0;  first <= 1'b1;
        end else if (!busy) begin
            if (v0) begin                                 // new snapshot
                snap_ld <= 1'b1;
                busy <= 1'b1;  iss <= 1'b1;
                if (pub_ok) begin                         // publish, switch bank
                    pub_bank <= wbank;
                    pub_cnt  <= cnt;
                    seq      <= seq + 16'd1;
                    wbank    <= ~wbank;
                    first    <= 1'b1;
                    cnt      <= 16'd1;
                end else begin
                    first    <= (cnt == 16'd0);
                    cnt      <= (cnt == 16'hFFFF) ? cnt : cnt + 16'd1;
                end
            end
        end else if (iss) begin
            if (slot_last) begin
                iss   <= 1'b0;
                drain <= 4'd9;
            end
        end else begin
            if (drain == 4'd0) busy <= 1'b0;
            else               drain <= drain - 4'd1;
        end
    end

    // per-lane pipeline:  I1 index -> I1b operand group (8:1) -> I2 operand
    //   (group, 5:1) -> I3 M -> I4 P -> I5 combine + memory read -> I6 add
    //   -> write.  v5b: the 34:1 operand mux was one stage (Vivado 2026-11-04,
    //   rx_clk 4 ns: -0.389 ns, 43 endpoints, jx0 fanout 120); it is now
    //   split in two registered halves, the control pipes are one longer.
    localparam NG = (N + 7) / 8;                // operand groups of 8 taps
    (* shreg_extract = "no" *) reg [SW-1:0] q1_slot = 0, q2_slot = 0, q3_slot = 0, q5_slot = 0, q6_slot = 0;
    (* shreg_extract = "no", max_fanout = 32 *) reg [SW-1:0] q4_slot = 0;
    (* shreg_extract = "no", max_fanout = 48 *) reg [7:1] qv = 7'd0;
    (* shreg_extract = "no", max_fanout = 48 *) reg [7:1] qf = 7'd0;
    (* shreg_extract = "no", max_fanout = 32 *) reg [7:1] qb = 7'd0;
    (* max_fanout = 24 *) reg [IW-1:0] ix0 = 0, jx0 = 0, ix1 = 0, jx1 = 0;
    reg [IW-4:0] hx0 = 0, hy0 = 0, hx1 = 0, hy1 = 0;   // group of ix/jx, one stage later
    always @(posedge clk) begin
        // I1: entry indices of this slot
        ix0 <= wi0;  jx0 <= wj0;
        ix1 <= wi1;  jx1 <= wj1;
        hx0 <= ix0[IW-1:3];  hy0 <= jx0[IW-1:3];  hx1 <= ix1[IW-1:3];  hy1 <= jx1[IW-1:3];
        q1_slot <= slot;
        qv <= {qv[6:1], iss};
        qf <= {qf[6:1], first};
        qb <= {qb[6:1], wbank};
        q2_slot <= q1_slot;  q3_slot <= q2_slot;  q4_slot <= q3_slot;  q5_slot <= q4_slot;
        q6_slot <= q5_slot;
    end
    // I1b: in every group of 8 snapshot taps pick tap idx[2:0]. The taps are
    // padded to 8 NG with zeros; each (group, lane, operand) has its own KEEP
    // copy of idx[2:0] (loaded with ix/jx), so every 8:1 mux has a fixed
    // group and a select of fanout 30.
    wire signed [DW-1:0] spr [0:8*NG-1], spi [0:8*NG-1];
    for (g = 0; g < 8*NG; g = g + 1) begin : g_pad
        if (g < N) begin : g_t
            assign spr[g] = snr[g];  assign spi[g] = sni[g];
        end else begin : g_z
            assign spr[g] = {DW{1'b0}};  assign spi[g] = {DW{1'b0}};
        end
    end
    wire signed [DW-1:0] gpr0 [0:NG-1], gpi0 [0:NG-1], gqr0 [0:NG-1], gqi0 [0:NG-1];
    wire signed [DW-1:0] gpr1 [0:NG-1], gpi1 [0:NG-1], gqr1 [0:NG-1], gqi1 [0:NG-1];
    genvar gk;
    for (g = 0; g < NG; g = g + 1) begin : g_op
        wire [8*DW-1:0] br, bi;                       // the 8 taps of this group
        for (gk = 0; gk < 8; gk = gk + 1) begin : g_b
            assign br[gk*DW +: DW] = spr[8*g + gk];
            assign bi[gk*DW +: DW] = spi[8*g + gk];
        end
        (* keep = "true", shreg_extract = "no" *) reg [2:0] sp0 = 3'd0, sq0 = 3'd0, sp1 = 3'd0, sq1 = 3'd1;
        reg signed [DW-1:0] pr0 = 0, pi0 = 0, qr0 = 0, qi0 = 0, pr1 = 0, pi1 = 0, qr1 = 0, qi1 = 0;
        always @(posedge clk) begin
            sp0 <= wi0[2:0];  sq0 <= wj0[2:0];        // = ix0/jx0/ix1/jx1 [2:0], own copy
            sp1 <= wi1[2:0];  sq1 <= wj1[2:0];
            pr0 <= br[sp0*DW +: DW];  pi0 <= bi[sp0*DW +: DW];
            qr0 <= br[sq0*DW +: DW];  qi0 <= bi[sq0*DW +: DW];
            pr1 <= br[sp1*DW +: DW];  pi1 <= bi[sp1*DW +: DW];
            qr1 <= br[sq1*DW +: DW];  qi1 <= bi[sq1*DW +: DW];
        end
        assign gpr0[g] = pr0;  assign gpi0[g] = pi0;  assign gqr0[g] = qr0;  assign gqi0[g] = qi0;
        assign gpr1[g] = pr1;  assign gpi1[g] = pi1;  assign gqr1[g] = qr1;  assign gqi1[g] = qi1;
    end

    // memories (block RAM): one word = {lane1 im, lane1 re, lane0 im, lane0 re}
    // (4 x 48 bits), 1024 deep = {bank, slot}. Two copies written together:
    //   macc : accumulate read port (clk), registered read (output register,
    //          reset = first sweep of a window)
    //   mrd  : read-out port for the ARM (aclk). AXI holds araddr stable from
    //          arvalid, one aclk before gnss_passthrough accepts it, so the
    //          synchronous read is ready at the accept edge.
    localparam MW = 4 * AW;
    (* ram_style = "block" *) reg [MW-1:0] macc [0:1023];
    (* ram_style = "block" *) reg [MW-1:0] mrd  [0:1023];
    reg        [MW-1:0] mq = {MW{1'b0}};
    integer im_;
    initial for (im_ = 0; im_ < 1024; im_ = im_ + 1) begin
        macc[im_] = {MW{1'b0}}; mrd[im_] = {MW{1'b0}};
    end

    // two complex multiply lanes (index l = 0, 1)
    reg signed [DW-1:0] opr [0:1], opi [0:1], oqr [0:1], oqi [0:1];
    reg signed [CP-1:0] m_rr [0:1], m_ii [0:1], m_ir [0:1], m_ri [0:1];
    reg signed [CP-1:0] p_rr [0:1], p_ii [0:1], p_ir [0:1], p_ri [0:1];
    reg signed [CP:0]   pr [0:1], pi_ [0:1];
    reg signed [AW-1:0] rd_r [0:1], rd_i [0:1];
    reg signed [AW-1:0] wd_r [0:1], wd_i [0:1];
    (* shreg_extract = "no", max_fanout = 32 *) reg [SW:0] wa = 0;
    (* shreg_extract = "no", max_fanout = 32 *) reg       we = 1'b0;
    wire       [SW:0]   ra = {qb[4], q4_slot};       // address at I4, data registered at I5
    integer il;
    initial for (il = 0; il < 2; il = il + 1) begin
        opr[il] = 0; opi[il] = 0; oqr[il] = 0; oqi[il] = 0;
        m_rr[il] = 0; m_ii[il] = 0; m_ir[il] = 0; m_ri[il] = 0;
        p_rr[il] = 0; p_ii[il] = 0; p_ir[il] = 0; p_ri[il] = 0;
        pr[il] = 0; pi_[il] = 0; rd_r[il] = 0; rd_i[il] = 0; wd_r[il] = 0; wd_i[il] = 0;
    end

    always @(posedge clk) begin
        // I2: operands (x_i, x_j) of the snapshot: the group picked at I1b
        opr[0] <= gpr0[hx0];  opi[0] <= gpi0[hx0];  oqr[0] <= gqr0[hy0];  oqi[0] <= gqi0[hy0];
        opr[1] <= gpr1[hx1];  opi[1] <= gpi1[hx1];  oqr[1] <= gqr1[hy1];  oqi[1] <= gqi1[hy1];
        for (j = 0; j < 2; j = j + 1) begin
            // I3: M   x_i conj(x_j) = (ir jr + ii ji) + j (ii jr - ir ji)
            m_rr[j] <= opr[j] * oqr[j];  m_ii[j] <= opi[j] * oqi[j];
            m_ir[j] <= opi[j] * oqr[j];  m_ri[j] <= opr[j] * oqi[j];
            // I4: P
            p_rr[j] <= m_rr[j];  p_ii[j] <= m_ii[j];
            p_ir[j] <= m_ir[j];  p_ri[j] <= m_ri[j];
            // I5: combine
            pr[j]  <= p_rr[j] + p_ii[j];
            pi_[j] <= p_ir[j] - p_ri[j];
            // I6: accumulate (the first sweep of a window overwrites)
            wd_r[j] <= rd_r[j] + {{(AW-CP-1){pr[j][CP]}},  pr[j]};
            wd_i[j] <= rd_i[j] + {{(AW-CP-1){pi_[j][CP]}}, pi_[j]};
        end
        // I4: block-RAM read (array latch); I5: output register, reset to 0
        // for the first sweep of a window
        mq <= macc[ra];
        if (qf[5]) begin
            rd_r[0] <= 0;  rd_i[0] <= 0;  rd_r[1] <= 0;  rd_i[1] <= 0;
        end else begin
            rd_r[0] <= mq[AW-1:0];      rd_i[0] <= mq[2*AW-1:AW];
            rd_r[1] <= mq[3*AW-1:2*AW]; rd_i[1] <= mq[4*AW-1:3*AW];
        end
        wa <= {qb[6], q6_slot};
        we <= qv[6];
        // I7: write back (both copies)
        if (we) begin
            macc[wa] <= {wd_i[1], wd_r[1], wd_i[0], wd_r[0]};
            mrd[wa]  <= {wd_i[1], wd_r[1], wd_i[0], wd_r[0]};
        end
    end

    // =====================================================================
    //  status counters to the AXI side (Gray code, 2-flop)
    // =====================================================================
    reg [15:0] novr_g = 0, nload_g = 0;
    always @(posedge clk) begin
        novr_g  <= novr  ^ (novr  >> 1);
        nload_g <= nload ^ (nload >> 1);
    end
    (* ASYNC_REG = "TRUE" *) reg [15:0] ng_m = 0, lg_m = 0;
    (* ASYNC_REG = "TRUE" *) reg [15:0] ng_s = 0, lg_s = 0;
    (* ASYNC_REG = "TRUE" *) reg [1:0]  la_m = 0;
    (* ASYNC_REG = "TRUE" *) reg [1:0]  la_s = 0;
    always @(posedge aclk) begin
        ng_m <= novr_g;   ng_s <= ng_m;
        lg_m <= nload_g;  lg_s <= lg_m;
        la_m <= {active, lock_ack};
        la_s <= la_m;
    end
    function [15:0] g2b;
        input [15:0] gv;
        integer b;
        begin
            g2b[15] = gv[15];
            for (b = 14; b >= 0; b = b - 1) g2b[b] = g2b[b+1] ^ gv[b];
        end
    endfunction

    // =====================================================================
    //  AXI read mux (combinational; the parent registers it)
    //  COV / SEQ / COUNT / bank are read across clocks: they only change at a
    //  publish, which cannot happen while LOCK_ACK is set (the ARM's protocol)
    // =====================================================================
    wire [6:0]  ce   = roff[8:2];                  // entry within the page
    reg  [MW-1:0] mo = {MW{1'b0}};
    // published bank / count / SEQ, synchronised to aclk. They change only at
    // a publish, which cannot happen while LOCK_ACK is set; LOCK_ACK reaches
    // aclk (la_s) after these have settled, so the ARM reads them stable.
    (* ASYNC_REG = "TRUE" *) reg        pb_m = 1'b1, pb_s = 1'b1;
    (* ASYNC_REG = "TRUE" *) reg [15:0] pc_m = 16'd0, pc_s = 16'd0;
    (* ASYNC_REG = "TRUE" *) reg [15:0] sq_m = 16'd0, sq_s = 16'd0;
    always @(posedge aclk) begin
        pb_m <= pub_bank;  pb_s <= pb_m;
        pc_m <= pub_cnt;   pc_s <= pc_m;
        sq_m <= seq;       sq_s <= sq_m;
    end
    always @(posedge aclk) mo <= mrd[{pb_s, page, ce[6:1]}];   // slot = 64 page + ce/2
    wire signed [AW-1:0] cr = ce[0] ? mo[3*AW-1:2*AW] : mo[AW-1:0];
    wire signed [AW-1:0] ci = ce[0] ? mo[4*AW-1:3*AW] : mo[2*AW-1:AW];
    wire [9:0]  wo = roff - OFF_W;
    always @* begin
        rd_data = 32'd0;
        if (roff < OFF_PAGE) begin                   // covariance page
            case (roff[1:0])
                2'd0: rd_data = cr[31:0];
                2'd1: rd_data = ci[31:0];
                2'd2: rd_data = {ci[47:32], cr[47:32]};
                default: rd_data = 32'd0;
            endcase
        end else if (roff >= OFF_W && roff < OFF_W + 2*N) begin
            rd_data = wo[0] ? {{(32-WM){wsh_i[wo >> 1][WM-1]}}, wsh_i[wo >> 1]}
                            : {{(32-WM){wsh_r[wo >> 1][WM-1]}}, wsh_r[wo >> 1]};
        end else begin
            case (roff)
                OFF_CTRL:  rd_data = ctrl;
                OFF_PAGE:  rd_data = {29'd0, page};
                OFF_STAT:  rd_data = {sq_s, 13'd0, la_s[1], pb_s, la_s[0]};
                OFF_COUNT: rd_data = {16'd0, pc_s};
                OFF_NOVR:  rd_data = {16'd0, g2b(ng_s)};
                OFF_INFO:  rd_data = {8'd3, REF[7:0], L[7:0], 8'h53};
                OFF_WLOAD: rd_data = {31'd0, wpend};
                OFF_NLOAD: rd_data = {16'd0, g2b(lg_s)};
                default:   rd_data = 32'd0;
            endcase
        end
    end

endmodule
