// ============================================================================
//  sfap_pi.cpp  --  space-frequency adaptive power inversion core (SFAP-PI v6.0, N = 256)
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2 (2021.1+)
//
//  WHAT IT DOES
//    Splits RX1 and RX2 into N = 256 frequency bins (windowed FFT, 50 %
//    overlap), applies an independent two-antenna power-inversion weight in
//    every bin, and rebuilds the time signal (inverse FFT, weighted overlap-
//    add). Per bin k of every frame:
//        Y_k = X1_k - b_k X2n_k - c1_k conj(X1_{-k}) - c2_k conj(X2n_{-k})
//    The RX1 coefficient is fixed at 1 (power inversion), so the output can
//    never be turned down as a whole. b_k follows the RX2/RX1 ratio of the
//    jammer across the band (cables, antenna spacing, the two analog
//    filters, multipath): SFAP does with one weight per bin what the v4
//    space-time filter did with 16 taps per antenna. c1_k, c2_k are the
//    image taps: the AD9361's I/Q imbalance leaves a mirror copy of the
//    jammer, ~50 dB down, at bin -k, which a linear weight cannot remove.
//
//  STRUCTURE (one sample pair in, one sample out, every call, II = 1)
//
//    x1 ----------------------------------------------- bypass delay --+
//    x2 -> g2 -> x2n                                                   |
//              |                                                       |
//    chain A (frames start at n = 0 mod 256):                           |
//      window -> FFT(x1), FFT(x2n) -> frame buffer -> weights -> IFFT  |
//              -> window ---------------------------------------+      |
//    chain B (frames start at n = 128 mod 256): the same          (+) -> mux -> 2^s -> y
//              -> window ---------------------------------------+
//
//    * Window: square root of the periodic Hann, sin(pi n / 256), on both
//      analysis and synthesis. w[n]^2 + w[n+128]^2 = 1, so with all weights
//      zero the two chains add up to x1 exactly (to the 16-bit window
//      rounding, ~1e-5).
//    * FFT: radix-2 single-path delay-feedback (R2SDF), decimation in
//      frequency, natural order in, bit-reversed bins out, latency N-1.
//      Unscaled. The bins grow by up to 8 bits; with the window's gain
//      (sum w = 0.64 N) a full-scale x2n bin stays below 1.5e7 < 2^24, so
//      25-bit data holds them.
//    * Frame buffer: one frame of bins per channel and chain, so bin k and
//      its image partner -k of the SAME frame are both available. Adds N.
//    * IFFT: R2SDF decimation in time, bit-reversed in, natural order out,
//      latency N-1, scaled by 1/2 per stage (1/N overall).
//    * Both chains run on the same sample counter, so their outputs are
//      aligned without any extra buffering: the chain-A output at call t
//      and the chain-B output at call t belong to the same input sample
//      t - SFAP_LAT.
//    * The twiddle multiplies sit OUTSIDE the delay-feedback loops (after
//      the loop in the DIF stages, before it in the DIT stages), so every
//      loop-carried path is one add -- that is what makes II = 1 at
//      100 MHz straightforward.
//
//  BYPASS (cfg[0] = 1): y[n] = 2^s x1[n - SFAP_LAT] exactly, from a plain
//    delay line of the same length as the filter path, so switching the
//    filter in and out never moves the satellites' code phase.
//
//  LATENCY: SFAP_LAT = 3N - 2 = 766 samples in both modes (12.5 us at 61.44
//    MS/s, 24.9 us at 30.72): a common delay on every satellite, absorbed by
//    the receiver's clock bias. (v4 space-time: 8 samples.)
//
//  WEIGHT UPDATES: a commit switches all 3 x 256 weights in one clock
//    (gnss_passthrough flips its double-buffered weight RAM); the
//    frame that is being weighted at that moment uses old weights for some
//    bins (bit-reversed order) and new ones for the rest. That one frame
//    (256 samples: 4.2 us at 61.44 MS/s, 8.3 us at 30.72) is a transient the
//    synthesis window smooths; the next frames are clean.
//
//  FIXED POINT
//    x1      : 12-bit, Q0 -> Q4 (16 bits)
//    x2n     : g2 x2, Q13 x Q0 -> Q4, saturated to 17 bits (2x RX full scale)
//    window  : unsigned Q16 (1.0 = 65536), rounded back to Q4
//    bins    : 25-bit signed, Q4, unscaled FFT
//    weights : 16-bit signed Q3.12; Y saturated to 24 bits
//    IFFT    : 25-bit, Q4, /2 per stage
//    output  : chain A + chain B (Q4), x 2^s, rounded to Q0, saturated to 16
//
//  TIMING / RESOURCES (estimate, xc7z020): 100 MHz sys_cpu_clk, II = 1, i.e.
//    up to 100 MS/s. ~160 DSP48 (4 forward + 2 inverse FFTs with 5 table
//    twiddle stages each -- the W8 stage is shift-and-add --, 6 complex weight
//    products, windows, g2). SDF delay lines of 64 and 128 samples are small
//    RAMs, the shorter ones register chains (SRL); frame buffers and the
//    bypass line in block RAM; the weights come in through memory ports.
// ============================================================================
#include "sfap_pi.h"
#include "sfap_tables.h"

typedef ap_int<25>             d_t;     // FFT / IFFT data, Q4
typedef ap_int<18>             c_t;     // twiddle / window, Q16
typedef ap_int<16>             w16_t;   // weights, Q3.12
typedef ap_uint<SFAP_LOG2N + 1> t_t;    // sample counter, mod 2N
typedef ap_uint<SFAP_LOG2N>    k_t;     // bin / frame position

static const int N = SFAP_N;

// ---------------------------------------------------------------------------
//  helpers
// ---------------------------------------------------------------------------
static d_t sat24(ap_int<48> v) {
#pragma HLS INLINE
    const ap_int<48> hi = (ap_int<48>(1) << 23) - 1, lo = -(ap_int<48>(1) << 23);
    if (v > hi) return (d_t)hi;
    if (v < lo) return (d_t)lo;
    return (d_t)v;
}

static ap_int<16> sat16(ap_int<34> v) {
#pragma HLS INLINE
    if (v > 32767)  return 32767;
    if (v < -32768) return -32768;
    return (ap_int<16>)v;
}

// (a + j b) * (wr + j wi), Q16 twiddle, rounded to nearest.
static void cmul_tw(d_t ar, d_t ai, c_t wr, c_t wi, d_t &yr, d_t &yi) {
#pragma HLS INLINE
    ap_int<45> pr = ap_int<45>(ar * wr) - ap_int<45>(ai * wi);
    ap_int<45> pi = ap_int<45>(ar * wi) + ap_int<45>(ai * wr);
    yr = (d_t)((pr + 32768) >> 16);
    yi = (d_t)((pi + 32768) >> 16);
}

// v x 46341 (= round(65536 / sqrt 2), the Q16 table value of W_8) with
// shifts and adds: 46341 = 2^15 + 2^13 + 2^12 + 2^10 + 2^8 + 2^2 + 1.
static ap_int<45> k46341(ap_int<27> v) {
#pragma HLS INLINE
    const ap_int<45> x = v;
    return (x << 15) + (x << 13) + (x << 12) + (x << 10) + (x << 8) + (x << 2) + x;
}

// x * W_8^q (inv = 0) or x * conj(W_8^q) (inv = 1), q = 0..3: bit-exactly
// what cmul_tw does with the Q16 table values (65536, 46341, 0, -46341), but
// without multipliers (saves 4 DSP48 per FFT).
static void cmul_w8(ap_uint<2> q, bool inv, d_t xr, d_t xi, d_t &yr, d_t &yi) {
#pragma HLS INLINE
    const ap_int<45> S = k46341(ap_int<27>(xr) + xi);       // 46341 (xr + xi)
    const ap_int<45> D = k46341(ap_int<27>(xr) - xi);       // 46341 (xr - xi)
    ap_int<45> pr, pi;
    if (!inv) {          // W8^1 = (1 - j)/sqrt2, W8^3 = (-1 - j)/sqrt2
        pr = (q == 1) ? S : (ap_int<45>)(-D);
        pi = (q == 1) ? (ap_int<45>)(-D) : (ap_int<45>)(-S);
    } else {             // conj: (1 + j)/sqrt2, (-1 + j)/sqrt2
        pr = (q == 1) ? D : (ap_int<45>)(-S);
        pi = (q == 1) ? S : D;
    }
    const d_t rr = (d_t)((pr + 32768) >> 16), ri = (d_t)((pi + 32768) >> 16);
    if (q == 0)      { yr = xr; yi = xi; }                    // x 1
    else if (q == 2) { if (!inv) { yr = xi;  yi = -xr; }       // x (-j)
                       else      { yr = -xi; yi = xr;  } }     // x (+j)
    else             { yr = rr; yi = ri; }
}

static k_t bitrev(k_t v) {
#pragma HLS INLINE
    k_t r;
    for (int b = 0; b < SFAP_LOG2N; b++) {
#pragma HLS UNROLL
        r[b] = v[SFAP_LOG2N - 1 - b];
    }
    return r;
}

// ---------------------------------------------------------------------------
//  SDF delay lines (in dif / dit below): a stage of span L delays by L
//  samples. Short lines are a register chain (SRL in the FPGA); lines of
//  SFAP_RAM_DELAY or more a circular buffer in a small RAM -- the same
//  values, bit for bit. The next access to a RAM address is L >= 64 calls
//  later, far beyond the pipeline depth, so the inter-call dependence HLS
//  would otherwise assume is false.
// ---------------------------------------------------------------------------
//  R2SDF decimation-in-frequency stage, span L (forward FFT).
//  Block of 2L inputs, c = t mod 2L:
//    c <  L : the input goes into the delay line; the output is the
//             previous block's difference (a - b) x W_2L^c.
//    c >= L : a = the input of L samples ago, b = this input;
//             output a + b, push a - b.
//  ID makes the delay line unique to one FFT instance.
// ---------------------------------------------------------------------------
template <int ID, int L>
static void dif(t_t t, d_t xr, d_t xi, d_t &yr, d_t &yi) {
#pragma HLS INLINE
    static d_t dr[L], di[L];
#pragma HLS ARRAY_PARTITION variable=dr complete
#pragma HLS ARRAY_PARTITION variable=di complete
    static ap_uint<50> dm[L];
#pragma HLS BIND_STORAGE variable=dm type=ram_s2p impl=lutram
#pragma HLS DEPENDENCE variable=dm inter false
    static ap_uint<SFAP_LOG2N> dp = 0;
    const bool ram = (L >= SFAP_RAM_DELAY);

    const k_t c = (k_t)(t & (2 * L - 1));
    const bool ph2 = (c >= L);
    d_t ar, ai;
    if (ram) { const ap_uint<50> v = dm[dp & (L - 1)]; ar = (d_t)v(24, 0); ai = (d_t)v(49, 25); }
    else     { ar = dr[L - 1]; ai = di[L - 1]; }
    const d_t sr = ar + xr, si = ai + xi;
    const d_t pr = ph2 ? (d_t)(ar - xr) : xr;
    const d_t pi = ph2 ? (d_t)(ai - xi) : xi;
    if (ram) {
        ap_uint<50> w;
        w(24, 0) = (ap_uint<25>)pr;
        w(49, 25) = (ap_uint<25>)pi;
        dm[dp & (L - 1)] = w;
        dp = (dp + 1) & (L - 1);
    } else {
        for (int k = L - 1; k > 0; k--) {
#pragma HLS UNROLL
            dr[k] = dr[k - 1];
            di[k] = di[k - 1];
        }
        dr[0] = pr;
        di[0] = pi;
    }

    d_t tr, ti;                                  // (a - b) x W_2L^c
    if (L == 1) {
        tr = ar; ti = ai;
    } else if (L == 2) {
        if (c[0]) { tr = ai; ti = -ar; }         // x (-j)
        else      { tr = ar; ti = ai; }
    } else if (L == 4) {
        cmul_w8((ap_uint<2>)(c & 3), false, ar, ai, tr, ti);
    } else {
        const int m = (int)(c & (L - 1)) * (N / (2 * L));
        cmul_tw(ar, ai, (c_t)SFAP_TW_RE[m], (c_t)SFAP_TW_IM[m], tr, ti);
    }
    yr = ph2 ? sr : tr;
    yi = ph2 ? si : ti;
}

// ---------------------------------------------------------------------------
//  R2SDF decimation-in-time stage, span L (inverse FFT, scaled by 1/2).
//  Block of 2L inputs (bit-reversed order), c = u mod 2L:
//    c <  L : the input (a) goes into the delay line; the output is the
//             previous block's (a - W b) / 2.
//    c >= L : b = this input x conj(W_2L^(c-L)), a = the input of L ago;
//             output (a + W b) / 2, push (a - W b) / 2.
// ---------------------------------------------------------------------------
template <int ID, int L>
static void dit(t_t u, d_t xr, d_t xi, d_t &yr, d_t &yi) {
#pragma HLS INLINE
    static d_t dr[L], di[L];
#pragma HLS ARRAY_PARTITION variable=dr complete
#pragma HLS ARRAY_PARTITION variable=di complete
    static ap_uint<50> dm[L];
#pragma HLS BIND_STORAGE variable=dm type=ram_s2p impl=lutram
#pragma HLS DEPENDENCE variable=dm inter false
    static ap_uint<SFAP_LOG2N> dp = 0;
    const bool ram = (L >= SFAP_RAM_DELAY);

    const k_t c = (k_t)(u & (2 * L - 1));
    const bool ph2 = (c >= L);
    d_t ar, ai;
    if (ram) { const ap_uint<50> v = dm[dp & (L - 1)]; ar = (d_t)v(24, 0); ai = (d_t)v(49, 25); }
    else     { ar = dr[L - 1]; ai = di[L - 1]; }

    d_t br, bi;                                  // x conj(W_2L^(c-L))
    if (L == 1) {
        br = xr; bi = xi;
    } else if (L == 2) {
        if (c[0]) { br = -xi; bi = xr; }         // x (+j)
        else      { br = xr;  bi = xi; }
    } else if (L == 4) {
        cmul_w8((ap_uint<2>)(c & 3), true, xr, xi, br, bi);
    } else {
        const int m = (int)(c & (L - 1)) * (N / (2 * L));
        cmul_tw(xr, xi, (c_t)SFAP_TW_RE[m], (c_t)(-SFAP_TW_IM[m]), br, bi);
    }
    const ap_int<26> sr = ap_int<26>(ar) + br, si = ap_int<26>(ai) + bi;
    const ap_int<26> qr = ap_int<26>(ar) - br, qi = ap_int<26>(ai) - bi;
    const d_t pr = ph2 ? (d_t)((qr + 1) >> 1) : xr;
    const d_t pi = ph2 ? (d_t)((qi + 1) >> 1) : xi;
    if (ram) {
        ap_uint<50> w;
        w(24, 0) = (ap_uint<25>)pr;
        w(49, 25) = (ap_uint<25>)pi;
        dm[dp & (L - 1)] = w;
        dp = (dp + 1) & (L - 1);
    } else {
        for (int k = L - 1; k > 0; k--) {
#pragma HLS UNROLL
            dr[k] = dr[k - 1];
            di[k] = di[k - 1];
        }
        dr[0] = pr;
        di[0] = pi;
    }
    yr = ph2 ? (d_t)((sr + 1) >> 1) : ar;
    yi = ph2 ? (d_t)((si + 1) >> 1) : ai;
}

// 256-point forward FFT, natural order in, bit-reversed out, latency N-1.
// Stage s sees c = t mod 2L_s (the delays of the stages before it are a
// multiple of 2L_s).
template <int ID>
static void fftN(t_t t, d_t xr, d_t xi, d_t &yr, d_t &yi) {
#pragma HLS INLINE
    d_t r1, i1, r2, i2, r3, i3, r4, i4, r5, i5, r6, i6, r7, i7;
    dif<ID, 128>(t, xr, xi, r1, i1);
    dif<ID,  64>(t, r1, i1, r2, i2);
    dif<ID,  32>(t, r2, i2, r3, i3);
    dif<ID,  16>(t, r3, i3, r4, i4);
    dif<ID,   8>(t, r4, i4, r5, i5);
    dif<ID,   4>(t, r5, i5, r6, i6);
    dif<ID,   2>(t, r6, i6, r7, i7);
    dif<ID,   1>(t, r7, i7, yr, yi);
}

// 256-point inverse FFT (x 1/N), bit-reversed in, natural order out, latency
// N-1. Its input frames start at t = N-1 mod N (the forward FFT's output
// alignment), and stage j is fed L_j - 1 samples later: c = (t + 2 - L) mod 2L.
template <int ID>
static void ifftN(t_t t, d_t xr, d_t xi, d_t &yr, d_t &yi) {
#pragma HLS INLINE
    d_t r1, i1, r2, i2, r3, i3, r4, i4, r5, i5, r6, i6, r7, i7;
    dit<ID,   1>((t_t)(t + 2 -   1), xr, xi, r1, i1);
    dit<ID,   2>((t_t)(t + 2 -   2), r1, i1, r2, i2);
    dit<ID,   4>((t_t)(t + 2 -   4), r2, i2, r3, i3);
    dit<ID,   8>((t_t)(t + 2 -   8), r3, i3, r4, i4);
    dit<ID,  16>((t_t)(t + 2 -  16), r4, i4, r5, i5);
    dit<ID,  32>((t_t)(t + 2 -  32), r5, i5, r6, i6);
    dit<ID,  64>((t_t)(t + 2 -  64), r6, i6, r7, i7);
    dit<ID, 128>((t_t)(t + 2 - 128), r7, i7, yr, yi);
}

// ---------------------------------------------------------------------------
//  One-frame bin buffer. Writes this frame's bin k into half wh, returns the
//  PREVIOUS frame's bin k and bin -k (its image partner) from the other half.
//  Two copies (one per read port), packed {imag, real}.
//  A bin written in one frame is read at the earliest N/2 + 1 samples later
//  (bin N-1 at position N-1, its partner -(N-1) = 1 read at position N/2 of
//  the next frame), far beyond the pipeline depth, and never by the same
//  call: the dependences HLS would otherwise assume are false.
// ---------------------------------------------------------------------------
template <int ID>
static void fbuf(ap_uint<1> wh, k_t k, d_t xr, d_t xi,
                 d_t &dr, d_t &di, d_t &mr, d_t &mi) {
#pragma HLS INLINE
    static ap_uint<50> bd[2 * N], bm[2 * N];
#pragma HLS DEPENDENCE variable=bd inter false
#pragma HLS DEPENDENCE variable=bm inter false
#pragma HLS DEPENDENCE variable=bd intra false
#pragma HLS DEPENDENCE variable=bm intra false
    const k_t km = (k_t)(0 - k);                 // -k mod N
    ap_uint<SFAP_LOG2N + 1> ra = 0, rm = 0, wa = 0;
    ra(SFAP_LOG2N - 1, 0) = k;  ra[SFAP_LOG2N] = ~wh;
    rm(SFAP_LOG2N - 1, 0) = km; rm[SFAP_LOG2N] = ~wh;
    wa(SFAP_LOG2N - 1, 0) = k;  wa[SFAP_LOG2N] = wh;
    const ap_uint<50> vd = bd[ra], vm = bm[rm];
    dr = (d_t)vd(24, 0);  di = (d_t)vd(49, 25);
    mr = (d_t)vm(24, 0);  mi = (d_t)vm(49, 25);
    ap_uint<50> w;
    w(24, 0) = (ap_uint<25>)xr;
    w(49, 25) = (ap_uint<25>)xi;
    bd[wa] = w;
    bm[wa] = w;
}

static d_t win(d_t x, c_t w) {
#pragma HLS INLINE
    return (d_t)((ap_int<44>(x * w) + 32768) >> 16);
}

// ---------------------------------------------------------------------------
//  One frame chain: analysis window, two FFTs, frame buffer, per-bin
//  weights, IFFT, synthesis window. tc = this chain's sample counter.
//  The weights are read from the memory ports (one read per array and chain
//  per sample: chain A uses port 0, chain B port 1).
// ---------------------------------------------------------------------------
template <int ID>
static void chain(t_t tc, d_t x1r, d_t x1i, d_t x2r, d_t x2i,
                  const ap_uint<32> WB[N], const ap_uint<32> WC1[N],
                  const ap_uint<32> WC2[N], d_t &zr, d_t &zi) {
#pragma HLS INLINE
    // analysis window
    const c_t wa = (c_t)SFAP_WIN[(int)(tc & (N - 1))];
    d_t a1r = win(x1r, wa), a1i = win(x1i, wa);
    d_t a2r = win(x2r, wa), a2i = win(x2i, wa);

    // forward FFTs
    d_t X1r, X1i, X2r, X2i;
    fftN<ID + 0>(tc, a1r, a1i, X1r, X1i);
    fftN<ID + 1>(tc, a2r, a2i, X2r, X2i);

    // frame position of the FFT output, its bin and the frame parity
    const t_t   to = tc + 1;
    const k_t   o  = (k_t)(to & (N - 1));
    const k_t   k  = bitrev(o);
    const ap_uint<1> wh = to[SFAP_LOG2N];

    d_t D1r, D1i, M1r, M1i, D2r, D2i, M2r, M2i;
    fbuf<ID + 0>(wh, k, X1r, X1i, D1r, D1i, M1r, M1i);
    fbuf<ID + 1>(wh, k, X2r, X2i, D2r, D2i, M2r, M2i);

    // per-bin power inversion (previous frame, bin k)
    const ap_uint<32> vb = WB[k], v1 = WC1[k], v2 = WC2[k];
    const w16_t br = (w16_t)vb(15, 0), bi = (w16_t)vb(31, 16);
    const w16_t c1r = (w16_t)v1(15, 0), c1i = (w16_t)v1(31, 16);
    const w16_t c2r = (w16_t)v2(15, 0), c2i = (w16_t)v2(31, 16);
    // b X2 + c1 conj(M1) + c2 conj(M2), Q4 x Q12 = Q16
    ap_int<48> pr = ap_int<48>(br * D2r) - ap_int<48>(bi * D2i)
                  + ap_int<48>(c1r * M1r) + ap_int<48>(c1i * M1i)
                  + ap_int<48>(c2r * M2r) + ap_int<48>(c2i * M2i);
    ap_int<48> pi = ap_int<48>(br * D2i) + ap_int<48>(bi * D2r)
                  + ap_int<48>(c1i * M1r) - ap_int<48>(c1r * M1i)
                  + ap_int<48>(c2i * M2r) - ap_int<48>(c2r * M2i);
    const ap_int<48> half = ap_int<48>(1) << (SFAP_WFRAC - 1);
    d_t Yr = sat24(ap_int<48>(D1r) - ((pr + half) >> SFAP_WFRAC));
    d_t Yi = sat24(ap_int<48>(D1i) - ((pi + half) >> SFAP_WFRAC));

    // inverse FFT and synthesis window
    d_t yr, yi;
    ifftN<ID + 2>(tc, Yr, Yi, yr, yi);
    const c_t ws = (c_t)SFAP_WIN[(int)((tc + 2) & (N - 1))];
    zr = win(yr, ws);
    zi = win(yi, ws);
}

// ---------------------------------------------------------------------------
//  top
// ---------------------------------------------------------------------------
void sfap_pi(hls::stream<ap_uint<64> > &s_axis_x,
             hls::stream<ap_uint<32> > &m_axis_y,
             const ap_uint<32> wb[SFAP_N],
             const ap_uint<32> wc1[SFAP_N],
             const ap_uint<32> wc2[SFAP_N],
             ap_uint<32>   cfg,
             ap_uint<32>   nrm) {
#pragma HLS INTERFACE axis port=s_axis_x
#pragma HLS INTERFACE axis port=m_axis_y
#pragma HLS INTERFACE ap_memory port=wb  storage_type=rom_2p
#pragma HLS INTERFACE ap_memory port=wc1 storage_type=rom_2p
#pragma HLS INTERFACE ap_memory port=wc2 storage_type=rom_2p
#pragma HLS INTERFACE ap_none port=cfg
#pragma HLS INTERFACE ap_none port=nrm
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=1

    static t_t t = 0;                              // sample counter mod 2N
    static ap_uint<10> bptr = 0;                   // bypass delay line
    static ap_uint<24> bdl[1024];
#pragma HLS DEPENDENCE variable=bdl inter false
#pragma HLS DEPENDENCE variable=bdl intra false

    if (s_axis_x.empty()) return;
    const ap_uint<64> in = s_axis_x.read();
    const ap_int<12> x1r = (ap_int<12>)in(11, 0),  x1i = (ap_int<12>)in(27, 16);
    const ap_int<12> x2r = (ap_int<12>)in(43, 32), x2i = (ap_int<12>)in(59, 48);

    // ---- input scaling: x1 -> Q4, x2n = g2 x2 -> Q4 (17-bit saturation) -----
    const d_t q1r = d_t(x1r) << SFAP_DFRAC, q1i = d_t(x1i) << SFAP_DFRAC;
    const ap_int<16> g2r = (ap_int<16>)nrm(15, 0), g2i = (ap_int<16>)nrm(31, 16);
    const ap_int<30> nr = ap_int<30>(g2r * x2r) - ap_int<30>(g2i * x2i);
    const ap_int<30> ni = ap_int<30>(g2r * x2i) + ap_int<30>(g2i * x2r);
    const int NS = SFAP_GFRAC - SFAP_DFRAC;                        // 9
    ap_int<30> vr = (nr + (1 << (NS - 1))) >> NS, vi = (ni + (1 << (NS - 1))) >> NS;
    if (vr > 65535) vr = 65535; if (vr < -65536) vr = -65536;
    if (vi > 65535) vi = 65535; if (vi < -65536) vi = -65536;
    const d_t q2r = (d_t)vr, q2i = (d_t)vi;

    // ---- the two frame chains ---------------------------------------------------
    d_t zar, zai, zbr, zbi;
    chain<0>(t, q1r, q1i, q2r, q2i, wb, wc1, wc2, zar, zai);
    chain<4>((t_t)(t + SFAP_HOP), q1r, q1i, q2r, q2i, wb, wc1, wc2, zbr, zbi);
    const ap_int<26> sr = ap_int<26>(zar) + zbr;                   // Q4
    const ap_int<26> si = ap_int<26>(zai) + zbi;

    // ---- bypass delay line: x1[n - SFAP_LAT] ------------------------------------
    const ap_uint<24> bv = bdl[(ap_uint<10>)(bptr - SFAP_LAT)];
    ap_uint<24> bw;
    bw(11, 0) = (ap_uint<12>)x1r;
    bw(23, 12) = (ap_uint<12>)x1i;
    bdl[bptr] = bw;
    bptr++;
    const ap_int<12> b1r = (ap_int<12>)bv(11, 0), b1i = (ap_int<12>)bv(23, 12);

    // ---- output: bypass or filter, make-up shift, rounding ----------------------
    ap_uint<3> s = cfg(10, 8);
    if (s > SFAP_MAXSH) s = SFAP_MAXSH;
    ap_int<16> yr, yi;
    if (cfg[0]) {
        yr = sat16(ap_int<34>(b1r) << s);
        yi = sat16(ap_int<34>(b1i) << s);
    } else {
        const ap_int<34> h = 1 << (SFAP_DFRAC - 1);
        yr = sat16(((ap_int<34>(sr) << s) + h) >> SFAP_DFRAC);
        yi = sat16(((ap_int<34>(si) << s) + h) >> SFAP_DFRAC);
    }
    ap_uint<32> out;
    out(15, 0)  = (ap_uint<16>)yr;
    out(31, 16) = (ap_uint<16>)yi;
    m_axis_y.write(out);
    t++;
}
