// ============================================================================
//  stap_pi.cpp  --  space-time power-inversion filter, weights from the ARM
//                   (antsdr-fw-patch crpa16 / crpa17, stap_pi 1.0)
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2
//
//  PIPELINE (one sample pair per clock, sys_cpu_clk 100 MHz, II = 1)
//    1. INPUT NORMALISATION   x2n = g2 * x2 (g2 real, Q3.13, from the firmware:
//       ratio of the two receivers' noise floors). x2n keeps 2 fractional bits.
//    2. SPACE-TIME FILTER     ST_NT taps on RX1, ST_NT on the normalised RX2,
//       ST_NIMG image taps on each (widely linear, AD9361 I/Q mirror):
//         y[n] = x1[n-R] - 2^-ws ( sum_{k != R} a_k x1[n-k] + sum_k b_k x2n[n-k]
//                        + sum_d c1_d conj(x1[n-R-d]) + sum_d c2_d conj(x2n[n-R-d]) )
//       R = ST_REF, d = -1, 0, 1. The RX1 tap at delay R is the reference, fixed
//       at 1 (power inversion: the output can never be turned down as a whole).
//    3. BYPASS                cfg[0] = 1: y = x1[n-R] exactly (all products forced
//       to 0), the same R-sample delay as the filter.
//    4. OUTPUT MAKE-UP GAIN   y <<= s, s = cfg[10:8] (0..7, 6 dB each), before the
//       rounding (the accumulator has 14 fractional bits).
//    5. FINE GAIN             gain[15:0], unsigned Q2.14, after the shift (2 DSP48);
//       1.0 gives the shifted output bit for bit.
//
//  WEIGHTS: calculated by the ARM (crpa16/17: NLMS on the captures,
//    gnss_aj_math.c aj_nlms_solve), written to gnss_passthrough's AJ_W / AJ_WC /
//    AJ_WCX shadow registers and committed in one clock. Signed Q3.12, held as
//    w * 2^ws (ws = cfg[14:12], finer steps for a very strong jammer). This core
//    has no feedback at all apart from the tap shift registers, the type of core
//    that met timing in crpa1-crpa4 (no weight smoothing, no NLMS loop: those
//    made the crpa10-12 core miss timing).
//
//  NUMBERS: x1 12 bits; x2n Q2 in 18 bits; weights Q3.12 (16 bits); products
//    accumulated in Q14, 48 bits. 3-multiplier complex products (cmul3).
//    DSP48: 3 per weight (crpa16, 8 + 8 + 3 + 3 image: 21 weights, 63; crpa17,
//    6 + 6: 17 weights, 51), g2 2, fine gain 2.
// ============================================================================
#include "stap_pi.h"

typedef ap_int<12> s12_t;
typedef ap_int<18> s18_t;            // normalised RX2, Q2
typedef ap_int<16> w16_t;
typedef ap_int<48> acc_t;            // Q14

static ap_int<16> sat16(acc_t v) {
#pragma HLS INLINE
    if (v > 32767)  return 32767;
    if (v < -32768) return -32768;
    return (ap_int<16>)v;
}

// (wr + j wi)(xr + j xi) with 3 multipliers:
//   k1 = wr (xr + xi), k2 = xr (wi - wr), k3 = xi (wr + wi)
//   re = k1 - k3, im = k1 + k2
template <int XW>
static void cmul3(w16_t wr, w16_t wi, ap_int<XW> xr, ap_int<XW> xi,
                  ap_int<XW + 19> &pr, ap_int<XW + 19> &pi) {
#pragma HLS INLINE
    ap_int<XW + 1> xs = ap_int<XW + 1>(xr) + ap_int<XW + 1>(xi);
    ap_int<17> d1 = ap_int<17>(wi) - ap_int<17>(wr);
    ap_int<17> d2 = ap_int<17>(wr) + ap_int<17>(wi);
    ap_int<XW + 18> k1 = wr * xs;
    ap_int<XW + 18> k2 = xr * d1;
    ap_int<XW + 18> k3 = xi * d2;
    pr = ap_int<XW + 19>(k1) - ap_int<XW + 19>(k3);
    pi = ap_int<XW + 19>(k1) + ap_int<XW + 19>(k2);
}

void stap_pi(hls::stream<ap_uint<64> > &s_axis_x,
             hls::stream<ap_uint<32> > &m_axis_y,
             ap_uint<32 * ST_NST>      wts,
             ap_uint<32 * 2 * ST_NIMG> wcj,
             ap_uint<32>   cfg,
             ap_uint<32>   nrm,
             ap_uint<32>   gain) {
#pragma HLS INTERFACE axis port=s_axis_x
#pragma HLS INTERFACE axis port=m_axis_y
#pragma HLS INTERFACE ap_none port=wts
#pragma HLS INTERFACE ap_none port=wcj
#pragma HLS INTERFACE ap_none port=cfg
#pragma HLS INTERFACE ap_none port=nrm
#pragma HLS INTERFACE ap_none port=gain
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=1

    static s12_t h1r[ST_NT], h1i[ST_NT];
    static s18_t h2r[ST_NT], h2i[ST_NT];
#pragma HLS ARRAY_PARTITION variable=h1r complete
#pragma HLS ARRAY_PARTITION variable=h1i complete
#pragma HLS ARRAY_PARTITION variable=h2r complete
#pragma HLS ARRAY_PARTITION variable=h2i complete
#pragma HLS RESET variable=h1r
#pragma HLS RESET variable=h1i
#pragma HLS RESET variable=h2r
#pragma HLS RESET variable=h2i

    if (s_axis_x.empty()) return;
    ap_uint<64> in = s_axis_x.read();
    s12_t x1r = (s12_t)in(11, 0),  x1i = (s12_t)in(27, 16);
    s12_t x2r = (s12_t)in(43, 32), x2i = (s12_t)in(59, 48);

    // ---- 1. input normalisation: x2n = g2 x2 (Q13 x Q0 -> Q2) ---------------
    // |g2r x2r| <= 32768 * 2048 = 2^26, after the shift by 11 within +-2^15.
    ap_int<16> g2r = (ap_int<16>)nrm(15, 0);
    ap_int<32> nr = ap_int<32>(g2r * x2r);
    ap_int<32> ni = ap_int<32>(g2r * x2i);
    const ap_int<32> nhalf = ap_int<32>(1) << (ST_GFRAC - ST_XFRAC - 1);
    s18_t x2nr = (s18_t)((nr + nhalf) >> (ST_GFRAC - ST_XFRAC));
    s18_t x2ni = (s18_t)((ni + nhalf) >> (ST_GFRAC - ST_XFRAC));

    for (int k = ST_NT - 1; k > 0; k--) {
#pragma HLS UNROLL
        h1r[k] = h1r[k - 1]; h1i[k] = h1i[k - 1];
        h2r[k] = h2r[k - 1]; h2i[k] = h2i[k - 1];
    }
    h1r[0] = x1r;  h1i[0] = x1i;
    h2r[0] = x2nr; h2i[0] = x2ni;

    // ---- control ---------------------------------------------------------------
    const bool byp = cfg[0];
    ap_uint<3> ws = cfg(14, 12);
    if (ws > ST_MAXWS) ws = ST_MAXWS;
    ap_uint<3> s = cfg(10, 8);
    if (s > ST_MAXSH) s = ST_MAXSH;

    // ---- the committed weights (bypass: all 0) ------------------------------------
    w16_t wtr[ST_NW], wti[ST_NW];
#pragma HLS ARRAY_PARTITION variable=wtr complete
#pragma HLS ARRAY_PARTITION variable=wti complete
    for (int j = 0; j < ST_NW; j++) {
#pragma HLS UNROLL
        w16_t tr, ti;
        if (j < ST_NST) { tr = (w16_t)wts(32 * j + 15, 32 * j); ti = (w16_t)wts(32 * j + 31, 32 * j + 16); }
        else { const int c = j - ST_NST; tr = (w16_t)wcj(32 * c + 15, 32 * c); ti = (w16_t)wcj(32 * c + 31, 32 * c + 16); }
        if (byp || j == ST_REF) { tr = 0; ti = 0; }          // word ST_REF (the reference) is never used
        wtr[j] = tr; wti[j] = ti;
    }

    // ---- 2. space-time filter, accumulated in Q14 ---------------------------
    const int Q = ST_WFRAC + ST_XFRAC;                   // 14
    acc_t ref_r = acc_t(h1r[ST_REF]) << Q;
    acc_t ref_i = acc_t(h1i[ST_REF]) << Q;
    acc_t sr = 0, si = 0;
    for (int k = 0; k < ST_NT; k++) {
#pragma HLS UNROLL
        if (k != ST_REF) {
            ap_int<31> pr, pi;
            cmul3<12>(wtr[k], wti[k], h1r[k], h1i[k], pr, pi);   // Q12
            sr += acc_t(pr) << ST_XFRAC;
            si += acc_t(pi) << ST_XFRAC;
        }
        {
            ap_int<37> pr, pi;
            const int j = ST_NT + k;
            cmul3<18>(wtr[j], wti[j], h2r[k], h2i[k], pr, pi);   // Q14
            sr += acc_t(pr);
            si += acc_t(pi);
        }
    }
    for (int c = 0; c < ST_NIMG; c++) {
#pragma HLS UNROLL
        // image taps: c1 conj(x1[n-a]) (Q12) and c2 conj(x2n[n-a]) (Q14)
        const int a = ST_REF - ST_IMGH + c;
        ap_int<32> pr1, pi1;
        ap_int<38> pr2, pi2;
        const int j1 = ST_NST + c, j2 = ST_NST + ST_NIMG + c;
        cmul3<13>(wtr[j1], wti[j1], ap_int<13>(h1r[a]), ap_int<13>(-ap_int<13>(h1i[a])), pr1, pi1);
        cmul3<19>(wtr[j2], wti[j2], ap_int<19>(h2r[a]), ap_int<19>(-ap_int<19>(h2i[a])), pr2, pi2);
        sr += (acc_t(pr1) << ST_XFRAC) + acc_t(pr2);
        si += (acc_t(pi1) << ST_XFRAC) + acc_t(pi2);
    }
    const acc_t wrnd = (ws == 0) ? acc_t(0) : (acc_t(1) << (ws - 1));
    acc_t ar = ref_r - ((sr + wrnd) >> ws);
    acc_t ai = ref_i - ((si + wrnd) >> ws);

    // ---- 4. make-up shift, 5. fine gain and rounding ----------------------------
    const ap_uint<16> g16 = gain(15, 0);
    const acc_t half = acc_t(1) << (Q - 1);
    ap_int<16> yr, yi;
    if (g16 == ST_GAIN_ONE) {                 // gain 1.0: plain shift and rounding
        yr = sat16(((ar << s) + half) >> Q);
        yi = sat16(((ai << s) + half) >> Q);
    } else {                                  // 6 fractional bits kept, x gain (Q2.14), rounded
        const int UF = 6;
        const acc_t uh = acc_t(1) << (Q - UF - 1);
        acc_t ur = ((ar << s) + uh) >> (Q - UF), ui = ((ai << s) + uh) >> (Q - UF);
        const acc_t umax = (acc_t(1) << 23) - 1, umin = -(acc_t(1) << 23);
        if (ur > umax) ur = umax;
        if (ur < umin) ur = umin;
        if (ui > umax) ui = umax;
        if (ui < umin) ui = umin;
        const ap_int<41> pr = ap_int<24>(ur) * ap_int<17>(g16);
        const ap_int<41> pi = ap_int<24>(ui) * ap_int<17>(g16);
        const acc_t gh = acc_t(1) << (UF + ST_GAIN_FRAC - 1);
        yr = sat16((acc_t(pr) + gh) >> (UF + ST_GAIN_FRAC));
        yi = sat16((acc_t(pi) + gh) >> (UF + ST_GAIN_FRAC));
    }
    ap_uint<32> out;
    out(15, 0)  = (ap_uint<16>)yr;
    out(31, 16) = (ap_uint<16>)yi;
    m_axis_y.write(out);
}
