// ============================================================================
//  smi_pi.cpp  --  space-time SMI power inversion core (SMI-PI v4.1)
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2
//
//  PIPELINE (one sample pair per clock)
//    1. INPUT NORMALISATION   x2n = g2 * x2
//       RX2 is scaled to RX1's level before anything else (g2 from the two
//       receivers' noise floors, learned by the firmware while there is no
//       jammer). The weights then stay near 1 whatever the two channels'
//       gains are (cable loss, antenna gain, LNA): an RX2 20 dB weaker than
//       RX1 would otherwise need a weight of 10, beyond the Q3.12 range
//       (|w| < 8), and the null would fail. x2n keeps 2 fractional bits so the
//       scaling adds no rounding noise of its own.
//    2. SPACE-TIME FILTER     16 taps on RX1 and 16 on the normalised RX2:
//         y[n] = x1[n-8] - sum_{k != 8} a_k x1[n-k] - sum_k b_k x2n[n-k]
//                        - c1 conj(x1[n-8]) - c2 conj(x2n[n-8])
//       The RX1 tap at delay 8 is the reference, fixed at 1 (power
//       inversion), so the output can never be turned down as a whole. The
//       RX2 taps follow the frequency-dependent RX1/RX2 ratio across the band
//       (cables, antenna spacing, the two analog filters); the RX1 taps also
//       cancel a reflected copy of the jammer.
//       c1, c2 are the IMAGE taps (widely-linear). The AD9361's I/Q mixers
//       are not perfectly balanced: each channel also carries a mirror image
//       of the jammer, ~50 dB down (its image rejection). Against a jammer
//       50 dB above the noise that image sits at the noise level and no
//       linear filter of two antennas can remove it (C model: +4.2 dB residual
//       at J/N 50 dB, +9 dB at 60 dB). The image is a copy of conj(jammer), so
//       one conjugate tap per channel cancels it (+0.2 dB and +1.4 dB). The
//       satellites have no image partner in the band, so the taps leave them
//       alone.
//    3. BYPASS                cfg[0] = 1: y[n] = x1[n-8], no weights at all.
//       The firmware keeps the filter bypassed while there is no jammer and
//       switches it in when one appears. Same 8-sample delay in both modes,
//       so switching never shifts the satellites' code phase.
//    4. OUTPUT MAKE-UP GAIN   y <<= s, s = cfg[10:8] (0..4, 6 dB each)
//       A strong jammer makes the RX gain loop cut the gain (26 dB for a
//       jammer 50 dB above the noise); the satellites then reach the output
//       at a few LSB, and rounding them to integers would cost SNR. The
//       accumulator has 14 fractional bits, so the shift restores the level
//       before the rounding, at no cost in precision. The firmware only sets
//       it while the jammer is nulled (a leaking jammer is never amplified).
//
//  WEIGHTS
//    The 33 weights are the SMI (closed-form) solution of the space-time
//    power-inversion problem, computed by the ARM from a 4096-sample DMA
//    capture of both channels (Source/Firmware/app_gnss_e310/gnss_aj_math.c)
//    a few hundred times a second while a jammer is present, and written to
//    gnss_passthrough, which hands weights, image taps, cfg and nrm to this
//    core all at once.
//
//  TIMING
//    II=1 at 100 MHz (sys_cpu_clk), i.e. up to 100 MS/s, beyond the AD9361's
//    61.44. No loop-carried state except the delay lines.
//
//  FIXED POINT
//    x1      : 12-bit signed, Q0
//    x2n     : 18-bit signed, Q2 (g2 x2, |g2| < 4)
//    weights : 16-bit signed Q3.12, complex multiply with 3 real multipliers
//              (k1 = wr (xr + xi), k2 = xr (wi - wr), k3 = xi (wr + wi);
//               re = k1 - k3, im = k1 + k2): 99 multipliers + 4 for g2
//    sum     : 48 bits, Q14; rounded to nearest after the make-up shift,
//              saturated to 16 bits.
// ============================================================================
#include "smi_pi.h"

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

// Complex product, 3 multipliers. Result Q12 x (Q of x).
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

void smi_pi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<1024> wts,
            ap_uint<64>   wcj,
            ap_uint<32>   cfg,
            ap_uint<32>   nrm) {
#pragma HLS INTERFACE axis port=s_axis_x
#pragma HLS INTERFACE axis port=m_axis_y
#pragma HLS INTERFACE ap_none port=wts
#pragma HLS INTERFACE ap_none port=wcj
#pragma HLS INTERFACE ap_none port=cfg
#pragma HLS INTERFACE ap_none port=nrm
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=1

    // Delay lines: index k holds the sample from k samples ago (0 = now).
    static s12_t h1r[SMI_NT], h1i[SMI_NT];
    static s18_t h2r[SMI_NT], h2i[SMI_NT];
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
    // |g2r x2r - g2i x2i| <= 2 * 32768 * 2048 = 2^28, so after the shift by
    // 11 the result is within +-2^17: 18 bits always hold it, no saturation.
    ap_int<16> g2r = (ap_int<16>)nrm(15, 0), g2i = (ap_int<16>)nrm(31, 16);
    ap_int<32> nr = ap_int<32>(g2r * x2r) - ap_int<32>(g2i * x2i);
    ap_int<32> ni = ap_int<32>(g2r * x2i) + ap_int<32>(g2i * x2r);
    const ap_int<32> nhalf = ap_int<32>(1) << (SMI_GFRAC - SMI_XFRAC - 1);
    s18_t x2nr = (s18_t)((nr + nhalf) >> (SMI_GFRAC - SMI_XFRAC));
    s18_t x2ni = (s18_t)((ni + nhalf) >> (SMI_GFRAC - SMI_XFRAC));

    for (int k = SMI_NT - 1; k > 0; k--) {
#pragma HLS UNROLL
        h1r[k] = h1r[k - 1]; h1i[k] = h1i[k - 1];
        h2r[k] = h2r[k - 1]; h2i[k] = h2i[k - 1];
    }
    h1r[0] = x1r;  h1i[0] = x1i;
    h2r[0] = x2nr; h2i[0] = x2ni;

    // ---- 2. space-time filter, accumulated in Q14 ---------------------------
    const int Q = SMI_WFRAC + SMI_XFRAC;                 // 14
    acc_t ref_r = acc_t(h1r[SMI_REF]) << Q;
    acc_t ref_i = acc_t(h1i[SMI_REF]) << Q;
    acc_t ar = ref_r, ai = ref_i;
    for (int k = 0; k < SMI_NT; k++) {
#pragma HLS UNROLL
        if (k != SMI_REF) {
            ap_int<31> pr, pi;
            w16_t wr = (w16_t)wts(32 * k + 15, 32 * k);
            w16_t wi = (w16_t)wts(32 * k + 31, 32 * k + 16);
            cmul3<12>(wr, wi, h1r[k], h1i[k], pr, pi);   // Q12
            ar -= acc_t(pr) << SMI_XFRAC;
            ai -= acc_t(pi) << SMI_XFRAC;
        }
        {
            ap_int<37> pr, pi;
            const int j = SMI_NT + k;
            w16_t wr = (w16_t)wts(32 * j + 15, 32 * j);
            w16_t wi = (w16_t)wts(32 * j + 31, 32 * j + 16);
            cmul3<18>(wr, wi, h2r[k], h2i[k], pr, pi);   // Q14
            ar -= acc_t(pr);
            ai -= acc_t(pi);
        }
    }
    {   // image taps: c1 conj(x1[n-8]) (Q12) and c2 conj(x2n[n-8]) (Q14)
        ap_int<32> pr1, pi1;
        ap_int<38> pr2, pi2;
        w16_t c1r = (w16_t)wcj(15, 0),  c1i = (w16_t)wcj(31, 16);
        w16_t c2r = (w16_t)wcj(47, 32), c2i = (w16_t)wcj(63, 48);
        cmul3<13>(c1r, c1i, ap_int<13>(h1r[SMI_REF]), ap_int<13>(-ap_int<13>(h1i[SMI_REF])), pr1, pi1);
        cmul3<19>(c2r, c2i, ap_int<19>(h2r[SMI_REF]), ap_int<19>(-ap_int<19>(h2i[SMI_REF])), pr2, pi2);
        ar -= (acc_t(pr1) << SMI_XFRAC) + acc_t(pr2);
        ai -= (acc_t(pi1) << SMI_XFRAC) + acc_t(pi2);
    }

    // ---- 3. bypass, 4. make-up shift and rounding -----------------------------
    if (cfg[0]) { ar = ref_r; ai = ref_i; }
    ap_uint<3> s = cfg(10, 8);
    if (s > SMI_MAXSH) s = SMI_MAXSH;
    const acc_t half = acc_t(1) << (Q - 1);
    ap_int<16> yr = sat16(((ar << s) + half) >> Q);
    ap_int<16> yi = sat16(((ai << s) + half) >> Q);
    ap_uint<32> out;
    out(15, 0)  = (ap_uint<16>)yr;
    out(31, 16) = (ap_uint<16>)yi;
    m_axis_y.write(out);
}
