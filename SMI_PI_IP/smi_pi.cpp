// ============================================================================
//  smi_pi.cpp  --  closed-form (sample-matrix-inversion) power inversion
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2
//
//  ALGORITHM
//    Power inversion with the reference element's weight held at 1:
//        minimise E|y|^2,  y = w^T x,  subject to w1 = 1.
//    For two elements the solution is closed form (Compton's power-inversion
//    solution w ~ R^-1 e1, normalised to w1 = 1), with diagonal loading L:
//
//        R12 = E[x1 conj(x2)]          R22 = E|x2|^2
//        a   = R12 / (R22 + L)
//        y   = x1 - a * x2
//
//    The expectations are exponential averages over 2^K samples:
//        A <- A + p - (A >> K),   A = 2^K * R
//
//  WHY THIS INSTEAD OF AN LMS (PI-NLMS, PL-NPI)
//    - It IS the optimum the LMS cores iterate towards; it gets there in about
//      one averaging window whatever the jammer power or eigenvalue spread.
//    - w1 = 1 is fixed, so the core cannot reduce noise (and the GNSS under it)
//      by turning the whole output down. With no jammer R12 ~ 0, a ~ 0, y ~ x1.
//      There is no gamma/step-size trade-off to tune.
//    - TIMING: the only loop-carried state is A (three 36-bit add/shift
//      recurrences). a is computed from the PREVIOUS A and is never fed back,
//      so the divider and both complex multiplies are plain feed-forward
//      pipeline stages. Nothing with a multiplier closes a loop.
//
//  FIXED POINT
//    x      : 16-bit signed, 12-bit RX values
//    p12,p22: 25/24-bit products
//    A      : 36 bits (25 + K + 1 for K = 10)
//    a      : Q8.16, 25 bits signed (|a| < 256), fits a DSP48 A port
//    divide : numerator and denominator normalised so the denominator has an
//             18-bit mantissa; 42/18-bit pipelined divider
// ============================================================================
#include "smi_pi.h"

static const int K = SMI_PI_K;
static const int AW = 25 + K + 2;           // accumulator width

typedef ap_int<16>  smp_t;
typedef ap_int<AW>  acc_t;
typedef ap_int<25>  wt_t;                   // Q8.16

static ap_int<16> sat16(ap_int<44> v) {
#pragma HLS INLINE
    if (v > 32767)  return 32767;
    if (v < -32768) return -32768;
    return (ap_int<16>)v;
}

// Index of the most significant set bit (0 for v == 0).
template <int W>
static ap_uint<7> msb_index(ap_uint<W> v) {
#pragma HLS INLINE
    ap_uint<7> p = 0;
    for (int i = 0; i < W; i++) {
#pragma HLS UNROLL
        if (v[i]) p = i;
    }
    return p;
}

// q = sat25(num * 2^16 / den), den > 0, truncating toward zero.
static wt_t weight_div(acc_t num, ap_uint<AW + 1> den) {
#pragma HLS INLINE
    ap_uint<7> m   = msb_index<AW + 1>(den);
    ap_uint<7> sft = (m > 17) ? ap_uint<7>(m - 17) : ap_uint<7>(0);
    ap_uint<18> dn = den >> sft;                       // [2^17, 2^18) or small
    ap_int<AW>  nn = num >> sft;
    // |q| >= 2^24 saturates anyway, so the numerator is clipped to 26 bits.
    const ap_int<AW> NMAX = (ap_int<AW>(1) << 25) - 1;
    if (nn > NMAX)  nn = NMAX;
    if (nn < -NMAX) nn = -NMAX;
    ap_int<42> n2 = ap_int<42>(nn) << 16;
    ap_int<42> q  = n2 / ap_int<42>(dn);
    if (q > 16777215)  q = 16777215;
    if (q < -16777216) q = -16777216;
    return (wt_t)q;
}

void smi_pi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<32> load,
            ap_uint<1>  adapt_en,
            ap_uint<2> *wt_band) {
#pragma HLS INTERFACE axis port=s_axis_x
#pragma HLS INTERFACE axis port=m_axis_y
#pragma HLS INTERFACE ap_none port=load
#pragma HLS INTERFACE ap_none port=adapt_en
#pragma HLS INTERFACE ap_none port=wt_band
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=2

    // The only state carried from sample to sample.
    static acc_t A12r = 0, A12i = 0;
    static acc_t A22  = 0;
#pragma HLS RESET variable=A12r
#pragma HLS RESET variable=A12i
#pragma HLS RESET variable=A22
    static ap_uint<2> band = 0;
#pragma HLS RESET variable=band

    *wt_band = band;
    if (s_axis_x.empty()) return;

    ap_uint<64> in = s_axis_x.read();
    smp_t x1r = in(15, 0),  x1i = in(31, 16);
    smp_t x2r = in(47, 32), x2i = in(63, 48);

    // ---- weight from the PREVIOUS covariance (feed-forward) ----------------
    acc_t a12r = A12r, a12i = A12i, a22 = A22;
    ap_uint<32> L = (load == 0) ? ap_uint<32>(1) : load;
    ap_uint<AW + 1> den = ap_uint<AW + 1>(a22) + (ap_uint<AW + 1>(L) << K);
    wt_t ar = weight_div(a12r, den);
    wt_t ai = weight_div(a12i, den);

    // ---- output y = x1 - a*x2 ----------------------------------------------
    ap_int<42> pr = ar * x2r - ai * x2i;               // Q.16
    ap_int<42> pi = ar * x2i + ai * x2r;
    ap_int<16> yr = sat16(ap_int<44>(x1r) - (pr >> 16));
    ap_int<16> yi = sat16(ap_int<44>(x1i) - (pi >> 16));
    ap_uint<32> out;
    out(15, 0)  = (ap_uint<16>)yr;
    out(31, 16) = (ap_uint<16>)yi;
    m_axis_y.write(out);

    // ---- weight-magnitude band (status only) -------------------------------
    wt_t mr = (ar < 0) ? wt_t(-ar) : ar;
    wt_t mi = (ai < 0) ? wt_t(-ai) : ai;
    wt_t mx = (mr > mi) ? mr : mi;
    band = (mx < (1 << 14)) ? 0 : (mx < (1 << 16)) ? 1 : (mx < (1 << 18)) ? 2 : 3;

    // ---- covariance update: the short recurrences ---------------------------
    if (adapt_en) {
        ap_int<25> p12r = ap_int<25>(x1r * x2r) + ap_int<25>(x1i * x2i);
        ap_int<25> p12i = ap_int<25>(x1i * x2r) - ap_int<25>(x1r * x2i);
        ap_int<25> p22  = ap_int<25>(x2r * x2r) + ap_int<25>(x2i * x2i);
        A12r = a12r + p12r - (a12r >> K);
        A12i = a12i + p12i - (a12i >> K);
        A22  = a22  + p22  - (a22  >> K);
    }
}
