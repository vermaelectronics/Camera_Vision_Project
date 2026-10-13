// ============================================================================
//  esprit_core.h -- shared window processing for the two ESPRIT HLS kernels
// ----------------------------------------------------------------------------
//  One call = one window:
//    1. read 2^log2_snap snapshots (II = 1) and accumulate the exact integer
//       moments needed for the covariance R, the pseudo-covariance P and the
//       fourth-order moment M4 of Eq. (7);
//    2. form C (cumulant, or covariance when use_cov = 1);
//    3. normalise, raise to the power p = 2^p_log2 by repeated squaring
//       (Eq. (38)), normalising after each step;
//    4. apply the M = 2 closed form of the selected algorithm and convert the
//       phase to a DOA (Eq. (21) / (35));
//    5. write the 9-word result packet (see esprit_hls.h).
//
//  AREA: step 1 is the only pipelined loop. Steps 2-5 run once per window, so
//  they are written as a sequence of calls to ONE shared single-precision
//  multiplier, adder, divider, square root and int->float converter
//  (non-inlined helpers limited to one instance each with ALLOCATION).
//  atan2 and asin are an 8-term minimax polynomial (|error| < 4e-8 rad) on
//  the same shared operators instead of the HLS math library. Single
//  precision is ample: the result is quantised to Q16 degrees and the
//  target accuracy is 0.05 deg. The steps-2-5 latency (~2k cycles, ~20 us at
//  100 MHz) is covered by the 1024-snapshot stream buffer in gnss_passthrough.
// ============================================================================
#ifndef ESPRIT_CORE_H_
#define ESPRIT_CORE_H_

#include "esprit_hls.h"

#if defined(__SYNTHESIS__) || defined(__VITIS_HLS__)
#include <hls_math.h>
#define E_SQRTF(x) hls::sqrt((float)(x))
#else
#include <cmath>
#define E_SQRTF(x) std::sqrt((float)(x))
#endif

enum { ALGO_IUESPRIT = 0, ALGO_MESPRIT = 1 };

// ---- shared operators (one hardware instance each, see ALLOCATION below) ----
static float esp_fmul(float a, float b) {
#pragma HLS INLINE off
    return a * b;
}
static float esp_fadd(float a, float b) {
#pragma HLS INLINE off
    return a + b;
}
static float esp_fdiv(float a, float b) {
#pragma HLS INLINE off
    return a / b;
}
static float esp_fsqrt(float a) {
#pragma HLS INLINE off
    return E_SQRTF(a);
}
static float esp_i2f(ap_int<64> v) {
#pragma HLS INLINE off
    return (float)(long long)v;
}

#define FM(a, b) esp_fmul((a), (b))
#define FA(a, b) esp_fadd((a), (b))
#define FS(a, b) esp_fadd((a), -(b))

static inline float e_absf(float v) { return v < 0.0f ? -v : v; }

static inline ap_int<32> e_q(float s) {          // s already scaled; round, saturate
    const float lim = 2147483520.0f;               // largest float below 2^31
    if (s > lim)  s = lim;
    if (s < -lim) s = -lim;
    return (ap_int<32>)(int)(s + (s >= 0.0f ? 0.5f : -0.5f));
}

// atan(t), 0 <= t <= 1: odd minimax polynomial, |error| < 3.8e-8 rad
static float esp_atan01(float t) {
    const float c[8] = { 9.999993357e-01f, -3.332986107e-01f, 1.994656879e-01f,
                        -1.390864494e-01f,  9.642236351e-02f, -5.591285677e-02f,
                         2.186332359e-02f, -4.054667831e-03f };
    float t2 = FM(t, t);
    float p = c[7];
POLY:
    for (int i = 6; i >= 0; i--) {
#pragma HLS PIPELINE off
        p = FA(FM(p, t2), c[i]);
    }
    return FM(p, t);
}

static float esp_atan2(float y, float x) {
    const float PI_2 = 1.5707963267948966f, PI_F = 3.1415926535897932f;
    float ax = e_absf(x), ay = e_absf(y);
    if (ax == 0.0f && ay == 0.0f) return 0.0f;
    bool swap = ay > ax;
    float t = swap ? esp_fdiv(ax, ay) : esp_fdiv(ay, ax);
    float r = esp_atan01(t);
    if (swap)     r = FS(PI_2, r);
    if (x < 0.0f) r = FS(PI_F, r);
    return (y < 0.0f) ? -r : r;
}

// asin(u), |u| <= 1
static float esp_asin(float u) {
    float c = FS(1.0f, FM(u, u));
    if (c < 0.0f) c = 0.0f;
    return esp_atan2(u, esp_fsqrt(c));
}

struct esp_moments {
    ap_int<48> r00, r11, r01r, r01i, p0r, p0i, p1r, p1i;
    ap_int<64> m00, m11, m01r, m01i;          // M4 sums >> 16
};

// ---------------------------------------------------------------- steps 2-5
template <int ALGO>
static void esprit_compute(const esp_moments &s, int L, int P, bool cov,
                           ap_uint<32> mu_q30, ap_int<32> k_q30, ap_uint<32> uctol_q24,
                           hls::stream<word_t> &m_axis_y)
{
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances=esp_fmul  limit=1
#pragma HLS ALLOCATION function instances=esp_fadd  limit=1
#pragma HLS ALLOCATION function instances=esp_fdiv  limit=1
#pragma HLS ALLOCATION function instances=esp_fsqrt limit=1
#pragma HLS ALLOCATION function instances=esp_i2f   limit=1
#pragma HLS ALLOCATION function instances=esp_atan2 limit=1

    // 2^-(L+30) and 2^-(L+44): Q1.15 products carry 2^30, M4 sums (>>16) 2^44
    const float INV2[15] = { 9.3132257e-10f, 4.6566129e-10f, 2.3283064e-10f, 1.1641532e-10f,
                             5.8207661e-11f, 2.9103830e-11f, 1.4551915e-11f, 7.2759576e-12f,
                             3.6379788e-12f, 1.8189894e-12f, 9.0949470e-13f, 4.5474735e-13f,
                             2.2737368e-13f, 1.1368684e-13f, 5.6843419e-14f };
    const float s2 = INV2[L];                     // 2^-(30+L)
    const float s4 = FM(s2, 6.1035156e-05f);      // * 2^-14 -> 2^-(44+L)

    float R00  = FM(esp_i2f(s.r00),  s2), R11  = FM(esp_i2f(s.r11),  s2);
    float R01r = FM(esp_i2f(s.r01r), s2), R01i = FM(esp_i2f(s.r01i), s2);

    float c00, c11, c01r, c01i;
    if (cov) {
        c00 = R00; c11 = R11; c01r = R01r; c01i = R01i;
    } else {
        float P0r = FM(esp_i2f(s.p0r), s2), P0i = FM(esp_i2f(s.p0i), s2);
        float P1r = FM(esp_i2f(s.p1r), s2), P1i = FM(esp_i2f(s.p1i), s2);
        float M00  = FM(esp_i2f(s.m00),  s4), M11  = FM(esp_i2f(s.m11),  s4);
        float M01r = FM(esp_i2f(s.m01r), s4), M01i = FM(esp_i2f(s.m01i), s4);
        float R00sq = FM(R00, R00);
        // Eq. (7): C = M4 - R(:,0) R(0,:) - P P^H - R R00
        c00  = FS(FS(M00, FA(R00sq, R00sq)), FA(FM(P0r, P0r), FM(P0i, P0i)));
        c11  = FS(FS(FS(M11, FA(FM(R01r, R01r), FM(R01i, R01i))),
                     FA(FM(P1r, P1r), FM(P1i, P1i))), FM(R11, R00));
        float tR = FA(R00, R00);
        c01r = FS(FS(M01r, FM(tR, R01r)), FA(FM(P0r, P1r), FM(P0i, P1i)));
        c01i = FS(FS(M01i, FM(tR, R01i)), FS(FM(P0i, P1r), FM(P0r, P1i)));
    }

    // ---- 3. normalise (max element = 1), C^p
    float mx = e_absf(c00);
    if (e_absf(c11)  > mx) mx = e_absf(c11);
    if (e_absf(c01r) > mx) mx = e_absf(c01r);
    if (e_absf(c01i) > mx) mx = e_absf(c01i);
    const bool no_signal = !(mx > 0.0f);
    float im = no_signal ? 0.0f : esp_fdiv(1.0f, mx);
    float a = FM(c00, im), d = FM(c11, im), br = FM(c01r, im), bi = FM(c01i, im);
    const float dbg_a = a, dbg_d = d, dbg_br = br, dbg_bi = bi;

POW:
    for (int k = 0; k < 3; k++) {
#pragma HLS PIPELINE off
        if (k < P && !no_signal) {
            float bb  = FA(FM(br, br), FM(bi, bi));
            float ad  = FA(a, d);
            float a2  = FA(FM(a, a), bb), d2 = FA(FM(d, d), bb);
            float br2 = FM(ad, br), bi2 = FM(ad, bi);
            float m = e_absf(a2);
            if (e_absf(d2)  > m) m = e_absf(d2);
            if (e_absf(br2) > m) m = e_absf(br2);
            if (e_absf(bi2) > m) m = e_absf(bi2);
            float inv = (m > 0.0f) ? esp_fdiv(1.0f, m) : 0.0f;
            a = FM(a2, inv); d = FM(d2, inv); br = FM(br2, inv); bi = FM(bi2, inv);
        }
    }

    // ---- 4. closed form, DOA
    const float mu = FM((float)(unsigned)mu_q30, 9.3132257461547852e-10f);   // 2^-30
    float phi, psi_out;
    bool off_circle = false;
    if (ALGO == ALGO_IUESPRIT) {
        float A   = FM(0.5f, FA(a, d));               // forward-backward average
        float r0  = FA(A, br), r1 = bi;               // K1 C_r / sqrt(2)
        float s0  = -bi,       s1 = FS(br, A);        // K2 C_r / sqrt(2) = -(A - br)
        float num = FA(FM(r0, s0), FM(r1, s1));
        float den = FA(FA(FM(r0, r0), FM(r1, r1)), mu);
        psi_out   = (den > 0.0f) ? esp_fdiv(num, den) : 0.0f;   // tan(phi/2)
        phi       = FM(2.0f, esp_atan2(num, den));
    } else {
        float ad  = FA(a, d);
        float bb  = FA(FM(br, br), FM(bi, bi));
        float den = FA(FA(FM(a, a), bb), mu);
        phi       = esp_atan2(-FM(ad, bi), FM(ad, br));          // arg((a+d) b*), den > 0
        psi_out   = (den > 0.0f) ? esp_fdiv(FM(e_absf(ad), esp_fsqrt(bb)), den) : 0.0f;  // |psi|
        off_circle = e_absf(FS(psi_out, 1.0f)) >
                     FM((float)(unsigned)uctol_q24, 5.9604644775390625e-8f);           // 2^-24
    }
    if (no_signal) { phi = 0.0f; psi_out = 0.0f; off_circle = false; }

    const float k = FM((float)(int)k_q30, 9.3132257461547852e-10f);     // Q2.30
    float u = -FM(k, phi);
    const bool ambiguous = (u > 1.0f) || (u < -1.0f);
    if (u > 1.0f)  u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    const float theta = esp_asin(u);

    unsigned flags = 0;
    if (no_signal)  flags |= FLAG_NO_SIGNAL;
    if (ambiguous)  flags |= FLAG_AMBIGUOUS;
    if (off_circle) flags |= FLAG_OFF_CIRCLE;

    // ---- 5. packet
    word_t w0 = 0;
    w0(31, 16) = ESPRIT_MAGIC;
    w0(11, 8)  = flags;
    m_axis_y.write(w0);
    m_axis_y.write((word_t)e_q(FM(phi, 268435456.0f)));             // Q28
    m_axis_y.write((word_t)e_q(FM(psi_out, 16777216.0f)));          // Q8.24
    m_axis_y.write((word_t)e_q(FM(theta, 268435456.0f)));           // Q28
    m_axis_y.write((word_t)e_q(FM(theta, 3754936.4f)));             // deg Q16 = rad * 65536*180/pi
    m_axis_y.write((word_t)e_q(FM(dbg_a, 1073741824.0f)));          // Q30
    m_axis_y.write((word_t)e_q(FM(dbg_d, 1073741824.0f)));
    m_axis_y.write((word_t)e_q(FM(dbg_br, 1073741824.0f)));
    m_axis_y.write((word_t)e_q(FM(dbg_bi, 1073741824.0f)));
}

// ---------------------------------------------------------------- window
template <int ALGO>
void esprit_window(hls::stream<snap_t> &s_axis_x,
                   hls::stream<word_t> &m_axis_y,
                   ap_uint<5> log2_snap, ap_uint<2> p_log2, ap_uint<1> use_cov,
                   ap_uint<32> mu_q30, ap_int<32> k_q30, ap_uint<32> uctol_q24)
{
    int L = log2_snap;
    if (L < ESPRIT_MIN_LOG2) L = ESPRIT_MIN_LOG2;
    if (L > ESPRIT_MAX_LOG2) L = ESPRIT_MAX_LOG2;
    const int N = 1 << L;

    // ---- 1. moments.  x0 = i0 + j q0 (RX1), x1 = i1 + j q1 (RX2), Q1.15
    ap_int<48> s_r00 = 0, s_r11 = 0, s_r01r = 0, s_r01i = 0;   // R = E[x x^H]
    ap_int<48> s_p0r = 0, s_p0i = 0, s_p1r = 0, s_p1i = 0;     // P(m) = E[x_m x0]
    ap_int<80> s_m00 = 0, s_m11 = 0, s_m01r = 0, s_m01i = 0;   // M4(m,n) = E[x_m |x0|^2 x_n^*]

ACC:
    for (int n = 0; n < N; n++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=128 max=16384 avg=1024
        snap_t w = s_axis_x.read();
        ap_int<16> i0 = w(15, 0), q0 = w(31, 16), i1 = w(47, 32), q1 = w(63, 48);

        ap_int<33> e00  = (ap_int<32>)(i0 * i0) + (ap_int<32>)(q0 * q0);    // |x0|^2
        ap_int<33> e11  = (ap_int<32>)(i1 * i1) + (ap_int<32>)(q1 * q1);    // |x1|^2
        ap_int<33> e01r = (ap_int<32>)(i0 * i1) + (ap_int<32>)(q0 * q1);    // Re x0 x1*
        ap_int<33> e01i = (ap_int<32>)(q0 * i1) - (ap_int<32>)(i0 * q1);    // Im x0 x1*
        ap_int<33> p0r  = (ap_int<32>)(i0 * i0) - (ap_int<32>)(q0 * q0);    // Re x0^2
        ap_int<33> p0i  = (ap_int<33>)(i0 * q0) * 2;                        // Im x0^2
        ap_int<33> p1r  = (ap_int<32>)(i1 * i0) - (ap_int<32>)(q1 * q0);    // Re x1 x0
        ap_int<33> p1i  = (ap_int<32>)(i1 * q0) + (ap_int<32>)(q1 * i0);    // Im x1 x0

        s_r00 += e00;  s_r11 += e11;  s_r01r += e01r; s_r01i += e01i;
        s_p0r += p0r;  s_p0i += p0i;  s_p1r  += p1r;  s_p1i  += p1i;
        s_m00  += (ap_int<66>)e00 * e00;
        s_m11  += (ap_int<66>)e00 * e11;
        s_m01r += (ap_int<66>)e00 * e01r;
        s_m01i += (ap_int<66>)e00 * e01i;
    }

    esp_moments m;
    m.r00 = s_r00; m.r11 = s_r11; m.r01r = s_r01r; m.r01i = s_r01i;
    m.p0r = s_p0r; m.p0i = s_p0i; m.p1r = s_p1r; m.p1i = s_p1i;
    m.m00 = s_m00 >> 16; m.m11 = s_m11 >> 16; m.m01r = s_m01r >> 16; m.m01i = s_m01i >> 16;

    esprit_compute<ALGO>(m, L, p_log2, use_cov == 1,
                         mu_q30, k_q30, uctol_q24, m_axis_y);
}

#endif
