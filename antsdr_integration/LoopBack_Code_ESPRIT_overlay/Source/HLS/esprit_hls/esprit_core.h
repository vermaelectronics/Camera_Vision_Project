// ============================================================================
//  esprit_core.h -- shared window processing for the two ESPRIT HLS kernels
// ----------------------------------------------------------------------------
//  One call = one window:
//    1. read 2^log2_snap snapshots (II = 1) and accumulate the exact integer
//       moments needed for the covariance R, the pseudo-covariance P and the
//       fourth-order moment M4 of Eq. (7);
//    2. form C (cumulant, or covariance when use_cov = 1) in double precision;
//    3. normalise, raise to the power p = 2^p_log2 by repeated squaring
//       (Eq. (38)), normalising after each step;
//    4. apply the M = 2 closed form of the selected algorithm and convert the
//       phase to a DOA (Eq. (21) / (35));
//    5. write the 9-word result packet (see esprit_hls.h).
//
//  Step 1 is the only pipelined loop; steps 2-5 run once per window. The
//  stream buffer in gnss_passthrough (1024 snapshots) holds the samples that
//  arrive while steps 2-5 run, so no snapshot is lost between windows.
// ============================================================================
#ifndef ESPRIT_CORE_H_
#define ESPRIT_CORE_H_

#include "esprit_hls.h"

#if defined(__SYNTHESIS__) || defined(__VITIS_HLS__)
#include <hls_math.h>
#define E_ATAN2F(y, x) hls::atan2((float)(y), (float)(x))
#define E_ASINF(x)     hls::asin((float)(x))
#define E_SQRTF(x)     hls::sqrt((float)(x))
#else
#include <cmath>
#define E_ATAN2F(y, x) std::atan2((float)(y), (float)(x))
#define E_ASINF(x)     std::asin((float)(x))
#define E_SQRTF(x)     std::sqrt((float)(x))
#endif

enum { ALGO_IUESPRIT = 0, ALGO_MESPRIT = 1 };

static inline float e_absf(float v) { return v < 0.0f ? -v : v; }
static inline double e_absd(double v) { return v < 0.0 ? -v : v; }

static inline ap_int<32> e_q(float v, float scale) {
    float s = v * scale;
    const float lim = 2147483520.0f;          // largest float below 2^31
    if (s > lim)  s = lim;
    if (s < -lim) s = -lim;
    return (ap_int<32>)(int)(s + (s >= 0.0f ? 0.5f : -0.5f));
}

template <int ALGO>
void esprit_window(hls::stream<snap_t> &s_axis_x,
                   hls::stream<word_t> &m_axis_y,
                   ap_uint<5> log2_snap, ap_uint<2> p_log2, ap_uint<1> use_cov,
                   ap_uint<32> mu_q30, ap_int<32> k_q30, ap_uint<32> uctol_q24)
{
    // ---------------------------------------------------------------- config
    int L = log2_snap;
    if (L < ESPRIT_MIN_LOG2) L = ESPRIT_MIN_LOG2;
    if (L > ESPRIT_MAX_LOG2) L = ESPRIT_MAX_LOG2;
    const int N = 1 << L;
    const int P = p_log2;
    const bool cov = (use_cov == 1);

    // ---------------------------------------------------------- 1. moments
    // x0 = i0 + j q0 (RX1), x1 = i1 + j q1 (RX2), Q1.15
    ap_int<48> s_r00 = 0, s_r11 = 0, s_r01r = 0, s_r01i = 0;   // R = E[x x^H]
    ap_int<48> s_p0r = 0, s_p0i = 0, s_p1r = 0, s_p1i = 0;     // P(m) = E[x_m x0]
    ap_int<80> s_m00 = 0, s_m11 = 0, s_m01r = 0, s_m01i = 0;   // M4(m,n) = E[x_m |x0|^2 x_n^*]

ACC:
    for (int n = 0; n < N; n++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=128 max=16384 avg=1024
        snap_t w = s_axis_x.read();
        ap_int<16> i0 = w(15, 0), q0 = w(31, 16), i1 = w(47, 32), q1 = w(63, 48);

        ap_int<33> e00   = (ap_int<32>)(i0 * i0) + (ap_int<32>)(q0 * q0);    // |x0|^2
        ap_int<33> e11   = (ap_int<32>)(i1 * i1) + (ap_int<32>)(q1 * q1);    // |x1|^2
        ap_int<33> e01r  = (ap_int<32>)(i0 * i1) + (ap_int<32>)(q0 * q1);    // Re x0 x1*
        ap_int<33> e01i  = (ap_int<32>)(q0 * i1) - (ap_int<32>)(i0 * q1);    // Im x0 x1*
        ap_int<33> p0r   = (ap_int<32>)(i0 * i0) - (ap_int<32>)(q0 * q0);    // Re x0^2
        ap_int<33> p0i   = (ap_int<33>)(i0 * q0) * 2;                        // Im x0^2
        ap_int<33> p1r   = (ap_int<32>)(i1 * i0) - (ap_int<32>)(q1 * q0);    // Re x1 x0
        ap_int<33> p1i   = (ap_int<32>)(i1 * q0) + (ap_int<32>)(q1 * i0);    // Im x1 x0

        s_r00  += e00;  s_r11  += e11;  s_r01r += e01r; s_r01i += e01i;
        s_p0r  += p0r;  s_p0i  += p0i;  s_p1r  += p1r;  s_p1i  += p1i;
        s_m00  += (ap_int<66>)e00 * e00;
        s_m11  += (ap_int<66>)e00 * e11;
        s_m01r += (ap_int<66>)e00 * e01r;
        s_m01i += (ap_int<66>)e00 * e01i;
    }

    // --------------------------------------------- 2. C = cumulant / covariance
    // sums of Q1.15 products carry a 2^30 scale, M4 sums 2^60; M4 sums are
    // shifted by 16 bits so they convert through a 64-bit integer.
    const double inv2 = 1.0 / ((double)N * 1073741824.0);               // 1 / (N 2^30)
    const double inv4 = 65536.0 / ((double)N * 1152921504606846976.0);  // 2^16 / (N 2^60)
    double R00  = (double)(long long)s_r00  * inv2;
    double R11  = (double)(long long)s_r11  * inv2;
    double R01r = (double)(long long)s_r01r * inv2;
    double R01i = (double)(long long)s_r01i * inv2;
    double P0r  = (double)(long long)s_p0r  * inv2;
    double P0i  = (double)(long long)s_p0i  * inv2;
    double P1r  = (double)(long long)s_p1r  * inv2;
    double P1i  = (double)(long long)s_p1i  * inv2;
    double M00  = (double)(long long)(ap_int<64>)(s_m00  >> 16) * inv4;
    double M11  = (double)(long long)(ap_int<64>)(s_m11  >> 16) * inv4;
    double M01r = (double)(long long)(ap_int<64>)(s_m01r >> 16) * inv4;
    double M01i = (double)(long long)(ap_int<64>)(s_m01i >> 16) * inv4;

    double c00, c11, c01r, c01i;
    if (cov) {
        c00 = R00; c11 = R11; c01r = R01r; c01i = R01i;
    } else {
        // Eq. (7): C = M4 - R(:,0) R(0,:) - P P^H - R R00
        c00  = M00 - 2.0 * R00 * R00 - (P0r * P0r + P0i * P0i);
        c11  = M11 - (R01r * R01r + R01i * R01i) - (P1r * P1r + P1i * P1i) - R11 * R00;
        c01r = M01r - 2.0 * R00 * R01r - (P0r * P1r + P0i * P1i);
        c01i = M01i - 2.0 * R00 * R01i - (P0i * P1r - P0r * P1i);
    }

    // ---------------------------------------------------- 3. normalise, C^p
    double mx = e_absd(c00);
    if (e_absd(c11)  > mx) mx = e_absd(c11);
    if (e_absd(c01r) > mx) mx = e_absd(c01r);
    if (e_absd(c01i) > mx) mx = e_absd(c01i);
    const bool no_signal = !(mx > 0.0);
    const double inv_mx = no_signal ? 0.0 : 1.0 / mx;
    float a  = (float)(c00  * inv_mx);
    float d  = (float)(c11  * inv_mx);
    float br = (float)(c01r * inv_mx);
    float bi = (float)(c01i * inv_mx);
    const float dbg_a = a, dbg_d = d, dbg_br = br, dbg_bi = bi;

POW:
    for (int k = 0; k < 3; k++) {
#pragma HLS LOOP_TRIPCOUNT min=0 max=3
        if (k < P && !no_signal) {
            float bb = br * br + bi * bi;
            float a2 = a * a + bb, d2 = d * d + bb;
            float br2 = (a + d) * br, bi2 = (a + d) * bi;
            float m = e_absf(a2);
            if (e_absf(d2)  > m) m = e_absf(d2);
            if (e_absf(br2) > m) m = e_absf(br2);
            if (e_absf(bi2) > m) m = e_absf(bi2);
            float im = (m > 0.0f) ? 1.0f / m : 0.0f;
            a = a2 * im; d = d2 * im; br = br2 * im; bi = bi2 * im;
        }
    }

    // ------------------------------------------------- 4. closed form, DOA
    const float mu = (float)mu_q30 * 9.3132257461547852e-10f;   // 2^-30
    float phi, psi_out, off = 0.0f;
    bool off_circle = false;
    if (ALGO == ALGO_IUESPRIT) {
        float A   = 0.5f * (a + d);                 // forward-backward average
        float r0  = A + br, r1 = bi;                // K1 C_r / sqrt(2)
        float s0  = -bi,    s1 = -(A - br);         // K2 C_r / sqrt(2)
        float num = r0 * s0 + r1 * s1;
        float den = r0 * r0 + r1 * r1 + mu;
        psi_out   = (den > 0.0f) ? num / den : 0.0f;          // tan(phi/2)
        phi       = 2.0f * E_ATAN2F(num, den);
    } else {
        float ad  = a + d;
        float bb  = br * br + bi * bi;
        float den = a * a + bb + mu;
        float pr  = ad * br, pi = -ad * bi;                   // (a+d) b*, / den > 0
        phi       = E_ATAN2F(pi, pr);
        psi_out   = (den > 0.0f) ? e_absf(ad) * E_SQRTF(bb) / den : 0.0f;   // |psi|
        off       = e_absf(psi_out - 1.0f);
        off_circle = off > (float)uctol_q24 * 5.9604644775390625e-8f;       // 2^-24
    }
    if (no_signal) { phi = 0.0f; psi_out = 0.0f; off_circle = false; }

    const float k = (float)k_q30 * 9.3132257461547852e-10f;     // lambda/(2 pi d), Q2.30
    float u = -k * phi;
    const bool ambiguous = (u > 1.0f) || (u < -1.0f);
    if (u > 1.0f)  u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    const float theta = E_ASINF(u);
    const float deg   = theta * 57.295779513082321f;

    unsigned flags = 0;
    if (no_signal)  flags |= FLAG_NO_SIGNAL;
    if (ambiguous)  flags |= FLAG_AMBIGUOUS;
    if (off_circle) flags |= FLAG_OFF_CIRCLE;

    // ---------------------------------------------------------- 5. packet
    word_t w0 = 0;
    w0(31, 16) = ESPRIT_MAGIC;
    w0(11, 8)  = flags;
    m_axis_y.write(w0);
    m_axis_y.write((word_t)e_q(phi, 268435456.0f));           // Q28
    m_axis_y.write((word_t)e_q(psi_out, 16777216.0f));        // Q8.24
    m_axis_y.write((word_t)e_q(theta, 268435456.0f));         // Q28
    m_axis_y.write((word_t)e_q(deg, 65536.0f));               // Q16
    m_axis_y.write((word_t)e_q(dbg_a, 1073741824.0f));        // Q30
    m_axis_y.write((word_t)e_q(dbg_d, 1073741824.0f));
    m_axis_y.write((word_t)e_q(dbg_br, 1073741824.0f));
    m_axis_y.write((word_t)e_q(dbg_bi, 1073741824.0f));
}

#endif
