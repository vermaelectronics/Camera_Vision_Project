// ============================================================================
//  cn0_array_sim.cpp  --  GPS C/N0 through the REAL fixed-point PL-NPI core (C model)
//
//  One satellite (PRN 1, C/N0 45 dB-Hz, L1 at +5.17 MHz from the LO, 50 MS/s)
//  and a jammer reach RX1 and RX2 in different directions (RX2 = h * (jammer
//  delayed by tau samples)). The result is the C/N0 a receiver would measure on
//  the core's output (1 ms correlations at the true code phase and at 40 wrong
//  ones). J/N is over the whole 50 MHz (J/S = J/N + 32 dB).
//
//    cn0_array_sim <J/S dB or -99> <kind 0 CW, 1 wideband 40 MHz, 2 CW + wideband>
//                  <mode 0 raw RX1, 1 core bypass, 2 PL-NPI ON> <flags> [tau samples]
//      flags: 1 = the core's CW notch on (in front of the loop in v2.5, on the
//                 output when built with -DPL_NPI_V25=0 = the v2.4 core)
//             2 = RX2 equaliser (3 taps, least squares on the first 8192 samples,
//                 like the firmware's fit)
//    env CWOFF = CW offset from L1 [Hz], default 2000
//  Run Tests/run_cn0_table.sh for the table in README_PL_NPI_v2.5.md.
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <complex>
#include <vector>
#include "pl_npi.h"
typedef std::complex<double> cd;

static const double FS = 50e6, FL1 = 1575.42e6 - 1570.25e6, RC = 1.023e6;
static unsigned long long rs = 88172645463325252ULL;
static double urand() { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return ((rs >> 11) + 0.5) / 9007199254740992.0; }
static double grand() { return std::sqrt(-2.0 * std::log(urand())) * std::cos(2 * M_PI * urand()); }
static int clip12(double v) { double r = std::floor(v + 0.5); return (int)(r > 2047 ? 2047 : r < -2048 ? -2048 : r); }

static void ca_code(int prn_g2a, int prn_g2b, int *code) {     // 1023 chips, +-1
    int g1[10], g2[10];
    for (int i = 0; i < 10; i++) g1[i] = g2[i] = 1;
    for (int n = 0; n < 1023; n++) {
        int o = g1[9] ^ (g2[prn_g2a - 1] ^ g2[prn_g2b - 1]);
        code[n] = o ? -1 : 1;
        int f1 = g1[2] ^ g1[9];
        int f2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];
        for (int i = 9; i > 0; i--) { g1[i] = g1[i - 1]; g2[i] = g2[i - 1]; }
        g1[0] = f1; g2[0] = f2;
    }
}

#include <cmath>
static unsigned notch_cfg() { return 1u << PL_NPI_CFG_NOTCH_LSB; }
static ap_uint<192> notch_coef(double f, double fs) {
    ap_uint<192> n = 0; double w = 2 * M_PI * f / fs, r = 1.0 - 1.0 / 256.0;
    std::complex<double> q = std::polar(1.0, w), c[3] = {q, r * q, r * r * q * q};
    for (int i = 0; i < 3; i++) {
        double sc_ = (i == 0) ? 32768.0 : 16384.0; int re = (int)std::lround(c[i].real() * sc_), im = (int)std::lround(c[i].imag() * sc_);
        if (re > 32767) re = 32767; if (im > 32767) im = 32767;
        ap_uint<32> v; v(15, 0) = (ap_uint<16>)(ap_int<16>)re; v(31, 16) = (ap_uint<16>)(ap_int<16>)im;
        n(32 * i + 31, 32 * i) = v;
    }
    return n;
}
#ifndef CORE_HAS_NCOEF
#define CORE_HAS_NCOEF 0
#endif


// usage: gps_sim3 <J/S dB> <kind 0 CW, 1 WB, 2 CW+WB> <mode 0 raw RX1, 1 bypass, 2 PL-NPI ON> <flags> [tau]
//   flags: 1 = the core's CW notch on (input in v2.5, output in a PL_NPI_V25=0 build), 2 = RX2 equaliser (3-tap LS
//          estimated from the first 8192 samples)
int main(int argc, char **argv) {
    double js = atof(argv[1]); int kind = atoi(argv[2]), mode = atoi(argv[3]), flags = atoi(argv[4]);
    double tau = argc > 5 ? atof(argv[5]) : 0.15;
    const double cwoff = (getenv("CWOFF") ? atof(getenv("CWOFF")) : 2000.0), sig = 20.0, n0 = 2 * sig * sig / FS, C = n0 * pow(10, 4.5);
    const double A = sqrt(C), AJ = (js > -90) ? sqrt(C * pow(10, js / 10)) : 0.0;
    const cd hj(0.6, -0.5), hs = std::polar(1.0, 0.7);
    int code[1023]; ca_code(2, 6, code);
    const int NLP = 33; double lp[NLP]; { double sum = 0;
        for (int k = 0; k < NLP; k++) { double m = k - (NLP - 1) / 2.0; double a = (m == 0) ? 0.8 : sin(M_PI * 0.8 * m) / (M_PI * m);
            lp[k] = a * (0.5 + 0.5 * cos(M_PI * m / (NLP / 2.0 + 1))); sum += lp[k] * lp[k]; }
        for (int k = 0; k < NLP; k++) lp[k] /= sqrt(sum); }
    const int NFD = 9; double fd[NFD]; for (int m = 0; m < NFD; m++) { double d = m - 4 - tau; fd[m] = ((d == 0) ? 1.0 : sin(M_PI * d) / (M_PI * d)) * (0.5 + 0.5 * cos(M_PI * d / 5.0)); }
    std::vector<cd> wn(NLP, cd(0, 0)), jh(NFD, cd(0, 0));
    const int NSET = (int)(0.004 * FS), NMEAS = (int)(0.020 * FS), N = NSET + NMEAS;
    std::vector<cd> x1(N), x2(N), y(N);
    double aw = (kind == 2) ? AJ * pow(10, -3.0 / 20) : AJ;      // wideband part
    double ac = (kind == 2) ? AJ * pow(10, -3.0 / 20) : AJ;      // CW part
    for (int n = 0; n < N; n++) {
        double t = n / FS;
        cd s = A * (double)code[(int)floor(fmod(t * RC, 1023.0))] * std::polar(1.0, 2 * M_PI * FL1 * t);
        cd j = 0, j2 = 0;
        if (kind == 0 || kind == 2) { cd c = ac * std::polar(1.0, 2 * M_PI * (FL1 + cwoff) * t + 0.3); j += c; j2 += c; }
        if (kind == 1 || kind == 2) {
            for (int k = NLP - 1; k > 0; k--) wn[k] = wn[k - 1];
            wn[0] = cd(grand(), grand()) / sqrt(2.0);
            cd a = 0; for (int k = 0; k < NLP; k++) a += lp[k] * wn[k];
            for (int k = NFD - 1; k > 0; k--) jh[k] = jh[k - 1];
            jh[0] = a * aw;
            j += jh[4]; for (int m = 0; m < NFD; m++) j2 += fd[m] * jh[m];
        }
        x1[n] = s + j + cd(grand(), grand()) * sig;
        x2[n] = hs * s + hj * j2 + cd(grand(), grand()) * sig;
    }
    ap_uint<96> eqcoef = 0;
    if (flags & 2) {                                   // RX2 equaliser estimated like the firmware will:
        // software notch on a COPY (the CW would dominate the fit), then 3-tap LS  x1[n] ~ sum_k f_k x2[n+1-k]
        std::vector<cd> a1 = x1, a2 = x2;
        if (kind == 0 || kind == 2) {
            double r = 1.0 - 1.0 / 256.0; cd q = std::polar(1.0, 2 * M_PI * (FL1 + cwoff) / FS), p = r * q;
            for (int ch = 0; ch < 2; ch++) { std::vector<cd> &x = ch ? a2 : a1; cd xp = 0, yp = 0;
                for (int n = 0; n < N; n++) { cd xn = x[n]; cd yn = xn - q * xp + p * yp; xp = xn; yp = yn; x[n] = yn; } }
        }
        cd R[3][3] = {}, rv[3] = {};
        for (int n = 8; n < 8192; n++) { cd u[3] = {a2[n + 1], a2[n], a2[n - 1]};
            for (int i = 0; i < 3; i++) { rv[i] += std::conj(u[i]) * a1[n]; for (int j = 0; j < 3; j++) R[i][j] += std::conj(u[i]) * u[j]; } }
        double tr = 0; for (int i = 0; i < 3; i++) tr += R[i][i].real(); for (int i = 0; i < 3; i++) R[i][i] += 1e-3 * tr / 3;
        cd f[3]; { cd M[3][4]; for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) M[i][j] = R[i][j]; M[i][3] = rv[i]; }
            for (int i = 0; i < 3; i++) { cd d = M[i][i]; for (int c = i; c < 4; c++) M[i][c] /= d;
                for (int i2 = 0; i2 < 3; i2++) if (i2 != i) { cd m = M[i2][i]; for (int c = i; c < 4; c++) M[i2][c] -= m * M[i][c]; } }
            for (int i = 0; i < 3; i++) f[i] = M[i][3]; }
        for (int i = 0; i < 3; i++) {
            int re = (int)std::lround(f[i].real() * 16384.0), im = (int)std::lround(f[i].imag() * 16384.0);
            ap_uint<32> v; v(15, 0) = (ap_uint<16>)(ap_int<16>)re; v(31, 16) = (ap_uint<16>)(ap_int<16>)im;
            eqcoef(32 * i + 31, 32 * i) = v;
        }
    }
    hls::stream<ap_uint<64> > xs; hls::stream<ap_uint<32> > ys;
    unsigned g_gamma = (unsigned)(131072.0 / 8.0 * 2 * sig * sig);
    unsigned cfg = (mode == 2) ? PL_NPI_MODE_ON : PL_NPI_MODE_BYPASS;
    ap_uint<192> ncoef = 0;
    if ((flags & 1) && (kind == 0 || kind == 2)) { cfg |= notch_cfg(); ncoef = notch_coef(FL1 + cwoff, FS); }
    if (flags & 2) cfg |= 0x4000u;                      // PL_NPI_CFG_EQ
    cfg |= PL_NPI_CFG_DCBLK;
    for (int n = 0; n < N; n++) {
        if (mode == 0) { y[n] = cd(clip12(x1[n].real()), clip12(x1[n].imag())); continue; }
        ap_uint<64> w;
        w(15, 0) = (ap_uint<16>)(ap_int<16>)clip12(x1[n].real()); w(31, 16) = (ap_uint<16>)(ap_int<16>)clip12(x1[n].imag());
        w(47, 32) = (ap_uint<16>)(ap_int<16>)clip12(x2[n].real()); w(63, 48) = (ap_uint<16>)(ap_int<16>)clip12(x2[n].imag());
        xs.write(w);
        ap_uint<32> st; ap_uint<192> wo; ap_uint<32> pw;
        pl_npi(xs, ys, g_gamma, 1, cfg, 1u << PL_NPI_GFRAC, 0xFFFFFFFFu, 0, ncoef, eqcoef, &st, &wo, &pw);
        ap_uint<32> o = ys.read();
        int k = n - PL_NPI_LATENCY;
        if (k >= 0) y[k] = cd((double)(ap_int<16>)o(15, 0), (double)(ap_int<16>)o(31, 16));
    }
    const int L = (int)(0.001 * FS), NB = NMEAS / L;
    double psig = 0, pnoise = 0;
    for (int b = 0; b < NB; b++) {
        int n0i = NSET + b * L; cd r0 = 0; std::vector<cd> ro(40, 0);
        for (int n = n0i; n < n0i + L; n++) {
            double t = n / FS; cd lo = std::polar(1.0, -2 * M_PI * FL1 * t); double cp = fmod(t * RC, 1023.0);
            cd yl = y[n] * lo; r0 += yl * (double)code[(int)floor(cp)];
            for (int k = 0; k < 40; k++) ro[k] += yl * (double)code[((int)floor(cp) + 20 + 24 * k) % 1023];
        }
        psig += std::norm(r0); for (int k = 0; k < 40; k++) pnoise += std::norm(ro[k]) / 40.0;
    }
    psig /= NB; pnoise /= NB;
    printf("%.1f\n", 10 * log10((psig - pnoise) / pnoise) + 30.0);
    return 0;
}
