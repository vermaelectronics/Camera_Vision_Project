// ============================================================================
//  testbench.cpp  --  C simulation of the SFAP-PI core (v5.0)
//
//  The weights come from the FIRMWARE's own solver
//  (Source/Firmware/app_sfap_lvds/gnss_aj_math.c, compiled into this
//  testbench), run on 4096-sample captures packed exactly like the DMA
//  buffer, so this tests the board's weight computation and the FPGA filter
//  together.
//
//  Signals at the raw 12-bit ADC scale, noise 20 LSB rms per I/Q component:
//      RX1 = IQ1( J + s + n1 )
//      RX2 = IQ2( a (H(J) + g s + n2) )
//  J: complex Gaussian jammer low-passed to 0.84 fs (fills the band), or CW.
//  H: 0.15-sample delay, 0.8 dB / 40 deg, slightly different filter.
//  a: RX2 channel gain relative to RX1 (scenario 7: -25 dB).
//  IQ: the AD9361's I/Q imbalance (scenarios 6, 8: ~48 dB image rejection).
//  s: weak BPSK "satellite" (1.023 Mchip/s at 50 MS/s, -9.08 MHz offset).
//
//  Scenario (argv[1]); each needs a freshly reset core, so the csim script
//  runs one per process:
//    1  core accuracy: random per-bin weights, image taps and g2 against a
//       double-precision WOLA model (error <= 1 LSB rms), latency exactly
//       SFAP_LAT, zero weights reconstruct x1 to +-1 LSB, bypass is exactly
//       2^s x1[n - SFAP_LAT], output saturation, shift > 4 acts as 4
//    2  wideband jammer 30 dB: suppression >= 35 dB, satellite SNR loss <= 3 dB,
//       and >= 10 dB more suppression than the single-weight (v3) mode
//    3  no jammer: bypass output is exactly x1[n-190]; with the filter in,
//       output within 0.5 dB of RX1 and satellite loss <= 0.5 dB
//    4  CW jammer 40 dB: suppression >= 40 dB
//    5  wideband jammer 30 dB plus a reflection (-6 dB, 3 samples later,
//       other direction): suppression >= 30 dB
//    6  wideband jammer 50 dB with I/Q imbalance: residual <= 1.5 dB with the
//       image taps (and >= 2 dB better than without), suppression >= 45 dB,
//       satellite loss <= 2 dB with the output make-up gain
//    7  RX2 25 dB weaker than RX1, wideband jammer 40 dB: with the input
//       normalisation suppression >= 35 dB and >= 10 dB more than without
//    8  wideband jammer 60 dB with I/Q imbalance: suppression >= 50 dB
//    9  TWO CW jammers, 40 dB each, from different directions at different
//       frequencies: suppression >= 35 dB, satellite loss <= 5 dB (one
//       broadband weight cannot do this: it must be >= 20 dB worse)
//  Prints "SFAP_PI_CSIM: PASS" or "SFAP_PI_CSIM: FAIL".
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <complex>
#include <algorithm>
#include "sfap_pi.h"
#include "gnss_aj_math.h"

typedef std::complex<double> cd;
typedef std::vector<cd> cvec;

static unsigned long long rng_s = 88172645463325252ULL;
static double urand() {
    rng_s ^= rng_s << 13; rng_s ^= rng_s >> 7; rng_s ^= rng_s << 17;
    return ((rng_s >> 11) + 0.5) / 9007199254740992.0;
}
static double grand() { return std::sqrt(-2.0 * std::log(urand())) * std::cos(2 * M_PI * urand()); }

static cvec fir(const cvec &x, const cvec &h, int delay) {
    int n = x.size(), m = h.size();
    cvec y(n, 0.0);
    for (int i = 0; i < n; i++)
        for (int k = 0; k < m; k++) {
            int j = i - k + delay;
            if (j >= 0 && j < n) y[i] += h[k] * x[j];
        }
    return y;
}

static cvec lowpass(double fc, int taps) {           // fc in fs units
    cvec h(taps);
    int c = taps / 2;
    for (int k = 0; k < taps; k++) {
        double t = k - c, w = 0.54 - 0.46 * std::cos(2 * M_PI * k / (taps - 1));
        h[k] = (t == 0 ? 2 * fc : std::sin(2 * M_PI * fc * t) / (M_PI * t)) * w;
    }
    return h;
}

static cvec frac_delay(double d, int taps) {         // windowed sinc, centred
    cvec h(taps);
    int c = taps / 2;
    for (int k = 0; k < taps; k++) {
        double t = k - c - d, w = 0.54 - 0.46 * std::cos(2 * M_PI * k / (taps - 1));
        h[k] = (std::fabs(t) < 1e-12 ? 1.0 : std::sin(M_PI * t) / (M_PI * t)) * w;
    }
    return h;
}

static double pwr(const cvec &x, int from) {
    double p = 0; int n = 0;
    for (size_t i = from; i < x.size(); i++, n++) p += std::norm(x[i]);
    return n ? p / n : 0;
}

static int q12(double v) {
    double r = std::floor(v + 0.5);
    if (r > 2047) r = 2047; if (r < -2048) r = -2048;
    return (int)r;
}

// ---- the core, driven sample by sample ---------------------------------------
// crpa13: the weights go through the core's write port into the bank not in use, then a commit
// flips the bank, as gnss_passthrough does
static ap_uint<16>   g_wseq = 0;
static ap_uint<9>    g_waddr = 0;
static ap_uint<32>   g_wdata = 0;
static ap_uint<1>    g_bank = 0;
static ap_uint<32>   g_cfg = SFAP_CFG_BYPASS;
static ap_uint<32>   g_nrm = 1 << SFAP_GFRAC;
static const int     LAT = SFAP_LAT, SKIP = SFAP_LAT + 2 * SFAP_N;
static long long     g_calls = 0;          // calls of the core so far (its frame clock)

static void core_idle();
static void set_weights(const int16_t *wre, const int16_t *wim) {
    // word i: b_k (i = k), c1_k (i = N + k), c2_k (i = 2N + k) = firmware order AJ_WB / AJ_WC1 / AJ_WC2
    for (int i = 0; i < SFAP_NW; i++) {
        ap_uint<32> w = 0;
        w(15, 0) = (ap_uint<16>)(ap_int<16>)wre[i];
        w(31, 16) = (ap_uint<16>)(ap_int<16>)wim[i];
        g_waddr = i; g_wdata = w; g_wseq++;
        core_idle();                       // one clock with no sample: the core takes the write
    }
    g_bank = g_bank ? 0 : 1;               // AJ_COMMIT
}

static void set_cfg(int bypass, int shift) {
    g_cfg = (bypass ? SFAP_CFG_BYPASS : 0u) | ((unsigned)(shift & 7) << SFAP_CFG_SH_LSB);
}

static void set_g2(int g2r, int g2i) {
    g_nrm = 0;
    g_nrm(15, 0)  = (ap_uint<16>)(ap_int<16>)g2r;
    g_nrm(31, 16) = (ap_uint<16>)(ap_int<16>)g2i;
}

static void core_idle() {
    hls::stream<ap_uint<64> > xs;
    hls::stream<ap_uint<32> > ys;
    sfap_pi(xs, ys, g_wseq, g_waddr, g_wdata, g_bank, g_cfg, g_nrm);
    if (!ys.empty()) { std::printf("core produced output without input\n"); std::exit(2); }
}

static void push(hls::stream<ap_uint<64> > &xs, hls::stream<ap_uint<32> > &ys,
                 int a, int b, int c, int d, cd *out) {
    int v[4] = {a, b, c, d};
    ap_uint<64> w = 0;
    for (int k = 0; k < 4; k++) w(16 * k + 15, 16 * k) = (ap_uint<16>)(ap_int<16>)v[k];
    xs.write(w);
    sfap_pi(xs, ys, g_wseq, g_waddr, g_wdata, g_bank, g_cfg, g_nrm);
    g_calls++;
    if (ys.empty()) { std::printf("core produced no output\n"); std::exit(2); }
    ap_uint<32> o = ys.read();
    if (out) *out = cd((double)(ap_int<16>)o(15, 0), (double)(ap_int<16>)o(31, 16));
}

// Runs (x1,x2) through the core after a zero flush longer than the latency;
// y[i] is the output for input i (it carries input i - LAT). The data starts
// on a frame boundary of the core (call count = 0 mod HOP), so the
// double-precision model's frames (offsets 0 and N/2) are the core's frames.
static cvec run_core(const std::vector<int> &a1r, const std::vector<int> &a1i,
                     const std::vector<int> &a2r, const std::vector<int> &a2i) {
    hls::stream<ap_uint<64> > xs;
    hls::stream<ap_uint<32> > ys;
    cvec y(a1r.size());
    for (int i = 0; i < LAT + 2 * SFAP_N || g_calls % SFAP_HOP; i++) push(xs, ys, 0, 0, 0, 0, 0);
    for (size_t i = 0; i < a1r.size(); i++) push(xs, ys, a1r[i], a1i[i], a2r[i], a2i[i], &y[i]);
    return y;
}

static cvec run_core_c(const cvec &x1, const cvec &x2) {
    int n = x1.size();
    std::vector<int> a(n), b(n), c(n), d(n);
    for (int i = 0; i < n; i++) {
        a[i] = q12(x1[i].real()); b[i] = q12(x1[i].imag());
        c[i] = q12(x2[i].real()); d[i] = q12(x2[i].imag());
    }
    return run_core(a, b, c, d);
}

// ---- scenario 1: accuracy against a double-precision model -----------------
// The FPGA filter bank in double: frames of N at offsets 0 and N/2 (mod N),
// window sin(pi n/N) in and out, per-bin weights, overlap-add.
static cvec wola_ref(const cvec &x1, const cvec &x2, const int16_t *wr, const int16_t *wi) {
    const int N = SFAP_N, L = x1.size();
    cvec r(L, 0.0);
    std::vector<double> w(N);
    for (int n = 0; n < N; n++) w[n] = std::floor(65536.0 * std::sin(M_PI * n / N) + 0.5) / 65536.0;
    auto W = [&](int base, int k) { return cd(wr[base + k], wi[base + k]) / 4096.0; };
    for (int off = 0; off < N; off += N / 2)
        for (int st = off; st + N <= L; st += N) {
            cvec X1(N), X2(N), Y(N);
            for (int k = 0; k < N; k++) {
                cd s1 = 0, s2 = 0;
                for (int n = 0; n < N; n++) {
                    cd e = std::polar(1.0, -2 * M_PI * k * n / N);
                    s1 += w[n] * x1[st + n] * e; s2 += w[n] * x2[st + n] * e;
                }
                X1[k] = s1; X2[k] = s2;
            }
            for (int k = 0; k < N; k++) {
                int m = (N - k) % N;
                Y[k] = X1[k] - W(AJ_WB, k) * X2[k] - W(AJ_WC1, k) * std::conj(X1[m])
                             - W(AJ_WC2, k) * std::conj(X2[m]);
            }
            for (int n = 0; n < N; n++) {
                cd s = 0;
                for (int k = 0; k < N; k++) s += Y[k] * std::polar(1.0, 2 * M_PI * k * n / N);
                r[st + n] += w[n] * s / (double)N;
            }
        }
    return r;
}

static int test_accuracy() {
    const int n = 12000;
    std::vector<int> a(n), b(n), c(n), d(n);
    cvec x1(n), x2(n), x2n(n);
    int16_t wr[AJ_NW], wi[AJ_NW];
    int ok = 1;
    for (int i = 0; i < n; i++) {
        a[i] = (int)(urand() * 1800) - 900; b[i] = (int)(urand() * 1800) - 900;
        c[i] = (int)(urand() * 1800) - 900; d[i] = (int)(urand() * 1800) - 900;
        x1[i] = cd(a[i], b[i]);
    }
    // g2 = 0.8 - 0.3j: the model normalises exactly as the core (Q4, rounded)
    const int g2r = 6554, g2i = -2458;
    for (int i = 0; i < n; i++) {
        long long nr = (long long)g2r * c[i] - (long long)g2i * d[i];
        long long ni = (long long)g2r * d[i] + (long long)g2i * c[i];
        x2n[i] = cd((double)((nr + 256) >> 9), (double)((ni + 256) >> 9)) / 16.0;
    }
    // (a) random weights over a realistic range
    for (int k = 0; k < AJ_NW; k++) {
        int span = (k < AJ_WC1) ? 8192 : 1024;
        wr[k] = (int16_t)((int)(urand() * span) - span / 2);
        wi[k] = (int16_t)((int)(urand() * span) - span / 2);
    }
    set_weights(wr, wi); set_g2(g2r, g2i); set_cfg(0, 0);
    cvec y = run_core(a, b, c, d);
    cvec r = wola_ref(x1, x2n, wr, wi);
    double e2 = 0, p2 = 0, emax = 0; int cnt = 0, best = -1; double bestv = 1e300;
    for (int i = SKIP; i < n - SFAP_N; i++) {
        cd e = y[i] - r[i - LAT];
        e2 += std::norm(e); p2 += std::norm(r[i - LAT]); cnt++;
        emax = std::max(emax, std::max(std::fabs(e.real()), std::fabs(e.imag())));
    }
    for (int dd = LAT - 4; dd <= LAT + 4; dd++) {
        double s = 0;
        for (int i = SKIP; i < n - SFAP_N; i++) s += std::norm(y[i] - r[i - dd]);
        if (s < bestv) { bestv = s; best = dd; }
    }
    std::printf("weights   : error %.3f LSB rms (max %.2f) on a %.0f LSB rms output, SER %.1f dB, latency %d (expect %d)\n",
                std::sqrt(e2 / cnt), emax, std::sqrt(p2 / cnt), 10 * std::log10(p2 / e2), best, LAT);
    ok &= std::sqrt(e2 / cnt) <= 1.0 && emax <= 4.0 && best == LAT;

    // (b) zero weights: the filter bank alone reconstructs x1
    int16_t z[AJ_NW] = {0};
    set_weights(z, z); set_cfg(0, 0);
    y = run_core(a, b, c, d);
    int bad = 0; double zmax = 0;
    for (int i = SKIP; i < n; i++) {
        cd e = y[i] - x1[i - LAT];
        zmax = std::max(zmax, std::max(std::fabs(e.real()), std::fabs(e.imag())));
        if (zmax > 1) bad++;
    }
    std::printf("zero w    : max |y - x1[n-%d]| = %.0f LSB\n", LAT, zmax);
    ok &= zmax <= 1.0;

    // (c) bypass with every shift: exactly 2^s x1[n - LAT] (saturated), 5..7 act as 4
    for (int i = 0; i < n; i++) {             // full 12-bit range, so x16 saturates
        a[i] = (int)(urand() * 4096) - 2048; b[i] = (int)(urand() * 4096) - 2048;
    }
    int sats = 0;
    for (int sh = 0; sh < 8; sh++) {
        set_cfg(1, sh);
        y = run_core(a, b, c, d);
        int s_eff = sh > SFAP_MAXSH ? SFAP_MAXSH : sh;
        bad = 0;
        for (int i = LAT; i < n; i++) {
            long long er = std::max(-32768LL, std::min(32767LL, (long long)a[i - LAT] << s_eff));
            long long ei = std::max(-32768LL, std::min(32767LL, (long long)b[i - LAT] << s_eff));
            if (std::abs(er) == 32767 || er == -32768) sats++;
            if ((long long)y[i].real() != er || (long long)y[i].imag() != ei) bad++;
        }
        if (bad) { std::printf("bypass s=%d: %d samples wrong\n", sh, bad); ok = 0; }
    }
    std::printf("bypass    : exact for s = 0..7 (%d saturated samples checked)\n", sats);
    ok &= sats > 0;

    // (d) full-scale input, large weights: output saturates, never wraps
    for (int i = 0; i < n; i++) {
        a[i] = (i & 1) ? 2047 : -2048; b[i] = (i & 2) ? 2047 : -2048;
        c[i] = (i & 4) ? 2047 : -2048; d[i] = -b[i];
    }
    for (int k = 0; k < AJ_NW; k++) { wr[k] = (k < AJ_WC1) ? -32768 : 32767; wi[k] = 0; }
    set_weights(wr, wi); set_g2(32767, 32767); set_cfg(0, 4);
    y = run_core(a, b, c, d);
    int pos = 0, neg = 0;
    for (int i = SKIP; i < n; i++) { if (y[i].real() == 32767) pos++; if (y[i].real() == -32768) neg++; }
    std::printf("overload  : %d / %d samples at +/- full scale\n", pos, neg);
    ok &= (pos + neg) > 0;
    return ok;
}

// ---- scenarios 2..9 ------------------------------------------------------------
struct Scen { double jnr_db; bool cw; double echo; double rx2_db; bool iq; bool two = false; };

// AD9361-like I/Q imbalance: gain error eps, phase error ph (degrees).
static void iq_imbalance(cvec &x, double eps, double ph_deg) {
    double ph = ph_deg * M_PI / 180;
    cd mu = (1.0 + (1 + eps) * std::polar(1.0, -ph)) / 2.0;
    cd nu = (1.0 - (1 + eps) * std::polar(1.0, ph)) / 2.0;
    for (auto &v : x) v = mu * v + nu * std::conj(v);
}
static double irr_db(double eps, double ph_deg) {
    double ph = ph_deg * M_PI / 180;
    cd mu = (1.0 + (1 + eps) * std::polar(1.0, -ph)) / 2.0;
    cd nu = (1.0 - (1 + eps) * std::polar(1.0, ph)) / 2.0;
    return 10 * std::log10(std::norm(mu) / std::norm(nu));
}

static int test_jammer(int id, Scen sc) {
    const int N = 3 * AJ_NSAMP + 20000;
    const double nrms = 20.0, fsat = -9.08 / 50.0;
    const double IQ1_EPS = 0.004, IQ1_PH = 0.3, IQ2_EPS = -0.003, IQ2_PH = -0.4;
    cvec J(N), J2, n1(N), n2(N), s1(N), s2(N);
    // jammer
    cvec Jb(N, 0.0), Jb2;
    if (sc.cw) {
        for (int i = 0; i < N; i++) J[i] = std::polar(1.0, 2 * M_PI * 0.06 * i);
        if (sc.two)     // second CW: other frequency, other direction (below)
            for (int i = 0; i < N; i++) Jb[i] = std::polar(1.0, 2 * M_PI * (-0.21) * i + 0.7);
    } else {
        for (int i = 0; i < N; i++) J[i] = cd(grand(), grand());
        J = fir(J, lowpass(0.42, 41), 20);
    }
    // RX2 sees it through a different path: 0.15-sample delay, 0.8 dB, 40 deg,
    // and a slightly different filter.
    cvec h2 = frac_delay(0.15, 21);
    cvec hd = {cd(0.03, 0.01), 1.0, cd(-0.02, 0.015)};
    J2 = fir(fir(J, h2, 10), hd, 1);
    cd g2p = std::polar(std::pow(10, 0.8 / 20), 40 * M_PI / 180);
    for (auto &v : J2) v *= g2p;
    Jb2 = Jb;           // the second jammer's RX2/RX1 ratio: -1.5 dB / -110 deg
    for (auto &v : Jb2) v *= std::polar(std::pow(10, -1.5 / 20), -110 * M_PI / 180);
    for (int i = 0; i < N; i++) { J[i] += Jb[i]; J2[i] += Jb2[i]; }
    if (sc.echo > 0) {      // a reflection 3 samples later from another direction
        cvec e1(N), e2(N);
        for (int i = 3; i < N; i++) { e1[i] = sc.echo * std::polar(1.0, 1.3) * J[i - 3]; e2[i] = sc.echo * std::polar(1.0, 2.9) * J[i - 3]; }
        for (int i = 0; i < N; i++) { J[i] += e1[i]; J2[i] += e2[i]; }
    }
    double pj = pwr(J, 0) / (sc.two ? 2.0 : 1.0), pn = 2 * nrms * nrms;
    double js = (sc.jnr_db > -90) ? std::sqrt(pn * std::pow(10, sc.jnr_db / 10) / pj) : 0.0;
    for (int i = 0; i < N; i++) { J[i] *= js; J2[i] *= js; }
    // noise and satellite
    int chip = 1; double cph = 0;
    for (int i = 0; i < N; i++) {
        n1[i] = cd(grand(), grand()) * nrms;
        n2[i] = cd(grand(), grand()) * nrms;
        cph += 1.023 / 50.0; if (cph >= 1) { cph -= 1; chip = urand() < 0.5 ? -1 : 1; }
        s1[i] = std::polar(0.1 * nrms * chip, 2 * M_PI * fsat * i);
        s2[i] = s1[i] * std::polar(1.0, -1.1);
    }
    // RX2 channel gain (cable, antenna, LNA)
    double a2 = std::pow(10, sc.rx2_db / 20);
    for (int i = 0; i < N; i++) { J2[i] *= a2; n2[i] *= a2; s2[i] *= a2; }

    // g2 as the firmware learns it: the two noise floors at the clean gain;
    // rx2n = what is left of the RX2/RX1 noise ratio after g2 (1 unless g2
    // hit its limit)
    double g2 = 1.0, rx2n = 1.0;
    {
        cvec q1(N), q2(N);
        for (int i = 0; i < N; i++) { q1[i] = n1[i]; q2[i] = n2[i]; }
        if (sc.iq) { iq_imbalance(q1, IQ1_EPS, IQ1_PH); iq_imbalance(q2, IQ2_EPS, IQ2_PH); }
        for (int i = 0; i < N; i++) {
            q1[i] = cd(q12(q1[i].real()), q12(q1[i].imag()));
            q2[i] = cd(q12(q2[i].real()), q12(q2[i].imag()));
        }
        g2 = std::sqrt(pwr(q1, 0) / pwr(q2, 0));
        if (g2 > 32767.0 / 8192) g2 = 32767.0 / 8192;
        if (g2 < 0.125) g2 = 0.125;
        g2 = std::floor(g2 * 8192 + 0.5) / 8192;          // the Q3.13 value
        rx2n = g2 * g2 * pwr(q2, 0) / pwr(q1, 0);
    }

    // matched gain: one common scale so the ADC peak stays below 1850
    cvec x1(N), x2(N), jx1(N), jx2(N), nx1(N), nx2(N);
    for (int i = 0; i < N; i++) {
        x1[i] = J[i] + n1[i] + s1[i];  x2[i] = J2[i] + n2[i] + s2[i];
        jx1[i] = J[i]; jx2[i] = J2[i]; nx1[i] = n1[i]; nx2[i] = n2[i];
    }
    if (sc.iq) {      // the imbalance is linear in I and Q, so per component
        iq_imbalance(x1, IQ1_EPS, IQ1_PH);  iq_imbalance(x2, IQ2_EPS, IQ2_PH);
        iq_imbalance(jx1, IQ1_EPS, IQ1_PH); iq_imbalance(jx2, IQ2_EPS, IQ2_PH);
        iq_imbalance(nx1, IQ1_EPS, IQ1_PH); iq_imbalance(nx2, IQ2_EPS, IQ2_PH);
    }
    double pk = 0;
    for (int i = 0; i < N; i++)
        pk = std::max(pk, std::max(std::max(std::fabs(x1[i].real()), std::fabs(x1[i].imag())),
                                   std::max(std::fabs(x2[i].real()), std::fabs(x2[i].imag()))));
    double g = 1.0;
    while (pk * g > 1850) g *= std::pow(10, -1.0 / 20);
    double cut_db = -20 * std::log10(g);
    for (int i = 0; i < N; i++) {
        J[i] *= g; n1[i] *= g; s1[i] *= g;
        jx1[i] *= g; jx2[i] *= g; nx1[i] *= g; nx2[i] *= g;
        x1[i] = cd(q12((x1[i] * g).real()), q12((x1[i] * g).imag()));
        x2[i] = cd(q12((x2[i] * g).real()), q12((x2[i] * g).imag()));
    }

    // firmware solver on three consecutive DMA-format captures
    auto solve = [&](int mode, double gn, aj_result_t &r) {
        static uint32_t buf[2 * AJ_NSAMP];
        std::memset(&r, 0, sizeof r);
        r.mode = mode; r.sig2_prev = -1; r.noise_ref = pn * g * g + 0.17; r.g2 = gn;
        r.rx2_noise = (gn == g2) ? rx2n : rx2n / (g2 * g2);
        aj_reset();
        for (int b = 0; b < 3; b++) {
            for (int t = 0; t < AJ_NSAMP; t++) {
                cd a = x1[b * AJ_NSAMP + t], c = x2[b * AJ_NSAMP + t];
                buf[2 * t]     = (uint32_t)(uint16_t)(int16_t)a.real() | ((uint32_t)(uint16_t)(int16_t)a.imag() << 16);
                buf[2 * t + 1] = (uint32_t)(uint16_t)(int16_t)c.real() | ((uint32_t)(uint16_t)(int16_t)c.imag() << 16);
            }
            if (aj_compute(buf, AJ_NSAMP, &r) != 0 || !r.ok) return false;
            r.sig2_prev = r.sig2;
        }
        return true;
    };
    // evaluation segment after the training captures
    const int e0 = 3 * AJ_NSAMP;
    auto seg = [&](const cvec &v) { return cvec(v.begin() + e0, v.end()); };
    struct Meas { double supp, loss, out_db, res; };
    auto measure = [&](const aj_result_t &r, double gn, int shift) {
        Meas m;
        double sc4 = std::pow(4.0, shift);
        set_weights(r.wre, r.wim);
        set_g2((int)std::floor(gn * 8192 + 0.5), 0);
        set_cfg(0, shift);
        cvec yj = run_core_c(seg(jx1), seg(jx2));
        cvec yx = run_core_c(seg(x1), seg(x2));
        cvec yn = run_core_c(seg(nx1), seg(nx2));
        m.supp = (sc.jnr_db > -90) ? 10 * std::log10(pwr(seg(J), 0) / std::max(pwr(yj, SKIP) / sc4, 1e-9)) : 0;
        // Satellite SNR by correlation with the known satellite waveform, on
        // the real (quantised, dithered by the noise) output, against RX1
        // with no jammer at all. The projection onto the waveform is the
        // satellite power, the rest is noise + residual jammer.
        cvec ref = seg(s1);
        auto snr = [&](const cvec &y, int dly) {
            cd c = 0; double pr = 0, py = 0; int cnt = 0;
            for (size_t i = SKIP; i < y.size(); i++) {
                cd rr = ref[i - dly];
                c += y[i] * std::conj(rr); pr += std::norm(rr); py += std::norm(y[i]); cnt++;
            }
            double ps = std::norm(c) / pr;
            return (ps / cnt) / (py / cnt - ps / cnt);
        };
        cvec clean(ref.size()), n1s = seg(n1);
        for (size_t i = 0; i < ref.size(); i++) clean[i] = n1s[i] / g + ref[i] / g;
        m.loss = 10 * std::log10(snr(clean, 0) / snr(yx, LAT));
        m.out_db = 10 * std::log10(pwr(yx, SKIP) / sc4 / pwr(seg(x1), 0));
        // residual: output with jammer / output of the same weights on the
        // noise alone (0 dB = jammer fully removed)
        m.res = 10 * std::log10(pwr(yx, SKIP) / std::max(pwr(yn, SKIP), 1e-9));
        return m;
    };
    auto shift_for = [&](const aj_result_t &r) {
        // the firmware's rule: 6 dB per step of RX gain cut, only while nulled
        int s = (int)(cut_db / 6.0);
        if (s > SFAP_MAXSH) s = SFAP_MAXSH;
        if (r.res_db > 3.0) s = 0;
        return s;
    };

    std::printf("scenario %d: JNR %.0f dB%s%s, RX gain cut %.1f dB", id, sc.jnr_db,
                sc.cw ? (sc.two ? " TWO CW" : " CW") : " wideband", sc.echo > 0 ? " + reflection" : "", cut_db);
    if (sc.rx2_db != 0) std::printf(", RX2 %+.0f dB (g2 %.3f = %+.1f dB)", sc.rx2_db, g2, 20 * std::log10(g2));
    if (sc.iq) std::printf(", I/Q image rejection %.1f / %.1f dB", irr_db(IQ1_EPS, IQ1_PH), irr_db(IQ2_EPS, IQ2_PH));
    std::printf("\n");

    if (id == 3) {
        // bypass: exactly x1[n-LAT]
        set_cfg(1, 0); set_g2((int)(g2 * 8192 + 0.5), 0);
        cvec yb = run_core_c(seg(x1), seg(x2));
        cvec xs = seg(x1);
        int bad = 0;
        for (size_t i = LAT; i < yb.size(); i++) if (yb[i] != xs[i - LAT]) bad++;
        std::printf("  bypass           : %d of %d samples differ from x1[n-%d]\n", bad, (int)yb.size() - LAT, LAT);
        if (bad) return 0;
    }

    aj_result_t r, r2;
    if (!solve(AJ_MODE_SFAP, g2, r)) { std::printf("solver failed\n"); return 0; }
    int sh = (id >= 6) ? shift_for(r) : 0;
    Meas m = measure(r, g2, sh);
    std::printf("  SFAP %d bins b+c1+c2: suppression %.1f dB, residual %+.2f dB, satellite SNR loss %.2f dB, output %+.2f dB re RX1%s\n", SFAP_N,
                m.supp, m.res, m.loss, m.out_db, sh ? "" : "");
    if (sh) std::printf("  output make-up    : %d (x%d, +%d dB)\n", sh, 1 << sh, 6 * sh);
    std::printf("  firmware figures  : J/N %.1f dB, suppression %.1f dB, residual %+.1f dB, mean |w|^2 %.2f, loading %.1f LSB^2 (%d passes), %d jammed bins\n",
                r.jn_db, r.supp_db, r.res_db, r.wn2, r.sig2, r.iters, r.jam_bins);
    std::printf("  detection         : coherence %.3f at lag %d -> J/N %.1f dB, RX1 %.0f RX2 %.0f LSB^2\n",
                r.coh, r.coh_lag, r.jn_coh_db, r.p1, r.p2);
    if (!sc.cw && sc.jnr_db > -90)
        std::printf("  jammer estimate   : bandwidth %.2f fs, centre %+.3f fs\n", r.bw, r.foff);
    if (sc.cw)
        std::printf("  jammer estimate   : bandwidth %.4f fs, centre %+.4f fs (true 0.06)\n", r.bw, r.foff);

    int ok = 1;
    switch (id) {
    case 2: {
        if (!solve(AJ_MODE_NB, g2, r2)) return 0;
        Meas m2 = measure(r2, g2, 0);
        std::printf("  one weight (v3)   : suppression %.1f dB, satellite SNR loss %.2f dB\n", m2.supp, m2.loss);
        ok = m.supp >= 35 && m.loss <= 3 && m.supp >= m2.supp + 10;
        break; }
    case 3: ok = std::fabs(m.out_db) <= 0.5 && std::fabs(m.loss) <= 0.5 && r.jn_db < 3 && r.jn_coh_db < -6; break;
    case 4: ok = m.supp >= 40 && m.loss <= 3; break;
    case 5: ok = m.supp >= 30 && m.loss <= 3; break;
    case 6: {
        if (!solve(AJ_MODE_LIN, g2, r2)) return 0;
        Meas m2 = measure(r2, g2, shift_for(r2));
        Meas m0 = measure(r, g2, 0);
        std::printf("  without image taps: suppression %.1f dB, residual %+.2f dB, satellite SNR loss %.2f dB\n", m2.supp, m2.res, m2.loss);
        std::printf("  without make-up   : satellite SNR loss %.2f dB\n", m0.loss);
        ok = m.res <= 1.5 && m2.res - m.res >= 2 && m.supp >= 45 && m.loss <= 2;
        break; }
    case 7: {
        if (!solve(AJ_MODE_SFAP, 1.0, r2)) return 0;
        Meas m2 = measure(r2, 1.0, 0);
        int k, sat = 0;
        double wmax = 0, wmax1 = 0;
        for (k = 0; k < AJ_NW; k++) {
            if (std::abs(r2.wre[k]) >= 32767 || std::abs(r2.wim[k]) >= 32767) sat++;
            wmax  = std::max(wmax,  std::hypot((double)r2.wre[k], (double)r2.wim[k]) / 4096);
            wmax1 = std::max(wmax1, std::hypot((double)r.wre[k],  (double)r.wim[k])  / 4096);
        }
        std::printf("  largest |w|       : %.2f with normalisation\n", wmax1);
        std::printf("  no normalisation  : suppression %.1f dB, residual %+.2f dB, largest |w| %.2f, %d weight components at the Q3.12 limit\n",
                    m2.supp, m2.res, wmax, sat);
        ok = m.supp >= 35 && m.loss <= 3 && m.supp >= m2.supp + 10;
        break; }
    case 8: ok = m.supp >= 50; break;
    case 9: {
        if (!solve(AJ_MODE_NB, g2, r2)) return 0;
        Meas m2 = measure(r2, g2, 0);
        std::printf("  one weight (v3)   : suppression %.1f dB, satellite SNR loss %.2f dB\n", m2.supp, m2.loss);
        // 5 dB: in the bins a jammer occupies, two antennas leave the
        // satellite the power-inversion pattern of that jammer's null (and
        // +3 dB of noise); with two jammers that is most of the band here.
        ok = m.supp >= 35 && m.loss <= 5 && m.supp >= m2.supp + 20;
        break; }
    }
    return ok;
}

int main(int argc, char **argv) {
    int scen = (argc > 1) ? std::atoi(argv[1]) : 2;
    int ok;
    // crpa6: the solver's averaging factor is adaptive by default in the firmware; the
    // scenarios are defined (and their limits set) with the fixed factor of SFAP-PI v5.0
    aj_set_av(AJ_AVG);
    switch (scen) {
    case 1: ok = test_accuracy(); break;
    case 2: ok = test_jammer(2, {30, false, 0, 0, false}); break;
    case 3: ok = test_jammer(3, {-99, false, 0, 0, false}); break;
    case 4: ok = test_jammer(4, {40, true, 0, 0, false}); break;
    case 5: ok = test_jammer(5, {30, false, 0.5, 0, false}); break;
    case 6: ok = test_jammer(6, {50, false, 0, 0, true}); break;
    case 7: ok = test_jammer(7, {40, false, 0, -25, false}); break;
    case 8: ok = test_jammer(8, {60, false, 0, 0, true}); break;
    case 9: ok = test_jammer(9, {40, true, 0, 0, false, true}); break;
    default: std::printf("unknown scenario %d\n", scen); ok = 0;
    }
    std::printf("SFAP_PI_CSIM: %s (scenario %d)\n", ok ? "PASS" : "FAIL", scen);
    return ok ? 0 : 1;
}
