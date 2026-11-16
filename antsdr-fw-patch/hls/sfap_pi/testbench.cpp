// ============================================================================
//  testbench.cpp  --  C simulation of the SFAP-PI core (v6.0, 256 bins)
//
//  The weights come from the FIRMWARE's own solver
//  (Source/Firmware/app_sfap_lvds/gnss_aj_math.c, compiled into this
//  testbench), run on AJ_NSAMP (8192)-sample captures packed exactly like the
//  DMA buffer, so this tests the board's weight computation and the FPGA
//  filter together. Scenarios 11..15 (v6.1) run SEQUENCES of captures.
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
//    3  no jammer: bypass output is exactly x1[n-SFAP_LAT]; with the filter in,
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
//   10  CW jammer 30 dB exactly ON the satellite's centre frequency, with the
//       satellite arriving through the SAME RX2/RX1 path as the jammer (the
//       broadside / same-direction case): the null takes the satellite's
//       centre bins with it, so the loss measures the notch width.
//       The notch is as wide as the CW's window leakage stays above the
//       noise (about +-3 bins at 30 dB). Pass: suppression >= 35 dB,
//       satellite loss <= 10 dB (v6.2 measures ~4.6 dB with the exact
//       satellite-loss method; v6.0's correlation estimate read ~8 dB). At a
//       stronger CW the notch widens: ~16 dB loss at 50-60 dB J/N, with
//       any window -- a satellite in the jammer's direction AND band is
//       mostly lost with two antennas.
//  11..15: v6.1, see the block above test_seq().
//  16..18: v6.2 (obstacle, moving channel, multipath), see test_change().
//  v6.2: the satellite loss is measured exactly in every scenario (the
//  satellite alone and everything else alone through the core).
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
static ap_uint<32>   g_wb[SFAP_N], g_wc1[SFAP_N], g_wc2[SFAP_N];   // weight RAM (committed bank)
static ap_uint<32>   g_cfg = SFAP_CFG_BYPASS;
static ap_uint<32>   g_nrm = 1 << SFAP_GFRAC;
static const int     LAT = SFAP_LAT, SKIP = SFAP_LAT + 2 * SFAP_N;
static long long     g_calls = 0;          // calls of the core so far (its frame clock)

static void set_weights(const int16_t *wre, const int16_t *wim) {
    auto word = [](int16_t re, int16_t im) {
        ap_uint<32> w = 0;
        w(15, 0)  = (ap_uint<16>)(ap_int<16>)re;
        w(31, 16) = (ap_uint<16>)(ap_int<16>)im;
        return w;
    };
    for (int k = 0; k < SFAP_N; k++) {
        g_wb[k]  = word(wre[AJ_WB + k],  wim[AJ_WB + k]);
        g_wc1[k] = word(wre[AJ_WC1 + k], wim[AJ_WC1 + k]);
        g_wc2[k] = word(wre[AJ_WC2 + k], wim[AJ_WC2 + k]);
    }
}

static void set_cfg(int bypass, int shift) {
    g_cfg = (bypass ? SFAP_CFG_BYPASS : 0u) | ((unsigned)(shift & 7) << SFAP_CFG_SH_LSB);
}

static void set_g2(int g2r, int g2i) {
    g_nrm = 0;
    g_nrm(15, 0)  = (ap_uint<16>)(ap_int<16>)g2r;
    g_nrm(31, 16) = (ap_uint<16>)(ap_int<16>)g2i;
}

static void push(hls::stream<ap_uint<64> > &xs, hls::stream<ap_uint<32> > &ys,
                 int a, int b, int c, int d, cd *out) {
    int v[4] = {a, b, c, d};
    ap_uint<64> w = 0;
    for (int k = 0; k < 4; k++) w(16 * k + 15, 16 * k) = (ap_uint<16>)(ap_int<16>)v[k];
    xs.write(w);
    sfap_pi(xs, ys, g_wb, g_wc1, g_wc2, g_cfg, g_nrm);
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
struct Scen { double jnr_db; bool cw; double echo; double rx2_db; bool iq; bool two = false; bool onsat = false; };

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
        const double fcw = sc.onsat ? fsat : 0.06;     // scenario 10: on the satellite
        for (int i = 0; i < N; i++) J[i] = std::polar(1.0, 2 * M_PI * fcw * i);
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
    if (sc.onsat) {     // the satellite comes the jammer's way (same RX2/RX1 path)
        s2 = fir(fir(s1, h2, 10), hd, 1);
        for (auto &v : s2) v *= g2p;
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
    cvec sq1 = s1, sq2 = s2;                   // the satellite alone, as received
    if (sc.iq) { iq_imbalance(sq1, IQ1_EPS, IQ1_PH); iq_imbalance(sq2, IQ2_EPS, IQ2_PH); }
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
        J[i] *= g; n1[i] *= g; s1[i] *= g; sq1[i] *= g; sq2[i] *= g;
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
        // v6.2: satellite SNR loss measured deterministically -- the
        // satellite alone (x100; the filter is linear) and everything else
        // alone through the core, against satellite / noise at RX1 with no
        // jammer. (v6.0 correlated the noisy output with the satellite,
        // which scattered by ~+-0.5 dB.)
        cvec sa1 = seg(sq1), sa2 = seg(sq2), na1 = seg(x1), na2 = seg(x2);
        for (size_t i = 0; i < sa1.size(); i++) {
            na1[i] -= sa1[i]; na2[i] -= sa2[i];
            sa1[i] *= 100.0; sa2[i] *= 100.0;
        }
        cvec ys = run_core_c(sa1, sa2), ynj = run_core_c(na1, na2);
        double s_in = pwr(seg(s1), 0), n_in = pwr(seg(n1), 0);     // both scaled by g
        m.loss = 10 * std::log10((s_in / n_in) / ((pwr(ys, SKIP) / 1e4) / pwr(ynj, SKIP)));
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
    std::printf("  SFAP %d bins b+c1+c2: suppression %.1f dB, residual %+.2f dB, satellite SNR loss %.2f dB, output %+.2f dB re RX1%s\n",
                SFAP_N, m.supp, m.res, m.loss, m.out_db, sh ? "" : "");
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
    case 10:
        std::printf("  notch             : bin width %.3f MHz at 61.44 MS/s; satellite loss = the\n"
                    "                      part of its spectrum inside the CW notch\n", 61.44 / SFAP_N);
        ok = m.supp >= 35 && m.loss <= 10;
        break;
    }
    return ok;
}


// ============================================================================
//  v6.1 scenarios 11..15: SEQUENCES of captures through the firmware solver,
//  as the engine runs them (adaptive averaging, weight hold), then the core
//  with the last committed weights. Each compares v6.1 with the v6.0
//  behaviour (averaging 0.5, no hold) in the same run.
//    11  intermittent wideband jammer 40 dB: on for 3 captures, off for 15
//        (~0.4 s, inside the 0.5 s jammer-OFF delay), then back: must be
//        nulled at once (v6.0: the gated bins had dropped to zero weight)
//    12  hopping narrowband jammer 40 dB, 16 channels, one hop per capture;
//        evaluated on the channel it used 12 captures before the last solve
//    13  per-bin detector: no false alarm in 300 noise-only captures (white,
//        and the 8 MHz profile's band-limited noise); a CW 10 dB under the
//        noise is detected, which the v6.0 broadband detector misses
//    14  steady wideband jammer 40 dB, 12 captures: drift-aware averaging
//        must not lose against the fixed 0.5
//    15  SAFETY: jammer A for 4 captures, then jammer B 15 dB weaker in the
//        same channel from another direction: the held A weights must not
//        be used where they would leave B (residual check)
//  The satellite loss here is measured deterministically: the satellite
//  alone and noise + jammer alone through the core.
// ============================================================================
static void jam_path(const cvec &J, cvec &J2) {
    cvec h2 = frac_delay(0.15, 21);
    cvec hd = {cd(0.03, 0.01), 1.0, cd(-0.02, 0.015)};
    J2 = fir(fir(J, h2, 10), hd, 1);
    cd g2p = std::polar(std::pow(10, 0.8 / 20), 40 * M_PI / 180);
    for (auto &v : J2) v *= g2p;
}
static cvec wb_noise(int n) {                       // fills the band
    cvec J(n); for (auto &v : J) v = cd(grand(), grand());
    return fir(J, lowpass(0.42, 41), 20);
}
static cvec nb_noise(int n, double f, double hw) {  // centre f, half-width hw (fs)
    cvec J(n); for (auto &v : J) v = cd(grand(), grand());
    J = fir(J, lowpass(hw, 101), 50);
    for (int i = 0; i < n; i++) J[i] *= std::polar(1.0, 2 * M_PI * f * i);
    return J;
}
// The engine's (gnss_aj.c) v6.2 averaging policy and g2 rule are the
// firmware's own functions (v6.2.1: aj_avg_policy, aj_g2_follow in
// gnss_aj_math.c), not copies.
static double aj_policy(int nep, double res, double res_prev) {
    return aj_avg_policy(nep, res, res_prev);
}
// g2_floor = the engine's g2_noise. Returns 1 if g2 changed.
static int g2_rule(const aj_result_t &r, double &g2, double g2_floor) {
    double t;
    if (!aj_g2_follow(&r, g2, g2_floor, &t)) return 0;
    g2 = t;
    return 1;
}
struct Seg { cvec j1; int dir = 0; double amp = 1.0; };   // jammer at RX1 (unscaled),
                                                    // direction A (0) or B (1)
struct SeqOut { double supp, loss, res_last; int held; };

static SeqOut run_seq(const std::vector<Seg> &caps, const Seg &ev, double jnr_db,
                      double pref, bool adapt, int hold) {
    const double nrms = 20.0, pn = 2 * nrms * nrms, fsat = -9.08 / 50.0;
    const int NE = 32000;
    double js = std::sqrt(pn * std::pow(10, jnr_db / 10) / pref);
    unsigned long long seed = rng_s;               // same noise for every call
    // all segments: x = J + n + s at RX1, path(J) + n2 + s2 at RX2
    auto build = [&](const Seg &sg, int len, cvec &x1, cvec &x2, cvec &jx1, cvec &jx2,
                     cvec &n1, cvec &s1) {
        cvec j1 = sg.j1, j2; jam_path(j1, j2);
        if (sg.dir == 1)   // direction B: another RX2/RX1 ratio, -1.5 dB / -110 deg
            for (auto &v : j2) v *= std::polar(std::pow(10, -2.3 / 20), -150 * M_PI / 180);
        for (auto &v : j1) v *= sg.amp;
        for (auto &v : j2) v *= sg.amp;
        x1.assign(len, 0.0); x2 = x1; jx1 = x1; jx2 = x1; n1 = x1; s1 = x1;
        int chip = 1; double cph = 0;
        for (int i = 0; i < len; i++) {
            cd a = cd(grand(), grand()) * nrms, b = cd(grand(), grand()) * nrms;
            cph += 1.023 / 50.0; if (cph >= 1) { cph -= 1; chip = urand() < 0.5 ? -1 : 1; }
            cd s = std::polar(0.1 * nrms * chip, 2 * M_PI * fsat * i);
            jx1[i] = js * j1[i]; jx2[i] = js * j2[i]; n1[i] = a; s1[i] = s;
            x1[i] = jx1[i] + a + s;
            x2[i] = jx2[i] + b + s * std::polar(1.0, -1.1);
        }
    };
    rng_s = seed;
    std::vector<cvec> cx1(caps.size()), cx2(caps.size());
    double pk = 0;
    for (size_t c = 0; c < caps.size(); c++) {
        cvec a, b, d, e;
        build(caps[c], AJ_NSAMP, cx1[c], cx2[c], a, b, d, e);
    }
    cvec ex1, ex2, ej1, ej2, en1, es1;
    build(ev, NE, ex1, ex2, ej1, ej2, en1, es1);
    auto peak = [&](const cvec &v) { for (auto &q : v) pk = std::max(pk, std::max(std::fabs(q.real()), std::fabs(q.imag()))); };
    for (size_t c = 0; c < caps.size(); c++) { peak(cx1[c]); peak(cx2[c]); }
    peak(ex1); peak(ex2);
    double g = 1.0;
    while (pk * g > 1850) g *= std::pow(10, -1.0 / 20);
    // the firmware solver, capture after capture, as the engine calls it
    static uint32_t buf[2 * AJ_NSAMP];
    static aj_result_t r;
    std::memset(&r, 0, sizeof r);
    aj_reset(); aj_set_hold(hold);
    r.mode = AJ_MODE_SFAP; r.sig2_prev = -1; r.noise_ref = pn * g * g + 0.17;
    r.g2 = 1.0; r.rx2_noise = 1.0;
    int nep = 0; double res_prev = 99;
    for (size_t c = 0; c < caps.size(); c++) {
        for (int t = 0; t < AJ_NSAMP; t++) {
            cd a = cx1[c][t] * g, b = cx2[c][t] * g;
            buf[2 * t]     = (uint32_t)(uint16_t)(int16_t)q12(a.real()) | ((uint32_t)(uint16_t)(int16_t)q12(a.imag()) << 16);
            buf[2 * t + 1] = (uint32_t)(uint16_t)(int16_t)q12(b.real()) | ((uint32_t)(uint16_t)(int16_t)q12(b.imag()) << 16);
        }
        r.avg = !adapt ? 0.0 : aj_policy(nep, r.res_db, res_prev);
        res_prev = (nep > 0) ? r.res_db : 99.0;
        nep++;
        if (aj_compute(buf, AJ_NSAMP, &r) != 0 || !r.ok) { std::printf("solver failed\n"); std::exit(1); }
        r.sig2_prev = r.sig2;
    }
    // the core with those weights on the evaluation segment
    SeqOut o; o.res_last = r.res_db; o.held = r.held_bins;
    set_weights(r.wre, r.wim); set_g2(8192, 0); set_cfg(0, 0);
    for (auto &v : ex1) v *= g; for (auto &v : ex2) v *= g;
    for (auto &v : ej1) v *= g; for (auto &v : ej2) v *= g;
    cvec yj = run_core_c(ej1, ej2);
    cvec yx = run_core_c(ex1, ex2);
    o.supp = 10 * std::log10(pwr(ej1, 0) / std::max(pwr(yj, SKIP), 1e-9));
    // Satellite SNR loss, measured deterministically: the satellite alone
    // (x100, the filter is linear) and noise + jammer alone through the core,
    // against the satellite / noise ratio at RX1 with no jammer.
    cvec sx1(NE), sx2(NE), nx1(NE), nx2(NE);
    for (int i = 0; i < NE; i++) {
        sx1[i] = 100.0 * g * es1[i]; sx2[i] = 100.0 * g * es1[i] * std::polar(1.0, -1.1);
        nx1[i] = ex1[i] - g * es1[i]; nx2[i] = ex2[i] - g * es1[i] * std::polar(1.0, -1.1);
    }
    cvec ys = run_core_c(sx1, sx2), yn = run_core_c(nx1, nx2);
    double s_in = pwr(es1, 0), n_in = pwr(en1, 0);
    double s_out = pwr(ys, SKIP) / 1e4 / (g * g), n_out = pwr(yn, SKIP) / (g * g);
    o.loss = 10 * std::log10((s_in / n_in) / (s_out / n_out));
    (void)yx;
    return o;
}

static int test_seq(int id) {
    const double JNR = 40;
    std::vector<Seg> caps; Seg ev;
    double pref = 0;
    if (id == 11) {                   // intermittent: on x3, then 15 captures (~0.4 s) off
        for (int c = 0; c < 18; c++) caps.push_back({c < 3 ? wb_noise(AJ_NSAMP) : cvec(AJ_NSAMP, 0.0)});
        ev.j1 = wb_noise(32000);
        pref = pwr(ev.j1, 0);
        std::printf("scenario 11: INTERMITTENT wideband jammer %.0f dB: on for 3 captures, off for the next 15\n"
                    "             (~0.4 s, less than the 0.5 s jammer-OFF delay), then back\n", JNR);
    } else if (id == 12) {             // hopping: channels 0..7, one per capture
        const double hw = 0.010;      // ~1.2 MHz wide at 61.44 MS/s
        for (int h = 0; h < 16; h++) caps.push_back({nb_noise(AJ_NSAMP, -0.375 + 0.05 * h, hw)});
        ev.j1 = nb_noise(32000, -0.375 + 0.05 * 3, hw); // channel 3, seen 12 captures ago
        pref = pwr(ev.j1, 0);
        std::printf("scenario %d: HOPPING narrowband jammer %.0f dB, 16 channels, 1 hop per capture;\n"
                    "             evaluated on the channel it used 12 captures before the last solve\n", id, JNR);
    } else if (id == 15) {            // direction change in the held bins
        const double hw = 0.010, f = -0.225, b = std::pow(10, -15.0 / 20);
        for (int c = 0; c < 4; c++) caps.push_back({nb_noise(AJ_NSAMP, f, hw), 0, 1.0});
        for (int c = 0; c < 8; c++) caps.push_back({nb_noise(AJ_NSAMP, f, hw), 1, b});
        ev.j1 = nb_noise(32000, f, hw); ev.dir = 1; ev.amp = b;
        pref = pwr(ev.j1, 0) * b * b;
        std::printf("scenario 15: jammer A %.0f dB in one channel for 4 captures, then jammer B 15 dB weaker,\n"
                    "             SAME channel, OTHER direction, for 8 captures: the held A weights must not be used\n", JNR);
    } else {                          // 14: steady
        for (int c = 0; c < 12; c++) caps.push_back({wb_noise(AJ_NSAMP)});
        ev.j1 = wb_noise(32000);
        pref = pwr(ev.j1, 0);
        std::printf("scenario 14: STEADY wideband jammer %.0f dB, 12 captures\n", JNR);
    }
    int ok = 1;
    if (id == 14) {
        SeqOut a = run_seq(caps, ev, JNR, pref, false, AJ_HOLD_SOLVES);
        SeqOut b = run_seq(caps, ev, JNR, pref, true,  AJ_HOLD_SOLVES);
        std::printf("  fixed avg 0.5 (v6.0)       : suppression %.1f dB, satellite SNR loss %.2f dB\n", a.supp, a.loss);
        std::printf("  drift-aware avg (v6.2)     : suppression %.1f dB, satellite SNR loss %.2f dB\n", b.supp, b.loss);
        ok = b.supp >= 35 && b.loss <= a.loss + 0.3;
    } else {
        SeqOut a = run_seq(caps, ev, JNR, pref, false, 0);
        SeqOut c = run_seq(caps, ev, JNR, pref, true, 0);
        SeqOut b = run_seq(caps, ev, JNR, pref, true, AJ_HOLD_SOLVES);
        std::printf("  v6.0 (avg 0.5, no hold)           : suppression %5.1f dB, satellite SNR loss %5.2f dB\n", a.supp, a.loss);
        std::printf("  drift-aware avg, hold off         : suppression %5.1f dB, satellite SNR loss %5.2f dB\n", c.supp, c.loss);
        std::printf("  v6.2 drift-aware avg + hold %3d   : suppression %5.1f dB, satellite SNR loss %5.2f dB, %d bins on held weights\n",
                    AJ_HOLD_SOLVES, b.supp, b.loss, b.held);
        if (id == 15) ok = b.supp >= c.supp - 1.0 && b.supp >= 30;   // hold must not hurt
        else          ok = b.supp >= 35 && b.supp >= a.supp - 1.0 && b.loss <= a.loss + 1.0;
    }
    return ok;
}

// Scenario 13: the per-bin detector on its own.
static int test_detect_one(bool narrow) {
    const double nrms = 20.0, ON = 8.0, OFF = 5.0;
    const int NB = AJ_NSAMP;
    static uint32_t buf[2 * AJ_NSAMP];
    static aj_result_t r;
    // the 8 MHz profile at 30.72 MS/s: noise only in the middle quarter of
    // the band, the rest at the ADC's quantisation floor
    cvec hlp = narrow ? lowpass(0.13, 63) : cvec(1, 1.0);
    double hp = 0; for (auto &v : hlp) hp += std::norm(v);
    auto noise = [&](int n) {
        cvec v(n + 64); for (auto &q : v) q = cd(grand(), grand());
        v = fir(v, hlp, hlp.size() / 2);
        cvec o(v.begin() + 32, v.begin() + 32 + n);
        for (auto &q : o) q *= nrms / std::sqrt(hp);
        return o;
    };
    auto pack = [&](const cvec &a, const cvec &b) {
        for (int t = 0; t < NB; t++) {
            buf[2 * t]     = (uint32_t)(uint16_t)(int16_t)q12(a[t].real()) | ((uint32_t)(uint16_t)(int16_t)q12(a[t].imag()) << 16);
            buf[2 * t + 1] = (uint32_t)(uint16_t)(int16_t)q12(b[t].real()) | ((uint32_t)(uint16_t)(int16_t)q12(b[t].imag()) << 16);
        }
    };
    std::memset(&r, 0, sizeof r);
    aj_forget_bins();
    r.noise_ref = 2 * nrms * nrms + 0.17; r.gain_lin = 1.0;
    int n_learn = 0;
    while (!aj_bins_known()) {         // the engine learns while quiet
        pack(noise(NB), noise(NB)); aj_detect(buf, NB, &r);
        aj_learn_bins(1.0, 0.002, 0.05); n_learn++;
    }
    int fa = 0; double mx = -99;
    for (int c = 0; c < 300; c++) {
        pack(noise(NB), noise(NB)); aj_detect(buf, NB, &r);
        mx = std::max(mx, r.bin_jn_max);
        if (r.bin_jn_max >= ON) fa++;
    }
    std::printf("  %s: armed after %d quiet captures; 300 noise-only captures: %d false alarms,\n"
                "    largest bin J/N %.1f dB (ON at %.0f, OFF under %.0f)\n",
                narrow ? "8 MHz profile (noise in the middle quarter)" : "wide profile (white noise)", n_learn, fa, mx, ON, OFF);
    int ok = (fa == 0 && mx < OFF);
    double jl[] = {-25, -20, -15, -10, -5};
    for (double j : jl) {
        cvec a = noise(NB), b = noise(NB), J(NB), J2;
        double f = 0.0371;            // between bin centres: worst scalloping
        for (int i = 0; i < NB; i++) J[i] = std::polar(1.0, 2 * M_PI * f * i);
        jam_path(J, J2);
        double js = std::sqrt(2 * nrms * nrms * std::pow(10, j / 10));
        for (int i = 0; i < NB; i++) { a[i] += js * J[i]; b[i] += js * J2[i]; }
        pack(a, b); aj_detect(buf, NB, &r);
        int v60 = r.jn_coh_db >= -3.0 || r.jn_db >= 6.0;
        int v61 = v60 || r.bin_jn_max >= ON;
        std::printf("  CW at total J/N %+4.0f dB: coherence J/N %+6.1f, power J/N %+6.1f, largest bin J/N %+5.1f dB -> v6.0 %s, v6.1 %s\n",
                    j, r.jn_coh_db, r.jn_db, r.bin_jn_max, v60 ? "ON " : "off", v61 ? "ON" : "off");
        if (!narrow && j == -10 && !(v61 && !v60)) ok = 0;
    }
    return ok;
}

static int test_detect() {
    std::printf("scenario 13: per-bin narrowband detector (CW half-way between two bins: worst case)\n");
    int a = test_detect_one(false);
    int b = test_detect_one(true);
    return a && b;
}


// ============================================================================
//  v6.2 scenarios 16..18: a CHANGING channel (obstacles, multipath)
//    16  an obstacle shadows the jammer at RX2 only by 18 / 24 / 30 dB:
//        the RX2 weight would need |b| > 8 (the Q3.12 limit); v6.2 raises g2
//        towards the jammer ratio (and starts the loading at the noise floor)
//    17  the RX2/RX1 jammer ratio turns at 30 and 100 deg/s (a moving
//        reflector or obstacle). One solve per T; weights evaluated 1.5 T
//        after the last capture. v6.2: T = 8.8 ms (4096-sample captures, no
//        frame overlap: ~1/3 of v6.1's solve time on the board) with the
//        drift-aware averaging; reference: v6.1's 25 ms and its averaging
//    18  a reflection of the jammer, -6 dB, from another direction: 0.5 us
//        late is 31 samples at 61.44 MS/s but 8 at 15.36 MS/s (gnss_band=5)
// ============================================================================
struct Ch { double a1 = 1, a2 = 1; int dly = 0; double echo = 0; double rot = 0; };
static void chan(const cvec &J, const Ch &c, cvec &j1, cvec &j2) {
    cvec p; jam_path(J, p);
    j1 = J; j2 = p;
    for (auto &v : j1) v *= c.a1;
    for (auto &v : j2) v *= c.a2 * std::polar(1.0, c.rot * M_PI / 180);
    if (c.echo > 0) for (size_t i = c.dly; i < J.size(); i++) {
        j1[i] += c.echo * std::polar(1.0, 1.3) * J[i - c.dly];
        j2[i] += c.echo * std::polar(1.0, 2.9) * J[i - c.dly];
    }
}
struct ChOut { double supp, loss, g2; };
// ncap captures with the channel of cap(c), then the core on chan ev.
// boot = 1: the jammer was already there at start-up (v6.2.1 test), as the
// engine's enter_jammed() then runs it: no learned noise floor (noise_ref
// -1, no bin gating), g2 from the jammer ratio of the first capture (and
// g2_noise = that g2), rx2n = 1.0.
static ChOut run_ch(const std::vector<Ch> &cap, const Ch &ev, double JNR, int policy, bool g2rule,
                    int boot = 0) {
    const double nrms = 20, pn = 2 * nrms * nrms, fsat = -9.08 / 50.0;
    const int NE = 24000;
    static uint32_t buf[2 * AJ_NSAMP]; static aj_result_t r;
    auto sat = [&](int n, cvec &s1, cvec &s2) {
        s1.resize(n); s2.resize(n); int chip = 1; double cph = 0;
        for (int i = 0; i < n; i++) {
            cph += 1.023 / 50.0; if (cph >= 1) { cph -= 1; chip = urand() < 0.5 ? -1 : 1; }
            s1[i] = std::polar(0.1 * nrms * chip, 2 * M_PI * fsat * i); s2[i] = s1[i] * std::polar(1.0, -1.1);
        }
    };
    cvec J0 = wb_noise(AJ_NSAMP);
    double js = std::sqrt(pn * std::pow(10, JNR / 10) / pwr(J0, 0));
    // one gain for everything: the strongest channel's peak under 1850
    double g = 1.0;
    {
        double pk = 0;
        auto pkof = [&](const Ch &c) { cvec a, b; chan(J0, c, a, b);
            for (auto &q : a) pk = std::max({pk, js * std::fabs(q.real()), js * std::fabs(q.imag())});
            for (auto &q : b) pk = std::max({pk, js * std::fabs(q.real()), js * std::fabs(q.imag())}); };
        for (auto &c : cap) pkof(c);
        pkof(ev);
        while ((pk + 4 * nrms) * g > 1850) g *= std::pow(10, -1.0 / 20);
    }
    std::memset(&r, 0, sizeof r); aj_reset();
    double g2 = 1.0, g2_floor = 1.0;
    r.mode = AJ_MODE_SFAP; r.sig2_prev = -1; r.noise_ref = pn * g * g + 0.17; r.g2 = 1; r.rx2_noise = 1;
    if (boot) r.noise_ref = -1.0;
    int nep = 0; double res_prev = 99;
    for (size_t c = 0; c < cap.size(); c++) {
        if (g2rule && nep >= 1 && g2_rule(r, g2, g2_floor)) {
            aj_reset(); r.sig2_prev = -1; r.g2 = g2;
            // the engine: rx2n = g2^2 / ratio12 (ratio12 = 1 learned here;
            // at start-up ratio12 is the jammer ratio = g2_floor^2)
            r.rx2_noise = boot ? g2 * g2 / (g2_floor * g2_floor) : g2 * g2; nep = 0;
        }
        cvec J = wb_noise(AJ_NSAMP), j1, j2, s1, s2; chan(J, cap[c], j1, j2); sat(AJ_NSAMP, s1, s2);
        for (int t = 0; t < AJ_NSAMP; t++) {
            cd a = g * (js * j1[t] + cd(grand(), grand()) * nrms + s1[t]);
            cd b = g * (js * j2[t] + cd(grand(), grand()) * nrms + s2[t]);
            buf[2*t]   = (uint32_t)(uint16_t)(int16_t)q12(a.real()) | ((uint32_t)(uint16_t)(int16_t)q12(a.imag()) << 16);
            buf[2*t+1] = (uint32_t)(uint16_t)(int16_t)q12(b.real()) | ((uint32_t)(uint16_t)(int16_t)q12(b.imag()) << 16);
        }
        if (boot && c == 0) {      // enter_jammed(): ratio12 = p1/p2, update_g2(), rx2n = 1
            if (aj_detect(buf, AJ_NSAMP, &r) != 0) { std::printf("detect failed\n"); std::exit(1); }
            double t = std::sqrt(r.p1 / r.p2);
            t = std::min(std::max(t, AJ_G2_MIN), AJ_G2_MAX);
            g2 = g2_floor = std::floor(t * 8192 + 0.5) / 8192;
            r.g2 = g2; r.rx2_noise = 1.0;
        }
        r.avg = policy == 1 ? (nep < 3 ? 0.0 : (r.res_db <= 1 ? 0.8 : (r.res_db > 3 ? 0.25 : 0.0)))   // v6.1
                            : aj_policy(nep, r.res_db, res_prev);                                       // v6.2
        res_prev = (nep > 0) ? r.res_db : 99.0;
        nep++;
        if (aj_compute(buf, AJ_NSAMP, &r) != 0 || !r.ok) { std::printf("solver failed\n"); std::exit(1); }
        r.sig2_prev = r.sig2;
    }
    set_weights(r.wre, r.wim); set_g2((int)std::floor(g2 * 8192 + 0.5), 0); set_cfg(0, 0);
    cvec J = wb_noise(NE), j1, j2, s1, s2, n1(NE), n2(NE); chan(J, ev, j1, j2); sat(NE, s1, s2);
    cvec jx1(NE), jx2(NE), nx1(NE), nx2(NE), sx1(NE), sx2(NE);
    for (int i = 0; i < NE; i++) {
        n1[i] = cd(grand(), grand()) * nrms; n2[i] = cd(grand(), grand()) * nrms;
        jx1[i] = g * js * j1[i]; jx2[i] = g * js * j2[i];
        nx1[i] = jx1[i] + g * n1[i]; nx2[i] = jx2[i] + g * n2[i];
        sx1[i] = 100 * g * s1[i]; sx2[i] = 100 * g * s2[i];
    }
    cvec yj = run_core_c(jx1, jx2), yn = run_core_c(nx1, nx2), ys = run_core_c(sx1, sx2);
    ChOut o;
    o.supp = 10 * std::log10(pwr(jx1, 0) / std::max(pwr(yj, SKIP), 1e-9));
    o.loss = 10 * std::log10((pwr(s1, 0) / pwr(n1, 0)) / ((pwr(ys, SKIP) / 1e4) / pwr(yn, SKIP)));
    o.g2 = g2;
    return o;
}

static int test_change(int id) {
    int ok = 1;
    if (id == 16) {
        std::printf("scenario 16: OBSTACLE shadowing the jammer at RX2 only (J/N 40 dB at RX1)\n");
        double at[] = {12, 18, 24, 30};
        for (double a : at) {
            Ch c; c.a2 = std::pow(10, -a / 20);
            std::vector<Ch> caps(6, c);
            ChOut o0 = run_ch(caps, c, 40, 2, false), o1 = run_ch(caps, c, 40, 2, true);
            std::printf("  RX2 shadowed %2.0f dB: g2 fixed %5.1f dB supp., sat. loss %5.1f dB | v6.2 g2 %.2f: %5.1f dB, loss %5.1f dB\n",
                        a, o0.supp, o0.loss, o1.g2, o1.supp, o1.loss);
            if (a <= 24 && !(o1.supp >= 20 && o1.loss <= 2.0)) ok = 0;
            if (a == 30 && !(o1.loss <= 3.0)) ok = 0;
        }
        // v6.2.1: the same, but the jammer was there at START-UP: no learned
        // noise floor, g2 from the jammer ratio, rx2n = 1 (enter_jammed()).
        // The v6.2 loading start (the learned floor) does not apply here.
        std::printf("  jammer present at start-up (no noise floor learned, g2 from the jammer, rx2n = 1):\n");
        double bt[] = {0, 12, 18, 24, 30};
        for (double a : bt) {
            Ch c; c.a2 = std::pow(10, -a / 20);
            std::vector<Ch> caps(6, c);
            ChOut o = run_ch(caps, c, 40, 2, true, 1);
            std::printf("  RX2 shadowed %2.0f dB: g2 %.2f: %5.1f dB supp., sat. loss %5.1f dB\n",
                        a, o.g2, o.supp, o.loss);
            if (a <= 24 && !(o.supp >= 20 && o.loss <= 2.0)) ok = 0;
            if (a == 30 && !(o.loss <= 3.0)) ok = 0;
        }
    } else if (id == 17) {
        std::printf("scenario 17: MOVING CHANNEL, the RX2/RX1 jammer ratio turns (J/N 40 dB)\n");
        double Rs[] = {30, 100};
        for (double R : Rs) {
            auto seq = [&](double T, int pol) {
                std::vector<Ch> caps; for (int c = 0; c < 12; c++) { Ch ch; ch.rot = R * T / 1000 * c; caps.push_back(ch); }
                Ch ev; ev.rot = R * T / 1000 * (11 + 1.5);
                return run_ch(caps, ev, 40, pol, true);
            };
            ChOut a = seq(25.0, 1), b = seq(8.8, 2);
            std::printf("  %3.0f deg/s: v6.1 timing (25 ms, its averaging) %5.1f dB, sat. loss %5.1f dB | v6.2 (8.8 ms, drift-aware) %5.1f dB, loss %5.1f dB\n",
                        R, a.supp, a.loss, b.supp, b.loss);
            if (!(b.loss <= a.loss - 3.0)) ok = 0;
        }
    } else {
        std::printf("scenario 18: MULTIPATH, reflection -6 dB from another direction (J/N 40 dB)\n");
        int dl[] = {4, 8, 16, 31};
        ChOut o8{}, o31{};
        for (int d : dl) {
            Ch c; c.echo = 0.5; c.dly = d;
            std::vector<Ch> caps(6, c);
            ChOut o = run_ch(caps, c, 40, 2, true);
            std::printf("  delay %2d samples = %.2f us at 61.44 MS/s, %.2f us at 15.36 MS/s: suppression %5.1f dB, sat. loss %5.1f dB\n",
                        d, d / 61.44, d / 15.36, o.supp, o.loss);
            if (d == 8) o8 = o;
            if (d == 31) o31 = o;
        }
        // the same 0.5 us reflection: 8 samples at 15.36 MS/s, 31 at 61.44
        std::printf("  a 0.5 us reflection: satellite loss %.1f dB at 61.44 MS/s, %.1f dB with gnss_band=5 (15.36 MS/s)\n",
                    o31.loss, o8.loss);
        ok = o8.supp >= 30 && o8.loss <= o31.loss - 10.0;
    }
    return ok;
}

int main(int argc, char **argv) {
    int scen = (argc > 1) ? std::atoi(argv[1]) : 2;
    int ok;
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
    case 10: ok = test_jammer(10, {30, true, 0, 0, false, false, true}); break;
    case 11: case 12: case 14: case 15: ok = test_seq(scen); break;
    case 13: ok = test_detect(); break;
    case 16: case 17: case 18: ok = test_change(scen); break;
    default: std::printf("unknown scenario %d\n", scen); ok = 0;
    }
    std::printf("SFAP_PI_CSIM: %s (scenario %d)\n", ok ? "PASS" : "FAIL", scen);
    return ok ? 0 : 1;
}
