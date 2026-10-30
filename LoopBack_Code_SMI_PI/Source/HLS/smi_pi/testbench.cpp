// ============================================================================
//  testbench.cpp  --  C simulation of the space-time SMI-PI core (v4.1)
//
//  The weights come from the FIRMWARE's own solver
//  (Source/Firmware/app_gnss_e310/gnss_aj_math.c, compiled into this
//  testbench), run on 4096-sample captures packed exactly like the DMA
//  buffer, so this tests the board's weight computation and the FPGA filter
//  together.
//
//  Signals at the raw 12-bit ADC scale, noise 20 LSB rms per I/Q component:
//      RX1 = IQ1( J + s + n1 )
//      RX2 = IQ2( a (H(J) + g s + n2) )
//  J: complex Gaussian jammer low-passed to 0.84 fs (it fills the band like
//     the 55 MHz jammer at 50 MS/s), or a CW tone.
//  H: what makes a wideband jammer hard for a single weight: a 0.15-sample
//     (3 ns at 50 MS/s) delay, 0.8 dB / 40 deg, and a slightly different
//     filter.
//  a: RX2 channel gain relative to RX1 (scenario 7: -25 dB).
//  IQ: the AD9361's I/Q imbalance (scenarios 6, 8: ~48 dB image rejection),
//     x -> mu x + nu conj(x).
//  s: weak BPSK "satellite" (1.023 Mchip/s at 50 MS/s, -9.08 MHz offset).
//
//  Scenario (argv[1]); each needs a freshly reset core, so the csim script
//  runs one per process:
//    1  bit-exact: random weights, image taps, g2, make-up shift and bypass
//       (full ranges) against an integer reference of the core
//    2  wideband jammer 30 dB: suppression >= 35 dB, satellite SNR loss <= 3 dB,
//       and >= 10 dB more suppression than the single-weight (v3) mode
//    3  no jammer: bypass output is exactly x1[n-8]; with the filter in,
//       output within 0.5 dB of RX1 and satellite loss <= 0.5 dB
//    4  CW jammer 40 dB: suppression >= 40 dB
//    5  wideband jammer 30 dB plus a reflection (-6 dB, 3 samples later,
//       other direction): suppression >= 30 dB
//    6  wideband jammer 50 dB with I/Q imbalance (RX gain cut so the ADC does
//       not clip): residual <= 1.5 dB with the image taps (and at least 2 dB
//       better than without them), suppression >= 45 dB, satellite loss
//       <= 2 dB with the output make-up gain
//    7  RX2 25 dB weaker than RX1, wideband jammer 40 dB: with the input
//       normalisation (g2 from the noise floors) suppression >= 35 dB and at
//       least 10 dB more than without it
//    8  wideband jammer 60 dB with I/Q imbalance: suppression >= 50 dB
//    9  CW 62 dB above the noise at -7.06 MHz from the LO (1575.4 MHz with the LO
//       at 1582.5 MHz, 61.44 MS/s), on top of the satellite, I/Q imbalance:
//       suppression >= 64 dB, residual <= 1 dB (SMI-PI 4.4 weight scale)
//    10 the same at 68 dB: suppression >= 64 dB, residual <= 2.5 dB
//    11, 12, 13  barrage (noise) jammer over the whole band, I/Q imbalance, satellite at the
//       LO, the custom band 1575.42 MHz / 18 MS/s of the field test: J/N 61, 51 and 41 dB
//       (argv[2] overrides, argv[3] = RX2 path delay in samples); also checks that the
//       output make-up brings the level into the receiver back near the clean one
//    14 the make-up rule (aj_makeup_shift) on a table of cases
//  Prints "SMI_PI_CSIM: PASS" or "SMI_PI_CSIM: FAIL".
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <complex>
#include "smi_pi.h"
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
static ap_uint<1024> g_w = 0;
static ap_uint<64>   g_wcj = 0;
static ap_uint<32>   g_cfg = SMI_CFG_BYPASS;
static ap_uint<32>   g_nrm = 1 << SMI_GFRAC;

static void set_weights(const int16_t *wre, const int16_t *wim) {
    g_w = 0;
    for (int k = 0; k < 32; k++) {
        g_w(32 * k + 15, 32 * k)      = (ap_uint<16>)(ap_int<16>)wre[k];
        g_w(32 * k + 31, 32 * k + 16) = (ap_uint<16>)(ap_int<16>)wim[k];
    }
    g_wcj = 0;
    for (int k = 0; k < 2; k++) {
        g_wcj(32 * k + 15, 32 * k)      = (ap_uint<16>)(ap_int<16>)wre[AJ_IMG1 + k];
        g_wcj(32 * k + 31, 32 * k + 16) = (ap_uint<16>)(ap_int<16>)wim[AJ_IMG1 + k];
    }
}

static void set_cfg(int bypass, int shift, int ws = 0) {
    g_cfg = (bypass ? SMI_CFG_BYPASS : 0u) | ((unsigned)(shift & 7) << SMI_CFG_SH_LSB) |
            ((unsigned)(ws & 7) << SMI_CFG_WS_LSB);
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
    smi_pi(xs, ys, g_w, g_wcj, g_cfg, g_nrm);
    if (ys.empty()) { std::printf("core produced no output\n"); std::exit(2); }
    ap_uint<32> o = ys.read();
    if (out) *out = cd((double)(ap_int<16>)o(15, 0), (double)(ap_int<16>)o(31, 16));
}

// Runs (x1,x2) through the core after a 32-sample zero flush; returns y.
static cvec run_core(const std::vector<int> &a1r, const std::vector<int> &a1i,
                     const std::vector<int> &a2r, const std::vector<int> &a2i) {
    hls::stream<ap_uint<64> > xs;
    hls::stream<ap_uint<32> > ys;
    cvec y(a1r.size());
    for (int i = 0; i < 32; i++) push(xs, ys, 0, 0, 0, 0, 0);
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

// ---- scenario 1: bit-exact ---------------------------------------------------
static long long sat_ll(long long v, long long lo, long long hi) { return v < lo ? lo : v > hi ? hi : v; }

static int test_bitexact() {
    hls::stream<ap_uint<64> > xs;
    hls::stream<ap_uint<32> > ys;
    const int SEG = 2000, NSEG = 12, n = SEG * NSEG;
    std::vector<int> a(n), b(n), c(n), d(n);
    std::vector<long long> x2nr(n), x2ni(n);
    int bad = 0, sat = 0, sat18c = 0, bypassed = 0, shifted = 0;
    int16_t wr[AJ_NW], wi[AJ_NW];
    int g2r = 0, g2i = 0, byp = 0, sh = 0, wsc = 0;
    for (int i = 0; i < n; i++) {
        bool ext = (i % 97) < 20;
        a[i] = ext ? (urand() < 0.5 ? -2048 : 2047) : (int)(urand() * 4096) - 2048;
        b[i] = ext ? (urand() < 0.5 ? -2048 : 2047) : (int)(urand() * 4096) - 2048;
        c[i] = ext ? (urand() < 0.5 ? -2048 : 2047) : (int)(urand() * 4096) - 2048;
        d[i] = ext ? (urand() < 0.5 ? -2048 : 2047) : (int)(urand() * 4096) - 2048;
    }
    for (int i = 0; i < n; i++) {
        if (i % SEG == 0) {          // new weights, image taps, g2, shift, bypass
            int seg = i / SEG;
            for (int k = 0; k < AJ_NW; k++) {
                int span = (seg % 3 == 0) ? 65536 : (seg % 3 == 1) ? 8192 : 1024;
                wr[k] = (int16_t)((int)(urand() * span) - span / 2);
                wi[k] = (int16_t)((int)(urand() * span) - span / 2);
            }
            if (seg == 0) { wr[0] = 32767; wi[0] = -32768; wr[16] = -32768; wi[16] = -32768;
                            wr[AJ_IMG1] = -32768; wi[AJ_IMG1] = 32767; wr[AJ_IMG2] = 32767; wi[AJ_IMG2] = -32768; }
            g2r = (seg == 1) ? 32767 : (seg == 2) ? -32768 : (int)(urand() * 65536) - 32768;
            g2i = (seg == 1) ? 32767 : (seg == 2) ? -32768 : (int)(urand() * 65536) - 32768;
            if (seg == 3) { g2r = 1 << SMI_GFRAC; g2i = 0; }
            byp = (seg % 4 == 3);
            sh = seg % 8;                                  // 0..7 all act (4.5)
            wsc = (seg * 3) % 8;                           // weight scale 0..7: 6, 7 must act as 5
            set_weights(wr, wi); set_cfg(byp, sh, wsc); set_g2(g2r, g2i);
        }
        // the core normalises RX2 when the sample enters
        long long nr = (long long)g2r * c[i] - (long long)g2i * d[i];
        long long ni = (long long)g2r * d[i] + (long long)g2i * c[i];
        const int S = SMI_GFRAC - SMI_XFRAC;
        long long vr = (nr + (1LL << (S - 1))) >> S, vi = (ni + (1LL << (S - 1))) >> S;
        if (vr > 131071 || vr < -131072 || vi > 131071 || vi < -131072) sat18c++;   // cannot happen
        x2nr[i] = vr; x2ni[i] = vi;
        cd y;
        push(xs, ys, a[i], b[i], c[i], d[i], &y);

        auto at  = [&](const std::vector<int> &v, int j) { return j >= 0 ? (long long)v[j] : 0LL; };
        auto atl = [&](const std::vector<long long> &v, int j) { return j >= 0 ? v[j] : 0LL; };
        const int Q = SMI_WFRAC + SMI_XFRAC;
        long long sr = at(a, i - SMI_REF) << Q, si = at(b, i - SMI_REF) << Q;
        if (!byp) {
            long long tr = 0, ti = 0;       // the weighted taps, summed before the weight scale
            for (int k = 0; k < SMI_NT; k++) {
                if (k != SMI_REF) {
                    long long xr = at(a, i - k), xi = at(b, i - k);
                    tr += (wr[k] * xr - wi[k] * xi) << SMI_XFRAC;
                    ti += (wr[k] * xi + wi[k] * xr) << SMI_XFRAC;
                }
                long long xr = atl(x2nr, i - k), xi = atl(x2ni, i - k);
                tr += wr[16 + k] * xr - wi[16 + k] * xi; ti += wr[16 + k] * xi + wi[16 + k] * xr;
            }
            {   // image taps: c conj(x)
                long long xr = at(a, i - SMI_REF), xi = -at(b, i - SMI_REF);
                tr += (wr[AJ_IMG1] * xr - wi[AJ_IMG1] * xi) << SMI_XFRAC;
                ti += (wr[AJ_IMG1] * xi + wi[AJ_IMG1] * xr) << SMI_XFRAC;
                xr = atl(x2nr, i - SMI_REF); xi = -atl(x2ni, i - SMI_REF);
                tr += wr[AJ_IMG2] * xr - wi[AJ_IMG2] * xi;
                ti += wr[AJ_IMG2] * xi + wi[AJ_IMG2] * xr;
            }
            int w_eff = wsc > SMI_MAXWS ? SMI_MAXWS : wsc;
            long long rnd = w_eff ? (1LL << (w_eff - 1)) : 0;
            sr -= (tr + rnd) >> w_eff; si -= (ti + rnd) >> w_eff;
        } else bypassed++;
        int s_eff = sh > SMI_MAXSH ? SMI_MAXSH : sh;
        if (s_eff) shifted++;
        long long half = 1LL << (Q - 1);
        long long er = ((sr << s_eff) + half) >> Q, ei = ((si << s_eff) + half) >> Q;
        if (er > 32767 || er < -32768 || ei > 32767 || ei < -32768) sat++;
        er = sat_ll(er, -32768, 32767); ei = sat_ll(ei, -32768, 32767);
        if ((long long)y.real() != er || (long long)y.imag() != ei) {
            if (bad < 5) std::printf("  mismatch at %d: core (%g,%g) ref (%lld,%lld)\n", i, y.real(), y.imag(), er, ei);
            bad++;
        }
    }
    std::printf("bit-exact: %d samples, %d mismatches; %d output saturations, %d bypassed,\n"
                "           %d with make-up shift, %d normalised RX2 beyond 18 bits (must be 0)\n",
                n, bad, sat, bypassed, shifted, sat18c);
    return bad == 0 && sat > 0 && sat18c == 0 && bypassed > 0 && shifted > 0;
}

// ---- scenario 14: the output make-up rule ---------------------------------------
static int test_makeup() {
    struct T { int cut; double out; int maxsh; int want; } t[] = {
        {0, 0, 7, 0}, {5, 0, 7, 0}, {6, 0, 7, 1}, {10, 0, 7, 1}, {34, 0.3, 7, 5}, {34, 4.8, 7, 5},
        {36, 3.3, 7, 5}, {42, 0, 7, 7}, {50, 0, 7, 7}, {40, 0, 4, 4},      // older bitstream: 4
        {34, 20, 7, 2}, {34, 40, 7, 0}, {26, 0.3, 7, 4}, {26, 0.3, 4, 4}, {-3, 0, 7, 0}};
    int bad = 0;
    for (auto &c : t) {
        int g = aj_makeup_shift(c.cut, c.out, c.maxsh);
        if (g != c.want) { std::printf("  cut %d dB, output %+.1f dB over noise, max %d: got %d, want %d\n", c.cut, c.out, c.maxsh, g, c.want); bad++; }
    }
    {   // ADC excess model against the simulated losses (J/N 61: +1.8, 66: +4.1, 71: +7.8 dB)
        double e61 = aj_adc_excess_db(61.0), e66 = aj_adc_excess_db(66.0), e71 = aj_adc_excess_db(71.0), e50 = aj_adc_excess_db(50.0);
        std::printf("ADC excess model: J/N 50 %.2f, 61 %.2f, 66 %.2f, 71 %.2f dB\n", e50, e61, e66, e71);
        if (e50 > 0.3 || std::fabs(e61 - 1.8) > 0.5 || std::fabs(e66 - 4.1) > 0.8 || std::fabs(e71 - 7.8) > 1.0) bad++;
    }
    std::printf("make-up rule: %d cases, %d wrong\n", (int)(sizeof t / sizeof t[0]), bad);
    return bad == 0;
}

// ---- scenarios 2..8 ------------------------------------------------------------
struct Scen { double jnr_db; bool cw; double echo; double rx2_db; bool iq; double cwf; double satf; double chip; double delay; };

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
    const double nrms = 20.0, fsat = (sc.satf != 0) ? sc.satf : -9.08 / 50.0, cwf = (sc.cwf != 0) ? sc.cwf : 0.06, chip_step = (sc.chip != 0) ? sc.chip : 1.023 / 50.0;
    const double IQ1_EPS = 0.004, IQ1_PH = 0.3, IQ2_EPS = -0.003, IQ2_PH = -0.4;
    cvec J(N), J2, n1(N), n2(N), s1(N), s2(N);
    // jammer
    if (sc.cw) {
        for (int i = 0; i < N; i++) J[i] = std::polar(1.0, 2 * M_PI * cwf * i);
    } else {
        for (int i = 0; i < N; i++) J[i] = cd(grand(), grand());
        J = fir(J, lowpass(0.42, 41), 20);
    }
    // RX2 sees it through a different path: 0.15-sample delay, 0.8 dB, 40 deg,
    // and a slightly different filter.
    cvec h2 = frac_delay((sc.delay != 0) ? sc.delay : 0.15, 21);
    cvec hd = {cd(0.03, 0.01), 1.0, cd(-0.02, 0.015)};
    J2 = fir(fir(J, h2, 10), hd, 1);
    cd g2p = std::polar(std::pow(10, 0.8 / 20), 40 * M_PI / 180);
    for (auto &v : J2) v *= g2p;
    if (sc.echo > 0) {      // a reflection 3 samples later from another direction
        cvec e1(N), e2(N);
        for (int i = 3; i < N; i++) { e1[i] = sc.echo * std::polar(1.0, 1.3) * J[i - 3]; e2[i] = sc.echo * std::polar(1.0, 2.9) * J[i - 3]; }
        for (int i = 0; i < N; i++) { J[i] += e1[i]; J2[i] += e2[i]; }
    }
    double pj = pwr(J, 0), pn = 2 * nrms * nrms;
    double js = (sc.jnr_db > -90) ? std::sqrt(pn * std::pow(10, sc.jnr_db / 10) / pj) : 0.0;
    for (int i = 0; i < N; i++) { J[i] *= js; J2[i] *= js; }
    // noise and satellite
    int chip = 1; double cph = 0;
    for (int i = 0; i < N; i++) {
        n1[i] = cd(grand(), grand()) * nrms;
        n2[i] = cd(grand(), grand()) * nrms;
        cph += chip_step; if (cph >= 1) { cph -= 1; chip = urand() < 0.5 ? -1 : 1; }
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
    struct Meas { double supp, loss, out_db, res, lvl; };
    auto measure = [&](const aj_result_t &r, double gn, int shift) {
        Meas m;
        double sc4 = std::pow(4.0, shift);
        set_weights(r.wre, r.wim);
        set_g2((int)std::floor(gn * 8192 + 0.5), 0);
        set_cfg(0, shift, r.ws);
        cvec yj = run_core_c(seg(jx1), seg(jx2));
        cvec yx = run_core_c(seg(x1), seg(x2));
        cvec yn = run_core_c(seg(nx1), seg(nx2));
        m.supp = (sc.jnr_db > -90) ? 10 * std::log10(pwr(seg(J), 0) / std::max(pwr(yj, 32) / sc4, 1e-9)) : 0;
        // Satellite SNR by correlation with the known satellite waveform, on
        // the real (quantised, dithered by the noise) output, against RX1
        // with no jammer at all. The projection onto the waveform is the
        // satellite power, the rest is noise + residual jammer.
        cvec ref = seg(s1);
        auto snr = [&](const cvec &y, int dly) {
            cd c = 0; double pr = 0, py = 0; int cnt = 0;
            for (size_t i = 32; i < y.size(); i++) {
                cd rr = ref[i - dly];
                c += y[i] * std::conj(rr); pr += std::norm(rr); py += std::norm(y[i]); cnt++;
            }
            double ps = std::norm(c) / pr;
            return (ps / cnt) / (py / cnt - ps / cnt);
        };
        cvec clean(ref.size()), n1s = seg(n1);
        for (size_t i = 0; i < ref.size(); i++) clean[i] = n1s[i] / g + ref[i] / g;
        m.loss = 10 * std::log10(snr(clean, 0) / snr(yx, SMI_REF));
        m.out_db = 10 * std::log10(pwr(yx, 32) / sc4 / pwr(seg(x1), 0));
        // output level into the receiver against the clean level (RX1 noise at full gain, no jammer)
        m.lvl = 10 * std::log10(pwr(yx, 32) / pn);
        // residual: output with jammer / output of the same weights on the
        // noise alone (0 dB = jammer fully removed)
        m.res = 10 * std::log10(pwr(yx, 32) / std::max(pwr(yn, 32), 1e-9));
        return m;
    };
    auto shift_for = [&](const aj_result_t &r) {
        // the firmware's rule (aj_makeup_shift): restore the clean output level, 6 dB per
        // step of RX gain cut, never more than 3 dB above it
        return aj_makeup_shift((int)cut_db, r.res_db + aj_db10(1.0 + r.wn2), SMI_MAXSH);
    };

    std::printf("scenario %d: JNR %.0f dB%s%s, RX gain cut %.1f dB", id, sc.jnr_db,
                sc.cw ? " CW" : " wideband", sc.echo > 0 ? " + reflection" : "", cut_db);
    if (sc.rx2_db != 0) std::printf(", RX2 %+.0f dB (g2 %.3f = %+.1f dB)", sc.rx2_db, g2, 20 * std::log10(g2));
    if (sc.iq) std::printf(", I/Q image rejection %.1f / %.1f dB", irr_db(IQ1_EPS, IQ1_PH), irr_db(IQ2_EPS, IQ2_PH));
    std::printf("\n");

    if (id == 3) {
        // bypass: exactly x1[n-8]
        set_cfg(1, 0); set_g2((int)(g2 * 8192 + 0.5), 0);
        cvec yb = run_core_c(seg(x1), seg(x2));
        cvec xs = seg(x1);
        int bad = 0;
        for (size_t i = SMI_REF; i < yb.size(); i++) if (yb[i] != xs[i - SMI_REF]) bad++;
        std::printf("  bypass           : %d of %d samples differ from x1[n-8]\n", bad, (int)yb.size() - SMI_REF);
        if (bad) return 0;
    }

    aj_result_t r, r2;
    if (!solve(AJ_MODE_STAP, g2, r)) { std::printf("solver failed\n"); return 0; }
    int sh = (id >= 6) ? shift_for(r) : 0;
    Meas m = measure(r, g2, sh);
    std::printf("  space-time 31+2 taps: suppression %.1f dB, residual %+.2f dB, satellite SNR loss %.2f dB, output %+.2f dB re RX1%s\n",
                m.supp, m.res, m.loss, m.out_db, sh ? "" : "");
    if (sh) std::printf("  output make-up    : %d (x%d, +%d dB)\n", sh, 1 << sh, 6 * sh);
    std::printf("  output level      : %+.1f dB against the clean level (RX gain cut %.0f dB, make-up +%d dB)\n", m.lvl, cut_db, 6 * sh);
    std::printf("  firmware figures  : J/N %.1f dB, suppression %.1f dB, residual %+.1f dB, |w|^2 %.2f, loading %.1f LSB^2 (%d passes), weight scale 2^%d\n",
                r.jn_db, r.supp_db, r.res_db, r.wn2, r.sig2, r.iters, r.ws);
    std::printf("  detection         : coherence %.3f at lag %d -> J/N %.1f dB, RX1 %.0f RX2 %.0f LSB^2\n",
                r.coh, r.coh_lag, r.jn_coh_db, r.p1, r.p2);
    if (!sc.cw && sc.jnr_db > -90)
        std::printf("  jammer estimate   : bandwidth %.2f fs, centre %+.3f fs\n", r.bw, r.foff);
    if (sc.cw)
        std::printf("  jammer estimate   : bandwidth %.4f fs, centre %+.4f fs (true %.4f)\n", r.bw, r.foff, cwf);

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
        if (!solve(AJ_MODE_STAP, 1.0, r2)) return 0;
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
    case 11: case 12: case 13: {
        // the level into the receiver must be back near the clean one: within -6 dB (the 6 dB step)
        // and +3.5 dB
        bool lvl_ok = m.lvl >= -6.5 && m.lvl <= 3.5;
        if (id == 11) ok = m.supp >= 54 && m.loss <= 4.5 && lvl_ok;        // 61 dB: at the ADC's limit
        else if (id == 12) ok = m.supp >= 54 && m.loss <= 1.0 && m.res <= 1.0 && lvl_ok;   // 51 dB
        else ok = m.supp >= 50 && m.loss <= 0.6 && m.res <= 0.6 && lvl_ok;                   // 41 dB
        break; }
    case 9: case 10: {
        // the weight scale off (ws = 0) on the same data, for comparison
        aj_result_t r0 = r;
        for (int k = 0; k < AJ_NW; k++) {
            r0.wre[k] = (int16_t)std::floor((double)r.wre[k] / (1 << r.ws) + 0.5);
            r0.wim[k] = (int16_t)std::floor((double)r.wim[k] / (1 << r.ws) + 0.5);
        }
        r0.ws = 0;
        Meas m0 = measure(r0, g2, sh);
        std::printf("  weight scale off  : suppression %.1f dB, residual %+.2f dB, satellite SNR loss %.2f dB\n", m0.supp, m0.res, m0.loss);
        if (id == 9)  ok = m.supp >= 64 && m.res <= 1.0 && m.loss <= 3;
        else          ok = m.supp >= 64 && m.res <= 2.5 && m.loss <= 3.5;
        break; }
    }
    return ok;
}

int main(int argc, char **argv) {
    int scen = (argc > 1) ? std::atoi(argv[1]) : 2;
    int ok;
    switch (scen) {
    case 1: ok = test_bitexact(); break;
    case 2: ok = test_jammer(2, {30, false, 0, 0, false}); break;
    case 3: ok = test_jammer(3, {-99, false, 0, 0, false}); break;
    case 4: ok = test_jammer(4, {40, true, 0, 0, false}); break;
    case 5: ok = test_jammer(5, {30, false, 0.5, 0, false}); break;
    case 6: ok = test_jammer(6, {50, false, 0, 0, true}); break;
    case 7: ok = test_jammer(7, {40, false, 0, -25, false}); break;
    case 8: ok = test_jammer(8, {60, false, 0, 0, true}); break;
    // CW on top of the GPS signal: 1575.4 MHz with the LO at 1582.5 MHz (-7.1 MHz), 61.44 MS/s
    case 9:  ok = test_jammer(9,  {62, true, 0, 0, true, -7.06 / 61.44, -7.06 / 61.44, 1.023 / 61.44}); break;
    case 10: ok = test_jammer(10, {68, true, 0, 0, true, -7.06 / 61.44, -7.06 / 61.44, 1.023 / 61.44}); break;
    // barrage (noise) jammer over the whole band, I/Q imbalance, satellite at the LO, custom band 5:
    // 1575.42 MHz, 18 MS/s. J/N from argv[2] (default 61: the field test at the jammer's "30" setting)
    case 11: case 12: case 13: {
        double jn = (argc > 2) ? std::atof(argv[2]) : (scen == 11 ? 61.0 : scen == 12 ? 51.0 : 41.0);
        ok = test_jammer(scen, {jn, false, 0, 0, true, 0, -0.02 / 18.0, 1.023 / 18.0, (argc > 3) ? std::atof(argv[3]) : 0.055}); break; }
    case 14: ok = test_makeup(); break;
    default: std::printf("unknown scenario %d\n", scen); ok = 0;
    }
    std::printf("SMI_PI_CSIM: %s (scenario %d)\n", ok ? "PASS" : "FAIL", scen);
    return ok ? 0 : 1;
}
