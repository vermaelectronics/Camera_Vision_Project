// ============================================================================
//  testbench.cpp  --  C simulation of the pipelined PL-NPI core (v2.3)
//
//  Two-element array at the E310 raw ADC scale (12-bit samples):
//      RX1 = IQ1( J + n1 ),   RX2 = IQ2( a (h J + n2) )
//  J: CW jammer, or a band-limited Gaussian jammer (scenario 9); h the RX2
//  path (0.6 - 0.5j); a the RX2 channel gain (scenario 5: -20 dB); IQ the
//  AD9361's I/Q imbalance (scenario 7: ~49 dB image rejection).
//
//  Scenario (argv[1]); each needs a freshly reset core, so the csim script
//  runs one per process. All must print "PL_NPI_CSIM: PASS".
//    1  CW 600 LSB, mode ON, gamma 1 (the v2.2 test): >= 40 dB below the
//       jammer within 2000 samples, no growth afterwards, > 1 gain band used
//    2  no jammer, mode ON, gamma 1e8 (the v2.2 test): the output is NOT
//       cancelled (within 3 dB of RX1)
//    3  bypass: mode BYPASS with a jammer, and mode AUTO with noise only:
//       output == RX1 for every sample, weights 0, detector never on
//    4  gated adaptation, mode AUTO: CW jammer 31 dB on at sample 5000, off
//       at 40000: detector on within 100 samples, residual <= 1 dB once
//       converged, back to bypass (output == RX1, weights 0) after the hold
//       time, exactly one detector event
//    5  RX2 20 dB weaker: with g2 = 4 (+12 dB) the null forms at least 5x
//       faster than with g2 = 1, residual <= 1 dB
//    6  (reserved, merged into 5)
//    7  CW jammer 50 dB above the noise with I/Q imbalance: residual <= 1.5
//       dB with the image taps, at least 2 dB better than without
//    8  output make-up shift: x4 (+12 dB) while filtering, ignored in bypass
//    9  wideband jammer 30 dB through a 0.15-sample path difference
//       (informational: one spatial weight cannot null it deeply)
//
//  Writing the stimulus/response to files for a comparison against the
//  original RTL core is enabled with the environment variable PL_NPI_DUMP
//  (scenario 1).
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <complex>
#include "pl_npi.h"

typedef std::complex<double> cd;

// ---- random numbers ----------------------------------------------------------
static unsigned lcg = 12345u;
static int unoise(int a) {                 // uniform integer in [-a, a]
    lcg = lcg * 1103515245u + 12345u;
    return (int)((lcg >> 8) % (unsigned)(2 * a + 1)) - a;
}
static unsigned long long rs = 88172645463325252ULL;
static double urand() {
    rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17;
    return ((rs >> 11) + 0.5) / 9007199254740992.0;
}
static double grand() { return std::sqrt(-2.0 * std::log(urand())) * std::cos(2 * M_PI * urand()); }

static int clip12(double v) {
    double r = std::floor(v + 0.5);
    if (r > 2047) r = 2047;
    if (r < -2048) r = -2048;
    return (int)r;
}

// ---- the core ------------------------------------------------------------------
static unsigned g_gamma = 1, g_cfg = PL_NPI_MODE_ON, g_nrm = 1u << PL_NPI_GFRAC;
static unsigned g_thr = 0xFFFFFFFFu, g_hold = 0;
static int g_adapt = 1;
static hls::stream<ap_uint<64> > xs;
static hls::stream<ap_uint<32> > ys;

// x1r/x1i: the RX1 input that this output sample belongs to (the core's
// output is PL_NPI_LATENCY samples late, in bypass and in filter mode).
struct Out { int yr, yi; unsigned st; double w[6]; unsigned pow; int x1r, x1i; };

static int hx_r[PL_NPI_LATENCY + 1], hx_i[PL_NPI_LATENCY + 1];

static Out step(int x1r, int x1i, int x2r, int x2i) {
    for (int k = PL_NPI_LATENCY; k > 0; k--) { hx_r[k] = hx_r[k - 1]; hx_i[k] = hx_i[k - 1]; }
    hx_r[0] = x1r; hx_i[0] = x1i;
    ap_uint<64> w;
    w(15, 0)  = (ap_uint<16>)(ap_int<16>)x1r;
    w(31, 16) = (ap_uint<16>)(ap_int<16>)x1i;
    w(47, 32) = (ap_uint<16>)(ap_int<16>)x2r;
    w(63, 48) = (ap_uint<16>)(ap_int<16>)x2i;
    xs.write(w);
    ap_uint<32> st = 0, pw = 0;
    ap_uint<192> wo = 0;
    pl_npi(xs, ys, g_gamma, (ap_uint<1>)g_adapt, g_cfg, g_nrm, g_thr, g_hold, &st, &wo, &pw);
    if (ys.empty()) { std::printf("core produced no output\n"); std::exit(2); }
    ap_uint<32> y = ys.read();
    Out o;
    o.yr = (ap_int<16>)y(15, 0);
    o.yi = (ap_int<16>)y(31, 16);
    o.st = (unsigned)st;
    for (int k = 0; k < 6; k++)
        o.w[k] = (double)(ap_int<32>)wo(32 * k + 31, 32 * k) / 1048576.0;
    o.pow = (unsigned)pw;
    o.x1r = hx_r[PL_NPI_LATENCY];
    o.x1i = hx_i[PL_NPI_LATENCY];
    return o;
}

static void set_g2(double g) {
    int q = (int)std::floor(g * 8192 + 0.5);
    if (q > 32767) q = 32767;
    g_nrm = (unsigned)(q & 0xFFFF);
}
static double g2_of_nrm() { return (double)(short)(g_nrm & 0xFFFF) / 8192.0; }

static void iq_imbalance(cd &x, double eps, double ph_deg) {
    double ph = ph_deg * M_PI / 180;
    cd mu = (1.0 + (1 + eps) * std::polar(1.0, -ph)) / 2.0;
    cd nu = (1.0 - (1 + eps) * std::polar(1.0, ph)) / 2.0;
    x = mu * x + nu * std::conj(x);
}

// A simple stimulus source: CW jammer through h, Gaussian noise, RX2 gain a,
// optional I/Q imbalance.
struct Src {
    double amp = 0, f = 0.0137, sig = 20, a2 = 1.0;
    cd h = cd(0.6, -0.5);
    bool iq = false;
    long n = 0;
    void next(int &x1r, int &x1i, int &x2r, int &x2i) {
        cd j = std::polar(amp, 2 * M_PI * f * (double)n);
        cd x1 = j + cd(grand(), grand()) * sig;
        cd x2 = a2 * (h * j + cd(grand(), grand()) * sig);
        if (iq) { iq_imbalance(x1, 0.004, 0.3); iq_imbalance(x2, -0.003, -0.4); }
        x1r = clip12(x1.real()); x1i = clip12(x1.imag());
        x2r = clip12(x2.real()); x2i = clip12(x2.imag());
        n++;
    }
};

// Output noise the present weights should give with no jammer: N1 (1 + |w2
// g2|^2 r2 + |c1|^2 + |c2 g2|^2 r2), r2 = RX2/RX1 noise power.
static double nout(const Out &o, double n1, double r2) {
    double g = g2_of_nrm();
    double w2 = o.w[0] * o.w[0] + o.w[1] * o.w[1];
    double c1 = o.w[2] * o.w[2] + o.w[3] * o.w[3];
    double c2 = o.w[4] * o.w[4] + o.w[5] * o.w[5];
    return n1 * (1.0 + (w2 + c2) * g * g * r2 + c1);
}

// ---- scenarios 1-2: the v2.2 tests ----------------------------------------------
static int test_v22(int scen) {
    const char *dump = getenv("PL_NPI_DUMP");
    FILE *fs = 0, *fr = 0;
    if (dump && scen == 1) {
        char a[512], b[512];
        snprintf(a, sizeof a, "%s_stim.hex", dump);
        snprintf(b, sizeof b, "%s_resp.txt", dump);
        fs = fopen(a, "w"); fr = fopen(b, "w");
    }
    double amp = (scen == 1) ? 600.0 : 0.0;
    int nsamp = (scen == 1) ? 6000 : 200000, na = (scen == 1) ? 2 : 20;
    g_gamma = (scen == 1) ? 1u : 100000000u;
    g_cfg = PL_NPI_MODE_ON;                     // image taps off: the v2.2 core
    double blk = 0, rx1p = 0, outp = 0, in_pow = amp * amp, min_blk = 1e30, last_blk = 0;
    int conv = -1, seen[4] = {0, 0, 0, 0};
    bool grew = false;
    const double hr = 0.6, hi = -0.5;
    for (int n = 0; n < nsamp; n++) {
        double ph = 2.0 * M_PI * 0.0137 * n;
        double jr = amp * cos(ph), ji = amp * sin(ph);
        int x1r = (int)jr + unoise(na), x1i = (int)ji + unoise(na);
        int x2r = (int)(hr * jr - hi * ji) + unoise(na);
        int x2i = (int)(hr * ji + hi * jr) + unoise(na);
        Out o = step(x1r, x1i, x2r, x2i);
        seen[(o.st >> PL_NPI_ST_BAND_LSB) & 3] = 1;
        if (fs) fprintf(fs, "%04x%04x%04x%04x\n", x2i & 0xFFFF, x2r & 0xFFFF, x1i & 0xFFFF, x1r & 0xFFFF);
        if (fr) fprintf(fr, "%d %d\n", o.yr, o.yi);
        blk += (double)o.yr * o.yr + (double)o.yi * o.yi;
        if (n >= nsamp / 2) {
            rx1p += (double)x1r * x1r + (double)x1i * x1i;
            outp += (double)o.yr * o.yr + (double)o.yi * o.yi;
        }
        if ((n + 1) % 100 == 0) {
            double p = blk / 100.0;
            if (conv < 0 && amp > 0 && p < in_pow * 1e-4) conv = n + 1;
            if (conv >= 0 && p < min_blk) min_blk = p;
            if (conv >= 0 && p > 10.0 * min_blk + 5.0) grew = true;
            last_blk = p;
            blk = 0;
        }
    }
    if (fs) fclose(fs);
    if (fr) fclose(fr);
    int bands = seen[0] + seen[1] + seen[2] + seen[3], ok = 1;
    if (scen == 1) {
        printf("jammer 600 LSB, gamma 1: last 100 %.2f  null %.1f dB  converged at sample %d  bands %d%s\n",
               last_blk, 10.0 * log10(last_blk / in_pow), conv, bands, grew ? "  GREW AGAIN" : "");
        if (conv < 0 || conv > 2000)  { printf("FAIL: no 40 dB null within 2000 samples\n"); ok = 0; }
        if (last_blk > in_pow * 1e-4) { printf("FAIL: final null shallower than 40 dB\n"); ok = 0; }
        if (grew)                     { printf("FAIL: output grew again after converging\n"); ok = 0; }
        if (bands < 2)                { printf("FAIL: only one gain band used\n"); ok = 0; }
    } else {
        double d = 10.0 * log10((outp + 1e-9) / (rx1p + 1e-9));
        printf("no jammer, noise +-20 LSB, gamma 1e8, 200000 samples: output vs RX1 power %.1f dB\n", d);
        if (d < -3.0) { printf("FAIL: wanted signal cancelled without a jammer\n"); ok = 0; }
    }
    return ok;
}

// ---- scenario 3: bypass ----------------------------------------------------------
static int test_bypass() {
    Src s; s.amp = 1500; s.sig = 20;
    int bad = 0, wbad = 0, det = 0;
    g_gamma = 1; g_cfg = PL_NPI_MODE_BYPASS | PL_NPI_CFG_IMAGE | (2u << PL_NPI_CFG_SH_LSB);
    for (int n = 0; n < 20000; n++) {
        int a, b, c, d; s.next(a, b, c, d);
        Out o = step(a, b, c, d);
        if (o.yr != o.x1r || o.yi != o.x1i) bad++;
        for (int k = 0; k < 6; k++) if (o.w[k] != 0.0) { wbad++; break; }
    }
    printf("bypass with a 1500 LSB jammer: %d samples differ from RX1, %d with non-zero weights\n", bad, wbad);
    // AUTO, noise only, threshold 6 dB above it. First 2000 samples of noise
    // in BYPASS: the detector average still holds the jammer of the part above.
    s.amp = 0;
    g_thr = (unsigned)(4.0 * 2 * 2 * 20 * 20); g_hold = 1000;
    for (int n = 0; n < 2000; n++) { int a, b, c, d; s.next(a, b, c, d); step(a, b, c, d); }
    g_cfg = PL_NPI_MODE_AUTO | PL_NPI_CFG_IMAGE;
    int bad2 = 0;
    unsigned cnt = 0, cnt0 = 0, pmax = 0;
    { int a, b, c, d; s.next(a, b, c, d); cnt0 = step(a, b, c, d).st >> PL_NPI_ST_CNT_LSB; }
    for (int n = 0; n < 50000; n++) {
        int a, b, c, d; s.next(a, b, c, d);
        Out o = step(a, b, c, d);
        if (o.yr != o.x1r || o.yi != o.x1i) bad2++;
        if (o.st & PL_NPI_ST_DET) det++;
        cnt = (o.st >> PL_NPI_ST_CNT_LSB) - cnt0;
        if (o.pow > pmax) pmax = o.pow;
    }
    printf("AUTO, noise only (threshold %u LSB^2, average peaked at %u): %d samples differ, detector on %d samples, %u events\n",
           g_thr, pmax, bad2, det, cnt);
    return bad == 0 && wbad == 0 && bad2 == 0 && det == 0 && cnt == 0;
}

// ---- scenario 4: gated adaptation ---------------------------------------------------
static int test_gating() {
    Src s; s.sig = 20;
    const double n1 = 2 * 20.0 * 20.0;
    g_gamma = (unsigned)(131072.0 / 8.0 * n1);       // the firmware's k = 1/8
    g_thr = (unsigned)(4.0 * 2 * n1); g_hold = 3000;
    g_cfg = PL_NPI_MODE_AUTO | PL_NPI_CFG_IMAGE;
    int on_at = -1, off_at = -1, bad_after = 0, wbad_after = 0;
    double pout = 0, nexp = 0; int cnt = 0;
    unsigned events = 0;
    for (int n = 0; n < 60000; n++) {
        s.amp = (n >= 5000 && n < 40000) ? 1000.0 : 0.0;
        int a, b, c, d; s.next(a, b, c, d);
        Out o = step(a, b, c, d);
        bool act = o.st & PL_NPI_ST_ACTIVE;
        if (on_at < 0 && act) on_at = n;
        if (n >= 40000 && off_at < 0 && !act) off_at = n;
        if (n >= 20000 && n < 40000) {
            pout += (double)o.yr * o.yr + (double)o.yi * o.yi;
            nexp += nout(o, n1, 1.0);
            cnt++;
        }
        if (off_at >= 0 && n > off_at + 20) {
            if (o.yr != o.x1r || o.yi != o.x1i) bad_after++;
            for (int k = 0; k < 6; k++) if (o.w[k] != 0.0) { wbad_after++; break; }
        }
        events = o.st >> PL_NPI_ST_CNT_LSB;
    }
    double res = 10 * log10(pout / nexp), jn = 10 * log10(1e6 / n1);
    printf("gated: jammer (J/N %.0f dB) on at 5000 -> filter in at %d (+%d); residual %.2f dB;\n"
           "       off at 40000 -> bypass at %d (hold 3000); after it %d samples differ from RX1,\n"
           "       %d with non-zero weights; %u detector event(s)\n",
           jn, on_at, on_at - 5000, res, off_at, bad_after, wbad_after, events);
    return on_at >= 5000 && on_at - 5000 <= 100 && res <= 1.0 && off_at > 43000 && off_at < 45000 &&
           bad_after == 0 && wbad_after == 0 && events == 1;
}

// ---- scenario 5: RX2 normalisation -------------------------------------------------
// Two runs in one process: the core is reset between them by BYPASS (weights
// to 0) for 200 samples.
#define CONV_N 60000
static int conv_run(double g2, double &res_end, int &conv_at) {
    Src s; s.sig = 20; s.amp = 1000; s.a2 = 0.1;
    const double n1 = 2 * 20.0 * 20.0, r2 = 0.01;
    g_cfg = PL_NPI_MODE_BYPASS;
    for (int n = 0; n < 200; n++) { int a, b, c, d; s.next(a, b, c, d); step(a, b, c, d); }
    set_g2(g2);
    g_gamma = (unsigned)(131072.0 / 8.0 * n1);
    g_cfg = PL_NPI_MODE_ON;
    conv_at = -1;
    double pb = 0, nb = 0, pt = 0, nt = 0;
    for (int n = 0; n < CONV_N; n++) {
        int a, b, c, d; s.next(a, b, c, d);
        Out o = step(a, b, c, d);
        pb += (double)o.yr * o.yr + (double)o.yi * o.yi;
        nb += nout(o, n1, r2);
        if ((n + 1) % 200 == 0) {
            if (conv_at < 0 && pb / nb < 2.0) conv_at = n + 1;
            pb = nb = 0;
        }
        if (n >= 50000) { pt += (double)o.yr * o.yr + (double)o.yi * o.yi; nt += nout(o, n1, r2); }
    }
    res_end = 10 * log10(pt / nt);
    return conv_at;
}

static int test_norm() {
    double r1, r4; int c1, c4;
    conv_run(1.0, r1, c1);
    conv_run(4.0, r4, c4);
    char t1[48], t4[48];
    if (c1 < 0) snprintf(t1, sizeof t1, "never (in %d samples)", CONV_N); else snprintf(t1, sizeof t1, "after %d samples", c1);
    if (c4 < 0) snprintf(t4, sizeof t4, "never (in %d samples)", CONV_N); else snprintf(t4, sizeof t4, "after %d samples", c4);
    printf("RX2 20 dB weaker, CW jammer 31 dB: residual < 3 dB %s with g2 = 1,\n"
           "       %s with g2 = 4 (+12 dB); final residual %.2f / %.2f dB\n", t1, t4, r1, r4);
    return c4 > 0 && (c1 < 0 || c1 >= 5 * c4) && r4 <= 1.0;
}

// ---- scenario 7: image taps -----------------------------------------------------------
static double img_run(bool image) {
    Src s; s.sig = 4; s.iq = true;
    const double n1 = 2 * 4.0 * 4.0;
    s.amp = std::sqrt(n1 * 1e5);                       // J/N 50 dB
    g_cfg = PL_NPI_MODE_BYPASS;
    for (int n = 0; n < 200; n++) { int a, b, c, d; s.next(a, b, c, d); step(a, b, c, d); }
    g_gamma = (unsigned)(131072.0 / 8.0 * n1);
    g_cfg = PL_NPI_MODE_ON | (image ? PL_NPI_CFG_IMAGE : 0u);
    double pt = 0, nt = 0;
    Out o = {0, 0, 0, {0}, 0, 0, 0};
    for (int n = 0; n < 80000; n++) {
        int a, b, c, d; s.next(a, b, c, d);
        o = step(a, b, c, d);
        if (n >= 40000) { pt += (double)o.yr * o.yr + (double)o.yi * o.yi; nt += nout(o, n1, 1.0); }
    }
    printf("  image taps %s: residual %.2f dB, |w2| %.3f, |c1| %.5f, |c2| %.5f\n", image ? "on " : "off",
           10 * log10(pt / nt), std::hypot(o.w[0], o.w[1]), std::hypot(o.w[2], o.w[3]), std::hypot(o.w[4], o.w[5]));
    return 10 * log10(pt / nt);
}

static int test_image() {
    printf("CW jammer 50 dB above the noise, I/Q imbalance ~49 dB image rejection:\n");
    double off = img_run(false), on = img_run(true);
    return on <= 1.5 && off - on >= 2.0;
}

// ---- scenario 8: make-up shift -------------------------------------------------------
static int test_shift() {
    Src s; s.sig = 20;
    g_gamma = 100000000u;
    g_cfg = PL_NPI_MODE_ON;
    double p0 = 0, p2 = 0;
    for (int n = 0; n < 100000; n++) {
        if (n == 50000) g_cfg = PL_NPI_MODE_ON | (2u << PL_NPI_CFG_SH_LSB);
        int a, b, c, d; s.next(a, b, c, d);
        Out o = step(a, b, c, d);
        double p = (double)o.yr * o.yr + (double)o.yi * o.yi;
        if (n >= 10000 && n < 50000) p0 += p;
        if (n >= 60000) p2 += p;
    }
    double d = 10 * log10((p2 / 40000) / (p0 / 40000));
    g_cfg = PL_NPI_MODE_BYPASS | (2u << PL_NPI_CFG_SH_LSB);
    int bad = 0;
    for (int n = 0; n < 2000; n++) {
        int a, b, c, d2; s.next(a, b, c, d2);
        Out o = step(a, b, c, d2);
        if (n > 20 && (o.yr != o.x1r || o.yi != o.x1i)) bad++;
    }
    printf("make-up shift 2: output %+.2f dB (expected +12.04); in bypass %d samples differ from RX1\n", d, bad);
    return std::fabs(d - 12.04) <= 0.3 && bad == 0;
}

// ---- scenario 9: wideband (informational) ----------------------------------------------
static int test_wideband() {
    const int N = 120000;
    std::vector<cd> J(N);
    for (auto &v : J) v = cd(grand(), grand());
    // low-pass 0.42 fs (41 taps) and a 0.15-sample delay on RX2
    std::vector<cd> L(N), D(N);
    for (int i = 0; i < N; i++) {
        cd a = 0, b = 0;
        for (int k = -20; k <= 20; k++) {
            int j = i - k; if (j < 0 || j >= N) continue;
            double w = 0.54 + 0.46 * cos(M_PI * k / 20.0);
            a += J[j] * (k == 0 ? 0.84 : sin(2 * M_PI * 0.42 * k) / (M_PI * k)) * w;
        }
        L[i] = a;
    }
    for (int i = 0; i < N; i++) {
        cd b = 0;
        for (int k = -10; k <= 10; k++) {
            int j = i - k; if (j < 0 || j >= N) continue;
            double t = k - 0.15, w = 0.54 + 0.46 * cos(M_PI * k / 10.0);
            b += L[j] * (sin(M_PI * t) / (M_PI * t)) * w;
        }
        D[i] = b;
    }
    double pj = 0; for (auto &v : L) pj += std::norm(v); pj /= N;
    const double n1 = 2 * 20.0 * 20.0, js = std::sqrt(n1 * 1000.0 / pj);   // 30 dB
    g_gamma = (unsigned)(131072.0 / 8.0 * n1);
    g_cfg = PL_NPI_MODE_ON;
    cd h(0.6, -0.5);
    double pt = 0, nt = 0;
    Out o = {0, 0, 0, {0}, 0, 0, 0};
    for (int n = 0; n < N; n++) {
        cd x1 = L[n] * js + cd(grand(), grand()) * 20.0;
        cd x2 = h * D[n] * js + cd(grand(), grand()) * 20.0;
        o = step(clip12(x1.real()), clip12(x1.imag()), clip12(x2.real()), clip12(x2.imag()));
        if (n >= N / 2) { pt += (double)o.yr * o.yr + (double)o.yi * o.yi; nt += nout(o, n1, 1.0); }
    }
    printf("wideband jammer 30 dB, 0.15-sample path difference (informational): residual %.1f dB,\n"
           "       i.e. suppression ~%.1f dB. One spatial weight matches RX1/RX2 at one frequency\n"
           "       only; SMI-PI v4.1 (31 space-time taps) is the design for this case.\n",
           10 * log10(pt / nt), 30.0 - 10 * log10(pt / nt));
    return 1;
}

int main(int argc, char **argv) {
    int scen = (argc > 1) ? atoi(argv[1]) : 1, ok;
    switch (scen) {
    case 1: case 2: ok = test_v22(scen); break;
    case 3: ok = test_bypass(); break;
    case 4: ok = test_gating(); break;
    case 5: ok = test_norm(); break;
    case 6: ok = 1; printf("(scenario 6 is part of 5)\n"); break;
    case 7: ok = test_image(); break;
    case 8: ok = test_shift(); break;
    case 9: ok = test_wideband(); break;
    default: printf("unknown scenario %d\n", scen); ok = 0;
    }
    printf(ok ? "PL_NPI_CSIM: PASS (scenario %d)\n" : "PL_NPI_CSIM: FAIL (scenario %d)\n", scen);
    return ok ? 0 : 1;
}
