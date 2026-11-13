// ============================================================================
//  testbench.cpp  --  C simulation of the space-time power-inversion filter
//                     stap_pi with the weights from the ARM (antsdr-fw-patch
//                     crpa16 / crpa17)
//
//  The weights come from the FIRMWARE's own NLMS solver aj_nlms_solve()
//  (gnss_crpa/src/gnss_aj_math.c, compiled into this testbench), run on
//  8192-sample captures packed exactly like the DMA buffer, one capture after
//  the other, as the board does; the make-up shift from aj_makeup_shift().
//
//  Signals at the raw 12-bit ADC scale, noise 20 LSB rms per I/Q component:
//      RX1 = IQ1( J + s + n1 )
//      RX2 = IQ2( a (H(J) + g s + n2) )
//  J: complex Gaussian jammer low-passed to 0.84 fs, or a CW tone.
//  H: 0.15-sample delay, 0.8 dB / 40 deg and a slightly different filter.
//  a: RX2 channel gain relative to RX1 (scenario 7: -25 dB).
//  IQ: AD9361-like I/Q imbalance (~48 dB image rejection), x -> mu x + nu conj(x).
//  s: weak BPSK "satellite" 20 dB below the noise.
//
//  Each jammer scenario: the ARM NLMS runs over NCAP captures (default 20, weights
//  carried from capture to capture, from 0), the last weights are committed to the
//  core, then the jammer, the noise and the sum are passed through separately:
//  suppression, residual (output / output of the same weights on the noise alone,
//  0 dB = fully nulled), satellite SNR loss. The OPTIMUM weights of the same taps
//  (Wiener solution in double precision) show how close the ARM NLMS gets.
//
//  Scenario (argv[1]); argv[2] = J/N, argv[3] = alpha, argv[4] = captures, argv[5] = passes.
//    1  bit-exact: random weights (full range), cfg, g2, gain, weight scale and bypass
//       against an integer reference model of the core
//    2  wideband jammer 30 dB          3  no jammer (bypass exact; filter in)
//    4  CW jammer 40 dB                5  wideband 30 dB + reflection 3 samples later
//    6  wideband 50 dB, I/Q imbalance  7  RX2 25 dB weaker: with and without g2
//    8  wideband 60 dB, I/Q imbalance  9  CW 62 dB on the satellite, I/Q imbalance
//    11, 12, 13  field cases J/N 61, 51, 41 dB (barrage, I/Q imbalance)
//    14 the make-up rule (firmware)    25 J/N 61 with an I/Q error across the band
//    26 the jammer's direction drifts by 30 deg over the run (tracking from capture to capture)
//    27 captures needed after the jammer appears (J/N 30 and 50 dB)
//  Prints "STAP_PI_CSIM: PASS" or "STAP_PI_CSIM: FAIL".
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <complex>
#include <algorithm>
#include <stdint.h>
#include "stap_pi.h"
#include "gnss_aj_math.h"

#if AJ_NT != ST_NT || AJ_REF != ST_REF || AJ_NIMG != ST_NIMG
#error "gnss_aj_math.h and stap_pi.h disagree on the taps"
#endif

typedef std::complex<double> cd;
typedef std::vector<cd> cvec;

static int g_ncap_hit = -1;            // the last scenario's capture count to <= 3 dB
static double g_alpha = -1;            // > 0: fixed step (argv[3]); else the firmware rule aj_anlms_alpha(J/N)
static int g_passes = AJ_ANLMS_PASSES_DEF;
static int g_ncap = 20;

static unsigned long long rng_s = 88172645463325252ULL;
static double urand() {
    rng_s ^= rng_s << 13; rng_s ^= rng_s >> 7; rng_s ^= rng_s << 17;
    return ((rng_s >> 11) + 0.5) / 9007199254740992.0;
}
static double grand() { return std::sqrt(-2.0 * std::log(urand())) * std::cos(2 * M_PI * urand()); }
static uint32_t urand32() { return (uint32_t)(urand() * 4294967296.0); }

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
static double pwr(const cvec &x, int from, int to = -1) {
    double p = 0; int n = 0;
    if (to < 0) to = x.size();
    for (int i = from; i < to; i++, n++) p += std::norm(x[i]);
    return n ? p / n : 0;
}
static int q12(double v) {
    double r = std::floor(v + 0.5);
    if (r > 2047) r = 2047; if (r < -2048) r = -2048;
    return (int)r;
}


// ---- the core, driven sample by sample ---------------------------------------
static ap_uint<32> g_cfg = ST_CFG_BYPASS;
static ap_uint<32> g_nrm = 1 << ST_GFRAC;
static ap_uint<32> g_gain = ST_GAIN_ONE;
static ap_uint<32 * ST_NST> g_wts = 0;
static ap_uint<32 * 2 * ST_NIMG> g_wcj = 0;

static void set_cfg(int bypass, int shift, int ws = 0) {
    g_cfg = (bypass ? ST_CFG_BYPASS : 0u) | ((unsigned)(shift & 7) << ST_CFG_SH_LSB) |
            ((unsigned)(ws & 7) << ST_CFG_WS_LSB);
}
static void set_g2(int g2r) { g_nrm = 0; g_nrm(15, 0) = (ap_uint<16>)(ap_int<16>)g2r; }
static void set_weights(const int16_t *wre, const int16_t *wim) {   // the firmware's word order
    g_wts = 0; g_wcj = 0;
    for (int j = 0; j < ST_NST; j++) {
        g_wts(32 * j + 15, 32 * j) = (ap_uint<16>)(ap_int<16>)wre[j];
        g_wts(32 * j + 31, 32 * j + 16) = (ap_uint<16>)(ap_int<16>)wim[j];
    }
    for (int c = 0; c < 2 * ST_NIMG; c++) {
        g_wcj(32 * c + 15, 32 * c) = (ap_uint<16>)(ap_int<16>)wre[ST_NST + c];
        g_wcj(32 * c + 31, 32 * c + 16) = (ap_uint<16>)(ap_int<16>)wim[ST_NST + c];
    }
}

static void push(hls::stream<ap_uint<64> > &xs, hls::stream<ap_uint<32> > &ys,
                 int a, int b, int c, int d, cd *out) {
    int v[4] = {a, b, c, d};
    ap_uint<64> w = 0;
    for (int k = 0; k < 4; k++) w(16 * k + 15, 16 * k) = (ap_uint<16>)(ap_int<16>)v[k];
    xs.write(w);
    stap_pi(xs, ys, g_wts, g_wcj, g_cfg, g_nrm, g_gain);
    if (ys.empty()) { std::printf("core produced no output\n"); std::exit(2); }
    ap_uint<32> o = ys.read();
    if (out) *out = cd((double)(ap_int<16>)o(15, 0), (double)(ap_int<16>)o(31, 16));
}

static hls::stream<ap_uint<64> > g_xs;
static hls::stream<ap_uint<32> > g_ys;

// Runs (x1,x2) through the core after a 32-sample zero flush; returns y.
static cvec run_core_c(const cvec &x1, const cvec &x2) {
    cvec y(x1.size());
    for (int i = 0; i < 32; i++) push(g_xs, g_ys, 0, 0, 0, 0, 0);
    for (size_t i = 0; i < x1.size(); i++)
        push(g_xs, g_ys, q12(x1[i].real()), q12(x1[i].imag()), q12(x2[i].real()), q12(x2[i].imag()), &y[i]);
    return y;
}

// ---- scenario 1: bit-exact against an integer reference model -----------------
static int64_t satv(int64_t v, int64_t lo, int64_t hi) { return v < lo ? lo : v > hi ? hi : v; }

struct RefCore {
    int64_t h1r[ST_NT], h1i[ST_NT], h2r[ST_NT], h2i[ST_NT];
    RefCore() { std::memset(this, 0, sizeof *this); }
    void step(int x1r, int x1i, int x2r, int x2i, uint32_t cfg, uint32_t nrm, uint32_t gain,
              const int *wre, const int *wim, int &yr_o, int &yi_o) {
        int64_t g2r = (int16_t)(nrm & 0xFFFF);
        int64_t x2nr = (g2r * x2r + 1024) >> 11, x2ni = (g2r * x2i + 1024) >> 11;
        for (int k = ST_NT - 1; k > 0; k--) { h1r[k] = h1r[k - 1]; h1i[k] = h1i[k - 1]; h2r[k] = h2r[k - 1]; h2i[k] = h2i[k - 1]; }
        h1r[0] = x1r; h1i[0] = x1i; h2r[0] = x2nr; h2i[0] = x2ni;
        bool byp = cfg & 1;
        int ws = std::min((int)((cfg >> 12) & 7), ST_MAXWS), s = std::min((int)((cfg >> 8) & 7), ST_MAXSH);
        int64_t wr[ST_NW], wi[ST_NW];
        for (int j = 0; j < ST_NW; j++) { wr[j] = (byp || j == ST_REF) ? 0 : wre[j]; wi[j] = (byp || j == ST_REF) ? 0 : wim[j]; }
        const int Q = ST_WFRAC + ST_XFRAC;
        int64_t sr = 0, si = 0;
        for (int k = 0; k < ST_NT; k++) {
            if (k != ST_REF) { sr += (wr[k] * h1r[k] - wi[k] * h1i[k]) * 4; si += (wr[k] * h1i[k] + wi[k] * h1r[k]) * 4; }
            int j = ST_NT + k;
            sr += wr[j] * h2r[k] - wi[j] * h2i[k]; si += wr[j] * h2i[k] + wi[j] * h2r[k];
        }
        for (int c = 0; c < ST_NIMG; c++) {
            int a = ST_REF - ST_IMGH + c, j1 = ST_NST + c, j2 = ST_NST + ST_NIMG + c;
            int64_t ar1 = h1r[a], ai1 = -h1i[a], ar2 = h2r[a], ai2 = -h2i[a];
            sr += (wr[j1] * ar1 - wi[j1] * ai1) * 4 + (wr[j2] * ar2 - wi[j2] * ai2);
            si += (wr[j1] * ai1 + wi[j1] * ar1) * 4 + (wr[j2] * ai2 + wi[j2] * ar2);
        }
        int64_t wrnd = ws ? (1LL << (ws - 1)) : 0;
        int64_t ar = (h1r[ST_REF] << Q) - ((sr + wrnd) >> ws), ai = (h1i[ST_REF] << Q) - ((si + wrnd) >> ws);
        int64_t g16 = gain & 0xFFFF, half = 1LL << (Q - 1);
        if (g16 == ST_GAIN_ONE) {
            yr_o = (int)satv(((ar << s) + half) >> Q, -32768, 32767);
            yi_o = (int)satv(((ai << s) + half) >> Q, -32768, 32767);
        } else {
            const int UF = 6;
            int64_t ur = satv(((ar << s) + (1LL << (Q - UF - 1))) >> (Q - UF), -(1LL << 23), (1LL << 23) - 1);
            int64_t ui = satv(((ai << s) + (1LL << (Q - UF - 1))) >> (Q - UF), -(1LL << 23), (1LL << 23) - 1);
            int64_t gh = 1LL << (UF + ST_GAIN_FRAC - 1);
            yr_o = (int)satv((ur * g16 + gh) >> (UF + ST_GAIN_FRAC), -32768, 32767);
            yi_o = (int)satv((ui * g16 + gh) >> (UF + ST_GAIN_FRAC), -32768, 32767);
        }
    }
};

static int test_bitexact() {
    static RefCore ref;
    const int N = 300000;
    int bad = 0;
    uint32_t cfg = ST_CFG_BYPASS, nrm = 8192, gain = ST_GAIN_ONE;
    int wre[ST_NW] = {0}, wim[ST_NW] = {0};
    int16_t qre[ST_NW] = {0}, qim[ST_NW] = {0};
    double amp = 2047;
    for (int i = 0; i < N; i++) {
        if (i % 2000 == 0) {             // a new commit: weights, cfg, g2, gain
            int byp = (i / 2000) % 9 == 0 || urand() < 0.08;
            int sh = (int)(urand() * 8), ws = (urand() < 0.5) ? 0 : (int)(urand() * 8);
            cfg = (byp ? 1u : 0u) | ((uint32_t)sh << 8) | ((uint32_t)ws << 12) | (urand() < 0.3 ? 0x8000u : 0u);
            nrm = (urand() < 0.5) ? 8192u : (urand32() & 0xFFFFu);
            gain = (urand() < 0.4) ? (uint32_t)ST_GAIN_ONE : (urand32() & 0xFFFFu);
            double wa = (urand() < 0.3) ? 32767.0 : (urand() < 0.5 ? 6000.0 : 500.0);
            for (int j = 0; j < ST_NW; j++) {
                wre[j] = (int)std::max(-32768.0, std::min(32767.0, std::floor((urand() * 2 - 1) * (wa + 1))));
                wim[j] = (int)std::max(-32768.0, std::min(32767.0, std::floor((urand() * 2 - 1) * (wa + 1))));
                qre[j] = (int16_t)wre[j]; qim[j] = (int16_t)wim[j];
            }
            set_weights(qre, qim);
            amp = (urand() < 0.2) ? 2047.0 : (urand() < 0.5 ? 300.0 : 30.0);
        }
        int x[4];
        for (int k = 0; k < 4; k++) {
            double v = (urand() < 0.002) ? ((urand() < 0.5) ? -2048 : 2047) : std::floor((urand() * 2 - 1) * amp);
            x[k] = (int)std::max(-2048.0, std::min(2047.0, v));
        }
        g_cfg = cfg; g_nrm = nrm; g_gain = gain;
        cd y;
        push(g_xs, g_ys, x[0], x[1], x[2], x[3], &y);
        int ryr, ryi;
        ref.step(x[0], x[1], x[2], x[3], cfg, nrm, gain, wre, wim, ryr, ryi);
        if ((int)y.real() != ryr || (int)y.imag() != ryi) {
            if (bad < 5) std::printf("  sample %d: core (%d, %d), reference (%d, %d)\n", i, (int)y.real(), (int)y.imag(), ryr, ryi);
            bad++;
        }
    }
    g_cfg = ST_CFG_BYPASS; g_gain = ST_GAIN_ONE;
    int byp_bad = 0;
    std::vector<int> ha(20000), hb(20000);
    for (int i = 0; i < 20000; i++) {
        ha[i] = (int)std::floor((urand() * 2 - 1) * 2047); hb[i] = (int)std::floor((urand() * 2 - 1) * 2047);
        cd y;
        push(g_xs, g_ys, ha[i], hb[i], hb[i], ha[i], &y);
        if (i >= ST_REF && ((int)y.real() != ha[i - ST_REF] || (int)y.imag() != hb[i - ST_REF])) byp_bad++;
    }
    std::printf("scenario 1: %d samples, new weights / cfg / g2 / gain every 2000: %d mismatches; bypass: %d of %d differ from x1[n-%d]\n",
                N, bad, byp_bad, 20000 - ST_REF, ST_REF);
    return bad == 0 && byp_bad == 0;
}

// ---- scenario 14: firmware functions --------------------------------------------
static int test_makeup() {
    struct T { int cut; double out; int maxsh; int want; } t[] = {
        {0, 0, 7, 0}, {5, 0, 7, 0}, {6, 0, 7, 1}, {10, 0, 7, 1}, {34, 0.3, 7, 5}, {34, 4.8, 7, 5},
        {36, 3.3, 7, 5}, {42, 0, 7, 7}, {50, 0, 7, 7}, {40, 0, 4, 4},
        {34, 20, 7, 2}, {34, 40, 7, 0}, {26, 0.3, 7, 4}, {26, 0.3, 4, 4}, {-3, 0, 7, 0}};
    int bad = 0;
    for (auto &c : t) {
        int g = aj_makeup_shift(c.cut, c.out, c.maxsh);
        if (g != c.want) { std::printf("  cut %d dB, output %+.1f dB over noise, max %d: got %d, want %d\n", c.cut, c.out, c.maxsh, g, c.want); bad++; }
    }
    {
        double e61 = aj_adc_excess_db(61.0), e66 = aj_adc_excess_db(66.0), e71 = aj_adc_excess_db(71.0), e50 = aj_adc_excess_db(50.0);
        std::printf("ADC excess model: J/N 50 %.2f, 61 %.2f, 66 %.2f, 71 %.2f dB\n", e50, e61, e66, e71);
        if (e50 > 0.3 || std::fabs(e61 - 1.8) > 0.5 || std::fabs(e66 - 4.1) > 0.8 || std::fabs(e71 - 7.8) > 1.0) bad++;
    }
    {
        struct F { int cut; double out; int sh; double want; } f[] = {
            {0, 0, 0, 0}, {10, 0, 1, 4}, {34, 0.3, 5, 4}, {26, 0.3, 4, 2}, {34, 40, 0, 0},
            {34, 5, 5, 2}, {34, 4, 5, 3}, {50, 0, 7, 5.99}, {13, 0, 2, 1}};
        for (auto &c : f) {
            double g = aj_makeup_fine_db(c.cut, c.out, c.sh);
            if (std::fabs(g - c.want) > 1e-9) { std::printf("  fine: cut %d, out %.1f, shift %d: got %.2f, want %.2f\n", c.cut, c.out, c.sh, g, c.want); bad++; }
        }
    }
    std::printf("make-up rule: %d wrong\n", bad);
    return bad == 0;
}

// pass limits that depend on the tap count (8 + 8 / 6 + 6), from the C simulation
#if ST_NT >= 8
#define LIM2 35
#define LIM6 45
#define LIM8 48
#define LIM11 48
#define LIM12 48
#define LIM13 44
#else
#define LIM2 33
#define LIM6 43
#define LIM8 46
#define LIM11 46
#define LIM12 46
#define LIM13 42
#endif

// ---- jammer scenarios ------------------------------------------------------------
struct Scen { double jnr_db; bool cw; double echo; double rx2_db; bool iq; double cwf; double satf; double chip; double delay;
              int ncap = 20; double drift = 0; double iqfd = 0; int echod = 3; };

static void iq_freqdep(cvec &x, double fd) {
    if (fd == 0) return;
    std::vector<double> q(x.size());
    for (size_t i = 0; i < x.size(); i++) q[i] = x[i].imag();
    for (size_t i = 1; i + 1 < x.size(); i++) x[i] = cd(x[i].real(), q[i] + fd * (q[i + 1] - q[i - 1]) / 2);
}
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

// the adaptive taps of sample n (as the core multiplies them), d = x1[n-R]
static void taps_of(const cvec &x1, const cvec &x2n, int n, cd *u) {
    int q = 0;
    for (int k = 0; k < ST_NT; k++) if (k != ST_REF) u[q++] = x1[n - k];
    for (int k = 0; k < ST_NT; k++) u[q++] = x2n[n - k];
    for (int c = 0; c < ST_NIMG; c++) u[q++] = std::conj(x1[n - (ST_REF - ST_IMGH + c)]);
    for (int c = 0; c < ST_NIMG; c++) u[q++] = std::conj(x2n[n - (ST_REF - ST_IMGH + c)]);
}
// Wiener weights of y = d - w.u over [a, b): R w = p, R_kj = E[u_j conj(u_k)], p_k = E[d conj(u_k)]
static std::vector<cd> wiener(const cvec &x1, const cvec &x2n, int a, int b) {
    const int M = AJ_MAXDIM;
    std::vector<cd> R(M * M, 0.0), p(M, 0.0), w(M, 0.0);
    cd u[AJ_MAXDIM];
    for (int n = a; n < b; n++) {
        taps_of(x1, x2n, n, u);
        cd d = x1[n - ST_REF];
        for (int k = 0; k < M; k++) {
            cd ck = std::conj(u[k]);
            p[k] += d * ck;
            for (int j = 0; j < M; j++) R[k * M + j] += u[j] * ck;
        }
    }
    double tr = 0;
    for (int k = 0; k < M; k++) tr += R[k * M + k].real();
    for (int k = 0; k < M; k++) R[k * M + k] += 1e-9 * tr;
    for (int c = 0; c < M; c++) {                     // Gaussian elimination, partial pivoting
        int pv = c;
        for (int r = c + 1; r < M; r++) if (std::abs(R[r * M + c]) > std::abs(R[pv * M + c])) pv = r;
        if (pv != c) { for (int j = 0; j < M; j++) std::swap(R[c * M + j], R[pv * M + j]); std::swap(p[c], p[pv]); }
        for (int r = c + 1; r < M; r++) {
            cd f = R[r * M + c] / R[c * M + c];
            for (int j = c; j < M; j++) R[r * M + j] -= f * R[c * M + j];
            p[r] -= f * p[c];
        }
    }
    for (int c = M - 1; c >= 0; c--) {
        cd s = p[c];
        for (int j = c + 1; j < M; j++) s -= R[c * M + j] * w[j];
        w[c] = s / R[c * M + c];
    }
    return w;
}
static cvec apply_w(const std::vector<cd> &w, const cvec &x1, const cvec &x2n) {
    cvec y(x1.size(), 0.0);
    cd u[AJ_MAXDIM];
    for (size_t n = ST_NT; n < x1.size(); n++) {
        taps_of(x1, x2n, n, u);
        cd s = x1[n - ST_REF];
        for (int j = 0; j < AJ_MAXDIM; j++) s -= w[j] * u[j];
        y[n] = s;
    }
    return y;
}


static int test_jammer(int id, Scen sc) {
    const int NT = sc.ncap * AJ_NSAMP, NE = 40000, N = NT + NE;
    const double nrms = 20.0, fsat = (sc.satf != 0) ? sc.satf : -9.08 / 50.0, cwf = (sc.cwf != 0) ? sc.cwf : 0.06,
                 chip_step = (sc.chip != 0) ? sc.chip : 1.023 / 50.0;
    const double IQ1_EPS = 0.004, IQ1_PH = 0.3, IQ2_EPS = -0.003, IQ2_PH = -0.4;
    cvec J(N), J2, n1(N), n2(N), s1(N), s2(N);
    if (sc.cw) for (int i = 0; i < N; i++) J[i] = std::polar(1.0, 2 * M_PI * cwf * i);
    else {
        for (int i = 0; i < N; i++) J[i] = cd(grand(), grand());
        J = fir(J, lowpass(0.42, 41), 20);
    }
    cvec h2 = frac_delay((sc.delay != 0) ? sc.delay : 0.15, 21);
    cvec hd = {cd(0.03, 0.01), 1.0, cd(-0.02, 0.015)};
    J2 = fir(fir(J, h2, 10), hd, 1);
    for (int i = 0; i < N; i++) {      // RX2 path: 0.8 dB, 40 deg (+ a drift over the run, scenario 26)
        double ph = 40.0 + sc.drift * (double)i / N;
        J2[i] *= std::polar(std::pow(10, 0.8 / 20), ph * M_PI / 180);
    }
    if (sc.echo > 0) {
        cvec e1(N), e2(N);
        for (int i = sc.echod; i < N; i++) { e1[i] = sc.echo * std::polar(1.0, 1.3) * J[i - sc.echod]; e2[i] = sc.echo * std::polar(1.0, 2.9) * J[i - sc.echod]; }
        for (int i = 0; i < N; i++) { J[i] += e1[i]; J2[i] += e2[i]; }
    }
    double pj = pwr(J, 0), pn = 2 * nrms * nrms;
    double js = (sc.jnr_db > -90) ? std::sqrt(pn * std::pow(10, sc.jnr_db / 10) / pj) : 0.0;
    for (int i = 0; i < N; i++) { J[i] *= js; J2[i] *= js; }
    int chip = 1; double cph = 0;
    for (int i = 0; i < N; i++) {
        n1[i] = cd(grand(), grand()) * nrms;
        n2[i] = cd(grand(), grand()) * nrms;
        cph += chip_step; if (cph >= 1) { cph -= 1; chip = urand() < 0.5 ? -1 : 1; }
        s1[i] = std::polar(0.1 * nrms * chip, 2 * M_PI * fsat * i);
        s2[i] = s1[i] * std::polar(1.0, -1.1);
    }
    double a2 = std::pow(10, sc.rx2_db / 20);
    for (int i = 0; i < N; i++) { J2[i] *= a2; n2[i] *= a2; s2[i] *= a2; }

    // g2 as the firmware learns it: the two noise floors at the clean gain
    double g2 = 1.0, rx2n = 1.0;
    {
        cvec q1(n1), q2(n2);
        if (sc.iq) { iq_freqdep(q1, sc.iqfd); iq_freqdep(q2, -sc.iqfd); iq_imbalance(q1, IQ1_EPS, IQ1_PH); iq_imbalance(q2, IQ2_EPS, IQ2_PH); }
        for (int i = 0; i < N; i++) { q1[i] = cd(q12(q1[i].real()), q12(q1[i].imag())); q2[i] = cd(q12(q2[i].real()), q12(q2[i].imag())); }
        g2 = std::sqrt(pwr(q1, 0) / pwr(q2, 0));
        if (g2 > 32767.0 / 8192) g2 = 32767.0 / 8192;
        if (g2 < 0.125) g2 = 0.125;
        g2 = std::floor(g2 * 8192 + 0.5) / 8192;
        rx2n = g2 * g2 * pwr(q2, 0) / pwr(q1, 0);
    }

    cvec x1(N), x2(N), jx1(N), jx2(N), nx1(N), nx2(N);
    for (int i = 0; i < N; i++) {
        x1[i] = J[i] + n1[i] + s1[i];  x2[i] = J2[i] + n2[i] + s2[i];
        jx1[i] = J[i]; jx2[i] = J2[i]; nx1[i] = n1[i]; nx2[i] = n2[i];
    }
    if (sc.iq) {
        iq_freqdep(x1, sc.iqfd);  iq_freqdep(x2, -sc.iqfd);
        iq_freqdep(jx1, sc.iqfd); iq_freqdep(jx2, -sc.iqfd);
        iq_freqdep(nx1, sc.iqfd); iq_freqdep(nx2, -sc.iqfd);
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

    std::printf("scenario %d: J/N %.0f dB%s%s, RX gain cut %.1f dB", id, sc.jnr_db,
                sc.jnr_db < -90 ? " (no jammer)" : sc.cw ? " CW" : " wideband", sc.echo > 0 ? " + reflection" : "", cut_db);
    if (sc.rx2_db != 0) std::printf(", RX2 %+.0f dB (g2 %.3f)", sc.rx2_db, g2);
    if (sc.iq) std::printf(", I/Q image rejection %.1f / %.1f dB", irr_db(IQ1_EPS, IQ1_PH), irr_db(IQ2_EPS, IQ2_PH));
    if (sc.drift != 0) std::printf(", direction drifting %.0f deg over the run", sc.drift);
    std::printf("\n");

    const int e0 = NT;
    auto seg = [&](const cvec &v) { return cvec(v.begin() + e0, v.end()); };
    struct Meas { double supp, loss, out_db, res, lvl; };
    auto snr_of = [&](const cvec &y, const cvec &ref, int dly) {
        cd c = 0; double pr = 0, py = 0; int cnt = 0;
        for (size_t i = 32; i < y.size(); i++) {
            cd rr = ref[i - dly];
            c += y[i] * std::conj(rr); pr += std::norm(rr); py += std::norm(y[i]); cnt++;
        }
        double ps = std::norm(c) / pr;
        return (ps / cnt) / (py / cnt - ps / cnt);
    };
    auto clean_snr = [&]() {
        cvec ref = seg(s1), cl(ref.size()), n1s = seg(n1);
        for (size_t i = 0; i < ref.size(); i++) cl[i] = n1s[i] / g + ref[i] / g;
        return snr_of(cl, ref, 0);
    };
    // the held core weights (NLMS off), output make-up shift sh
    auto measure = [&](int shift, int ws) {
        Meas m;
        double sc4 = std::pow(4.0, shift);
        set_cfg(0, shift, ws);
        cvec yj = run_core_c(seg(jx1), seg(jx2));
        cvec yx = run_core_c(seg(x1), seg(x2));
        cvec yn = run_core_c(seg(nx1), seg(nx2));
        m.supp = (sc.jnr_db > -90) ? 10 * std::log10(pwr(seg(J), 0) / std::max(pwr(yj, 32) / sc4, 1e-9)) : 0;
        m.loss = 10 * std::log10(clean_snr() / snr_of(yx, seg(s1), ST_REF));
        m.out_db = 10 * std::log10(pwr(yx, 32) / sc4 / pwr(seg(x1), 0));
        m.lvl = 10 * std::log10(pwr(yx, 32) / pn);
        m.res = 10 * std::log10(pwr(yx, 32) / std::max(pwr(yn, 32), 1e-9));
        return m;
    };

    if (id == 3) {
        set_cfg(1, 0); set_g2((int)(g2 * 8192 + 0.5));
        cvec yb = run_core_c(seg(x1), seg(x2)), xs = seg(x1);
        int bad = 0;
        for (size_t i = ST_REF; i < yb.size(); i++) if (yb[i] != xs[i - ST_REF]) bad++;
        std::printf("  bypass            : %d of %d samples differ from x1[n-%d]\n", bad, (int)yb.size() - ST_REF, ST_REF);
        if (bad) return 0;
    }

    // ---- the ARM: NLMS over the captures, one after the other, weights from 0 ---------------
    const double nz_cap = pn * g * g + 0.17;           // receiver noise at the capture's gain
    aj_result_t r; std::memset(&r, 0, sizeof r);
    auto train = [&](double g2v, const char *tag, std::vector<double> *res_c, int *c3) {
        static uint32_t buf[2 * AJ_NSAMP];
        aj_nlms_reset();
        if (res_c) res_c->clear();
        if (c3) *c3 = -1;
        int above = 0;
        const double al = (g_alpha > 0) ? g_alpha : aj_anlms_alpha(sc.jnr_db);   // the firmware: from the detected J/N
        for (int c = 0; c < sc.ncap; c++) {
            for (int t = 0; t < AJ_NSAMP; t++) {
                cd a = x1[c * AJ_NSAMP + t], b = x2[c * AJ_NSAMP + t];
                buf[2 * t]     = (uint32_t)(uint16_t)(int16_t)a.real() | ((uint32_t)(uint16_t)(int16_t)a.imag() << 16);
                buf[2 * t + 1] = (uint32_t)(uint16_t)(int16_t)b.real() | ((uint32_t)(uint16_t)(int16_t)b.imag() << 16);
            }
            int rc = aj_nlms_solve(buf, AJ_NSAMP, g2v, (g2v == g2) ? rx2n : rx2n / (g2 * g2), al, g_passes, &r);
            if (rc < 0) { std::printf("aj_nlms_solve failed\n"); std::exit(2); }
            double rd = 10 * std::log10(r.arm_pout / (nz_cap * (1.0 + r.wn2)));   // the ARM's own residual
            if (res_c) res_c->push_back(rd);
            if (rd > 3.0) above = c + 1;
        }
        if (c3) *c3 = (above < sc.ncap) ? above + 1 : -1;  // first capture after which it stays <= 3 dB
        set_weights(r.wre, r.wim);
        set_g2((int)std::floor(g2v * 8192 + 0.5));
        std::printf("  %-18s: ARM NLMS alpha %.3f, %d passes, %d captures of %d samples, weight scale 2^%d\n",
                    tag, al, g_passes, sc.ncap, AJ_NSAMP, r.ws);
    };

    std::vector<double> rc;
    int c3 = -1;
    train(g2, "ARM NLMS", &rc, &c3);
    double wn2 = r.wn2;
    Meas m0 = measure(0, r.ws);
    int sh = aj_makeup_shift((int)cut_db, m0.res + aj_db10(1.0 + wn2), ST_MAXSH);
    Meas m = sh ? measure(sh, r.ws) : m0;
    double wmax = 0;
    for (int j = 0; j < ST_NW; j++) wmax = std::max(wmax, std::hypot((double)r.wre[j], (double)r.wim[j]) / (4096.0 * (1 << r.ws)));

    // optimum weights of the same taps (double precision)
    cvec x2n(N);
    for (int i = 0; i < N; i++) x2n[i] = x2[i] * g2;
    std::vector<cd> wo = wiener(x1, x2n, std::max(ST_NT, NT - 100000), NT);
    cvec j2n(jx2), n2n(nx2);
    for (auto &v : j2n) v *= g2;
    for (auto &v : n2n) v *= g2;
    cvec oj = apply_w(wo, seg(jx1), seg(j2n)), ox = apply_w(wo, seg(x1), seg(x2n)), on = apply_w(wo, seg(nx1), seg(n2n));
    double o_supp = (sc.jnr_db > -90) ? 10 * std::log10(pwr(seg(J), 0) / std::max(pwr(oj, 32), 1e-9)) : 0;
    double o_res = 10 * std::log10(pwr(ox, 32) / pwr(on, 32));

    std::printf("  core, ARM weights : suppression %.1f dB, residual %+.2f dB, satellite SNR loss %.2f dB, output %+.2f dB re RX1\n",
                m.supp, m.res, m.loss, m.out_db);
    std::printf("  ARM residual      : per capture");
    for (size_t k = 0; k < rc.size(); k++) if (k < 6 || k + 3 >= rc.size()) std::printf(" %+.1f", rc[k]); else if (k == 6) std::printf(" ...");
    std::printf(" dB\n");
    std::printf("  optimum (Wiener)  : suppression %.1f dB, residual %+.2f dB (same %d taps, double precision)\n", o_supp, o_res, AJ_MAXDIM);
    if (sc.jnr_db > -90)
        std::printf("  convergence       : residual <= 3 dB from capture %d on (of %d)\n", c3, sc.ncap);
    std::printf("  weights           : |w|^2 %.2f, largest |w| %.3f; make-up shift %d, output level %+.1f dB against the clean level\n",
                wn2, wmax, sh, m.lvl);

    g_ncap_hit = c3;
    // Limits from the C simulation (20 captures, alpha and passes as the firmware's defaults).
    int ok = 1;
    switch (id) {
    case 2:  ok = m.supp >= LIM2 && m.loss <= 3 && m.res <= 1.5; break;
    case 3:  ok = std::fabs(m.out_db) <= 0.5 && std::fabs(m.loss) <= 1.0 && m.res <= 0.5; break;
    case 4:  ok = m.supp >= 50 && m.loss <= 3 && m.res <= 0.5; break;
    // 6 + 6 taps (reference delay 3) cannot reach a reflection 3 samples later on RX1
    case 5:  ok = (ST_NT >= 8) ? (m.supp >= 30 && m.res <= 1.5) : (m.supp >= 20 && m.res <= o_res + 1.5); break;
    case 6:  ok = m.supp >= LIM6 && m.loss <= 3.0; break;
    case 7: {
        train(1.0, "no normalisation", 0, 0);
        Meas mn = measure(0, r.ws);
        std::printf("  no normalisation  : suppression %.1f dB, residual %+.2f dB\n", mn.supp, mn.res);
        ok = m.supp >= 30 && m.supp >= mn.supp + 5;
        break; }
    case 8:  ok = m.supp >= LIM8; break;
    case 9:  ok = m.supp >= 60 && m.res <= 1.5 && m.loss <= 3.5; break;
    case 11: ok = m.supp >= LIM11 && m.lvl >= -6.5 && m.lvl <= 3.5; break;
    case 12: ok = m.supp >= LIM12 && m.lvl >= -6.5 && m.lvl <= 3.5; break;
    case 13: ok = m.supp >= LIM13 && m.loss <= 1.5 && m.lvl >= -6.5 && m.lvl <= 3.5; break;
    case 25: ok = m.supp >= LIM11; break;
    case 26: {
        // ARM residual while the direction keeps drifting: 2nd quarter of the captures against the last
        int q = rc.size() / 4;
        double r1 = 0, r4 = 0;
        for (int k = q; k < 2 * q; k++) r1 += rc[k];
        for (int k = 3 * q; k < 4 * q; k++) r4 += rc[k];
        r1 /= q; r4 /= q;
        std::printf("  tracking          : ARM residual %+.2f dB (2nd quarter), %+.2f dB (last quarter)\n", r1, r4);
        ok = r4 <= r1 + 1.5 && r4 <= 4.0;
        break; }
    }
    return ok;
}


// ---- scenario 27: captures needed after the jammer appears -------------------------------
static int test_convergence() {
    double jns[] = {30, 50};
    int lim[] = {3, 10};
    int ok = 1;
    for (int k = 0; k < 2; k++) {
        Scen sc = {jns[k], false, 0, 0, true, 0, 0, 0, 0};
        sc.ncap = 20;
        test_jammer(27, sc);
        int good = g_ncap_hit >= 1 && g_ncap_hit <= lim[k];
        std::printf("  -> J/N %.0f dB: %s (<= 3 dB from capture %d, limit %d)\n", jns[k], good ? "in time" : "TOO SLOW", g_ncap_hit, lim[k]);
        ok = ok && good;
    }
    return ok;
}

int main(int argc, char **argv) {
    int scen = (argc > 1) ? std::atoi(argv[1]) : 2;
    double jn_arg = (argc > 2) ? std::atof(argv[2]) : -1000;
    if (argc > 3 && std::atof(argv[3]) > 0) g_alpha = std::atof(argv[3]);
    if (argc > 4 && std::atoi(argv[4]) > 0) g_ncap = std::atoi(argv[4]);
    if (argc > 5 && std::atoi(argv[5]) > 0) g_passes = std::atoi(argv[5]);
    auto J = [&](double d) { return jn_arg > -999 ? jn_arg : d; };
    int ok = 1;
    Scen sc = {0, false, 0, 0, false, 0, 0, 0, 0};
    switch (scen) {
    case 1: ok = test_bitexact(); break;
    case 14: ok = test_makeup(); break;
    case 27: ok = test_convergence(); break;
    default:
        switch (scen) {
        case 2: sc = {J(30), false, 0, 0, false, 0, 0, 0, 0}; break;
        case 3: sc = {-99, false, 0, 0, false, 0, 0, 0, 0}; break;
        case 4: sc = {J(40), true, 0, 0, false, 0, 0, 0, 0}; break;
        case 5: sc = {J(30), false, 0.5, 0, false, 0, 0, 0, 0}; break;
        case 6: sc = {J(50), false, 0, 0, true, 0, 0, 0, 0}; break;
        case 7: sc = {J(40), false, 0, -25, false, 0, 0, 0, 0}; break;
        case 8: sc = {J(60), false, 0, 0, true, 0, 0, 0, 0}; break;
        case 9: sc = {J(62), true, 0, 0, true, -7.06 / 61.44, -7.06 / 61.44, 1.023 / 61.44, 0}; break;
        case 11: case 12: case 13:
            sc = {J(scen == 11 ? 61.0 : scen == 12 ? 51.0 : 41.0), false, 0, 0, true, 0, -0.02 / 18.0, 1.023 / 18.0, 0.055}; break;
        case 25: sc = {J(61), false, 0, 0, true, 0, -0.02 / 18.0, 1.023 / 18.0, 0.055}; sc.iqfd = 0.03; break;
        case 26: sc = {J(50), false, 0, 0, true, 0, 0, 0, 0}; sc.drift = 30; sc.ncap = 40; break;
        default: std::printf("unknown scenario %d\n", scen); return 1;
        }
        if (argc > 4 && std::atoi(argv[4]) > 0) sc.ncap = g_ncap;
        ok = test_jammer(scen, sc);
    }
    std::printf("STAP_PI_CSIM: %s (scenario %d)\n", ok ? "PASS" : "FAIL", scen);
    return ok ? 0 : 1;
}
