// ============================================================================
//  testbench.cpp  --  C simulation of the NLMS-only space-time core (nlms_pi 1.0,
//                     antsdr-fw-patch crpa13)
//
//  The weights are adapted by the core itself (hardware NLMS); nothing is
//  solved on the ARM. The step word comes from the firmware's own
//  aj_nlms_word() and the output make-up from aj_makeup_shift()
//  (gnss_crpa/src/gnss_aj_math.c, compiled into this testbench).
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
//  Each jammer scenario: the core runs with NLMS on over a training stretch
//  (the jammer switched on at sample 0, weights from 0), then the weights are
//  held (nlms[31] = 0) and the jammer, the noise and the sum are passed
//  through separately: suppression, residual (output / output of the same
//  weights on the noise alone, 0 dB = fully nulled), satellite SNR loss. The
//  same figures for the OPTIMUM weights of the same taps (Wiener solution
//  in double precision from the training data) show how close NLMS gets.
//
//  Scenario (argv[1]); each needs a freshly reset core, so the csim script
//  runs one per process. argv[2] / argv[3] override J/N and the step alpha.
//    1  bit-exact: random inputs, random cfg / g2 / gain / NLMS words (steps
//       large enough to saturate the weights), bypass and weight-scale changes,
//       against an integer reference model of the core; also the weight monitor
//    2  wideband jammer 30 dB
//    3  no jammer: bypass is exactly x1[n-R]; with the filter in and NLMS on,
//       output within 0.5 dB of RX1, satellite loss <= 0.5 dB
//    4  CW jammer 40 dB
//    5  wideband jammer 30 dB plus a reflection (-6 dB, 3 samples later)
//    6  wideband jammer 50 dB with I/Q imbalance (image taps)
//    7  RX2 25 dB weaker than RX1, wideband 40 dB: with and without g2
//    8  wideband jammer 60 dB with I/Q imbalance
//    9  CW 62 dB on top of the satellite, I/Q imbalance (weight scale)
//    11, 12, 13  barrage over the whole band, I/Q imbalance, J/N 61, 51, 41 dB
//       (the field cases)
//    14 the make-up rule and the NLMS step word (firmware functions)
//    25 J/N 61 with an I/Q error that changes across the band
//    26 the jammer's direction drifts by 30 deg during the run: NLMS follows,
//       held weights do not
//    27 convergence time after the jammer appears (J/N 30, 50 and 60 dB)
//  Prints "NLMS_PI_CSIM: PASS" or "NLMS_PI_CSIM: FAIL".
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <complex>
#include <algorithm>
#include <stdint.h>
#include "nlms_pi.h"
#include "gnss_aj_math.h"

#if AJ_NT != NL_NT || AJ_REF != NL_REF || AJ_NIMG != NL_NIMG || AJ_NLMS_RF != NL_RF || AJ_NLMS_CS != NL_CS
#error "gnss_aj_math.h and nlms_pi.h disagree on the taps or the NLMS format"
#endif

typedef std::complex<double> cd;
typedef std::vector<cd> cvec;

static int g_t3 = -1;                  // the last scenario's convergence sample (<= 3 dB)
static double g_alpha = -1;            // > 0: fixed step (argv[3]); else the firmware's rule aj_nlms_alpha(|rho1|)

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
static ap_uint<32> g_cfg = NL_CFG_BYPASS;
static ap_uint<32> g_nrm = 1 << NL_GFRAC;
static ap_uint<32> g_gain = NL_GAIN_ONE;
static ap_uint<32> g_nlms = 0;
static int g_wre[NL_NW], g_wim[NL_NW];        // the weight monitor, as gnss_passthrough keeps it

static void set_cfg(int bypass, int shift, int ws = 0) {
    g_cfg = (bypass ? NL_CFG_BYPASS : 0u) | ((unsigned)(shift & 7) << NL_CFG_SH_LSB) |
            ((unsigned)(ws & 7) << NL_CFG_WS_LSB);
}
static void set_g2(int g2r) { g_nrm = 0; g_nrm(15, 0) = (ap_uint<16>)(ap_int<16>)g2r; }

static void push(hls::stream<ap_uint<64> > &xs, hls::stream<ap_uint<32> > &ys,
                 int a, int b, int c, int d, cd *out, uint64_t *mon = 0) {
    int v[4] = {a, b, c, d};
    ap_uint<64> w = 0;
    for (int k = 0; k < 4; k++) w(16 * k + 15, 16 * k) = (ap_uint<16>)(ap_int<16>)v[k];
    xs.write(w);
    ap_uint<40> wm = 0;
    nlms_pi(xs, ys, g_cfg, g_nrm, g_gain, g_nlms, wm);
    if (ys.empty()) { std::printf("core produced no output\n"); std::exit(2); }
    ap_uint<32> o = ys.read();
    if (out) *out = cd((double)(ap_int<16>)o(15, 0), (double)(ap_int<16>)o(31, 16));
    int j = (int)wm(39, 32);
    if (j < NL_NW) { g_wre[j] = (int)(ap_int<16>)wm(15, 0); g_wim[j] = (int)(ap_int<16>)wm(31, 16); }
    if (mon) *mon = (uint64_t)wm;
}

// The core's state persists between calls (static), as in hardware.
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

// |w|^2 of the monitored weights (as used: / 2^ws), the RX2 taps weighted by rx2n
static double mon_wn2(int ws, double rx2n) {
    double s = 0;
    for (int j = 0; j < NL_NW; j++) {
        double p = ((double)g_wre[j] * g_wre[j] + (double)g_wim[j] * g_wim[j]) / std::pow(4096.0 * (1 << ws), 2);
        s += (j >= NL_NT && j < 2 * NL_NT) ? p * rx2n : p;
    }
    return s;
}

// ---- scenario 1: bit-exact against an integer reference model -----------------
static int64_t satv(int64_t v, int64_t lo, int64_t hi) { return v < lo ? lo : v > hi ? hi : v; }
static int64_t wrap(int64_t v, int bits) {           // two's complement wrap to 'bits'
    uint64_t m = (bits >= 64) ? ~0ULL : ((1ULL << bits) - 1);
    uint64_t u = (uint64_t)v & m;
    if (bits < 64 && (u >> (bits - 1)) & 1) u |= ~m;
    return (int64_t)u;
}

struct RefCore {
    int64_t h1r[NL_NT], h1i[NL_NT], h2r[NL_NT], h2i[NL_NT];
    int64_t d1r[NL_NT], d1i[NL_NT], d2r[NL_NT], d2i[NL_NT];
    int64_t xb[NL_HB][4], ehr[NL_HB], ehi[NL_HB];
    int wp, ws_prev, mon;
    int64_t cwr[NL_NW], cwi[NL_NW];
    RefCore() { std::memset(this, 0, sizeof *this); }

    void step(int x1r, int x1i, int x2r, int x2i, uint32_t cfg, uint32_t nrm, uint32_t gain, uint32_t nlms,
              int &yr_o, int &yi_o, uint64_t &wmon) {
        const int RF = NL_RF;
        int64_t g2r = (int16_t)(nrm & 0xFFFF);
        int64_t x2nr = (g2r * x2r + 1024) >> 11, x2ni = (g2r * x2i + 1024) >> 11;
        int rp = (wp - NL_D) & (NL_HB - 1);
        int64_t xo[4] = {xb[rp][0], xb[rp][1], xb[rp][2], xb[rp][3]};
        int64_t eor = ehr[rp], eoi = ehi[rp];
        xb[wp][0] = x1r; xb[wp][1] = x1i; xb[wp][2] = x2nr; xb[wp][3] = x2ni;
        for (int k = NL_NT - 1; k > 0; k--) {
            h1r[k] = h1r[k - 1]; h1i[k] = h1i[k - 1]; h2r[k] = h2r[k - 1]; h2i[k] = h2i[k - 1];
            d1r[k] = d1r[k - 1]; d1i[k] = d1i[k - 1]; d2r[k] = d2r[k - 1]; d2i[k] = d2i[k - 1];
        }
        h1r[0] = x1r; h1i[0] = x1i; h2r[0] = x2nr; h2i[0] = x2ni;
        d1r[0] = xo[0]; d1i[0] = xo[1]; d2r[0] = xo[2]; d2i[0] = xo[3];

        bool byp = cfg & 1;
        int ws = std::min((int)((cfg >> 12) & 7), NL_MAXWS), s = std::min((int)((cfg >> 8) & 7), NL_MAXSH);
        bool clr = byp || ws != ws_prev;
        ws_prev = ws;
        bool nl = ((nlms >> 31) & 1) && !byp;
        int64_t nm = nlms & 0xFFFF, nsh = (nlms >> 16) & 31;

        int64_t wr[NL_NW], wi[NL_NW];
        for (int j = 0; j < NL_NW; j++) {
            if (j == NL_REF) { wr[j] = wi[j] = 0; continue; }
            int64_t zr, zi;
            if (j < NL_NT) { zr = d1r[j] * 2; zi = wrap(-(d1i[j] * 2), 17); }
            else if (j < 2 * NL_NT) { zr = d2r[j - NL_NT] >> 1; zi = wrap(-(d2i[j - NL_NT] >> 1), 17); }
            else if (j < 2 * NL_NT + NL_NIMG) { int a = NL_REF - NL_IMGH + (j - 2 * NL_NT); zr = d1r[a] * 2; zi = d1i[a] * 2; }
            else { int a = NL_REF - NL_IMGH + (j - 2 * NL_NT - NL_NIMG); zr = d2r[a] >> 1; zi = d2i[a] >> 1; }
            int64_t pr = eor * zr - eoi * zi, pi = eor * zi + eoi * zr;
            int64_t dr = (pr + (1 << (NL_CS - 1))) >> NL_CS, di = (pi + (1 << (NL_CS - 1))) >> NL_CS;
            const int64_t dhi = (1LL << (16 + RF + 2)) - 1, dlo = -(1LL << (16 + RF + 2));
            dr = satv(dr, dlo, dhi); di = satv(di, dlo, dhi);
            const int64_t whi = 32767LL << RF, wlo = -(32768LL << RF);
            if (clr) { cwr[j] = 0; cwi[j] = 0; }
            else if (nl) { cwr[j] = satv(cwr[j] + dr, wlo, whi); cwi[j] = satv(cwi[j] + di, wlo, whi); }
            wr[j] = (cwr[j] + (1 << (RF - 1))) >> RF;
            wi[j] = (cwi[j] + (1 << (RF - 1))) >> RF;
        }
        wmon = ((uint64_t)(mon & 0xFF) << 32) | ((uint64_t)(uint16_t)wi[mon] << 16) | (uint64_t)(uint16_t)wr[mon];
        mon = (mon == NL_NW - 1) ? 0 : mon + 1;

        const int Q = NL_WFRAC + NL_XFRAC;
        int64_t sr = 0, si = 0;
        for (int k = 0; k < NL_NT; k++) {
            if (k != NL_REF) { sr += (wr[k] * h1r[k] - wi[k] * h1i[k]) * 4; si += (wr[k] * h1i[k] + wi[k] * h1r[k]) * 4; }
            int j = NL_NT + k;
            sr += wr[j] * h2r[k] - wi[j] * h2i[k]; si += wr[j] * h2i[k] + wi[j] * h2r[k];
        }
        for (int c = 0; c < NL_NIMG; c++) {
            int a = NL_REF - NL_IMGH + c, j1 = 2 * NL_NT + c, j2 = 2 * NL_NT + NL_NIMG + c;
            int64_t ar1 = h1r[a], ai1 = -h1i[a], ar2 = h2r[a], ai2 = -h2i[a];
            sr += (wr[j1] * ar1 - wi[j1] * ai1) * 4 + (wr[j2] * ar2 - wi[j2] * ai2);
            si += (wr[j1] * ai1 + wi[j1] * ar1) * 4 + (wr[j2] * ai2 + wi[j2] * ar2);
        }
        int64_t wrnd = ws ? (1LL << (ws - 1)) : 0;
        int64_t ar = (h1r[NL_REF] << Q) - ((sr + wrnd) >> ws), ai = (h1i[NL_REF] << Q) - ((si + wrnd) >> ws);

        int64_t yr = satv((ar + (1 << (Q - 1))) >> Q, -131072, 131071), yi = satv((ai + (1 << (Q - 1))) >> Q, -131072, 131071);
        int64_t rh = nsh ? (1LL << (nsh - 1)) : 0;
        const int64_t emax = (1LL << (NL_EW - 1)) - 1, emin = -(1LL << (NL_EW - 1));
        ehr[wp] = satv((yr * nm + rh) >> nsh, emin, emax);
        ehi[wp] = satv((yi * nm + rh) >> nsh, emin, emax);
        wp = (wp + 1) & (NL_HB - 1);

        int64_t g16 = gain & 0xFFFF, half = 1LL << (Q - 1);
        if (g16 == NL_GAIN_ONE) {
            yr_o = (int)satv(((ar << s) + half) >> Q, -32768, 32767);
            yi_o = (int)satv(((ai << s) + half) >> Q, -32768, 32767);
        } else {
            const int UF = 6;
            int64_t ur = satv(((ar << s) + (1LL << (Q - UF - 1))) >> (Q - UF), -(1LL << 23), (1LL << 23) - 1);
            int64_t ui = satv(((ai << s) + (1LL << (Q - UF - 1))) >> (Q - UF), -(1LL << 23), (1LL << 23) - 1);
            int64_t gh = 1LL << (UF + NL_GAIN_FRAC - 1);
            yr_o = (int)satv((ur * g16 + gh) >> (UF + NL_GAIN_FRAC), -32768, 32767);
            yi_o = (int)satv((ui * g16 + gh) >> (UF + NL_GAIN_FRAC), -32768, 32767);
        }
    }
};

static int test_bitexact() {
    static RefCore ref;
    const int N = 400000;
    int bad = 0, badm = 0, sat_seen = 0, nonzero = 0;
    uint32_t cfg = NL_CFG_BYPASS, nrm = 8192, gain = NL_GAIN_ONE, nl = 0;
    double amp = 2047;
    for (int i = 0; i < N; i++) {
        if (i % 3000 == 0) {             // new random settings
            int seg = (i / 3000) % 8;
            int byp = (seg == 0) ? 1 : (urand() < 0.1);
            int sh = (int)(urand() * 8), ws = (urand() < 0.6) ? 0 : (int)(urand() * 8);   // ws 6, 7 = 5
            cfg = (byp ? 1u : 0u) | ((uint32_t)sh << 8) | ((uint32_t)ws << 12) | (urand() < 0.3 ? 0x8000u : 0u);
            nrm = (urand() < 0.5) ? 8192u : (urand32() & 0xFFFFu);
            gain = (urand() < 0.4) ? (uint32_t)NL_GAIN_ONE : (urand32() & 0xFFFFu);
            uint32_t m = urand32() & 0xFFFFu, s = (uint32_t)(urand() * 32);
            nl = (urand() < 0.8 ? 0x80000000u : 0u) | (s << 16) | m | (urand32() & 0x7FE00000u);   // junk in the unused bits
            amp = (urand() < 0.2) ? 2047.0 : (urand() < 0.5 ? 300.0 : 30.0);
        }
        int x[4];
        for (int k = 0; k < 4; k++) {
            double v = (urand() < 0.002) ? ((urand() < 0.5) ? -2048 : 2047) : std::floor((urand() * 2 - 1) * amp);
            x[k] = (int)std::max(-2048.0, std::min(2047.0, v));
        }
        g_cfg = cfg; g_nrm = nrm; g_gain = gain; g_nlms = nl;
        cd y; uint64_t wm;
        push(g_xs, g_ys, x[0], x[1], x[2], x[3], &y, &wm);
        int ryr, ryi; uint64_t rwm;
        ref.step(x[0], x[1], x[2], x[3], cfg, nrm, gain, nl, ryr, ryi, rwm);
        if ((int)y.real() != ryr || (int)y.imag() != ryi) {
            if (bad < 5) std::printf("  sample %d: core (%d, %d), reference (%d, %d)\n", i, (int)y.real(), (int)y.imag(), ryr, ryi);
            bad++;
        }
        if (wm != rwm) {
            if (badm < 5) std::printf("  sample %d: weight monitor core 0x%010llx, reference 0x%010llx\n", i, (unsigned long long)wm, (unsigned long long)rwm);
            badm++;
        }
        for (int j = 0; j < NL_NW; j++) {
            if (ref.cwr[j] == (32767LL << NL_RF) || ref.cwr[j] == -(32768LL << NL_RF)) sat_seen = 1;
            if (ref.cwr[j] != 0) nonzero++;
        }
    }
    // bypass: exactly x1[n-R]
    g_cfg = NL_CFG_BYPASS; g_gain = NL_GAIN_ONE; g_nlms = 0x80000000u | 0xFFFF;
    int byp_bad = 0;
    std::vector<int> ha(20000), hb(20000);
    for (int i = 0; i < 20000; i++) {
        ha[i] = (int)std::floor((urand() * 2 - 1) * 2047); hb[i] = (int)std::floor((urand() * 2 - 1) * 2047);
        cd y;
        push(g_xs, g_ys, ha[i], hb[i], hb[i], ha[i], &y);
        if (i >= NL_REF && ((int)y.real() != ha[i - NL_REF] || (int)y.imag() != hb[i - NL_REF])) byp_bad++;
    }
    std::printf("scenario 1: %d samples, random settings every 3000: %d output and %d weight-monitor mismatches;\n"
                "  weight state reached the Q3.12 limit: %s; bypass: %d of %d samples differ from x1[n-%d]\n",
                N, bad, badm, sat_seen ? "yes" : "NO", byp_bad, 20000 - NL_REF, NL_REF);
    return bad == 0 && badm == 0 && sat_seen && nonzero > 0 && byp_bad == 0;
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
    {   // the NLMS step word: m 2^-sh = alpha 2^(12+ws+RF-1+CS) / (nw P), m in [32768, 65535]
        double cases[][3] = {{1.0 / 128, 800.0, 0}, {1.0 / 128, 4.0e6, 0}, {1.0 / 64, 1.0e5, 2}, {0.5, 50.0, 5}};
        for (auto &c : cases) {
            uint32_t w = aj_nlms_word(c[0], c[1], (int)c[2], AJ_MAXDIM);
            double m = w & 0xFFFF, sh = (w >> 16) & 31;
            double want = c[0] * std::pow(2.0, 12 + c[2] + AJ_NLMS_RF - 1 + AJ_NLMS_CS) / (AJ_MAXDIM * c[1]);
            double got = m / std::pow(2.0, sh);
            bool ok = (want > 65535.0) ? (w == 0x8000FFFFu)          // beyond the word: the largest step
                                       : ((w >> 31) && std::fabs(got / want - 1) < 1e-4 && (m >= 32768 || sh == 0));
            std::printf("  NLMS word alpha %.4f, P %.0f, ws %d: 0x%08x (%.6g, want %.6g)%s\n", c[0], c[1], (int)c[2], w, got, want, ok ? "" : "  WRONG");
            if (!ok) bad++;
        }
    }
    std::printf("make-up rule and NLMS word: %d wrong\n", bad);
    return bad == 0;
}

// ---- jammer scenarios ------------------------------------------------------------
struct Scen { double jnr_db; bool cw; double echo; double rx2_db; bool iq; double cwf; double satf; double chip; double delay;
              int ntrain = 300000; int ws = 0; double drift = 0; double iqfd = 0; int echod = 3; };

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
    for (int k = 0; k < NL_NT; k++) if (k != NL_REF) u[q++] = x1[n - k];
    for (int k = 0; k < NL_NT; k++) u[q++] = x2n[n - k];
    for (int c = 0; c < NL_NIMG; c++) u[q++] = std::conj(x1[n - (NL_REF - NL_IMGH + c)]);
    for (int c = 0; c < NL_NIMG; c++) u[q++] = std::conj(x2n[n - (NL_REF - NL_IMGH + c)]);
}
// Wiener weights of y = d - w.u over [a, b): R w = p, R_kj = E[u_j conj(u_k)], p_k = E[d conj(u_k)]
static std::vector<cd> wiener(const cvec &x1, const cvec &x2n, int a, int b) {
    const int M = AJ_MAXDIM;
    std::vector<cd> R(M * M, 0.0), p(M, 0.0), w(M, 0.0);
    cd u[AJ_MAXDIM];
    for (int n = a; n < b; n++) {
        taps_of(x1, x2n, n, u);
        cd d = x1[n - NL_REF];
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
    for (size_t n = NL_NT; n < x1.size(); n++) {
        taps_of(x1, x2n, n, u);
        cd s = x1[n - NL_REF];
        for (int j = 0; j < AJ_MAXDIM; j++) s -= w[j] * u[j];
        y[n] = s;
    }
    return y;
}

static int test_jammer(int id, Scen sc) {
    const int NT = sc.ntrain, NE = 40000, N = NT + NE;
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
        g_nlms = 0; set_cfg(0, shift, ws);
        cvec yj = run_core_c(seg(jx1), seg(jx2));
        cvec yx = run_core_c(seg(x1), seg(x2));
        cvec yn = run_core_c(seg(nx1), seg(nx2));
        m.supp = (sc.jnr_db > -90) ? 10 * std::log10(pwr(seg(J), 0) / std::max(pwr(yj, 32) / sc4, 1e-9)) : 0;
        m.loss = 10 * std::log10(clean_snr() / snr_of(yx, seg(s1), NL_REF));
        m.out_db = 10 * std::log10(pwr(yx, 32) / sc4 / pwr(seg(x1), 0));
        m.lvl = 10 * std::log10(pwr(yx, 32) / pn);
        m.res = 10 * std::log10(pwr(yx, 32) / std::max(pwr(yn, 32), 1e-9));
        return m;
    };

    if (id == 3) {
        set_cfg(1, 0); set_g2((int)(g2 * 8192 + 0.5)); g_nlms = 0;
        cvec yb = run_core_c(seg(x1), seg(x2)), xs = seg(x1);
        int bad = 0;
        for (size_t i = NL_REF; i < yb.size(); i++) if (yb[i] != xs[i - NL_REF]) bad++;
        std::printf("  bypass            : %d of %d samples differ from x1[n-%d]\n", bad, (int)yb.size() - NL_REF, NL_REF);
        if (bad) return 0;
    }

    // ---- training: the jammer from sample 0, weights from 0, NLMS on ---------------------
    auto train = [&](double g2v, int ws, const char *tag, std::vector<double> *res_t, int *t3, int *t1) {
        set_g2((int)std::floor(g2v * 8192 + 0.5));
        set_cfg(1, 0, ws);                                   // bypass first: weights cleared
        g_nlms = 0;
        for (int i = 0; i < 128; i++) push(g_xs, g_ys, 0, 0, 0, 0, 0);
        set_cfg(0, 0, ws);
        double p_in = pwr(cvec(x1.begin(), x1.begin() + 8192), 0);   // the FPGA meter / first capture
        cd c1 = 0; double p1 = 0;                             // RX1 lag-1 correlation (aj_detect's rho1)
        for (int i = 1; i < 8192; i++) { c1 += x1[i] * std::conj(x1[i - 1]); p1 += std::norm(x1[i]); }
        double rho = std::abs(c1) / p1, al = (g_alpha > 0) ? g_alpha : aj_nlms_alpha(rho);
        uint32_t w = aj_nlms_word(al, p_in, ws, AJ_MAXDIM);
        g_nlms = w;
        cvec y(NT);
        for (int i = 0; i < NT; i++)
            push(g_xs, g_ys, (int)x1[i].real(), (int)x1[i].imag(), (int)x2[i].real(), (int)x2[i].imag(), &y[i]);
        g_nlms = 0;
        // live output power in windows of 2048 against the noise through the final (held) weights
        cvec yn = run_core_c(seg(nx1), seg(nx2));
        double pno = pwr(yn, 32);
        if (res_t) res_t->clear();
        if (t3) *t3 = -1;
        if (t1) *t1 = -1;
        int above3 = 0;
        for (int a = 0; a + 2048 <= NT; a += 2048) {
            double r = 10 * std::log10(pwr(y, a, a + 2048) / pno);
            if (res_t) res_t->push_back(r);
            if (t3 && *t3 < 0 && r <= 3.0) *t3 = a + 2048;
            if (r > 3.0) above3 = a + 2048;
            if (t1 && *t1 < 0 && r <= 1.0) *t1 = a + 2048;
        }
        if (t3 && *t3 >= 0) *t3 = above3;                     // last time it was above 3 dB
        std::printf("  %-18s: |rho1| %.3f -> NLMS step alpha %.5f, word 0x%08x (P_in %.0f LSB^2, ws %d), %d samples\n",
                    tag, rho, al, w, p_in, ws, NT);
        return pwr(y, NT - 20000, NT) / pno;
    };

    std::vector<double> rt;
    int t3 = -1, t1 = -1;
    double live = train(g2, sc.ws, "NLMS training", &rt, &t3, &t1);
    double wn2 = mon_wn2(sc.ws, rx2n);
    Meas m0 = measure(0, sc.ws);
    int sh = aj_makeup_shift((int)cut_db, m0.res + aj_db10(1.0 + wn2), NL_MAXSH);
    Meas m = sh ? measure(sh, sc.ws) : m0;
    double wmax = 0;
    for (int j = 0; j < NL_NW; j++) wmax = std::max(wmax, std::hypot((double)g_wre[j], (double)g_wim[j]) / (4096.0 * (1 << sc.ws)));

    // optimum weights of the same taps (double precision)
    cvec x2n(N);
    for (int i = 0; i < N; i++) x2n[i] = x2[i] * g2;
    std::vector<cd> wo = wiener(x1, x2n, std::max(NL_NT, NT - 100000), NT);
    cvec j2n(jx2), n2n(nx2);
    for (auto &v : j2n) v *= g2;
    for (auto &v : n2n) v *= g2;
    cvec oj = apply_w(wo, seg(jx1), seg(j2n)), ox = apply_w(wo, seg(x1), seg(x2n)), on = apply_w(wo, seg(nx1), seg(n2n));
    double o_supp = (sc.jnr_db > -90) ? 10 * std::log10(pwr(seg(J), 0) / std::max(pwr(oj, 32), 1e-9)) : 0;
    double o_res = 10 * std::log10(pwr(ox, 32) / pwr(on, 32));

    std::printf("  NLMS, held weights: suppression %.1f dB, residual %+.2f dB, satellite SNR loss %.2f dB, output %+.2f dB re RX1\n",
                m.supp, m.res, m.loss, m.out_db);
    std::printf("  NLMS, adapting    : residual %+.2f dB over the last 20000 training samples\n", 10 * std::log10(live));
    std::printf("  optimum (Wiener)  : suppression %.1f dB, residual %+.2f dB (same %d taps, double precision)\n", o_supp, o_res, AJ_MAXDIM);
    if (sc.jnr_db > -90)
        std::printf("  convergence       : residual <= 3 dB from sample %d (%.0f us at 30.72 MS/s), <= 1 dB first at %d\n",
                    t3, t3 / 30.72, t1);
    std::printf("  weights           : |w|^2 %.2f, largest |w| %.3f; make-up shift %d, output level %+.1f dB against the clean level\n",
                wn2, wmax, sh, m.lvl);

    g_t3 = t3;
    // Limits from the C simulation after 300000 samples (about 10 ms at 30.72 MS/s) with the
    // firmware's step rule; the loop keeps converging after that (100 ms: J/N 50 +1.2 dB,
    // 51 +0.8 dB, 60 +6.2 dB, 61 +7.6 dB residual, optimum +0.8, +0.7, +4.0, +4.0 dB).
    int ok = 1;
    switch (id) {
    case 2:  ok = m.supp >= 35 && m.loss <= 3 && m.res <= 1.0; break;
    case 3:  ok = std::fabs(m.out_db) <= 0.5 && std::fabs(m.loss) <= 1.0 && m.res <= 0.3 && wn2 <= 0.05; break;
    case 4:  ok = m.supp >= 55 && m.loss <= 3 && m.res <= 0.3; break;
    // 6 + 6 taps (reference delay 3) cannot reach a reflection 3 samples later on RX1 (it would need
    // delay 6): the optimum of these taps is itself only ~24 dB there (crpa1 SMI 6 + 6: 20.4 dB)
    case 5:  ok = (NL_NT >= 8) ? (m.supp >= 33 && m.res <= 1.0) : (m.supp >= 20 && m.res <= o_res + 1.5); break;
    case 6:  ok = m.supp >= 45 && m.loss <= 2.5 && m.res <= 3.5; break;
    case 7: {
        // the same without the normalisation (g2 = 1)
        train(1.0, sc.ws, "no normalisation", 0, 0, 0);
        Meas mn = measure(0, sc.ws);
        std::printf("  no normalisation  : suppression %.1f dB, residual %+.2f dB\n", mn.supp, mn.res);
        ok = m.supp >= 30 && m.supp >= mn.supp + 10;
        break; }
    case 8:  ok = m.supp >= 45; break;
    case 9:  ok = m.supp >= 65 && m.res <= 1.0 && m.loss <= 3; break;
    case 11: ok = m.supp >= 43 && m.lvl >= -6.5 && m.lvl <= 3.5; break;
    case 12: ok = m.supp >= 43 && m.res <= 6.0 && m.lvl >= -6.5 && m.lvl <= 3.5; break;
    case 13: ok = m.supp >= 40 && m.res <= 2.0 && m.loss <= 1.0 && m.lvl >= -6.5 && m.lvl <= 3.5; break;
    case 25: ok = m.supp >= 44; break;
    case 26: {
        // the adapting output while the direction keeps drifting: 2nd quarter against the last
        int q = rt.size() / 4;
        double r1 = 0, r4 = 0;
        for (int k = q; k < 2 * q; k++) r1 += rt[k];
        for (int k = 3 * q; k < 4 * q; k++) r4 += rt[k];
        r1 /= q; r4 /= q;
        std::printf("  tracking          : adapting residual %+.2f dB (2nd quarter), %+.2f dB (last quarter)\n", r1, r4);
        ok = r4 <= r1 + 1.5 && r4 <= 4.0;
        break; }
    }
    return ok;
}

// ---- scenario 27: convergence time after the jammer appears --------------------------
static int test_convergence() {
    // J/N 30 within 1 ms, 50 within 10 ms (residual <= 3 dB and staying there)
    double jns[] = {30, 50};
    double lim[] = {30720, 307200};
    int ok = 1;
    for (int k = 0; k < 2; k++) {
        Scen sc = {jns[k], false, 0, 0, true, 0, 0, 0, 0};
        sc.ntrain = 400000;
        test_jammer(27, sc);
        int good = g_t3 >= 0 && g_t3 <= lim[k];
        std::printf("  -> J/N %.0f dB: %s\n", jns[k], good ? "in time" : "TOO SLOW");
        ok = ok && good;
    }
    return ok;
}

int main(int argc, char **argv) {
    int scen = (argc > 1) ? std::atoi(argv[1]) : 2;
    double jn_arg = (argc > 2) ? std::atof(argv[2]) : -1000;
    if (argc > 3) g_alpha = std::atof(argv[3]);
    int ntr = (argc > 4) ? std::atoi(argv[4]) : 0;
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
        case 3: sc = {-99, false, 0, 0, false, 0, 0, 0, 0}; sc.ntrain = 100000; if (g_alpha <= 0) g_alpha = 1.0 / 128; break;   // no jammer: the firmware keeps the filter bypassed; NLMS at a small step here
        case 4: sc = {J(40), true, 0, 0, false, 0, 0, 0, 0}; break;
        case 5: sc = {J(30), false, 0.5, 0, false, 0, 0, 0, 0}; break;
        case 6: sc = {J(50), false, 0, 0, true, 0, 0, 0, 0}; break;
        case 7: sc = {J(40), false, 0, -25, false, 0, 0, 0, 0}; break;
        case 8: sc = {J(60), false, 0, 0, true, 0, 0, 0, 0}; break;
        case 9: sc = {J(62), true, 0, 0, true, -7.06 / 61.44, -7.06 / 61.44, 1.023 / 61.44, 0}; sc.ws = 3; break;
        case 11: case 12: case 13:
            sc = {J(scen == 11 ? 61.0 : scen == 12 ? 51.0 : 41.0), false, 0, 0, true, 0, -0.02 / 18.0, 1.023 / 18.0, 0.055}; break;
        case 25: sc = {J(61), false, 0, 0, true, 0, -0.02 / 18.0, 1.023 / 18.0, 0.055}; sc.iqfd = 0.03; break;
        case 26: sc = {J(50), false, 0, 0, true, 0, 0, 0, 0}; sc.drift = 30; sc.ntrain = 400000; break;
        default: std::printf("unknown scenario %d\n", scen); return 1;
        }
        if (ntr > 0) sc.ntrain = ntr;
        if (argc > 5) sc.ws = std::atoi(argv[5]);
        ok = test_jammer(scen, sc);
    }
    std::printf("NLMS_PI_CSIM: %s (scenario %d)\n", ok ? "PASS" : "FAIL", scen);
    return ok ? 0 : 1;
}
