// ============================================================================
//  testbench.cpp  --  C simulation of the CRPA filter core of one algorithm
//  (smi_pi for crpa1, stap_pi for crpa27..31) together with that algorithm's
//  ARM weight solver (Source/CRPA/<algo>/arm/gnss_aj_math.c, compiled in), so
//  the weights the core is tested with are exactly the ones the board
//  computes. Used by build_hls.tcl (Vitis HLS csim_design) and by
//  Automation/Linux/csim_gcc.sh (plain g++).
//
//  Compile with -DCRPA_SMI (smi_pi.h / smi_pi.cpp) or -DCRPA_STAP
//  (stap_pi.h / stap_pi.cpp); the include path selects the algorithm's copy.
//
//  SCENARIOS (argv[1]; none = all)
//    1  core accuracy: random weights, weight scale, make-up shift, g2 (and
//       the fine gain on stap_pi) against a double-precision model of the
//       filter, every output within 1 LSB; bypass is exact (x1[n-REF]).
//    2  wideband jammer 40 dB over the noise, one direction
//    3  CW jammer 45 dB
//    4  wideband jammer 40 dB, RX2 through a different 2-tap channel
//       (frequency-dependent RX1/RX2 ratio: needs the space-time taps)
//       and RX2 6 dB weaker (g2 learned from the noise floors)
//    5  no jammer: the solved weights must not raise the output noise
//    6  moving jammer: the direction turns by 2 degrees of phase per capture
//       (the crpa27..31 tracking / prediction; crpa1: SMI with averaging)
//  Each prints the jammer suppression and the residual (output over the
//  output noise, 0 dB = fully nulled) measured on the CORE's output, on
//  fresh samples after the weights were committed, and PASS / FAIL.
// ============================================================================
#if defined(CRPA_SMI)
#include "smi_pi.h"
#define CORE_NT   SMI_NT
#define CORE_REF  SMI_REF
#define CORE_NWTS 64
#define CORE_NWCJ 2
typedef ap_uint<2048> wts_t;
typedef ap_uint<64>   wcj_t;
#elif defined(CRPA_STAP)
#include "stap_pi.h"
#define CORE_NT   ST_NT
#define CORE_REF  ST_REF
#define CORE_NWTS ST_NST
#define CORE_NWCJ (2 * ST_NIMG)
typedef ap_uint<32 * ST_NST>      wts_t;
typedef ap_uint<32 * 2 * ST_NIMG> wcj_t;
#else
#error "compile with -DCRPA_SMI or -DCRPA_STAP"
#endif
#include "gnss_aj_math.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <random>
#include <vector>
#include <complex>

#if AJ_NT != CORE_NT
#error "AJ_NT (gnss_aj_math.h) does not match the core's taps"
#endif
#if AJ_REF != CORE_REF
#error "AJ_REF (gnss_aj_math.h) does not match the core's reference delay"
#endif
#if (AJ_NW - AJ_NSPT) != CORE_NWCJ
#error "image tap count of the solver and the core differ"
#endif

typedef std::complex<double> cd;
static std::mt19937 rng(12345);
static double gauss() { static std::normal_distribution<double> g(0.0, 1.0); return g(rng); }

// ---- the core, one sample at a time -------------------------------------------
struct core_set { uint32_t w[64]; uint32_t c[8]; uint32_t cfg, nrm, gain; };
static core_set cur;

static void set_bypass() { std::memset(&cur, 0, sizeof(cur)); cur.cfg = 1; cur.nrm = 0x2000; cur.gain = 16384; }

static int16_t sat12(int v) { return (int16_t)(v > 2047 ? 2047 : v < -2048 ? -2048 : v); }

static cd core_step(int x1r, int x1i, int x2r, int x2i)
{
    hls::stream<ap_uint<64> > in;
    hls::stream<ap_uint<32> > out;
    ap_uint<64> s = 0;
    s(15, 0)  = (ap_uint<16>)(ap_int<16>)sat12(x1r);
    s(31, 16) = (ap_uint<16>)(ap_int<16>)sat12(x1i);
    s(47, 32) = (ap_uint<16>)(ap_int<16>)sat12(x2r);
    s(63, 48) = (ap_uint<16>)(ap_int<16>)sat12(x2i);
    in.write(s);
    wts_t w = 0;
    wcj_t c = 0;
    for (int k = 0; k < CORE_NWTS; k++) w(32 * k + 31, 32 * k) = cur.w[k];
    for (int k = 0; k < CORE_NWCJ; k++) c(32 * k + 31, 32 * k) = cur.c[k];
#if defined(CRPA_SMI)
    smi_pi(in, out, w, c, cur.cfg, cur.nrm);
#else
    stap_pi(in, out, w, c, cur.cfg, cur.nrm, cur.gain);
#endif
    ap_uint<32> y = out.read();
    return cd((double)(ap_int<16>)y(15, 0), (double)(ap_int<16>)y(31, 16));
}

static void flush() { for (int k = 0; k < 2 * CORE_NT + 4; k++) core_step(0, 0, 0, 0); }

// ---- signal generation ------------------------------------------------------------
struct scen {
    double noise_sd;        // per I/Q, LSB
    double jn_db;           // jammer over the (complex) noise, dB; < -50 = none
    int    cw;              // 1 = CW tone
    double cw_f;            // CW frequency, fraction of fs
    cd     h2[2];           // RX2 channel to the jammer: h2[0] j[n] + h2[1] j[n-1]
    double rx2_db;          // RX2 receiver gain relative to RX1 (noise and signal)
    double turn_deg;        // jammer direction change per capture (phase of h2)
};

struct gen {
    scen sc; double phase = 0.0, tphase = 0.0; cd jprev = 0.0; long n = 0;
    void block(int N, int with_j, int with_n, std::vector<int> &x1r, std::vector<int> &x1i,
               std::vector<int> &x2r, std::vector<int> &x2i, unsigned seed) {
        std::mt19937 r(seed);
        std::normal_distribution<double> g(0.0, 1.0);
        double ja = (sc.jn_db < -50.0) ? 0.0 : sc.noise_sd * std::sqrt(2.0) * std::pow(10.0, sc.jn_db / 20.0);
        double g2 = std::pow(10.0, sc.rx2_db / 20.0);
        cd rot = std::polar(1.0, tphase * M_PI / 180.0);
        x1r.resize(N); x1i.resize(N); x2r.resize(N); x2i.resize(N);
        for (int t = 0; t < N; t++) {
            cd j;
            if (sc.cw) { j = std::polar(ja, phase); phase += 2.0 * M_PI * sc.cw_f; }
            else j = cd(g(r), g(r)) * (ja / std::sqrt(2.0));
            cd n1 = cd(g(r), g(r)) * sc.noise_sd, n2 = cd(g(r), g(r)) * sc.noise_sd;
            cd a1 = (with_j ? j : 0.0) + (with_n ? n1 : 0.0);
            cd a2 = g2 * ((with_j ? rot * (sc.h2[0] * j + sc.h2[1] * jprev) : 0.0) + (with_n ? n2 : 0.0));
            jprev = j;
            x1r[t] = (int)std::lround(a1.real()); x1i[t] = (int)std::lround(a1.imag());
            x2r[t] = (int)std::lround(a2.real()); x2i[t] = (int)std::lround(a2.imag());
        }
    }
};

static void pack(const std::vector<int> &a, const std::vector<int> &b, const std::vector<int> &c,
                 const std::vector<int> &d, std::vector<uint32_t> &buf)
{
    int N = (int)a.size();
    buf.resize(2 * N);
    for (int t = 0; t < N; t++) {
        buf[2 * t]     = ((uint32_t)(uint16_t)(sat12(b[t]) & 0xFFF) << 16) | (uint16_t)(sat12(a[t]) & 0xFFF);
        buf[2 * t + 1] = ((uint32_t)(uint16_t)(sat12(d[t]) & 0xFFF) << 16) | (uint16_t)(sat12(c[t]) & 0xFFF);
    }
}

static double run_power(const std::vector<int> &a, const std::vector<int> &b, const std::vector<int> &c,
                        const std::vector<int> &d)
{
    flush();
    double p = 0.0; int cnt = 0;
    for (size_t t = 0; t < a.size(); t++) {
        cd y = core_step(a[t], b[t], c[t], d[t]);
        if ((int)t >= 2 * CORE_NT) { p += std::norm(y); cnt++; }
    }
    return p / cnt;
}

static double pw(const std::vector<int> &a, const std::vector<int> &b)
{
    double p = 0.0;
    for (size_t t = 0; t < a.size(); t++) p += (double)a[t] * a[t] + (double)b[t] * b[t];
    return p / a.size();
}

// the ARM result -> the words the firmware writes (gnss_pt_write_aj)
static void commit(const aj_result_t &r, double g2)
{
    std::memset(cur.w, 0, sizeof(cur.w));
    std::memset(cur.c, 0, sizeof(cur.c));
    for (int k = 0; k < AJ_NW; k++) {
        uint32_t wd = ((uint32_t)(uint16_t)r.wim[k] << 16) | (uint16_t)r.wre[k];
        if (k < AJ_NSPT) cur.w[k] = wd; else cur.c[k - AJ_NSPT] = wd;
    }
    cur.cfg = ((uint32_t)r.ws << 12);
    cur.nrm = (uint32_t)(int32_t)std::lround(g2 * 8192.0) & 0xFFFF;
    cur.gain = 16384;
}

// ---- scenario 1: core against a double model ------------------------------------------------
static int scen_accuracy()
{
    int bad = 0, nchk = 0;
    for (int trial = 0; trial < 6; trial++) {
        std::uniform_int_distribution<int> u(-2048, 2047), wsd(0, 5), shd(0, 7);
        int ws = wsd(rng), sh = (trial == 0) ? 0 : shd(rng) % 3;
        double wscale = 0.02 * (1 << ws);
        set_bypass();
        for (int k = 0; k < AJ_NW; k++) {
            int re = (int)std::lround(gauss() * wscale * 4096.0), im = (int)std::lround(gauss() * wscale * 4096.0);
            uint32_t wd = ((uint32_t)(uint16_t)(int16_t)im << 16) | (uint16_t)(int16_t)re;
            if (k < AJ_NSPT) cur.w[k] = wd; else cur.c[k - AJ_NSPT] = wd;
        }
        double g2 = 0.5 + 1.5 * (trial / 6.0);
        int g2q = (int)std::lround(g2 * 8192.0);
        cur.nrm = (uint32_t)g2q;
        cur.cfg = ((uint32_t)ws << 12) | ((uint32_t)sh << 8) | (trial == 5 ? 1u : 0u);
#if defined(CRPA_STAP)
        cur.gain = (trial == 3) ? 11585 : 16384;   // 0.707 on one trial
#endif
        flush();
        std::vector<int> h1r(CORE_NT, 0), h1i(CORE_NT, 0); std::vector<double> h2r(CORE_NT, 0), h2i(CORE_NT, 0);
        for (int t = 0; t < 3000; t++) {
            int a = u(rng) / 64, b = u(rng) / 64, c = u(rng) / 64, d = u(rng) / 64;   // small: no saturation
            cd y = core_step(a, b, c, d);
            for (int k = CORE_NT - 1; k > 0; k--) { h1r[k] = h1r[k-1]; h1i[k] = h1i[k-1]; h2r[k] = h2r[k-1]; h2i[k] = h2i[k-1]; }
            // x2n exactly as the core: round(g2q x / 2^11) / 4
            auto x2n = [&](int x) { long p = (long)g2q * x; return (double)((p + 1024) >> 11) / 4.0; };
            h1r[0] = a; h1i[0] = b; h2r[0] = x2n(c); h2i[0] = x2n(d);
            if (t < 2 * CORE_NT) continue;
            cd ref(h1r[CORE_REF], h1i[CORE_REF]), s = 0.0;
            auto W = [&](int k) { uint32_t wd = k < AJ_NSPT ? cur.w[k] : cur.c[k - AJ_NSPT];
                                  return cd((int16_t)(wd & 0xFFFF), (int16_t)(wd >> 16)) / 4096.0; };
            for (int k = 0; k < CORE_NT; k++) {
                if (k != CORE_REF) s += W(k) * cd(h1r[k], h1i[k]);
                s += W(CORE_NT + k) * cd(h2r[k], h2i[k]);
            }
#if defined(CRPA_SMI)
            s += W(AJ_NSPT) * std::conj(ref) + W(AJ_NSPT + 1) * std::conj(cd(h2r[CORE_REF], h2i[CORE_REF]));
#else
            for (int c2 = 0; c2 < ST_NIMG; c2++) {
                int a2 = CORE_REF - ST_IMGH + c2;
                s += W(AJ_NSPT + c2) * std::conj(cd(h1r[a2], h1i[a2]));
                s += W(AJ_NSPT + ST_NIMG + c2) * std::conj(cd(h2r[a2], h2i[a2]));
            }
#endif
            cd e = (cur.cfg & 1) ? ref : ref - s / (double)(1 << ws);
            e *= (double)(1 << sh);
#if defined(CRPA_STAP)
            e *= (double)cur.gain / 16384.0;
#endif
            double tol = (cur.cfg & 1) ? 0.0 : 1.01;
            if ((cur.cfg & 1) && sh == 0) tol = 0.0;
            e = cd(std::max(-32768.0, std::min(32767.0, e.real())), std::max(-32768.0, std::min(32767.0, e.imag())));
            double dr = std::fabs(y.real() - e.real()), di = std::fabs(y.imag() - e.imag());
            if (!(cur.cfg & 1)) { dr = std::fabs(y.real() - e.real()); }
            nchk++;
            if (dr > tol + ((cur.cfg & 1) ? 0.5 : 0.0) || di > tol + ((cur.cfg & 1) ? 0.5 : 0.0)) {
                if (bad < 5) printf("  trial %d t %d: core (%g, %g) model (%g, %g)\n", trial, t, y.real(), y.imag(), e.real(), e.imag());
                bad++;
            }
        }
    }
    printf("scenario 1 core accuracy: %d of %d outputs off by more than 1 LSB (bypass: exact)\n", bad, nchk);
    return bad == 0;
}

// ---- scenarios 2..6: jammer through the solver and the core -----------------------------------
static int scen_jammer(int id, const char *name, scen sc, int caps, double min_supp, double max_res, int want_jammer)
{
    gen G; G.sc = sc;
    const int N = AJ_NSAMP;
    std::vector<int> a, b, c, d;
    std::vector<uint32_t> buf;
    aj_result_t r; std::memset(&r, 0, sizeof(r));
    double nz = 2.0 * sc.noise_sd * sc.noise_sd + 1.0 / 6.0;    // complex noise + rounding [LSB^2]
    double rx2lin = std::pow(10.0, sc.rx2_db / 10.0);
    // g2 as the firmware learns it: sqrt(P1 / P2) on noise alone
    double g2 = std::sqrt(1.0 / rx2lin);
    if (g2 > 3.99) g2 = 3.99;
    if (g2 < 0.25) g2 = 0.25;
    g2 = std::lround(g2 * 8192.0) / 8192.0;
    double rx2n = g2 * g2 * rx2lin;           // RX2 noise after g2, relative to RX1
#if defined(CRPA_SMI)
    aj_reset();
    r.sig2_prev = -1.0;
#else
    aj_nlms_reset();
    aj_committed(0);
    aj_track_t trk; aj_track_reset(&trk);
    aj_pred_t pred; aj_pred_reset(&pred);
    int direct_left = AJ_DIRECT_DEF;
#endif
    set_bypass();
    int ok = 1, nfail = 0;
    double supp = 0.0, res = 0.0, wn2 = 0.0;
    for (int cap = 0; cap < caps; cap++) {
        G.tphase = sc.turn_deg * cap;
        G.block(N, 1, 1, a, b, c, d, 1000u + 17u * cap + 101u * id);
        pack(a, b, c, d, buf);
        r.noise_ref = nz;
#if defined(CRPA_SMI)
        r.mode = AJ_MODE_STAP; r.g2 = g2; r.rx2_noise = rx2n; r.avg = 0.0; r.gate = 0;
        if (aj_compute(buf.data(), N, &r) != 0 || !r.ok) { nfail++; continue; }
        r.sig2_prev = r.sig2;
#else
        aj_detect(buf.data(), N, &r);
        int direct = direct_left > 0, rc;
        if (direct) { rc = aj_direct_solve(buf.data(), N, g2, rx2n, nz, AJ_DIRECT_LOAD_DEF, &r); direct_left--; }
        else rc = aj_nlms_solve(buf.data(), N, g2, rx2n, aj_anlms_alpha(r.jn_db), AJ_ANLMS_PASSES_DEF, &r);
        if (rc < 0) { nfail++; continue; }
        double rdb = aj_db10(r.arm_pout / (nz * (1.0 + r.wn2)));
        if (aj_track_update(&trk, direct, rdb)) direct_left = 1;
        aj_pred_after(&pred, direct, rc == 0, nz, 1.0, 1, &r);
        aj_committed(&r);
#endif
        commit(r, g2);
        wn2 = r.wn2;
    }
    // measure the core with the last weights on fresh samples (next capture's direction)
    G.tphase = sc.turn_deg * caps;
    std::vector<int> ja, jb, jc, jd, na, nb, nc, nd;
    unsigned seed = 7777u + id;
    G.jprev = 0.0; G.phase = 0.0; G.block(N, 1, 1, a, b, c, d, seed);
    G.jprev = 0.0; G.phase = 0.0; G.block(N, 1, 0, ja, jb, jc, jd, seed);
    G.jprev = 0.0; G.phase = 0.0; G.block(N, 0, 1, na, nb, nc, nd, seed);
    double p_tot = run_power(a, b, c, d);
    double p_n   = run_power(na, nb, nc, nd);
    double p_jout = run_power(ja, jb, jc, jd);
    double p_jin = pw(ja, jb);
    double p_nin = pw(na, nb);
    supp = (p_jin > 0.0) ? 10.0 * std::log10(p_jin / std::max(p_jout, 1e-3)) : 0.0;
    res = 10.0 * std::log10(p_tot / p_n);
    double nrise = 10.0 * std::log10(p_n / p_nin);
    if (supp > 60.0) supp = 60.0;
    if (want_jammer) ok = (supp >= min_supp) && (res <= max_res) && nfail == 0;
    else             ok = (nrise <= max_res) && nfail == 0;
    const char *verdict = ok ? "PASS" : "FAIL";
    if (want_jammer == 2) { verdict = "INFO"; ok = (nfail == 0); }   // reported, not judged
    printf("scenario %d %s: J/N %.1f dB, suppression %.1f dB (min %.0f), residual %.2f dB (max %.1f), "
           "output noise %+.2f dB, |w|^2 %.3f, %d captures, %d solver failures -> %s\n",
           id, name, sc.jn_db, supp, min_supp, res, max_res, nrise, wn2, caps, nfail, verdict);
    return ok;
}

int main(int argc, char **argv)
{
    int want = (argc > 1) ? atoi(argv[1]) : 0, ok = 1;
    printf("CRPA C simulation: %s, %d + %d taps, reference delay %d, %d weight words + %d image\n",
#if defined(CRPA_SMI)
           "smi_pi (crpa1)",
#else
           "stap_pi (crpa27..31)",
#endif
           CORE_NT, CORE_NT, CORE_REF, AJ_NSPT, AJ_NW - AJ_NSPT);
    scen base; base.noise_sd = 3.0; base.jn_db = 40.0; base.cw = 0; base.cw_f = 0.0;
    base.h2[0] = std::polar(1.0, 1.1); base.h2[1] = 0.0; base.rx2_db = 0.0; base.turn_deg = 0.0;
    for (int s = 1; s <= 6; s++) {
        if (want && s != want) continue;
        int r = 1;
        scen sc = base;
        switch (s) {
        case 1: r = scen_accuracy(); break;
        case 2: r = scen_jammer(2, "wideband 40 dB", sc, 6, 30.0, 3.0, 1); break;
        case 3: sc.cw = 1; sc.cw_f = 0.1234; sc.jn_db = 45.0; r = scen_jammer(3, "CW 45 dB", sc, 6, 35.0, 3.0, 1); break;
        case 4: sc.h2[0] = std::polar(0.8, -0.7); sc.h2[1] = std::polar(0.35, 2.0); sc.rx2_db = -6.0;
                r = scen_jammer(4, "wideband 40 dB, 2-tap RX2 channel, RX2 -6 dB", sc, 8, 25.0, 3.0, 1); break;
        case 5: sc.jn_db = -99.0; r = scen_jammer(5, "no jammer", sc, 4, 0.0, 1.0, 0); break;
        case 6: sc.turn_deg = 2.0;
#if defined(CRPA_SMI)
                /* crpa1 has no moving-jammer tracking / prediction (crpa27..31 add it): the
                 * weights are one capture old, so the residual is reported, not judged */
                r = scen_jammer(6, "wideband 40 dB, moving 2 deg/capture (crpa1: no prediction, informational)", sc, 12, 20.0, 6.0, 2);
#else
                r = scen_jammer(6, "wideband 40 dB, moving 2 deg/capture", sc, 12, 20.0, 6.0, 1);
#endif
                break;
        }
        printf("CRPA_CSIM: %s (scenario %d)\n", r ? "PASS" : "FAIL", s);
        ok &= r;
    }
    return ok ? 0 : 1;
}
