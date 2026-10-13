// ============================================================================
//  tb_esprit_hls.cpp -- C-simulation testbench for iuesprit_hls / mesprit_hls
// ----------------------------------------------------------------------------
//  Build selects the kernel:  -DTB_ALGO=0 (I-UESPRIT)  or  -DTB_ALGO=1 (M-ESPRIT)
//  (run_hls.tcl passes it through add_files -tb -cflags).
//
//  Every window is generated here (two-element ULA, d = lambda/2, Q1.15
//  quantised), run through the kernel, and checked against an independent
//  double-precision reference that follows the paper's equations directly
//  (complex arithmetic, Eq. (7), Eq. (38), closed form):
//    * kernel vs reference      |d theta| < 0.01 deg   (arithmetic check)
//    * kernel vs true angle     |d theta| < 0.5 deg    (20 dB SNR)
//    * flags: no_signal on zero input; M-ESPRIT off_circle on noise only
//  Prints "ESPRIT_HLS_TB: ... PASS|FAIL" lines and returns 0 only on PASS,
//  which is what Vitis HLS csim_design / cosim_design use as the verdict.
// ============================================================================
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "esprit_hls.h"

#ifndef TB_ALGO
#define TB_ALGO 1
#endif

typedef std::complex<double> cd;
static const double PI = 3.14159265358979323846;
static int g_fail = 0, g_checks = 0;

static void check(const char *what, bool ok) {
    g_checks++;
    if (!ok) g_fail++;
    std::printf("ESPRIT_HLS_TB: %-58s %s\n", what, ok ? "PASS" : "FAIL");
}

// ---- deterministic RNG (no libc dependence between hosts) ------------------
static unsigned long long g_rng = 88172645463325252ULL;
static double urand() { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 7; g_rng ^= g_rng << 17;
                        return (g_rng >> 11) * (1.0 / 9007199254740992.0); }
static double nrand() { double u1 = urand() + 1e-300, u2 = urand();
                        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2 * PI * u2); }

struct Win { std::vector<int> i0, q0, i1, q1; };

// kind 0 = tone, 1 = QPSK, 2 = noise only, 3 = all zero
static Win make_window(double theta_deg, int N, double snr_db, double amp, int kind) {
    Win w;
    const double ph = -2.0 * PI * 0.5 * std::sin(theta_deg * PI / 180.0);   // element 1 phase
    const double npow = (kind == 2) ? amp * amp : amp * amp / std::pow(10.0, snr_db / 10.0);
    const double sg = std::sqrt(npow / 2.0);
    const double ph0 = 2 * PI * urand();
    for (int n = 0; n < N; n++) {
        cd s = 0.0;
        if (kind == 0) s = amp * std::polar(1.0, 2 * PI * 0.0137 * n + ph0);
        if (kind == 1) s = amp * std::polar(1.0, PI / 4 + PI / 2 * (int)(urand() * 4));
        cd x0 = s, x1 = s * std::polar(1.0, ph);
        if (kind != 3) { x0 += cd(sg * nrand(), sg * nrand()); x1 += cd(sg * nrand(), sg * nrand()); }
        auto q = [](double v) { double r = std::floor(v * 32768.0 + 0.5);
                                return (int)(r > 32767 ? 32767 : (r < -32768 ? -32768 : r)); };
        w.i0.push_back(q(x0.real())); w.q0.push_back(q(x0.imag()));
        w.i1.push_back(q(x1.real())); w.q1.push_back(q(x1.imag()));
    }
    return w;
}

struct Res { unsigned flags; double phi, psi, theta, deg; };

// ---- independent double-precision reference -------------------------------
static Res reference(const Win &w, int algo, int p_log2, bool cov, double mu, double k, double uctol) {
    const int N = (int)w.i0.size();
    cd R[2][2] = {}, Pm[2] = {}, M4[2][2] = {};
    for (int n = 0; n < N; n++) {
        cd x[2] = { cd(w.i0[n], w.q0[n]) / 32768.0, cd(w.i1[n], w.q1[n]) / 32768.0 };
        double e0 = std::norm(x[0]);
        for (int m = 0; m < 2; m++) {
            Pm[m] += x[m] * x[0];
            for (int c = 0; c < 2; c++) { R[m][c] += x[m] * std::conj(x[c]); M4[m][c] += x[m] * e0 * std::conj(x[c]); }
        }
    }
    for (int m = 0; m < 2; m++) { Pm[m] /= N; for (int c = 0; c < 2; c++) { R[m][c] /= N; M4[m][c] /= N; } }
    cd C[2][2];
    for (int m = 0; m < 2; m++) for (int c = 0; c < 2; c++)
        C[m][c] = cov ? R[m][c] : M4[m][c] - R[m][0] * R[0][c] - Pm[m] * std::conj(Pm[c]) - R[m][c] * R[0][0];
    double a = C[0][0].real(), d = C[1][1].real(); cd b = C[0][1];
    double mx = std::max(std::max(std::fabs(a), std::fabs(d)), std::max(std::fabs(b.real()), std::fabs(b.imag())));
    Res r = {0, 0, 0, 0, 0};
    if (!(mx > 0)) { r.flags |= FLAG_NO_SIGNAL; return r; }
    a /= mx; d /= mx; b /= mx;
    for (int i = 0; i < p_log2; i++) {
        double a2 = a * a + std::norm(b), d2 = d * d + std::norm(b); cd b2 = (a + d) * b;
        double m = std::max(std::max(std::fabs(a2), std::fabs(d2)), std::max(std::fabs(b2.real()), std::fabs(b2.imag())));
        a = a2 / m; d = d2 / m; b = b2 / m;
    }
    if (algo == 0) {
        double A = 0.5 * (a + d), r0 = A + b.real(), r1 = b.imag(), s0 = -b.imag(), s1 = -(A - b.real());
        double num = r0 * s0 + r1 * s1, den = r0 * r0 + r1 * r1 + mu;
        r.psi = num / den; r.phi = 2.0 * std::atan2(num, den);
    } else {
        cd psi = (a + d) * std::conj(b) / (a * a + std::norm(b) + mu);
        r.psi = std::abs(psi); r.phi = std::arg(psi);
        if (std::fabs(r.psi - 1.0) > uctol) r.flags |= FLAG_OFF_CIRCLE;
    }
    double u = -k * r.phi;
    if (std::fabs(u) > 1.0) { r.flags |= FLAG_AMBIGUOUS; u = u > 0 ? 1.0 : -1.0; }
    r.theta = std::asin(u); r.deg = r.theta * 180.0 / PI;
    return r;
}

// ---- run one window through the kernel --------------------------------------
static Res run_kernel(const Win &w, int log2_snap, int p_log2, bool cov, unsigned mu_q30,
                      int k_q30, unsigned uctol_q24, bool *pkt_ok) {
    hls::stream<snap_t> in;
    hls::stream<word_t> out;
    for (size_t n = 0; n < w.i0.size(); n++) {
        snap_t s = 0;
        s(15, 0)  = (ap_uint<16>)(w.i0[n] & 0xFFFF);
        s(31, 16) = (ap_uint<16>)(w.q0[n] & 0xFFFF);
        s(47, 32) = (ap_uint<16>)(w.i1[n] & 0xFFFF);
        s(63, 48) = (ap_uint<16>)(w.q1[n] & 0xFFFF);
        in.write(s);
    }
#if TB_ALGO == 0
    iuesprit_hls(in, out, log2_snap, p_log2, cov ? 1 : 0, mu_q30, k_q30);
    (void)uctol_q24;
#else
    mesprit_hls(in, out, log2_snap, p_log2, cov ? 1 : 0, mu_q30, k_q30, uctol_q24);
#endif
    Res r = {0, 0, 0, 0, 0};
    *pkt_ok = (out.size() == (size_t)ESPRIT_PKT_WORDS) && in.empty();
    if (out.size() < (size_t)ESPRIT_PKT_WORDS) return r;
    word_t w0 = out.read();
    *pkt_ok = *pkt_ok && ((unsigned)w0(31, 16) == ESPRIT_MAGIC);
    r.flags = (unsigned)w0(11, 8);
    r.phi   = (int)(unsigned)out.read() / 268435456.0;
    r.psi   = (int)(unsigned)out.read() / 16777216.0;
    r.theta = (int)(unsigned)out.read() / 268435456.0;
    r.deg   = (int)(unsigned)out.read() / 65536.0;
    for (int i = 0; i < 4; i++) out.read();
    return r;
}

int main() {
    const char *name = TB_ALGO == 0 ? "I-UESPRIT (iuesprit_hls)" : "M-ESPRIT (mesprit_hls)";
    std::printf("ESPRIT_HLS_TB: ===== %s C-simulation =====\n", name);
    const unsigned MU = 1024;            // mu = 2^-20
    const int K = 341782638;             // k = 1/pi (d = lambda/2), Q2.30
    const unsigned UCTOL = 0x00400000;   // 0.25
    const double mu = MU / 1073741824.0, k = K / 1073741824.0, uctol = UCTOL / 16777216.0;
    char msg[160];
    double worst_ref = 0, worst_true = 0;
    bool pkt_all = true;

    struct Case { double th; int L, P; bool cov; int kind; const char *what; };
    std::vector<Case> cases;
    for (double th = -60; th <= 60.1; th += 15) cases.push_back({th, 10, 2, false, 0, "sweep"});
    for (int P = 0; P < 4; P++)                cases.push_back({25.0, 10, P, false, 0, "p = 2^n"});
    cases.push_back({-40.0, 7, 2, false, 0, "128 snapshots"});
    cases.push_back({-40.0, 14, 2, false, 0, "16384 snapshots"});
    cases.push_back({10.0, 10, 2, true, 0, "covariance"});
    cases.push_back({-20.0, 10, 2, false, 1, "QPSK"});

    for (const Case &c : cases) {
        Win w = make_window(c.th, 1 << c.L, 20.0, 0.3, c.kind);
        bool ok;
        Res h = run_kernel(w, c.L, c.P, c.cov, MU, K, UCTOL, &ok);
        Res r = reference(w, TB_ALGO, c.P, c.cov, mu, k, uctol);
        double er = std::fabs(h.deg - r.deg), et = std::fabs(h.deg - c.th);
        worst_ref = std::max(worst_ref, er); worst_true = std::max(worst_true, et);
        pkt_all = pkt_all && ok;
        std::printf("ESPRIT_HLS_TB:   %-15s true %7.2f  kernel %9.4f  ref %9.4f  |psi| %.4f  flags 0x%x\n",
                    c.what, c.th, h.deg, r.deg, h.psi, h.flags);
        std::snprintf(msg, sizeof msg, "%s %+.0f deg: kernel = reference (< 0.01 deg)", c.what, c.th);
        check(msg, er < 0.01 && (h.flags & ~FLAG_OFF_CIRCLE) == (r.flags & ~FLAG_OFF_CIRCLE));
        std::snprintf(msg, sizeof msg, "%s %+.0f deg: within 0.5 deg of the true angle", c.what, c.th);
        check(msg, et < 0.5);
    }
    check("every window produced one 9-word packet with the magic word", pkt_all);
    std::printf("ESPRIT_HLS_TB:   worst |kernel - reference| %.5f deg, worst |kernel - true| %.4f deg\n",
                worst_ref, worst_true);

    // zero input -> no_signal
    {
        Win w = make_window(0, 1024, 0, 0.0, 3);
        bool ok; Res h = run_kernel(w, 10, 2, false, MU, K, UCTOL, &ok);
        check("all-zero input sets the no_signal flag", ok && (h.flags & FLAG_NO_SIGNAL));
    }
#if TB_ALGO == 1
    // noise only -> eigenvalue off the unit circle in most windows (Remark A)
    {
        int offc = 0;
        for (int i = 0; i < 16; i++) {
            Win w = make_window(0, 1024, 0, 0.3, 2);
            bool ok; Res h = run_kernel(w, 10, 2, false, MU, K, UCTOL, &ok);
            if (h.flags & FLAG_OFF_CIRCLE) offc++;
        }
        std::snprintf(msg, sizeof msg, "noise only: off_circle in %d/16 windows (>= 8)", offc);
        check(msg, offc >= 8);
    }
#endif
    std::printf("ESPRIT_HLS_TB: ============================================\n");
    if (g_fail == 0) std::printf("ESPRIT_HLS_TB_RESULT: PASS (%d checks)\n", g_checks);
    else             std::printf("ESPRIT_HLS_TB_RESULT: FAIL (%d of %d checks failed)\n", g_fail, g_checks);
    return g_fail ? 1 : 0;
}
