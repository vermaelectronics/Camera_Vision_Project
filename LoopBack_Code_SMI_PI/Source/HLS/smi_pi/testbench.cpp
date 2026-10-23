// ============================================================================
//  testbench.cpp  --  C simulation of the space-time SMI-PI core (v4.0)
//
//  The weights come from the FIRMWARE's own solver
//  (Source/Firmware/app_gnss_e310/gnss_aj_math.c, compiled into this
//  testbench), run on a 4096-sample capture packed exactly like the DMA
//  buffer, so this tests the board's weight computation and the FPGA filter
//  together.
//
//  Signals at the raw 12-bit ADC scale, noise 20 LSB rms per I/Q component:
//      RX1 = J + s + n1
//      RX2 = H(J) + g s + n2
//  J: complex Gaussian jammer low-passed to 0.84 fs (it fills the band like
//     the 55 MHz jammer at 50 MS/s), or a CW tone.
//  H: what makes a wideband jammer hard for a single weight: a 0.15-sample
//     (3 ns at 50 MS/s) delay, 0.8 dB / 40 deg, and a slightly different
//     filter.
//  s: weak BPSK "satellite" (1.023 Mchip/s at 50 MS/s, -9.08 MHz offset).
//
//  Scenario (argv[1]); each needs a freshly reset core, so the csim script
//  runs one per process:
//    1  bit-exact: random weights and samples (full ranges) against an
//       integer reference of the filter
//    2  wideband jammer 30 dB: suppression >= 35 dB, satellite SNR loss <= 3 dB,
//       and >= 10 dB more suppression than the single-weight (v3) mode
//    3  no jammer: output power within 0.5 dB of RX1, satellite loss <= 0.5 dB
//    4  CW jammer 40 dB: suppression >= 40 dB
//    5  wideband jammer 30 dB plus a reflection (-6 dB, 3 samples later,
//       other direction): suppression >= 30 dB
//    6  wideband jammer 50 dB (RX gain cut so the ADC does not clip):
//       suppression >= 40 dB
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

static void set_weights(const int16_t *wre, const int16_t *wim) {
    g_w = 0;
    for (int k = 0; k < 32; k++) {
        g_w(32 * k + 15, 32 * k)      = (ap_uint<16>)(ap_int<16>)wre[k];
        g_w(32 * k + 31, 32 * k + 16) = (ap_uint<16>)(ap_int<16>)wim[k];
    }
}

// Runs (x1,x2) through the core after a 32-sample zero flush; returns y.
static cvec run_core(const std::vector<int> &a1r, const std::vector<int> &a1i,
                     const std::vector<int> &a2r, const std::vector<int> &a2i) {
    hls::stream<ap_uint<64> > xs;
    hls::stream<ap_uint<32> > ys;
    cvec y;
    int n = a1r.size();
    for (int i = -32; i < n; i++) {
        int v[4] = {0, 0, 0, 0};
        if (i >= 0) { v[0] = a1r[i]; v[1] = a1i[i]; v[2] = a2r[i]; v[3] = a2i[i]; }
        ap_uint<64> w = 0;
        for (int c = 0; c < 4; c++) w(16 * c + 15, 16 * c) = (ap_uint<16>)(ap_int<16>)v[c];
        xs.write(w);
        smi_pi(xs, ys, g_w);
        if (ys.empty()) { std::printf("core produced no output\n"); std::exit(2); }
        ap_uint<32> o = ys.read();
        if (i >= 0) y.push_back(cd((double)(ap_int<16>)o(15, 0), (double)(ap_int<16>)o(31, 16)));
    }
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
static int test_bitexact() {
    int16_t wr[32], wi[32];
    for (int k = 0; k < 32; k++) {
        wr[k] = (int16_t)((int)(urand() * 65536) - 32768);
        wi[k] = (int16_t)((int)(urand() * 65536) - 32768);
    }
    wr[0] = 32767; wi[0] = -32768; wr[16] = -32768; wi[16] = -32768;   // extremes
    set_weights(wr, wi);
    int n = 20000, bad = 0, sat = 0;
    std::vector<int> a(n), b(n), c(n), d(n);
    for (int i = 0; i < n; i++) {
        bool ext = (i % 97) < 20;
        a[i] = ext ? (urand() < 0.5 ? -2048 : 2047) : (int)(urand() * 4096) - 2048;
        b[i] = ext ? (urand() < 0.5 ? -2048 : 2047) : (int)(urand() * 4096) - 2048;
        c[i] = ext ? (urand() < 0.5 ? -2048 : 2047) : (int)(urand() * 4096) - 2048;
        d[i] = ext ? (urand() < 0.5 ? -2048 : 2047) : (int)(urand() * 4096) - 2048;
    }
    cvec y = run_core(a, b, c, d);
    for (int i = 0; i < n; i++) {
        long long sr = 0, si = 0;
        auto at = [&](const std::vector<int> &v, int j) { return j >= 0 ? (long long)v[j] : 0LL; };
        sr = at(a, i - SMI_REF) << SMI_WFRAC; si = at(b, i - SMI_REF) << SMI_WFRAC;
        for (int k = 0; k < SMI_NT; k++) {
            if (k != SMI_REF) {
                long long xr = at(a, i - k), xi = at(b, i - k);
                sr -= wr[k] * xr - wi[k] * xi; si -= wr[k] * xi + wi[k] * xr;
            }
            long long xr = at(c, i - k), xi = at(d, i - k);
            sr -= wr[16 + k] * xr - wi[16 + k] * xi; si -= wr[16 + k] * xi + wi[16 + k] * xr;
        }
        long long half = 1LL << (SMI_WFRAC - 1);
        long long er = (sr + half) >> SMI_WFRAC, ei = (si + half) >> SMI_WFRAC;
        if (er > 32767 || er < -32768 || ei > 32767 || ei < -32768) sat++;
        er = er > 32767 ? 32767 : er < -32768 ? -32768 : er;
        ei = ei > 32767 ? 32767 : ei < -32768 ? -32768 : ei;
        if ((long long)y[i].real() != er || (long long)y[i].imag() != ei) {
            if (bad < 5) std::printf("  mismatch at %d: core (%g,%g) ref (%lld,%lld)\n", i, y[i].real(), y[i].imag(), er, ei);
            bad++;
        }
    }
    std::printf("bit-exact: %d samples, %d mismatches, %d saturated\n", n, bad, sat);
    return bad == 0 && sat > 0;
}

// ---- scenarios 2..6 ------------------------------------------------------------
struct Scen { double jnr_db; bool cw; double echo; };

static int test_jammer(int id, Scen sc) {
    const int N = 3 * AJ_NSAMP + 20000;
    const double nrms = 20.0, fsat = -9.08 / 50.0;
    cvec J(N), J2, n1(N), n2(N), s1(N), s2(N);
    // jammer
    if (sc.cw) {
        for (int i = 0; i < N; i++) J[i] = std::polar(1.0, 2 * M_PI * 0.06 * i);
    } else {
        for (int i = 0; i < N; i++) J[i] = cd(grand(), grand());
        J = fir(J, lowpass(0.42, 41), 20);
    }
    // RX2 sees it through a different path: 0.15-sample delay, 0.8 dB, 40 deg,
    // and a slightly different filter.
    cvec h2 = frac_delay(0.15, 21);
    cvec hd = {cd(0.03, 0.01), 1.0, cd(-0.02, 0.015)};
    J2 = fir(fir(J, h2, 10), hd, 1);
    cd g2 = std::polar(std::pow(10, 0.8 / 20), 40 * M_PI / 180);
    for (auto &v : J2) v *= g2;
    if (sc.echo > 0) {      // a reflection 3 samples later from another direction
        cvec e1(N), e2(N);
        for (int i = 3; i < N; i++) { e1[i] = sc.echo * std::polar(1.0, 1.3) * J[i - 3]; e2[i] = sc.echo * std::polar(1.0, 2.9) * J[i - 3]; }
        for (int i = 0; i < N; i++) { J[i] += e1[i]; J2[i] += e2[i]; }
    }
    double pj = pwr(J, 0), pn = 2 * nrms * nrms;
    double js = std::sqrt(pn * std::pow(10, sc.jnr_db / 10) / pj);
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
    // matched gain: one common scale so the ADC peak stays below 1850
    double pk = 0;
    for (int i = 0; i < N; i++) {
        cd a = J[i] + n1[i] + s1[i], b = J2[i] + n2[i] + s2[i];
        pk = std::max(pk, std::max(std::max(std::fabs(a.real()), std::fabs(a.imag())), std::max(std::fabs(b.real()), std::fabs(b.imag()))));
    }
    double g = 1.0;
    while (pk * g > 1850) g *= std::pow(10, -1.0 / 20);
    cvec x1(N), x2(N);
    for (int i = 0; i < N; i++) {
        J[i] *= g; J2[i] *= g; n1[i] *= g; n2[i] *= g; s1[i] *= g; s2[i] *= g;
        x1[i] = cd(q12((J[i] + n1[i] + s1[i]).real()), q12((J[i] + n1[i] + s1[i]).imag()));
        x2[i] = cd(q12((J2[i] + n2[i] + s2[i]).real()), q12((J2[i] + n2[i] + s2[i]).imag()));
    }

    // firmware solver on three consecutive DMA-format captures
    auto solve = [&](int mode, aj_result_t &r) {
        static uint32_t buf[2 * AJ_NSAMP];
        std::memset(&r, 0, sizeof r);
        r.mode = mode; r.sig2_prev = -1; r.noise_ref = pn * g * g + 0.17;
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
    auto measure = [&](const aj_result_t &r, double &supp, double &sat_loss, double &out_db) {
        set_weights(r.wre, r.wim);
        cvec yj = run_core_c(seg(J), seg(J2));
        cvec yx = run_core_c(seg(x1), seg(x2));
        supp = (sc.jnr_db > -90) ? 10 * std::log10(pwr(seg(J), 0) / std::max(pwr(yj, 32), 1e-9)) : 0;
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
        for (size_t i = 0; i < ref.size(); i++) clean[i] = n1s[i] + ref[i];
        sat_loss = 10 * std::log10(snr(clean, 0) / snr(yx, SMI_REF));
        out_db = 10 * std::log10(pwr(yx, 32) / pwr(seg(x1), 0));
    };

    aj_result_t r, rnb;
    if (!solve(AJ_MODE_STAP, r)) { std::printf("solver failed\n"); return 0; }
    double supp, loss, outdb;
    measure(r, supp, loss, outdb);
    std::printf("scenario %d: JNR %.0f dB%s%s, RX gain cut %.1f dB\n", id, sc.jnr_db,
                sc.cw ? " CW" : " wideband", sc.echo > 0 ? " + reflection" : "", -20 * std::log10(g));
    std::printf("  space-time 31 taps: suppression %.1f dB, satellite SNR loss %.2f dB, output %+.2f dB re RX1\n", supp, loss, outdb);
    std::printf("  firmware figures : J/N %.1f dB, suppression %.1f dB, residual %+.1f dB, |w|^2 %.2f, loading %.1f LSB^2 (%d passes)\n",
                r.jn_db, r.supp_db, r.res_db, r.wn2, r.sig2, r.iters);
    if (!sc.cw && sc.jnr_db > -90)
        std::printf("  jammer estimate  : bandwidth %.2f fs, centre %+.3f fs\n", r.bw, r.foff);
    if (sc.cw)
        std::printf("  jammer estimate  : bandwidth %.4f fs, centre %+.4f fs (true 0.06)\n", r.bw, r.foff);

    int ok = 1;
    switch (id) {
    case 2: {
        double s_nb, l_nb, o_nb;
        if (!solve(AJ_MODE_NB, rnb)) return 0;
        measure(rnb, s_nb, l_nb, o_nb);
        std::printf("  one weight (v3)  : suppression %.1f dB, satellite SNR loss %.2f dB\n", s_nb, l_nb);
        ok = supp >= 35 && loss <= 3 && supp >= s_nb + 10;
        break; }
    case 3: ok = std::fabs(outdb) <= 0.5 && std::fabs(loss) <= 0.5 && r.jn_db < 3; break;
    case 4: ok = supp >= 40 && loss <= 3; break;
    case 5: ok = supp >= 30 && loss <= 3; break;
    case 6: ok = supp >= 40; break;
    }
    return ok;
}

int main(int argc, char **argv) {
    int scen = (argc > 1) ? std::atoi(argv[1]) : 2;
    int ok;
    switch (scen) {
    case 1: ok = test_bitexact(); break;
    case 2: ok = test_jammer(2, {30, false, 0}); break;
    case 3: ok = test_jammer(3, {-99, false, 0}); break;
    case 4: ok = test_jammer(4, {40, true, 0}); break;
    case 5: ok = test_jammer(5, {30, false, 0.5}); break;
    case 6: ok = test_jammer(6, {50, false, 0}); break;
    default: std::printf("unknown scenario %d\n", scen); ok = 0;
    }
    std::printf("SMI_PI_CSIM: %s (scenario %d)\n", ok ? "PASS" : "FAIL", scen);
    return ok ? 0 : 1;
}
