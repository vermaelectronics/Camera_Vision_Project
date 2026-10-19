// ============================================================================
//  testbench.cpp  --  C simulation of the LCMV CRPA HLS core (lcmv_crpa)
//
//  Runs the same loop the ARM runs on the board (gnss_lcmv_auto.c): read the
//  covariance R from the core, solve w = R^-1 C (C^H R^-1 C)^-1 f with the
//  firmware's own solver (Source/Firmware/app_gnss_e310/lcmv_math.c), load
//  the weights, and measure the output.
//
//  Scene: 2 elements, d = lambda/2, a(theta) = [1, exp(j*pi*sin(theta))]
//    jammer : noise-like, 400 LSB rms per I/Q (JNR ~ 46 dB)
//    GPS    : CW tone, 20 LSB, look direction
//    noise  : 2 LSB rms per I/Q per channel
//
//  Checks (all must pass for "LCMV_CSIM: PASS"):
//    T1 covariance of a known scene (R11, R22 within 10 %, |R12| ~ jammer)
//    T2 jammer-angle LCMV (paper eq. 9), GPS 30 / jammer 60 deg:
//       residual < -40 dB, GPS tone gain 0 dB +/- 1 dB
//    T3 covariance-only LCMV (look constraint), same scene: same limits
//    T4 jammer moves to -25 deg, covariance-only re-solve: same limits
//    T5 clipping is counted in sat_count
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "lcmv_crpa.h"
#include "lcmv_math.h"

static const double PI = 3.14159265358979;
static unsigned lcg = 777u;
static double gauss() {                     // Box-Muller
    lcg = lcg * 1103515245u + 12345u; double u1 = ((lcg >> 8) + 1.0) / 16777217.0;
    lcg = lcg * 1103515245u + 12345u; double u2 = ((lcg >> 8) + 1.0) / 16777217.0;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
}
static int q12(double v) {
    int k = (int)lrint(v);
    return k > 2047 ? 2047 : (k < -2048 ? -2048 : k);
}
static ap_uint<32> pack(int i, int q) {
    ap_uint<32> w;
    w(15, 0)  = (ap_uint<16>)(ap_int<16>)i;
    w(31, 16) = (ap_uint<16>)(ap_int<16>)q;
    return w;
}

// ---- DUT wrapper ---------------------------------------------------------------
static ap_int<32> W[4] = { 65536, 0, 0, 0 };      // w1 = 1 (identity)
static ap_uint<32> c11, c22, cblk, csat;
static ap_int<32>  c12r, c12i;

static double jam_deg = 60.0, gps_deg = 30.0;
static const double JAM = 400.0, TONE = 20.0, NOISE = 2.0, TONE_CYC = 0.0371;
static long n_global = 0;

struct Meas { double rx1, out, tre, tim; long n; };

static void run(long nsamp, Meas *m)
{
    hls::stream<ap_uint<32> > s1, s2, so;
    for (long k = 0; k < nsamp; k++, n_global++) {
        double jr = JAM * gauss(), ji = JAM * gauss();
        double ph = 2.0 * PI * TONE_CYC * n_global;
        double tr = TONE * cos(ph), ti = TONE * sin(ph);
        double pj = PI * sin(jam_deg * PI / 180.0), pt = PI * sin(gps_deg * PI / 180.0);
        int x1r = q12(jr + tr + NOISE * gauss()), x1i = q12(ji + ti + NOISE * gauss());
        int x2r = q12(jr * cos(pj) - ji * sin(pj) + tr * cos(pt) - ti * sin(pt) + NOISE * gauss());
        int x2i = q12(jr * sin(pj) + ji * cos(pj) + tr * sin(pt) + ti * cos(pt) + NOISE * gauss());
        s1.write(pack(x1r, x1i));
        s2.write(pack(x2r, x2i));
        lcmv_crpa(s1, s2, so, W[0], W[1], W[2], W[3],
                  &c11, &c22, &c12r, &c12i, &cblk, &csat);
        ap_uint<32> y = so.read();
        double yr = (double)(ap_int<16>)y(15, 0), yi = (double)(ap_int<16>)y(31, 16);
        if (m) {
            m->rx1 += (double)x1r * x1r + (double)x1i * x1i;
            m->out += yr * yr + yi * yi;
            m->tre += yr * cos(ph) + yi * sin(ph);
            m->tim += yi * cos(ph) - yr * sin(ph);
            m->n++;
        }
    }
}

static int fails = 0;
static void check(bool c, const char *what) {
    printf("  %s : %s\n", c ? "ok  " : "FAIL", what);
    if (!c) fails++;
}

static void load(const lcmv_result_t &w) {
    W[0] = (int)lrint(w.w1.re * 65536.0); W[1] = (int)lrint(w.w1.im * 65536.0);
    W[2] = (int)lrint(w.w2.re * 65536.0); W[3] = (int)lrint(w.w2.im * 65536.0);
}

static lcmv_cov_t read_cov() {
    lcmv_cov_t r;
    r.r11 = (double)c11 / 16.0;  r.r22 = (double)c22 / 16.0;
    r.r12.re = (double)c12r / 16.0;  r.r12.im = (double)c12i / 16.0;
    return r;
}

static void measure(const char *name, double *res_db, double *tone_db) {
    Meas m = {0, 0, 0, 0, 0};
    run(8000, &m);
    double tp = (m.tre * m.tre + m.tim * m.tim) / ((double)m.n * m.n);
    *res_db  = 10.0 * log10((m.out / m.n - tp + 1e-9) / (m.rx1 / m.n));
    *tone_db = 20.0 * log10(sqrt(tp) / TONE + 1e-12);
    printf("  %s: residual %.1f dB re RX1, GPS tone gain %.2f dB\n", name, *res_db, *tone_db);
}

int main()
{
    lcmv_result_t w;
    double res, tone;

    printf("T1 covariance of a known scene (GPS %.0f, jammer %.0f deg)\n", gps_deg, jam_deg);
    run(3 * 4096 + 10, 0);
    lcmv_cov_t r = read_cov();
    double ej = 2.0 * JAM * JAM, et = 2.0 * TONE * TONE, en = 2.0 * NOISE * NOISE;
    printf("  R11 %.0f  R22 %.0f  R12 %.0f%+.0fj  (expected R11 ~ %.0f), blocks %u\n",
           r.r11, r.r22, r.r12.re, r.r12.im, ej + et + en, (unsigned)cblk);
    check(fabs(r.r11 / (ej + et + en) - 1.0) < 0.1, "R11 within 10 %");
    check(fabs(r.r22 / (ej + et + en) - 1.0) < 0.1, "R22 within 10 %");
    check(fabs(sqrt(r.r12.re * r.r12.re + r.r12.im * r.r12.im) / ej - 1.0) < 0.1, "|R12| = jammer power");
    check(cblk == 3, "3 blocks completed");

    printf("T2 jammer-angle LCMV (paper eq. 9), GPS 30, jammer 60\n");
    lcmv_solve(&r, gps_deg, 1, jam_deg, 0.0, &w);
    printf("  w1 %.5f%+.5fj  w2 %.5f%+.5fj\n", w.w1.re, w.w1.im, w.w2.re, w.w2.im);
    load(w);
    measure("T2", &res, &tone);
    check(res < -40.0, "null < -40 dB");
    check(fabs(tone) < 1.0, "GPS gain 0 dB +/- 1 dB");

    printf("T3 covariance-only LCMV (no jammer angle)\n");
    run(2 * 4096, 0);
    r = read_cov();
    lcmv_solve(&r, gps_deg, 0, 0.0, 0.0, &w);
    printf("  JNR %.1f dB, null found at %.1f deg\n", w.jnr_db, w.null_deg);
    load(w);
    measure("T3", &res, &tone);
    check(res < -40.0, "null < -40 dB");
    check(fabs(tone) < 1.0, "GPS gain 0 dB +/- 1 dB");

    printf("T4 jammer moves to -25 deg, covariance-only re-solve\n");
    jam_deg = -25.0;
    run(2 * 4096, 0);                    // one block fully in the new scene
    r = read_cov();
    lcmv_solve(&r, gps_deg, 0, 0.0, 0.0, &w);
    printf("  null found at %.1f deg\n", w.null_deg);
    load(w);
    measure("T4", &res, &tone);
    check(res < -40.0, "null follows the jammer");
    check(fabs(tone) < 1.0, "GPS gain 0 dB +/- 1 dB");

    printf("T5 clipping counter\n");
    unsigned s0 = (unsigned)csat;
    W[0] = 100 * 65536; W[1] = 0; W[2] = 0; W[3] = 0;
    run(100, 0);
    printf("  sat_count %u -> %u\n", s0, (unsigned)csat);
    check((unsigned)csat > s0 + 50, "clipped samples counted");

    printf(fails ? "LCMV_CSIM: FAIL (%d)\n" : "LCMV_CSIM: PASS\n", fails);
    return fails ? 1 : 0;
}
