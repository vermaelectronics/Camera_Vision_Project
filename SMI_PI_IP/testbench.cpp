// ============================================================================
//  testbench.cpp  --  C simulation of the SMI power-inversion core
//
//  Two-element array at the E310 raw ADC scale (12-bit samples):
//      x1 = J + s + n1
//      x2 = h J + g s + n2      h = 0.6 - 0.5j   (jammer direction)
//                               g = -0.2 + 0.9j  (satellite direction)
//  J: CW jammer, s: weak BPSK "satellite", n1/n2: independent uniform noise
//  of +-20 LSB (about 280 LSB^2 per element).
//
//  The output SINR of s is measured by correlation and compared with the
//  best any 2-element weight can achieve (jammer fully nulled):
//      SINR_opt = |s|^2 * (|v|^2 - |u^H v|^2/|u|^2) / sigma^2,  u=[1,h], v=[1,g]
//
//  Scenario (argv[1]); each needs a freshly reset core, so the csim script
//  runs one per process:
//    1  jammer 600 LSB: 40 dB null within 2000 samples, steady-state SINR
//       within 1 dB of the optimum
//    2  no jammer: output power within 1 dB of RX1, SINR loss < 1.5 dB
//    3  jammer 2000 LSB switched on/off every 25000 samples: away from the
//       switching instants no 1000-sample block of output exceeds 36 dB
//    4  weak jammer 60 LSB: SINR within 1.5 dB of the optimum
//  Prints "SMI_PI_CSIM: PASS" or "SMI_PI_CSIM: FAIL".
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "smi_pi.h"

static unsigned lcg_n = 4242u;
static int noise(int a) {
    lcg_n = lcg_n * 1103515245u + 12345u;
    return (int)((lcg_n >> 8) % (unsigned)(2 * a + 1)) - a;
}
static unsigned lcg_b = 999u;
static int bpsk() {
    lcg_b = lcg_b * 1664525u + 1013904223u;
    return ((lcg_b >> 16) & 1) ? 1 : -1;
}

static const double HR = 0.6, HI = -0.5, GR = -0.2, GI = 0.9;
static const double SA = 6.0;           // satellite amplitude (LSB)
static const int    NA = 20;            // noise +-NA LSB

struct Res {
    double sinr_db, conv, out_db, rx1_db, max_blk_db, null_db;
};

static Res run(double amp, int nsamp, int toggle, unsigned load) {
    hls::stream<ap_uint<64> > xs;
    hls::stream<ap_uint<32> > ys;
    ap_uint<2> band = 0;
    Res r = {0, -1, 0, 0, -1e9, 0};
    double cjr = 0, cji = 0, cjp = 0;     // correlation of y with the jammer, per block
    double ssr = 0, ssi = 0, pyy = 0, px1 = 0, cnt = 0, blk = 0;
    int on = 1, since_edge = 0;
    for (int n = 0; n < nsamp; n++) {
        if (toggle && n && n % toggle == 0) { on = !on; since_edge = 0; }
        since_edge++;
        double a = on ? amp : 0.0, ph = 2.0 * M_PI * 0.0137 * n;
        double jr = a * cos(ph), ji = a * sin(ph);
        int s = bpsk();
        int x1r = (int)lround(jr + SA * s) + noise(NA);
        int x1i = (int)lround(ji) + noise(NA);
        int x2r = (int)lround(HR * jr - HI * ji + GR * SA * s) + noise(NA);
        int x2i = (int)lround(HR * ji + HI * jr + GI * SA * s) + noise(NA);
        ap_uint<64> w;
        w(15, 0)  = (ap_uint<16>)(ap_int<16>)x1r;
        w(31, 16) = (ap_uint<16>)(ap_int<16>)x1i;
        w(47, 32) = (ap_uint<16>)(ap_int<16>)x2r;
        w(63, 48) = (ap_uint<16>)(ap_int<16>)x2i;
        xs.write(w);
        smi_pi(xs, ys, load, 1, &band);
        ap_uint<32> y = ys.read();
        int yr = (ap_int<16>)y(15, 0), yi = (ap_int<16>)y(31, 16);

        double p = (double)yr * yr + (double)yi * yi;
        blk += p;
        // y = c*J + (rest): c from the correlation with the known jammer
        cjr += yr * jr + yi * ji; cji += yi * jr - yr * ji; cjp += jr * jr + ji * ji;
        if ((n + 1) % 1000 == 0) {
            double bp = blk / 1000.0;
            if (amp > 0 && cjp > 0) {
                double c2 = (cjr * cjr + cji * cji) / (cjp * cjp);   // residual jammer gain^2
                if (r.conv < 0 && c2 < 1e-4) r.conv = n + 1;
                r.null_db = 10 * log10(c2 + 1e-30);
            }
            cjr = cji = cjp = 0;
            if (since_edge > 3000 && n > 3000) {
                double db = 10 * log10(bp + 1e-9);
                if (db > r.max_blk_db) r.max_blk_db = db;
            }
            blk = 0;
        }
        if (n >= nsamp / 2) {
            ssr += yr * s; ssi += yi * s; pyy += p; cnt += 1;
            px1 += (double)x1r * x1r + (double)x1i * x1i;
        }
    }
    double gr = ssr / cnt, gi = ssi / cnt, sig = gr * gr + gi * gi;
    double tot = pyy / cnt;
    r.sinr_db = 10 * log10(sig / (tot - sig));
    r.out_db  = 10 * log10(tot);
    r.rx1_db  = 10 * log10(px1 / cnt);
    return r;
}

static double sinr_opt_db(bool jammer) {
    double sig2 = 2.0 * NA * (NA + 1) / 3.0;            // noise power per element
    double v2 = 1 + GR * GR + GI * GI;
    if (!jammer) return 10 * log10(SA * SA * v2 / sig2);  // (MRC; PI keeps w1 = 1)
    double u2 = 1 + HR * HR + HI * HI;
    double cr = 1 + HR * GR + HI * GI, ci = HR * GI - HI * GR;   // u^H v
    return 10 * log10(SA * SA * (v2 - (cr * cr + ci * ci) / u2) / sig2);
}

int main(int argc, char **argv) {
    int scen = (argc > 1) ? atoi(argv[1]) : 1;
    int fail = 0;
    const unsigned LOAD = 16;
    if (scen == 1) {
        Res r = run(600.0, 200000, 0, LOAD);
        double opt = sinr_opt_db(true);
        printf("jammer 600 LSB: 40 dB null by sample %.0f, final jammer null %.1f dB, SINR %.2f dB\n"
               "  (optimum %.2f dB, RX1 alone %.1f dB), output %.1f dB vs jammer %.1f dB\n",
               r.conv, r.null_db, r.sinr_db, opt,
               10 * log10(SA * SA / (600.0 * 600 + 2.0 * NA * (NA + 1) / 3)),
               r.out_db, 10 * log10(600.0 * 600));
        if (r.conv < 0 || r.conv > 2000) { printf("FAIL: no 40 dB null within 2000 samples\n"); fail = 1; }
        if (r.sinr_db < opt - 1.0)       { printf("FAIL: SINR more than 1 dB below the optimum\n"); fail = 1; }
    } else if (scen == 2) {
        Res r = run(0.0, 200000, 0, LOAD);
        double rx1_sinr = 10 * log10(SA * SA / (2.0 * NA * (NA + 1) / 3));
        printf("no jammer: output %.2f dB vs RX1 %.2f dB, SINR %.2f dB (RX1 alone %.2f dB)\n",
               r.out_db, r.rx1_db, r.sinr_db, rx1_sinr);
        if (fabs(r.out_db - r.rx1_db) > 1.0) { printf("FAIL: output level differs from RX1 by > 1 dB\n"); fail = 1; }
        if (r.sinr_db < rx1_sinr - 1.5)      { printf("FAIL: SINR loss > 1.5 dB without a jammer\n"); fail = 1; }
    } else if (scen == 3) {
        Res r = run(2000.0, 200000, 25000, LOAD);
        printf("jammer 2000 LSB on/off every 25000 samples: worst settled 1000-sample block %.1f dB "
               "(jammer %.1f dB)\n", r.max_blk_db, 10 * log10(2000.0 * 2000));
        if (r.max_blk_db > 36.0) { printf("FAIL: output not settled after a switch\n"); fail = 1; }
    } else if (scen == 4) {
        Res r = run(60.0, 200000, 0, LOAD);
        double opt = sinr_opt_db(true);
        printf("weak jammer 60 LSB: SINR %.2f dB (optimum %.2f dB)\n", r.sinr_db, opt);
        if (r.sinr_db < opt - 1.5) { printf("FAIL: SINR more than 1.5 dB below the optimum\n"); fail = 1; }
    }
    printf(fail ? "SMI_PI_CSIM: FAIL\n" : "SMI_PI_CSIM: PASS\n");
    return fail;
}
