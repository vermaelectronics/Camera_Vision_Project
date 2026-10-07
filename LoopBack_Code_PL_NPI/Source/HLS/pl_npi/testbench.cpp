// ============================================================================
//  testbench.cpp  --  C simulation of the pipelined PL-NPI core
//
//  Two-element array at the E310 raw ADC scale (12-bit samples):
//      RX1 = J + n1,   RX2 = h * J + n2,   h = 0.6 - 0.5j
//  J is a CW jammer, n1/n2 independent noise (uniform, about +-NOISE LSB).
//
//  Checks (all must pass for "PL_NPI_CSIM: PASS"):
//    1. Jammer 600 LSB, gamma 1: output power after convergence is at least
//       40 dB below the jammer power, reached within 2000 samples.
//    2. Same, and the output does not grow again later (no oscillation from
//       the delayed update).
//    3. Gain-band coverage: more than one Eq. 11 band is used.
//    4. No jammer, noise +-20 LSB, gamma 1e8, 200000 samples: the output is
//       NOT cancelled in steady state (second half within 3 dB of RX1), i.e.
//       gamma protects the wanted signal. gamma acts as diagonal loading of
//       about gamma/2^17 LSB^2 (see pl_npi.cpp); it must be well above the
//       noise power per channel (about 267 LSB^2 here). A smaller gamma
//       (1e6) only slows the cancellation down, it does not prevent it.
//
//  Writing the stimulus/response to files for a comparison against the
//  original RTL core is enabled with the environment variable PL_NPI_DUMP.
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "pl_npi.h"

static unsigned lcg = 12345u;
static int noise(int a) {                  // uniform integer in [-a, a]
    lcg = lcg * 1103515245u + 12345u;
    return (int)((lcg >> 8) % (unsigned)(2 * a + 1)) - a;
}

struct Result {
    double in_pow, first_blk, last_blk;
    int conv_sample;
    bool grew;
    int bands_seen;
    double out_vs_rx1_db;
};

static Result run(double amp, unsigned gamma, int nsamp, int noise_a,
                  FILE *fstim, FILE *fresp) {
    hls::stream<ap_uint<64> > xs;
    hls::stream<ap_uint<32> > ys;
    ap_uint<2> band = 0;
    Result r = {amp * amp, 0, 0, -1, false, 0, 0};
    int seen[4] = {0, 0, 0, 0};
    const double hr = 0.6, hi = -0.5;
    double blk = 0, rx1p = 0, outp = 0, prev_blk = -1, min_blk = 1e30;
    int nb = 0;

    for (int n = 0; n < nsamp; n++) {
        double ph = 2.0 * M_PI * 0.0137 * n;
        double jr = amp * cos(ph), ji = amp * sin(ph);
        int x1r = (int)jr + noise(noise_a), x1i = (int)ji + noise(noise_a);
        int x2r = (int)(hr * jr - hi * ji) + noise(noise_a);
        int x2i = (int)(hr * ji + hi * jr) + noise(noise_a);
        ap_uint<64> w;
        w(15, 0)  = (ap_uint<16>)(ap_int<16>)x1r;
        w(31, 16) = (ap_uint<16>)(ap_int<16>)x1i;
        w(47, 32) = (ap_uint<16>)(ap_int<16>)x2r;
        w(63, 48) = (ap_uint<16>)(ap_int<16>)x2i;
        xs.write(w);
        pl_npi(xs, ys, gamma, 1, &band);
        ap_uint<32> y = ys.read();
        int yr = (ap_int<16>)y(15, 0), yi = (ap_int<16>)y(31, 16);
        seen[(int)band] = 1;
        if (fstim) fprintf(fstim, "%016llx\n", (unsigned long long)w.to_uint64());
        if (fresp) fprintf(fresp, "%d %d\n", yr, yi);

        blk += (double)yr * yr + (double)yi * yi;
        if (n >= nsamp / 2) {
            rx1p += (double)x1r * x1r + (double)x1i * x1i;
            outp += (double)yr * yr + (double)yi * yi;
        }
        if ((n + 1) % 100 == 0) {
            double p = blk / 100.0;
            if (nb == 0) r.first_blk = p;
            if (r.conv_sample < 0 && amp > 0 && p < r.in_pow * 1e-4)
                r.conv_sample = n + 1;
            if (r.conv_sample >= 0 && p < min_blk) min_blk = p;
            if (r.conv_sample >= 0 && p > 10.0 * min_blk + 5.0) r.grew = true;
            prev_blk = p;
            r.last_blk = p;
            blk = 0;
            nb++;
        }
    }
    (void)prev_blk;
    for (int k = 0; k < 4; k++) r.bands_seen += seen[k];
    r.out_vs_rx1_db = 10.0 * log10((outp + 1e-9) / (rx1p + 1e-9));
    return r;
}

int main(int argc, char **argv) {
    // Each scenario needs a freshly reset core; the HLS function keeps its
    // state in statics, so the scenario is chosen on the command line and
    // the csim script runs this executable once per scenario. Without an
    // argument, scenario 1 (jammer) runs.
    int scen = (argc > 1) ? atoi(argv[1]) : 1;
    const char *dump = getenv("PL_NPI_DUMP");
    FILE *fs = 0, *fr = 0;
    if (dump) {
        char a[512], b[512];
        snprintf(a, sizeof a, "%s_stim.hex", dump);
        snprintf(b, sizeof b, "%s_resp.txt", dump);
        fs = fopen(a, "w");
        fr = fopen(b, "w");
    }
    int fail = 0;
    if (scen == 1) {
        Result r = run(600.0, 1u, 6000, 2, fs, fr);
        printf("jammer 600 LSB, gamma 1: in %.0f  first 100 %.1f  last 100 %.2f"
               "  null %.1f dB  converged at sample %d  bands %d%s\n",
               r.in_pow, r.first_blk, r.last_blk,
               10.0 * log10(r.last_blk / r.in_pow), r.conv_sample, r.bands_seen,
               r.grew ? "  GREW AGAIN" : "");
        if (r.conv_sample < 0 || r.conv_sample > 2000) { printf("FAIL: no 40 dB null within 2000 samples\n"); fail = 1; }
        if (r.last_blk > r.in_pow * 1e-4)              { printf("FAIL: final null shallower than 40 dB\n"); fail = 1; }
        if (r.grew)                                    { printf("FAIL: output grew again after converging\n"); fail = 1; }
        if (r.bands_seen < 2)                          { printf("FAIL: only one gain band used\n"); fail = 1; }
    } else if (scen == 2) {
        Result r = run(0.0, 100000000u, 200000, 20, fs, fr);
        printf("no jammer, noise +-20 LSB, gamma 1e8, 200000 samples: output vs RX1 power %.1f dB\n",
               r.out_vs_rx1_db);
        if (r.out_vs_rx1_db < -3.0) { printf("FAIL: wanted signal cancelled without a jammer\n"); fail = 1; }
    } else if (scen == 3) {
        Result r = run(0.0, 1u, 6000, 20, fs, fr);
        printf("no jammer, noise +-20 LSB, gamma 1 (informational): output vs RX1 power %.1f dB\n",
               r.out_vs_rx1_db);
    }
    if (fs) fclose(fs);
    if (fr) fclose(fr);
    printf(fail ? "PL_NPI_CSIM: FAIL\n" : "PL_NPI_CSIM: PASS\n");
    return fail;
}
