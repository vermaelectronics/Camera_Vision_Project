// Generates RTL test vectors for tb_gnss_passthrough_nlms.v from the
// pi_nlms C model: 2-element CW jammer + noise at AD9361 RX scale (12-bit
// signed, sign-extended to 16), and the C model's output for every sample.
//
//   stim.hex : per sample "QQQQIIII QQQQIIII" (element 0, element 1)
//   exp.hex  : per sample "QQQQIIII" pi_nlms output (16-bit, before 12-bit sat)
//
// Usage: gen_vectors <num_samples> <mu_shift_ctrl> <outdir>
#include "pi_nlms.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

static uint32_t pack(int16_t i, int16_t q) {
    return ((uint32_t)(uint16_t)q << 16) | (uint16_t)i;
}

int main(int argc, char **argv) {
    const int     N        = argc > 1 ? atoi(argv[1]) : 65536;
    const int16_t mu_shift = argc > 2 ? (int16_t)atoi(argv[2]) : 0;
    const char   *dir      = argc > 3 ? argv[3] : ".";

    const double JAMMER_AMP = 1500.0;        // ~-3 dBFS of a 12-bit ADC
    const double NOISE_AMP  = 25.0;
    const double F_NORM     = 0.1;           // jammer freq / sample rate
    const double PHASE_DIFF = M_PI / 3.0;    // element-to-element phase

    char path[512];
    snprintf(path, sizeof path, "%s/stim.hex", dir);
    FILE *fs = fopen(path, "w");
    snprintf(path, sizeof path, "%s/exp.hex", dir);
    FILE *fe = fopen(path, "w");
    if (!fs || !fe) { perror("fopen"); return 1; }

    hls::stream<axis_t> in1, in2, out;
    srand(1);
    for (int n = 0; n < N; n++) {
        double ph = 2.0 * M_PI * F_NORM * n;
        auto noise = [&] { return ((rand() % 2001) - 1000) / 1000.0 * NOISE_AMP; };
        int16_t i0 = (int16_t)lround(JAMMER_AMP * cos(ph) + noise());
        int16_t q0 = (int16_t)lround(JAMMER_AMP * sin(ph) + noise());
        int16_t i1 = (int16_t)lround(JAMMER_AMP * cos(ph + PHASE_DIFF) + noise());
        int16_t q1 = (int16_t)lround(JAMMER_AMP * sin(ph + PHASE_DIFF) + noise());

        axis_t a, b;
        a.data = pack(i0, q0); a.keep = -1; a.strb = -1; a.last = 0;
        b.data = pack(i1, q1); b.keep = -1; b.strb = -1; b.last = 0;
        in1.write(a);
        in2.write(b);
        pi_nlms(in1, in2, mu_shift, 0, out);

        fprintf(fs, "%08x%08x\n", (unsigned)pack(i1, q1), (unsigned)pack(i0, q0));
        fprintf(fe, "%08x\n", (unsigned)out.read().data);
    }
    fclose(fs);
    fclose(fe);
    printf("wrote %d samples to %s (mu_shift_ctrl=%d)\n", N, dir, mu_shift);
    return 0;
}
