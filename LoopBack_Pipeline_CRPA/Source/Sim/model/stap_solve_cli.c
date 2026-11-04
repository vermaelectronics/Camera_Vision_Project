/*
 * stap_solve_cli.c -- host-side test harness for stap_solver.c
 *
 *   stap_solve_cli <corr.txt> <L> <mvdr|pi> <theta_deg> <d/lambda> <IF/fs> <WW> <OW> <headroom> <out.txt>
 *
 * corr.txt : STAP_NRES lines "re im" (decimal, as read from the core), followed by
 *            STAP_M*STAP_HT head samples and STAP_M*STAP_HT tail samples "re im"
 *            (channel-major)
 * out.txt  : line 1 "exp out_shift", then STAP_T lines "wq_re wq_im"
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stap_solver.h"

static int64_t   cre[STAP_NRES], cim[STAP_NRES];
static stap_cplx R[STAP_T][STAP_T];
static stap_edges ed;

int main(int argc, char **argv)
{
    stap_cplx S[STAP_T], w[STAP_T];
    int32_t wr[STAP_T], wi[STAP_T];
    FILE *f;
    int e, i, sh, ww, ow, hr;
    uint32_t L;

    if (argc != 11) {
        fprintf(stderr, "usage: %s corr.txt L mvdr|pi theta d/lambda IF/fs WW OW headroom out.txt\n", argv[0]);
        return 2;
    }
    f = fopen(argv[1], "r");
    if (!f) { perror(argv[1]); return 1; }
    for (i = 0; i < STAP_NRES; i++)
        if (fscanf(f, "%lld %lld", (long long *)&cre[i], (long long *)&cim[i]) != 2) {
            fprintf(stderr, "short corr file\n"); return 1;
        }
    {
        int m, k, part;
        for (part = 0; part < 2; part++)
            for (m = 0; m < STAP_M; m++)
                for (k = 0; k < STAP_HT; k++) {
                    int32_t *pr = part ? &ed.tail_re[m][k] : &ed.head_re[m][k];
                    int32_t *pi = part ? &ed.tail_im[m][k] : &ed.head_im[m][k];
                    if (fscanf(f, "%d %d", pr, pi) != 2) { fprintf(stderr, "short edge data\n"); return 1; }
                }
    }
    fclose(f);

    L  = (uint32_t)strtoul(argv[2], NULL, 10);
    ww = atoi(argv[7]); ow = atoi(argv[8]); hr = atoi(argv[9]);

    stap_build_cov(cre, cim, &ed, L, R);
    if (strcmp(argv[3], "pi") == 0) stap_steering_pi(S);
    else stap_steering(atof(argv[5]), atof(argv[4]), atof(argv[6]), S);

    if (stap_solve_mvdr(R, S, 1e-9, w) != 0) { fprintf(stderr, "Cholesky failed\n"); return 1; }
    e  = stap_quantize(w, ww, wr, wi);
    sh = stap_out_shift(R, w, e, ow, hr);

    f = fopen(argv[10], "w");
    if (!f) { perror(argv[10]); return 1; }
    fprintf(f, "%d %d\n", e, sh);
    for (i = 0; i < STAP_T; i++) fprintf(f, "%d %d\n", wr[i], wi[i]);
    fclose(f);
    printf("solver: exp=%d out_shift=%d\n", e, sh);
    return 0;
}
