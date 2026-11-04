/*
 * stap_solver.h -- STAP weight solver that runs on the CPU next to the FPGA core
 * (Zynq ARM, MicroBlaze, RISC-V soft core ...). Plain C99, no malloc, no libc
 * beyond <math.h>; the fixed-size loops are also Vitis-HLS friendly.
 *
 *   corr results from stap_corr  ->  stap_build_cov()  ->  R  (T x T, Hermitian)
 *   steering vector             ->  stap_steering()   ->  S
 *   MVDR  w = R^-1 S / (S^H R^-1 S)     (Cholesky, stap_solve_mvdr)
 *   quantise to WW-bit integers + output shift      (stap_quantize, stap_out_shift)
 */
#ifndef STAP_SOLVER_H
#define STAP_SOLVER_H

#include <stdint.h>

#define STAP_M   7                              /* antenna elements           */
#define STAP_N   16                             /* taps per element           */
#define STAP_T   (STAP_M * STAP_N)              /* 112 adaptive weights       */
#define STAP_NL  (2 * STAP_N - 1)               /* lags per channel pair (31) */
#define STAP_NP  (STAP_M * (STAP_M + 1) / 2)    /* channel pairs (28)         */
#define STAP_NRES (STAP_NP * STAP_NL)           /* correlation results (868)  */
#define STAP_HT  (2 * STAP_N - 2)               /* edge samples per channel   */

/* Edge samples of the captured block, read through the raw-sample window:
 *   head[m][i] = x_m[i],      tail[m][i] = x_m[L+i],     i = 0 .. STAP_HT-1   */
typedef struct {
    int32_t head_re[STAP_M][STAP_HT], head_im[STAP_M][STAP_HT];
    int32_t tail_re[STAP_M][STAP_HT], tail_im[STAP_M][STAP_HT];
} stap_edges;

typedef struct { double re, im; } stap_cplx;

/* Index of correlation result for channel pair a<=b and lag tau in [-(N-1), N-1]. */
int stap_corr_index(int a, int b, int tau);

/* Build the exact T x T covariance matrix  R = Xm*Xm'/L  (as in STAP_constraint.m)
 * from the stap_corr results plus the edge samples.
 * corr_re/corr_im : STAP_NRES signed integers read from the core
 * L               : number of snapshots used (CORR_LEN)                     */
void stap_build_cov(const int64_t *corr_re, const int64_t *corr_im, const stap_edges *ed,
                    uint32_t L, stap_cplx R[STAP_T][STAP_T]);

/* Space-time steering vector for look angle theta_deg.
 *   d_over_lambda : element spacing / wavelength (0.5 for the MATLAB model)
 *   fif_over_fs   : IF / sample rate (10/60 for the MATLAB model)            */
void stap_steering(double d_over_lambda, double theta_deg, double fif_over_fs,
                   stap_cplx S[STAP_T]);

/* Power-inversion constraint: S = [1 0 0 ... 0]. */
void stap_steering_pi(stap_cplx S[STAP_T]);

/* MVDR / LCMV solution. diag_load is added to the diagonal relative to the
 * mean diagonal power (e.g. 1e-6). R is not modified. Returns 0 on success,
 * -1 if the matrix is not positive definite.                               */
int stap_solve_mvdr(const stap_cplx R[STAP_T][STAP_T], const stap_cplx S[STAP_T],
                    double diag_load, stap_cplx w[STAP_T]);

/* Quantise weights to ww-bit two's complement integers: wq = round(w * 2^exp).
 * Returns exp (the power-of-two scale that was applied).                   */
int stap_quantize(const stap_cplx w[STAP_T], int ww,
                  int32_t wq_re[STAP_T], int32_t wq_im[STAP_T]);

/* Recommended OUT_SHIFT so that the output RMS sits at 2^(ow-1) / 2^headroom_bits
 * (e.g. headroom_bits = 3 -> 18 dB below full scale).                       */
int stap_out_shift(const stap_cplx R[STAP_T][STAP_T], const stap_cplx w[STAP_T],
                   int exp, int ow, int headroom_bits);

/* Beam response w^H * a(theta, f) of the space-time filter (linear magnitude). */
double stap_response(const stap_cplx w[STAP_T], double d_over_lambda, double theta_deg,
                     double f_over_fs, double frf_over_fc);

#endif
