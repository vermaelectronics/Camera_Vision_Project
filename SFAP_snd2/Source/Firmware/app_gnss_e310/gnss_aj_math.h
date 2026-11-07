/******************************************************************************
 *  gnss_aj_math.h
 *  Space-frequency SMI power inversion: the weight computation (SFAP-PI v5.0).
 *
 *  Pure C, no hardware access, no libm. The same file is compiled into the
 *  firmware (gnss_aj.c calls it once per captured block) and into the HLS
 *  C testbench (Source/HLS/sfap_pi/testbench.cpp), so the weights the core is
 *  tested with are exactly the weights the board computes.
 *
 *  FILTER (in the FPGA, sfap_pi_0):
 *      x2n = g2 x2   (RX2 normalised to RX1 before the FFT)
 *      frames of AJ_N = 64 samples, hop 32, window sin(pi n / 64) on analysis
 *      and synthesis (weighted overlap-add). Per frame and bin k:
 *        Y_k = X1_k - b_k X2n_k - c1_k conj(X1_{-k}) - c2_k conj(X2n_{-k})
 *      The RX1 coefficient is fixed at 1 (power inversion).
 *
 *  WEIGHTS (here), independently in every bin k:
 *      v = [X2n_k, conj(X1_{-k}), conj(X2n_{-k})],  d = X1_k
 *      minimise E|d - w^T v|^2  ->  (M_k + L_k D) w = g_k
 *      M_k = E[conj(v) v^T], g_k = E[conj(v) d] over the 127 frames of one
 *      4096-sample capture, analysed with the SAME window and hop as the
 *      FPGA, averaged with the previous block (AJ_AVG) while the jammer is
 *      tracked. D = the relative noise of each tap (RX2 taps: rx2_noise).
 *      L_k (diagonal loading) is bin k's noise, from the canceller's own
 *      output: L = Pout / (1 + |w|^2), iterated to a fixed point per bin. A
 *      bin with no jammer in it ends up with w ~ 0 (its satellites untouched).
 *
 *  DETECTION (aj_detect, no weights): RX1/RX2 powers and the RX1-RX2
 *      coherence from a few time-domain lag sums. Run on every capture while
 *      there is no jammer; the full solve only runs once one is detected.
 *
 *  WEIGHT WORDS (gnss_passthrough):
 *      i =   0..63   (0x200 + 4i)          b_k,  k = i        (RX2, linear)
 *      i =  64..127  (0x300 + 4(i-64))     c1_k, k = i - 64   (image, RX1)
 *      i = 128..191  (0x400 + 4(i-128))    c2_k, k = i - 128  (image, RX2)
 *      k is the bin in natural FFT order (k >= 32: negative frequencies).
 *      [15:0] real, [31:16] imaginary, signed Q3.12 (1.0 = 4096).
 *****************************************************************************/
#ifndef GNSS_AJ_MATH_H_
#define GNSS_AJ_MATH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AJ_LOG2N      6
#define AJ_N         64     /* bins = FFT size; must match SFAP_N             */
#define AJ_HOP       32     /* frame hop (50 % overlap); must match SFAP_HOP  */
#define AJ_NW       192     /* weight words: 64 b + 64 c1 + 64 c2             */
#define AJ_WB         0     /* first b_k word                                 */
#define AJ_WC1       64     /* first c1_k word                                */
#define AJ_WC2      128     /* first c2_k word                                */
#define AJ_MAXDIM     3     /* adaptive weights per bin                       */
#define AJ_WFRAC     12     /* Q3.12                                          */
#define AJ_GFRAC     13     /* RX2 normalisation g2, Q3.13                    */
#define AJ_NSAMP   4096     /* samples per block (one DMA capture)            */
#define AJ_LAT      190     /* FPGA input -> output latency [samples]         */

/* Space-time names kept for code that still refers to them. */
#define AJ_NT        16     /* lag sums used by detection                     */

#define AJ_MODE_SFAP  0     /* per bin: b + 2 image taps (default)            */
#define AJ_MODE_LIN   1     /* per bin: b only, no image taps                 */
#define AJ_MODE_NB    2     /* one b for all bins: the narrowband v3 SMI-PI   */
#define AJ_NMODES     3

/* Weight of the previous block's statistics in the solve (0 = none). */
#define AJ_AVG       0.5

/* Lowest noise estimate allowed [LSB^2]: 12-bit quantisation of I and Q. */
#define AJ_QFLOOR    0.25

typedef struct {
    /* ---- set by the caller -------------------------------------------- */
    int     mode;           /* AJ_MODE_*                                      */
    double  sig2_prev;      /* last noise estimate, <= 0 if none              */
    double  rho_noise_re;   /* lag-1 correlation of the noise alone (learned  */
    double  rho_noise_im;   /*   while no jammer is present), 0 if unknown    */
    double  noise_ref;      /* receiver noise floor at the present gain       */
                            /*   [LSB^2], learned while no jammer, <= 0: none */
    double  g2;             /* RX2 normalisation the core applies (the Q3.13  */
                            /*   value written to AJ_NORM), <= 0 means 1      */
    double  rx2_noise;      /* normalised RX2 noise / RX1 noise: 1 when g2    */
                            /*   equalises them, less when g2 hit its limit;  */
                            /*   <= 0 means 1                                 */

    /* ---- results ------------------------------------------------------ */
    int16_t wre[AJ_NW], wim[AJ_NW];   /* weights to write, Q3.12             */
    double  p1;             /* RX1 power [LSB^2]                              */
    double  p2;             /* RX2 power [LSB^2], raw (before g2)             */
    double  coh;            /* RX1-RX2 coherence, best lag (0..1)             */
    int     coh_lag;        /* that lag [samples]                             */
    double  jn_coh_db;      /* J/N implied by the coherence [dB]              */
    double  pout;           /* output power of the weights that were running */
                            /* while this block was captured [LSB^2]         */
    double  pout_wn2;       /* mean |w|^2 of those weights (noise-weighted)   */
    double  sig2;           /* mean loading = canceller's noise estimate      */
    double  wn2;            /* mean |w|^2 of the new weights (noise-weighted) */
    double  noise;          /* noise used for the figures below [LSB^2]       */
    double  jn_db;          /* jammer-to-noise at RX1 [dB]                    */
    double  supp_db;        /* jammer suppression [dB] (capped at 60)         */
    double  res_db;         /* output / output noise [dB], 0 = fully nulled  */
    double  rho1_re, rho1_im;   /* RX1 lag-1 correlation of this block        */
    double  foff;           /* jammer centre, fraction of fs (-0.5..0.5)      */
    double  bw;             /* jammer bandwidth, fraction of fs (0..1)        */
    int     iters;          /* loading iterations used (largest bin)          */
    int     jam_bins;       /* bins whose J/N exceeds 3 dB                    */
    int     wmax_bin;       /* bin with the largest |b_k|                     */
    double  wmax;           /* that |b_k|                                     */
    double  bin_jn_db[AJ_N];/* J/N per bin [dB], natural order                */
    int     ok;             /* 1 = weights valid                              */
} aj_result_t;

/* Detection only (no weights): fills p1, p2, coh, coh_lag, jn_coh_db,
 * rho1_re/im and, with r->noise_ref > 0, noise and jn_db; ok = 1.
 * Returns 0 on success. */
int aj_detect(const uint32_t *buf, int nsamp, aj_result_t *r);

/* buf: one DMA capture, 2 words per sample instant:
 *      word 2n   = {Q1[31:16], I1[15:0]}   RX1 (AD9361 channel 0)
 *      word 2n+1 = {Q2[31:16], I2[15:0]}   RX2 (AD9361 channel 1)
 *      12-bit samples, right-aligned (only [11:0] is used).
 * nsamp must be AJ_NSAMP. Returns 0 on success. */
int aj_compute(const uint32_t *buf, int nsamp, aj_result_t *r);

/* J/N, suppression and residual from r->p1, pout, pout_wn2 and sig2, with
 * noise_ref as the noise floor. aj_compute() calls it with r->noise_ref. */
void aj_metrics(aj_result_t *r, double noise_ref);

/* Forget the previous weights and the averaged statistics (after the
 * filter was bypassed, the mode, g2 or the RX gain changed). */
void aj_reset(void);

/* Helpers (no libm on the board). */
double aj_sqrt(double x);
double aj_db10(double x);              /* 10*log10(x), -200 for x <= 0   */
double aj_atan2(double y, double x);   /* radians                        */

#ifdef __cplusplus
}
#endif

#endif
