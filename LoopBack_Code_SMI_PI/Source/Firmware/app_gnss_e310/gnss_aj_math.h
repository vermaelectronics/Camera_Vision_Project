/******************************************************************************
 *  gnss_aj_math.h
 *  Space-time SMI power inversion: the weight computation (SMI-PI v4.1).
 *
 *  Pure C, no hardware access, no libm. The same file is compiled into the
 *  firmware (gnss_aj.c calls it once per captured block) and into the HLS
 *  C testbench (Source/HLS/smi_pi/testbench.cpp), so the weights the core is
 *  tested with are exactly the weights the board computes.
 *
 *  FILTER (in the FPGA, smi_pi_0):
 *      x2n = g2 x2   (RX2 normalised to RX1 at the core's input)
 *      y[n] = x1[n-8] - sum_{k != 8} a_k x1[n-k] - sum_k b_k x2n[n-k]
 *                     - c1 conj(x1[n-8]) - c2 conj(x2n[n-8])
 *      k = 0..15. 31 space-time weights + 2 image (widely-linear) weights.
 *      The RX1 tap at delay 8 is the reference and is fixed at 1 (power
 *      inversion: w1 = 1).
 *
 *  WEIGHTS (here):
 *      minimise E|y|^2  ->  (M + L D) w = g
 *      u = the active tap samples, d = x1[n-8], M = E[conj(u) u^T],
 *      g = E[conj(u) d], D = diag(noise of each tap relative to RX1's, x4
 *      for RX1's own taps).
 *      M and g are the EXACT sample statistics of one contiguous block of
 *      AJ_NSAMP samples, built from 63 lag sums plus edge corrections (a
 *      plain Toeplitz matrix from lag sums leaves ~K/N relative errors that
 *      cost 20+ dB of null at a 40 dB jammer), averaged with the previous
 *      block (AJ_AVG) while the jammer is tracked.
 *      L (diagonal loading) is the noise power, estimated from the output
 *      of the canceller itself: L = Pout / (1 + |w|^2), iterated to a fixed
 *      point. No manual setting.
 *
 *  DETECTION (aj_detect, no weights): RX1/RX2 powers and the RX1-RX2
 *      coherence, a few lag sums only. Run on every capture while there is
 *      no jammer; the full solve only runs once one is detected.
 *
 *  WEIGHT WORDS (gnss_passthrough):
 *      k = 0..15  (0x100 + 4k)  RX1 tap at delay k (word 8 is ignored: the
 *                               reference)
 *      k = 16..31 (0x100 + 4k)  RX2 tap at delay k-16
 *      k = 32     (0x190)       image tap c1, RX1
 *      k = 33     (0x194)       image tap c2, RX2
 *      [15:0] real, [31:16] imaginary, signed Q3.12 (1.0 = 4096).
 *****************************************************************************/
#ifndef GNSS_AJ_MATH_H_
#define GNSS_AJ_MATH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AJ_NT        16     /* taps per channel, delays 0..15                 */
#define AJ_REF        8     /* RX1 reference delay                            */
#define AJ_NSPT      32     /* space-time weight words (RX1 16 + RX2 16)      */
#define AJ_IMG1      32     /* image tap c1: conj(x1[n-8])                    */
#define AJ_IMG2      33     /* image tap c2: conj(x2n[n-8])                   */
#define AJ_NW        34     /* weight words                                   */
#define AJ_MAXDIM    33     /* adaptive weights                               */
#define AJ_WFRAC     12     /* Q3.12                                          */
#define AJ_GFRAC     13     /* RX2 normalisation g2, Q3.13                    */
#define AJ_NSAMP   4096     /* samples per block (one DMA capture)            */

#define AJ_MODE_STAP  0     /* RX1 15 + RX2 16 + 2 image taps (default)       */
#define AJ_MODE_RX2   1     /* RX2 16 + 2 image taps                          */
#define AJ_MODE_NB    2     /* one weight, RX2 at delay 8: the old SMI-PI     */
#define AJ_MODE_LIN   3     /* RX1 15 + RX2 16, no image taps (v4.0)          */
#define AJ_NMODES     4

/* Weight of the previous block's statistics in the solve (0 = none). */
#define AJ_AVG       0.5

/* Extra diagonal loading on RX1's own taps (see gnss_aj_math.c). */
#define AJ_RX1_LOAD  4.0

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
    double  jn_coh_db;      /* J/N implied by the coherence [dB]: a jammer    */
                            /*   both antennas see; noise alone gives ~-15    */
    double  pout;           /* output power of the weights that were running */
                            /* while this block was captured [LSB^2]         */
    double  pout_wn2;       /* |w|^2 of those weights (noise-weighted)        */
    double  sig2;           /* loading used = canceller's noise estimate      */
    double  wn2;            /* |w|^2 of the new weights, each tap weighted by */
                            /*   its noise relative to RX1 (rx2_noise)        */
    double  noise;          /* noise used for the figures below [LSB^2]       */
    double  jn_db;          /* jammer-to-noise at RX1 [dB]                    */
    double  supp_db;        /* jammer suppression [dB] (capped at 60)         */
    double  res_db;         /* output / output noise [dB], 0 = fully nulled  */
    double  rho1_re, rho1_im;   /* RX1 lag-1 correlation of this block        */
    double  foff;           /* jammer centre, fraction of fs (-0.5..0.5)      */
    double  bw;             /* jammer bandwidth, fraction of fs (0..1)        */
    int     iters;          /* loading iterations used                        */
    int     ok;             /* 1 = weights valid                              */
} aj_result_t;

/* DC offset of RX1 / RX2 [LSB] in the last capture; removed before the
 * statistics, the detection and the weights. */
void aj_last_dc(double *dc1, double *dc2);

/* Detection only (no weights): fills p1, p2, coh, coh_lag, jn_coh_db,
 * rho1_re/im and, with r->noise_ref > 0, noise and jn_db; ok = 1.
 * About a third of the work of aj_compute(). Returns 0 on success. */
int aj_detect(const uint32_t *buf, int nsamp, aj_result_t *r);

/* buf: one DMA capture, 2 words per sample instant:
 *      word 2n   = {Q1[31:16], I1[15:0]}   RX1 (AD9361 channel 0)
 *      word 2n+1 = {Q2[31:16], I2[15:0]}   RX2 (AD9361 channel 1)
 *      12-bit samples, right-aligned (only [11:0] is used).
 * nsamp must be AJ_NSAMP. Returns 0 on success. */
int aj_compute(const uint32_t *buf, int nsamp, aj_result_t *r);

/* J/N, suppression and residual from r->p1, pout, pout_wn2 and sig2, with
 * noise_ref as the noise floor (see gnss_aj_math.c). aj_compute() calls it
 * with r->noise_ref; call again to re-evaluate with another floor. */
void aj_metrics(aj_result_t *r, double noise_ref);

/* Forget the previous weights and the averaged statistics (after the
 * filter was bypassed, the mode, g2 or the RX gain changed), so the next
 * solve and its suppression figure start from this block alone. */
void aj_reset(void);

/* Helpers (no libm on the board). */
double aj_sqrt(double x);
double aj_db10(double x);              /* 10*log10(x), -200 for x <= 0   */
double aj_atan2(double y, double x);   /* radians                        */

#ifdef __cplusplus
}
#endif

#endif
