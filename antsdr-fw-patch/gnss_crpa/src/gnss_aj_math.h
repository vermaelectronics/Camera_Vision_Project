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

#ifndef AJ_NT
#define AJ_NT       10    /* crpa7: 10 taps per antenna (10 + 10 = 20 taps); must match SMI_NT (crpa4/5: 8, crpa3: 7, crpa1/2: 6) */
#endif
/* AJ_NT: taps per channel, delays 0..31 (4.9; 16 before) */
#ifndef AJ_REF
#define AJ_REF        5     /* crpa7: 5; must match SMI_REF (crpa3-5: 4, 4.9.12: 2). RX1 reference delay (4.8: was 8; echoes up to 11 samples can be cancelled) */
#endif
#define AJ_NSPT      (2 * AJ_NT)       /* space-time weight words: RX1 AJ_NT, then RX2 AJ_NT */
#define AJ_IMG1      (2 * AJ_NT)       /* image tap c1: conj(x1[n-REF])                  */
#define AJ_IMG2      (2 * AJ_NT + 1)   /* image tap c2: conj(x2n[n-REF])                 */
#define AJ_NW        (2 * AJ_NT + 2)   /* weight words                                   */
#define AJ_MAXDIM    (2 * AJ_NT + 1)   /* adaptive weights                               */
#define AJ_WFRAC     12     /* Q3.12                                          */
#define AJ_GFRAC     13     /* RX2 normalisation g2, Q3.13                    */
#define AJ_MAXWS      5     /* weight scale: words hold w * 2^ws (SMI-PI 4.4)  */
#define AJ_MAXSH      7     /* output make-up shift, 6 dB per step (4.5; the 4.1-4.4 core stops at 4) */
#define AJ_OUT_HEADROOM_DB  3.0   /* output may be this far above the clean level      */
#ifndef AJ_NSAMP
#define AJ_NSAMP   8192     /* samples per block (one DMA capture). 4.9.4: 8192 (was 4096); 16384 broke the chirp detector and 2 tests */
#endif

#define AJ_MODE_STAP  0     /* RX1 NT-1 + RX2 NT + 2 image taps (default)     */
#define AJ_MODE_RX2   1     /* RX2 NT + 2 image taps                          */
#define AJ_MODE_NB    2     /* one weight, RX2 at delay 8: the old SMI-PI     */
#define AJ_MODE_LIN   3     /* RX1 NT-1 + RX2 NT, no image taps (v4.0)        */
#define AJ_NMODES     4

/* Weight of the previous block's statistics in the solve (0 = none). */
#ifndef AJ_AVG
#define AJ_AVG       0.5
#endif

/* Diagonal loading alpha, in units of the thermal noise power sigma^2 (4.9.2).
 * RX1's own taps (the reflection taps) get more than the RX2 taps (see gnss_aj_math.c).
 * With a learned noise floor alpha is held at AJ_RX2_LOAD .. AJ_RX1_LOAD x sigma^2 whatever the jammer
 * residual does ("alpha 2 to 5 sigma^2, clamped just above the thermal noise"). */
#ifndef AJ_RX1_LOAD
#define AJ_RX1_LOAD  5.0
#endif
#ifndef AJ_RX2_LOAD
#define AJ_RX2_LOAD  2.0
#endif
/* With a learned floor the canceller's noise estimate is held within this factor of it (linear). */
#ifndef AJ_SIG2_BAND
#define AJ_SIG2_BAND 1.25
#endif

/* Adaptive averaging (4.9.9): AJ_AVG is adjusted from the solver's own results instead of being fixed.
 *   AJ_AV_ADAPT  1 = adapt (default), 0 = fixed at AJ_AVG
 *   AJ_AV_MIN/MAX  limits of the factor (more = more memory, smoother; less = faster, noisier)
 *   AJ_AV_STEP   change per solve
 *   AJ_JIT_HI    relative change of the weights between two solves above which they count as jittering
 *   AJ_RES_TRACK residual [dB] above which, with stable weights, the filter is not following the jammer */
#ifndef AJ_AV_ADAPT
#define AJ_AV_ADAPT  1
#endif
#ifndef AJ_AV_MIN
#define AJ_AV_MIN    0.30
#endif
#ifndef AJ_AV_MAX
#define AJ_AV_MAX    0.90
#endif
#ifndef AJ_AV_STEP
#define AJ_AV_STEP   0.05
#endif
#ifndef AJ_JIT_HI
#define AJ_JIT_HI    0.15
#endif
#ifndef AJ_RES_TRACK
#define AJ_RES_TRACK 6.0
#endif

/* 4.9.9: the averaging factor set from the console (gnss_aj_avg=): a value turns the adaptation off. */
void   aj_set_av(double v);
void   aj_set_av_auto(void);
int    aj_av_is_auto(void);
double aj_av_value(void);

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
    double  avg;            /* weight of the past statistics in this solve,   */
                            /*   0..0.995; <= 0 means AJ_AVG (0.5). A long    */
                            /*   memory (0.98 = about 50 blocks) makes a slow */
                            /*   chirp look like a wideband jammer (4.6)      */
    int     gate;           /* 1 = skip a block that holds no jammer (pulsed   */
                            /*   jammer: its gaps must not become weights);    */
                            /*   aj_compute() then returns 1, keeps the old    */
                            /*   weights and statistics, and fills p1, p2, coh,  */
                            /*   jn_coh_db, jn_db (4.7)                         */
    double  gate_coh_db;    /* ... coherence below this [dB]                   */
    double  gate_pwr;       /* ... and RX1 power below noise_ref x this (linear) */
    int     gated;          /* result: this block was skipped                  */
    double  rx2_noise;      /* normalised RX2 noise / RX1 noise: 1 when g2    */
                            /*   equalises them, less when g2 hit its limit;  */
                            /*   <= 0 means 1                                 */

    /* ---- results ------------------------------------------------------ */
    int16_t wre[AJ_NW], wim[AJ_NW];   /* weights to write: w * 2^ws, Q3.12  */
    int     ws;             /* weight scale for AJ_CFG[14:12], 0..AJ_MAXWS    */
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

/* ---- Slow chirp / sweeping jammer (SMI-PI 4.6) -----------------------------
 * A jammer that is narrow inside one 4096-sample block but whose centre moves
 * from solve to solve (a chirp that takes longer than ~1 ms per sweep) gives
 * a rank-1 statistic that is good at ONE frequency only (4 dB left after a
 * step of 5 MHz). If the statistics are averaged over a long memory the
 * jammer fills its whole sweep range and looks like a wideband jammer, and
 * the same 33 weights null it at every frequency. aj_sweep_update() watches
 * the jammer's centre frequency from solve to solve and says when to use the
 * long memory. Fast chirps (wide inside one block) and steady jammers are not
 * touched. */
#define AJ_SWEEP_AVG    0.98   /* memory while a sweep is seen (about 50 blocks)  */
#define AJ_SWEEP_STEP   0.004  /* centre moved by more than this (x fs) = moving */
#define AJ_SWEEP_BW     0.2    /* only for a jammer narrower than this (x fs)    */
#define AJ_SWEEP_ON     10     /* score to switch the long memory on             */
#define AJ_SWEEP_MAX    30     /* score ceiling = solves of quiet before off     */

typedef struct { double last; int have; int score; int on; } aj_sweep_t;

void aj_sweep_reset(aj_sweep_t *s);
/* Call after every solve while jammed. Returns the r->avg to use for the next
 * solve: AJ_SWEEP_AVG while a sweep is seen, 0 (= AJ_AVG) otherwise. */
double aj_sweep_update(aj_sweep_t *s, const aj_result_t *r);

/* DC offset of RX1 / RX2 [LSB] in the last capture; removed before the
 * statistics, the detection and the weights. */
void aj_last_dc(double *dc1, double *dc2);

/* Detection only (no weights): fills p1, p2, coh, coh_lag, jn_coh_db,
 * rho1_re/im and, with r->noise_ref > 0, noise and jn_db; ok = 1.
 * About a third of the work of aj_compute(). Returns 0 on success. */
int aj_detect(const uint32_t *buf, int nsamp, aj_result_t *r);

/* Returns 0 on success, 1 when r->gate skipped a jammer-free block (r->gated = 1), < 0 on error. */
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
/* Output make-up shift for an RX gain cut of cut_db and a filter output out_db above the
 * noise (at the present gain, 0 = nothing but noise): restores the clean output level,
 * 6 dB per step, but never leaves the output more than AJ_OUT_HEADROOM_DB above it. */
int  aj_makeup_shift(int cut_db, double out_db, int max_shift);
/* crpa8: the rest of the make-up in dB (0 .. 5.99) on top of 6 x shift, by the same rule:
 * restore the RX gain cut, never more than AJ_OUT_HEADROOM_DB above the clean level. */
double aj_makeup_fine_db(int cut_db, double out_db, int shift);
/* Expected excess of the receiver's noise floor, in dB, from the 12-bit ADC's own
 * quantisation noise when a jammer jn_db above the noise has to be kept off full scale
 * (about 0 below J/N 54, +1.8 dB at 61, +4 dB at 66, +8 dB at 71). */
double aj_adc_excess_db(double jn_db);
void aj_set_max_ws(int ws);   /* largest weight scale the bitstream supports; 0 = older than 4.4 */

/* Helpers (no libm on the board). */
double aj_sqrt(double x);
double aj_db10(double x);              /* 10*log10(x), -200 for x <= 0   */
double aj_atan2(double y, double x);   /* radians                        */

#ifdef __cplusplus
}
#endif

#endif
