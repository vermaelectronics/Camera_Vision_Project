/******************************************************************************
 *  gnss_aj_math.h
 *  Space-frequency SMI power inversion: the weight computation (SFAP-PI v5.0).
 *
 *  antsdr-fw-patch crpa6: the SFAP-PI v5.0 solver (LoopBack SFAP_PassLVDS,
 *  Source/Firmware/app_sfap_lvds) behind the interface of the SMI-PI 4.9.14
 *  firmware (gnss_aj.c): averaging factor (r->avg, aj_set_av), pulse gate
 *  (r->gate), DC removal (aj_last_dc), sweep detector, output make-up rule.
 *  AJ_LOG2N selects the FFT size: 7 = 128 bins (default), 6 = 64 bins
 *  (v5.0). It must match SFAP_LOG2N of the FPGA core and AJ_LOG2N of
 *  gnss_passthrough.v; the firmware checks AJ_INFO at start-up.
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

#include "gnss_sfap_cfg.h"  /* AJ_LOG2N: 7 = 128 bins (default), 6 = 64 bins   */
#define AJ_N        (1 << AJ_LOG2N)   /* bins = FFT size; must match SFAP_N   */
#define AJ_HOP      (AJ_N / 2)  /* frame hop (50 % overlap); must match SFAP_HOP */
#define AJ_NW       (3 * AJ_N)  /* weight words: N b + N c1 + N c2            */
#define AJ_WB         0     /* first b_k word                                 */
#define AJ_WC1      AJ_N    /* first c1_k word                                */
#define AJ_WC2      (2 * AJ_N)  /* first c2_k word                            */
#define AJ_MAXDIM     3     /* adaptive weights per bin                       */
#define AJ_WFRAC     12     /* Q3.12                                          */
#define AJ_GFRAC     13     /* RX2 normalisation g2, Q3.13                    */
#ifndef AJ_NSAMP
#define AJ_NSAMP   4096     /* samples per block (one DMA capture)            */
#endif
#define AJ_LAT      (3 * AJ_N - 2)    /* FPGA input -> output latency [samples] */

#define AJ_NLAG      16     /* lag sums used by detection                     */

#define AJ_MODE_SFAP  0     /* per bin: b + 2 image taps (default)            */
#define AJ_MODE_LIN   1     /* per bin: b only, no image taps                 */
#define AJ_MODE_NB    2     /* one b for all bins: the narrowband v3 SMI-PI   */
#define AJ_NMODES     3
#define AJ_MODE_STAP  AJ_MODE_SFAP   /* the default mode, by its 4.9.14 name  */

#define AJ_MAXSH      4     /* largest output make-up shift of the core (x16) */
#define AJ_MAXWS      0     /* no weight scale in the SFAP core               */
#define AJ_OUT_HEADROOM_DB  3.0   /* output may be this far above the clean level */

/* Weight of the previous block's statistics in the solve (0 = none). */
#ifndef AJ_AVG
#define AJ_AVG       0.5
#endif
/* Adaptive averaging (as SMI-PI 4.9.9): AJ_AVG is adjusted from the solver's own
 * results unless a value is set by hand (aj_set_av) or by the caller (r->avg). */
#ifndef AJ_AV_ADAPT
#define AJ_AV_ADAPT  1
#endif
#define AJ_AV_MIN    0.30
#define AJ_AV_MAX    0.90
#define AJ_AV_STEP   0.05
#define AJ_JIT_HI    0.15   /* relative change of the weights between two solves = jittering */
#define AJ_RES_TRACK 6.0    /* residual [dB] above which, with stable weights, track faster  */
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
    double  rx2_noise;      /* normalised RX2 noise / RX1 noise: 1 when g2    */
                            /*   equalises them, less when g2 hit its limit;  */
                            /*   <= 0 means 1                                 */
    double  avg;            /* weight of the earlier blocks in this solve;    */
                            /*   <= 0: the adaptive / manual factor           */
    int     gate;           /* 1 = skip a block with no jammer in it (pulsed  */
                            /*   jammer): weights and statistics are kept     */
    double  gate_coh_db;    /*   "no jammer": coherence J/N below this ...    */
    double  gate_pwr;       /*   ... and RX1 power below noise_ref x this     */

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
    int     gated;          /* 1 = this block was skipped by the gate         */
    int     ws;             /* weight scale: always 0 (no such stage in SFAP) */
    int     ok;             /* 1 = weights valid                              */
} aj_result_t;

/* ---- slow chirp / sweeping jammer (as SMI-PI 4.6) ------------------------------
 * A narrow jammer whose centre moves from solve to solve: a long memory makes it
 * fill its sweep range in the statistics. With SFAP every bin has its own weight,
 * so the long memory simply lets every bin of the sweep range keep its null. */
#define AJ_SWEEP_AVG    0.98
#define AJ_SWEEP_STEP   0.004
#define AJ_SWEEP_BW     0.2
#define AJ_SWEEP_ON     10
#define AJ_SWEEP_MAX    30
typedef struct { double last; int have; int score; int on; } aj_sweep_t;
void   aj_sweep_reset(aj_sweep_t *s);
double aj_sweep_update(aj_sweep_t *s, const aj_result_t *r);

/* DC offset of RX1 / RX2 [LSB] in the last capture (removed before the statistics). */
void aj_last_dc(double *dc1, double *dc2);
/* Output make-up shift for an RX gain cut of cut_db and a filter output out_db above
 * the noise: 6 dB per step, never more than AJ_OUT_HEADROOM_DB above the clean level. */
int  aj_makeup_shift(int cut_db, double out_db, int max_shift);
/* Expected excess of the noise floor from the ADC's quantisation noise at J/N jn_db. */
double aj_adc_excess_db(double jn_db);
void aj_set_max_ws(int ws);   /* kept for the engine; SFAP has no weight scale */

/* Detection only (no weights): fills p1, p2, coh, coh_lag, jn_coh_db,
 * rho1_re/im and, with r->noise_ref > 0, noise and jn_db; ok = 1.
 * Returns 0 on success. */
int aj_detect(const uint32_t *buf, int nsamp, aj_result_t *r);

/* Returns 0 on success, 1 when r->gate skipped a jammer-free block (r->gated = 1), < 0 on error. */
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
