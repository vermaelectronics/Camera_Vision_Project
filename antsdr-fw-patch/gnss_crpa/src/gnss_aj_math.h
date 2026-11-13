/******************************************************************************
 *  gnss_aj_math.h
 *  Space-frequency SMI power inversion: the weight computation (SFAP-PI v6.1,
 *  256 bins; the FPGA core is unchanged from v6.0). antsdr-fw-patch crpa15: the
 *  user's SFAP_PI_256v3 (v6.2.1) solver, used by the Linux firmware (gnss_crpa)
 *  of design A; only the two engine helpers at the end were added.
 *
 *  Pure C, no hardware access, no libm. The same file is compiled into the
 *  firmware (gnss_aj.c calls it once per captured block) and into the HLS
 *  C testbench (Source/HLS/sfap_pi/testbench.cpp), so the weights the core is
 *  tested with are exactly the weights the board computes.
 *
 *  FILTER (in the FPGA, sfap_pi_0):
 *      x2n = g2 x2   (RX2 normalised to RX1 before the FFT)
 *      frames of AJ_N = 256 samples, hop 128, window sin(pi n / 256) on analysis
 *      and synthesis (weighted overlap-add). Per frame and bin k:
 *        Y_k = X1_k - b_k X2n_k - c1_k conj(X1_{-k}) - c2_k conj(X2n_{-k})
 *      The RX1 coefficient is fixed at 1 (power inversion).
 *
 *  WEIGHTS (here), independently in every bin k:
 *      v = [X2n_k, conj(X1_{-k}), conj(X2n_{-k})],  d = X1_k
 *      minimise E|d - w^T v|^2  ->  (M_k + L_k D) w = g_k
 *      M_k = E[conj(v) v^T], g_k = E[conj(v) d] over the frames of one
 *      AJ_NSAMP-sample capture (v6.2: 4096 samples, 16 frames without
 *      overlap), analysed with the SAME window as the FPGA, averaged with
 *      the previous blocks while the jammer is tracked (weight
 *      aj_result_t.avg, set by the engine). D = the relative noise of each tap (RX2 taps: rx2_noise).
 *      L_k (diagonal loading) is bin k's noise, from the canceller's own
 *      output: L = Pout / (1 + |w|^2), iterated to a fixed point per bin. A
 *      bin with no jammer in it ends up with w ~ 0 (its satellites untouched).
 *      Bin gating: a bin under AJ_GATE_DB J/N (against the noise floor of
 *      THIS block -- v6.1 fix) gets zero weight...
 *      WEIGHT HOLD (v6.1): ...unless the bin held a strong jammer within the
 *      last AJ_HOLD_SOLVES solves: then the b_k solved at that time runs
 *      (checked on this block: it must leave no more than AJ_HOLD_RES x the
 *      noise). Pulsed, hopping and sweeping jammers stay nulled in every bin
 *      they visit; a new jammer from another direction gets fresh weights.
 *
 *  DETECTION (aj_detect, no weights): RX1/RX2 powers and the RX1-RX2
 *      coherence from a few time-domain lag sums, and (v6.1) RX1 per-bin
 *      powers against a learned per-bin noise floor (bin_jn_max): a CW ~10 dB
 *      under the total noise is seen. Run on every capture while there is no
 *      jammer; the full solve only runs once one is detected.
 *
 *  WEIGHT WORDS (gnss_passthrough):
 *      i =   0..255  (0x1000 + 4i)         b_k,  k = i        (RX2, linear)
 *      i = 256..511  (0x1400 + 4(i-256))   c1_k, k = i - 256  (image, RX1)
 *      i = 512..767  (0x1800 + 4(i-512))   c2_k, k = i - 512  (image, RX2)
 *      k is the bin in natural FFT order (k >= 128: negative frequencies).
 *      [15:0] real, [31:16] imaginary, signed Q3.12 (1.0 = 4096).
 *****************************************************************************/
#ifndef GNSS_AJ_MATH_H_
#define GNSS_AJ_MATH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AJ_LOG2N      8
#define AJ_N        256     /* bins = FFT size; must match SFAP_N             */
#define AJ_HOP      128     /* frame hop (50 % overlap); must match SFAP_HOP  */
#define AJ_NW       768     /* weight words: 256 b + 256 c1 + 256 c2          */
#define AJ_WB         0     /* first b_k word                                 */
#define AJ_WC1      256     /* first c1_k word                                */
#define AJ_WC2      512     /* first c2_k word                                */
#define AJ_MAXDIM     3     /* adaptive weights per bin                       */
#define AJ_WFRAC     12     /* Q3.12                                          */
#define AJ_GFRAC     13     /* RX2 normalisation g2, Q3.13                    */
#ifndef AJ_NSAMP
#define AJ_NSAMP   4096     /* samples per block (one DMA capture). v6.2:     */
#endif                      /*   4096 (v6.0/6.1: 8192). With AJ_SHOP = AJ_N   */
#ifndef AJ_SHOP             /*   a solve costs ~1/3 of v6.1's: ~3x more weight */
#define AJ_SHOP     256     /*   updates per second, so a moving channel       */
#endif                      /*   (obstacle, reflector) is tracked ~3x closer.  */
                            /* AJ_SHOP: solver frame spacing, AJ_HOP (50 %     */
                            /*   overlap, as the FPGA) or AJ_N (no overlap).   */
#define AJ_LAT      766     /* FPGA input -> output latency [samples], 3N-2   */

/* Space-time names kept for code that still refers to them. */
#define AJ_NT        16     /* lag sums used by detection                     */

#define AJ_MODE_SFAP  0     /* per bin: b + 2 image taps (default)            */
#define AJ_MODE_LIN   1     /* per bin: b only, no image taps                 */
#define AJ_MODE_NB    2     /* one b for all bins: the narrowband v3 SMI-PI   */
#define AJ_NMODES     3

/* Weight of the previous block's statistics in the solve (0 = none).
 * Default when the caller leaves aj_result_t.avg at 0; v6.1 engine adapts it
 * (AJ_AVG_STEADY while nulled, AJ_AVG_FAST while the residual is high). */
#define AJ_AVG       0.5

/* v6.1 weight hold: a bin whose J/N reached AJ_HOLD_DB keeps its last solved
 * weights for AJ_HOLD_SOLVES solves after its J/N fell below AJ_GATE_DB,
 * instead of dropping to zero. A jammer from one direction needs the same
 * b_k whenever it is in bin k, so pulsed, hopping and sweeping jammers stay
 * nulled in the bins they visit between two captures. 0 disables it. */
#ifndef AJ_HOLD_SOLVES
#define AJ_HOLD_SOLVES 100  /* ~1 s at ~100 solves/s (v6.2 rate; v6.1: 40)     */
#endif
#define AJ_HOLD_DB       6.0  /* store weights solved at this bin J/N or more */
#define AJ_HOLD_STORE_DB 1.5  /* ...and within this of the held level        */
#define AJ_HOLD_KEEP_DB  3.0  /* run held weights while their level is this  */
                              /*   far above the bin's present J/N           */
#define AJ_HOLD_DECAY_DB 0.5  /* held level decays per solve [dB]            */
#define AJ_HOLD_RES      2.0  /* held weights must leave <= 2x the noise     */

/* v6.1 per-bin detection: RX1 bins against a learned per-bin noise floor,
 * so a narrowband jammer far below the broadband thresholds is seen. The
 * bins next to DC (LO leakage) are left out of the maximum. */
#define AJ_DC_GUARD    1    /* bins 0, +-1 not used for detection              */

/* Bin gating (SFAP modes): a bin whose J/N is below this [dB] gets zero
 * weights, i.e. passes RX1 untouched (needs the learned noise floor). */
#ifndef AJ_GATE_DB
#define AJ_GATE_DB   0.0
#endif

/* Lowest noise estimate allowed [LSB^2]: 12-bit quantisation of I and Q. */
#define AJ_QFLOOR    0.25

/* v6.2 engine policies (gnss_aj.c), kept here so the C-sim testbench runs
 * the SAME code as the firmware (v6.2.1: aj_g2_follow, aj_avg_policy).
 *   g2 range (AJ_NORM is Q3.13) */
#define AJ_G2_MAX         (32767.0 / 8192.0)
#define AJ_G2_MIN         0.125
/*   drift-aware averaging: this capture only when the residual rises by
 *   AJ_AVG_RISE_DB or is above AJ_AVG_TRACK_DB, else AJ_AVG */
#define AJ_AVG_RISE_DB    0.5
#define AJ_AVG_TRACK_DB   1.0
/*   g2 follows a jammer that RX2 sees weaker (obstacle at RX2): only while
 *   J/N >= AJ_G2J_JN_DB, g2 x RX2 peak <= AJ_G2J_PEAK (core input saturates
 *   at ~4094), changes only by >= AJ_G2J_HYST_DB */
#define AJ_G2J_JN_DB      10.0
#define AJ_G2J_HYST_DB    3.0
#define AJ_G2J_PEAK       3800.0

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
    double  avg;            /* v6.1: weight of the old statistics, 0..0.95;   */
                            /*   0 = AJ_AVG, < 0 = no averaging               */
    double  gain_lin;       /* v6.1: RX gain, linear power, relative to the   */
                            /*   gain the bin floor is normalised to (the     */
                            /*   engine passes 10^(gain_dB/10)); <= 0: the    */
                            /*   per-bin floor is not used                    */

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
    double  bin_jn_max;     /* v6.1: largest per-bin J/N [dB] (DC guard out), */
                            /*   -99 when no bin floor is known               */
    int     bin_jn_kmax;    /* its bin                                        */
    int     held_bins;      /* v6.1: bins running held weights                */
    int     pk2;            /* v6.2: largest |I| or |Q| of RX2 in the capture */
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

/* Forget the previous weights, the averaged statistics and the held
 * weights (filter bypassed, mode or g2 changed). */
void aj_reset(void);

/* v6.1: forget the averaged statistics only; held weights stay (the RX gain
 * stepped: both channels step together, the null direction is unchanged). */
void aj_reset_stats(void);

/* v6.1: learn the per-bin noise floor from the LAST aj_detect() block (call
 * only when the engine has decided the block is noise: no jammer, RX gain at
 * its clean reference). gain_lin as in aj_result_t; dt_s = seconds since the
 * last call. Follows the floor down quickly and up only at rise_db_s. */
void aj_learn_bins(double gain_lin, double dt_s, double rise_db_s);
void aj_forget_bins(void);           /* the RF profile changed            */

/* v6.1: weight hold length in solves (default AJ_HOLD_SOLVES, 0 = off,
 * i.e. the v6.0 behaviour). */
void aj_set_hold(int solves);
int  aj_get_hold(void);
int  aj_bins_known(void);            /* 1 once the floor has been learned */

/* v6.2.1: the engine's per-solve policies, shared with the C-sim.
 * aj_avg_policy: the aj_result_t.avg value for the next solve (0 = AJ_AVG,
 *   -1 = this capture only), from the solves in this episode and the last
 *   two residuals.
 * aj_g2_follow: from the last solve's powers (r->jn_db, p1, p2, pk2), the g2
 *   for a jammer that RX2 sees weaker. g2_floor = the noise-floor g2 (never
 *   goes below it). Returns 1 and sets *g2_out (Q3.13-rounded) if g2 should
 *   change from g2_now, else 0. */
double aj_avg_policy(int n_ep, double res_db, double res_prev_db);
int    aj_g2_follow(const aj_result_t *r, double g2_now, double g2_floor, double *g2_out);

/* ---- antsdr-fw-patch crpa15 (design A, Linux): helpers of the engine ----------------------- */
#define AJ_MAXSH      4     /* output make-up shift, 6 dB per step (SFAP_MAXSH of the core) */
#define AJ_OUT_HEADROOM_DB  3.0   /* output may be this far above the clean level      */
/* Output make-up shift for an RX gain cut of cut_db and a filter output out_db above the
 * noise: restores the clean output level, 6 dB per step, never more than
 * AJ_OUT_HEADROOM_DB above it. */
int    aj_makeup_shift(int cut_db, double out_db, int max_shift);
/* Expected excess of the receiver's noise floor from the 12-bit ADC's own quantisation
 * noise when a jammer jn_db above the noise has to be kept off full scale. */
double aj_adc_excess_db(double jn_db);

/* Helpers (no libm on the board). */
double aj_sqrt(double x);
double aj_db10(double x);              /* 10*log10(x), -200 for x <= 0   */
double aj_atan2(double y, double x);   /* radians                        */

#ifdef __cplusplus
}
#endif

#endif
