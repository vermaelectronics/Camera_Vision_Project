/******************************************************************************
 *  gnss_aj_math.h
 *  crpa13: jammer detection, NLMS step and output make-up rules for the
 *  space-time power-inversion filter whose weights the FPGA adapts itself
 *  (nlms_pi, hardware NLMS). crpa1-crpa12 solved the weights here (SMI); that
 *  solver is gone.
 *
 *  Pure C, no hardware access, no libm. The same file is compiled into the
 *  firmware (gnss_aj.c) and into the HLS C testbench (hls/nlms_pi/
 *  testbench.cpp), so the step word and the make-up the core is tested with
 *  are exactly the board's.
 *
 *  FILTER (in the FPGA, nlms_pi_0):
 *      x2n = g2 x2   (RX2 normalised to RX1 at the core's input)
 *      y[n] = x1[n-R] - sum_{k != R} a_k x1[n-k] - sum_k b_k x2n[n-k]
 *                     - sum_d c1_d conj(x1[n-R-d]) - sum_d c2_d conj(x2n[n-R-d])
 *      k = 0..AJ_NT-1, d = -1, 0, 1, R = AJ_REF. The RX1 tap at delay R is the
 *      reference, fixed at 1 (power inversion). Every sample the core moves each
 *      weight by alpha y conj(u) / (nw P_in) (delayed NLMS, 64 samples).
 *
 *  WEIGHT WORDS (read back from gnss_passthrough, the core's weight monitor):
 *      k = 0..AJ_NT-1       (0x400 + 4k)  RX1 tap at delay k (word AJ_REF reads 0)
 *      k = AJ_NT..2AJ_NT-1  (0x400 + 4k)  RX2 tap at delay k-AJ_NT
 *      k = AJ_IMG(p, d)     (0x190, 0x194, 0x1C0..0x1CC) image taps
 *      [15:0] real, [31:16] imaginary, signed Q3.12 x 2^ws (1.0 = 4096 at ws 0).
 *
 *  DETECTION (aj_detect): RX1/RX2 powers, the RX1-RX2 coherence and RX1's
 *      lag-1 correlation (jammer bandwidth) from one capture.
 *****************************************************************************/

#ifndef GNSS_AJ_MATH_H_
#define GNSS_AJ_MATH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef AJ_NT
#define AJ_NT        8    /* taps per antenna (8 + 8); must match NL_NT of nlms_pi */
#endif
#ifndef AJ_REF
#define AJ_REF       4    /* RX1 reference delay; must match NL_REF */
#endif
#define AJ_NSPT      (2 * AJ_NT)       /* space-time weight words: RX1 AJ_NT, then RX2 AJ_NT */
/* AJ_NIMG image (conjugate) taps per channel at delays REF-1, REF, REF+1: the AD9361's I/Q
 * mirror changes across a wide band. */
#ifndef AJ_NIMG
#define AJ_NIMG       3     /* must match NL_NIMG of the core                  */
#endif
#define AJ_IMGH      ((AJ_NIMG - 1) / 2)
#define AJ_IMG0      (2 * AJ_NT)       /* first image tap: c1 at REF-H .. REF+H, then c2 */
#define AJ_IMG(p, d) (AJ_IMG0 + ((p) - 1) * AJ_NIMG + AJ_IMGH + (d))   /* channel p, delay REF+d */
#define AJ_IMG1      AJ_IMG(1, 0)      /* image tap c1 at REF: conj(x1[n-REF])           */
#define AJ_IMG2      AJ_IMG(2, 0)      /* image tap c2 at REF: conj(x2n[n-REF])          */
#define AJ_NW        (2 * AJ_NT + 2 * AJ_NIMG)       /* weight words                     */
#define AJ_MAXDIM    (2 * AJ_NT - 1 + 2 * AJ_NIMG)   /* adaptive weights                 */
#define AJ_WFRAC     12     /* Q3.12                                          */
#define AJ_GFRAC     13     /* RX2 normalisation g2, Q3.13                    */
#define AJ_MAXWS      5     /* weight scale: the core holds w * 2^ws           */
#define AJ_MAXSH      7     /* output make-up shift, 6 dB per step             */
#define AJ_OUT_HEADROOM_DB  3.0   /* output may be this far above the clean level      */
#ifndef AJ_NSAMP
#define AJ_NSAMP   8192     /* samples per capture (one DMA transfer)         */
#endif

/* Lowest noise estimate allowed [LSB^2]: 12-bit quantisation of I and Q. */
#define AJ_QFLOOR    0.25

typedef struct {
    /* ---- set by the caller -------------------------------------------- */
    double  rho_noise_re;   /* lag-1 correlation of the noise alone (learned  */
    double  rho_noise_im;   /*   while no jammer is present), 0 if unknown    */
    double  noise_ref;      /* receiver noise floor at the present gain       */
                            /*   [LSB^2], learned while no jammer, <= 0: none */
    /* ---- aj_detect ------------------------------------------------------ */
    double  p1;             /* RX1 power [LSB^2]                              */
    double  p2;             /* RX2 power [LSB^2], raw (before g2)             */
    double  coh;            /* RX1-RX2 coherence, best lag (0..1)             */
    int     coh_lag;        /* that lag [samples]                             */
    double  jn_coh_db;      /* J/N implied by the coherence [dB]: a jammer    */
                            /*   both antennas see; noise alone gives ~-15    */
    double  noise;          /* noise used for jn_db [LSB^2]                   */
    double  jn_db;          /* jammer-to-noise at RX1 [dB]                    */
    double  rho1_re, rho1_im;   /* RX1 lag-1 correlation of this capture      */
    double  foff;           /* jammer centre, fraction of fs (-0.5..0.5)      */
    double  bw;             /* jammer bandwidth, fraction of fs (0..1)        */
    int     ok;             /* 1 = valid                                      */
    /* ---- filled by gnss_aj.c from the FPGA (weights read back, meters) --- */
    int16_t wre[AJ_NW], wim[AJ_NW];   /* the running weights, w * 2^ws, Q3.12 */
    int     ws;             /* weight scale in use                            */
    double  wn2;            /* |w|^2 of the running weights, each tap weighted */
                            /*   by its noise relative to RX1                  */
    double  supp_db;        /* jammer suppression [dB] (capped at 60)         */
    double  res_db;         /* output / output noise [dB], 0 = fully nulled  */
} aj_result_t;

/* DC offset of RX1 / RX2 [LSB] in the last capture; removed before the
 * statistics, the detection and the weights. */
void aj_last_dc(double *dc1, double *dc2);

/* Detection only (no weights): fills p1, p2, coh, coh_lag, jn_coh_db,
 * rho1_re/im and, with r->noise_ref > 0, noise and jn_db; ok = 1.
 * Returns 0 on success. */
int aj_detect(const uint32_t *buf, int nsamp, aj_result_t *r);

/* Output make-up shift for an RX gain cut of cut_db and a filter output out_db above the
 * noise (at the present gain, 0 = nothing but noise): restores the clean output level,
 * 6 dB per step, but never leaves the output more than AJ_OUT_HEADROOM_DB above it. */
int  aj_makeup_shift(int cut_db, double out_db, int max_shift);
/* crpa8: the rest of the make-up in dB (0 .. 5.99) on top of 6 x shift, by the same rule:
 * restore the RX gain cut, never more than AJ_OUT_HEADROOM_DB above the clean level. */
double aj_makeup_fine_db(int cut_db, double out_db, int shift);
/* crpa10: hardware NLMS step. The core moves each weight word (x 2^ws) by
 * (y m 2^-sh) conj(u) 2^-AJ_NLMS_CS in units of 2^-AJ_NLMS_RF word LSB, u in Q1; this
 * returns the nlms register word ([15:0] m, [20:16] sh, [31] enable) for a normalised step
 * alpha: dw = alpha y conj(u) / (nw p_in), p_in = mean |u|^2 [LSB^2] (the RX1 power),
 * nw = the number of adaptive weights. 0 when off (alpha <= 0 or no power). */
#define AJ_NLMS_RF   16     /* must match NL_RF of nlms_pi */
#define AJ_NLMS_CS    8     /* must match NL_CS of nlms_pi */
uint32_t aj_nlms_word(double alpha, double p_in, int ws, int nw);
/* crpa13: the NLMS step alpha for a jammer whose RX1 lag-1 correlation magnitude is rho
 * (aj_detect: |rho1|; about 1 for a CW or a narrow jammer, 0.2-0.5 for a wideband one).
 * The delayed loop (D = 64 samples) is stable up to about alpha 1/48 for a CW (all its
 * power in one eigenmode: C simulation, 1/32 diverged); a wideband jammer spreads its
 * power over the taps, so it takes a 4 x larger step, and converges best near 1/16
 * (C simulation, J/N 50: 1/128 38 dB, 1/32 46 dB, 1/16 48 dB, 1/8 48 dB, 1/4 diverged,
 * after 10 ms). rho >= AJ_NLMS_RHO_NB: AJ_NLMS_ALPHA_NB; rho <= AJ_NLMS_RHO_WB:
 * AJ_NLMS_ALPHA_WB; geometric in between. */
#define AJ_NLMS_ALPHA_NB  (1.0 / 64.0)
#define AJ_NLMS_ALPHA_WB  (1.0 / 16.0)
#define AJ_NLMS_RHO_NB    0.95
#define AJ_NLMS_RHO_WB    0.60
double aj_nlms_alpha(double rho);
/* Expected excess of the receiver's noise floor, in dB, from the 12-bit ADC's own
 * quantisation noise when a jammer jn_db above the noise has to be kept off full scale
 * (about 0 below J/N 54, +1.8 dB at 61, +4 dB at 66, +8 dB at 71). */
double aj_adc_excess_db(double jn_db);

/* Helpers (no libm on the board). */
double aj_sqrt(double x);
double aj_db10(double x);              /* 10*log10(x), -200 for x <= 0   */
double aj_atan2(double y, double x);   /* radians                        */

#ifdef __cplusplus
}
#endif

#endif
