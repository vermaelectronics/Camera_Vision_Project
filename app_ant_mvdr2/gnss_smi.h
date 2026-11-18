/***************************************************************************//**
 *   @file   gnss_smi.h
 *   @brief  GNSS-CRPA MOD-25: SMI-MVDR STAP driver and solver (MVDR window 1.14)
 *
 * The FPGA (stap_smi.v) accumulates the N x N space-time covariance
 *     R = sum x x^H,   x = [RX1(n..n-L+1), RX2(n..n-L+1), conj RX1(n-REF), conj RX2(n-REF)]
 * N = 2L + 2. The layout is read from INFO at run time:
 *     INFO 0x02030853 : v2  L = 8,  REF = 3, N = 18  (MVDR 1.14,  covariance flat)
 *     INFO 0x03xxxx53 : v3  L, REF from INFO; MVDR 1.14b ships L = 16, REF = 8,
 *                       N = 34 (covariance read in pages of 128 entries)
 * so one firmware runs either bitstream.
 * over 2^KEXP snapshots and publishes it. This module reads it, solves the
 * loaded MVDR / LCMV weights in double precision
 *     w = (R + lambda I)^-1 a / (a^H (R + lambda I)^-1 a)
 *     lambda = alpha * lambda_min(R)          (robust diagonal loading)
 * and writes them back; the FPGA filters y = w^H x.
 *   a = e_REF               : SMI power inversion (default). Unity gain on
 *                             RX1 tap REF, everything else minimised: every
 *                             satellite passes, every jammer is nulled.
 *   a = e_REF + e^{j phi} e_{L+REF} : steered (gnss_steer=). Unity gain AND
 *                             +3 dB array gain toward ONE direction (phi =
 *                             RX2-RX1 phase of that direction, from the
 *                             jammer estimator's own K = lambda/2d and phase
 *                             calibration, so gnss_jam? angles and steering
 *                             angles are the same).
 * Called from the main loop (gnss_smi_poll), about once per millisecond.
 ******************************************************************************/
#ifndef GNSS_SMI_H_
#define GNSS_SMI_H_

#include <stdint.h>

/* ---- register window (byte offsets in the gnss_passthrough aperture) ---- */
#define GNSS_SMI_BASE            0x400U
/* v2 (L 8): entry e at 0x400 + 16 e ; v3 (paged): PAGE = e >> 7, entry at
 * 0x400 + 16 (e & 127). Each: +0 re lo, +4 im lo, +8 {im hi16, re hi16} */
#define GNSS_SMI_REG_COV_V2(e)   (GNSS_SMI_BASE + 16U * (uint32_t)(e))
#define GNSS_SMI_REG_COV_V3(e)   (GNSS_SMI_BASE + 16U * ((uint32_t)(e) & 127U))
#define GNSS_SMI_REG_PAGE        0xC00U   /* v3: RW [2:0] covariance page            */
#define GNSS_SMI_REG_CTRL        0xEC0U   /* RW [0] EN [1] LOCK [2] DC removal OFF [11:8] KEXP */
#define GNSS_SMI_REG_STAT        0xEC4U   /* RO [0] LOCK_ACK [1] bank [2] ACTIVE [31:16] SEQ */
#define GNSS_SMI_REG_COUNT       0xEC8U   /* RO snapshots in the published window    */
#define GNSS_SMI_REG_NOVR        0xECCU   /* RO samples closer than 4 clocks         */
#define GNSS_SMI_REG_INFO        0xED0U   /* RO {ver, REF, L, 'S'}                   */
#define GNSS_SMI_REG_WLOAD       0xED4U   /* W apply the weight shadow; R [0] pending */
#define GNSS_SMI_REG_NLOAD       0xED8U   /* RO weight sets applied                  */
#define GNSS_SMI_W_BASE_V2       0xF00U   /* weight k: re +8k, im +8k+4, Q4.20 25 bits */
#define GNSS_SMI_W_BASE_V3       0xC40U

#define GNSS_SMI_CTRL_EN         (1U << 0)
#define GNSS_SMI_CTRL_LOCK       (1U << 1)
#define GNSS_SMI_CTRL_DCOFF      (1U << 2)
#define GNSS_SMI_CTRL_KEXP(k)    (((uint32_t)(k) & 0xFU) << 8)
#define GNSS_SMI_ST_LOCK_ACK     (1U << 0)
#define GNSS_SMI_ST_ACTIVE       (1U << 2)
#define GNSS_SMI_INFO_ID         0x02030853U   /* v2 (+2 image taps), REF 3, L 8, 'S' */
#define GNSS_SMI_INFO_V3         0x03U         /* INFO[31:24] of the paged layout     */

#define GNSS_SMI_LMAX            16            /* largest L this driver handles       */
#define GNSS_SMI_NI              2             /* widely-linear image taps: conj RX1/RX2 at REF */
#define GNSS_SMI_NMAX            (2 * GNSS_SMI_LMAX + GNSS_SMI_NI)   /* 34 */
#define GNSS_SMI_KEXP_DEFAULT    9U            /* L 8 : 512 snapshots, ~0.23 ms at 45 MSPS */
#define GNSS_SMI_KEXP_DEFAULT_16 8U            /* L 16: 256 snapshots = 7.5 N, ~0.43 ms     */
#define GNSS_SMI_ALPHA_DEFAULT   3.0           /* loading = 3 x lambda_min           */

typedef struct {
    int      present;       /* stap_smi in the bitstream (MVDR >= 1.14, INFO ok) */
    int      layout;        /* 2 = flat (L 8), 3 = paged                         */
    int      L, N, ref;     /* taps per antenna, N = 2L + 2, reference tap       */
    int      La, Na;        /* taps in use per antenna (taps=), solved vector    */
    uint32_t info;          /* INFO as read                                      */
    int      enabled;       /* CTRL.EN written by this driver                   */
    int      hw_active;     /* STAT.ACTIVE (EN reached the datapath)            */
    uint32_t kexp;
    double   alpha;
    int      steer;         /* 0 = power inversion, 1 = steered                 */
    double   steer_phi_deg; /* RX2-RX1 phase of the look direction (raw)        */
    double   steer_theta;   /* angle the user gave (deg from broadside), or 999 */
    uint32_t seq;           /* last window solved                                */
    uint32_t count;         /* snapshots in it                                   */
    uint32_t updates;       /* weight sets written                               */
    uint32_t fails;         /* windows not solved (not positive definite, ...)  */
    uint32_t skipped;       /* windows published while we were busy (seq jumps) */
    uint32_t novr, nload;
    uint32_t novr_new;      /* overruns since gnss_smi_mark_novr() (end of auto-start) */
    double   lam_min;       /* smallest eigenvalue per snapshot, x units^2      */
    double   lam_dbfs;      /* ... as dBFS of the 12-bit input (noise per tap)  */
    double   load;          /* lambda used, per snapshot                         */
    double   wnorm;         /* sum |w|^2 as applied (after the 2^-shift)         */
    double   ref_gain;      /* |w_REF|^2 as applied                              */
    double   pin_dbfs;      /* tr(R)/N per snapshot as dBFS (input power / tap)  */
    double   pout_rel_db;   /* w^H R w / R_ref,ref (output vs RX1 input power)   */
    int      shift;         /* weights scaled 2^-shift to fit Q4.20              */
    int      dc_off;        /* 1 = FPGA DC removal at the SMI input disabled     */
    double   img_gain;      /* |w| of the two image taps (sum of squares)        */
    /* MOD-26: figures of the last solved window (gnss_report?, suppression) */
    double   pin_dbfs_ref;  /* RX1 reference-tap power, dBFS (jammer + noise)    */
    double   p2_dbfs_ref;   /* RX2 power at the same tap, dBFS (MOD-26b)          */
    double   pout_dbfs;     /* w^H R w, dBFS on the RX1 scale (unity = no change)*/
    double   wnorm_u;       /* sum |w|^2 of the solution (before the 2^-shift)   */
    double   jf_hz;         /* strongest signal's frequency offset from the LO   */
    double   jbw_hz;        /* its bandwidth estimate (0 = narrowband / CW)      */
    int      jbw_class;     /* 0 narrow (< ~fs/40), 1 band-limited, 2 wideband  */
    double   coh12;         /* |R12|^2 / (R11 R22) at the reference taps         */
    double   ph21_deg;      /* RX2 - RX1 phase of the strongest signal, deg      */
    double   w2max;         /* largest RX2 tap |w| ...                           */
    int      w2max_k;       /* ... and its delay (0..L-1)                        */
    int      avg_sh;        /* MOD-31: covariance averaged over windows, beta 2^-avg_sh (0 off) */
    uint32_t avg_resets;    /* MOD-31: restarts of the average on a power step  */
    double   keff;          /* MOD-31: effective snapshots in the solved R      */
    /* LVDS: steady averaging -- after avg_hold_ms with no power step the
     * average slows to beta 2^-avg_sh_st (default avg_sh + 2): 4x more
     * snapshots, half the weight jitter; any power step goes back to fast */
    int      avg_sh_st;     /* steady shift (0 = same as avg_sh)                */
    int      avg_steady;    /* 1 = the slow (steady) beta is in use now          */
    /* LVDS: solver timing, microseconds (last / max since smitime=0) */
    double   t_read_us, t_avg_us, t_solve_us, t_write_us, t_post_us, t_total_us;
    double   t_max_us;      /* longest total                                    */
    double   t_int_us;      /* mean time between weight updates                 */
} gnss_smi_state_t;

int    gnss_smi_available(void);          /* reads the hardware              */
int    gnss_smi_enable(int on);           /* 0 ok, -1 not present            */
int    gnss_smi_is_enabled(void);
int    gnss_smi_poll(void);               /* main loop: 1 = a window was solved */
void   gnss_smi_time_clear(void);
int    gnss_smi_set_avg_steady(int sh);   /* 0 = off, 1..6 */
void   gnss_smi_get_state(gnss_smi_state_t *s);
int    gnss_smi_set_kexp(uint32_t k);     /* 6..12                           */
int    gnss_smi_set_taps(int la);         /* taps per antenna in use, 1..L   */
int    gnss_smi_get_taps(void);
int    gnss_smi_weight_taps(int *n1, int *n2); /* non-zero taps, last weights */
uint32_t gnss_smi_get_kexp(void);
int    gnss_smi_set_alpha(double a);      /* 0.5..1000                       */
void   gnss_smi_set_dc(int on);           /* FPGA DC removal (default on)    */
double gnss_smi_get_alpha(void);
/* MOD-31: average the covariance over successive windows before solving
 * (EMA, beta = 2^-sh; 0 = off, the MOD-30 behaviour). Restarted at once when
 * the input power steps by > 1.5 dB (a jammer appears / goes, a gain step),
 * so the reaction to a new jammer stays one window. */
int    gnss_smi_set_avg(int sh);          /* 0..4                            */
int    gnss_smi_get_avg(void);
/* steering: theta = cone angle from broadside (deg, same sign as gnss_jam?);
 * az/el/bhdg: satellite azimuth / elevation and the compass bearing of the
 * array's +90 deg (end-fire) direction; phase: raw RX2-RX1 phase in degrees */
void   gnss_smi_steer_off(void);
int    gnss_smi_steer_theta(double theta_deg);
int    gnss_smi_steer_azel(double az_deg, double el_deg, double bhdg_deg);
void   gnss_smi_steer_phase(double phi_deg);
void   gnss_smi_print(void);              /* gnss_smi?                       */
void   gnss_smi_mark_novr(void);          /* overrun baseline (radio set-up done) */
/* self-test of the solver on a synthetic covariance (no hardware):
 * returns 0 if the MVDR constraint and the null are met */
int    gnss_smi_selftest(int verbose);
/* MOD-26: power response of the filter now at a frequency offset from the LO,
 * averaged over arrival directions: 10 log10(|H1(f)|^2 + |H2(f)|^2) - the
 * same for the unity reference (0 dB = untouched). -99 = no weights yet. */
double gnss_smi_response_db(double f_off_hz, double fs_hz);
/* MOD-29: per-antenna lag correlations r(0..L-1) of the last window, 12-bit
 * LSB^2 per sample; returns L (0 = no window yet). For gnss_spec? / gnss_bist? */
int    gnss_smi_lag_corr(int ant, double *rr, double *ri);
double gnss_herm_psd(int L, const double *ar, const double *ai, double f); /* S(f), f = freq/fs */
double gnss_smi_psd(int ant, double f);                                      /* last window, 12-bit LSB^2 */
double gnss_herm_tone_noise(int L, double *ar, double *ai, double *tone);  /* same, L x L block (modified) */
double gnss_smi_tone_noise(int ant, double *tone);                          /* last window, 12-bit LSB^2 */
/* LVDS / ant board: the ONE suppression figure, the standard definition:
 *
 *   S = (J/N)_in / (J/N)_out
 *   (J/N)_in  = Rj[ref][ref] / sn2                 RX1 reference tap
 *   (J/N)_out = w^H Rj w / (sn2 * ||w||^2)          array output
 *   Rj  = R - sn2 I            jammer (+ satellites) part of the covariance
 *   sn2 = lambda_min / (1 - sqrt(N/Keff))^2         noise per element, with the
 *                                                    small-sample bias removed
 *                                                    (Keff = snapshots in the solved R)
 * J/N, not power, so rescaling the weights changes nothing. Over all taps
 * (space-time), so CW and wideband are measured the same way.
 * (J/N)_out is only measurable down to ~3/sqrt(Keff) (estimation noise of R):
 * below that S is a lower bound ('>').
 * Reference values from the closed form (power inversion, one jammer, M
 * coherent elements, diagonal loading a: rho' = rho/(1+a), beta = rho'/(1+M rho')):
 *   S_th = (1 + M rho')^2 [ (1-beta)^2 + (M-1) beta^2 ]   ~ M(M-1) rho'^2
 *   M = 2  (one weight per antenna, narrowband):  ~ 2 JNR_dB + 3 - 20log(1+a)
 * SMI loss (Reed-Mallett-Brennan): (K-N+2)/(K+1), K = effective snapshots.
 * C/N0 loss from the jammer left (wideband, spread like the noise):
 *   10 log10(1 + (J/N)_out). */
typedef struct {
    int    valid;      /* weights + window exist and a jammer is present (J/N_in > 3 dB) */
    double jn_in_db;   /* (J/N)_in, RX1 reference tap                     */
    double jn_out_db;  /* (J/N)_out, array output (or its upper bound)    */
    double supp_db;    /* S = (J/N)_in / (J/N)_out                        */
    int    supp_lo;    /* 1 = S is a lower bound (J/N_out not measurable) */
    double sth2_db;    /* closed form, 2 elements (1 weight/antenna), with the loading */
    double smi_loss_db;/* RMB average SINR loss for N, Keff               */
    double cn0_loss_db;/* C/N0 lost to the jammer left at the output      */
    double sn2_dbfs;   /* noise per element, dBFS (bias-corrected)        */
    double amp_db;     /* jammer RX2/RX1 power at the reference tap       */
    double ph_deg;     /* jammer RX2-RX1 phase at the reference tap       */
} gnss_null_t;
int  gnss_smi_null_analysis(gnss_null_t *o);   /* 0 = done, -1 no window/weights */

/* MOD-26: called after every solved window (gnss_report.c) */
void   gnss_rep_on_solve(void);

#endif /* GNSS_SMI_H_ */
