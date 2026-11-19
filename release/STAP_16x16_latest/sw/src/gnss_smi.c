/***************************************************************************//**
 *   @file   gnss_smi.c
 *   @brief  GNSS-CRPA MOD-25: SMI-MVDR STAP driver and solver.
 *
 * WHY SMI
 *   The NLMS STAP (gnss_null=2) converges along the eigenvalue spread of R:
 *   strong jammer directions fast, the weak residual directions (a barrage
 *   seen through slightly different RX1/RX2 filters) slowly, and with a mu
 *   misadjustment that adds noise. Sample-matrix inversion takes the exact
 *   minimum-output-power solution of each window in one step:
 *     simulation, 2 x 8 taps, KEXP 9, alpha 3 (RTL + this solver):
 *       CW 40 dB JNR      : residual 0.37 dB over the noise (NLMS 0.6, PI 3.4)
 *       barrage 30 dB JNR : 3.9 dB                            (NLMS 4.9, PI 23)
 *       no jammer         : satellites unchanged (0.03 dB)
 *       steered to a satellite: +2.2 dB SNR on it (array gain, ideal 3)
 *     1.14b, 2 x 16 taps (REF 8: 8 taps before, 7 after), KEXP 8:
 *       barrage, RX2 echo 6 samples -20 dB : 3.9 dB   (8 taps 9.9 dB)
 *       barrage, RX2 echo 12 samples -14 dB: 10.4 dB  (8 taps 16.4 dB)
 *       barrage + -25 dBc image + echo 10  : 10.8 dB  (8 taps 14.0 dB)
 *       CW 0.2 dB, no jammer 0.1 dB; this C solver on the RTL covariance
 *       + RTL filter: barrage 3.7 dB, CW 0.19 dB (gnss_smi_hosttest.c)
 *   The layout (L, REF, register map) is read from INFO: the same firmware
 *   runs the 1.14 (L 8) and 1.14b (L 16) bitstreams.
 *   Reaction: one window (2^KEXP snapshots, a snapshot every ~19 samples:
 *   KEXP 9 = 0.23 ms at 45 MSPS) + this solve (~1 ms main-loop cadence).
 *   A jammer switching faster than that is better served by gnss_null=2.
 *
 * ROBUSTNESS
 *   - diagonal loading lambda = alpha * lambda_min(R): lambda_min of the
 *     space-time covariance is the noise power per tap, so the loading tracks
 *     the RX gain and the AD9361 floor by itself (no absolute setting).
 *     alpha 3: < 0.1 dB SMI loss with no jammer, null depth kept.
 *   - lambda_min by inverse iteration on the Cholesky factor (warm start
 *     from the previous eigenvector: 6 iterations).
 *   - a non-positive-definite window (cannot happen with loading unless the
 *     data are corrupt) retries with 10x the loading, then keeps the old
 *     weights (FAILS counter).
 *   - weights larger than Q4.20 allows are scaled by 2^-shift: the output
 *     level drops by 6 dB per step and AOLC takes it back.
 *   - the published window is LOCKed while it is read, so it is consistent.
 ******************************************************************************/
/* LVDS: the solver runs ~170 times a second; build it optimised even in a
 * Debug (-O0) Vitis configuration. Pure arithmetic, same results. */
#pragma GCC optimize ("O2")
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "gnss_smi.h"

#ifndef GNSS_SMI_HOST
#include "gnss_passthrough.h"
#include "no_os_delay.h"
#include "gnss_time.h"
#define RD(a)       gnss_pt_read(a)
#define WR(a, v)    gnss_pt_write((a), (v))
#define UDELAY(n)   no_os_udelay(n)
#else
/* host build (selftest only): see gnss_smi_host.c */
uint32_t gnss_smi_host_rd(uint32_t a);
void     gnss_smi_host_wr(uint32_t a, uint32_t v);
#define RD(a)       gnss_smi_host_rd(a)
#define WR(a, v)    gnss_smi_host_wr((a), (v))
#define UDELAY(n)   ((void)0)
typedef unsigned long long gnss_time_t;              /* host: a monotonic clock */
#include <time.h>
#define GNSS_COUNTS_PER_SECOND 1000000000ULL
static gnss_time_t gnss_time_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (gnss_time_t)ts.tv_sec * 1000000000ULL + (gnss_time_t)ts.tv_nsec;
}
#define GNSS_PT_REG_MVDR_ID          0x100U
#define GNSS_PT_REG_MVDR_VERSION     0x104U
#define GNSS_PT_MVDR_EXPECTED_ID     0x4D564452U
#define GNSS_PT_MVDR_VERSION_SMI     0x0001000EU
#define GNSS_PT_REG_JAM_CTRL         0x1C0U
#define GNSS_PT_REG_JAM_CAL          0x1C4U
#define GNSS_PT_REG_JAM_K            0x1C8U
#define GNSS_PT_JAM_CTRL_INV         (1U << 1)
static uint32_t gnss_pt_get_fs(void) { return 60000000U; }
#ifndef GNSS_REP_HOST
void gnss_rep_on_solve(void) { }
#endif
#endif

#define NM    GNSS_SMI_NMAX
#define GNSS_SMI_AVG_DEFAULT   2          /* MOD-31: beta 1/4 */
/* run-time layout (from INFO): taps per antenna, vector length, reference */
static int     nL = 8, N = 18, REF = 3, lay = 2;
static uint32_t cur_page = 0xFFFFFFFFU;
#define PI_D  3.14159265358979323846

/* x is the IQ-corrected sample: 12-bit ADC value x 4 (IQF = 2) */
#define X_PER_LSB2   16.0
#define FS2_12       (2048.0 * 2048.0)

/* ------------------------------------------------------------------ state */
static gnss_smi_state_t sm = {
    .layout = 2, .L = 8, .N = 18, .ref = 3,
    .kexp = GNSS_SMI_KEXP_DEFAULT, .alpha = GNSS_SMI_ALPHA_DEFAULT, .steer_theta = 999.0,
    .avg_sh = GNSS_SMI_AVG_DEFAULT, .keff = 0.0, .avg_sh_st = GNSS_SMI_AVG_DEFAULT + 2,
    .leak_db = -99.0, .leak_ref_db = -99.0,
};
#define GNSS_SMI_AVG_HOLD_US   1000000.0      /* fast beta for 1 s after a power step */
static gnss_time_t t_avg_reset = 0;
static gnss_time_t t_last_solve = 0;
static double tus(gnss_time_t a, gnss_time_t b)
{
    return (double)(b - a) * 1e6 / (double)GNSS_COUNTS_PER_SECOND;
}
void gnss_smi_time_clear(void) { sm.t_max_us = 0.0; }
int gnss_smi_set_avg_steady(int sh)
{
    if (sh < 0 || sh > 6) { return -1; }
    sm.avg_sh_st = sh;
    return 0;
}
static int     probed = 0;
static int     have_v = 0;
static void    window_figures(void);
static int     kexp_user = 0;             /* gnss_smi=k given: keep it      */
static int     have_win = 0;              /* MOD-29: Rr/Ri hold a window    */
static double  Rr[NM][NM], Ri[NM][NM];    /* Hermitian, full                */
static double  Lr[NM][NM], Li[NM][NM];    /* Cholesky factor (lower)        */
static double  ar[NM], ai[NM];            /* constraint vector              */
static double  vr[NM], vi[NM];            /* eigenvector estimate (warm)    */
static double  ur[NM], ui[NM];
static double  wr_[NM], wi_[NM];
/* MOD-31: covariance averaged over windows (EMA with a reset on power steps).
 * Without it every ~6 ms weight set comes from one 256-snapshot window: the
 * estimation noise moves each satellite by ~1.1 dB rms / ~5 deg rms from one
 * set to the next (PC simulation, no jammer and CW 40 dB alike). Averaged
 * with beta 1/4: ~0.3 dB / ~1.2 deg, a steadier signal for the receiver's
 * acquisition and tracking, and ~7x more snapshots in R (deeper barrage null). */
#define GNSS_SMI_AVG_STEP      1.41           /* reset when the trace moves > 1.5 dB */
static double  Er[NM][NM], Ei[NM][NM];
static int     have_avg = 0;
static double  avg_tr = 0.0;                   /* trace of E */
/* RX gain changed (AGC, gnss_rxoff): the two channels never step by exactly the
 * same gain and phase, so the averaged R (old gain + new gain) no longer fits
 * the data and the null leaks until the old windows decay (~30 updates in the
 * steady average). Start the average again: the next solve (~1.5 ms) uses
 * only windows taken after the step. */
/* MOD-33: the window being accumulated when the gain steps holds samples from
 * both gains (a step in level AND in RX1/RX2 phase): solving it leaks the
 * null for one update (~1.5 ms) and the jammer reaches the DAC rails. Skip
 * every window whose sequence number is not at least 2 past the one current
 * at the step (~0.6 ms at 61.44 MSPS); the old weights stay loaded meanwhile. */
static int      gain_skip = 0;
static uint32_t gain_seq  = 0;
static uint32_t gain_skipped = 0;
uint32_t gnss_smi_gain_skipped(void) { return gain_skipped; }

/* MOD-33c: CLEAN HOLD. With no jammer, power inversion with 16 time taps
 * whitens the band: the filter keeps moving with the noise estimate (667
 * updates/s), so the satellites see a slowly wandering amplitude, phase and
 * group delay -- the receiver's carrier / code loops pull in and lock a little
 * slower than on the raw RX1 path. While the SMI J/N stays under clean_in_db
 * for 2 s, TX1 gets the fixed reference tap (RX1 delayed 8 samples: exactly
 * the pass=1 signal) and nothing more is written. The solver keeps running on
 * every window, so the first window with J/N over clean_out_db writes the
 * nulling weights again (~1.5 ms): a jammer is nulled as before. */
static int      clean_en     = 1;
static double   clean_in_db  = -3.0;     /* enter under this J/N for 2 s */
static double   clean_out_db = 0.0;      /* leave over this J/N at once */
static int      clean_hold   = 0;
static int      clean_cnt_ok = 0;        /* consecutive solves under clean_in_db */
static uint32_t clean_enters = 0;
static gnss_time_t clean_t0;
void gnss_smi_set_clean(int en, double in_db, double out_db)
{
    clean_en = en ? 1 : 0;
    if (out_db < in_db) { out_db = in_db; }
    clean_in_db = in_db;
    clean_out_db = out_db;
    if (!clean_en) { clean_hold = 0; clean_cnt_ok = 0; }
}
/* ---------------------------------------------------------------- MOD6
 * 1. Residual-driven averaging: before each window is averaged in, the
 *    current weights are applied to it alone. If the jammer left rises 3 dB
 *    over its usual value (jammer or platform moving / turning), the average
 *    runs fast (beta 2^-avg_sh) for 1 s; 6 dB over, it restarts from this
 *    window. Static jammer: steady averaging as before.
 * 2. Pulsed jammer: window trace against a quiet (noise-only) reference. Four
 *    or more pulse starts within 2 s = pulse mode: only jammer-on windows are
 *    averaged and solved, the average does not restart when the pulse ends,
 *    and the null stays loaded between pulses (no leak at every pulse edge).
 *    Ends 3 s after the last pulse start. Short pulses (many per window) look
 *    like a steady jammer and need nothing.
 * 3. Stable jammer class for the AGC (jbw_wide): the per-window class flips
 *    at gain steps / clipping; the AGC barrage rule followed it (a CW got the
 *    slow 5-s barrage rise on 16 Nov). */
static int      track_en = 1, pulse_en = 1;
static double   sn2_last = 0.0, jn_last_db = -99.0;
static double   tr_q = 0.0;                       /* quiet window trace */
static gnss_time_t p_edge[4];                     /* last pulse starts  */
static int      p_ne = 0, p_ei = 0;
static double   wide_score = 0.0;
static double   tr_clean = 0.0;                   /* window trace while in clean hold */
static double   tr_win = 0.0;                     /* trace of the newest raw window   */
static double   tr_noise = 0.0;                   /* window trace with no jammer (J/N < -3 dB) */
/* MOD7: the noise per element = antenna part (moves with the gain) + the
 * AD9361's own floor (does not). MOD6 scaled all of it with the gain, so
 * after a deep cut the reference fell 10-35 dB under the real noise and the
 * tracking saw a "leak" that was not there (52k restarts on 18 Nov). */
static double   floor_sn2 = 0.0;                  /* AD9361 internal noise per element */
static double   ant_sn2 = 0.0;                    /* antenna noise per element, now    */
static gnss_time_t t_restart;                     /* last tracking restart             */
static int      have_t_restart = 0;
static double noise_ref(void) { return (ant_sn2 > 0.0) ? ant_sn2 + floor_sn2 : 0.0; }
/* MOD11: spatial gate on the solved (averaged) R, reference taps.
 * One source in both antennas with J/N x1, x2 per antenna gives
 *     coh = |R12|^2/(R11 R22) = x1/(1+x1) * x2/(1+x2),
 * so s = sqrt(coh) = x/(1+x) for the geometric-mean x: x = s/(1-s), free
 * of any noise estimate. Jammer J/N 6 dB -> coh 0.64; 3 dB -> 0.44;
 * 0 dB -> 0.25. Bench, no jammer, two active antennas side by side: 0.10
 * (mutual coupling) = -2.6 dB. Incoherent when under 0 dB, coherent again
 * over +3 dB. A source in ONE antenna only (other blocked) is the exception:
 * the power then differs by > 12 dB between the antennas, and the gate is
 * not applied (MVDR uses the other antenna). */
#define COH_IN_DB    0.0
#define COH_OUT_DB   3.0
#define COH_BAL_DB  12.0
static void coh_gate(void)
{
    double r11 = Rr[REF][REF], r22 = Rr[nL + REF][nL + REF];
    double xr = Rr[REF][nL + REF], xi = Ri[REF][nL + REF], c, sq, bal;
    if (!(r11 > 0.0) || !(r22 > 0.0)) { sm.incoh = 0; sm.jc_db = 99.0; return; }
    c  = (xr * xr + xi * xi) / (r11 * r22);
    if (c < 1e-12) { c = 1e-12; }
    sq = sqrt(c);
    if (sq > 0.9999) { sq = 0.9999; }
    sm.jc_db = 10.0 * log10(sq / (1.0 - sq));
    bal = 10.0 * log10(r11 / r22);
    if (fabs(bal) > COH_BAL_DB)        { sm.incoh = 0; }
    else if (sm.jc_db < COH_IN_DB)     { sm.incoh = 1; }
    else if (sm.jc_db > COH_OUT_DB)    { sm.incoh = 0; }
}
void gnss_smi_set_floor_dbfs(double dbfs)
{
    floor_sn2 = (dbfs > -150.0) ? pow(10.0, dbfs / 10.0) * X_PER_LSB2 * FS2_12 : 0.0;
    ant_sn2 = 0.0;        /* new radio plan (auto-start): relearn the antenna noise */
    tr_noise = 0.0;
}
/* MOD7: jammer type from the raw windows */
#define JH 128
static gnss_time_t jh_t[JH];
static float    jh_f[JH];                          /* centre frequency, Hz (NaN-free: valid flag) */
static int8_t   jh_c[JH];                          /* instantaneous class, -1 = no jammer */
static int8_t   jh_v[JH];                          /* frequency valid */
static int      jh_i = 0, jh_n = 0;
static int      jt_cand = 0, jt_cnt = 0;
static gnss_time_t jt_t_seen;                     /* last window with a jammer */
static int      have_jt_seen = 0;
static gnss_time_t sw_wrap[6];
static int      sw_nw = 0, sw_wi = 0;
static int      sw_last_dir = 0;
const char *gnss_smi_jtype_name(int jt)
{
    switch (jt) {
    case GNSS_JT_CW:       return "CW";
    case GNSS_JT_MULTI:    return "MULTI-TONE";
    case GNSS_JT_WIDE:     return "WIDEBAND";
    case GNSS_JT_CHIRP:    return "CHIRP";
    case GNSS_JT_SAWTOOTH: return "SAWTOOTH";
    default:               return "none";
    }
}
void gnss_smi_set_track(int en) { track_en = en ? 1 : 0; }
int  gnss_smi_get_track(void)  { return track_en; }
void gnss_smi_set_pulse(int en) { pulse_en = en ? 1 : 0; if (!pulse_en) { sm.pulse_mode = 0; } }
int  gnss_smi_get_pulse(void)  { return pulse_en; }
void gnss_smi_gain_changed_db(double d_db)
{
    if (tr_q > 0.0) { tr_q *= pow(10.0, d_db / 10.0); }
    if (tr_clean > 0.0) { tr_clean *= pow(10.0, d_db / 10.0); }
    if (tr_noise > 0.0) { tr_noise *= pow(10.0, d_db / 10.0); }
    if (ant_sn2 > 0.0) { ant_sn2 *= pow(10.0, d_db / 10.0); }
    if (sm.sweep_mode && have_avg) {
        /* MOD7: a sweep needs the whole sweep in the average; a restart costs
         * the null until it is rebuilt (up to ~100 ms). Scale it instead (the
         * straddling window is still skipped). */
        double g = pow(10.0, d_db / 10.0);
        int i, j;
        for (i = 0; i < N; i++) { for (j = 0; j < N; j++) { Er[i][j] *= g; Ei[i][j] *= g; } }
        avg_tr *= g;
        if (sm.present) { gain_seq = RD(GNSS_SMI_REG_STAT) >> 16; gain_skip = 1; }
        return;
    }
    gnss_smi_gain_changed();
}


/* MOD7: per raw window: instantaneous class + centre frequency; every window:
 * the jammer type over the last 0.3 s (hysteresis 16 windows).
 * Classes from the lag correlations |r(k)| / (P - noise), k = 0..L-1:
 *   CW        flat (>= 0.8 at the last lag)
 *   MULTI     falls under 0.4 and comes back over 0.7 (tones beating)
 *   WIDE      gone after one lag (< 0.5 at lag 1)
 *   BAND      decays in between (band-limited noise, or a sweep segment)
 * Swept (CHIRP / SAWTOOTH): the centre frequency of narrow windows moves
 * over > 2 MHz within 0.3 s; one-way ramps with jumps back = SAWTOOTH. */
static void mod7_classify(gnss_time_t now, double trw)
{
    double nz = noise_ref(), fs = (double)gnss_pt_get_fs(), p0 = trw / (double)N;
    double rr[GNSS_SMI_LMAX], r1r = 0.0, r1i = 0.0;
    int k, i, ant, cls = -1, fv = 0, c;
    float f = 0.0f;

    if (!(nz > 0.0)) { nz = sn2_last; }
    if (!sm.incoh && nz > 0.0 && p0 > 2.0 * nz && nL >= 4) {   /* J/N over 0 dB; MOD11: coherent */
        double pj = p0 - nz, mn = 1.0;
        int dips = 0, below = 0, revive = 0;
        for (k = 0; k < nL; k++) {
            double sr = 0.0, si = 0.0;
            int n = 0;
            for (ant = 0; ant < 2; ant++) {
                for (i = 0; i + k < nL; i++, n++) {
                    sr += Rr[ant * nL + i][ant * nL + i + k];
                    si += Ri[ant * nL + i][ant * nL + i + k];
                }
            }
            sr /= (double)n; si /= (double)n;
            if (k == 1) { r1r = sr; r1i = si; }
            rr[k] = sqrt(sr * sr + si * si) / pj;
            if (rr[k] > 1.0) { rr[k] = 1.0; }
        }
        /* tones beat: |r(k)| falls under 0.4 and comes back over 0.7 (noise
         * of any bandwidth: sinc side lobes stay under ~0.25) */
        for (k = 1; k < nL; k++) {
            if (rr[k] < mn) { mn = rr[k]; }
            if (!below && rr[k] < 0.4) { below = 1; dips++; }
            else if (below && rr[k] > 0.7) { below = 0; revive++; }
        }
        if (rr[nL - 1] >= 0.8 && mn >= 0.6)      { cls = 0; }
        else if (revive >= 1)                    { cls = 1; sm.ntones = dips + 1; }
        else if (rr[1] < 0.5)                    { cls = 3; }
        else                                     { cls = 2; }
        if (rr[1] > 0.85 && fs > 0.0) { fv = 1; f = (float)(atan2(r1i, r1r) / (2.0 * PI_D) * fs); }   /* MOD11: narrow windows only (was 0.5) */
        jt_t_seen = now;
        have_jt_seen = 1;
    }
    jh_t[jh_i] = now; jh_f[jh_i] = f; jh_c[jh_i] = (int8_t)cls; jh_v[jh_i] = (int8_t)fv;
    jh_i = (jh_i + 1) % JH;
    if (jh_n < JH) { jh_n++; }

    /* ---- the last 0.3 s */
    {
        int n_on = 0, cnt[4] = {0, 0, 0, 0}, nfv = 0, pos = 0, neg = 0, big = 0, start, m, idx, havep = 0;
        double fmin = 1e12, fmax = -1e12, span, small_sum = 0.0, fprev = 0.0;
        int cand;
        start = (jh_i - jh_n + JH) % JH;
        for (m = 0; m < jh_n; m++) {                      /* span of valid frequencies */
            idx = (start + m) % JH;
            if (tus(jh_t[idx], now) > 3.0e5 || jh_c[idx] < 0) { continue; }
            n_on++;
            cnt[jh_c[idx]]++;
            if (jh_v[idx]) {
                nfv++;
                if (jh_f[idx] < fmin) { fmin = jh_f[idx]; }
                if (jh_f[idx] > fmax) { fmax = jh_f[idx]; }
            }
        }
        span = (nfv >= 2) ? fmax - fmin : 0.0;
        for (m = 0; m < jh_n; m++) {                      /* steps between narrow windows */
            idx = (start + m) % JH;
            if (tus(jh_t[idx], now) > 3.0e5 || jh_c[idx] < 0 || !jh_v[idx]) { continue; }
            if (havep) {
                double d = (double)jh_f[idx] - fprev;
                if (span > 0.0 && fabs(d) >= 0.4 * span) { big++; }
                else if (d > 0.0) { pos++; small_sum += d; }
                else if (d < 0.0) { neg++; small_sum -= d; }
            }
            fprev = jh_f[idx];
            havep = 1;
        }
        /* sawtooth period: the newest step being a jump back */
        if (fv && span > 2.0e6) {
            int pi_ = (jh_i - 2 + JH) % JH;
            if (jh_c[pi_] >= 0 && jh_v[pi_]) {
                double d = (double)f - (double)jh_f[pi_];
                int dir = (pos > neg) ? 1 : -1;
                if (fabs(d) >= 0.4 * span && ((d > 0.0) ? 1 : -1) == -dir) {
                    sw_wrap[sw_wi] = now; sw_wi = (sw_wi + 1) % 6;
                    if (sw_nw < 6) { sw_nw++; }
                }
                sw_last_dir = dir;
            }
        }
        if (!have_jt_seen || tus(jt_t_seen, now) > 1.0e6 || n_on < 4) {
            cand = GNSS_JT_NONE;
        } else if (n_on >= 12 && nfv >= (6 * n_on) / 10 && span > 2.0e6 && small_sum > 1.0e6) {
            cand = (big >= 1 && pos + neg >= 6 && (pos >= 3 * neg || neg >= 3 * pos)) ?
                   GNSS_JT_SAWTOOTH : GNSS_JT_CHIRP;
            sm.sweep_lo_hz = fmin;
            sm.sweep_hi_hz = fmax;
        } else if (cnt[0] * 10 >= n_on * 6) {
            cand = GNSS_JT_CW;
        } else if (cnt[1] * 10 >= n_on * 3) {
            cand = GNSS_JT_MULTI;
        } else if ((cnt[2] + cnt[3]) * 2 >= n_on) {
            cand = GNSS_JT_WIDE;
        } else {
            cand = GNSS_JT_CW;
        }
        if (cand == jt_cand) { if (jt_cnt < 1000) { jt_cnt++; } }
        else { jt_cand = cand; jt_cnt = 1; }
        c = (cand == GNSS_JT_NONE) ? 64 : 16;           /* leave a type slower than enter it */
        if (jt_cnt >= c && sm.jtype != cand) {
            int was_sweep = sm.sweep_mode;
            sm.jtype = cand;
            sm.sweep_mode = (cand == GNSS_JT_CHIRP || cand == GNSS_JT_SAWTOOTH) ? 1 : 0;
            if (sm.sweep_mode && !was_sweep) {
                sw_nw = 0;
                sm.sweep_period_ms = 0.0;
            }
            if (was_sweep && !sm.sweep_mode) { have_avg = 0; }
        }
        if (sm.sweep_mode) {
            double per = 0.0, tu = (sm.t_int_us > 100.0) ? sm.t_int_us : 1500.0;
            if (sw_nw >= 3) {
                gnss_time_t a = sw_wrap[(sw_wi - sw_nw + 6) % 6], b = sw_wrap[(sw_wi + 5) % 6];
                per = tus(a, b) / 1000.0 / (double)(sw_nw - 1);
            }
            sm.sweep_period_ms = per;
            /* memory >= 2 sweeps: 2^6 updates ~ 100 ms, 2^7 ~ 200 ms */
            sm.sweep_sh = (per > 0.0 && 2.0 * per * 1000.0 > 64.0 * tu) ? 7 : 6;
        }
    }
}

/* returns 1 = pulse mode and the pulse is off: do not average or solve */
static int mod6_window(gnss_time_t now)
{
    double trw = 0.0;
    int i;
    for (i = 0; i < N; i++) { trw += Rr[i][i]; }
    tr_win = trw;
    if (!(trw > 0.0)) { return 0; }
    mod7_classify(now, trw);
    /* quiet reference: follows the minimum, creeps up ~2 dB/s */
    if (!(tr_q > 0.0) || trw < tr_q) { tr_q = trw; }
    else { tr_q *= 1.0007; if (tr_q > trw) { tr_q = trw; } }
    if (!sm.pulse_on && trw > 4.0 * tr_q) {          /* +6 dB: a pulse starts */
        sm.pulse_on = 1;
        p_edge[p_ei] = now;
        p_ei = (p_ei + 1) & 3;
        if (p_ne < 4) { p_ne++; }
    } else if (sm.pulse_on && trw < 2.0 * tr_q) {    /* back under +3 dB */
        sm.pulse_on = 0;
    }
    if (pulse_en && p_ne == 4) {
        gnss_time_t newest = p_edge[(p_ei + 3) & 3], oldest = p_edge[p_ei];
        if (!sm.pulse_mode && tus(oldest, newest) <= 2.0e6) {
            sm.pulse_mode = 1;
            sm.pulse_modes++;
        }
        if (sm.pulse_mode && tus(newest, now) > 3.0e6) {
            sm.pulse_mode = 0;
            p_ne = 0;
        }
    } else if (!pulse_en) {
        sm.pulse_mode = 0;
    }
    if (sm.pulse_mode && !sm.pulse_on) {
        sm.pulse_skipped++;
        return 1;
    }
    /* residual-driven averaging (on the raw window, before averaging) */
    sm.leak_db = -99.0;
    /* jammer present: by J/N, or by power over the no-jammer trace (a fast
     * moving jammer smears the average, which makes J/N read low) */
    if (have_avg && sm.updates > 0U && sn2_last > 0.0 &&
        (jn_last_db > 6.0 || (noise_ref() > 0.0 && trw > 4.0 * (double)N * noise_ref()))) {
        double por = 0.0, wn = 0.0, lk;
        int j;
        for (i = 0; i < N; i++) {
            double ar = 0.0, ai = 0.0;
            wn += wr_[i] * wr_[i] + wi_[i] * wi_[i];
            for (j = 0; j < N; j++) {
                ar += Rr[i][j] * wr_[j] - Ri[i][j] * wi_[j];
                ai += Rr[i][j] * wi_[j] + Ri[i][j] * wr_[j];
            }
            por += wr_[i] * ar + wi_[i] * ai;
        }
        if (wn > 0.0 && por > 0.0) {
            /* noise per element: the no-jammer reference if known (not inflated by a smeared R) */
            double sn2 = (noise_ref() > 0.0) ? noise_ref() : sn2_last;   /* MOD7: + AD9361 floor */
            lk = 10.0 * log10(por / (wn * sn2));
            sm.leak_db = lk;
            if (sm.leak_ref_db < -90.0) { sm.leak_ref_db = (lk < 1.0) ? lk : 1.0; }
            if (!track_en || sm.sweep_mode) {
                /* measured only (gnss_track=0; a sweep leaks by design between
                 * its frequencies: the long average is the cure, not a restart) */
            } else if (lk > sm.leak_ref_db + 6.0 &&
                       (!have_t_restart || tus(t_restart, now) > 3.0e4)) {
                have_avg = 0;                         /* solve from this window (<= 1 per 30 ms) */
                sm.leak_reset++;
                t_restart = now;
                have_t_restart = 1;
            } else if (lk > sm.leak_ref_db + 3.0) {
                t_avg_reset = now;                    /* fast beta for 1 s */
                sm.avg_steady = 0;
                sm.leak_fast++;
            }
            if (!track_en || sm.sweep_mode || lk <= sm.leak_ref_db + 3.0) {
                sm.leak_ref_db += (lk - sm.leak_ref_db) / 64.0;
                if (sm.leak_ref_db > 1.0) { sm.leak_ref_db = 1.0; }   /* a slow drift still triggers */
            }
        }
    }
    return 0;
}

void gnss_smi_get_clean(int *en, double *in_db, double *out_db, int *hold, uint32_t *enters)
{
    if (en) { *en = clean_en; }
    if (in_db) { *in_db = clean_in_db; }
    if (out_db) { *out_db = clean_out_db; }
    if (hold) { *hold = clean_hold; }
    if (enters) { *enters = clean_enters; }
}
void gnss_smi_gain_changed(void)
{
    if (have_avg) { sm.avg_resets++; }
    have_avg = 0;
    if (sm.present) {
        gain_seq  = RD(GNSS_SMI_REG_STAT) >> 16;
        gain_skip = 1;
    }
}
static double  avg_s2 = 1.0;                   /* sum of squared EMA weights (avg_tr: above) */

/* register addresses of the active layout */
static uint32_t w_re(int k) { return (lay == 3 ? GNSS_SMI_W_BASE_V3 : GNSS_SMI_W_BASE_V2) + 8U * (uint32_t)k; }
static uint32_t cov_addr(int e)
{
    if (lay != 3) { return GNSS_SMI_REG_COV_V2(e); }
    if (((uint32_t)e >> 7) != cur_page) {
        cur_page = (uint32_t)e >> 7;
        WR(GNSS_SMI_REG_PAGE, cur_page);
    }
    return GNSS_SMI_REG_COV_V3(e);
}
/* Build option (STAP package latest1): taps per antenna in use at boot.
 * 0 or not defined = every tap the bitstream has (MOD13 behaviour);
 * the build passes GNSS_SMI_TAPS_BOOT=5 for the 5x5 image. taps= still
 * changes it at run time. */
#ifndef GNSS_SMI_TAPS_BOOT
#define GNSS_SMI_TAPS_BOOT 0
#endif
static void set_layout(int v, int l, int r)
{
    lay = v;  nL = l;  N = 2 * l + GNSS_SMI_NI;  REF = r;
    sm.layout = v;  sm.L = l;  sm.N = N;  sm.ref = r;
    if (sm.La <= 0 || sm.La > l) {
        sm.La = (GNSS_SMI_TAPS_BOOT >= 1 && GNSS_SMI_TAPS_BOOT <= l) ? GNSS_SMI_TAPS_BOOT : l;
    }
    sm.Na = 2 * sm.La + GNSS_SMI_NI;
    have_avg = 0;                              /* MOD-31 */
    if (!kexp_user) { sm.kexp = (l > 8) ? GNSS_SMI_KEXP_DEFAULT_16 : GNSS_SMI_KEXP_DEFAULT; }
}

/* ---------------------------------------------------------- linear algebra */
/* A = R + lam I = L L^H ; returns 0 on success */
static int chol(double lam)
{
    int i, j, k;
    for (j = 0; j < N; j++) {
        double d = Rr[j][j] + lam;
        for (k = 0; k < j; k++) { d -= Lr[j][k] * Lr[j][k] + Li[j][k] * Li[j][k]; }
        if (!(d > 0.0)) { return -1; }
        d = sqrt(d);
        Lr[j][j] = d;  Li[j][j] = 0.0;
        for (i = j + 1; i < N; i++) {
            double sr = Rr[i][j], si = Ri[i][j];
            for (k = 0; k < j; k++) {          /* L[i][k] conj(L[j][k]) */
                sr -= Lr[i][k] * Lr[j][k] + Li[i][k] * Li[j][k];
                si -= Li[i][k] * Lr[j][k] - Lr[i][k] * Li[j][k];
            }
            Lr[i][j] = sr / d;  Li[i][j] = si / d;
        }
        for (i = 0; i < j; i++) { Lr[i][j] = 0.0; Li[i][j] = 0.0; }
    }
    return 0;
}

/* u = A^-1 v  (A = L L^H) */
static void solve(const double *br, const double *bi, double *xr, double *xi)
{
    double yr[NM], yi[NM];
    int i, k;
    for (i = 0; i < N; i++) {                  /* L y = b */
        double sr = br[i], si = bi[i];
        for (k = 0; k < i; k++) {
            sr -= Lr[i][k] * yr[k] - Li[i][k] * yi[k];
            si -= Lr[i][k] * yi[k] + Li[i][k] * yr[k];
        }
        yr[i] = sr / Lr[i][i];  yi[i] = si / Lr[i][i];
    }
    for (i = N - 1; i >= 0; i--) {             /* L^H x = y */
        double sr = yr[i], si = yi[i];
        for (k = i + 1; k < N; k++) {          /* conj(L[k][i]) x[k] */
            sr -= Lr[k][i] * xr[k] + Li[k][i] * xi[k];
            si -= Lr[k][i] * xi[k] - Li[k][i] * xr[k];
        }
        xr[i] = sr / Lr[i][i];  xi[i] = si / Lr[i][i];
    }
}

/* MVDR weights for the covariance in Rr/Ri (any scale) and the vector a.
 * Fills wr_/wi_, sm.lam_min/load (same scale as R). 0 = ok. */
static int mvdr_solve_n(void)
{
    double tr = 0.0, eps, nrm, lam, gr, gi, g2;
    int i, it, tries;

    for (i = 0; i < N; i++) { tr += Rr[i][i]; }
    if (!(tr > 0.0)) { return -1; }
    eps = 1e-9 * tr / N + 1e-6;
    if (chol(eps) != 0) { return -1; }

    /* lambda_min by inverse iteration (warm start) */
    if (!have_v) {
        for (i = 0; i < N; i++) { vr[i] = 1.0 / 4.0; vi[i] = 0.0; }
        have_v = 1;
    }
    nrm = 0.0;
    for (it = 0; it < 6; it++) {
        solve(vr, vi, ur, ui);
        nrm = 0.0;
        for (i = 0; i < N; i++) { nrm += ur[i] * ur[i] + ui[i] * ui[i]; }
        nrm = sqrt(nrm);
        if (!(nrm > 0.0)) { have_v = 0; return -1; }
        for (i = 0; i < N; i++) { vr[i] = ur[i] / nrm; vi[i] = ui[i] / nrm; }
    }
    sm.lam_min = 1.0 / nrm;
    lam = sm.alpha * sm.lam_min;
    if (lam < eps) { lam = eps; }

    for (tries = 0; tries < 3; tries++) {
        if (chol(lam) == 0) { break; }
        lam *= 10.0;
    }
    if (tries == 3) { return -1; }
    sm.load = lam;

    solve(ar, ai, ur, ui);
    gr = 0.0; gi = 0.0;                        /* g = a^H u */
    for (i = 0; i < N; i++) {
        gr += ar[i] * ur[i] + ai[i] * ui[i];
        gi += ar[i] * ui[i] - ai[i] * ur[i];
    }
    g2 = gr * gr + gi * gi;
    if (!(g2 > 0.0)) { return -1; }
    for (i = 0; i < N; i++) {                  /* w = u / g */
        wr_[i] = (ur[i] * gr + ui[i] * gi) / g2;
        wi_[i] = (ui[i] * gr - ur[i] * gi) / g2;
    }
    return 0;
}


/* taps= : solve on the taps in use only (La per antenna, centred on REF like
 * the full layout: La/2 newer + the rest older, image taps kept). The other
 * taps get weight 0, so the FPGA filter is exactly an La-tap STAP. */
static double  Sr_[NM][NM], Si_[NM][NM];
static int     act_idx[NM];

static int act_list(void)
{
    int k0, k1, k, n = 0, i;
    if (sm.La >= nL) { for (i = 0; i < N; i++) { act_idx[i] = i; } return N; }
    k0 = REF - sm.La / 2;
    if (k0 < 0) { k0 = 0; }
    k1 = k0 + sm.La - 1;
    if (k1 > nL - 1) { k1 = nL - 1; k0 = k1 - sm.La + 1; }
    for (k = k0; k <= k1; k++) { act_idx[n++] = k; }
    for (k = k0; k <= k1; k++) { act_idx[n++] = nL + k; }
    for (i = 2 * nL; i < N; i++) { act_idx[n++] = i; }
    return n;
}

static int mvdr_solve(void)
{
    int na, n0, i, j, rc;
    double tar[NM], tai[NM];

    na = act_list();
    sm.Na = na;
    if (na == N) { return mvdr_solve_n(); }
    n0 = N;
    for (i = 0; i < N; i++) {
        tar[i] = ar[i]; tai[i] = ai[i];
        for (j = 0; j < N; j++) { Sr_[i][j] = Rr[i][j]; Si_[i][j] = Ri[i][j]; }
    }
    for (i = 0; i < na; i++) {
        ar[i] = tar[act_idx[i]]; ai[i] = tai[act_idx[i]];
        for (j = 0; j < na; j++) { Rr[i][j] = Sr_[act_idx[i]][act_idx[j]]; Ri[i][j] = Si_[act_idx[i]][act_idx[j]]; }
    }
    N = na;
    rc = mvdr_solve_n();
    N = n0;
    {
        double xr[NM], xi[NM];
        for (i = 0; i < na; i++) { xr[i] = wr_[i]; xi[i] = wi_[i]; }
        for (i = 0; i < N; i++) { wr_[i] = 0.0; wi_[i] = 0.0; }
        if (rc == 0) { for (i = 0; i < na; i++) { wr_[act_idx[i]] = xr[i]; wi_[act_idx[i]] = xi[i]; } }
    }
    for (i = 0; i < N; i++) {
        ar[i] = tar[i]; ai[i] = tai[i];
        for (j = 0; j < N; j++) { Rr[i][j] = Sr_[i][j]; Ri[i][j] = Si_[i][j]; }
    }
    return rc;
}

/* lambda_min of the full R (all taps), for the noise estimate when fewer taps
 * are solved: the reduced R can hide jammer energy in its smallest eigenvalue */
static double lam_min_full(void)
{
    double tr = 0.0, eps, nrm = 0.0, xr[NM], xi[NM], yr[NM], yi[NM];
    int i, it;
    for (i = 0; i < N; i++) { tr += Rr[i][i]; }
    if (!(tr > 0.0)) { return 0.0; }
    eps = 1e-9 * tr / N + 1e-6;
    if (chol(eps) != 0) { return 0.0; }
    for (i = 0; i < N; i++) { xr[i] = 1.0 / (1.0 + i); xi[i] = 0.0; }
    for (it = 0; it < 12; it++) {
        solve(xr, xi, yr, yi);
        nrm = 0.0;
        for (i = 0; i < N; i++) { nrm += yr[i] * yr[i] + yi[i] * yi[i]; }
        nrm = sqrt(nrm);
        if (!(nrm > 0.0)) { return 0.0; }
        for (i = 0; i < N; i++) { xr[i] = yr[i] / nrm; xi[i] = yi[i] / nrm; }
    }
    return 1.0 / nrm;
}

int gnss_smi_set_taps(int la)
{
    (void)gnss_smi_available();
    if (la < 1 || la > nL) { return -1; }
    sm.La = la;
    sm.Na = 2 * la + GNSS_SMI_NI;
    have_v = 0;                                /* new vector length: cold start */
    return 0;
}
/* taps that really carry a weight in the last set written to the FPGA */
int gnss_smi_weight_taps(int *n1, int *n2)
{
    int k, a = 0, b = 0;
    if (!sm.updates) { if (n1) { *n1 = 0; } if (n2) { *n2 = 0; } return -1; }
    if (clean_hold) { if (n1) { *n1 = 1; } if (n2) { *n2 = 0; } return 0; }   /* MOD-33c */
    for (k = 0; k < nL; k++) {
        if (wr_[k] != 0.0 || wi_[k] != 0.0) { a++; }
        if (wr_[nL + k] != 0.0 || wi_[nL + k] != 0.0) { b++; }
    }
    if (n1) { *n1 = a; }
    if (n2) { *n2 = b; }
    return 0;
}
int gnss_smi_get_taps(void) { return sm.La > 0 ? sm.La : nL; }

/* -------------------------------------------------------------- hardware */
int gnss_smi_available(void)
{
    if (!probed) {
        uint32_t info = 0;
        sm.present = 0;
        if (RD(GNSS_PT_REG_MVDR_ID) == GNSS_PT_MVDR_EXPECTED_ID &&
            RD(GNSS_PT_REG_MVDR_VERSION) >= GNSS_PT_MVDR_VERSION_SMI) {
            info = RD(GNSS_SMI_REG_INFO);
            if (info == GNSS_SMI_INFO_ID) {
                set_layout(2, 8, 3);
                sm.present = 1;
            } else if ((info >> 24) == GNSS_SMI_INFO_V3 && (info & 0xFFU) == 0x53U) {
                int l = (int)((info >> 8) & 0xFFU), r = (int)((info >> 16) & 0xFFU);
                if (l >= 2 && l <= GNSS_SMI_LMAX && r < l) {
                    set_layout(3, l, r);
                    cur_page = 0xFFFFFFFFU;
                    sm.present = 1;
                }
            }
        }
        sm.info = info;
        probed = 1;
    }
    return sm.present;
}

static void set_a(void)
{
    int i;
    for (i = 0; i < N; i++) { ar[i] = 0.0; ai[i] = 0.0; }
    ar[REF] = 1.0;
    if (sm.steer) {
        double p = sm.steer_phi_deg * PI_D / 180.0;
        ar[nL + REF] = cos(p);
        ai[nL + REF] = sin(p);
    }
}

static uint32_t ctrl_word(int lock)
{
    return GNSS_SMI_CTRL_KEXP(sm.kexp) | (sm.enabled ? GNSS_SMI_CTRL_EN : 0U) |
           (lock ? GNSS_SMI_CTRL_LOCK : 0U) | (sm.dc_off ? GNSS_SMI_CTRL_DCOFF : 0U);
}

void gnss_smi_set_dc(int on)
{
    sm.dc_off = on ? 0 : 1;
    if (gnss_smi_available()) { WR(GNSS_SMI_REG_CTRL, ctrl_word(0)); }
}

/* write wr_/wi_ (scaled to fit Q4.20) and apply */
static void write_weights(void)
{
    double mx = 0.0, sc, wn = 0.0;
    int i, k = 0, t;

    for (i = 0; i < N; i++) {
        if (fabs(wr_[i]) > mx) { mx = fabs(wr_[i]); }
        if (fabs(wi_[i]) > mx) { mx = fabs(wi_[i]); }
    }
    while (mx * ldexp(1.0, -k) >= 15.99 && k < 20) { k++; }
    sc = ldexp(1048576.0, -k);
    for (t = 0; t < 1000 && (RD(GNSS_SMI_REG_WLOAD) & 1U); t++) { UDELAY(1); }
    for (i = 0; i < N; i++) {
        double xr = wr_[i] * sc, xi = wi_[i] * sc;
        long qr = (long)floor(xr + 0.5), qi = (long)floor(xi + 0.5);
        WR(w_re(i),      (uint32_t)qr);
        WR(w_re(i) + 4U, (uint32_t)qi);
        wn += ((double)qr * qr + (double)qi * qi) / (1048576.0 * 1048576.0);
    }
    WR(GNSS_SMI_REG_WLOAD, 1U);
    sm.shift    = k;
    sm.wnorm    = wn;
    sm.img_gain = 0.0;
    for (i = 2 * nL; i < N; i++) {
        sm.img_gain += (wr_[i] * wr_[i] + wi_[i] * wi_[i]) * ldexp(1.0, -2 * k);
    }
    sm.ref_gain = (wr_[REF] * wr_[REF] + wi_[REF] * wi_[REF]) * ldexp(1.0, -2 * k);
}

/* MOD-33c: the reference tap alone (RX1 delayed REF samples = the pass=1 path) */
static void write_identity(void)
{
    double sr[NM], si[NM];
    int i;
    for (i = 0; i < N; i++) { sr[i] = wr_[i]; si[i] = wi_[i]; wr_[i] = 0.0; wi_[i] = 0.0; }
    wr_[REF] = 1.0;
    write_weights();
    for (i = 0; i < N; i++) { wr_[i] = sr[i]; wi_[i] = si[i]; }
}

/* conventional weights a / (a^H a): the start before the first window */
static void write_conventional(void)
{
    double aa = 0.0;
    int i;
    set_a();
    for (i = 0; i < N; i++) { aa += ar[i] * ar[i] + ai[i] * ai[i]; }
    for (i = 0; i < N; i++) { wr_[i] = ar[i] / aa; wi_[i] = ai[i] / aa; }
    write_weights();
}

int gnss_smi_enable(int on)
{
    if (!gnss_smi_available()) { return -1; }
    if (on && !sm.enabled) {
        ant_sn2 = 0.0; tr_noise = 0.0; jh_n = 0; sm.jtype = GNSS_JT_NONE; sm.sweep_mode = 0;   /* MOD7 */
        write_conventional();
        sm.seq = RD(GNSS_SMI_REG_STAT) >> 16;  /* solve the next full window */
        have_v = 0;
    }
    sm.enabled = on ? 1 : 0;
    WR(GNSS_SMI_REG_CTRL, ctrl_word(0));
    return 0;
}
int gnss_smi_is_enabled(void) { return sm.enabled; }

int gnss_smi_set_kexp(uint32_t k)
{
    if (k < 6U || k > 12U) { return -1; }
    sm.kexp = k;
    kexp_user = 1;
    have_avg = 0;                              /* MOD-31 */
    if (gnss_smi_available()) { WR(GNSS_SMI_REG_CTRL, ctrl_word(0)); }
    return 0;
}
uint32_t gnss_smi_get_kexp(void) { return sm.kexp; }
int gnss_smi_set_alpha(double a)
{
    if (!(a >= 0.5 && a <= 1000.0)) { return -1; }
    sm.alpha = a;
    return 0;
}
double gnss_smi_get_alpha(void) { return sm.alpha; }

/* ------------------------------------------------------------- steering */
void gnss_smi_steer_off(void)
{
    sm.steer = 0;  sm.steer_theta = 999.0;  sm.steer_phi_deg = 0.0;
    set_a();
}

void gnss_smi_steer_phase(double phi_deg)
{
    sm.steer = 1;  sm.steer_phi_deg = phi_deg;
    set_a();
}

/* phi (RX2 - RX1) of a source at theta from broadside, using the jammer
 * estimator's K = lambda/(2d) and its phase calibration, so that a source
 * that gnss_jam? reports at angle theta is steered to by theta. */
int gnss_smi_steer_theta(double theta_deg)
{
    double k, cal, psi;
    uint32_t kq = RD(GNSS_PT_REG_JAM_K) & 0xFFFFU;
    int16_t  cb = (int16_t)(RD(GNSS_PT_REG_JAM_CAL) & 0xFFFFU);
    int      inv = (RD(GNSS_PT_REG_JAM_CTRL) & GNSS_PT_JAM_CTRL_INV) ? 1 : 0;

    if (theta_deg < -90.0 || theta_deg > 90.0) { return -1; }
    k   = (kq ? (double)kq : 16384.0) / 16384.0;          /* lambda / (2d) */
    cal = (double)cb * PI_D / 32768.0;
    psi = PI_D * sin(theta_deg * PI_D / 180.0) / k + cal; /* jammer_est psi_raw */
    if (inv) { psi = -psi; }                              /* psi_raw = +arg(r12) */
    psi = atan2(sin(psi), cos(psi));
    gnss_smi_steer_phase(psi * 180.0 / PI_D);
    sm.steer_theta = theta_deg;
    return 0;
}

int gnss_smi_steer_azel(double az_deg, double el_deg, double bhdg_deg)
{
    double s;
    if (el_deg < 0.0 || el_deg > 90.0) { return -1; }
    /* cone angle: sin(theta) = cos(el) cos(az - bearing of the +90 deg end) */
    s = cos(el_deg * PI_D / 180.0) * cos((az_deg - bhdg_deg) * PI_D / 180.0);
    if (s > 1.0) { s = 1.0; }
    if (s < -1.0) { s = -1.0; }
    return gnss_smi_steer_theta(asin(s) * 180.0 / PI_D);
}

/* ------------------------------------------------------- MOD-31 averaging */
/* E <- E + beta (R - E), then R <- E. Restarted (E = R) on the first window,
 * when the trace steps by more than 1.5 dB either way (jammer on / off,
 * RX gain step, gnss_spec / gnss_bist gain changes) and when the layout or
 * averaging changes. keff = K / sum(w^2) of the EMA weights. */
static void smi_average(double k)
{
    double tr = 0.0, b;
    int i, j;

    for (i = 0; i < N; i++) { tr += Rr[i][i]; }
    if (sm.avg_sh <= 0) { have_avg = 0; sm.keff = k; return; }
    if (!have_avg || !(avg_tr > 0.0) ||
        (!sm.sweep_mode && (tr > GNSS_SMI_AVG_STEP * avg_tr ||        /* MOD7: a sweep in/out of */
         (!sm.pulse_mode && tr * GNSS_SMI_AVG_STEP < avg_tr)))) {     /* band is no step; MOD6: pulse end */
        for (i = 0; i < N; i++) {
            for (j = 0; j < N; j++) { Er[i][j] = Rr[i][j]; Ei[i][j] = Ri[i][j]; }
        }
        if (have_avg) { sm.avg_resets++; }
        have_avg = 1;
        avg_s2 = 1.0;
        t_avg_reset = gnss_time_now();
        sm.avg_steady = 0;
    } else {
        int sh = sm.avg_sh;
        sm.avg_steady = (sm.avg_sh_st > sm.avg_sh &&
                         tus(t_avg_reset, gnss_time_now()) >= GNSS_SMI_AVG_HOLD_US) ? 1 : 0;
        if (sm.avg_steady) { sh = sm.avg_sh_st; }
        if (sm.sweep_mode && sm.sweep_sh > sh) { sh = sm.sweep_sh; sm.avg_steady = 1; }   /* MOD7 */
        b = 1.0 / (double)(1 << sh);
        for (i = 0; i < N; i++) {
            for (j = 0; j < N; j++) {
                Er[i][j] += b * (Rr[i][j] - Er[i][j]);
                Ei[i][j] += b * (Ri[i][j] - Ei[i][j]);
                Rr[i][j] = Er[i][j];
                Ri[i][j] = Ei[i][j];
            }
        }
        avg_s2 = (1.0 - b) * (1.0 - b) * avg_s2 + b * b;
    }
    avg_tr = 0.0;
    for (i = 0; i < N; i++) { avg_tr += Er[i][i]; }
    sm.keff = k / avg_s2;
}

int gnss_smi_set_avg(int sh)
{
    if (sh < 0 || sh > 4) { return -1; }
    sm.avg_sh = sh;
    sm.avg_sh_st = sh ? ((sh + 2 > 6) ? 6 : sh + 2) : 0;   /* steady: 4x slower */
    have_avg  = 0;
    return 0;
}
int gnss_smi_get_avg(void) { return sm.avg_sh; }

/* lambda_min of a K-snapshot sample covariance sits low by ~(1 - sqrt(N/K))^2
 * (Marchenko-Pastur edge). The reported noise is normalised to one raw
 * window (K = cnt), so the floor learned without a jammer (th_smi_off_db)
 * stays valid whatever the averaging does. */
static double lam_norm_db(double k)
{
    double a, b;
    if (!(k > (double)N) || !(sm.keff > (double)N)) { return 0.0; }
    a = 1.0 - sqrt((double)N / k);
    b = 1.0 - sqrt((double)N / sm.keff);
    return 20.0 * log10(a / b);
}

/* ------------------------------------------------------------------ poll */
static int64_t s48(uint32_t lo, uint32_t hi16)
{
    uint64_t v = ((uint64_t)(hi16 & 0xFFFFU) << 32) | (uint64_t)lo;
    if (v & (1ULL << 47)) { v |= 0xFFFF000000000000ULL; }
    return (int64_t)v;
}

int gnss_smi_poll(void)
{
    uint32_t st, seq, cnt, w0, w1, w2;
    int i, j, e, t;
    gnss_time_t t0, t1, t2, t3, t4, t5;

    if (!sm.enabled || !sm.present) { return 0; }
    st  = RD(GNSS_SMI_REG_STAT);
    sm.hw_active = (st & GNSS_SMI_ST_ACTIVE) ? 1 : 0;
    seq = st >> 16;
    if (seq == sm.seq) { return 0; }
    t0 = gnss_time_now();

    /* lock the published window while reading it */
    WR(GNSS_SMI_REG_CTRL, ctrl_word(1));
    for (t = 0; t < 200 && !(RD(GNSS_SMI_REG_STAT) & GNSS_SMI_ST_LOCK_ACK); t++) { UDELAY(1); }
    st  = RD(GNSS_SMI_REG_STAT);
    seq = st >> 16;
    cnt = RD(GNSS_SMI_REG_COUNT) & 0xFFFFU;
    if (!(st & GNSS_SMI_ST_LOCK_ACK) || cnt == 0U) {
        WR(GNSS_SMI_REG_CTRL, ctrl_word(0));
        return 0;
    }
    if (gain_skip) {                           /* MOD-33: window straddles a gain step */
        if (((seq - gain_seq) & 0xFFFFU) < 2U) {
            WR(GNSS_SMI_REG_CTRL, ctrl_word(0));
            sm.seq = seq;
            gain_skipped++;
            return 0;
        }
        gain_skip = 0;
    }
    e = 0;
    cur_page = 0xFFFFFFFFU;                    /* (re)write PAGE before the first entry */
    for (i = 0; i < N; i++) {
        for (j = i; j < N; j++, e++) {
            uint32_t a = cov_addr(e);
            w0 = RD(a);
            w1 = RD(a + 4U);
            w2 = RD(a + 8U);
            Rr[i][j] = (double)s48(w0, w2) / (double)cnt;
            Ri[i][j] = (double)s48(w1, w2 >> 16) / (double)cnt;
            Rr[j][i] = Rr[i][j];
            Ri[j][i] = -Ri[i][j];
        }
    }
    WR(GNSS_SMI_REG_CTRL, ctrl_word(0));
    t1 = gnss_time_now();
    if (mod6_window(t1)) {                     /* MOD6: pulse off -- keep the null */
        sm.seq = seq;
        sm.count = cnt;
        return 0;
    }
    smi_average((double)cnt);                  /* MOD-31 */
    t2 = gnss_time_now();
    have_win = 1;                              /* MOD-29: gnss_spec? / gnss_bist? */
    if (seq != ((sm.seq + 1U) & 0xFFFFU)) { sm.skipped++; }
    sm.seq   = seq;
    sm.count = cnt;

    set_a();
    if (mvdr_solve() != 0) {
        sm.fails++;
        return 1;
    }
    t3 = gnss_time_now();
    {
        int write_solved = 1;
        double jn = 99.0;
        if (sm.updates > 0) {                  /* MOD6: J/N + noise of this solve */
            gnss_null_t o;
            if (gnss_smi_null_analysis(&o) == 0) {
                jn = o.jn_in_db;
                sn2_last = pow(10.0, o.sn2_dbfs / 10.0) * X_PER_LSB2 * FS2_12;
            }
            /* MOD11: J/N above is against the spectral minimum of R (the
             * AD9361 floor at the band edges). Antenna noise shaped by the
             * antenna's own filter is coloured, so it reads as a "band-limited
             * jammer" there (bench, no jammer: 21 dB, and the null cut L1 by
             * 24 dB). Incoherent excess = noise: no jammer, clean hold. */
            coh_gate();
            if (sm.incoh && jn > -10.0) { jn = -10.0; sm.incoh_n++; }
            jn_last_db = jn;
            if (jn < -3.0 && tr_win > 0.0) {
                double a = tr_win / (double)N - floor_sn2;          /* MOD7: antenna part */
                if (a < 0.1 * tr_win / (double)N) { a = 0.1 * tr_win / (double)N; }
                tr_noise = (tr_noise > 0.0) ? tr_noise + (tr_win - tr_noise) / 64.0 : tr_win;
                ant_sn2 = (ant_sn2 > 0.0) ? ant_sn2 + (a - ant_sn2) / 64.0 : a;
            }
            if (jn < 3.0 && !(noise_ref() > 0.0 && tr_win > 4.0 * (double)N * noise_ref())) {
                sm.leak_ref_db = -99.0;
            }
        }
        if (clean_en && sm.updates > 0 && !sm.pulse_mode && !sm.sweep_mode) {
            if (!clean_hold) {
                if (jn < clean_in_db) {
                    if (clean_cnt_ok++ == 0) { clean_t0 = t3; }
                    if (tus(clean_t0, t3) >= 2.0e6) {
                        clean_hold = 1;
                        clean_enters++;
                        tr_clean = tr_win;
                        write_identity();
                        write_solved = 0;
                    }
                } else {
                    clean_cnt_ok = 0;
                }
            } else if (jn > clean_out_db || (tr_clean > 0.0 && tr_win > 2.0 * tr_clean)) {
                clean_hold = 0;               /* jammer (or +3 dB of power): null from this window on */
                clean_cnt_ok = 0;
            } else {
                write_solved = 0;             /* hold the fixed reference tap */
                if (tr_win > 0.0) { tr_clean += (tr_win - tr_clean) / 64.0; }
            }
        } else {
            clean_hold = 0;
        }
        if (write_solved) { write_weights(); }
    }
    t4 = gnss_time_now();
    sm.updates++;
    {
        double tr = 0.0, por = 0.0, poi = 0.0;
        for (i = 0; i < N; i++) { tr += Rr[i][i]; }
        for (i = 0; i < N; i++) {              /* w^H R w */
            double tr_ = 0.0, ti_ = 0.0;
            for (j = 0; j < N; j++) {
                tr_ += Rr[i][j] * wr_[j] - Ri[i][j] * wi_[j];
                ti_ += Rr[i][j] * wi_[j] + Ri[i][j] * wr_[j];
            }
            por += wr_[i] * tr_ + wi_[i] * ti_;
            poi += wr_[i] * ti_ - wi_[i] * tr_;
        }
        (void)poi;
        sm.pin_dbfs    = 10.0 * log10(tr / N / X_PER_LSB2 / FS2_12 + 1e-30);
        sm.lam_dbfs    = 10.0 * log10(sm.lam_min / X_PER_LSB2 / FS2_12 + 1e-30) + lam_norm_db((double)cnt);
        sm.pout_rel_db = 10.0 * log10((por > 1e-30 ? por : 1e-30) / (Rr[REF][REF] > 1e-30 ? Rr[REF][REF] : 1e-30));
        sm.pin_dbfs_ref = 10.0 * log10(Rr[REF][REF] / X_PER_LSB2 / FS2_12 + 1e-30);
        sm.p2_dbfs_ref  = 10.0 * log10(Rr[nL + REF][nL + REF] / X_PER_LSB2 / FS2_12 + 1e-30);
        sm.pout_dbfs    = 10.0 * log10((por > 1e-30 ? por : 1e-30) / X_PER_LSB2 / FS2_12);
    }
    window_figures();
    if (jn_last_db > 6.0) {                    /* MOD6: stable class for the AGC */
        wide_score += (((sm.jbw_class >= 1) ? 1.0 : 0.0) - wide_score) / 128.0;
        if (!sm.jbw_wide && wide_score > 0.7) { sm.jbw_wide = 1; }
        else if (sm.jbw_wide && wide_score < 0.3) { sm.jbw_wide = 0; }
    } else if (jn_last_db < 3.0) {
        wide_score = 0.5;
        sm.jbw_wide = 0;
    }
    gnss_rep_on_solve();
    t5 = gnss_time_now();
    sm.t_read_us  = tus(t0, t1);
    sm.t_avg_us   = tus(t1, t2);
    sm.t_solve_us = tus(t2, t3);
    sm.t_write_us = tus(t3, t4);
    sm.t_post_us  = tus(t4, t5);
    sm.t_total_us = tus(t0, t5);
    if (sm.t_total_us > sm.t_max_us) { sm.t_max_us = sm.t_total_us; }
    if (t_last_solve) {
        double d = tus(t_last_solve, t4);
        sm.t_int_us = (sm.t_int_us > 0.0 && d < 1e6) ? sm.t_int_us + (d - sm.t_int_us) / 16.0 : d;
    }
    t_last_solve = t4;
    return 1;
}

/* MOD-26: what the window says about the strongest signal (the jammer when
 * there is one) and about the weights. From the RX1/RX2 tap correlations:
 *   r(k) = mean over the taps of R[i][i+k]  (= E x(n) x*(n-k))
 *   frequency  = arg r(1) / 2pi x fs
 *   bandwidth  from |r(k)| / (r(0) - noise): a rectangular band B gives
 *              |sinc(B k / fs)|; CW stays ~1 over all 16 lags */
static double sinc_inv(double rho)            /* x in [0,1] with sinc(x) = rho */
{
    double lo = 0.0, hi = 1.0, x, v;
    int it;
    for (it = 0; it < 30; it++) {
        x = 0.5 * (lo + hi);
        v = sin(PI_D * x) / (PI_D * x);
        if (v > rho) { lo = x; } else { hi = x; }
    }
    return 0.5 * (lo + hi);
}

static void window_figures(void)
{
    double r1r = 0.0, r1i = 0.0, p0 = 0.0, nz, fs = (double)gnss_pt_get_fs();
    double rho[GNSS_SMI_LMAX], wm = -1.0, a;
    int i, k, ant, kfirst = 0;

    sm.wnorm_u = 0.0;
    for (i = 0; i < N; i++) { sm.wnorm_u += wr_[i] * wr_[i] + wi_[i] * wi_[i]; }
    for (k = 0; k < nL; k++) {
        a = sqrt(wr_[nL + k] * wr_[nL + k] + wi_[nL + k] * wi_[nL + k]);
        if (a > wm) { wm = a; sm.w2max_k = k; }
    }
    sm.w2max = wm;
    /* reference-tap coherence and phase between the antennas */
    {
        double xr = Rr[REF][nL + REF], xi = Ri[REF][nL + REF];     /* x1 conj(x2) */
        double d  = Rr[REF][REF] * Rr[nL + REF][nL + REF];
        sm.coh12    = (d > 0.0) ? (xr * xr + xi * xi) / d : 0.0;
        sm.ph21_deg = atan2(-xi, xr) * 180.0 / PI_D;               /* arg(x2 conj x1) */
    }
    /* lag correlations, both antennas */
    for (k = 0; k < nL; k++) {
        double sr = 0.0, si = 0.0;
        int n = 0;
        for (ant = 0; ant < 2; ant++) {
            for (i = 0; i + k < nL; i++, n++) {
                sr += Rr[ant * nL + i][ant * nL + i + k];
                si += Ri[ant * nL + i][ant * nL + i + k];
            }
        }
        sr /= (double)n;  si /= (double)n;
        if (k == 0) { p0 = sr; }
        if (k == 1) { r1r = sr; r1i = si; }
        rho[k] = sqrt(sr * sr + si * si);
    }
    nz = sm.lam_min;                               /* noise per tap (a little low) */
    sm.jf_hz = (fs > 0.0) ? atan2(r1i, r1r) / (2.0 * PI_D) * fs : 0.0;
    if (p0 - nz <= 0.0) {
        sm.jbw_class = 2;  sm.jbw_hz = fs;
        return;
    }
    for (k = 1; k < nL; k++) {
        rho[k] /= (p0 - nz);
        if (rho[k] > 1.0) { rho[k] = 1.0; }
        if (!kfirst && rho[k] < 0.5) { kfirst = k; }
    }
    if (rho[nL - 1] >= 0.8) {                       /* flat over all lags: CW-like */
        sm.jbw_class = 0;
        sm.jbw_hz = sinc_inv(rho[nL - 1]) * fs / (double)(nL - 1);
    } else if (kfirst == 1) {                       /* gone after one lag */
        sm.jbw_class = 2;
        sm.jbw_hz = sinc_inv(rho[1]) * fs;
    } else {
        k = kfirst ? kfirst : nL - 1;
        sm.jbw_class = 1;
        sm.jbw_hz = sinc_inv(rho[k]) * fs / (double)k;
    }
}

/* MOD-29: lag correlations of one antenna in the last window read,
 *   r(k) = E x(n) x*(n-k), k = 0..L-1, averaged over the tap pairs,
 * in 12-bit LSB^2 per sample (0 dBFS = 2048^2). Returns L, 0 if no window. */
int gnss_smi_lag_corr(int ant, double *rr, double *ri)
{
    int k, i, b;
    if (!have_win || ant < 0 || ant > 1) { return 0; }
    b = ant * nL;
    for (k = 0; k < nL; k++) {
        double sr = 0.0, si = 0.0;
        for (i = 0; i + k < nL; i++) {
            sr += Rr[b + i][b + i + k];
            si += Ri[b + i][b + i + k];
        }
        rr[k] = sr / (double)(nL - k) / X_PER_LSB2;
        ri[k] = si / (double)(nL - k) / X_PER_LSB2;
    }
    return nL;
}

/* MOD-29: power spectral density from an L x L Hermitian sample covariance
 * (row-major re/im), as a Hann-tapered periodogram averaged over the window's
 * snapshots:  S(f) = a^H R a / sum h^2,  a_k = h_k exp(-j 2 pi f k).
 * A quadratic form of a covariance matrix is never negative; Hann sidelobes
 * keep a strong spur from leaking into the other bins. f = frequency / fs in
 * [-0.5, 0.5). The mean of S over the band equals the power per sample, so
 * S / fs is the density per Hz. */
double gnss_herm_psd(int L, const double *ar, const double *ai, double f)
{
    double hr[GNSS_SMI_LMAX], hi[GNSS_SMI_LMAX], h2 = 0.0, p = 0.0, w = 2.0 * PI_D * f;
    int i, j;
    if (L < 2 || L > GNSS_SMI_LMAX) { return 0.0; }
    for (i = 0; i < L; i++) {
        double h = 0.5 - 0.5 * cos(2.0 * PI_D * (i + 0.5) / L);
        hr[i] = h * cos(w * i);  hi[i] = -h * sin(w * i);
        h2 += h * h;
    }
    for (i = 0; i < L; i++) {
        double yr = 0.0, yi = 0.0;
        for (j = 0; j < L; j++) {
            yr += ar[i * L + j] * hr[j] - ai[i * L + j] * hi[j];
            yi += ar[i * L + j] * hi[j] + ai[i * L + j] * hr[j];
        }
        p += hr[i] * yr + hi[i] * yi;          /* Re(conj(a_i) y_i) */
    }
    p /= h2;
    return p > 0.0 ? p : 0.0;
}
/* the same on one antenna's block of the last SMI window, 12-bit LSB^2 */
double gnss_smi_psd(int ant, double f)
{
    static double ar[GNSS_SMI_LMAX * GNSS_SMI_LMAX], ai[GNSS_SMI_LMAX * GNSS_SMI_LMAX];
    int i, j, b;
    if (!have_win || ant < 0 || ant > 1) { return 0.0; }
    b = ant * nL;
    for (i = 0; i < nL; i++) {
        for (j = 0; j < nL; j++) {
            ar[i * nL + j] = Rr[b + i][b + j] / X_PER_LSB2;
            ai[i * nL + j] = Ri[b + i][b + j] / X_PER_LSB2;
        }
    }
    return gnss_herm_psd(nL, ar, ai, f);
}

/* MOD-29: noise power per tap under at most two sinusoids (a BIST tone and its
 * IQ image), from an L x L Hermitian sample covariance A (row-major, re/im):
 *   noise = (trace(A) - lambda1 - lambda2) / (L - 2)
 * lambda1, lambda2 = the two largest eigenvalues (power iteration + deflation).
 * The trace is exact, so only the two strong directions are removed: unlike a
 * predictor on averaged lags, this stays accurate with a tone 60+ dB over the
 * noise. Also returns lambda1 / L (the tone power per tap) in *tone. */
static double top_eig(int L, double *ar, double *ai, double *vr, double *vi)
{
    double ur[GNSS_SMI_LMAX], ui[GNSS_SMI_LMAX], nrm = 0.0, lam = 0.0;
    int i, j, it;
    for (i = 0; i < L; i++) { vr[i] = 1.0 + 0.1 * i; vi[i] = 0.05 * i; }
    for (it = 0; it < 80; it++) {
        nrm = 0.0;
        for (i = 0; i < L; i++) {
            double sr = 0.0, si = 0.0;
            for (j = 0; j < L; j++) {
                sr += ar[i * L + j] * vr[j] - ai[i * L + j] * vi[j];
                si += ar[i * L + j] * vi[j] + ai[i * L + j] * vr[j];
            }
            ur[i] = sr; ui[i] = si; nrm += sr * sr + si * si;
        }
        nrm = sqrt(nrm);
        if (!(nrm > 0.0)) { return 0.0; }
        for (i = 0; i < L; i++) { vr[i] = ur[i] / nrm; vi[i] = ui[i] / nrm; }
    }
    for (i = 0; i < L; i++) {                       /* Rayleigh quotient v^H A v */
        double sr = 0.0, si = 0.0;
        for (j = 0; j < L; j++) {
            sr += ar[i * L + j] * vr[j] - ai[i * L + j] * vi[j];
            si += ar[i * L + j] * vi[j] + ai[i * L + j] * vr[j];
        }
        lam += vr[i] * sr + vi[i] * si;
    }
    return lam;
}
double gnss_herm_tone_noise(int L, double *ar, double *ai, double *tone)
{
    double vr[GNSS_SMI_LMAX], vi[GNSS_SMI_LMAX], tr = 0.0, l1, l2, n;
    int i, j;
    if (L < 4 || L > GNSS_SMI_LMAX) { return -1.0; }
    for (i = 0; i < L; i++) { tr += ar[i * L + i]; }
    l1 = top_eig(L, ar, ai, vr, vi);
    for (i = 0; i < L; i++) {                       /* deflate: A - l1 v v^H */
        for (j = 0; j < L; j++) {
            ar[i * L + j] -= l1 * (vr[i] * vr[j] + vi[i] * vi[j]);
            ai[i * L + j] -= l1 * (vi[i] * vr[j] - vr[i] * vi[j]);
        }
    }
    l2 = top_eig(L, ar, ai, vr, vi);
    if (tone) { *tone = l1 / L; }
    n = (tr - l1 - l2) / (double)(L - 2);
    return n > 0.0 ? n : 0.0;
}
/* the same on one antenna's block of the last SMI window, 12-bit LSB^2 */
double gnss_smi_tone_noise(int ant, double *tone)
{
    static double ar[GNSS_SMI_LMAX * GNSS_SMI_LMAX], ai[GNSS_SMI_LMAX * GNSS_SMI_LMAX];
    double n, t = 0.0;
    int i, j, b;
    if (!have_win || ant < 0 || ant > 1) { return -1.0; }
    b = ant * nL;
    for (i = 0; i < nL; i++) {
        for (j = 0; j < nL; j++) {
            ar[i * nL + j] = Rr[b + i][b + j] / X_PER_LSB2;
            ai[i * nL + j] = Ri[b + i][b + j] / X_PER_LSB2;
        }
    }
    n = gnss_herm_tone_noise(nL, ar, ai, &t);
    if (tone) { *tone = t; }
    return n;
}

double gnss_smi_response_db(double f_off_hz, double fs_hz)
{
    double h1r = 0.0, h1i = 0.0, h2r = 0.0, h2i = 0.0, w, c, sn;
    int k;
    if (!sm.updates || fs_hz <= 0.0) { return -99.0; }
    w = 2.0 * PI_D * f_off_hz / fs_hz;
    for (k = 0; k < nL; k++) {                     /* sum conj(w_k) e^{-j w k} */
        c = cos(w * k);  sn = -sin(w * k);
        h1r += wr_[k] * c + wi_[k] * sn;          h1i += wr_[k] * sn - wi_[k] * c;
        h2r += wr_[nL + k] * c + wi_[nL + k] * sn; h2i += wr_[nL + k] * sn - wi_[nL + k] * c;
    }
    w = h1r * h1r + h1i * h1i + h2r * h2r + h2i * h2i;
    return 10.0 * log10(w > 1e-12 ? w : 1e-12);
}

/* ---------------------------------------------------- null analysis (LVDS) */
int gnss_smi_null_analysis(gnss_null_t *o)
{
    double keff = (sm.keff > (double)N) ? sm.keff : (double)(1UL << (sm.kexp ? sm.kexp : 8));
    int    na   = (sm.Na > 0 && sm.Na <= N) ? sm.Na : N;
    double lmin = sm.lam_min;
    double bias = 1.0 - sqrt((double)N / keff);       /* lambda_min of the solved R sits low */
    double sn2, jin, jout = 0.0, wn = 0.0, flo, rho, rp, beta;
    int i, j;

    memset(o, 0, sizeof(*o));
    if (!have_win || !sm.updates || !(sm.lam_min > 0.0) || N <= 0) { return -1; }
    if (bias < 0.05) { bias = 0.05; }
    if (na < N) {                                     /* taps < L: noise from all taps */
        double lf = lam_min_full();
        if (lf > 0.0) { lmin = lf; }
    } else {
        bias = 1.0 - sqrt((double)na / keff);
        if (bias < 0.05) { bias = 0.05; }
    }
    sn2 = lmin / (bias * bias);                       /* noise per element */
    for (i = 0; i < N; i++) {
        double tr = 0.0, ti = 0.0;
        wn += wr_[i] * wr_[i] + wi_[i] * wi_[i];
        for (j = 0; j < N; j++) {                       /* (Rj w)_i */
            double rr = Rr[i][j] - (i == j ? sn2 : 0.0), ri = Ri[i][j];
            tr += rr * wr_[j] - ri * wi_[j];
            ti += rr * wi_[j] + ri * wr_[j];
        }
        jout += wr_[i] * tr + wi_[i] * ti;              /* Re(conj(w_i) (Rj w)_i) */
    }
    jin = (Rr[REF][REF] - sn2) / sn2;                   /* (J/N)_in  */
    jout = jout / (sn2 * wn);                           /* (J/N)_out */
    o->sn2_dbfs = 10.0 * log10(sn2 / X_PER_LSB2 / FS2_12 + 1e-30);
    o->jn_in_db = 10.0 * log10(jin > 1e-6 ? jin : 1e-6);
    flo = 3.0 / sqrt(keff);                             /* measurable (J/N)_out */
    if (jout < flo) { jout = flo; o->supp_lo = 1; }
    o->jn_out_db = 10.0 * log10(jout);
    o->supp_db = o->jn_in_db - o->jn_out_db;
    if (o->supp_db < 0.0) { o->supp_db = 0.0; }
    o->cn0_loss_db = o->supp_lo ? 0.0 : 10.0 * log10(1.0 + jout);
    rho = jin > 0.0 ? jin : 0.0;                         /* closed form, M = 2 */
    (void)na;
    rp = rho / (1.0 + sm.alpha);
    beta = rp / (1.0 + 2.0 * rp);
    o->sth2_db = 10.0 * log10((1.0 + 2.0 * rp) * (1.0 + 2.0 * rp) *
                              ((1.0 - beta) * (1.0 - beta) + beta * beta));
    o->smi_loss_db = (keff > na) ? -10.0 * log10((keff - na + 2.0) / (keff + 1.0)) : 99.0;
    {
        double xr = Rr[REF][nL + REF], xi = Ri[REF][nL + REF];
        double d = Rr[REF][REF] - sn2, p2 = Rr[nL + REF][nL + REF] - sn2;
        o->amp_db = (d > 0.0 && p2 > 0.0) ? 10.0 * log10(p2 / d) : 0.0;
        o->ph_deg = atan2(-xi, xr) * 180.0 / PI_D;
    }
    o->valid = (o->jn_in_db > 1.0);
    return 0;
}

static uint32_t novr_base = 0;
void gnss_smi_mark_novr(void)
{
    if (gnss_smi_available()) { novr_base = RD(GNSS_SMI_REG_NOVR) & 0xFFFFU; }
}

void gnss_smi_get_state(gnss_smi_state_t *s)
{
    if (!s) { return; }
    if (sm.present) {
        uint32_t st = RD(GNSS_SMI_REG_STAT);
        sm.hw_active = (st & GNSS_SMI_ST_ACTIVE) ? 1 : 0;
        sm.novr  = RD(GNSS_SMI_REG_NOVR) & 0xFFFFU;
        sm.novr_new = (sm.novr - novr_base) & 0xFFFFU;
        sm.nload = RD(GNSS_SMI_REG_NLOAD) & 0xFFFFU;
    }
    *s = sm;
}

/* --------------------------------------------------------------- report */
static void pf(double v, int dec)
{
    long m = (long)pow(10.0, dec);
    long x = (long)floor(fabs(v) * (double)m + 0.5);
    if (dec > 0) { printf("%s%ld.%0*ld", (v < 0.0) ? "-" : "", x / m, dec, x % m); }
    else         { printf("%s%ld", (v < 0.0) ? "-" : "", x); }
}

void gnss_smi_print(void)
{
    gnss_smi_state_t s;
    if (!gnss_smi_available()) {
        printf("SMI-STAP: not in this bitstream (MVDR window < 1.14).\n");
        return;
    }
    gnss_smi_get_state(&s);
    printf("SMI-STAP (2 antennas x %d taps in use of %d + 2 image taps = %d, sample-matrix inversion on the ARM)\n",
           s.La, s.L, s.Na);
    printf("  layout      : INFO 0x%08lX  v%d, reference RX1 tap %d (%d newer + %d older taps)\n",
           (unsigned long)s.info, s.layout, s.ref, s.ref, s.L - 1 - s.ref);
    printf("  output      : %s\n", s.hw_active ? "SMI drives the output" :
                                   s.enabled ? "enabled, not active yet" : "off (gnss_null=3 to use it)");
    printf("  constraint  : ");
    if (s.steer) {
        printf("STEERED, RX2-RX1 phase "); pf(s.steer_phi_deg, 1);
        if (s.steer_theta < 900.0) { printf(" deg (theta "); pf(s.steer_theta, 1); printf(" deg)\n"); }
        else                       { printf(" deg (set directly)\n"); }
    } else {
        printf("power inversion (unity gain on RX1 tap %d; every direction passes)\n", REF);
    }
    printf("  window      : 2^%lu snapshots (%lu in the last)   loading ",
           (unsigned long)s.kexp, (unsigned long)s.count);
    pf(s.alpha, 2); printf(" x lambda_min\n");
    printf("  solved      : %lu weight sets, %lu windows skipped while busy, %lu failed\n",
           (unsigned long)s.updates, (unsigned long)s.skipped, (unsigned long)s.fails);
    if (s.avg_sh > 0) {                        /* MOD-31 */
        printf("  averaging   : covariance over windows, beta 1/%d (gnss_smiavg=%d); ~%lu snapshots solved,\n"
               "                restarted %lu times on a power step (jammer on/off, gain step)\n",
               1 << s.avg_sh, s.avg_sh, (unsigned long)(s.keff + 0.5), (unsigned long)s.avg_resets);
    } else {
        printf("  averaging   : off (gnss_smiavg=0): every window solved on its own\n");
    }
    if (s.updates) {
        printf("  input/tap   : "); pf(s.pin_dbfs, 1);
        printf(" dBFS    noise (lambda_min): "); pf(s.lam_dbfs, 1); printf(" dBFS\n");
        printf("  output      : "); pf(s.pout_rel_db, 1);
        printf(" dB vs RX1 input  (jammer + noise removed down to the floor)\n");
        printf("  sum |w|^2   : "); pf(s.wnorm, 3); printf("  (noise cost ");
        pf(10.0 * log10(s.wnorm > 1e-9 ? s.wnorm : 1e-9), 1);
        printf(" dB)   |w_ref|^2 "); pf(s.ref_gain, 3);
        printf("   scale 2^-%d\n", s.shift);
        printf("  image taps  : |w|^2 "); pf(s.img_gain, 4);
        printf("  (cancel the AD9361 IQ image of the jammer; ~0 = no image seen)\n");
    }
    printf("  DC removal  : %s (FPGA, at the SMI input)\n", s.dc_off ? "OFF" : "on");
    printf("  hardware    : weight sets applied %lu, sample overruns %lu since start%s\n",
           (unsigned long)s.nload, (unsigned long)s.novr_new,
           s.novr_new ? "  <-- needs >= 4 clocks per sample (2R2T)" : "");
    printf("                (%lu in total, including the radio set-up at boot)\n", (unsigned long)s.novr);
}

/* ------------------------------------------------------------- selftest */
/* Synthetic R: noise sigma^2 on every tap + a CW jammer (RX2 = 0.9 e^{j0.7}
 * RX1, 0.13 cycles/sample) at 40 dB. Checks: a^H w = 1 and the output
 * power ~ the noise. */
int gnss_smi_selftest(int verbose)
{
    double xr[NM], xi[NM], jr, ji, p = 0.0, cr, ci, por = 0.0;
    double sig2 = 100.0, J = 1e6, f = 2.0 * PI_D * 0.13;
    int i, j, ok;
    gnss_smi_state_t save = sm;

    for (i = 0; i < nL; i++) {                 /* jammer space-time vector */
        xr[i] = cos(-f * i);  xi[i] = sin(-f * i);
        xr[nL + i] = 0.9 * cos(-f * i + 0.7);
        xi[nL + i] = 0.9 * sin(-f * i + 0.7);
    }
    /* image taps: conj of RX1 / RX2 at the reference delay */
    xr[2 * nL]     = xr[REF];       xi[2 * nL]     = -xi[REF];
    xr[2 * nL + 1] = xr[nL + REF];  xi[2 * nL + 1] = -xi[nL + REF];
    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            Rr[i][j] = J * (xr[i] * xr[j] + xi[i] * xi[j]) + (i == j ? sig2 : 0.0);
            Ri[i][j] = J * (xi[i] * xr[j] - xr[i] * xi[j]);
        }
    }
    sm.alpha = GNSS_SMI_ALPHA_DEFAULT;
    sm.steer = 0;
    have_v = 0;
    set_a();
    if (mvdr_solve() != 0) { sm = save; return -1; }
    cr = 0.0; ci = 0.0;                        /* a^H w */
    for (i = 0; i < N; i++) {
        cr += ar[i] * wr_[i] + ai[i] * wi_[i];
        ci += ar[i] * wi_[i] - ai[i] * wr_[i];
        p  += wr_[i] * xr[i] + wi_[i] * xi[i]; /* jammer response re(w^H x) */
    }
    jr = 0.0; ji = 0.0;
    for (i = 0; i < N; i++) {
        jr += wr_[i] * xr[i] + wi_[i] * xi[i];
        ji += wr_[i] * xi[i] - wi_[i] * xr[i];
    }
    for (i = 0; i < N; i++) {                  /* w^H R w */
        double tr_ = 0.0, ti_ = 0.0;
        for (j = 0; j < N; j++) {
            tr_ += Rr[i][j] * wr_[j] - Ri[i][j] * wi_[j];
            ti_ += Rr[i][j] * wi_[j] + Ri[i][j] * wr_[j];
        }
        por += wr_[i] * tr_ + wi_[i] * ti_;
    }
    ok = (fabs(cr - 1.0) < 1e-9 && fabs(ci) < 1e-9 &&
          10.0 * log10(J * (jr * jr + ji * ji) / sig2 + 1e-30) < -30.0 &&
          10.0 * log10(por / sig2) < 0.5);
    if (verbose) {
        printf("SMI selftest: lambda_min "); pf(sm.lam_min, 2);
        printf(" (noise %ld), a^H w = ", (long)sig2); pf(cr, 6);
        printf(", jammer response "); pf(10.0 * log10(J * (jr * jr + ji * ji) / sig2 + 1e-30), 1);
        printf(" dB vs noise, output "); pf(10.0 * log10(por / sig2), 2);
        printf(" dB vs noise -> %s\n", ok ? "PASS" : "FAIL");
    }
    (void)p;
    sm = save;
    have_v = 0;
    return ok ? 0 : -1;
}
