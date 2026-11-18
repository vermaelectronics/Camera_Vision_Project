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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "gnss_smi.h"

#ifndef GNSS_SMI_HOST
#include "gnss_passthrough.h"
#include "no_os_delay.h"
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
    .avg_sh = GNSS_SMI_AVG_DEFAULT, .keff = 0.0,
};
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
static double  avg_tr = 0.0, avg_s2 = 1.0;    /* trace of E; sum of squared EMA weights */

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
static void set_layout(int v, int l, int r)
{
    lay = v;  nL = l;  N = 2 * l + GNSS_SMI_NI;  REF = r;
    sm.layout = v;  sm.L = l;  sm.N = N;  sm.ref = r;
    if (sm.La <= 0 || sm.La > l) { sm.La = l; }
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
    if (!have_avg || !(avg_tr > 0.0) || tr > GNSS_SMI_AVG_STEP * avg_tr || tr * GNSS_SMI_AVG_STEP < avg_tr) {
        for (i = 0; i < N; i++) {
            for (j = 0; j < N; j++) { Er[i][j] = Rr[i][j]; Ei[i][j] = Ri[i][j]; }
        }
        if (have_avg) { sm.avg_resets++; }
        have_avg = 1;
        avg_s2 = 1.0;
    } else {
        b = 1.0 / (double)(1 << sm.avg_sh);
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

void gnss_smi_poll(void)
{
    uint32_t st, seq, cnt, w0, w1, w2;
    int i, j, e, t;

    if (!sm.enabled || !sm.present) { return; }
    st  = RD(GNSS_SMI_REG_STAT);
    sm.hw_active = (st & GNSS_SMI_ST_ACTIVE) ? 1 : 0;
    seq = st >> 16;
    if (seq == sm.seq) { return; }

    /* lock the published window while reading it */
    WR(GNSS_SMI_REG_CTRL, ctrl_word(1));
    for (t = 0; t < 200 && !(RD(GNSS_SMI_REG_STAT) & GNSS_SMI_ST_LOCK_ACK); t++) { UDELAY(1); }
    st  = RD(GNSS_SMI_REG_STAT);
    seq = st >> 16;
    cnt = RD(GNSS_SMI_REG_COUNT) & 0xFFFFU;
    if (!(st & GNSS_SMI_ST_LOCK_ACK) || cnt == 0U) {
        WR(GNSS_SMI_REG_CTRL, ctrl_word(0));
        return;
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
    smi_average((double)cnt);                  /* MOD-31 */
    have_win = 1;                              /* MOD-29: gnss_spec? / gnss_bist? */
    if (seq != ((sm.seq + 1U) & 0xFFFFU)) { sm.skipped++; }
    sm.seq   = seq;
    sm.count = cnt;

    set_a();
    if (mvdr_solve() != 0) {
        sm.fails++;
        return;
    }
    write_weights();
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
    gnss_rep_on_solve();
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
