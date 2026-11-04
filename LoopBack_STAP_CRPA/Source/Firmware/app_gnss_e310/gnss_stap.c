/******************************************************************************
 *  gnss_stap.c
 *  Driver for the CRPA anti-jam stage. See gnss_stap.h.
 *  Original work for the ANTSDR E310 V1 GNSS CRPA pipeline.
 *
 *  ADAPTATION LOOP (one "update")
 *    1. CORR_START: the PL captures CORR_LEN + 30 samples of RX1/RX2 and
 *       computes 3 channel pairs x 31 lags of correlation (about 1 ms at
 *       30.72 MSPS for CORR_LEN = 5000).
 *    2. Read the 93 correlations and 2 x 30 edge samples per channel.
 *    3. stap_build_cov(): exact 32x32 covariance R = Xm*Xm'/L.
 *    4. Power inversion  w = R^-1 e0 / (e0' R^-1 e0)   (default), or
 *       MVDR             w = R^-1 s  / (s'  R^-1 s)    (stap_mode=1).
 *    5. Quantise to 18 bits, write the shadow bank, W_COMMIT: the PL switches
 *       to the new weights on a sample boundary, with no gap in the output.
 *
 *  BACKGROUND OPERATION
 *    gnss_stap_service() is non-blocking. console.c calls it while it waits
 *    for keyboard input, so with stap_auto=<ms> the weights keep tracking the
 *    jammers while the console stays usable.
 *****************************************************************************/

#include "gnss_stap.h"
#include "no_os_axi_io.h"
#include "no_os_delay.h"
#include <stdio.h>
#include <math.h>

#ifdef XILINX_PLATFORM
#include "xtime_l.h"
#define STAP_HAVE_TIME 1
#endif

#define STAP_WW            18          /* weight width in the PL           */
#define STAP_OW            12          /* output width in the PL           */
#define STAP_FS_POWER      (2048.0 * 2048.0)   /* 12-bit full-scale |x|^2  */
#define STAP_DONE_TIMEOUT  200000U     /* status polls before giving up    */

gnss_stap_state_t gnss_stap_st = {
    .present        = 0,
    .mode           = GNSS_STAP_MODE_PI,
    .theta_deg      = 0.0,
    .d_over_lambda  = 0.5,
    .fif_over_fs    = 0.0,             /* AD9361 LO on L1: GPS at 0 Hz     */
    .corr_len       = 5000U,
    .headroom_bits  = 3,
    .auto_period_ms = 0U,
    .ch1_copy       = 0,
};

/* big buffers live in .bss, not on the stack */
static int64_t    s_cre[STAP_NRES], s_cim[STAP_NRES];
static stap_edges s_ed;
static stap_cplx  s_R[STAP_T][STAP_T];

/* ------------------------------------------------------------------------ */
uint32_t gnss_stap_read(uint32_t offset)
{
    uint32_t v = 0;
    no_os_axi_io_read(GNSS_STAP_BASEADDR, offset, &v);
    return v;
}

void gnss_stap_write(uint32_t offset, uint32_t value)
{
    no_os_axi_io_write(GNSS_STAP_BASEADDR, offset, value);
}

static uint32_t now_ms(void)
{
#ifdef STAP_HAVE_TIME
    XTime t;
    XTime_GetTime(&t);
    return (uint32_t)(t / (COUNTS_PER_SECOND / 1000U));
#else
    static uint32_t fake = 0;   /* no timer: one tick per service call */
    return fake++;
#endif
}

static int32_t cdb(double p, double ref)
{
    if (p <= 0.0) return -99900;
    return (int32_t)floor(1000.0 * log10(p / ref) + 0.5);
}

static void print_cdb(const char *label, int32_t v)
{
    int32_t a = (v < 0) ? -v : v;
    printf("%s%s%ld.%02ld dB", label, (v < 0) ? "-" : "",
           (long)(a / 100), (long)(a % 100));
}

static int lclk_alive(void)
{
    return (gnss_stap_read(GNSS_STAP_REG_WRAP_STAT) & 1U) ? 1 : 0;
}

static void write_user_ctrl(int aj_en)
{
    uint32_t uc = 0;
    if (aj_en) uc |= GNSS_STAP_UC_AJ_EN;
    if (gnss_stap_st.ch1_copy) uc |= GNSS_STAP_UC_CH1_COPY;
    gnss_stap_write(GNSS_STAP_REG_USER_CTRL, uc);
}

/* ------------------------------------------------------------------------ */
int32_t gnss_stap_probe(void)
{
    uint32_t wid, id, cfg, cfg2, cfg3;

    gnss_stap_st.present = 0;
    wid = gnss_stap_read(GNSS_STAP_REG_WRAP_ID);
    if (wid != GNSS_STAP_WRAP_ID) {
        printf("gnss_stap: not found at 0x%08lx (read 0x%08lx). The bitstream has no\n"
               "           CRPA stage; the board runs as a plain loopback.\n",
               (unsigned long)GNSS_STAP_BASEADDR, (unsigned long)wid);
        return GNSS_STAP_ERR_ABSENT;
    }
    if (!lclk_alive()) {
        printf("gnss_stap: AD9361 interface clock (l_clk) is not running.\n"
               "           Initialise the AD9361 first.\n");
        return GNSS_STAP_ERR_NO_CLK;
    }
    id   = gnss_stap_read(GNSS_STAP_REG_ID);
    cfg  = gnss_stap_read(GNSS_STAP_REG_CONFIG);
    cfg2 = gnss_stap_read(GNSS_STAP_REG_CONFIG2);
    cfg3 = gnss_stap_read(GNSS_STAP_REG_CONFIG3);
    if (id != GNSS_STAP_ID) {
        printf("gnss_stap: core ID 0x%08lx, expected 0x%08lx\n",
               (unsigned long)id, (unsigned long)GNSS_STAP_ID);
        return GNSS_STAP_ERR_ABSENT;
    }
    if ((cfg & 0xFFU) != STAP_M || ((cfg >> 8) & 0xFFU) != STAP_N ||
        ((cfg2 >> 8) & 0xFFU) != STAP_OW) {
        printf("gnss_stap: CONFIG mismatch: PL M=%lu N=%lu OW=%lu, firmware M=%d N=%d OW=%d\n",
               (unsigned long)(cfg & 0xFFU), (unsigned long)((cfg >> 8) & 0xFFU),
               (unsigned long)((cfg2 >> 8) & 0xFFU), STAP_M, STAP_N, STAP_OW);
        return GNSS_STAP_ERR_CONFIG;
    }
    gnss_stap_write(GNSS_STAP_REG_CORR_LEN, gnss_stap_st.corr_len);
    write_user_ctrl(0);                                 /* start in bypass */
    gnss_stap_st.present      = 1;
    gnss_stap_st.have_weights = 0;
    printf("gnss_stap: CRPA stage present at 0x%08lx: %lu elements x %lu taps,\n"
           "           FOLD %lu, correlator %lu lags/pass. Output: BYPASS until\n"
           "           the first weight update (stap_update=1 / stap_auto=N).\n",
           (unsigned long)GNSS_STAP_BASEADDR, (unsigned long)(cfg & 0xFFU),
           (unsigned long)((cfg >> 8) & 0xFFU), (unsigned long)(cfg2 & 0xFFU),
           (unsigned long)(cfg3 & 0xFFU));
    return GNSS_STAP_OK;
}

/* steps 2..5 of an update, once the PL reports DONE */
static int32_t collect_and_solve(void)
{
    stap_cplx S[STAP_T], w[STAP_T];
    int32_t   wr[STAP_T], wi[STAP_T];
    uint32_t  L = gnss_stap_st.corr_len;
    int       t, m, i, part, e, sh;
    double    pout = 0.0;

    for (t = 0; t < STAP_NRES; t++) {
        uint32_t o = GNSS_STAP_CORR_BASE + 16U * (uint32_t)t;
        s_cre[t] = (int64_t)(((uint64_t)gnss_stap_read(o + 4U)  << 32) | gnss_stap_read(o + 0U));
        s_cim[t] = (int64_t)(((uint64_t)gnss_stap_read(o + 12U) << 32) | gnss_stap_read(o + 8U));
    }
    for (part = 0; part < 2; part++) {
        gnss_stap_write(GNSS_STAP_REG_RAW_BASE, part ? L : 0U);
        for (m = 0; m < STAP_M; m++)
            for (i = 0; i < STAP_HT; i++) {
                uint32_t v = gnss_stap_read(GNSS_STAP_RAW_WINDOW + 4U * (uint32_t)(64 * m + i));
                int32_t re = (int16_t)(v >> 16), im = (int16_t)(v & 0xFFFFU);
                if (part) { s_ed.tail_re[m][i] = re; s_ed.tail_im[m][i] = im; }
                else      { s_ed.head_re[m][i] = re; s_ed.head_im[m][i] = im; }
            }
    }

    stap_build_cov(s_cre, s_cim, &s_ed, L, s_R);
    if (gnss_stap_st.mode == GNSS_STAP_MODE_MVDR)
        stap_steering(gnss_stap_st.d_over_lambda, gnss_stap_st.theta_deg,
                      gnss_stap_st.fif_over_fs, S);
    else
        stap_steering_pi(S);
    if (stap_solve_mvdr(s_R, S, 1e-9, w) != 0) return GNSS_STAP_ERR_SOLVE;

    e  = stap_quantize(w, STAP_WW, wr, wi);
    sh = stap_out_shift(s_R, w, e, STAP_OW, gnss_stap_st.headroom_bits);

    for (t = 0; t < STAP_T; t++) {
        gnss_stap_write(GNSS_STAP_WEIGHT_BASE + 8U * (uint32_t)t,      (uint32_t)wr[t]);
        gnss_stap_write(GNSS_STAP_WEIGHT_BASE + 8U * (uint32_t)t + 4U, (uint32_t)wi[t]);
    }
    gnss_stap_write(GNSS_STAP_REG_OUT_SHIFT, (uint32_t)sh);
    gnss_stap_write(GNSS_STAP_REG_CTRL, GNSS_STAP_CTRL_W_COMMIT);

    /* diagnostics: element input powers and predicted output power w'Rw */
    for (m = 0; m < STAP_M; m++)
        gnss_stap_st.in_power_cdb[m] = cdb(s_R[m * STAP_N][m * STAP_N].re, STAP_FS_POWER);
    for (i = 0; i < STAP_T; i++) {
        double ar = 0.0, ai = 0.0;
        for (t = 0; t < STAP_T; t++) {
            ar += s_R[i][t].re * w[t].re - s_R[i][t].im * w[t].im;
            ai += s_R[i][t].re * w[t].im + s_R[i][t].im * w[t].re;
        }
        pout += w[i].re * ar + w[i].im * ai;
    }
    gnss_stap_st.out_power_cdb   = cdb(pout, STAP_FS_POWER);
    gnss_stap_st.suppression_cdb = gnss_stap_st.in_power_cdb[0] - gnss_stap_st.out_power_cdb;
    gnss_stap_st.weight_exp      = e;
    gnss_stap_st.out_shift       = sh;
    gnss_stap_st.have_weights    = 1;
    gnss_stap_st.updates++;
    return GNSS_STAP_OK;
}

static int32_t start_capture(void)
{
    if (!lclk_alive()) return GNSS_STAP_ERR_NO_CLK;
    gnss_stap_write(GNSS_STAP_REG_CORR_LEN, gnss_stap_st.corr_len);
    gnss_stap_write(GNSS_STAP_REG_CTRL, GNSS_STAP_CTRL_CORR_START);
    return GNSS_STAP_OK;
}

int32_t gnss_stap_update(void)
{
    uint32_t n;
    int32_t  ret;

    if (!gnss_stap_st.present) return GNSS_STAP_ERR_ABSENT;
    if ((ret = start_capture()) != GNSS_STAP_OK) { gnss_stap_st.errors++; return ret; }
    for (n = 0; n < STAP_DONE_TIMEOUT; n++) {
        if (gnss_stap_read(GNSS_STAP_REG_STATUS) & GNSS_STAP_ST_DONE) break;
        no_os_udelay(10);
    }
    if (n == STAP_DONE_TIMEOUT) { gnss_stap_st.errors++; return GNSS_STAP_ERR_TIMEOUT; }
    ret = collect_and_solve();
    if (ret != GNSS_STAP_OK) gnss_stap_st.errors++;
    return ret;
}

void gnss_stap_service(void)
{
    static int      busy = 0;
    static uint32_t t_last = 0, t_start = 0;
    uint32_t        t;

    if (!gnss_stap_st.present || gnss_stap_st.auto_period_ms == 0U) { busy = 0; return; }
    t = now_ms();
    if (!busy) {
        if ((uint32_t)(t - t_last) < gnss_stap_st.auto_period_ms) return;
        if (start_capture() != GNSS_STAP_OK) { gnss_stap_st.errors++; t_last = t; return; }
        busy = 1; t_start = t;
        return;
    }
    if (gnss_stap_read(GNSS_STAP_REG_STATUS) & GNSS_STAP_ST_DONE) {
        if (collect_and_solve() != GNSS_STAP_OK) gnss_stap_st.errors++;
        busy = 0; t_last = t;
    } else if ((uint32_t)(t - t_start) > 2000U) {          /* 2 s: give up, retry */
        gnss_stap_st.errors++;
        busy = 0; t_last = t;
    }
}

int32_t gnss_stap_set_enable(int enable)
{
    if (!gnss_stap_st.present) return GNSS_STAP_ERR_ABSENT;
    if (enable && !gnss_stap_st.have_weights) return GNSS_STAP_ERR_NO_WEIGHTS;
    write_user_ctrl(enable);
    return GNSS_STAP_OK;
}

int gnss_stap_get_enable(void)
{
    if (!gnss_stap_st.present) return 0;
    return (gnss_stap_read(GNSS_STAP_REG_USER_CTRL) & GNSS_STAP_UC_AJ_EN) ? 1 : 0;
}

void gnss_stap_print_state(void)
{
    uint32_t ws, st, uc;
    int m;

    printf("---- CRPA / STAP (gnss_stap @ 0x%08lx) ----\n", (unsigned long)GNSS_STAP_BASEADDR);
    if (!gnss_stap_st.present) { printf("  not present / not probed\n"); return; }
    ws = gnss_stap_read(GNSS_STAP_REG_WRAP_STAT);
    st = gnss_stap_read(GNSS_STAP_REG_STATUS);
    uc = gnss_stap_read(GNSS_STAP_REG_USER_CTRL);
    printf("  output         : %s%s\n", (uc & GNSS_STAP_UC_AJ_EN) ? "ANTI-JAM (beamformer)" : "BYPASS (raw RX1)",
           (uc & GNSS_STAP_UC_CH1_COPY) ? ", TX2 = beamformer" : ", TX2 = raw RX2");
    printf("  algorithm      : %s", gnss_stap_st.mode == GNSS_STAP_MODE_MVDR ? "MVDR" : "power inversion");
    if (gnss_stap_st.mode == GNSS_STAP_MODE_MVDR)
        printf(", look %ld deg, d/lambda x1000 = %ld", (long)gnss_stap_st.theta_deg,
               (long)(gnss_stap_st.d_over_lambda * 1000.0));
    printf("\n  snapshots      : %lu   auto period: %lu ms%s\n",
           (unsigned long)gnss_stap_st.corr_len, (unsigned long)gnss_stap_st.auto_period_ms,
           gnss_stap_st.auto_period_ms ? "" : " (off)");
    printf("  updates/errors : %lu / %lu\n", (unsigned long)gnss_stap_st.updates,
           (unsigned long)gnss_stap_st.errors);
    printf("  l_clk running  : %s   bridge timeouts: %lu   samples: %lu\n",
           (ws & 1U) ? "yes" : "NO", (unsigned long)(ws >> 16),
           (unsigned long)gnss_stap_read(GNSS_STAP_REG_SMP_COUNT));
    printf("  PL status      : busy %lu, done %lu, commit pending %lu, overrun %lu\n",
           (unsigned long)(st & 1U), (unsigned long)((st >> 1) & 1U),
           (unsigned long)((st >> 2) & 1U), (unsigned long)((st >> 3) & 1U));
    if (gnss_stap_st.have_weights) {
        for (m = 0; m < STAP_M; m++) {
            printf("  RX%d input      : ", m + 1);
            print_cdb("", gnss_stap_st.in_power_cdb[m]);
            printf("FS\n");
        }
        print_cdb("  output (pred.) : ", gnss_stap_st.out_power_cdb);
        printf("FS\n");
        print_cdb("  power removed  : ", gnss_stap_st.suppression_cdb);
        printf("   (RX1 in / out; ~ jammer suppression when a jammer dominates)\n");
        printf("  weight exp / out shift: %d / %d\n", gnss_stap_st.weight_exp, gnss_stap_st.out_shift);
    } else {
        printf("  no weights yet\n");
    }
    printf("--------------------------------------------\n");
}

/* ======================== console commands ============================== */
static const char *err_text(int32_t r)
{
    switch (r) {
    case GNSS_STAP_OK:             return "ok";
    case GNSS_STAP_ERR_ABSENT:     return "CRPA stage not present";
    case GNSS_STAP_ERR_NO_CLK:     return "AD9361 l_clk not running";
    case GNSS_STAP_ERR_CONFIG:     return "PL/firmware size mismatch";
    case GNSS_STAP_ERR_TIMEOUT:    return "covariance engine timeout";
    case GNSS_STAP_ERR_SOLVE:      return "covariance not positive definite (no RX2 signal?)";
    case GNSS_STAP_ERR_NO_WEIGHTS: return "no weights yet - run stap_update=1 first";
    default:                       return "error";
    }
}

void get_stap_status(double *param, char param_no)
{
    (void)param; (void)param_no;
    gnss_stap_print_state();
}

void get_stap_en(double *param, char param_no)
{
    (void)param; (void)param_no;
    printf("stap_en=%d\n", gnss_stap_get_enable());
}

void set_stap_en(double *param, char param_no)
{
    int32_t r;
    if (param_no < 1) { printf("stap_en= needs 0 or 1\n"); return; }
    r = gnss_stap_set_enable(param[0] != 0.0);
    printf("stap_en=%d (%s)\n", gnss_stap_get_enable(), err_text(r));
}

void set_stap_update(double *param, char param_no)
{
    int32_t r;
    (void)param; (void)param_no;
    r = gnss_stap_update();
    printf("stap_update: %s\n", err_text(r));
    if (r == GNSS_STAP_OK) gnss_stap_print_state();
}

void get_stap_auto(double *param, char param_no)
{
    (void)param; (void)param_no;
    printf("stap_auto=%lu ms\n", (unsigned long)gnss_stap_st.auto_period_ms);
}

void set_stap_auto(double *param, char param_no)
{
    double v;
    if (param_no < 1) { printf("stap_auto= needs a period in ms (0 = off)\n"); return; }
    v = param[0];
    if (v < 0.0) v = 0.0;
    if (v > 60000.0) v = 60000.0;
    gnss_stap_st.auto_period_ms = (uint32_t)v;
    printf("stap_auto=%lu ms%s\n", (unsigned long)gnss_stap_st.auto_period_ms,
           gnss_stap_st.auto_period_ms ? " (adapting in the background)" : " (off)");
}

void set_stap_mode(double *param, char param_no)
{
    if (param_no < 1) { printf("stap_mode= 0 (power inversion) or 1 (MVDR)\n"); return; }
    gnss_stap_st.mode = (param[0] != 0.0) ? GNSS_STAP_MODE_MVDR : GNSS_STAP_MODE_PI;
    printf("stap_mode=%d (%s); takes effect at the next update\n", gnss_stap_st.mode,
           gnss_stap_st.mode ? "MVDR" : "power inversion");
}

void set_stap_theta(double *param, char param_no)
{
    if (param_no < 1) { printf("stap_theta= needs degrees\n"); return; }
    if (param[0] < -90.0 || param[0] > 90.0) { printf("stap_theta: -90..90\n"); return; }
    gnss_stap_st.theta_deg = param[0];
    printf("stap_theta=%ld deg (MVDR look direction)\n", (long)param[0]);
}

void set_stap_len(double *param, char param_no)
{
    if (param_no < 1 || param[0] < 64.0 || param[0] > 8162.0) {
        printf("stap_len= 64..8162 snapshots\n"); return;
    }
    gnss_stap_st.corr_len = (uint32_t)param[0];
    printf("stap_len=%lu\n", (unsigned long)gnss_stap_st.corr_len);
}

void set_stap_ch1(double *param, char param_no)
{
    if (param_no < 1) { printf("stap_ch1= 0 (TX2 = raw RX2) or 1 (TX2 = beamformer)\n"); return; }
    gnss_stap_st.ch1_copy = (param[0] != 0.0);
    if (gnss_stap_st.present) write_user_ctrl(gnss_stap_get_enable());
    printf("stap_ch1=%d\n", gnss_stap_st.ch1_copy);
}
