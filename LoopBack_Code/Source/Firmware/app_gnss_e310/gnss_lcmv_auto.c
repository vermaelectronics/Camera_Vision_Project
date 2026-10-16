/******************************************************************************
 *  gnss_lcmv_auto.c
 *  Automatic operation of the 2-element LCMV CRPA. See gnss_lcmv_auto.h.
 *  Original work for the ANTSDR E310 V1 GNSS CRPA project.
 *****************************************************************************/
#include "gnss_lcmv_auto.h"
#include "lcmv_math.h"
#include "gnss_passthrough.h"
#include "ad9361_api.h"
#include "ad9361.h"
#include "axi_adc_core.h"
#include "console.h"
#include "no_os_delay.h"
#include <stdio.h>
#include <math.h>

#ifdef XILINX_PLATFORM
#include "xtime_l.h"
#endif

#define GAIN_MIN_DB        0
#define GAIN_MAX_DB       71
#define GAIN_RMS_HIGH    700.0   /* |x| rms above this (of 2048): step down 3 dB */
#define GAIN_RMS_LOW     180.0   /* below this: step up 1 dB                     */
#define LINK_PERIOD_US  2000000U
#define LINK_MAX_RETUNE      5   /* automatic re-tunes after boot                */
#define WEIGHT_PERIOD_US  1000U
#define GAIN_PERIOD_US  100000U
#define JAMMER_JNR_DB       10.0 /* eigenvalue ratio that counts as a jammer     */

static struct ad9361_rf_phy *phy;
static int      mode        = GNSS_LCMV_OFF;
static int      cov_hw      = 0;     /* bitstream has the covariance (v3.1+)   */
/* Angles as in the reference design (LCMV_Manu, steering_vector_2x2.v):
 * signed integer degrees from broadside, -90..+90, steering vector
 * a(theta) = [1, exp(j*pi*sin(theta))]; defaults GPS 30, jammer 60. */
static int      gps_ang     = 30;
static int      jam_ang     = 60;
static int      jam_set     = 1;
#define TO_BROADSIDE(a)   (a)
static double   eps         = 0.0;
static lcmv_c_t cal         = { 1.0, 0.0 };
static int      gain_auto   = 1;
static int32_t  gain_db     = 0;

static lcmv_cov_t    cov_last;
static int           cov_valid  = 0;
static uint32_t      cov_blocks = 0;
static lcmv_result_t res_last;
static int           res_valid  = 0;
static uint32_t      solves     = 0;
static uint32_t      solve_errs = 0;

static uint64_t t_link, t_weight, t_gain;
static uint32_t link_rx_last = 0;
static int      link_ok      = 0;
static int      link_retunes = 0;
static uint32_t sat_last     = 0;

/* ---- helpers -------------------------------------------------------------- */

static uint64_t now_us(void)
{
#ifdef XILINX_PLATFORM
    XTime t;
    XTime_GetTime(&t);
    return (uint64_t)t / (COUNTS_PER_SECOND / 1000000U);
#else
    static uint64_t fake = 0;
    return fake += 1000U;
#endif
}

/* "-12.345" without floating-point printf */
static const char *fmt(char *buf, double v, int dec)
{
    long scale = 1, ip, fp;
    int  i, neg = v < 0;
    for (i = 0; i < dec; i++) scale *= 10;
    if (neg) v = -v;
    ip = (long)v;
    fp = (long)((v - (double)ip) * (double)scale + 0.5);
    if (fp >= scale) { ip++; fp -= scale; }
    if (dec > 0) sprintf(buf, "%s%ld.%0*ld", neg ? "-" : "", ip, dec, fp);
    else         sprintf(buf, "%s%ld", neg ? "-" : "", ip);
    return buf;
}

static int32_t q16(double v)
{
    if (v >  255.99) v =  255.99;
    if (v < -255.99) v = -255.99;
    return (int32_t)lrint(v * 65536.0);
}

static void ctrl_update(uint32_t set, uint32_t clr)
{
    uint32_t c = gnss_pt_read(GNSS_PT_REG_CONTROL);
    gnss_pt_write(GNSS_PT_REG_CONTROL, (c & ~clr) | set);
}

static int set_both_gains(int32_t db)
{
    if (!phy) return -1;
    if (db < GAIN_MIN_DB) db = GAIN_MIN_DB;
    if (db > GAIN_MAX_DB) db = GAIN_MAX_DB;
    ad9361_set_rx_gain_control_mode(phy, 0, RF_GAIN_MGC);
    ad9361_set_rx_gain_control_mode(phy, 1, RF_GAIN_MGC);
    if (ad9361_set_rx_rf_gain(phy, 0, db) != 0) return -1;
    if (ad9361_set_rx_rf_gain(phy, 1, db) != 0) return -1;
    gain_db = db;
    return 0;
}

/* One coherent covariance block (the five words change together in the PL;
 * re-read if a block boundary fell between the reads). */
static int read_cov(lcmv_cov_t *r, uint32_t *blocks)
{
    int i;
    for (i = 0; i < 3; i++) {
        uint32_t b0  = gnss_pt_read(GNSS_PT_REG_COV_BLOCKS);
        uint32_t r11 = gnss_pt_read(GNSS_PT_REG_COV_R11);
        uint32_t r22 = gnss_pt_read(GNSS_PT_REG_COV_R22);
        int32_t  rr  = (int32_t)gnss_pt_read(GNSS_PT_REG_COV_R12_RE);
        int32_t  ri  = (int32_t)gnss_pt_read(GNSS_PT_REG_COV_R12_IM);
        uint32_t b1  = gnss_pt_read(GNSS_PT_REG_COV_BLOCKS);
        if (b0 == b1) {
            r->r11    = (double)r11 / 16.0;     /* Q.4 -> LSB^2 */
            r->r22    = (double)r22 / 16.0;
            r->r12.re = (double)rr  / 16.0;
            r->r12.im = (double)ri  / 16.0;
            *blocks   = b1;
            return 0;
        }
    }
    return -1;
}

/* Solve the paper's formula from the latest covariance and load the PL. */
static void update_weights(int force)
{
    lcmv_cov_t r;
    uint32_t   blocks;
    lcmv_result_t w;
    lcmv_c_t   w2hw;

    if (read_cov(&r, &blocks) != 0)
        return;
    if (!force && cov_valid && blocks == cov_blocks)
        return;                                   /* no new block yet */
    cov_last   = r;
    cov_blocks = blocks;
    cov_valid  = 1;

    lcmv_cov_calibrate(&r, cal);                  /* x2 -> c * x2 */
    if (lcmv_solve(&r, (double)TO_BROADSIDE(gps_ang), mode == GNSS_LCMV_ANGLE,
                   (double)TO_BROADSIDE(jam_ang), eps, &w) != 0) {
        solve_errs++;                             /* e.g. no RX samples: R = 0 */
        return;
    }
    res_last  = w;
    res_valid = 1;
    solves++;

    /* beamformer: y = conj(w1) x1 + conj(w2hw) x2, with w2hw = w2 conj(c) */
    w2hw.re = w.w2.re * cal.re + w.w2.im * cal.im;
    w2hw.im = w.w2.im * cal.re - w.w2.re * cal.im;
    gnss_pt_write(GNSS_PT_REG_SW_W1_RE, (uint32_t)q16(w.w1.re));
    gnss_pt_write(GNSS_PT_REG_SW_W1_IM, (uint32_t)q16(w.w1.im));
    gnss_pt_write(GNSS_PT_REG_SW_W2_RE, (uint32_t)q16(w2hw.re));
    gnss_pt_write(GNSS_PT_REG_SW_W2_IM, (uint32_t)q16(w2hw.im));   /* commit */
}

static void gain_step(void)
{
    double   p, rms;
    uint32_t sat;
    int32_t  next = gain_db;

    if (!cov_valid) return;
    p   = cov_last.r11 > cov_last.r22 ? cov_last.r11 : cov_last.r22;
    rms = sqrt(p);
    sat = gnss_pt_read(GNSS_PT_REG_LCMV_SAT_COUNT);

    if (rms > GAIN_RMS_HIGH || sat != sat_last)
        next = gain_db - 3;
    else if (rms < GAIN_RMS_LOW)
        next = gain_db + 1;
    sat_last = sat;

    if (next < GAIN_MIN_DB) next = GAIN_MIN_DB;
    if (next > GAIN_MAX_DB) next = GAIN_MAX_DB;
    if (next != gain_db)
        set_both_gains(next);
}

/* ---- RX link ---------------------------------------------------------------- */

static int rx_moving(void)
{
    uint32_t c0 = gnss_pt_read(GNSS_PT_REG_RX_COUNT_CH0);
    no_os_mdelay(2);
    return gnss_pt_read(GNSS_PT_REG_RX_COUNT_CH0) != c0;
}

int gnss_rx_link_check(int attempts, int verbose)
{
    int i, ok = rx_moving();
    uint32_t st = 0;

    for (i = 0; i < attempts && !ok && phy; i++) {
        if (phy->rx_adc)
            axi_adc_read(phy->rx_adc, AXI_ADC_REG_STATUS, &st);
        printf("gnss_rx: RX samples are NOT reaching the PL (ADC status 0x%lx); "
               "re-tuning the AD9361 RX interface (%d/%d)\n\r",
               (unsigned long)st, i + 1, attempts);
        ad9361_dig_tune(phy, 0, verbose ? BE_VERBOSE : 0);
        no_os_mdelay(5);
        ok = rx_moving();
    }
    if (verbose || !ok)
        printf("gnss_rx: RX link %s\n\r", ok ? "OK (RX1+RX2 samples reach the PL)"
               : "FAILED - check gnss_adc? (RX interface not aligned)");
    link_ok = ok;
    return ok;
}

/* ---- public ------------------------------------------------------------------ */

/* ANGLE and PL modes need the jammer angle: until it is given, the LCMV
 * stays armed but off (TX1 = RX1). */
static int waiting_for_jammer(void)
{
    return (mode == GNSS_LCMV_ANGLE || mode == GNSS_LCMV_PL) && !jam_set;
}

static void apply_mode(void)
{
    if (mode == GNSS_LCMV_OFF || waiting_for_jammer()) {
        ctrl_update(0, GNSS_PT_CTRL_LCMV_EN | GNSS_PT_CTRL_LCMV_SW);
    } else if (mode == GNSS_LCMV_PL) {
        ctrl_update(GNSS_PT_CTRL_LCMV_EN, GNSS_PT_CTRL_LCMV_SW);
    } else {
        update_weights(1);            /* weights ready before switching */
        ctrl_update(GNSS_PT_CTRL_LCMV_EN | GNSS_PT_CTRL_LCMV_SW, 0);
    }
}

void gnss_lcmv_set_mode(int m)
{
    if (m < GNSS_LCMV_OFF || m > GNSS_LCMV_PL) return;
    if ((m == GNSS_LCMV_AUTO || m == GNSS_LCMV_ANGLE) && !cov_hw) {
        printf("gnss_lcmv: bitstream has no covariance estimator (needs v3.1); "
               "using the PL angle solver\n\r");
        m = (m == GNSS_LCMV_AUTO) ? GNSS_LCMV_OFF : GNSS_LCMV_PL;
    }
    mode = m;
    if (waiting_for_jammer())
        printf("gnss_lcmv: waiting for the jammer angle: "
               "gnss_lcmv_angles=<gps> <jammer>  (-90..+90 deg from broadside)\n\r");
    apply_mode();
}

int gnss_lcmv_get_mode(void) { return mode; }

void gnss_lcmv_set_angles(int gps, int jam)
{
    gps_ang = gps;
    jam_ang = jam;
    jam_set = 1;
    gnss_pt_lcmv_set_angles(TO_BROADSIDE(gps_ang), TO_BROADSIDE(jam_ang));
    apply_mode();                     /* starts the LCMV if it was waiting */
}

void gnss_lcmv_set_eps(double e)
{
    eps = e;
    gnss_pt_lcmv_set_eps((float)e);
}

void gnss_lcmv_set_cal(double re, double im)
{
    cal.re = re;
    cal.im = im;
    gnss_pt_lcmv_set_rx2_cal((float)re, (float)im);
}

void gnss_lcmv_gain_auto(int on)
{
    gain_auto = on ? 1 : 0;
    if (on && phy) {
        int32_t g = gain_db;
        ad9361_get_rx_rf_gain(phy, 0, &g);
        set_both_gains(g);            /* start matched at RX1's gain */
    }
}

int gnss_lcmv_gain_manual(int32_t db)
{
    gain_auto = 0;
    return set_both_gains(db);
}

void gnss_lcmv_poll(void)
{
    uint64_t t = now_us();

    if (!phy) return;

    /* 1. RX link watchdog */
    if (t - t_link >= LINK_PERIOD_US) {
        uint32_t c = gnss_pt_read(GNSS_PT_REG_RX_COUNT_CH0);
        t_link  = t;
        link_ok = (c != link_rx_last);
        link_rx_last = c;
        if (!link_ok && link_retunes < LINK_MAX_RETUNE) {
            link_retunes++;
            gnss_rx_link_check(1, 0);
            link_rx_last = gnss_pt_read(GNSS_PT_REG_RX_COUNT_CH0);
        }
    }

    /* 2. LCMV weights from the covariance */
    if ((mode == GNSS_LCMV_AUTO || mode == GNSS_LCMV_ANGLE) &&
        t - t_weight >= WEIGHT_PERIOD_US) {
        t_weight = t;
        update_weights(0);
    } else if (cov_hw && t - t_weight >= GAIN_PERIOD_US) {
        /* LCMV off or PL mode: keep R current for the gain loop and reports */
        lcmv_cov_t r; uint32_t b;
        t_weight = t;
        if (read_cov(&r, &b) == 0) { cov_last = r; cov_blocks = b; cov_valid = 1; }
    }

    /* 3. matched RX gain */
    if (gain_auto && t - t_gain >= GAIN_PERIOD_US) {
        t_gain = t;
        gain_step();
    }
}

void gnss_lcmv_init(struct ad9361_rf_phy *p)
{
    uint32_t ver = gnss_pt_read(GNSS_PT_REG_VERSION);

    phy    = p;
    cov_hw = (ver >= 0x00030001U);

    printf("gnss_lcmv: automatic start (bitstream %lu.%lu)\n\r",
           (unsigned long)(ver >> 16), (unsigned long)(ver & 0xFFFFU));

    /* 1. RX link: re-tune the AD9361 RX interface if samples do not flow */
    gnss_rx_link_check(3, 1);
    link_rx_last = gnss_pt_read(GNSS_PT_REG_RX_COUNT_CH0);

    /* 2. defaults (reference design): no calibration, GPS 30 deg, jammer 60 deg */
    gnss_lcmv_set_cal(1.0, 0.0);
    gnss_lcmv_set_eps(0.0);
    gps_ang = 30;
    jam_ang = 60;
    jam_set = 1;
    gnss_pt_lcmv_set_angles(TO_BROADSIDE(gps_ang), TO_BROADSIDE(jam_ang));

    /* 3. matched automatic RX gain */
    gnss_lcmv_gain_auto(1);

    /* 4. LCMV with the jammer angle (paper eq. 9 on the measured covariance),
     *    running from boot with the default angles GPS 30, jammer 60. */
    t_link = t_weight = t_gain = now_us();
    console_set_idle_hook(gnss_lcmv_poll);
    printf("gnss_lcmv: RX gain AUTO matched (%ld dB)\n\r", (long)gain_db);
    gnss_lcmv_set_mode(cov_hw ? GNSS_LCMV_ANGLE : GNSS_LCMV_PL);
    printf("gnss_lcmv: LCMV on, GPS %d deg, jammer %d deg "
           "(gnss_lcmv_angles=<gps> <jammer> to change)\n\r", gps_ang, jam_ang);
}

/* ---- reports ------------------------------------------------------------------- */

static double dbfs(double p)       /* |x|^2 in LSB^2 -> dB re full scale */
{
    return 10.0 * log10(p / (2.0 * 2048.0 * 2048.0) + 1e-30);
}

void gnss_rx_gain_report(void)
{
    uint8_t m1 = 0, m2 = 0;
    int32_t g1 = 0, g2 = 0;
    if (!phy) return;
    ad9361_get_rx_gain_control_mode(phy, 0, &m1);
    ad9361_get_rx_gain_control_mode(phy, 1, &m2);
    ad9361_get_rx_rf_gain(phy, 0, &g1);
    ad9361_get_rx_rf_gain(phy, 1, &g2);
    printf("GNSS_RX_GAIN: %s, RX1 %ld dB, RX2 %ld dB %s\n\r",
           gain_auto ? "AUTO matched" : "manual", (long)g1, (long)g2,
           (m1 == RF_GAIN_MGC && m2 == RF_GAIN_MGC && g1 == g2)
               ? "(matched)" : "(NOT matched)");
}

void gnss_lcmv_print(void)
{
    static const char *names[] = { "OFF",
                                   "2 = AUTO (covariance, look constraint only)",
                                   "1 = JAMMER ANGLE (covariance, paper eq. 9)",
                                   "3 = PL angle solver" };
    char a[24], b[24], c2[24], d[24];
    uint32_t st = gnss_pt_read(GNSS_PT_REG_STATUS);

    printf("\n\r--- LCMV CRPA ---------------------------------------------\n\r");
    printf("  mode           : %s\n\r", names[mode]);
    printf("  driving TX1    : %s\n\r", (st & GNSS_PT_ST_LCMV_ACTIVE) ? "YES" :
           waiting_for_jammer() ? "no (set the jammer angle: gnss_lcmv_angles=<gps> <jammer>)" :
           (mode != GNSS_LCMV_OFF && !(st & GNSS_PT_ST_PASS_EN_SYNCED))
               ? "no (TX path off: gnss_tx=1)" : "no");
    printf("  angles         : degrees from broadside, -90..+90\n\r");
    printf("  GPS look angle : %d deg\n\r", gps_ang);
    if (jam_set) printf("  jammer angle   : %d deg\n\r", jam_ang);
    else         printf("  jammer angle   : NOT SET\n\r");
    printf("  RX link        : %s\n\r", link_ok ? "OK" : "NO SAMPLES (see gnss_adc?)");
    gnss_rx_gain_report();

    if (cov_valid) {
        printf("  covariance R   : R11 %s dBFS  R22 %s dBFS  |R12| %s dBFS  (block %lu)\n\r",
               fmt(a, dbfs(cov_last.r11), 1), fmt(b, dbfs(cov_last.r22), 1),
               fmt(c2, dbfs(sqrt(cov_last.r12.re * cov_last.r12.re +
                                  cov_last.r12.im * cov_last.r12.im)), 1),
               (unsigned long)cov_blocks);
    } else {
        printf("  covariance R   : not available\n\r");
    }
    if (res_valid && (mode == GNSS_LCMV_AUTO || mode == GNSS_LCMV_ANGLE)) {
        printf("  eigenvalues    : JNR %s dB -> %s\n\r", fmt(a, res_last.jnr_db, 1),
               res_last.jnr_db > JAMMER_JNR_DB ? "JAMMER DETECTED" : "no jammer");
        if (res_last.null_valid)
            printf("  array null at  : %s deg\n\r", fmt(a, res_last.null_deg, 1));
        printf("  w1             : %s %sj\n\r", fmt(a, res_last.w1.re, 5), fmt(b, res_last.w1.im, 5));
        printf("  w2             : %s %sj\n\r", fmt(c2, res_last.w2.re, 5), fmt(d, res_last.w2.im, 5));
        printf("  solves         : %lu (%lu skipped: no valid R)\n\r",
               (unsigned long)solves, (unsigned long)solve_errs);
    }
    printf("  output clipped : %lu samples\n\r",
           (unsigned long)gnss_pt_read(GNSS_PT_REG_LCMV_SAT_COUNT));
    printf("-----------------------------------------------------------\n\r");
}
