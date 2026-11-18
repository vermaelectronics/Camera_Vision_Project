/***************************************************************************//**
 * @file   gnss_autostart.c
 * @brief  GNSS-CRPA MOD-15: automatic MVDR start, health check, RX gain
 *         auto-level and the console event monitor. See gnss_autostart.h.
 *
 * Safety ordering, the same rule every TX path in this project follows:
 *   TX1 attenuation goes to the hardware maximum FIRST, everything is set up
 *   and checked with TX1 at maximum, and the operating attenuation is applied
 *   as the LAST step -- and only when no check FAILED.
 * TX1 is an RF output on the GPS L1 frequency: conducted, attenuated coax
 * into the receiver only.
 ******************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ad9361_api.h"
#include "no_os_delay.h"
#include "gnss_l1.h"
#include "gnss_passthrough.h"
#include "gnss_smi.h"         /* MOD-25: SMI-MVDR STAP */
#include "gnss_dash.h"        /* MOD-25: colours */
#include "gnss_report.h"      /* MOD-26: report, events, SMI jammer figures */
#include "gnss_mvdr_auto.h"    /* MOD-26: operator auto steering (gnss_beam=) */
#include "gnss_config.h"
#include "gnss_autostart.h"
#include "gnss_temp.h"
#include "command.h"
#include "gnss_quiet.h"     /* LVDS console: engine output captured, see gnss_quiet.h */
static int32_t rx2_off_v(void);   /* LVDS: RX2 = RX1 - gnss_rxoff */
#define GNSS_QUIET_THIS_FILE
#undef  printf
#define printf        gnss_qprintf
#undef  console_print
#define console_print gnss_qprintf
#undef  vprintf
#define vprintf       gnss_qvprintf   /* res()/more() tails were printed */
#undef  putchar
#define putchar       gnss_qputchar

extern struct ad9361_rf_phy *ad9361_phy;

/* ------------------------------------------------------------------------- */
/*  Report helpers                                                           */
/* ------------------------------------------------------------------------- */
static int n_pass, n_warn, n_fail;
static int bg_printed = 0;      /* background output since the last check */
int gnss_prompt_open = 0;       /* MOD-28: the prompt ends the console line */
/* MOD-28: background output is about to be printed: leave the prompt line */
static void bg_mark(void)
{
    if (gnss_prompt_open) { printf("\r\n"); gnss_prompt_open = 0; }
    bg_printed = 1;
}
static int mon_reset_req = 0;   /* MOD-19: next monitor poll only takes a baseline */
static double iq_worst_dbc(const gnss_pt_mvdr_state_t *s);   /* below */
static int32_t guard_auto_gain;                               /* below */
static int smi_mon_init;                                     /* MOD-25, below */
static int agc_auto;                                          /* below */
static void agc_go_auto(int32_t g);                           /* below */
static uint32_t agc_changes_min;                              /* below */
static int32_t agc_clean;                                     /* below, MOD-26 */
static int agc_since_change;                                  /* below, MOD-27 */
static int32_t agc_cur;
static void settle_smi(int ms);                               /* below, MOD-29 */
static double rx_target_eff;    /* defined with the RX auto-level below */
static int monitor_armed = 0;

#define LV_PASS 0
#define LV_WARN 1
#define LV_FAIL 2
#define LV_INFO 3

static void res(int lv, const char *fmt, ...)
{
    static const char *tag[4] = { "PASS", "WARN", "FAIL", "INFO" };
    va_list ap;

    if (lv == LV_PASS) { n_pass++; }
    if (lv == LV_WARN) { n_warn++; }
    if (lv == LV_FAIL) { n_fail++; }
    {   /* MOD-25: coloured tag (gnss_color=0 for plain text) */
        static const int col[4] = { GC_GRN, GC_YEL, GC_RED, GC_DIM };
        printf("  %s[%s]%s ", gnss_c(col[lv]), tag[lv], gnss_c(GC_RST));
    }
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

/* "      " continuation line under a result */
static void more(const char *fmt, ...)
{
    va_list ap;
    printf("         ");
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

/* Prints v with one decimal, integer maths only (no %f in this printf). */
static void pdb(double v)
{
    long t = (long)(v * 10.0 + ((v < 0.0) ? -0.5 : 0.5));
    printf("%s%ld.%ld", (t < 0) ? "-" : "", labs(t) / 10L, labs(t) % 10L);
}

static void pmhz(uint32_t hz)
{
    printf("%lu.%03lu MHz", (unsigned long)(hz / 1000000UL),
           (unsigned long)((hz % 1000000UL) / 1000UL));
}

static void section(const char *s)
{
    printf("\n -- %s\n", s);
}

static void summary(const char *what)
{
    printf("\n===============================================================\n");
    printf(" %s: %d PASS, %d WARN, %d FAIL\n", what, n_pass, n_warn, n_fail);
}

/* dBFS of a raw peak (12-bit, full scale 2048). */
static double peak_dbfs(uint32_t pk)
{
    if (pk == 0U) { return -99.0; }
    return 20.0 * log10((double)pk / 2048.0);
}

/* MVDR output level in dBFS from OUT_LVL (same conversion as gnss_mvdr?). */
static double out_dbfs(const gnss_pt_mvdr_state_t *s)
{
    return 3.0103 * ((double)s->out_lvl_q4 / 16.0 - 23.0);
}

static int32_t gnss_plan_gain0(void)
{
    int32_t g = GNSS_CFG_RX_GAIN_START_DB;
    gnss_get_plan(NULL, NULL, NULL, &g, NULL);
    return g;
}
static uint32_t gnss_plan_atten(void)
{
    uint32_t a = GNSS_CFG_TX1_ATTEN_MDB;
    gnss_get_plan(NULL, NULL, NULL, NULL, &a);
    return a;
}

static int mvdr_present(void)
{
    return gnss_pt_read(GNSS_PT_REG_MVDR_ID) == GNSS_PT_MVDR_EXPECTED_ID;
}

/* Puts TX1 to silence: DAC source zero, passthrough muted, max attenuation. */
static void tx_silence(void)
{
    double p[1];
    p[0] = 0;
    gnss_tx_raw(p, 1);
}

/* ------------------------------------------------------------------------- */
/*  Individual checks (shared by the start and the health check)            */
/* ------------------------------------------------------------------------- */

/* 1. FPGA identity. Returns 0 if the MVDR core is usable. */
static int chk_fpga(int probe)
{
    uint32_t v, mid, mv;

    if (probe) {
        if (gnss_pt_probe() != GNSS_PT_OK) {
            res(LV_FAIL, "FPGA: gnss_passthrough not reachable (ID/scratch test).\n");
            more("Is the bitstream programmed? Is the IP at 0x43C10000?\n");
            return -1;
        }
    } else if (gnss_pt_read(GNSS_PT_REG_ID) != GNSS_PT_EXPECTED_ID) {
        res(LV_FAIL, "FPGA: gnss_passthrough ID 0x%08lx, expected 0x%08lx.\n",
            (unsigned long)gnss_pt_read(GNSS_PT_REG_ID), (unsigned long)GNSS_PT_EXPECTED_ID);
        return -1;
    }
    v = gnss_pt_read(GNSS_PT_REG_VERSION);
    if (v == GNSS_PT_EXPECTED_VERSION) {
        res(LV_PASS, "FPGA: gnss_passthrough v%lu.%lu\n",
            (unsigned long)(v >> 16), (unsigned long)(v & 0xFFFFU));
    } else {
        res(LV_WARN, "FPGA: gnss_passthrough version 0x%08lx, expected 0x%08lx.\n",
            (unsigned long)v, (unsigned long)GNSS_PT_EXPECTED_VERSION);
    }
    mid = gnss_pt_read(GNSS_PT_REG_MVDR_ID);
    if (mid != GNSS_PT_MVDR_EXPECTED_ID) {
        res(LV_FAIL, "FPGA: MVDR core missing (ID 0x%08lx). Program the MVDR bitstream.\n",
            (unsigned long)mid);
        return -1;
    }
    mv = gnss_pt_read(GNSS_PT_REG_MVDR_VERSION);
    if (mv >= GNSS_PT_MVDR_EXPECTED_VERSION) {
        res(LV_PASS, "FPGA: MVDR core v1.%lu (SMI-MVDR STAP%s: CW + barrage nulling)\n",
            (unsigned long)(mv & 0xFFFFU),
            gnss_pt_stap_available() ? " + NLMS STAP" : "; NLMS STAP not built");
    } else if (mv >= GNSS_PT_MVDR_VERSION_SMI) {
        /* MOD-26b: 1.14 / 1.14b work, but their 2x2 window never forgets a
         * disturbance (R22 stayed corrupt after a rate change, 4 Nov) */
        res(LV_WARN, "FPGA: MVDR core v1.%lu (SMI-MVDR STAP). v1.15 is current: its 2x2\n",
            (unsigned long)(mv & 0xFFFFU));
        more("covariance heals itself after a sample-rate change (jammer estimator,\n");
        more("RX noise per channel). SMI itself is not affected. Reprogram when you can.\n");
    } else if (mv >= GNSS_PT_MVDR_VERSION_STAP) {
        res(LV_WARN, "FPGA: MVDR core v1.%lu: STAP (NLMS) but no SMI-STAP (needs v1.14).\n",
            (unsigned long)(mv & 0xFFFFU));
        more("gnss_null=3 runs as 2. Program the v1.14 bitstream for SMI.\n");
    } else if (mv >= GNSS_PT_MVDR_VERSION_TIMING) {
        res(LV_WARN, "FPGA: MVDR core v1.%lu: no STAP (needs v1.13). One weight per antenna:\n",
            (unsigned long)(mv & 0xFFFFU));
        more("CW jammers are nulled, a wideband / barrage jammer only partly.\n");
        more("Program the v1.13 bitstream for gnss_null=2 (STAP).\n");
    } else if (mv >= GNSS_PT_MVDR_VERSION_JAM2) {
        res(LV_WARN, "FPGA: MVDR core v1.%lu, not the timing-closed v1.12.\n",
            (unsigned long)(mv & 0xFFFFU));
        more("Works, but Vivado timing was not met on that build. Reprogram.\n");
    } else {
        res(LV_WARN, "FPGA: MVDR core v1.%lu is old: RX auto-level needs v1.11,\n",
            (unsigned long)(mv & 0xFFFFU));
        more("AOLC v1.10, IQ correction v1.9. Those steps are skipped. Reprogram.\n");
    }
    return 0;
}

/* 2. Radio plan read back from the AD9361. Returns the number of FAILs. */
static int chk_radio(void)
{
    uint64_t rlo = 0, tlo = 0;
    uint32_t rfs = 0, tfs = 0, rbw = 0, tbw = 0;
    uint8_t  m1 = 0xFF, m2 = 0xFF;
    int32_t  g1 = -99, g2 = -99;
    int      f = 0;
    int64_t  d;
    uint32_t pbw = 0, pfs = 0;

    ad9361_get_rx_lo_freq(ad9361_phy, &rlo);
    ad9361_get_tx_lo_freq(ad9361_phy, &tlo);
    d = (int64_t)rlo - (int64_t)gnss_get_lo_hz();
    if (d < 0) { d = -d; }
    if (d <= 1000 && (((int64_t)tlo - (int64_t)gnss_get_lo_hz()) <= 1000) &&
        (((int64_t)gnss_get_lo_hz() - (int64_t)tlo) <= 1000)) {
        res(LV_PASS, "radio: LO RX %lu Hz, TX %lu Hz\n", (unsigned long)rlo, (unsigned long)tlo);
    } else {
        res(LV_FAIL, "radio: LO RX %lu Hz, TX %lu Hz, expected %lu Hz.\n",
            (unsigned long)rlo, (unsigned long)tlo, (unsigned long)gnss_get_lo_hz());
        f++;
    }

    ad9361_get_rx_sampling_freq(ad9361_phy, &rfs);
    ad9361_get_tx_sampling_freq(ad9361_phy, &tfs);
    gnss_get_plan(NULL, &pbw, &pfs, NULL, NULL);
    if (labs((long)rfs - (long)pfs) <= (long)(pfs / 1000UL) &&
        labs((long)tfs - (long)pfs) <= (long)(pfs / 1000UL)) {
        res(LV_PASS, "radio: sample rate RX %lu, TX %lu SPS\n",
            (unsigned long)rfs, (unsigned long)tfs);
    } else {
        res(LV_FAIL, "radio: sample rate RX %lu, TX %lu SPS, expected %lu.\n",
            (unsigned long)rfs, (unsigned long)tfs, (unsigned long)pfs);
        f++;
    }

    ad9361_get_rx_rf_bandwidth(ad9361_phy, &rbw);
    ad9361_get_tx_rf_bandwidth(ad9361_phy, &tbw);
    if (labs((long)rbw - (long)pbw) <= (long)(pbw / 20UL) &&
        labs((long)tbw - (long)pbw) <= (long)(pbw / 20UL)) {
        res(LV_PASS, "radio: RF bandwidth RX %lu, TX %lu Hz\n",
            (unsigned long)rbw, (unsigned long)tbw);
    } else {
        res(LV_WARN, "radio: RF bandwidth RX %lu, TX %lu Hz, expected %lu.\n",
            (unsigned long)rbw, (unsigned long)tbw, (unsigned long)pbw);
    }

    ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &m1);
    ad9361_get_rx_gain_control_mode(ad9361_phy, 1, &m2);
    ad9361_get_rx_rf_gain(ad9361_phy, 0, &g1);
    ad9361_get_rx_rf_gain(ad9361_phy, 1, &g2);
    if (m1 == 0 && m2 == 0 && g1 == g2) {
        res(LV_PASS, "radio: RX1/RX2 gain MANUAL, %ld dB on both\n", (long)g1);
    } else if (m1 == 0 && m2 == 0 && g1 - g2 == rx2_off_v()) {
        res(LV_PASS, "radio: RX1/RX2 gain MANUAL, RX1 %ld dB, RX2 %ld dB (gnss_rxoff=%ld)\n",
            (long)g1, (long)g2, (long)rx2_off_v());
    } else {
        res(LV_FAIL, "radio: RX1 mode %lu gain %ld dB, RX2 mode %lu gain %ld dB.\n",
            (unsigned long)m1, (long)g1, (unsigned long)m2, (long)g2);
        more("MVDR needs MANUAL (mode 0) and EQUAL gain. Fix: gnss_rxauto=1\n");
        f++;
    }
    return f;
}

/* 3. l_clk. Returns 1 on FAIL. */
static int chk_lclk(void)
{
    uint32_t fs = 0, meas, exp;
    long     d;

    ad9361_get_rx_sampling_freq(ad9361_phy, &fs);
    exp  = 4U * fs;
    meas = gnss_pt_lclk_hz();
    if (meas == 0U) {
        res(LV_FAIL, "l_clk: reads 0 -- the AD9361 interface clock is not running.\n");
        return 1;
    }
    d = labs((long)meas - (long)exp);
    if (meas > GNSS_PT_LCLK_CONSTRAINT_MHZ * 1000000UL) {
        res(LV_FAIL, "l_clk: "); pmhz(meas);
        printf(" is ABOVE the %lu MHz the FPGA timing was closed at.\n",
               (unsigned long)GNSS_PT_LCLK_CONSTRAINT_MHZ);
        more("Lower the sample rate (gnss_config.h) or re-close timing.\n");
        return 1;
    }
    if (d > (long)(exp / 33U)) {
        res(LV_WARN, "l_clk: "); pmhz(meas); printf(", expected "); pmhz(exp);
        printf(" (4 x fs).\n");
        return 0;
    }
    res(LV_PASS, "l_clk: "); pmhz(meas);
    printf(" (4 x fs), timing closed at %lu MHz\n", (unsigned long)GNSS_PT_LCLK_CONSTRAINT_MHZ);
    return 0;
}

/* 4. TX path bits: DAC accepting our samples, passthrough + MVDR selected,
 * adaptive mode live. Returns the number of FAILs. */
static int chk_path(void)
{
    uint32_t st = gnss_pt_read(GNSS_PT_REG_STATUS);
    uint32_t ctl = gnss_pt_read(GNSS_PT_REG_CONTROL);
    gnss_pt_mvdr_state_t s;
    int f = 0;

    if ((st & GNSS_PT_ST_DAC_EN_I0) && (st & GNSS_PT_ST_DAC_EN_Q0)) {
        res(LV_PASS, "path: DAC is taking the MVDR samples (dac_enable 1/1)\n");
    } else {
        res(LV_FAIL, "path: DAC is NOT taking our samples (dac_enable %lu/%lu).\n",
            (unsigned long)((st >> 2) & 1U), (unsigned long)((st >> 3) & 1U));
        more("Nothing from the MVDR reaches TX1. Run gnss_auto=1.\n");
        f++;
    }
    if ((st & GNSS_PT_ST_PASS_EN_SYNCED) && (st & GNSS_PT_ST_MVDR_SEL_SYNCED) &&
        !(ctl & GNSS_PT_CTRL_MUTE)) {
        res(LV_PASS, "path: RX -> MVDR -> TX1 selected, not muted\n");
    } else {
        res(LV_FAIL, "path: pass_en %lu, mvdr_sel %lu, mute %lu (want 1, 1, 0).\n",
            (unsigned long)((st >> 16) & 1U), (unsigned long)((st >> 17) & 1U),
            (unsigned long)((ctl >> 1) & 1U));
        f++;
    }
    gnss_pt_mvdr_get_state(&s);
    if (s.mode_live == (int)GNSS_PT_MVDR_MODE_ADAPTIVE && !s.freeze) {
        res(LV_PASS, "MVDR: adaptive mode live%s\n", s.commit_pending ? " (commit pending)" : "");
    } else {
        res(LV_FAIL, "MVDR: mode %ld%s, want adaptive (1).\n",
            (long)s.mode_live, s.freeze ? " FROZEN" : "");
        f++;
    }
    return f;
}

/* 5. Samples moving: RX/TX counters and the FIFO, MVDR windows and weight
 * updates. Read-only. Returns the number of FAILs. */
static int chk_flow(uint32_t ms)
{
    gnss_pt_state_t      a, b;
    gnss_pt_mvdr_state_t s0, s1;
    uint32_t drx, dtx, dov, dun, dbl, dup;
    int f = 0;

    gnss_pt_get_state(&a);
    gnss_pt_mvdr_get_state(&s0);
    no_os_mdelay(ms);
    gnss_pt_get_state(&b);
    gnss_pt_mvdr_get_state(&s1);
    drx = b.rx_count_ch0 - a.rx_count_ch0;
    dtx = b.tx_count_ch0 - a.tx_count_ch0;
    dov = b.overflow_count - a.overflow_count;
    dun = b.underflow_count - a.underflow_count;
    dbl = s1.n_blocks - s0.n_blocks;
    /* windows absorbed by the null HOLD (jammer just left) are handled by
     * design and do not produce a weight update */
    dup = (s1.n_upd + s1.n_hold) - (s0.n_upd + s0.n_hold);

    if (drx == 0U) {
        res(LV_FAIL, "data: NO RX samples in %lu ms. AD9361 RX path not delivering.\n",
            (unsigned long)ms);
        f++;
    } else if (dtx == 0U) {
        res(LV_FAIL, "data: RX ok but NO TX requests in %lu ms. DAC side stopped.\n",
            (unsigned long)ms);
        f++;
    } else {
        res(LV_PASS, "data: RX %lu / TX %lu sample counts in %lu ms\n",
            (unsigned long)drx, (unsigned long)dtx, (unsigned long)ms);
    }
    if (dov == 0U && dun == 0U) {
        res(LV_PASS, "data: FIFO no overflow / underflow (level %lu)\n",
            (unsigned long)b.fifo_level_ch0);
    } else {
        res(LV_WARN, "data: FIFO overflow +%lu, underflow +%lu in %lu ms.\n",
            (unsigned long)dov, (unsigned long)dun, (unsigned long)ms);
        more("RX and TX rates differ: check rx/tx sample rate are equal.\n");
    }
    if (dbl == 0U) {
        res(LV_FAIL, "MVDR: no covariance windows in %lu ms (RX2 enabled? l_clk?).\n",
            (unsigned long)ms);
        f++;
    } else if (dup == 0U && gnss_pt_profile_effective() == 3) {
        /* MOD-26b: SMI drives TX1; the single-weight core is a spare */
        double c1 = 0.0, c2 = 0.0;
        int cs = gnss_pt_cov_check(&s1, &c1, &c2);
        res(LV_WARN, "MVDR: single-weight core makes NO weight updates (skip %lu)%s\n",
            (unsigned long)(s1.n_skip - s0.n_skip),
            (cs == 0) ? ": its 2x2 covariance is INVALID" : "");
        if (cs == 0) {
            more("RX1 %ld dBFS, RX2 %ld dBFS (R22 corrupt: impossible or zero). SMI is not\n",
                 (long)floor(c1 + 0.5), (long)floor(c2 + 0.5));
            more("affected; the jammer estimator is. MVDR 1.15 heals it: reprogram.\n");
        } else {
            more("SMI drives TX1, so this does not stop the anti-jam.\n");
        }
    } else if (dup == 0U) {
        res(LV_FAIL, "MVDR: %lu windows but NO weight updates (skip %lu, zero %lu).\n",
            (unsigned long)dbl, (unsigned long)(s1.n_skip - s0.n_skip),
            (unsigned long)(s1.n_zero - s0.n_zero));
        f++;
    } else {
        res(LV_PASS, "MVDR: %lu windows, %lu weight updates / holds in %lu ms\n",
            (unsigned long)dbl, (unsigned long)dup, (unsigned long)ms);
    }
    if (s1.n_drop - s0.n_drop) {
        res(LV_WARN, "MVDR: %lu weight sets dropped (calculator too slow).\n",
            (unsigned long)(s1.n_drop - s0.n_drop));
    }
    if (s1.skew_sticky) {
        res(LV_WARN, "MVDR: RX1/RX2 valid mismatch seen (skew). Check RX2 is enabled\n");
        more("and both channels have the same sample rate and gain.\n");
    }
    /* a few rail hits from noise peaks are normal (Gaussian peaks ~11 dB
     * above an -18 dBFS floor); warn above 1e-4 of the samples */
    if ((s1.n_osat - s0.n_osat) > (uint32_t)(gnss_pt_get_fs() / 10000UL) * ms / 1000U) {
        res(LV_WARN, "output: %lu samples clipped at the 12-bit DAC in %lu ms.\n",
            (unsigned long)(s1.n_osat - s0.n_osat), (unsigned long)ms);
        more("Lower the AOLC target (bigger GNSS_CFG_AOLC_TARGET_DB).\n");
    }
    return f;
}

/* 6. Input level: ADC clipping, headroom, noise floor vs target, RX1/RX2
 * balance. Read-only. verdict = 0: the auto-level just ran and already gave
 * its verdict, so the noise floor is printed as [INFO]. */
static void chk_level(int verdict)
{
    gnss_pt_mvdr_state_t s;
    gnss_pt_jam_state_t  j;
    double n1, n2, nf, pk;

    gnss_pt_mvdr_get_state(&s);
    gnss_pt_jam_get_state(&j);
    if (s.clip1 || s.clip2) {
        res(LV_WARN, "input: ADC CLIPPING, RX1 %lu / RX2 %lu samples on a rail.\n",
            (unsigned long)s.clip1, (unsigned long)s.clip2);
        more("A clipped jammer cannot be nulled. Run gnss_rxauto=1.\n");
    } else {
        res(LV_PASS, "input: no ADC clipping\n");
    }
    if (s.have_peak) {
        pk = peak_dbfs((s.peak1 > s.peak2) ? s.peak1 : s.peak2);
        res((pk > -3.0) ? LV_WARN : LV_INFO, "input: peak ");
        pdb(pk); printf(" dBFS (headroom "); pdb(-pk); printf(" dB)\n");
    }
    if (j.have_ch && j.n1_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE &&
        j.n2_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE) {
        n1 = (double)j.n1_q8 / 256.0;
        n2 = (double)j.n2_q8 / 256.0;
        nf = 0.5 * (n1 + n2);
        if (!verdict) {
            res(LV_INFO, "input: noise floor RX1 "); pdb(n1); printf(" / RX2 "); pdb(n2);
            printf(" dBFS\n");
        } else if (fabs(nf - rx_target_eff) <= (double)GNSS_CFG_RX_NF_TOL_DB) {
            res(LV_PASS, "input: noise floor RX1 "); pdb(n1); printf(" / RX2 "); pdb(n2);
            printf(" dBFS (target "); pdb(rx_target_eff); printf(")\n");
        } else {
            res(LV_WARN, "input: noise floor RX1 "); pdb(n1); printf(" / RX2 "); pdb(n2);
            printf(" dBFS, target "); pdb(rx_target_eff); printf(". Run gnss_rxauto=1.\n");
        }
        if (fabs(n1 - n2) > 3.0 && !(j.available && j.valid && (j.present || j.jnr_db_q8 > 0))) {
            /* MOD7: not with a jammer present (it reaches the two antennas unequally) */
            res(LV_WARN, "input: RX1 and RX2 noise differ by "); pdb(fabs(n1 - n2));
            printf(" dB at equal gain.\n");
            more("Antenna, LNA power or cable difference. MVDR still works,\n");
            more("but the null is shallower. Check the weaker channel.\n");
        }
    } else {
        res(LV_INFO, "input: per-channel noise floor not available (MVDR < 1.11 or no estimate yet)\n");
    }
}

/* 7. IQ correction, AOLC, jammer / coherence. Read-only. */
/* MOD-24: which core drives the output, and that STAP is really running */
static void chk_stap(uint32_t ms)
{
    gnss_pt_mvdr_state_t s;
    int p = gnss_pt_get_profile();

    if (p == 3) {
        /* MOD-25: SMI-MVDR STAP. The main loop is not running yet: poll the
         * solver here until a few windows are solved. */
        gnss_smi_state_t ss;
        int i;
        if (!gnss_smi_available()) {
            res(LV_WARN, "null core: SMI-STAP asked (gnss_null=3) but the bitstream has none:\n");
            more("running as %s. Program the MVDR v1.14 bitstream.\n",
                 gnss_pt_stap_available() ? "STAP (NLMS)" : "MVDR (one weight)");
            if (!gnss_pt_stap_available()) { return; }
            p = 2;
        } else {
            for (i = 0; i < 50; i++) { gnss_smi_poll(); no_os_mdelay(1); }
            gnss_smi_get_state(&ss);
            if (!ss.hw_active) {
                res(LV_FAIL, "null core: SMI enabled but not active (CTRL.EN did not reach the datapath).\n");
                more("gnss_null=3 again; gnss_smi?; gnss_health?\n");
                return;
            }
            if (ss.updates == 0U) {
                res(LV_FAIL, "null core: SMI active but no window solved in 50 ms (%lu failed).\n",
                    (unsigned long)ss.fails);
                more("Is the RX running (samples in)? gnss_smi?\n");
                return;
            }
            res(LV_PASS, "null core: SMI-MVDR STAP (2 x %d + 2 image taps), %lu windows solved, lambda_min ",
                ss.La, (unsigned long)ss.updates);
            pdb(ss.lam_dbfs);
            printf(" dBFS, |w|^2 ");
            pdb(ss.wnorm);
            printf("\n");
            {   /* MOD-25: the count since the bitstream load includes the
                 * sample-rate switch at boot; only a GROWING count matters */
                uint32_t n0 = ss.novr;
                no_os_mdelay(100);
                gnss_smi_get_state(&ss);
                if (ss.novr != n0) {
                    res(LV_WARN, "SMI: %lu sample overruns in 100 ms (needs >= 4 clocks per sample, 2R2T).\n",
                        (unsigned long)((ss.novr - n0) & 0xFFFFU));
                } else if (n0) {
                    res(LV_INFO, "SMI: %lu overruns during the radio set-up (rate change), none since\n",
                        (unsigned long)n0);
                }
            }
            more("CW and wideband / barrage jammers are nulled; gnss_supp? for the depth.\n");
            return;
        }
    }
    if (p != 2) {
        res(LV_INFO, "null core %d: %s (gnss_null=3 for SMI-STAP)\n", p,
            (p == 1) ? "plain power inversion, one weight" : "MVDR, one weight, loading + hold/glide");
        return;
    }
    if (!gnss_pt_stap_available()) {
        res(LV_WARN, "null core: STAP asked (gnss_null=2) but the bitstream has none:\n");
        more("running as MVDR (one weight). Program the MVDR v1.13 bitstream.\n");
        return;
    }
    no_os_mdelay(ms);                       /* status crosses once per window */
    gnss_pt_mvdr_get_state(&s);
    if (!s.stap_active) {
        res(LV_FAIL, "null core: STAP enabled but not active (STAP_CTRL 0x%08lx).\n",
            (unsigned long)s.stap_ctrl);
        more("Commit did not reach the datapath? gnss_null=2 again; gnss_health?\n");
        return;
    }
    res(LV_PASS, "null core: STAP (2 antennas x 5 taps), mu 2^-%lu, |w|^2 ",
        (unsigned long)((s.stap_ctrl >> 4) & 0xFU));
    pdb((double)s.stap_wnorm / 16777216.0);
    printf("\n");
    more("CW and wideband / barrage jammers are nulled; gnss_supp? for the depth.\n");
}

static void chk_mvdr_features(uint32_t ms)
{
    gnss_pt_mvdr_state_t s0, s;
    gnss_pt_jam_state_t  j;
    uint32_t tgt;
    double   lvl;

    gnss_pt_mvdr_get_state(&s0);
    no_os_mdelay(ms);
    gnss_pt_mvdr_get_state(&s);

    if (s.have_iq && !(s.iq_ctrl & GNSS_PT_IQ_OFF) && iq_worst_dbc(&s) > -35.0) {
        res(LV_WARN, "IQ: image estimate "); pdb(iq_worst_dbc(&s));
        printf(" dBc is implausible (AD9361 ~ -50 dBc).\n");
        more("A jammer near the LO (LO = L1) misleads the estimator. Compare the\n");
        more("null with gnss_iq=0, or move the LO: gnss_lo=1584.5.\n");
    } else if (s.have_iq) {
        if (s.iq_ctrl & GNSS_PT_IQ_OFF) {
            res(LV_INFO, "IQ: image correction OFF (default since MOD-21; gnss_iq=1 to use it)\n");
        } else if (s.n_iq_upd != s0.n_iq_upd) {
            res(LV_PASS, "IQ: image correction ON, adapting on the jammer (%lu updates)\n",
                (unsigned long)s.n_iq_upd);
        } else if (s.st_jam) {
            res(LV_WARN, "IQ: jammer present but the image correction is not updating.\n");
            more("It adapts only on a STRONG jammer (JNR > ~21 dB). gnss_iq?\n");
        } else {
            /* iq_est learns only from a strong jammer, by design: without
             * one it holds its coefficients (a noise-only update would be
             * worse than the AD9361's own image). */
            res(LV_PASS, "IQ: image correction ON, armed (learns on a strong jammer;\n");
            more("%lu updates so far -- 0 is normal before the first jammer)\n",
                 (unsigned long)s.n_iq_upd);
        }
    }

    if (s.have_aolc) {
        if (s.out_gain_reg & GNSS_PT_OG_AOLC_EN) {
            tgt = (s.out_gain_reg >> 16) & 0xFFU;
            lvl = out_dbfs(&s);
            if (fabs(lvl + (double)tgt) <= 2.0) {
                res(LV_PASS, "output: AOLC holding "); pdb(lvl);
                printf(" dBFS (target -%lu), gain ", (unsigned long)tgt);
                pdb(6.0206 * (double)s.out_gain_q8 / 256.0); printf(" dB\n");
            } else {
                res(LV_WARN, "output: AOLC at "); pdb(lvl);
                printf(" dBFS, target -%lu (gain ", (unsigned long)tgt);
                pdb(6.0206 * (double)s.out_gain_q8 / 256.0);
                printf(" dB, at a limit?). RX gain far too low/high? gnss_rxauto=1\n");
            }
        } else {
            res(LV_WARN, "output: AOLC OFF (manual OUT_GAIN %lu). gnss_aolc=%lu\n",
                (unsigned long)s.out_gain, (unsigned long)GNSS_CFG_AOLC_TARGET_DB);
        }
    }

    if (s.have_smooth) {
        if (s.st_jam) {
            gnss_pt_jam_get_state(&j);
            res(LV_INFO, "jammer: DETECTED and being nulled");
            if (j.available && j.valid && j.present) {
                printf(", JNR "); pdb((double)j.jnr_db_q8 / 256.0);
                printf(" dB, angle "); pdb((double)j.theta_cdeg / 100.0); printf(" deg");
            }
            printf("\n");
        } else if (s.coh_l2 < 6U && !s.st_hold && !s.st_glide) {   /* MOD-19; MOD7: not while a jammer just left */
            res(LV_WARN, "jammer: none detected, but RX1/RX2 are COHERENT (COH_L2 %lu).\n",
                (unsigned long)s.coh_l2);
            more("Splitter / shared-LNA bench? The null would take the satellites.\n");
            more("Set gnss_coh=%lu (below COH_L2) or use separate antennas.\n",
                 (unsigned long)((s.coh_l2 > 2U) ? (s.coh_l2 - 2U) : 1U));
        } else {
            res(LV_PASS, "jammer: none (RX1/RX2 incoherent, COH_L2 %lu): satellites pass\n",
                (unsigned long)s.coh_l2);
        }
        if (s.st_hold)  { res(LV_INFO, "jammer: just left, null HELD\n"); }
        if (s.st_glide) { res(LV_INFO, "jammer: gone, weights GLIDING back\n"); }
    }
}

/* 8. TX1 attenuation and what that means at the receiver. */
static void chk_tx(int expect_open)
{
    uint32_t a = 0;
    long     dbm10;

    if (ad9361_get_tx_attenuation(ad9361_phy, 0, &a) != 0) {
        res(LV_WARN, "TX1: attenuation read failed\n");
        return;
    }
    if (a >= GNSS_L1_TX_MAX_ATTEN_MDB) {
        res(expect_open ? LV_WARN : LV_INFO, "TX1: at maximum attenuation (%lu mdB): SILENT\n",
            (unsigned long)a);
        return;
    }
    /* noise out = FS dBm - AOLC target - attenuation, in 0.1 dB */
    dbm10 = 10L * GNSS_CFG_TX_FS_DBM - 10L * (long)GNSS_CFG_AOLC_TARGET_DB - (long)(a / 100U);
    res(LV_INFO, "TX1: %lu mdB attenuation -> noise floor out of TX1 ~ ", (unsigned long)a);
    pdb((double)dbm10 / 10.0); printf(" dBm\n");
    more("Best C/N0 when the noise at the RECEIVER input is %ld..%ld dBm\n",
         (long)GNSS_CFG_RX_NOISE_MIN_DBM, (long)GNSS_CFG_RX_NOISE_MAX_DBM);
    more("(subtract any external attenuator). Tune with tx1_attenuation=.\n");
}

/* ------------------------------------------------------------------------- */
/*  RX gain auto-level                                                       */
/* ------------------------------------------------------------------------- */
/* Noise target actually in use: GNSS_CFG_RX_NF_TARGET_DBFS, or higher when
 * the AD9361's own floor would otherwise swamp the antenna noise. */
static double rx_target_eff = (double)GNSS_CFG_RX_NF_TARGET_DBFS;
static double rx_floor_meas = -200.0;    /* MOD-26: AD9361 floor at start */
double gnss_rx_target_dbfs(void) { return rx_target_eff; }
double gnss_rx_floor_dbfs(void)  { return rx_floor_meas; }

/* MOD-30: the AD9361 floor on this board is not white: in the centre of the
 * band it is a normal ~-78 dBFS/MHz, toward the band edges (|f| > ~0.2 fs) its
 * ADC noise rises ~20 dB, and that edge noise is most of the -46.8 dBFS total.
 * The satellites only see the floor at their own frequency, so the gain is
 * levelled on that (white-equivalent: density x fs, comparable with the total):
 *   gnss_floor=0  whole band (MOD-29: antenna noise 15 dB over the total floor)
 *   gnss_floor=1  GPS L1 C/A + Galileo E1 / BeiDou B1C (default)
 *   gnss_floor=2  every signal inside the band (GLONASS, B1I too: the worst one) */
static int    floor_mode   = 0;    /* MOD-31: whole band = highest gain = best C/N0 (A/B test 10 Nov) */
static double rx_floor_sig = -200.0;    /* white-equivalent floor for floor_mode */
int  gnss_floor_mode(void) { return floor_mode; }
void gnss_floor_mode_set(int m) { if (m >= 0 && m <= 2) { floor_mode = m; } }
const char *gnss_floor_mode_name(int m)
{
    return (m == 0) ? "whole band (highest gain)" : (m == 1) ? "GPS L1 / Galileo E1 / BeiDou B1C" : "every signal in the band";
}
double gnss_rx_floor_sig_dbfs(void)
{
    return (floor_mode != 0 && rx_floor_sig > -150.0) ? rx_floor_sig : rx_floor_meas;
}

static void set_rx_gain_both(int32_t g)
{
    int32_t g2 = g - rx2_off_v();
    if (g2 < GNSS_CFG_RX_GAIN_MIN_DB) { g2 = GNSS_CFG_RX_GAIN_MIN_DB; }
    if (g2 > GNSS_CFG_RX_GAIN_MAX_DB) { g2 = GNSS_CFG_RX_GAIN_MAX_DB; }
    int32_t g_old = g;
    (void)ad9361_get_rx_rf_gain(ad9361_phy, 0, &g_old);
    ad9361_set_rx_gain_control_mode(ad9361_phy, 0, 0);
    ad9361_set_rx_gain_control_mode(ad9361_phy, 1, 0);
    ad9361_set_rx_rf_gain(ad9361_phy, 0, g);
    ad9361_set_rx_rf_gain(ad9361_phy, 1, g2);
    gnss_smi_gain_changed_db((double)(g - g_old));   /* fresh covariance after the step (MOD6: + dB) */
}
/* re-apply the RX2 offset at the current RX1 gain (gnss_rxoff=) */
void gnss_rx2_off_apply(void)
{
    int32_t g = 0;
    if (ad9361_phy && ad9361_get_rx_rf_gain(ad9361_phy, 0, &g) == 0) { set_rx_gain_both(g); }
}

static int rx_meas_one = 0;   /* MOD-26b: 1/2 = only that channel's noise is plausible */
typedef struct {
    int    ok;          /* per-channel noise estimate available */
    int    clip;
    double n1, n2, nf;  /* dBFS */
    double pk;          /* dBFS */
    int    jam;
} rx_meas_t;

/* Sets the gain, waits for fresh windows and noise estimates, measures. */
static void rx_measure(int32_t g, rx_meas_t *m, int print)
{
    gnss_pt_mvdr_state_t s;
    gnss_pt_jam_state_t  j;
    int tries;

    set_rx_gain_both(g);
    memset(m, 0, sizeof(*m));
    for (tries = 0; tries < 4; tries++) {
        no_os_mdelay(40);   /* window refill + 2^6-window jammer/noise average */
        gnss_pt_mvdr_get_state(&s);
        gnss_pt_jam_get_state(&j);
        m->ok = (j.have_ch && j.n1_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE &&
                 j.n2_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE) ? 1 : 0;
        if (m->ok) { break; }
    }
    m->clip = (s.clip1 || s.clip2) ? 1 : 0;
    m->pk   = s.have_peak ? peak_dbfs((s.peak1 > s.peak2) ? s.peak1 : s.peak2) : -99.0;
    m->n1   = m->ok ? (double)j.n1_q8 / 256.0 : -99.0;
    m->n2   = m->ok ? (double)j.n2_q8 / 256.0 : -99.0;
    m->nf   = 0.5 * (m->n1 + m->n2);
    /* MOD-26b: a channel above -3 dBFS or 15 dB off the other is not noise
     * (a build with a corrupt R22 read RX2 at +8 dBFS and set the target to
     * -4 dBFS): use the plausible channel */
    if (m->ok && (m->n2 > -3.0 || m->n2 - m->n1 > 15.0) && m->n1 <= -3.0) {
        m->nf = m->n1;  rx_meas_one = 1;
    } else if (m->ok && (m->n1 > -3.0 || m->n1 - m->n2 > 15.0) && m->n2 <= -3.0) {
        m->nf = m->n2;  rx_meas_one = 2;
    }
    m->jam  = (j.available && j.present) ? 1 : 0;
    if (print) {
        printf("         gain %2ld dB: noise ", (long)g);
        if (m->ok) { pdb(m->n1); printf(" / "); pdb(m->n2); printf(" dBFS"); }
        else       { printf("(no estimate)"); }
        printf(", peak "); pdb(m->pk); printf(" dBFS%s%s\n", m->clip ? ", CLIPPING" : "",
               m->jam ? ", jammer" : "");
    }
}

/* Power sum / difference in dB */
static double db2lin(double d) { return pow(10.0, d / 10.0); }

/* MOD-29: gnss_spec? / gnss_spec=G -- the RX noise spectrum of each antenna,
 * from the SMI window's 16 x 16 covariance block of each antenna (Hann-tapered
 * periodogram: ~fs/10 resolution). Output muted while it runs; gain restored. Tells whether the floor
 * is flat (thermal / digital errors), rises at the band edges (ADC / filter
 * noise) or has a peak (a spur: move the LO and see if it moves). */
#define SPEC_BINS 32
static double dbfs_lsb2(double p) { return 10.0 * log10(p / (2048.0 * 2048.0) + 1e-30); }

/* MOD-30: PSD of the last SMI window at f (fraction of fs, wrapped) */
static double psd_at(int ant, double f)
{
    while (f >= 0.5)  { f -= 1.0; }
    while (f < -0.5)  { f += 1.0; }
    return gnss_smi_psd(ant, f);
}
/* White-equivalent noise (12-bit LSB^2 per sample) one signal sees: PSD at its
 * offset and +-hw/2 (weights 1,2,1), on both signs of the offset (the tap
 * order fixes the sign of f; taking the worse side makes the figure
 * independent of it), averaged over the two antennas. */
static double sig_white(double off_hz, double hw_hz, double fs)
{
    double tot = 0.0, v, w;
    int ant, sg;
    for (ant = 0; ant < 2; ant++) {
        w = 0.0;
        for (sg = -1; sg <= 1; sg += 2) {
            v = (psd_at(ant, sg * (off_hz - 0.5 * hw_hz) / fs) + 2.0 * psd_at(ant, sg * off_hz / fs) +
                 psd_at(ant, sg * (off_hz + 0.5 * hw_hz) / fs)) / 4.0;
            if (v > w) { w = v; }
        }
        tot += w;
    }
    return 0.5 * tot;
}
/* The floor (dBFS, white-equivalent) for a mode, from the last SMI window.
 * mode 1: the worse of GPS L1 C/A and Galileo E1 / B1C; mode 2: the worst
 * signal inside the band; mode 0 or nothing inside the band: -200.
 * print = 1: one line per signal inside the band. */
static double sig_floor(int mode, int print)
{
    uint64_t lo = 0;
    uint32_t bw = 0;
    double fs = (double)gnss_pt_get_fs(), half, off, mhz, hw, v, worst = -1.0, worst_l1 = -1.0;
    const char *name;
    int i;

    if (mode == 0 && !print) { return -200.0; }
    gnss_get_plan(&lo, &bw, NULL, NULL, NULL);
    if (fs <= 0.0) { return -200.0; }
    half = 0.5 * (((double)bw < fs) ? (double)bw : fs) / 1e6;
    for (i = 0; gnss_rep_signal(i, &name, &mhz, &hw); i++) {
        off = mhz - (double)lo / 1e6;
        if (fabs(off) + hw > half) { continue; }               /* not (fully) in the band */
        v = sig_white(off * 1e6, hw * 1e6, fs);
        if (v > worst) { worst = v; }
        if (fabs(mhz - 1575.42) < 0.01 && v > worst_l1) { worst_l1 = v; }
        if (print) {
            printf("    %-23s ", name); if (off >= 0.0) { printf("+"); } pdb(off);
            printf(" MHz: "); pdb(dbfs_lsb2(v / fs * 1e6)); printf(" dBFS/MHz = "); pdb(dbfs_lsb2(v));
            printf(" dBFS white-equivalent\n");
        }
    }
    if (mode == 1 && worst_l1 > 0.0) { return dbfs_lsb2(worst_l1); }
    if (mode != 0 && worst > 0.0)    { return dbfs_lsb2(worst); }
    return -200.0;
}

void gnss_spec_scan(int gain)
{
    double r1r[GNSS_SMI_LMAX], r1i[GNSS_SMI_LMAX], r2r[GNSS_SMI_LMAX], r2i[GNSS_SMI_LMAX];
    double d1[SPEC_BINS], d2[SPEC_BINS], fs = (double)gnss_pt_get_fs(), mn = 1e9, mx = -1e9;
    double cen = 0.0, edg = 0.0, med, srt[SPEC_BINS], f;
    int32_t g0 = 0;
    int L, L2, b, i, j, nc = 0, ne = 0, bmax = 0;

    if (!ad9361_phy || !gnss_smi_available() || !gnss_smi_is_enabled()) {
        printf("gnss_spec: needs the radio and SMI running (gnss_null=3)\n");
        return;
    }
    if (gain > 70) { gain = 70; }
    ad9361_get_rx_rf_gain(ad9361_phy, 0, &g0);
    gnss_rep_hold(1);                                  /* MOD-30: no false jammer */
    gnss_pt_set_mute(1);
    if (gain >= 0) { set_rx_gain_both(gain); }
    settle_smi(80);
    L  = gnss_smi_lag_corr(0, r1r, r1i);
    L2 = gnss_smi_lag_corr(1, r2r, r2i);
    for (b = 0; b < SPEC_BINS; b++) {                  /* this window, before the gain moves back */
        f = ((double)b - SPEC_BINS / 2) / SPEC_BINS;
        d1[b] = dbfs_lsb2(gnss_smi_psd(0, f) / fs * 1e6);
        d2[b] = dbfs_lsb2(gnss_smi_psd(1, f) / fs * 1e6);
    }
    printf("GNSS_SPEC: RX noise density from the SMI window, RX gain %ld dB%s, output muted\n",
           (long)(gain >= 0 ? gain : g0), gain >= 0 ? " (set for the scan)" : "");
    if (L >= 4 && L2 == L && fs > 0.0) {
        printf("  where the satellites are (both antennas, worse side of the LO):\n");
        (void)sig_floor(0, 1);
    }
    if (gain >= 0) { set_rx_gain_both(g0); agc_cur = g0; }
    settle_smi(40);
    gnss_pt_set_mute(0);
    gnss_rep_hold(0);
    if (L < 4 || L2 != L || fs <= 0.0) { printf("gnss_spec: no SMI window\n"); return; }

    for (b = 0; b < SPEC_BINS; b++) {
        f = ((double)b - SPEC_BINS / 2) / SPEC_BINS;
        if (d1[b] < mn) { mn = d1[b]; }
        if (d1[b] > mx) { mx = d1[b]; bmax = b; }
        srt[b] = d1[b];
        if (fabs(f) < 10e6 / fs) { cen += d1[b]; nc++; }
        if (fabs(f) > 0.4) { edg += d1[b]; ne++; }
    }
    for (i = 1; i < SPEC_BINS; i++) {               /* median */
        double v = srt[i];
        for (j = i - 1; j >= 0 && srt[j] > v; j--) { srt[j + 1] = srt[j]; }
        srt[j + 1] = v;
    }
    med = 0.5 * (srt[SPEC_BINS / 2 - 1] + srt[SPEC_BINS / 2]);
    cen = nc ? cen / nc : med;  edg = ne ? edg / ne : med;

    printf("  resolution ~"); pdb(fs / 10.0 / 1e6); printf(" MHz (16 taps, Hann taper, one SMI window)\n");
    printf("  offset MHz  RX1 dBFS/MHz  RX2 dBFS/MHz\n");
    for (b = 0; b < SPEC_BINS; b++) {
        int n = (int)((d1[b] - mn) + 0.5);
        f = ((double)b - SPEC_BINS / 2) / SPEC_BINS * fs / 1e6;
        printf("  "); if (f >= 0.0) { printf("+"); } pdb(f); printf("   "); pdb(d1[b]); printf("      "); pdb(d2[b]); printf("  |");
        if (n > 40) { n = 40; }
        for (i = 0; i < n; i++) { printf("#"); }
        printf("\n");
    }
    printf("  total power: RX1 "); pdb(dbfs_lsb2(r1r[0])); printf(" dBFS, RX2 "); pdb(dbfs_lsb2(r2r[0]));
    printf(" dBFS\n  density: centre +-10 MHz "); pdb(cen); printf(", band edges "); pdb(edg);
    printf(", median "); pdb(med); printf(" dBFS/MHz\n");
    printf("  SHAPE: ");
    if (mx - med > 3.0 && fabs(((double)bmax - SPEC_BINS / 2) / SPEC_BINS) <= 0.4) {
        f = ((double)bmax - SPEC_BINS / 2) / SPEC_BINS * fs / 1e6;
        printf("PEAK at "); if (f >= 0.0) { printf("+"); } pdb(f); printf(" MHz, "); pdb(mx - med);
        printf(" dB over the median: a spur or interference.\n");
        printf("         Shift the LO (gnss_lo=1583.5) and scan again: a spur that moves is made on the board.\n");
    } else if (edg - cen > 4.0) {
        printf("RISES AT THE BAND EDGES (+"); pdb(edg - cen); printf(" dB): ADC noise of the AD9361 (sigma-delta ADC).\n");
        printf("         Only the floor at each satellite's frequency matters (list above); the gain is levelled\n");
        printf("         on it (gnss_floor?). gnss_clk? shows the ADC clock, gnss_osr= changes it for a test.\n");
    } else {
        printf("FLAT (edges "); if (edg - cen >= 0.0) { printf("+"); } pdb(edg - cen); printf(" dB vs centre): white noise -- thermal, or digital errors (run gnss_bist?).\n");
    }
}

/* MOD-29: gnss_bist? -- is the FPGA data link clean? The AD9361 injects its
 * digital BIST tone in place of the ADC samples (no ADC or analog noise), the
 * SMI window measures it; the two strongest eigen-directions of each antenna's
 * 16 x 16 block (the tone and its IQ image) are removed: what is left is noise
 * added between the AD9361 and the SMI input
 * (LVDS interface errors, FPGA path). Compared with the normal floor. */
void gnss_bist_test(void)
{
    double rr[GNSS_SMI_LMAX], ri[GNSS_SMI_LMAX], fl[2], tp[2] = {-99.0, -99.0}, nz[2] = {-99.0, -99.0};
    uint32_t fs = gnss_pt_get_fs();
    int32_t  g0 = 0;
    int ant, L, ok = 1;

    if (!ad9361_phy || !gnss_smi_available() || !gnss_smi_is_enabled() || fs == 0U) {
        printf("gnss_bist: needs the radio and SMI running (gnss_null=3)\n");
        return;
    }
    ad9361_get_rx_rf_gain(ad9361_phy, 0, &g0);
    printf("GNSS_BIST: AD9361 digital tone in place of the ADC data (output muted ~0.5 s)\n");
    gnss_rep_hold(1);                                  /* MOD-30 */
    gnss_pt_set_mute(1);
    set_rx_gain_both(GNSS_CFG_RX_GAIN_MIN_DB);
    settle_smi(80);
    for (ant = 0; ant < 2; ant++) {
        L = gnss_smi_lag_corr(ant, rr, ri);
        fl[ant] = L ? dbfs_lsb2(rr[0]) : -99.0;
    }
    /* tone at 2/32 fs (3.84 MHz at 61.44 MS/s), -12 dBFS, both channels */
    if (ad9361_bist_tone(ad9361_phy, BIST_INJ_RX, fs / 16U, 12, 0) != 0) {
        printf("  ad9361_bist_tone failed -- test not run\n");
        ok = 0;
    } else {
        settle_smi(80);
        for (ant = 0; ant < 2; ant++) {
            double n, t = 0.0;
            n = gnss_smi_tone_noise(ant, &t);
            tp[ant] = (n >= 0.0) ? dbfs_lsb2(t) : -99.0;
            nz[ant] = (n >= 0.0) ? dbfs_lsb2(n) : -99.0;
            if (nz[ant] < -90.0) { nz[ant] = -90.0; }          /* below what one window resolves */
        }
        ad9361_bist_tone(ad9361_phy, BIST_DISABLE, 0, 0, 0);
    }
    set_rx_gain_both(g0);
    agc_cur = g0;
    settle_smi(40);
    gnss_pt_set_mute(0);
    gnss_rep_hold(0);
    if (!ok) { return; }

    printf("  normal floor at minimum gain (BIST off): RX1 "); pdb(fl[0]); printf(" / RX2 "); pdb(fl[1]); printf(" dBFS\n");
    for (ant = 0; ant < 2; ant++) {
        const char *v;
        printf("  RX%d with the BIST tone: tone ", ant + 1); pdb(tp[ant]);
        printf(" dBFS, noise left after removing it "); pdb(nz[ant]); printf(" dBFS -> ");
        if (tp[ant] < -30.0)             { v = "NO TONE SEEN: the BIST did not reach the FPGA (check the driver / ENSM)"; }
        else if (nz[ant] <= -65.0)       { v = "CLEAN interface"; }
        else if (nz[ant] <= -55.0)       { v = "MARGINAL: some interface or FPGA-path noise"; }
        else                             { v = "ERRORS: noise is added between the AD9361 and the FPGA"; }
        printf("%s\n", v);
    }
    printf("  VERDICT: ");
    if (tp[0] < -30.0 || tp[1] < -30.0) {
        printf("inconclusive (no tone).\n");
    } else if (nz[0] <= -65.0 && nz[1] <= -65.0) {
        printf("the data link is clean, so the ");
        pdb(0.5 * (fl[0] + fl[1]));
        printf(" dBFS floor is made BEFORE it:\n           ADC / analog chain / board (power, clocks, TX leakage). Run gnss_spec=0 and the gnss_floor? tests.\n");
    } else {
        printf("noise is added on the AD9361 -> FPGA link or in the FPGA path:\n"
               "           check the RX delay tuning at boot, LVDS settings, and the bitstream timing.\n");
    }
}

/* MOD-27: gnss_floor? -- the receiver noise against RX gain, both channels.
 * The output is muted for the ~1 s it takes; gain and AGC are restored. Run
 * it with the antennas on, then with RX1/RX2 terminated in 50 ohm: if the
 * low-gain floor stays at the same dBFS, it comes from the board / AD9361
 * (TX leakage, clocks, supply), not from the antennas. */
void gnss_floor_scan(void)
{
    static const int32_t gl[] = { 0, 10, 20, 30, 40, 50, 60, 70 };
    rx_meas_t m;
    int32_t   g0 = 0;
    double    lo = 0.0, fsig = -200.0;
    unsigned  i;
    uint32_t  a2 = 0;

    if (!ad9361_phy || !mvdr_present()) { printf("gnss_floor: radio / MVDR core missing\n"); return; }
    ad9361_get_rx_rf_gain(ad9361_phy, 0, &g0);
    ad9361_get_tx_attenuation(ad9361_phy, 1, &a2);
    printf("GNSS_FLOOR: RX noise vs gain (output muted ~1 s; TX1 %s, TX2 %lu mdB)\n",
           gnss_tx_is_on() ? "on" : "off", (unsigned long)a2);
    gnss_rep_hold(1);                                  /* MOD-30 */
    gnss_pt_set_mute(1);
    for (i = 0; i < sizeof(gl) / sizeof(gl[0]); i++) {
        rx_measure(gl[i], &m, 1);
        if (i == 0) {
            lo = m.nf;
            if (gnss_smi_available() && gnss_smi_is_enabled()) {   /* MOD-30 */
                settle_smi(80);
                printf("         at %ld dB, where the satellites are:\n", (long)gl[0]);
                (void)sig_floor(0, 1);
                fsig = sig_floor(floor_mode ? floor_mode : 1, 0);
            }
        }
    }
    set_rx_gain_both(g0);
    agc_cur = g0;
    settle_smi(40);
    gnss_pt_set_mute(0);
    gnss_rep_hold(0);
    printf("  floor at minimum gain: "); pdb(lo);
    printf(" dBFS over the whole band");
    if (fsig > -150.0) {
        printf(";\n  where the satellites are (%s): ", gnss_floor_mode_name(floor_mode ? floor_mode : 1));
        pdb(fsig); printf(" dBFS white-equivalent.\n  The gain is levelled on the %s floor (gnss_floor=%d)",
                          floor_mode ? "satellite-band" : "whole-band", floor_mode);
    }
    printf(".\n  (a normal AD9361 sits below -70 dBFS over the band).\n"
           "  Where the noise starts to rise 1 dB per dB of gain, the antenna noise\n"
           "  dominates; the gain there + 15 dB is the useful minimum under a jammer.\n"
           "  Repeat with RX1/RX2 terminated (50 ohm): the same floor = board/AD9361.\n"
           "  RX gain back to %ld dB.\n", (long)g0);
}

int gnss_rx_autolevel(int standalone)
{
    rx_meas_t a, b, m;
    int32_t   g, g_probe;
    double    floor_db = -200.0, tgt, pa, pb, i0, fe;
    long      step;
    int       it, result = -1, hr_limited = 0, floor_limited = 0;

    rx_meas_one = 0;

    if (standalone) {
        n_pass = n_warn = n_fail = 0;
        printf("\n=== RX gain auto-level (target noise floor %ld dBFS) ===\n",
               (long)GNSS_CFG_RX_NF_TARGET_DBFS);
    }
    if (!ad9361_phy || !mvdr_present()) {
        res(LV_FAIL, "RX auto-level: radio or MVDR core missing\n");
        return -1;
    }
    if (gnss_pt_read(GNSS_PT_REG_MVDR_VERSION) < GNSS_PT_MVDR_VERSION_JAM2) {
        res(LV_WARN, "RX auto-level needs MVDR v1.11; gain left at its value.\n");
        return 1;
    }

    /* Step 1: the AD9361's OWN noise floor. At the minimum gain the antenna
     * contributes ~nothing, so what is measured there is the chip's floor
     * (ADC, baseband, LO leakage / flicker at zero-IF). A second point 30 dB
     * higher tells how much of it is antenna noise. The target is then
     * raised, if needed, so the antenna noise stays >= 10 dB above the floor
     * -- otherwise the auto-level would chase a floor that does not move
     * with gain and end at 0 dB gain, burying the satellites. */
    printf("         measuring the AD9361 internal floor:\n");
    rx_measure(GNSS_CFG_RX_GAIN_MIN_DB, &a, 1);
    /* MOD-30: the floor where the satellites are, from the SMI window at the
     * minimum gain (the antenna adds ~nothing there) */
    rx_floor_sig = -200.0;
    if (floor_mode != 0 && a.ok && !a.clip && gnss_smi_available() && gnss_smi_is_enabled()) {
        gnss_rep_hold(1);
        settle_smi(80);
        rx_floor_sig = sig_floor(floor_mode, 0);
        gnss_rep_hold(0);
    }
    g_probe = GNSS_CFG_RX_GAIN_MIN_DB + 30;
    rx_measure(g_probe, &b, 1);
    if (!a.ok || !b.ok) {
        res(LV_FAIL, "RX auto-level: no noise estimate (jammer estimator off?)\n");
        return -1;
    }
    if (a.clip) {
        floor_db = -200.0;          /* input hot even at minimum gain */
    } else {
        pa = db2lin(a.nf);
        pb = db2lin(b.nf);
        /* N(g) = F + I*10^(g/10):  pa = F + I,  pb = F + 1000 I */
        i0 = (pb - pa) / 999.0;
        if (i0 < 0.0) { i0 = 0.0; }
        floor_db = (pa - i0 > 0.0) ? 10.0 * log10(pa - i0) : -200.0;
        if (b.nf - a.nf < 6.0) { floor_db = a.nf; }   /* flat: floor dominates */
    }
    /* MOD-30: antenna noise MARGIN dB over the floor the satellites see (fe);
     * the measured total also carries the whole-band floor, so the target
     * for the total is floor_db + (fe + MARGIN), as powers. fe = floor_db
     * (gnss_floor=0) gives MOD-29's floor + 15.1 dB. */
    fe = floor_db;
    if (floor_db > -150.0 && floor_mode != 0 && rx_floor_sig > -150.0) { fe = rx_floor_sig; }
    tgt = (double)GNSS_CFG_RX_NF_TARGET_DBFS;
    if (floor_db > -150.0) {
        double want = 10.0 * log10(db2lin(floor_db) + db2lin(fe + (double)GNSS_CFG_RX_FLOOR_MARGIN_DB));
        if (want > tgt) { tgt = want; floor_limited = 1; }
    }
    if (tgt > (double)GNSS_CFG_RX_NF_TARGET_MAX_DBFS) {
        tgt = (double)GNSS_CFG_RX_NF_TARGET_MAX_DBFS;   /* MOD-26b: never level the noise this hot */
    }
    rx_target_eff = tgt;
    rx_floor_meas = floor_db;            /* MOD-26: gnss_report? */
    gnss_smi_set_floor_dbfs(floor_db);   /* MOD7: the tracking's noise reference */
    gnss_pt_set_rx_target_dbfs(tgt);     /* gnss_rxgain? uses it too */
    if (rx_meas_one) {
        res(LV_WARN, "input: RX%d per-channel noise is implausible (%s): levelling on RX%d only\n",
            3 - rx_meas_one, (rx_meas_one == 1) ? "corrupt R22?" : "corrupt R11?", rx_meas_one);
    }
    if (floor_db > -150.0) {
        res(floor_limited ? LV_WARN : LV_PASS, "AD9361 internal floor ");
        pdb(floor_db); printf(" dBFS over the whole band");
        if (fe != floor_db) {
            printf(",\n"); more("where the satellites are ("); printf("%s): ", gnss_floor_mode_name(floor_mode));
            pdb(fe); printf(" dBFS white-equivalent");
        }
        if (floor_limited) {
            printf(".\n"); more("Too high for the %ld dBFS target: levelling to ", (long)GNSS_CFG_RX_NF_TARGET_DBFS);
            pdb(tgt); printf(" dBFS (antenna noise %ld dB\n", (long)GNSS_CFG_RX_FLOOR_MARGIN_DB);
            more("above the %s floor). gnss_floor=0/1/2 picks the band protected:\n",
                 (fe != floor_db) ? "satellite-band" : "whole-band");
            more("0 whole band (default: highest gain, best C/N0), 1 L1/E1/B1C, 2 every signal.\n");
        } else {
            printf(" (well below the target)\n");
        }
    }

    /* Step 2: level the antenna noise to the target, from the probe gain. */
    g = g_probe;
    m = b;
    for (it = 0; it < 12; it++) {
        if (m.clip || m.pk > -1.0) {
            step = -10;
            hr_limited = 1;
        } else {
            step = (long)floor(tgt - m.nf + 0.5);
            hr_limited = 0;
            /* keep >= 3 dB above the strongest sample (a jammer) */
            if ((double)step > -3.0 - m.pk) {
                long lim = (long)floor(-3.0 - m.pk);
                if (lim < step) { step = lim; hr_limited = 1; }
            }
            if (!hr_limited && step >= -(long)GNSS_CFG_RX_NF_TOL_DB &&
                step <= (long)GNSS_CFG_RX_NF_TOL_DB) {
                result = 0;
                break;
            }
            if (hr_limited && step >= -1 && step <= 1) {
                result = 1;
                break;
            }
            if (step > 20)  { step = 20; }
            if (step < -20) { step = -20; }
        }
        if ((long)g + step > GNSS_CFG_RX_GAIN_MAX_DB) { step = GNSS_CFG_RX_GAIN_MAX_DB - g; }
        if ((long)g + step < GNSS_CFG_RX_GAIN_MIN_DB) { step = GNSS_CFG_RX_GAIN_MIN_DB - g; }
        if (step == 0) {
            result = 1;     /* at a gain limit */
            break;
        }
        g += (int32_t)step;
        rx_measure(g, &m, 1);
    }
    set_rx_gain_both(g);
    agc_go_auto(g);                      /* AUTO gain continues from here */
    guard_auto_gain = g;
    if (result == 0 && !m.jam) { agc_clean = g; }   /* MOD-26: clean reference */

    if (result == 0) {
        res(LV_PASS, "RX gain %ld dB (manual, both): noise ", (long)g);
        pdb(m.nf); printf(" dBFS on target, peak "); pdb(m.pk); printf(" dBFS\n");
    } else if (result == 1 && hr_limited) {
        res(LV_WARN, "RX gain %ld dB, held down by a strong signal: noise ", (long)g);
        pdb(m.nf); printf(" dBFS (target "); pdb(tgt); printf(").\n");
        more("Kept 3 dB of ADC headroom for the jammer. Each 6 dB below the\n");
        more("target costs the satellites ~0.1-0.35 dB C/N0: fine while it nulls.\n");
    } else if (result == 1 && g >= GNSS_CFG_RX_GAIN_MAX_DB) {
        res(LV_WARN, "RX gain at MAXIMUM %ld dB and the noise is only ", (long)g);
        pdb(m.nf); printf(" dBFS (target "); pdb(tgt); printf(").\n");
        more("Too little antenna signal: antenna / LNA power (bias-T) / cable /\n");
        more("RF switch path? With a passive antenna this is expected.\n");
    } else if (result == 1) {
        res(LV_WARN, "RX gain at MINIMUM %ld dB and the input is still hot (noise ", (long)g);
        pdb(m.nf); printf(" dBFS).\n");
        more("Add an attenuator in front of RX1/RX2 or reduce the LNA gain.\n");
    } else {
        res(LV_WARN, "RX gain %ld dB: did not settle in 12 steps (noise ", (long)g);
        pdb(m.nf); printf(" dBFS). Signal changing? Run gnss_rxauto=1 again.\n");
        result = 1;
    }
    if (standalone) {
        summary("RX AUTO-LEVEL");
        printf("===============================================================\n");
    }
    return result;
}

/* ------------------------------------------------------------------------- */
/*  Automatic start                                                          */
/* ------------------------------------------------------------------------- */
static void banner(const char *t)
{
    printf("\n===============================================================\n");
    printf(" %s\n", t);
    uint64_t lo; uint32_t bw, fs, at; int32_t g0;
    gnss_get_plan(&lo, &bw, &fs, &g0, &at);
    printf("   LO %lu Hz, BW %lu Hz, %lu SPS (gnss_band? / gnss_plan=)\n",
           (unsigned long)lo, (unsigned long)bw, (unsigned long)fs);
    printf("   RX gain AUTO from %ld dB, AOLC -%lu dBFS, TX1 %lu mdB\n",
           (long)g0, (unsigned long)GNSS_CFG_AOLC_TARGET_DB, (unsigned long)at);
    printf("===============================================================\n");
}

/* MOD-26: run the SMI solver for ms milliseconds (the main loop is not
 * running during the automatic start) */
static void settle_smi(int ms)
{
    int i;
    for (i = 0; i < ms; i++) {
        gnss_smi_poll();
        no_os_mdelay(1);
    }
}

static int abort_silent(const char *why)
{
    tx_silence();
    monitor_armed = 1;          /* Enter still shows the live status */
    summary("MVDR AUTO-START");
    printf(" RESULT: FAILED at \"%s\" -- TX1 is SILENT.\n", why);
    printf("         Fix the [FAIL] line above, then gnss_auto=1.\n");
    printf("===============================================================\n");
    return n_fail ? n_fail : 1;
}

int gnss_autostart_run(void)
{
    double   p[1];
    uint32_t a = 0;
    uint32_t mv;

    n_pass = n_warn = n_fail = 0;
    monitor_armed = 0;
    banner("MVDR AUTO-START");
    {
        int bn = -1; const char *bname = "";
        gnss_band_info(&bn, &bname);
        gnss_rep_event("AUTO-START  gnss_band=%d, %s", bn, bname);
    }

    section("1/8 FPGA");
    if (chk_fpga(1) != 0) { return abort_silent("FPGA"); }
    mv = gnss_pt_read(GNSS_PT_REG_MVDR_VERSION);

    section("2/8 radio");
    if (!ad9361_phy) {
        res(LV_FAIL, "radio: ad9361_phy is NULL -- AD9361 did not initialise.\n");
        return abort_silent("radio init");
    }
    /* TX1 silent first: everything below happens with TX1 at maximum. */
    gnss_tx_silence_quiet();
    if (ad9361_get_tx_attenuation(ad9361_phy, 0, &a) == 0 && a >= GNSS_L1_TX_MAX_ATTEN_MDB) {
        res(LV_PASS, "TX1: held quiet (%lu mdB) while setting up -- opens in step 8\n",
            (unsigned long)a);
    } else {
        res(LV_FAIL, "TX1: attenuation not confirmed at maximum (read %lu).\n",
            (unsigned long)a);
        return abort_silent("TX1 silence");
    }
    printf("         applying the radio plan (the AD9361 lines below are its echo):\n");
    gnss_radio_apply_defaults();
    no_os_mdelay(20);
    if (chk_radio() != 0) { return abort_silent("radio plan"); }
    {   /* MOD-26: TX2 unused, held quiet */
        uint32_t a2 = gnss_tx2_hold();
        if (a2 == GNSS_CFG_TX2_ATTEN_MDB) {
            res(LV_PASS, "TX2: held at %lu mdB (unused: TX1 carries the GNSS output)\n", (unsigned long)a2);
        } else {
            res(LV_WARN, "TX2: attenuation reads %lu mdB, wanted %lu\n", (unsigned long)a2,
                (unsigned long)GNSS_CFG_TX2_ATTEN_MDB);
        }
    }

    section("3/8 interface clock");
    if (chk_lclk() != 0) { return abort_silent("l_clk"); }

    section("4/8 MVDR + TX path (TX1 still at maximum attenuation)");
    gnss_set_op_atten_mdb(GNSS_L1_TX_MAX_ATTEN_MDB);   /* gnss_mvdr=1 opens to MAX */
    p[0] = 1;
    set_gnss_mvdr(p, 1);
    gnss_pt_clear_counters();
    no_os_mdelay(10);
    if (chk_path() != 0) { return abort_silent("MVDR/TX path"); }
    if (chk_flow(100) != 0) { return abort_silent("data flow"); }

    section("5/8 RX gain");
#if GNSS_AUTO_RX_GAIN
    if (gnss_rx_autolevel(0) < 0) { return abort_silent("RX gain"); }
#else
    res(LV_INFO, "RX auto-level disabled (GNSS_AUTO_RX_GAIN 0): gain %ld dB\n",
        (long)gnss_plan_gain0());
#endif
    chk_level(GNSS_AUTO_RX_GAIN ? 0 : 1);

    section("6/8 output level");
    if (mv >= GNSS_PT_MVDR_VERSION_AOLC) {
        if (gnss_pt_mvdr_set_aolc(GNSS_CFG_AOLC_TARGET_DB) == GNSS_PT_OK) {
            res(LV_PASS, "AOLC on, target -%lu dBFS\n", (unsigned long)GNSS_CFG_AOLC_TARGET_DB);
        } else {
            res(LV_FAIL, "AOLC commit failed.\n");
            return abort_silent("AOLC");
        }
        no_os_mdelay(300);    /* loop settles in ~0.1 s */
    } else {
        (void)gnss_pt_mvdr_set_out_gain(2U);
        res(LV_WARN, "AOLC not in this bitstream: manual OUT_GAIN 2. Tune gnss_ogain=\n");
    }

    section("7/8 MVDR running");
    if (chk_flow(100) != 0) { return abort_silent("MVDR running"); }
    chk_mvdr_features(200);

    chk_stap(300);

    section("8/8 TX1");
    if (n_fail != 0) { return abort_silent("final check"); }
    gnss_set_op_atten_mdb(gnss_plan_atten());
    if (gnss_l1_set_tx_attenuation(ad9361_phy, gnss_plan_atten()) == GNSS_L1_OK) {
        res(LV_PASS, "TX1: opened to the operating attenuation\n");
        gnss_tx_mark_on(1);
        gnss_rep_event("TX1 ON      attenuation %lu mdB, anti-jam output", (unsigned long)gnss_plan_atten());
    } else {
        res(LV_FAIL, "TX1: operating attenuation not confirmed.\n");
        return abort_silent("TX1 open");
    }
    chk_tx(1);
    if (gnss_get_lo_hz() == 1575420000ULL) {
        res(LV_INFO, "LO = L1 (zero-IF): if C/N0 is low, suspect TX LO leakage on L1\n");
        more("To compare: gnss_lo=1584.5 (GPS then at -9.08 MHz, no rebuild).\n");
    }

    summary("MVDR AUTO-START");
    printf(" GNSS_TX: ON - the MVDR output (jammer nulled) is transmitting on TX1\n"
           "          at %lu mdB attenuation. gnss_tx? / gnss_mvdr? confirm it.\n",
           (unsigned long)gnss_plan_atten());
    if (n_warn == 0) {
        printf(" RESULT: MVDR RUNNING on TX1, everything as it should be.\n");
    } else {
        printf(" RESULT: MVDR RUNNING on TX1, %d WARNING(s) above -- it works,\n", n_warn);
        printf("         but read them: each says what to change.\n");
    }
    printf("   Enter (empty line) = one-line live status + events\n");
    printf("   gnss_health? = full check   gnss_auto=0 = TX1 silent\n");
    printf("===============================================================\n");
    monitor_armed = 1;
    mon_reset_req = 1;       /* MOD-19: fresh baseline, no events from our own restart */
    smi_mon_init  = 0;       /* MOD-25: SMI overrun / fail baseline from now */
    gnss_smi_mark_novr();    /* MOD-25: overruns counted from here (set-up done) */
    gnss_temp_poll();        /* MOD12: boot temperature (calibrations just ran) */
    gnss_monitor_poll(0);    /* baseline for the event monitor */
    settle_smi(150);         /* MOD-26: a few SMI windows for the figures */
    gnss_monitor_poll(0);    /* MOD-25: learns the no-jammer floor for gnss_check? */
    settle_smi(150);
    gnss_dash_check();       /* MOD-25: the one-page check (gnss_check? any time) */
    gnss_rep_print();        /* MOD-26: the full report (gnss_report? any time) */
    return 0;
}

int gnss_health_check(int banner_on)
{
    n_pass = n_warn = n_fail = 0;
    if (banner_on) { banner("MVDR HEALTH CHECK (read-only)"); }

    section("FPGA");
    if (chk_fpga(0) != 0) { goto done; }
    section("radio");
    if (!ad9361_phy) {
        res(LV_FAIL, "radio: ad9361_phy is NULL\n");
        goto done;
    }
    (void)chk_radio();
    (void)chk_lclk();
    section("MVDR + TX path");
    (void)chk_path();
    (void)chk_flow(100);
    section("input");
    chk_level(1);
    section("MVDR features");
    chk_mvdr_features(200);
    chk_stap(0);
    section("TX1");
    chk_tx(1);
done:
    if (banner_on) {
        summary("HEALTH");
        printf(" RESULT: %s\n", n_fail ? "PROBLEM -- see the [FAIL] lines"
                                      : (n_warn ? "working, with warnings" : "all good"));
        printf("===============================================================\n");
    }
    monitor_armed = 1;
    return n_fail;
}

/* ------------------------------------------------------------------------- */
/*  Event monitor                                                            */
/* ------------------------------------------------------------------------- */
static int guard_en;            /* clip guard, below */

typedef struct {
    int      valid;
    uint32_t ovf, unf, osat, blocks, upd, drop;
    int      clip, jam, hold, dac, path;
    uint32_t lclk;
    int      lvl_off;
} mon_t;

static mon_t mon;
static int   clip_clean_s = 0;             /* MOD-25: seconds without output clipping */
static int   lvl_reported = 0, iq_reported = 0;
static int   wb_count = 0, wb_reported = 0;     /* MOD-23: wideband jammer */
static uint32_t stap_ns_last = 0;               /* MOD-24: STAP anti-windup count */
static uint32_t smi_fail_last = 0, smi_novr_last = 0;   /* MOD-25: SMI solver / overruns */
static int      smi_mon_init = 0;                       /* baseline taken             */
static int      mon_retune = 0;                         /* MOD11: retune glitch window, monitor ticks */
static int      mon_retune_said = 0;
void gnss_mon_retune(void) { mon_retune = 4; mon_retune_said = 0; }

/* Worst (largest) IQ-image coefficient of RX1/RX2 in dBc, Q20 values. */
static double iq_worst_dbc(const gnss_pt_mvdr_state_t *s)
{
    double b1 = ((double)s->iq_b1_re * s->iq_b1_re + (double)s->iq_b1_im * s->iq_b1_im);
    double b2 = ((double)s->iq_b2_re * s->iq_b2_re + (double)s->iq_b2_im * s->iq_b2_im);
    double b  = (b1 > b2) ? b1 : b2;
    if (b <= 0.0) { return -120.0; }
    return 10.0 * log10(b / (1048576.0 * 1048576.0));
}

static void ev(const char *fmt, ...)
{
    va_list ap;
    char    line[104];
    bg_mark();
    printf("  [EVENT] ");
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    va_start(ap, fmt);              /* MOD-26: the first line goes to the report log */
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    {
        char *nl = strchr(line, '\n');
        if (nl) { *nl = 0; }
    }
    gnss_rep_event("%s", line);
}

void gnss_monitor_poll(int show_status)
{
    gnss_pt_state_t      t;
    gnss_pt_mvdr_state_t s;
    gnss_pt_jam_state_t  j;
    mon_t    m;
    uint32_t st, a = 0;
    int32_t  g = 0;
    double   lvl = 0.0;
    int      first;

    if (!monitor_armed || !mvdr_present()) {
        if (show_status) {
            printf("  (status: run gnss_auto=1 or gnss_health? first)\n");
        }
        return;
    }
    gnss_pt_get_state(&t);
    gnss_pt_mvdr_get_state(&s);
    gnss_pt_jam_get_state(&j);
    st = gnss_pt_read(GNSS_PT_REG_STATUS);

    memset(&m, 0, sizeof(m));
    m.valid  = 1;
    m.ovf    = t.overflow_count;
    m.unf    = t.underflow_count;
    m.osat   = s.n_osat;
    m.blocks = s.n_blocks;
    m.upd    = s.n_upd + s.n_hold;   /* hold-absorbed windows count */
    m.drop   = s.n_drop;
    m.clip   = (s.clip1 || s.clip2) ? 1 : 0;
    m.jam    = s.st_jam;
    m.hold   = s.st_hold;
    m.dac    = ((st & GNSS_PT_ST_DAC_EN_I0) && (st & GNSS_PT_ST_DAC_EN_Q0)) ? 1 : 0;
    m.path   = ((st & GNSS_PT_ST_PASS_EN_SYNCED) && (st & GNSS_PT_ST_MVDR_SEL_SYNCED)) ? 1 : 0;
    m.lclk   = gnss_pt_lclk_hz();
    /* MOD-25: learn the thermal floor while no jammer is present (and the
     * ADC is clean), keep the RX gain current for the suppression figures */
    if (ad9361_phy && ad9361_get_rx_rf_gain(ad9361_phy, 0, &g) == 0) {
        gnss_smi_state_t ssl;
        gnss_rep_aj_t    ral;
        memset(&ssl, 0, sizeof(ssl));
        if (gnss_smi_available()) { gnss_smi_get_state(&ssl); }
        gnss_rep_get(&ral);
        if (ssl.hw_active && ssl.updates > 0U && ral.valid && !ral.jammed && !m.clip) {
            /* MOD-26b: no jammer -> the RX1 power IS the floor (SMI window;
             * independent of the 2 x 2 covariance, which a build broke) */
            gnss_pt_thermal_learn_floor(ssl.pin_dbfs_ref, g);
        } else if (!ssl.hw_active && !s.st_jam && !j.present && !m.clip && s.have_nf && s.nf_est > 0U &&
                   gnss_pt_cov_check(&s, NULL, NULL) == 1) {
            gnss_pt_thermal_learn(10.0 * log10((double)s.nf_est /
                                  ((double)s.block_n * (double)s.cov_scale * 2048.0 * 2048.0)), g);
        } else {
            gnss_pt_thermal_gain(g);
        }
    }
    if (s.have_aolc && (s.out_gain_reg & GNSS_PT_OG_AOLC_EN)) {
        lvl = out_dbfs(&s);
        m.lvl_off = (fabs(lvl + (double)((s.out_gain_reg >> 16) & 0xFFU)) > 3.0) ? 1 : 0;
    }

    /* MOD-26: TX2 is unused: keep it at its hold attenuation */
    if (ad9361_phy && ad9361_get_tx_attenuation(ad9361_phy, 1, &a) == 0 && a != GNSS_CFG_TX2_ATTEN_MDB) {
        uint32_t was = a;
        a = gnss_tx2_hold();
        ev("TX2 attenuation was %lu mdB: restored to %lu (TX2 is unused).\n",
           (unsigned long)was, (unsigned long)a);
    }

    if (mon_reset_req) { mon.valid = 0; mon_reset_req = 0; }
    first = !mon.valid;
    if (!first) {
        if (m.dac != mon.dac) {
            ev(m.dac ? "DAC path back: TX1 carries the MVDR output again.\n"
                     : "DAC path LOST: TX1 is no longer fed by the MVDR. gnss_auto=1\n");
        }
        if (m.path != mon.path) {
            ev(m.path ? "RX -> MVDR -> TX1 selected again.\n"
                      : "MVDR no longer selected (gnss_mvdr=0 or gnss_tx=0?).\n");
        }
        if (m.path && s.mode_live == (int)GNSS_PT_MVDR_MODE_ADAPTIVE && mon.path &&
            m.upd >= mon.upd &&           /* counters cleared (gnss_mvdr=1): skip */
            (m.blocks == mon.blocks || m.upd == mon.upd)) {
            /* polled twice in quick succession? look again 5 ms later (a
             * window arrives every ~23 us, a weight update every window) */
            gnss_pt_mvdr_state_t s2;
            no_os_mdelay(5);
            gnss_pt_mvdr_get_state(&s2);
            if (s2.n_blocks == m.blocks) {
                ev("MVDR STALLED: no new covariance window in 5 ms. gnss_health?\n");
            } else if (s2.n_upd + s2.n_hold == m.upd && !(s2.have_smi && s2.smi_active)) {
                /* MOD-26b: with SMI driving TX1 the single-weight core is a
                 * spare; its idle state is reported once below, not each second */
                ev("MVDR: windows arrive but NO weight updates (n_skip %lu). gnss_health?\n",
                   (unsigned long)s2.n_skip);
            }
            m.blocks = s2.n_blocks;
            m.upd    = s2.n_upd + s2.n_hold;
        }
        {   /* MOD-26b: a corrupt 2 x 2 covariance (R11/R22), reported once */
            static int cov_reported = 0;
            double c1 = 0.0, c2 = 0.0;
            if (gnss_pt_cov_check(&s, &c1, &c2) == 0 && !cov_reported) {
                cov_reported = 1;
                ev("2x2 COVARIANCE INVALID: RX1 %ld dBFS, RX2 %ld dBFS (RX2 sum R22 corrupt).\n"
                   "          Jammer estimator / single-weight core blind; SMI is NOT affected\n"
                   "          (own covariance). MVDR 1.15 (self-healing window) fixes it: reprogram.\n",
                   (long)floor(c1 + 0.5), (long)floor(c2 + 0.5));
            }
        }
        if (guard_en) {
            /* the clip guard reports clipping itself, with what it did */
        } else if (m.clip && !mon.clip) {
            ev("ADC CLIPPING started (RX1 %lu / RX2 %lu). A clipped jammer is not\n"
               "          nulled: gnss_rxauto=1\n",
               (unsigned long)s.clip1, (unsigned long)s.clip2);
        } else if (!m.clip && mon.clip) {
            ev("ADC clipping stopped.\n");
        }
        if (m.jam && !mon.jam) {
            ev("JAMMER DETECTED -- nulling it");
            if (j.available && j.valid && j.present) {
                printf(" (JNR "); pdb((double)j.jnr_db_q8 / 256.0);
                printf(" dB, angle "); pdb((double)j.theta_cdeg / 100.0); printf(" deg)");
            }
            printf(".\n");
        } else if (!m.jam && mon.jam) {
            ev("jammer gone%s.\n", m.hold ? ": null held, then gliding back" : "");
        }
        if (mon_retune > 0 && (m.ovf != mon.ovf || m.unf != mon.unf)) {
            if (!mon_retune_said) {               /* MOD11: expected, once */
                ev("RX filter retune: brief FIFO / SMI-window glitch (expected; null rebuilt).\n");
                mon_retune_said = 1;
            }
        } else if ((m.ovf > mon.ovf || m.unf > mon.unf) && m.ovf >= mon.ovf && m.unf >= mon.unf) {
            ev("FIFO overflow +%lu / underflow +%lu: RX and TX rates differ.\n",
               (unsigned long)(m.ovf - mon.ovf), (unsigned long)(m.unf - mon.unf));
        }
        /* report only above 1e-4 of the samples since the last check: a
         * handful of rail hits per second are Gaussian noise peaks */
        gnss_rep_aj_t rt;
        gnss_rep_get(&rt);
        if (m.osat > mon.osat &&           /* a cnt_clear resets it: no event then */
            (m.osat - mon.osat) > (uint32_t)(gnss_pt_get_fs() / 10000UL) &&
            ((rt.valid && rt.jammed && gnss_rep_uptime_s() - rt.since_s < 3.0) || agc_since_change < 20)) {
            /* MOD-27: a jammer just appeared or the RX gain just stepped: the
             * output clips until the first null and the AOLC follow-up (a few
             * ms). That is a transient, not a level problem: no back-off
             * (field 5 Nov: one onset burst held TX1 2..6 dB low for minutes) */
            ev("OUTPUT CLIPPING: %lu samples during a jammer onset / RX gain step (transient, no back-off).\n",
               (unsigned long)(m.osat - mon.osat));
        } else if (m.osat > mon.osat &&
            (m.osat - mon.osat) > (uint32_t)(gnss_pt_get_fs() / 10000UL)) {
            /* MOD-25: clipping the DAC distorts every satellite on TX1: back
             * the AOLC target off 2 dB per second of clipping (max 12 dB) */
            int b0 = (int)gnss_pt_aolc_backoff_db(), b = gnss_pt_aolc_backoff(+2);
            ev("OUTPUT CLIPPING: %lu samples hit the 12-bit DAC rails.\n",
               (unsigned long)(m.osat - mon.osat));
            if (b > b0) {
                printf("          AOLC backed off to %d dB below its target (TX1 level %d dB lower).\n",
                       b, b);
            } else if (b >= 0) {
                printf("          AOLC back-off already at its %d dB limit: residue or bursts too large\n"
                       "          for the output; gnss_supp?, gnss_agc?\n", b);
            }
            clip_clean_s = 0;
        } else if (gnss_pt_aolc_backoff_db() > 0U && ++clip_clean_s >= 10) {
            /* MOD-27: 10 s without clipping: give 1 dB back (was 30 s) */
            clip_clean_s = 0;
            (void)gnss_pt_aolc_backoff(-1);
        }
        if (m.drop > mon.drop) {
            ev("%lu weight sets dropped.\n", (unsigned long)(m.drop - mon.drop));
        }
        if (labs((long)m.lclk - (long)mon.lclk) > (long)(mon.lclk / 50U)) {
            ev("l_clk changed: "); pmhz(mon.lclk); printf(" -> "); pmhz(m.lclk);
            printf(" (sample rate changed?)\n");
        }
        /* AOLC needs ~0.1 s after any gain change: report only when it is
         * off target on two checks in a row */
        if (m.lvl_off && mon.lvl_off && !lvl_reported) {
            ev("output level off target ("); pdb(lvl);
            printf(" dBFS): AOLC at a gain limit? gnss_rxauto=1\n");
            lvl_reported = 1;
        } else if (!m.lvl_off && lvl_reported) {
            ev("output level back on target.\n");
            lvl_reported = 0;
        }
        /* MOD-24: STAP weights pinned at their +/-16 limit: only an extreme or
         * clipped jammer does that (the clip guard should prevent clipping) */
        if (s.have_stap && s.stap_active && s.stap_nwsat > stap_ns_last + 1000U &&
            s.stap_nwsat >= stap_ns_last) {
            ev("STAP anti-windup: +%lu limited updates (weights at +/-16).\n"
               "          Clipped or extreme jammer? gnss_agc?, gnss_supp?\n",
               (unsigned long)(s.stap_nwsat - stap_ns_last));
        }
        stap_ns_last = s.stap_nwsat;
        /* MOD-25: SMI windows that could not be solved, sample overruns */
        if (s.have_smi && s.smi_active) {
            gnss_smi_state_t ss;
            gnss_smi_get_state(&ss);
            if (ss.fails > smi_fail_last) {
                ev("SMI: %lu window(s) not solved (covariance not positive definite).\n"
                   "          Corrupt samples? gnss_smi?, gnss_health?\n",
                   (unsigned long)(ss.fails - smi_fail_last));
            }
            if (!smi_mon_init) { smi_novr_last = ss.novr; smi_fail_last = ss.fails; smi_mon_init = 1; }
            if (ss.novr != smi_novr_last && mon_retune > 0) {
                if (!mon_retune_said) {           /* MOD11 */
                    ev("RX filter retune: brief FIFO / SMI-window glitch (expected; null rebuilt).\n");
                    mon_retune_said = 1;
                }
            } else if (ss.novr != smi_novr_last) {
                ev("SMI: %lu sample overrun(s): samples closer than 4 clocks (needs 2R2T).\n",
                   (unsigned long)((ss.novr - smi_novr_last) & 0xFFFFU));
            }
            smi_fail_last = ss.fails;
            smi_novr_last = ss.novr;
        }
        /* MOD-23: a jammer the 2-element null cannot take below the noise
         * because RX1/RX2 differ across the band (wideband / barrage) */
        if (m.jam && s.cov_coherent && s.cov_r11 > 0 && s.cov_r22 > 0 && s.have_nf && s.nf_est > 0) {
            double r11 = (double)s.cov_r11, r22 = (double)s.cov_r22;
            double c2  = ((double)s.cov_r12_re * s.cov_r12_re + (double)s.cov_r12_im * s.cov_r12_im)
                         / (r11 * r22);
            double nf  = (double)s.nf_est, jnr, lim;
            if (c2 > 0.999999) { c2 = 0.999999; }
            jnr = (r11 > nf) ? 10.0 * log10((r11 - nf) / nf) : 0.0;
            lim = -10.0 * log10(1.0 - c2);
            if (jnr > lim + 6.0 && GNSS_PT_ST_ACTIVE(&s)) {
                /* MOD-24: STAP equalises RX1/RX2 across the band: report once */
                if (++wb_count == 3 && !wb_reported) {
                    double pr, jn, ac; int lo;
                    ev("WIDEBAND JAMMER ("); pdb(jnr);
                    printf(" dB over the noise): one weight would stop at "); pdb(lim);
                    printf(" dB;\n          %s is equalising RX1/RX2 across the band.", GNSS_PT_ST_NAME(&s));
                    if (gnss_pt_supp_summary(&pr, &jn, &ac, &lo) == 0) {
                        printf(" ACTUAL cancellation %s", lo ? ">" : ""); pdb(ac);
                        printf(" dB.");
                    }
                    printf(" gnss_supp?\n");
                    wb_reported = 1;
                }
            } else if (jnr > lim + 6.0) {
                if (++wb_count == 3 && !wb_reported) {
                    ev("WIDEBAND JAMMER: null limited to "); pdb(lim);
                    printf(" dB, the jammer is "); pdb(jnr);
                    printf(" dB over the noise.\n"
                           "          RX1/RX2 differ across the band, so ~"); pdb(jnr - lim);
                    printf(" dB of it stays and the\n"
                           "          satellites go under it. %s\n"
                           "          Or narrow the band: gnss_band=4 (20 MHz) / gnss_band=5 (8 MHz);\n"
                           "          match RX1/RX2 antennas, LNAs, cables.\n",
                           gnss_smi_available()     ? "Use SMI-STAP: gnss_null=3." :
                           gnss_pt_stap_available() ? "Use STAP: gnss_null=2."
                                                    : "STAP needs the MVDR v1.13 bitstream.");
                    wb_reported = 1;
                }
            } else {
                wb_count = 0;
            }
        } else {
            wb_count = 0;
            if (!m.jam) { wb_reported = 0; }
        }

        /* MOD-20: switch it OFF by itself. A wrong coefficient (seen: -18 ..
         * -28 dBc in the field with a barrage jammer near the LO) ADDS a
         * mirrored copy of the jammer to both channels -- a second "jammer"
         * a 2-element array cannot null as well. The AD9361's own image
         * (~ -45..-50 dBc) is far less harmful. gnss_iq=1 / gnss_band= turn
         * it back on. */
        if (s.have_iq && !(s.iq_ctrl & GNSS_PT_IQ_OFF) && iq_worst_dbc(&s) > -35.0) {
            double dbc = iq_worst_dbc(&s);
            (void)gnss_pt_mvdr_set_iq(0);
            ev("IQ-image correction estimate "); pdb(dbc);
            printf(" dBc is implausible (AD9361:\n"
                   "          about -50): a jammer near the LO misleads it. Correction\n"
                   "          switched OFF so it cannot add a mirrored jammer (gnss_iq=1\n"
                   "          turns it on again; LO off the jammer, e.g. gnss_band=2, avoids it).\n");
            iq_reported = 1;
        }
    }
    mon = m;
    if (mon_retune > 0) { mon_retune--; }   /* MOD11 */

    if (show_status) {
        ad9361_get_tx_attenuation(ad9361_phy, 0, &a);
        ad9361_get_rx_rf_gain(ad9361_phy, 0, &g);
        gnss_rep_aj_t ra;
        gnss_rep_get(&ra);
        printf("  STATUS: %s",
               (m.dac && m.path && s.mode_live == (int)GNSS_PT_MVDR_MODE_ADAPTIVE)
                   ? ((s.have_smi && s.smi_active) ? "SMI ON" :
                      (s.have_stap && s.stap_active) ? "STAP ON" : "MVDR ON") : "MVDR NOT RUNNING");
        if (ra.valid && s.have_smi && s.smi_active) {
            gnss_rep_status_field();          /* MOD-26: J/N, suppression, residual */
        } else {
            printf(" | jam %s", m.jam ? "YES" : (m.hold ? "held" : "no"));
        }
        printf(" | RX %ld dB", (long)g);
        if (j.have_ch && j.n1_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE) {
            printf(" noise "); pdb((double)j.n1_q8 / 256.0); printf("/");
            pdb((double)j.n2_q8 / 256.0);
        }
        printf(" | clip %lu/%lu", (unsigned long)s.clip1, (unsigned long)s.clip2);
        if (s.have_aolc) { printf(" | out "); pdb(out_dbfs(&s)); }
        if (m.jam && !(ra.valid && s.have_smi && s.smi_active)) {
            /* MOD-24: print / JNR / ACTUAL (gnss_supp? explains them) */
            double pr, jn, ac; int lo;
            if (gnss_pt_supp_summary(&pr, &jn, &ac, &lo) == 0) {
                printf(" | supp "); pdb(pr); printf("/"); pdb(jn);
                printf("/%s", lo ? ">" : ""); pdb(ac); printf(" dB");
            }
        }
        printf(" | TX %lu mdB", (unsigned long)a);
        printf(" | l_clk "); pmhz(m.lclk);
        printf("%s\n", (m.dac && m.path && !m.clip && !m.lvl_off) ? " | OK" : " | CHECK");
    }
}

/* ------------------------------------------------------------------------- */
/*  MVDR-safe automatic RX gain (AGC) + background tick                      */
/* ------------------------------------------------------------------------- */
/* One gain for RX1 AND RX2, always equal and manual at the AD9361 (its own
 * AGCs move the channels independently and break the null). Two modes:
 *
 * AUTO (default, gnss_agc=1, rx1_gc_mode=2):
 *   the gain is always the HIGHEST the jammer allows, up to the best-C/N0
 *   target (antenna noise 15 dB above the AD9361 floor measured at start):
 *   - every 10 ms: ADC clipping or a peak above -1 dBFS -> -6 dB at once
 *   - every 100 ms: peak above -3 dBFS -> -1 dB (keeps 3 dB headroom)
 *   - every 0.5 s: noise below target and the peak would stay <= -6 dBFS
 *     -> up to +3 dB; noise more than 3 dB above target, no jammer -> down
 *   So with a jammer the gain sits 3..6 dB below clipping (the least C/N0
 *   lost), and without one it climbs back to the target by itself.
 * MANUAL (gnss_agc=0, rxN_rf_gain=, gnss_rxgain=):
 *   the gain you set; clipping still lowers it, and it returns to your
 *   value once there is headroom.
 * gnss_guard=0 switches off even the clipping protection.
 *
 * AOLC follows every step (~19 ms), so TX1 keeps its level throughout. */
static int      guard_en  = GNSS_CFG_CLIP_GUARD;   /* any automatic action  */
static int      agc_auto  = 1;                     /* AUTO / MANUAL         */
int gnss_agc_is_auto(void) { return agc_auto; }    /* MOD-25: gnss_check?   */
static int32_t  agc_man   = -1;                    /* MANUAL gain           */
static int32_t  agc_cur   = -1;                    /* gain we last set/saw  */
static int32_t  guard_auto_gain = -1;              /* auto-level result     */
/* MOD-26: for gnss_report? */
static int32_t  agc_clean = -1;                    /* gain at target, no jammer */
static char     agc_why[96] = "not running yet";
int32_t     gnss_agc_gain_now(void)   { return agc_cur; }
int32_t     gnss_agc_clean_gain(void) { return agc_clean; }
const char *gnss_agc_reason(void)     { return agc_why; }
static int      agc_slow_n = 0, agc_hold_n = 0, agc_quiet_n = 0;
static int      agc_min_warned = 0;
static int32_t  agc_report_from = -1;              /* pending summary       */
static uint32_t agc_steps_down = 0, agc_steps_up = 0;
/* MOD-31: anti-hunting. Every AD9361 gain step is an RF phase / amplitude
 * step on every satellite; a gain that goes up, clips, goes down and up
 * again keeps the receiver re-acquiring (slow lock with a jammer). A
 * clipping step-down within 3 s of a step-up is a "hunt": further step-ups
 * are then locked out for 10, 20, 40, 80 s (doubling each hunt). The lock
 * clears as soon as the jammer is gone, so the gain still returns fast. */
static int      agc_up_age  = 1000;       /* 100 ms ticks since the last step-up */
static int      agc_up_lock = 0;          /* 100 ms ticks of step-up lock-out left */
static uint32_t agc_hunts   = 0, agc_hunts_total = 0;
static int      agc_clear_n = 0;          /* 100 ms ticks without a jammer        */
uint32_t gnss_agc_hunts(void) { return agc_hunts_total; }

int gnss_guard_is_on(void) { return guard_en; }

/* MANUAL at g (gnss_rxgain=, rxN_rf_gain= with MVDR running) */
void gnss_guard_set_op_gain(int32_t g)
{
    agc_auto = 0;
    agc_man  = g;
    agc_cur  = g;
    agc_hold_n = 0;
}

static void agc_go_auto(int32_t g)
{
    agc_auto = 1;
    agc_cur  = g;
    agc_hold_n = 0;
}

/* AUTO (rxN_gc_mode=1/2/3 with MVDR running) */
void gnss_guard_auto(void)
{
    int32_t g = 0;
    guard_en = 1;
    if (ad9361_phy) { ad9361_get_rx_rf_gain(ad9361_phy, 0, &g); }
    agc_go_auto(g);
    printf("  RX gain AUTO (MVDR-safe): climbs to the best-C/N0 target, backs off\n"
           "  only as far as a jammer needs. gnss_agc? shows it.\n");
}

void gnss_agc_set_mode(int autom)
{
    int32_t g = 0;
    if (ad9361_phy) { ad9361_get_rx_rf_gain(ad9361_phy, 0, &g); }
    guard_en = 1;
    if (autom) { agc_go_auto(g); } else { gnss_guard_set_op_gain(g); }
}

void gnss_guard_enable(int en)
{
    int32_t g = 0;
    guard_en = en ? 1 : 0;
    if (ad9361_phy && ad9361_get_rx_rf_gain(ad9361_phy, 0, &g) == 0) { agc_cur = g; }
}

void gnss_agc_print(void)
{
    int32_t g = 0;
    uint32_t pk = gnss_pt_read(GNSS_PT_REG_MVDR_COV_PEAK);
    uint32_t nn = gnss_pt_read(GNSS_PT_REG_NOISE_CH);
    uint32_t p1 = pk & 0xFFFU, p2 = (pk >> 16) & 0xFFFU;
    if (ad9361_phy) { ad9361_get_rx_rf_gain(ad9361_phy, 0, &g); }
    printf("GNSS_AGC: %s, RX1/RX2 gain %ld dB\n",
           !guard_en ? "OFF (no automatic gain changes at all; gnss_guard=1)"
           : agc_auto ? "AUTO (MVDR-safe)" : "MANUAL (clipping still protected)", (long)g);
    if (agc_auto) {
        printf("  target noise "); pdb(rx_target_eff);
        printf(" dBFS (best C/N0; floor + %ld dB)\n", (long)GNSS_CFG_RX_FLOOR_MARGIN_DB);
    } else {
        printf("  manual gain %ld dB%s\n", (long)agc_man,
               (g < agc_man) ? "  (lowered: a strong signal needs the headroom)" : "");
    }
    if ((nn & 0xFFFFU) != 0x8000U) {
        printf("  noise RX1/RX2 "); pdb((double)(int16_t)(nn & 0xFFFFU) / 256.0);
        printf(" / "); pdb((double)(int16_t)(nn >> 16) / 256.0); printf(" dBFS\n");
    }
    printf("  ADC peak "); pdb(peak_dbfs((p1 > p2) ? p1 : p2));
    printf(" dBFS (clips at 0; with a jammer AUTO keeps it at -4..0, <= 0.1 %% clipped)\n");
    printf("  steps down %lu, up %lu since boot; %lu in the last minute\n",
           (unsigned long)agc_steps_down, (unsigned long)agc_steps_up,
           (unsigned long)agc_changes_min);
    printf("  anti-hunt: %lu hunts since boot%s", (unsigned long)agc_hunts_total,
           agc_up_lock > 0 ? "" : "\n");
    if (agc_up_lock > 0) { printf(", step-ups locked for %d s more\n", (agc_up_lock + 9) / 10); }
    printf("  (few steps = steady phase for the receiver: best for lock)\n");
}

/* compatibility: gnss_guard? */
void gnss_guard_print(void) { gnss_agc_print(); }

static int mvdr_on_tx(void)
{
    uint32_t st = gnss_pt_read(GNSS_PT_REG_STATUS);
    return ((st & GNSS_PT_ST_PASS_EN_SYNCED) && (st & GNSS_PT_ST_MVDR_SEL_SYNCED)) ? 1 : 0;
}

/* MOD-18: AOLC is sped up for 150 ms after every gain step (see
 * gnss_pt_mvdr_aolc_fast): the output, and every satellite in it, is back at
 * its level in ~3 ms instead of ~40 ms. */
static int      agc_aolc_fast_n = 0;      /* 10 ms ticks left in fast mode    */
static int32_t  agc_pk_hold = 0;          /* max ADC peak (LSB), this 0.5 s   */
static int32_t  agc_pk_prev = 0;          /* ... and the previous 0.5 s       */
static int      agc_pk_age  = 0;
static int      agc_since_change = 1000;  /* 100 ms ticks since a gain step   */
static uint32_t agc_changes_min = 0, agc_changes_cnt = 0;
static int      agc_min_n = 0;
static int      agc_clip_hits = 0;        /* fast checks with any clipping    */

/* LVDS / ant board: after a fast step-down the clip counter of the next
 * windows can still be from before the step, so the fast loop waits 3 ticks
 * (30 ms) before it may step again: one cut per real overload, not a cascade
 * (a cascade over-cut, then needed several step-ups = several RF phase steps
 * on every satellite while the receiver was trying to hold lock). */
static int agc_fast_blank = 0;

/* LVDS / ant board: barrage AGC on the clip RATE instead of the peak.
 * A noise-like jammer has Gaussian tails: its 1-s peak sits 11-13 dB over
 * its rms, so a peak limit keeps the gain ~5 dB lower than the ADC needs.
 * Clipping 1e-4 of the samples clips at ~3.9 sigma: the clipped energy is
 * ~60 dB under the jammer, far under the null depth, and the gain (= the
 * satellite C/N0 when the AD9361 floor limits it) rises 4-6 dB.
 * agc_clip_ppm = allowed clip rate (gnss_agcclip=ppm, 0 = old peak rule). */
static int      agc_clip_ppm = 100;
static int      agc_bar = 0;              /* barrage rule active (slow loop)   */
static int      agc_bar_slow = 0;         /* slow ticks: rise every 5 s after a cut */
static int      agc_jam_hold = 0;         /* MOD6: slow ticks the jammer still counts as present */
static int      agc_pulse = 0;            /* MOD6: SMI pulse mode                     */
static uint32_t agc_cs = 0, agc_cw = 0;   /* clipped samples / windows, 100 ms */
static uint32_t agc_rs[20], agc_rw[20];   /* last 2 s, per 100 ms              */
static int      agc_ri = 0;
static double   agc_rate1 = 0.0, agc_rate2 = 0.0, agc_rate01 = 0.0;   /* ppm   */
int  gnss_agc_set_clip_ppm(int ppm)
{
    if (ppm < 0 || ppm > 2000) { return -1; }
    agc_clip_ppm = ppm;
    return 0;
}
int  gnss_agc_get_clip_ppm(void) { return agc_clip_ppm; }
int  gnss_agc_barrage_rule(double *rate_ppm_1s)
{
    if (rate_ppm_1s) { *rate_ppm_1s = agc_rate1; }
    return agc_bar;
}

/* LVDS / ant board: RX2 gain = RX1 gain - rx2_off (gnss_rxoff=dB): one
 * channel hotter than the other (antenna LNA, cable) sets the ADC headroom
 * of both; with the offset both reach the ADC limit together. The MVDR
 * weights absorb a flat gain difference; the STAP taps a small band shape. */
static int32_t  rx2_off = 0;
int  gnss_rx2_off_set(int db)
{
    if (db < -20 || db > 20) { return -1; }
    rx2_off = db;
    return 0;
}
int  gnss_rx2_off_get(void) { return (int)rx2_off; }
static int32_t rx2_off_v(void) { return rx2_off; }

/* LVDS / ant board: step-up with a jammer, gnss_agcstep=dB,s */
static long agc_jstep_db = 1;              /* dB per step   (1..15) */
static int  agc_jstep_s  = 1;              /* s between steps (1..10) */
int gnss_agc_set_jstep(int db, int sec)
{
    if (db < 1 || db > 15 || sec < 1 || sec > 10) { return -1; }
    agc_jstep_db = db;
    agc_jstep_s  = sec;
    return 0;
}
void gnss_agc_get_jstep(int *db, int *sec)
{
    if (db)  { *db  = (int)agc_jstep_db; }
    if (sec) { *sec = agc_jstep_s; }
}

static void agc_apply(int32_t g, int32_t ng, int up)
{
    if (ng > GNSS_CFG_RX_GAIN_MAX_DB) { ng = GNSS_CFG_RX_GAIN_MAX_DB; }
    if (ng < GNSS_CFG_RX_GAIN_MIN_DB) { ng = GNSS_CFG_RX_GAIN_MIN_DB; }
    if (ng == g) { return; }
    if (agc_aolc_fast_n == 0) { (void)gnss_pt_mvdr_aolc_fast(1); }
    agc_aolc_fast_n = 15;
    set_rx_gain_both(ng);
    agc_cur = ng;
    if (up) { agc_steps_up++; } else { agc_steps_down++; }
    if (up) {
        agc_up_age = 0;
    } else if (agc_up_age < 30) {             /* MOD-31: went up, then had to come down: a hunt */
        int sh = (agc_hunts < 3U) ? (int)agc_hunts : 3;
        agc_hunts++;
        agc_hunts_total++;
        agc_up_lock = 100 << sh;              /* 10, 20, 40, 80 s */
        agc_up_age = 1000;
    }
    if (agc_report_from < 0) { agc_report_from = g; }
    agc_quiet_n = 0;
    agc_since_change = 0;
    agc_changes_cnt++;
    if (!up) { agc_pk_hold = 0; agc_pk_prev = 0; agc_pk_age = 0; agc_fast_blank = 3; }  /* stale now */
}

/* every 10 ms: clipping -> back off at once, by how badly it clips */
static void agc_fast(void)
{
    uint32_t c  = gnss_pt_read(GNSS_PT_REG_MVDR_COV_CLIP);
    uint32_t p  = gnss_pt_read(GNSS_PT_REG_MVDR_COV_PEAK);
    uint32_t c1 = c & 0xFFFFU, c2 = c >> 16, cm = (c1 > c2) ? c1 : c2;
    uint32_t p1 = p & 0xFFFU, p2 = (p >> 16) & 0xFFFU, pk = (p1 > p2) ? p1 : p2;
    int32_t  g = 0, step;

    if (agc_aolc_fast_n > 0 && --agc_aolc_fast_n == 0) {
        (void)gnss_pt_mvdr_aolc_fast(0);          /* back to the normal loop */
    }
    /* peak max-hold over ~1 s: the slow decisions use it, so a noise-like
     * jammer's peak (which varies window to window) does not make the gain
     * hunt -- every AD9361 gain step is also a small RF phase step, which
     * the receiver's tracking loops do not like */
    if ((int32_t)pk > agc_pk_hold) { agc_pk_hold = (int32_t)pk; }
    if (++agc_pk_age >= 50) {                     /* two 0.5-s buckets */
        agc_pk_prev = agc_pk_hold;
        agc_pk_hold = 0;
        agc_pk_age  = 0;
    }
    /* MOD-20: a few clipped samples per window are allowed (<= 8 of 8192 =
     * 0.1 %): with a noise-like jammer that is ~3 dB more RX gain, i.e. ~3 dB
     * more satellite C/N0 when the AD9361 floor limits it, for clipping
     * distortion ~50 dB under the jammer (well under the null depth). */
    if (agc_fast_blank > 0) {                 /* LVDS: window may predate the last step */
        agc_fast_blank--;
        return;
    }
    if (cm > 0U) { agc_clip_hits++; }
    agc_cs += cm;
    agc_cw++;
    if (agc_bar) {
        /* barrage rule: a few clipped samples per window are the operating
         * point; cut at once only when the jammer clearly stepped up */
        if (cm <= 41U) { return; }
    } else if (cm <= 8U) { return; }
    if (ad9361_get_rx_rf_gain(ad9361_phy, 0, &g) != 0) { return; }
    if (g <= GNSS_CFG_RX_GAIN_MIN_DB) {
        if (!agc_min_warned) {
            agc_min_warned = 1;
            gnss_rep_event("ADC CLIP    at the MINIMUM RX gain: jammer too strong for the ADC (attenuate RX1+RX2)");
            bg_mark();
            printf("  [AGC] ADC clipping at the MINIMUM RX gain: the jammer is too strong\n"
                   "        for the ADC and cannot be nulled while it clips. Add an\n"
                   "        attenuator in front of RX1 AND RX2, or move the jammer away.\n");
        }
        return;
    }
    agc_min_warned = 0;
    /* clipped share of the 8192-sample window says how far over it is */
    if (agc_bar) {
        step = (cm > 410U) ? 10 : 4;        /* barrage: rate rule fine-tunes after */
        agc_bar_slow = 600;                 /* 60 s of slow rises after a cut      */
    }
    else if (cm > 410U) { step = 15; }      /* > 5 %   */
    else if (cm > 41U)  { step = 10; }      /* > 0.5 % */
    else                { step = 6;  }      /* > 0.1 % */
    agc_apply(g, g - step, 0);
    agc_hold_n = 0;
}

/* every 100 ms */
static void agc_slow(void)
{
    uint32_t nn = gnss_pt_read(GNSS_PT_REG_NOISE_CH);
    uint32_t st = gnss_pt_read(GNSS_PT_REG_MVDR_STATUS);
    double   pk = peak_dbfs((uint32_t)((agc_pk_hold > agc_pk_prev) ? agc_pk_hold : agc_pk_prev)), nf = 0.0;
    int      have_nf = ((nn & 0xFFFFU) != 0x8000U && (nn >> 16) != 0x8000U) ? 1 : 0;
    /* MOD-26: with SMI running the jammer decision is the SMI one (J/N from
     * the window it solves): the single-weight core's HOLD state lingers
     * after a jammer is gone and used to keep the gain rising slowly */
    gnss_rep_aj_t ra;
    int      jam;
    long     room, want, step;
    double   pk_now, peak_lim = -4.0;      /* MOD-27: ADC peak the gain may reach */
    int32_t  g = 0;
    int      clip_hits;

    gnss_rep_get(&ra);
    jam = ra.valid ? ra.jammed : ((st & (GNSS_PT_MVDR_ST_JAM | GNSS_PT_MVDR_ST_HOLD)) ? 1 : 0);
    {
        /* MOD6: a pulsed jammer is "gone" between pulses; rising then (0.3 s,
         * up to 10 dB) and cutting at the next pulse made the gain hunt. The
         * jammer counts as present for 3 s after it was last seen, and all
         * through SMI pulse mode. */
        gnss_smi_state_t sp;
        gnss_smi_get_state(&sp);
        agc_pulse = sp.pulse_mode;
        if (jam || agc_pulse) { agc_jam_hold = 30; jam = 1; }
        else if (agc_jam_hold > 0) { agc_jam_hold--; jam = 1; }
    }
    /* without a jammer the current 0.5-s peak bucket is enough (the last
     * jammer peak must not hold the gain down for another second) */
    pk_now = (!jam && agc_pk_age >= 10) ? peak_dbfs((uint32_t)agc_pk_hold) : pk;
    if (jam) {                                           /* MOD-31: lock-out ends with the jammer */
        agc_clear_n = 0;
    } else if (++agc_clear_n >= 10) {
        agc_up_lock = 0;
        agc_hunts = 0;
    }
    if (ad9361_get_rx_rf_gain(ad9361_phy, 0, &g) != 0) { return; }
    if (agc_cur >= 0 && g != agc_cur) {
        /* changed by someone else (a vendor command outside MVDR, a script) */
        bg_mark();
        printf("  [AGC] RX gain changed to %ld dB from outside: MANUAL at %ld dB\n"
               "        (gnss_agc=1 returns to AUTO)\n", (long)g, (long)g);
        gnss_guard_set_op_gain(g);
        return;
    }
    agc_cur = g;
    if (agc_since_change < 1000) { agc_since_change++; }
    if (agc_up_age < 1000) { agc_up_age++; }            /* MOD-31 */
    if (agc_up_lock > 0)   { agc_up_lock--; }
    if (++agc_min_n >= 600) {                         /* gain steps per minute */
        agc_min_n = 0;
        agc_changes_min = agc_changes_cnt;
        agc_changes_cnt = 0;
    }
    if (have_nf) {
        double n1 = (double)(int16_t)(nn & 0xFFFFU) / 256.0, n2 = (double)(int16_t)(nn >> 16) / 256.0;
        nf = 0.5 * (n1 + n2);
        /* MOD-26b: per-channel noise must be plausible (see rx_measure) */
        if ((n2 > -3.0 || n2 - n1 > 15.0) && n1 <= -3.0)      { nf = n1; }
        else if ((n1 > -3.0 || n1 - n2 > 15.0) && n2 <= -3.0) { nf = n2; }
        else if (n1 > -3.0 && n2 > -3.0)                     { have_nf = 0; }
    }
    if (ra.valid) {
        /* MOD-26b: with SMI running, its window is the reference: without a
         * jammer the RX1 power IS the noise; with one, the learned floor.
         * It does not depend on the 2 x 2 covariance (R22) at all. */
        gnss_smi_state_t sa;
        gnss_smi_get_state(&sa);
        if (sa.hw_active && sa.updates > 0U) {
            nf = ra.jammed ? ra.nf_dbfs : sa.pin_dbfs_ref;
            have_nf = 1;
            /* MOD-27: a noise-like jammer (barrage) has Gaussian peaks: the
             * window maximum sits ~10 dB over its rms, and a sample on the
             * rail is a rare tail event, not distortion. Let its peaks run
             * to -2 dBFS (a few clipped samples per window are tolerated by
             * the fast loop): ~2.5 dB more RX gain = ~2.5 dB less C/N0 lost
             * to the AD9361 floor. CW keeps the -4 dBFS margin. */
            /* MOD7: the barrage rule (peak -2 dBFS + clip rate) only for NOISE
             * (wideband / band-limited). CW, multi-tone, chirp and sawtooth have a
             * steady envelope: clipping them gains nothing, so -4 dBFS. */
            if (jam && sa.jtype == GNSS_JT_WIDE) { peak_lim = -2.0; }
        }
    }
    /* Hysteresis, so the gain does not hunt (each step costs the receiver a
     * small phase step):  down at a peak above -2 dBFS (1 dB, every 0.3 s);
     * up only when the 1-s peak would stay <= -7 dBFS after the raise and
     * nothing has changed for 2 s; noise must be >= 2 dB under the target. */
    clip_hits = agc_clip_hits;
    agc_clip_hits = 0;
    {
        /* clip rate, ppm of the samples, over 0.1 / 1 / 2 s */
        uint32_t s1 = 0, w1 = 0, s2 = 0, w2 = 0;
        int i, k;
        agc_rs[agc_ri] = agc_cs; agc_rw[agc_ri] = agc_cw;
        agc_rate01 = agc_cw ? 1e6 * (double)agc_cs / (8192.0 * (double)agc_cw) : 0.0;
        agc_cs = 0; agc_cw = 0;
        for (i = 0; i < 20; i++) {
            k = (agc_ri - i + 20) % 20;
            if (i < 10) { s1 += agc_rs[k]; w1 += agc_rw[k]; }
            s2 += agc_rs[k]; w2 += agc_rw[k];
        }
        agc_ri = (agc_ri + 1) % 20;
        agc_rate1 = w1 ? 1e6 * (double)s1 / (8192.0 * (double)w1) : 0.0;
        agc_rate2 = w2 ? 1e6 * (double)s2 / (8192.0 * (double)w2) : 0.0;
    }
    agc_bar = (agc_auto && jam && peak_lim > -4.0 && agc_clip_ppm > 0) ? 1 : 0;
    if (agc_bar) {
        double t = (double)agc_clip_ppm;
        long   want_b = have_nf ? (long)floor(rx_target_eff - nf + 0.5) : 0;
        if (agc_bar_slow > 0) { agc_bar_slow--; }
        if (agc_rate01 > 20.0 * t && agc_since_change >= 2) {
            agc_bar_slow = 600;
            agc_apply(g, g - 2, 0);
            sprintf(agc_why, "barrage: clipping %.0f ppm (limit %d): -2 dB", agc_rate01, agc_clip_ppm);
        } else if (agc_rate1 > 6.0 * t && agc_since_change >= 5) {      /* MOD6: was 5x */
            agc_bar_slow = 600;
            agc_apply(g, g - 1, 0);
            sprintf(agc_why, "barrage: clipping %.0f ppm (limit %d): -1 dB", agc_rate1, agc_clip_ppm);
        } else if (want_b < 2) {
            sprintf(agc_why, "barrage: at the target noise (clipping %.0f ppm)", agc_rate1);
        } else if (agc_up_lock > 0) {
            sprintf(agc_why, "barrage: held %ld dB under the target: anti-hunt lock, %d s left",
                    want_b, (agc_up_lock + 9) / 10);
        } else if (agc_rate2 < 0.15 * t &&                              /* MOD6: was 0.25x */
                   agc_since_change >= ((agc_bar_slow || agc_pulse) ? 50 : 10 * agc_jstep_s) &&
                   agc_up_age >= ((agc_bar_slow || agc_pulse) ? 50 : 10 * agc_jstep_s)) {
            /* after a cut the gain sits near the jammer's limit: creep up
             * every 5 s for 60 s instead of every second (fewer steps, each
             * step briefly costs null depth) */
            step = (want_b < agc_jstep_db) ? want_b : agc_jstep_db;
            if (step > 2) { step = 2; }          /* tails: never more than 2 dB at once */
            agc_apply(g, g + (int32_t)step, 1);
            sprintf(agc_why, "barrage: clipping %.1f ppm (limit %d): +%ld dB", agc_rate2, agc_clip_ppm, step);
        } else {
            sprintf(agc_why, "barrage: held %ld dB under the target, clipping %.0f ppm (limit %d ppm)",
                    want_b, agc_rate1, agc_clip_ppm);
        }
    } else if (clip_hits >= ((peak_lim > -4.0) ? 6 : 3)) {   /* light clipping in 3 (barrage: 6) of 10 checks */
        if (agc_since_change >= 2) {         /* MOD-27: every 0.2 s, 2 dB when clipping often */
            agc_apply(g, g - ((clip_hits >= 8) ? 2 : 1), 0);
            sprintf(agc_why, "lowering: light ADC clipping");
        }
    } else if (agc_since_change >= (jam ? (agc_pulse ? 50 : 10) : 3)) {   /* MOD-26: 1 s with a jammer (MOD6: pulsed 5 s), else 0.3 s */
        room = (long)floor(peak_lim - pk_now); /* MOD-20/27: peak may sit at -4 (barrage -2)..0 dBFS */
        step = 0;
        if (agc_auto) {
            want = have_nf ? (long)floor(rx_target_eff - nf + 0.5) : 0;
            if (want >= 2 && room >= (jam ? 3 : 2) && agc_up_lock > 0) {   /* MOD-31 */
                sprintf(agc_why, "held %ld dB under the target: anti-hunt lock, %d s left (%lu hunts)",
                        want, (agc_up_lock + 9) / 10, (unsigned long)agc_hunts);
            } else if (want >= 2 && room >= (jam ? 3 : 2)) {        /* MOD-31: 1 dB more margin with a jammer */
                step = want;
                if (jam) {
                    /* LVDS / ant board: with a jammer the gain rises by at
                     * most agc_jstep_db every agc_jstep_s (default 1 dB / 1 s,
                     * gnss_agcstep=dB,s), never closer than 1 dB to the ADC
                     * limit (1-s peak max-hold) and only with >= 3 dB of room.
                     * A 1-dB step is under the 1.5-dB trace step that restarts
                     * the covariance averaging, and a uniform gain change does
                     * not move the MVDR weights, so the null holds through it. */
                    long lim = agc_jstep_db;
                    /* MOD6: far under the ADC limit (after a deep cut) rise up
                     * to 3 dB a step: every step now restarts the covariance
                     * and skips the straddling window, so a bigger step costs
                     * the null nothing more, and the satellites get their
                     * C/N0 back in seconds instead of minutes */
                    if (room >= 7 && !agc_pulse && lim < 3) { lim = 3; }
                    if (agc_pulse) { lim = 1; }
                    if (step > room - 1) { step = room - 1; }
                    if (step > lim) { step = lim; }
                    if (agc_up_age < (agc_pulse ? 50 : 10 * agc_jstep_s)) { step = 0; }
                } else {
                    if (step > room) { step = room; }
                    if (step > 10) { step = 10; }               /* AOLC fast absorbs it */
                }
                if (step > 0) {
                    sprintf(agc_why, "rising to the target noise (+%ld dB now, %ld to go)", step, want);
                } else {
                    sprintf(agc_why, "settling with the jammer before the next rise");
                }
            } else if (want <= -4 && !jam) {
                step = want + 2;                      /* back to ~target */
                if (step < -6) { step = -6; }
                sprintf(agc_why, "lowering to the target noise (%ld dB)", step);
            } else if (want >= 2) {
                sprintf(agc_why, "held %ld dB under the target: %s peak %ld dBFS at the ADC limit",
                        want, jam ? "jammer" : "signal", (long)floor(pk_now + 0.5));
            } else {
                if (!jam && have_nf) { agc_clean = g; }
                sprintf(agc_why, "at the target noise (best C/N0)%s", jam ? ", jammer below the limit" : "");
            }
        } else if (g < agc_man && room >= 2) {
            step = agc_man - g;
            if (step > room) { step = room; }
            if (step > 6)    { step = 6; }
        }
        if (step != 0) {
            agc_apply(g, g + (int32_t)step, step > 0);
        }
    }

    /* one summary line once the gain has stopped moving for 1 s */
    if (agc_report_from >= 0 && ++agc_quiet_n >= 10 && agc_report_from == agc_cur) {
        agc_report_from = -1;               /* went down and back: nothing to say */
    } else if (agc_report_from >= 0 && agc_quiet_n >= 10) {
        bg_mark();
        printf("  [AGC] RX gain %ld -> %ld dB (%s)", (long)agc_report_from, (long)agc_cur,
               agc_auto ? "AUTO" : "MANUAL");
        if (have_nf) { printf(", noise "); pdb(nf); printf(" dBFS"); }
        printf(", peak "); pdb(pk); printf(" dBFS%s\n",
               jam ? ", jammer: gain at the ADC limit"
                   : (agc_auto && have_nf && nf > rx_target_eff - 3.5) ? ": best C/N0" : "");
        gnss_rep_event("RX GAIN     %ld -> %ld dB (%s%s)", (long)agc_report_from, (long)agc_cur,
                       agc_auto ? "AUTO" : "MANUAL", jam ? ", jammer" : "");
        agc_report_from = -1;
    }
}

/* MOD-20: deferred restart after a plan change from the vendor commands */
static int restart_pending = 0;            /* 10 ms ticks left, 0 = none */
void gnss_request_restart(void) { restart_pending = 100; }

/* MOD-26: gnss_beam=2 with SMI: broadside beam (+3 dB there) while there is
 * no jammer, power inversion (every direction) as soon as one appears. Only
 * a steering this code set is undone; gnss_steer= by hand is left alone. */
static int smi_beam_auto = 0;
/* MOD-26b: a beam steered (gnss_steer=) toward the jammer cannot null it: the
 * unity-gain constraint keeps that direction open (field: look 0-1 deg,
 * jammer at 5 deg -> sum|w|^2 17, noise cost 12 dB, residual above the
 * noise). While a jammer sits within 45 deg of RX2-RX1 phase of the look
 * direction, SMI runs power inversion; the steering comes back by itself
 * when the jammer is gone. A new gnss_steer= cancels the suspension. */
static int    steer_susp = 0;
static double steer_susp_phi = 0.0, steer_susp_theta = 999.0;
void gnss_steer_user_changed(void) { steer_susp = 0; smi_beam_auto = 0; }
static void smi_beam_follow(void)
{
    gnss_smi_state_t ss;
    int want;
    if (!gnss_smi_is_enabled()) { return; }
    gnss_smi_get_state(&ss);
    if (!steer_susp && ss.steer && !smi_beam_auto && gnss_rep_jammed() && ss.coh12 > 0.5) {
        double d = ss.ph21_deg - ss.steer_phi_deg;
        while (d > 180.0)   { d -= 360.0; }
        while (d < -180.0)  { d += 360.0; }
        if (d < 45.0 && d > -45.0) {
            steer_susp = 1;
            steer_susp_phi = ss.steer_phi_deg;
            steer_susp_theta = ss.steer_theta;
            gnss_smi_steer_off();
            gnss_rep_event("STEER       look direction %ld deg (RX2-RX1 phase) from the jammer: power inversion while jammed",
                           (long)floor(d + 0.5));
            ev("SMI steering suspended: the jammer is %ld deg (RX2-RX1 phase) from the look\n"
               "          direction, which a 2-antenna beam cannot both keep and null. Power\n"
               "          inversion until the jammer is gone (gnss_steer=999 cancels it).\n",
               (long)floor(d + 0.5));
        }
        return;
    }
    if (steer_susp) {
        if (!gnss_rep_jammed()) {
            steer_susp = 0;
            if (steer_susp_theta < 900.0) { (void)gnss_smi_steer_theta(steer_susp_theta); }
            else                          { gnss_smi_steer_phase(steer_susp_phi); }
            gnss_rep_event("STEER       jammer gone: steering restored");
        }
        return;
    }
    want = (gnss_mvdr_auto_get_mode() == GNSS_BEAM_AUTO_BROAD && !gnss_rep_jammed()) ? 1 : 0;
    if (ss.steer && !smi_beam_auto) { return; }          /* the operator's own steering */
    if (want && !ss.steer) {
        if (gnss_smi_steer_theta(0.0) == 0) {
            smi_beam_auto = 1;
            gnss_rep_event("BEAM        SMI steered to broadside (gnss_beam=2, no jammer)");
        }
    } else if (!want && ss.steer && smi_beam_auto) {
        gnss_smi_steer_off();
        smi_beam_auto = 0;
        gnss_rep_event("BEAM        SMI back to power inversion (%s)",
                       gnss_rep_jammed() ? "jammer" : "gnss_beam off");
    }
}

int gnss_background_tick(void)
{
    static int n10 = 0, n100 = 0;
    int p;

    (void)gnss_spool_drain(64U);   /* MOD10: periodic KPI / report, <= one UART FIFO per 10 ms */
    if (restart_pending > 0 && --restart_pending == 0) {
        bg_mark();
        (void)gnss_autostart_run();
    }

    if (guard_en && monitor_armed && ad9361_phy && mvdr_present() && mvdr_on_tx()) {
        agc_fast();
        if (++n10 >= 10) {                 /* 100 ms */
            n10 = 0;
            agc_slow();
            gnss_mvdr_auto_poll();         /* MOD-26: gnss_beam= (no-op when off) */
            smi_beam_follow();
        }
    }
    if (++n100 >= 100) {                   /* events once a second */
        n100 = 0;
        gnss_temp_poll();                  /* MOD12: temperatures, LO lock */
        gnss_monitor_poll(0);
    }
    p = bg_printed;
    bg_printed = 0;
    return p;
}
