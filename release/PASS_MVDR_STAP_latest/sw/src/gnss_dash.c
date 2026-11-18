/***************************************************************************//**
 *   @file   gnss_dash.c
 *   @brief  GNSS-CRPA MOD-25: colour console check page (gnss_check?) and
 *           the test-command list (gnss_cmds?). See gnss_dash.h.
 *
 * Everything here only READS state (no register writes), so it is safe to
 * run at any time, also with a jammer on. Numbers are printed with integer
 * maths (no %f in this printf).
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "gnss_dash.h"
#include "gnss_passthrough.h"
#include "gnss_smi.h"
#include "gnss_autostart.h"
#include "gnss_report.h"
#include "ad9361_api.h"

extern struct ad9361_rf_phy *ad9361_phy;

/* ------------------------------------------------------------- colours */
static int gnss_color_on = 1;
void gnss_color_set(int on) { gnss_color_on = on ? 1 : 0; }
int  gnss_color_get(void)   { return gnss_color_on; }
const char *gnss_c(int c)
{
    static const char *code[] = { "\x1b[0m", "\x1b[1;31m", "\x1b[1;32m", "\x1b[1;33m",
                                  "\x1b[1;36m", "\x1b[1m", "\x1b[2m", "\x1b[1;35m" };
    if (!gnss_color_on || c < 0 || c > GC_MAG) { return ""; }
    return code[c];
}

/* ------------------------------------------------------------- helpers */
#define ST_OK   0
#define ST_WARN 1
#define ST_FAIL 2
#define ST_INFO 3
static int n_ok, n_warn, n_fail;

/* v with one decimal into buf (sign kept), integer maths */
static const char *f1(char *buf, double v)
{
    long m = (long)(v * 10.0 + ((v >= 0.0) ? 0.5 : -0.5));
    if (m < 0) { sprintf(buf, "-%ld.%ld", (-m) / 10L, (-m) % 10L); }
    else       { sprintf(buf, "%ld.%ld", m / 10L, m % 10L); }
    return buf;
}
static const char *f3(char *buf, double v)
{
    long m = (long)(v * 1000.0 + ((v >= 0.0) ? 0.5 : -0.5));
    long a = (m < 0) ? -m : m;
    sprintf(buf, "%s%ld.%03ld", (m < 0) ? "-" : "", a / 1000L, a % 1000L);
    return buf;
}

static void section(const char *num, const char *title)
{
    printf("\n %s%s %s %s%s\n", gnss_c(GC_CYN), num, title, "", gnss_c(GC_RST));
    printf(" %s-----------------------------------------------------------------------------%s\n",
           gnss_c(GC_DIM), gnss_c(GC_RST));
}

/* one row:  [ OK ]  label ........ value   (status first, fixed columns,
 * fits an 80-column terminal) */
#include <stdarg.h>
#include "gnss_quiet.h"     /* LVDS console: engine output captured, see gnss_quiet.h */
#define GNSS_QUIET_THIS_FILE
#undef  printf
#define printf        gnss_qprintf
#undef  console_print
#define console_print gnss_qprintf
#undef  vprintf
#define vprintf       gnss_qvprintf   /* res()/more() tails were printed */
#undef  putchar
#define putchar       gnss_qputchar
static void row(int st, const char *l, const char *fmt, ...)
{
    char dots[32];
    int  n = 22 - (int)strlen(l);
    va_list ap;
    switch (st) {
    case ST_OK:   n_ok++;   printf("  %s[ OK ]%s ", gnss_c(GC_GRN), gnss_c(GC_RST)); break;
    case ST_WARN: n_warn++; printf("  %s[WARN]%s ", gnss_c(GC_YEL), gnss_c(GC_RST)); break;
    case ST_FAIL: n_fail++; printf("  %s[FAIL]%s ", gnss_c(GC_RED), gnss_c(GC_RST)); break;
    default:                printf("  %s[INFO]%s ", gnss_c(GC_DIM), gnss_c(GC_RST)); break;
    }
    if (n < 2) { n = 2; }
    memset(dots, '.', (size_t)n);
    dots[n] = '\0';
    printf("%s %s%s%s ", l, gnss_c(GC_DIM), dots, gnss_c(GC_RST));
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
}
/* continuation line under a row */
static void note(const char *fmt, ...)
{
    va_list ap;
    printf("                                 %s", gnss_c(GC_DIM));
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("%s\n", gnss_c(GC_RST));
}

/* 0..72 dB bar, 3 dB per character */
static void bar(double db, int lower_bound)
{
    int i, n = (int)(db / 3.0 + 0.5);
    if (n < 0) { n = 0; }
    if (n > 24) { n = 24; }
    printf("%s|", gnss_c(GC_GRN));
    for (i = 0; i < n; i++) { putchar('#'); }
    printf("%s%s", lower_bound ? ">" : "", gnss_c(GC_DIM));
    for (i = n + (lower_bound ? 1 : 0); i < 24; i++) { putchar('.'); }
    printf("|%s", gnss_c(GC_RST));
}

/* ----------------------------------------------------------- gnss_check? */
#define G  gnss_c(GC_GRN)
#define Y  gnss_c(GC_YEL)
#define RD gnss_c(GC_RED)
#define M  gnss_c(GC_MAG)
#define B  gnss_c(GC_BLD)
#define X  gnss_c(GC_RST)
static void rule(void)
{
    printf("%s==============================================================================%s\n", B, X);
}

void gnss_dash_check(void)
{
    gnss_pt_mvdr_state_t s;
    gnss_pt_jam_state_t  j;
    gnss_pt_supp_info_t  si;
    gnss_rep_aj_t        ra;
    gnss_smi_state_t     ss;
    char b1[24], b2[24], b3[24];
    int  prof, eff, smi_on, have_smi, st;
    int32_t g = 0;
    uint32_t back;

    n_ok = n_warn = n_fail = 0;
    gnss_pt_mvdr_get_state(&s);
    gnss_pt_jam_get_state(&j);
    have_smi = gnss_smi_available();
    memset(&ss, 0, sizeof(ss));
    if (have_smi) { gnss_smi_get_state(&ss); }
    prof = gnss_pt_get_profile();
    eff  = gnss_pt_profile_effective();
    smi_on = have_smi && ss.hw_active;

    printf("\n");
    rule();
    printf("%s  GNSS ANTI-JAM CHECK%s      MVDR core v%lu.%lu   firmware MOD13\n", B, X,
           (unsigned long)(s.version >> 16), (unsigned long)(s.version & 0xFFFFU));
    rule();
    if (s.id != GNSS_PT_MVDR_EXPECTED_ID) {
        printf("  %sMVDR core not in this bitstream.%s\n", RD, X);
        return;
    }

    /* ---------------- 1 null core ---------------- */
    section("1", "NULL CORE  (what drives TX1)");
    row((eff == 3) ? ST_OK : (prof == 3 ? ST_FAIL : ST_WARN), "core",
        "%s%s%s  (gnss_null=%d)", (eff == 3) ? G : Y,
        (eff == 3) ? "SMI-MVDR STAP" : (eff == 2) ? "NLMS STAP" :
        (eff == 1) ? "plain power inversion" : "MVDR, one weight", X, prof);
    if (have_smi) {
        row(ST_OK, "SMI in bitstream", "%sYES  (2 x %d taps + 2 image, MVDR 1.14%s)%s", G, ss.L,
            ss.L > 8 ? "b" : "", X);
    } else {
        row(ST_FAIL, "SMI in bitstream", "%sNO  (needs MVDR 1.14)%s", RD, X);
    }
    if (have_smi) {
        row(smi_on ? ST_OK : (prof == 3 ? ST_FAIL : ST_INFO), "SMI drives output", "%s%s%s",
            smi_on ? G : RD, smi_on ? "YES" : ss.enabled ? "enabled, not active yet" : "NO", X);
        st = !smi_on ? ST_INFO : (ss.updates == 0U) ? ST_FAIL : ss.fails ? ST_WARN : ST_OK;
        row(st, "solver", "%lu solved, %s%lu failed%s, %lu skipped",
            (unsigned long)ss.updates, ss.fails ? RD : "", (unsigned long)ss.fails, X,
            (unsigned long)ss.skipped);
        row(ss.novr_new ? ST_WARN : ST_OK, "sample overruns", "%lu since start%s", (unsigned long)ss.novr_new,
            ss.novr_new ? "  (needs >= 4 clk/sample)" : "");
        note("%lu in total, incl. the radio set-up at boot", (unsigned long)ss.novr);
    }

    /* ---------------- 2 SMI features ---------------- */
    if (have_smi) {
        section("2", "SMI FEATURES");
        {
            int w1 = 0, w2 = 0, k0 = ss.ref - ss.La / 2;
            if (k0 < 0) { k0 = 0; }
            if (k0 + ss.La > ss.L) { k0 = ss.L - ss.La; }
            row(ss.La < ss.L ? ST_INFO : ST_OK, "space-time taps",
                "%s2 antennas x %d taps IN USE%s (of %d; +2 image = %d solved)", ss.La < ss.L ? Y : G,
                ss.La, X, ss.L, ss.Na);
            note("taps %d..%d of each antenna, reference RX1 tap %d (layout v%d); taps=N changes it",
                 k0, k0 + ss.La - 1, ss.ref, ss.layout);
            if (gnss_smi_weight_taps(&w1, &w2) == 0) {
                note("weights in the FPGA now: %d taps on RX1, %d on RX2 non-zero", w1, w2);
            }
        }
        if (ss.updates == 0U) {
            row(ST_INFO, "image taps", "%sON%s (2 taps), no weights yet", G, X);
        } else {
            row(ST_OK, "image taps", "%sON%s   |w|^2 %s", G, X, f3(b1, ss.img_gain));
            if (ss.img_gain > 1e-3) { note("-> %sAD9361 IQ image present: being cancelled%s", M, gnss_c(GC_DIM)); }
            else                    { note("-> no IQ image seen (nothing to cancel)"); }
        }
        if (ss.dc_off) {
            row(ST_WARN, "DC removal", "%sOFF  (gnss_smi=%lu,3,1 = on)%s", Y, (unsigned long)ss.kexp, X);
        } else {
            row(ST_OK, "DC removal", "%sON   (FPGA, SMI input)%s", G, X);
        }
        row(ST_OK, "diagonal loading", "%sON%s   %s x lambda_min  (+%s dB)", G, X,
            f1(b1, ss.alpha), f1(b2, 10.0 * log10(ss.alpha > 0.0 ? ss.alpha : 1.0)));
        note("on all %d taps: spatial + temporal + image", ss.N);
        row(ST_INFO, "temporal taper", "%sNOT used%s (by design: costs depth)", Y, X);
        row(ST_INFO, "window", "2^%lu = %lu snapshots", (unsigned long)ss.kexp,
            (unsigned long)(1UL << ss.kexp));
        if (ss.steer) {
            row(ST_INFO, "constraint", "%sSTEERED%s  RX2-RX1 %s deg", M, X, f1(b1, ss.steer_phi_deg));
        } else {
            row(ST_INFO, "constraint", "power inversion (all sats pass)");
        }
        if (ss.updates) {
            row(ss.shift ? ST_WARN : ST_OK, "weights", "sum|w|^2 %s  noise cost %s dB",
                f3(b1, ss.wnorm), f1(b2, 10.0 * log10(ss.wnorm > 1e-9 ? ss.wnorm : 1e-9)));
            note("|w_ref|^2 %s   scale 2^-%d", f3(b3, ss.ref_gain), ss.shift);
            row(ST_INFO, "noise per tap", "%s dBFS  (lambda_min)", f1(b1, ss.lam_dbfs));
        }
    }

    /* ---------------- 3 jammer + suppression ---------------- */
    section("3", "JAMMER AND SUPPRESSION  (thermal-floor figures)");
    gnss_rep_get(&ra);
    if (smi_on && ra.valid) {
        /* MOD-26: from the SMI window itself (gnss_report? has every figure) */
        if (!ra.jammed) {
            row(ST_OK, "jammer", "%snone above the noise%s  (J/N %s dB)", G, X, f1(b1, ra.jnr_db));
        } else {
            row(ST_INFO, "jammer", "%sPRESENT%s  J/N %s dB at the ADC", RD, X, f1(b1, ra.jnr_db));
            if (ra.have_clean) { note("%s dB vs the antenna noise (gain cut included)", f1(b2, ra.jnr_ant_db)); }
            if (j.coherent && j.present) { note("angle %s deg from broadside", f1(b2, (double)j.theta_cdeg / 100.0)); }
            row(ST_INFO, "SUPPRESSION", "%s%s%s dB%s  jammer in / jammer out", B, ra.supp_lo ? ">" : "",
                f1(b1, ra.supp_db), X);
            printf("                                 ");
            bar(ra.supp_db, ra.supp_lo);
            printf("  0..72 dB\n");
            if (ra.res_lo) {
                row(ST_OK, "residual", "%s>10 dB under the noise: null complete%s", G, X);
            } else if (ra.res_db <= 0.0) {
                row(ST_OK, "residual", "%s%s dB vs noise%s", G, f1(b1, ra.res_db), X);
            } else {
                row(ra.res_db > 3.0 ? ST_FAIL : ST_WARN, "residual", "%s+%s dB ABOVE the noise%s",
                    RD, f1(b1, ra.res_db), X);
                note("satellites lose C/N0: gnss_report?");
            }
            if (ra.t_null_ms >= 0.0) { note("nulled %s ms after it appeared", f1(b2, ra.t_null_ms)); }
        }
        row(ST_INFO, "noise floor", "%s dBFS  (%s)", f1(b1, ra.nf_dbfs),
            (ra.nf_src == 1) ? "SMI-measured" : (ra.nf_src == 2) ? "learned" : "SMI estimate");
        if (ra.have_clean && ra.rise_db >= 1.0) {
            row(ST_WARN, "C/N0 cost of gain cut", "%s~%s dB%s  (floor above the antenna noise)", Y, f1(b1, ra.rise_db), X);
        }
    } else if (gnss_pt_supp_info(&si) != 0) {
        row(ST_INFO, "figures", "not available yet");
    } else if (!si.have_jam) {
        row(ST_OK, "jammer", "%snone above the noise%s", G, X);
        row(ST_INFO, "noise floor", "%s dBFS  (%s)", f1(b1, si.noise_dbfs),
            (si.nf_src == 1) ? "SMI-measured" : (si.nf_src == 2) ? "learned" : "covariance");
    } else {
        row(ST_INFO, "jammer", "%sPRESENT%s  %s dBFS at the ADC", RD, X, f1(b1, si.jam_dbfs));
        if (j.coherent && j.present) { note("angle %s deg from broadside", f1(b2, (double)j.theta_cdeg / 100.0)); }
        row(ST_INFO, "JNR", "%s dB", f1(b1, si.jnr_db));
        row(si.nf_src ? ST_OK : ST_WARN, "noise floor", "%s dBFS (%s)", f1(b1, si.noise_dbfs),
            (si.nf_src == 1) ? "thermal, SMI-measured" :
            (si.nf_src == 2) ? "thermal: learned" : "covariance: may hide jammer");
        if (si.nf_src) {
            if (si.jfloor_dbc > -90.0) {
                row(ST_INFO, "jammer-derived floor", "%s dBc", f1(b1, si.jfloor_dbc));
                note("cov. floor %s dBFS (IQ image/decorrelation)", f1(b2, si.cov_floor_dbfs));
            } else {
                row(ST_INFO, "jammer-derived floor", "none");
            }
        }
        row(ST_INFO, "ACTUAL cancellation", "%s%s%s dB%s ", B, si.actual_lo ? ">" : "", f1(b1, si.actual_db), X);
        printf("                                 ");
        bar(si.actual_db, si.actual_lo);
        printf("  0..72 dB\n");
        if (si.actual_lo) {
            row(ST_OK, "residual", "%s>10 dB under the noise: null complete%s", G, X);
        } else if (si.res_db <= 0.0) {
            row(ST_OK, "residual", "%s%s dB vs noise%s", G, f1(b1, si.res_db), X);
        } else {
            row(si.res_db > 3.0 ? ST_FAIL : ST_WARN, "residual", "%s+%s dB ABOVE the noise%s",
                RD, f1(b1, si.res_db), X);
            note("satellites lose C/N0: gnss_supp?, gnss_smi?");
        }
        row(ST_INFO, "print suppression", "%s dB  (display; capped by JNR)", f1(b1, si.print_db));
    }

    /* ---------------- 4 RX / ADC ---------------- */
    section("4", "RECEIVER / ADC");
    if (ad9361_phy) { ad9361_get_rx_rf_gain(ad9361_phy, 0, &g); }
    row(ST_INFO, "RX gain", "%ld dB  (%s)", (long)g, gnss_agc_is_auto() ? "AUTO" : "MANUAL");
    row((s.clip1 || s.clip2) ? ST_WARN : ST_OK, "ADC clipping", "%s%lu / %lu%s  (RX1 / RX2, last window)",
        (s.clip1 || s.clip2) ? RD : G, (unsigned long)s.clip1, (unsigned long)s.clip2, X);
    {   /* MOD-26b: the 2 x 2 covariance behind the jammer estimator */
        double c1 = 0.0, c2 = 0.0;
        int cs = gnss_pt_cov_check(&s, &c1, &c2);
        if (cs == 0) {
            row(ST_FAIL, "2x2 covariance", "%sINVALID%s  RX1 %s / RX2 %s dBFS", RD, X, f1(b1, c1), f1(b2, c2));
            note("R22 corrupt: estimator blind, SMI unaffected; MVDR 1.15 heals it");
        } else if (cs == 1) {
            row(ST_OK, "2x2 covariance", "valid  RX1 %s / RX2 %s dBFS", f1(b1, c1), f1(b2, c2));
        }
    }

    /* ---------------- 5 output ---------------- */
    section("5", "OUTPUT TO TX1");
    back = gnss_pt_aolc_backoff_db();
    if (s.have_aolc && (s.out_gain_reg & GNSS_PT_OG_AOLC_EN)) {
        row(ST_OK, "AOLC", "%sON%s  target -%lu, measured %s dBFS", G, X,
            (unsigned long)((s.out_gain_reg >> 16) & 0xFFU),
            f1(b1, 3.0103 * ((double)s.out_lvl_q4 / 16.0 - 23.0)));
    } else {
        row(ST_WARN, "AOLC", "%sOFF%s  (manual output gain)", Y, X);
    }
    if (back) {
        row(ST_WARN, "clip back-off", "%s%lu dB%s  (output was clipping)", Y, (unsigned long)back, X);
        note("1 dB back per 30 s without clipping");
    } else {
        row(ST_OK, "clip back-off", "%s0 dB%s  (no output clipping)", G, X);
    }
    row(ST_INFO, "DAC clipped samples", "%lu since the counter clear", (unsigned long)s.n_osat);

    /* ---------------- verdict ---------------- */
    printf("\n");
    rule();
    printf("  VERDICT: ");
    if (n_fail)      { printf("%s%d FAIL%s", RD, n_fail, X); }
    else if (n_warn) { printf("%sOK, %d WARN%s", Y, n_warn, X); }
    else             { printf("%sALL OK%s", G, X); }
    printf("   (%d ok / %d warn / %d fail)\n", n_ok, n_warn, n_fail);
    printf("  details: gnss_report?  gnss_smi?  gnss_health?   commands: gnss_cmds?\n");
    rule();
}

/* ------------------------------------------------------------ gnss_cmds? */
static void cmd(const char *c, const char *what)
{
    printf("   %s%-22s%s %s\n", gnss_c(GC_GRN), c, gnss_c(GC_RST), what);
}

void gnss_dash_cmds(void)
{
    printf("\n");
    rule();
    printf("%s  ANTI-JAM TEST COMMANDS%s   (in the order you use them)\n", gnss_c(GC_BLD), gnss_c(GC_RST));
    rule();
    section("A", "QUICK CHECK  (no jammer, then with jammer)");
    cmd("gnss_check?",          "this one-page check: every feature + verdict");
    cmd("gnss_report?",         "FULL report: radio, signals, gain, jammer, suppression, events");
    cmd("(Enter)",              "one-line live STATUS (core, jam, RX gain, clip, supp)");
    cmd("gnss_health?",         "full PASS / WARN / FAIL check of the whole chain");
    section("B", "NULL CORE");
    cmd("gnss_null?",           "core in use");
    cmd("gnss_null=3",          "SMI-MVDR STAP (default, best null)");
    cmd("gnss_null=0",          "one weight per antenna (A/B reference)");
    cmd("gnss_mvdr=0 / =1",     "raw RX1 to TX1 (reference) / back to the null");
    printf("   %s(gnss_null=2 NLMS STAP is not built in 1.14: it falls back to 0)%s\n",
           gnss_c(GC_DIM), gnss_c(GC_RST));
    section("C", "SMI DETAILS");
    cmd("gnss_smi?",            "solver, image taps, DC removal, loading, weights");
    cmd("gnss_smi=8,3,1",       "window 2^8, loading 3, DC on (default 16 taps; 8 taps: 9)");
    cmd("gnss_smi=10,3",        "longer window: steadier/deeper, slower to react");
    cmd("gnss_smi=8,1",         "lighter loading: deeper barrage null, more noise cost");
    cmd("gnss_steer=az,el,b",   "steer to a satellite (b = bearing of array +90 end)");
    cmd("gnss_steer=999",       "back to power inversion (all satellites)");
    cmd("gnss_beam=2 / =0",     "auto: broadside beam while no jammer, PI when jammed / off");
    cmd("gnss_mvsteer=theta",   "single-weight MVDR steering (gnss_null=0 only)");
    section("D", "JAMMER AND SUPPRESSION");
    cmd("gnss_jam?",            "jammer present, angle, power, JNR");
    cmd("gnss_supp?",           "J/N, SUPPRESSION, residual (SMI) + single-weight view");
    cmd("gnss_report?",         "everything in one block: copy it into the log");
    section("E", "RECEIVER GAIN");
    cmd("gnss_agc?",            "RX gain mode, gain, noise, ADC peak");
    cmd("gnss_agc=1",           "AUTO RX gain (default)");
    cmd("gnss_rxgain=45",       "fixed-gain test: 40 / 45 / 49, same jammer");
    section("F", "OUTPUT TO TX1");
    cmd("gnss_aolc?",           "output level, target, clip back-off");
    cmd("gnss_aolc=24",         "AOLC target -24 dBFS (default)");
    cmd("tx1_attenuation=34000", "TX1 level into the GNSS receiver (mdB, bigger = quieter)");
    cmd("gnss_band? / =3",      "radio plan + signals in band (3 = default, all systems)");
    cmd("gnss_floor?",          "RX noise vs gain: is the AD9361 / board floor too high?");
    cmd("gnss_kpi?",            "every figure on one key=value line (test scripts)");
    cmd("gnss_spec=0",          "board noise spectrum at 0 dB gain: flat, edges or spurs?");
    cmd("gnss_bist?",           "AD9361 -> FPGA data-link test (digital tone)");
    cmd("gnss_clk?",            "AD9361 clock chain: ADC clock, decimation, RX FIR");
    cmd("gnss_floor=0",         "gain on the whole-band floor (default; 1 = L1/E1 floor)");
    cmd("gnss_smiavg=2",        "SMI covariance averaging 0..4 (steadier satellites)");
    cmd("gnss_osr=8",           "ADC oversampling test 0/4/8/12 (restarts MVDR)");
    printf("   %s(TX2 is unused and always held at 89000 mdB)%s\n", gnss_c(GC_DIM), gnss_c(GC_RST));
    section("G", "DISPLAY");
    cmd("gnss_color=0 / =1",    "plain text / colours");
    section("H", "ON THE PC (logs + verdicts automatically)");
    cmd("gnss_autotest.py",     "bring-up + all cores (about 4 min)");
    cmd("  -s jam",             "jammer sweep: set each level, press Enter");
    cmd("  -s sweep",           "SMI window / loading sweep");
    cmd("  -s soak --minutes 60", "long run");
    printf("\n");
}
