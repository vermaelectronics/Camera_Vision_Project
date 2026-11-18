/***************************************************************************//**
 *   @file   gnss_report.c
 *   @brief  GNSS-CRPA MOD-26: anti-jam report (gnss_report?), event log and
 *           the SMI-based jammer / suppression figures. See gnss_report.h.
 *
 * Plain text (no colour codes) so the block can be copied from any terminal.
 * Numbers are printed with integer maths (no %f in this printf).
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "gnss_report.h"
#include "gnss_temp.h"
#include "gnss_smi.h"
#include "gnss_passthrough.h"
#include "gnss_autostart.h"
#include "command.h"
#include "gnss_config.h"
#include "ad9361_api.h"
#include "gnss_time.h"     /* was xtime_l.h (not in the 2023.2 SDT BSP) */
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

extern struct ad9361_rf_phy *ad9361_phy;

/* ---------------------------------------------------------------- time */
static gnss_time_t t0;
static int   t0_set = 0;
void gnss_rep_init(void)
{
    t0 = gnss_time_now();
    t0_set = 1;
}
double gnss_rep_uptime_s(void)
{
    gnss_time_t t;
    if (!t0_set) { gnss_rep_init(); }
    t = gnss_time_now();
    return (double)(t - t0) / (double)GNSS_COUNTS_PER_SECOND;
}

/* ------------------------------------------------------------- helpers */
static const char *f1(char *buf, double v)          /* "-12.3" */
{
    long m = (long)(v * 10.0 + ((v >= 0.0) ? 0.5 : -0.5));
    if (m < 0) { sprintf(buf, "-%ld.%ld", (-m) / 10L, (-m) % 10L); }
    else       { sprintf(buf, "%ld.%ld", m / 10L, m % 10L); }
    return buf;
}
static const char *s1(char *buf, double v)          /* "+12.3" / "-12.3" */
{
    if (v >= 0.0) { buf[0] = '+'; f1(buf + 1, v); }
    else          { f1(buf, v); }
    return buf;
}
static const char *f2(char *buf, double v)          /* "12.34" */
{
    long m = (long)(v * 100.0 + ((v >= 0.0) ? 0.5 : -0.5));
    long a = (m < 0) ? -m : m;
    sprintf(buf, "%s%ld.%02ld", (m < 0) ? "-" : "", a / 100L, a % 100L);
    return buf;
}
static const char *f3(char *buf, double v)          /* "12.345" */
{
    long m = (long)(v * 1000.0 + ((v >= 0.0) ? 0.5 : -0.5));
    long a = (m < 0) ? -m : m;
    sprintf(buf, "%s%ld.%03ld", (m < 0) ? "-" : "", a / 1000L, a % 1000L);
    return buf;
}
static double db10(double x) { return 10.0 * log10(x > 1e-30 ? x : 1e-30); }
static double lin(double d)  { return pow(10.0, d / 10.0); }

/* --------------------------------------------------------------- events */
#define EV_N   24
#define EV_LEN 104
static char     ev_txt[EV_N][EV_LEN];
static double   ev_t[EV_N];
static uint32_t ev_total = 0;

void gnss_rep_event(const char *fmt, ...)
{
    va_list ap;
    char   *p = ev_txt[ev_total % EV_N];
    size_t  n;
    va_start(ap, fmt);
    vsnprintf(p, EV_LEN, fmt, ap);
    va_end(ap);
    n = strlen(p);
    while (n > 0 && (p[n - 1] == '\n' || p[n - 1] == '\r' || p[n - 1] == ' ')) { p[--n] = 0; }
    ev_t[ev_total % EV_N] = gnss_rep_uptime_s();
    ev_total++;
}

/* LVDS console: read the event log (gnss_ant.c prints the new ones) */
uint32_t gnss_rep_event_total(void) { return ev_total; }
int gnss_rep_event_get(uint32_t i, double *t, const char **txt)
{
    if (i >= ev_total || ev_total - i > EV_N) { return -1; }
    *t = ev_t[i % EV_N];  *txt = ev_txt[i % EV_N];
    return 0;
}

/* --------------------------------------------------- jammer state machine */
#define JAM_ON_DB    6.0       /* J/N to declare a jammer            */
#define JAM_OFF_DB   3.0       /* ... and to clear it, held OFF_HOLD  */
#define JAM_OFF_HOLD 0.5       /* s                                  */

static gnss_rep_aj_t aj;
static double t_jam_on = 0.0, t_below = -1.0;
static int    nulled = 0, res_warned = 0;
static double t_res_high = -1.0;

static const char *bw_text(char *buf, const gnss_smi_state_t *ss)
{
    char b[24];
    if (ss->jbw_class == 0)      { sprintf(buf, "NARROWBAND / CW (< %s MHz)", f1(b, ss->jbw_hz / 1e6 + 0.05)); }
    else if (ss->jbw_class == 1) { sprintf(buf, "band-limited, ~%s MHz wide", f1(b, ss->jbw_hz / 1e6)); }
    else                         { sprintf(buf, "WIDEBAND / barrage, ~%s MHz wide", f1(b, ss->jbw_hz / 1e6)); }
    return buf;
}

/* MOD-30: diagnostics (gnss_spec, gnss_bist, gnss_floor, the floor measurement
 * of the automatic start) change the RX gain under the solver for a moment.
 * The CLEAR / JAMMED state machine is held while they run and for 0.5 s after,
 * so a gain step is not reported as a jammer (seen in MOD-29: a 0.6 s "CW at
 * 1575.42 MHz" after gnss_spec=0 in the 4 MHz band). */
static int    rep_hold_n     = 0;
static double rep_hold_until = -1.0;
void gnss_rep_hold(int on)
{
    if (on) { rep_hold_n++; return; }
    if (rep_hold_n > 0) { rep_hold_n--; }
    rep_hold_until = gnss_rep_uptime_s() + 0.5;
}
static int rep_held(void)
{
    return rep_hold_n > 0 || gnss_rep_uptime_s() < rep_hold_until;
}

void gnss_rep_on_solve(void)
{
    gnss_smi_state_t ss;
    double nf, nfl, pin, jin, nout, pout, jout, t, nf_clean;
    int    src;
    int32_t g = gnss_agc_gain_now(), gc = 0;
    char   b1[24], b2[24], b3[24], b4[48];
    uint64_t lo = 0;

    gnss_smi_get_state(&ss);
    if (!ss.updates) { return; }
    if (rep_held()) { return; }                        /* MOD-30 */
    aj.n_solves++;
    if (gnss_pt_thermal_now(ss.lam_dbfs, ss.hw_active && ss.count >= 64U, g, &nf, &src) != 0) {
        return;
    }
    t = gnss_rep_uptime_s();
    nfl  = lin(nf);
    pin  = lin(ss.pin_dbfs_ref);
    jin  = pin - nfl;
    nout = nfl * ss.wnorm_u;
    pout = lin(ss.pout_dbfs);
    jout = pout - nout;
    aj.valid    = 1;
    aj.nf_dbfs  = nf;
    aj.nf_src   = src;
    aj.pin_dbfs = ss.pin_dbfs_ref;
    aj.gain_db  = g;
    aj.noise_cost_db = db10(ss.wnorm_u);
    aj.jnr_db   = (jin > 0.0) ? db10(jin / nfl) : -99.0;
    aj.jin_dbfs = (jin > 0.0) ? db10(jin) : -200.0;
    if (jout > 0.1 * nout) { aj.res_db = db10(jout / nout); aj.res_lo = 0; }
    else                   { aj.res_db = -10.0;              aj.res_lo = 1; jout = 0.1 * nout; }
    if (jin > 0.0) { aj.supp_db = db10(jin / jout); aj.supp_lo = aj.res_lo; }
    else           { aj.supp_db = 0.0;              aj.supp_lo = 0; }
    aj.have_clean = (gnss_pt_thermal_clean(&nf_clean, &gc) == 0 && g >= 0) ? 1 : 0;
    if (aj.have_clean) {
        double nant = nf_clean + (double)(g - gc);         /* antenna noise at this gain */
        double ftot = gnss_rx_floor_dbfs(), fsig = gnss_rx_floor_sig_dbfs();
        aj.clean_gain_db = gc;
        aj.jnr_ant_db = (jin > 0.0) ? aj.jin_dbfs - nant : -99.0;
        aj.rise_db    = nf - nant;                       /* floor rise = C/N0 cost */
        if (aj.rise_db < 0.0) { aj.rise_db = 0.0; }
        /* MOD-30: the AD9361 floor is not white on this board (ADC noise rises
         * toward the band edges), so the cost is worked out with the floor
         * where the protected satellites are:
         *   A  = antenna noise (clean total minus the whole-band floor), scaled by the gain cut
         *   E  = measured excess over A + whole-band floor (jammer-made noise, taken as white)
         *   cost = 10log((A + Fsig + E) / A) - 10log((Aclean + Fsig) / Aclean) */
        if (ftot > -150.0 && fsig > -150.0 && lin(nf_clean) > 1.05 * lin(ftot)) {
            double ac = lin(nf_clean) - lin(ftot);
            double an = ac * lin((double)(g - gc));
            double ex = nfl - an - lin(ftot);
            double c;
            if (ex < 0.0) { ex = 0.0; }
            c = db10((an + lin(fsig) + ex) / an) - db10((ac + lin(fsig)) / ac);
            aj.rise_db    = (c > 0.0) ? c : 0.0;
            aj.jnr_ant_db = (jin > 0.0) ? aj.jin_dbfs - db10(an) : -99.0;
        }
    }

    /* state machine */
    /* MOD11: a jammer must make RX1/RX2 coherent (gnss_smi.c coh_gate):
     * incoherent in-band excess (antenna noise, a hot LNA) is noise */
    if (!aj.jammed) {
        if (aj.jnr_db > JAM_ON_DB && !ss.incoh && ss.jc_db > 3.0) {
            aj.jammed = 1;  aj.since_s = t;  t_jam_on = t;  t_below = -1.0;
            aj.jnr_peak_db = aj.jnr_db;  aj.t_null_ms = -1.0;  nulled = 0;
            res_warned = 0;  t_res_high = -1.0;
            gnss_get_plan(&lo, NULL, NULL, NULL, NULL);
            gnss_rep_event("JAMMER ON   J/N %s dB, %s, RF %s MHz (LO %s MHz)",
                           s1(b1, aj.jnr_db), bw_text(b4, &ss),
                           f2(b2, ((double)lo + ss.jf_hz) / 1e6), s1(b3, ss.jf_hz / 1e6));
        }
    } else {
        if (aj.jnr_db > aj.jnr_peak_db) { aj.jnr_peak_db = aj.jnr_db; }
        if (!nulled && aj.res_db <= 0.0 && aj.jnr_db > JAM_ON_DB) {
            /* MOD-27: only on a window that still holds the jammer (right
             * after a gain cut the J/N can read low for one window) */
            nulled = 1;
            aj.t_null_ms = (t - t_jam_on) * 1000.0;
            gnss_rep_event("NULLED      after %s ms (stap? shows the suppression)",
                           f1(b1, aj.t_null_ms));
        }
        if (nulled && aj.res_db > 3.0) {
            if (t_res_high < 0.0) { t_res_high = t; }
            if (!res_warned && t - t_res_high > 1.0) {
                res_warned = 1;
                gnss_rep_event("RESIDUAL    jammer left ABOVE the noise for > 1 s (selftest? says why)");
            }
        } else {
            t_res_high = -1.0;
        }
        if (aj.jnr_db < JAM_OFF_DB || ss.incoh) {
            if (t_below < 0.0) { t_below = t; }
            if (t - t_below >= JAM_OFF_HOLD) {
                aj.jammed = 0;  aj.since_s = t;
                gnss_rep_event("JAMMER OFF  after %s s, peak J/N %s dB",
                               f1(b1, t_below - t_jam_on), s1(b2, aj.jnr_peak_db));
            }
        } else {
            t_below = -1.0;
        }
    }
}

void gnss_rep_get(gnss_rep_aj_t *a) { *a = aj; }
int  gnss_rep_jammed(void)          { return aj.valid && aj.jammed; }

/* --------------------------------------------------------------- signals */
typedef struct { const char *name; double mhz, hw; } sig_t;
static const sig_t sigs[] = {
    { "GPS L1 C/A, QZSS, SBAS", 1575.420, 1.023 },
    { "Galileo E1, BeiDou B1C", 1575.420, 2.046 },
    { "BeiDou B1I",             1561.098, 2.046 },
    { "GLONASS G1 k=-7",        1598.0625, 0.511 },
    { "GLONASS G1 k=0",         1602.000, 0.511 },
    { "GLONASS G1 k=+6",        1605.375, 0.511 },
};
#define NSIG ((int)(sizeof(sigs) / sizeof(sigs[0])))

/* MOD-30: the table for gnss_autostart.c (floor where the satellites are) */
int gnss_rep_signal(int i, const char **name, double *mhz, double *hw)
{
    if (i < 0 || i >= NSIG) { return 0; }
    if (name) { *name = sigs[i].name; }
    if (mhz)  { *mhz  = sigs[i].mhz; }
    if (hw)   { *hw   = sigs[i].hw; }
    return 1;
}

void gnss_rep_print_signals(int with_response)
{
    uint64_t lo = 0;
    uint32_t bw = 0, fs = 0;
    double   half, off, r, jf = 0.0, jw = 0.0;
    char     b1[24], b2[24], b3[24];
    int      i, jam = 0;
    gnss_smi_state_t ss;

    gnss_get_plan(&lo, &bw, &fs, NULL, NULL);
    half = 0.5 * (double)((bw < fs) ? bw : fs) / 1e6;
    memset(&ss, 0, sizeof(ss));
    if (with_response && gnss_smi_available()) {
        gnss_smi_get_state(&ss);
        if (!ss.updates) { with_response = 0; }
        jam = gnss_rep_jammed();
        jf  = ss.jf_hz / 1e6;
        jw  = 0.5 * ss.jbw_hz / 1e6;
    } else {
        with_response = 0;
    }
    printf("              signal                   RF MHz    from LO   received?%s\n",
           with_response ? "  SMI gain" : "");
    for (i = 0; i < NSIG; i++) {
        off = sigs[i].mhz - (double)lo / 1e6;
        printf("              %-23s %9s  %8s   %-9s", sigs[i].name, f3(b1, sigs[i].mhz),
               s1(b2, off), (fabs(off) + sigs[i].hw <= half) ? "OK" :
                            (fabs(off) - sigs[i].hw < half)  ? "EDGE" : "OUTSIDE");
        if (with_response && fabs(off) - sigs[i].hw < half) {
            r = gnss_smi_response_db(off * 1e6, (double)fs);
            printf("  %s dB", s1(b3, r));
            if (jam && fabs(off - jf) < sigs[i].hw + jw + 1.0) { printf("  <- jammer on it"); }
        }
        printf("\n");
    }
}

static void hdr(const char *name) { printf("  %-12s", name); }
static const char *pad = "              ";

/* JAMMER + SUPPRESSION sections (also gnss_supp? with SMI running) */
static void print_jam_supp(const gnss_rep_aj_t *pa, const gnss_smi_state_t *pss,
                           const gnss_pt_jam_state_t *pj, uint64_t lo, double t)
{
    gnss_rep_aj_t a = *pa;
    gnss_smi_state_t ss = *pss;
    gnss_pt_jam_state_t j = *pj;
    char b1[24], b2[24], b3[24], b4[48];

    hdr("JAMMER");
    if (!a.valid) {
        printf("no figures yet (SMI not solving, or no floor)\n");
    } else if (!a.jammed) {
        printf("CLEAR for %s s (J/N %s dB: nothing above the noise)\n", f1(b1, t - a.since_s), s1(b2, a.jnr_db));
    } else {
        printf("JAMMED for %s s (since t = %s s)\n", f1(b1, t - a.since_s), f1(b2, a.since_s));
        printf("%sJ/N %s dB at the ADC (peak %s), jammer %s dBFS\n", pad, s1(b1, a.jnr_db),
               s1(b2, a.jnr_peak_db), f1(b3, a.jin_dbfs));
        if (a.have_clean) {
            printf("%sJ/N %s dB vs the antenna noise (RX gain cut %ld dB included)\n", pad,
                   s1(b1, a.jnr_ant_db), (long)(a.clean_gain_db - a.gain_db));
        }
        printf("%s%s\n", pad, bw_text(b4, &ss));
        printf("%scentre RF %s MHz (%s MHz from the LO)\n", pad,
               f2(b1, ((double)lo + ss.jf_hz) / 1e6), s1(b2, ss.jf_hz / 1e6));
        printf("%sRX1/RX2 coherence %s, RX2-RX1 phase %s deg", pad, f3(b1, ss.coh12), s1(b2, ss.ph21_deg));
        if (j.available && j.valid && j.present) { printf(", angle %s deg", s1(b3, (double)j.theta_cdeg / 100.0)); }
        printf("\n");
    }

    hdr("SUPPRESSION");
    if (!a.valid) {
        printf("n/a\n");
    } else if (!a.jammed) {
        printf("no jammer to suppress (filter passes the band: noise cost %s dB)\n", s1(b1, a.noise_cost_db));
    } else {
        printf("%s%s dB  = jammer in / jammer out (null depth)\n", a.supp_lo ? ">" : "", f1(b1, a.supp_db));
        if (a.res_lo) {
            printf("%sresidual > 10 dB UNDER the output noise: jammer fully nulled\n", pad);
        } else {
            printf("%sresidual %s dB vs the output noise%s\n", pad, s1(b1, a.res_db),
                   (a.res_db > 0.0) ? "  <- ABOVE the noise: satellites lose C/N0" :
                   (a.res_db > -6.0) ? "  (close to the noise)" : "");
        }
        printf("%sJ/N %s dB in -> %s dB out", pad, s1(b1, a.jnr_db), a.res_lo ? "< -10" : s1(b2, a.res_db));
        if (a.t_null_ms >= 0.0) { printf(", time to null %s ms", f1(b3, a.t_null_ms)); }
        printf("\n");
        if (a.have_clean) {
            double out = (a.res_lo ? -10.0 : a.res_db) + a.rise_db + a.noise_cost_db;
            printf("%svs the antenna noise: jammer %s dB in, %s%s dB out\n", pad,
                   s1(b1, a.jnr_ant_db), a.res_lo ? "< " : "", s1(b2, out));
            printf("%s(the SMI-PI log's 'suppression' is the J/N in, when its residual reads 0)\n", pad);
        }
    }

}

void gnss_rep_print_supp(void)
{
    gnss_smi_state_t    ss;
    gnss_rep_aj_t       a;
    gnss_pt_jam_state_t j;
    uint64_t lo = 0;
    memset(&ss, 0, sizeof(ss));
    if (gnss_smi_available()) { gnss_smi_get_state(&ss); }
    gnss_rep_get(&a);
    gnss_pt_jam_get_state(&j);
    gnss_get_plan(&lo, NULL, NULL, NULL, NULL);
    print_jam_supp(&a, &ss, &j, lo, gnss_rep_uptime_s());
}

/* ----------------------------------------------------------------- report */

void gnss_rep_print(void)
{
    gnss_pt_mvdr_state_t s;
    gnss_smi_state_t     ss;
    gnss_rep_aj_t        a;
    gnss_pt_jam_state_t  j;
    uint64_t lo = 0;
    uint32_t bw = 0, fs = 0, a1 = 0, a2 = 0, op = 0, i, n0;
    int32_t  g = -1;
    int      band = -1, have_smi;
    const char *bname = "";
    char     b1[24], b2[24], b3[24];
    double   t = gnss_rep_uptime_s(), pk1, pk2;

    gnss_pt_mvdr_get_state(&s);
    gnss_pt_jam_get_state(&j);
    have_smi = gnss_smi_available();
    memset(&ss, 0, sizeof(ss));
    if (have_smi) { gnss_smi_get_state(&ss); }
    gnss_rep_get(&a);
    gnss_get_plan(&lo, &bw, &fs, NULL, NULL);
    gnss_band_info(&band, &bname);
    op = gnss_get_op_atten_mdb();
    if (ad9361_phy) {
        ad9361_get_tx_attenuation(ad9361_phy, 0, &a1);
        ad9361_get_tx_attenuation(ad9361_phy, 1, &a2);
        ad9361_get_rx_rf_gain(ad9361_phy, 0, &g);
    }
    if (g < 0) { g = gnss_agc_gain_now(); }

    printf("\n===================== GNSS ANTI-JAM REPORT (copy from here) =====================\n");
    hdr("SYSTEM");
    printf("firmware MOD13, MVDR core %lu.%lu, uptime %s s\n",
           (unsigned long)(s.version >> 16), (unsigned long)(s.version & 0xFFFFU), f1(b1, t));
    if (have_smi) {
        printf("%sSMI layout v%d: 2 antennas x %d taps IN USE (of %d) + 2 image taps = %d solved (INFO 0x%08lX)\n",
               pad, ss.layout, ss.La, ss.L, ss.Na, (unsigned long)ss.info);
    }
    hdr("TEMPERATURE");                             /* MOD12 */
    gnss_temp_print_line(pad);

    hdr("SUMMARY");
    if (!a.valid) {
        printf("no SMI figures yet\n");
    } else if (!a.jammed) {
        printf("CLEAR - no jammer; the filter passes the band (noise cost %s dB)\n", s1(b1, a.noise_cost_db));
    } else {
        printf("JAMMED - J/N %s dB, suppression %s%s dB, residual %s dB vs noise\n", s1(b1, a.jnr_db),
               a.supp_lo ? ">" : "", f1(b2, a.supp_db), a.res_lo ? "< -10" : s1(b3, a.res_db));
        if (a.have_clean && a.rise_db >= 1.0) {
            printf("%sRX gain cut %ld dB costs the satellites ~%s dB C/N0 (noise floor rise)\n", pad,
                   (long)(a.clean_gain_db - a.gain_db), f1(b1, a.rise_db));
        }
        if (a.have_clean && a.rise_db >= 3.0 && a.res_lo) {
            /* MOD-27: say what limits the satellites: here not the null */
            printf("%sLIMIT: receiver dynamic range, not the null. The jammer is %s dB over\n"
                   "%sthe antenna noise; this receiver holds ~%s dB before the AD9361 floor\n"
                   "%s(%s dBFS where the satellites are) costs C/N0. Less jammer at the antenna, or a lower floor, helps.\n",
                   pad, f1(b1, a.jnr_ant_db), pad, f1(b2, a.jnr_ant_db - a.rise_db), pad,
                   f1(b3, gnss_rx_floor_sig_dbfs()));
        } else if (!a.res_lo && a.res_db > 0.0) {
            printf("%sLIMIT: the null (residual above the noise): gnss_smi=10,3, check RX1/RX2 match\n", pad);
        }
    }

    hdr("RADIO");
    if (band >= 0) { printf("gnss_band=%d \"%s\"\n%s", band, bname, pad); }
    printf("LO %s MHz, %s MS/s, RF bandwidth %s MHz\n", f3(b1, (double)lo / 1e6),
           f2(b2, (double)fs / 1e6), f1(b3, (double)bw / 1e6));
    printf("%sTX1 %s, attenuation %s dB (operating %s dB)%s\n", pad,
           gnss_tx_is_on() ? "TRANSMITTING" : "SILENT", f2(b1, a1 / 1000.0), f2(b2, op / 1000.0),
           gnss_tx_is_on() ? "" : "  <- gnss_auto=1");
    printf("%sTX2 attenuation %s dB (held at %s dB: TX2 unused)\n", pad, f2(b1, a2 / 1000.0),
           f2(b2, GNSS_CFG_TX2_ATTEN_MDB / 1000.0));

    hdr("SIGNALS");
    printf("receiver must have each constellation switched ON\n");
    gnss_rep_print_signals(1);

    hdr("RX GAIN");
    printf("%ld dB both channels (%s)", (long)g, gnss_agc_is_auto() ? "AUTO" : "MANUAL");
    if (gnss_agc_clean_gain() >= 0) {
        printf(", clean reference %ld dB", (long)gnss_agc_clean_gain());
        if (g >= 0 && gnss_agc_clean_gain() > g) { printf(" (cut %ld dB)", (long)(gnss_agc_clean_gain() - g)); }
    }
    printf("\n%s%s\n", pad, gnss_agc_reason());
    pk1 = s.have_peak ? 20.0 * log10((double)(s.peak1 ? s.peak1 : 1) / 2048.0) : -99.0;
    pk2 = s.have_peak ? 20.0 * log10((double)(s.peak2 ? s.peak2 : 1) / 2048.0) : -99.0;
    printf("%sADC peak %s / %s dBFS, clipping %lu / %lu (RX1 / RX2, last window)\n", pad,
           f1(b1, pk1), f1(b2, pk2), (unsigned long)s.clip1, (unsigned long)s.clip2);

    hdr("NOISE");
    if (a.valid) {
        printf("floor %s dBFS per element now (%s)\n", f1(b1, a.nf_dbfs),
               (a.nf_src == 1) ? "SMI lambda_min + learned offset" :
               (a.nf_src == 2) ? "learned without jammer, scaled with gain" : "SMI estimate, not learned yet");
    } else {
        printf("not measured yet\n");
    }
    if (have_smi && ss.updates) {
        printf("%sRX1 %s / RX2 %s dBFS at the reference tap (SMI window)\n", pad,
               f1(b1, ss.pin_dbfs_ref), f1(b2, ss.p2_dbfs_ref));
    }
    {   /* MOD-26b: the single-weight 2 x 2 covariance must be plausible */
        double c1 = 0.0, c2 = 0.0;
        int cs = gnss_pt_cov_check(&s, &c1, &c2);
        if (cs == 0) {
            printf("%s2x2 covariance INVALID: RX1 %s, RX2 %s dBFS (R22 corrupt) -> jammer\n"
                   "%sestimator blind; SMI unaffected. MVDR 1.15 heals it (reprogram)\n",
                   pad, f1(b1, c1), f1(b2, c2), pad);
        } else if (cs == 1 && gnss_pt_cov_bad_seen(&c1, &c2)) {
            printf("%s2x2 covariance valid now, but was INVALID earlier (RX1 %s, RX2 %s dBFS)\n",
                   pad, f1(b1, c1), f1(b2, c2));
        }
    }
    printf("%starget %s dBFS", pad, f1(b1, gnss_rx_target_dbfs()));
    if (gnss_rx_floor_dbfs() > -150.0) { printf(", AD9361 internal floor %s dBFS", f1(b2, gnss_rx_floor_dbfs())); }
    printf("\n");
    if (gnss_floor_mode() != 0 && gnss_rx_floor_sig_dbfs() > -150.0) {     /* MOD-30 */
        printf("%sfloor where the satellites are (%s): %s dBFS white-equivalent;\n"
               "%sgain levelled on it (gnss_floor=%d; gnss_floor=0 = whole band as MOD-29)\n",
               pad, gnss_floor_mode_name(gnss_floor_mode()), f1(b1, gnss_rx_floor_sig_dbfs()),
               pad, gnss_floor_mode());
    }
    if (a.valid && a.have_clean && a.rise_db >= 0.5) {
        printf("%sfloor is %s dB above the antenna noise at this gain -> satellites lose\n"
               "%s~%s dB C/N0 (the jammer forced the gain down; weakest signals go first)\n",
               pad, f1(b1, a.rise_db), pad, f1(b2, a.rise_db));
    }

    hdr("ANTI-JAM");
    if (!have_smi) {
        printf("SMI not in this bitstream (MVDR >= 1.14 needed)\n");
    } else {
        printf("SMI-MVDR STAP %s, %s\n", ss.hw_active ? "ON, drives TX1" : ss.enabled ? "enabled, not active" : "OFF",
               ss.steer ? "STEERED" : "power inversion (every satellite passes)");
        printf("%swindow 2^%lu = %lu snapshots, loading %s x lambda_min, DC removal %s\n", pad,
               (unsigned long)ss.kexp, (unsigned long)(1UL << ss.kexp), f1(b1, ss.alpha), ss.dc_off ? "OFF" : "ON");
        printf("%s%lu windows solved, %lu failed, %lu skipped; sample overruns %lu since start\n", pad,
               (unsigned long)ss.updates, (unsigned long)ss.fails, (unsigned long)ss.skipped,
               (unsigned long)ss.novr_new);
    }

    print_jam_supp(&a, &ss, &j, lo, t);

    hdr("WEIGHTS");
    if (have_smi && ss.updates) {
        printf("sum|w|^2 %s (noise cost %s dB), |w_ref|^2 %s, scale 2^-%d\n", f3(b1, ss.wnorm_u),
               s1(b2, db10(ss.wnorm_u)), f3(b3, ss.ref_gain), ss.shift);
        printf("%slargest RX2 tap at delay %d (|w| %s), image taps |w|^2 %s\n", pad, ss.w2max_k,
               f3(b1, ss.w2max), f3(b2, ss.img_gain));
    } else {
        printf("none yet\n");
    }

    hdr("OUTPUT");
    if (s.have_aolc && (s.out_gain_reg & GNSS_PT_OG_AOLC_EN)) {
        printf("AOLC target -%lu dBFS, measured %s dBFS, clip back-off %lu dB\n",
               (unsigned long)((s.out_gain_reg >> 16) & 0xFFU),
               f1(b1, 3.0103 * ((double)s.out_lvl_q4 / 16.0 - 23.0)), (unsigned long)gnss_pt_aolc_backoff_db());
    } else {
        printf("AOLC OFF (manual output gain)\n");
    }
    printf("%sDAC clipped samples %lu since the counter clear\n", pad, (unsigned long)s.n_osat);

    hdr("EVENTS");
    n0 = (ev_total > EV_N) ? ev_total - EV_N : 0U;
    printf("%lu in total%s\n", (unsigned long)ev_total, ev_total ? ", the last ones:" : "");
    for (i = n0; i < ev_total; i++) {
        printf("%s%9s s  %s\n", pad, f1(b1, ev_t[i % EV_N]), ev_txt[i % EV_N]);
    }
    printf("=================================== END REPORT ==================================\n");
}

/* STATUS line field */
void gnss_rep_status_field(void)
{
    char b1[24], b2[24], b3[24];
    if (!aj.valid) { printf(" | jam ?"); return; }
    if (!aj.jammed) { printf(" | jam no"); return; }
    printf(" | JAM J/N %s supp %s%s res %s", s1(b1, aj.jnr_db), aj.supp_lo ? ">" : "",
           f1(b2, aj.supp_db), aj.res_lo ? "<-10" : s1(b3, aj.res_db));
}

/* MOD-28: gnss_kpi? -- every figure of the report on ONE line of key=value,
 * for scripts (antsdr_gnss_tester.py polls it). Unknown values read "na".
 * Bounds are separate flags (supp_lo=1: supp is a lower bound; res_lo=1:
 * residual > 10 dB under the noise, res then reads -10.0). */
static void kv1(const char *k, int ok, double v)        /* " k=-12.3" / " k=na" */
{
    char b[24];
    if (ok) { printf(" %s=%s", k, f1(b, v)); } else { printf(" %s=na", k); }
}
static void kv3(const char *k, int ok, double v)
{
    char b[24];
    if (ok) { printf(" %s=%s", k, f3(b, v)); } else { printf(" %s=na", k); }
}
void gnss_rep_print_kpi(void)
{
    gnss_pt_mvdr_state_t s;
    gnss_smi_state_t     ss;
    gnss_pt_jam_state_t  j;
    gnss_rep_aj_t        a;
    uint64_t lo = 0;
    uint32_t a1 = 0, a2 = 0;
    int32_t  g = -1, gc;
    int      have_smi, smi_ok, jam, cut_ok, aolc;
    const char *mode;

    gnss_pt_mvdr_get_state(&s);
    gnss_pt_jam_get_state(&j);
    have_smi = gnss_smi_available();
    memset(&ss, 0, sizeof(ss));
    if (have_smi) { gnss_smi_get_state(&ss); }
    gnss_rep_get(&a);
    gnss_get_plan(&lo, NULL, NULL, NULL, NULL);
    if (ad9361_phy) {
        ad9361_get_tx_attenuation(ad9361_phy, 0, &a1);
        ad9361_get_tx_attenuation(ad9361_phy, 1, &a2);
        ad9361_get_rx_rf_gain(ad9361_phy, 0, &g);
    }
    if (g < 0) { g = gnss_agc_gain_now(); }
    gc = gnss_agc_clean_gain();
    smi_ok = have_smi && ss.updates;
    jam = a.valid && a.jammed;
    cut_ok = (gc >= 0 && g >= 0);
    aolc = s.have_aolc && (s.out_gain_reg & GNSS_PT_OG_AOLC_EN);

    if (!gnss_tx_is_on())                                        { mode = "TXOFF"; }
    else if (!s.mvdr_sel || s.mode_live == (int)GNSS_PT_MVDR_MODE_BYPASS) { mode = "BYPASS"; }
    else if (s.have_smi && s.smi_active)                         { mode = "SMI"; }
    else if (s.have_stap && s.stap_active)                       { mode = "STAP"; }
    else                                                         { mode = "MVDR"; }

    printf("KPI");
    kv1("t", 1, gnss_rep_uptime_s());
    printf(" mode=%s state=%s", mode, !a.valid ? "NA" : (a.jammed ? "JAMMED" : "CLEAR"));
    kv1("jn", a.valid, a.jnr_db);
    kv1("jnpk", jam, a.jnr_peak_db);
    kv1("jn_ant", jam && a.have_clean, a.jnr_ant_db);
    kv1("supp", jam, a.supp_db);
    printf(" supp_lo=%d", jam ? a.supp_lo : 0);
    kv1("res", jam, a.res_lo ? -10.0 : a.res_db);
    printf(" res_lo=%d", jam ? a.res_lo : 0);
    kv1("cn0cost", a.valid && a.have_clean, (a.rise_db > 0.0) ? a.rise_db : 0.0);
    kv1("nf", a.valid, a.nf_dbfs);
    kv1("tgt", 1, gnss_rx_target_dbfs());
    kv1("adfloor", gnss_rx_floor_dbfs() > -150.0, gnss_rx_floor_dbfs());
    kv1("adfloor_sig", gnss_rx_floor_sig_dbfs() > -150.0, gnss_rx_floor_sig_dbfs());   /* MOD-30 */
    printf(" fmode=%d osr=%d", gnss_floor_mode(), gnss_adc_osr_now());
    printf(" avg=%d keff=%lu hunts=%lu", gnss_smi_get_avg(), (unsigned long)(have_smi ? ss.keff + 0.5 : 0.0),
           (unsigned long)gnss_agc_hunts());                                          /* MOD-31 */
    printf(" gain=%ld clean=%ld cut=%ld agc=%s", (long)g, (long)gc,
           cut_ok ? (long)((gc > g) ? gc - g : 0) : -1L, gnss_agc_is_auto() ? "AUTO" : "MANUAL");
    kv1("pk1", s.have_peak, 20.0 * log10((double)(s.peak1 ? s.peak1 : 1) / 2048.0));
    kv1("pk2", s.have_peak, 20.0 * log10((double)(s.peak2 ? s.peak2 : 1) / 2048.0));
    printf(" clip1=%lu clip2=%lu", (unsigned long)s.clip1, (unsigned long)s.clip2);
    kv1("p1", smi_ok, ss.pin_dbfs_ref);
    kv1("p2", smi_ok, ss.p2_dbfs_ref);
    kv3("wn", smi_ok, ss.wnorm_u);
    kv1("ncost", smi_ok, db10(ss.wnorm_u));
    kv3("img", smi_ok, ss.img_gain);
    if (jam && smi_ok) { printf(" jtype=%d", ss.jbw_class); } else { printf(" jtype=na"); }
    kv1("jbw", jam && smi_ok, ss.jbw_hz / 1e6);
    kv1("jrf", jam && smi_ok, ((double)lo + ss.jf_hz) / 1e6);
    kv3("coh", jam && smi_ok, ss.coh12);
    kv1("jcoh", smi_ok, ss.jc_db);                      /* MOD11: coherent J/N, dB */
    printf(" incoh=%d", smi_ok ? ss.incoh : 0);
    kv1("ph", jam && smi_ok, ss.ph21_deg);
    kv1("ang", jam && j.available && j.valid && j.present, (double)j.theta_cdeg / 100.0);
    kv1("tnull", jam && a.t_null_ms >= 0.0, a.t_null_ms);
    kv1("out", aolc, 3.0103 * ((double)s.out_lvl_q4 / 16.0 - 23.0));
    printf(" backoff=%lu dacclip=%lu", (unsigned long)gnss_pt_aolc_backoff_db(), (unsigned long)s.n_osat);
    printf(" solved=%lu failed=%lu skipped=%lu ovr=%lu", (unsigned long)ss.updates,
           (unsigned long)ss.fails, (unsigned long)ss.skipped, (unsigned long)ss.novr_new);
    printf(" steer=%d kexp=%lu", ss.steer, (unsigned long)ss.kexp);
    kv1("alpha", have_smi, ss.alpha);
    printf(" tx=%d", gnss_tx_is_on());
    kv1("tx1", ad9361_phy != NULL, a1 / 1000.0);
    kv1("tx2", ad9361_phy != NULL, a2 / 1000.0);
    {                                               /* MOD12 */
        gnss_temp_t tp;
        gnss_temp_get(&tp);
        kv1("temp", tp.have_rf, tp.rf_c);
        kv1("ztemp", tp.have_pl, tp.pl_c);
        printf(" lolock=%d", (tp.rx_lock == 1 && tp.tx_lock == 1) ? 1 : 0);
    }
    printf(" ev=%lu\n", (unsigned long)ev_total);
}
