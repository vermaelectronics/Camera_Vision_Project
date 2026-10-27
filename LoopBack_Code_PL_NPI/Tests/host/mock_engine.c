/* ============================================================================
 *  mock_engine.c  --  host regression test of the PL-NPI v2.4 firmware engine
 *  (gnss_npi_aj.c + gnss_npi_det.c, compiled unchanged) against a simulated
 *  radio, DMA and a behavioural model of pl_npi_0 v2.1 (power detector, modes,
 *  converged single weight, v2.4 status bits and notch registers).
 *
 *  Not a model of the RF hardware: it checks the firmware's logic (detection,
 *  JAMMER STATUS hysteresis, report consistency, CW scan and notch
 *  programming, gating) with synthetic captures.
 *
 *  usage: mock_engine <scenario 1..8>; prints MOCK: PASS/FAIL (scenario n)
 * ========================================================================== */
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <complex.h>
#include "gnss_npi_aj.h"
#include "gnss_npi_det.h"
#include "gnss_npi_agc.h"
#include "gnss_passthrough.h"
#include "axi_dmac.h"
#include "ad9361_api.h"
#include "xtime_l.h"

uint32_t mock_buf[2 * NPI_NSAMP];
static struct ad9361_rf_phy phy_obj;
struct ad9361_rf_phy *ad9361_phy = &phy_obj;
static struct axi_dmac dmac_obj;
struct axi_dmac *rx_dmac = &dmac_obj;

static XTime now_t;
void XTime_GetTime(XTime *t) { now_t += 333; *t = now_t; }
static double t_s(void) { return (double)now_t / COUNTS_PER_SECOND; }

/* ---- console: to stdout, or into a buffer while a report is captured ---- */
static int quiet_out;
static char cap_buf[65536]; static int cap_on, cap_len;
static unsigned keys; static int typing;
unsigned int console_key_count(void) { return keys; }
int console_typing(void) { return typing; }
void console_print(char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    char out[4096]; int o = 0;
    for (const char *p = fmt; *p && o < 4000; p++) {
        if (*p != '%') { out[o++] = *p; continue; }
        p++;
        if (*p == 'd') o += sprintf(out + o, "%ld", va_arg(ap, long));
        else if (*p == 's') o += sprintf(out + o, "%s", va_arg(ap, char *));
        else if (*p == 'x') o += sprintf(out + o, "%lx", va_arg(ap, long));
        else out[o++] = *p;
    }
    out[o] = 0; va_end(ap);
    if (cap_on) { int n = (int)strlen(out); if (cap_len + n < (int)sizeof cap_buf - 1) { memcpy(cap_buf + cap_len, out, n); cap_len += n; cap_buf[cap_len] = 0; } return; }
    if (!quiet_out) fputs(out, stdout);
}
void Xil_DCacheInvalidateRange(uintptr_t a, unsigned int l) { (void)a; (void)l; }

/* ---- RX gain (AGC not modelled: fixed gain) + gamma ---- */
static int g_mode = GNSS_RXG_AD9361; static int32_t g_gain = 40; static uint32_t g_chg;
static int g_gated = 1; static double g_gamma_n1; static int g_gamma_calls;
int gnss_rxg_mode(void) { return g_mode; }
void gnss_rxg_set_mode(int m) { g_mode = m; }
int32_t gnss_rxg_gain(void) { return g_gain; }
int32_t gnss_rxg_ref(void) { return g_gain; }
uint32_t gnss_rxg_changes(void) { return g_chg; }
void gnss_rxg_stop_tx_hold(void) {}
void gnss_rxg_poll(void) {}
int gnss_rxg_peak(void) { return 900; }
uint32_t gnss_rxg_att_base(void) { return 70000; }
int gnss_rxg_comp_db(void) { return 0; }
void gnss_npi_set_gated(int on) { g_gated = on; }
uint32_t gnss_npi_k_eighths(void) { return g_gated ? 1 : 256; }
void gnss_npi_gamma_noise(double n1, int jammed) { (void)jammed; g_gamma_n1 = n1; g_gamma_calls++; }
const char *gnss_rxg_mode_txt(void) { return "AD9361 slow-attack AGC per channel (mock)"; }
int gnss_txdma_is_running(void) { return 0; }
int32_t ad9361_do_calib_run(struct ad9361_rf_phy *p, uint32_t c, int32_t a) { (void)p; (void)c; (void)a; return 0; }

/* ---- registers + core model ---- */
static uint32_t ctrl, gamma_r = 1;
static uint32_t cfg_a = 0xD, nrm_a = 0x2000, thr_a = 0xFFFFFFFF, hold_a = 0;
static uint32_t ncoef_a[6];
static int commits, snaps, ncoef_writes, mock_tx = 1, core_det, core_cnt; static double det_until;
static double complex h21 = 0;
static double last_pow;
static int core_active(void) { uint32_t m = cfg_a & 3; return m == 2 || m == 3 || (m == 0 && core_det); }
static uint32_t core_st(void) {
    return (core_det ? 1u : 0u) | (core_active() ? 2u : 0u) | (core_active() && (cfg_a & 3) != 3 ? 0x10u : 0u) |
           (2u << 5) | ((cfg_a & 8u) ? 0x100u : 0u) | ((uint32_t)(core_cnt & 0xFFFF) << 16);
}
uint32_t gnss_pt_read(uint32_t off) {
    if (off == GNSS_PT_REG_VERSION) return 0x00020004U;
    if (off == GNSS_PT_REG_STATUS) return ((ctrl & GNSS_PT_CTRL_NPI_EN) ? GNSS_PT_ST_NPI_EN_SYNCED : 0) |
        (mock_tx ? (GNSS_PT_ST_DAC_EN_I0 | GNSS_PT_ST_DAC_EN_Q0 | GNSS_PT_ST_PASS_EN_SYNCED) : 0);
    if (off == GNSS_PT_REG_CONTROL) return ctrl;
    if (off == GNSS_PT_REG_NPI_GAMMA) return gamma_r;
    if (off == GNSS_PT_REG_NPI_CFG_ACTIVE) return cfg_a;
    if (off == GNSS_PT_REG_NPI_SNAP) return core_st();
    return 0;
}
void gnss_pt_write(uint32_t off, uint32_t v) { if (off == GNSS_PT_REG_CONTROL) ctrl = v; }
void gnss_pt_set_npi(int en) { if (en) ctrl |= GNSS_PT_CTRL_NPI_EN; else ctrl &= ~GNSS_PT_CTRL_NPI_EN; }
void gnss_pt_set_npi_freeze(int f) { if (f) ctrl |= GNSS_PT_CTRL_NPI_FREEZE; else ctrl &= ~GNSS_PT_CTRL_NPI_FREEZE; }
void gnss_pt_npi_write(uint32_t cfg, int32_t g2, uint32_t thr, uint32_t hold) {
    cfg_a = cfg; nrm_a = (uint32_t)(uint16_t)g2; thr_a = thr; hold_a = hold; commits++;
}
void gnss_pt_npi_write_ncoef(const uint32_t w[6]) { memcpy(ncoef_a, w, sizeof ncoef_a); ncoef_writes++; }
void gnss_pt_npi_snapshot(gnss_pt_npi_snap_t *s) {
    double g2 = (double)(int16_t)nrm_a / 8192.0;
    snaps++;
    memset(s->w, 0, sizeof s->w);
    if (core_active() && cabs(h21) > 0) {
        double complex w = -1.0 / (h21 * g2);
        s->w[0] = (int32_t)lround(creal(w) * 1048576.0);
        s->w[1] = (int32_t)lround(cimag(w) * 1048576.0);
    }
    s->st = core_st(); s->pow = (uint32_t)last_pow;
}

/* ---- radio ---- */
static uint64_t m_rlo = 1575420000ULL, m_tlo = 1575420000ULL; static uint32_t m_rfs = 30720000, m_tfs = 30720000, m_bw = 18000000;
static uint8_t m_rfir, m_tfir;
int32_t ad9361_get_rx_sampling_freq(struct ad9361_rf_phy *p, uint32_t *f) { (void)p; *f = m_rfs; return 0; }
int32_t ad9361_get_tx_sampling_freq(struct ad9361_rf_phy *p, uint32_t *f) { (void)p; *f = m_tfs; return 0; }
int32_t ad9361_get_rx_lo_freq(struct ad9361_rf_phy *p, uint64_t *f) { (void)p; *f = m_rlo; return 0; }
int32_t ad9361_get_tx_lo_freq(struct ad9361_rf_phy *p, uint64_t *f) { (void)p; *f = m_tlo; return 0; }
int32_t ad9361_get_tx_attenuation(struct ad9361_rf_phy *p, uint8_t ch, uint32_t *a) { (void)p; (void)ch; *a = 70000; return 0; }
int32_t ad9361_get_rx_rf_bandwidth(struct ad9361_rf_phy *p, uint32_t *b) { (void)p; *b = m_bw; return 0; }
int32_t ad9361_get_tx_rf_bandwidth(struct ad9361_rf_phy *p, uint32_t *b) { (void)p; *b = m_bw > 40000000U ? 40000000U : m_bw; return 0; }
int32_t ad9361_get_rx_rf_gain(struct ad9361_rf_phy *p, uint8_t ch, int32_t *g) { (void)p; (void)ch; *g = g_gain; return 0; }
int32_t ad9361_set_rx_lo_freq(struct ad9361_rf_phy *p, uint64_t f) { (void)p; m_rlo = f; return 0; }
int32_t ad9361_set_tx_lo_freq(struct ad9361_rf_phy *p, uint64_t f) { (void)p; m_tlo = f; return 0; }
int32_t ad9361_set_rx_sampling_freq(struct ad9361_rf_phy *p, uint32_t f) { (void)p; m_rfs = m_tfs = f; return 0; }
int32_t ad9361_set_tx_sampling_freq(struct ad9361_rf_phy *p, uint32_t f) { (void)p; m_rfs = m_tfs = f; return 0; }
int32_t ad9361_set_rx_rf_bandwidth(struct ad9361_rf_phy *p, uint32_t b) { (void)p; m_bw = b; return 0; }
int32_t ad9361_set_tx_rf_bandwidth(struct ad9361_rf_phy *p, uint32_t b) { (void)p; (void)b; return 0; }
int32_t ad9361_set_rx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t e) { (void)p; m_rfir = e; return 0; }
int32_t ad9361_set_tx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t e) { (void)p; m_tfir = e; return 0; }
int32_t ad9361_get_rx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t *e) { (void)p; *e = m_rfir; return 0; }
int32_t ad9361_get_tx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t *e) { (void)p; *e = m_tfir; return 0; }

/* ---- signal + DMA ----
 * noise 20 LSB per component on both channels (independent), times nmul;
 * wideband jammer (Gaussian, common, RX2 through h) at jam_db over the
 * noise; CW at cw_jn dB over the TOTAL noise at cw_f (fraction of fs);
 * DC dc1/dc2. */
static unsigned long long rs = 88172645463325252ULL;
static double ur(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return ((rs >> 11) + 0.5) / 9007199254740992.0; }
static double gr(void) { return sqrt(-2 * log(ur())) * cos(2 * M_PI * ur()); }
static double jam_db = 30, nmul = 1.0, cw_jn = -99, cw_f = 0.1034, dc1 = 0, dc2 = 0;
static int jam_on = 0; static long nsamp_abs; static int dma_dead;
static void fill(void) {
    const double nrms = 20.0 * nmul, ja = 20.0 * sqrt(pow(10, jam_db / 10));
    const double cwa = (cw_jn > -90) ? sqrt(2 * 400.0 * pow(10, cw_jn / 10)) : 0.0;
    const double complex h = 0.62 * cexp(I * 0.7);
    double pw = 0;
    h21 = jam_on ? h : 0;
    for (int t = 0; t < NPI_NSAMP; t++, nsamp_abs++) {
        double complex j = jam_on ? ja * (gr() + I * gr()) : 0;
        double complex c = cwa * cexp(I * 2 * M_PI * cw_f * (double)nsamp_abs);
        double complex x1 = j + c + nrms * (gr() + I * gr()) + dc1;
        double complex x2 = h * (j + c) + nrms * (gr() + I * gr()) + dc2;
        int a = (int)lround(creal(x1)), b = (int)lround(cimag(x1)), d = (int)lround(creal(x2)), e = (int)lround(cimag(x2));
        #define CL(v) ((v) > 2047 ? 2047 : (v) < -2048 ? -2048 : (v))
        a = CL(a); b = CL(b); d = CL(d); e = CL(e);
        pw += (double)a * a + (double)b * b + (double)d * d + (double)e * e;
        mock_buf[2 * t] = (uint32_t)(uint16_t)(int16_t)a | ((uint32_t)(uint16_t)(int16_t)b << 16);
        mock_buf[2 * t + 1] = (uint32_t)(uint16_t)(int16_t)d | ((uint32_t)(uint16_t)(int16_t)e << 16);
    }
    nsamp_abs += 3 * NPI_NSAMP;                     /* gap between captures */
    last_pow = pw / NPI_NSAMP;
    if (last_pow > (double)thr_a) { if (!core_det) core_cnt++; core_det = 1; det_until = t_s() + (double)hold_a / m_rfs; }
    else if (core_det && last_pow <= thr_a / 2.0 && t_s() > det_until) core_det = 0;
}
static uint32_t pend, ctrlr = 1;
int32_t axi_dmac_read(struct axi_dmac *d, uint32_t r, uint32_t *v) { (void)d;
    if (r == AXI_DMAC_REG_IRQ_PENDING) *v = pend; else if (r == AXI_DMAC_REG_CTRL) *v = ctrlr; else *v = 0; return 0; }
int32_t axi_dmac_write(struct axi_dmac *d, uint32_t r, uint32_t v) { (void)d;
    if (r == AXI_DMAC_REG_IRQ_PENDING) pend &= ~v;
    else if (r == AXI_DMAC_REG_CTRL) ctrlr = v;
    else if (r == AXI_DMAC_REG_TRANSFER_SUBMIT) { if (dma_dead) return 0; fill(); now_t += 333 * 67; pend = AXI_DMAC_IRQ_SOT | AXI_DMAC_IRQ_EOT; }
    return 0; }

/* ---- harness ---- */
static int st_prev = -1, n_on, n_off, n_unknown;
static void run(double secs) {
    double tend = t_s() + secs;
    while (t_s() < tend) {
        gnss_idle();
        now_t += 333 * 600;
        int st = gnss_aj_status();
        if (st != st_prev) { if (st == 2) n_on++; else if (st == 1 && st_prev == 2) n_off++; else if (st == 0) n_unknown++; st_prev = st; }
    }
}
static void reset_counts(void) { n_on = n_off = n_unknown = 0; }
static void report_to(char *dst, size_t n) { cap_on = 1; cap_len = 0; cap_buf[0] = 0; gnss_aj_report(); cap_on = 0; snprintf(dst, n, "%s", cap_buf); }
static int has(const char *txt, const char *pat) { return strstr(txt, pat) != NULL; }
static double coef_angle(uint32_t w) { return atan2((double)(int16_t)(w >> 16), (double)(int16_t)(w & 0xFFFF)); }
static int check(int cond, const char *what) { printf("  [%s] %s\n", cond ? "ok" : "FAIL", what); return cond; }

static void boot(void) {
    gnss_band_set(GNSS_BAND_DEFAULT);
    gnss_aj_init();
    gnss_aj_enable(1);
    gnss_aj_note_tx(1);
    run(3.0);                                       /* learn the floor */
}

int main(int argc, char **argv) {
    int scen = argc > 1 ? atoi(argv[1]) : 1, ok = 1;
    static char r1[65536], r2[65536];
    quiet_out = getenv("MOCK_VERBOSE") ? 0 : 1;
    boot();
    switch (scen) {
    case 1: {   /* status hysteresis: wideband jammer 30 dB, on 3 s / off 3 s, twice */
        printf("scenario 1: wideband jammer 30 dB on/off twice\n");
        reset_counts();
        for (int k = 0; k < 2; k++) { jam_on = 1; run(3.0); jam_on = 0; run(3.0); }
        ok &= check(n_on == 2 && n_off == 2, "exactly 2 ON and 2 OFF transitions");
        ok &= check(gnss_aj_status() == 1, "status OFF at the end");
        break; }
    case 2: {   /* jammer level wandering around the coherence threshold */
        printf("scenario 2: coherent jammer wandering -8..+2 dB around the threshold, 20 s\n");
        reset_counts();
        double tend = t_s() + 20.0;
        jam_on = 1;
        while (t_s() < tend) { jam_db = -8 + 10 * ur(); run(0.05); }
        printf("  transitions: %d ON, %d OFF in 20 s\n", n_on, n_off);
        ok &= check(n_on + n_off <= 2, "no alternating ON/OFF (<= 1 ON and 1 OFF)");
        jam_on = 0; jam_db = 30; run(3.0);
        break; }
    case 3: {   /* report: one coherent snapshot, no side effects */
        printf("scenario 3: gnss_report twice back to back, jammer ON\n");
        jam_on = 1; run(2.0);
        int c0 = commits, s0 = snaps, w0 = ncoef_writes; uint32_t cfg0 = cfg_a; XTime t0 = now_t;
        report_to(r1, sizeof r1);
        report_to(r2, sizeof r2);
        /* the "ms old" figure may differ by the time the first print took */
        char *a = strstr(r1, "ms old"), *b = strstr(r2, "ms old");
        int same = a && b && strcmp(a, b) == 0;
        ok &= check(same, "two reports identical (after the snapshot age)");
        ok &= check(commits == c0 && snaps == s0 && ncoef_writes == w0 && cfg_a == cfg0, "no register write, no snapshot latch, no commit");
        ok &= check(has(r1, "JAMMER STATUS: ON\n") && !has(r1, "JAMMER STATUS: OFF"), "exactly one status line: ON");
        ok &= check(now_t - t0 < 333333ULL * 50, "report did not run the engine (no capture time passed)");
        if (getenv("MOCK_SHOW")) fputs(r1, stdout);
        jam_on = 0; run(3.0);
        report_to(r1, sizeof r1);
        ok &= check(has(r1, "JAMMER STATUS: OFF\n"), "status OFF after the jammer");
        break; }
    case 4: {   /* CW: detection, notch frequency, release */
        const double fs = 50e6, f = (1575.42e6 + 2e3 - 1570.25e6) / fs;
        printf("scenario 4: CW at L1 + 2 kHz, J/N -12 dB (J/S ~20 dB), 4 s, then off\n");
        reset_counts();
        cw_f = f; cw_jn = -12;
        double t_on = t_s(); int on_seen = 0; double t_det = -1;
        while (t_s() - t_on < 4.0) { run(0.01); if (!on_seen && gnss_aj_status() == 2) { on_seen = 1; t_det = t_s() - t_on; } }
        report_to(r1, sizeof r1);
        double ang = coef_angle(ncoef_a[0]), ferr = (ang / (2 * M_PI) - f) * fs;
        printf("  detected after %.0f ms; notch q angle -> %.0f Hz from the CW\n", t_det * 1000, ferr);
        ok &= check(on_seen && t_det < 0.3, "JAMMER STATUS ON within 300 ms");
        ok &= check(fabs(ferr) < 3000.0, "notch 0 within 3 kHz of the CW");
        ok &= check((cfg_a & GNSS_PT_NPI_CFG_NOTCH(0)) != 0, "notch 0 enabled in the core");
        ok &= check((cfg_a & 3) == 0, "PL-NPI filter not forced on by a CW below the noise (mode AUTO)");
        ok &= check(has(r1, "CONFIRMED") && has(r1, "[0] 1575.42"), "report shows the confirmed CW and the notch");
        cw_jn = -99; run(3.0);
        ok &= check(gnss_aj_status() == 1 && (cfg_a & GNSS_PT_NPI_CFG_NOTCH(0)) == 0, "status OFF and notch off after the CW");
        ok &= check(n_on == 1 && n_off == 1, "one ON and one OFF");
        break; }
    case 5: {   /* weak CW: below the detection edge -> OFF, no flicker */
        printf("scenario 5: CW at J/N -27 dB (J/S ~5 dB) for 10 s\n");
        reset_counts();
        cw_f = 0.1034; cw_jn = -27; run(10.0);
        ok &= check(n_on == 0 && gnss_aj_status() == 1, "status stays OFF");
        cw_jn = -99;
        break; }
    case 6: {   /* incoherent noise bursts (LNA noise) -> not a jammer */
        printf("scenario 6: +10 dB noise bursts independent in RX1/RX2, 15 s\n");
        reset_counts();
        double tend = t_s() + 15.0;
        while (t_s() < tend) { nmul = (fmod(t_s(), 3.0) < 1.4) ? 3.16 : 1.0; run(0.05); }
        nmul = 1.0;
        ok &= check(n_on == 0, "status never ON");
        break; }
    case 7: {   /* no capture -> UNKNOWN, and back */
        printf("scenario 7: DMA stops for 2 s, then resumes\n");
        dma_dead = 1; run(2.0);
        report_to(r1, sizeof r1);
        ok &= check(gnss_aj_status() == 0 && has(r1, "JAMMER STATUS: UNKNOWN"), "status UNKNOWN while nothing is measured");
        dma_dead = 0; run(2.0);
        ok &= check(gnss_aj_status() == 1, "status OFF again once captures resume");
        break; }
    case 8: {   /* calibrated CW J/S levels: 10, 15, 30, 40 dB (S/N in 50 MHz = -32 dB) */
        printf("scenario 8: CW at J/S 10 / 15 / 30 / 40 dB (J/N over 50 MHz = J/S - 32 dB), 4 s each\n");
        const double jss[4] = {10, 15, 30, 40};
        for (int i = 0; i < 4; i++) {
            reset_counts();
            cw_f = (1575.42e6 + 2e3 - 1570.25e6) / 50e6; cw_jn = jss[i] - 32;
            run(4.0);
            int st = gnss_aj_status(); int nt = (cfg_a & GNSS_PT_NPI_CFG_NOTCH(0)) ? 1 : 0;
            printf("  J/S %2.0f dB: status %s, notch %s, %d ON / %d OFF transitions\n", jss[i],
                   st == 2 ? "ON" : st == 1 ? "OFF" : "UNKNOWN", nt ? "on" : "off", n_on, n_off);
            ok &= check(n_on <= 1 && n_off == 0, "steady (no ON/OFF alternation)");
            if (jss[i] >= 15) ok &= check(st == 2 && nt, "ON with the notch");
            cw_jn = -99; run(3.0);
        }
        break; }
    default: printf("unknown scenario\n"); ok = 0;
    }
    printf(ok ? "MOCK: PASS (scenario %d)\n" : "MOCK: FAIL (scenario %d)\n", scen);
    return ok ? 0 : 1;
}
