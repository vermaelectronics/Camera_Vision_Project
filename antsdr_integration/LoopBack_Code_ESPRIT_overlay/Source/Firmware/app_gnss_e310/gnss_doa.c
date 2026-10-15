/******************************************************************************
 *  gnss_doa.c
 *  Driver, self-test and console reporting for the ESPRIT DOA monitor
 *  (gnss_passthrough v1.2, registers 0x100..0x194).
 *
 *  Every line printed by the self-test starts with "DOA_SELFTEST:", every
 *  monitor line with "DOA:", so a PuTTY log can be grepped.
 *
 *  No libm is used: sine/cosine/atan2 are integer CORDIC routines and all
 *  angles are printed from fixed point with integer arithmetic (the console
 *  printf is not guaranteed to support %f).
 *
 *  Original work for the ANTSDR E310 V1 GNSS CRPA project.
 *****************************************************************************/

#include "gnss_doa.h"
#include "no_os_delay.h"
#include <stdio.h>

#define DOA_PI_Q28        843314857L      /* pi in Q28                      */
#define DOA_HALF_PI_Q28   421657428L
#define DOA_DEG2RAD_Q28   4685082L        /* pi/180 in Q28                  */
#define DOA_K_HALF_LAMBDA 341782638U      /* 1/pi in Q30 (d = lambda/2)     */
#define DOA_CORDIC_GAIN   652032874L      /* 1/1.6467602581 in Q30          */
#define DOA_TIMEOUT_MS    3000U

static const int32_t doa_atan_q28[28] = {
    210828714, 124459457, 65760959, 33381290, 16755422, 8385879, 4193963,
    2097109, 1048571, 524287, 262144, 131072, 65536, 32768, 16384, 8192,
    4096, 2048, 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2 };

/* --------------------------------------------------------------------------
 * register access
 * -------------------------------------------------------------------------- */
static uint32_t rd(uint32_t off)              { return gnss_pt_read(off); }
static void     wr(uint32_t off, uint32_t v)  { gnss_pt_write(off, v); }

static void ctrl_update(uint32_t clear_mask, uint32_t set_mask)
{
    uint32_t c = rd(DOA_REG_CTRL);
    c = (c & ~clear_mask) | set_mask;
    wr(DOA_REG_CTRL, c);
}

/* --------------------------------------------------------------------------
 * integer CORDIC
 * -------------------------------------------------------------------------- */
static void cordic_rot(int32_t ang_q28, int32_t *c_q15, int32_t *s_q15)
{
    int32_t x, y, z = ang_q28, xn;
    int     neg = 0, i;

    /* reduce to [-pi, pi] then to [-pi/2, pi/2] */
    while (z >  DOA_PI_Q28) { z -= 2 * DOA_PI_Q28; }
    while (z < -DOA_PI_Q28) { z += 2 * DOA_PI_Q28; }
    if (z >  DOA_HALF_PI_Q28) { z -= DOA_PI_Q28; neg = 1; }
    if (z < -DOA_HALF_PI_Q28) { z += DOA_PI_Q28; neg = 1; }

    x = DOA_CORDIC_GAIN;  y = 0;            /* Q30 */
    for (i = 0; i < 28; i++) {
        if (z >= 0) { xn = x - (y >> i); y = y + (x >> i); z -= doa_atan_q28[i]; }
        else        { xn = x + (y >> i); y = y - (x >> i); z += doa_atan_q28[i]; }
        x = xn;
    }
    if (neg) { x = -x; y = -y; }
    *c_q15 = (x + (1 << 14)) >> 15;
    *s_q15 = (y + (1 << 14)) >> 15;
    if (*c_q15 > 32767) { *c_q15 = 32767; }
    if (*s_q15 > 32767) { *s_q15 = 32767; }
}

int32_t gnss_doa_sin_q15(int32_t ang_q28) { int32_t c, s; cordic_rot(ang_q28, &c, &s); return s; }
int32_t gnss_doa_cos_q15(int32_t ang_q28) { int32_t c, s; cordic_rot(ang_q28, &c, &s); return c; }

int32_t gnss_doa_atan2_q28(int32_t y, int32_t x)
{
    int32_t z = 0, xn;
    int     i;

    if (x == 0 && y == 0) { return 0; }
    while ((x > (1 << 28)) || (x < -(1 << 28)) || (y > (1 << 28)) || (y < -(1 << 28))) {
        x >>= 1; y >>= 1;
    }
    if (x < 0) {                          /* rotate by pi into the right half */
        z = (y >= 0) ? DOA_PI_Q28 : -DOA_PI_Q28;
        x = -x; y = -y;
    }
    for (i = 0; i < 28; i++) {
        if (y > 0) { xn = x + (y >> i); y = y - (x >> i); z += doa_atan_q28[i]; }
        else       { xn = x - (y >> i); y = y + (x >> i); z -= doa_atan_q28[i]; }
        x = xn;
    }
    return z;
}

/* --------------------------------------------------------------------------
 * printing helpers (fixed point -> decimal, no %f)
 * -------------------------------------------------------------------------- */
static void put_fixed(int64_t v, int frac_bits, int decimals)
{
    int64_t  scale = 1, a, ip, fp;
    int      k;
    for (k = 0; k < decimals; k++) { scale *= 10; }
    a  = (v < 0) ? -v : v;
    ip = a >> frac_bits;
    fp = (((a & ((1LL << frac_bits) - 1)) * scale) + (1LL << (frac_bits - 1))) >> frac_bits;
    if (fp >= scale) { ip += 1; fp -= scale; }
    printf("%c%ld.", (v < 0) ? '-' : ' ', (long)ip);
    for (k = decimals - 1; k >= 0; k--) {
        int64_t d = fp;
        int     j;
        for (j = 0; j < k; j++) { d /= 10; }
        printf("%d", (int)(d % 10));
    }
}
static void put_deg(int32_t q16)   { put_fixed(q16, 16, 3); }
static void put_q24(int32_t q24)   { put_fixed(q24, 24, 4); }
static void put_mdeg(int32_t m)    { put_fixed(((int64_t)m << 16) / 1000, 16, 3); }

static uint64_t isqrt64(uint64_t v)
{
    uint64_t r = 0, b = 1ULL << 62;
    while (b > v) { b >>= 2; }
    while (b) {
        if (v >= r + b) { v -= r + b; r = (r >> 1) + b; }
        else            { r >>= 1; }
        b >>= 2;
    }
    return r;
}

/* standard deviation (milli-degrees) from mean (Q16) and mean square (Q32) */
static int32_t std_mdeg(int32_t mean_q16, uint32_t sq_lo, uint32_t sq_hi)
{
    uint64_t msq  = ((uint64_t)sq_hi << 32) | sq_lo;
    int64_t  m2   = (int64_t)mean_q16 * (int64_t)mean_q16;
    int64_t  var  = (int64_t)msq - m2;
    if (var <= 0) { return 0; }
    return (int32_t)((isqrt64((uint64_t)var) * 1000ULL) >> 16);
}

static int32_t iabs32(int32_t v) { return (v < 0) ? -v : v; }

/* --------------------------------------------------------------------------
 * phase helpers (firmware only; the FPGA already provides everything)
 *
 *   PHI registers hold arg(x1/x0) AFTER the CAL rotation x1' = x1 (c + j s).
 *   With alpha = atan2(s, c):   phi_cal = phi_raw + alpha
 *                               phi_raw = phi_cal - alpha
 *   doa_cal=1 writes alpha = -(measured offset), so "offset removed" = -alpha.
 * -------------------------------------------------------------------------- */
#define DOA_COH_SOURCE_Q15  29491     /* coherence >= 0.90: one coherent source */
#define DOA_COH_NOISE_Q15   16384     /* coherence <  0.50: noise only          */
#define DOA_PEAK_CLIP       31000U    /* near ADC full scale (32767)            */
#define DOA_LOG2_FS_Q16     983037L   /* log2(32767) in Q16                     */

static struct {
    int      valid;
    uint32_t reg;                     /* DOA_CAL value doa_cal=1 wrote          */
    int32_t  offset_q16;              /* measured RX2-RX1 offset, deg Q16       */
    int32_t  coh_q15;                 /* coherence of the calibration signal    */
    uint32_t n;                       /* windows averaged                       */
    int      check_valid;             /* re-measured right after calibrating    */
    int32_t  check_phase_q16;         /* residual phase vs expected 0, deg Q16  */
    int32_t  check_doa_q16;           /* residual DOA vs expected 0, deg Q16    */
} cal_info;

static int32_t q28_to_degq16(int32_t a)
{
    return (int32_t)(((int64_t)a * 180 * 65536) / DOA_PI_Q28);
}

static int32_t wrap_q28(int64_t a)
{
    while (a >  DOA_PI_Q28) { a -= 2 * (int64_t)DOA_PI_Q28; }
    while (a < -DOA_PI_Q28) { a += 2 * (int64_t)DOA_PI_Q28; }
    return (int32_t)a;
}

static int32_t cal_alpha_q28(uint32_t cal)
{
    return gnss_doa_atan2_q28((int16_t)(cal >> 16), (int16_t)(cal & 0xFFFFU));
}

/* log2(v) in Q16, v > 0 (bit-by-bit, integer only) */
static int32_t log2_q16(uint32_t v)
{
    int32_t  ip = 0, r, k;
    uint64_t m;
    while ((v >> ip) > 1U) { ip++; }
    m = ((uint64_t)v << 30) >> ip;            /* Q30, in [1, 2) */
    r = ip << 16;
    for (k = 15; k >= 0; k--) {
        m = (m * m) >> 30;
        if (m >= (2ULL << 30)) { m >>= 1; r |= (1 << k); }
    }
    return r;
}

/* 20 log10(peak / 32767) in tenths of a dB */
static int32_t dbfs_x10(uint32_t peak)
{
    if (peak == 0U) { return -999; }
    return (int32_t)(((int64_t)(log2_q16(peak) - DOA_LOG2_FS_Q16) * 60206) / 65536000);
}

/* deg Q16 -> tenths of a degree, rounded */
static int32_t degq16_x10(int32_t q16)
{
    int64_t v = (int64_t)q16 * 10;
    return (int32_t)((v + ((v >= 0) ? 32768 : -32768)) / 65536);
}

static void put_x10(int32_t v)
{
    int32_t a = iabs32(v);
    printf("%c%ld.%ld", (v < 0) ? '-' : ' ', (long)(a / 10), (long)(a % 10));
}

/* coherence (Q15) and mean angle (Q28) of sum(cos), sum(sin) over n unit vectors */
static void circ_mean(int64_t sc, int64_t ss, uint32_t n, int32_t *mean_q28, int32_t *coh_q15)
{
    uint64_t r;
    if (n == 0U) { *mean_q28 = 0; *coh_q15 = 0; return; }
    r = isqrt64((uint64_t)(sc * sc) + (uint64_t)(ss * ss)) / n;
    *coh_q15 = (r > 32767U) ? 32767 : (int32_t)r;
    while ((sc > 0x3FFFFFFF) || (sc < -0x3FFFFFFF) || (ss > 0x3FFFFFFF) || (ss < -0x3FFFFFFF)) {
        sc >>= 1; ss >>= 1;
    }
    *mean_q28 = gnss_doa_atan2_q28((int32_t)ss, (int32_t)sc);
}

/* DOA shift (deg Q16) that a phase offset alone causes at broadside for the
 * current K:  theta = asin(-k * offset).  This is how far the angle would be
 * off if the board/cable offset were NOT removed by the calibration. */
static int32_t doa_shift_for_phase(int32_t off_q28)
{
    int64_t  u = ((int64_t)(int32_t)rd(DOA_REG_K) * -(int64_t)off_q28) >> 30;   /* Q28 */
    uint64_t c2;
    int32_t  c, a;
    if (u >  (1LL << 28)) { u =  (1LL << 28); }
    if (u < -(1LL << 28)) { u = -(1LL << 28); }
    c2 = (1ULL << 56) - (uint64_t)(u * u);
    c  = (int32_t)isqrt64(c2);                                                 /* Q28 */
    a  = gnss_doa_atan2_q28((int32_t)u, c);
    return q28_to_degq16(a);
}

static void put_shift(int32_t off_q28)
{
    printf("= ");
    put_deg(doa_shift_for_phase(off_q28));
    printf(" deg DOA error if not removed (broadside)");
}

/* --------------------------------------------------------------------------
 * state
 * -------------------------------------------------------------------------- */
void gnss_doa_get_state(doa_state_t *st)
{
    uint32_t caps = rd(DOA_REG_CAPS);
    st->has_i    = (caps & 1U) ? 1 : 0;
    st->has_m    = (caps & 2U) ? 1 : 0;
    st->ctrl     = rd(DOA_REG_CTRL);
    st->status   = rd(DOA_REG_STATUS);
    st->in_count = rd(DOA_REG_IN_COUNT);
    st->peak     = rd(DOA_REG_PEAK);

    st->i.seq         = rd(DOA_REG_I_SEQ);
    st->i.phi_q28     = (int32_t)rd(DOA_REG_I_PHI);
    st->i.psi_q24     = (int32_t)rd(DOA_REG_I_PSI);
    st->i.deg_q16     = (int32_t)rd(DOA_REG_I_DEG);
    st->i.avg_seq     = rd(DOA_REG_I_AVG_SEQ);
    st->i.avg_deg_q16 = (int32_t)rd(DOA_REG_I_AVG_DEG);
    st->i.std_mdeg    = std_mdeg(st->i.avg_deg_q16, rd(DOA_REG_I_AVG_SQ_LO), rd(DOA_REG_I_AVG_SQ_HI));
    st->i.flags       = (st->status >> 8) & 0xFU;

    st->m.seq         = rd(DOA_REG_M_SEQ);
    st->m.phi_q28     = (int32_t)rd(DOA_REG_M_PHI);
    st->m.psi_q24     = (int32_t)rd(DOA_REG_M_PSIMAG);
    st->m.deg_q16     = (int32_t)rd(DOA_REG_M_DEG);
    st->m.avg_seq     = rd(DOA_REG_M_AVG_SEQ);
    st->m.avg_deg_q16 = (int32_t)rd(DOA_REG_M_AVG_DEG);
    st->m.std_mdeg    = std_mdeg(st->m.avg_deg_q16, rd(DOA_REG_M_AVG_SQ_LO), rd(DOA_REG_M_AVG_SQ_HI));
    st->m.flags       = (st->status >> 12) & 0xFU;
    st->m_offcirc     = rd(DOA_REG_M_OFFCIRC);
}

/* wait until the averaged-result counter of the primary estimator has moved
 * on by `blocks`; returns DOA_OK or DOA_ERR_TIMEOUT */
static int32_t wait_avg(uint32_t blocks)
{
    uint32_t caps = rd(DOA_REG_CAPS);
    uint32_t reg  = (caps & 2U) ? DOA_REG_M_AVG_SEQ : DOA_REG_I_AVG_SEQ;
    uint32_t base = rd(reg);
    uint32_t t;
    for (t = 0; t < DOA_TIMEOUT_MS; t++) {
        if ((rd(reg) - base) >= blocks) {
            /* the other estimator finishes within a few cycles */
            no_os_udelay(5);
            return DOA_OK;
        }
        no_os_mdelay(1);
    }
    return DOA_ERR_TIMEOUT;
}

/* --------------------------------------------------------------------------
 * configuration
 * -------------------------------------------------------------------------- */
int32_t gnss_doa_probe(void)
{
    uint32_t id   = rd(DOA_REG_ID);
    uint32_t ver  = rd(DOA_REG_VERSION);
    uint32_t caps = rd(DOA_REG_CAPS);

    if (id != DOA_EXPECTED_ID) {
        printf("DOA: not present (DOA_ID at 0x%08lx reads 0x%08lx, expected 0x%08lx).\n"
               "     The bitstream predates gnss_passthrough v1.2. Re-package the IP\n"
               "     and rebuild the FPGA (Docs/ESPRIT/README_ESPRIT.md, section 7).\n",
               (unsigned long)(GNSS_PT_BASEADDR + DOA_REG_ID),
               (unsigned long)id, (unsigned long)DOA_EXPECTED_ID);
        return DOA_ERR_ID;
    }
    printf("DOA: ESPRIT DOA monitor present, version %lu.%lu, estimators:%s%s (%s), max window 2^%lu\n",
           (unsigned long)(ver >> 16), (unsigned long)(ver & 0xFFFFU),
           (caps & 1U) ? " I-UESPRIT" : "", (caps & 2U) ? " M-ESPRIT" : "",
           (caps & 4U) ? "Vitis HLS IP in the block design" : "RTL core in gnss_passthrough",
           (unsigned long)((caps >> 8) & 0xFFU));
    return DOA_OK;
}

void gnss_doa_defaults(void)
{
    wr(DOA_REG_CTRL,   DOA_CTRL_P(2) | DOA_CTRL_SNAP(10) | DOA_CTRL_AVG(4));
    wr(DOA_REG_MU,     1024U);
    wr(DOA_REG_K,      DOA_K_HALF_LAMBDA);
    wr(DOA_REG_UCTOL,  0x00400000U);
    wr(DOA_REG_DECIM,  0U);
    wr(DOA_REG_CAL,    0x00007FFFU);
    wr(DOA_REG_TG_FTW, 0x0147AE14U);
    wr(DOA_REG_TG_DPHASE, 0U);
    wr(DOA_REG_TG_CFG, 0x00400064U);
}

void gnss_doa_enable_live(int enable)
{
    if (enable) {
        ctrl_update(DOA_CTRL_SRC_TEST | DOA_CTRL_ENABLE, 0);
        ctrl_update(0, DOA_CTRL_ENABLE);
        printf("DOA: live RX1/RX2 DOA ENABLED (receive-only, TX path untouched)\n");
    } else {
        ctrl_update(DOA_CTRL_ENABLE, 0);
        printf("DOA: disabled\n");
    }
}

void gnss_doa_set_window(uint32_t n)
{
    if (n < 7)  { n = 7; }
    if (n > 14) { n = 14; }
    ctrl_update(DOA_CTRL_SNAP(0x1F), DOA_CTRL_SNAP(n));
    printf("DOA: window = 2^%lu = %lu snapshots\n", (unsigned long)n, (unsigned long)(1UL << n));
}

void gnss_doa_set_p(uint32_t n)
{
    n &= 3U;
    ctrl_update(DOA_CTRL_P(3), DOA_CTRL_P(n));
    printf("DOA: C^p with p = %lu\n", (unsigned long)(1UL << n));
}

void gnss_doa_set_avg(uint32_t n)
{
    if (n > 10) { n = 10; }
    ctrl_update(DOA_CTRL_AVG(0xF), DOA_CTRL_AVG(n));
    printf("DOA: averaging %lu results per reported value\n", (unsigned long)(1UL << n));
}

void gnss_doa_set_cov(int use_cov)
{
    ctrl_update(DOA_CTRL_USE_COV, use_cov ? DOA_CTRL_USE_COV : 0);
    printf("DOA: subspace from %s\n", use_cov ? "COVARIANCE R (Gaussian-noise jammers)"
                                              : "4th-order CUMULANT C (paper default)");
}

void gnss_doa_set_decim(uint32_t n)
{
    wr(DOA_REG_DECIM, n & 0xFFFFU);
    printf("DOA: keeping 1 of every %lu snapshots\n", (unsigned long)((n & 0xFFFFU) + 1U));
}

void gnss_doa_set_spacing(double spacing_m, double freq_hz)
{
    const double pi = 3.14159265358979323846;
    double lambda, k;
    uint32_t kq;
    if (spacing_m <= 0.0 || freq_hz <= 0.0) {
        printf("DOA: spacing and frequency must be positive\n");
        return;
    }
    lambda = 299792458.0 / freq_hz;
    k  = lambda / (2.0 * pi * spacing_m);
    if (k > 1.99) { k = 1.99; }
    kq = (uint32_t)(k * 1073741824.0 + 0.5);
    wr(DOA_REG_K, kq);
    printf("DOA: spacing %ld mm at %ld kHz -> lambda %ld mm, d/lambda = %ld/1000, K_Q30 = %lu\n",
           (long)(spacing_m * 1000.0 + 0.5), (long)(freq_hz / 1000.0),
           (long)(lambda * 1000.0 + 0.5), (long)(spacing_m / lambda * 1000.0 + 0.5),
           (unsigned long)kq);
    if (spacing_m > lambda / 2.0) {
        printf("DOA: WARNING d > lambda/2: the angle is unambiguous only for\n"
               "     |sin(theta)| < lambda/(2d); outside that range it aliases.\n");
    }
}

/* --------------------------------------------------------------------------
 * reporting
 * -------------------------------------------------------------------------- */
static void print_est(const char *name, const doa_est_t *e, int is_m, uint32_t offc, uint32_t navg)
{
    printf("DOA:   %-9s window #%-8lu theta ", name, (unsigned long)e->seq);
    put_deg(e->deg_q16);
    printf(" deg   avg #%-6lu mean ", (unsigned long)e->avg_seq);
    put_deg(e->avg_deg_q16);
    printf(" deg  std ");
    put_mdeg(e->std_mdeg);
    printf(" deg  phi ");
    put_fixed(e->phi_q28, 28, 4);
    printf(" rad  %s", is_m ? "|psi| " : "psi ");
    put_q24(e->psi_q24);
    printf("  flags 0x%lx", (unsigned long)e->flags);
    if (is_m) { printf("  off-circle %lu/%lu", (unsigned long)offc, (unsigned long)navg); }
    printf("\n");
}

static void print_flags_legend(uint32_t fi, uint32_t fm)
{
    if ((fi | fm) & 1U) { printf("DOA:   ! no_signal: cumulant matrix was all zero (no input?)\n"); }
    if ((fi | fm) & 2U) { printf("DOA:   ! ambiguous: |k*phi| > 1, angle clipped to +/-90 deg (check K / spacing)\n"); }
    if ((fi | fm) & 4U) { printf("DOA:   ! overrun: a window was dropped (window too short for the sample rate)\n"); }
    if (fm & 8U)        { printf("DOA:   ! off_circle: |psi| far from 1 -> no coherent source (Remark A)\n"); }
}

static void quick_jammer(void);

void gnss_doa_print_result(void)
{
    doa_state_t s;
    uint32_t navg;
    gnss_doa_get_state(&s);
    navg = 1UL << (((s.ctrl >> 16) & 0xFU) > 10 ? 10 : ((s.ctrl >> 16) & 0xFU));

    printf("DOA: ---- ESPRIT direction of arrival (source: %s, %s) ----\n",
           (s.ctrl & DOA_CTRL_ENABLE) ? ((s.ctrl & DOA_CTRL_SRC_TEST) ? "TEST GENERATOR" : "LIVE RX1/RX2")
                                      : "DISABLED",
           (s.ctrl & DOA_CTRL_USE_COV) ? "covariance" : "cumulant");
    if (s.has_i) { print_est("I-UESPRIT", &s.i, 0, 0, navg); }
    if (s.has_m) { print_est("M-ESPRIT",  &s.m, 1, s.m_offcirc, navg); }
    printf("DOA:   snapshots %lu  peak |x0| %lu  |x1| %lu (of 32767)%s\n",
           (unsigned long)s.in_count, (unsigned long)(s.peak & 0xFFFFU),
           (unsigned long)(s.peak >> 16),
           (s.status & 1U) ? "  CDC-OVERFLOW!" : "");
    {
        int32_t alpha = cal_alpha_q28(rd(DOA_REG_CAL));
        int32_t pcal  = s.has_m ? s.m.phi_q28 : s.i.phi_q28;
        printf("DOA:   phase RX2-RX1 (last window): raw ");
        put_deg(q28_to_degq16(wrap_q28((int64_t)pcal - alpha)));
        printf(" deg  calibrated ");
        put_deg(q28_to_degq16(pcal));
        printf(" deg\n");
        if (rd(DOA_REG_CAL) == 0x00007FFFU) {
            printf("DOA:   calibration offset removed: none (not calibrated, doa_cal=1)\n");
        } else {
            printf("DOA:   calibration offset removed ");
            put_deg(q28_to_degq16(-alpha));
            printf(" deg phase ");
            put_shift(-alpha);
            printf("\n");
        }
    }
    if (s.ctrl & DOA_CTRL_ENABLE) { quick_jammer(); }
    print_flags_legend(s.i.flags, s.m.flags);
}

void gnss_doa_print_regs(void)
{
    uint32_t off;
    printf("DOA: register dump, base 0x%08lx\n", (unsigned long)GNSS_PT_BASEADDR);
    for (off = 0x100U; off <= 0x194U; off += 4U) {
        printf("DOA:   0x%03lx = 0x%08lx\n", (unsigned long)off, (unsigned long)rd(off));
    }
}

void gnss_doa_print_status(void)
{
    uint32_t c = rd(DOA_REG_CTRL), cal = rd(DOA_REG_CAL);
    int32_t  cs = (int16_t)(cal & 0xFFFFU), sn = (int16_t)(cal >> 16);
    int32_t  calang = gnss_doa_atan2_q28(sn, cs);
    int32_t  calq16 = (int32_t)(((int64_t)calang * 180 * 65536) / DOA_PI_Q28);

    printf("DOA: ---- configuration ----\n");
    printf("DOA:   enable %lu  source %s  subspace %s  p = %lu  window %lu  average %lu\n",
           (unsigned long)(c & 1U), (c & DOA_CTRL_SRC_TEST) ? "test-gen" : "live RX",
           (c & DOA_CTRL_USE_COV) ? "covariance" : "cumulant",
           (unsigned long)(1UL << ((c >> 4) & 3U)), (unsigned long)(1UL << ((c >> 8) & 0x1FU)),
           (unsigned long)(1UL << ((c >> 16) & 0xFU)));
    printf("DOA:   MU_Q30 %lu  K_Q30 %lu (1/pi = 341782638 for d = lambda/2)  UCTOL_Q24 0x%08lx\n",
           (unsigned long)rd(DOA_REG_MU), (unsigned long)rd(DOA_REG_K), (unsigned long)rd(DOA_REG_UCTOL));
    printf("DOA:   DECIM %lu  CAL cos %ld sin %ld (rotation ", (unsigned long)rd(DOA_REG_DECIM),
           (long)cs, (long)sn);
    put_deg(calq16);
    printf(" deg)\n");
    printf("DOA:   TG_FTW 0x%08lx  TG_DPHASE 0x%08lx  TG_CFG 0x%08lx\n",
           (unsigned long)rd(DOA_REG_TG_FTW), (unsigned long)rd(DOA_REG_TG_DPHASE),
           (unsigned long)rd(DOA_REG_TG_CFG));
    gnss_doa_print_result();
}

int32_t gnss_doa_monitor(uint32_t n)
{
    doa_state_t s;
    uint32_t k, navg;
    if (n == 0) { n = 10; }
    printf("DOA: monitoring %lu averaged results (Ctrl-C does not abort; keep N small)\n",
           (unsigned long)n);
    for (k = 0; k < n; k++) {
        if (wait_avg(1) != DOA_OK) {
            printf("DOA: no new result within %u ms - is DOA enabled and is RX running?\n",
                   DOA_TIMEOUT_MS);
            return DOA_ERR_TIMEOUT;
        }
        gnss_doa_get_state(&s);
        navg = 1UL << (((s.ctrl >> 16) & 0xFU) > 10 ? 10 : ((s.ctrl >> 16) & 0xFU));
        printf("DOA: #%-4lu", (unsigned long)(k + 1));
        if (s.has_i) { printf(" I-UESPRIT "); put_deg(s.i.avg_deg_q16); printf(" (std "); put_mdeg(s.i.std_mdeg); printf(")"); }
        if (s.has_m) {
            printf("  M-ESPRIT "); put_deg(s.m.avg_deg_q16); printf(" (std "); put_mdeg(s.m.std_mdeg);
            printf(") |psi| "); put_q24(s.m.psi_q24);
            printf(" offc %lu/%lu", (unsigned long)s.m_offcirc, (unsigned long)navg);
        }
        printf("  pk %lu/%lu  fl %lx/%lx\n", (unsigned long)(s.peak & 0xFFFFU),
               (unsigned long)(s.peak >> 16), (unsigned long)s.i.flags, (unsigned long)s.m.flags);
    }
    return DOA_OK;
}

/* --------------------------------------------------------------------------
 * test generator
 * -------------------------------------------------------------------------- */
/* inter-element phase that makes the estimator report theta for the CURRENT K:
 *   theta = asin(-k*phi)  ->  phi = -sin(theta)/k,  DPHASE = phi/(2 pi) * 2^32 */
static uint32_t dphase_for(double theta_deg)
{
    const double pi = 3.14159265358979323846;
    int32_t  ang  = (int32_t)(theta_deg * (double)DOA_DEG2RAD_Q28);
    double   s    = gnss_doa_sin_q15(ang) / 32768.0;
    double   k    = rd(DOA_REG_K) / 1073741824.0;
    double   phi  = -s / k;                       /* rad */
    double   turns = phi / (2.0 * pi);
    while (turns >  0.5) { turns -= 1.0; }
    while (turns < -0.5) { turns += 1.0; }
    return (uint32_t)(int32_t)(turns * 4294967296.0);
}

int32_t gnss_doa_test_angle(double theta_deg)
{
    doa_state_t s;
    uint32_t saved_ctrl = rd(DOA_REG_CTRL);
    uint32_t saved_cal  = rd(DOA_REG_CAL);
    int32_t  exp_q16 = (int32_t)(theta_deg * 65536.0);
    int32_t  ret = DOA_OK;

    /* The calibration rotation sits after the source mux, so it would also
     * rotate the synthetic source. Measure with an identity CAL, then put the
     * user's calibration and mode back. */
    wr(DOA_REG_CTRL, 0);
    wr(DOA_REG_CAL, 0x00007FFFU);
    wr(DOA_REG_TG_DPHASE, dphase_for(theta_deg));
    wr(DOA_REG_CTRL, (saved_ctrl & ~DOA_CTRL_TG_NOISE) | DOA_CTRL_ENABLE | DOA_CTRL_SRC_TEST);
    if (wait_avg(2) != DOA_OK) {
        printf("DOA: test generator produced no result (timeout)\n");
        ret = DOA_ERR_TIMEOUT;
    } else {
        gnss_doa_get_state(&s);
        printf("DOA: test generator set to ");
        put_deg(exp_q16);
        printf(" deg (DPHASE 0x%08lx, current K)\n", (unsigned long)rd(DOA_REG_TG_DPHASE));
        if (s.has_i) { printf("DOA:   I-UESPRIT mean "); put_deg(s.i.avg_deg_q16); printf("  error "); put_deg(s.i.avg_deg_q16 - exp_q16); printf(" deg\n"); }
        if (s.has_m) { printf("DOA:   M-ESPRIT  mean "); put_deg(s.m.avg_deg_q16); printf("  error "); put_deg(s.m.avg_deg_q16 - exp_q16); printf(" deg  |psi| "); put_q24(s.m.psi_q24); printf("\n"); }
    }
    wr(DOA_REG_CTRL, 0);
    wr(DOA_REG_CAL, saved_cal);
    wr(DOA_REG_CTRL, saved_ctrl);
    printf("DOA: previous mode and calibration restored\n");
    return ret;
}

/* --------------------------------------------------------------------------
 * self-test
 * -------------------------------------------------------------------------- */
static const struct { int16_t deg; uint32_t dphase; } doa_sweep[] = {
    { -60, 0x6ED9EBA1U }, { -45, 0x5A82799AU }, { -30, 0x40000000U },
    { -15, 0x2120FB83U }, {   0, 0x00000000U }, {  15, 0xDEDF047DU },
    {  30, 0xC0000000U }, {  45, 0xA57D8666U }, {  60, 0x9126145FU } };
#define DOA_SWEEP_N  (sizeof(doa_sweep) / sizeof(doa_sweep[0]))

static int st_fail, st_checks;

static void st_check(const char *what, int ok)
{
    st_checks++;
    if (!ok) { st_fail++; }
    printf("DOA_SELFTEST: %-58s %s\n", what, ok ? "PASS" : "FAIL");
}

int32_t gnss_doa_selftest(int verbose)
{
    uint32_t sv_ctrl = rd(DOA_REG_CTRL), sv_k = rd(DOA_REG_K), sv_cal = rd(DOA_REG_CAL);
    uint32_t sv_cfg = rd(DOA_REG_TG_CFG), sv_dph = rd(DOA_REG_TG_DPHASE), sv_dec = rd(DOA_REG_DECIM);
    uint32_t caps, i, c0, c1;
    doa_state_t s;
    int32_t  err_i, err_m, worst_i = 0, worst_m = 0;
    const int32_t tol = 3277;                      /* 0.05 deg in Q16 */

    st_fail = 0; st_checks = 0;
    printf("DOA_SELFTEST: ===== ESPRIT DOA hardware self-test =====\n");

    /* S1 identity ----------------------------------------------------------*/
    st_check("S1 DOA block present (DOA_ID = 0x444F4131)", rd(DOA_REG_ID) == DOA_EXPECTED_ID);
    if (rd(DOA_REG_ID) != DOA_EXPECTED_ID) {
        printf("DOA_SELFTEST: RESULT FAIL - bitstream has no DOA block\n");
        return DOA_ERR_ID;
    }
    caps = rd(DOA_REG_CAPS);
    st_check("S1 at least one estimator built", (caps & 3U) != 0);
    st_check("S1 gnss_passthrough reports v1.2", gnss_pt_read(GNSS_PT_REG_VERSION) == 0x00010002U);

    /* S2 test-generator sweep, noiseless ------------------------------------*/
    wr(DOA_REG_CTRL, 0);
    wr(DOA_REG_K, DOA_K_HALF_LAMBDA);
    wr(DOA_REG_CAL, 0x00007FFFU);
    wr(DOA_REG_DECIM, 0);
    wr(DOA_REG_TG_CFG, 0x00000008U);               /* 12.5 MS/s, full scale, no noise */
    wr(DOA_REG_TG_DPHASE, doa_sweep[0].dphase);
    wr(DOA_REG_CTRL, DOA_CTRL_ENABLE | DOA_CTRL_SRC_TEST |
                     DOA_CTRL_P(2) | DOA_CTRL_SNAP(10) | DOA_CTRL_AVG(2));
    printf("DOA_SELFTEST: S2 sweep: set angle | I-UESPRIT (error) | M-ESPRIT (error) |psi|\n");
    for (i = 0; i < DOA_SWEEP_N; i++) {
        wr(DOA_REG_TG_DPHASE, doa_sweep[i].dphase);
        if (wait_avg(2) != DOA_OK) { st_check("S2 result produced (timeout)", 0); break; }
        gnss_doa_get_state(&s);
        err_i = s.i.avg_deg_q16 - ((int32_t)doa_sweep[i].deg << 16);
        err_m = s.m.avg_deg_q16 - ((int32_t)doa_sweep[i].deg << 16);
        if (s.has_i && iabs32(err_i) > worst_i) { worst_i = iabs32(err_i); }
        if (s.has_m && iabs32(err_m) > worst_m) { worst_m = iabs32(err_m); }
        printf("DOA_SELFTEST:   %4d deg |", (int)doa_sweep[i].deg);
        if (s.has_i) { put_deg(s.i.avg_deg_q16); printf(" ("); put_deg(err_i); printf(")"); } else { printf("      n/a        "); }
        printf(" |");
        if (s.has_m) { put_deg(s.m.avg_deg_q16); printf(" ("); put_deg(err_m); printf(") "); put_q24(s.m.psi_q24); }
        printf("\n");
    }
    if (caps & 1U) { st_check("S2 I-UESPRIT sweep -60..60 deg, every error < 0.05 deg", worst_i < tol); }
    if (caps & 2U) { st_check("S2 M-ESPRIT  sweep -60..60 deg, every error < 0.05 deg", worst_m < tol); }

    /* S3 noise robustness ----------------------------------------------------*/
    wr(DOA_REG_CTRL, 0);
    wr(DOA_REG_TG_CFG, 0x00300008U);               /* noise >> 3 */
    wr(DOA_REG_TG_DPHASE, 0xD438AF17U);            /* +20 deg */
    wr(DOA_REG_CTRL, DOA_CTRL_ENABLE | DOA_CTRL_SRC_TEST | DOA_CTRL_TG_NOISE |
                     DOA_CTRL_P(2) | DOA_CTRL_SNAP(10) | DOA_CTRL_AVG(4));
    if (wait_avg(2) == DOA_OK) {
        gnss_doa_get_state(&s);
        printf("DOA_SELFTEST: S3 noisy +20 deg: I mean "); put_deg(s.i.avg_deg_q16);
        printf(" std "); put_mdeg(s.i.std_mdeg); printf(" | M mean "); put_deg(s.m.avg_deg_q16);
        printf(" std "); put_mdeg(s.m.std_mdeg); printf("\n");
        if (caps & 1U) { st_check("S3 I-UESPRIT with noise within 0.5 deg", iabs32(s.i.avg_deg_q16 - (20 << 16)) < 32768); }
        if (caps & 2U) { st_check("S3 M-ESPRIT  with noise within 0.5 deg", iabs32(s.m.avg_deg_q16 - (20 << 16)) < 32768); }
    } else { st_check("S3 result produced (timeout)", 0); }

    /* S4 calibration rotation ------------------------------------------------*/
    wr(DOA_REG_CTRL, 0);
    wr(DOA_REG_TG_CFG, 0x00000008U);
    wr(DOA_REG_TG_DPHASE, 0xC0000000U);            /* phase -90 deg (= +30 deg) */
    wr(DOA_REG_CAL, 0x7FFF0000U);                  /* rotate element 1 by +90 deg */
    wr(DOA_REG_CTRL, DOA_CTRL_ENABLE | DOA_CTRL_SRC_TEST |
                     DOA_CTRL_P(2) | DOA_CTRL_SNAP(10) | DOA_CTRL_AVG(2));
    if (wait_avg(2) == DOA_OK) {
        gnss_doa_get_state(&s);
        printf("DOA_SELFTEST: S4 -90 deg phase + CAL +90 deg -> I "); put_deg(s.i.avg_deg_q16);
        printf("  M "); put_deg(s.m.avg_deg_q16); printf(" (expect 0)\n");
        st_check("S4 calibration rotation cancels the phase (|theta| < 0.05)",
                 iabs32((caps & 2U) ? s.m.avg_deg_q16 : s.i.avg_deg_q16) < tol);
    } else { st_check("S4 result produced (timeout)", 0); }
    wr(DOA_REG_CAL, 0x00007FFFU);

    /* S5 no-source detection (M-ESPRIT unit-circle test, Remark A) ----------*/
    if (caps & 2U) {
        wr(DOA_REG_CTRL, 0);
        wr(DOA_REG_TG_CFG, 0x002F0008U);           /* amplitude >> 15 (~1 LSB), noise >> 2 */
        wr(DOA_REG_CTRL, DOA_CTRL_ENABLE | DOA_CTRL_SRC_TEST | DOA_CTRL_TG_NOISE |
                         DOA_CTRL_P(2) | DOA_CTRL_SNAP(10) | DOA_CTRL_AVG(4));
        if (wait_avg(2) == DOA_OK) {
            gnss_doa_get_state(&s);
            printf("DOA_SELFTEST: S5 noise only: |psi| "); put_q24(s.m.psi_q24);
            printf("  off-circle %lu/16\n", (unsigned long)s.m_offcirc);
            st_check("S5 M-ESPRIT flags 'no source' (off-circle >= 8/16)", s.m_offcirc >= 8U);
        } else { st_check("S5 result produced (timeout)", 0); }
    }

    /* S6 live RX path --------------------------------------------------------*/
    wr(DOA_REG_CTRL, 0);
    wr(DOA_REG_CTRL, DOA_CTRL_ENABLE | DOA_CTRL_P(2) | DOA_CTRL_SNAP(10) | DOA_CTRL_AVG(4));
    no_os_mdelay(2);
    c0 = rd(DOA_REG_IN_COUNT);
    no_os_mdelay(10);
    c1 = rd(DOA_REG_IN_COUNT);
    printf("DOA_SELFTEST: S6 live RX1/RX2 snapshots in 10 ms: %lu (expect ~ fs/100)\n",
           (unsigned long)(c1 - c0));
    st_check("S6 live RX1/RX2 samples reach the estimators", (c1 - c0) > 1000U);
    st_check("S6 no CDC FIFO overflow", (rd(DOA_REG_STATUS) & 1U) == 0U);

    /* restore ----------------------------------------------------------------*/
    wr(DOA_REG_CTRL, 0);
    wr(DOA_REG_K, sv_k); wr(DOA_REG_CAL, sv_cal); wr(DOA_REG_TG_CFG, sv_cfg);
    wr(DOA_REG_TG_DPHASE, sv_dph); wr(DOA_REG_DECIM, sv_dec);
    wr(DOA_REG_CTRL, sv_ctrl);

    printf("DOA_SELFTEST: ===== RESULT %s (%d/%d checks passed) =====\n",
           st_fail ? "FAIL" : "PASS", st_checks - st_fail, st_checks);
    (void)verbose;
    return st_fail ? DOA_ERR_SELFTEST : DOA_OK;
}

/* collect the phase of `nwin` consecutive estimator windows (primary
 * estimator: M-ESPRIT if built, else I-UESPRIT). The phase is averaged as a
 * unit vector, so the result is robust near +/-180 deg, and the length of the
 * mean vector is the COHERENCE: 1.0 when RX1 and RX2 carry the same signal
 * with a stable phase, about 1/sqrt(nwin) for independent noise. */
typedef struct {
    int      is_m;
    uint32_t n, offc, pk0, pk1, flags;
    int32_t  cal_q28;                 /* mean phase after CAL (gives the DOA)   */
    int32_t  raw_q28;                 /* mean phase as received                 */
    int32_t  alpha_q28;               /* CAL rotation                           */
    int32_t  coh_q15;
    int32_t  psi_q24;                 /* mean |psi| (M-ESPRIT)                  */
} doa_phase_t;

static int32_t collect_phase(uint32_t nwin, doa_phase_t *o)
{
    uint32_t caps = rd(DOA_REG_CAPS);
    uint32_t seq_reg, phi_reg, tol, last, t, q, pk;
    int64_t  sc = 0, ss = 0, psum = 0;

    o->is_m = (caps & 2U) ? 1 : 0;
    o->n = 0; o->offc = 0; o->pk0 = 0; o->pk1 = 0; o->flags = 0;
    o->cal_q28 = 0; o->raw_q28 = 0; o->coh_q15 = 0; o->psi_q24 = 0;
    o->alpha_q28 = cal_alpha_q28(rd(DOA_REG_CAL));
    if (!(rd(DOA_REG_CTRL) & DOA_CTRL_ENABLE)) { return DOA_ERR_NO_INPUT; }
    seq_reg = o->is_m ? DOA_REG_M_SEQ : DOA_REG_I_SEQ;
    phi_reg = o->is_m ? DOA_REG_M_PHI : DOA_REG_I_PHI;
    tol  = rd(DOA_REG_UCTOL);
    last = rd(seq_reg);
    for (t = 0; t < 5U * DOA_TIMEOUT_MS && o->n < nwin; t++) {
        q = rd(seq_reg);
        if (q != last) {
            int32_t phi = (int32_t)rd(phi_reg);
            last = q;
            sc += gnss_doa_cos_q15(phi);
            ss += gnss_doa_sin_q15(phi);
            if (o->is_m) {
                int32_t p = (int32_t)rd(DOA_REG_M_PSIMAG);
                psum += p;
                if ((uint32_t)iabs32(p - (1 << 24)) > tol) { o->offc++; }
            }
            pk = rd(DOA_REG_PEAK);
            if ((pk & 0xFFFFU) > o->pk0) { o->pk0 = pk & 0xFFFFU; }
            if ((pk >> 16)     > o->pk1) { o->pk1 = pk >> 16; }
            o->n++;
        } else {
            no_os_udelay(200);
        }
    }
    if (o->n == 0U) { return DOA_ERR_TIMEOUT; }
    o->flags = (rd(DOA_REG_STATUS) >> (o->is_m ? 12 : 8)) & 0xFU;
    circ_mean(sc, ss, o->n, &o->cal_q28, &o->coh_q15);
    o->raw_q28 = wrap_q28((int64_t)o->cal_q28 - o->alpha_q28);
    o->psi_q24 = (int32_t)(psum / (int64_t)o->n);
    return DOA_OK;
}

static int32_t phase_verdict(const doa_phase_t *p)
{
    if (p->n == 0U)                     { return DOA_VERDICT_NO_DATA; }
    if (p->flags & 1U)                  { return DOA_VERDICT_NO_SIGNAL; }
    if (p->coh_q15 < DOA_COH_NOISE_Q15) { return DOA_VERDICT_NOISE; }
    if (p->coh_q15 >= DOA_COH_SOURCE_Q15 && (!p->is_m || 4U * p->offc <= p->n)) {
        return DOA_VERDICT_SOURCE;
    }
    return DOA_VERDICT_UNCERTAIN;
}

static const char *verdict_word(int32_t v)
{
    switch (v) {
    case DOA_VERDICT_SOURCE:    return "JAMMER:YES";
    case DOA_VERDICT_NOISE:     return "JAMMER:NO";
    case DOA_VERDICT_UNCERTAIN: return "JAMMER:??";
    case DOA_VERDICT_NO_SIGNAL: return "NO-SIGNAL";
    default:                    return "NO-DATA";
    }
}

/* --------------------------------------------------------------------------
 * expected DOA and "measured - expected" differences
 *
 * doa_expect=<deg> sets the bearing you KNOW the source has (cable-delay
 * table, turntable position, ...). The default 0 deg is the splitter /
 * broadside case, which is also what the calibration aims for.
 * -------------------------------------------------------------------------- */
static int32_t expect_q16  = 0;
static int     expect_user = 0;

/* phase (Q28 rad) that a source at theta (deg Q16) gives for the current K:
 * theta = asin(-k phi)  ->  phi = -sin(theta) / k */
static int32_t phase_for_deg(int32_t deg_q16)
{
    int32_t k   = (int32_t)rd(DOA_REG_K);
    int32_t ang = (int32_t)(((int64_t)deg_q16 * DOA_DEG2RAD_Q28) >> 16);
    int64_t s   = gnss_doa_sin_q15(ang);
    if (k <= 0) { return 0; }
    return wrap_q28(-(s << 43) / k);
}

void gnss_doa_set_expect(double deg)
{
    if (deg >  89.0) { deg =  89.0; }
    if (deg < -89.0) { deg = -89.0; }
    expect_q16  = (int32_t)(deg * 65536.0 + ((deg < 0) ? -0.5 : 0.5));
    expect_user = 1;
    printf("DOA: expected DOA set to ");
    put_deg(expect_q16);
    printf(" deg -> expected phase RX2-RX1 ");
    put_deg(q28_to_degq16(phase_for_deg(expect_q16)));
    printf(" deg (current K). doa_phase? / doa_watch= / doa? now show measured - expected.\n");
}

static void print_jammer_line(int32_t v, int32_t deg_q16, int32_t coh_q15)
{
    switch (v) {
    case DOA_VERDICT_SOURCE:
        printf("DOA:   JAMMER DETECTED: YES - strong coherent source at ");
        put_deg(deg_q16);
        printf(" deg (coherence ");
        put_fixed(coh_q15, 15, 3);
        printf(")\n");
        break;
    case DOA_VERDICT_NOISE:
        printf("DOA:   JAMMER DETECTED: NO  - noise only (coherence ");
        put_fixed(coh_q15, 15, 3);
        printf("); satellites are below the noise\n");
        break;
    case DOA_VERDICT_UNCERTAIN:
        printf("DOA:   JAMMER DETECTED: UNCERTAIN - partly coherent (coherence ");
        put_fixed(coh_q15, 15, 3);
        printf("), see doa_phase?\n");
        break;
    case DOA_VERDICT_NO_SIGNAL:
        printf("DOA:   JAMMER DETECTED: NO  - no signal (all-zero input)\n");
        break;
    default:
        printf("DOA:   JAMMER DETECTED: no data (doa=1? RX running?)\n");
        break;
    }
}

static void print_expect(int32_t meas_deg_q16, int32_t meas_phase_q28, int32_t v)
{
    int32_t exp_ph = phase_for_deg(expect_q16);
    int32_t d_deg  = meas_deg_q16 - expect_q16;
    int32_t d_ph   = q28_to_degq16(wrap_q28((int64_t)meas_phase_q28 - exp_ph));
    printf("DOA:   expected       : DOA ");
    put_deg(expect_q16);
    printf(" deg  phase ");
    put_deg(q28_to_degq16(exp_ph));
    printf(" deg   %s\n", expect_user ? "(your known bearing, doa_expect)" : "(default 0 = splitter/broadside; set doa_expect=<deg>)");
    if (v != DOA_VERDICT_SOURCE && v != DOA_VERDICT_UNCERTAIN) {
        printf("DOA:   difference     : n/a - no coherent source, the measured angle is noise\n");
        return;
    }
    printf("DOA:   difference     : DOA ");
    put_deg(d_deg);
    printf(" deg  phase ");
    put_deg(d_ph);
    printf(" deg   (measured - expected)  %s\n",
           (iabs32(d_deg) <= 65536) ? "-> within 1 deg" : "-> OFF by more than 1 deg");
}

/* --------------------------------------------------------------------------
 * RX1/RX2 phase calibration
 *
 * Feed the SAME signal into RX1 and RX2 (signal generator, or a jammer /
 * test source, through a 2-way splitter with equal-length cables). The true
 * inter-element phase is then 0, so whatever phase the estimator reports is
 * the board + cable offset. It is averaged as a unit vector and the opposite
 * rotation is written to DOA_CAL.
 *
 * The coherence of those 64 windows is checked first: below 0.90 RX1 and RX2
 * are NOT seeing the same signal (nothing connected, sky antennas with only
 * satellites, one cable missing) and the "offset" would be random. The
 * calibration is then refused and the previous DOA_CAL is put back.
 * -------------------------------------------------------------------------- */
int32_t gnss_doa_calibrate(void)
{
    uint32_t caps = rd(DOA_REG_CAPS);
    uint32_t sv_ctrl = rd(DOA_REG_CTRL), sv_cal = rd(DOA_REG_CAL);
    int32_t  c, s, q16;
    doa_phase_t p;
    doa_state_t st;

    printf("DOA: calibrating RX1/RX2 phase. RX1 and RX2 must see the SAME signal\n"
           "     (splitter, equal cables). Collecting 64 windows ...\n");
    wr(DOA_REG_CTRL, 0);
    wr(DOA_REG_CAL, 0x00007FFFU);
    wr(DOA_REG_CTRL, (sv_ctrl & ~(DOA_CTRL_SRC_TEST)) | DOA_CTRL_ENABLE);
    (void)collect_phase(64, &p);
    if (p.n < 8U) {
        wr(DOA_REG_CAL, sv_cal);
        printf("DOA: calibration FAILED - only %lu results (is RX running? doa? to check).\n"
               "     Previous calibration kept.\n", (unsigned long)p.n);
        return DOA_ERR_NO_INPUT;
    }
    printf("DOA:   coherence ");
    put_fixed(p.coh_q15, 15, 3);
    printf("  RX1 peak %lu (", (unsigned long)p.pk0); put_x10(dbfs_x10(p.pk0));
    printf(" dBFS)  RX2 peak %lu (", (unsigned long)p.pk1); put_x10(dbfs_x10(p.pk1));
    printf(" dBFS)\n");
    if (p.coh_q15 < DOA_COH_SOURCE_Q15) {
        wr(DOA_REG_CAL, sv_cal);
        printf("DOA: calibration REFUSED - coherence below 0.900: RX1 and RX2 do not carry\n"
               "     the same signal (noise only? sky antennas see satellites BELOW the noise\n"
               "     floor, which cannot be used). Feed one strong signal through a splitter\n"
               "     and repeat. Previous calibration kept.\n");
        return DOA_ERR_INCOHERENT;
    }
    c = gnss_doa_cos_q15(-p.cal_q28);
    s = gnss_doa_sin_q15(-p.cal_q28);
    wr(DOA_REG_CAL, ((uint32_t)(uint16_t)s << 16) | (uint32_t)(uint16_t)c);
    q16 = q28_to_degq16(p.cal_q28);
    cal_info.valid      = 1;
    cal_info.reg        = rd(DOA_REG_CAL);
    cal_info.offset_q16 = q16;
    cal_info.coh_q15    = p.coh_q15;
    cal_info.n          = p.n;
    cal_info.check_valid = 0;
    printf("DOA: measured RX2-RX1 offset ");
    put_deg(q16);
    printf(" deg over %lu windows\n", (unsigned long)p.n);
    printf("DOA:   = difference from the expected 0.000 deg (same signal, equal cables)\n");
    printf("DOA:   ");
    put_shift(p.cal_q28);
    printf("\n");
    printf("DOA:   -> DOA_CAL = 0x%08lx (cos %ld, sin %ld) removes it\n",
           (unsigned long)rd(DOA_REG_CAL), (long)c, (long)s);
    if (wait_avg(2) == DOA_OK && collect_phase(32, &p) == DOA_OK) {
        int32_t dq;
        gnss_doa_get_state(&st);
        dq = (caps & 2U) ? st.m.avg_deg_q16 : st.i.avg_deg_q16;
        cal_info.check_valid     = 1;
        cal_info.check_phase_q16 = q28_to_degq16(p.cal_q28);
        cal_info.check_doa_q16   = dq;
        printf("DOA: calibration check (same signal, expected 0.000 deg):\n");
        printf("DOA:   phase  measured ");
        put_deg(cal_info.check_phase_q16);
        printf(" deg   difference ");
        put_deg(cal_info.check_phase_q16);
        printf(" deg\n");
        printf("DOA:   DOA    measured ");
        put_deg(dq);
        printf(" deg   difference ");
        put_deg(dq);
        printf(" deg   -> %s\n", (iabs32(dq) <= 32768) ? "GOOD (within 0.5 deg)"
                                                          : "CHECK (more than 0.5 deg off)");
        printf("DOA: calibration is lost at power-off.\n");
    }
    return DOA_OK;
}

void gnss_doa_cal_reset(void)
{
    wr(DOA_REG_CAL, 0x00007FFFU);
    cal_info.valid = 0;
    cal_info.check_valid = 0;
    printf("DOA: calibration cleared (identity)\n");
}

/* "doa_cal?" -- and the calibration line of doa_phase? */
static void print_cal_line(void)
{
    uint32_t cal   = rd(DOA_REG_CAL);
    int32_t  alpha = cal_alpha_q28(cal);
    if (cal == 0x00007FFFU) {
        printf("DOA:   calibration    : NONE (identity). doa_cal=1 with one signal split to\n"
               "                        RX1 + RX2 removes the board/cable phase offset.\n");
    } else if (cal_info.valid && cal_info.reg == cal) {
        printf("DOA:   calibration    : ACTIVE  offset removed ");
        put_deg(cal_info.offset_q16);
        printf(" deg  (doa_cal=1, %lu windows, coherence ", (unsigned long)cal_info.n);
        put_fixed(cal_info.coh_q15, 15, 3);
        printf(")\n");
        printf("DOA:                    ");
        put_shift(-alpha);
        printf("\n");
        if (cal_info.check_valid) {
            printf("DOA:   calibration chk: right after doa_cal=1 the common signal read phase ");
            put_deg(cal_info.check_phase_q16);
            printf(" deg,\n                        DOA ");
            put_deg(cal_info.check_doa_q16);
            printf(" deg (difference from the expected 0.000 deg)\n");
        }
    } else {
        printf("DOA:   calibration    : ACTIVE  offset removed ");
        put_deg(q28_to_degq16(-alpha));
        printf(" deg  (DOA_CAL = 0x%08lx, written directly)\n", (unsigned long)cal);
        printf("DOA:                    ");
        put_shift(-alpha);
        printf("\n");
    }
}

void gnss_doa_print_cal(void)
{
    printf("DOA: ---- RX1/RX2 phase calibration ----\n");
    print_cal_line();
}

/* "doa_phase?" -- phase, calibration, level, coherence and a jammer verdict */
int32_t gnss_doa_print_phase(uint32_t nwin)
{
    doa_phase_t p;
    doa_state_t st;
    int32_t     v, r, ratio_x10;
    uint32_t    navg;

    if (nwin < 4U)   { nwin = 4U; }
    if (nwin > 256U) { nwin = 256U; }
    r = collect_phase(nwin, &p);
    gnss_doa_get_state(&st);
    v = phase_verdict(&p);
    navg = 1UL << (((st.ctrl >> 16) & 0xFU) > 10 ? 10 : ((st.ctrl >> 16) & 0xFU));

    printf("DOA: ==== PHASE / CALIBRATION / JAMMER CHECK  (%s, %lu windows of %lu snapshots) ====\n",
           p.is_m ? "M-ESPRIT" : "I-UESPRIT", (unsigned long)p.n,
           (unsigned long)(1UL << ((st.ctrl >> 8) & 0x1FU)));
    if (r == DOA_ERR_NO_INPUT) {
        printf("DOA:   DOA is disabled. Type  doa=1  first.\n");
        return DOA_VERDICT_NO_DATA;
    }
    if (r != DOA_OK) {
        printf("DOA:   no estimator results within the timeout - is RX running?\n");
        return DOA_VERDICT_NO_DATA;
    }
    print_cal_line();
    printf("DOA:   phase RX2-RX1  : raw        ");
    put_deg(q28_to_degq16(p.raw_q28));
    printf(" deg   as received (includes the board/cable offset)\n");
    printf("DOA:                    calibrated ");
    put_deg(q28_to_degq16(p.cal_q28));
    printf(" deg   after CAL; this phase gives the DOA\n");
    printf("DOA:   coherence      : ");
    put_fixed(p.coh_q15, 15, 3);
    printf("   (1.000 = same signal on RX1 and RX2, ~0 = independent noise)\n");
    if (p.is_m) {
        printf("DOA:   |psi|          : ");
        put_q24(p.psi_q24);
        printf("   = RX2/RX1 amplitude ratio for one source; off-circle %lu/%lu windows\n",
               (unsigned long)p.offc, (unsigned long)p.n);
    }
    printf("DOA:   level          : RX1 peak %5lu (", (unsigned long)p.pk0); put_x10(dbfs_x10(p.pk0));
    printf(" dBFS)   RX2 peak %5lu (", (unsigned long)p.pk1); put_x10(dbfs_x10(p.pk1));
    printf(" dBFS)\n");
    printf("DOA:   DOA            : mean ");
    put_deg(p.is_m ? st.m.avg_deg_q16 : st.i.avg_deg_q16);
    printf(" deg  std ");
    put_mdeg(p.is_m ? st.m.std_mdeg : st.i.std_mdeg);
    printf(" deg  (average of %lu windows)\n", (unsigned long)navg);
    print_expect(p.is_m ? st.m.avg_deg_q16 : st.i.avg_deg_q16, p.cal_q28, v);
    print_jammer_line(v, p.is_m ? st.m.avg_deg_q16 : st.i.avg_deg_q16, p.coh_q15);

    switch (v) {
    case DOA_VERDICT_SOURCE:
        printf("DOA:   VERDICT        : >>> JAMMER / STRONG COHERENT SOURCE at ");
        put_deg(p.is_m ? st.m.avg_deg_q16 : st.i.avg_deg_q16);
        printf(" deg <<<\n");
        printf("DOA:                    One signal well above the noise reaches both elements with a\n"
               "                        stable phase. GNSS satellites are ~20 dB BELOW the noise and\n"
               "                        never look like this, so it is an interferer (or your bench\n"
               "                        signal generator).%s\n",
               (cal_info.valid || rd(DOA_REG_CAL) != 0x00007FFFU) ? "" :
               "  NOT calibrated: the angle includes the board offset.");
        break;
    case DOA_VERDICT_NOISE:
        printf("DOA:   VERDICT        : NO JAMMER - RX1 and RX2 see independent noise only.\n"
               "                        Satellites may be present but are ~20 dB below the noise\n"
               "                        floor; ESPRIT on raw samples cannot locate them (it locates\n"
               "                        strong interferers). The DOA angle above is meaningless.\n");
        break;
    case DOA_VERDICT_UNCERTAIN:
        if (p.is_m && p.coh_q15 >= DOA_COH_SOURCE_Q15) {
            /* stable phase, |psi| off the unit circle: for one source
             * |psi| = |x1| / |x0|, so this is usually an RX gain mismatch */
            ratio_x10 = dbfs_x10((uint32_t)((((int64_t)p.psi_q24) * 32767) >> 24));
            printf("DOA:   VERDICT        : UNCERTAIN - stable phase (a source is present) but |psi| is\n"
                   "                        off 1: RX2 is ");
            put_x10(ratio_x10);
            printf(" dB vs RX1. Usually unequal RX gains (else two\n"
                   "                        sources / multipath). Set equal manual gains\n"
                   "                        (rx1_gc_mode=0 rx2_gc_mode=0, same rx1/rx2_rf_gain), re-check.\n");
        } else {
            printf("DOA:   VERDICT        : UNCERTAIN - partly coherent: weak source near the noise,\n"
                   "                        two sources / multipath, or a moving source.\n");
        }
        break;
    case DOA_VERDICT_NO_SIGNAL:
        printf("DOA:   VERDICT        : NO SIGNAL - the estimator input is all zero.\n");
        break;
    default:
        printf("DOA:   VERDICT        : no data\n");
        break;
    }
    if (p.pk0 >= DOA_PEAK_CLIP || p.pk1 >= DOA_PEAK_CLIP) {
        printf("DOA:   ! CLIPPING: a peak is near full scale; lower the RX gain or the source level.\n");
    }
    if (p.flags & 2U) {
        printf("DOA:   ! ambiguous: |k*phi| > 1 - check doa_spacing_mm (angle clipped to +/-90 deg).\n");
    }
    return v;
}

/* "doa_watch=N" -- one line per 16 windows */
int32_t gnss_doa_watch(uint32_t n)
{
    doa_phase_t p;
    doa_state_t st;
    uint32_t    k;
    int32_t     v;
    if (n == 0U)    { n = 10U; }
    if (n > 1000U)  { n = 1000U; }
    printf("DOA: watching %lu x 16 windows  (raw / calibrated phase RX2-RX1, coherence, level,\n"
           "     DOA, err = DOA - expected ", (unsigned long)n);
    put_deg(expect_q16);
    printf(" deg, jammer detected)\n");
    print_cal_line();
    for (k = 0; k < n; k++) {
        if (collect_phase(16, &p) != DOA_OK) {
            printf("DOA: no estimator results - is DOA enabled (doa=1) and RX running?\n");
            return DOA_ERR_TIMEOUT;
        }
        gnss_doa_get_state(&st);
        printf("DOA: #%-4lu raw ", (unsigned long)(k + 1));
        put_x10(degq16_x10(q28_to_degq16(p.raw_q28)));
        printf("  cal ");
        put_x10(degq16_x10(q28_to_degq16(p.cal_q28)));
        printf(" deg  coh ");
        put_fixed(p.coh_q15, 15, 2);
        if (p.is_m) { printf("  |psi| "); put_fixed(p.psi_q24, 24, 2); }
        printf("  RX1 "); put_x10(dbfs_x10(p.pk0));
        printf(" RX2 "); put_x10(dbfs_x10(p.pk1));
        printf(" dBFS  DOA ");
        put_deg(p.is_m ? st.m.avg_deg_q16 : st.i.avg_deg_q16);
        printf(" err");
        v = phase_verdict(&p);
        if (v == DOA_VERDICT_SOURCE || v == DOA_VERDICT_UNCERTAIN) {
            put_deg((p.is_m ? st.m.avg_deg_q16 : st.i.avg_deg_q16) - expect_q16);
        } else {
            printf("    ---");
        }
        printf("  %s\n", verdict_word(v));
    }
    return DOA_OK;
}

/* the jammer and expected/difference lines of doa? (16 windows) */
static void quick_jammer(void)
{
    doa_phase_t p;
    doa_state_t st;
    int32_t     deg;
    if (collect_phase(16, &p) != DOA_OK) {
        print_jammer_line(DOA_VERDICT_NO_DATA, 0, 0);
        return;
    }
    gnss_doa_get_state(&st);
    deg = p.is_m ? st.m.avg_deg_q16 : st.i.avg_deg_q16;
    print_jammer_line(phase_verdict(&p), deg, p.coh_q15);
    print_expect(deg, p.cal_q28, phase_verdict(&p));
}

/* --------------------------------------------------------------------------
 * boot
 * -------------------------------------------------------------------------- */
int32_t gnss_doa_boot_check(void)
{
    int32_t r;
    printf("\n\r=== ESPRIT DOA monitor (gnss_passthrough v1.2) ===\n\r");
    if (gnss_doa_probe() != DOA_OK) { return DOA_ERR_ID; }
    gnss_doa_defaults();
    r = gnss_doa_selftest(0);
    gnss_doa_defaults();
    gnss_doa_enable_live(1);
    printf("DOA: type  doa?  for the live angle,  doa_selftest?  to re-run the test,\n"
           "     doa_help?  for every DOA command.\n");
    return r;
}
