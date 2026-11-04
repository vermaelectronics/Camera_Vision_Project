/******************************************************************************
 *  gnss_mode.c -- five preset radio modes. See gnss_mode.h.
 *
 *  ORDER OF OPERATIONS (gnss_mode_apply)
 *    1. check the whole mode against the board limits -- nothing is touched
 *       if any value is out of range
 *    2. TX1 attenuation to the maximum (89.75 dB): quiet while retuning
 *    3. DAC muted, loopback buffer off (gnss_passthrough >= 1.2 flushes its
 *       elastic buffer when pass_en = 0)
 *    4. RX + TX LO, verified by readback
 *    5. sample rate (sets the RX and TX clock chains together), verified
 *    6. RX + TX RF bandwidth
 *    7. RX1 + RX2 gain control mode, then gain (manual mode only)
 *    8. counters cleared, loopback buffer back on (re-primes from empty) if
 *       it was on, mute restored
 *    9. CRPA: clear the overrun flag; if weights were in use, adapt them to
 *       the new configuration at once
 *   10. TX1 attenuation to the mode value
 *  then the AD9361 state is printed (same as gnss_status?).
 *****************************************************************************/
#include "gnss_mode.h"
#include "gnss_l1.h"
#include "gnss_passthrough.h"
#include "gnss_stap.h"
#include "ad9361_api.h"
#include "no_os_delay.h"
#include <stdio.h>

extern struct ad9361_rf_phy *ad9361_phy;

/* ==========================================================================
 *  THE FIVE MODES -- edit here.
 *  Centre frequencies: GPS L1 / Galileo E1 1575.420 MHz, BeiDou B1I
 *  1561.098 MHz, GLONASS L1 FDMA centre 1602.000 MHz (channels -7..+6 span
 *  about 1598.06 - 1605.38 MHz).
 * ======================================================================== */
const gnss_mode_t gnss_modes[GNSS_MODE_COUNT] = {
    /* name                               LO Hz           fs Hz      RX BW      TX BW      gain ctl          gain  TX att */
    { "GPS L1 wide (boot default, AGC)",  1575420000ULL,  30720000U, 18000000U, 18000000U, GNSS_GC_SLOW_AGC,   0,  70000U },
    { "GPS L1 C/A narrow, manual gain",   1575420000ULL,  15360000U,  4000000U,  4000000U, GNSS_GC_MANUAL,    50,  70000U },
    { "GPS L1 strong jammer, low gain",   1575420000ULL,  30720000U, 18000000U, 18000000U, GNSS_GC_MANUAL,    30,  70000U },
    { "BeiDou B1I, manual gain",          1561098000ULL,  15360000U,  4000000U,  4000000U, GNSS_GC_MANUAL,    50,  70000U },
    { "GLONASS L1, manual gain",          1602000000ULL,  15360000U, 10000000U, 10000000U, GNSS_GC_MANUAL,    50,  70000U },
};

static int s_current = 0;

int gnss_mode_current(void) { return s_current; }

static const char *gc_name(uint8_t gc)
{
    switch (gc) {
    case GNSS_GC_MANUAL:     return "manual";
    case GNSS_GC_FAST_AGC:   return "fast AGC";
    case GNSS_GC_SLOW_AGC:   return "slow AGC";
    case GNSS_GC_HYBRID_AGC: return "hybrid AGC";
    default:                 return "?";
    }
}

static uint64_t absdiff64(uint64_t a, uint64_t b) { return (a > b) ? (a - b) : (b - a); }

static int32_t check_mode(const gnss_mode_t *m)
{
    if (m->lo_hz < GNSS_MODE_MIN_LO_HZ || m->lo_hz > GNSS_MODE_MAX_LO_HZ) {
        printf("gnss_mode: LO %llu Hz outside the L band this board is set up for\n",
               (unsigned long long)m->lo_hz);
        return -1;
    }
    if (m->fs_hz < GNSS_MODE_MIN_FS_HZ || m->fs_hz > GNSS_MODE_MAX_FS_HZ) {
        printf("gnss_mode: sample rate %lu Hz outside %lu..%lu Hz (the FPGA interface is\n"
               "           timed for at most 30.72 MSPS with both RX channels)\n",
               (unsigned long)m->fs_hz, (unsigned long)GNSS_MODE_MIN_FS_HZ,
               (unsigned long)GNSS_MODE_MAX_FS_HZ);
        return -1;
    }
    if (m->rx_bw_hz < 200000U || m->rx_bw_hz > m->fs_hz ||
        m->tx_bw_hz < 200000U || m->tx_bw_hz > m->fs_hz) {
        printf("gnss_mode: bandwidth must be 0.2 MHz .. sample rate\n");
        return -1;
    }
    if (m->gc_mode > GNSS_GC_HYBRID_AGC ||
        (m->gc_mode == GNSS_GC_MANUAL && (m->rx_gain_db < 0 || m->rx_gain_db > 71))) {
        printf("gnss_mode: gain control %u / gain %ld dB invalid (manual gain 0..71 dB)\n",
               (unsigned)m->gc_mode, (long)m->rx_gain_db);
        return -1;
    }
    if (m->tx_att_mdb < GNSS_MODE_MIN_ATT_MDB || m->tx_att_mdb > GNSS_MODE_MAX_ATT_MDB) {
        printf("gnss_mode: TX attenuation %lu mdB outside %lu..%lu mdB\n",
               (unsigned long)m->tx_att_mdb, (unsigned long)GNSS_MODE_MIN_ATT_MDB,
               (unsigned long)GNSS_MODE_MAX_ATT_MDB);
        return -1;
    }
    return 0;
}

int32_t gnss_mode_apply(int mode)
{
    struct ad9361_rf_phy *phy = ad9361_phy;
    const gnss_mode_t *m;
    uint64_t lo_back = 0;
    uint32_t rx_fs = 0, tx_fs = 0;
    uint32_t pt_ctrl;
    int      pass_was_on, mute_was_on, stap_was_on, ch, fail = 0;
    int32_t  ret, g = 0;

    if (!phy) { printf("gnss_mode: ad9361_phy is NULL\n"); return -1; }
    if (mode < 1 || mode > GNSS_MODE_COUNT) {
        printf("gnss_mode: choose 1..%d (gnss_mode? lists them)\n", GNSS_MODE_COUNT);
        return -1;
    }
    m = &gnss_modes[mode - 1];
    if (check_mode(m) != 0) { printf("gnss_mode: mode %d NOT applied\n", mode); return -1; }

    printf("gnss_mode: applying mode %d - %s\n", mode, m->name);

    /* 2. quiet while retuning */
    (void)gnss_l1_set_tx_attenuation(phy, GNSS_L1_TX_MAX_ATTEN_MDB);

    /* 3. loopback buffer off: flushes the elastic buffer, so a rate change
     *    cannot leave it stuck full (overflow) or empty (underflow) */
    pt_ctrl     = gnss_pt_read(GNSS_PT_REG_CONTROL);
    pass_was_on = (pt_ctrl & GNSS_PT_CTRL_PASS_EN) ? 1 : 0;
    mute_was_on = (pt_ctrl & GNSS_PT_CTRL_MUTE) ? 1 : 0;
    stap_was_on = gnss_stap_get_enable();
    gnss_pt_set_mute(1);                       /* DAC gets zeros while retuning */
    if (pass_was_on) gnss_pt_set_passthrough(0);

    /* 4. centre frequency, RX and TX identical */
    ret  = ad9361_set_rx_lo_freq(phy, m->lo_hz);
    ret |= ad9361_set_tx_lo_freq(phy, m->lo_hz);
    if (ret != 0) { printf("gnss_mode: LO set failed (%ld)\n", (long)ret); fail = 1; }
    if (ad9361_get_rx_lo_freq(phy, &lo_back) != 0 || absdiff64(lo_back, m->lo_hz) > 100U) {
        printf("gnss_mode: RX LO readback %llu Hz, wanted %llu Hz\n",
               (unsigned long long)lo_back, (unsigned long long)m->lo_hz);
        fail = 1;
    }
    if (ad9361_get_tx_lo_freq(phy, &lo_back) != 0 || absdiff64(lo_back, m->lo_hz) > 100U) {
        printf("gnss_mode: TX LO readback %llu Hz, wanted %llu Hz\n",
               (unsigned long long)lo_back, (unsigned long long)m->lo_hz);
        fail = 1;
    }

    /* 5. sample rate: one clock chain for RX and TX */
    ret = ad9361_set_rx_sampling_freq(phy, m->fs_hz);
    if (ret != 0) { printf("gnss_mode: sample rate set failed (%ld)\n", (long)ret); fail = 1; }
    (void)ad9361_get_rx_sampling_freq(phy, &rx_fs);
    (void)ad9361_get_tx_sampling_freq(phy, &tx_fs);
    if (rx_fs != tx_fs) {
        printf("gnss_mode: RX %lu Hz and TX %lu Hz sample rates differ; setting TX\n",
               (unsigned long)rx_fs, (unsigned long)tx_fs);
        (void)ad9361_set_tx_sampling_freq(phy, m->fs_hz);
        (void)ad9361_get_tx_sampling_freq(phy, &tx_fs);
        if (rx_fs != tx_fs) fail = 1;
    }

    /* 6. bandwidths */
    if (ad9361_set_rx_rf_bandwidth(phy, m->rx_bw_hz) != 0) { printf("gnss_mode: RX bandwidth failed\n"); fail = 1; }
    if (ad9361_set_tx_rf_bandwidth(phy, m->tx_bw_hz) != 0) { printf("gnss_mode: TX bandwidth failed\n"); fail = 1; }

    /* 7. gain: the same on both CRPA elements */
    for (ch = 0; ch < 2; ch++) {
        if (ad9361_set_rx_gain_control_mode(phy, (uint8_t)ch, m->gc_mode) != 0) {
            printf("gnss_mode: RX%d gain control mode failed\n", ch + 1); fail = 1;
        }
        if (m->gc_mode == GNSS_GC_MANUAL) {
            if (ad9361_set_rx_rf_gain(phy, (uint8_t)ch, m->rx_gain_db) != 0 ||
                ad9361_get_rx_rf_gain(phy, (uint8_t)ch, &g) != 0 || g != m->rx_gain_db) {
                printf("gnss_mode: RX%d gain readback %ld dB, wanted %ld dB\n",
                       ch + 1, (long)g, (long)m->rx_gain_db);
                fail = 1;
            }
        }
    }

    /* 8. loopback back on, re-primed from empty */
    no_os_mdelay(5);
    gnss_pt_clear_counters();
    if (pass_was_on) gnss_pt_set_passthrough(1);
    no_os_mdelay(1);
    if (!mute_was_on) gnss_pt_set_mute(0);

    /* 9. CRPA: the old weights belong to the old configuration */
    if (gnss_stap_st.present) {
        gnss_stap_clear_overrun();
        if (stap_was_on || gnss_stap_st.auto_period_ms) {
            if (gnss_stap_update() == GNSS_STAP_OK) {
                printf("gnss_mode: CRPA weights re-adapted for the new configuration\n");
            } else {
                (void)gnss_stap_set_enable(0);
                printf("gnss_mode: CRPA re-adaptation failed; output set to BYPASS\n");
            }
        }
    }

    /* 10. mode attenuation last */
    (void)gnss_l1_set_tx_attenuation(phy, m->tx_att_mdb);

    s_current = mode;
    gnss_l1_print_status(phy);
    if (fail) {
        printf("gnss_mode: mode %d applied WITH ERRORS - check the lines above\n", mode);
        return -1;
    }
    printf("gnss_mode: mode %d OK\n", mode);
    if (!pass_was_on)
        printf("gnss_mode: TX is silent. gnss_tx=1 starts at 89750 mdB (safety); then\n"
               "           gnss_mode=%d again, or tx1_attenuation=%lu, for this mode's level.\n",
               mode, (unsigned long)m->tx_att_mdb);
    return 0;
}

/* ======================== console commands ============================== */
void get_gnss_mode(double *param, char param_no)
{
    int i;
    (void)param; (void)param_no;
    printf("gnss_mode=%d%s\n", s_current, s_current ? "" : " (none since boot: boot configuration)");
    printf("  N  centre MHz   rate MSPS  RX/TX BW MHz  gain            TX att  name\n");
    for (i = 0; i < GNSS_MODE_COUNT; i++) {
        const gnss_mode_t *m = &gnss_modes[i];
        char gain[24];
        if (m->gc_mode == GNSS_GC_MANUAL) snprintf(gain, sizeof gain, "manual %2ld dB", (long)m->rx_gain_db);
        else                              snprintf(gain, sizeof gain, "%s", gc_name(m->gc_mode));
        printf(" %c%d  %4lu.%06lu  %2lu.%03lu     %2lu.%01lu / %2lu.%01lu   %-15s %5lu   %s\n",
               (i + 1 == s_current) ? '*' : ' ', i + 1,
               (unsigned long)(m->lo_hz / 1000000ULL), (unsigned long)(m->lo_hz % 1000000ULL),
               (unsigned long)(m->fs_hz / 1000000U), (unsigned long)((m->fs_hz % 1000000U) / 1000U),
               (unsigned long)(m->rx_bw_hz / 1000000U), (unsigned long)((m->rx_bw_hz % 1000000U) / 100000U),
               (unsigned long)(m->tx_bw_hz / 1000000U), (unsigned long)((m->tx_bw_hz % 1000000U) / 100000U),
               gain, (unsigned long)m->tx_att_mdb, m->name);
    }
    printf("  select with gnss_mode=N; edit the table in gnss_mode.c\n");
}

void set_gnss_mode(double *param, char param_no)
{
    if (param_no < 1) { printf("gnss_mode= needs 1..%d (gnss_mode? lists them)\n", GNSS_MODE_COUNT); return; }
    (void)gnss_mode_apply((int)param[0]);
}
