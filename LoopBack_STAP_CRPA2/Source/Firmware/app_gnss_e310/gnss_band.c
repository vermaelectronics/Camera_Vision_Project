/******************************************************************************
 *  gnss_band.c -- five preset GNSS bands, the power-on autostart and the
 *  TX -> RX leakage check. See gnss_band.h.
 *
 *  ORDER OF OPERATIONS (gnss_band_apply)
 *    1. check the whole band against the board limits -- nothing is touched
 *       if any value is out of range
 *    2. TX1 attenuation to the maximum (89.75 dB): quiet while retuning
 *    3. DAC muted, loopback buffer off (gnss_passthrough >= 1.2 flushes its
 *       elastic buffer when pass_en = 0)
 *    4. RX + TX LO, verified by readback
 *    5. sample rate (sets the RX and TX clock chains together), verified
 *    6. RX + TX RF bandwidth
 *    7. RX1 + RX2 gain control mode, then gain (manual mode only), verified
 *    8. counters cleared, loopback buffer back on (re-primes from empty) if
 *       it was on, mute restored
 *    9. CRPA: clear the overrun flag; if weights were in use, adapt them to
 *       the new configuration at once
 *   10. TX1 attenuation to the band value
 *  then the AD9361 state is printed (same as gnss_status?).
 *
 *  POWER-ON (gnss_band_autostart, SD-card build only)
 *    band GNSS_DEFAULT_BAND -> TX -> RX leakage check -> CRPA weights ->
 *    anti-jam output ON -> background weight updates every
 *    GNSS_DEPLOY_STAP_PERIOD_MS. No console input is needed.
 *****************************************************************************/
#include "gnss_band.h"
#include "gnss_l1.h"
#include "gnss_passthrough.h"
#include "gnss_stap.h"
#include "ad9361_api.h"
#include "no_os_delay.h"
#include <stdio.h>

extern struct ad9361_rf_phy *ad9361_phy;

/* ==========================================================================
 *  THE FIVE BANDS -- edit here.
 *  TX attenuation 10000..89750 mdB (10..89.75 dB); all bands ship at 20000
 *  (20 dB). RX1 and RX2 always get the same manual gain (CRPA elements).
 *  Centre frequencies: GPS L1 / Galileo E1 1575.420 MHz, BeiDou B1I
 *  1561.098 MHz, GLONASS L1 FDMA centre 1602.000 MHz (channels -7..+6 span
 *  about 1598.06 - 1605.38 MHz).
 * ======================================================================== */
const gnss_band_t gnss_bands[GNSS_BAND_COUNT] = {
    /* name                               LO Hz           fs Hz      RX BW      TX BW      gain ctl        gain  TX att */
    { "GPS L1 / Galileo E1 wide",         1575420000ULL,  30720000U, 18000000U, 18000000U, GNSS_GC_MANUAL,  40,  20000U },
    { "GPS L1 C/A narrow",                1575420000ULL,  15360000U,  4000000U,  4000000U, GNSS_GC_MANUAL,  40,  20000U },
    { "GPS L1 low gain (strong jammer)",  1575420000ULL,  30720000U, 18000000U, 18000000U, GNSS_GC_MANUAL,  25,  20000U },
    { "BeiDou B1I",                       1561098000ULL,  15360000U,  4000000U,  4000000U, GNSS_GC_MANUAL,  40,  20000U },
    { "GLONASS L1",                       1602000000ULL,  15360000U, 10000000U, 10000000U, GNSS_GC_MANUAL,  40,  20000U },
};

static int s_current = 0;

int gnss_band_current(void) { return s_current; }

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

static void print_cdb(const char *label, int32_t v)
{
    int32_t a = (v < 0) ? -v : v;
    printf("%s%s%ld.%02ld dB", label, (v < 0) ? "-" : "+", (long)(a / 100), (long)(a % 100));
}

static int32_t check_band(const gnss_band_t *m)
{
    if (m->lo_hz < GNSS_BAND_MIN_LO_HZ || m->lo_hz > GNSS_BAND_MAX_LO_HZ) {
        printf("gnss_band: LO %llu Hz outside the L band this board is set up for\n",
               (unsigned long long)m->lo_hz);
        return -1;
    }
    if (m->fs_hz < GNSS_BAND_MIN_FS_HZ || m->fs_hz > GNSS_BAND_MAX_FS_HZ) {
        printf("gnss_band: sample rate %lu Hz outside %lu..%lu Hz (the FPGA interface is\n"
               "           timed for at most 30.72 MSPS with both RX channels)\n",
               (unsigned long)m->fs_hz, (unsigned long)GNSS_BAND_MIN_FS_HZ,
               (unsigned long)GNSS_BAND_MAX_FS_HZ);
        return -1;
    }
    if (m->rx_bw_hz < 200000U || m->rx_bw_hz > m->fs_hz ||
        m->tx_bw_hz < 200000U || m->tx_bw_hz > m->fs_hz) {
        printf("gnss_band: bandwidth must be 0.2 MHz .. sample rate\n");
        return -1;
    }
    if (m->gc_mode > GNSS_GC_HYBRID_AGC ||
        (m->gc_mode == GNSS_GC_MANUAL && (m->rx_gain_db < 0 || m->rx_gain_db > 71))) {
        printf("gnss_band: gain control %u / gain %ld dB invalid (manual gain 0..71 dB)\n",
               (unsigned)m->gc_mode, (long)m->rx_gain_db);
        return -1;
    }
    if (m->tx_att_mdb < GNSS_BAND_MIN_ATT_MDB || m->tx_att_mdb > GNSS_BAND_MAX_ATT_MDB) {
        printf("gnss_band: TX attenuation %lu mdB outside %lu..%lu mdB\n",
               (unsigned long)m->tx_att_mdb, (unsigned long)GNSS_BAND_MIN_ATT_MDB,
               (unsigned long)GNSS_BAND_MAX_ATT_MDB);
        return -1;
    }
    return 0;
}

int32_t gnss_band_apply(int band)
{
    struct ad9361_rf_phy *phy = ad9361_phy;
    const gnss_band_t *m;
    uint64_t lo_back = 0;
    uint32_t rx_fs = 0, tx_fs = 0, pt_ctrl;
    int      pass_was_on, mute_was_on, stap_was_on, ch, fail = 0;
    int32_t  ret, g = 0;

    if (!phy) { printf("gnss_band: ad9361_phy is NULL\n"); return -1; }
    if (band < 1 || band > GNSS_BAND_COUNT) {
        printf("gnss_band: choose 1..%d (gnss_band? lists them)\n", GNSS_BAND_COUNT);
        return -1;
    }
    m = &gnss_bands[band - 1];
    if (check_band(m) != 0) { printf("gnss_band: band %d NOT applied\n", band); return -1; }

    printf("gnss_band: applying band %d - %s\n", band, m->name);

    /* 2. quiet while retuning */
    (void)gnss_l1_set_tx_attenuation(phy, GNSS_L1_TX_MAX_ATTEN_MDB);

    /* 3. mute, loopback buffer off: flushes the elastic buffer, so a rate
     *    change cannot leave it stuck full (overflow) or empty (underflow) */
    pt_ctrl     = gnss_pt_read(GNSS_PT_REG_CONTROL);
    pass_was_on = (pt_ctrl & GNSS_PT_CTRL_PASS_EN) ? 1 : 0;
    mute_was_on = (pt_ctrl & GNSS_PT_CTRL_MUTE) ? 1 : 0;
    stap_was_on = gnss_stap_get_enable();
    gnss_pt_set_mute(1);
    if (pass_was_on) gnss_pt_set_passthrough(0);

    /* 4. centre frequency, RX and TX identical */
    ret  = ad9361_set_rx_lo_freq(phy, m->lo_hz);
    ret |= ad9361_set_tx_lo_freq(phy, m->lo_hz);
    if (ret != 0) { printf("gnss_band: LO set failed (%ld)\n", (long)ret); fail = 1; }
    if (ad9361_get_rx_lo_freq(phy, &lo_back) != 0 || absdiff64(lo_back, m->lo_hz) > 100U) {
        printf("gnss_band: RX LO readback %llu Hz, wanted %llu Hz\n",
               (unsigned long long)lo_back, (unsigned long long)m->lo_hz);
        fail = 1;
    }
    if (ad9361_get_tx_lo_freq(phy, &lo_back) != 0 || absdiff64(lo_back, m->lo_hz) > 100U) {
        printf("gnss_band: TX LO readback %llu Hz, wanted %llu Hz\n",
               (unsigned long long)lo_back, (unsigned long long)m->lo_hz);
        fail = 1;
    }

    /* 5. sample rate: one clock chain for RX and TX */
    ret = ad9361_set_rx_sampling_freq(phy, m->fs_hz);
    if (ret != 0) { printf("gnss_band: sample rate set failed (%ld)\n", (long)ret); fail = 1; }
    (void)ad9361_get_rx_sampling_freq(phy, &rx_fs);
    (void)ad9361_get_tx_sampling_freq(phy, &tx_fs);
    if (rx_fs != tx_fs) {
        printf("gnss_band: RX %lu Hz and TX %lu Hz sample rates differ; setting TX\n",
               (unsigned long)rx_fs, (unsigned long)tx_fs);
        (void)ad9361_set_tx_sampling_freq(phy, m->fs_hz);
        (void)ad9361_get_tx_sampling_freq(phy, &tx_fs);
        if (rx_fs != tx_fs) fail = 1;
    }

    /* 6. bandwidths */
    if (ad9361_set_rx_rf_bandwidth(phy, m->rx_bw_hz) != 0) { printf("gnss_band: RX bandwidth failed\n"); fail = 1; }
    if (ad9361_set_tx_rf_bandwidth(phy, m->tx_bw_hz) != 0) { printf("gnss_band: TX bandwidth failed\n"); fail = 1; }

    /* 7. gain: the same on both CRPA elements */
    for (ch = 0; ch < 2; ch++) {
        if (ad9361_set_rx_gain_control_mode(phy, (uint8_t)ch, m->gc_mode) != 0) {
            printf("gnss_band: RX%d gain control mode failed\n", ch + 1); fail = 1;
        }
        if (m->gc_mode == GNSS_GC_MANUAL) {
            if (ad9361_set_rx_rf_gain(phy, (uint8_t)ch, m->rx_gain_db) != 0 ||
                ad9361_get_rx_rf_gain(phy, (uint8_t)ch, &g) != 0 || g != m->rx_gain_db) {
                printf("gnss_band: RX%d gain readback %ld dB, wanted %ld dB\n",
                       ch + 1, (long)g, (long)m->rx_gain_db);
                fail = 1;
            }
        }
    }

    /* 8. loopback back on, re-primed from empty; mute as it was */
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
                printf("gnss_band: CRPA weights re-adapted for the new configuration\n");
            } else {
                (void)gnss_stap_set_enable(0);
                printf("gnss_band: CRPA re-adaptation failed; output set to BYPASS\n");
            }
        }
    }

    /* 10. band attenuation last */
    (void)gnss_l1_set_tx_attenuation(phy, m->tx_att_mdb);

    s_current = band;
    gnss_l1_print_status(phy);
    if (fail) {
        printf("gnss_band: band %d applied WITH ERRORS - check the lines above\n", band);
        return -1;
    }
    printf("gnss_band: band %d OK\n", band);
    if (!pass_was_on)
        printf("gnss_band: TX is silent. gnss_tx=1 starts at 89750 mdB (safety); then\n"
               "           gnss_band=%d again, or tx1_attenuation=%lu, for this band's level.\n",
               band, (unsigned long)m->tx_att_mdb);
    return 0;
}

/* ==========================================================================
 *  TX -> RX leakage check
 *  The E310 retransmits on the SAME frequency it receives. Some of TX1 always
 *  couples back into RX1/RX2 on the board. At high TX power (low attenuation)
 *  this leakage can rise above the receiver noise: the loopback then feeds
 *  its own output back into itself, and the real satellites are buried. The
 *  check compares the input power with the DAC muted and running.
 * ======================================================================== */
int32_t gnss_band_loop_check(int verbose)
{
    uint32_t ctrl, st, att = 0;
    int      tx_on;
    int32_t  off0, off1, on0, on1, rise0, rise1, rise;

    if (!gnss_stap_st.present) {
        if (verbose) printf("gnss_loop: needs the CRPA stage (power measurement); not available\n");
        return -1;
    }
    st    = gnss_pt_read(GNSS_PT_REG_STATUS);
    tx_on = (st & GNSS_PT_ST_DAC_EN_I0) && (st & GNSS_PT_ST_DAC_EN_Q0) &&
            (st & GNSS_PT_ST_PASS_EN_SYNCED);
    if (!tx_on) {
        if (verbose) printf("gnss_loop: TX is not running (gnss_tx=1 first); nothing to measure\n");
        return -1;
    }
    ctrl = gnss_pt_read(GNSS_PT_REG_CONTROL);

    gnss_pt_set_mute(1);                      /* TX1 carries zeros */
    no_os_mdelay(5);
    if (gnss_stap_update() != GNSS_STAP_OK) {
        if (!(ctrl & GNSS_PT_CTRL_MUTE)) gnss_pt_set_mute(0);
        return -1;
    }
    off0 = gnss_stap_st.in_power_cdb[0];
    off1 = gnss_stap_st.in_power_cdb[1];

    gnss_pt_set_mute(0);                      /* TX1 running */
    no_os_mdelay(5);
    if (gnss_stap_update() != GNSS_STAP_OK) {
        if (ctrl & GNSS_PT_CTRL_MUTE) gnss_pt_set_mute(1);
        return -1;
    }
    on0 = gnss_stap_st.in_power_cdb[0];
    on1 = gnss_stap_st.in_power_cdb[1];
    if (ctrl & GNSS_PT_CTRL_MUTE) gnss_pt_set_mute(1);

    rise0 = on0 - off0;
    rise1 = on1 - off1;
    rise  = (rise0 > rise1) ? rise0 : rise1;
    (void)ad9361_get_tx_attenuation(ad9361_phy, 0, &att);

    if (verbose) {
        printf("---- TX -> RX leakage check (TX1 attenuation %lu mdB) ----\n", (unsigned long)att);
        printf("  RX1 input: TX muted ");
        print_cdb("", off0); printf("FS, TX on "); print_cdb("", on0); printf("FS, rise ");
        print_cdb("", rise0); printf("\n");
        printf("  RX2 input: TX muted ");
        print_cdb("", off1); printf("FS, TX on "); print_cdb("", on1); printf("FS, rise ");
        print_cdb("", rise1); printf("\n");
        if (rise > 300) {
            printf("  WARNING: TX1 is leaking back into the receivers (%ld.%02ld dB above the\n"
                   "           muted level). The loopback is feeding its own output back into\n"
                   "           itself, which buries the satellites. Each +10000 mdB of\n"
                   "           tx1_attenuation lowers the leakage by 10 dB, e.g.\n"
                   "             tx1_attenuation=%lu\n"
                   "           then run gnss_loop? again until the rise is below 1 dB.\n",
                   (long)(rise / 100), (long)(rise % 100),
                   (unsigned long)((att + 10000U > GNSS_BAND_MAX_ATT_MDB) ? GNSS_BAND_MAX_ATT_MDB : att + 10000U));
        } else {
            printf("  OK: no significant TX -> RX leakage at this attenuation\n");
        }
        printf("---------------------------------------------------------\n");
    }
    return rise;
}

/* ==========================================================================
 *  Power-on: everything the SD card needs, without the console
 * ======================================================================== */
int32_t gnss_band_autostart(void)
{
    int band = GNSS_DEFAULT_BAND;

    if (band < 1 || band > GNSS_BAND_COUNT) band = 1;
    printf("gnss_auto: power-on band %d - %s\n", band, gnss_bands[band - 1].name);
    if (gnss_band_apply(band) != 0)
        printf("gnss_auto: band %d not applied cleanly; continuing with what was set\n", band);

#ifndef GNSS_DEPLOY_NO_CRPA
    if (!gnss_stap_st.present) {
        printf("gnss_deploy: CRPA stage not present; TX carries raw RX1 (bypass)\n");
        return -1;
    }
    (void)gnss_band_loop_check(1);
    if (gnss_stap_update() == GNSS_STAP_OK && gnss_stap_set_enable(1) == GNSS_STAP_OK) {
        gnss_stap_st.auto_period_ms = GNSS_DEPLOY_STAP_PERIOD_MS;
        printf("gnss_deploy: CRPA anti-jam ON (power inversion, adapting every %lu ms)\n",
               (unsigned long)GNSS_DEPLOY_STAP_PERIOD_MS);
        gnss_stap_print_state();
        return 0;
    }
    printf("gnss_deploy: CRPA not started; TX carries raw RX1 (bypass)\n");
    return -1;
#else
    (void)gnss_band_loop_check(1);
    printf("gnss_deploy: plain loopback build (GNSS_DEPLOY_NO_CRPA): CRPA left in bypass\n");
    return 0;
#endif
}

/* ======================== console commands ============================== */
void get_gnss_band(double *param, char param_no)
{
    int i;
    (void)param; (void)param_no;
    printf("gnss_band=%d%s   (power-on band %d)\n", s_current,
           s_current ? "" : " (none since boot: boot configuration)", GNSS_DEFAULT_BAND);
    printf("  N  centre MHz   rate MSPS  RX/TX BW MHz  gain            TX att  name\n");
    for (i = 0; i < GNSS_BAND_COUNT; i++) {
        const gnss_band_t *m = &gnss_bands[i];
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
    printf("  select with gnss_band=N; edit the table in gnss_band.c\n");
}

void set_gnss_band(double *param, char param_no)
{
    if (param_no < 1) { printf("gnss_band= needs 1..%d (gnss_band? lists them)\n", GNSS_BAND_COUNT); return; }
    (void)gnss_band_apply((int)param[0]);
}

void get_gnss_loop(double *param, char param_no)
{
    (void)param; (void)param_no;
    (void)gnss_band_loop_check(1);
}
