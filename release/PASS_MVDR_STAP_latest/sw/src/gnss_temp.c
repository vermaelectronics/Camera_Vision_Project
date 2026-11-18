/*
 * gnss_temp.c -- MOD12: board temperature monitor and LO lock watch.
 *
 * Why there is no "temperature compensation" in the signal path:
 *   - The SMI-STAP weights are solved again every ~1.5 ms from the data, so
 *     any RX1/RX2 gain or phase drift with temperature is part of what they
 *     learn. Nothing in the null uses a fixed calibration.
 *   - The AGC keeps the noise on target, so the AD9361 gain drift
 *     (a few dB over the full temperature range) is taken out as well.
 *   - RX and TX use the same LO frequency from the same reference, so a
 *     reference-clock drift cancels between down- and up-conversion (the
 *     receiver sees no extra Doppler); the sample clock is common to RX and
 *     TX, so the FIFO stays balanced.
 *   - The AD9361 keeps its RX quadrature and DC tracking running.
 * What does depend on temperature, and is watched here:
 *   - the RF PLLs (VCO calibrated at boot): a lost lock is re-tuned at once;
 *   - the TX quadrature / LO-leakage calibration is done once at boot: a
 *     large change from the boot temperature is reported;
 *   - the Zynq die: the FPGA timing was closed with ~0 ns slack on two
 *     clocks, valid only inside the device's junction-temperature range.
 */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "ad9361_api.h"
#include "ad9361.h"
#include "gnss_temp.h"
#include "gnss_report.h"
#include "gnss_smi.h"
#include "gnss_autostart.h"
#include "xparameters.h"
#include "gnss_quiet.h"
/* MOD13: same as gnss_report.c -- the report line must go through
 * gnss_qprintf, so it lands in the automatic-print spool in order */
#undef  printf
#define printf        gnss_qprintf

#ifndef GNSS_CFG_ZYNQ_TEMP
#define GNSS_CFG_ZYNQ_TEMP 1          /* 0 = do not touch the XADC */
#endif
#if GNSS_CFG_ZYNQ_TEMP && (defined(XPAR_XADCPS_0_DEVICE_ID) || defined(XPAR_XADCPS_0_BASEADDR))
#define HAVE_XADC 1
#include "xadcps.h"
static XAdcPs xadc;
static int    xadc_state = 0;         /* 0 not tried, 1 ok, -1 failed */
#else
#define HAVE_XADC 0
#endif

extern struct ad9361_rf_phy *ad9361_phy;

#define T_STEP_C      10.0    /* report every 10 C of change               */
#define T_CAL_C       25.0    /* boot calibrations: report once beyond this */
#define T_RF_HOT_C    85.0    /* AD9361 hot                                 */
#define T_PL_HOT_C    85.0    /* Zynq commercial grade Tj max               */

static gnss_temp_t tm = { 0, 0, 0, 999, -999, 0, 0, -999, -1, -1, 0 };
static double rf_last_ev = -999.0;
static int    cal_warned = 0, rf_hot = 0, pl_hot = 0;

static int read_rf(double *c)
{
    int32_t mdeg = 0;
    if (!ad9361_phy || ad9361_get_temperature(ad9361_phy, &mdeg) != 0) { return -1; }
    *c = (double)mdeg / 1000.0;
    return (*c > -60.0 && *c < 150.0) ? 0 : -1;
}

static int read_pl(double *c)
{
#if HAVE_XADC
    if (xadc_state == 0) {
        XAdcPs_Config *cfg;
#ifdef SDT
        cfg = XAdcPs_LookupConfig(XPAR_XADCPS_0_BASEADDR);
#else
        cfg = XAdcPs_LookupConfig(XPAR_XADCPS_0_DEVICE_ID);
#endif
        xadc_state = (cfg && XAdcPs_CfgInitialize(&xadc, cfg, cfg->BaseAddress) == XST_SUCCESS) ? 1 : -1;
    }
    if (xadc_state == 1) {
        uint16_t raw = XAdcPs_GetAdcData(&xadc, XADCPS_CH_TEMP);
        *c = (double)XAdcPs_RawToTemperature(raw);
        return (*c > -60.0 && *c < 150.0) ? 0 : -1;
    }
#endif
    (void)c;
    return -1;
}

static int pll_locked(uint32_t reg)
{
    int32_t v;
    if (!ad9361_phy) { return -1; }
    v = ad9361_spi_read(ad9361_phy->spi, reg);
    if (v < 0) { return -1; }
    return (v & VCO_LOCK) ? 1 : 0;
}

void gnss_temp_poll(void)
{
    double c;
    int    rl, tl;

    if (read_rf(&c) == 0) {
        if (!tm.have_rf) { tm.rf_boot_c = c; rf_last_ev = c; }
        tm.have_rf = 1;
        tm.rf_c = c;
        if (c < tm.rf_min_c) { tm.rf_min_c = c; }
        if (c > tm.rf_max_c) { tm.rf_max_c = c; }
        if (fabs(c - rf_last_ev) >= T_STEP_C) {
            gnss_rep_event("TEMP        AD9361 %.0f C (boot %.0f C)", c, tm.rf_boot_c);
            rf_last_ev = c;
        }
        if (!cal_warned && fabs(c - tm.rf_boot_c) >= T_CAL_C) {
            cal_warned = 1;
            gnss_rep_event("TEMP        %+.0f C since the boot calibration: TX1 LO leakage may rise (reboot recalibrates)",
                           c - tm.rf_boot_c);
        }
        if (!rf_hot && c >= T_RF_HOT_C) {
            rf_hot = 1;
            gnss_rep_event("TEMP HIGH   AD9361 %.0f C: improve cooling / airflow", c);
        } else if (rf_hot && c < T_RF_HOT_C - 5.0) {
            rf_hot = 0;
        }
    }
    if (read_pl(&c) == 0) {
        tm.have_pl = 1;
        tm.pl_c = c;
        if (c > tm.pl_max_c) { tm.pl_max_c = c; }
        if (!pl_hot && c >= T_PL_HOT_C) {
            pl_hot = 1;
            gnss_rep_event("TEMP HIGH   Zynq %.0f C: over the junction limit the FPGA timing is not guaranteed", c);
        } else if (pl_hot && c < T_PL_HOT_C - 5.0) {
            pl_hot = 0;
        }
    }

    /* RF PLLs: re-tune (re-runs the VCO calibration) if a lock is lost */
    rl = pll_locked(REG_RX_CP_OVERRANGE_VCO_LOCK);
    tl = pll_locked(REG_TX_CP_OVERRANGE_VCO_LOCK);
    if (rl == 0 || tl == 0) {
        uint64_t f;
        gnss_rep_event("LO UNLOCK   %s%s%s at %.0f C: re-tuning", rl == 0 ? "RX" : "",
                       (rl == 0 && tl == 0) ? "+" : "", tl == 0 ? "TX" : "", tm.rf_c);
        gnss_mon_retune();
        if (rl == 0 && ad9361_get_rx_lo_freq(ad9361_phy, &f) == 0) { (void)ad9361_set_rx_lo_freq(ad9361_phy, f); }
        if (tl == 0 && ad9361_get_tx_lo_freq(ad9361_phy, &f) == 0) { (void)ad9361_set_tx_lo_freq(ad9361_phy, f); }
        gnss_smi_gain_changed_db(0.0);
        tm.relocks++;
        rl = pll_locked(REG_RX_CP_OVERRANGE_VCO_LOCK);
        tl = pll_locked(REG_TX_CP_OVERRANGE_VCO_LOCK);
        gnss_rep_event("LO          %s", (rl != 0 && tl != 0) ? "locked again" : "STILL UNLOCKED: check the reference clock");
    }
    tm.rx_lock = rl;
    tm.tx_lock = tl;
}

void gnss_temp_get(gnss_temp_t *t) { *t = tm; }

void gnss_temp_print_line(const char *pad)
{
    if (tm.have_rf) {
        printf("AD9361 %.1f C (boot %.1f, min %.1f, max %.1f)", tm.rf_c, tm.rf_boot_c, tm.rf_min_c, tm.rf_max_c);
    } else {
        printf("AD9361 n/a");
    }
    if (tm.have_pl) { printf(", Zynq %.1f C (max %.1f)", tm.pl_c, tm.pl_max_c); }
    else            { printf(", Zynq n/a"); }
    printf("\n%sLO lock RX %s / TX %s%s", pad,
           tm.rx_lock < 0 ? "n/a" : (tm.rx_lock ? "yes" : "NO"),
           tm.tx_lock < 0 ? "n/a" : (tm.tx_lock ? "yes" : "NO"),
           tm.relocks ? "" : ", no re-tunes since boot");
    if (tm.relocks) { printf(", %lu re-tune(s) since boot", tm.relocks); }
    printf("  (die temperatures, sensor +/- a few C)\n");
}
