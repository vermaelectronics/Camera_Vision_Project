/******************************************************************************
 *  gnss_l1.c
 *  GPS L1 configuration and status reporting for the ANTSDR E310 V1.
 *  Original work for this project.
 *****************************************************************************/

#include "gnss_l1.h"
#include "gnss_passthrough.h"
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include <stdio.h>

/* The AD9361 synthesiser cannot land exactly on an arbitrary frequency.
 * Accept anything inside GNSS_L1_LO_TOLERANCE_HZ of the target. */
static int gnss_l1_lo_within_tolerance(uint64_t actual)
{
    uint64_t target = GNSS_L1_CENTRE_HZ;
    uint64_t diff = (actual > target) ? (actual - target) : (target - actual);
    return (diff <= GNSS_L1_LO_TOLERANCE_HZ);
}

/* Drives all eight band-select lines to one polarity. Split out so that
 * gnss_l1_band_probe() and gnss_capture_both_polarities() can exercise both
 * without duplicating the list. */
void gnss_l1_set_band_polarity(struct ad9361_rf_phy *phy, uint8_t h, uint8_t l)
{
    no_os_gpio_set_value(phy->gpio_desc_rx1_ctrl_h, h);
    no_os_gpio_set_value(phy->gpio_desc_rx1_ctrl_l, l);
    no_os_gpio_set_value(phy->gpio_desc_tx1_ctrl_h, h);
    no_os_gpio_set_value(phy->gpio_desc_tx1_ctrl_l, l);
    no_os_gpio_set_value(phy->gpio_desc_rx2_ctrl_h, h);
    no_os_gpio_set_value(phy->gpio_desc_rx2_ctrl_l, l);
    no_os_gpio_set_value(phy->gpio_desc_tx2_ctrl_h, h);
    no_os_gpio_set_value(phy->gpio_desc_tx2_ctrl_l, l);
}

/* Both halves of the band selection, set together and read back.
 * See the long commentary in gnss_l1.h for why this is not optional. */
int32_t gnss_l1_select_rf_band(struct ad9361_rf_phy *phy)
{
    int32_t  ret;
    uint32_t rx_port = 0xFFFFFFFFU, tx_port = 0xFFFFFFFFU;

    if (!phy) { return GNSS_L1_ERR_PHY; }

    /* 1. External SPDT switches -> the 5 MHz..3 GHz baluns.
     *    Both RX/TX and both channels, because a two-element CRPA will use
     *    channel 1 as well and a half-configured front end is worse than an
     *    obviously wrong one.
     *
     *    POLARITY IS UNRESOLVED -- see GNSS_L1_BAND_LOW_CTRL_* in gnss_l1.h
     *    and ISSUE-0008. The ES2 schematic's printed truth table contradicts
     *    the vendor firmware, and this board is an ES2. */
    gnss_l1_set_band_polarity(phy,
                           GNSS_L1_BAND_LOW_CTRL_H, GNSS_L1_BAND_LOW_CTRL_L);

    /* 2. AD9361 internal ports must match the balun the switch just selected. */
    ret = ad9361_set_rx_rf_port_input(phy, B_BALANCED);
    if (ret != 0) {
        printf("gnss_l1: ad9361_set_rx_rf_port_input(B_BALANCED) failed (%ld)\n",
               (long)ret);
        return GNSS_L1_ERR_VERIFY;
    }
    ret = ad9361_set_tx_rf_port_output(phy, TXB);
    if (ret != 0) {
        printf("gnss_l1: ad9361_set_tx_rf_port_output(TXB) failed (%ld)\n",
               (long)ret);
        return GNSS_L1_ERR_VERIFY;
    }

    /* 3. Read back. The vendor default is A_BALANCED / TXA, so a silent
     *    failure here would leave the exact fault this function exists to fix. */
    if (ad9361_get_rx_rf_port_input(phy, &rx_port) != 0 ||
        ad9361_get_tx_rf_port_output(phy, &tx_port) != 0) {
        printf("gnss_l1: could not read the RF port selection back\n");
        return GNSS_L1_ERR_VERIFY;
    }
    if (rx_port != B_BALANCED || tx_port != TXB) {
        printf("gnss_l1: RF PORT SELECTION NOT CONFIRMED. Wanted RX=%d TX=%d,\n"
               "         read RX=%lu TX=%lu. At L1 the external switches are on\n"
               "         the 5M-3G baluns (B ports); if the AD9361 is on A the\n"
               "         RF chain is broken in both directions.\n",
               (int)B_BALANCED, (int)TXB,
               (unsigned long)rx_port, (unsigned long)tx_port);
        return GNSS_L1_ERR_VERIFY;
    }

    printf("gnss_l1: RF band = LOW (5 MHz - 3 GHz). AD9361 RX port B_BALANCED,\n"
           "         TX port TXB (confirmed by read-back). External switch\n"
           "         GPIOs driven h=%d l=%d -- POLARITY UNVERIFIED.\n",
           (int)GNSS_L1_BAND_LOW_CTRL_H, (int)GNSS_L1_BAND_LOW_CTRL_L);
    return GNSS_L1_OK;
}

int32_t gnss_l1_band_probe(struct ad9361_rf_phy *phy)
{
    /* index 0 = vendor-firmware polarity, index 1 = ES2 schematic polarity */
    static const uint8_t cand_h[2] = { 0, 1 };
    static const uint8_t cand_l[2] = { 1, 0 };
    static const char   *cand_src[2] = {
        "vendor firmware command.c", "ES2 schematic sheet 13 truth table"
    };
    struct rf_rssi  rssi;
    struct rf_rx_gain gain;
    uint32_t i;

    if (!phy) { return GNSS_L1_ERR_PHY; }

    printf("\n=== Band-select polarity probe ===\n");
    printf("Requires a real signal on RX1 (GNSS antenna or a generator at L1).\n"
           "With nothing connected BOTH rows will look alike and prove nothing.\n"
           "Lower RSSI = stronger signal. Under AGC, lower RX gain also means\n"
           "a stronger signal. The correct polarity is the better row.\n\n");

    for (i = 0; i < 2; i++) {
        gnss_l1_set_band_polarity(phy, cand_h[i], cand_l[i]);
        /* Let the switch settle and the slow-attack AGC re-converge. */
        no_os_mdelay(500);

        printf("  h=%d l=%d  (%s)\n", (int)cand_h[i], (int)cand_l[i], cand_src[i]);
        if (ad9361_get_rx_rf_gain(phy, 0, &gain.gain_db) == 0) {
            printf("      RX1 gain : %ld dB\n", (long)gain.gain_db);
        }
        if (ad9361_get_rx_rssi(phy, 0, &rssi) == 0 && rssi.multiplier > 0) {
            printf("      RX1 RSSI : %lu.%02lu dB\n",
                   (unsigned long)(rssi.symbol / (uint32_t)rssi.multiplier),
                   (unsigned long)(rssi.symbol % (uint32_t)rssi.multiplier));
        } else {
            printf("      RX1 RSSI : read failed\n");
        }
    }

    /* Leave the board in the configured default, whatever the operator decides. */
    gnss_l1_set_band_polarity(phy,
                           GNSS_L1_BAND_LOW_CTRL_H, GNSS_L1_BAND_LOW_CTRL_L);
    printf("\n  restored default h=%d l=%d. If the OTHER row was better, flip\n"
           "  GNSS_L1_BAND_LOW_CTRL_H / _L in gnss_l1.h and record the\n"
           "  measured numbers alongside it.\n",
           (int)GNSS_L1_BAND_LOW_CTRL_H, (int)GNSS_L1_BAND_LOW_CTRL_L);
    printf("=============================================\n");
    return GNSS_L1_OK;
}

int32_t gnss_l1_configure_rx(struct ad9361_rf_phy *phy)
{
    int32_t  ret;
    uint64_t lo_readback = 0;

    if (!phy) { return GNSS_L1_ERR_PHY; }

    printf("\ngnss_l1: configuring for GPS L1 (%llu Hz)\n",
           (unsigned long long)GNSS_L1_CENTRE_HZ);

    /* Band selection FIRST. The LO is meaningless if the signal never reaches
     * the mixer, and this is not inherited correctly from the vendor defaults
     * (ISSUE-0007). */
    ret = gnss_l1_select_rf_band(phy);
    if (ret != GNSS_L1_OK) {
        printf("gnss_l1: RF band selection failed; not proceeding to tune\n");
        return ret;
    }

    /* The E310 RX1/TX1 front ends share the AD9361 synthesisers.  Both are set
     * to L1 so that the retransmit path comes out on the same frequency it went
     * in on, which is what the Phase-1 passthrough proof of concept requires. */
    ret = ad9361_set_rx_lo_freq(phy, GNSS_L1_CENTRE_HZ);
    if (ret != 0) {
        printf("gnss_l1: ad9361_set_rx_lo_freq failed (%ld)\n", (long)ret);
        return GNSS_L1_ERR_LO;
    }
    ret = ad9361_set_tx_lo_freq(phy, GNSS_L1_CENTRE_HZ);
    if (ret != 0) {
        printf("gnss_l1: ad9361_set_tx_lo_freq failed (%ld)\n", (long)ret);
        return GNSS_L1_ERR_LO;
    }

    /* Read back rather than assume the write took effect.
     *
     * The AD9361 RF synthesiser has finite frequency resolution, so the
     * readback is not bit-exact. Measured on hardware at L1: asked for
     * 1575420000 Hz, got 1575419998 Hz -- 2 Hz low. An exact-equality check
     * rejected a perfectly good tune.
     *
     * GNSS_L1_LO_TOLERANCE_HZ is far tighter than anything that matters for
     * GPS L1 acquisition (the C/A code has a ~1 kHz Doppler search bin) while
     * still catching a genuinely wrong LO. */
    ret = ad9361_get_rx_lo_freq(phy, &lo_readback);
    if (ret != 0 || !gnss_l1_lo_within_tolerance(lo_readback)) {
        printf("gnss_l1: RX LO out of tolerance: wanted %llu, got %llu\n",
               (unsigned long long)GNSS_L1_CENTRE_HZ,
               (unsigned long long)lo_readback);
        return GNSS_L1_ERR_VERIFY;
    }
    printf("gnss_l1: RX LO = %llu Hz (%+lld Hz from target)\n",
           (unsigned long long)lo_readback,
           (long long)lo_readback - (long long)GNSS_L1_CENTRE_HZ);

    ret = ad9361_get_tx_lo_freq(phy, &lo_readback);
    if (ret != 0 || !gnss_l1_lo_within_tolerance(lo_readback)) {
        printf("gnss_l1: TX LO out of tolerance: wanted %llu, got %llu\n",
               (unsigned long long)GNSS_L1_CENTRE_HZ,
               (unsigned long long)lo_readback);
        return GNSS_L1_ERR_VERIFY;
    }
    printf("gnss_l1: TX LO = %llu Hz (%+lld Hz from target)\n",
           (unsigned long long)lo_readback,
           (long long)lo_readback - (long long)GNSS_L1_CENTRE_HZ);

    /* Automatic gain control for a live sky signal; the incoming level is not
     * known in advance. */
    ret = ad9361_set_rx_gain_control_mode(phy, 0, RF_GAIN_SLOWATTACK_AGC);
    if (ret != 0) {
        printf("gnss_l1: could not set AGC mode (%ld); continuing with the "
               "existing gain mode\n", (long)ret);
    }

    printf("gnss_l1: LO set and verified on RX and TX\n");
    printf("gnss_l1: sample rate and bandwidth left at the MicroPhase defaults\n"
           "         (%u Hz / %u Hz). These are NOT tuned for L1; see\n"
           "         Source/Config/board_e310_v1.json -> gnss_application.UNRESOLVED\n",
           (unsigned)GNSS_L1_VENDOR_SAMPLE_RATE_HZ,
           (unsigned)GNSS_L1_VENDOR_RF_BANDWIDTH_HZ);

    return GNSS_L1_OK;
}

int32_t gnss_l1_set_tx_attenuation(struct ad9361_rf_phy *phy, uint32_t mdB)
{
    int32_t ret;
    uint32_t back = 0;

    if (!phy) { return GNSS_L1_ERR_PHY; }

    ret = ad9361_set_tx_attenuation(phy, 0, mdB);
    if (ret != 0) {
        printf("gnss_l1: ad9361_set_tx_attenuation failed (%ld)\n", (long)ret);
        return GNSS_L1_ERR_VERIFY;
    }
    ret = ad9361_get_tx_attenuation(phy, 0, &back);
    if (ret != 0) {
        printf("gnss_l1: could not read TX attenuation back\n");
        return GNSS_L1_ERR_VERIFY;
    }
    printf("gnss_l1: TX1 attenuation set to %u mdB (read back %u mdB)\n",
           (unsigned)mdB, (unsigned)back);
    return GNSS_L1_OK;
}

/* ---------------------------------------------------------------------------
 *  DAC data-source control.
 *
 *  The chain that matters, all of it in the pinned vendor RTL:
 *
 *    axi_ad9361_tx.v:239          dac_data_i0 -> i_tx_channel_0.dma_data
 *    axi_ad9361_tx_channel.v:301  4'h2: dac_data_out_int <= dma_data[15:4];
 *    axi_ad9361_tx_channel.v:303  default: dac_data_out_int <= dac_dds_data_s;
 *    axi_ad9361_tx_channel.v:272  dac_enable <= (dac_data_sel == 4'h2);
 *
 *  So dac_enable is a read-back of the source select, and only select 2 lets
 *  gnss_passthrough's output reach the AD9361 at all.
 * ------------------------------------------------------------------------- */
int32_t gnss_l1_set_dac_source(struct ad9361_rf_phy *phy,
                               enum axi_dac_data_sel sel)
{
    static const char *name[] = {
        "DDS", "SED", "DMA", "ZERO", "PN7", "PN15", "PN23", "PN31", "LB", "PNXX"
    };
    uint32_t status;
    int      want, got_i, got_q;

    if (!phy || !phy->tx_dac) {
        printf("gnss_l1: no tx_dac; cannot set the DAC data source\n");
        return GNSS_L1_ERR_PHY;
    }

    /* chan = -1 applies to every channel. */
    axi_dac_set_datasel(phy->tx_dac, -1, sel);

    /* gnss_passthrough republishes its status only every 256 sample clocks and
     * then crosses it into the AXI domain, so give it time to refresh before
     * reading. 256 clocks is ~8 us; 2 ms is generous. */
    no_os_mdelay(2);

    status = gnss_pt_read(GNSS_PT_REG_STATUS);
    want   = (sel == AXI_DAC_DATA_SEL_DMA) ? 1 : 0;
    got_i  = (status & GNSS_PT_ST_DAC_EN_I0) ? 1 : 0;
    got_q  = (status & GNSS_PT_ST_DAC_EN_Q0) ? 1 : 0;

    printf("gnss_l1: DAC data source -> %s; dac_enable i/q reads %d / %d\n",
           ((unsigned)sel < (sizeof(name) / sizeof(name[0]))) ? name[sel] : "?",
           got_i, got_q);

    if (got_i != want || got_q != want) {
        printf("gnss_l1: DAC source NOT confirmed. Wanted dac_enable = %d on\n"
               "         both I and Q, read %d / %d (STATUS 0x%08lx).\n"
               "         The DAC core did not accept CHAN_CNTRL_7; samples from\n"
               "         gnss_passthrough will NOT reach the AD9361.\n",
               want, got_i, got_q, (unsigned long)status);
        return GNSS_L1_ERR_VERIFY;
    }
    return GNSS_L1_OK;
}

int32_t gnss_l1_tx_silence(struct ad9361_rf_phy *phy)
{
    uint32_t i;

    if (!phy || !phy->tx_dac) {
        printf("gnss_l1: no tx_dac; cannot silence TX\n");
        return GNSS_L1_ERR_PHY;
    }

    /* Two DDS tone slots per DAC channel, indexed 0 .. (2*num_channels - 1);
     * the same indexing axi_dac_data_setup() uses. */
    for (i = 0; i < (uint32_t)phy->tx_dac->num_channels * 2U; i++) {
        axi_dac_dds_set_scale(phy->tx_dac, i, 0);
    }

    printf("gnss_l1: TX silenced - %u DDS tone slots set to zero scale\n"
           "         (vendor default was 3 MHz at 5%% of full scale)\n",
           (unsigned)(phy->tx_dac->num_channels * 2));

    return gnss_l1_set_dac_source(phy, AXI_DAC_DATA_SEL_ZERO);
}

int32_t gnss_l1_enable_tx(struct ad9361_rf_phy *phy, gnss_l1_rf_path_t path)
{
    if (!phy) { return GNSS_L1_ERR_PHY; }

    if (path != GNSS_L1_PATH_CONDUCTED_WITH_ATTENUATION) {
        printf("\n"
               "*******************************************************************\n"
               "gnss_l1: REFUSING to enable TX.\n"
               "\n"
               "  This build retransmits on the GPS L1 centre frequency. It must\n"
               "  only be enabled into a CONDUCTED coaxial path with external\n"
               "  attenuation fitted between TX1 and the GNSS receiver\n"
               "  (project requirements 54 and 55).\n"
               "\n"
               "  To proceed, the caller must pass\n"
               "  GNSS_L1_PATH_CONDUCTED_WITH_ATTENUATION, which is an assertion\n"
               "  that you have physically verified the RF path.\n"
               "\n"
               "  Nothing in software can check what is actually attached to TX1.\n"
               "*******************************************************************\n");
        return GNSS_L1_ERR_UNSAFE;
    }

    printf("gnss_l1: conducted path asserted by the caller; enabling TX1 at a\n"
           "         conservative %u mdB attenuation\n",
           (unsigned)GNSS_L1_TX_SAFE_START_ATTEN_MDB);

    return gnss_l1_set_tx_attenuation(phy, GNSS_L1_TX_SAFE_START_ATTEN_MDB);
}

void gnss_l1_print_status(struct ad9361_rf_phy *phy)
{
    uint64_t rx_lo = 0, tx_lo = 0;
    uint32_t rx_bw = 0, tx_bw = 0, rx_fs = 0, tx_fs = 0, tx_att = 0;
    struct rf_rssi rssi;
    struct rf_rx_gain gain;

    if (!phy) { printf("gnss_l1: no phy\n"); return; }

    printf("\n--- AD9361 runtime state ---\n");

    if (ad9361_get_rx_lo_freq(phy, &rx_lo) == 0) {
        printf("  RX LO          : %llu Hz\n", (unsigned long long)rx_lo);
    } else { printf("  RX LO          : read failed\n"); }

    if (ad9361_get_tx_lo_freq(phy, &tx_lo) == 0) {
        printf("  TX LO          : %llu Hz\n", (unsigned long long)tx_lo);
    } else { printf("  TX LO          : read failed\n"); }

    if (ad9361_get_rx_sampling_freq(phy, &rx_fs) == 0) {
        printf("  RX sample rate : %u Hz\n", (unsigned)rx_fs);
    }
    if (ad9361_get_tx_sampling_freq(phy, &tx_fs) == 0) {
        printf("  TX sample rate : %u Hz\n", (unsigned)tx_fs);
    }
    if (ad9361_get_rx_rf_bandwidth(phy, &rx_bw) == 0) {
        printf("  RX bandwidth   : %u Hz\n", (unsigned)rx_bw);
    }
    if (ad9361_get_tx_rf_bandwidth(phy, &tx_bw) == 0) {
        printf("  TX bandwidth   : %u Hz\n", (unsigned)tx_bw);
    }
    if (ad9361_get_tx_attenuation(phy, 0, &tx_att) == 0) {
        printf("  TX1 attenuation: %u mdB\n", (unsigned)tx_att);
    }
    if (ad9361_get_rx_rf_gain(phy, 0, &gain.gain_db) == 0) {
        printf("  RX1 gain       : %ld dB\n", (long)gain.gain_db);
    }
    if (ad9361_get_rx_rssi(phy, 0, &rssi) == 0) {
        if (rssi.multiplier > 0) {
            printf("  RX1 RSSI       : %lu.%02lu dB\n",
                   (unsigned long)(rssi.symbol / (uint32_t)rssi.multiplier),
                   (unsigned long)(rssi.symbol % (uint32_t)rssi.multiplier));
        } else {
            printf("  RX1 RSSI       : raw %lu (multiplier reported as %ld)\n",
                   (unsigned long)rssi.symbol, (long)rssi.multiplier);
        }
    }
    printf("  ENSM state     : 0x%02x\n", ad9361_ensm_get_state(phy));
    printf("----------------------------\n");
}

int32_t gnss_l1_selftest(struct ad9361_rf_phy *phy)
{
    int32_t ret;

    printf("\n==================================================\n");
    printf(" GNSS L1 passthrough self-test\n");
    printf("==================================================\n");

    /* 1. Is the custom block in the bitstream and reachable? */
    ret = gnss_pt_probe();
    if (ret != GNSS_PT_OK) {
        printf("SELFTEST: FAIL - custom IP not reachable\n");
        return ret;
    }

    /* 2. AD9361 on L1. */
    ret = gnss_l1_configure_rx(phy);
    if (ret != GNSS_L1_OK) {
        printf("SELFTEST: FAIL - L1 configuration\n");
        return ret;
    }

    gnss_l1_print_status(phy);

    /* 3. Are samples actually moving through the PL?
     *
     * The DAC data source is left at ZERO for this, so nothing is transmitted.
     * dac_valid still pulses and the counters and elastic buffer behave
     * identically -- the source select only decides what the DAC core does
     * with the words we hand it. */
    gnss_pt_clear_counters();
    gnss_pt_set_passthrough(1);
    no_os_mdelay(50);

    ret = gnss_pt_check_dataflow(200);
    gnss_pt_print_state();

    if (ret != GNSS_PT_OK) {
        printf("SELFTEST: FAIL - no sample flow through gnss_passthrough\n");
        return ret;
    }

    /* 4. Will those samples actually be accepted by the AD9361 DAC?
     *
     * This is the check that was missing until 2026-09-14. The 2026-09-11
     * bring-up passed step 3 while the DAC core was in DDS mode and was
     * discarding every word this block produced.
     *
     * MUTE FIRST. With mute set, gnss_passthrough drives 0x0000 no matter what
     * pass_en says, so the DAC is handed silence while we confirm it is
     * listening. Nothing is transmitted at any point in this test. */
    gnss_pt_set_mute(1);
    no_os_mdelay(2);

    ret = gnss_l1_set_dac_source(phy, AXI_DAC_DATA_SEL_DMA);

    /* Back to silent regardless of the outcome. */
    (void)gnss_l1_set_dac_source(phy, AXI_DAC_DATA_SEL_ZERO);
    gnss_pt_set_mute(0);

    if (ret != GNSS_L1_OK) {
        printf("SELFTEST: FAIL - the AD9361 DAC will not accept samples from\n"
               "          gnss_passthrough (data-source select not confirmed)\n");
        return ret;
    }

    printf("SELFTEST: PASS - RX samples reach the TX datapath AND the AD9361\n"
           "          DAC accepts them when its source is set to DMA\n");
    printf("NOTE: this proves the digital path only. It says nothing about RF\n"
           "      output level or GNSS receiver behaviour.\n");
    printf("NOTE: TX is left SILENT - DAC source ZERO, DDS scale 0. Use the\n"
           "      conducted-path interlock in gnss_l1_enable_tx() before any\n"
           "      transmission, and only once the attenuation is calculated.\n");

    /* 5. ISSUE-0008. Runs after the graded SELFTEST line so it cannot affect
     *    the markers Test_Hardware.ps1 greps for. Receive-only and harmless. */
    gnss_l1_band_probe(phy);

    return GNSS_L1_OK;
}
