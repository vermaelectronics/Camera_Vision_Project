/******************************************************************************
 *  gnss_l1.h
 *  GPS L1 configuration and status reporting for the ANTSDR E310 V1.
 *
 *  Requirement 49: the PS configures the AD9361, LO, sample rate, bandwidth,
 *  gains and attenuation.
 *  Requirement 50: that configuration is repeatable from code, not clicked in
 *  a GUI.
 *
 *  Original work for this project.
 *****************************************************************************/
#ifndef GNSS_L1_H_
#define GNSS_L1_H_

#include <stdint.h>
#include "ad9361_api.h"
#include "axi_dac_core.h"   /* enum axi_dac_data_sel */

/* --------------------------------------------------------------------------
 *  Fixed by the project requirements.
 * -------------------------------------------------------------------------- */
#define GNSS_L1_CENTRE_HZ           1575420000ULL   /* GPS L1, requirement 37 */

/* How far the actual LO may sit from the target and still be accepted.
 *
 * The AD9361 RF synthesiser has finite resolution. Measured on this board at
 * L1: asked for 1575420000 Hz, got 1575419998 Hz. 100 Hz is two orders of
 * magnitude tighter than the ~1 kHz Doppler bins a GPS L1 C/A search uses, so
 * it cannot mask a problem that would matter, while still catching an LO that
 * is genuinely on the wrong frequency. */
#define GNSS_L1_LO_TOLERANCE_HZ     100ULL

/* --------------------------------------------------------------------------
 *  Inherited from the MicroPhase baseline, NOT chosen by this project.
 *
 *  Source: Vendor/MicroPhase_E310_V1/antsdr_standalone/app_e310/main.c,
 *          AD9361_InitParam default_init_param.
 *
 *  These are deliberately left at the vendor values.  30.72 MSPS with 18 MHz
 *  of RF bandwidth is far wider than a GPS L1 C/A signal needs, but narrowing
 *  it is a design decision that has not been taken yet -- see the UNRESOLVED
 *  list in Source/Config/board_e310_v1.json.  Do not silently retune these.
 * -------------------------------------------------------------------------- */
#define GNSS_L1_VENDOR_SAMPLE_RATE_HZ   30720000U
#define GNSS_L1_VENDOR_RF_BANDWIDTH_HZ  18000000U
#define GNSS_L1_VENDOR_TX_ATTEN_MDB     10000U     /* 10.0 dB */

/* --------------------------------------------------------------------------
 *  Safety.
 *
 *  Requirements 54 and 55: the first GNSS validation is a CONDUCTED test over
 *  coax with external attenuation, not an over-the-air retransmission, and it
 *  starts at conservative TX power.
 *
 *  gnss_l1_enable_tx() therefore refuses to run until the caller has asserted
 *  that a conducted path with attenuation is actually in place.  This is a
 *  procedural interlock, not an RF safety device -- it cannot detect what is
 *  really connected to TX1.
 * -------------------------------------------------------------------------- */
#define GNSS_L1_TX_SAFE_START_ATTEN_MDB  30000U    /* 30 dB, conservative     */

/* The QUIETEST the AD9361 can transmit: the full 89.75 dB of TX attenuation,
 * which is the top of the part's 0 - 89.75 dB range in 0.25 dB steps.
 *
 * With TX1 at roughly +7 dBm unattenuated, this puts about -83 dBm on the
 * connector. For scale: live GPS L1 arrives at an antenna at about -130 dBm,
 * and a GNSS module fed by an active antenna sees roughly -105 dBm. So -83 dBm
 * is still ~22 dB HOT for a GNSS receiver and will saturate it -- but it is
 * ~90 dB below the ~+10 dBm where front-end damage begins. That margin is why
 * this is the correct value to start a conducted test at, and why
 * GNSS_L1_TX_SAFE_START_ATTEN_MDB (30 dB, i.e. about -23 dBm) is NOT.
 *
 * Step DOWN from here while watching the receiver. Never start below it. */
#define GNSS_L1_TX_MAX_ATTEN_MDB         89750U    /* 89.75 dB = quietest     */

/* --------------------------------------------------------------------------
 *  Deployment (SD-card, unattended) TX attenuation. GNSS-CRPA MOD-8.
 *
 *  Only used when GNSS_DEPLOY_AUTO_TX is defined, which only
 *  Automation/PowerShell/New-Deployment.ps1 does.
 *
 *  70000 mdB = 70 dB, about -63 dBm out. This is the value MEASURED to work on
 *  2026-09-15 (TEST-025): at 89.75 dB the receiver saw one satellite at
 *  26 dB-Hz and got no fix; at 70 dB it held a 3D fix with 8 satellites used and
 *  C/N0 up to 38 dB-Hz. It is not a calculated link budget -- it was found by
 *  stepping down and stopping when the receiver tracked.
 *
 *  SAFETY: -63 dBm is ~73 dB below the ~+10 dBm where GNSS front ends are
 *  damaged, so it cannot harm a receiver. It is still ~40 dB HOT compared with
 *  live GNSS, which is what makes it work over a short conducted path.
 *
 *  CHANGE THIS, not the code, if a deployment needs a different level. Larger
 *  number = quieter. Anything below about 40000 into a directly connected
 *  receiver is pointlessly loud and should be justified first.
 * -------------------------------------------------------------------------- */
#ifndef GNSS_DEPLOY_TX_ATTEN_MDB
#define GNSS_DEPLOY_TX_ATTEN_MDB         70000U    /* 70 dB, measured working  */
#endif

typedef enum {
    GNSS_L1_PATH_UNCONFIRMED = 0,
    GNSS_L1_PATH_CONDUCTED_WITH_ATTENUATION = 0x5AFE
} gnss_l1_rf_path_t;

/* Return codes */
#define GNSS_L1_OK             0
#define GNSS_L1_ERR_PHY       -1
#define GNSS_L1_ERR_LO        -2
#define GNSS_L1_ERR_VERIFY    -3
#define GNSS_L1_ERR_UNSAFE    -4

/* --------------------------------------------------------------------------
 *  API
 * -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
 *  RF band selection.
 *
 *  The E310 V1 splits every RF port into two paths and selects between them
 *  with a SKY13335-381LF SPDT switch. Both halves must agree:
 *
 *    1. the EXTERNAL switch, driven by FPGA pins via gpio_o[7:0]
 *       (system_top.v lines 119-126, system.xdc pins G14/C20/B19/B20/...)
 *    2. the AD9361's INTERNAL RF port selection, because each switch output
 *       goes to a different balun wired to a different AD9361 port pair
 *
 *  Schematic ANT_E310_Public.pdf sheet 13 "RF TX/RX":
 *      B ports -> 5 MHz - 3 GHz balun   (low band)
 *      A ports -> 3 GHz - 6 GHz balun   (high band)
 *
 *  Vendor decode, command.c get/set_{tx,rx}_lo_freq:
 *      <= 3 GHz : *_ctrl_h = 0, *_ctrl_l = 1, RX = B_BALANCED, TX = TXB
 *      >  3 GHz : *_ctrl_h = 1, *_ctrl_l = 0, RX = A_BALANCED, TX = TXA
 *
 *  GPS L1 at 1575.42 MHz is LOW BAND.
 *
 *  WHY THIS FUNCTION EXISTS. That decode lives only in the console command
 *  handlers. gnss_l1_configure_rx() calls ad9361_set_{rx,tx}_lo_freq() directly
 *  and so bypasses it entirely. The external switches happen to come up in the
 *  low-band position (ad9361.c:1025-1032 at reset), but the AD9361's internal
 *  ports do NOT: main.c:240-241 set rx_rf_port_input_select and
 *  tx_rf_port_input_select to 0, which is A_BALANCED / TXA -- the HIGH band.
 *  The two halves therefore disagreed at L1 and the RF chain was broken in
 *  both directions. See ISSUE-0007.
 * -------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------
 *  GPIO POLARITY FOR THE LOW BAND -- UNRESOLVED. See ISSUE-0008.
 *
 *  The two authorities disagree, and the board in use is the one they disagree
 *  about:
 *
 *    Vendor firmware (command.c, #ifdef ANTSDR_E310), for LO <= 3 GHz:
 *        *_ctrl_h = 0, *_ctrl_l = 1
 *
 *    ANT_E310_ES2_Public.pdf sheet 13 -- the schematic for the TYPE-C board,
 *    which is the unit connected here -- prints a truth table beside all four
 *    switches, identically:
 *                    H   L
 *            5M-3G   1   0
 *            3G-6G   0   1
 *        i.e. LOW band requires *_ctrl_h = 1, *_ctrl_l = 0. The OPPOSITE.
 *
 *  Neither can be dismissed. The pin assignment is identical in both
 *  schematics and matches system.xdc, so there is no wiring swap that
 *  reconciles them. The older ANT_E310_Public.pdf names the part
 *  (SKY13335-381LF) but prints no truth table; the ES2 PDF prints the table but
 *  omits the part number, so the switch datasheet cannot arbitrate either.
 *
 *  This is a HARDWARE FACT and must be MEASURED, not guessed (hard rule 1).
 *  gnss_l1_band_probe() does exactly that in one run.
 *
 *  Default below is the VENDOR FIRMWARE polarity, on the reasoning that it is
 *  what ships and is presumably field-exercised -- NOT because it is confirmed.
 *  Flip these two lines if the probe says otherwise.
 * -------------------------------------------------------------------------- */
#define GNSS_L1_BAND_LOW_CTRL_H   0
#define GNSS_L1_BAND_LOW_CTRL_L   1

int32_t gnss_l1_select_rf_band(struct ad9361_rf_phy *phy);

/* Drives all eight external band-select lines to one polarity. Exposed so that
 * gnss_l1_band_probe() and gnss_capture_both_polarities() can exercise both
 * candidates from one definition. Does NOT touch the AD9361 internal ports. */
void gnss_l1_set_band_polarity(struct ad9361_rf_phy *phy, uint8_t h, uint8_t l);

/* Resolves ISSUE-0008 empirically. Drives both candidate polarities in turn and
 * reports RX1 gain and RSSI for each, then restores the configured default.
 *
 * Interpretation: with a signal present on RX1 -- a GNSS antenna, or a
 * generator at L1 -- the CORRECT polarity routes the signal through the
 * 5 MHz-3 GHz balun and will show a LOWER RSSI figure (RSSI here is reported as
 * attenuation, so lower = stronger) and, under AGC, a LOWER RX gain.
 *
 * With nothing connected to RX1 both readings will look the same and the test
 * proves NOTHING. Connect a signal first. */
int32_t gnss_l1_band_probe(struct ad9361_rf_phy *phy);

/* Applies the L1 receive configuration: RF band selection, LO to 1575.42 MHz
 * on both RX and TX synthesisers, vendor sample rate and bandwidth retained.
 * Reads every value back and fails if the hardware did not accept it
 * (requirement 60 in spirit: do not report success without evidence). */
int32_t gnss_l1_configure_rx(struct ad9361_rf_phy *phy);

/* Enables TX1 at a conservative attenuation.  Refuses unless path ==
 * GNSS_L1_PATH_CONDUCTED_WITH_ATTENUATION. */
int32_t gnss_l1_enable_tx(struct ad9361_rf_phy *phy, gnss_l1_rf_path_t path);

/* Sets TX attenuation in millibels down from full scale.  Larger = quieter. */
int32_t gnss_l1_set_tx_attenuation(struct ad9361_rf_phy *phy, uint32_t mdB);

/* --------------------------------------------------------------------------
 *  DAC data-source control.
 *
 *  The AD9361 DAC core does NOT take the samples gnss_passthrough drives
 *  unless the channel's data-source select is DMA (2).  See the STATUS [2]/[3]
 *  commentary in Source/HDL/gnss_passthrough.v for the vendor RTL that decides
 *  this.  Getting this wrong is silent: TX_COUNT still advances, the FIFO
 *  still behaves, and the DAC emits its DDS tone instead of your samples.
 * -------------------------------------------------------------------------- */

/* Points the DAC channels at `sel` and then CONFIRMS it by reading the
 * dac_enable bits back out of gnss_passthrough's STATUS register, which are a
 * direct read-back of (dac_data_sel == 2) inside axi_ad9361.  Returns
 * GNSS_L1_ERR_VERIFY if the hardware did not end up where it was told to go. */
int32_t gnss_l1_set_dac_source(struct ad9361_rf_phy *phy,
                               enum axi_dac_data_sel sel);

/* Makes TX1 RF-silent: DDS tone amplitude to zero on every tone slot, then
 * DAC data source to ZERO (4'h3).
 *
 * WHY THIS EXISTS.  The inherited vendor init leaves the DAC emitting a tone.
 * tx_dac_init in main.c passes channels = NULL, so axi_dac_data_setup() takes
 * its fallback branch (axi_dac_core.c:1002-1012) and programs every DDS slot
 * to 3 MHz at 50000 micro-units = 5% of full scale, then main.c selects DDS.
 * With the ENSM in FDD (0x0a) and TX1 attenuation at 10 dB, that is a live CW
 * carrier roughly 3 MHz off the GPS L1 centre on TX1, present from the moment
 * the firmware runs.  Nothing in this project asked for it and nothing needs
 * it, so it is silenced at start-up and TX output becomes deliberate only.
 *
 * This is a default, NOT an RF safety device.  It cannot know what is attached
 * to TX1.  The conducted-path interlock in gnss_l1_enable_tx() still governs
 * actual transmission. */
int32_t gnss_l1_tx_silence(struct ad9361_rf_phy *phy);

/* Prints LO, sample rate, bandwidth, gain, attenuation and RSSI
 * (requirement 58).  Every value is read back from the device. */
void    gnss_l1_print_status(struct ad9361_rf_phy *phy);

/* One-shot bring-up used by the hardware test entry point: configure RX,
 * report status, verify the custom IP sees samples moving. */
int32_t gnss_l1_selftest(struct ad9361_rf_phy *phy);

#endif /* GNSS_L1_H_ */
