/******************************************************************************
 *  gnss_lcmv_auto.h
 *  Automatic operation of the 2-element LCMV CRPA (bitstream v3.1+).
 *
 *  Everything here runs by itself from boot; the console only changes it.
 *
 *  1. RX link watchdog  At boot, and every 2 s afterwards, checks that RX
 *                       samples reach the PL (gnss_passthrough RX counter).
 *                       If not, re-runs the AD9361 RX interface tuning.
 *  2. LCMV weights      Reads the 2x2 covariance R measured in the PL and
 *                       solves the paper's  w = R^-1 C (C^H R^-1 C)^-1 f
 *                       (lcmv_math.c) about every 1 ms, then loads the
 *                       weights into the PL beamformer.
 *                         AUTO  (default) look constraint only: the null
 *                               follows the jammer, no jammer angle needed.
 *                         ANGLE look + enforcement constraint toward a
 *                               known jammer angle (paper eq. 9).
 *  3. Matched RX gain   RX1 and RX2 always at the SAME manual gain (keeps
 *                       the RX2 calibration valid), stepped together every
 *                       100 ms to keep the input below ADC clipping.
 *
 *  Original work for the ANTSDR E310 V1 GNSS CRPA project.
 *****************************************************************************/
#ifndef GNSS_LCMV_AUTO_H_
#define GNSS_LCMV_AUTO_H_

#include <stdint.h>

#define GNSS_LCMV_OFF    0   /* TX1 = RX1                                  */
#define GNSS_LCMV_AUTO   1   /* covariance LCMV, look constraint (default) */
#define GNSS_LCMV_ANGLE  2   /* covariance LCMV, look + jammer angle       */
#define GNSS_LCMV_PL     3   /* PL angle solver only (no covariance)       */

struct ad9361_rf_phy;

/* Boot: RX link check, defaults (AUTO, GPS look 0 deg, auto gain), and the
 * console idle hook that runs everything afterwards. */
void gnss_lcmv_init(struct ad9361_rf_phy *phy);

/* Console idle hook (registered by gnss_lcmv_init). */
void gnss_lcmv_poll(void);

void gnss_lcmv_set_mode(int mode);
int  gnss_lcmv_get_mode(void);
void gnss_lcmv_set_gps_angle(int deg);
void gnss_lcmv_set_jam_angle(int deg);
void gnss_lcmv_set_eps(double eps);
void gnss_lcmv_set_cal(double re, double im);

/* Gain: auto = matched loop (default); manual = fixed dB on both channels. */
void gnss_lcmv_gain_auto(int on);
int  gnss_lcmv_gain_manual(int32_t db);

/* RX link: returns 1 if RX samples are reaching the PL. With retune, runs
 * the AD9361 RX interface tuning up to 'attempts' times until they do. */
int  gnss_rx_link_check(int attempts, int verbose);

void gnss_lcmv_print(void);
void gnss_rx_gain_report(void);

#endif /* GNSS_LCMV_AUTO_H_ */
