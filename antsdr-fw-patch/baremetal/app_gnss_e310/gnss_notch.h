/******************************************************************************
 * gnss_notch.h -- automatic CW notch (antsdr-fw-patch crpa5).
 *
 * The FPGA (smi_pi 2.6, AJ_INFO layout 8) can remove one CW tone from RX1 and
 * RX2 in front of the space-time filter. This module decides when:
 *   gnss_notch=1 (default) AUTOMATIC: every 50 ms the capture is searched for a
 *     CW tone (gnss_notch_math.c). Seen in 3 checks in a row at the same
 *     frequency -> notch ON at that frequency; followed if it drifts; not seen
 *     for 2 s -> notch OFF. A wideband or partial-band jammer is never taken for
 *     a tone, so with it the notch stays off and the anti-jam works as before.
 *   gnss_notch=0 OFF.  gnss_notch_rf=<MHz> ON at a fixed RF frequency (=2).
 * While the notch is on, the weights are computed from the capture with the
 * tone removed by the software model of the same notch, so the space-time
 * filter spends its one spatial null on what the notch leaves (a wideband
 * jammer), not on the tone.
 *****************************************************************************/
#ifndef GNSS_NOTCH_H_
#define GNSS_NOTCH_H_

#include <stdint.h>

#define GNSS_NOTCH_OFF    0
#define GNSS_NOTCH_AUTO   1
#define GNSS_NOTCH_FIXED  2

void gnss_notch_init(void);             /* anti-jam start: is the notch in the bitstream? */
/* One anti-jam capture (aj_compute() format): detect, switch, and return the
 * capture the solver should use (a notched copy while the notch is on).
 * noise = receiver noise floor [LSB^2] at the present gain, <= 0 if unknown. */
const uint32_t *gnss_notch_capture(const uint32_t *raw, int nsamp, double noise);
int  gnss_notch_take_change(void);      /* 1 once after the notch switched or moved */
void gnss_notch_off(void);              /* anti-jam off, band / radio change */
void gnss_notch_set_mode(int mode);
int  gnss_notch_set_rf(double rf_mhz);  /* fixed notch at this RF frequency */
int  gnss_notch_available(void);
int  gnss_notch_on(void);
void gnss_notch_print(void);            /* gnss_notch? */
void gnss_notch_report_line(void);      /* one line for gnss_aj? / gnss_report? */

#endif
