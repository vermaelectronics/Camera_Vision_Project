/***************************************************************************//**
 *   @file   gnss_dash.h
 *   @brief  GNSS-CRPA MOD-25: colour console check page and test-command list
 *
 *   gnss_check?   one page: null core, every SMI feature (image taps, DC
 *                 removal, diagonal loading, taper, window, constraint),
 *                 honest suppression figures, RX/ADC, output/clip back-off,
 *                 and a verdict. Each line ends in [ OK ] / [WARN] / [FAIL]
 *                 / [INFO].
 *   gnss_cmds?    the commands used to test the anti-jam, in order.
 *   gnss_color=   1 = ANSI colours (default), 0 = plain text (terminals
 *                 without colour, log files).
 ******************************************************************************/
#ifndef GNSS_DASH_H_
#define GNSS_DASH_H_

/* ANSI colour strings; "" when colours are off */
#define GC_RST   0
#define GC_RED   1
#define GC_GRN   2
#define GC_YEL   3
#define GC_CYN   4
#define GC_BLD   5
#define GC_DIM   6
#define GC_MAG   7
const char *gnss_c(int c);
void        gnss_color_set(int on);
int         gnss_color_get(void);

void        gnss_dash_check(void);    /* gnss_check? */
void        gnss_dash_cmds(void);     /* gnss_cmds?  */

#endif /* GNSS_DASH_H_ */
