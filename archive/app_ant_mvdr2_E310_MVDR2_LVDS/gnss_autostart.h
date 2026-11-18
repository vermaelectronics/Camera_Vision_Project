/***************************************************************************//**
 * @file   gnss_autostart.h
 * @brief  GNSS-CRPA MOD-15: automatic MVDR start, health check and event
 *         monitor. Every step prints one line tagged
 *             [PASS]  as it should be
 *             [WARN]  works, but not as well as it could -- the line says why
 *                     and what to do
 *             [FAIL]  wrong; TX1 is kept (or put back) SILENT
 *             [INFO]  a measurement, no verdict
 *         and the run ends with a summary. TX1 opens only with 0 FAIL.
 ******************************************************************************/
#ifndef GNSS_AUTOSTART_H_
#define GNSS_AUTOSTART_H_

#include <stdint.h>

/* The whole start: FPGA + MVDR identity, TX1 silenced, radio plan applied and
 * read back, l_clk, MVDR + DAC path enabled, data flow, RX gain auto-level,
 * IQ correction, AOLC, jammer / coherence, TX1 opened to the operating
 * attenuation. Returns the number of FAILs (0 = MVDR running on TX1). */
int  gnss_autostart_run(void);

/* Read-only: the same checks on the running system, changes nothing.
 * Returns the number of FAILs. banner = 1 prints the heading and summary. */
int  gnss_health_check(int banner);

/* Steps the RX1+RX2 manual gain (always equal) until the noise floor per
 * element reads GNSS_CFG_RX_NF_TARGET_DBFS, keeping >= 3 dB ADC headroom
 * when a jammer is present. Works with the jammer on or off (MVDR >= 1.11
 * separates the noise from the jammer). Returns 0 = on target, 1 = WARN
 * (limited), -1 = FAIL. standalone = 1 prints its own heading. */
int  gnss_rx_autolevel(int standalone);

/* Event monitor, called by the console loop on every line entered.
 * Prints an [EVENT] line for everything that changed since the last call
 * (ADC / output clipping started, FIFO overflow, jammer arrived / left,
 * MVDR stalled, DAC path lost, l_clk changed, output level off target...).
 * show_status = 1 (an empty line was entered) also prints a one-line status.
 * Silent until gnss_autostart_run() or gnss_health_check() has run once. */
void gnss_monitor_poll(int show_status);

/* Clip guard (GNSS_CFG_CLIP_GUARD): when the ADC clips (a strong jammer),
 * the RX gain steps down 6 dB at a time on both channels, and back up 3 dB
 * at a time to the operating gain once the jammer is gone or weaker. */
void gnss_guard_enable(int en);
void gnss_guard_set_op_gain(int32_t g);   /* gnss_rxgain= / auto-level      */
void gnss_guard_print(void);
int  gnss_guard_is_on(void);
/* MVDR-safe "AGC": back to the auto-level gain, clip guard on. */
void gnss_guard_auto(void);
/* MOD-17: RX gain mode: 1 = AUTO (MVDR-safe), 0 = MANUAL at the current gain */
void gnss_agc_set_mode(int autom);
void gnss_agc_print(void);

/* Called by the console loop every ~10 ms while no key is pressed: runs the
 * AUTO gain (clipping check every 10 ms, gain moves every 100 ms / 0.5 s), and
 * once a second the event monitor. Returns 1 if it printed
 * anything (the console then re-prints the half-typed line). */
int  gnss_background_tick(void);
/* MOD-20: re-run the automatic start 1 s from now (restarted by each call) */
void gnss_request_restart(void);

/* MOD-25: RX gain mode for the gnss_check? page: 1 = AUTO, 0 = MANUAL */
int gnss_agc_is_auto(void);
/* MOD-26: for gnss_report? */
int32_t     gnss_agc_gain_now(void);      /* RX gain the AGC set/saw, -1 unknown   */
int32_t     gnss_agc_clean_gain(void);    /* gain at the target without jammer, -1 */
const char *gnss_agc_reason(void);        /* what the AGC is doing and why         */
double      gnss_rx_target_dbfs(void);    /* noise target in use                   */
double      gnss_rx_floor_dbfs(void);     /* AD9361 floor measured at start        */
void        gnss_floor_scan(void);        /* MOD-27: gnss_floor? noise vs gain      */
void        gnss_spec_scan(int gain);     /* MOD-29: gnss_spec? / =G noise spectrum  */
void        gnss_bist_test(void);         /* MOD-29: gnss_bist? FPGA data-link test  */
/* MOD-30: the gain is levelled on the AD9361 floor where the satellites are */
int         gnss_floor_mode(void);         /* 0 whole band, 1 L1/E1/B1C, 2 all signals */
void        gnss_floor_mode_set(int m);
const char *gnss_floor_mode_name(int m);
double      gnss_rx_floor_sig_dbfs(void);
uint32_t    gnss_agc_hunts(void);          /* MOD-31: AGC hunts since boot (anti-hunt lock) */  /* floor for the mode, white-equivalent dBFS */
void        gnss_steer_user_changed(void); /* MOD-26b: gnss_steer= by hand          */

/* MOD-28: 1 while the console prompt "E310> " is the last thing on the line
 * (set by main.c). Background output then starts on a new line first. */
extern int  gnss_prompt_open;

#endif /* GNSS_AUTOSTART_H_ */
