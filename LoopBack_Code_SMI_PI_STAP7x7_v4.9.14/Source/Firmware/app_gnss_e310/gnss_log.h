/* ============================================================================
 *  gnss_log.h  --  periodic snapshot of everything the console can print (4.9.6)
 *
 *  Every period (default 60 s, gnss_log=N seconds, 0 = off) the firmware runs
 *  gnss_status?, gnss_tx?, gnss_band?, gnss_rx_gain?, gnss_smi?, radio?,
 *  tx1_attenuation?, gnss_aj? and gnss_report? with their output captured into a
 *  RAM buffer, adds a time stamp, and
 *    - with -DGNSS_LOG_SD (build_all.sh --defines GNSS_LOG_SD): writes it to the
 *      SD card as a text file  0:/LOG/<YYYYMMDD>/<HHMMSS>.TXT  (8.3 names, one
 *      folder per day; without a clock: 0:/LOG/UPTIME/T<seconds>.TXT);
 *    - without it (default): prints the same block on the console between
 *      "===== GNSS_LOG BEGIN" and "===== GNSS_LOG END" lines, so the serial
 *      terminal's own log file keeps it.
 *  There is no real-time clock: the time stamp is the uptime, plus the wall
 *  clock if gnss_time=Y M D h m s was typed after boot.
 * ========================================================================== */
#ifndef GNSS_LOG_H_
#define GNSS_LOG_H_

void gnss_log_init(void);                    /* once, after the console and the anti-jam engine are up */
void gnss_log_poll(void);                    /* from the idle loop */
void gnss_log_set_period(unsigned seconds);  /* 0 = off */
void gnss_log_set_clock(int y, int mo, int d, int h, int mi, int s);
void gnss_log_now(void);                     /* one snapshot now */
void gnss_log_print_state(void);

#endif
