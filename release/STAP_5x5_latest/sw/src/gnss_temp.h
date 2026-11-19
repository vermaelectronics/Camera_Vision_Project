/*
 * gnss_temp.h -- MOD12: board temperature monitor (AD9361 die + Zynq die)
 * and RX/TX LO lock watch.
 */
#ifndef GNSS_TEMP_H_
#define GNSS_TEMP_H_

typedef struct {
    int    have_rf;          /* AD9361 sensor read OK                         */
    double rf_c;             /* AD9361 die, deg C (now)                       */
    double rf_boot_c;        /* ... at boot (when the calibrations ran)        */
    double rf_min_c, rf_max_c;
    int    have_pl;          /* Zynq XADC read OK                             */
    double pl_c;             /* Zynq die, deg C (now)                         */
    double pl_max_c;
    int    rx_lock, tx_lock; /* AD9361 RF PLLs locked (1) / not (0) / -1 n.a. */
    unsigned long relocks;   /* LO re-tunes after a lost lock                 */
} gnss_temp_t;

void gnss_temp_poll(void);                 /* once a second (event monitor) */
void gnss_temp_get(gnss_temp_t *t);
void gnss_temp_print_line(const char *pad); /* one report line, "\n" ended  */

#endif /* GNSS_TEMP_H_ */
