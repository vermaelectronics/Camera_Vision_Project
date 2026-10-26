/******************************************************************************
 *  gnss_aj.h  --  SMI-PI v4.1 anti-jam engine (runs on the ARM)
 *
 *  WHAT IT DOES, CONTINUOUSLY, WITH NO OPERATOR SETTING
 *    1. Captures 4096 samples of RX1 and RX2 into DDR over the existing ADC
 *       DMA (a fan-out of the RX stream; the PL path is not interrupted).
 *    2. CLEAR (no jammer): the filter is BYPASSED -- TX1 carries RX1 exactly,
 *       8 samples later -- and each capture only goes through a cheap
 *       detector (aj_detect: powers + RX1/RX2 coherence). Meanwhile it learns
 *       the receiver noise floor and the RX2/RX1 level ratio (the input
 *       normalisation g2 the FPGA applies to RX2).
 *    3. JAMMED: from the capture that detected the jammer on, every capture
 *       is solved (aj_compute: exact space-time statistics, 31 + 2 image
 *       weights, loading = the canceller's own noise estimate, Cholesky in
 *       double precision) and the weights are committed to the FPGA filter
 *       together with cfg (filter in, output make-up shift) and g2.
 *    4. Back to CLEAR (bypass) once the jammer has been gone for 0.5 s.
 *    Every change is printed on the console with a time stamp:
 *         [AJ   12.345 s] *** JAMMER ON *** ...
 *         [AJ   12.361 s] NULLED in 16 ms ...
 *         [AJ   45.678 s] jammer OFF after 33.3 s ...
 *    plus a status line every gnss_aj_log= seconds (default 30, held back while the operator types), a CSV line
 *    every gnss_aj_csv= ms (default off) and the full gnss_report? block.
 *
 *  Runs from the console idle hook (gnss_idle), like the matched RX gain loop.
 *****************************************************************************/
#ifndef GNSS_AJ_H_
#define GNSS_AJ_H_

#include <stdint.h>

/* DDR buffer for the captures: 4096 samples x 2 channels x 4 bytes = 32 KiB.
 * Away from the boot captures (0x10000000, 0x10800000; gnss_capture.h). */
#ifndef GNSS_AJ_BUF_ADDR
#define GNSS_AJ_BUF_ADDR   0x11000000U
#endif

#define GNSS_AJ_VERSION_STR  "4.1.1"

/* RF profiles (gnss_band=). RX and TX are always set together, so the
 * retransmitted signal comes out on the frequency it came in on and the
 * board's clock error cancels between down- and up-conversion. */
#define GNSS_BAND_L1          0   /* 1575.42 MHz, 30.72 MS/s, 18 MHz: L1 only      */
#define GNSS_BAND_ALL         1   /* 1582.5 MHz, 61.44 MS/s, 56 MHz: GPS, Galileo, */
                                  /*   QZSS, SBAS, BeiDou B1I+B1C, GLONASS G1      */
#define GNSS_BAND_NO_BDS      2   /* 1589.5 MHz, 50 MS/s: GPS, Galileo, GLONASS    */
#define GNSS_BAND_NO_GLO      3   /* 1568.25 MHz, 50 MS/s: GPS, Galileo, BeiDou    */
#define GNSS_BAND_COUNT       4
#define GNSS_BAND_DEFAULT     GNSS_BAND_ALL

void gnss_aj_init(void);            /* once, after the radio and DMA are up  */
void gnss_aj_enable(int on);        /* filter in the TX1 path + engine       */
void gnss_aj_command(int on);       /* the same, by the operator (gnss_smi=) */
int  gnss_aj_enabled(void);
int  gnss_aj_user_off(void);        /* 1 if the operator typed gnss_smi=0    */
void gnss_aj_freeze(int on);        /* keep the present weights              */
void gnss_aj_set_mode(int mode);    /* AJ_MODE_STAP / _RX2 / _NB / _LIN      */
void gnss_aj_set_log(int seconds);  /* periodic status line, 0 = events only */
void gnss_aj_set_csv(int ms);       /* CSV line period, 0 = off              */
int  gnss_aj_jammer_on(void);
void gnss_aj_poll(void);
void gnss_aj_print(void);           /* gnss_aj? / gnss_smi?                  */
void gnss_aj_print_short(void);
void gnss_aj_report(void);          /* gnss_report?: everything, for a copy  */
void gnss_aj_note_tx(int on);       /* gnss_tx= tells the engine (time stamp) */

/* Waits for the engine's capture in flight (if any) and leaves the RX DMA
 * free. Call before anything else submits a transfer on rx_dmac. */
void gnss_aj_dma_release(void);

/* RF profile; returns 0 on success. */
int  gnss_band_set(int band);
int  gnss_band_get(void);
void gnss_band_print(void);

/* The console idle hook: matched RX gain loop + anti-jam engine + radio
 * guard (keeps the gnss_band= profile and the AD9361 FIRs off). */
void gnss_idle(void);

#endif
