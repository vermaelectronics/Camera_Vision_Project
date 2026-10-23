/******************************************************************************
 *  gnss_aj.h  --  SMI-PI v4.0 anti-jam engine (runs on the ARM)
 *
 *  WHAT IT DOES, CONTINUOUSLY, WITH NO OPERATOR SETTING
 *    1. Captures 4096 samples of RX1 and RX2 into DDR over the existing ADC
 *       DMA (a fan-out of the RX stream; the PL path is not interrupted).
 *    2. Computes the 31 space-time weights (gnss_aj_math.c): exact sample
 *       statistics, loading = the noise power estimated from the canceller's
 *       own output, complex Cholesky solve in double precision.
 *    3. Writes them to gnss_passthrough and commits them in one go; the
 *       16+16-tap filter in the FPGA (smi_pi_0) uses them from the next sample.
 *    4. Decides whether a jammer is present and prints it on the console:
 *         [AJ] *** JAMMER ON ***  ...   when it appears
 *         [AJ] jammer OFF ...           when it goes away
 *       plus a status line every gnss_aj_log= seconds (default 5).
 *    About 100-200 weight updates per second; a new jammer is nulled within
 *    one or two updates (~10-20 ms), well inside a receiver's tracking loops.
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

/* RF profiles (gnss_band=). RX and TX are always set together, so the
 * retransmitted signal comes out on the frequency it came in on and the
 * board's clock error cancels between down- and up-conversion. */
#define GNSS_BAND_GPS_L1      0   /* 1575.42 MHz, 30.72 MS/s, 18 MHz: GPS/GAL/QZSS L1 */
#define GNSS_BAND_MULTI       1   /* 1584.5 MHz, 50 MS/s, 56 MHz: + GLONASS G1, BeiDou B1I */
#define GNSS_BAND_MULTI_MAX   2   /* 1584.5 MHz, 61.44 MS/s, 56 MHz: widest the AD9361 does */
#define GNSS_BAND_DEFAULT     GNSS_BAND_MULTI

void gnss_aj_init(void);            /* once, after the radio and DMA are up  */
void gnss_aj_enable(int on);        /* filter in the TX1 path + engine       */
void gnss_aj_command(int on);       /* the same, by the operator (gnss_smi=) */
int  gnss_aj_enabled(void);
int  gnss_aj_user_off(void);        /* 1 if the operator typed gnss_smi=0    */
void gnss_aj_freeze(int on);        /* keep the present weights              */
void gnss_aj_set_mode(int mode);    /* AJ_MODE_STAP / _RX2 / _NB             */
void gnss_aj_set_log(int seconds);  /* periodic status line, 0 = events only */
int  gnss_aj_jammer_on(void);
void gnss_aj_poll(void);
void gnss_aj_print(void);           /* gnss_aj? / gnss_smi?                  */
void gnss_aj_print_short(void);

/* Waits for the engine's capture in flight (if any) and leaves the RX DMA
 * free. Call before anything else submits a transfer on rx_dmac. */
void gnss_aj_dma_release(void);

/* RF profile; returns 0 on success. */
int  gnss_band_set(int band);
int  gnss_band_get(void);
void gnss_band_print(void);

/* The console idle hook: matched RX gain loop + anti-jam engine. */
void gnss_idle(void);

#endif
