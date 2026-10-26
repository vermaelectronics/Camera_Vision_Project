/******************************************************************************
 *  gnss_npi_aj.h  --  PL-NPI v2.3 anti-jam engine (runs on the ARM)
 *
 *  The PL-NPI weights adapt in the FPGA (pl_npi_0). This engine supervises
 *  the core, with no operator setting:
 *    1. GATING. The core runs in AUTO mode: its power detector switches the
 *       filter in and the adaptation on within ~1 us of a jammer appearing
 *       (input 6 dB above the noise floor; the engine sets that threshold and
 *       keeps it scaled to the RX gain), and back to BYPASS (TX1 = RX1,
 *       weights 0) 0.5 s after it has gone. With no jammer nothing adapts.
 *    2. WEAK JAMMERS. Every 4096-sample capture of RX1+RX2 (the existing ADC
 *       DMA) goes through a detector (powers + RX1/RX2 coherence,
 *       gnss_npi_det.c): a jammer both antennas see but below the core's
 *       power threshold forces the core ON.
 *    3. RX2 NORMALISATION. Learns the RX2/RX1 noise ratio while there is no
 *       jammer and gives the core g2 = sqrt(N1/N2).
 *    4. MAKE-UP GAIN. While the jammer is nulled, the core's output is
 *       multiplied by 2^s (s = RX gain cut / 6 dB, at most 4).
 *    5. REPORTING. Reads back the core's weights and measures what they do
 *       to each capture (J/N, suppression, residual), prints time-stamped
 *       events, a status line every gnss_aj_log= s, a CSV line every
 *       gnss_aj_csv= ms and the full gnss_report? block.
 *    6. RF PROFILE. gnss_band= (default 1: 1582.5 MHz, 61.44 MS/s, 56 MHz,
 *       which carries GPS, Galileo, QZSS, SBAS, BeiDou B1I+B1C and GLONASS),
 *       AD9361 FIRs off, and a guard that restores it if anything else
 *       changes the LO, the rate or the FIRs.
 *
 *  Runs from the console idle hook (gnss_idle), like the matched RX gain loop.
 *****************************************************************************/
#ifndef GNSS_NPI_AJ_H_
#define GNSS_NPI_AJ_H_

#include <stdint.h>

/* DDR buffer for the captures: 4096 samples x 2 channels x 4 bytes = 32 KiB.
 * Away from the boot captures (0x10000000, 0x10800000; gnss_capture.h). */
#ifndef GNSS_AJ_BUF_ADDR
#define GNSS_AJ_BUF_ADDR   0x11000000U
#endif

#define GNSS_AJ_VERSION_STR  "PL-NPI 2.3.6"

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

/* gnss_aj_mode= */
#define GNSS_AJ_MODE_GATED      0   /* gated by the detectors, image taps (default) */
#define GNSS_AJ_MODE_GATED_NOIMG 1  /* gated, no image taps                        */
#define GNSS_AJ_MODE_ALWAYS     2   /* always filtering and adapting, image taps    */
#define GNSS_AJ_MODE_ALWAYS_NOIMG 3 /* always, no image taps (closest to v2.2)      */
#define GNSS_AJ_NMODES          4

void gnss_aj_init(void);            /* once, after the radio and DMA are up  */
void gnss_aj_enable(int on);        /* core in the TX1 path + engine         */
void gnss_aj_command(int on);       /* the same, by the operator (gnss_npi=) */
int  gnss_aj_enabled(void);
int  gnss_aj_user_off(void);        /* 1 if the operator typed gnss_npi=0    */
void gnss_aj_freeze(int on);        /* weights frozen (CONTROL[6])           */
void gnss_aj_set_mode(int mode);    /* GNSS_AJ_MODE_*                        */
void gnss_aj_set_log(int seconds);  /* periodic status line, 0 = events only */
void gnss_aj_set_events(int on);  /* 0 = no event lines (gnss_report? keeps them) */
void gnss_aj_set_csv(int ms);       /* CSV line period, 0 = off              */
int  gnss_aj_jammer_on(void);       /* the core is filtering a jammer        */
void gnss_aj_poll(void);
void gnss_aj_print(void);           /* gnss_aj? / gnss_npi?                  */
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
