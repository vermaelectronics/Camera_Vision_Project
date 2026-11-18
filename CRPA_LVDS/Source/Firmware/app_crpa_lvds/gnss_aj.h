/******************************************************************************
 *  gnss_aj.h  --  CRPA anti-jam engine (runs on the ARM), one CRPA algorithm
 *  per build: crpa1 (SMI-PI, smi_pi core) or crpa27..31 (STAP-NLMS, stap_pi
 *  core, 8 + 8 .. 18 + 18 taps). The algorithm's solver is the
 *  gnss_aj_math.{c,h} build_all.sh --algo <algo> copies next to this file.
 *
 *  WHAT IT DOES, CONTINUOUSLY, WITH NO OPERATOR SETTING
 *    1. Captures AJ_NSAMP (8192) samples of RX1 and RX2 into DDR over the
 *       existing ADC DMA (a fan-out of the RX stream; the PL path is not
 *       interrupted).
 *    2. CLEAR (no jammer): the filter is BYPASSED -- TX1 carries RX1 exactly,
 *       REF samples later (the same delay as the filter) -- and each capture
 *       only goes through the detector (aj_detect: powers + RX1/RX2
 *       coherence). Meanwhile it learns the receiver noise floor and the
 *       RX2/RX1 level ratio (the input normalisation g2 the FPGA applies).
 *    3. JAMMED: from the capture that detected the jammer on, every capture
 *       is solved and the space-time weights are committed to the FPGA
 *       filter together with cfg (filter in, weight scale, make-up shift),
 *       g2 and (stap_pi) the fine output gain:
 *         crpa1      aj_compute (SMI, averaged statistics, Cholesky);
 *         crpa27..31 aj_direct_solve on the first capture, then aj_nlms_solve
 *                    on every capture; aj_track_* re-solves directly when the
 *                    null is lost, aj_pred_* commits weights predicted ahead
 *                    while a moving jammer leaves the held weights behind.
 *    4. Back to CLEAR (bypass) once the jammer has been gone for 0.5 s.
 *    Every change is printed on the console with a time stamp:
 *         [AJ   12.345 s] *** JAMMER ON *** ...
 *         [AJ   12.361 s] NULLED in 16 ms ...
 *         [AJ   45.678 s] jammer OFF after 33.3 s ...
 *    plus a status line every crpa_log= seconds (default 30, held back while
 *    the operator types), a CSV line every crpa_csv= ms (default off) and the
 *    full crpa_report? block.
 *
 *  Runs from the console idle hook (gnss_idle), like the matched RX gain loop.
 *****************************************************************************/
#ifndef GNSS_AJ_H_
#define GNSS_AJ_H_

#include <stdint.h>

/* DDR buffer for the captures: AJ_NSAMP (8192) samples x 2 channels x 4
 * bytes = 64 KiB. Away from the boot captures (0x10000000, 0x10800000;
 * gnss_capture.h). */
#ifndef GNSS_AJ_BUF_ADDR
#define GNSS_AJ_BUF_ADDR   0x11000000U
#endif

#define GNSS_AJ_VERSION_STR  "7.0"

/* RF profiles (gnss_band=). RX and TX are always set together, so the
 * retransmitted signal comes out on the frequency it came in on and the
 * board's clock error cancels between down- and up-conversion. */
#define GNSS_BAND_L1          0   /* 1575.42 MHz, 30.72 MS/s, 18 MHz: L1 only      */
#define GNSS_BAND_ALL         1   /* boot profile: gnss_boot_config.h (1568.5 MHz, 30.72 MS/s, 20 MHz); was 1582.5 MHz, 61.44 MS/s, 56 MHz: GPS, Galileo, */
                                  /*   QZSS, SBAS, BeiDou B1I+B1C, GLONASS G1      */
#define GNSS_BAND_NO_BDS      2   /* 1589.5 MHz, 50 MS/s: GPS, Galileo, GLONASS    */
#define GNSS_BAND_NO_GLO      3   /* 1568.25 MHz, 50 MS/s: GPS, Galileo, BeiDou    */
#define GNSS_BAND_L1_AJ       4   /* v6.1: 1575.42 MHz, 30.72 MS/s, 8 MHz: L1 C/A,*/
                                  /*   E1, B1C main lobes only (anti-jam narrow) */
#define GNSS_BAND_L1_MP       5   /* v6.2: 1575.42 MHz, 15.36 MS/s, 6 MHz: L1 C/A,*/
                                  /*   E1, B1C; 4x the multipath delay the taps  */
                                  /*   reach at 61.44 MS/s                        */
#define GNSS_BAND_CUSTOM      6   /* v6.2.2: any LO / rate / bandwidth, set with  */
                                  /* gnss_band_custom=LO_MHz FS_MSPS BW_MHz       */
#define GNSS_BAND_COUNT       7
#define GNSS_BAND_DEFAULT     GNSS_BAND_ALL

void gnss_aj_init(void);            /* once, after the radio and DMA are up  */
void gnss_aj_enable(int on);        /* filter in the TX1 path + engine       */
void gnss_aj_command(int on);       /* the same, by the operator (crpa=)     */
int  gnss_aj_enabled(void);
int  gnss_aj_user_off(void);        /* 1 if the operator typed crpa=0        */
void gnss_aj_freeze(int on);        /* keep the present weights              */
/* Algorithm settings. Return 0 = set, -1 = value out of range, -2 = not in
 * this algorithm (crpa_mode= is crpa1 only; direct / track / predict /
 * passes are crpa27..31 only). */
int  gnss_aj_set_mode(int mode);    /* crpa1: AJ_MODE_STAP / _RX2 / _NB / _LIN */
int  gnss_aj_set_direct(int n);     /* direct solves at a jammer's start      */
int  gnss_aj_set_track(int on);     /* moving-jammer re-solve                 */
int  gnss_aj_set_predict(int on);   /* weight prediction                      */
int  gnss_aj_set_passes(int n);     /* NLMS passes per capture                */
void gnss_aj_set_log(int seconds);  /* periodic status line, 0 = events only */
void gnss_aj_set_csv(int ms);       /* CSV line period, 0 = off              */
/* the matched gain loop stepped both RX gains by delta_db (hook kept for
 * gnss_smi_agc.c; the space-time weights stay valid) */
void gnss_aj_gain_stepped(int delta_db);
int  gnss_aj_jammer_on(void);
void gnss_aj_poll(void);
void gnss_aj_print(void);           /* crpa_aj?                               */
void gnss_aj_print_short(void);     /* crpa?                                  */
void gnss_aj_report(void);          /* crpa_report?: everything, for a copy   */
void gnss_aj_note_tx(int on);       /* gnss_tx= tells the engine (time stamp) */

/* Waits for the engine's capture in flight (if any) and leaves the RX DMA
 * free. Call before anything else submits a transfer on rx_dmac. */
void gnss_aj_dma_release(void);

/* RF profile; returns 0 on success. */
int  gnss_band_set(int band);
/* v6.2.2: set and apply profile 6 (CUSTOM). Returns 0 if applied, -1 if a value
 * is out of range (nothing changed), -2 if the AD9361 refused it (the previous
 * profile is put back). */
int  gnss_band_set_custom(double lo_mhz, double fs_msps, double bw_mhz);
int  gnss_band_get(void);
void gnss_band_print(void);

/* The console idle hook: matched RX gain loop + anti-jam engine + radio
 * guard (keeps the gnss_band= profile and the AD9361 FIRs off). */
void gnss_idle(void);

#endif
