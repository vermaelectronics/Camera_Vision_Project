/******************************************************************************
 *  gnss_aj.h  --  SFAP-PI v6.3 anti-jam engine, 256 bins (runs on the ARM)
 *
 *  WHAT IT DOES, CONTINUOUSLY, WITH NO OPERATOR SETTING
 *    1. Captures AJ_NSAMP (v6.2: 4096) samples of RX1 and RX2 into DDR over the existing ADC
 *       DMA (a fan-out of the RX stream; the PL path is not interrupted).
 *    2. CLEAR (no jammer): the filter is BYPASSED -- TX1 carries RX1 exactly,
 *       782 samples later -- and each capture only goes through a cheap
 *       detector (aj_detect: powers + RX1/RX2 coherence + v6.1 per-bin
 *       narrowband check). Meanwhile it learns the receiver noise floor (also
 *       per bin, v6.1) and the RX2/RX1 level ratio (the input normalisation
 *       g2 the FPGA applies to RX2).
 *    3. JAMMED: from the capture that detected the jammer on, every capture
 *       is solved (aj_compute: the FPGA's 256-bin filter bank in double, per-
 *       bin statistics, one weight + 2 image taps per bin, loading = each
 *       bin's own noise estimate, Cholesky in double precision) and the
 *       3 x 256 weights are committed to the FPGA filter
 *       together with cfg (filter in, output make-up shift) and g2.
 *       v6.1: statistics averaging adapts (0.8 while nulled, 0.25 while the
 *       residual is high); bins a jammer visited keep their weights for ~1 s
 *       (weight hold, sfap-pi_hold=); an RX gain step no longer drops them.
 *    4. Back to CLEAR (bypass) once the jammer has been gone for 0.5 s (v6.1:
 *       including the per-bin check, so a narrowband jammer keeps it on).
 *    v6.3: every capture is first BLANKED like the FPGA blanks the stream
 *       (threshold 5 x the capture's robust sigma, committed in AJ_CFG), so
 *       pulses never enter the statistics; pulses alone (> 0.5 % blanked)
 *       also switch the filter in (it then carries the blanked RX1). After
 *       each solve the bins that still hold two jammers from two directions
 *       are EXCISED (mask committed with the weights). The gain loop ignores
 *       the blanked pulses, so they no longer pull the RX gain down.
 *    Every change is printed on the console with a time stamp:
 *         [AJ   12.345 s] *** JAMMER ON *** ...
 *         [AJ   12.361 s] NULLED in 16 ms ...
 *         [AJ   45.678 s] jammer OFF after 33.3 s ...
 *    plus a status line every sfap-pi_log= seconds (default 30, held back while the operator types), a CSV line
 *    every sfap-pi_csv= ms (default off) and the full sfap-pi_report? block.
 *
 *  Runs from the console idle hook (gnss_idle), like the matched RX gain loop.
 *****************************************************************************/
#ifndef GNSS_AJ_H_
#define GNSS_AJ_H_

#include <stdint.h>

/* DDR buffer for the captures: AJ_NSAMP samples x 2 channels x 4 bytes
 * (v6.2: 4096 -> 32 KiB; the 64 KiB reserved for v6.0's 8192 still fits).
 * Away from the boot captures (0x10000000, 0x10800000; gnss_capture.h). */
#ifndef GNSS_AJ_BUF_ADDR
#define GNSS_AJ_BUF_ADDR   0x11000000U
#endif

#define GNSS_AJ_VERSION_STR  "6.3a"   /* 6.3 + gnss_band=7 (1568.5 MHz) and 8 (MANUAL) */

/* RF profiles (gnss_band=). RX and TX are always set together, so the
 * retransmitted signal comes out on the frequency it came in on and the
 * board's clock error cancels between down- and up-conversion. */
#define GNSS_BAND_L1          0   /* 1575.42 MHz, 30.72 MS/s, 18 MHz: L1 only      */
#define GNSS_BAND_ALL         1   /* 1582.5 MHz, 61.44 MS/s, 56 MHz: GPS, Galileo, */
                                  /*   QZSS, SBAS, BeiDou B1I+B1C, GLONASS G1      */
#define GNSS_BAND_NO_BDS      2   /* 1589.5 MHz, 50 MS/s: GPS, Galileo, GLONASS    */
#define GNSS_BAND_NO_GLO      3   /* 1568.25 MHz, 50 MS/s: GPS, Galileo, BeiDou    */
#define GNSS_BAND_L1_AJ       4   /* v6.1: 1575.42 MHz, 30.72 MS/s, 8 MHz: L1 C/A,*/
                                  /*   E1, B1C main lobes only (anti-jam narrow) */
#define GNSS_BAND_L1_MP       5   /* v6.2: 1575.42 MHz, 15.36 MS/s, 6 MHz: L1 C/A,*/
                                  /*   E1, B1C; 4x the multipath delay spread the */
                                  /*   256 bins tolerate at 61.44 MS/s            */
#define GNSS_BAND_CUSTOM      6   /* v6.2.2: any LO / rate / bandwidth, set with  */
                                  /* gnss_band_custom=LO_MHz FS_MSPS BW_MHz       */
#define GNSS_BAND_1568_20     7   /* v6.3a: 1568.5 MHz, 30.72 MS/s, 20 MHz,       */
                                  /*   RX gain 40 dB, TX1 attenuation 30 dB      */
#define GNSS_BAND_MANUAL      8   /* v6.3a: MANUAL preset, values below; changed */
                                  /*   live with gnss_band_manual=LO FS BW RXG ATT */
#define GNSS_BAND_COUNT       9
#define GNSS_BAND_DEFAULT     GNSS_BAND_ALL

/* v6.3a: start values of the MANUAL preset (gnss_band=8). Edit here before
 * building to change what gnss_band=8 selects after every boot;
 * gnss_band_manual= changes them at run time (not kept over a reboot).
 * RX gain 0 = leave the RX gain as it is; TX1 attenuation 0 = leave it
 * (a profile never asks for 0 dB, the loudest setting). */
#ifndef GNSS_BAND_MANUAL_LO_HZ
#define GNSS_BAND_MANUAL_LO_HZ     1575420000ULL   /* LO [Hz], RX and TX       */
#endif
#ifndef GNSS_BAND_MANUAL_FS_HZ
#define GNSS_BAND_MANUAL_FS_HZ       30720000U     /* sample rate [S/s]        */
#endif
#ifndef GNSS_BAND_MANUAL_BW_HZ
#define GNSS_BAND_MANUAL_BW_HZ       18000000U     /* RF bandwidth [Hz]        */
#endif
#ifndef GNSS_BAND_MANUAL_RXGAIN_DB
#define GNSS_BAND_MANUAL_RXGAIN_DB   0             /* 1..76 dB, 0 = unchanged  */
#endif
#ifndef GNSS_BAND_MANUAL_TXATT_MDB
#define GNSS_BAND_MANUAL_TXATT_MDB   0             /* mdB, 0 = unchanged       */
#endif

void gnss_aj_init(void);            /* once, after the radio and DMA are up  */
void gnss_aj_enable(int on);        /* filter in the TX1 path + engine       */
void gnss_aj_command(int on);       /* the same, by the operator (sfap-pi=) */
int  gnss_aj_enabled(void);
int  gnss_aj_user_off(void);        /* 1 if the operator typed sfap-pi=0    */
void gnss_aj_freeze(int on);        /* keep the present weights              */
void gnss_aj_set_mode(int mode);    /* AJ_MODE_SFAP / _LIN / _NB             */
void gnss_aj_set_log(int seconds);  /* periodic status line, 0 = events only */
void gnss_aj_set_csv(int ms);       /* CSV line period, 0 = off              */
/* v6.3: pulse blanker and bin excision, 1 = on (default) */
void gnss_aj_set_blank(int on);
int  gnss_aj_get_blank(void);
void gnss_aj_set_excise(int on);
int  gnss_aj_get_excise(void);
/* v6.3: the matched gain loop stepped both RX gains by delta_db; the blanker
 * threshold (ADC LSB) is rescaled and re-committed at once, so a gain step
 * between two captures neither blanks the continuous signal nor lets pulses
 * through. */
void gnss_aj_gain_stepped(int delta_db);
int  gnss_aj_jammer_on(void);
void gnss_aj_poll(void);
void gnss_aj_print(void);           /* sfap-pi_aj? / sfap-pi?                  */
void gnss_aj_print_short(void);
void gnss_aj_report(void);          /* sfap-pi_report?: everything, for a copy  */
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
/* v6.3a: MANUAL preset (gnss_band=8): LO, rate, bandwidth, RX gain [dB]
 * (0 = unchanged) and TX1 attenuation [dB] (0 = unchanged), applied at once.
 * Returns 0, -1 out of range (nothing changed), -2 refused by the AD9361. */
int  gnss_band_set_manual(double lo_mhz, double fs_msps, double bw_mhz,
                          double rxgain_db, double txatt_db);
/* v6.3a: TX1 attenuation [mdB] the active profile asks for, or 0 if it
 * leaves TX1 attenuation alone (gnss_tx=1 then uses its own start level). */
int32_t gnss_band_tx_att_mdb(void);
int  gnss_band_get(void);
void gnss_band_print(void);

/* The console idle hook: matched RX gain loop + anti-jam engine + radio
 * guard (keeps the gnss_band= profile and the AD9361 FIRs off). */
void gnss_idle(void);

#endif
