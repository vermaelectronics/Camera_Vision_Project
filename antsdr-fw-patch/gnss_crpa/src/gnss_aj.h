/******************************************************************************
 *  gnss_aj.h  --  anti-jam engine (runs on the ARM; crpa16 / crpa17: the weights
 *                  are calculated by NLMS on the ARM and applied by stap_pi)
 *
 *  WHAT IT DOES, CONTINUOUSLY, WITH NO OPERATOR SETTING
 *    1. Captures 8192 samples of RX1 and RX2 into DDR over the existing ADC
 *       DMA (a fan-out of the RX stream; the PL path is not interrupted).
 *    2. CLEAR (no jammer): the filter is BYPASSED -- TX1 carries RX1 exactly,
 *       4 samples later -- and each capture only goes through a cheap
 *       detector (aj_detect: powers + RX1/RX2 coherence). Meanwhile it learns
 *       the receiver noise floor and the RX2/RX1 level ratio (the input
 *       normalisation g2 the FPGA applies to RX2).
 *    3. JAMMED: the filter is switched in. Per capture the ARM measures the
 *       jammer (J/N, bandwidth), runs an NLMS over the capture (2 passes by
 *       default, continuing from the weights of the previous capture, step
 *       from J/N), commits the 21 / 17 weights with the weight scale that fits
 *       them, sets the output make-up gain and reads the FPGA power meters
 *       (suppression, residual).
 *    4. Back to CLEAR (bypass, weights cleared) once the jammer has been gone
 *       for 0.5 s.
 *    Every change is printed on the console with a time stamp:
 *         [AJ   12.345 s] *** JAMMER ON *** ...
 *         [AJ   12.361 s] NULLED in 16 ms ...
 *         [AJ   45.678 s] jammer OFF after 33.3 s ...
 *    plus a status line every gnss_aj_log= seconds (default 0 = off, held back while the operator types), a CSV line
 *    every gnss_aj_csv= ms (default off) and the full gnss_report? block.
 *
 *  Runs from the console idle hook (gnss_idle), like the matched RX gain loop.
 *****************************************************************************/
#ifndef GNSS_AJ_H_
#define GNSS_AJ_H_

#include <stdint.h>
#include "gnss_aj_math.h"
#if AJ_NT == 8
#define AJ_TAPS_TXT "8 + 8"
#define AJ_PKG_TXT  "crpa16"
#elif AJ_NT == 6
#define AJ_TAPS_TXT "6 + 6"
#define AJ_PKG_TXT  "crpa17"
#else
#define AJ_TAPS_TXT "?"
#define AJ_PKG_TXT  "crpa?"
#endif

/* DDR buffer for the captures: AJ_NSAMP (8192) samples x 2 channels x 4 bytes = 64 KiB.
 * Away from the boot captures (0x10000000, 0x10800000; gnss_capture.h). */
#ifndef GNSS_AJ_BUF_ADDR
#define GNSS_AJ_BUF_ADDR   0x11000000U
#endif

#define GNSS_AJ_VERSION_STR  AJ_PKG_TXT " STAP-PI 4.9.14 (" AJ_TAPS_TXT " taps, 3 + 3 image taps, weights by NLMS on the ARM, no FPGA NLMS, fine gain, clip counters, power meters, low-DSP bitstream, level lock, TX2 = CRPA copy, muted, 89.75 dB)"

/* RF profiles (gnss_band=). RX and TX are always set together, so the
 * retransmitted signal comes out on the frequency it came in on and the
 * board's clock error cancels between down- and up-conversion. */
#define GNSS_BAND_COUNT       8   /* seven fixed presets 0..6 (4.9.5) + 7 = custom (antsdr-fw-patch gnss_crpa) */
#define GNSS_BAND_CUSTOM      7   /* gnss_band=7,<LO>,<rate>,<BW>,<TX1 att>,<RX gain>: set in the field */
#define GNSS_BAND_DEFAULT     0   /* 1569.42 MHz, 61.44 MS/s, 56 MHz, TX att 30 dB, RX gain 50 dB */

void gnss_aj_init(void);            /* once, after the radio and DMA are up  */
void gnss_aj_enable(int on);        /* filter in the TX1 path + engine       */
void gnss_aj_command(int on);       /* the same, by the operator (gnss_smi=) */
int  gnss_aj_enabled(void);
int  gnss_aj_user_off(void);        /* 1 if the operator typed gnss_smi=0    */
void gnss_aj_freeze(int on);        /* hold the present weights (NLMS stops) */
void gnss_aj_set_blank(int mode);           /* pulse blanker: 0 off, 1 auto (default), 2 on (bitstream 4.6) */
void gnss_aj_set_blank_db(double db);       /* blanker threshold above the output noise [dB], 3..30      */
int  gnss_aj_blank_available(void);         /* 1 = the bitstream has the blanker                          */
void gnss_aj_set_log(int seconds);  /* periodic status line, 0 = events only */
void gnss_aj_set_events(int on);  /* 0 = no event lines (gnss_report? keeps them) */
void gnss_aj_set_csv(int ms);
/* crpa16/17: NLMS on the ARM (gnss_aj_nlms=, gnss_aj_nlms_mu=, gnss_aj_nlms_pass=, gnss_aj_nlms?).
 * on: -1 = leave, 1 = adapt (default), 0 = hold the weights; alpha: < 0 = leave,
 * 0 = automatic (from J/N), > 0 = manual step; passes: <= 0 = leave, 1..8 passes per capture */
void gnss_aj_set_nlms(int on, double alpha, int passes);
int  gnss_aj_nlms(void);            /* 1 = adapting */
double gnss_aj_nlms_mu(void);       /* the manual step, 0 = automatic */
int  gnss_aj_nlms_pass(void);
void gnss_aj_nlms_line(void);
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
/* band 7 (custom): the values live in the band table; gnss_band7.c parses,
 * checks and saves them (/mnt/jffs2/gnss_crpa.conf). */
void gnss_band_custom_set(uint64_t lo, uint32_t fs, uint32_t bw, uint32_t att_mdb, int32_t rxg_db);
void gnss_band_custom_get(uint64_t *lo, uint32_t *fs, uint32_t *bw, uint32_t *att_mdb, int32_t *rxg_db);
/* crpa5: gnss_rx_agc= is saved with the band 7 values and restored at start-up; crpa16/17: and
 * gnss_aj_nlms_mu= (ARM NLMS step x 1e6, 0 = automatic), gnss_aj_nlms_pass=. -1 = leave. */
void gnss_settings_save(int rx_agc, int mu_e6, int passes);
void gnss_settings_apply(void);      /* after gnss_aj_enable(1) */
int  gnss_band_get(void);
void gnss_band_levels(void);
void gnss_levels_adopt(void);        /* 4.9.13: the present RX gain / TX attenuation become the locked values */
uint32_t gnss_levels_repairs(void);  /* how often the level lock had to put them back */
void gnss_aj_csv_once(void);                 /* the AJCSV header + one line (for the snapshot log) */                 /* the current band's TX attenuation (TX1, TX2) and RX gain */
void gnss_band_print(void);
void gnss_radio_cmd(double *p, char n);   /* radio=: LO, rate, attenuation, gain, bandwidth by hand */
char *gnss_fmt(double v, int dec);        /* "1575.4200" without printf's %f */
void gnss_radio_print(void);              /* radio? */
/* unit helpers (value in MHz / MS/s or in Hz is recognised by size); 0 = ok, -1 = out of range */
int  gnss_radio_lo_hz(double v, uint64_t *hz);
int  gnss_radio_rate_hz(double v, uint32_t *hz);
int  gnss_radio_bw_hz(double v, uint32_t *hz);
int  gnss_radio_att_mdb(double db, uint32_t *mdb);
void gnss_aj_radio_manual(void);   /* rx/tx_lo_freq=, rx/tx_samp_freq=, rx/tx_fir_en=: keep what the operator set */

/* The console idle hook: matched RX gain loop + anti-jam engine + radio
 * guard (keeps the gnss_band= profile and the AD9361 FIRs off). */
void gnss_idle(void);

#endif
