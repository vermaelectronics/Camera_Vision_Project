/*
 * gnss_npi_agc.h -- matched RX gain control, automatic gamma and TX level
 *                  hold for PL-NPI.
 *
 * WHY
 *   The AD9361's own AGC runs RX1 and RX2 independently. When a jammer
 *   appears it turns each channel down at a different moment; every unequal
 *   step changes the RX1/RX2 ratio the PL-NPI weights were adapted to, and
 *   the null is lost until the weights re-converge (C model with the real
 *   HLS core, 1500 LSB jammer: worst residual -20 dB with unequal 6 dB
 *   steps, -55 dB with equal steps; -40 dB reached 33 us after switch-on). The lower gain also lowers the level that is retransmitted,
 *   so the satellites look weaker at the receiver.
 *
 *   gamma (diagonal loading) was a fixed console value, 1 at power-on. At 1
 *   PL-NPI cancels the receiver noise and the satellites under it with it
 *   (C model: satellites -39 dB, i.e. none visible). The right value depends
 *   on the noise power, which changes with every gain step.
 *
 * WHAT (mode GNSS_RXG_MATCHED, the default with gnss_npi=1)
 *   - Both channels in manual gain, ALWAYS at the same value, so the PL-NPI
 *     weights is not disturbed by gain changes.
 *   - Every 5 ms the firmware reads 256 RX samples of each channel from the
 *     gnss_passthrough snapshot registers. Peak above GNSS_RXG_PEAK_HIGH
 *     (close to ADC clipping): both gains down together, by up to 12 dB per
 *     step (fast attack: a jammer 80 dB above the noise is absorbed in
 *     ~20 ms). Peak below
 *     GNSS_RXG_PEAK_LOW for 20 ms: both up in one step sized from the peak
 *     (fast release, ~100 ms back to the reference), never above the reference
 *     gain (the clean, no-jammer gain captured when the loop starts).
 *   - TX level hold: while the gain is G dB below the reference, TX1
 *     attenuation is lowered by G dB (at most GNSS_RXG_MAX_COMP_MDB), so the
 *     retransmitted GNSS level stays where it was before the jammer. Only
 *     while TX1 is transmitting, PL-NPI is on and the output is NOT louder
 *     than the clean reference output (i.e. the jammer is really nulled);
 *     otherwise the operator's attenuation is restored.
 *   - Automatic gamma: gamma = 2^17 * k * noise power per channel, with k
 *     itself automatic: 32 while no jammer is seen (satellites pass almost
 *     untouched), 1/8 as soon as one is (deepest null); see gnss_npi_agc.c.
 *     The noise power is measured from the same snapshots, followed only
 *     while the input is within 3 dB of it (no jammer) and rescaled with each
 *     gain step, so a jammer never pulls gamma up.
 *     v2.3, gated adaptation (the default): the core only adapts while a
 *     jammer is present, so k is always the jammer value 1/8.
 *   - v2.3: both channels are checked every 200 ms and put back to the same
 *     manual gain if anything else (GUI, vendor commands) changed them.
 */
#ifndef GNSS_NPI_AGC_H_
#define GNSS_NPI_AGC_H_

#include <stdint.h>

#define GNSS_RXG_MANUAL   0   /* both fixed at one gain, no loop          */
#define GNSS_RXG_AD9361   1   /* AD9361 slow AGC per channel (old default) */
#define GNSS_RXG_MATCHED  2   /* matched loop + TX level hold             */

/* v2.4: the gain policy gnss_aj_enable applies at start-up (once), the
 * deployment's configuration: the AD9361 slow-attack AGC (gc mode 2) on both
 * channels, starting from 40 dB. gnss_rx_agc=2 switches to the matched loop
 * (equal gains, TX level hold, gain-cut detection). Both overridable with
 * build_all.sh --defines. */
#ifndef GNSS_RXG_DEFAULT_MODE
#define GNSS_RXG_DEFAULT_MODE  GNSS_RXG_AD9361
#endif
#ifndef GNSS_RXG_AGC_START_DB
#define GNSS_RXG_AGC_START_DB  40
#endif

#define GNSS_RXG_PEAK_HIGH     1850   /* of 2047: back off before clipping  */
#define GNSS_RXG_PEAK_LOW      1000   /* both below this: room to go up     */
#define GNSS_RXG_MAX_COMP_MDB 50000U  /* TX level hold limited to 50 dB     */
#define GNSS_RXG_MIN_ATT_MDB  30000U  /* hold never below 30 dB attenuation */
#define GNSS_RXG_REF_MIN_DB       64  /* clean reference never starts lower */

void    gnss_rxg_set_mode(int mode);   /* starts/stops the loop             */
int     gnss_rxg_mode(void);
void    gnss_rxg_set_gain(int32_t db); /* both channels; also the reference */
void    gnss_rxg_stop_tx_hold(void);   /* operator attenuation back         */
void    gnss_rxg_poll(void);           /* console idle hook                 */
void    gnss_rxg_print(void);
void    gnss_npi_gamma_auto(int on);   /* 1 = automatic gamma (default)     */
int     gnss_npi_gamma_is_auto(void);
void    gnss_npi_gamma_k(uint32_t k);  /* 0 = automatic k (default), 1..64 fixed */
void    gnss_npi_set_gated(int on);    /* 1 = core adapts only with a jammer:  */
                                       /*     automatic k is then always 1/8  */

/* For the anti-jam engine (gnss_npi_aj.c). */
int32_t  gnss_rxg_gain(void);          /* present RX gain [dB]              */
int32_t  gnss_rxg_ref(void);           /* clean reference gain [dB]         */
uint32_t gnss_rxg_changes(void);       /* counts every gain step            */
int      gnss_rxg_peak(void);          /* last ADC peak of both channels    */
uint32_t gnss_rxg_att_base(void);      /* operator's TX1 attenuation [mdB]  */
int      gnss_rxg_comp_db(void);       /* TX level hold in effect [dB]      */
uint32_t gnss_npi_k_eighths(void);     /* k in use, in eighths (1 = 1/8)    */
/* v2.4: gamma from the engine's noise floor n1 [LSB^2, RX1, present gain]
 * in the modes without the matched loop (which measures its own). */
void     gnss_npi_gamma_noise(double n1, int jammed);
const char *gnss_rxg_mode_txt(void);

#endif
