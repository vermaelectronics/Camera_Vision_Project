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
 *   - Every ~20 ms the firmware reads 512 RX samples of each channel from the
 *     gnss_passthrough snapshot registers. Peak above GNSS_RXG_PEAK_HIGH
 *     (close to ADC clipping): both gains down together. Peak below
 *     GNSS_RXG_PEAK_LOW for 60 ms: both up 1 dB, never above the reference
 *     gain (the clean, no-jammer gain captured when the loop starts).
 *   - TX level hold: while the gain is G dB below the reference, TX1
 *     attenuation is lowered by G dB (at most GNSS_RXG_MAX_COMP_MDB), so the
 *     retransmitted GNSS level stays where it was before the jammer. Only
 *     while TX1 is transmitting, PL-NPI is on and the output is NOT louder
 *     than the clean reference output (i.e. the jammer is really nulled);
 *     otherwise the operator's attenuation is restored.
 *   - Automatic gamma: gamma = 2^17 * k * noise power per channel (k = 4).
 *     The noise power is measured from the same snapshots, followed only
 *     while the input is within 3 dB of it (no jammer) and rescaled with each
 *     gain step, so a jammer never pulls gamma up.
 */
#ifndef GNSS_NPI_AGC_H_
#define GNSS_NPI_AGC_H_

#include <stdint.h>

#define GNSS_RXG_MANUAL   0   /* both fixed at one gain, no loop          */
#define GNSS_RXG_AD9361   1   /* AD9361 slow AGC per channel (old default) */
#define GNSS_RXG_MATCHED  2   /* matched loop + TX level hold             */

#define GNSS_RXG_PEAK_HIGH     1700   /* of 2047: back off before clipping  */
#define GNSS_RXG_PEAK_LOW       600   /* both below this: room to go up     */
#define GNSS_RXG_MAX_COMP_MDB 30000U  /* TX level hold limited to 30 dB     */

void    gnss_rxg_set_mode(int mode);   /* starts/stops the loop             */
int     gnss_rxg_mode(void);
void    gnss_rxg_set_gain(int32_t db); /* both channels; also the reference */
void    gnss_rxg_stop_tx_hold(void);   /* operator attenuation back         */
void    gnss_rxg_poll(void);           /* console idle hook                 */
void    gnss_rxg_print(void);
void    gnss_npi_gamma_auto(int on);   /* 1 = automatic gamma (default)     */
int     gnss_npi_gamma_is_auto(void);
void    gnss_npi_gamma_k(uint32_t k);  /* gamma = 2^17 * k * noise, k 1..64 */

#endif
