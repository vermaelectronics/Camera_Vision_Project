/***************************************************************************//**
 *   @file   gnss_ant.h
 *   @brief  Operator layer for the LVDS / ant board: the three switches
 *           gnss_tx, pass, stap with their interlocks, the main menu, and
 *           short aliases. Everything underneath is the MVDR2 firmware.
 *
 *     boot      gnss_tx=0 pass=0 stap=0  (TX1 silent, radio plan applied)
 *     gnss_tx=1 TX1 on at the operating attenuation, no signal (muted)
 *     pass=1    -> gnss_tx=1, stap=0     raw RX1 on TX1
 *     stap=1    -> gnss_tx=1, pass=1     MVDR2 automatic start: RX gain
 *                                        level, AOLC, SMI-STAP null, TX1 on
 *     stap=0    back to pass=1 (raw RX1, TX1 stays on)
 *     pass=0    stap=0 too, TX1 stays on (muted)
 *     gnss_tx=0 everything off, TX1 silent
*******************************************************************************/
#ifndef GNSS_ANT_H_
#define GNSS_ANT_H_

void gnss_ant_boot(void);
void gnss_ant_menu(void);

int  gnss_ant_tx_state(void);	/* 1 = TX1 chain on (hardware read-back)   */
int  gnss_ant_pass_state(void);	/* 1 = RX path reaches TX1 (pass_en, unmuted) */
int  gnss_ant_stap_state(void);	/* 1 = anti-jam output on TX1 (mvdr_sel)    */

void set_gnss_tx(double *param, char param_no);		/* gnss_tx=  */
void get_gnss_tx(double *param, char param_no);		/* gnss_tx?  */
void set_pass(double *param, char param_no);		/* pass=     */
void get_pass(double *param, char param_no);		/* pass?     */
void set_stap(double *param, char param_no);		/* stap=     */
void get_stap(double *param, char param_no);		/* stap?     */
void get_ant_status(double *param, char param_no);	/* status?   */
void get_rx_info(double *param, char param_no);		/* rx?       */
void get_noise(double *param, char param_no);		/* noise?    */
void set_noise(double *param, char param_no);		/* noise=G   */
void get_guide(double *param, char param_no);		/* guide?    */

#endif
