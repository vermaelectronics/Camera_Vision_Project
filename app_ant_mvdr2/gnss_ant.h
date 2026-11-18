/***************************************************************************//**
 *   @file   gnss_ant.h
 *   @brief  Console of the LVDS / ant board (see gnss_ant.c).
 *
 *     boot      gnss_tx=0 pass=0 stap=0  (TX1 silent, radio plan applied)
 *     gnss_tx=1 TX1 on, muted            pass=1  raw RX1 (gnss_tx=1, stap=0)
 *     stap=1    anti-jam (gnss_tx=1, pass=1)    stap=0  back to raw RX1
*******************************************************************************/
#ifndef GNSS_ANT_H_
#define GNSS_ANT_H_

void gnss_ant_boot(void);
void gnss_ant_menu(void);
int  gnss_ant_poll_events(void);	/* prints new jammer/null events; 1 = printed */

int  gnss_ant_tx_state(void);
int  gnss_ant_pass_state(void);
int  gnss_ant_stap_state(void);

/* control */
void set_gnss_tx(double *param, char param_no);
void set_pass(double *param, char param_no);
void set_stap(double *param, char param_no);
/* pages */
void get_ant_help(double *param, char param_no);
void get_ant_status(double *param, char param_no);
void get_rx_info(double *param, char param_no);
void get_stap(double *param, char param_no);
void get_selftest(double *param, char param_no);
void get_ant_kpi(double *param, char param_no);
void get_ant_gnss_tx(double *param, char param_no);      /* gnss_tx?      */
void get_ant_gnss_status(double *param, char param_no);  /* gnss_status?  */
void get_ant_jam(double *param, char param_no);          /* gnss_jam?     */
void set_agcstep(double *param, char param_no);          /* gnss_agcstep= */
/* settings */
void get_band(double *param, char param_no);
void set_band(double *param, char param_no);
void set_tx1_att(double *param, char param_no);
void set_agc(double *param, char param_no);
void set_rxgain(double *param, char param_no);
void set_smi(double *param, char param_no);
void set_smiavg(double *param, char param_no);
void get_taps(double *param, char param_no);
void get_rxoff(double *param, char param_no);
void set_rxoff(double *param, char param_no);
void set_agcclip(double *param, char param_no);
void set_ajbw(double *param, char param_no);
void set_taps(double *param, char param_no);

#endif
