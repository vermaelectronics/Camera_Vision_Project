/***************************************************************************//**
 *   @file   gnss_report.h
 *   @brief  GNSS-CRPA MOD-26: anti-jam report (gnss_report?), event log and
 *           the SMI-based jammer / suppression figures.
 *
 * The figures come from the SMI window itself (the same covariance the
 * weights are solved from), so they are there whenever SMI runs:
 *   P_in  = RX1 reference-tap power            N   = thermal floor per element
 *   P_out = w^H R w                            N_out = N x sum |w|^2
 *   J/N   = (P_in - N) / N                     residual = (P_out - N_out) / N_out
 *   suppression = jammer in / jammer out = (P_in - N) / (P_out - N_out)
 * The state machine (CLEAR / JAMMED, time to null, events) runs on every
 * solved window (gnss_rep_on_solve, called by gnss_smi_poll).
 ******************************************************************************/
#ifndef GNSS_REPORT_H_
#define GNSS_REPORT_H_

#include <stdint.h>

typedef struct {
    int     valid;          /* a window has been solved and a floor is known     */
    int     jammed;         /* state: jammer present (hysteresis)                */
    double  since_s;        /* time of the last state change                     */
    double  nf_dbfs;        /* thermal floor per element at the present gain     */
    int     nf_src;         /* 1 SMI+learned, 2 learned, 3 SMI estimate          */
    double  pin_dbfs;       /* RX1 reference-tap power                           */
    double  jin_dbfs;       /* jammer at the ADC (-200 = none)                   */
    double  jnr_db;         /* J/N at the ADC                                    */
    double  jnr_peak_db;    /* peak J/N of this jammer                           */
    double  res_db;         /* residual vs output noise (-99 none)               */
    int     res_lo;         /* residual > 10 dB under the noise (bound)          */
    double  supp_db;        /* suppression = jammer in / jammer out              */
    int     supp_lo;        /* supp_db is a lower bound                          */
    double  noise_cost_db;  /* 10 log10 sum |w|^2                                */
    int     have_clean;     /* the floor without jammer is known                 */
    int32_t gain_db, clean_gain_db;
    double  jnr_ant_db;     /* jammer vs the ANTENNA noise (gain cut included)   */
    double  rise_db;        /* floor now vs antenna noise at this gain: C/N0 cost */
    double  t_null_ms;      /* time to null of this jammer (-1 = not yet)        */
    uint32_t n_solves;      /* windows seen                                      */
} gnss_rep_aj_t;

void   gnss_rep_init(void);                      /* boot: uptime 0              */
double gnss_rep_uptime_s(void);
void   gnss_rep_event(const char *fmt, ...);     /* one line, no '\n' needed    */
void   gnss_rep_get(gnss_rep_aj_t *a);
int    gnss_rep_jammed(void);                    /* 1 = JAMMED                  */
void   gnss_rep_print(void);                     /* gnss_report?                */
void   gnss_rep_print_supp(void);                /* JAMMER + SUPPRESSION only   */
/* one compact field for the STATUS line ("jam no" / "J/N +36 supp 46 res -10") */
void   gnss_rep_status_field(void);
/* MOD-28: gnss_kpi? -- all report figures on one key=value line (scripts) */
void   gnss_rep_print_kpi(void);
/* the GNSS signals in the band now (also used by gnss_band?) */
void   gnss_rep_print_signals(int with_response);
/* MOD-30: entry i of the signal table (0 = past the end) */
int    gnss_rep_signal(int i, const char **name, double *mhz, double *hw);
/* MOD-30: hold the CLEAR/JAMMED state machine while a diagnostic changes the
 * gain (1 = start, 0 = end; it stays held 0.5 s after the end) */
void   gnss_rep_hold(int on);

#endif /* GNSS_REPORT_H_ */
