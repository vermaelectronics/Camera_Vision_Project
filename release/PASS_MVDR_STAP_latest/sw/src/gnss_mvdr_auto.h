/******************************************************************************
 *  gnss_mvdr_auto.h   --  GNSS-CRPA MOD-25
 *  Fully automatic noise-subspace steering for the 2-channel spatial MVDR.
 *
 *  No angle, no protractor, no gnss_jcal=1 per power-up:
 *    * the RX1/RX2 covariance is read from the MVDR window, averaged and
 *      normalised by its trace,
 *    * its NOISE eigenvector (smaller eigenvalue = orthogonal to the jammer)
 *      is written as the steering vector A1/A2,
 *    * the null angle (jammer) and the main-lobe angle (where the array gain
 *      now points) are reported from the jammer estimator geometry.
 *
 *  With no jammer the noise eigenvector is undefined (R ~ sigma^2 I), so the
 *  module falls back to:
 *    mode 1 : power inversion [1,0]  -- omni, every satellite kept (default)
 *    mode 2 : broadside beam          -- +3 dB at broadside, nulls endfire
 *
 *  HOOK REQUIRED: call gnss_mvdr_auto_poll() from the periodic background
 *  tick (gnss_background_tick() / the main loop). It does nothing while the
 *  mode is 0, so it is safe to call always.
 *****************************************************************************/
#ifndef GNSS_MVDR_AUTO_H_
#define GNSS_MVDR_AUTO_H_

#include <stdint.h>

#define GNSS_BEAM_OFF          0   /* steering back to power inversion        */
#define GNSS_BEAM_AUTO_OMNI    1   /* auto; no jammer -> power inversion      */
#define GNSS_BEAM_AUTO_BROAD   2   /* auto; no jammer -> broadside beam       */

/* Background polls between updates. ~2 s with a ~100 ms tick. */
#define GNSS_BEAM_POLLS        20

/* Last result, for reports (angles in hundredths of a degree). */
typedef struct {
    int      mode;            /* GNSS_BEAM_*                                  */
    int      valid;           /* at least one update has run                 */
    int      mvdr_ok;         /* MVDR core present and routed to TX1         */
    int      stap_active;     /* STAP drives the output: steering unused     */
    int      jam;             /* jammer judged present for this update       */
    int      jam_from_est;    /* decision from jammer_est.v (else coherence) */
    int      have_null;       /* null_cdeg is valid                          */
    int      have_beam;       /* beam_cdeg is valid                          */
    int      beam_clip;       /* main lobe outside visible space (clipped)   */
    long     null_cdeg;       /* jammer / null angle from broadside          */
    long     beam_cdeg;       /* main-lobe angle from broadside              */
    long     spread_c2;       /* eigenvalue spread lambda_max/min, 0.01 dB   */
    long     coh_c3;          /* coherence |R12|^2/(R11 R22), x1000          */
    int16_t  a1_re, a1_im, a2_re, a2_im;   /* steering vector written (Q14)   */
    const char *source;       /* "noise eigenvector", "power inversion", ... */
    uint32_t n_update;        /* updates computed                            */
    uint32_t n_commit;        /* updates actually written (vector changed)   */
    uint32_t n_nocov;         /* polls with no coherent covariance           */
} gnss_beam_report_t;

/* mode = GNSS_BEAM_*. verbose = print a one-line confirmation. Turning it
 * OFF writes power inversion [1,0] back. */
void gnss_mvdr_auto_enable(int mode, int verbose);
int  gnss_mvdr_auto_get_mode(void);
int  gnss_mvdr_auto_is_enabled(void);

/* Call from the background tick. Cheap when disabled. */
void gnss_mvdr_auto_poll(void);

/* One update right now (used by gnss_beam? and when enabling).
 * Returns 0 on success, negative when the MVDR core / covariance is not
 * usable at the moment. */
int  gnss_mvdr_auto_update_now(void);

/* Copy of the last result. */
void gnss_mvdr_auto_get_report(gnss_beam_report_t *r);

/* Full report (gnss_beam?) and a one-line summary (status / health). */
void gnss_mvdr_auto_print(void);
void gnss_mvdr_auto_print_line(void);

#endif /* GNSS_MVDR_AUTO_H_ */
