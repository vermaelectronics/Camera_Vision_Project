/******************************************************************************
 *  gnss_mvdr_steer.h   --  GNSS-CRPA MOD-25
 *  Directional steering-vector support for spatial MVDR, and the steering
 *  latch kept in gnss_passthrough.c.
 *****************************************************************************/
#ifndef GNSS_MVDR_STEER_H_
#define GNSS_MVDR_STEER_H_

#include <stdint.h>

/* ---- Who set the steering vector (gnss_passthrough.c latch) ------------- */
#define GNSS_PT_STEER_PI      0   /* power inversion [1,0], the default      */
#define GNSS_PT_STEER_MANUAL  1   /* gnss_steer=theta                        */
#define GNSS_PT_STEER_AUTO    2   /* gnss_beam=1/2 noise eigenvector         */
#define GNSS_PT_STEER_BROAD   3   /* gnss_beam=2 fallback, broadside beam    */

/* Pack a Q14 complex value into an A1/A2 register word (im << 16 | re). */
#define GNSS_PT_STEER_PACK(re, im) \
    ((((uint32_t)(uint16_t)(int16_t)(im)) << 16) | (uint32_t)(uint16_t)(int16_t)(re))

/* Writes A1/A2, remembers them (so gnss_pt_mvdr_write_defaults() -- every
 * gnss_mvdr=1 / gnss_auto=1 / gnss_lo= / gnss_band= restart -- re-applies
 * the same vector instead of resetting it to [1,0]) and COMMITs.
 * angle_cdeg: main-lobe angle in 0.01 deg, only shown when have_angle.
 * Returns GNSS_PT_OK, or an error when the MVDR window is not present (the
 * latch is still updated, so the next MVDR start uses it). */
int32_t gnss_pt_steer_apply(uint32_t a1, uint32_t a2, int src,
                            long angle_cdeg, int have_angle);

/* Back to power inversion [1,0] (latch cleared) and COMMIT. */
int32_t gnss_pt_steer_reset(void);

/* Current latch. Any pointer may be NULL. */
void gnss_pt_steer_get(uint32_t *a1, uint32_t *a2, int *src,
                       long *angle_cdeg, int *have_angle);

/* Name of a GNSS_PT_STEER_* source, for reports. */
const char *gnss_pt_steer_src_name(int src);

/* ---- Manual directional steering (gnss_steer=) -------------------------- */

/* Compute and write a directional steering vector, then COMMIT.
 *
 * theta_deg     : satellite angle from broadside in degrees
 *                 (+ = towards RX2, - = towards RX1)
 * phi_cal_rad   : RX1-RX2 phase calibration in radians
 *                 (pass NAN to use the value stored in JAM_CAL)
 * k_q14         : lambda/(2*d) in Q14 (pass 0 to use the value in JAM_K)
 * verbose       : print the result
 *
 * Returns 0 on success, negative on error.
 */
int gnss_mvdr_set_steering(double theta_deg, double phi_cal_rad,
                           uint16_t k_q14, int verbose);

/* Same geometry, used by gnss_beam=2 for the no-jammer broadside fallback:
 * fills the two register words without writing anything. */
void gnss_mvdr_steering_words(double theta_deg, double phi_cal_rad,
                              uint16_t k_q14, uint32_t *a1, uint32_t *a2);

#endif /* GNSS_MVDR_STEER_H_ */
