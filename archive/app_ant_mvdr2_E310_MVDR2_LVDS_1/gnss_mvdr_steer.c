/******************************************************************************
 *  gnss_mvdr_steer.c   --  GNSS-CRPA MOD-25
 *  Directional steering-vector support for spatial MVDR (manual angle).
 *
 *  printf() here has NO float support (same as the rest of the project):
 *  every printed number is scaled to a long first.
 *****************************************************************************/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "gnss_mvdr_steer.h"
#include "gnss_passthrough.h"
#include "gnss_quiet.h"     /* LVDS console: engine output captured, see gnss_quiet.h */
#define GNSS_QUIET_THIS_FILE
#undef  printf
#define printf        gnss_qprintf
#undef  console_print
#define console_print gnss_qprintf

/* Q14 full scale (1.0 = 16384) */
#define A_ONE_Q14          16384

#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif

#ifndef NAN
#define NAN  (0.0/0.0)
#endif

/* Direct offsets (from mvdr_regs.v / gnss_passthrough.h)
 * These are used if the macros are not visible to the compiler. */
#ifndef GNSS_PT_REG_JAM_CAL
#define GNSS_PT_REG_JAM_CAL          (0x100U + 0xC4U)
#endif
#ifndef GNSS_PT_REG_JAM_K
#define GNSS_PT_REG_JAM_K            (0x100U + 0xC8U)
#endif

/* Prints hundredths as +12.34 / -0.05 without float printf. */
static void steer_print_c2(double v)
{
    long m = (long)(v * 100.0 + ((v >= 0.0) ? 0.5 : -0.5));
    printf("%c%ld.%02ld", (m < 0) ? '-' : '+', labs(m) / 100L, labs(m) % 100L);
}

/* ------------------------------------------------------------------ */
/* Read phase-calibration from the jammer estimator (binary angle)   */
/* ------------------------------------------------------------------ */
static double gnss_mvdr_read_phase_cal_rad(void)
{
    uint32_t cal_bin = gnss_pt_read(GNSS_PT_REG_JAM_CAL) & 0xFFFFU;
    int16_t  cal_s   = (int16_t)cal_bin;
    return (double)cal_s * M_PI / 32768.0;
}

/* ------------------------------------------------------------------ */
/* Read K = lambda/(2*d) from the jammer estimator (Q14)             */
/* ------------------------------------------------------------------ */
static uint16_t gnss_mvdr_read_k_q14(void)
{
    return (uint16_t)(gnss_pt_read(GNSS_PT_REG_JAM_K) & 0xFFFFU);
}

/* ------------------------------------------------------------------ */
/* Geometry -> register words (no hardware write)                    */
/* ------------------------------------------------------------------ */
static double steer_words(double theta_deg, double *phi_cal_rad,
                          uint16_t k_q14, uint32_t *a1, uint32_t *a2,
                          int16_t *a2_re_o, int16_t *a2_im_o)
{
    double phi_rad, k;
    int16_t a2_re, a2_im;

    /* Use hardware K if not supplied */
    if (k_q14 == 0) {
        k_q14 = gnss_mvdr_read_k_q14();
        if (k_q14 == 0) k_q14 = A_ONE_Q14;   /* safe default = 1.0 */
    }
    /* Use hardware phase cal if not supplied (NAN) */
    if (isnan(*phi_cal_rad)) {
        *phi_cal_rad = gnss_mvdr_read_phase_cal_rad();
    }

    k = (double)k_q14 / (double)A_ONE_Q14;
    if (k < 1e-6) k = 1.0;

    /* phi = (pi / k) * sin(theta) + phi_cal
     * (matches the geometry used by jammer_est.v) */
    phi_rad = (M_PI / k) * sin(theta_deg * M_PI / 180.0) + *phi_cal_rad;

    a2_re = (int16_t)lround((double)A_ONE_Q14 * cos(phi_rad));
    a2_im = (int16_t)lround((double)A_ONE_Q14 * sin(phi_rad));

    *a1 = GNSS_PT_STEER_PACK(A_ONE_Q14, 0);
    *a2 = GNSS_PT_STEER_PACK(a2_re, a2_im);
    if (a2_re_o) *a2_re_o = a2_re;
    if (a2_im_o) *a2_im_o = a2_im;
    return phi_rad;
}

void gnss_mvdr_steering_words(double theta_deg, double phi_cal_rad,
                              uint16_t k_q14, uint32_t *a1, uint32_t *a2)
{
    (void)steer_words(theta_deg, &phi_cal_rad, k_q14, a1, a2, NULL, NULL);
}

/* ------------------------------------------------------------------ */
/* Compute and write the steering vector                             */
/* ------------------------------------------------------------------ */
int gnss_mvdr_set_steering(double theta_deg, double phi_cal_rad,
                           uint16_t k_q14, int verbose)
{
    double phi_rad;
    uint32_t reg_a1, reg_a2;
    int16_t a2_re, a2_im;
    long cdeg = (long)(theta_deg * 100.0 + ((theta_deg >= 0.0) ? 0.5 : -0.5));

    phi_rad = steer_words(theta_deg, &phi_cal_rad, k_q14,
                          &reg_a1, &reg_a2, &a2_re, &a2_im);

    if (gnss_pt_steer_apply(reg_a1, reg_a2, GNSS_PT_STEER_MANUAL, cdeg, 1)
            != GNSS_PT_OK) {
        printf("gnss_steer: MVDR window not present / COMMIT failed\n"
               "            (the vector is kept and applied at the next MVDR start)\n");
        return -1;
    }

    if (verbose) {
        printf("gnss_steer: theta=");
        steer_print_c2(theta_deg);
        printf(" deg  phi=");
        steer_print_c2(phi_rad * 180.0 / M_PI);
        printf(" deg (cal ");
        steer_print_c2(phi_cal_rad * 180.0 / M_PI);
        printf(" deg)\n");
        printf("  A1 = %ld + j%ld    A2 = %ld + j%ld   (Q14)\n",
               (long)A_ONE_Q14, 0L, (long)a2_re, (long)a2_im);
    }
    return 0;
}
