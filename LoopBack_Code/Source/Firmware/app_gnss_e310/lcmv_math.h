/******************************************************************************
 *  lcmv_math.h
 *  2-element LCMV weight solver, PS side (pure math, no hardware access).
 *
 *  R. Li, X. Zhao, X. W. Shi, "Derivative Constrained Robust LCMV Beamforming
 *  Algorithm", PIER C vol. 4, pp. 43-52, 2008:
 *
 *      min  w^H R w    subject to   C^H w = f
 *      w  = R^-1 C (C^H R^-1 C)^-1 f                           (paper eq. 9)
 *
 *  R    2x2 array covariance E[x x^H], measured in the PL (crpa_cov_2ch.v),
 *       with diagonal loading R + L*I for robustness (L = the noise floor,
 *       i.e. the smaller eigenvalue of R).
 *  C    constraint matrix, columns are steering vectors
 *       a(theta) = [1, exp(j*pi*sin(theta))]   (d = lambda/2)
 *  f    constraint values
 *
 *  Constraint sets used:
 *    AUTO   C = [a_s],        f = [1]        look constraint only. R alone
 *           places the null on whatever jammer is present: no jammer angle
 *           is needed (the paper's LCMV with the jammer rows removed).
 *    ANGLE  C = [a_s, a_J],   f = [1, eps]   look + enforcement constraint
 *           (paper eq. 9). With 2 elements the paper's derivative row
 *           forces w2 = 0, so it is not used.
 *
 *  Original work for the ANTSDR E310 V1 GNSS CRPA project.
 *****************************************************************************/
#ifndef LCMV_MATH_H_
#define LCMV_MATH_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { double re, im; } lcmv_c_t;

typedef struct {
    double   r11, r22;     /* real diagonal                          */
    lcmv_c_t r12;          /* E[x1 conj(x2)]; R21 = conj(R12)        */
} lcmv_cov_t;

typedef struct {
    lcmv_c_t w1, w2;       /* weights, y = conj(w1) x1 + conj(w2) x2 */
    double   lambda_max;   /* eigenvalues of R (before loading)      */
    double   lambda_min;
    double   loading;      /* L added to the diagonal                */
    double   jnr_db;       /* 10 log10(lambda_max / lambda_min)      */
    double   null_deg;     /* direction of the array null, degrees   */
    int      null_valid;   /* 0 if w2 ~ 0 (no null in visible space) */
    int      constraints;  /* constraints actually used (1 or 2)     */
} lcmv_result_t;

/* Steering vector a(theta) = [1, exp(j*pi*sin(theta))], theta in degrees. */
void lcmv_steer(double theta_deg, lcmv_c_t a[2]);

/* Apply the RX2 calibration c (x2 -> c*x2) to a raw covariance:
 * R11' = R11, R22' = |c|^2 R22, R12' = conj(c) R12. */
void lcmv_cov_calibrate(lcmv_cov_t *r, lcmv_c_t c);

/* Solve w = R^-1 C (C^H R^-1 C)^-1 f.
 *   gps_deg       look direction (gain 1)
 *   use_jammer    0 = AUTO (look constraint only), 1 = ANGLE (look + null)
 *   jam_deg, eps  jammer direction and enforcement value (ANGLE only)
 * Falls back to the look constraint alone when look and jammer are too
 * close for the 2x2 system. Returns 0 on success, -1 if R is unusable. */
int  lcmv_solve(const lcmv_cov_t *r, double gps_deg,
                int use_jammer, double jam_deg, double eps,
                lcmv_result_t *out);

/* Array response |w^H a(theta)|^2 in dB (for reports and tests). */
double lcmv_response_db(const lcmv_result_t *w, double theta_deg);

#ifdef __cplusplus
}
#endif

#endif /* LCMV_MATH_H_ */
