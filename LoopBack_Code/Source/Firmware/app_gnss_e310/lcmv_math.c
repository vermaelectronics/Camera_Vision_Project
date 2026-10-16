/******************************************************************************
 *  lcmv_math.c
 *  2-element LCMV weight solver (see lcmv_math.h). Pure math, host-testable.
 *  Original work for the ANTSDR E310 V1 GNSS CRPA project.
 *****************************************************************************/
#include "lcmv_math.h"
#include <math.h>

#define LCMV_PI 3.14159265358979323846

static lcmv_c_t c_mk(double re, double im)      { lcmv_c_t z = { re, im }; return z; }
static lcmv_c_t c_add(lcmv_c_t a, lcmv_c_t b)   { return c_mk(a.re + b.re, a.im + b.im); }
static lcmv_c_t c_sub(lcmv_c_t a, lcmv_c_t b)   { return c_mk(a.re - b.re, a.im - b.im); }
static lcmv_c_t c_mul(lcmv_c_t a, lcmv_c_t b)
{
    return c_mk(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re);
}
static lcmv_c_t c_conj(lcmv_c_t a)              { return c_mk(a.re, -a.im); }
static lcmv_c_t c_scale(lcmv_c_t a, double s)   { return c_mk(a.re * s, a.im * s); }
static double   c_abs2(lcmv_c_t a)              { return a.re * a.re + a.im * a.im; }
static lcmv_c_t c_div(lcmv_c_t a, lcmv_c_t b)
{
    double d = c_abs2(b);
    return c_scale(c_mul(a, c_conj(b)), 1.0 / d);
}

void lcmv_steer(double theta_deg, lcmv_c_t a[2])
{
    double ph = LCMV_PI * sin(theta_deg * LCMV_PI / 180.0);
    a[0] = c_mk(1.0, 0.0);
    a[1] = c_mk(cos(ph), sin(ph));
}

void lcmv_cov_calibrate(lcmv_cov_t *r, lcmv_c_t c)
{
    r->r22 *= c_abs2(c);
    r->r12  = c_mul(c_conj(c), r->r12);
}

/* y = R^-1 v for the loaded Hermitian 2x2 R (det > 0 checked by caller). */
static void r_inv_mul(double a, double d, lcmv_c_t b, double det,
                      const lcmv_c_t v[2], lcmv_c_t y[2])
{
    /* R = [a b; conj(b) d],  R^-1 = 1/det [d -b; -conj(b) a] */
    y[0] = c_scale(c_sub(c_scale(v[0], d), c_mul(b, v[1])), 1.0 / det);
    y[1] = c_scale(c_sub(c_scale(v[1], a), c_mul(c_conj(b), v[0])), 1.0 / det);
}

/* u^H v */
static lcmv_c_t c_dot(const lcmv_c_t u[2], const lcmv_c_t v[2])
{
    return c_add(c_mul(c_conj(u[0]), v[0]), c_mul(c_conj(u[1]), v[1]));
}

int lcmv_solve(const lcmv_cov_t *r, double gps_deg,
               int use_jammer, double jam_deg, double eps,
               lcmv_result_t *out)
{
    double tr, df, disc, L, a, d, det;
    lcmv_c_t as[2], aj[2], b1[2], b2[2];

    tr   = r->r11 + r->r22;
    if (!(tr > 0.0) || r->r11 < 0.0 || r->r22 < 0.0)
        return -1;

    /* eigenvalues of the Hermitian 2x2 R */
    df   = r->r11 - r->r22;
    disc = sqrt(df * df + 4.0 * c_abs2(r->r12));
    out->lambda_max = 0.5 * (tr + disc);
    out->lambda_min = 0.5 * (tr - disc);
    if (out->lambda_min < 0.0) out->lambda_min = 0.0;

    /* diagonal loading at the noise floor (robust LCMV) */
    L = out->lambda_min;
    if (L < 1e-6 * tr) L = 1e-6 * tr;
    out->loading = L;
    out->jnr_db  = 10.0 * log10(out->lambda_max / L);

    a   = r->r11 + L;
    d   = r->r22 + L;
    det = a * d - c_abs2(r->r12);
    if (!(det > 0.0))
        return -1;

    lcmv_steer(gps_deg, as);
    r_inv_mul(a, d, r->r12, det, as, b1);            /* R^-1 a_s */
    out->constraints = 1;

    if (use_jammer) {
        lcmv_c_t g11, g12, g21, g22, gdet, f1, f2, x1, x2;
        lcmv_steer(jam_deg, aj);
        r_inv_mul(a, d, r->r12, det, aj, b2);        /* R^-1 a_J */
        /* G = C^H R^-1 C (2x2 Hermitian) */
        g11 = c_dot(as, b1);  g12 = c_dot(as, b2);
        g21 = c_dot(aj, b1);  g22 = c_dot(aj, b2);
        gdet = c_sub(c_mul(g11, g22), c_mul(g12, g21));
        /* well conditioned only if look and jammer are separated */
        if (sqrt(c_abs2(gdet)) > 1e-4 * sqrt(c_abs2(g11) * c_abs2(g22))) {
            f1 = c_mk(1.0, 0.0);
            f2 = c_mk(eps, 0.0);
            /* x = G^-1 f */
            x1 = c_div(c_sub(c_mul(g22, f1), c_mul(g12, f2)), gdet);
            x2 = c_div(c_sub(c_mul(g11, f2), c_mul(g21, f1)), gdet);
            /* w = R^-1 C x */
            out->w1 = c_add(c_mul(b1[0], x1), c_mul(b2[0], x2));
            out->w2 = c_add(c_mul(b1[1], x1), c_mul(b2[1], x2));
            out->constraints = 2;
        }
    }

    if (out->constraints == 1) {
        /* w = R^-1 a_s / (a_s^H R^-1 a_s)  (real, positive denominator) */
        double g = c_dot(as, b1).re;
        if (!(g > 0.0))
            return -1;
        out->w1 = c_scale(b1[0], 1.0 / g);
        out->w2 = c_scale(b1[1], 1.0 / g);
    }

    /* null of y = conj(w1) x1 + conj(w2) x2 for x = a(theta):
     * conj(w1) + conj(w2) e^{j pi sin(theta)} = 0 */
    out->null_valid = 0;
    if (c_abs2(out->w2) > 1e-12 * (c_abs2(out->w1) + 1e-300)) {
        lcmv_c_t e = c_div(c_scale(c_conj(out->w1), -1.0), c_conj(out->w2));
        double s = atan2(e.im, e.re) / LCMV_PI;
        if (s >= -1.0 && s <= 1.0) {
            out->null_deg   = asin(s) * 180.0 / LCMV_PI;
            out->null_valid = 1;
        }
    }
    return 0;
}

double lcmv_response_db(const lcmv_result_t *w, double theta_deg)
{
    lcmv_c_t a[2], wv[2];
    wv[0] = w->w1; wv[1] = w->w2;
    lcmv_steer(theta_deg, a);
    return 10.0 * log10(c_abs2(c_dot(wv, a)) + 1e-30);
}
