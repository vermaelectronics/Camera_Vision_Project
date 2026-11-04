/*
 * stap_solver.c -- see stap_solver.h
 */
#include "stap_solver.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static stap_cplx cmul(stap_cplx a, stap_cplx b)  { stap_cplx r = { a.re*b.re - a.im*b.im, a.re*b.im + a.im*b.re }; return r; }
static stap_cplx cmulc(stap_cplx a, stap_cplx b) { stap_cplx r = { a.re*b.re + a.im*b.im, a.im*b.re - a.re*b.im }; return r; } /* a*conj(b) */
static stap_cplx cconj(stap_cplx a)              { stap_cplx r = { a.re, -a.im }; return r; }

int stap_corr_index(int a, int b, int tau)
{
    int p = 0, i;
    for (i = 0; i < a; i++) p += STAP_M - i;     /* pairs (i, i..M-1) before row a */
    p += b - a;
    return p * STAP_NL + tau + (STAP_N - 1);
}

void stap_build_cov(const int64_t *corr_re, const int64_t *corr_im, const stap_edges *ed,
                    uint32_t L, stap_cplx R[STAP_T][STAP_T])
{
    /* L*R[(a,k),(b,kk)] = c_ab[k-kk]
     *                   + sum_{j=kk}^{N-2}     x_a[j+k-kk] conj(x_b[j])     (head)
     *                   - sum_{j=kk+L}^{N-2+L} x_a[j+k-kk] conj(x_b[j])     (tail)
     * computed exactly in 64-bit integers for a <= b, mirrored for a > b.   */
    const double s = 1.0 / (double)L;
    int a, b, k, kk, j;
    for (a = 0; a < STAP_M; a++)
        for (b = a; b < STAP_M; b++)
            for (k = 0; k < STAP_N; k++)
                for (kk = 0; kk < STAP_N; kk++) {
                    const int tau = k - kk;
                    const int e = stap_corr_index(a, b, tau);
                    int64_t vr = corr_re[e], vi = corr_im[e];
                    for (j = kk; j <= STAP_N - 2; j++) {
                        int64_t ar = ed->head_re[a][j + tau], ai = ed->head_im[a][j + tau];
                        int64_t br = ed->head_re[b][j],       bi = ed->head_im[b][j];
                        int64_t tr = ed->tail_re[a][j + tau], ti = ed->tail_im[a][j + tau];
                        int64_t ur = ed->tail_re[b][j],       ui = ed->tail_im[b][j];
                        vr += (ar*br + ai*bi) - (tr*ur + ti*ui);
                        vi += (ai*br - ar*bi) - (ti*ur - tr*ui);
                    }
                    R[a*STAP_N + k][b*STAP_N + kk].re =  (double)vr * s;
                    R[a*STAP_N + k][b*STAP_N + kk].im =  (double)vi * s;
                    R[b*STAP_N + kk][a*STAP_N + k].re =  (double)vr * s;
                    R[b*STAP_N + kk][a*STAP_N + k].im = -(double)vi * s;
                }
}

void stap_steering(double d_over_lambda, double theta_deg, double fif_over_fs,
                   stap_cplx S[STAP_T])
{
    /* matches STAP_constraint.m (with the 2*pi*d/lambda bug fixed):
     *   Ss(m) = exp(-j 2pi d/lambda sin(theta) m),  St(k) = exp(j 2pi IF/fs k)    */
    const double ws = 2.0 * M_PI * d_over_lambda * sin(theta_deg * M_PI / 180.0);
    const double wt = 2.0 * M_PI * fif_over_fs;
    int m, k;
    for (m = 0; m < STAP_M; m++)
        for (k = 0; k < STAP_N; k++) {
            double ph = -ws * m + wt * k;
            S[m*STAP_N + k].re = cos(ph);
            S[m*STAP_N + k].im = sin(ph);
        }
}

void stap_steering_pi(stap_cplx S[STAP_T])
{
    int t;
    for (t = 0; t < STAP_T; t++) { S[t].re = 0.0; S[t].im = 0.0; }
    S[0].re = 1.0;
}

/* work matrix for the Cholesky factor (static: 200 kB, no stack blow-up on MCUs) */
static stap_cplx G[STAP_T][STAP_T];

int stap_solve_mvdr(const stap_cplx R[STAP_T][STAP_T], const stap_cplx S[STAP_T],
                    double diag_load, stap_cplx w[STAP_T])
{
    stap_cplx y[STAP_T];
    double tr = 0.0, load;
    int i, j, k;

    for (i = 0; i < STAP_T; i++) tr += R[i][i].re;
    load = diag_load * tr / STAP_T;

    /* Cholesky  R = G G^H  (G lower triangular) */
    for (j = 0; j < STAP_T; j++) {
        double d = R[j][j].re + load;
        for (k = 0; k < j; k++) d -= G[j][k].re*G[j][k].re + G[j][k].im*G[j][k].im;
        if (d <= 0.0) return -1;
        d = sqrt(d);
        G[j][j].re = d; G[j][j].im = 0.0;
        for (i = j + 1; i < STAP_T; i++) {
            stap_cplx v = R[i][j];
            for (k = 0; k < j; k++) {
                stap_cplx p = cmulc(G[i][k], G[j][k]);
                v.re -= p.re; v.im -= p.im;
            }
            G[i][j].re = v.re / d;
            G[i][j].im = v.im / d;
        }
        for (i = 0; i < j; i++) { G[i][j].re = 0.0; G[i][j].im = 0.0; }
    }

    /* forward:  G y = S */
    for (i = 0; i < STAP_T; i++) {
        stap_cplx v = S[i];
        for (k = 0; k < i; k++) {
            stap_cplx p = cmul(G[i][k], y[k]);
            v.re -= p.re; v.im -= p.im;
        }
        y[i].re = v.re / G[i][i].re;
        y[i].im = v.im / G[i][i].re;
    }
    /* backward: G^H u = y   (u stored in w) */
    for (i = STAP_T - 1; i >= 0; i--) {
        stap_cplx v = y[i];
        for (k = i + 1; k < STAP_T; k++) {
            stap_cplx p = cmul(cconj(G[k][i]), w[k]);
            v.re -= p.re; v.im -= p.im;
        }
        w[i].re = v.re / G[i][i].re;
        w[i].im = v.im / G[i][i].re;
    }
    /* normalise: w = u / (S^H u) */
    {
        stap_cplx den = { 0.0, 0.0 }, inv;
        double m2;
        for (i = 0; i < STAP_T; i++) {
            stap_cplx p = cmulc(w[i], S[i]);          /* conj(S)*u */
            den.re += p.re; den.im += p.im;
        }
        m2 = den.re*den.re + den.im*den.im;
        if (m2 == 0.0) return -1;
        inv.re = den.re / m2; inv.im = -den.im / m2;  /* 1/den */
        for (i = 0; i < STAP_T; i++) w[i] = cmul(w[i], inv);
    }
    return 0;
}

int stap_quantize(const stap_cplx w[STAP_T], int ww, int32_t wq_re[STAP_T], int32_t wq_im[STAP_T])
{
    const double fs = (double)((1L << (ww - 1)) - 1);
    double mx = 0.0, sc;
    int t, e;
    for (t = 0; t < STAP_T; t++) {
        if (fabs(w[t].re) > mx) mx = fabs(w[t].re);
        if (fabs(w[t].im) > mx) mx = fabs(w[t].im);
    }
    if (mx == 0.0) mx = 1.0;
    e  = (int)floor(log2(fs / mx));
    sc = ldexp(1.0, e);
    for (t = 0; t < STAP_T; t++) {
        double r = floor(w[t].re * sc + 0.5), i = floor(w[t].im * sc + 0.5);
        if (r >  fs) r =  fs;
        if (r < -fs - 1) r = -fs - 1;
        if (i >  fs) i =  fs;
        if (i < -fs - 1) i = -fs - 1;
        wq_re[t] = (int32_t)r;
        wq_im[t] = (int32_t)i;
    }
    return e;
}

int stap_out_shift(const stap_cplx R[STAP_T][STAP_T], const stap_cplx w[STAP_T],
                   int exp, int ow, int headroom_bits)
{
    /* predicted output power  P = w^H R w  (ADC LSB^2) */
    double P = 0.0, rms, target;
    int i, j, s;
    for (i = 0; i < STAP_T; i++) {
        stap_cplx acc = { 0.0, 0.0 };
        for (j = 0; j < STAP_T; j++) {
            stap_cplx p = cmul(R[i][j], w[j]);
            acc.re += p.re; acc.im += p.im;
        }
        P += w[i].re*acc.re + w[i].im*acc.im;          /* Re{conj(w_i) * (Rw)_i} */
    }
    rms    = sqrt(P > 0.0 ? P : 1e-30) * ldexp(1.0, exp);  /* complex RMS before shift */
    target = ldexp(1.0, ow - 1 - headroom_bits);
    s = (int)floor(log2(rms / target) + 0.5);
    if (s < 0)  s = 0;
    if (s > 63) s = 63;
    return s;
}

double stap_response(const stap_cplx w[STAP_T], double d_over_lambda, double theta_deg,
                     double f_over_fs, double frf_over_fc)
{
    const double ws = 2.0 * M_PI * d_over_lambda * frf_over_fc * sin(theta_deg * M_PI / 180.0);
    const double wt = 2.0 * M_PI * f_over_fs;
    stap_cplx acc = { 0.0, 0.0 };
    int m, k;
    for (m = 0; m < STAP_M; m++)
        for (k = 0; k < STAP_N; k++) {
            double ph = -ws * m + wt * k;
            stap_cplx a = { cos(ph), sin(ph) };
            stap_cplx p = cmulc(a, w[m*STAP_N + k]);   /* conj(w)*a */
            acc.re += p.re; acc.im += p.im;
        }
    return sqrt(acc.re*acc.re + acc.im*acc.im);
}
