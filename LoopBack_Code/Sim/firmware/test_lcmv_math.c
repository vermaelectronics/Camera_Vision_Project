/* Host unit test of the PS LCMV solver (Source/Firmware/.../lcmv_math.c).
 *   cc -O2 -I../../Source/Firmware/app_gnss_e310 test_lcmv_math.c \
 *      ../../Source/Firmware/app_gnss_e310/lcmv_math.c -lm && ./a.out      */
#include <stdio.h>
#include <math.h>
#include "lcmv_math.h"

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("  FAIL: %s\n", msg); fails++; } \
                           else printf("  ok  : %s\n", msg); } while (0)

/* R = noise*I + J a_J a_J^H + S a_s a_s^H, x2 measured as x2_true * g */
static lcmv_cov_t scene(double noise, double jam_deg, double jnr_db,
                        double gps_deg, double snr_db, lcmv_c_t g)
{
    lcmv_c_t aj[2], as[2];
    double J = noise * pow(10, jnr_db / 10), S = noise * pow(10, snr_db / 10);
    lcmv_cov_t r;
    lcmv_steer(jam_deg, aj); lcmv_steer(gps_deg, as);
    /* x2 = g * a[1] * s  ->  R12 = E[x1 conj(x2)] = P conj(g a[1]) */
    double g2 = g.re * g.re + g.im * g.im;
    r.r11 = noise + J + S;
    r.r22 = (noise + J + S) * g2;
    lcmv_c_t ja = { aj[1].re * g.re - aj[1].im * g.im, aj[1].re * g.im + aj[1].im * g.re };
    lcmv_c_t sa = { as[1].re * g.re - as[1].im * g.im, as[1].re * g.im + as[1].im * g.re };
    r.r12.re = J * ja.re + S * sa.re;
    r.r12.im = -(J * ja.im + S * sa.im);
    return r;
}

int main(void)
{
    lcmv_result_t w;
    lcmv_c_t one = { 1, 0 };
    lcmv_cov_t r;

    printf("T1 AUTO (covariance, no jammer angle): GPS 10, jammer 40 deg, JNR 46 dB\n");
    r = scene(8.0, 40, 46, 10, -20, one);
    CHECK(lcmv_solve(&r, 10, 0, 0, 0, &w) == 0, "solve");
    printf("       w1 = %+.5f %+.5fj  w2 = %+.5f %+.5fj  JNR %.1f dB  null %.2f deg\n",
           w.w1.re, w.w1.im, w.w2.re, w.w2.im, w.jnr_db, w.null_deg);
    CHECK(fabs(lcmv_response_db(&w, 10)) < 0.01, "look gain 0 dB");
    CHECK(lcmv_response_db(&w, 40) < -40, "jammer response < -40 dB");
    CHECK(w.null_valid && fabs(w.null_deg - 40) < 0.5, "null found at 40 deg without the angle");

    printf("T2 AUTO: jammer moves to -25 deg (no setting changed)\n");
    r = scene(8.0, -25, 46, 10, -20, one);
    lcmv_solve(&r, 10, 0, 0, 0, &w);
    printf("       null %.2f deg, jammer response %.1f dB\n", w.null_deg, lcmv_response_db(&w, -25));
    CHECK(lcmv_response_db(&w, -25) < -40, "follows the jammer to -25 deg");
    CHECK(fabs(lcmv_response_db(&w, 10)) < 0.01, "look gain 0 dB");

    printf("T3 ANGLE (paper eq. 9, look + enforcement), GPS 10, jammer 40, eps 0\n");
    r = scene(8.0, 40, 46, 10, -20, one);
    lcmv_solve(&r, 10, 1, 40, 0.0, &w);
    printf("       w1 = %+.5f %+.5fj  w2 = %+.5f %+.5fj\n", w.w1.re, w.w1.im, w.w2.re, w.w2.im);
    CHECK(w.constraints == 2, "two constraints used");
    CHECK(fabs(w.w1.re - 0.5) < 1e-4 && fabs(w.w1.im - 0.55098) < 1e-4 &&
          fabs(w.w2.re - 0.71333) < 1e-4 && fabs(w.w2.im + 0.21156) < 1e-4,
          "matches the PL angle solver / hardware readback");
    CHECK(lcmv_response_db(&w, 40) < -100, "exact null at 40 deg");

    printf("T4 ANGLE eps = 0.01 (enforcement)\n");
    lcmv_solve(&r, 10, 1, 40, 0.01, &w);
    CHECK(fabs(lcmv_response_db(&w, 40) - (-40)) < 0.01, "jammer response = 20log10(eps) = -40 dB");

    printf("T5 RX2 error 0.85 x e^{j190}, AUTO with and without calibration\n");
    lcmv_c_t g = { 0.85 * cos(190 * M_PI / 180), 0.85 * sin(190 * M_PI / 180) };
    r = scene(8.0, 40, 46, 10, -20, g);
    lcmv_solve(&r, 10, 0, 0, 0, &w);
    /* without calibration the jammer is still nulled (adaptive), the look
     * direction is what moves */
    printf("       uncalibrated: null at %.1f deg (true jammer 40)\n", w.null_deg);
    lcmv_c_t c = { 1.0 / 0.85 * cos(-190 * M_PI / 180), 1.0 / 0.85 * sin(-190 * M_PI / 180) };
    lcmv_cov_calibrate(&r, c);
    lcmv_solve(&r, 10, 0, 0, 0, &w);
    CHECK(lcmv_response_db(&w, 40) < -40 && fabs(lcmv_response_db(&w, 10)) < 0.01,
          "calibrated: null at 40, look gain 0 dB");

    printf("T6 no jammer (noise only): weights = a_s / 2 (full array gain to GPS)\n");
    r = scene(8.0, 40, -100, 10, -100, one);
    lcmv_solve(&r, 10, 0, 0, 0, &w);
    printf("       JNR %.1f dB, |w1| %.3f |w2| %.3f\n", w.jnr_db,
           sqrt(w.w1.re*w.w1.re + w.w1.im*w.w1.im), sqrt(w.w2.re*w.w2.re + w.w2.im*w.w2.im));
    CHECK(fabs(lcmv_response_db(&w, 10)) < 0.01, "look gain 0 dB");
    CHECK(w.jnr_db < 3.5, "no jammer detected");

    printf(fails ? "FAIL (%d)\n" : "PASS\n", fails);
    return fails != 0;
}
