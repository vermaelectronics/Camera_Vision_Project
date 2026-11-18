/******************************************************************************
 *  gnss_mvdr_auto.c   --  GNSS-CRPA MOD-25
 *  Fully automatic noise-subspace steering for the 2-channel spatial MVDR.
 *  See gnss_mvdr_auto.h for the behaviour and the poll hook.
 *
 *  MATHS (convention checked against gnss_pt_mvdr_print_w2_check(), where
 *  power inversion gives w2/w1 = -conj(R12)/R22):
 *
 *      R = [ R11        R12 ]      R12 = COV_R12_RE + j COV_R12_IM
 *          [ conj(R12)  R22 ]
 *
 *      lambda_min,max = (tr -/+ sqrt((R11-R22)^2 + 4|R12|^2)) / 2
 *
 *  Noise eigenvector (R - lambda_min I) v = 0, either row:
 *      v = [ R12 , lambda_min - R11 ]            (row 1)
 *      v = [ lambda_min - R22 , conj(R12) ]      (row 2)
 *  The better-conditioned of the two is used. (The MOD-25 draft used
 *  [R11 - lambda_min, R12], which is a ROW of R - lambda_min I. Those rows
 *  span the JAMMER eigenvector, so the beam was pointed at the jammer.)
 *
 *  With a = v_min the MVDR weight w = R^-1 a / (a^H R^-1 a) = v_min:
 *  the null sits on the jammer, and with two elements the remaining degree
 *  of freedom puts the main lobe at electrical phase psi_jammer + 180 deg.
 *
 *  printf() has NO float support: every printed number is a scaled long.
 *****************************************************************************/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gnss_mvdr_auto.h"
#include "gnss_mvdr_steer.h"
#include "gnss_passthrough.h"
#include "gnss_quiet.h"     /* LVDS console: engine output captured, see gnss_quiet.h */
#define GNSS_QUIET_THIS_FILE
#undef  printf
#define printf        gnss_qprintf
#undef  console_print
#define console_print gnss_qprintf

#define A_ONE_Q14           16384
#define AUTO_NSNAP          16      /* covariance windows averaged per update */
#define AUTO_MAX_READS      256     /* bound on snapshot reads per update     */
#define AUTO_MIN_CHANGE     164     /* ~1 % of Q14: smaller changes not written */
#define AUTO_COH_JAM        0.5     /* coherence = jammer when no estimator    */
#define AUTO_RELEASE_POLLS  2       /* no-jammer updates before the fallback   */

#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif

/* Trace-normalised covariance (independent of the RX gain). */
typedef struct {
    double r11, r22, cr, ci;
} auto_cov_t;

static int          beam_mode   = GNSS_BEAM_OFF;
static int          poll_count  = 0;
static int          avg_valid   = 0;
static int          avg_jam     = 0;
static int          nojam_run   = 0;
static auto_cov_t   avg;
static gnss_beam_report_t rep;

/* ------------------------------------------------------------------ */
/* Printing helpers (integer only)                                    */
/* ------------------------------------------------------------------ */
static void auto_print_c2(long v100)
{
    long a = labs(v100);
    printf("%c%ld.%02ld", (v100 < 0) ? '-' : '+', a / 100L, a % 100L);
}

static long auto_to_c2(double v)
{
    return (long)(v * 100.0 + ((v >= 0.0) ? 0.5 : -0.5));
}

/* Wrap hundredths of a degree into (-18000, 18000]. */
static long auto_wrap_cdeg(long c)
{
    while (c >  18000L) c -= 36000L;
    while (c <= -18000L) c += 36000L;
    return c;
}

/* sin(theta) = k * psi / 180 deg  ->  theta in 0.01 deg, clipped. */
static long auto_psi_to_theta_cdeg(long psi_cdeg, double k, int *clip)
{
    double s = k * (double)psi_cdeg / 18000.0;
    *clip = 0;
    if (s >  1.0) { s =  1.0; *clip = 1; }
    if (s < -1.0) { s = -1.0; *clip = 1; }
    return auto_to_c2(asin(s) * 180.0 / M_PI);
}

/* ------------------------------------------------------------------ */
/* Averaged, trace-normalised covariance from several windows         */
/* ------------------------------------------------------------------ */
static int auto_read_cov(auto_cov_t *out, gnss_pt_mvdr_state_t *last_st)
{
    gnss_pt_mvdr_state_t st;
    uint32_t last_blk = 0xFFFFFFFFU;
    double s11 = 0.0, s22 = 0.0, scr = 0.0, sci = 0.0, tr;
    int got = 0, reads = 0;

    while (got < AUTO_NSNAP && reads < AUTO_MAX_READS) {
        reads++;
        gnss_pt_mvdr_get_state(&st);
        if (st.id != GNSS_PT_MVDR_EXPECTED_ID || !st.have_r12) { break; }
        if (!st.cov_coherent || st.n_blocks == last_blk)        { continue; }
        last_blk = st.n_blocks;

        tr = (double)st.cov_r11 + (double)st.cov_r22;
        if (st.cov_r11 <= 0 || st.cov_r22 <= 0 || tr <= 0.0)   { continue; }

        s11 += (double)st.cov_r11    / tr;
        s22 += (double)st.cov_r22    / tr;
        scr += (double)st.cov_r12_re / tr;
        sci += (double)st.cov_r12_im / tr;
        got++;
    }
    if (last_st) { *last_st = st; }
    if (got < AUTO_NSNAP / 2) { return -1; }

    out->r11 = s11 / got;
    out->r22 = s22 / got;
    out->cr  = scr / got;
    out->ci  = sci / got;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Noise eigenvector of the 2x2 Hermitian R                          */
/* Returns 0 and v (unit norm, v1 real >= 0), or -1 if R ~ sigma^2 I. */
/* ------------------------------------------------------------------ */
static int auto_noise_eigvec(const auto_cov_t *R,
                             double *v1r, double *v1i,
                             double *v2r, double *v2i,
                             double *lmax, double *lmin)
{
    double tr  = R->r11 + R->r22;
    double c2  = R->cr * R->cr + R->ci * R->ci;
    double dif = R->r11 - R->r22;
    double sq  = sqrt(dif * dif + 4.0 * c2);
    double ar, ai, br, bi, nA, nB, n, pr, pi, m1, m2, tr_, ti_;

    *lmax = 0.5 * (tr + sq);
    *lmin = 0.5 * (tr - sq);

    /* row 1: [R12, lmin - R11]      row 2: [lmin - R22, conj(R12)] */
    nA = c2 + (*lmin - R->r11) * (*lmin - R->r11);
    nB = c2 + (*lmin - R->r22) * (*lmin - R->r22);
    if (nA >= nB) {
        ar = R->cr;          ai = R->ci;
        br = *lmin - R->r11; bi = 0.0;
        n  = sqrt(nA);
    } else {
        ar = *lmin - R->r22; ai = 0.0;
        br = R->cr;          bi = -R->ci;
        n  = sqrt(nB);
    }
    /* Degenerate: equal eigenvalues (no coherent source) -> no direction */
    if (n < 1e-9 || sq < 1e-6 * tr) { return -1; }

    ar /= n; ai /= n; br /= n; bi /= n;

    /* Phase reference: make the larger element real positive, the other
     * keeps the relative phase. Then scale the larger one to 1.0 (Q14). */
    m1 = sqrt(ar * ar + ai * ai);
    m2 = sqrt(br * br + bi * bi);
    if (m1 >= m2) { pr = ar / m1; pi = -ai / m1; }   /* conj(phase of a) */
    else          { pr = br / m2; pi = -bi / m2; }

    tr_ = ar * pr - ai * pi; ti_ = ar * pi + ai * pr; ar = tr_; ai = ti_;
    tr_ = br * pr - bi * pi; ti_ = br * pi + bi * pr; br = tr_; bi = ti_;

    n = (m1 >= m2) ? m1 : m2;
    *v1r = ar / n; *v1i = ai / n;
    *v2r = br / n; *v2i = bi / n;
    return 0;
}

static int16_t auto_q14(double x)
{
    long v = lround(x * (double)A_ONE_Q14);
    if (v >  32767L) v =  32767L;
    if (v < -32768L) v = -32768L;
    return (int16_t)v;
}

static int auto_words_differ(uint32_t a, uint32_t b)
{
    long dr = (long)(int16_t)(a & 0xFFFFU) - (long)(int16_t)(b & 0xFFFFU);
    long di = (long)(int16_t)(a >> 16)     - (long)(int16_t)(b >> 16);
    return (labs(dr) > AUTO_MIN_CHANGE || labs(di) > AUTO_MIN_CHANGE) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/* One update                                                         */
/* ------------------------------------------------------------------ */
int gnss_mvdr_auto_update_now(void)
{
    gnss_pt_mvdr_state_t st;
    gnss_pt_jam_state_t  js;
    auto_cov_t cur;
    double v1r, v1i, v2r, v2i, lmax, lmin, k, coh;
    uint32_t a1, a2, old_a1, old_a2;
    int src, old_src, jam, est_ok, clip, c0;
    long beam_cdeg = 0;
    int  have_beam = 0;

    if (beam_mode == GNSS_BEAM_OFF) { return -1; }

    rep.mode = beam_mode;

    if (gnss_pt_read(GNSS_PT_REG_MVDR_ID) != GNSS_PT_MVDR_EXPECTED_ID) {
        rep.mvdr_ok = 0;
        return -2;
    }
    if (auto_read_cov(&cur, &st) != 0) {
        rep.n_nocov++;
        rep.mvdr_ok = st.mvdr_sel;
        return -3;
    }
    rep.mvdr_ok     = st.mvdr_sel;
    rep.stap_active = st.have_stap ? st.stap_active : 0;
    rep.n_update++;

    /* ---- jammer decision: FPGA estimator (with hysteresis) if present --- */
    gnss_pt_jam_get_state(&js);
    est_ok = (js.available && js.cfg_enable && js.valid && !js.no_signal) ? 1 : 0;
    coh = (cur.r11 > 0.0 && cur.r22 > 0.0)
        ? (cur.cr * cur.cr + cur.ci * cur.ci) / (cur.r11 * cur.r22) : 0.0;
    jam = est_ok ? js.present : ((coh > AUTO_COH_JAM) ? 1 : 0);
    rep.jam_from_est = est_ok;
    rep.coh_c3 = (long)(coh * 1000.0 + 0.5);

    /* ---- smoothing: average with the last update, restart on a change -- */
    if (avg_valid && avg_jam == jam) {
        avg.r11 = 0.5 * (avg.r11 + cur.r11);
        avg.r22 = 0.5 * (avg.r22 + cur.r22);
        avg.cr  = 0.5 * (avg.cr  + cur.cr);
        avg.ci  = 0.5 * (avg.ci  + cur.ci);
    } else {
        avg = cur;
    }
    avg_valid = 1;
    avg_jam   = jam;

    k = (js.available && js.cfg_k_q14) ? (double)js.cfg_k_q14 / 16384.0 : 1.0;

    rep.have_null = 0;
    rep.beam_clip = 0;
    rep.spread_c2 = 0;

    if (jam && auto_noise_eigvec(&avg, &v1r, &v1i, &v2r, &v2i, &lmax, &lmin) == 0) {
        nojam_run = 0;
        rep.jam = 1;
        if (lmin < 1e-12) lmin = 1e-12;
        rep.spread_c2 = auto_to_c2(10.0 * log10(lmax / lmin));

        a1  = GNSS_PT_STEER_PACK(auto_q14(v1r), auto_q14(v1i));
        a2  = GNSS_PT_STEER_PACK(auto_q14(v2r), auto_q14(v2i));
        src = GNSS_PT_STEER_AUTO;

        if (est_ok) {
            /* Null = jammer angle measured by jammer_est.v (calibrated with
             * JAM_CAL, sign per gnss_jinv=). Main lobe = psi + 180 deg. */
            long psi = (long)js.psi_cdeg;
            if (!js.sin_clip &&
                labs(auto_psi_to_theta_cdeg(psi, k, &c0) - (long)js.theta_cdeg) > 200L) {
                /* psi register not in the expected units: rebuild from theta */
                psi = auto_to_c2((18000.0 / 100.0 / k) *
                                 sin((double)js.theta_cdeg / 100.0 * M_PI / 180.0));
            }
            rep.null_cdeg = (long)js.theta_cdeg;
            rep.have_null = 1;
            beam_cdeg = auto_psi_to_theta_cdeg(auto_wrap_cdeg(psi + 18000L), k, &clip);
            rep.beam_clip = clip;
            have_beam = 1;
        } else {
            /* Old bitstream (no estimator): same geometry as gnss_steer=,
             * phase of the vector against JAM_CAL. Sign NOT verified. */
            double cal = (double)(int16_t)(gnss_pt_read(GNSS_PT_REG_JAM_CAL) & 0xFFFFU)
                         * 180.0 / 32768.0;
            double ph  = atan2(v2i * v1r - v2r * v1i, v2r * v1r + v2i * v1i)
                         * 180.0 / M_PI - cal;
            long   pc  = auto_wrap_cdeg(auto_to_c2(ph));
            beam_cdeg = auto_psi_to_theta_cdeg(pc, k, &clip);
            rep.beam_clip = clip;
            rep.null_cdeg = auto_psi_to_theta_cdeg(auto_wrap_cdeg(pc + 18000L), k, &c0);
            rep.have_null = 1;
            have_beam = 1;
        }
        rep.source = "noise eigenvector";
    } else {
        /* No jammer (or R ~ sigma^2 I): the noise subspace has no direction.
         * Keep the last vector for AUTO_RELEASE_POLLS updates (a pulsed
         * jammer must not flip the pattern every update), then fall back. */
        rep.jam = 0;
        gnss_pt_steer_get(NULL, NULL, &old_src, NULL, NULL);
        if (old_src == GNSS_PT_STEER_AUTO && ++nojam_run < AUTO_RELEASE_POLLS) {
            rep.source = "noise eigenvector (held, jammer just left)";
            rep.valid  = 1;
            return 0;
        }
        if (beam_mode == GNSS_BEAM_AUTO_BROAD) {
            gnss_mvdr_steering_words(0.0, NAN, 0, &a1, &a2);
            src = GNSS_PT_STEER_BROAD;
            beam_cdeg = 0; have_beam = 1;
            rep.source = "broadside beam (no jammer)";
        } else {
            a1  = GNSS_PT_STEER_PACK(A_ONE_Q14, 0);
            a2  = 0U;
            src = GNSS_PT_STEER_PI;
            rep.source = "power inversion [1,0] (no jammer, omni)";
        }
    }

    rep.have_beam = have_beam;
    rep.beam_cdeg = beam_cdeg;
    rep.a1_re = (int16_t)(a1 & 0xFFFFU); rep.a1_im = (int16_t)(a1 >> 16);
    rep.a2_re = (int16_t)(a2 & 0xFFFFU); rep.a2_im = (int16_t)(a2 >> 16);
    rep.valid = 1;

    /* ---- write only when it changed, and only while MVDR drives TX1 ---- */
    gnss_pt_steer_get(&old_a1, &old_a2, &old_src, NULL, NULL);
    if (old_src == src && !auto_words_differ(a1, old_a1) && !auto_words_differ(a2, old_a2)) {
        return 0;
    }
    if (!st.mvdr_sel) {
        return 0;          /* gnss_mvdr=0: nothing to steer; next start uses latch */
    }
    if (gnss_pt_steer_apply(a1, a2, src, beam_cdeg, have_beam) != GNSS_PT_OK) {
        return -4;
    }
    rep.n_commit++;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Background poll                                                    */
/* ------------------------------------------------------------------ */
void gnss_mvdr_auto_poll(void)
{
    if (beam_mode == GNSS_BEAM_OFF) return;
    if (++poll_count < GNSS_BEAM_POLLS) return;
    poll_count = 0;

    /* Only while the MVDR core is present and routed to TX1 */
    if (gnss_pt_read(GNSS_PT_REG_MVDR_ID) != GNSS_PT_MVDR_EXPECTED_ID) return;
    if (!(gnss_pt_read(GNSS_PT_REG_STATUS) & GNSS_PT_ST_MVDR_SEL_SYNCED)) return;

    (void)gnss_mvdr_auto_update_now();
}

/* ------------------------------------------------------------------ */
/* Public API                                                        */
/* ------------------------------------------------------------------ */
void gnss_mvdr_auto_enable(int mode, int verbose)
{
    int src;

    if (mode < GNSS_BEAM_OFF || mode > GNSS_BEAM_AUTO_BROAD) mode = GNSS_BEAM_AUTO_OMNI;
    beam_mode  = mode;
    avg_valid  = 0;
    nojam_run  = 0;
    poll_count = 0;
    memset(&rep, 0, sizeof(rep));
    rep.mode = mode;

    if (mode == GNSS_BEAM_OFF) {
        /* Hand the array back to power inversion if auto owned it. */
        gnss_pt_steer_get(NULL, NULL, &src, NULL, NULL);
        if (src == GNSS_PT_STEER_AUTO || src == GNSS_PT_STEER_BROAD) {
            (void)gnss_pt_steer_reset();
        }
        if (verbose) {
            printf("gnss_beam: OFF -- steering vector is %s\n",
                   gnss_pt_steer_src_name(src == GNSS_PT_STEER_MANUAL
                                          ? GNSS_PT_STEER_MANUAL : GNSS_PT_STEER_PI));
        }
        return;
    }

    (void)gnss_mvdr_auto_update_now();     /* first vector at once */
    if (verbose) {
        printf("gnss_beam: AUTO (noise subspace, every ~%ld polls); no jammer -> %s\n",
               (long)GNSS_BEAM_POLLS,
               (mode == GNSS_BEAM_AUTO_BROAD) ? "broadside beam" : "power inversion (omni)");
        gnss_mvdr_auto_print_line();
        if (rep.stap_active) {
            printf("  NOTE: STAP drives the output (gnss_null=2): the steering vector\n"
                   "        is computed and reported but NOT used. gnss_null=0 to use it.\n");
        }
    }
}

int gnss_mvdr_auto_get_mode(void)   { return beam_mode; }
int gnss_mvdr_auto_is_enabled(void) { return (beam_mode != GNSS_BEAM_OFF) ? 1 : 0; }

void gnss_mvdr_auto_get_report(gnss_beam_report_t *r)
{
    if (r) { *r = rep; r->mode = beam_mode; }
}

/* ------------------------------------------------------------------ */
/* Reports                                                           */
/* ------------------------------------------------------------------ */
void gnss_mvdr_auto_print_line(void)
{
    if (beam_mode == GNSS_BEAM_OFF) {
        int src;
        long ang;
        int have;
        gnss_pt_steer_get(NULL, NULL, &src, &ang, &have);
        printf("  BEAM : auto off, %s", gnss_pt_steer_src_name(src));
        if (have) { printf(" at "); auto_print_c2(ang); printf(" deg"); }
        printf("\n");
        return;
    }
    if (!rep.valid) {
        printf("  BEAM : auto, waiting for a covariance (%s)\n",
               rep.mvdr_ok ? "MVDR on" : "MVDR not routed: gnss_mvdr=1");
        return;
    }
    if (rep.jam) {
        printf("  BEAM : auto, jammer ");
        if (rep.have_null) { auto_print_c2(rep.null_cdeg); printf(" deg (nulled)"); }
        printf(", main lobe ");
        if (rep.have_beam) { auto_print_c2(rep.beam_cdeg); printf(" deg%s", rep.beam_clip ? " (edge)" : ""); }
        printf("%s\n", rep.stap_active ? "  [STAP output: not used]" : "");
    } else {
        printf("  BEAM : auto, no jammer -> %s%s\n", rep.source ? rep.source : "-",
               rep.stap_active ? "  [STAP output: not used]" : "");
    }
}

void gnss_mvdr_auto_print(void)
{
    if (beam_mode != GNSS_BEAM_OFF) { (void)gnss_mvdr_auto_update_now(); }

    printf("\n--- gnss_beam: automatic steering (noise subspace) ---\n");
    printf("  mode           : %s\n",
           (beam_mode == GNSS_BEAM_OFF) ? "OFF (gnss_beam=1 to start)" :
           (beam_mode == GNSS_BEAM_AUTO_BROAD) ? "AUTO, no jammer -> broadside beam (gnss_beam=2)"
                                             : "AUTO, no jammer -> power inversion (gnss_beam=1)");
    if (beam_mode == GNSS_BEAM_OFF) {
        gnss_mvdr_auto_print_line();
        printf("-----------------------------------\n");
        return;
    }
    printf("  MVDR           : %s\n", rep.mvdr_ok ? "routed to TX1"
                                    : "NOT routed to TX1 (gnss_mvdr=1); vector kept for the next start");
    if (rep.stap_active) {
        printf("  output         : STAP (gnss_null=2) -- the steering vector is NOT\n"
               "                   used by the output. gnss_null=0 = MVDR uses it.\n");
    }
    if (!rep.valid) {
        printf("  no coherent covariance yet (N_BLOCKS not advancing?)\n");
        printf("-----------------------------------\n");
        return;
    }
    printf("  jammer         : %s (%s)\n", rep.jam ? "PRESENT" : "none",
           rep.jam_from_est ? "jammer estimator" : "covariance coherence, no estimator");
    printf("  coherence      : %ld.%03ld  |R12|^2/(R11 R22)\n",
           rep.coh_c3 / 1000L, rep.coh_c3 % 1000L);
    if (rep.jam) {
        printf("  eigen spread   : "); auto_print_c2(rep.spread_c2);
        printf(" dB  (lambda_max / lambda_min)\n");
        if (rep.have_null) {
            printf("  null (jammer)  : "); auto_print_c2(rep.null_cdeg);
            printf(" deg from broadside (+ = towards RX2)\n");
        }
        if (rep.have_beam) {
            printf("  main lobe      : "); auto_print_c2(rep.beam_cdeg);
            printf(" deg from broadside%s\n",
                   rep.beam_clip ? "  [edge of visible space]" : "");
        }
        if (!rep.jam_from_est) {
            printf("                   (no jammer estimator: angle uses the gnss_steer=\n"
                   "                    sign convention, not verified on this build)\n");
        }
    } else if (rep.have_beam) {
        printf("  main lobe      : "); auto_print_c2(rep.beam_cdeg);
        printf(" deg (broadside fallback)\n");
    }
    printf("  steering A1    : re=%ld im=%ld\n", (long)rep.a1_re, (long)rep.a1_im);
    printf("  steering A2    : re=%ld im=%ld   (Q14, 16384 = 1.0)\n",
           (long)rep.a2_re, (long)rep.a2_im);
    printf("  source         : %s\n", rep.source ? rep.source : "-");
    printf("  updates        : %lu computed, %lu written, %lu without covariance\n",
           (unsigned long)rep.n_update, (unsigned long)rep.n_commit,
           (unsigned long)rep.n_nocov);
    printf("  (GNSS signals sit ~20 dB under the noise, so no covariance shows a\n"
           "   satellite. The main lobe is the best direction LEFT after the null;\n"
           "   with 2 antennas the null fixes the whole pattern.)\n");
    printf("-----------------------------------\n");
}
