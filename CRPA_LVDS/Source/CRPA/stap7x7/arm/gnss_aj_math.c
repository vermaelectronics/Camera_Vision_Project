/******************************************************************************
 *  gnss_aj_math.c
 *  crpa13: jammer detection on the captures, the NLMS step and the output
 *  make-up rules. The weights are adapted in the FPGA (nlms_pi, hardware NLMS);
 *  the SMI solver of crpa1-crpa12 is gone. See gnss_aj_math.h.
 *****************************************************************************/

/* The detection sums run over a whole capture: full optimisation even in a
 * debug build. Nothing in this file depends on debug code generation. */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize ("O3")
#endif

#include "gnss_aj_math.h"

#define AJ_PI 3.14159265358979323846

/* ---- small maths without libm ------------------------------------------- */

double aj_sqrt(double x)
{
	double s = 1.0, y;
	int i;
	if(x <= 0.0) return 0.0;
	while(x > 4.0)  { x *= 0.25; s *= 2.0; }
	while(x < 0.25) { x *= 4.0;  s *= 0.5; }
	y = 1.0;
	for(i = 0; i < 8; i++) y = 0.5 * (y + x / y);
	return y * s;
}

double aj_db10(double x)
{
	double r = 0.0;
	if(x <= 0.0) return -200.0;
	while(x >= 10.0) { x *= 0.1; r += 10.0; }
	while(x < 1.0)   { x *= 10.0; r -= 10.0; }
	{
		/* ln(x) = 2 atanh((x-1)/(x+1)), 1 <= x < 10 */
		double y = (x - 1.0) / (x + 1.0), y2 = y * y, t = y, s = 0.0;
		int k;
		for(k = 1; k < 40; k += 2) { s += t / k; t *= y2; }
		r += 2.0 * s * 4.342944819032518;   /* 10 / ln(10) */
	}
	return r;
}

static double aj_atan(double z)
{
	int inv = 0, neg = 0, k;
	double r, z2, t, s;
	if(z < 0.0) { z = -z; neg = 1; }
	if(z > 1.0) { z = 1.0 / z; inv = 1; }
	/* two half-angle reductions: atan(z) = 2 atan(z / (1 + sqrt(1 + z^2))) */
	z = z / (1.0 + aj_sqrt(1.0 + z * z));
	z = z / (1.0 + aj_sqrt(1.0 + z * z));
	z2 = z * z; t = z; s = 0.0;
	for(k = 1; k < 24; k += 2) { s += ((k & 2) ? -t : t) / k; t *= z2; }
	r = 4.0 * s;
	if(inv) r = AJ_PI / 2.0 - r;
	return neg ? -r : r;
}

double aj_atan2(double y, double x)
{
	if(x > 0.0) return aj_atan(y / x);
	if(x < 0.0) return (y >= 0.0) ? aj_atan(y / x) + AJ_PI : aj_atan(y / x) - AJ_PI;
	if(y > 0.0) return AJ_PI / 2.0;
	if(y < 0.0) return -AJ_PI / 2.0;
	return 0.0;
}

static double aj_sin(double t)     /* 0 <= t <= pi */
{
	double t2, term, s;
	int k;
	if(t > AJ_PI / 2.0) t = AJ_PI - t;
	t2 = t * t; term = t; s = t;
	for(k = 1; k < 10; k++) {
		term *= -t2 / ((2.0 * k) * (2.0 * k + 1.0));
		s += term;
	}
	return s;
}

/* x in (0,1] with sin(pi x)/(pi x) = rho, 0 < rho < 1. */
static double aj_sincinv(double rho)
{
	double lo = 0.0, hi = 1.0, mid, v;
	int i;
	if(rho >= 1.0) return 0.0;
	if(rho <= 0.0) return 1.0;
	for(i = 0; i < 40; i++) {
		mid = 0.5 * (lo + hi);
		v = aj_sin(AJ_PI * mid) / (AJ_PI * mid);
		if(v > rho) lo = mid; else hi = mid;
	}
	return 0.5 * (lo + hi);
}

/* ---- block statistics ---------------------------------------------------- */

static int16_t X1R[AJ_NSAMP], X1I[AJ_NSAMP], X2R[AJ_NSAMP], X2I[AJ_NSAMP];

/* S_pq(l) = sum_{t=st}^{en-1} conj(x_p[t]) x_q[t-l] */
static int64_t S11r[AJ_NT], S11i[AJ_NT];
static int64_t S22r[AJ_NT], S22i[AJ_NT];
static int64_t S12r[2 * AJ_NT - 1], S12i[2 * AJ_NT - 1];

static int16_t sx12(uint32_t v)
{
	int32_t s = (int32_t)(v & 0xFFFU);
	return (int16_t)((s & 0x800) ? s - 0x1000 : s);
}

/* The sums below run over a whole capture. Each product of two 12-bit samples is below 2^22, so 128 of
 * them (two per term) fit a 32-bit accumulator: summing in chunks of
 * AJ_CHUNK in 32 bits and only then in 64 is exact and about twice as fast
 * on the Cortex-A9 as a 64-bit add per sample. */
#define AJ_CHUNK 128

static void lag_sum(const int16_t *ar, const int16_t *ai,
                    const int16_t *br, const int16_t *bi,
                    int l, int st, int en, int64_t *sr, int64_t *si)
{
	int64_t re = 0, im = 0;
	int t = st;
	while(t < en) {
		int stop = (en - t > AJ_CHUNK) ? t + AJ_CHUNK : en;
		int32_t cr = 0, ci = 0;
		for(; t < stop; t++) {
			int32_t a_r = ar[t], a_i = ai[t];
			int32_t b_r = br[t - l], b_i = bi[t - l];
			cr += a_r * b_r + a_i * b_i;
			ci += a_r * b_i - a_i * b_r;
		}
		re += cr; im += ci;
	}
	*sr = re; *si = im;
}

static double dc_1, dc_2;

void aj_last_dc(double *dc1, double *dc2) { *dc1 = dc_1; *dc2 = dc_2; }

static void unpack(const uint32_t *buf, int nsamp)
{
	int t;
	for(t = 0; t < nsamp; t++) {
		uint32_t w1 = buf[2 * t], w2 = buf[2 * t + 1];
		X1R[t] = sx12(w1); X1I[t] = sx12(w1 >> 16);
		X2R[t] = sx12(w2); X2I[t] = sx12(w2 >> 16);
	}
	/* Remove each channel's DC (the AD9361's LO leakage / DC offset): a
	 * constant is "seen" by both channels at every lag and would look like a
	 * jammer to the coherence detector and the image taps. */
	{
		int32_t s1r = 0, s1i = 0, s2r = 0, s2i = 0, h = nsamp / 2;
		int16_t m1r, m1i, m2r, m2i;
		for(t = 0; t < nsamp; t++) {
			s1r += X1R[t]; s1i += X1I[t]; s2r += X2R[t]; s2i += X2I[t];
		}
		dc_1 = aj_sqrt((double)s1r * s1r + (double)s1i * s1i) / (double)nsamp;
		dc_2 = aj_sqrt((double)s2r * s2r + (double)s2i * s2i) / (double)nsamp;
		m1r = (int16_t)((s1r + (s1r >= 0 ? h : -h)) / nsamp);
		m1i = (int16_t)((s1i + (s1i >= 0 ? h : -h)) / nsamp);
		m2r = (int16_t)((s2r + (s2r >= 0 ? h : -h)) / nsamp);
		m2i = (int16_t)((s2i + (s2i >= 0 ? h : -h)) / nsamp);
		for(t = 0; t < nsamp; t++) {
			X1R[t] -= m1r; X1I[t] -= m1i; X2R[t] -= m2r; X2I[t] -= m2i;
		}
	}
}

/* RX1-RX2 coherence |E[conj(x1[t]) x2[t-l]]| / sqrt(P1 P2), best lag.
 * A jammer both antennas see makes it J/(J+N); noise alone ~1/sqrt(cnt). */
static void coherence(aj_result_t *r, int cnt)
{
	double p1 = (double)S11r[0], p2 = (double)S22r[0], best = 0.0, c;
	int l, lag = 0;
	(void)cnt;
	if(p1 > 0.0 && p2 > 0.0) {
		double den = 1.0 / (p1 * p2);
		for(l = 0; l < 2 * AJ_NT - 1; l++) {
			double re = (double)S12r[l], im = (double)S12i[l];
			double m2 = (re * re + im * im) * den;
			if(m2 > best) { best = m2; lag = l - (AJ_NT - 1); }
		}
	}
	c = aj_sqrt(best);
	if(c > 0.999) c = 0.999;
	r->coh = c;
	r->coh_lag = lag;
	r->jn_coh_db = (c > 0.0) ? aj_db10(c / (1.0 - c)) : -99.0;
}

/* Jammer centre and bandwidth, from RX1's lag-1 and lag-15 correlations
 * minus the noise's own (s2 = noise power). */
static void spectrum(aj_result_t *r, int cnt, double p1, double s2)
{
	double inv = 1.0 / (double)cnt;
	double c1r = (double)S11r[1] * inv, c1i = (double)S11i[1] * inv;
	double jr, ji, jm, j15;
	r->rho1_re = (p1 > 0.0) ? c1r / p1 : 0.0;
	r->rho1_im = (p1 > 0.0) ? c1i / p1 : 0.0;
	r->foff = 0.0; r->bw = 1.0;
	if(s2 > 0.0 && p1 - s2 > 0.0) {
		jr = (c1r - s2 * r->rho_noise_re) / (p1 - s2);
		ji = (c1i - s2 * r->rho_noise_im) / (p1 - s2);
		jm = aj_sqrt(jr * jr + ji * ji);
		if(jm > 1.0) jm = 1.0;
		r->foff = -aj_atan2(ji, jr) / (2.0 * AJ_PI);
		if(jm < 0.98) {
			r->bw = aj_sincinv(jm);
		} else {
			/* narrow: the lag-15 correlation resolves below fs/15 */
			double c15r = (double)S11r[AJ_NT - 1] * inv, c15i = (double)S11i[AJ_NT - 1] * inv;
			j15 = aj_sqrt(c15r * c15r + c15i * c15i) / (p1 - s2);
			if(j15 > 1.0) j15 = 1.0;
			r->bw = aj_sincinv(j15) / (double)(AJ_NT - 1);
		}
	}
}

int aj_detect(const uint32_t *buf, int nsamp, aj_result_t *r)
{
	const int st = 2 * AJ_NT, en = nsamp - AJ_NT;
	int l, cnt = en - st;
	double inv, noise;

	r->ok = 0;
	if(!buf || nsamp != AJ_NSAMP) return -1;
	unpack(buf, nsamp);
	lag_sum(X1R, X1I, X1R, X1I, 0, st, en, &S11r[0], &S11i[0]);
	lag_sum(X1R, X1I, X1R, X1I, 1, st, en, &S11r[1], &S11i[1]);
	lag_sum(X1R, X1I, X1R, X1I, AJ_NT - 1, st, en, &S11r[AJ_NT - 1], &S11i[AJ_NT - 1]);
	lag_sum(X2R, X2I, X2R, X2I, 0, st, en, &S22r[0], &S22i[0]);
	for(l = -(AJ_NT - 1); l < AJ_NT; l++)
		lag_sum(X1R, X1I, X2R, X2I, l, st, en,
		        &S12r[l + AJ_NT - 1], &S12i[l + AJ_NT - 1]);
	inv = 1.0 / (double)cnt;
	r->p1 = (double)S11r[0] * inv;
	r->p2 = (double)S22r[0] * inv;
	coherence(r, cnt);
	noise = r->noise_ref;
	if(noise > 0.0) {
		if(noise < AJ_QFLOOR) noise = AJ_QFLOOR;
		r->noise = noise;
		r->jn_db = (r->p1 > noise) ? aj_db10((r->p1 - noise) / noise) : -99.0;
	} else {
		r->noise = 0.0;
		r->jn_db = -99.0;
	}
	spectrum(r, cnt, r->p1, r->noise);
	r->ok = 1;
	return 0;
}

double aj_adc_excess_db(double jn_db)
{
	/* The jammer sits near full scale whatever its level (the RX gain follows it), about
	 * 64 dB above the ADC's quantisation noise for a noise-like jammer (crest factor ~4);
	 * the receiver's thermal noise is jn_db below the jammer: quant / thermal = 10^((jn-64)/10). */
	double x = 1.0;
	int n = (int)(jn_db - 64.0), k;
	double step = (n >= 0) ? 1.2589 : 0.7943;
	k = (n >= 0) ? n : -n;
	while(k-- > 0) x *= step;
	return aj_db10(1.0 + x);
}

double aj_makeup_fine_db(int cut_db, double out_db, int shift)
{
	double want = (double)cut_db, safe = (double)cut_db - (out_db > 0.0 ? out_db : 0.0) + AJ_OUT_HEADROOM_DB, f;
	if(cut_db <= 0) return 0.0;
	f = (want < safe ? want : safe) - 6.0 * (double)shift;
	if(f < 0.0) f = 0.0;
	if(f > 5.99) f = 5.99;
	return f;
}

double aj_anlms_alpha(double jn_db)
{
	if(jn_db < 35.0) return 0.05;
	if(jn_db < 50.0) return AJ_ANLMS_ALPHA_DEF;
	return 0.2;
}

int aj_makeup_shift(int cut_db, double out_db, int max_shift)
{
	/* Output relative to the clean level, in dB: -cut + 6 s + out_db. The target is
	 * 0 dB (s = cut / 6, rounded down: at most 5 dB low); a filter that leaves the
	 * jammer in (out_db large) is amplified only up to +AJ_OUT_HEADROOM_DB. */
	int s_restore, s_safe, s;
	if(cut_db <= 0) return 0;
	if(max_shift > AJ_MAXSH) max_shift = AJ_MAXSH;
	s_restore = cut_db / 6;
	s_safe = (int)((double)cut_db - (out_db > 0.0 ? out_db : 0.0) + AJ_OUT_HEADROOM_DB);
	s_safe = (s_safe < 0) ? -1 : s_safe / 6;
	s = (s_restore < s_safe) ? s_restore : s_safe;
	if(s > max_shift) s = max_shift;
	if(s < 0) s = 0;
	return s;
}

/* ---- crpa16 / crpa17 / crpa18: NLMS on the ARM (see gnss_aj_math.h) ------------------------- */
static double nl_wr[AJ_NW], nl_wi[AJ_NW];

void aj_nlms_reset(void)
{
	int j;
	for(j = 0; j < AJ_NW; j++) { nl_wr[j] = 0.0; nl_wi[j] = 0.0; }
}

static int16_t nl_q312(double v)
{
	double s = v * 4096.0;
	s = (s >= 0.0) ? s + 0.5 : s - 0.5;
	if(s > 32767.0)  return 32767;
	if(s < -32768.0) return -32768;
	return (int16_t)s;
}

/* The NLMS / direct weights (nl_wr, nl_wi) -> r->wre / wim (Q3.12 x 2^ws), ws, wn2. Returns 0, or 1
 * when they were not finite or out of range (|w| > 8): they are then reset to 0. */
static double nl_rx2n = 1.0;              /* RX2 noise (normalised) of the last solve, for wn2 */

/* weights (wr, wi) -> r->wre / wim (Q3.12 x 2^ws), ws, wn2. Returns 1 (r untouched) when they are
 * not finite or out of range (|w| > 8). */
static int nl_pack(const double *wr, const double *wi, double rx2n, aj_result_t *r)
{
	double wmax = 0.0, wn2 = 0.0, sc;
	int j, ws;

	for(j = 0; j < AJ_NW; j++) {
		double m = (j == AJ_REF) ? 0.0 : wr[j] * wr[j] + wi[j] * wi[j];
		if(!(m == m) || m > 64.0) return 1;   /* not finite, or |w| > 8 */
		if(m > wmax) wmax = m;
		wn2 += (j >= AJ_NT && j < AJ_NSPT) || j >= AJ_IMG0 + AJ_NIMG ? m * rx2n : m;
	}
	/* the finest weight scale the words fit: |w| 2^ws <= 7.5 (Q3.12 stops at 8) */
	wmax = aj_sqrt(wmax);
	ws = 0;
	while(ws < AJ_MAXWS && wmax * (double)(2 << ws) <= 7.5) ws++;
	sc = (double)(1 << ws);
	for(j = 0; j < AJ_NW; j++) {
		r->wre[j] = (j == AJ_REF) ? 0 : nl_q312(wr[j] * sc);
		r->wim[j] = (j == AJ_REF) ? 0 : nl_q312(wi[j] * sc);
	}
	r->ws = ws;
	r->wn2 = wn2;
	return 0;
}

/* The NLMS / direct weights (nl_wr, nl_wi) -> r. Returns 0, or 1 when they were not finite or out
 * of range: they are then reset to 0. */
static int nl_finish(double rx2n, aj_result_t *r)
{
	nl_wr[AJ_REF] = 0.0; nl_wi[AJ_REF] = 0.0;
	nl_rx2n = rx2n;
	if(nl_pack(nl_wr, nl_wi, rx2n, r)) {
		aj_nlms_reset();
		return 1;
	}
	return 0;
}

/* crpa27: the output of the weights held from the last capture (the NLMS state as it is now) on
 * this capture, before it is solved: what the core gave out until this solve (no make-up gain). */
static double cm_wr[AJ_NW], cm_wi[AJ_NW];   /* the weights committed to the core (all 0 = bypass) */

/* crpa27: the output power of two weight sets (a, b) on the capture in X (no make-up gain), and
 * their |w|^2; one pass over the samples */
static double nl_wn2(const double *wr, const double *wi, double rx2n)
{
	double wn2 = 0.0;
	int j;
	for(j = 0; j < AJ_NW; j++) {
		const double m = (j == AJ_REF) ? 0.0 : wr[j] * wr[j] + wi[j] * wi[j];
		wn2 += (j >= AJ_NT && j < AJ_NSPT) || j >= AJ_IMG0 + AJ_NIMG ? m * rx2n : m;
	}
	return wn2;
}

static void nl_out2(const double *ar, const double *ai, const double *br, const double *bi, double g2,
                    double *pa, double *pb)
{
	double ur[AJ_NW], ui[AJ_NW], sa = 0.0, sb = 0.0;
	int n, j, c, cnt = 0;
	ur[AJ_REF] = 0.0; ui[AJ_REF] = 0.0;
	for(n = AJ_NT - 1; n < AJ_NSAMP; n++) {
		double xr = X1R[n - AJ_REF], xi = X1I[n - AJ_REF], yr = xr, yi = xi, zr = xr, zi = xi;
		for(j = 0; j < AJ_NT; j++) {
			if(j != AJ_REF) { ur[j] = X1R[n - j]; ui[j] = X1I[n - j]; }
			ur[AJ_NT + j] = g2 * X2R[n - j]; ui[AJ_NT + j] = g2 * X2I[n - j];
		}
		for(c = 0; c < AJ_NIMG; c++) {           /* image taps: conj(x) at REF-1, REF, REF+1 */
			const int a = AJ_REF - AJ_IMGH + c;
			ur[AJ_IMG0 + c] = X1R[n - a];                  ui[AJ_IMG0 + c] = -X1I[n - a];
			ur[AJ_IMG0 + AJ_NIMG + c] = g2 * X2R[n - a];   ui[AJ_IMG0 + AJ_NIMG + c] = -g2 * X2I[n - a];
		}
		for(j = 0; j < AJ_NW; j++) {
			yr -= ar[j] * ur[j] - ai[j] * ui[j];
			yi -= ar[j] * ui[j] + ai[j] * ur[j];
			if(br) {
				zr -= br[j] * ur[j] - bi[j] * ui[j];
				zi -= br[j] * ui[j] + bi[j] * ur[j];
			}
		}
		sa += yr * yr + yi * yi;
		sb += zr * zr + zi * zi;
		cnt++;
	}
	*pa = cnt ? sa / cnt : 0.0;
	if(pb) *pb = cnt ? sb / cnt : 0.0;
}

static double cm_wr[AJ_NW], cm_wi[AJ_NW];   /* the weights committed to the core (all 0 = bypass) */
static int    cm_same = 0;                  /* they are the NLMS state (nothing predicted) */
static int    pr_last = 0;                  /* aj_pred_after predicted the weights in r last time */

/* crpa27: the weights held from the last capture (the NLMS state as it is now) and the ones the
 * core ran with since then, on this capture before it is solved */
static void nl_held(double g2, double rx2n, aj_result_t *r)
{
	r->wn2_hold = nl_wn2(nl_wr, nl_wi, rx2n);
	if(cm_same) {                                /* one weight set: the core ran with the solved weights */
		nl_out2(nl_wr, nl_wi, 0, 0, g2, &r->arm_phold, 0);
		r->arm_pcore = r->arm_phold;
		r->wn2_core = r->wn2_hold;
	} else {
		nl_out2(nl_wr, nl_wi, cm_wr, cm_wi, g2, &r->arm_phold, &r->arm_pcore);
		r->wn2_core = nl_wn2(cm_wr, cm_wi, rx2n);
	}
}

void aj_committed(const aj_result_t *r)
{
	int j;
	const double sc = r ? 1.0 / (4096.0 * (double)(1 << r->ws)) : 0.0;
	for(j = 0; j < AJ_NW; j++) {
		cm_wr[j] = r ? sc * (double)r->wre[j] : 0.0;
		cm_wi[j] = r ? sc * (double)r->wim[j] : 0.0;
	}
	cm_same = (r != 0) && !pr_last;              /* the solved weights (to the 16-bit words) */
}

int aj_nlms_solve(const uint32_t *buf, int nsamp, double g2, double rx2n,
                  double alpha, int passes, aj_result_t *r)
{
	double ur[AJ_NW], ui[AJ_NW];
	double pin = 0.0, pout = 0.0;
	int n, j, p, cnt = 0;

	if(!buf || !r || nsamp != AJ_NSAMP || alpha <= 0.0 || passes < 1) return -1;
	if(passes > AJ_ANLMS_PASSES_MAX) passes = AJ_ANLMS_PASSES_MAX;
	if(g2 <= 0.0) g2 = 1.0;
	if(rx2n <= 0.0) rx2n = 1.0;
	unpack(buf, nsamp);                      /* 12-bit samples, DC removed (as aj_detect) */
	nl_held(g2, rx2n, r);                    /* crpa27: the held weights on this capture  */
	ur[AJ_REF] = 0.0; ui[AJ_REF] = 0.0;      /* the reference tap: fixed at 1, no weight  */
	for(p = 0; p < passes; p++) {
		const int last = (p == passes - 1);
		for(n = AJ_NT - 1; n < nsamp; n++) {
			double dr = X1R[n - AJ_REF], di = X1I[n - AJ_REF];
			double yr = dr, yi = di, nrm = 1.0, mu, er, ei;
			int c;
			for(j = 0; j < AJ_NT; j++) {
				if(j != AJ_REF) { ur[j] = X1R[n - j]; ui[j] = X1I[n - j]; }
				ur[AJ_NT + j] = g2 * X2R[n - j]; ui[AJ_NT + j] = g2 * X2I[n - j];
			}
			for(c = 0; c < AJ_NIMG; c++) {      /* image taps: conj(x) at REF-1, REF, REF+1 */
				const int a = AJ_REF - AJ_IMGH + c;
				ur[AJ_IMG0 + c] = X1R[n - a];                  ui[AJ_IMG0 + c] = -X1I[n - a];
				ur[AJ_IMG0 + AJ_NIMG + c] = g2 * X2R[n - a];   ui[AJ_IMG0 + AJ_NIMG + c] = -g2 * X2I[n - a];
			}
			for(j = 0; j < AJ_NW; j++) {
				yr -= nl_wr[j] * ur[j] - nl_wi[j] * ui[j];
				yi -= nl_wr[j] * ui[j] + nl_wi[j] * ur[j];
				nrm += ur[j] * ur[j] + ui[j] * ui[j];
			}
			mu = alpha / nrm;
			er = mu * yr; ei = mu * yi;
			for(j = 0; j < AJ_NW; j++) {     /* w += mu y conj(u) */
				nl_wr[j] += er * ur[j] + ei * ui[j];
				nl_wi[j] += ei * ur[j] - er * ui[j];
			}
			if(last) { pin += dr * dr + di * di; pout += yr * yr + yi * yi; cnt++; }
		}
	}
	r->arm_pin = cnt ? pin / cnt : 0.0;
	r->arm_pout = cnt ? pout / cnt : 0.0;
	return nl_finish(rx2n, r);
}

/* ---- crpa18: the direct (Wiener) solve that starts the NLMS ----------------------------------
 * The tap samples of the core, u[n] (AJ_MAXDIM of them), are integers times 1 (RX1) or g2 (RX2):
 * the sums run on the raw 12-bit samples in 32-bit chunks (exact), g2 is applied afterwards.
 *     R_kj = sum conj(u_k) u_j,  p_k = sum conj(u_k) x1[n-REF],  (R + load I) w = p
 * The solution is the best set of weights for this capture (the "optimum" of the C simulation);
 * it becomes the NLMS state, so the next captures continue from it. */
static int16_t dvr[AJ_MAXDIM], dvi[AJ_MAXDIM];     /* the taps of one sample, raw          */
static double  dsc[AJ_MAXDIM];                     /* their scale: 1 or g2                 */
static int     dmap[AJ_MAXDIM];                    /* tap -> weight word                   */
static double  dRr[AJ_MAXDIM * AJ_MAXDIM], dRi[AJ_MAXDIM * AJ_MAXDIM];
static double  dpr[AJ_MAXDIM], dpi[AJ_MAXDIM];

static void dtaps(int n)
{
	int q = 0, k, c;
	for(k = 0; k < AJ_NT; k++)
		if(k != AJ_REF) { dvr[q] = X1R[n - k]; dvi[q] = X1I[n - k]; q++; }
	for(k = 0; k < AJ_NT; k++) { dvr[q] = X2R[n - k]; dvi[q] = X2I[n - k]; q++; }
	for(c = 0; c < AJ_NIMG; c++) { const int a = AJ_REF - AJ_IMGH + c; dvr[q] = X1R[n - a]; dvi[q] = (int16_t)-X1I[n - a]; q++; }
	for(c = 0; c < AJ_NIMG; c++) { const int a = AJ_REF - AJ_IMGH + c; dvr[q] = X2R[n - a]; dvi[q] = (int16_t)-X2I[n - a]; q++; }
}

int aj_direct_solve(const uint32_t *buf, int nsamp, double g2, double rx2n, double noise, double load,
                    aj_result_t *r)
{
	enum { M = AJ_MAXDIM };
	static int32_t cr[M * M], ci[M * M], qr[M], qi[M];
	static int64_t ar[M * M], ai[M * M], br[M], bi[M];
	double tr = 0.0, pin = 0.0, pout = 0.0;
	int n, k, j, q, i, t, cnt = 0;

	if(!buf || !r || nsamp != AJ_NSAMP) return -1;
	if(g2 <= 0.0) g2 = 1.0;
	if(rx2n <= 0.0) rx2n = 1.0;
	unpack(buf, nsamp);
	nl_held(g2, rx2n, r);                    /* crpa27: the held weights on this capture */
	/* tap order and scale, as aj_nlms_solve's u[] (the weight words, without the reference) */
	q = 0;
	for(k = 0; k < AJ_NT; k++) if(k != AJ_REF) { dmap[q] = k; dsc[q] = 1.0; q++; }
	for(k = 0; k < AJ_NT; k++) { dmap[q] = AJ_NT + k; dsc[q] = g2; q++; }
	for(k = 0; k < AJ_NIMG; k++) { dmap[q] = AJ_IMG0 + k; dsc[q] = 1.0; q++; }
	for(k = 0; k < AJ_NIMG; k++) { dmap[q] = AJ_IMG0 + AJ_NIMG + k; dsc[q] = g2; q++; }
	for(k = 0; k < M * M; k++) { ar[k] = 0; ai[k] = 0; }
	for(k = 0; k < M; k++) { br[k] = 0; bi[k] = 0; }
	/* upper triangle; each product of two 12-bit samples < 2^22, two per term: 128 per 32-bit chunk */
	n = AJ_NT - 1;
	while(n < nsamp) {
		int stop = (nsamp - n > 128) ? n + 128 : nsamp;
		for(k = 0; k < M * M; k++) { cr[k] = 0; ci[k] = 0; }
		for(k = 0; k < M; k++) { qr[k] = 0; qi[k] = 0; }
		for(; n < stop; n++) {
			const int32_t d_r = X1R[n - AJ_REF], d_i = X1I[n - AJ_REF];
			dtaps(n);
			for(k = 0; k < M; k++) {
				const int32_t u_r = dvr[k], u_i = dvi[k];
				int32_t *rr = &cr[k * M], *ri = &ci[k * M];
				qr[k] += u_r * d_r + u_i * d_i;           /* conj(u_k) d */
				qi[k] += u_r * d_i - u_i * d_r;
				for(j = k; j < M; j++) {                  /* conj(u_k) u_j */
					rr[j] += u_r * dvr[j] + u_i * dvi[j];
					ri[j] += u_r * dvi[j] - u_i * dvr[j];
				}
			}
			cnt++;
		}
		for(k = 0; k < M; k++) {
			br[k] += qr[k]; bi[k] += qi[k];
			for(j = k; j < M; j++) { ar[k * M + j] += cr[k * M + j]; ai[k * M + j] += ci[k * M + j]; }
		}
	}
	if(cnt < 2 * M) return -1;
	for(k = 0; k < M; k++) {
		for(j = k; j < M; j++) {
			const double f = dsc[k] * dsc[j];
			dRr[k * M + j] = f * (double)ar[k * M + j];
			dRi[k * M + j] = f * (double)ai[k * M + j];
			dRr[j * M + k] = dRr[k * M + j];             /* Hermitian */
			dRi[j * M + k] = -dRi[k * M + j];
		}
		dpr[k] = dsc[k] * (double)br[k];
		dpi[k] = dsc[k] * (double)bi[k];
		tr += dRr[k * M + k];
	}
	/* Diagonal loading = load x the receiver noise per tap (noise: RX1 noise floor at the present
	 * gain [LSB^2]; the normalised RX2 has about the same). Everything at or below the noise --
	 * the satellites -- is then left alone, as the NLMS does, while the jammer far above it is
	 * nulled; at least the ADC's quantisation noise and 1e-7 of the mean tap power, so the
	 * solve stays well conditioned. */
	{
		double ld = 1e-7 * tr / M, qf = AJ_QFLOOR * (double)cnt, nl = load * noise * (double)cnt;
		if(ld < qf) ld = qf;
		if(ld < nl) ld = nl;
		for(k = 0; k < M; k++) dRr[k * M + k] += ld;
	}
	/* Cholesky R = L L^H (L in the lower triangle of dR), then L z = p, L^H w = z */
	for(j = 0; j < M; j++) {
		double dd = dRr[j * M + j];
		for(t = 0; t < j; t++) dd -= dRr[j * M + t] * dRr[j * M + t] + dRi[j * M + t] * dRi[j * M + t];
		if(!(dd > 0.0)) { aj_nlms_reset(); return 1; }
		dd = aj_sqrt(dd);
		dRr[j * M + j] = dd; dRi[j * M + j] = 0.0;
		for(i = j + 1; i < M; i++) {
			double sr = dRr[i * M + j], si = dRi[i * M + j];
			for(t = 0; t < j; t++) {                      /* - L_it conj(L_jt) */
				const double lr = dRr[i * M + t], li = dRi[i * M + t], mr = dRr[j * M + t], mi = dRi[j * M + t];
				sr -= lr * mr + li * mi;
				si -= li * mr - lr * mi;
			}
			dRr[i * M + j] = sr / dd; dRi[i * M + j] = si / dd;
		}
	}
	for(i = 0; i < M; i++) {                              /* L z = p (z in dp) */
		double sr = dpr[i], si = dpi[i];
		for(t = 0; t < i; t++) {
			sr -= dRr[i * M + t] * dpr[t] - dRi[i * M + t] * dpi[t];
			si -= dRr[i * M + t] * dpi[t] + dRi[i * M + t] * dpr[t];
		}
		dpr[i] = sr / dRr[i * M + i]; dpi[i] = si / dRr[i * M + i];
	}
	for(i = M - 1; i >= 0; i--) {                         /* L^H w = z (w in dp) */
		double sr = dpr[i], si = dpi[i];
		for(t = i + 1; t < M; t++) {                      /* - conj(L_ti) w_t */
			sr -= dRr[t * M + i] * dpr[t] + dRi[t * M + i] * dpi[t];
			si -= dRr[t * M + i] * dpi[t] - dRi[t * M + i] * dpr[t];
		}
		dpr[i] = sr / dRr[i * M + i]; dpi[i] = si / dRr[i * M + i];
	}
	aj_nlms_reset();
	for(k = 0; k < M; k++) { nl_wr[dmap[k]] = dpr[k]; nl_wi[dmap[k]] = dpi[k]; }   /* w for the scaled taps, as the NLMS */
	/* the output of these weights on the capture (what the ARM reports as its residual):
	 * y = d - w . u with u = dsc * raw, so the effective weight per raw tap is w * dsc */
	for(n = AJ_NT - 1; n < nsamp; n++) {
		double yr = X1R[n - AJ_REF], yi = X1I[n - AJ_REF];
		pin += yr * yr + yi * yi;
		dtaps(n);
		for(k = 0; k < M; k++) {
			const double wr = dpr[k] * dsc[k], wi = dpi[k] * dsc[k];
			yr -= wr * dvr[k] - wi * dvi[k];
			yi -= wr * dvi[k] + wi * dvr[k];
		}
		pout += yr * yr + yi * yi;
	}
	r->arm_pin = pin / (double)cnt;
	r->arm_pout = pout / (double)cnt;
	return nl_finish(rx2n, r);
}

/* ---- crpa27/28/29: moving-jammer tracking (see gnss_aj_math.h) ---------------------------- */
void aj_track_reset(aj_track_t *t)
{
	if(!t) return;
	t->best_db = 99.0;
	t->above = 0;
	t->since = 0;
}

int aj_track_update(aj_track_t *t, int direct, double res_db)
{
	if(!t) return 0;
	if(direct) {                 /* the direct solve's own residual is in-sample: the reference is */
		t->best_db = 99.0;       /* taken from the NLMS captures that follow                        */
		t->above = 0;
		t->since = 0;
		return 0;
	}
	t->since++;
	if(t->best_db > 98.0 || res_db < t->best_db) {
		t->best_db = res_db;
		t->above = 0;
		return 0;
	}
	if(res_db > t->best_db + AJ_TRACK_DB && res_db > AJ_TRACK_MIN_DB) t->above++;
	else t->above = 0;
	if(t->above >= AJ_TRACK_N && t->since >= AJ_TRACK_GAP) {
		t->redo++;
		t->above = 0;
		return 1;
	}
	return 0;
}

/* ---- crpa27/28/29: weight prediction (see gnss_aj_math.h) --------------------------------- */
void aj_pred_reset(aj_pred_t *p)
{
	int j;
	if(!p) return;
	for(j = 0; j < AJ_NW; j++) { p->wr[j] = p->wi[j] = 0.0; p->vr[j] = p->vi[j] = 0.0; }
	p->have = 0;
	p->on = 0; p->n_on = 0; p->n_off = 0;
	p->stale_db = 0.0;
}

double aj_hold_db(const aj_result_t *r, double noise)
{
	if(!r || noise <= 0.0 || r->arm_phold <= 0.0) return 0.0;
	return aj_db10(r->arm_phold / (noise * (1.0 + r->wn2_hold)));
}

double aj_core_db(const aj_result_t *r, double noise)
{
	if(!r || noise <= 0.0 || r->arm_pcore <= 0.0) return 0.0;
	return aj_db10(r->arm_pcore / (noise * (1.0 + r->wn2_core)));
}

int aj_pred_after(aj_pred_t *p, int direct, int ok, double noise, double lead, int enable, aj_result_t *r)
{
	double pwr[AJ_NW], pwi[AJ_NW], fresh_db, held_db, stale;
	int j;

	pr_last = 0;
	if(!p || !r) return 0;
	if(!ok) { aj_pred_reset(p); return 0; }      /* weights restarted from 0 */
	/* the change of the solved weights; a direct solve (a new anchor) starts it again */
	if(direct || p->have == 0) {
		for(j = 0; j < AJ_NW; j++) { p->vr[j] = 0.0; p->vi[j] = 0.0; }
		p->have = 1;
	} else {
		/* smoothed, unless this change does not fit the estimate at all (the jammer stopped, turned
		 * or sped up): then the new change is taken as it is */
		double e2 = 0.0, v2 = 0.0, beta = AJ_PRED_BETA;
		for(j = 0; j < AJ_NW; j++) {
			const double dr = nl_wr[j] - p->wr[j] - p->vr[j], di = nl_wi[j] - p->wi[j] - p->vi[j];
			e2 += dr * dr + di * di;
			v2 += p->vr[j] * p->vr[j] + p->vi[j] * p->vi[j];
		}
		if(p->have == 1 || e2 > AJ_PRED_JUMP * v2) beta = 1.0;
		for(j = 0; j < AJ_NW; j++) {
			const double dr = nl_wr[j] - p->wr[j], di = nl_wi[j] - p->wi[j];
			p->vr[j] += beta * (dr - p->vr[j]); p->vi[j] += beta * (di - p->vi[j]);
		}
		p->have = 2;
	}
	for(j = 0; j < AJ_NW; j++) { p->wr[j] = nl_wr[j]; p->wi[j] = nl_wi[j]; }

	/* the gate: how much worse the held weights did than the fresh solve */
	if(noise <= 0.0 || r->arm_phold <= 0.0 || r->arm_pout <= 0.0) return 0;
	fresh_db = aj_db10(r->arm_pout / (noise * (1.0 + r->wn2)));
	held_db = aj_hold_db(r, noise);
	stale = held_db - (fresh_db > 0.0 ? fresh_db : 0.0);
	p->stale_db = stale;
	if(direct) {
		/* no change known yet: keep the gate as it is */
	} else if(!p->on) {
		p->n_on = (stale >= AJ_PRED_ON_DB) ? p->n_on + 1 : 0;
		if(p->n_on >= AJ_PRED_ON_N) { p->on = 1; p->n_off = 0; }
	} else {
		p->n_off = (stale < AJ_PRED_OFF_DB) ? p->n_off + 1 : 0;
		if(p->n_off >= AJ_PRED_OFF_N) { p->on = 0; p->n_on = 0; }
	}
	if(!enable || !p->on || p->have < 2) return 0;
	if(lead < 0.0) lead = 0.0;
	if(lead > AJ_PRED_LEAD_MAX) lead = AJ_PRED_LEAD_MAX;
	for(j = 0; j < AJ_NW; j++) { pwr[j] = nl_wr[j] + lead * p->vr[j]; pwi[j] = nl_wi[j] + lead * p->vi[j]; }
	if(nl_pack(pwr, pwi, nl_rx2n, r)) return 0;   /* out of range: the solved weights stay */
	p->used++;
	pr_last = 1;
	return 1;
}
