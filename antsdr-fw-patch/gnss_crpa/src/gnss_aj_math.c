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

uint32_t aj_nlms_word(double alpha, double p_in, int ws, int nw)
{
	/* m 2^-sh = alpha 2^(12 + ws + RF - 1 + CS) / (nw p_in)   (the -1: u is in Q1) */
	double t;
	int sh = 0, e = 12 + ws + AJ_NLMS_RF - 1 + AJ_NLMS_CS;
	if(alpha <= 0.0 || p_in <= 0.0 || nw <= 0) return 0U;
	t = alpha / ((double)nw * p_in);
	while(e > 0) { t *= 2.0; e--; }
	while(t < 32768.0 && sh < 31) { t *= 2.0; sh++; }
	if(t > 65535.0) t = 65535.0;
	return 0x80000000U | ((uint32_t)sh << 16) | (uint32_t)(t + 0.5);
}

double aj_nlms_alpha(double rho)
{
	double t, a;
	if(rho < 0.0) rho = -rho;
	if(rho >= AJ_NLMS_RHO_NB) return AJ_NLMS_ALPHA_NB;
	if(rho <= AJ_NLMS_RHO_WB) return AJ_NLMS_ALPHA_WB;
	/* AJ_NLMS_ALPHA_NB x (WB / NB)^t, t = 0..1 (WB / NB = 4 = 2^2: 2^(2t) by halving steps) */
	t = (AJ_NLMS_RHO_NB - rho) / (AJ_NLMS_RHO_NB - AJ_NLMS_RHO_WB);
	a = AJ_NLMS_ALPHA_NB;
	while(t >= 0.5) { a *= 2.0; t -= 0.5; }
	return a * (1.0 + 2.0 * t);          /* linear inside the last half step (no libm) */
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
