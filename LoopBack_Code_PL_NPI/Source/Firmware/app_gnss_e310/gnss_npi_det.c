/******************************************************************************
 *  gnss_npi_det.c
 *  PL-NPI v2.3: jammer detection and measurement on the ARM.
 *  See gnss_npi_det.h. The helpers and the coherence / spectrum code are the
 *  same as in the SMI-PI firmware (gnss_aj_math.c), kept separate so the two
 *  designs stay independent.
 *****************************************************************************/

/* The Vitis XSCT flow builds the application at -O0; these sums run over
 * ~4000 samples per capture, so the file asks for full optimisation. */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize ("O3")
#endif

#include "gnss_npi_det.h"

#define NPI_PI 3.14159265358979323846

/* ---- small maths without libm ------------------------------------------- */

double npi_sqrt(double x)
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

double npi_db10(double x)
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

static double npi_atan(double z)
{
	int inv = 0, neg = 0, k;
	double r, z2, t, s;
	if(z < 0.0) { z = -z; neg = 1; }
	if(z > 1.0) { z = 1.0 / z; inv = 1; }
	/* two half-angle reductions: atan(z) = 2 atan(z / (1 + sqrt(1 + z^2))) */
	z = z / (1.0 + npi_sqrt(1.0 + z * z));
	z = z / (1.0 + npi_sqrt(1.0 + z * z));
	z2 = z * z; t = z; s = 0.0;
	for(k = 1; k < 24; k += 2) { s += ((k & 2) ? -t : t) / k; t *= z2; }
	r = 4.0 * s;
	if(inv) r = NPI_PI / 2.0 - r;
	return neg ? -r : r;
}

static double npi_atan2(double y, double x)
{
	if(x > 0.0) return npi_atan(y / x);
	if(x < 0.0) return (y >= 0.0) ? npi_atan(y / x) + NPI_PI : npi_atan(y / x) - NPI_PI;
	if(y > 0.0) return NPI_PI / 2.0;
	if(y < 0.0) return -NPI_PI / 2.0;
	return 0.0;
}

static double npi_sin(double t)     /* 0 <= t <= pi */
{
	double t2, term, s;
	int k;
	if(t > NPI_PI / 2.0) t = NPI_PI - t;
	t2 = t * t; term = t; s = t;
	for(k = 1; k < 10; k++) {
		term *= -t2 / ((2.0 * k) * (2.0 * k + 1.0));
		s += term;
	}
	return s;
}

/* x in (0,1] with sin(pi x)/(pi x) = rho, 0 < rho < 1. */
static double npi_sincinv(double rho)
{
	double lo = 0.0, hi = 1.0, mid, v;
	int i;
	if(rho >= 1.0) return 0.0;
	if(rho <= 0.0) return 1.0;
	for(i = 0; i < 40; i++) {
		mid = 0.5 * (lo + hi);
		v = npi_sin(NPI_PI * mid) / (NPI_PI * mid);
		if(v > rho) lo = mid; else hi = mid;
	}
	return 0.5 * (lo + hi);
}

/* ---- capture statistics ---------------------------------------------------- */

static int16_t X1R[NPI_NSAMP], X1I[NPI_NSAMP], X2R[NPI_NSAMP], X2I[NPI_NSAMP];
static int64_t S11r[NPI_NLAG], S11i[NPI_NLAG], S22r0;
static int64_t S12r[2 * NPI_NLAG - 1], S12i[2 * NPI_NLAG - 1];

static int16_t sx12(uint32_t v)
{
	int32_t s = (int32_t)(v & 0xFFFU);
	return (int16_t)((s & 0x800) ? s - 0x1000 : s);
}

/* S(l) = sum_{t=st}^{en-1} conj(a[t]) b[t-l]. Each product of two 12-bit
 * samples is below 2^22, so 128 of them fit 32 bits: exact chunked sums. */
#define NPI_CHUNK 128
static void lag_sum(const int16_t *ar, const int16_t *ai,
                    const int16_t *br, const int16_t *bi,
                    int l, int st, int en, int64_t *sr, int64_t *si)
{
	int64_t re = 0, im = 0;
	int t = st;
	while(t < en) {
		int stop = (en - t > NPI_CHUNK) ? t + NPI_CHUNK : en;
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

int npi_detect(const uint32_t *buf, int nsamp, npi_det_t *r)
{
	const int st = 2 * NPI_NLAG, en = nsamp - NPI_NLAG;
	int t, l, lag = 0, cnt = en - st;
	int64_t dummy;
	double inv, best = 0.0, c, noise;

	r->ok = 0;
	if(!buf || nsamp != NPI_NSAMP) return -1;
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
		r->dc1 = npi_sqrt((double)s1r * s1r + (double)s1i * s1i) / (double)nsamp;
		r->dc2 = npi_sqrt((double)s2r * s2r + (double)s2i * s2i) / (double)nsamp;
		m1r = (int16_t)((s1r + (s1r >= 0 ? h : -h)) / nsamp);
		m1i = (int16_t)((s1i + (s1i >= 0 ? h : -h)) / nsamp);
		m2r = (int16_t)((s2r + (s2r >= 0 ? h : -h)) / nsamp);
		m2i = (int16_t)((s2i + (s2i >= 0 ? h : -h)) / nsamp);
		for(t = 0; t < nsamp; t++) {
			X1R[t] -= m1r; X1I[t] -= m1i; X2R[t] -= m2r; X2I[t] -= m2i;
		}
	}
	lag_sum(X1R, X1I, X1R, X1I, 0, st, en, &S11r[0], &S11i[0]);
	lag_sum(X1R, X1I, X1R, X1I, 1, st, en, &S11r[1], &S11i[1]);
	lag_sum(X1R, X1I, X1R, X1I, NPI_NLAG - 1, st, en, &S11r[NPI_NLAG - 1], &S11i[NPI_NLAG - 1]);
	lag_sum(X2R, X2I, X2R, X2I, 0, st, en, &S22r0, &dummy);
	for(l = -(NPI_NLAG - 1); l < NPI_NLAG; l++)
		lag_sum(X1R, X1I, X2R, X2I, l, st, en,
		        &S12r[l + NPI_NLAG - 1], &S12i[l + NPI_NLAG - 1]);
	inv = 1.0 / (double)cnt;
	r->p1 = (double)S11r[0] * inv;
	r->p2 = (double)S22r0 * inv;

	/* RX1-RX2 coherence at the best lag: J/(J+N) for a jammer both antennas
	 * see, ~1/sqrt(cnt) for noise alone */
	if(S11r[0] > 0 && S22r0 > 0) {
		double den = 1.0 / ((double)S11r[0] * (double)S22r0);
		for(l = 0; l < 2 * NPI_NLAG - 1; l++) {
			double re = (double)S12r[l], im = (double)S12i[l];
			double m2 = (re * re + im * im) * den;
			if(m2 > best) { best = m2; lag = l - (NPI_NLAG - 1); }
		}
	}
	c = npi_sqrt(best);
	if(c > 0.999) c = 0.999;
	r->coh = c;
	r->coh_lag = lag;
	r->jn_coh_db = (c > 0.0) ? npi_db10(c / (1.0 - c)) : -99.0;

	/* power J/N over the learned floor */
	noise = r->noise_ref;
	if(noise > 0.0) {
		if(noise < NPI_QFLOOR) noise = NPI_QFLOOR;
		r->noise = noise;
		r->jn_db = (r->p1 > noise) ? npi_db10((r->p1 - noise) / noise) : -99.0;
	} else {
		r->noise = 0.0;
		r->jn_db = -99.0;
	}

	/* jammer centre and bandwidth from RX1's lag-1 and lag-15 correlations */
	{
		double c1r = (double)S11r[1] * inv, c1i = (double)S11i[1] * inv;
		double p1 = r->p1, s2 = r->noise, jr, ji, jm, j15;
		r->rho1_re = (p1 > 0.0) ? c1r / p1 : 0.0;
		r->rho1_im = (p1 > 0.0) ? c1i / p1 : 0.0;
		r->foff = 0.0; r->bw = 1.0;
		if(s2 > 0.0 && p1 - s2 > 0.0) {
			jr = (c1r - s2 * r->rho_noise_re) / (p1 - s2);
			ji = (c1i - s2 * r->rho_noise_im) / (p1 - s2);
			jm = npi_sqrt(jr * jr + ji * ji);
			if(jm > 1.0) jm = 1.0;
			r->foff = -npi_atan2(ji, jr) / (2.0 * NPI_PI);
			if(jm < 0.98) {
				r->bw = npi_sincinv(jm);
			} else {
				double c15r = (double)S11r[NPI_NLAG - 1] * inv, c15i = (double)S11i[NPI_NLAG - 1] * inv;
				j15 = npi_sqrt(c15r * c15r + c15i * c15i) / (p1 - s2);
				if(j15 > 1.0) j15 = 1.0;
				r->bw = npi_sincinv(j15) / (double)(NPI_NLAG - 1);
			}
		}
	}
	r->ok = 1;
	return 0;
}

int npi_measure(const uint32_t *buf, int nsamp, const int32_t *w, npi_det_t *r)
{
	const double sc = 1.0 / 1048576.0;                 /* Q12.20 */
	double g2 = (r->g2 > 0.0) ? r->g2 : 1.0;
	double nr2 = (r->rx2_noise > 0.0) ? r->rx2_noise : 1.0;
	double w2r, w2i, c1r, c1i, c2r, c2i, acc = 0.0, noise, jin, jout;
	int t, k;

	(void)buf;
	if(nsamp != NPI_NSAMP) return -1;
	for(k = 0; k < 6; k++) r->w[k] = (double)w[k] * sc;
	/* the core multiplies x2n = g2 x2: fold g2 into the RX2 weights */
	w2r = r->w[0] * g2; w2i = r->w[1] * g2;
	c1r = r->w[2];      c1i = r->w[3];
	c2r = r->w[4] * g2; c2i = r->w[5] * g2;
	for(t = 0; t < nsamp; t++) {
		double a = X1R[t], b = X1I[t], c = X2R[t], d = X2I[t];
		double yr = a + (w2r * c - w2i * d) + (c1r * a + c1i * b) + (c2r * c + c2i * d);
		double yi = b + (w2r * d + w2i * c) + (c1i * a - c1r * b) + (c2i * c - c2r * d);
		acc += yr * yr + yi * yi;
	}
	r->pout = acc / (double)nsamp;
	/* adaptive weights in the normalised domain, each with its noise */
	r->wn2 = (r->w[0] * r->w[0] + r->w[1] * r->w[1]) * nr2 +
	         (r->w[2] * r->w[2] + r->w[3] * r->w[3]) +
	         (r->w[4] * r->w[4] + r->w[5] * r->w[5]) * nr2;
	/* noise: the learned floor; with none yet, assume the output is noise */
	noise = r->noise;
	if(noise <= 0.0) noise = r->pout / (1.0 + r->wn2);
	if(noise < NPI_QFLOOR) noise = NPI_QFLOOR;
	r->noise = noise;
	r->nout = noise * (1.0 + r->wn2);
	jin  = r->p1 - noise;
	jout = r->pout - r->nout;
	r->jn_db  = (jin > 0.0) ? npi_db10(jin / noise) : -99.0;
	r->res_db = npi_db10((r->pout > 0.0 ? r->pout : NPI_QFLOOR) / r->nout);
	if(jin <= 0.0)               r->supp_db = 0.0;
	else if(jout <= jin * 1e-6)  r->supp_db = 60.0;
	else                         r->supp_db = npi_db10(jin / jout);
	if(r->supp_db < 0.0) r->supp_db = 0.0;
	return 0;
}
