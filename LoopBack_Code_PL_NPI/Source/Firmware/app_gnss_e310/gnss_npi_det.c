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

/* ---- v2.5: RX2 equaliser, applied to the capture and fitted on request ---------- */
#define NPI_EQ_R 0.99609375            /* notch pole radius = 1 - 2^-8 (as the FPGA's) */
static double EQF[3][2];
static int    EQON;
static double EA1R[NPI_NSAMP], EA1I[NPI_NSAMP], EA2R[NPI_NSAMP], EA2I[NPI_NSAMP];

void npi_eq_set(const double f[3][2], int on)
{
	int k;
	for(k = 0; k < 3; k++) { EQF[k][0] = f[k][0]; EQF[k][1] = f[k][1]; }
	EQON = on ? 1 : 0;
}

/* first-order complex notch y[n] = x[n] - q x[n-1] + r q y[n-1], q = e^{j 2 pi f} */
static void eq_notch(double *xr, double *xi, double f, int n)
{
	double s, c, qr, qi, pr, pi_, xpr = 0.0, xpi = 0.0, ypr = 0.0, ypi = 0.0;
	int t;
	npi_sincos(2.0 * NPI_PI * f, &s, &c);
	qr = c; qi = s; pr = NPI_EQ_R * c; pi_ = NPI_EQ_R * s;
	for(t = 0; t < n; t++) {
		double ar = xr[t], ai = xi[t];
		double yr = ar - (qr * xpr - qi * xpi) + (pr * ypr - pi_ * ypi);
		double yi = ai - (qr * xpi + qi * xpr) + (pr * ypi + pi_ * ypr);
		xpr = ar; xpi = ai; ypr = yr; ypi = yi;
		xr[t] = yr; xi[t] = yi;
	}
}

/* power of x1 - sum_l f_l u_l from the moments:  r11 - 2 Re(sum f_l conj(v_l)) + f^H R f */
static double eq_resid(const double fr[3], const double fi[3], const double Rr[3][3], const double Ri[3][3],
                       const double vr[3], const double vi[3], double r11)
{
	double p = r11;
	int k, l;
	for(l = 0; l < 3; l++) p -= 2.0 * (fr[l] * vr[l] + fi[l] * vi[l]);
	for(k = 0; k < 3; k++)
		for(l = 0; l < 3; l++) {
			double tr = fr[k] * fr[l] + fi[k] * fi[l], ti = fr[k] * fi[l] - fi[k] * fr[l];
			p += tr * Rr[k][l] - ti * Ri[k][l];
		}
	return p;
}

static void eq_fit(npi_det_t *r)
{
	const int n = NPI_NSAMP, b = NPI_NSAMP - 9;
	int a = 8;                       /* skip the notch start-up (pole time constant 256 samples) when notching */
	double Rr[3][3] = {{0}}, Ri[3][3] = {{0}}, vr[3] = {0}, vi[3] = {0}, r11 = 0.0;
	double Qr[3][3], Qi[3][3], Mr[3][4], Mi[3][4], fr[3], fi[3], f1r[3] = {0}, f1i[3] = {0};
	double cnt = (double)(b - a + 1), tr = 0.0, p_fit, p_one, p_cur, noise;
	int t, j, k, l, c;

	r->eq_ok = 0;
	for(t = 0; t < n; t++) { EA1R[t] = X1R[t]; EA1I[t] = X1I[t]; EA2R[t] = X2R[t]; EA2I[t] = X2I[t]; }
	for(j = 0; j < r->eq_ncw && j < 2; j++) {
		eq_notch(EA1R, EA1I, r->eq_cwf[j], n);
		eq_notch(EA2R, EA2I, r->eq_cwf[j], n);
		a = 1100;
	}
	for(t = a; t <= b; t++) {
		double ur[3], ui[3], xr = EA1R[t], xi = EA1I[t];
		ur[0] = EA2R[t + 1]; ui[0] = EA2I[t + 1];
		ur[1] = EA2R[t];     ui[1] = EA2I[t];
		ur[2] = EA2R[t - 1]; ui[2] = EA2I[t - 1];
		r11 += xr * xr + xi * xi;
		for(k = 0; k < 3; k++) {
			vr[k] += ur[k] * xr + ui[k] * xi;
			vi[k] += ur[k] * xi - ui[k] * xr;
			for(l = 0; l < 3; l++) {
				Rr[k][l] += ur[k] * ur[l] + ui[k] * ui[l];
				Ri[k][l] += ur[k] * ui[l] - ui[k] * ur[l];
			}
		}
	}
	r11 /= cnt;
	for(k = 0; k < 3; k++) {
		vr[k] /= cnt; vi[k] /= cnt;
		for(l = 0; l < 3; l++) { Rr[k][l] /= cnt; Ri[k][l] /= cnt; }
	}
	noise = r->noise_ref;
	if(noise > 0.0 && noise < NPI_QFLOOR) noise = NPI_QFLOOR;
	r->eq_jn_db = (noise > 0.0 && r11 > noise) ? npi_db10((r11 - noise) / noise) : -99.0;
	if(r11 <= 0.0 || Rr[1][1] <= 0.0) return;
	for(k = 0; k < 3; k++) for(l = 0; l < 3; l++) { Qr[k][l] = Rr[k][l]; Qi[k][l] = Ri[k][l]; }
	for(k = 0; k < 3; k++) tr += Rr[k][k];
	for(k = 0; k < 3; k++) {                      /* ridge: 1e-3 of the mean power */
		for(l = 0; l < 3; l++) { Mr[k][l] = Rr[k][l]; Mi[k][l] = Ri[k][l]; }
		Mr[k][k] += 1e-3 * tr / 3.0;
		Mr[k][3] = vr[k]; Mi[k][3] = vi[k];
	}
	for(k = 0; k < 3; k++) {                      /* Gauss-Jordan, R is Hermitian positive definite */
		double dr = Mr[k][k], di = Mi[k][k], dd = dr * dr + di * di, ir, ii;
		if(dd <= 0.0) return;
		ir = dr / dd; ii = -di / dd;
		for(c = k; c < 4; c++) { double xr = Mr[k][c], xi = Mi[k][c]; Mr[k][c] = xr * ir - xi * ii; Mi[k][c] = xr * ii + xi * ir; }
		for(j = 0; j < 3; j++) {
			double mr, mi;
			if(j == k) continue;
			mr = Mr[j][k]; mi = Mi[j][k];
			for(c = k; c < 4; c++) {
				Mr[j][c] -= mr * Mr[k][c] - mi * Mi[k][c];
				Mi[j][c] -= mr * Mi[k][c] + mi * Mr[k][c];
			}
		}
	}
	for(k = 0; k < 3; k++) { fr[k] = Mr[k][3]; fi[k] = Mi[k][3]; }
	f1r[1] = vr[1] / Rr[1][1]; f1i[1] = vi[1] / Rr[1][1];        /* one complex weight (what PL-NPI has) */
	p_fit = eq_resid(fr, fi, Qr, Qi, vr, vi, r11);
	p_one = eq_resid(f1r, f1i, Qr, Qi, vr, vi, r11);
	if(EQON) {
		double cr[3], ci[3];
		for(k = 0; k < 3; k++) { cr[k] = EQF[k][0]; ci[k] = EQF[k][1]; }
		p_cur = eq_resid(cr, ci, Qr, Qi, vr, vi, r11);
	} else {
		p_cur = p_one;
	}
	if(p_fit <= 1e-9) p_fit = 1e-9;
	for(k = 0; k < 3; k++) { r->eq_f[k][0] = fr[k]; r->eq_f[k][1] = fi[k]; }
	r->eq_gain_db = npi_db10(p_one / p_fit);
	r->eq_cur_db  = npi_db10((p_cur > 1e-9 ? p_cur : 1e-9) / p_fit);
	r->eq_ok = 1;
}

/* the taps on RX2 of the capture (centre sample t, neighbours t+1 and t-1); 12-bit clip */
static void eq_apply(void)
{
	int t;
	double pr = X2R[0], pi_ = X2I[0];
	for(t = 1; t < NPI_NSAMP - 1; t++) {
		double cr = X2R[t], ci = X2I[t], nr = X2R[t + 1], ni = X2I[t + 1];
		double er = EQF[0][0] * nr - EQF[0][1] * ni + EQF[1][0] * cr - EQF[1][1] * ci + EQF[2][0] * pr - EQF[2][1] * pi_;
		double ei = EQF[0][0] * ni + EQF[0][1] * nr + EQF[1][0] * ci + EQF[1][1] * cr + EQF[2][0] * pi_ + EQF[2][1] * pr;
		int32_t ir = (int32_t)(er + (er >= 0.0 ? 0.5 : -0.5)), ii = (int32_t)(ei + (ei >= 0.0 ? 0.5 : -0.5));
		pr = cr; pi_ = ci;
		X2R[t] = (int16_t)(ir > 2047 ? 2047 : ir < -2048 ? -2048 : ir);
		X2I[t] = (int16_t)(ii > 2047 ? 2047 : ii < -2048 ? -2048 : ii);
	}
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
	/* v2.5: fit the equaliser on the raw capture if asked, then show the
	 * loop what the FPGA's equaliser gives it */
	if(r->eq_req) eq_fit(r); else r->eq_ok = 0;
	if(EQON) eq_apply();
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

/* ---- v2.4: narrowband (CW) scan ------------------------------------------------
 * 4096-point FFT of RX1 and RX2 (the capture of the last npi_detect, DC
 * removed), Hann window, power spectra averaged over scans (each new scan
 * weighs 1/2). A CW concentrates in ~1.5 bins, the noise spreads over 4096:
 * a CW 34 dB below the total noise is still 0 dB in its bin. A peak counts
 * only if it stands above the median bin in BOTH antennas (an external
 * transmitter reaches both; a spur of one receiver does not) and is narrow
 * (3 bins away it is 10 dB down). No libm. */
static double CWR[NPI_CW_N], CWI[NPI_CW_N], TWR[NPI_CW_N / 2], TWI[NPI_CW_N / 2];
static double WIN[NPI_CW_N], AV1[NPI_CW_N], AV2[NPI_CW_N], TMP[NPI_CW_N];
static int    cw_init, cw_avg;

void npi_sincos(double x, double *s, double *c)
{
	/* x to [-pi, pi], then to [-pi/4, pi/4] with the octant symmetries */
	double k = x / (2.0 * NPI_PI), t, t2, sn, cs;
	int q;
	k = (k >= 0.0) ? (double)(long)(k + 0.5) : -(double)(long)(-k + 0.5);
	x -= k * 2.0 * NPI_PI;
	q = (x >= 0.0) ? (int)(x / (NPI_PI / 2.0) + 0.5) : -(int)(-x / (NPI_PI / 2.0) + 0.5);
	t = x - (double)q * (NPI_PI / 2.0);
	t2 = t * t;
	sn = t * (1.0 - t2 / 6.0 * (1.0 - t2 / 20.0 * (1.0 - t2 / 42.0 * (1.0 - t2 / 72.0 * (1.0 - t2 / 110.0 * (1.0 - t2 / 156.0))))));
	cs = 1.0 - t2 / 2.0 * (1.0 - t2 / 12.0 * (1.0 - t2 / 30.0 * (1.0 - t2 / 56.0 * (1.0 - t2 / 90.0 * (1.0 - t2 / 132.0)))));
	switch(q & 3) {
	case 0:  *s = sn;  *c = cs;  break;
	case 1:  *s = cs;  *c = -sn; break;
	case 2:  *s = -sn; *c = -cs; break;
	default: *s = -cs; *c = sn;  break;
	}
}

static void cw_setup(void)
{
	int i;
	for(i = 0; i < NPI_CW_N / 2; i++) {
		double s, c;
		npi_sincos(-2.0 * NPI_PI * (double)i / (double)NPI_CW_N, &s, &c);
		TWR[i] = c; TWI[i] = s;
	}
	for(i = 0; i < NPI_CW_N; i++) {
		double s, c;
		npi_sincos(2.0 * NPI_PI * (double)i / (double)NPI_CW_N, &s, &c);
		WIN[i] = 0.5 - 0.5 * c;
	}
	cw_init = 1;
}

static void fft(double *re, double *im)
{
	int n = NPI_CW_N, i, j, len, k;
	for(i = 1, j = 0; i < n; i++) {           /* bit reversal */
		int bit = n >> 1;
		for(; j & bit; bit >>= 1) j ^= bit;
		j ^= bit;
		if(i < j) {
			double t = re[i]; re[i] = re[j]; re[j] = t;
			t = im[i]; im[i] = im[j]; im[j] = t;
		}
	}
	for(len = 2; len <= n; len <<= 1) {
		int step = n / len, half = len >> 1;
		for(i = 0; i < n; i += len) {
			for(k = 0; k < half; k++) {
				double wr = TWR[k * step], wi = TWI[k * step];
				double xr = re[i + k + half], xi = im[i + k + half];
				double tr = xr * wr - xi * wi, ti = xr * wi + xi * wr;
				re[i + k + half] = re[i + k] - tr; im[i + k + half] = im[i + k] - ti;
				re[i + k] += tr;                  im[i + k] += ti;
			}
		}
	}
}

/* k-th smallest of a[0..n-1] (a is reordered) */
static double kth(double *a, int n, int k)
{
	int lo = 0, hi = n - 1;
	while(lo < hi) {
		double p = a[(lo + hi) >> 1];
		int i = lo, j = hi;
		while(i <= j) {
			while(a[i] < p) i++;
			while(a[j] > p) j--;
			if(i <= j) { double t = a[i]; a[i] = a[j]; a[j] = t; i++; j--; }
		}
		if(k <= j) hi = j; else if(k >= i) lo = i; else break;
	}
	return a[k];
}

static void spectrum(const int16_t *xr, const int16_t *xi, double *av)
{
	int t;
	for(t = 0; t < NPI_CW_N; t++) { CWR[t] = WIN[t] * xr[t]; CWI[t] = WIN[t] * xi[t]; }
	fft(CWR, CWI);
	for(t = 0; t < NPI_CW_N; t++) {
		double p = CWR[t] * CWR[t] + CWI[t] * CWI[t];
		av[t] = (cw_avg == 0) ? p : 0.5 * av[t] + 0.5 * p;
	}
}

void npi_cw_reset(void) { cw_avg = 0; }

int npi_cw_scan(double det_db, npi_cw_t *o)
{
	const int n = NPI_CW_N, edge = (int)(0.45 * NPI_CW_N);
	double med1, med2, thr = 1.0, best[NPI_CW_MAX];
	int i, k, pk[NPI_CW_MAX];

	if(!o) return -1;
	o->n = 0;
	if(!cw_init) cw_setup();
	spectrum(X1R, X1I, AV1);
	spectrum(X2R, X2I, AV2);
	cw_avg++;
	for(i = 0; i < n; i++) TMP[i] = AV1[i];
	med1 = kth(TMP, n, n / 2);
	for(i = 0; i < n; i++) TMP[i] = AV2[i];
	med2 = kth(TMP, n, n / 2);
	o->floor1 = med1; o->floor2 = med2;
	if(med1 <= 0.0 || med2 <= 0.0) return 0;
	for(i = 0; i < det_db; i++) thr *= 1.2589254117941673;    /* 10^(det_db/10) */
	for(k = 0; k < NPI_CW_MAX; k++) { best[k] = 0.0; pk[k] = -1; }
	for(i = 0; i < n; i++) {
		int ks = (i < n / 2) ? i : i - n, j, narrow = 1;
		double m1, m2, m;
		if(ks > -4 && ks < 4) continue;                 /* DC region */
		if(ks > edge || ks < -edge) continue;           /* outside the flat band */
		m1 = AV1[i] / med1; m2 = AV2[i] / med2;
		m = (m1 < m2) ? m1 : m2;
		if(m < thr) continue;
		if(AV1[i] < AV1[(i + 1) % n] || AV1[i] < AV1[(i + n - 1) % n]) continue;   /* local max */
		for(j = 3; j <= 4; j++)
			if(AV1[(i + j) % n] > 0.1 * AV1[i] || AV1[(i + n - j) % n] > 0.1 * AV1[i]) narrow = 0;
		if(!narrow) continue;
		/* keep the two strongest, at least 8 bins apart */
		for(k = 0; k < NPI_CW_MAX; k++) {
			if(pk[k] >= 0) {
				int d = i - pk[k];
				if(d < 0) d = -d;
				if(d > n / 2) d = n - d;
				if(d < 8) { if(m > best[k]) { best[k] = m; pk[k] = i; } goto next; }
			}
		}
		if(m > best[0]) { best[1] = best[0]; pk[1] = pk[0]; best[0] = m; pk[0] = i; }
		else if(m > best[1]) { best[1] = m; pk[1] = i; }
	next: ;
	}
	for(k = 0; k < NPI_CW_MAX; k++) {
		double a, b, c, d = 0.0, den;
		int i0 = pk[k], ks;
		if(i0 < 0) continue;
		ks = (i0 < n / 2) ? i0 : i0 - n;
		a = npi_db10(AV1[(i0 + n - 1) % n]); b = npi_db10(AV1[i0]); c = npi_db10(AV1[(i0 + 1) % n]);
		den = a - 2.0 * b + c;
		if(den < -1e-9) d = 0.5 * (a - c) / den;
		if(d > 0.5) d = 0.5;
		if(d < -0.5) d = -0.5;
		o->f[o->n] = ((double)ks + d) / (double)n;
		o->snr1[o->n] = npi_db10(AV1[i0] / med1);
		o->snr2[o->n] = npi_db10(AV2[i0] / med2);
		o->n++;
	}
	return 0;
}
