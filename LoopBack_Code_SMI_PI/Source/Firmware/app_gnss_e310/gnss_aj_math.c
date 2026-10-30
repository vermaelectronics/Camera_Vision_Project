/******************************************************************************
 *  gnss_aj_math.c
 *  Space-time SMI power inversion: the weight computation. See gnss_aj_math.h.
 *****************************************************************************/

/* The Vitis XSCT flow builds the application in its Debug configuration
 * (-O0). This file does ~1 million multiply-accumulates per weight update,
 * which at -O0 would cut the update rate several-fold, so it asks GCC for
 * full optimisation itself. Nothing in it depends on debug code generation. */
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

/* Full 34x34 statistics F_ij = E[conj(v_i) v_j] of this block:
 *   v_i = x_{p(i)}[n - d(i)]       i = 0..31 (the space-time taps)
 *   v_32 = conj(x1[n-8]), v_33 = conj(x2[n-8])   (the image taps)
 * RX2 entries are scaled by g2 (the core's input normalisation). */
static double FR[AJ_NW][AJ_NW], FI[AJ_NW][AJ_NW];
/* The same, averaged over blocks (what the solve uses). */
static double AR[AJ_NW][AJ_NW], AI[AJ_NW][AJ_NW];
static int    avg_valid;

/* Working matrices for the solve */
static double MR[AJ_MAXDIM][AJ_MAXDIM], MI[AJ_MAXDIM][AJ_MAXDIM];
static double LR[AJ_MAXDIM][AJ_MAXDIM], LI[AJ_MAXDIM][AJ_MAXDIM];

static int16_t sx12(uint32_t v)
{
	int32_t s = (int32_t)(v & 0xFFFU);
	return (int16_t)((s & 0x800) ? s - 0x1000 : s);
}

/* The sums below run over ~4000 samples, 127 of them per solve: the ARM's
 * main cost. Each product of two 12-bit samples is below 2^22, so 128 of
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

/* sum_{n=st}^{en-1} x_p[n-a] x_q[n-b]   (no conjugate: the image statistics) */
static void comp_sum(const int16_t *ar, const int16_t *ai, int a,
                     const int16_t *br, const int16_t *bi, int b,
                     int st, int en, int64_t *sr, int64_t *si)
{
	int64_t re = 0, im = 0;
	int n = st;
	while(n < en) {
		int stop = (en - n > AJ_CHUNK) ? n + AJ_CHUNK : en;
		int32_t cr = 0, ci = 0;
		for(; n < stop; n++) {
			int32_t a_r = ar[n - a], a_i = ai[n - a];
			int32_t b_r = br[n - b], b_i = bi[n - b];
			cr += a_r * b_r - a_i * b_i;
			ci += a_r * b_i + a_i * b_r;
		}
		re += cr; im += ci;
	}
	*sr = re; *si = im;
}

static void chan(int p, const int16_t **r, const int16_t **i)
{
	if(p == 1) { *r = X1R; *i = X1I; }
	else       { *r = X2R; *i = X2I; }
}

static int is_rx2(int i) { return (i >= AJ_NT && i < AJ_NSPT) || i == AJ_IMG2; }

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

/* Builds F from one block (g2: RX2 normalisation). Returns the number of
 * terms per entry. */
static int build_stats(const uint32_t *buf, int nsamp, double g2)
{
	const int st = 2 * AJ_NT, en = nsamp - AJ_NT;
	int l, i, j;
	double inv;

	unpack(buf, nsamp);
	for(l = 0; l < AJ_NT; l++) {
		lag_sum(X1R, X1I, X1R, X1I, l, st, en, &S11r[l], &S11i[l]);
		lag_sum(X2R, X2I, X2R, X2I, l, st, en, &S22r[l], &S22i[l]);
	}
	for(l = -(AJ_NT - 1); l < AJ_NT; l++)
		lag_sum(X1R, X1I, X2R, X2I, l, st, en,
		        &S12r[l + AJ_NT - 1], &S12i[l + AJ_NT - 1]);

	/* F_ij, i <= j: sum over n in [st,en) of conj(x_p[n-a]) x_q[n-b]
	 *             = S_pq(b-a) + sum_[st-a,st) - sum_[en-a,en)
	 * (the window of the lag sum, shifted by a). The edge sums are short. */
	inv = 1.0 / (double)(en - st);
	for(i = 0; i < AJ_NSPT; i++) {
		int p = (i < AJ_NT) ? 1 : 2, a = i % AJ_NT;
		const int16_t *pr, *pi;
		chan(p, &pr, &pi);
		for(j = i; j < AJ_NSPT; j++) {
			int q = (j < AJ_NT) ? 1 : 2, b = j % AJ_NT;
			const int16_t *qr, *qi;
			int64_t sr, si, hr, hi, tr, ti;
			l = b - a;
			chan(q, &qr, &qi);
			if(p == 1 && q == 1)      { sr = S11r[l]; si = S11i[l]; }
			else if(p == 2 && q == 2) { sr = S22r[l]; si = S22i[l]; }
			else                      { sr = S12r[l + AJ_NT - 1]; si = S12i[l + AJ_NT - 1]; }
			lag_sum(pr, pi, qr, qi, l, st - a, st, &hr, &hi);
			lag_sum(pr, pi, qr, qi, l, en - a, en, &tr, &ti);
			FR[i][j] = (double)(sr + hr - tr) * inv;
			FI[i][j] = (double)(si + hi - ti) * inv;
			FR[j][i] = FR[i][j];
			FI[j][i] = -FI[i][j];
		}
	}
	/* Image taps (v_32 = conj(x1[n-8]), v_33 = conj(x2[n-8])):
	 *   F[img_p][j]     = E[x_p[n-8] v_j]               (j a space-time tap)
	 *   F[img_p][img_q] = conj(F[(p,8)][(q,8)])                            */
	for(i = AJ_IMG1; i <= AJ_IMG2; i++) {
		int p = (i == AJ_IMG1) ? 1 : 2, ri = (p == 1) ? AJ_REF : AJ_NT + AJ_REF;
		const int16_t *pr, *pi;
		chan(p, &pr, &pi);
		for(j = 0; j < AJ_NSPT; j++) {
			int q = (j < AJ_NT) ? 1 : 2, b = j % AJ_NT;
			const int16_t *qr, *qi;
			int64_t sr, si;
			chan(q, &qr, &qi);
			comp_sum(pr, pi, AJ_REF, qr, qi, b, st, en, &sr, &si);
			FR[i][j] = (double)sr * inv;
			FI[i][j] = (double)si * inv;
			FR[j][i] = FR[i][j];
			FI[j][i] = -FI[i][j];
		}
		for(j = AJ_IMG1; j <= AJ_IMG2; j++) {
			int rj = (j == AJ_IMG1) ? AJ_REF : AJ_NT + AJ_REF;
			FR[i][j] = FR[ri][rj];
			FI[i][j] = -FI[ri][rj];
		}
	}
	/* The core filters g2 x2, not x2: scale the RX2 rows and columns. */
	if(g2 != 1.0) {
		for(i = 0; i < AJ_NW; i++)
			for(j = 0; j < AJ_NW; j++) {
				double f = (is_rx2(i) ? g2 : 1.0) * (is_rx2(j) ? g2 : 1.0);
				FR[i][j] *= f; FI[i][j] *= f;
			}
	}
	return en - st;
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

/* ---- solve (M + s2 D) w = g, Hermitian positive definite ----------------- */

static int chol_solve(int n, double s2, const double *ld, const double *gr,
                      const double *gi, double *wr, double *wi)
{
	double yr[AJ_MAXDIM], yi[AJ_MAXDIM];
	int i, j, k;

	for(j = 0; j < n; j++) {
		double d = MR[j][j] + s2 * ld[j];
		for(k = 0; k < j; k++) d -= LR[j][k] * LR[j][k] + LI[j][k] * LI[j][k];
		if(d <= 0.0) return -1;
		d = aj_sqrt(d);
		LR[j][j] = d; LI[j][j] = 0.0;
		for(i = j + 1; i < n; i++) {
			double tr = MR[i][j], ti = MI[i][j];
			for(k = 0; k < j; k++) {
				/* t -= L[i][k] * conj(L[j][k]) */
				tr -= LR[i][k] * LR[j][k] + LI[i][k] * LI[j][k];
				ti -= LI[i][k] * LR[j][k] - LR[i][k] * LI[j][k];
			}
			LR[i][j] = tr / d; LI[i][j] = ti / d;
		}
	}
	for(i = 0; i < n; i++) {                       /* L y = g */
		double tr = gr[i], ti = gi[i];
		for(k = 0; k < i; k++) {
			tr -= LR[i][k] * yr[k] - LI[i][k] * yi[k];
			ti -= LR[i][k] * yi[k] + LI[i][k] * yr[k];
		}
		yr[i] = tr / LR[i][i]; yi[i] = ti / LR[i][i];
	}
	for(i = n - 1; i >= 0; i--) {                  /* L^H w = y */
		double tr = yr[i], ti = yi[i];
		for(k = i + 1; k < n; k++) {
			/* t -= conj(L[k][i]) * w[k] */
			tr -= LR[k][i] * wr[k] + LI[k][i] * wi[k];
			ti -= LR[k][i] * wi[k] - LI[k][i] * wr[k];
		}
		wr[i] = tr / LR[i][i]; wi[i] = ti / LR[i][i];
	}
	return 0;
}

/* Weights of the previous block, for the out-of-sample check (they are the
 * weights the FPGA was actually running while this block was captured). */
static int    prev_n = 0, prev_idx[AJ_MAXDIM];
static double prev_wr[AJ_MAXDIM], prev_wi[AJ_MAXDIM], prev_nr[AJ_MAXDIM];

void aj_reset(void)
{
	prev_n = 0;
	avg_valid = 0;
}

/* E|y|^2 for weights (wr,wi) on taps idx[], with statistics (fr,fi). */
static double out_power(double (*fr)[AJ_NW], double (*fi)[AJ_NW],
                        int n, const int *idx, const double *wr, const double *wi)
{
	double mr = 0.0, qr = 0.0;
	int i, j;
	for(i = 0; i < n; i++) {
		double ar = 0.0, ai = 0.0;
		mr += wr[i] * fr[idx[i]][AJ_REF] + wi[i] * fi[idx[i]][AJ_REF];
		for(j = 0; j < n; j++) {
			ar += fr[idx[i]][idx[j]] * wr[j] - fi[idx[i]][idx[j]] * wi[j];
			ai += fr[idx[i]][idx[j]] * wi[j] + fi[idx[i]][idx[j]] * wr[j];
		}
		qr += wr[i] * ar + wi[i] * ai;
	}
	return fr[AJ_REF][AJ_REF] - 2.0 * mr + qr;
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

static int max_ws = AJ_MAXWS;
void aj_set_max_ws(int ws) { max_ws = (ws < 0) ? 0 : (ws > AJ_MAXWS) ? AJ_MAXWS : ws; }

static int16_t q312(double v)
{
	double s = v * (double)(1 << AJ_WFRAC);
	s = (s >= 0.0) ? s + 0.5 : s - 0.5;
	if(s > 32767.0)  return 32767;
	if(s < -32768.0) return -32768;
	return (int16_t)s;
}

void aj_metrics(aj_result_t *r, double noise_ref)
{
	/* The canceller's own noise estimate (r->sig2) also contains any jammer
	 * it could not remove, so on its own it would always report a perfect
	 * null. noise_ref is the receiver's noise floor learned while there was
	 * no jammer (scaled to the present RX gain). */
	double noise = r->sig2, jin, nout, jout;
	/* Trust the learned floor unless the canceller output proves the noise
	 * is lower by more than 3 dB (the floor was learned with a jammer on). */
	if(noise_ref > 0.0 && r->sig2 > 0.5 * noise_ref) noise = noise_ref;
	if(noise < AJ_QFLOOR) noise = AJ_QFLOOR;
	r->noise = noise;
	jin  = r->p1 - noise;
	nout = noise * (1.0 + r->pout_wn2);
	jout = r->pout - nout;
	r->jn_db  = (jin > 0.0) ? aj_db10(jin / noise) : -99.0;
	r->res_db = aj_db10((r->pout > 0.0 ? r->pout : AJ_QFLOOR) / nout);
	if(jin <= 0.0)               r->supp_db = 0.0;
	else if(jout <= jin * 1e-6)  r->supp_db = 60.0;
	else                         r->supp_db = aj_db10(jin / jout);
	if(r->supp_db < 0.0) r->supp_db = 0.0;
}

int aj_compute(const uint32_t *buf, int nsamp, aj_result_t *r)
{
	int idx[AJ_MAXDIM], n = 0, i, j, k, it, cnt;
	double gr[AJ_MAXDIM], gi[AJ_MAXDIM], wr[AJ_MAXDIM], wi[AJ_MAXDIM], ld[AJ_MAXDIM];
	double nr[AJ_MAXDIM];
	double p1, p2, s2, pout = 0.0, wn2 = 0.0, lo, hi, pev, pev_wn2, g2, nr2;

	r->ok = 0;
	if(!buf || nsamp != AJ_NSAMP) return -1;
	g2 = (r->g2 > 0.0) ? r->g2 : 1.0;
	nr2 = (r->rx2_noise > 0.0) ? r->rx2_noise : 1.0;
	cnt = build_stats(buf, nsamp, g2);
	coherence(r, cnt);

	/* statistics for the solve: this block, averaged with the last ones */
	if(avg_valid && AJ_AVG > 0.0) {
		for(i = 0; i < AJ_NW; i++)
			for(j = 0; j < AJ_NW; j++) {
				AR[i][j] = AJ_AVG * AR[i][j] + (1.0 - AJ_AVG) * FR[i][j];
				AI[i][j] = AJ_AVG * AI[i][j] + (1.0 - AJ_AVG) * FI[i][j];
			}
	} else {
		for(i = 0; i < AJ_NW; i++)
			for(j = 0; j < AJ_NW; j++) { AR[i][j] = FR[i][j]; AI[i][j] = FI[i][j]; }
	}
	avg_valid = 1;

	/* active taps */
	if(r->mode == AJ_MODE_NB) {
		idx[n++] = AJ_NT + AJ_REF;
	} else {
		for(k = (r->mode == AJ_MODE_RX2) ? AJ_NT : 0; k < AJ_NSPT; k++)
			if(k != AJ_REF) idx[n++] = k;
		if(r->mode != AJ_MODE_LIN) { idx[n++] = AJ_IMG1; idx[n++] = AJ_IMG2; }
	}
	for(i = 0; i < n; i++) {
		for(j = 0; j < n; j++) {
			MR[i][j] = AR[idx[i]][idx[j]];
			MI[i][j] = AI[idx[i]][idx[j]];
		}
		gr[i] = AR[idx[i]][AJ_REF];
		gi[i] = AI[idx[i]][AJ_REF];
		/* RX1's own taps get 4x (6 dB) the loading. They are there for a
		 * reflected jammer, which is far above the noise; without the extra
		 * loading they would also start predicting (and so removing) any
		 * narrowband signal that sits just under the noise -- the
		 * satellites. Measured on the testbench satellite: 0.58 dB lost
		 * with 1x, < 0.2 dB with 4x, no change in jammer suppression. */
		/* Each tap is loaded with ITS noise: RX2's normalised noise is
		 * nr2 x RX1's when g2 could not equalise them (|g2| < 4). */
		nr[i] = is_rx2(idx[i]) ? nr2 : 1.0;
		ld[i] = ((idx[i] < AJ_NT) ? AJ_RX1_LOAD : 1.0) * nr[i];
	}
	p1 = FR[AJ_REF][AJ_REF];
	p2 = FR[AJ_NT + AJ_REF][AJ_NT + AJ_REF] / (g2 * g2);

	/* Out-of-sample output power: the PREVIOUS weights (the ones the FPGA
	 * ran while this block was captured) on THIS block. Unlike the power
	 * of the new weights on their own training block, this includes a jammer
	 * that has moved, appeared or that the filter cannot follow, so it is
	 * what the suppression and residual figures are computed from. */
	pev = -1.0; pev_wn2 = 0.0;
	if(prev_n > 0) {
		pev = out_power(FR, FI, prev_n, prev_idx, prev_wr, prev_wi);
		for(k = 0; k < prev_n; k++) pev_wn2 += (prev_wr[k] * prev_wr[k] + prev_wi[k] * prev_wi[k]) * prev_nr[k];
	}
	hi = AR[AJ_REF][AJ_REF];
	if(AR[AJ_NT + AJ_REF][AJ_NT + AJ_REF] < hi) hi = AR[AJ_NT + AJ_REF][AJ_NT + AJ_REF];
	if(hi < AJ_QFLOOR) hi = AJ_QFLOOR;
	lo = AJ_QFLOOR;

	/* Loading = noise power, from the canceller's own output: start from the
	 * last estimate (or the input power), solve, take Pout/(1+|w|^2), repeat
	 * until it stops moving. Converges in 1-2 passes once tracking, in
	 * ~5 from a cold start into a 40 dB jammer. */
	s2 = r->sig2_prev;
	if(s2 < lo || s2 > hi) s2 = hi;
	for(it = 0; it < 10; it++) {
		double ns, ratio;
		if(chol_solve(n, s2, ld, gr, gi, wr, wi) != 0) return -2;
		wn2 = 0.0;
		for(k = 0; k < n; k++) wn2 += (wr[k] * wr[k] + wi[k] * wi[k]) * nr[k];
		pout = out_power(AR, AI, n, idx, wr, wi);
		ns = pout / (1.0 + wn2);
		if(ns < lo) ns = lo;
		if(ns > hi) ns = hi;
		ratio = ns / s2;
		s2 = ns;
		if(ratio > 0.79 && ratio < 1.26) break;
	}
	r->iters = it + 1;

	/* Weight scale: the FPGA divides the sum by 2^ws, so the weights can be
	 * written ws bits larger. The largest ws that keeps every component
	 * inside Q3.12 (|v| < 7.5) gives the finest steps: a strong CW is a
	 * minimum-norm null spread over 16 taps (~0.05 each) and needs the
	 * weights to ~1e-4, finer than one Q3.12 step (2.4e-4). */
	{
		double mx = 0.0;
		int ws = 0;
		for(i = 0; i < n; i++) {
			if(wr[i] > mx) mx = wr[i];
			if(-wr[i] > mx) mx = -wr[i];
			if(wi[i] > mx) mx = wi[i];
			if(-wi[i] > mx) mx = -wi[i];
		}
		while(ws < max_ws && mx * (double)(2 << ws) < 7.5) ws++;
		r->ws = ws;
	}
	for(k = 0; k < AJ_NW; k++) { r->wre[k] = 0; r->wim[k] = 0; }
	for(i = 0; i < n; i++) {
		r->wre[idx[i]] = q312(wr[i] * (double)(1 << r->ws));
		r->wim[idx[i]] = q312(wi[i] * (double)(1 << r->ws));
	}

	r->p1 = p1; r->p2 = p2; r->sig2 = s2; r->wn2 = wn2;
	if(pev < 0.0) {                 /* first block: no history */
		pev = out_power(FR, FI, n, idx, wr, wi);
		pev_wn2 = wn2;
	}
	r->pout = pev; r->pout_wn2 = pev_wn2;
	for(i = 0; i < n; i++) {      /* what the FPGA will run: the quantised values */
		prev_idx[i] = idx[i];
		prev_wr[i] = (double)r->wre[idx[i]] / (double)(1 << AJ_WFRAC) / (double)(1 << r->ws);
		prev_wi[i] = (double)r->wim[idx[i]] / (double)(1 << AJ_WFRAC) / (double)(1 << r->ws);
		prev_nr[i] = nr[i];
	}
	prev_n = n;
	aj_metrics(r, r->noise_ref);

	/* Spectrum of what is above the noise, from RX1's lag correlations. */
	spectrum(r, cnt, p1, r->noise);
	r->ok = 1;
	return 0;
}
