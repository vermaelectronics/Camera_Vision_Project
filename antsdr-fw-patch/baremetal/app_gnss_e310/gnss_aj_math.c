/******************************************************************************
 *  gnss_aj_math.c
 *  Space-frequency SMI power inversion (SFAP-PI v5.0): the weight
 *  computation. See gnss_aj_math.h.
 *
 *  antsdr-fw-patch crpa6: the v5.0 solver unchanged in its mathematics; added
 *  around it what the SMI-PI 4.9.14 engine (gnss_aj.c) uses: DC removal, the
 *  averaging factor (manual / adaptive / per solve), the pulse gate, the sweep
 *  detector and the output make-up rule. FFT size from AJ_LOG2N (64 or 128 bins).
 *****************************************************************************/

/* The Vitis XSCT flow builds the application in its Debug configuration
 * (-O0). This file does ~0.5 million multiply-accumulates per weight update,
 * which at -O0 would cut the update rate several-fold, so it asks GCC for
 * full optimisation itself. Nothing in it depends on debug code generation. */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize ("O3")
#endif

#include "gnss_aj_math.h"
#ifdef AJ_DEBUG
#include <stdio.h>
#endif

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

/* ---- trigonometry for the tables (no libm) -------------------------------- */

static double aj_sin2pi(double f)          /* sin(2 pi f), any f in [0, 1) */
{
	double t = 2.0 * AJ_PI * f;
	if(t <= AJ_PI) return aj_sin(t);
	return -aj_sin(t - AJ_PI);
}

static double aj_cos2pi(double f)
{
	f += 0.25;
	if(f >= 1.0) f -= 1.0;
	return aj_sin2pi(f);
}

/* ---- block data ------------------------------------------------------------ */

static int16_t X1R[AJ_NSAMP], X1I[AJ_NSAMP], X2R[AJ_NSAMP], X2I[AJ_NSAMP];

/* S_pq(l) = sum_{t=st}^{en-1} conj(x_p[t]) x_q[t-l]   (detection, spectrum) */
static int64_t S11r[AJ_NLAG], S11i[AJ_NLAG];
static int64_t S22r[1], S22i[1];
static int64_t S12r[2 * AJ_NLAG - 1], S12i[2 * AJ_NLAG - 1];

static int16_t sx12(uint32_t v)
{
	int32_t s = (int32_t)(v & 0xFFFU);
	return (int16_t)((s & 0x800) ? s - 0x1000 : s);
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
	/* crpa6 (as SMI-PI 4.9): remove each channel's DC (the AD9361's LO leakage /
	 * DC offset): a constant is "seen" by both channels at every lag and would
	 * look like a jammer to the coherence detector and the image taps. */
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

/* Each product of two 12-bit samples is below 2^22, so 128 of them (two per
 * term) fit a 32-bit accumulator: summing in chunks of AJ_CHUNK in 32 bits
 * and only then in 64 is exact and about twice as fast on the Cortex-A9. */
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

/* The lag sums detection and the jammer description need. Returns the
 * number of terms. */
static int lag_stats(void)
{
	const int st = 2 * AJ_NLAG, en = AJ_NSAMP - AJ_NLAG;
	int l;
	lag_sum(X1R, X1I, X1R, X1I, 0, st, en, &S11r[0], &S11i[0]);
	lag_sum(X1R, X1I, X1R, X1I, 1, st, en, &S11r[1], &S11i[1]);
	lag_sum(X1R, X1I, X1R, X1I, AJ_NLAG - 1, st, en, &S11r[AJ_NLAG - 1], &S11i[AJ_NLAG - 1]);
	lag_sum(X2R, X2I, X2R, X2I, 0, st, en, &S22r[0], &S22i[0]);
	for(l = -(AJ_NLAG - 1); l < AJ_NLAG; l++)
		lag_sum(X1R, X1I, X2R, X2I, l, st, en,
		        &S12r[l + AJ_NLAG - 1], &S12i[l + AJ_NLAG - 1]);
	return en - st;
}

/* RX1-RX2 coherence |E[conj(x1[t]) x2[t-l]]| / sqrt(P1 P2), best lag.
 * A jammer both antennas see makes it J/(J+N); noise alone ~1/sqrt(cnt). */
static void coherence(aj_result_t *r)
{
	double p1 = (double)S11r[0], p2 = (double)S22r[0], best = 0.0, c;
	int l, lag = 0;
	if(p1 > 0.0 && p2 > 0.0) {
		double den = 1.0 / (p1 * p2);
		for(l = 0; l < 2 * AJ_NLAG - 1; l++) {
			double re = (double)S12r[l], im = (double)S12i[l];
			double m2 = (re * re + im * im) * den;
			if(m2 > best) { best = m2; lag = l - (AJ_NLAG - 1); }
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
			double c15r = (double)S11r[AJ_NLAG - 1] * inv, c15i = (double)S11i[AJ_NLAG - 1] * inv;
			j15 = aj_sqrt(c15r * c15r + c15i * c15i) / (p1 - s2);
			if(j15 > 1.0) j15 = 1.0;
			r->bw = aj_sincinv(j15) / (double)(AJ_NLAG - 1);
		}
	}
}

static void detect_core(aj_result_t *r, int cnt)
{
	double inv = 1.0 / (double)cnt, noise;
	r->p1 = (double)S11r[0] * inv;
	r->p2 = (double)S22r[0] * inv;
	coherence(r);
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
}

int aj_detect(const uint32_t *buf, int nsamp, aj_result_t *r)
{
	int cnt;
	r->ok = 0;
	if(!buf || nsamp != AJ_NSAMP) return -1;
	unpack(buf, nsamp);
	cnt = lag_stats();
	detect_core(r, cnt);
	r->ok = 1;
	return 0;
}

/* ---- the FPGA's analysis filter bank, in double ------------------------------ */

/* Window sin(pi n / N) (= the FPGA's SFAP_WIN / 65536) and twiddles. */
static double WIN[AJ_N], TWR[AJ_N / 2], TWI[AJ_N / 2];
static int    BREV[AJ_N];
static int    tables_ok;
#define AJ_ENORM ((double)AJ_N / 2.0)    /* sum of w^2: white noise s2 -> bin s2 x ENORM */
#define AJ_NFRM  ((AJ_NSAMP - AJ_N) / AJ_HOP + 1)   /* 127 frames per capture       */

static void tables(void)
{
	int n, b;
	if(tables_ok) return;
	for(n = 0; n < AJ_N; n++) {
		/* the FPGA's Q16 values, so both analyse with the same window */
		double w = aj_sin2pi(0.5 * (double)n / (double)AJ_N);
		WIN[n] = (double)(int32_t)(w * 65536.0 + 0.5) / 65536.0;
		BREV[n] = 0;
		for(b = 0; b < AJ_LOG2N; b++)
			if(n & (1 << b)) BREV[n] |= 1 << (AJ_LOG2N - 1 - b);
	}
	for(n = 0; n < AJ_N / 2; n++) {
		TWR[n] =  aj_cos2pi((double)n / (double)AJ_N);
		TWI[n] = -aj_sin2pi((double)n / (double)AJ_N);
	}
	tables_ok = 1;
}

/* In-place radix-2 FFT of N points, natural order in and out. */
static void fft(double *re, double *im)
{
	int i, j, len, half, step;
	for(i = 0; i < AJ_N; i++) {
		j = BREV[i];
		if(j > i) {
			double t = re[i]; re[i] = re[j]; re[j] = t;
			t = im[i]; im[i] = im[j]; im[j] = t;
		}
	}
	for(len = 2; len <= AJ_N; len <<= 1) {
		half = len >> 1;
		step = AJ_N / len;
		for(i = 0; i < AJ_N; i += len) {
			for(j = 0; j < half; j++) {
				double wr = TWR[j * step], wi = TWI[j * step];
				double ar = re[i + j], ai = im[i + j];
				double br = re[i + j + half], bi = im[i + j + half];
				double tr = br * wr - bi * wi, ti = br * wi + bi * wr;
				re[i + j] = ar + tr;          im[i + j] = ai + ti;
				re[i + j + half] = ar - tr;   im[i + j + half] = ai - ti;
			}
		}
	}
}

/* ---- per-bin statistics -------------------------------------------------------- */

/* F_k[i][j] = E[conj(v_i) v_j] over the frames of one block, bin k:
 *   v_0 = X1_k (the reference d), v_1 = X2n_k,
 *   v_2 = conj(X1_{-k}), v_3 = conj(X2n_{-k})                            */
#define AJ_NV 4
static double FR[AJ_N][AJ_NV][AJ_NV], FI[AJ_N][AJ_NV][AJ_NV];
/* The same, averaged over blocks (what the solve uses). */
static double AR[AJ_N][AJ_NV][AJ_NV], AI[AJ_N][AJ_NV][AJ_NV];
static int    avg_valid;

/* The weights a WOLA pass applies: per bin k up to 3 taps on v_1..v_3. */
static int    wola_n, wola_idx[AJ_MAXDIM];
static double wola_wr[AJ_N][AJ_MAXDIM], wola_wi[AJ_N][AJ_MAXDIM];
static double OLR[AJ_NSAMP], OLI[AJ_NSAMP];

/* Frame f's windowed FFTs of x1 and g2 x2 into b1, b2. */
static void frame_fft(int f, double g2, double *b1r, double *b1i, double *b2r, double *b2i)
{
	int n, st = f * AJ_HOP;
	for(n = 0; n < AJ_N; n++) {
		double w = WIN[n];
		b1r[n] = w * X1R[st + n];       b1i[n] = w * X1I[st + n];
		b2r[n] = w * g2 * X2R[st + n];  b2i[n] = w * g2 * X2I[st + n];
	}
	fft(b1r, b1i);
	fft(b2r, b2i);
}

/* The FPGA's output for frame f with the wola_* weights: Y_k, inverse FFT,
 * synthesis window, added into OL at the frame's position. */
static void frame_out(int f, const double *b1r, const double *b1i,
                      const double *b2r, const double *b2i)
{
	double yr[AJ_N], yi[AJ_N];
	int k, i, n, st = f * AJ_HOP;
	for(k = 0; k < AJ_N; k++) {
		int m = (AJ_N - k) & (AJ_N - 1);
		double ar = b1r[k], ai = b1i[k];
		for(i = 0; i < wola_n; i++) {
			double vr, vi, wr = wola_wr[k][i], wi = wola_wi[k][i];
			switch(wola_idx[i]) {
			case 1:  vr = b2r[k]; vi =  b2i[k]; break;
			case 2:  vr = b1r[m]; vi = -b1i[m]; break;
			default: vr = b2r[m]; vi = -b2i[m]; break;
			}
			ar -= wr * vr - wi * vi;
			ai -= wr * vi + wi * vr;
		}
		/* inverse FFT as conj(FFT(conj(Y))) / N */
		yr[k] = ar; yi[k] = -ai;
	}
	fft(yr, yi);
	for(n = 0; n < AJ_N; n++) {
		double w = WIN[n] / (double)AJ_N;
		OLR[st + n] += w * yr[n];
		OLI[st + n] -= w * yi[n];
	}
}

/* Power of the overlap-added output where both frame chains cover it. */
static double ol_power(void)
{
	double p = 0.0;
	int t;
	for(t = AJ_N; t < AJ_NSAMP - AJ_N; t++) p += OLR[t] * OLR[t] + OLI[t] * OLI[t];
	return p / (double)(AJ_NSAMP - 2 * AJ_N);
}

static void ol_clear(void)
{
	int t;
	for(t = 0; t < AJ_NSAMP; t++) { OLR[t] = 0.0; OLI[t] = 0.0; }
}

/* Builds F from one block and, with wola_n > 0, also runs the wola_*
 * weights through the FPGA's synthesis on the same frames (OL). */
static void build_stats(double g2)
{
	static double b1r[AJ_N], b1i[AJ_N], b2r[AJ_N], b2i[AJ_N];
	double vr[AJ_NV], vi[AJ_NV], inv = 1.0 / (double)AJ_NFRM;
	int f, k, i, j;

	for(k = 0; k < AJ_N; k++)
		for(i = 0; i < AJ_NV; i++)
			for(j = 0; j < AJ_NV; j++) { FR[k][i][j] = 0.0; FI[k][i][j] = 0.0; }
	if(wola_n > 0) ol_clear();

	for(f = 0; f < AJ_NFRM; f++) {
		frame_fft(f, g2, b1r, b1i, b2r, b2i);
		if(wola_n > 0) frame_out(f, b1r, b1i, b2r, b2i);
		for(k = 0; k < AJ_N; k++) {
			int m = (AJ_N - k) & (AJ_N - 1);
			vr[0] = b1r[k]; vi[0] =  b1i[k];
			vr[1] = b2r[k]; vi[1] =  b2i[k];
			vr[2] = b1r[m]; vi[2] = -b1i[m];
			vr[3] = b2r[m]; vi[3] = -b2i[m];
			for(i = 0; i < AJ_NV; i++)
				for(j = i; j < AJ_NV; j++) {
					/* conj(v_i) v_j */
					FR[k][i][j] += vr[i] * vr[j] + vi[i] * vi[j];
					FI[k][i][j] += vr[i] * vi[j] - vi[i] * vr[j];
				}
		}
	}
	for(k = 0; k < AJ_N; k++)
		for(i = 0; i < AJ_NV; i++)
			for(j = i; j < AJ_NV; j++) {
				FR[k][i][j] *= inv;  FI[k][i][j] *= inv;
				FR[k][j][i] = FR[k][i][j];
				FI[k][j][i] = -FI[k][i][j];
			}
}

/* The FPGA output power of the wola_* weights on this block (a second pass
 * over the frames; only needed when there were no previous weights). */
static double wola_power(double g2)
{
	static double b1r[AJ_N], b1i[AJ_N], b2r[AJ_N], b2i[AJ_N];
	int f;
	ol_clear();
	for(f = 0; f < AJ_NFRM; f++) {
		frame_fft(f, g2, b1r, b1i, b2r, b2i);
		frame_out(f, b1r, b1i, b2r, b2i);
	}
	return ol_power();
}

/* ---- solve (M + s2 D) w = g, Hermitian positive definite, n <= 3 ---------- */

static int chol_solve(int n, double (*mr)[AJ_MAXDIM], double (*mi)[AJ_MAXDIM],
                      double s2, const double *ld, const double *gr,
                      const double *gi, double *wr, double *wi)
{
	double LR[AJ_MAXDIM][AJ_MAXDIM], LI[AJ_MAXDIM][AJ_MAXDIM];
	double yr[AJ_MAXDIM], yi[AJ_MAXDIM];
	int i, j, k;

	for(j = 0; j < n; j++) {
		double d = mr[j][j] + s2 * ld[j];
		for(k = 0; k < j; k++) d -= LR[j][k] * LR[j][k] + LI[j][k] * LI[j][k];
		if(d <= 0.0) return -1;
		d = aj_sqrt(d);
		LR[j][j] = d; LI[j][j] = 0.0;
		for(i = j + 1; i < n; i++) {
			double tr = mr[i][j], ti = mi[i][j];
			for(k = 0; k < j; k++) {
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
			tr -= LR[k][i] * wr[k] + LI[k][i] * wi[k];
			ti -= LR[k][i] * wi[k] - LI[k][i] * wr[k];
		}
		wr[i] = tr / LR[i][i]; wi[i] = ti / LR[i][i];
	}
	return 0;
}

/* E|d - w^T v|^2 for weights w on taps idx[] (1..3), statistics f. */
static double out_power(double (*fr)[AJ_NV], double (*fi)[AJ_NV],
                        int n, const int *idx, const double *wr, const double *wi)
{
	double mr = 0.0, qr = 0.0;
	int i, j;
	for(i = 0; i < n; i++) {
		double ar = 0.0, ai = 0.0;
		mr += wr[i] * fr[idx[i]][0] + wi[i] * fi[idx[i]][0];
		for(j = 0; j < n; j++) {
			ar += fr[idx[i]][idx[j]] * wr[j] - fi[idx[i]][idx[j]] * wi[j];
			ai += fr[idx[i]][idx[j]] * wi[j] + fi[idx[i]][idx[j]] * wr[j];
		}
		qr += wr[i] * ar + wi[i] * ai;
	}
	return fr[0][0] - 2.0 * mr + qr;
}

/* Loaded solve of one set of statistics (a bin, or all bins summed for the
 * narrowband mode), loading iterated to the canceller's own noise. */
static int solve_one(double (*fr)[AJ_NV], double (*fi)[AJ_NV], int n, const int *idx,
                     const double *nr, double s2, double lo, double *wr, double *wi,
                     double *s2_out, double *wn2_out, int *iters)
{
	double MR[AJ_MAXDIM][AJ_MAXDIM], MI[AJ_MAXDIM][AJ_MAXDIM];
	double gr[AJ_MAXDIM], gi[AJ_MAXDIM], hi, wn2 = 0.0;
	int i, j, k, it;

	for(i = 0; i < n; i++) {
		for(j = 0; j < n; j++) { MR[i][j] = fr[idx[i]][idx[j]]; MI[i][j] = fi[idx[i]][idx[j]]; }
		gr[i] = fr[idx[i]][0];
		gi[i] = fi[idx[i]][0];
	}
	hi = fr[0][0];
	if(fr[1][1] < hi) hi = fr[1][1];
	if(hi < lo) hi = lo;
	if(s2 < lo || s2 > hi) s2 = hi;
	for(it = 0; it < 10; it++) {
		double ns, ratio, pout;
		if(chol_solve(n, MR, MI, s2, nr, gr, gi, wr, wi) != 0) return -1;
		wn2 = 0.0;
		for(k = 0; k < n; k++) wn2 += (wr[k] * wr[k] + wi[k] * wi[k]) * nr[k];
		pout = out_power(fr, fi, n, idx, wr, wi);
		ns = pout / (1.0 + wn2);
		if(ns < lo) ns = lo;
		if(ns > hi) ns = hi;
		ratio = ns / s2;
		s2 = ns;
		if(ratio > 0.79 && ratio < 1.26) break;
	}
	*s2_out = s2;
	*wn2_out = wn2;
	*iters = it + 1;
	return 0;
}

/* The weights of the previous block (the ones the FPGA ran while this block
 * was captured) are wola_*; prev_nr their taps' relative noise. And each
 * bin's last loading. */
static double prev_nr[AJ_MAXDIM];
static double s2_bin[AJ_N];
static int    s2_valid;

/* ---- averaging factor (as SMI-PI 4.9.9) ------------------------------------------ */
static double av_adapt = AJ_AVG;      /* the factor in use (AJ_AVG when not adapting) */
static int    av_auto = AJ_AV_ADAPT;  /* 1 = adapt it, 0 = keep the manual value      */
static double av_manual = AJ_AVG;     /* the value set by hand (gnss_aj_avg=)         */

void aj_set_av(double v)
{
	if(v < 0.05) v = 0.05;
	if(v > 0.995) v = 0.995;
	av_manual = v; av_adapt = v; av_auto = 0;
}
void aj_set_av_auto(void)
{
	av_auto = 1; av_adapt = AJ_AVG;
}
int    aj_av_is_auto(void) { return av_auto; }
double aj_av_value(void)   { return av_adapt; }

void aj_reset(void)
{
	wola_n = 0;
	avg_valid = 0;
	s2_valid = 0;
	av_adapt = av_auto ? AJ_AVG : av_manual;
}

void aj_set_max_ws(int ws) { (void)ws; }   /* SFAP has no weight scale */

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

void aj_sweep_reset(aj_sweep_t *s)
{
	s->last = 0.0; s->have = 0; s->score = 0; s->on = 0;
}

double aj_sweep_update(aj_sweep_t *s, const aj_result_t *r)
{
	double d;

	if(!r->ok || r->bw >= AJ_SWEEP_BW) {      /* wide inside the block: not a sweep */
		s->have = 0;
		if(s->score > 0) s->score--;
	} else {
		if(s->have) {
			d = r->foff - s->last;
			if(d < 0.0) d = -d;
			if(d > 0.5) d = 1.0 - d;
			if(d > AJ_SWEEP_STEP) s->score += 5;
			else if(s->score > 0) s->score--;
			if(s->score > AJ_SWEEP_MAX) s->score = AJ_SWEEP_MAX;
		}
		s->last = r->foff; s->have = 1;
	}
	if(s->score >= AJ_SWEEP_ON) s->on = 1;
	else if(s->score == 0) s->on = 0;
	return s->on ? AJ_SWEEP_AVG : 0.0;
}

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
	int idx[AJ_MAXDIM], n = 0, i, j, k, cnt, itmax = 0;
	double nr[AJ_MAXDIM], wr[AJ_MAXDIM], wi[AJ_MAXDIM];
	double g2, nr2, lo, pev = 0.0, pev_wn2 = 0.0, s2sum = 0.0, wn2sum = 0.0;
	double nbin, av, jnum = 0.0, jden = 0.0;
	int    same;

	r->ok = 0;
	if(!buf || nsamp != AJ_NSAMP) return -1;
	tables();
	g2  = (r->g2 > 0.0) ? r->g2 : 1.0;
	nr2 = (r->rx2_noise > 0.0) ? r->rx2_noise : 1.0;
	if(r->sig2_prev <= 0.0) s2_valid = 0;

	unpack(buf, nsamp);
	cnt = lag_stats();
	detect_core(r, cnt);
	r->ws = 0;

	/* crpa6 (as SMI-PI 4.7). Pulsed jammer: a block captured in a gap holds no jammer.
	 * Averaged in, it would pull the weights back to "no jammer" and the next pulse
	 * would find them useless. Keep the old weights and statistics; the caller still
	 * gets this block's power and coherence (detect_core above). */
	r->gated = 0;
	if(r->gate && r->jn_coh_db < r->gate_coh_db &&
	   (r->noise_ref <= 0.0 || r->p1 < r->noise_ref * r->gate_pwr)) {
		r->gated = 1;
		r->ok = 1;
		return 1;
	}

	/* statistics, and the previous weights' output on this block (OL) */
	build_stats(g2);

	/* statistics for the solve: this block, averaged with the last ones */
	av = (r->avg > 0.0) ? r->avg : av_adapt;
	if(av > 0.995) av = 0.995;
	for(k = 0; k < AJ_N; k++)
		for(i = 0; i < AJ_NV; i++)
			for(j = 0; j < AJ_NV; j++) {
				if(avg_valid && av > 0.0) {
					AR[k][i][j] = av * AR[k][i][j] + (1.0 - av) * FR[k][i][j];
					AI[k][i][j] = av * AI[k][i][j] + (1.0 - av) * FI[k][i][j];
				} else {
					AR[k][i][j] = FR[k][i][j];
					AI[k][i][j] = FI[k][i][j];
				}
			}
	avg_valid = 1;

	/* active taps of a bin: 1 = X2n_k, 2 = conj(X1_-k), 3 = conj(X2n_-k) */
	idx[n] = 1; nr[n++] = nr2;
	if(r->mode == AJ_MODE_SFAP) {
		idx[n] = 2; nr[n++] = 1.0;
		idx[n] = 3; nr[n++] = nr2;
	}

	/* Out-of-sample output power: the PREVIOUS weights (what the FPGA ran
	 * while this block was captured) on THIS block, through the FPGA's own
	 * synthesis (window + overlap-add). The analysis-domain power of the bins
	 * would overstate it: the frame-edge errors of per-bin weighting are
	 * what the synthesis window removes. */
	if(wola_n > 0) {
		pev = ol_power();
		for(k = 0; k < AJ_N; k++)
			for(i = 0; i < wola_n; i++)
				pev_wn2 += (wola_wr[k][i] * wola_wr[k][i] + wola_wi[k][i] * wola_wi[k][i]) * prev_nr[i];
		pev_wn2 /= (double)AJ_N;
	}

	/* the same taps as the previous solve? (for the weight jitter below) */
	same = (wola_n == n);
	for(i = 0; same && i < n; i++) if(wola_idx[i] != idx[i]) same = 0;

	lo = AJ_QFLOOR * AJ_ENORM;
	for(k = 0; k < AJ_NW; k++) { r->wre[k] = 0; r->wim[k] = 0; }
	r->wmax = 0.0; r->wmax_bin = 0; r->jam_bins = 0;
	nbin = (r->noise > 0.0 ? r->noise : AJ_QFLOOR) * AJ_ENORM;

	if(r->mode == AJ_MODE_NB) {
		/* one weight: the statistics of all bins summed = the broadband,
		 * single-weight power inversion of v3 */
		static double SR[AJ_NV][AJ_NV], SI[AJ_NV][AJ_NV];
		double s2, wn2;
		int it;
		for(i = 0; i < AJ_NV; i++)
			for(j = 0; j < AJ_NV; j++) {
				SR[i][j] = 0.0; SI[i][j] = 0.0;
				for(k = 0; k < AJ_N; k++) { SR[i][j] += AR[k][i][j]; SI[i][j] += AI[k][i][j]; }
				SR[i][j] /= (double)AJ_N; SI[i][j] /= (double)AJ_N;
			}
		s2 = s2_valid ? s2_bin[0] : -1.0;
		if(solve_one(SR, SI, n, idx, nr, s2, lo, wr, wi, &s2, &wn2, &it) != 0) return -2;
		for(k = 0; k < AJ_N; k++) {
			double nwr = (double)q312(wr[0]) / 4096.0, nwi = (double)q312(wi[0]) / 4096.0;
			s2_bin[k] = s2;
			if(same) {
				double dr = nwr - wola_wr[k][0], di = nwi - wola_wi[k][0];
				jnum += dr * dr + di * di;
			}
			jden += nwr * nwr + nwi * nwi;
			wola_wr[k][0] = nwr;
			wola_wi[k][0] = nwi;
			r->wre[AJ_WB + k] = q312(wr[0]);
			r->wim[AJ_WB + k] = q312(wi[0]);
		}
		s2sum = s2 * AJ_N; wn2sum = wn2 * AJ_N; itmax = it;
		r->wmax = aj_sqrt(wr[0] * wr[0] + wi[0] * wi[0]);
	} else {
		for(k = 0; k < AJ_N; k++) {
			double s2 = s2_valid ? s2_bin[k] : -1.0, wn2, m;
			int it;
			if(solve_one(AR[k], AI[k], n, idx, nr, s2, lo, wr, wi, &s2, &wn2, &it) != 0)
				return -2;
			s2_bin[k] = s2;
			s2sum += s2;
			wn2sum += wn2;
			if(it > itmax) itmax = it;
			for(i = 0; i < n; i++) {
				int16_t qr = q312(wr[i]), qi = q312(wi[i]);
				int word = (idx[i] == 1) ? AJ_WB + k : (idx[i] == 2) ? AJ_WC1 + k : AJ_WC2 + k;
				double nwr = (double)qr / 4096.0, nwi = (double)qi / 4096.0;
				r->wre[word] = qr;
				r->wim[word] = qi;
				if(same) {
					double dr = nwr - wola_wr[k][i], di = nwi - wola_wi[k][i];
					jnum += dr * dr + di * di;
				}
				jden += nwr * nwr + nwi * nwi;
				wola_wr[k][i] = nwr;                   /* what the FPGA will run */
				wola_wi[k][i] = nwi;
			}
			m = aj_sqrt(wr[0] * wr[0] + wi[0] * wi[0]);
			if(m > r->wmax) { r->wmax = m; r->wmax_bin = k; }
		}
	}
	s2_valid = 1;

	/* J/N per bin (vs. a white noise floor at noise_ref) */
	for(k = 0; k < AJ_N; k++) {
		double p = AR[k][0][0];
		r->bin_jn_db[k] = (p > nbin) ? aj_db10((p - nbin) / nbin) : -99.0;
		if(r->bin_jn_db[k] > 3.0) r->jam_bins++;
	}

	r->sig2 = s2sum / ((double)AJ_N * AJ_ENORM);
	r->wn2  = wn2sum / (double)AJ_N;
	r->iters = itmax;
	{
		int first = (wola_n == 0);
		for(i = 0; i < n; i++) { wola_idx[i] = idx[i]; prev_nr[i] = nr[i]; }
		wola_n = n;
		if(first) {               /* first block: no history, the new weights */
			pev = wola_power(g2);
			pev_wn2 = r->wn2;
		}
	}
	r->pout = pev;
	r->pout_wn2 = pev_wn2;
#ifdef AJ_DEBUG
	{
		printf("AJ_DEBUG p1 %.2f pout %.3f pout_wn2 %.3f sig2 %.3f noise_ref %.3f\n",
		       r->p1, pev, pev_wn2, r->sig2, r->noise_ref);
	}
#endif
	aj_metrics(r, r->noise_ref);
	/* crpa6 (as SMI-PI 4.9.9): adapt the averaging factor. Jitter = how much this solve
	 * moved the weights (all bins), relative to their size. Only when the factor is
	 * neither set by hand nor by the caller (a sweep sets r->avg itself). */
	if(av_auto && r->avg <= 0.0) {
		double jit = (same && jden > 0.0) ? jnum / jden : 1.0;
		if(r->res_db > AJ_RES_TRACK && jit < AJ_JIT_HI) av_adapt -= AJ_AV_STEP;   /* stable but not nulled: track faster */
		else if(jit > AJ_JIT_HI)                       av_adapt += AJ_AV_STEP;   /* jittering: smooth more */
		else                                           av_adapt += 0.1 * (AJ_AVG - av_adapt);   /* drift back to AJ_AVG */
		if(av_adapt < AJ_AV_MIN) av_adapt = AJ_AV_MIN;
		if(av_adapt > AJ_AV_MAX) av_adapt = AJ_AV_MAX;
	}
	spectrum(r, cnt, r->p1, r->noise);
	r->ok = 1;
	return 0;
}
