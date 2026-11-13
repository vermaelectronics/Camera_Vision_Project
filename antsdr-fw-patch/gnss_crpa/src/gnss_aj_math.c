/******************************************************************************
 *  gnss_aj_math.c
 *  Space-frequency SMI power inversion (SFAP-PI v6.0): the weight
 *  computation. See gnss_aj_math.h.
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
static int64_t S11r[AJ_NT], S11i[AJ_NT];
static int64_t S22r[1], S22i[1];
static int64_t S12r[2 * AJ_NT - 1], S12i[2 * AJ_NT - 1];

static int16_t sx12(uint32_t v)
{
	int32_t s = (int32_t)(v & 0xFFFU);
	return (int16_t)((s & 0x800) ? s - 0x1000 : s);
}

static int pk2_last;

static void unpack(const uint32_t *buf, int nsamp)
{
	int t, pk = 0;
	for(t = 0; t < nsamp; t++) {
		uint32_t w1 = buf[2 * t], w2 = buf[2 * t + 1];
		X1R[t] = sx12(w1); X1I[t] = sx12(w1 >> 16);
		X2R[t] = sx12(w2); X2I[t] = sx12(w2 >> 16);
		{
			int a = X2R[t] < 0 ? -X2R[t] : X2R[t], b = X2I[t] < 0 ? -X2I[t] : X2I[t];
			if(a > pk) pk = a;
			if(b > pk) pk = b;
		}
	}
	pk2_last = pk;
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
	const int st = 2 * AJ_NT, en = AJ_NSAMP - AJ_NT;
	int l;
	lag_sum(X1R, X1I, X1R, X1I, 0, st, en, &S11r[0], &S11i[0]);
	lag_sum(X1R, X1I, X1R, X1I, 1, st, en, &S11r[1], &S11i[1]);
	lag_sum(X1R, X1I, X1R, X1I, AJ_NT - 1, st, en, &S11r[AJ_NT - 1], &S11i[AJ_NT - 1]);
	lag_sum(X2R, X2I, X2R, X2I, 0, st, en, &S22r[0], &S22i[0]);
	for(l = -(AJ_NT - 1); l < AJ_NT; l++)
		lag_sum(X1R, X1I, X2R, X2I, l, st, en,
		        &S12r[l + AJ_NT - 1], &S12i[l + AJ_NT - 1]);
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
			double c15r = (double)S11r[AJ_NT - 1] * inv, c15i = (double)S11i[AJ_NT - 1] * inv;
			j15 = aj_sqrt(c15r * c15r + c15i * c15i) / (p1 - s2);
			if(j15 > 1.0) j15 = 1.0;
			r->bw = aj_sincinv(j15) / (double)(AJ_NT - 1);
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

static void bin_scan(aj_result_t *r);

int aj_detect(const uint32_t *buf, int nsamp, aj_result_t *r)
{
	int cnt;
	r->ok = 0;
	if(!buf || nsamp != AJ_NSAMP) return -1;
	unpack(buf, nsamp);
	r->pk2 = pk2_last;
	cnt = lag_stats();
	detect_core(r, cnt);
	bin_scan(r);
	r->ok = 1;
	return 0;
}

/* ---- the FPGA's analysis filter bank, in double ------------------------------ */

/* Window sin(pi n / N) (= the FPGA's SFAP_WIN / 65536) and twiddles. */
static double WIN[AJ_N], TWR[AJ_N / 2], TWI[AJ_N / 2];
static int    BREV[AJ_N];
static int    tables_ok;
#define AJ_ENORM ((double)AJ_N / 2.0)    /* sum of w^2: white noise s2 -> bin s2 x ENORM */
/* v6.2: the solver's frames may be spaced AJ_SHOP apart (AJ_HOP = the
 * FPGA's 50 % overlap, or AJ_N = no overlap: half the FFTs per capture). */
#define AJ_NFRM  ((AJ_NSAMP - AJ_N) / AJ_SHOP + 1)  /* frames per capture           */

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

/* ---- v6.1: per-bin noise floor and narrowband detection ----------------------
 * PB[k]  : RX1 power in bin k of the last aj_detect() block, per sample
 *          (bin power / AJ_ENORM, so white noise of s2 LSB^2 gives s2).
 * NFB[k] : learned receiver noise in bin k at gain_lin = 1 [LSB^2]; the
 *          floor at gain g is NFB[k] g + AJ_QFLOOR (quantisation is white).
 * The detector uses 32 frames without overlap (half the work of the solve's
 * 63): the per-bin estimate scatters ~0.8 dB rms, the largest of 253 bins on
 * pure noise ~+2.5 dB, far under the 10 dB the engine needs to switch on.
 * The floor needs AJ_LEARN_N quiet blocks before the detector is armed. */
static double PB[AJ_N], NFB[AJ_N];
static int    pb_ok, nfb_ok;

static double bin_floor(int k, double g)          /* expected noise, bin k */
{
	return NFB[k] * g + AJ_QFLOOR;
}

static int bin_guarded(int k)                     /* next to DC: LO leakage */
{
	return k <= AJ_DC_GUARD || k >= AJ_N - AJ_DC_GUARD;
}

/* Largest per-bin J/N of powers p[k] (per sample), against the learned floor. */
static void bin_peak(aj_result_t *r, const double *p, double scale)
{
	int k;
	r->bin_jn_max = -99.0;
	r->bin_jn_kmax = 0;
	if(!nfb_ok || r->gain_lin <= 0.0) return;
	for(k = 0; k < AJ_N; k++) {
		double f = bin_floor(k, r->gain_lin), q = p[k] * scale, jn;
		if(bin_guarded(k) || q <= f) continue;
		jn = aj_db10((q - f) / f);
		if(jn > r->bin_jn_max) { r->bin_jn_max = jn; r->bin_jn_kmax = k; }
	}
}

/* Learning: the blocks the engine calls noise are averaged AJ_LEARN_N at a
 * time (per-bin scatter ~0.1 dB instead of ~0.8 dB), then the floor follows
 * that average down at once and up only at rise_db_s, like the broadband
 * floor. Without the averaging, "down at once" on single blocks would sit
 * the floor on the low tail of their scatter, ~2.5 dB too low. */
#define AJ_LEARN_N 64
static double PACC[AJ_N];
static int    nacc;
static double tacc;

void aj_learn_bins(double gain_lin, double dt_s, double rise_db_s)
{
	int k;
	if(!pb_ok || gain_lin <= 0.0) return;
	if(dt_s < 0.0) dt_s = 0.0;
	if(dt_s > 1.0) dt_s = 1.0;
	for(k = 0; k < AJ_N; k++) PACC[k] += (PB[k] - AJ_QFLOOR) / gain_lin;
	nacc++;
	tacc += dt_s;
	pb_ok = 0;                          /* each block is used once */
	if(nacc < AJ_LEARN_N) return;
	{
		/* 10^(rise tacc / 10), tacc <= a few s: 1 + x ln10 + (x ln10)^2/2 */
		double x = 0.2302585 * rise_db_s * tacc, up = 1.0 + x + 0.5 * x * x;
		for(k = 0; k < AJ_N; k++) {
			double cand = PACC[k] / (double)nacc;
			if(cand < 1e-6) cand = 1e-6;
			if(!nfb_ok || cand < NFB[k]) NFB[k] = cand;
			else {
				double lim = NFB[k] * up;
				NFB[k] = (cand < lim) ? cand : lim;
			}
			PACC[k] = 0.0;
		}
	}
	nacc = 0;
	tacc = 0.0;
	nfb_ok = 1;
}

void aj_forget_bins(void)
{
	int k;
	nfb_ok = 0; pb_ok = 0; nacc = 0; tacc = 0.0;
	for(k = 0; k < AJ_N; k++) PACC[k] = 0.0;
}
int  aj_bins_known(void)  { return nfb_ok; }

static void bin_scan(aj_result_t *r)
{
	static double br[AJ_N], bi[AJ_N];
	const int nfr = AJ_NSAMP / AJ_N;
	int f, n, k;
	tables();
	for(k = 0; k < AJ_N; k++) PB[k] = 0.0;
	for(f = 0; f < nfr; f++) {
		int st = f * AJ_N;
		for(n = 0; n < AJ_N; n++) {
			br[n] = WIN[n] * X1R[st + n];
			bi[n] = WIN[n] * X1I[st + n];
		}
		fft(br, bi);
		for(k = 0; k < AJ_N; k++) PB[k] += br[k] * br[k] + bi[k] * bi[k];
	}
	for(k = 0; k < AJ_N; k++) PB[k] /= (double)nfr * AJ_ENORM;
	pb_ok = 1;
	bin_peak(r, PB, 1.0);
}

/* Frame f's windowed FFTs of x1 and g2 x2 into b1, b2. */
static void frame_fft(int f, double g2, double *b1r, double *b1i, double *b2r, double *b2i)
{
	int n, st = f * AJ_SHOP;
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
	int k, i, n, st = f * AJ_SHOP;
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
	/* with frames AJ_N apart only one of the FPGA's two chains is modelled:
	 * each sample is weighted by w^2 (mean 1/2) instead of w^2 + w'^2 = 1 */
	return p * ((double)AJ_SHOP / (double)AJ_HOP) / (double)(AJ_NSAMP - 2 * AJ_N);
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

/* v6.1 hold: last weights solved while bin k held a jammer (per tap 1..3)
 * and the solves since; -1 = nothing held. */
static double hwr[AJ_N][AJ_MAXDIM], hwi[AJ_N][AJ_MAXDIM];
static int    hage[AJ_N] = {-1};
static double hjn[AJ_N];                 /* J/N of the bin when stored [dB] */
static int    hold_init;
static int    hold_n = AJ_HOLD_SOLVES;   /* run-time setting, 0 = off */

void aj_set_hold(int solves) { hold_n = (solves < 0) ? 0 : solves; }
int  aj_get_hold(void)       { return hold_n; }

static void hold_clear(void)
{
	int k;
	for(k = 0; k < AJ_N; k++) hage[k] = -1;
	hold_init = 1;
}

/* Expected noise in bin k [bin power units]: the learned per-bin floor when
 * there is one, else white at nbin. */
static double bin_nb(const aj_result_t *r, int k, double nbin)
{
	if(nfb_ok && r->gain_lin > 0.0) return bin_floor(k, r->gain_lin) * AJ_ENORM;
	return nbin;
}

/* v6.2.1: bin noise when no floor has been learned (the jammer was there
 * since start-up, or since a gnss_band= change). The smallest generalised
 * eigenvalue of the bin's [X1_k, X2n_k] covariance against the noise model
 * D = diag(1, nr2): for ONE jammer plus noise, R = J h h^H + s2 D, so it is
 * s2 itself -- the noise, whatever the jammer's power. Solves
 *   nr2 l^2 - (a nr2 + b) l + (a b - |c|^2) = 0,  a = E|X1|^2, b = E|X2n|^2.
 * Replaces the v6.0 start at min(P1, P2) (the jammer's level), which the
 * loading iteration could accept as converged with a large RX2 noise weight
 * (g2 raised for a shadowed RX2): no null. Returns -1 if not usable. */
static double bin_lmin(double (*fr)[AJ_NV], double (*fi)[AJ_NV], double nr2)
{
	double a = fr[0][0], b = fr[1][1];
	double c2 = fr[0][1] * fr[0][1] + fi[0][1] * fi[0][1];
	double s = a * nr2 + b, det = a * b - c2, disc, l;
	if(nr2 <= 0.0 || a <= 0.0 || b <= 0.0) return -1.0;
	if(det < 0.0) det = 0.0;
	disc = s * s - 4.0 * nr2 * det;
	if(disc < 0.0) disc = 0.0;
	/* the smaller root, in the cancellation-free form 2 det / (s + sqrt) */
	l = 2.0 * det / (s + aj_sqrt(disc));
	return (l > 0.0) ? l : -1.0;
}

double aj_avg_policy(int n_ep, double res_db, double res_prev_db)
{
	if(n_ep < 3) return 0.0;                                     /* AJ_AVG */
	if(res_db > res_prev_db + AJ_AVG_RISE_DB || res_db > AJ_AVG_TRACK_DB)
		return -1.0;                                     /* this capture only */
	return 0.0;                                                  /* AJ_AVG */
}

int aj_g2_follow(const aj_result_t *r, double g2_now, double g2_floor, double *g2_out)
{
	double t, d;
	if(r->jn_db < AJ_G2J_JN_DB || r->p2 <= AJ_QFLOOR || g2_now <= 0.0) return 0;
	t = aj_sqrt(r->p1 / r->p2);                   /* RX1/RX2 jammer amplitude */
	if(t > AJ_G2_MAX) t = AJ_G2_MAX;
	if(r->pk2 > 0 && t > AJ_G2J_PEAK / (double)r->pk2) t = AJ_G2J_PEAK / (double)r->pk2;
	if(t < g2_floor) t = g2_floor;                /* only ever raised          */
	d = aj_db10(t * t / (g2_now * g2_now));
	if(d < AJ_G2J_HYST_DB && d > -AJ_G2J_HYST_DB) return 0;
	*g2_out = (double)((int32_t)(t * 8192.0 + 0.5)) / 8192.0;
	return 1;
}

void aj_reset_stats(void)
{
	wola_n = 0;
	avg_valid = 0;
	s2_valid = 0;
	if(!hold_init) hold_clear();
}

void aj_reset(void)
{
	aj_reset_stats();
	hold_clear();
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
	double nbin, avg, nflat;

	r->ok = 0;
	if(!buf || nsamp != AJ_NSAMP) return -1;
	tables();
	if(!hold_init) hold_clear();
	g2  = (r->g2 > 0.0) ? r->g2 : 1.0;
	nr2 = (r->rx2_noise > 0.0) ? r->rx2_noise : 1.0;
	if(r->sig2_prev <= 0.0) s2_valid = 0;

	unpack(buf, nsamp);
	r->pk2 = pk2_last;
	cnt = lag_stats();
	detect_core(r, cnt);
	/* statistics, and the previous weights' output on this block (OL) */
	build_stats(g2);

	/* statistics for the solve: this block, averaged with the last ones
	 * (v6.1: the caller may set the weight, see aj_result_t.avg) */
	avg = (r->avg > 0.0) ? r->avg : (r->avg < 0.0 ? 0.0 : AJ_AVG);
	if(avg > 0.95) avg = 0.95;
	for(k = 0; k < AJ_N; k++)
		for(i = 0; i < AJ_NV; i++)
			for(j = 0; j < AJ_NV; j++) {
				if(avg_valid && avg > 0.0) {
					AR[k][i][j] = avg * AR[k][i][j] + (1.0 - avg) * FR[k][i][j];
					AI[k][i][j] = avg * AI[k][i][j] + (1.0 - avg) * FI[k][i][j];
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

	lo = AJ_QFLOOR * AJ_ENORM;
	for(k = 0; k < AJ_NW; k++) { r->wre[k] = 0; r->wim[k] = 0; }
	r->wmax = 0.0; r->wmax_bin = 0; r->jam_bins = 0; r->held_bins = 0;
	/* v6.1: the noise floor for THIS block (noise_ref is already scaled to
	 * the present RX gain); v6.0 used the previous solve's figure, one RX
	 * gain step out of date after the AGC moved. */
	nflat = (r->noise_ref > 0.0) ? r->noise_ref : (r->noise > 0.0 ? r->noise : AJ_QFLOOR);
	if(nflat < AJ_QFLOOR) nflat = AJ_QFLOOR;
	nbin = nflat * AJ_ENORM;

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
			s2_bin[k] = s2;
			wola_wr[k][0] = (double)q312(wr[0]) / 4096.0;
			wola_wi[k][0] = (double)q312(wi[0]) / 4096.0;
			r->wre[AJ_WB + k] = q312(wr[0]);
			r->wim[AJ_WB + k] = q312(wi[0]);
		}
		s2sum = s2 * AJ_N; wn2sum = wn2 * AJ_N; itmax = it;
		r->wmax = aj_sqrt(wr[0] * wr[0] + wi[0] * wi[0]);
	} else {
		/* Bin gating: with a known noise floor, a bin whose own J/N is below
		 * AJ_GATE_DB keeps zero weights -- it passes RX1 untouched. Such a
		 * bin holds no jammer (or only a window-leakage tail), so a weight
		 * there would only add RX2 noise, or, when a satellite shares the
		 * jammer's direction, cut the satellite in bins the jammer does not
		 * really occupy. Narrows the notch around a CW tone to the bins the
		 * tone actually fills. */
		const int gate = (r->noise_ref > 0.0);
		for(k = 0; k < AJ_N; k++) {
			double p = AR[k][0][0], nb = bin_nb(r, k, nbin);
			r->bin_jn_db[k] = (p > nb) ? aj_db10((p - nb) / nb) : -99.0;
		}
		for(k = 0; k < AJ_N; k++) {
			/* v6.2: with no previous loading, start at the bin's known noise
			 * floor. Started at min(P1, P2) -- the jammer's level -- the
			 * fixed-point iteration moves only slowly when an RX2 tap has a
			 * large noise weight (g2 raised for a shadowed RX2), and its
			 * stopping test accepted that start: loading at the jammer's
			 * level, |b| ~ 0.1, no null (a latent v6.0 fault).
			 * v6.2.1: with NO learned floor (jammed since start-up or since
			 * a band change) the same fault is avoided by starting at the
			 * bin's noise estimated from its own statistics (bin_lmin). */
			double s2 = s2_valid ? s2_bin[k] : (gate ? bin_nb(r, k, nbin) : bin_lmin(AR[k], AI[k], nr2)), wn2, m, hwn2;
			double hr[AJ_MAXDIM], hi_[AJ_MAXDIM];
			int it, gated, use_held;
			if(solve_one(AR[k], AI[k], n, idx, nr, s2, lo, wr, wi, &s2, &wn2, &it) != 0)
				return -2;
			gated = gate && (r->bin_jn_db[k] < AJ_GATE_DB);
			/* v6.1 weight hold. A jammer from one direction needs the same
			 * b_k (and image taps) whenever it is in bin k. Weights solved
			 * while a strong jammer filled the bin are kept and run when
			 * the bin's statistics have since faded (the jammer pulsed off,
			 * hopped or swept on), so its return is nulled at once and as
			 * deeply as before. Checked on THIS block: held weights that
			 * leave more than AJ_HOLD_RES x the noise (another jammer, a
			 * new direction) are not used. Only b_k is held. */
			use_held = 0;
			if(gate && hold_n > 0 && hage[k] >= 0 && hage[k] < hold_n) {
				double eff = hjn[k] - AJ_HOLD_DECAY_DB * (double)hage[k];
				if(gated || eff >= r->bin_jn_db[k] + AJ_HOLD_KEEP_DB) {
					double po, nb = bin_nb(r, k, nbin);
					hwn2 = 0.0;
					/* b_k only: the image taps act on bin -k, which may
					 * hold another (hopping) jammer by now; they correct
					 * a ~-48 dB I/Q image and come back with the next
					 * fresh solve */
					for(i = 0; i < n; i++) {
						hr[i] = (idx[i] == 1) ? hwr[k][0] : 0.0;
						hi_[i] = (idx[i] == 1) ? hwi[k][0] : 0.0;
						hwn2 += (hr[i] * hr[i] + hi_[i] * hi_[i]) * nr[i];
					}
					po = out_power(AR[k], AI[k], n, idx, hr, hi_);
					if(po <= AJ_HOLD_RES * nb * (1.0 + hwn2)) use_held = 1;
				}
			}
			if(use_held) {
				for(i = 0; i < n; i++) { wr[i] = hr[i]; wi[i] = hi_[i]; }
				wn2 = hwn2;
				hage[k]++;
				r->held_bins++;
			} else if(gated) {
				for(i = 0; i < n; i++) { wr[i] = 0.0; wi[i] = 0.0; }
				wn2 = 0.0;
				if(hage[k] >= 0 && ++hage[k] >= hold_n) hage[k] = -1;
			} else {
				/* fresh weights; stored when the jammer is strong and at
				 * least as strong as what is held (so the fading tail of a
				 * jammer that has moved on never replaces the deep ones) */
				if(r->bin_jn_db[k] >= AJ_HOLD_DB &&
				   (hage[k] < 0 || hage[k] >= hold_n ||
				    r->bin_jn_db[k] >= hjn[k] - AJ_HOLD_DECAY_DB * (double)hage[k] - AJ_HOLD_STORE_DB)) {
					for(i = 0; i < AJ_MAXDIM; i++) { hwr[k][i] = 0.0; hwi[k][i] = 0.0; }
					for(i = 0; i < n; i++) { hwr[k][idx[i] - 1] = wr[i]; hwi[k][idx[i] - 1] = wi[i]; }
					hjn[k] = r->bin_jn_db[k];
					hage[k] = 0;
				} else if(hage[k] >= 0) {
					hage[k]++;
				}
			}
			s2_bin[k] = s2;
			s2sum += s2;
			wn2sum += wn2;
			if(it > itmax) itmax = it;
			for(i = 0; i < n; i++) {
				int16_t qr = q312(wr[i]), qi = q312(wi[i]);
				int word = (idx[i] == 1) ? AJ_WB + k : (idx[i] == 2) ? AJ_WC1 + k : AJ_WC2 + k;
				r->wre[word] = qr;
				r->wim[word] = qi;
				wola_wr[k][i] = (double)qr / 4096.0;   /* what the FPGA will run */
				wola_wi[k][i] = (double)qi / 4096.0;
			}
			m = aj_sqrt(wr[0] * wr[0] + wi[0] * wi[0]);
			if(m > r->wmax) { r->wmax = m; r->wmax_bin = k; }
		}
	}
	s2_valid = 1;

	/* J/N per bin (vs. the per-bin floor when learned, else white) */
	for(k = 0; k < AJ_N; k++) {
		double p = AR[k][0][0], nb = bin_nb(r, k, nbin);
		r->bin_jn_db[k] = (p > nb) ? aj_db10((p - nb) / nb) : -99.0;
		if(r->bin_jn_db[k] > 3.0) r->jam_bins++;
	}
	/* narrowband detector figure on THIS block (not the average), for the
	 * engine's jammer-gone decision */
	{
		static double pf[AJ_N];
		for(k = 0; k < AJ_N; k++) pf[k] = FR[k][0][0] / AJ_ENORM;
		bin_peak(r, pf, 1.0);
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
	spectrum(r, cnt, r->p1, r->noise);
	r->ok = 1;
	return 0;
}

/* ---- antsdr-fw-patch crpa15 (design A, Linux): helpers of the engine ----------------------- */
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
	int s_restore, s_safe, sh;
	if(cut_db <= 0) return 0;
	if(max_shift > AJ_MAXSH) max_shift = AJ_MAXSH;
	s_restore = cut_db / 6;
	s_safe = (int)((double)cut_db - (out_db > 0.0 ? out_db : 0.0) + AJ_OUT_HEADROOM_DB);
	s_safe = (s_safe < 0) ? -1 : s_safe / 6;
	sh = (s_restore < s_safe) ? s_restore : s_safe;
	if(sh > max_shift) sh = max_shift;
	if(sh < 0) sh = 0;
	return sh;
}
