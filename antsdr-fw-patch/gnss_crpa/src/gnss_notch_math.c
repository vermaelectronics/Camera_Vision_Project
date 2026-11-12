/******************************************************************************
 * gnss_notch_math.c -- CW tone detector and software model of the FPGA notch
 * (antsdr-fw-patch crpa5). See gnss_notch_math.h.
 *
 * Detector: 8 Hann-windowed 1024-point FFTs of RX1 (Welch average). A CW tone
 * is a bin at least NT_DET_DB above the median bin and NT_SHOULDER_DB above
 * every bin 5..12 bins away (a wideband or partial-band jammer raises the
 * median and the shoulders together, so it is never taken for a tone).
 * Frequency: parabolic interpolation on the peak (about 0.05 bin), then the
 * phase advance of the tone between the two halves of the 8192-sample capture
 * (a few Hz at 30.72 MS/s for a tone 20 dB above the noise).
 * No libm (bare-metal build): cos/sin come from the notch's own NCO table.
 *****************************************************************************/
#include "gnss_notch_math.h"
#include "gnss_aj_math.h"     /* aj_db10, aj_atan2 */
#include "smi_pi_nco.h"

#define NT_PI 3.14159265358979323846

static double nco_c(uint32_t ph) { return (double)SMI_NCO_COS[ph >> (32 - SMI_NCO_BITS)] / 32767.0; }
static double nco_s(uint32_t ph) { return (double)SMI_NCO_SIN[ph >> (32 - SMI_NCO_BITS)] / 32767.0; }

static int ext12(uint32_t v) { return ((int32_t)(v << 20)) >> 20; }

uint32_t nt_pinc(double f)
{
    double v;
    while (f >= 0.5) f -= 1.0;
    while (f < -0.5) f += 1.0;
    v = f * 4294967296.0;
    v = (v >= 0.0) ? v + 0.5 : v - 0.5;
    return (uint32_t)(int64_t)v;
}

double nt_freq(uint32_t pinc)
{
    return (double)(int32_t)pinc / 4294967296.0;
}

/* ---- 1024-point radix-2 FFT (in place) ---------------------------------------- */
static void fft1024(double *re, double *im)
{
    int i, j, k, len;
    for (i = 1, j = 0; i < NT_FFT; i++) {
        int bit = NT_FFT >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            double t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (len = 2; len <= NT_FFT; len <<= 1) {
        int step = SMI_NCO_N / len;                  /* table index step for 2 pi / len */
        for (i = 0; i < NT_FFT; i += len) {
            for (k = 0; k < len / 2; k++) {
                int ti = k * step;
                double wr = (double)SMI_NCO_COS[ti] / 32767.0;
                double wi = -(double)SMI_NCO_SIN[ti] / 32767.0;   /* exp(-j 2 pi k / len) */
                double ur = re[i + k], ui = im[i + k];
                double xr = re[i + k + len / 2], xi = im[i + k + len / 2];
                double vr = xr * wr - xi * wi, vi = xr * wi + xi * wr;
                re[i + k] = ur + vr;           im[i + k] = ui + vi;
                re[i + k + len / 2] = ur - vr; im[i + k + len / 2] = ui - vi;
            }
        }
    }
}

/* k-th smallest of n values (quickselect, destroys the order of a). */
static double kth(double *a, int n, int k)
{
    int lo = 0, hi = n - 1;
    while (lo < hi) {
        double p = a[(lo + hi) / 2];
        int i = lo, j = hi;
        while (i <= j) {
            while (a[i] < p) i++;
            while (a[j] > p) j--;
            if (i <= j) { double t = a[i]; a[i] = a[j]; a[j] = t; i++; j--; }
        }
        if (k <= j) hi = j;
        else if (k >= i) lo = i;
        else break;
    }
    return a[k];
}

static int bin_ok(int k) { return k > 2 && k < NT_FFT - 2; }   /* not DC (LO leakage, DC offset) */

int nt_find_tone(const uint32_t *buf, int nsamp, double noise, nt_tone_t *t)
{
    static double re[NT_FFT], im[NT_FFT], p[NT_FFT], tmp[NT_FFT];
    double mr = 0.0, mi = 0.0, med, pk, sh = 0.0, wsum = 0.0;
    int nseg = nsamp / NT_FFT, s, n, k, kmax = -1, cnt = 0, d;

    t->found = 0; t->f = 0.0; t->peak_db = 0.0; t->shoulder_db = 0.0; t->tone_db = -200.0;
    if (nseg < 1) return 0;
    if (nseg > 8) nseg = 8;
    for (n = 0; n < nseg * NT_FFT; n++) { mr += ext12(buf[2 * n]); mi += ext12(buf[2 * n] >> 16); }
    mr /= nseg * NT_FFT; mi /= nseg * NT_FFT;
    for (k = 0; k < NT_FFT; k++) p[k] = 0.0;
    for (k = 0; k < NT_FFT; k++) {
        double w = 0.5 - 0.5 * (double)SMI_NCO_COS[k * (SMI_NCO_N / NT_FFT)] / 32767.0;
        wsum += w * w;
    }
    for (s = 0; s < nseg; s++) {
        for (k = 0; k < NT_FFT; k++) {
            uint32_t v = buf[2 * (s * NT_FFT + k)];
            double w = 0.5 - 0.5 * (double)SMI_NCO_COS[k * (SMI_NCO_N / NT_FFT)] / 32767.0;
            re[k] = ((double)ext12(v) - mr) * w;
            im[k] = ((double)ext12(v >> 16) - mi) * w;
        }
        fft1024(re, im);
        for (k = 0; k < NT_FFT; k++) p[k] += re[k] * re[k] + im[k] * im[k];
    }
    for (k = 0; k < NT_FFT; k++) {
        p[k] /= (double)nseg * wsum;                 /* per-sample power in the bin */
        if (bin_ok(k)) {
            tmp[cnt++] = p[k];
            if (kmax < 0 || p[k] > p[kmax]) kmax = k;
        }
    }
    if (kmax < 0) return 0;
    med = kth(tmp, cnt, cnt / 2);
    pk = p[kmax];
    for (d = 5; d <= 12; d++) {
        int a = (kmax + d) & (NT_FFT - 1), b = (kmax - d) & (NT_FFT - 1);
        if (bin_ok(a) && p[a] > sh) sh = p[a];
        if (bin_ok(b) && p[b] > sh) sh = p[b];
    }
    t->peak_db = aj_db10(pk / (med > 1e-12 ? med : 1e-12));
    t->shoulder_db = aj_db10(pk / (sh > 1e-12 ? sh : 1e-12));
    if (t->peak_db < NT_DET_DB || t->shoulder_db < NT_SHOULDER_DB) return 0;

    {   /* coarse frequency: parabola through the log powers around the peak */
        double a = aj_db10(p[(kmax - 1) & (NT_FFT - 1)] + 1e-12), b = aj_db10(pk + 1e-12),
               c = aj_db10(p[(kmax + 1) & (NT_FFT - 1)] + 1e-12), den = a - 2.0 * b + c, dlt = 0.0;
        double f0, tone = 0.0;
        int ks = (kmax < NT_FFT / 2) ? kmax : kmax - NT_FFT;
        uint32_t pinc, ph = 0;
        double zar = 0.0, zai = 0.0, zbr = 0.0, zbi = 0.0, xr, xi, cr, ci, dphi;
        int half = nsamp / 2;
        if (den < -1e-9) dlt = 0.5 * (a - c) / den;
        if (dlt > 0.5) dlt = 0.5;
        if (dlt < -0.5) dlt = -0.5;
        f0 = ((double)ks + dlt) / NT_FFT;
        /* fine frequency: phase advance of the tone between the two halves */
        pinc = nt_pinc(f0);
        f0 = nt_freq(pinc);
        for (n = 0; n < nsamp; n++) {
            double c0 = nco_c(ph), s0 = nco_s(ph);
            xr = ext12(buf[2 * n]) - mr; xi = ext12(buf[2 * n] >> 16) - mi;
            cr = xr * c0 + xi * s0;          /* x exp(-j theta) */
            ci = xi * c0 - xr * s0;
            if (n < half) { zar += cr; zai += ci; } else { zbr += cr; zbi += ci; }
            ph += pinc;
        }
        dphi = aj_atan2(zbi * zar - zbr * zai, zbr * zar + zbi * zai);
        t->f = f0 + dphi / (2.0 * NT_PI * (double)half);
        while (t->f >= 0.5) t->f -= 1.0;
        while (t->f < -0.5) t->f += 1.0;
        tone = ((zar + zbr) * (zar + zbr) + (zai + zbi) * (zai + zbi)) / ((double)nsamp * (double)nsamp);
        if (noise > 0.0) t->tone_db = aj_db10(tone / noise);
    }
    t->found = 1;
    return 1;
}

void nt_apply(const uint32_t *in, uint32_t *out, int nsamp, uint32_t pinc)
{
    static double dr[2][NT_L], di[2][NT_L];
    double sr[2] = {0.0, 0.0}, si[2] = {0.0, 0.0};
    uint32_t ph = 0;
    int n, ch;
    for (n = 0; n < nsamp; n++) {
        double c0 = nco_c(ph), s0 = nco_s(ph);
        int slot = n & (NT_L - 1), cntw = (n < NT_L) ? n + 1 : NT_L;
        uint32_t w[2];
        for (ch = 0; ch < 2; ch++) {
            uint32_t v = in[2 * n + ch];
            double xr = ext12(v), xi = ext12(v >> 16), ar, ai, mr2, mi2, yr, yi;
            int qr, qi;
            ar = xr * c0 + xi * s0;               /* mixed down */
            ai = xi * c0 - xr * s0;
            if (n >= NT_L) { sr[ch] -= dr[ch][slot]; si[ch] -= di[ch][slot]; }
            dr[ch][slot] = ar; di[ch][slot] = ai;
            sr[ch] += ar; si[ch] += ai;
            mr2 = sr[ch] / cntw; mi2 = si[ch] / cntw;
            yr = xr - (mr2 * c0 - mi2 * s0);      /* minus the mean, mixed back up */
            yi = xi - (mi2 * c0 + mr2 * s0);
            qr = (int)(yr >= 0.0 ? yr + 0.5 : yr - 0.5);
            qi = (int)(yi >= 0.0 ? yi + 0.5 : yi - 0.5);
            if (qr > 2047) qr = 2047;
            if (qr < -2048) qr = -2048;
            if (qi > 2047) qi = 2047;
            if (qi < -2048) qi = -2048;
            w[ch] = ((uint32_t)(uint16_t)(int16_t)qr) | ((uint32_t)(uint16_t)(int16_t)qi << 16);
        }
        out[2 * n] = w[0];
        out[2 * n + 1] = w[1];
        ph += pinc;
    }
}
