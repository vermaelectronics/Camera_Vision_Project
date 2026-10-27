/* Unit test of the firmware CW scan (npi_cw_scan in gnss_npi_det.c).
 * A CW at L1 + 1.2 kHz (LO 1570.25 MHz, 50 MS/s) in white noise of 20 LSB
 * per component, RX2 = 0.78 * RX1 rotated by 0.6 rad.  J/N is over the whole
 * 50 MHz.  Six captures per level; the last four are counted.
 *   pass: no detection with no CW and at J/N -25 dB (no false CW),
 *         every scan detected at J/N >= -17 dB, frequency error < 3 kHz
 *         (the 4096-point bin is 12.2 kHz). */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "gnss_npi_det.h"

static double gr(void) {
    double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}
static uint32_t buf[2 * NPI_NSAMP];
static int c12(double v) { long r = lround(v); return r > 2047 ? 2047 : r < -2048 ? -2048 : (int)r; }

int main(void) {
    const double fs = 50e6, f = 5.17e6 + 1234.0;
    const double jn[] = {-99, -25, -20, -17, -13, -10, 0, 10};
    int fail = 0;
    srand(1);
    for (int a = 0; a < 8; a++) {
        npi_cw_reset();
        int det = 0; double ferr = 0, emax = 0, snr = 0; long n0 = 0;
        for (int scan = 0; scan < 6; scan++) {
            double amp = (jn[a] > -90) ? sqrt(2 * 400.0 * pow(10, jn[a] / 10)) : 0;
            for (int t = 0; t < NPI_NSAMP; t++) {
                long n = n0 + t; double ph = 2 * M_PI * f * n / fs;
                int x1r = c12(amp * cos(ph) + 20 * gr()), x1i = c12(amp * sin(ph) + 20 * gr());
                int x2r = c12(0.78 * amp * cos(ph + 0.6) + 20 * gr()), x2i = c12(0.78 * amp * sin(ph + 0.6) + 20 * gr());
                buf[2 * t] = (uint16_t)x1r | ((uint32_t)(uint16_t)x1i << 16);
                buf[2 * t + 1] = (uint16_t)x2r | ((uint32_t)(uint16_t)x2i << 16);
            }
            n0 += NPI_NSAMP * 3;
            npi_det_t r = {0}; npi_detect(buf, NPI_NSAMP, &r);
            npi_cw_t o; npi_cw_scan(12, &o);
            if (scan >= 2 && o.n > 0) {
                det++; ferr = o.f[0] * fs - f; snr = o.snr1[0];
                if (fabs(ferr) > emax) emax = fabs(ferr);
            }
        }
        int ok = 1;
        if (jn[a] <= -25 && det != 0) ok = 0;
        if (jn[a] >= -17 && (det != 4 || emax > 3000)) ok = 0;
        printf("CW J/N_total %6.1f dB: detected in %d of 4 scans, freq err %8.1f Hz, bin SNR %5.1f dB  %s\n",
               jn[a], det, ferr, snr, ok ? "ok" : "FAIL");
        fail |= !ok;
    }
    printf("CW_SCAN: %s\n", fail ? "FAIL" : "PASS");
    return fail;
}
