/******************************************************************************
 *  gnss_npi_det.h
 *  PL-NPI v2.3: jammer detection and measurement on the ARM.
 *
 *  The PL-NPI weights are adapted in the FPGA (pl_npi_0). The ARM only
 *    - DETECTS: RX1/RX2 powers and their coherence from a 4096-sample DMA
 *      capture of both channels (a jammer reaches both antennas, receiver
 *      noise does not, so the coherence finds jammers weaker than the noise
 *      that the FPGA's power detector cannot see);
 *    - MEASURES: what the FPGA's present weights do to that capture
 *      (output power, suppression, residual), from the weights it reads
 *      back through gnss_passthrough (NPI_SNAP).
 *  Pure C, no hardware access, no libm.
 *****************************************************************************/
#ifndef GNSS_NPI_DET_H_
#define GNSS_NPI_DET_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NPI_NSAMP   4096    /* samples per capture                           */
#define NPI_QFLOOR  0.25    /* lowest noise allowed [LSB^2]: 12-bit quantisation */
#define NPI_NLAG      16    /* coherence searched over lags -15..15            */

typedef struct {
    /* ---- set by the caller -------------------------------------------- */
    double  noise_ref;      /* RX1 noise floor at the present gain [LSB^2],   */
                            /*   learned while no jammer; <= 0: unknown      */
    double  rho_noise_re;   /* lag-1 correlation of the noise alone           */
    double  rho_noise_im;
    double  g2;             /* RX2 normalisation in the FPGA (<= 0 means 1)   */
    double  rx2_noise;      /* normalised RX2 noise / RX1 noise (<= 0: 1)     */

    /* ---- npi_detect() ------------------------------------------------- */
    double  p1, p2;         /* RX1, RX2 power [LSB^2] (raw)                   */
    double  coh;            /* RX1-RX2 coherence, best lag (0..1)             */
    int     coh_lag;
    double  jn_coh_db;      /* J/N implied by the coherence [dB]              */
    double  noise;          /* noise used [LSB^2] (0 if unknown)              */
    double  jn_db;          /* power J/N at RX1 [dB], -99 if no floor         */
    double  rho1_re, rho1_im;
    double  foff, bw;       /* jammer centre / bandwidth, fraction of fs      */

    /* ---- npi_measure() ------------------------------------------------ */
    double  w[6];           /* weights used: w2 re/im, c1 re/im, c2 re/im     */
    double  pout;           /* output power of those weights on the capture   */
    double  wn2;            /* noise-weighted |w|^2 of the adaptive weights   */
    double  nout;           /* output noise those weights give [LSB^2]        */
    double  supp_db;        /* jammer suppression [dB], capped at 60          */
    double  res_db;         /* output / output noise [dB], 0 = fully nulled   */
    int     ok;
} npi_det_t;

/* buf: one DMA capture, word 2n = RX1 {Q[31:16], I[15:0]}, word 2n+1 = RX2,
 * 12-bit samples right-aligned. nsamp must be NPI_NSAMP. Return 0 on success. */
int npi_detect(const uint32_t *buf, int nsamp, npi_det_t *r);

/* Output of the FPGA filter with weights w (Q12.20 words as read back, in
 * the order w2 re, w2 im, c1 re, c1 im, c2 re, c2 im) on the same capture:
 *   y = x1 + w2 g2 x2 + c1 conj(x1) + c2 conj(g2 x2).
 * Call after npi_detect() on the same buffer (uses p1 and noise). */
int npi_measure(const uint32_t *buf, int nsamp, const int32_t *w, npi_det_t *r);

double npi_sqrt(double x);
double npi_db10(double x);              /* 10*log10(x), -200 for x <= 0   */

#ifdef __cplusplus
}
#endif

#endif
