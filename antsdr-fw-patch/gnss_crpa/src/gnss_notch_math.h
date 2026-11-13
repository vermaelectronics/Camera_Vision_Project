/******************************************************************************
 * gnss_notch_math.h -- CW notch: tone detector and software model of the FPGA
 * notch (antsdr-fw-patch crpa5). Pure C, no board access: also compiled into
 * the smi_pi C simulation (hls/smi_pi/testbench.cpp).
 *
 * The FPGA notch (smi_pi 2.6, see hls/smi_pi/smi_pi.h, port ntc) removes one
 * tone from RX1 and from the normalised RX2 before the space-time filter:
 *     d[n]  = x[n] exp(-j theta[n]),  m[n] = mean of the last L values of d,
 *     x'[n] = x[n] - m[n] exp(+j theta[n]),  L = 2^NT_LOG2L,
 * theta = a 32-bit NCO phase advanced by the phase increment every sample.
 *****************************************************************************/
#ifndef GNSS_NOTCH_MATH_H_
#define GNSS_NOTCH_MATH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NT_LOG2L      7       /* must match SMI_NOTCH_LOG2L of the FPGA core   */
#define NT_L          (1 << NT_LOG2L)
#define NT_FFT        1024    /* detector FFT length                           */
#define NT_DET_DB     25.0    /* tone peak above the median bin [dB]           */
#define NT_SHOULDER_DB 20.0   /* and this far above the bins 5..12 away        */

typedef struct {
    int    found;      /* 1 = a CW tone (narrow, strong) is present             */
    double f;          /* its frequency, fraction of fs (-0.5 .. 0.5)           */
    double peak_db;    /* peak bin over the median bin [dB]                     */
    double shoulder_db;/* peak bin over the highest bin 5..12 bins away [dB]    */
    double tone_db;    /* tone power over the receiver noise floor [dB] (if a  */
                       /*   noise floor was given, else -200)                   */
} nt_tone_t;

/* Look for one CW tone in RX1 of a capture (buf in the aj_compute() format:
 * word 2n = RX1 {Q[27:16], I[11:0]}, word 2n+1 = RX2). noise = receiver noise
 * floor [LSB^2 per sample] or <= 0 if unknown. Returns t->found. */
int nt_find_tone(const uint32_t *buf, int nsamp, double noise, nt_tone_t *t);

/* NCO phase increment for a tone at f (fraction of fs). */
uint32_t nt_pinc(double f);
/* ... and back. */
double nt_freq(uint32_t pinc);

/* Software model of the FPGA notch on one capture: out = in with the tone at
 * pinc removed from RX1 and RX2 (12-bit, saturated). The first L samples use a
 * shorter average (no history before the capture). */
void nt_apply(const uint32_t *in, uint32_t *out, int nsamp, uint32_t pinc);

#ifdef __cplusplus
}
#endif

#endif
