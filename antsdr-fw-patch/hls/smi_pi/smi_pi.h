// ============================================================================
//  smi_pi.h  --  space-time SMI power inversion core (SMI-PI v4.4; antsdr-fw-patch crpa10: 8 + 8 taps, 3 + 3 image taps, weight smoothing, hardware NLMS)
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2
//
//  See smi_pi.cpp for the filter, the fixed-point formats and why the
//  weights are computed on the ARM instead of in this core.
// ============================================================================
#ifndef SMI_PI_H
#define SMI_PI_H

// the 64 weight words are one 2048-bit port (the default limit of ap_int is 1024)
#ifndef AP_INT_MAX_W
#define AP_INT_MAX_W 4096
#endif
#include "ap_int.h"
#include "hls_stream.h"

// Taps per channel (delays 0..31, 4.9: was 16) and the RX1 reference delay. Must match
// AJ_NT / AJ_REF in Source/Firmware/app_gnss_e310/gnss_aj_math.h.
#ifndef SMI_NT
#define SMI_NT    8    /* antsdr-fw-patch crpa10: 8 taps per antenna (8 + 8 = 16 taps); crpa7-9: 10, crpa4/5: 8, crpa3: 7, crpa1/2: 6 */
#endif
#ifndef SMI_REF
#define SMI_REF    4    /* crpa10 (8 + 8): reference delay 4, as crpa4 (4 before, 3 after). crpa7-9 (10 + 10): reference tap at delay 5: 5 pre-cursor taps, 4 after. In C simulation delay 5 is never below 8 + 8 (delay 4); delay 4 lost 2.5 dB on the reflection case (34.9 against 37.4 dB). crpa4/5 (8 + 8): 4 */
#endif
// DSP48 budget of this core (crpa10, 8 + 8): 7 RX1 taps + 8 RX2 taps + 6 image taps = 21 complex
// multipliers x 3 (Gauss form) = 63, plus 2 for the real RX2 gain g2, plus 2 for the fine output gain
// = 67; NLMS: 21 complex updates x 3 + 2 for the step = 65; in all 132.
// (crpa9, 10 + 10 without NLMS: 79; crpa4 8 + 8: 53, 7 + 7: 47, 6 + 6: 41, 5 + 5: 35.)
// RX1 taps (the highest delays) whose multipliers are built from LUTs, not DSP48 (4.9.1: the 32-tap build needed 229 DSP48 of 220).
// 12 taps = 36 multipliers = about 36 DSP48 saved, about 8000 LUT spent. 0 = all DSP48.
#ifndef SMI_NFAB
#define SMI_NFAB  0    /* 4.9.9: no LUT multipliers (they cost LUT and power) */
#endif
// crpa9: image (conjugate) taps per channel, at delays REF-1, REF, REF+1 (crpa8 and older: 1).
// Must match AJ_NIMG of the firmware. 4 more complex multipliers than crpa8: 12 DSP48 (79 in all).
#ifndef SMI_NIMG
#define SMI_NIMG   3
#endif
#define SMI_IMGH   ((SMI_NIMG - 1) / 2)
#define SMI_WFRAC 12          // weights are signed Q3.12 (1.0 = 4096)
#define SMI_GFRAC 13          // RX2 normalisation gain, signed Q3.13 (1.0 = 8192)
#define SMI_XFRAC  2          // fractional bits kept on the normalised RX2
#define SMI_MAXSH  7          // largest output make-up shift (x128 = +42 dB; 4.1 to 4.4: 4)

// cfg bits
#define SMI_CFG_BYPASS   0x1u  // 1 = output x1[n-SMI_REF]: no weights at all
#define SMI_CFG_SH_LSB   8     // [10:8] output make-up shift, 0..7 (6 dB each; 4.5: up to 7)
#define SMI_CFG_WS_LSB   12    // [14:12] weight scale ws, 0..5: the weight words hold
                               //   w * 2^ws, the core divides the sum by 2^ws (SMI-PI 4.4)
#define SMI_MAXWS        5
#define SMI_CFG_RAMP_LSB 16    // crpa8: [19:16] smoothing k, 0 = off (instant, as crpa7); 1..15: weights and
                               //   gain move 1/256 of the way to the committed value every 2^(k-1)
                               //   samples, time constant 2^(k+7) samples (k = 10: 131072 = 4.3 ms at 30.72 MS/s)
#define SMI_RAMP_RS      8     // 1/256 of the remaining step per tick
#define SMI_RAMP_RF     16     // fractional bits of the running values (crpa10: 16, for the NLMS steps; crpa8/9: 9)
#define SMI_GAIN_FRAC   14     // crpa8: fine output gain, unsigned Q2.14 (1.0 = 16384), after the make-up shift
#define SMI_GAIN_ONE    16384

// crpa10: hardware NLMS (delayed LMS) on top of the ARM's SMI weights.
// Every sample each adaptive weight moves by e[n-D] conj(u[n-D]) >> SMI_NLMS_CS, where
// e = (y m) >> sh is the filter output (before the make-up gain) times the normalised step
// m 2^-sh set by the ARM, and u the tap's input; the weights are still pulled to the ARM's
// committed set by the smoothing (cfg[19:16] must be > 0), so NLMS refines the SMI solution
// between two ARM solves. The delay D lets the loop run at one sample per clock.
// nlms port: [15:0] m (unsigned), [20:16] sh (0..31), [31] enable (only with the filter in).
#ifndef SMI_NLMS
#define SMI_NLMS         1     // 0: core without the NLMS hardware (SMI_NLMS=0 ./build.sh hls)
#endif
#define SMI_NLMS_D      24     // delay of the error / inputs used for the update [samples]
#define SMI_NLMS_CS      8     // fixed shift of e conj(u)
#define SMI_NLMS_EW     24     // bits of e (saturated)

// s_axis_x : {rx2_q, rx2_i, rx1_q, rx1_i}, each a 16-bit word holding a
//            12-bit RX sample right-aligned ([11:0] is used).
// m_axis_y : {y_q, y_i}, signed 16-bit:
//            filter:  y[n] = 2^s (x1[n-R] - sum_{k!=R} a_k x1[n-k]   (R = SMI_REF)
//                                        - sum_k b_k x2n[n-k]
//                                        - sum_d c1_d conj(x1[n-R-d]) - sum_d c2_d conj(x2n[n-R-d]))   (d = -1, 0, 1)
//            bypass:  y[n] = 2^s x1[n-R]
//            x2n = g2 * x2 (RX2 normalised to RX1, kept with 2 extra bits)
// wts      : weight words, word k at bits [32k+31 : 32k] (port sized for 64; 4.9.13 uses 14):
//              k = 0..SMI_NT-1          -> a_k (RX1, delay k); word SMI_REF is ignored
//              k = SMI_NT..2*SMI_NT-1   -> b_{k-SMI_NT} (RX2 normalised, delay k-SMI_NT)
//            each {imag[31:16], real[15:0]}, signed Q3.12.
// wcj      : image taps (crpa9: 2 x SMI_NIMG words), word c at [32c+31 : 32c]:
//              c = 0..SMI_NIMG-1          -> c1 on conj(x1[n-(R-H+c)])   (H = SMI_IMGH)
//              c = SMI_NIMG..2*SMI_NIMG-1 -> c2 on conj(x2n[n-(R-H+c-SMI_NIMG)])
//            same word format as wts (crpa8 and older: 2 words, c1 and c2 at R).
// cfg      : [0] bypass, [10:8] output make-up shift s (0..7),
//            [14:12] weight scale ws (0..5; 6, 7 act as 5),
//            [19:16] crpa8 smoothing k (0 = off; see SMI_CFG_RAMP_LSB). With k > 0 bypass
//            fades the weights to 0, and a change of ws or s takes effect at once.
//            The weight words are w * 2^ws (still signed Q3.12). A jammer that
//            is 60-70 dB above the noise needs the weights to 1e-4 or better;
//            Q3.12 alone (2.4e-4) leaves a residual above the noise, and the
//            small weights of a minimum-norm null are the worst case. The
//            firmware picks the largest ws that keeps every weight inside
//            Q3.12, the core takes it out again after the sum (no range or
//            precision lost: the sum is 14 fractional bits wide).
// nrm      : g2 = real[15:0], signed Q3.13 (1.0 = 0x2000). 4.9.13: g2 is real; nrm[31:16]
//            is ignored (the firmware always wrote 0 there: gnss_pt_write_aj), which
//            saves the two g2i multipliers with the same x2n bit for bit.
// nlms     : crpa10: [15:0] step mantissa m, [20:16] step shift sh, [31] enable (see SMI_NLMS_D).
// gain     : crpa8: [15:0] fine output gain, unsigned Q2.14 (1.0 = 0x4000), applied after
//            the make-up shift and smoothed like the weights. Exactly 1.0 gives the crpa7
//            output bit for bit.
// wts, wcj, cfg, nrm and gain come from gnss_passthrough's committed registers, in the
// same clock domain as this core, so a commit changes all of them at once.
void smi_pi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<2048> wts,
            ap_uint<32 * 2 * SMI_NIMG> wcj,
            ap_uint<32>   cfg,
            ap_uint<32>   nrm,
            ap_uint<32>   gain,
            ap_uint<32>   nlms);

#endif
