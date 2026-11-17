// ============================================================================
//  smi_pi.h  --  space-time SMI power inversion core (SMI-PI v4.4; 4.9.14: 6 + 6 taps)
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
#define SMI_NT    6    /* 4.9.14 6x6 variant: 6 taps per antenna (6 + 6 = 12 taps); 4.9.14 also exists as 7x7, 6x6 and 5x5 */
#ifndef SMI_REF
#define SMI_REF    3    /* 4.9.14 6x6: reference tap at delay 3: 3 pre-cursor taps, reach NT-1-REF = 2 samples */
#endif
// DSP48 budget of this core (4.9.14, 6 + 6): 5 RX1 taps + 6 RX2 taps + 2 image taps = 13 complex
// multipliers x 3 (Gauss form) = 39, plus 2 for the real RX2 gain g2 = 41 DSP48 (7 + 7: 47,
// 6 + 6: 41, 5 + 5: 35; 4.9.11 / 4.9.12 had 4 for a complex g2).
// RX1 taps (the highest delays) whose multipliers are built from LUTs, not DSP48 (4.9.1: the 32-tap build needed 229 DSP48 of 220).
// 12 taps = 36 multipliers = about 36 DSP48 saved, about 8000 LUT spent. 0 = all DSP48.
#ifndef SMI_NFAB
#define SMI_NFAB  0    /* 4.9.9: no LUT multipliers (they cost LUT and power) */
#endif
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

// s_axis_x : {rx2_q, rx2_i, rx1_q, rx1_i}, each a 16-bit word holding a
//            12-bit RX sample right-aligned ([11:0] is used).
// m_axis_y : {y_q, y_i}, signed 16-bit:
//            filter:  y[n] = 2^s (x1[n-R] - sum_{k!=R} a_k x1[n-k]   (R = SMI_REF)
//                                        - sum_k b_k x2n[n-k]
//                                        - c1 conj(x1[n-R]) - c2 conj(x2n[n-R]))
//            bypass:  y[n] = 2^s x1[n-R]
//            x2n = g2 * x2 (RX2 normalised to RX1, kept with 2 extra bits)
// wts      : weight words, word k at bits [32k+31 : 32k] (port sized for 64; 4.9.13 uses 14):
//              k = 0..SMI_NT-1          -> a_k (RX1, delay k); word SMI_REF is ignored
//              k = SMI_NT..2*SMI_NT-1   -> b_{k-SMI_NT} (RX2 normalised, delay k-SMI_NT)
//            each {imag[31:16], real[15:0]}, signed Q3.12.
// wcj      : image taps, [31:0] c1 (RX1), [63:32] c2 (RX2 normalised), same
//            word format as wts.
// cfg      : [0] bypass, [10:8] output make-up shift s (0..7),
//            [14:12] weight scale ws (0..5; 6, 7 act as 5).
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
// wts, wcj, cfg and nrm come from gnss_passthrough's committed registers, in the
// same clock domain as this core, so a commit changes all three at once.
void smi_pi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<2048> wts,
            ap_uint<64>   wcj,
            ap_uint<32>   cfg,
            ap_uint<32>   nrm);

#endif
