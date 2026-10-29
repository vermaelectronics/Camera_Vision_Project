// ============================================================================
//  smi_pi.h  --  space-time SMI power inversion core (SMI-PI v4.4)
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2
//
//  See smi_pi.cpp for the filter, the fixed-point formats and why the
//  weights are computed on the ARM instead of in this core.
// ============================================================================
#ifndef SMI_PI_H
#define SMI_PI_H

#include "ap_int.h"
#include "hls_stream.h"

// Taps per channel (delays 0..15) and the RX1 reference delay. Must match
// AJ_NT / AJ_REF in Source/Firmware/app_gnss_e310/gnss_aj_math.h.
#define SMI_NT    16
#define SMI_REF    8
#define SMI_WFRAC 12          // weights are signed Q3.12 (1.0 = 4096)
#define SMI_GFRAC 13          // RX2 normalisation gain, signed Q3.13 (1.0 = 8192)
#define SMI_XFRAC  2          // fractional bits kept on the normalised RX2
#define SMI_MAXSH  4          // largest output make-up shift (x16 = +24 dB)

// cfg bits
#define SMI_CFG_BYPASS   0x1u  // 1 = output x1[n-8]: no weights at all
#define SMI_CFG_SH_LSB   8     // [10:8] output make-up shift, 0..4 (6 dB each)
#define SMI_CFG_WS_LSB   12    // [14:12] weight scale ws, 0..5: the weight words hold
                               //   w * 2^ws, the core divides the sum by 2^ws (SMI-PI 4.4)
#define SMI_MAXWS        5

// s_axis_x : {rx2_q, rx2_i, rx1_q, rx1_i}, each a 16-bit word holding a
//            12-bit RX sample right-aligned ([11:0] is used).
// m_axis_y : {y_q, y_i}, signed 16-bit:
//            filter:  y[n] = 2^s (x1[n-8] - sum_{k!=8} a_k x1[n-k]
//                                        - sum_k b_k x2n[n-k]
//                                        - c1 conj(x1[n-8]) - c2 conj(x2n[n-8]))
//            bypass:  y[n] = 2^s x1[n-8]
//            x2n = g2 * x2 (RX2 normalised to RX1, kept with 2 extra bits)
// wts      : 32 weight words, word k at bits [32k+31 : 32k]:
//              k = 0..15  -> a_k (RX1, delay k); word 8 is ignored
//              k = 16..31 -> b_{k-16} (RX2 normalised, delay k-16)
//            each {imag[31:16], real[15:0]}, signed Q3.12.
// wcj      : image taps, [31:0] c1 (RX1), [63:32] c2 (RX2 normalised), same
//            word format as wts.
// cfg      : [0] bypass, [10:8] output make-up shift s (0..4),
//            [14:12] weight scale ws (0..5; 6, 7 act as 5).
//            The weight words are w * 2^ws (still signed Q3.12). A jammer that
//            is 60-70 dB above the noise needs the weights to 1e-4 or better;
//            Q3.12 alone (2.4e-4) leaves a residual above the noise, and the
//            small weights of a minimum-norm null are the worst case. The
//            firmware picks the largest ws that keeps every weight inside
//            Q3.12, the core takes it out again after the sum (no range or
//            precision lost: the sum is 14 fractional bits wide).
// nrm      : g2 = {imag[31:16], real[15:0]}, signed Q3.13 (1.0 = 0x2000).
// wts, wcj, cfg and nrm come from gnss_passthrough's committed registers, in the
// same clock domain as this core, so a commit changes all three at once.
void smi_pi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<1024> wts,
            ap_uint<64>   wcj,
            ap_uint<32>   cfg,
            ap_uint<32>   nrm);

#endif
