// ============================================================================
//  smi_pi.h  --  space-time SMI power inversion core (SMI-PI v4.0)
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
#define SMI_NT   16
#define SMI_REF   8
#define SMI_WFRAC 12          // weights are signed Q3.12 (1.0 = 4096)

// s_axis_x : {rx2_q, rx2_i, rx1_q, rx1_i}, each a 16-bit word holding a
//            12-bit RX sample right-aligned ([11:0] is used).
// m_axis_y : {y_q, y_i}, signed 16-bit, same scale as RX1:
//            y[n] = x1[n-8] - sum_{k!=8} a_k x1[n-k] - sum_k b_k x2[n-k]
// wts      : 32 weight words of 32 bits, word k at bits [32k+31 : 32k]:
//              k = 0..15  -> a_k (RX1, delay k); word 8 is ignored
//              k = 16..31 -> b_{k-16} (RX2, delay k-16)
//            each {imag[31:16], real[15:0]}, signed Q3.12.
//            Driven by gnss_passthrough's committed weight registers, in the
//            same clock domain as this core, so a commit is atomic: every
//            sample uses either the whole old set or the whole new set.
void smi_pi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<1024> wts);

#endif
