// ============================================================================
//  lcmv_crpa.h  --  2-element LCMV CRPA core (Vitis HLS), block-design IP lcmv_0
//  ANTSDR E310 V1 / GNSS CRPA project
//
//  LCMV of R. Li, X. Zhao, X. W. Shi, PIER C vol. 4, 2008:
//      w = R^-1 C (C^H R^-1 C)^-1 f,        R = E[x x^H]
//  Split between this core (PL, every sample) and the ARM (lcmv_math.c):
//    PL  : R11 = E|x1|^2, R22 = E|x2|^2, R12 = E[x1 conj(x2)]  (4096-pair blocks)
//          y  = conj(w1) x1 + conj(w2) x2                     (beamformer)
//    ARM : reads R, solves w, writes w1/w2 (through gnss_passthrough).
//
//  Ports (all in the AD9361 sample clock domain, axi_ad9361/l_clk):
//    in1, in2 : AXI-Stream {q[31:16], i[15:0]}, RX1 / RX2 samples at the raw
//               ADC scale (12 bits, sign-extended to 16).
//    out_r    : AXI-Stream {q, i}, beamformer output, saturated to 12 bits
//               and sign-extended to 16 (same format as the input).
//    w1_re .. w2_im : weights, Q16.16 (bits [24:0] used, |w| < 128), the
//               final beamformer weights with the RX2 calibration included.
//    cov_*    : last completed covariance block, Q.4 LSB^2.
//    cov_blocks : blocks completed. sat_count : output samples clipped.
// ============================================================================
#ifndef LCMV_CRPA_H
#define LCMV_CRPA_H

#include "ap_int.h"
#include "hls_stream.h"

#define LCMV_COV_LOG2N 12          // 4096 sample pairs per covariance block

void lcmv_crpa(hls::stream<ap_uint<32> > &in1,
               hls::stream<ap_uint<32> > &in2,
               hls::stream<ap_uint<32> > &out_r,
               ap_int<32>   w1_re,
               ap_int<32>   w1_im,
               ap_int<32>   w2_re,
               ap_int<32>   w2_im,
               ap_uint<32> *cov_r11,
               ap_uint<32> *cov_r22,
               ap_int<32>  *cov_r12_re,
               ap_int<32>  *cov_r12_im,
               ap_uint<32> *cov_blocks,
               ap_uint<32> *sat_count);

#endif
