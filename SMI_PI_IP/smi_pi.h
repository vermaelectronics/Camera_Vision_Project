// ============================================================================
//  smi_pi.h  --  closed-form (sample-matrix-inversion) power inversion core
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2
//
//  See smi_pi.cpp for the algorithm, the fixed-point formats and why this
//  core meets the 8 ns rx_clk where the LMS-type cores (PI-NLMS, PL-NPI)
//  struggle.
// ============================================================================
#ifndef SMI_PI_H
#define SMI_PI_H

#include "ap_int.h"
#include "hls_stream.h"

// Averaging time constant of the covariance estimate: 2^SMI_PI_K samples.
// 10 = 1024 samples, 33 us at 30.72 MS/s. Overridable from build_hls.tcl.
#ifndef SMI_PI_K
#define SMI_PI_K 10
#endif

// s_axis_x : {rx2_q, rx2_i, rx1_q, rx1_i}, each a signed 16-bit sample at the
//            raw ADC scale (12-bit RX samples, sign-extended, not shifted).
// m_axis_y : {y_q, y_i} = x1 - a*x2, same scale, saturated to signed 16 bits.
// load     : diagonal loading L in LSB^2 added to E|x2|^2. Only keeps the
//            division well-behaved; 0 acts as 1. Typical 16..1000.
// adapt_en : 1 = covariance (and so the weight) tracks, 0 = frozen.
// wt_band  : |a| of the current weight, 0: <0.25  1: <1  2: <4  3: >=4.
//            0 means essentially nothing is being cancelled (no jammer).
void smi_pi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<32> load,
            ap_uint<1>  adapt_en,
            ap_uint<2> *wt_band);

#endif
