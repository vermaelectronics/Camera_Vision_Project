// ============================================================================
//  pl_npi.h  --  pipelined PL-NPI power-inversion core (Vitis HLS)
//  ANTSDR E310 V1 / GNSS CRPA project
//
//  See pl_npi.cpp for the algorithm, the fixed-point formats and how this
//  version differs from the original single-cycle RTL
//  (Source/HDL/pl_npi_reference/rtl/pi_power_inversion_pl_npi.v).
// ============================================================================
#ifndef PL_NPI_H
#define PL_NPI_H

#include "ap_int.h"
#include "hls_stream.h"

// s_axis_x : {rx2_q, rx2_i, rx1_q, rx1_i}, each a signed 16-bit sample at the
//            raw ADC scale (12-bit RX samples, sign-extended, not shifted).
// m_axis_y : {y_q, y_i}, the array output at the same scale, saturated to
//            signed 16 bits.
// gamma    : regulariser of mu = 1/(2 x^T x + gamma); 0 is used as 1.
// adapt_en : 1 = weights adapt, 0 = weights frozen.
// gain_band: Eq. 11 gain segment of the latest sample, 0..3 = x1.00..x1.20.
void pl_npi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<32> gamma,
            ap_uint<1>  adapt_en,
            ap_uint<2> *gain_band);

#endif
