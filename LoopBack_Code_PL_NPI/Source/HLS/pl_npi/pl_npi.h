// ============================================================================
//  pl_npi.h  --  pipelined PL-NPI power-inversion core (Vitis HLS), v2.3
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

#define PL_NPI_NW        3     // adaptive weights: w2 (RX2), c1, c2 (image)
#define PL_NPI_GFRAC    13     // RX2 normalisation g2, signed Q3.13 (1.0 = 8192)
#define PL_NPI_MAXSH     4     // largest output make-up shift (x16 = +24 dB)
#define PL_NPI_LATENCY   2     // output sample k carries input sample k-2 (filter and bypass)

// cfg bits
#define PL_NPI_MODE_AUTO   0u  // [1:0] = 0: the power detector switches the
                               //   filter and the adaptation on and off
#define PL_NPI_MODE_BYPASS 1u  //         1: always TX1 = RX1, weights held at 0
#define PL_NPI_MODE_ON     2u  //         2: filter in and adapting, always
#define PL_NPI_MODE_HOLD   3u  //         3: filter in, weights frozen
#define PL_NPI_CFG_IMAGE   0x4u // [2] image taps c1, c2 on
#define PL_NPI_CFG_DCBLK   0x8u // [3] input DC blocker on (v2.4)
#define PL_NPI_CFG_SH_LSB  8   // [10:8] output make-up shift, 0..4
#define PL_NPI_CFG_NOTCH_LSB 12 // [13:12] CW notch k on (v2.4)

// v2.4: CW notch filters on the output (after PL-NPI / bypass). Number
// compiled in (0..2); the core reports it in npi_st[7:5].
#ifndef PL_NPI_NOTCHES
#define PL_NPI_NOTCHES     2
#endif
#define PL_NPI_NFRAC      14    // notch coefficients, signed Q1.14
#define PL_NPI_DC_SHIFT   14    // DC blocker: corner fs * 2^-14 / (2 pi)

// npi_st bits
#define PL_NPI_ST_DET      0x1u  // [0] power detector on
#define PL_NPI_ST_ACTIVE   0x2u  // [1] filter in the path (not bypassed)
#define PL_NPI_ST_BAND_LSB 2     // [3:2] Eq. 11 gain band of the latest sample
#define PL_NPI_ST_ADAPT    0x10u // [4] weights adapting
#define PL_NPI_ST_NOTCH_LSB 5    // [7:5] notch filters compiled in (v2.4)
#define PL_NPI_ST_DCBLK    0x100u // [8] DC blocker on (v2.4)
#define PL_NPI_ST_CNT_LSB  16    // [31:16] detector switch-on count

// s_axis_x : {rx2_q, rx2_i, rx1_q, rx1_i}, each a signed 16-bit sample at the
//            raw ADC scale (12-bit RX samples, sign-extended, not shifted).
// m_axis_y : {y_q, y_i}, the array output at the same scale, saturated to
//            signed 16 bits.
// gamma    : regulariser of mu = 1/(2 x^T x + gamma); 0 is used as 1.
// adapt_en : 0 = weights frozen (CONTROL[6], gnss_npi_freeze=1).
// cfg      : mode, image taps, make-up shift (bits above).
// nrm      : g2 = {imag[31:16], real[15:0]}, signed Q3.13: the core uses
//            x2n = g2 * x2 in place of RX2.
// det_thr  : detector threshold on the averaged |x1|^2 + |x2|^2 [LSB^2]:
//            on above it, off det_hold samples after it falls below half.
// det_hold : see det_thr.
// ncoef    : v2.4, notch k (0, 1) in words 3k..3k+2 (32 bits each,
//            {imag[31:16], real[15:0]}, signed Q1.14): q = e^{j w0}, p = r q,
//            p^2. Notch H(z) = (1 - q z^-1) / (1 - p z^-1), r <= 1 - 2^-8.
// npi_st   : status (bits above).
// wout     : the three weights, Q12.20, word k at [64k+63 : 64k] =
//            {imag[63:32], real[31:0]}: k = 0 w2, 1 c1, 2 c2.
// det_pow  : the detector's averaged input power [LSB^2].
void pl_npi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<32>  gamma,
            ap_uint<1>   adapt_en,
            ap_uint<32>  cfg,
            ap_uint<32>  nrm,
            ap_uint<32>  det_thr,
            ap_uint<32>  det_hold,
            ap_uint<192> ncoef,
            ap_uint<32>  *npi_st,
            ap_uint<192> *wout,
            ap_uint<32>  *det_pow);

#endif
