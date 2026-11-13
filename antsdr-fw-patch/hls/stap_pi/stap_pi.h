// ============================================================================
//  stap_pi.h  --  space-time power-inversion FILTER, weights from the ARM
//                 (antsdr-fw-patch crpa16 / crpa17, stap_pi 1.0)
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2
//
//  The ARM calculates the weights (crpa16/17: an NLMS algorithm running on the
//  8192-sample captures, gnss_crpa/src/gnss_aj_math.c aj_nlms_solve) and writes
//  them to gnss_passthrough; this core only applies them, one sample per
//  clock. It is the filter of the crpa13 nlms_pi core without the NLMS loop,
//  i.e. the crpa1-crpa9 type of core (weights on ap_none ports), without the
//  crpa8-12 weight smoothing.
//
//  See stap_pi.cpp for the filter and the fixed-point formats.
// ============================================================================
#ifndef STAP_PI_H
#define STAP_PI_H

#include "ap_int.h"
#include "hls_stream.h"

#ifndef ST_NT
#define ST_NT      8     // taps per antenna (crpa16: 8 + 8; crpa17: 6 + 6)
#endif
#ifndef ST_REF
#define ST_REF     4     // RX1 reference tap delay, fixed at 1 (crpa16: 4; crpa17: 3)
#endif
#define ST_NIMG    3     // image (conjugate) taps per antenna at delays REF-1, REF, REF+1
#define ST_IMGH    ((ST_NIMG - 1) / 2)
#define ST_NST     (2 * ST_NT)                   // space-time weight words (word ST_REF unused)
#define ST_NW      (ST_NST + 2 * ST_NIMG)        // all weight words

#define ST_WFRAC  12     // weights signed Q3.12 (1.0 = 4096), held as w * 2^ws
#define ST_GFRAC  13     // RX2 normalisation g2, signed Q3.13 (1.0 = 8192)
#define ST_XFRAC   2     // fractional bits kept on the normalised RX2
#define ST_MAXSH   7     // largest output make-up shift (6 dB each)
#define ST_MAXWS   5     // largest weight scale

// cfg (AJ_CFG, 0x188)
#define ST_CFG_BYPASS   0x1u   // [0] 1 = y = x1[n-REF] exactly (weights not used)
#define ST_CFG_SH_LSB   8      // [10:8] output make-up shift 0..7
#define ST_CFG_WS_LSB   12     // [14:12] weight scale ws 0..5: the words hold w * 2^ws
// gain (AJ_GAIN, 0x1B0): fine output gain, unsigned Q2.14 after the make-up shift
#define ST_GAIN_FRAC   14
#define ST_GAIN_ONE    16384

// wts : word j (j = 0..ST_NST-1) at bits [32j+31:32j], {imag[31:16], real[15:0]}, Q3.12 x 2^ws
//         j < ST_NT: RX1 tap at delay j (word ST_REF is ignored: the reference)
//         ST_NT <= j < ST_NST: RX2 tap at delay j - ST_NT (on the normalised RX2)
// wcj : the 2 x ST_NIMG image taps, word c at [32c+31:32c]: c1 on conj(x1[n-a]) for
//         a = REF-1, REF, REF+1 (c = 0..2), then c2 on conj(x2n[n-a]) (c = 3..5)
// The words are the COMMITTED values (gnss_passthrough switches the whole set in one
// clock on AJ_COMMIT).
void stap_pi(hls::stream<ap_uint<64> > &s_axis_x,
             hls::stream<ap_uint<32> > &m_axis_y,
             ap_uint<32 * ST_NST>      wts,
             ap_uint<32 * 2 * ST_NIMG> wcj,
             ap_uint<32>   cfg,
             ap_uint<32>   nrm,
             ap_uint<32>   gain);

#endif
