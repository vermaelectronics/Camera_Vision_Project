// ============================================================================
//  nlms_pi.h  --  space-time power-inversion core with the weights adapted by
//                 a hardware NLMS loop only (antsdr-fw-patch crpa13, nlms_pi 1.0;
//                 crpa14: 6 + 6 taps, nlms_pi 1.1)
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2
//
//  crpa13: the SMI solver is gone. crpa1-crpa12 computed the weights on the
//  ARM (sample-matrix inversion, SMI) and wrote them to the FPGA; crpa10-12
//  also refined them with an NLMS loop in the FPGA. Here the NLMS loop is the
//  only adaptation: the weights live in the core, start at zero and follow the
//  jammer sample by sample. The ARM only sets the step size, RX2 gain g2,
//  bypass, output gain and reads the weights back for its reports.
//
//  See nlms_pi.cpp for the filter, the fixed-point formats and the timing.
// ============================================================================
#ifndef NLMS_PI_H
#define NLMS_PI_H

#include "ap_int.h"
#include "hls_stream.h"

#define NL_NT      6     // taps per antenna (crpa14: 6 + 6; crpa13 and crpa10-12: 8 + 8)
#define NL_REF     3     // RX1 reference tap delay (fixed at 1); crpa14: 3 (as crpa1/2 6 + 6), crpa13: 4
#define NL_NIMG    3     // image (conjugate) taps per antenna at delays REF-1, REF, REF+1
#define NL_IMGH    ((NL_NIMG - 1) / 2)
#define NL_NW      (2 * NL_NT + 2 * NL_NIMG)   // 18 weight words (word NL_REF is never used); crpa13: 22

#define NL_WFRAC  12     // weights as used by the filter: signed Q3.12 (1.0 = 4096)
#define NL_GFRAC  13     // RX2 normalisation g2, signed Q3.13 (1.0 = 8192)
#define NL_XFRAC   2     // fractional bits kept on the normalised RX2
#define NL_MAXSH   7     // largest output make-up shift (6 dB each)
#define NL_MAXWS   5     // largest weight scale

// cfg (AJ_CFG, 0x188)
#define NL_CFG_BYPASS   0x1u   // [0] 1 = y = x1[n-REF]; the weights are cleared to 0
#define NL_CFG_SH_LSB   8      // [10:8] output make-up shift 0..7
#define NL_CFG_WS_LSB   12     // [14:12] weight scale ws 0..5: the weights are held as w * 2^ws
                               //         (finer steps for a very strong jammer). A change of ws
                               //         clears the weights (they re-adapt from 0).
// gain (AJ_GAIN, 0x1B0): fine output gain, unsigned Q2.14 after the make-up shift
#define NL_GAIN_FRAC   14
#define NL_GAIN_ONE    16384

// nlms (AJ_NLMS, 0x1E0): [31] adapt, [20:16] sh, [15:0] m. Every sample each weight moves by
//   e[n-D] conj(u[n-D]) >> NL_CS, e = y m 2^-sh (y = output before the make-up gain).
//   [31] = 0 with the filter in: the weights are held (frozen).
#define NL_D      64     // delay of the error / inputs used for the update [samples] (crpa10-12: 24)
#define NL_HB    128     // circular buffers of 128 slots (> NL_D)
#define NL_HBB     7     // their pointer width
#if NL_D != 64
#error "the DEPENDENCE distance in nlms_pi.cpp is written as 64: change it with NL_D"
#endif
#define NL_RF     16     // fractional bits of the weight state below the Q3.12 LSB
#define NL_CS      8     // fixed shift of e conj(u)
#define NL_EW     24     // bits of e (saturated)

// wmon: one weight per sample, round robin, for the ARM's reports:
//   [39:32] index j (0..NL_NW-1), [31:16] imaginary, [15:0] real (Q3.12 x 2^ws, as used).
//   j < NL_NT: RX1 tap at delay j (j = NL_REF is the reference: always 0)
//   NL_NT <= j < 2 NL_NT: RX2 tap at delay j - NL_NT
//   then c1 at delays REF-1, REF, REF+1 and c2 at the same delays.
void nlms_pi(hls::stream<ap_uint<64> > &s_axis_x,
             hls::stream<ap_uint<32> > &m_axis_y,
             ap_uint<32>   cfg,
             ap_uint<32>   nrm,
             ap_uint<32>   gain,
             ap_uint<32>   nlms,
             ap_uint<40>   &wmon);

#endif
