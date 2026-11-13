// ============================================================================
//  sfap_pi.h  --  space-frequency adaptive power inversion core (SFAP-PI v5.0)
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2021.1 (and later)
//
//  See sfap_pi.cpp for the structure, the fixed-point formats and the
//  latency. The weights are computed by the ARM
//  (Source/Firmware/app_sfap_lvds/gnss_aj_math.c) and reach this core through
//  gnss_passthrough's committed AJ_* registers.
// ============================================================================
#ifndef SFAP_PI_H
#define SFAP_PI_H

// FFT size and hop. Must match AJ_N / AJ_HOP in gnss_aj_math.h (firmware) and
// AJ_LOG2N in gnss_passthrough.v.
//   antsdr-fw-patch crpa13: 128 bins only (0.24 MHz per bin at 30.72 MS/s, 0.48 MHz at
//   61.44 MS/s); the 64-bin option of crpa6 is gone.
#ifndef SFAP_LOG2N
#define SFAP_LOG2N   7
#endif
#if SFAP_LOG2N != 7
#error "crpa13: sfap_pi is built for 128 bins (SFAP_LOG2N 7) only"
#endif
#define SFAP_N      (1 << SFAP_LOG2N)
#define SFAP_HOP    (SFAP_N / 2)        // 50 % overlap, two frame chains
#define SFAP_NW     (3 * SFAP_N)        // weight words: b_k, then c1_k, then c2_k (384)
#include "ap_int.h"
#include "hls_stream.h"

// Input -> output latency in samples, identical with the filter in and in
// bypass (so switching never moves the satellites' code phase):
//   forward FFT N-1 + one-frame image buffer N + inverse FFT N-1.
#define SFAP_LAT    (3 * SFAP_N - 2)    // 382 samples for 128 bins (190 for 64)

#define SFAP_WFRAC  12                  // weights, signed Q3.12 (1.0 = 4096)
#define SFAP_GFRAC  13                  // RX2 normalisation g2, signed Q3.13
#define SFAP_DFRAC   4                  // fractional bits kept in the data path
#define SFAP_MAXSH   4                  // largest output make-up shift (x16)

// cfg bits (AJ_CFG)
#define SFAP_CFG_BYPASS  0x1u           // 1 = output x1[n - SFAP_LAT], exactly
#define SFAP_CFG_SH_LSB  8              // [10:8] output make-up shift 0..4

// s_axis_x : {rx2_q, rx2_i, rx1_q, rx1_i}, each a 16-bit word holding a
//            12-bit RX sample right-aligned ([11:0] is used).
// m_axis_y : {y_q, y_i}, signed 16-bit, one output per input:
//            filter : y = 2^s * WOLA-synthesis{ Y_k }, per frame and bin k
//                     Y_k = X1_k - b_k X2n_k - c1_k conj(X1_-k) - c2_k conj(X2n_-k)
//            bypass : y[n] = 2^s x1[n - SFAP_LAT]
//            x2n = g2 * x2 (RX2 normalised to RX1 before the FFT)
// crpa13: the per-bin weights live in block RAM inside the core, in two banks:
// the filter reads bank `wbank`, gnss_passthrough writes the other one through
// the write port and flips wbank with AJ_COMMIT (all 384 words change at once,
// as before). A write is taken in every clock in which wseq changes:
// wseq     : write counter (gnss_passthrough adds 1 per weight write)
// waddr    : word i = 0 .. 3N-1: b_k (i = k), c1_k (i = N + k), c2_k (i = 2N + k);
//            bin k in natural FFT order (k >= N/2 = negative frequencies)
// wdata    : {imag[31:16], real[15:0]}, signed Q3.12. b_k weights X2n at bin k,
//            c1_k / c2_k (image, widely-linear taps) conj(X1 / X2n at bin -k).
// wbank    : the bank the filter uses (writes go to the other one)
// cfg      : [0] bypass, [10:8] make-up shift.
// nrm      : g2 = {imag[31:16], real[15:0]}, signed Q3.13 (1.0 = 0x2000).
void sfap_pi(hls::stream<ap_uint<64> > &s_axis_x,
             hls::stream<ap_uint<32> > &m_axis_y,
             ap_uint<16>   wseq,
             ap_uint<9>    waddr,
             ap_uint<32>   wdata,
             ap_uint<1>    wbank,
             ap_uint<32>   cfg,
             ap_uint<32>   nrm);

#endif
