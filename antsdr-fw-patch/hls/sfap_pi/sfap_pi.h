// ============================================================================
//  sfap_pi.h  --  space-frequency adaptive power inversion core (SFAP-PI v6.0,
//  256 bins). ANTSDR E310 / GNSS CRPA project, Vitis HLS 2023.2 (2021.1+)
//
//  See sfap_pi.cpp for the structure, the fixed-point formats and the
//  latency. The weights are computed by the ARM
//  (Source/Firmware/app_sfap_lvds/gnss_aj_math.c), written to
//  gnss_passthrough's double-buffered weight RAM and read by this core
//  through its wb/wc1/wc2 memory ports (the committed bank).
// ============================================================================
#ifndef SFAP_PI_H
#define SFAP_PI_H

#include "ap_int.h"
#include "hls_stream.h"

// FFT size and hop. Must match AJ_N / AJ_HOP in gnss_aj_math.h and the
// tables in sfap_tables.h (gen_tables.py). N = 256 bins: 0.24 MHz per bin at
// 61.44 MS/s, 0.12 MHz at 30.72 MS/s.
#define SFAP_LOG2N   8
#define SFAP_N     256
#define SFAP_HOP   128                  // 50 % overlap, two frame chains
// Input -> output latency in samples, identical with the filter in and in
// bypass (so switching never moves the satellites' code phase):
//   forward FFT N-1 + one-frame image buffer N + inverse FFT N-1.
#define SFAP_LAT    (3 * SFAP_N - 2)    // 766 samples (12.5 us at 61.44 MS/s)
// SDF stages with a delay line of at least this many samples keep it in a
// small RAM (circular buffer) instead of a register chain.
#define SFAP_RAM_DELAY 64

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
// wb       : b_k, word k = bin k (natural FFT order, k >= N/2 = negative
//            frequencies), {imag[31:16], real[15:0]}, signed Q3.12.
// wc1, wc2 : image (widely-linear) taps c1_k on conj(X1 at bin -k) and
//            c2_k on conj(X2n at bin -k), same layout.
//            The three weight arrays are READ-ONLY MEMORY PORTS (ap_memory,
//            two read ports each: wb_address0/ce0/q0 and wb_address1/ce1/q1,
//            read latency 1). gnss_passthrough v6.0 serves them from its
//            double-buffered weight RAM (committed bank).
// cfg      : [0] bypass, [10:8] make-up shift.
// nrm      : g2 = {imag[31:16], real[15:0]}, signed Q3.13 (1.0 = 0x2000).
void sfap_pi(hls::stream<ap_uint<64> > &s_axis_x,
             hls::stream<ap_uint<32> > &m_axis_y,
             const ap_uint<32> wb[SFAP_N],
             const ap_uint<32> wc1[SFAP_N],
             const ap_uint<32> wc2[SFAP_N],
             ap_uint<32>   cfg,
             ap_uint<32>   nrm);

#endif
