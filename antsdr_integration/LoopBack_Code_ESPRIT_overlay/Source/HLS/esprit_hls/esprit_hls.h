// ============================================================================
//  esprit_hls.h -- Vitis HLS ESPRIT DOA kernels for the ANTSDR E310 CRPA
//                  (two-element array, RX1 = element 0, RX2 = element 1)
// ----------------------------------------------------------------------------
//  Algorithms: J. Li, G. Yang, X. Ren, Z. Fu, S. Qiu, "Modified ESPRIT and
//  Unitary-Transformation Form for Computationally Efficient DOA Estimation
//  on FPGA", IEEE Trans. Instrum. Meas., vol. 74, 2025, Art. 2003209.
//
//    esprit_iu_hls : I-UESPRIT (Sec. III-B, real-valued unitary form)
//    esprit_m_hls  : M-ESPRIT  (Sec. III-A, complex, unit-circle test)
//
//  Both use the 4th-order cumulant matrix of Eq. (7) (or the covariance when
//  use_cov = 1), the C^p sharpening of Eq. (38) and the closed forms for
//  M = 2 that the RTL cores and the Python models also use:
//      M-ESPRIT : psi = (a + d) b* / (a^2 + |b|^2 + mu),  phi = arg(psi)
//      I-UESPRIT: psi = r.s / (|r|^2 + mu),               phi = 2 atan(psi)
//      theta = asin(-k phi),  k = lambda / (2 pi d)
//
//  BLOCK-DESIGN INTERFACE (same pattern as the SMI power-inversion HLS IP)
//    ap_clk / ap_rst_n      sys_cpu_clk / gnss_passthrough doa_hls_rst_n
//    s_axis_x  (AXIS, 64b)  snapshots from gnss_passthrough m_axis_doa
//                           tdata = {q1, i1, q0, i0}, each int16 Q1.15
//    m_axis_y  (AXIS, 32b)  one 9-word result packet per window, back into
//                           gnss_passthrough s_axis_res (layout below)
//    log2_snap, p_log2, use_cov, mu_q30, k_q30 [, uctol_q24]
//                           ap_none scalars from gnss_passthrough doa_cfg_*
//    ap_ctrl_none           free running: one window per invocation
//
//  RESULT PACKET (m_axis_y, in this order)
//    w0  {16'hE5A1 magic, 4'b0, flags[3:0], 8'd0}
//          flags I-UESPRIT {0, 0, ambiguous, no_signal}
//          flags M-ESPRIT  {off_circle, 0, ambiguous, no_signal}
//    w1  phi_q28     electrical phase, rad Q28
//    w2  psi_q24     I: psi = tan(phi/2)   M: |psi| (~1.0 for a real source)
//    w3  theta_q28   DOA, rad Q28
//    w4  deg_q16     DOA, degrees Q16
//    w5  c00_q30     normalised C (max element = 1), debug
//    w6  c11_q30
//    w7  c01re_q30
//    w8  c01im_q30
//  gnss_passthrough writes these into the same registers the RTL cores use
//  (I_* at 0x138.., M_* at 0x15C..), so the firmware is unchanged.
//
//  Original work for this project.
// ============================================================================
#ifndef ESPRIT_HLS_H_
#define ESPRIT_HLS_H_

#include <ap_int.h>
#include <hls_stream.h>

typedef ap_uint<64> snap_t;      // {q1, i1, q0, i0}
typedef ap_uint<32> word_t;

const int  ESPRIT_PKT_WORDS = 9;
const unsigned ESPRIT_MAGIC = 0xE5A1u;
const int  ESPRIT_MIN_LOG2  = 7;    // 128 snapshots
const int  ESPRIT_MAX_LOG2  = 14;   // 16384 snapshots

// flag bits (w0[11:8])
const unsigned FLAG_NO_SIGNAL  = 1u << 0;
const unsigned FLAG_AMBIGUOUS  = 1u << 1;
const unsigned FLAG_OFF_CIRCLE = 1u << 3;   // M-ESPRIT only

void esprit_iu_hls(hls::stream<snap_t> &s_axis_x,
                   hls::stream<word_t> &m_axis_y,
                   ap_uint<5>  log2_snap,
                   ap_uint<2>  p_log2,
                   ap_uint<1>  use_cov,
                   ap_uint<32> mu_q30,
                   ap_int<32>  k_q30);

void esprit_m_hls(hls::stream<snap_t> &s_axis_x,
                  hls::stream<word_t> &m_axis_y,
                  ap_uint<5>  log2_snap,
                  ap_uint<2>  p_log2,
                  ap_uint<1>  use_cov,
                  ap_uint<32> mu_q30,
                  ap_int<32>  k_q30,
                  ap_uint<32> uctol_q24);

#endif
