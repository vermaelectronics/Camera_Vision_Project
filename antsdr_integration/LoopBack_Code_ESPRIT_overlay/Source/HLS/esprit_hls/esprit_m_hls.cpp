// ============================================================================
//  esprit_m_hls.cpp -- Vitis HLS top: M-ESPRIT DOA estimator (2 elements)
//  See esprit_hls.h for the interface and esprit_core.h for the algorithm.
// ============================================================================
#include "esprit_core.h"

void esprit_m_hls(hls::stream<snap_t> &s_axis_x,
                  hls::stream<word_t> &m_axis_y,
                  ap_uint<5>  log2_snap,
                  ap_uint<2>  p_log2,
                  ap_uint<1>  use_cov,
                  ap_uint<32> mu_q30,
                  ap_int<32>  k_q30,
                  ap_uint<32> uctol_q24)
{
#pragma HLS INTERFACE mode=axis    port=s_axis_x
#pragma HLS INTERFACE mode=axis    port=m_axis_y
#pragma HLS INTERFACE mode=ap_none port=log2_snap
#pragma HLS INTERFACE mode=ap_none port=p_log2
#pragma HLS INTERFACE mode=ap_none port=use_cov
#pragma HLS INTERFACE mode=ap_none port=mu_q30
#pragma HLS INTERFACE mode=ap_none port=k_q30
#pragma HLS INTERFACE mode=ap_none port=uctol_q24
#pragma HLS INTERFACE mode=ap_ctrl_none port=return
    esprit_window<ALGO_MESPRIT>(s_axis_x, m_axis_y, log2_snap, p_log2, use_cov,
                                mu_q30, k_q30, uctol_q24);
}
