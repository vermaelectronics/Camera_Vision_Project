// ============================================================================
//  lcmv_crpa.cpp  --  2-element LCMV CRPA core (Vitis HLS)
//  ANTSDR E310 V1 / GNSS CRPA project. See lcmv_crpa.h for the ports.
//
//  One sample pair per call. ap_ctrl_none: the core runs continuously from
//  reset, like pl_npi / pi_nlms in the earlier designs.
//
//  #pragma HLS PIPELINE II=2: in the E310 2R2T LVDS mode the AD9361
//  interface delivers at most one sample pair every second l_clk (8 ns), so
//  II=2 loses nothing.
//
//  FIXED POINT
//    x   : ap_int<12> raw ADC samples
//    w   : ap_int<25> Q8.16 (from the 32-bit Q16.16 ports)
//    y   : sum of four 25x12 products (ap_int<40>), rounded >> 16 and
//          saturated to 12 bits; every clipped sample counts in sat_count
//    R   : products ap_int<25>, 48-bit block accumulators, output = block
//          sum >> (LOG2N - 4), i.e. the block average in Q.4 (LSB^2 * 16)
// ============================================================================
#include "lcmv_crpa.h"

typedef ap_int<12> smp_t;
typedef ap_int<25> w_t;

static ap_int<12> sat12(ap_int<41> v, bool &clipped) {
#pragma HLS INLINE
    if (v > 2047)  { clipped = true; return 2047; }
    if (v < -2048) { clipped = true; return -2048; }
    clipped = false;
    return (ap_int<12>)v;
}

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
               ap_uint<32> *sat_count)
{
#pragma HLS INTERFACE axis port=in1
#pragma HLS INTERFACE axis port=in2
#pragma HLS INTERFACE axis port=out_r
#pragma HLS INTERFACE ap_none port=w1_re
#pragma HLS INTERFACE ap_none port=w1_im
#pragma HLS INTERFACE ap_none port=w2_re
#pragma HLS INTERFACE ap_none port=w2_im
#pragma HLS INTERFACE ap_none port=cov_r11
#pragma HLS INTERFACE ap_none port=cov_r22
#pragma HLS INTERFACE ap_none port=cov_r12_re
#pragma HLS INTERFACE ap_none port=cov_r12_im
#pragma HLS INTERFACE ap_none port=cov_blocks
#pragma HLS INTERFACE ap_none port=sat_count
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=2

    // ---- state ---------------------------------------------------------------
    static ap_int<48> acc11 = 0, acc22 = 0, acc12r = 0, acc12i = 0;
#pragma HLS RESET variable=acc11
#pragma HLS RESET variable=acc22
#pragma HLS RESET variable=acc12r
#pragma HLS RESET variable=acc12i
    static ap_uint<LCMV_COV_LOG2N> cnt = 0;
#pragma HLS RESET variable=cnt
    static ap_uint<32> r11 = 0, r22 = 0, blocks = 0, nsat = 0;
#pragma HLS RESET variable=r11
#pragma HLS RESET variable=r22
#pragma HLS RESET variable=blocks
#pragma HLS RESET variable=nsat
    static ap_int<32> r12r = 0, r12i = 0;
#pragma HLS RESET variable=r12r
#pragma HLS RESET variable=r12i

    // ---- one snapshot x = [x1, x2] -------------------------------------------
    ap_uint<32> a = in1.read();
    ap_uint<32> b = in2.read();
    smp_t x1r = (smp_t)a(11, 0),  x1i = (smp_t)a(27, 16);
    smp_t x2r = (smp_t)b(11, 0),  x2i = (smp_t)b(27, 16);

    // ---- beamformer  y = conj(w1) x1 + conj(w2) x2 ------------------------------
    w_t a1r = (w_t)w1_re(24, 0), a1i = (w_t)w1_im(24, 0);
    w_t a2r = (w_t)w2_re(24, 0), a2i = (w_t)w2_im(24, 0);
    ap_int<40> yr = (ap_int<40>)(a1r * x1r) + (ap_int<40>)(a1i * x1i)
                  + (ap_int<40>)(a2r * x2r) + (ap_int<40>)(a2i * x2i);
    ap_int<40> yi = (ap_int<40>)(a1r * x1i) - (ap_int<40>)(a1i * x1r)
                  + (ap_int<40>)(a2r * x2i) - (ap_int<40>)(a2i * x2r);
    bool cr, ci;
    ap_int<12> or_ = sat12((yr + 32768) >> 16, cr);
    ap_int<12> oi  = sat12((yi + 32768) >> 16, ci);
    if (cr || ci) nsat++;

    ap_uint<32> o;
    o(15, 0)  = (ap_uint<16>)(ap_int<16>)or_;
    o(31, 16) = (ap_uint<16>)(ap_int<16>)oi;
    out_r.write(o);

    // ---- covariance  R = E[x x^H] -------------------------------------------------
    ap_int<25> p11  = (ap_int<25>)(x1r * x1r) + (ap_int<25>)(x1i * x1i);
    ap_int<25> p22  = (ap_int<25>)(x2r * x2r) + (ap_int<25>)(x2i * x2i);
    ap_int<25> p12r = (ap_int<25>)(x1r * x2r) + (ap_int<25>)(x1i * x2i);
    ap_int<25> p12i = (ap_int<25>)(x1i * x2r) - (ap_int<25>)(x1r * x2i);

    ap_int<48> s11 = acc11 + p11, s22 = acc22 + p22;
    ap_int<48> s12r = acc12r + p12r, s12i = acc12i + p12i;
    const int SH = LCMV_COV_LOG2N - 4;           // block average, Q.4
    if (cnt == (ap_uint<LCMV_COV_LOG2N>)(-1)) {
        r11  = (ap_uint<32>)s11(SH + 31, SH);
        r22  = (ap_uint<32>)s22(SH + 31, SH);
        r12r = (ap_int<32>)s12r(SH + 31, SH);
        r12i = (ap_int<32>)s12i(SH + 31, SH);
        blocks++;
        acc11 = 0; acc22 = 0; acc12r = 0; acc12i = 0;
    } else {
        acc11 = s11; acc22 = s22; acc12r = s12r; acc12i = s12i;
    }
    cnt++;

    // outputs hold the last completed block (driven every call)
    *cov_r11    = r11;
    *cov_r22    = r22;
    *cov_r12_re = r12r;
    *cov_r12_im = r12i;
    *cov_blocks = blocks;
    *sat_count  = nsat;
}
