// ============================================================================
//  smi_pi.cpp  --  space-time SMI power inversion core (SMI-PI v4.0)
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2
//
//  WHY v4 (what failed in v3.x on the bench)
//    v3 had ONE complex weight a: y = x1 - a*x2. One number can only match
//    the two receive paths at one frequency. A wideband jammer sees a
//    different RX1/RX2 ratio at every frequency in the band (cable length,
//    antenna spacing, the two AD9361 analog filters, reflections), so the
//    null is shallow across the band. Simulated, 50 MS/s, jammer 30 dB above
//    the noise, filling the band:
//        cable difference 0.5 ns : one weight 29.5 dB, this core 46 dB
//        cable difference 3 ns   : one weight 12.9 dB, this core 40.6 dB
//        jammer + reflection -6 dB, 50 ns later: one weight 5 dB, this core 40 dB
//    12.9 dB is what the bench showed: satellites gone at 20 dB and above.
//
//  FILTER
//    A tapped delay line on BOTH channels, 16 taps each (delays 0..15):
//        y[n] = x1[n-8] - sum_{k != 8} a_k x1[n-k] - sum_k b_k x2[n-k]
//    The RX1 tap at delay 8 is the reference, fixed at 1 (power inversion,
//    w1 = 1), so the output can never be turned down as a whole. The 16 RX2
//    taps follow the frequency-dependent RX1/RX2 ratio; the 15 RX1 taps let
//    the filter also cancel a reflected copy of the jammer arriving from
//    another direction.
//
//  WEIGHTS
//    The 31 weights are the SMI (closed-form) solution of the space-time
//    power-inversion problem, computed by the ARM from a 4096-sample DMA
//    capture of both channels (Source/Firmware/app_gnss_e310/gnss_aj_math.c)
//    a few hundred times a second, and written to gnss_passthrough, which
//    hands them to this core all at once. A 31x31 complex solve in fixed
//    point inside the FPGA would need a floating-point-class dynamic range
//    (jammers 0..60 dB above the noise); the Cortex-A9 does it in double in
//    well under a millisecond. Nothing in the sample path waits for it: the
//    core always runs with the last complete weight set.
//
//  TIMING
//    One sample pair per clock (II=1) at 100 MHz (sys_cpu_clk), i.e. up to
//    100 MS/s, beyond the AD9361's 61.44. No loop-carried state except the
//    delay lines, so every multiplier is a plain pipeline stage.
//
//  FIXED POINT
//    x       : 12-bit signed (from [11:0] of each 16-bit word)
//    weights : 16-bit signed Q3.12, |w| < 8
//    product : complex multiply with 3 real multipliers (DSP48 pre-adder):
//                k1 = wr (xr + xi), k2 = xr (wi - wr), k3 = xi (wr + wi)
//                re = k1 - k3,      im = k1 + k2
//              93 multipliers for the 31 taps instead of 124.
//    sum     : 40 bits, then rounded to nearest (>> 12) and saturated to 16.
// ============================================================================
#include "smi_pi.h"

typedef ap_int<12> s12_t;
typedef ap_int<16> w16_t;
typedef ap_int<31> prod_t;
typedef ap_int<40> acc_t;

static ap_int<16> sat16(acc_t v) {
#pragma HLS INLINE
    if (v > 32767)  return 32767;
    if (v < -32768) return -32768;
    return (ap_int<16>)v;
}

static void cmul3(w16_t wr, w16_t wi, s12_t xr, s12_t xi, prod_t &pr, prod_t &pi) {
#pragma HLS INLINE
    ap_int<13> xs = ap_int<13>(xr) + ap_int<13>(xi);
    ap_int<17> d1 = ap_int<17>(wi) - ap_int<17>(wr);
    ap_int<17> d2 = ap_int<17>(wr) + ap_int<17>(wi);
    ap_int<30> k1 = wr * xs;
    ap_int<30> k2 = xr * d1;
    ap_int<30> k3 = xi * d2;
    pr = prod_t(k1) - prod_t(k3);
    pi = prod_t(k1) + prod_t(k2);
}

void smi_pi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<1024> wts) {
#pragma HLS INTERFACE axis port=s_axis_x
#pragma HLS INTERFACE axis port=m_axis_y
#pragma HLS INTERFACE ap_none port=wts
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=1

    // Delay lines: index k holds the sample from k samples ago (0 = now).
    static s12_t h1r[SMI_NT], h1i[SMI_NT], h2r[SMI_NT], h2i[SMI_NT];
#pragma HLS ARRAY_PARTITION variable=h1r complete
#pragma HLS ARRAY_PARTITION variable=h1i complete
#pragma HLS ARRAY_PARTITION variable=h2r complete
#pragma HLS ARRAY_PARTITION variable=h2i complete
#pragma HLS RESET variable=h1r
#pragma HLS RESET variable=h1i
#pragma HLS RESET variable=h2r
#pragma HLS RESET variable=h2i

    if (s_axis_x.empty()) return;
    ap_uint<64> in = s_axis_x.read();

    for (int k = SMI_NT - 1; k > 0; k--) {
#pragma HLS UNROLL
        h1r[k] = h1r[k - 1]; h1i[k] = h1i[k - 1];
        h2r[k] = h2r[k - 1]; h2i[k] = h2i[k - 1];
    }
    h1r[0] = (s12_t)in(11, 0);  h1i[0] = (s12_t)in(27, 16);
    h2r[0] = (s12_t)in(43, 32); h2i[0] = (s12_t)in(59, 48);

    acc_t ar = acc_t(h1r[SMI_REF]) << SMI_WFRAC;
    acc_t ai = acc_t(h1i[SMI_REF]) << SMI_WFRAC;
    for (int k = 0; k < SMI_NT; k++) {
#pragma HLS UNROLL
        prod_t pr, pi;
        if (k != SMI_REF) {
            w16_t wr = (w16_t)wts(32 * k + 15, 32 * k);
            w16_t wi = (w16_t)wts(32 * k + 31, 32 * k + 16);
            cmul3(wr, wi, h1r[k], h1i[k], pr, pi);
            ar -= pr; ai -= pi;
        }
        {
            const int j = SMI_NT + k;
            w16_t wr = (w16_t)wts(32 * j + 15, 32 * j);
            w16_t wi = (w16_t)wts(32 * j + 31, 32 * j + 16);
            cmul3(wr, wi, h2r[k], h2i[k], pr, pi);
            ar -= pr; ai -= pi;
        }
    }

    // Round to nearest, not floor: a floor leaves a -0.5 LSB DC offset and
    // extra quantisation noise on every output sample (see v3.1).
    const acc_t half = acc_t(1) << (SMI_WFRAC - 1);
    ap_int<16> yr = sat16((ar + half) >> SMI_WFRAC);
    ap_int<16> yi = sat16((ai + half) >> SMI_WFRAC);
    ap_uint<32> out;
    out(15, 0)  = (ap_uint<16>)yr;
    out(31, 16) = (ap_uint<16>)yi;
    m_axis_y.write(out);
}
