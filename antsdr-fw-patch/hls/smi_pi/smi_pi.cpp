// ============================================================================
//  smi_pi.cpp  --  space-time SMI power inversion core (SMI-PI v4.4; antsdr-fw-patch crpa9: 10 + 10 taps, 3 + 3 image taps, weight smoothing, fine gain)
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2
//
//  PIPELINE (one sample pair per clock)
//    1. INPUT NORMALISATION   x2n = g2 * x2
//       RX2 is scaled to RX1's level before anything else (g2 from the two
//       receivers' noise floors, learned by the firmware while there is no
//       jammer). The weights then stay near 1 whatever the two channels'
//       gains are (cable loss, antenna gain, LNA): an RX2 20 dB weaker than
//       RX1 would otherwise need a weight of 10, beyond the Q3.12 range
//       (|w| < 8), and the null would fail. x2n keeps 2 fractional bits so the
//       scaling adds no rounding noise of its own.
//    2. SPACE-TIME FILTER     SMI_NT taps on RX1 and SMI_NT on the normalised RX2
//       (crpa7: 10 + 10, reference R = SMI_REF = 5; 32 + 32 in 4.9.7, 16 + 16 before):
//         y[n] = x1[n-R] - sum_{k != R} a_k x1[n-k] - sum_k b_k x2n[n-k]
//                        - sum_d c1_d conj(x1[n-R-d]) - sum_d c2_d conj(x2n[n-R-d])   (crpa9: d = -1, 0, 1)
//       The RX1 tap at delay R is the reference, fixed at 1 (power
//       inversion), so the output can never be turned down as a whole. The
//       RX2 taps follow the frequency-dependent RX1/RX2 ratio across the band
//       (cables, antenna spacing, the two analog filters); the RX1 taps also
//       cancel a reflected copy of the jammer.
//       c1, c2 are the IMAGE taps (widely-linear). The AD9361's I/Q mixers
//       are not perfectly balanced: each channel also carries a mirror image
//       of the jammer, ~50 dB down (its image rejection). Against a jammer
//       50 dB above the noise that image sits at the noise level and no
//       linear filter of two antennas can remove it (C model: +4.2 dB residual
//       at J/N 50 dB, +9 dB at 60 dB). The image is a copy of conj(jammer), so
//       one conjugate tap per channel cancels it (+0.2 dB and +1.4 dB). The
//       satellites have no image partner in the band, so the taps leave them
//       alone. crpa9: the mirror is not the same across a wide band (the I and
//       Q analog filters differ), so each channel has SMI_NIMG = 3 image taps,
//       at delays R-1, R, R+1 (C simulation, scenario 25, J/N 61 dB: residual
//       +8.6 dB with one tap, +3.2 dB with three).
//    3. BYPASS                cfg[0] = 1: y[n] = x1[n-R] (crpa8: all weights 0, so with
//       smoothing on the filter fades out instead of switching off).
//       The firmware keeps the filter bypassed while there is no jammer and
//       switches it in when one appears. Same SMI_REF-sample delay in both modes,
//       so switching never shifts the satellites' code phase.
//    4. OUTPUT MAKE-UP GAIN   y <<= s, s = cfg[10:8] (0..7, 6 dB each)
//       A strong jammer makes the RX gain loop cut the gain (26 dB for a
//       jammer 50 dB above the noise, 36 dB at 61 dB); the satellites then
//       reach the output at a few LSB, and rounding them to integers would
//       cost SNR. The accumulator has 14 fractional bits, so the shift
//       restores the level before the rounding, at no cost in precision.
//       v4.5: up to 7 steps (+42 dB, was 4 = +24 dB). The output into the
//       GNSS receiver has to stay at the clean level, or its own noise starts
//       to dominate. The firmware (aj_makeup_shift) sets the shift from the
//       gain cut and the residual so that the output is never more than 3 dB
//       above the clean level (a leaking jammer is never amplified).
//
//    5. SMOOTHING (crpa8)     cfg[19:16] = k > 0: the weights, the image taps and the
//       fine output gain move from the old to the newly committed values with a
//       time constant of 2^(k+7) samples instead of in one clock, so a new weight
//       set does not make a phase or gain step in the satellites the GNSS receiver
//       is tracking. k = 0 is the crpa7 behaviour, bit for bit.
//    6. FINE GAIN (crpa8)     gain[15:0], Q2.14, after the 6 dB make-up shift
//       (2 DSP48), so the firmware can hold the level into the receiver within
//       a fraction of a dB. 1.0 gives the crpa7 output bit for bit.
//
//  WEIGHT SCALE (v4.4)
//    The weight words hold w * 2^ws (ws = cfg[14:12], 0..5) and the sum of the
//    weighted taps is divided by 2^ws before it is subtracted from the
//    reference tap. A CW 60-70 dB above the noise has to be cancelled to 1e-4
//    or better, and the null is a minimum-norm spread of one spatial weight
//    over 16 taps (about 0.05 each), so a Q3.12 step (2.4e-4) leaves a residual
//    above the noise. With ws the same 16-bit words give 2^-12 / 2^ws steps.
//
//  WEIGHTS
//    The 33 weights are the SMI (closed-form) solution of the space-time
//    power-inversion problem, computed by the ARM from a 4096-sample DMA
//    capture of both channels (Source/Firmware/app_gnss_e310/gnss_aj_math.c)
//    a few hundred times a second while a jammer is present, and written to
//    gnss_passthrough, which hands weights, image taps, cfg and nrm to this
//    core all at once.
//
//  TIMING
//    II=1 at 100 MHz (sys_cpu_clk), i.e. up to 100 MS/s, beyond the AD9361's
//    61.44. No loop-carried state except the delay lines.
//
//  FIXED POINT
//    x1      : 12-bit signed, Q0
//    x2n     : 18-bit signed, Q2 (g2 x2, |g2| < 4)
//    weights : 16-bit signed Q3.12, complex multiply with 3 real multipliers
//              (k1 = wr (xr + xi), k2 = xr (wi - wr), k3 = xi (wr + wi);
//               re = k1 - k3, im = k1 + k2): crpa7 (10 + 10): 63 multipliers + 2 for the real g2 = 65 DSP48 (crpa8: + 2 for the fine gain = 67;
//               crpa9: + 4 image taps x 3 = 79);
//               (4.9, 32 taps: 195 + 4 for g2, SMI_NFAB = 12 RX1 taps (36 multipliers) built from LUTs)
//    sum     : 48 bits, Q14; the weighted taps are summed first, divided by
//              2^ws (weight scale, v4.4), then subtracted from the reference;
//              rounded to nearest after the make-up shift, saturated to 16 bits.
// ============================================================================
#include "smi_pi.h"

typedef ap_int<12> s12_t;
typedef ap_int<18> s18_t;            // normalised RX2, Q2
typedef ap_int<16> w16_t;
typedef ap_int<48> acc_t;            // Q14

static ap_int<16> sat16(acc_t v) {
#pragma HLS INLINE
    if (v > 32767)  return 32767;
    if (v < -32768) return -32768;
    return (ap_int<16>)v;
}

// Complex product, 3 multipliers. Result Q12 x (Q of x).
template <int XW>
static void cmul3(w16_t wr, w16_t wi, ap_int<XW> xr, ap_int<XW> xi,
                  ap_int<XW + 19> &pr, ap_int<XW + 19> &pi) {
#pragma HLS INLINE
    ap_int<XW + 1> xs = ap_int<XW + 1>(xr) + ap_int<XW + 1>(xi);
    ap_int<17> d1 = ap_int<17>(wi) - ap_int<17>(wr);
    ap_int<17> d2 = ap_int<17>(wr) + ap_int<17>(wi);
    ap_int<XW + 18> k1 = wr * xs;
    ap_int<XW + 18> k2 = xr * d1;
    ap_int<XW + 18> k3 = xi * d2;
    pr = ap_int<XW + 19>(k1) - ap_int<XW + 19>(k3);
    pi = ap_int<XW + 19>(k1) + ap_int<XW + 19>(k2);
}

// The same for an RX1 tap (12-bit data), with the three multipliers built from LUTs instead of DSP48
// (SMI_NFAB taps use this, see smi_pi.h). The pragmas are at function scope, next to the variables.
static void cmul3_fab(w16_t wr, w16_t wi, ap_int<12> xr, ap_int<12> xi,
                      ap_int<31> &pr, ap_int<31> &pi) {
#pragma HLS INLINE
    ap_int<13> xs = ap_int<13>(xr) + ap_int<13>(xi);
    ap_int<17> d1 = ap_int<17>(wi) - ap_int<17>(wr);
    ap_int<17> d2 = ap_int<17>(wr) + ap_int<17>(wi);
    ap_int<30> k1 = wr * xs;
    ap_int<30> k2 = xr * d1;
    ap_int<30> k3 = xi * d2;
#pragma HLS BIND_OP variable=k1 op=mul impl=fabric
#pragma HLS BIND_OP variable=k2 op=mul impl=fabric
#pragma HLS BIND_OP variable=k3 op=mul impl=fabric
    pr = ap_int<31>(k1) - ap_int<31>(k3);
    pi = ap_int<31>(k1) + ap_int<31>(k2);
}

// ---- crpa8: smoothing of weights and output gain ------------------------------------
// One running value per weight (and for the output gain) with SMI_RAMP_RF extra
// fractional bits. Every tick it moves 1/2^SMI_RAMP_RS of the way to the committed
// target: cur += round((target - cur) / 256). A tick is every 2^(k-1) samples
// (k = cfg[19:16], 1..15), so the time constant is 2^(k+7) samples; no barrel
// shifter, no multiplier. k = 0: cur = target at once (crpa7 behaviour, bit for bit).
// It settles within 2^(RS-1) / 2^RF = 0.25 LSB of the target, so the rounded value
// used by the multipliers ends exactly on the target.
typedef ap_int<16 + SMI_RAMP_RF + 1> rv_t;

static rv_t ramp_step(rv_t cur, w16_t target, bool snap, bool tick) {
#pragma HLS INLINE
    const rv_t t = rv_t(target) << SMI_RAMP_RF;
    if (snap) return t;
    if (!tick) return cur;
    const ap_int<16 + SMI_RAMP_RF + 2> d = ap_int<16 + SMI_RAMP_RF + 2>(t) - cur;
    return cur + rv_t((d + (1 << (SMI_RAMP_RS - 1))) >> SMI_RAMP_RS);
}

static w16_t ramp_val(rv_t cur) {
#pragma HLS INLINE
    return (w16_t)((cur + (1 << (SMI_RAMP_RF - 1))) >> SMI_RAMP_RF);
}

void smi_pi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<2048> wts,
            ap_uint<32 * 2 * SMI_NIMG> wcj,
            ap_uint<32>   cfg,
            ap_uint<32>   nrm,
            ap_uint<32>   gain) {
#pragma HLS INTERFACE axis port=s_axis_x
#pragma HLS INTERFACE axis port=m_axis_y
#pragma HLS INTERFACE ap_none port=wts
#pragma HLS INTERFACE ap_none port=wcj
#pragma HLS INTERFACE ap_none port=cfg
#pragma HLS INTERFACE ap_none port=nrm
#pragma HLS INTERFACE ap_none port=gain
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=1

    // Delay lines: index k holds the sample from k samples ago (0 = now).
    static s12_t h1r[SMI_NT], h1i[SMI_NT];
    static s18_t h2r[SMI_NT], h2i[SMI_NT];
#pragma HLS ARRAY_PARTITION variable=h1r complete
#pragma HLS ARRAY_PARTITION variable=h1i complete
#pragma HLS ARRAY_PARTITION variable=h2r complete
#pragma HLS ARRAY_PARTITION variable=h2i complete
#pragma HLS RESET variable=h1r
#pragma HLS RESET variable=h1i
#pragma HLS RESET variable=h2r
#pragma HLS RESET variable=h2i
    // crpa8: running weights (2*SMI_NT space-time + 2*SMI_NIMG image taps, crpa9), output gain
    static rv_t cwr[2 * SMI_NT + 2 * SMI_NIMG], cwi[2 * SMI_NT + 2 * SMI_NIMG];
#pragma HLS ARRAY_PARTITION variable=cwr complete
#pragma HLS ARRAY_PARTITION variable=cwi complete
#pragma HLS RESET variable=cwr
#pragma HLS RESET variable=cwi
    static rv_t        cgain = rv_t(SMI_GAIN_ONE) << SMI_RAMP_RF;
    static ap_uint<15> rcnt = 0;
    static ap_uint<3>  ws_prev = 0, sh_prev = 0;
#pragma HLS RESET variable=cgain
#pragma HLS RESET variable=rcnt
#pragma HLS RESET variable=ws_prev
#pragma HLS RESET variable=sh_prev

    if (s_axis_x.empty()) return;
    ap_uint<64> in = s_axis_x.read();
    s12_t x1r = (s12_t)in(11, 0),  x1i = (s12_t)in(27, 16);
    s12_t x2r = (s12_t)in(43, 32), x2i = (s12_t)in(59, 48);

    // ---- 1. input normalisation: x2n = g2 x2 (Q13 x Q0 -> Q2) ---------------
    // 4.9.13: g2 is real. The firmware learns it from the two noise floors (a
    // power ratio) and always wrote 0 into nrm[31:16], so the two g2i products
    // of the complex form were always zero: two multipliers (2 DSP48) saved,
    // x2n unchanged bit for bit.
    // |g2r x2r| <= 32768 * 2048 = 2^26, so after the shift by 11 the result is
    // within +-2^15: 18 bits always hold it, no saturation.
    ap_int<16> g2r = (ap_int<16>)nrm(15, 0);
    ap_int<32> nr = ap_int<32>(g2r * x2r);
    ap_int<32> ni = ap_int<32>(g2r * x2i);
    const ap_int<32> nhalf = ap_int<32>(1) << (SMI_GFRAC - SMI_XFRAC - 1);
    s18_t x2nr = (s18_t)((nr + nhalf) >> (SMI_GFRAC - SMI_XFRAC));
    s18_t x2ni = (s18_t)((ni + nhalf) >> (SMI_GFRAC - SMI_XFRAC));

    for (int k = SMI_NT - 1; k > 0; k--) {
#pragma HLS UNROLL
        h1r[k] = h1r[k - 1]; h1i[k] = h1i[k - 1];
        h2r[k] = h2r[k - 1]; h2i[k] = h2i[k - 1];
    }
    h1r[0] = x1r;  h1i[0] = x1i;
    h2r[0] = x2nr; h2i[0] = x2ni;

    // ---- crpa8: weight and gain smoothing -------------------------------------
    // Bypass is "all weights 0": with smoothing on, switching the filter in or out
    // fades as well; with k = 0 the weights are 0 at once and y = x1[n-R] exactly.
    const bool byp = cfg[0];
    ap_uint<3> ws = cfg(14, 12);
    if (ws > SMI_MAXWS) ws = SMI_MAXWS;
    ap_uint<3> s = cfg(10, 8);
    if (s > SMI_MAXSH) s = SMI_MAXSH;
    const ap_uint<4> rk = cfg(SMI_CFG_RAMP_LSB + 3, SMI_CFG_RAMP_LSB);
    const ap_uint<15> rmask = (rk == 0) ? ap_uint<15>(0) : ap_uint<15>((ap_uint<16>(1) << (rk - 1)) - 1);
    const bool tick = ((rcnt & rmask) == 0);
    rcnt++;
    const bool wsnap = (rk == 0) || (ws != ws_prev);    // weight words of another scale: no ramp across
    const bool gsnap = (rk == 0) || (s != sh_prev);     // 6 dB make-up step: the fine gain jumps with it
    ws_prev = ws; sh_prev = s;
    w16_t wtr[2 * SMI_NT + 2 * SMI_NIMG], wti[2 * SMI_NT + 2 * SMI_NIMG];
#pragma HLS ARRAY_PARTITION variable=wtr complete
#pragma HLS ARRAY_PARTITION variable=wti complete
    for (int j = 0; j < 2 * SMI_NT + 2 * SMI_NIMG; j++) {
#pragma HLS UNROLL
        w16_t tr, ti;
        if (j < 2 * SMI_NT) { tr = (w16_t)wts(32 * j + 15, 32 * j); ti = (w16_t)wts(32 * j + 31, 32 * j + 16); }
        else { const int c = j - 2 * SMI_NT; tr = (w16_t)wcj(32 * c + 15, 32 * c); ti = (w16_t)wcj(32 * c + 31, 32 * c + 16); }
        if (byp || j == SMI_REF) { tr = 0; ti = 0; }       // word SMI_REF (RX1 reference) is never used
        cwr[j] = ramp_step(cwr[j], tr, wsnap, tick);
        cwi[j] = ramp_step(cwi[j], ti, wsnap, tick);
        wtr[j] = ramp_val(cwr[j]);
        wti[j] = ramp_val(cwi[j]);
    }
    {
        // the gain is unsigned Q2.14 (0 .. 3.99994); keep it in a signed running value
        const rv_t t = rv_t(ap_uint<16>(gain(15, 0))) << SMI_RAMP_RF;
        if (gsnap) cgain = t;
        else if (tick) {
            const ap_int<16 + SMI_RAMP_RF + 2> d = ap_int<16 + SMI_RAMP_RF + 2>(t) - cgain;
            cgain = cgain + rv_t((d + (1 << (SMI_RAMP_RS - 1))) >> SMI_RAMP_RS);
        }
    }
    const ap_uint<16> g16 = (ap_uint<16>)((cgain + (1 << (SMI_RAMP_RF - 1))) >> SMI_RAMP_RF);

    // ---- 2. space-time filter, accumulated in Q14 ---------------------------
    // sr, si = sum of all weighted taps (weights are w * 2^ws); the reference
    // tap is added after the sum is scaled back by 2^ws.
    const int Q = SMI_WFRAC + SMI_XFRAC;                 // 14
    acc_t ref_r = acc_t(h1r[SMI_REF]) << Q;
    acc_t ref_i = acc_t(h1i[SMI_REF]) << Q;
    acc_t sr = 0, si = 0;
    for (int k = 0; k < SMI_NT; k++) {
#pragma HLS UNROLL
        if (k != SMI_REF) {
            ap_int<31> pr, pi;
            if (k >= SMI_NT - SMI_NFAB) cmul3_fab(wtr[k], wti[k], h1r[k], h1i[k], pr, pi);   // Q12, LUT multipliers (saves DSP48)
            else                        cmul3<12>(wtr[k], wti[k], h1r[k], h1i[k], pr, pi);   // Q12
            sr += acc_t(pr) << SMI_XFRAC;
            si += acc_t(pi) << SMI_XFRAC;
        }
        {
            ap_int<37> pr, pi;
            const int j = SMI_NT + k;
            cmul3<18>(wtr[j], wti[j], h2r[k], h2i[k], pr, pi);   // Q14
            sr += acc_t(pr);
            si += acc_t(pi);
        }
    }
    for (int c = 0; c < SMI_NIMG; c++) {
#pragma HLS UNROLL
        // image taps (crpa9: SMI_NIMG per channel at delays R-H .. R+H):
        // c1 conj(x1[n-a]) (Q12) and c2 conj(x2n[n-a]) (Q14)
        const int a = SMI_REF - SMI_IMGH + c;
        ap_int<32> pr1, pi1;
        ap_int<38> pr2, pi2;
        const int j1 = 2 * SMI_NT + c, j2 = 2 * SMI_NT + SMI_NIMG + c;
        cmul3<13>(wtr[j1], wti[j1], ap_int<13>(h1r[a]), ap_int<13>(-ap_int<13>(h1i[a])), pr1, pi1);
        cmul3<19>(wtr[j2], wti[j2], ap_int<19>(h2r[a]), ap_int<19>(-ap_int<19>(h2i[a])), pr2, pi2);
        sr += (acc_t(pr1) << SMI_XFRAC) + acc_t(pr2);
        si += (acc_t(pi1) << SMI_XFRAC) + acc_t(pi2);
    }
    const acc_t wrnd = (ws == 0) ? acc_t(0) : (acc_t(1) << (ws - 1));
    acc_t ar = ref_r - ((sr + wrnd) >> ws);
    acc_t ai = ref_i - ((si + wrnd) >> ws);

    // ---- 3. make-up shift, 4. fine gain (crpa8) and rounding --------------------
    // (bypass: all weights 0 above, so ar = ref exactly once they have faded out)
    const acc_t half = acc_t(1) << (Q - 1);
    ap_int<16> yr, yi;
    if (g16 == SMI_GAIN_ONE) {                // gain 1.0: the crpa7 rounding, bit for bit
        yr = sat16(((ar << s) + half) >> Q);
        yi = sat16(((ai << s) + half) >> Q);
    } else {                                  // 6 fractional bits kept, x gain (Q2.14), rounded
        const int UF = 6;
        const acc_t uh = acc_t(1) << (Q - UF - 1);
        acc_t ur = ((ar << s) + uh) >> (Q - UF), ui = ((ai << s) + uh) >> (Q - UF);
        const acc_t umax = (acc_t(1) << 23) - 1, umin = -(acc_t(1) << 23);
        if (ur > umax) ur = umax; if (ur < umin) ur = umin;
        if (ui > umax) ui = umax; if (ui < umin) ui = umin;
        const ap_int<41> pr = ap_int<24>(ur) * ap_int<17>(g16);
        const ap_int<41> pi = ap_int<24>(ui) * ap_int<17>(g16);
        const acc_t gh = acc_t(1) << (UF + SMI_GAIN_FRAC - 1);
        yr = sat16((acc_t(pr) + gh) >> (UF + SMI_GAIN_FRAC));
        yi = sat16((acc_t(pi) + gh) >> (UF + SMI_GAIN_FRAC));
    }
    ap_uint<32> out;
    out(15, 0)  = (ap_uint<16>)yr;
    out(31, 16) = (ap_uint<16>)yi;
    m_axis_y.write(out);
}
