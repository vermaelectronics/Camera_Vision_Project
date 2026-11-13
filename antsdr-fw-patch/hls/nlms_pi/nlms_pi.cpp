// ============================================================================
//  nlms_pi.cpp  --  space-time power inversion, weights from a hardware NLMS
//                   loop only (antsdr-fw-patch crpa13, nlms_pi 1.0)
//  ANTSDR E310 V1 / GNSS CRPA project, Vitis HLS 2023.2
//
//  This is the crpa12 space-time filter (smi_pi 3.1: 8 + 8 taps, 3 + 3 image
//  taps, RX2 normalisation, make-up shift, fine gain) with the SMI weight path
//  removed: no weight words from the ARM, no weight smoothing. The weights are
//  adapted by the delayed NLMS loop of crpa10 only, starting from zero.
//
//  PIPELINE (one sample pair per clock, sys_cpu_clk 100 MHz)
//    1. INPUT NORMALISATION   x2n = g2 * x2 (g2 real, Q3.13, from the firmware:
//       ratio of the two receivers' noise floors). x2n keeps 2 fractional bits.
//    2. SPACE-TIME FILTER     NL_NT taps on RX1, NL_NT on the normalised RX2,
//       NL_NIMG image taps on each (widely linear, AD9361 I/Q mirror):
//         y[n] = x1[n-R] - 2^-ws ( sum_{k != R} a_k x1[n-k] + sum_k b_k x2n[n-k]
//                        + sum_d c1_d conj(x1[n-R-d]) + sum_d c2_d conj(x2n[n-R-d]) )
//       R = NL_REF = 4, d = -1, 0, 1. The RX1 tap at delay R is the reference,
//       fixed at 1 (power inversion: the output can never be turned down as a
//       whole, only the parts the two antennas see coherently).
//    3. BYPASS                cfg[0] = 1: all weights cleared to 0, y = x1[n-R]
//       exactly (the same R-sample delay in both modes).
//    4. OUTPUT MAKE-UP GAIN   y <<= s, s = cfg[10:8] (0..7, 6 dB each), before
//       the rounding (the accumulator has 14 fractional bits).
//    5. FINE GAIN             gain[15:0], unsigned Q2.14, after the shift (2 DSP48).
//       1.0 gives the shifted output bit for bit. No smoothing (crpa13): the
//       firmware changes it in small steps.
//
//  ADAPTATION (the only one in crpa13)
//    Every sample, with nlms[31] = 1 and the filter in, each adaptive weight w_j
//    (21 of them: 7 RX1, 8 RX2, 6 image) moves by
//        w_j += (e[n-D] conj(u_j[n-D])) >> NL_CS,     e = y m 2^-sh
//    u_j = the sample tap j multiplies (for an image tap conj(x), so its conj(u)
//    is x), y = the filter output before the make-up gain (rounded to LSB,
//    18 bits), m = nlms[15:0], sh = nlms[20:16]. That is the gradient of |y|^2:
//    the loop drives the output power to its minimum, which, with the reference
//    tap fixed at 1, nulls whatever arrives at both antennas coherently (the
//    jammer) and leaves the satellites (below the noise) alone.
//    The ARM normalises the step: m 2^-sh = alpha 2^(12+ws+RF-1+CS) / (nw P_in),
//    P_in = RX1 power from the FPGA meter, alpha = gnss_aj_nlms_mu (aj_nlms_word
//    in gnss_aj_math.c). nlms[31] = 0 with the filter in holds the weights.
//
//    Delayed LMS: the update uses the error and the inputs of D = NL_D = 64
//    samples ago, from two LUTRAM circular buffers. The error of sample n is
//    known only at the end of the pipeline (filter, adder tree, error scaling:
//    about 20-30 clocks); with D = 64 the loop never needs it earlier, so the
//    pipeline runs at II = 1 with the HLS clock target met. crpa10-12 used
//    D = 24, too close to that latency, and the weight smoothing added a long
//    one-clock feedback (subtract, round, add, add, saturate): HLS reached
//    II = 3 at 41.7 ns there. Here the only one-clock feedback is the weight
//    state itself: one add and a saturation per weight.
//    Stability of delayed LMS needs alpha well below 1/D: the firmware default
//    is alpha = 1/128 (C simulation: scenarios 2-12).
//
//  WEIGHT SCALE ws = cfg[14:12]: the state holds w 2^ws, so a very strong jammer
//    (60-70 dB) gets 2^-12-ws weight steps. A change of ws clears the weights.
//
//  WEIGHT STATE: 16 + NL_RF + 1 = 33 bits per component (Q3.12 plus 16 more
//    fractional bits for the small NLMS steps), saturated at the Q3.12 range.
//    The filter uses the state rounded to Q3.12.
//
//  WEIGHT MONITOR: wmon (ap_vld) carries one rounded weight per sample, round
//    robin over the 22 words, with its index in [39:32]; gnss_passthrough keeps
//    the last value of each and the ARM reads them at the old weight addresses
//    (0x400.., 0x190, 0x194, 0x1C0..0x1CC).
//
//  NUMBERS: x1 12 bits; x2n Q2 in 18 bits; weights Q3.12 (16 bits); products
//    accumulated in Q14, 48 bits. 3-multiplier complex products (cmul3).
//    DSP48: filter 21 x 3 = 63, NLMS steps 21 x 3 = 63, g2 2, error 2, fine
//    gain 2: about 132.
// ============================================================================
#include "nlms_pi.h"

typedef ap_int<12> s12_t;
typedef ap_int<18> s18_t;            // normalised RX2, Q2
typedef ap_int<16> w16_t;
typedef ap_int<48> acc_t;            // Q14

typedef ap_int<16 + NL_RF + 1> rv_t;      // weight state, 33 bits
typedef ap_int<NL_EW> e_t;                // NLMS error e = y m 2^-sh
typedef ap_int<17>    z_t;                // NLMS input conj(u) (Q1)
typedef ap_int<16 + NL_RF + 22> dl_t;     // one NLMS step in weight-state units
typedef ap_int<16 + NL_RF + 3> dn_t;      // the step, saturated (35 bits)

static ap_int<16> sat16(acc_t v) {
#pragma HLS INLINE
    if (v > 32767)  return 32767;
    if (v < -32768) return -32768;
    return (ap_int<16>)v;
}

// (wr + j wi)(xr + j xi) with 3 multipliers:
//   k1 = wr (xr + xi), k2 = xr (wi - wr), k3 = xi (wr + wi)
//   re = k1 - k3, im = k1 + k2
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

// e z (complex, 3 multipliers), >> NL_CS, rounded
static void nlms_step(e_t er, e_t ei, z_t zr, z_t zi, dl_t &dr, dl_t &di) {
#pragma HLS INLINE
    const ap_int<NL_EW + 1> es = ap_int<NL_EW + 1>(er) + ap_int<NL_EW + 1>(ei);
    const ap_int<18> zd = ap_int<18>(zi) - ap_int<18>(zr), zs = ap_int<18>(zr) + ap_int<18>(zi);
    const ap_int<NL_EW + 19> k1 = zr * es;     // zr (er + ei)
    const ap_int<NL_EW + 19> k2 = er * zd;     // er (zi - zr)
    const ap_int<NL_EW + 19> k3 = ei * zs;     // ei (zr + zi)
    const ap_int<NL_EW + 20> pr = ap_int<NL_EW + 20>(k1) - k3;   // re(e z)
    const ap_int<NL_EW + 20> pi = ap_int<NL_EW + 20>(k1) + k2;   // im(e z)
    const ap_int<NL_EW + 20> h = ap_int<NL_EW + 20>(1) << (NL_CS - 1);
    dr = (dl_t)((pr + h) >> NL_CS);
    di = (dl_t)((pi + h) >> NL_CS);
}

static dn_t dl_sat(dl_t d) {
#pragma HLS INLINE
    const dl_t hi = (dl_t(1) << (16 + NL_RF + 2)) - 1, lo = -(dl_t(1) << (16 + NL_RF + 2));
    return (dn_t)(d > hi ? hi : d < lo ? lo : d);
}

// The one-clock feedback of the core: state + step, saturated at the Q3.12 range.
static rv_t w_update(rv_t cur, bool clr, bool nl, dn_t d) {
#pragma HLS INLINE
    if (clr) return 0;
    if (!nl) return cur;
    typedef ap_int<16 + NL_RF + 4> v_t;   // 36 bits
    v_t v = v_t(cur) + v_t(d);
    const v_t hi = v_t(32767) << NL_RF;
    const v_t lo = -(v_t(32768) << NL_RF);
    if (v > hi) v = hi;
    if (v < lo) v = lo;
    return (rv_t)v;
}

static w16_t w_round(rv_t cur) {
#pragma HLS INLINE
    return (w16_t)((cur + (1 << (NL_RF - 1))) >> NL_RF);
}

void nlms_pi(hls::stream<ap_uint<64> > &s_axis_x,
             hls::stream<ap_uint<32> > &m_axis_y,
             ap_uint<32>   cfg,
             ap_uint<32>   nrm,
             ap_uint<32>   gain,
             ap_uint<32>   nlms,
             ap_uint<40>   &wmon) {
#pragma HLS INTERFACE axis port=s_axis_x
#pragma HLS INTERFACE axis port=m_axis_y
#pragma HLS INTERFACE ap_none port=cfg
#pragma HLS INTERFACE ap_none port=nrm
#pragma HLS INTERFACE ap_none port=gain
#pragma HLS INTERFACE ap_none port=nlms
#pragma HLS INTERFACE ap_vld port=wmon
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=1

    // tap lines: x[n-k], k = 0..NL_NT-1, and the same delayed by NL_D for the update
    static s12_t h1r[NL_NT], h1i[NL_NT];
    static s18_t h2r[NL_NT], h2i[NL_NT];
    static s12_t d1r[NL_NT], d1i[NL_NT];
    static s18_t d2r[NL_NT], d2i[NL_NT];
#pragma HLS ARRAY_PARTITION variable=h1r complete
#pragma HLS ARRAY_PARTITION variable=h1i complete
#pragma HLS ARRAY_PARTITION variable=h2r complete
#pragma HLS ARRAY_PARTITION variable=h2i complete
#pragma HLS ARRAY_PARTITION variable=d1r complete
#pragma HLS ARRAY_PARTITION variable=d1i complete
#pragma HLS ARRAY_PARTITION variable=d2r complete
#pragma HLS ARRAY_PARTITION variable=d2i complete
#pragma HLS RESET variable=h1r
#pragma HLS RESET variable=h1i
#pragma HLS RESET variable=h2r
#pragma HLS RESET variable=h2i
#pragma HLS RESET variable=d1r
#pragma HLS RESET variable=d1i
#pragma HLS RESET variable=d2r
#pragma HLS RESET variable=d2i

    // NL_D-sample delay of the input (60 bits: x1 12 + 12, x2n 18 + 18) and the error
    // history: circular buffers in LUTRAM, written at wp, read at wp - NL_D.
    static ap_uint<60> xb[NL_HB];
    static e_t ehr[NL_HB], ehi[NL_HB];
#pragma HLS BIND_STORAGE variable=xb type=ram_s2p impl=lutram
#pragma HLS BIND_STORAGE variable=ehr type=ram_s2p impl=lutram
#pragma HLS BIND_STORAGE variable=ehi type=ram_s2p impl=lutram
#pragma HLS DEPENDENCE variable=xb type=inter direction=RAW distance=64 dependent=true
#pragma HLS DEPENDENCE variable=ehr type=inter direction=RAW distance=64 dependent=true
#pragma HLS DEPENDENCE variable=ehi type=inter direction=RAW distance=64 dependent=true
    static ap_uint<NL_HBB> wp = 0;
#pragma HLS RESET variable=wp

    // weight state
    static rv_t cwr[NL_NW], cwi[NL_NW];
#pragma HLS ARRAY_PARTITION variable=cwr complete
#pragma HLS ARRAY_PARTITION variable=cwi complete
#pragma HLS RESET variable=cwr
#pragma HLS RESET variable=cwi
    static ap_uint<3> ws_prev = 0;
    static ap_uint<5> mon = 0;
#pragma HLS RESET variable=ws_prev
#pragma HLS RESET variable=mon

    if (s_axis_x.empty()) return;
    ap_uint<64> in = s_axis_x.read();
    s12_t x1r = (s12_t)in(11, 0),  x1i = (s12_t)in(27, 16);
    s12_t x2r = (s12_t)in(43, 32), x2i = (s12_t)in(59, 48);

    // ---- 1. input normalisation: x2n = g2 x2 (Q13 x Q0 -> Q2) ---------------
    // |g2r x2r| <= 32768 * 2048 = 2^26, after the shift by 11 within +-2^15.
    ap_int<16> g2r = (ap_int<16>)nrm(15, 0);
    ap_int<32> nr = ap_int<32>(g2r * x2r);
    ap_int<32> ni = ap_int<32>(g2r * x2i);
    const ap_int<32> nhalf = ap_int<32>(1) << (NL_GFRAC - NL_XFRAC - 1);
    s18_t x2nr = (s18_t)((nr + nhalf) >> (NL_GFRAC - NL_XFRAC));
    s18_t x2ni = (s18_t)((ni + nhalf) >> (NL_GFRAC - NL_XFRAC));

    // ---- delayed input and error (NL_D samples ago) --------------------------
    const ap_uint<NL_HBB> rp = wp - NL_D;
    const ap_uint<60> xo = xb[rp];
    const e_t eor = ehr[rp], eoi = ehi[rp];
    {
        ap_uint<60> xn;
        xn(11, 0)  = (ap_uint<12>)x1r;  xn(23, 12) = (ap_uint<12>)x1i;
        xn(41, 24) = (ap_uint<18>)x2nr; xn(59, 42) = (ap_uint<18>)x2ni;
        xb[wp] = xn;
    }

    for (int k = NL_NT - 1; k > 0; k--) {
#pragma HLS UNROLL
        h1r[k] = h1r[k - 1]; h1i[k] = h1i[k - 1];
        h2r[k] = h2r[k - 1]; h2i[k] = h2i[k - 1];
        d1r[k] = d1r[k - 1]; d1i[k] = d1i[k - 1];
        d2r[k] = d2r[k - 1]; d2i[k] = d2i[k - 1];
    }
    h1r[0] = x1r;  h1i[0] = x1i;
    h2r[0] = x2nr; h2i[0] = x2ni;
    d1r[0] = (s12_t)xo(11, 0);  d1i[0] = (s12_t)xo(23, 12);
    d2r[0] = (s18_t)xo(41, 24); d2i[0] = (s18_t)xo(59, 42);

    // ---- control -------------------------------------------------------------
    const bool byp = cfg[0];
    ap_uint<3> ws = cfg(14, 12);
    if (ws > NL_MAXWS) ws = NL_MAXWS;
    ap_uint<3> s = cfg(10, 8);
    if (s > NL_MAXSH) s = NL_MAXSH;
    const bool clr = byp || (ws != ws_prev);     // bypass or a new weight scale: weights to 0
    ws_prev = ws;
    const bool nl = nlms[31] && !byp;
    const ap_uint<16> nm = nlms(15, 0);
    const ap_uint<5>  nsh = nlms(20, 16);

    // ---- NLMS weight update ----------------------------------------------------
    w16_t wtr[NL_NW], wti[NL_NW];
#pragma HLS ARRAY_PARTITION variable=wtr complete
#pragma HLS ARRAY_PARTITION variable=wti complete
    for (int j = 0; j < NL_NW; j++) {
#pragma HLS UNROLL
        if (j == NL_REF) {                       // the reference tap: fixed at 1, no word
            wtr[j] = 0; wti[j] = 0;
        } else {
            // conj(u) of this weight, NL_D samples ago (Q1); the image taps take conj(x),
            // so their conj(u) is x itself
            z_t zr, zi;
            if (j < NL_NT)          { zr = z_t(d1r[j]) << 1;          zi = -(z_t(d1i[j]) << 1); }
            else if (j < 2 * NL_NT) { zr = z_t(d2r[j - NL_NT] >> 1);  zi = -z_t(d2i[j - NL_NT] >> 1); }
            else if (j < 2 * NL_NT + NL_NIMG) {
                const int a = NL_REF - NL_IMGH + (j - 2 * NL_NT);
                zr = z_t(d1r[a]) << 1; zi = z_t(d1i[a]) << 1;
            } else {
                const int a = NL_REF - NL_IMGH + (j - 2 * NL_NT - NL_NIMG);
                zr = z_t(d2r[a] >> 1); zi = z_t(d2i[a] >> 1);
            }
            dl_t dr, di;
            nlms_step(eor, eoi, zr, zi, dr, di);
            cwr[j] = w_update(cwr[j], clr, nl, dl_sat(dr));
            cwi[j] = w_update(cwi[j], clr, nl, dl_sat(di));
            wtr[j] = w_round(cwr[j]);
            wti[j] = w_round(cwi[j]);
        }
    }

    // ---- weight monitor: one word per sample, round robin --------------------------
    {
        ap_uint<40> m;
        m(15, 0)  = (ap_uint<16>)wtr[mon];
        m(31, 16) = (ap_uint<16>)wti[mon];
        m(39, 32) = (ap_uint<8>)mon;
        wmon = m;
        mon = (mon == NL_NW - 1) ? ap_uint<5>(0) : ap_uint<5>(mon + 1);
    }

    // ---- 2. space-time filter, accumulated in Q14 ---------------------------
    const int Q = NL_WFRAC + NL_XFRAC;                   // 14
    acc_t ref_r = acc_t(h1r[NL_REF]) << Q;
    acc_t ref_i = acc_t(h1i[NL_REF]) << Q;
    acc_t sr = 0, si = 0;
    for (int k = 0; k < NL_NT; k++) {
#pragma HLS UNROLL
        if (k != NL_REF) {
            ap_int<31> pr, pi;
            cmul3<12>(wtr[k], wti[k], h1r[k], h1i[k], pr, pi);   // Q12
            sr += acc_t(pr) << NL_XFRAC;
            si += acc_t(pi) << NL_XFRAC;
        }
        {
            ap_int<37> pr, pi;
            const int j = NL_NT + k;
            cmul3<18>(wtr[j], wti[j], h2r[k], h2i[k], pr, pi);   // Q14
            sr += acc_t(pr);
            si += acc_t(pi);
        }
    }
    for (int c = 0; c < NL_NIMG; c++) {
#pragma HLS UNROLL
        // image taps: c1 conj(x1[n-a]) (Q12) and c2 conj(x2n[n-a]) (Q14)
        const int a = NL_REF - NL_IMGH + c;
        ap_int<32> pr1, pi1;
        ap_int<38> pr2, pi2;
        const int j1 = 2 * NL_NT + c, j2 = 2 * NL_NT + NL_NIMG + c;
        cmul3<13>(wtr[j1], wti[j1], ap_int<13>(h1r[a]), ap_int<13>(-ap_int<13>(h1i[a])), pr1, pi1);
        cmul3<19>(wtr[j2], wti[j2], ap_int<19>(h2r[a]), ap_int<19>(-ap_int<19>(h2i[a])), pr2, pi2);
        sr += (acc_t(pr1) << NL_XFRAC) + acc_t(pr2);
        si += (acc_t(pi1) << NL_XFRAC) + acc_t(pi2);
    }
    const acc_t wrnd = (ws == 0) ? acc_t(0) : (acc_t(1) << (ws - 1));
    acc_t ar = ref_r - ((sr + wrnd) >> ws);
    acc_t ai = ref_i - ((si + wrnd) >> ws);

    // ---- NLMS error of this sample: e = y m 2^-sh (y rounded to LSB, 18 bits), into the
    // history for the update NL_D samples later
    {
        const acc_t qh = acc_t(1) << (Q - 1);
        acc_t yr = (ar + qh) >> Q, yi = (ai + qh) >> Q;
        if (yr > 131071) yr = 131071;
        if (yr < -131072) yr = -131072;
        if (yi > 131071) yi = 131071;
        if (yi < -131072) yi = -131072;
        const ap_int<36> pr = ap_int<18>(yr) * ap_int<17>(nm), pi = ap_int<18>(yi) * ap_int<17>(nm);
        const ap_int<36> rh = (nsh == 0) ? ap_int<36>(0) : ap_int<36>(ap_int<36>(1) << (nsh - 1));
        ap_int<36> er = (pr + rh) >> nsh, ei = (pi + rh) >> nsh;
        const ap_int<36> emax = (ap_int<36>(1) << (NL_EW - 1)) - 1, emin = -(ap_int<36>(1) << (NL_EW - 1));
        if (er > emax) er = emax;
        if (er < emin) er = emin;
        if (ei > emax) ei = emax;
        if (ei < emin) ei = emin;
        ehr[wp] = (e_t)er; ehi[wp] = (e_t)ei;
    }
    wp++;

    // ---- 4. make-up shift, 5. fine gain and rounding ----------------------------
    const ap_uint<16> g16 = gain(15, 0);
    const acc_t half = acc_t(1) << (Q - 1);
    ap_int<16> yr, yi;
    if (g16 == NL_GAIN_ONE) {                 // gain 1.0: plain shift and rounding
        yr = sat16(((ar << s) + half) >> Q);
        yi = sat16(((ai << s) + half) >> Q);
    } else {                                  // 6 fractional bits kept, x gain (Q2.14), rounded
        const int UF = 6;
        const acc_t uh = acc_t(1) << (Q - UF - 1);
        acc_t ur = ((ar << s) + uh) >> (Q - UF), ui = ((ai << s) + uh) >> (Q - UF);
        const acc_t umax = (acc_t(1) << 23) - 1, umin = -(acc_t(1) << 23);
        if (ur > umax) ur = umax;
        if (ur < umin) ur = umin;
        if (ui > umax) ui = umax;
        if (ui < umin) ui = umin;
        const ap_int<41> pr = ap_int<24>(ur) * ap_int<17>(g16);
        const ap_int<41> pi = ap_int<24>(ui) * ap_int<17>(g16);
        const acc_t gh = acc_t(1) << (UF + NL_GAIN_FRAC - 1);
        yr = sat16((acc_t(pr) + gh) >> (UF + NL_GAIN_FRAC));
        yi = sat16((acc_t(pi) + gh) >> (UF + NL_GAIN_FRAC));
    }
    ap_uint<32> out;
    out(15, 0)  = (ap_uint<16>)yr;
    out(31, 16) = (ap_uint<16>)yi;
    m_axis_y.write(out);
}
