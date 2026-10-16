// ============================================================================
//  pl_npi.cpp  --  pipelined PL-NPI power-inversion core (Vitis HLS)
//  ANTSDR E310 V1 / GNSS CRPA project
// ----------------------------------------------------------------------------
//  ALGORITHM (unchanged from the original RTL)
//    PL-NPI, Jia et al., "FPGA Implementation of Variable Step Power
//    Inversion Array for BeiDou Receiver", IEEE Access vol. 11, 2023, Eq. 11,
//    as implemented in Source/HDL/pl_npi_reference/rtl/
//    pi_power_inversion_pl_npi.v with M=2, DATA_W=16, WEIGHT_W=32,
//    WEIGHT_FRAC=20, LPF_SHIFT=18, Q_FRAC=32, ALPHA_GAIN_SHIFT=17,
//    GAIN_FRAC=16:
//
//      w      = wp + wo,  wo = [1, 0]                    (Q12.20)
//      s(n)   = sum_i x_i(n) * w_i                       (array output)
//      alpha  = floor(2^32 / (2 * sum_i |x_i(n)|^2 + gamma))
//      gain   = 1.20 / 1.10 / 1.05 / 1.00 for |Re s| > 32 / 16 / 8 / else
//      corr_i = conj(x_i) * s
//      term_i = (corr_i * alpha * gain) >> 31              (in Q.20 units)
//      wp_i   = sat32(wp_i - ((wp_i + term_i) >> 18))
//      y      = sat16(s >> 20)
//
//    Per sample: wp -= wp/2^18 + corr/(2(2 x^T x + gamma)), a leaky LMS. Where
//    2 x^T x << gamma its steady state is diagonally loaded power inversion,
//    w = L (R + L I)^-1 wo with L ~= gamma / 2^17 [LSB^2]: directions of the
//    input covariance R much stronger than L are nulled, weaker ones pass.
//
//  WHAT IS DIFFERENT, AND WHY
//    The original RTL computes a whole weight update in ONE clock: a 67x51-bit
//    multiply feeding 118-bit add/shift/saturate logic. On the xc7z020 at the
//    8 ns rx_clk constraint that failed by ~20 ns (WNS -19.998 ns). Three
//    changes, none of which alters the algorithm above:
//
//    1. NARROWER ARITHMETIC. The update discards its low 31 bits anyway, so:
//       - the correlation uses s at Q.8 instead of Q.20 (s >> 12, 25 bits);
//       - alpha*gain is kept as an 18-bit mantissa plus a power-of-two
//         exponent instead of a 50-bit integer;
//       - the update term and the leak are computed in 48 bits, not 118.
//       The big multiply becomes 42x18 bits (2 DSP48s) instead of 67x51.
//       The output path y = sat16(s >> 20) is exact, as before.
//
//    2. DELAYED UPDATE. The feedback loop
//         w -> s -> corr -> corr*alpha -> term -> w
//       is split by registers into five stages (A..E below), one stage per
//       sample, each holding at most one multiply. The update computed from
//       sample n therefore reaches the weights four samples later than in the
//       original ("delayed LMS"). Checked in simulation against the original
//       core: same null depth and convergence (see testbench.cpp).
//
//    3. SIDE CALCULATIONS REGISTERED. The Eq. 11 gain band comes from the
//       registered s of stage A; the four candidate alpha*gain mantissas are
//       computed from the current sample's alpha before the band selects one,
//       so selecting the gain is just a multiplexer inside the loop.
//
//    4. ALPHA FROM THE SAME SAMPLE AS THE CORRELATION. The original used the
//       latest output of a 33-stage reciprocal pipeline, i.e. alpha from ~16
//       samples EARLIER than the correlation it multiplies. When the input
//       power jumps (a jammer switching on after a quiet period) that stale
//       alpha is the noise-level one, up to ~1000x too large, and the weights
//       blow up: in simulation the original's output rose to 73-83 dB (above
//       the 56 dB jammer itself) for thousands of samples after each jammer
//       switch-on. Here alpha is computed per sample and delayed through the
//       pipeline (mkA/mkB) so it always belongs to the sample in stage C.
//
//  TIMING
//    One sample every two clocks (#pragma HLS PIPELINE II=2): the AD9361 in
//    2R2T mode delivers a sample pair at most every 2nd l_clk. build_hls.tcl
//    fails the build unless HLS reports II=2 and meets the clock target.
//
//    At II=2 every register in the weight-update loop gives the loop two
//    clocks. With only the five stage registers A..E the loop had 10 clocks
//    for two wide multiplies, sums, saturations and shifts; HLS then chained
//    a 32x16 multiply, two adds and a saturation into ONE clock (estimated
//    13.4 ns at an 8 ns target, measured on the 2023.2 build PC). NQ =
//    PL_NPI_EXTRA_DELAY pure delay registers (tQ, default 4) between stage D
//    and stage E raise the budget to 2*(5+NQ) = 18 clocks.
//
//    The cost is a delayed LMS: the update reaches the weights NQ samples
//    later. With the full step that oscillates (simulated: 93 dB output even
//    at NQ = 1), so the step is halved (PL_NPI_STEP_SHIFT = 1). C-simulation
//    with NQ = 4, shift 1 against the original (NQ = 0, shift 0):
//      jammer 600 LSB      null -50.7 dB (was -50.0), converged at 300 samples
//      jammer 2000 steady  residual 18-19 dB (was 19-20 dB)
//      jammer 3000 on/off every 3000 samples: settles within 3 blocks, then
//                          flat (same as the original)
// ============================================================================
#include "pl_npi.h"

typedef ap_int<16> smp_t;     // input sample, raw ADC scale
typedef ap_int<32> w_t;       // weight, Q12.20

// Eq. 11 gain constants, GAIN_FRAC = 16, as the original RTL computes them
// (integer division, i.e. rounded down).
// Pure delay registers between stage D and stage E (see "TIMING" in the
// header). Each one gives the weight-update loop two more clocks at II=2.
#ifndef PL_NPI_EXTRA_DELAY
#define PL_NPI_EXTRA_DELAY 4
#endif
static const int NQ = PL_NPI_EXTRA_DELAY;
// The step is alpha*gain / 2^PL_NPI_STEP_SHIFT. A delayed LMS is only stable
// if step x delay stays small, so the extra delay above needs a smaller step.
#ifndef PL_NPI_STEP_SHIFT
#define PL_NPI_STEP_SHIFT 1
#endif

static const ap_uint<17> GAIN_K[4] = {65536, 68812, 72089, 78643};

// |Re s| thresholds at the Q.8 scale of the stage-A register
// (8, 16, 32 sample units, the original THRESH1/2/3 = n << WEIGHT_FRAC).
static const ap_int<25> TH1 = 8  << 8;
static const ap_int<25> TH2 = 16 << 8;
static const ap_int<25> TH3 = 32 << 8;

static ap_int<16> sat16(ap_int<50> v) {
#pragma HLS INLINE
    if (v > 32767)  return 32767;
    if (v < -32768) return -32768;
    return (ap_int<16>)v;
}

static ap_int<25> sat25(ap_int<38> v) {
#pragma HLS INLINE
    const ap_int<38> mx = (ap_int<38>(1) << 24) - 1;
    const ap_int<38> mn = -(ap_int<38>(1) << 24);
    if (v > mx) return (ap_int<25>)mx;
    if (v < mn) return (ap_int<25>)mn;
    return (ap_int<25>)v;
}

static ap_int<48> sat48(ap_int<80> v) {
#pragma HLS INLINE
    const ap_int<80> mx = (ap_int<80>(1) << 47) - 1;
    const ap_int<80> mn = -(ap_int<80>(1) << 47);
    if (v > mx) return (ap_int<48>)mx;
    if (v < mn) return (ap_int<48>)mn;
    return (ap_int<48>)v;
}

static w_t sat32(ap_int<49> v) {
#pragma HLS INLINE
    const ap_int<49> mx = 2147483647;
    const ap_int<49> mn = -2147483647 - 1;
    if (v > mx) return (w_t)mx;
    if (v < mn) return (w_t)mn;
    return (w_t)v;
}

// Index of the highest set bit (0 when v == 0).
template <int W>
static ap_uint<7> msb_index(ap_uint<W> v) {
#pragma HLS INLINE
    ap_uint<7> p = 0;
    for (int i = 0; i < W; i++) {
#pragma HLS UNROLL
        if (v[i]) p = i;
    }
    return p;
}

void pl_npi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<32> gamma,
            ap_uint<1>  adapt_en,
            ap_uint<2> *gain_band)
{
#pragma HLS INTERFACE axis port=s_axis_x
#pragma HLS INTERFACE axis port=m_axis_y
#pragma HLS INTERFACE ap_none port=gamma
#pragma HLS INTERFACE ap_none port=adapt_en
#pragma HLS INTERFACE ap_none port=gain_band
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=2

    // ---- state: weights and the four pipeline registers ------------------
    static w_t wp_re[2] = {0, 0}, wp_im[2] = {0, 0};          // w - wo
#pragma HLS ARRAY_PARTITION variable=wp_re complete
#pragma HLS ARRAY_PARTITION variable=wp_im complete
#pragma HLS RESET variable=wp_re
#pragma HLS RESET variable=wp_im
    // stage A -> B : s at Q.8 and the sample it came from
    static ap_int<25> sA_re = 0, sA_im = 0;
#pragma HLS RESET variable=sA_re
#pragma HLS RESET variable=sA_im
    static smp_t xA_re[2] = {0, 0}, xA_im[2] = {0, 0};
#pragma HLS ARRAY_PARTITION variable=xA_re complete
#pragma HLS ARRAY_PARTITION variable=xA_im complete
#pragma HLS RESET variable=xA_re
#pragma HLS RESET variable=xA_im
    // stage B -> C : correlation conj(x)*s and the Eq. 11 band
    static ap_int<42> cB_re[2] = {0, 0}, cB_im[2] = {0, 0};
#pragma HLS ARRAY_PARTITION variable=cB_re complete
#pragma HLS ARRAY_PARTITION variable=cB_im complete
#pragma HLS RESET variable=cB_re
#pragma HLS RESET variable=cB_im
    static ap_uint<2> bandB = 0;
#pragma HLS RESET variable=bandB
    // stage C -> D : corr * mantissa(alpha*gain), and its shift
    static ap_int<61> pC_re[2] = {0, 0}, pC_im[2] = {0, 0};
#pragma HLS ARRAY_PARTITION variable=pC_re complete
#pragma HLS ARRAY_PARTITION variable=pC_im complete
#pragma HLS RESET variable=pC_re
#pragma HLS RESET variable=pC_im
    static ap_int<8> shC = 0;
#pragma HLS RESET variable=shC
    // alpha*gain candidates delayed to line up with the stage-C sample:
    // mkA/ekA hold sample n-1, mkB/ekB sample n-2 (= the sample in cB).
    static ap_uint<18> mkA[4] = {0, 0, 0, 0}, mkB[4] = {0, 0, 0, 0};
    static ap_int<8>   ekA[4] = {0, 0, 0, 0}, ekB[4] = {0, 0, 0, 0};
#pragma HLS ARRAY_PARTITION variable=mkA complete
#pragma HLS ARRAY_PARTITION variable=mkB complete
#pragma HLS ARRAY_PARTITION variable=ekA complete
#pragma HLS ARRAY_PARTITION variable=ekB complete
#pragma HLS RESET variable=mkA
#pragma HLS RESET variable=mkB
#pragma HLS RESET variable=ekA
#pragma HLS RESET variable=ekB
    // stage D -> E : update term in Q.20 weight units
    static ap_int<48> tD_re[2] = {0, 0}, tD_im[2] = {0, 0};
#pragma HLS ARRAY_PARTITION variable=tD_re complete
#pragma HLS ARRAY_PARTITION variable=tD_im complete
#pragma HLS RESET variable=tD_re
#pragma HLS RESET variable=tD_im
#if PL_NPI_EXTRA_DELAY > 0
    // tD delayed by NQ more samples before stage E uses it.
    static ap_int<48> tQ_re[NQ][2], tQ_im[NQ][2];
#pragma HLS ARRAY_PARTITION variable=tQ_re complete dim=0
#pragma HLS ARRAY_PARTITION variable=tQ_im complete dim=0
#pragma HLS RESET variable=tQ_re
#pragma HLS RESET variable=tQ_im
#define TE_RE(i) tQ_re[NQ - 1][i]
#define TE_IM(i) tQ_im[NQ - 1][i]
#else
#define TE_RE(i) tD_re[i]
#define TE_IM(i) tD_im[i]
#endif

    *gain_band = bandB;

    if (s_axis_x.empty()) return;
    ap_uint<64> in = s_axis_x.read();

    smp_t x_re[2], x_im[2];
#pragma HLS ARRAY_PARTITION variable=x_re complete
#pragma HLS ARRAY_PARTITION variable=x_im complete
    x_re[0] = (smp_t)in(15, 0);
    x_im[0] = (smp_t)in(31, 16);
    x_re[1] = (smp_t)in(47, 32);
    x_im[1] = (smp_t)in(63, 48);

    // ---- feed-forward: alpha = 2^32 / (2 x^T x + gamma) and the four
    //      candidate alpha*gain values as (18-bit mantissa, shift) ---------
    ap_uint<36> pw = 0;
    for (int i = 0; i < 2; i++) {
#pragma HLS UNROLL
        pw += (ap_uint<36>)(x_re[i] * x_re[i]) + (ap_uint<36>)(x_im[i] * x_im[i]);
    }
    ap_uint<32> g   = (gamma == 0) ? ap_uint<32>(1) : gamma;
    ap_uint<38> den = ((ap_uint<38>)pw << 1) + g;
    ap_uint<33> q   = (ap_uint<33>(1) << 32) / den;              // Q0.32

    // Normalise q to an 18-bit mantissa: q ~= qm * 2^(qp - 17).
    ap_uint<7>  qp = msb_index<33>(q);
    ap_uint<18> qm = (qp >= 17) ? (ap_uint<18>)(q >> (qp - 17))
                                : (ap_uint<18>)(q << (17 - qp));

    // alpha*gain_k ~= m_k * 2^(e_k): the product qm*G is 34 or 35 bits.
    ap_uint<18> mk[4];
    ap_int<8>   ek[4];
#pragma HLS ARRAY_PARTITION variable=mk complete
#pragma HLS ARRAY_PARTITION variable=ek complete
    for (int k = 0; k < 4; k++) {
#pragma HLS UNROLL
        ap_uint<35> pr = (ap_uint<35>)qm * GAIN_K[k];
        bool top = pr[34];
        mk[k] = top ? (ap_uint<18>)(pr >> 17) : (ap_uint<18>)(pr >> 16);
        // apl = q*G ~= mk * 2^(qp - 17 + (top ? 17 : 16))
        // term = corr_q8 * 2^12 * apl / 2^31  =>  shift = exponent - 19
        ek[k] = (ap_int<8>)qp - 17 + (top ? 17 : 16) - 19;
    }

    // ---- stage E: weight update from the term registered in stage D -----
    w_t nwp_re[2], nwp_im[2];
#pragma HLS ARRAY_PARTITION variable=nwp_re complete
#pragma HLS ARRAY_PARTITION variable=nwp_im complete
    for (int i = 0; i < 2; i++) {
#pragma HLS UNROLL
        ap_int<49> lr = ((ap_int<49>)wp_re[i] + TE_RE(i)) >> 18;
        ap_int<49> li = ((ap_int<49>)wp_im[i] + TE_IM(i)) >> 18;
        nwp_re[i] = adapt_en ? sat32((ap_int<49>)wp_re[i] - lr) : wp_re[i];
        nwp_im[i] = adapt_en ? sat32((ap_int<49>)wp_im[i] - li) : wp_im[i];
    }

    // ---- stage D: shift the stage-C product into Q.20 weight units -------
    ap_int<48> ntD_re[2], ntD_im[2];
#pragma HLS ARRAY_PARTITION variable=ntD_re complete
#pragma HLS ARRAY_PARTITION variable=ntD_im complete
    for (int i = 0; i < 2; i++) {
#pragma HLS UNROLL
        ap_int<80> vr = pC_re[i], vi = pC_im[i];
        if (shC >= 0) { vr = vr << shC;    vi = vi << shC; }
        else          { vr = vr >> (-shC); vi = vi >> (-shC); }
        ntD_re[i] = sat48(vr);
        ntD_im[i] = sat48(vi);
    }

    // ---- stage C: corr * mantissa of alpha*gain(band) --------------------
    ap_int<61> npC_re[2], npC_im[2];
#pragma HLS ARRAY_PARTITION variable=npC_re complete
#pragma HLS ARRAY_PARTITION variable=npC_im complete
    ap_uint<18> m_sel  = mkB[bandB];
    ap_int<8>   sh_sel = ekB[bandB] - PL_NPI_STEP_SHIFT;
    for (int i = 0; i < 2; i++) {
#pragma HLS UNROLL
        npC_re[i] = cB_re[i] * (ap_int<19>)m_sel;
        npC_im[i] = cB_im[i] * (ap_int<19>)m_sel;
    }

    // ---- stage B: corr = conj(x) * s, and the Eq. 11 band of |Re s| ------
    ap_int<42> ncB_re[2], ncB_im[2];
#pragma HLS ARRAY_PARTITION variable=ncB_re complete
#pragma HLS ARRAY_PARTITION variable=ncB_im complete
    for (int i = 0; i < 2; i++) {
#pragma HLS UNROLL
        ncB_re[i] = (ap_int<42>)(xA_re[i] * sA_re) + (ap_int<42>)(xA_im[i] * sA_im);
        ncB_im[i] = (ap_int<42>)(xA_re[i] * sA_im) - (ap_int<42>)(xA_im[i] * sA_re);
    }
    ap_int<25> yabs = (sA_re < 0) ? (ap_int<25>)(-sA_re) : sA_re;
    ap_uint<2> nband = (yabs > TH3) ? 3 : (yabs > TH2) ? 2 : (yabs > TH1) ? 1 : 0;

    // ---- stage A: s = sum x * (wp + wo) with the current weights ---------
    ap_int<50> s_re = 0, s_im = 0;
    for (int i = 0; i < 2; i++) {
#pragma HLS UNROLL
        w_t wr = wp_re[i] + ((i == 0) ? w_t(1 << 20) : w_t(0));
        w_t wi = wp_im[i];
        s_re += (ap_int<50>)(x_re[i] * wr) - (ap_int<50>)(x_im[i] * wi);
        s_im += (ap_int<50>)(x_re[i] * wi) + (ap_int<50>)(x_im[i] * wr);
    }

    ap_uint<32> out;
    out(15, 0)  = (ap_uint<16>)sat16(s_re >> 20);
    out(31, 16) = (ap_uint<16>)sat16(s_im >> 20);
    m_axis_y.write(out);

    // ---- commit all registers (every stage reads the OLD values above) ---
    for (int i = 0; i < 2; i++) {
#pragma HLS UNROLL
        wp_re[i] = nwp_re[i];  wp_im[i] = nwp_im[i];
#if PL_NPI_EXTRA_DELAY > 0
        for (int j = NQ - 1; j > 0; j--) {
#pragma HLS UNROLL
            tQ_re[j][i] = tQ_re[j - 1][i];  tQ_im[j][i] = tQ_im[j - 1][i];
        }
        tQ_re[0][i] = tD_re[i];  tQ_im[0][i] = tD_im[i];
#endif
        tD_re[i] = ntD_re[i];  tD_im[i] = ntD_im[i];
        pC_re[i] = npC_re[i];  pC_im[i] = npC_im[i];
        cB_re[i] = ncB_re[i];  cB_im[i] = ncB_im[i];
        xA_re[i] = x_re[i];    xA_im[i] = x_im[i];
    }
    for (int k = 0; k < 4; k++) {
#pragma HLS UNROLL
        mkB[k] = mkA[k];  ekB[k] = ekA[k];
        mkA[k] = mk[k];   ekA[k] = ek[k];
    }
    shC   = sh_sel;
    bandB = nband;
    sA_re = sat25(s_re >> 12);
    sA_im = sat25(s_im >> 12);
}
