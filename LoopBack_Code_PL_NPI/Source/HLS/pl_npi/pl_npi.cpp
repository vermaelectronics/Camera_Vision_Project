// ============================================================================
//  pl_npi.cpp  --  pipelined PL-NPI power-inversion core (Vitis HLS), v2.3
//  ANTSDR E310 V1 / GNSS CRPA project
// ----------------------------------------------------------------------------
//  v2.3 (this version) -- what is new, in pipeline order
//    1. INPUT NORMALISATION  x2n = g2 * x2 (nrm, Q3.13; firmware learns g2
//       from the two noise floors). The NLMS step is normalised by
//       |x1|^2 + |x2|^2, so an RX2 20 dB weaker than RX1 adapted its weight
//       100x slower than it should; scaled to RX1's level it converges at
//       the full rate, and its weight stays near 1.
//    2. IMAGE TAPS  y += c1 conj(x1) + c2 conj(x2n), adapted by the same rule
//       (their regressor is conj(x), so the gradient is x * s). They cancel
//       the AD9361's I/Q mirror image of a strong jammer (~50 dB down: at
//       the noise level against a 50 dB jammer), which no linear weight can.
//       They use a 4x smaller step (PL_NPI_IMG_SHIFT): their regressor
//       carries the jammer power twice and the delayed update must stay
//       stable.
//    3. GATED ADAPTATION  cfg[1:0] mode AUTO: a power detector (|x1|^2 +
//       |x2|^2 averaged over ~64 samples) switches the filter in and the
//       adaptation on as soon as the input rises above det_thr (set by the
//       firmware 6 dB above the noise floor), within ~1 us of a jammer
//       appearing, and back to BYPASS det_hold samples after it falls below
//       half of det_thr. In bypass TX1 = RX1 exactly and the weights are held
//       at 0, so with no jammer nothing is adapted and nothing is lost; each
//       jammer starts from w = 0. The firmware can also force BYPASS, ON (a
//       jammer weaker than the power threshold, found by RX1/RX2 coherence)
//       or HOLD.
//    4. OUTPUT MAKE-UP GAIN  y <<= s (cfg[10:8], 0..4) before the rounding,
//       while the filter is in: restores the level after a large RX gain
//       cut without losing precision. Ignored in bypass.
//    5. CLOCK  II=1 at 100 MHz on sys_cpu_clk, between two async AXI-Stream
//       FIFOs (it was II=2 on the AD9361 l_clk, which tied it to 30.72 MS/s):
//       up to 100 MS/s, enough for the 61.44 MS/s profile that carries
//       BeiDou B1I and all of GLONASS. At II=1 each register in the weight
//       loop gives one clock, so the loop has PL_NPI_EXTRA_DELAY = 8 extra
//       delay registers (budget 13 clocks) and a quarter step (shift 2) to
//       keep the longer delayed update stable.
//
//  v2.2.0 and earlier: as described below.
//  ALGORITHM (from the original RTL; v2.2.0 adds the w1 = 1 constraint, see
//  "CONSTRAINED POWER INVERSION" below)
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
//  CONSTRAINED POWER INVERSION (v2.2.0, PL_NPI_CONSTRAIN_W1 = 1)
//    The paper keeps RX1's weight near 1 only through the leak towards
//    wo = [1, 0]. Power minimisation pulls the whole weight vector towards 0,
//    and with a strong jammer it wins: the C model gave |w1| = 0.19 at
//    jammer 1500 LSB and 0.04 once the gain loop had cut the RX gain by
//    40 dB. The null direction stayed right (w2/w1 = -a), but the whole
//    output -- satellites included -- shrank below 1 LSB and was lost to the
//    output rounding and the 12-bit DAC (satellite gain -19 dB, output SINR
//    -25.7 dB against -18.6 dB for SMI-PI).
//    Now w1 is held at exactly 1 (Compton's power-inversion constraint: wp1
//    is never updated) and only w2 adapts; its leak pulls it towards 0, so
//    the jammer is nulled by w2 alone and everything else keeps RX1's level.
//    C model output SINR in dB, CW jammer held at ~1200 LSB by the gain loop,
//    noise and satellite reduced by the gain cut:
//
//      gain cut (jammer increase)   0     20     30     40     50 dB
//      unconstrained v2.1.10      -6.8   -7.8  -13.3  -25.7  -37.7
//      constrained   v2.2.0       -6.7   -7.1   -9.5  -16.6  -26.1
//      SMI-PI (rounded output)    -6.6   -6.9   -9.3  -16.4  -25.9
//
//    The remaining loss at very large cuts is the 12-bit ADC: the receiver
//    noise falls below the ADC's own quantisation noise (1/6 LSB^2 per
//    channel), which no weight can remove.
//    The weight-1 update logic is now unused, which also shortens the
//    timing-critical loop.
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
//  TIMING (v2.2.0; v2.3: II=1 at 10 ns, NQ = 8, see the top)
//    One sample every two clocks (pipeline II=2): the AD9361 in
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
//    at NQ = 1), so the step is reduced by 2^PL_NPI_STEP_SHIFT. v2.2.0 uses
//    shift 1 (half step) with the w1 constraint and a rounded output;
//    C-simulation, NQ = 4: jammer 600 LSB testbench null 44.8 dB below the
//    input (the noise is now passed, not cancelled), converged at 200
//    samples; a 1500 LSB jammer switched on is nulled within the first 1000
//    samples (~30 us), switched off the weight relaxes to 0 through the
//    2^-18 leak (~8.5 ms).
// ============================================================================
#include "pl_npi.h"

typedef ap_int<16> smp_t;     // input sample, raw ADC scale
typedef ap_int<19> reg_t;     // regressor, Q2 (raw x 4), conj-safe width
typedef ap_int<32> w_t;       // weight, Q12.20

// Pure delay registers between stage D and stage E (see "TIMING"). Each one
// gives the weight-update loop one more clock at II=1.
#ifndef PL_NPI_EXTRA_DELAY
#define PL_NPI_EXTRA_DELAY 8
#endif
static const int NQ = PL_NPI_EXTRA_DELAY;
#ifndef PL_NPI_CONSTRAIN_W1
#define PL_NPI_CONSTRAIN_W1 1      // 1 = w1 held at 1 (v2.2.0+), the only form in v2.3
#endif
#if !PL_NPI_CONSTRAIN_W1
#error "v2.3 implements the constrained form only (w1 = 1)"
#endif
// The step is alpha*gain / 2^PL_NPI_STEP_SHIFT. A delayed LMS is only stable
// if step x delay stays small, so the extra delay above needs a smaller step.
#ifndef PL_NPI_STEP_SHIFT
#define PL_NPI_STEP_SHIFT 2
#endif
// Extra step reduction of the image taps (their regressor block carries the
// jammer power twice: conj(x1) and conj(x2n) are coherent).
#ifndef PL_NPI_IMG_SHIFT
#define PL_NPI_IMG_SHIFT 2
#endif
#define PL_NPI_DET_SHIFT 6         // detector average over 2^6 samples
#define PL_NPI_DET_FRAC  8         // fraction bits of the detector average

// Eq. 11 gain constants, GAIN_FRAC = 16, as the original RTL computes them
// (integer division, i.e. rounded down).
static const ap_uint<17> GAIN_K[4] = {65536, 68812, 72089, 78643};

// |Re s| thresholds at the Q.8 scale of the stage-A register
// (8, 16, 32 sample units, the original THRESH1/2/3 = n << WEIGHT_FRAC).
static const ap_int<25> TH1 = 8  << 8;
static const ap_int<25> TH2 = 16 << 8;
static const ap_int<25> TH3 = 32 << 8;

static ap_int<16> sat16(ap_int<62> v) {
#pragma HLS INLINE
    if (v > 32767)  return 32767;
    if (v < -32768) return -32768;
    return (ap_int<16>)v;
}

static ap_int<25> sat25(ap_int<40> v) {
#pragma HLS INLINE
    const ap_int<40> mx = (ap_int<40>(1) << 24) - 1;
    const ap_int<40> mn = -(ap_int<40>(1) << 24);
    if (v > mx) return (ap_int<25>)mx;
    if (v < mn) return (ap_int<25>)mn;
    return (ap_int<25>)v;
}

static ap_int<48> sat48(ap_int<84> v) {
#pragma HLS INLINE
    const ap_int<84> mx = (ap_int<84>(1) << 47) - 1;
    const ap_int<84> mn = -(ap_int<84>(1) << 47);
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
            ap_uint<32>  gamma,
            ap_uint<1>   adapt_en,
            ap_uint<32>  cfg,
            ap_uint<32>  nrm,
            ap_uint<32>  det_thr,
            ap_uint<32>  det_hold,
            ap_uint<32>  *npi_st,
            ap_uint<192> *wout,
            ap_uint<32>  *det_pow)
{
#pragma HLS INTERFACE axis port=s_axis_x
#pragma HLS INTERFACE axis port=m_axis_y
#pragma HLS INTERFACE ap_none port=gamma
#pragma HLS INTERFACE ap_none port=adapt_en
#pragma HLS INTERFACE ap_none port=cfg
#pragma HLS INTERFACE ap_none port=nrm
#pragma HLS INTERFACE ap_none port=det_thr
#pragma HLS INTERFACE ap_none port=det_hold
#pragma HLS INTERFACE ap_none port=npi_st
#pragma HLS INTERFACE ap_none port=wout
#pragma HLS INTERFACE ap_none port=det_pow
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=1

    // ---- state: weights (w2, c1, c2) and the pipeline registers ----------
    static w_t wp_re[PL_NPI_NW] = {0, 0, 0}, wp_im[PL_NPI_NW] = {0, 0, 0};
#pragma HLS ARRAY_PARTITION variable=wp_re complete
#pragma HLS ARRAY_PARTITION variable=wp_im complete
#pragma HLS RESET variable=wp_re
#pragma HLS RESET variable=wp_im
    // stage A -> B : s at Q.8 and the regressors it came from
    static ap_int<25> sA_re = 0, sA_im = 0;
#pragma HLS RESET variable=sA_re
#pragma HLS RESET variable=sA_im
    static reg_t uA_re[PL_NPI_NW] = {0, 0, 0}, uA_im[PL_NPI_NW] = {0, 0, 0};
#pragma HLS ARRAY_PARTITION variable=uA_re complete
#pragma HLS ARRAY_PARTITION variable=uA_im complete
#pragma HLS RESET variable=uA_re
#pragma HLS RESET variable=uA_im
    // stage B -> C : correlation conj(u)*s (Q10) and the Eq. 11 band
    static ap_int<45> cB_re[PL_NPI_NW] = {0, 0, 0}, cB_im[PL_NPI_NW] = {0, 0, 0};
#pragma HLS ARRAY_PARTITION variable=cB_re complete
#pragma HLS ARRAY_PARTITION variable=cB_im complete
#pragma HLS RESET variable=cB_re
#pragma HLS RESET variable=cB_im
    static ap_uint<2> bandB = 0;
#pragma HLS RESET variable=bandB
    // stage C -> D : corr * mantissa(alpha*gain), and its shift
    static ap_int<64> pC_re[PL_NPI_NW] = {0, 0, 0}, pC_im[PL_NPI_NW] = {0, 0, 0};
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
    static ap_int<48> tD_re[PL_NPI_NW] = {0, 0, 0}, tD_im[PL_NPI_NW] = {0, 0, 0};
#pragma HLS ARRAY_PARTITION variable=tD_re complete
#pragma HLS ARRAY_PARTITION variable=tD_im complete
#pragma HLS RESET variable=tD_re
#pragma HLS RESET variable=tD_im
#if PL_NPI_EXTRA_DELAY > 0
    // tD delayed by NQ more samples before stage E uses it.
    static ap_int<48> tQ_re[NQ][PL_NPI_NW], tQ_im[NQ][PL_NPI_NW];
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
    // power detector
    static ap_uint<34> pacc = 0;              // average, PL_NPI_DET_FRAC fraction bits
    static ap_uint<1>  det = 0;
    static ap_uint<32> hold = 0;
    static ap_uint<16> det_cnt = 0;
#pragma HLS RESET variable=pacc
#pragma HLS RESET variable=det
#pragma HLS RESET variable=hold
#pragma HLS RESET variable=det_cnt

    // ---- mode ----------------------------------------------------------------
    ap_uint<2> mode = cfg(1, 0);
    bool img    = cfg[2];
    bool active = (mode == PL_NPI_MODE_ON) || (mode == PL_NPI_MODE_HOLD) ||
                  (mode == PL_NPI_MODE_AUTO && det == 1);
    bool adapt  = adapt_en && ((mode == PL_NPI_MODE_ON) ||
                               (mode == PL_NPI_MODE_AUTO && det == 1));
    ap_uint<3> sh = cfg(10, 8);
    if (sh > PL_NPI_MAXSH) sh = PL_NPI_MAXSH;

    {
        ap_uint<32> st = 0;
        st[0] = det;
        st[1] = active;
        st(3, 2) = bandB;
        st[4] = adapt;
        st(31, 16) = det_cnt;
        *npi_st = st;
        ap_uint<192> wo = 0;
        for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
            wo(64 * i + 31, 64 * i)      = (ap_uint<32>)wp_re[i];
            wo(64 * i + 63, 64 * i + 32) = (ap_uint<32>)wp_im[i];
        }
        *wout = wo;
        *det_pow = (ap_uint<32>)(pacc >> PL_NPI_DET_FRAC);
    }

    if (s_axis_x.empty()) return;
    ap_uint<64> in = s_axis_x.read();

    smp_t x1r = (smp_t)in(15, 0),  x1i = (smp_t)in(31, 16);
    smp_t x2r = (smp_t)in(47, 32), x2i = (smp_t)in(63, 48);

    // ---- 1. input normalisation: x2n = g2 x2 (Q13 x Q0 -> Q2) ---------------
    // |g2r x2r - g2i x2i| <= 2 * 32768 * 32768 = 2^31 for full-scale 16-bit
    // words (2^28 for the 12-bit samples): 34 bits hold it.
    ap_int<16> g2r = (ap_int<16>)nrm(15, 0), g2i = (ap_int<16>)nrm(31, 16);
    ap_int<34> nr = ap_int<34>(g2r * x2r) - ap_int<34>(g2i * x2i);
    ap_int<34> ni = ap_int<34>(g2r * x2i) + ap_int<34>(g2i * x2r);
    const ap_int<34> nhalf = ap_int<34>(1) << (PL_NPI_GFRAC - 2 - 1);
    reg_t x2nr = (reg_t)((nr + nhalf) >> (PL_NPI_GFRAC - 2));
    reg_t x2ni = (reg_t)((ni + nhalf) >> (PL_NPI_GFRAC - 2));
    reg_t x1qr = (reg_t)(ap_int<19>(x1r) << 2), x1qi = (reg_t)(ap_int<19>(x1i) << 2);

    // regressors (Q2): u0 = x2n, u1 = conj(x1), u2 = conj(x2n)
    reg_t u_re[PL_NPI_NW], u_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=u_re complete
#pragma HLS ARRAY_PARTITION variable=u_im complete
    u_re[0] = x2nr;  u_im[0] = x2ni;
    u_re[1] = x1qr;  u_im[1] = -x1qi;
    u_re[2] = x2nr;  u_im[2] = -x2ni;

    // ---- 3. power detector -----------------------------------------------------
    {
        ap_uint<25> pin = (ap_uint<25>)(x1r * x1r) + (ap_uint<25>)(x1i * x1i) +
                          (ap_uint<25>)(x2r * x2r) + (ap_uint<25>)(x2i * x2i);
        ap_int<36> d = (ap_int<36>)((ap_uint<34>)pin << PL_NPI_DET_FRAC) - (ap_int<36>)pacc;
        ap_uint<34> npacc = (ap_uint<34>)((ap_int<36>)pacc + (d >> PL_NPI_DET_SHIFT));
        ap_uint<32> pav = (ap_uint<32>)(pacc >> PL_NPI_DET_FRAC);
        ap_uint<1>  ndet = det;
        ap_uint<32> nhold = hold;
        if (pav > det_thr) {
            ndet = 1;
            nhold = det_hold;
        } else if (pav <= (det_thr >> 1)) {
            if (hold == 0) ndet = 0;
            else nhold = hold - 1;
        }
        if (ndet == 1 && det == 0) det_cnt = det_cnt + 1;
        det  = ndet;
        hold = nhold;
        pacc = npacc;
    }

    // ---- feed-forward: alpha = 2^32 / (2 (|x1|^2 + |x2n|^2) + gamma) and the
    //      four candidate alpha*gain values as (18-bit mantissa, shift) -------
    // |x2n|^2 at the raw scale: x2n is Q2, so its square is >> 4.
    ap_uint<36> pw = (ap_uint<36>)(x1r * x1r) + (ap_uint<36>)(x1i * x1i) +
                     (((ap_uint<40>)(x2nr * x2nr) + (ap_uint<40>)(x2ni * x2ni)) >> 4);
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
        // term = corr_q10 * 2^10 * apl / 2^31  =>  shift = exponent - 21
        ek[k] = (ap_int<8>)qp - 17 + (top ? 17 : 16) - 21;
    }

    // ---- stage E: weight update from the term registered in stage D -----
    w_t nwp_re[PL_NPI_NW], nwp_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=nwp_re complete
#pragma HLS ARRAY_PARTITION variable=nwp_im complete
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        ap_int<49> lr = ((ap_int<49>)wp_re[i] + TE_RE(i)) >> 18;
        ap_int<49> li = ((ap_int<49>)wp_im[i] + TE_IM(i)) >> 18;
        nwp_re[i] = adapt ? sat32((ap_int<49>)wp_re[i] - lr) : wp_re[i];
        nwp_im[i] = adapt ? sat32((ap_int<49>)wp_im[i] - li) : wp_im[i];
        // bypassed: weights held at 0 (each jammer starts from w = 0);
        // image taps off: c1 = c2 = 0
        if (!active || (i > 0 && !img)) { nwp_re[i] = 0; nwp_im[i] = 0; }
    }

    // ---- stage D: shift the stage-C product into Q.20 weight units -------
    ap_int<48> ntD_re[PL_NPI_NW], ntD_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=ntD_re complete
#pragma HLS ARRAY_PARTITION variable=ntD_im complete
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        ap_int<8> shi = shC - ((i > 0) ? PL_NPI_IMG_SHIFT : 0);
        ap_int<84> vr = pC_re[i], vi = pC_im[i];
        if (shi >= 0) { vr = vr << shi;    vi = vi << shi; }
        else          { vr = vr >> (-shi); vi = vi >> (-shi); }
        ntD_re[i] = sat48(vr);
        ntD_im[i] = sat48(vi);
    }

    // ---- stage C: corr * mantissa of alpha*gain(band) --------------------
    ap_int<64> npC_re[PL_NPI_NW], npC_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=npC_re complete
#pragma HLS ARRAY_PARTITION variable=npC_im complete
    ap_uint<18> m_sel  = mkB[bandB];
    ap_int<8>   sh_sel = ekB[bandB] - PL_NPI_STEP_SHIFT;
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        npC_re[i] = cB_re[i] * (ap_int<19>)m_sel;
        npC_im[i] = cB_im[i] * (ap_int<19>)m_sel;
    }

    // ---- stage B: corr = conj(u) * s, and the Eq. 11 band of |Re s| ------
    ap_int<45> ncB_re[PL_NPI_NW], ncB_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=ncB_re complete
#pragma HLS ARRAY_PARTITION variable=ncB_im complete
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        ncB_re[i] = (ap_int<45>)(uA_re[i] * sA_re) + (ap_int<45>)(uA_im[i] * sA_im);
        ncB_im[i] = (ap_int<45>)(uA_re[i] * sA_im) - (ap_int<45>)(uA_im[i] * sA_re);
    }
    ap_int<25> yabs = (sA_re < 0) ? (ap_int<25>)(-sA_re) : sA_re;
    ap_uint<2> nband = (yabs > TH3) ? 3 : (yabs > TH2) ? 2 : (yabs > TH1) ? 1 : 0;

    // ---- stage A: s = x1 + sum u * w with the current weights (Q22) ------
    ap_int<56> s_re = (ap_int<56>)x1qr << 20, s_im = (ap_int<56>)x1qi << 20;
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        s_re += (ap_int<56>)(u_re[i] * wp_re[i]) - (ap_int<56>)(u_im[i] * wp_im[i]);
        s_im += (ap_int<56>)(u_re[i] * wp_im[i]) + (ap_int<56>)(u_im[i] * wp_re[i]);
    }

    // ---- output: bypass = x1; filter = round(s * 2^sh / 2^22), saturated ---
    // Round to nearest, not floor (v2.1.10: truncation cost up to 3.7 dB of
    // output SINR once the gain loop had turned the RX gain down).
    ap_uint<32> out;
    if (active) {
        const ap_int<62> half = ap_int<62>(1) << 21;
        out(15, 0)  = (ap_uint<16>)sat16((((ap_int<62>)s_re << sh) + half) >> 22);
        out(31, 16) = (ap_uint<16>)sat16((((ap_int<62>)s_im << sh) + half) >> 22);
    } else {
        out(15, 0)  = (ap_uint<16>)x1r;
        out(31, 16) = (ap_uint<16>)x1i;
    }
    m_axis_y.write(out);

    // ---- commit all registers (every stage reads the OLD values above) ---
    for (int i = 0; i < PL_NPI_NW; i++) {
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
        uA_re[i] = u_re[i];    uA_im[i] = u_im[i];
    }
    for (int k = 0; k < 4; k++) {
#pragma HLS UNROLL
        mkB[k] = mkA[k];  ekB[k] = ekA[k];
        mkA[k] = mk[k];   ekA[k] = ek[k];
    }
    shC   = sh_sel;
    bandB = nband;
    sA_re = sat25(s_re >> 14);
    sA_im = sat25(s_im >> 14);
}
