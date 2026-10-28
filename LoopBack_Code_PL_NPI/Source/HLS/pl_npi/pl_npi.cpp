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
//       BeiDou B1I and all of GLONASS. At II=1 the logic between two
//       registers of the weight loop must fit in one 10 ns clock, and HLS
//       cannot move registers written as static variables. v2.3.1 splits the
//       loop into 9 processing registers (A1 A2 A3 B1 B2 C D1 D2 E1) plus
//       PL_NPI_EXTRA_DELAY = 3 pure delays: 12 samples of update delay, as
//       in v2.3.0 (which had 4 + 8 and missed timing: 19.6 ns estimated for
//       u*w + sum + saturation in one clock). A quarter step (shift 2) keeps
//       the delayed update stable. Output latency: PL_NPI_LATENCY = 2
//       samples, the same for filter and bypass.
//
//  v2.4 (this version) -- added around the unchanged PL-NPI pipeline
//    6. DC BLOCKER on both inputs (cfg[3]): removes the AD9361 DC offset /
//       LO leakage before everything else, so it is neither retransmitted
//       (a carrier on the LO frequency into the GNSS receiver) nor seen as a
//       signal common to RX1 and RX2.
//    7. CW NOTCHES (PL_NPI_NOTCHES = 2, cfg[13:12], coefficients ncoef from
//       the firmware's FFT detector) on the output. A CW jammer at or below
//       the wideband noise -- J/S 30-40 dB against GPS C/A -- costs the
//       receiver 6-13 dB of C/N0 (C simulation, testbench scenario 12), and
//       spatial power inversion cannot null it (it minimises the TOTAL
//       power, which such a CW hardly raises). A 62 kHz notch removes it.
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
//  TIMING (v2.2.0; v2.3.1: II=1 at 10 ns, 9 stages + NQ = 3, see the top)
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

// Pure delay registers between stage D2 and stage E1 (see "TIMING"). The
// weight-update delay is 9 + PL_NPI_EXTRA_DELAY samples (12 by default, the
// value the C simulation validated).
#ifndef PL_NPI_EXTRA_DELAY
#define PL_NPI_EXTRA_DELAY 3
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

#ifdef PL_NPI_DEBUG
int pl_npi_dbg[4];
#endif
#if PL_NPI_NOTCHES > 0
// One CW notch (see "CW notches" below). c = {p^2, p, q} words, 3 x 32 bits.
// The state lives in the caller's registers; en = 0 passes x and clears them.
static inline void notch_step(bool en, ap_uint<96> c,
                              ap_int<17> &xr, ap_int<17> &xi,
                              ap_int<25> &ns_re, ap_int<25> &ns_im,
                              ap_int<25> &nm_re, ap_int<25> &nm_im,
                              ap_int<17> &nx_re, ap_int<17> &nx_im)
{
#pragma HLS INLINE
    const ap_int<42> nhalf = ap_int<42>(1) << (PL_NPI_NFRAC - 1);
    const ap_int<42> qhalf = ap_int<42>(1) << (PL_NPI_QFRAC - 1);
    ap_int<16> qr = (ap_int<16>)c(15, 0),  qi = (ap_int<16>)c(31, 16);
    ap_int<16> pr = (ap_int<16>)c(47, 32), pi = (ap_int<16>)c(63, 48);
    ap_int<16> p2r = (ap_int<16>)c(79, 64), p2i = (ap_int<16>)c(95, 80);
    // u = x + p x[n-1]
    const ap_int<42> uhalf = ap_int<42>(1) << (PL_NPI_NFRAC - 2);
    ap_int<42> upr = (ap_int<42>)(pr * nx_re) - (ap_int<42>)(pi * nx_im) + uhalf;
    ap_int<42> upi = (ap_int<42>)(pr * nx_im) + (ap_int<42>)(pi * nx_re) + uhalf;
    ap_int<28> ur = ((ap_int<28>)xr << 1) + (ap_int<28>)(upr >> (PL_NPI_NFRAC - 1));
    ap_int<28> ui = ((ap_int<28>)xi << 1) + (ap_int<28>)(upi >> (PL_NPI_NFRAC - 1));
    // s = u + p^2 s[n-2], saturated to 25 bits
    ap_int<29> sr = (ap_int<29>)ur + nm_re, si = (ap_int<29>)ui + nm_im;
    const ap_int<29> smax = (ap_int<29>(1) << 24) - 1, smin = -(ap_int<29>(1) << 24);
    ap_int<25> s25r = (sr > smax) ? (ap_int<25>)smax : (sr < smin) ? (ap_int<25>)smin : (ap_int<25>)sr;
    ap_int<25> s25i = (si > smax) ? (ap_int<25>)smax : (si < smin) ? (ap_int<25>)smin : (ap_int<25>)si;
    // y = s - q s[n-1]
    ap_int<42> yqr = (ap_int<42>)(qr * ns_re) - (ap_int<42>)(qi * ns_im) + qhalf;
    ap_int<42> yqi = (ap_int<42>)(qr * ns_im) + (ap_int<42>)(qi * ns_re) + qhalf;
    ap_int<28> y2r = (ap_int<28>)s25r - (ap_int<28>)(yqr >> PL_NPI_QFRAC);   // x 2
    ap_int<28> y2i = (ap_int<28>)s25i - (ap_int<28>)(yqi >> PL_NPI_QFRAC);
    ap_int<28> yr = (y2r + 1) >> 1, yi = (y2i + 1) >> 1;
    // next p^2 s[n-1] from the OLD s[n-1]: |p^2 s| <= |s|, no saturation
    ap_int<42> mr = (ap_int<42>)(p2r * ns_re) - (ap_int<42>)(p2i * ns_im) + nhalf;
    ap_int<42> mi = (ap_int<42>)(p2r * ns_im) + (ap_int<42>)(p2i * ns_re) + nhalf;
    nm_re = en ? (ap_int<25>)(mr >> PL_NPI_NFRAC) : ap_int<25>(0);
    nm_im = en ? (ap_int<25>)(mi >> PL_NPI_NFRAC) : ap_int<25>(0);
    ns_re = en ? s25r : ap_int<25>(0);
    ns_im = en ? s25i : ap_int<25>(0);
    nx_re = en ? xr : ap_int<17>(0);
    nx_im = en ? xi : ap_int<17>(0);
    if (en) {
        xr = (yr > 32767) ? ap_int<17>(32767) : (yr < -32768) ? ap_int<17>(-32768) : (ap_int<17>)yr;
        xi = (yi > 32767) ? ap_int<17>(32767) : (yi < -32768) ? ap_int<17>(-32768) : (ap_int<17>)yi;
    }
}
#endif

void pl_npi(hls::stream<ap_uint<64> > &s_axis_x,
            hls::stream<ap_uint<32> > &m_axis_y,
            ap_uint<32>  gamma,
            ap_uint<1>   adapt_en,
            ap_uint<32>  cfg,
            ap_uint<32>  nrm,
            ap_uint<32>  det_thr,
            ap_uint<32>  det_hold,
            ap_uint<192> ncoef,
            ap_uint<96>  eqcoef,
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
#pragma HLS INTERFACE ap_none port=ncoef
#pragma HLS INTERFACE ap_none port=eqcoef
#pragma HLS INTERFACE ap_none port=npi_st
#pragma HLS INTERFACE ap_none port=wout
#pragma HLS INTERFACE ap_none port=det_pow
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=1
    // ---- state ---------------------------------------------------------------
    // Every static below is a register. At II=1 the logic between two of them
    // must fit in ONE clock: HLS cannot move them (v2.3.0 had the three u*w
    // products, the 7-term sum and the saturation between wp and sA: 19.6 ns
    // estimated). The weight loop is now
    //   wp -> A1 pa -> A2 t0/t1 -> A3 sA -> B1 pb -> B2 cB -> C pC -> D1 vD
    //      -> D2 tD -> tQ[NQ] -> E1 dl -> E2 wp
    // and each arrow is at most one wide multiply, two adds, or one add +
    // saturation. Values that only depend on the input sample (regressors u,
    // x1, alpha*gain) travel beside it in the *A1/*S/*B registers, so every
    // stage works on one sample. 9 + NQ registers: NQ = 3 gives the same
    // 12-sample update delay as v2.3.0 (4 stages + NQ = 8), which the C
    // simulation validated.
    static w_t wp_re[PL_NPI_NW] = {0, 0, 0}, wp_im[PL_NPI_NW] = {0, 0, 0};
    static w_t wd_re[PL_NPI_NW] = {0, 0, 0}, wd_im[PL_NPI_NW] = {0, 0, 0};  // wp one sample ago
#pragma HLS ARRAY_PARTITION variable=wp_re complete
#pragma HLS ARRAY_PARTITION variable=wp_im complete
#pragma HLS ARRAY_PARTITION variable=wd_re complete
#pragma HLS ARRAY_PARTITION variable=wd_im complete
#pragma HLS RESET variable=wp_re
#pragma HLS RESET variable=wp_im
#pragma HLS RESET variable=wd_re
#pragma HLS RESET variable=wd_im
    // A1: the four real products of u * w for each weight
    static ap_int<51> pa_rr[PL_NPI_NW], pa_ii[PL_NPI_NW], pa_ri[PL_NPI_NW], pa_ir[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=pa_rr complete
#pragma HLS ARRAY_PARTITION variable=pa_ii complete
#pragma HLS ARRAY_PARTITION variable=pa_ri complete
#pragma HLS ARRAY_PARTITION variable=pa_ir complete
#pragma HLS RESET variable=pa_rr
#pragma HLS RESET variable=pa_ii
#pragma HLS RESET variable=pa_ri
#pragma HLS RESET variable=pa_ir
    // A2: two partial sums of s
    static ap_int<56> t0_re = 0, t0_im = 0, t1_re = 0, t1_im = 0;
#pragma HLS RESET variable=t0_re
#pragma HLS RESET variable=t0_im
#pragma HLS RESET variable=t1_re
#pragma HLS RESET variable=t1_im
    // A3: s at Q.8
    static ap_int<25> sA_re = 0, sA_im = 0;
#pragma HLS RESET variable=sA_re
#pragma HLS RESET variable=sA_im
    // B1: the four real products of conj(u) * s, and the Eq. 11 band
    static ap_int<44> pb_rr[PL_NPI_NW], pb_ii[PL_NPI_NW], pb_ri[PL_NPI_NW], pb_ir[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=pb_rr complete
#pragma HLS ARRAY_PARTITION variable=pb_ii complete
#pragma HLS ARRAY_PARTITION variable=pb_ri complete
#pragma HLS ARRAY_PARTITION variable=pb_ir complete
#pragma HLS RESET variable=pb_rr
#pragma HLS RESET variable=pb_ii
#pragma HLS RESET variable=pb_ri
#pragma HLS RESET variable=pb_ir
    static ap_uint<2> band1 = 0;
#pragma HLS RESET variable=band1
    // B2: correlation (Q10), the selected alpha*gain mantissa and shift
    static ap_int<45> cB_re[PL_NPI_NW] = {0, 0, 0}, cB_im[PL_NPI_NW] = {0, 0, 0};
#pragma HLS ARRAY_PARTITION variable=cB_re complete
#pragma HLS ARRAY_PARTITION variable=cB_im complete
#pragma HLS RESET variable=cB_re
#pragma HLS RESET variable=cB_im
    static ap_uint<2>  bandB = 0;
    static ap_uint<18> mselB = 0;
    static ap_int<8>   shB = 0;
#pragma HLS RESET variable=bandB
#pragma HLS RESET variable=mselB
#pragma HLS RESET variable=shB
    // C: corr * mantissa
    static ap_int<64> pC_re[PL_NPI_NW] = {0, 0, 0}, pC_im[PL_NPI_NW] = {0, 0, 0};
#pragma HLS ARRAY_PARTITION variable=pC_re complete
#pragma HLS ARRAY_PARTITION variable=pC_im complete
#pragma HLS RESET variable=pC_re
#pragma HLS RESET variable=pC_im
    static ap_int<8> shC = 0, shC_img = 0;     // shift for w2, for c1/c2
#pragma HLS RESET variable=shC
#pragma HLS RESET variable=shC_img
    // D1: shifted into Q.20 weight units; D2: saturated
    static ap_int<84> vD_re[PL_NPI_NW], vD_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=vD_re complete
#pragma HLS ARRAY_PARTITION variable=vD_im complete
#pragma HLS RESET variable=vD_re
#pragma HLS RESET variable=vD_im
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
    // E1: the update incl. the 2^-18 leak, in weight LSBs
    static ap_int<49> dl_re[PL_NPI_NW] = {0, 0, 0}, dl_im[PL_NPI_NW] = {0, 0, 0};
#pragma HLS ARRAY_PARTITION variable=dl_re complete
#pragma HLS ARRAY_PARTITION variable=dl_im complete
#pragma HLS RESET variable=dl_re
#pragma HLS RESET variable=dl_im
    // per-sample values travelling beside the sample
    static reg_t uA1_re[PL_NPI_NW], uA1_im[PL_NPI_NW], uS_re[PL_NPI_NW], uS_im[PL_NPI_NW];
    static reg_t uB_re[PL_NPI_NW], uB_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=uA1_re complete
#pragma HLS ARRAY_PARTITION variable=uA1_im complete
#pragma HLS ARRAY_PARTITION variable=uS_re complete
#pragma HLS ARRAY_PARTITION variable=uS_im complete
#pragma HLS ARRAY_PARTITION variable=uB_re complete
#pragma HLS ARRAY_PARTITION variable=uB_im complete
#pragma HLS RESET variable=uA1_re
#pragma HLS RESET variable=uA1_im
#pragma HLS RESET variable=uS_re
#pragma HLS RESET variable=uS_im
#pragma HLS RESET variable=uB_re
#pragma HLS RESET variable=uB_im
    static reg_t x1qA_re = 0, x1qA_im = 0;                          // x1 at Q2, for the sum
    static smp_t x1A_re = 0, x1A_im = 0, x1S_re = 0, x1S_im = 0;    // raw x1, for bypass
#pragma HLS RESET variable=x1qA_re
#pragma HLS RESET variable=x1qA_im
#pragma HLS RESET variable=x1A_re
#pragma HLS RESET variable=x1A_im
#pragma HLS RESET variable=x1S_re
#pragma HLS RESET variable=x1S_im
    static ap_uint<18> mkA1[4], mkS[4], mkB0[4], mkB1[4];
    static ap_int<8>   ekA1[4], ekS[4], ekB0[4], ekB1[4];
#pragma HLS ARRAY_PARTITION variable=mkA1 complete
#pragma HLS ARRAY_PARTITION variable=mkS complete
#pragma HLS ARRAY_PARTITION variable=mkB0 complete
#pragma HLS ARRAY_PARTITION variable=mkB1 complete
#pragma HLS ARRAY_PARTITION variable=ekA1 complete
#pragma HLS ARRAY_PARTITION variable=ekS complete
#pragma HLS ARRAY_PARTITION variable=ekB0 complete
#pragma HLS ARRAY_PARTITION variable=ekB1 complete
#pragma HLS RESET variable=mkA1
#pragma HLS RESET variable=mkS
#pragma HLS RESET variable=mkB0
#pragma HLS RESET variable=mkB1
#pragma HLS RESET variable=ekA1
#pragma HLS RESET variable=ekS
#pragma HLS RESET variable=ekB0
#pragma HLS RESET variable=ekB1
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
        st(7, 5) = PL_NPI_NOTCHES;
        st[8] = cfg[3];
        st[9] = (PL_NPI_V25 && PL_NPI_EQ) ? (bool)cfg[14] : false;
        st[10] = (PL_NPI_V25 && PL_NPI_NOTCH_IN && PL_NPI_NOTCHES > 0);
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

    smp_t r1r = (smp_t)in(15, 0),  r1i = (smp_t)in(31, 16);
    smp_t r2r = (smp_t)in(47, 32), r2i = (smp_t)in(63, 48);

    // ---- 0. DC blocker (v2.4): x - DC, DC from a first-order average,
    //      corner fs * 2^-14 / (2 pi) (490 Hz at 50 MS/s). Removes the
    //      AD9361's DC offset / LO leakage, which would otherwise be
    //      retransmitted as a carrier on the LO frequency and look like a
    //      signal common to RX1 and RX2. Always tracking; applied with cfg[3].
    //      Each accumulator is its own one-add-one-subtract loop.
    static ap_int<32> dacc[4] = {0, 0, 0, 0};
#pragma HLS ARRAY_PARTITION variable=dacc complete
#pragma HLS RESET variable=dacc
    smp_t x1r, x1i, x2r, x2i;
    smp_t dcn_re[2], dcn_im[2];            // DC-blocked RX1, RX2
    {
        const bool dcb = cfg[3];
        smp_t raw[4] = {r1r, r1i, r2r, r2i};
        smp_t cor[4];
#pragma HLS ARRAY_PARTITION variable=raw complete
#pragma HLS ARRAY_PARTITION variable=cor complete
        for (int k = 0; k < 4; k++) {
#pragma HLS UNROLL
            ap_int<18> d = (ap_int<18>)raw[k] - (ap_int<18>)(dacc[k] >> PL_NPI_DC_SHIFT);
            dacc[k] = dacc[k] + d;
            ap_int<18> v = dcb ? d : (ap_int<18>)raw[k];
            cor[k] = (v > 32767) ? smp_t(32767) : (v < -32768) ? smp_t(-32768) : (smp_t)v;
        }
        dcn_re[0] = cor[0]; dcn_im[0] = cor[1]; dcn_re[1] = cor[2]; dcn_im[1] = cor[3];
    }

#if PL_NPI_V25
    // ---- 0b. v2.5 pre-processing pipeline (the PL-NPI loop below is unchanged) --
    //   DC -> R1 -> CW notches on both channels -> R2 -> RX2 equaliser (centre
    //   tap, R3 products, R4 sums, R5 round + saturate). Every register is one
    //   sample, so every stage is a short path; RX1 is delayed to match RX2:
    //   both leave PL_NPI_PRE_LAT = 6 samples after their input (checked with
    //   an impulse, C simulation: the centre tap lines up with RX1), which is
    //   the extra output latency.
    //   The notches are in FRONT of the adaptive loop: a CW (even 30 dB above
    //   the noise) never reaches the weights, so it neither costs the NLMS its
    //   spatial degree of freedom nor modulates noise around itself, and a CW
    //   plus a wideband jammer can both be removed.
    //   The equaliser maps RX2 onto RX1 for the jammer (a few ns of cable or
    //   filter mismatch, which one complex weight cannot absorb: a 40 MHz
    //   jammer through 0.5 sample of mismatch is nulled only ~11 dB, C/N0 -18
    //   dB at J/N 30); the firmware estimates the taps and sets cfg[14].
    static smp_t dcR_re[2], dcR_im[2], ntR_re[2], ntR_im[2];
    static smp_t xd_re[4], xd_im[4], eh_re[2], eh_im[2];
    static ap_int<32> ep_rr[3], ep_ii[3], ep_ri[3], ep_ir[3];
    static ap_int<35> es_re = 0, es_im = 0;
    static smp_t eo_re = 0, eo_im = 0;
#pragma HLS ARRAY_PARTITION variable=dcR_re complete
#pragma HLS ARRAY_PARTITION variable=dcR_im complete
#pragma HLS ARRAY_PARTITION variable=ntR_re complete
#pragma HLS ARRAY_PARTITION variable=ntR_im complete
#pragma HLS ARRAY_PARTITION variable=xd_re complete
#pragma HLS ARRAY_PARTITION variable=xd_im complete
#pragma HLS ARRAY_PARTITION variable=eh_re complete
#pragma HLS ARRAY_PARTITION variable=eh_im complete
#pragma HLS ARRAY_PARTITION variable=ep_rr complete
#pragma HLS ARRAY_PARTITION variable=ep_ii complete
#pragma HLS ARRAY_PARTITION variable=ep_ri complete
#pragma HLS ARRAY_PARTITION variable=ep_ir complete
#pragma HLS RESET variable=dcR_re
#pragma HLS RESET variable=dcR_im
#pragma HLS RESET variable=ntR_re
#pragma HLS RESET variable=ntR_im
#pragma HLS RESET variable=xd_re
#pragma HLS RESET variable=xd_im
#pragma HLS RESET variable=eh_re
#pragma HLS RESET variable=eh_im
#pragma HLS RESET variable=ep_rr
#pragma HLS RESET variable=ep_ii
#pragma HLS RESET variable=ep_ri
#pragma HLS RESET variable=ep_ir
#pragma HLS RESET variable=es_re
#pragma HLS RESET variable=es_im
#pragma HLS RESET variable=eo_re
#pragma HLS RESET variable=eo_im
    smp_t nt_re[2], nt_im[2];                      // notch outputs -> R2
#pragma HLS ARRAY_PARTITION variable=nt_re complete
#pragma HLS ARRAY_PARTITION variable=nt_im complete
#if PL_NPI_NOTCH_IN && PL_NPI_NOTCHES > 0
    static ap_int<25> pns_re[2 * PL_NPI_NOTCHES], pns_im[2 * PL_NPI_NOTCHES];
    static ap_int<25> pnm_re[2 * PL_NPI_NOTCHES], pnm_im[2 * PL_NPI_NOTCHES];
    static ap_int<17> pnx_re[2 * PL_NPI_NOTCHES], pnx_im[2 * PL_NPI_NOTCHES];
#pragma HLS ARRAY_PARTITION variable=pns_re complete
#pragma HLS ARRAY_PARTITION variable=pns_im complete
#pragma HLS ARRAY_PARTITION variable=pnm_re complete
#pragma HLS ARRAY_PARTITION variable=pnm_im complete
#pragma HLS ARRAY_PARTITION variable=pnx_re complete
#pragma HLS ARRAY_PARTITION variable=pnx_im complete
#pragma HLS RESET variable=pns_re
#pragma HLS RESET variable=pns_im
#pragma HLS RESET variable=pnm_re
#pragma HLS RESET variable=pnm_im
#pragma HLS RESET variable=pnx_re
#pragma HLS RESET variable=pnx_im
    for (int ch = 0; ch < 2; ch++) {
#pragma HLS UNROLL
        ap_int<17> xr = dcR_re[ch], xi = dcR_im[ch];
        for (int k = 0; k < PL_NPI_NOTCHES; k++) {
#pragma HLS UNROLL
            const int i = ch * PL_NPI_NOTCHES + k;
            notch_step(cfg[PL_NPI_CFG_NOTCH_LSB + k], ncoef(96 * k + 95, 96 * k), xr, xi,
                       pns_re[i], pns_im[i], pnm_re[i], pnm_im[i], pnx_re[i], pnx_im[i]);
        }
        nt_re[ch] = (smp_t)xr;
        nt_im[ch] = (smp_t)xi;
    }
#else
    for (int ch = 0; ch < 2; ch++) {
#pragma HLS UNROLL
        nt_re[ch] = dcR_re[ch];
        nt_im[ch] = dcR_im[ch];
    }
#endif

    // equaliser: e = f0 h0 + f1 h1 + f2 h2, h0 = newest R2 sample of RX2
    ap_int<16> fc_re[3], fc_im[3];
#pragma HLS ARRAY_PARTITION variable=fc_re complete
#pragma HLS ARRAY_PARTITION variable=fc_im complete
    const bool eqon = (PL_NPI_EQ != 0) && cfg[14];
    for (int k = 0; k < 3; k++) {
#pragma HLS UNROLL
        fc_re[k] = eqon ? (ap_int<16>)eqcoef(32 * k + 15, 32 * k)      : ap_int<16>(k == 1 ? (1 << PL_NPI_EQFRAC) : 0);
        fc_im[k] = eqon ? (ap_int<16>)eqcoef(32 * k + 31, 32 * k + 16) : ap_int<16>(0);
    }
    smp_t h_re[3], h_im[3];
#pragma HLS ARRAY_PARTITION variable=h_re complete
#pragma HLS ARRAY_PARTITION variable=h_im complete
    h_re[0] = ntR_re[1]; h_im[0] = ntR_im[1];
    h_re[1] = eh_re[0];  h_im[1] = eh_im[0];
    h_re[2] = eh_re[1];  h_im[2] = eh_im[1];
    // R3: the four real products per tap
    ap_int<32> nep_rr[3], nep_ii[3], nep_ri[3], nep_ir[3];
#pragma HLS ARRAY_PARTITION variable=nep_rr complete
#pragma HLS ARRAY_PARTITION variable=nep_ii complete
#pragma HLS ARRAY_PARTITION variable=nep_ri complete
#pragma HLS ARRAY_PARTITION variable=nep_ir complete
    for (int k = 0; k < 3; k++) {
#pragma HLS UNROLL
        nep_rr[k] = fc_re[k] * h_re[k];
        nep_ii[k] = fc_im[k] * h_im[k];
        nep_ri[k] = fc_re[k] * h_im[k];
        nep_ir[k] = fc_im[k] * h_re[k];
    }
    // R4: sums of the previous sample's products
    ap_int<35> nes_re = ((ap_int<35>)ep_rr[0] - ep_ii[0]) + ((ap_int<35>)ep_rr[1] - ep_ii[1]) + ((ap_int<35>)ep_rr[2] - ep_ii[2]);
    ap_int<35> nes_im = ((ap_int<35>)ep_ri[0] + ep_ir[0]) + ((ap_int<35>)ep_ri[1] + ep_ir[1]) + ((ap_int<35>)ep_ri[2] + ep_ir[2]);
    // R5: round, saturate to 16 bits
    const ap_int<35> ehalf = ap_int<35>(1) << (PL_NPI_EQFRAC - 1);
    ap_int<22> er = (ap_int<22>)((es_re + ehalf) >> PL_NPI_EQFRAC);
    ap_int<22> ei = (ap_int<22>)((es_im + ehalf) >> PL_NPI_EQFRAC);
    smp_t neo_re = (er > 32767) ? smp_t(32767) : (er < -32768) ? smp_t(-32768) : (smp_t)er;
    smp_t neo_im = (ei > 32767) ? smp_t(32767) : (ei < -32768) ? smp_t(-32768) : (smp_t)ei;

    // the aligned pair the loop sees
    x1r = xd_re[3]; x1i = xd_im[3];
    x2r = eo_re;    x2i = eo_im;
#ifdef PL_NPI_DEBUG
    pl_npi_dbg[0] = (int)x1r; pl_npi_dbg[1] = (int)x2r;     // C simulation only: alignment check
#endif

    // commit the pre-processing registers (the delay lines take the OLD R2
    // values, so they are shifted before R2 is overwritten)
    for (int j = 3; j > 0; j--) {
#pragma HLS UNROLL
        xd_re[j] = xd_re[j - 1];  xd_im[j] = xd_im[j - 1];
    }
    xd_re[0] = ntR_re[0];  xd_im[0] = ntR_im[0];
    eh_re[1] = eh_re[0];   eh_im[1] = eh_im[0];
    eh_re[0] = ntR_re[1];  eh_im[0] = ntR_im[1];
    for (int ch = 0; ch < 2; ch++) {
#pragma HLS UNROLL
        dcR_re[ch] = dcn_re[ch];  dcR_im[ch] = dcn_im[ch];
        ntR_re[ch] = nt_re[ch];   ntR_im[ch] = nt_im[ch];
    }
    for (int k = 0; k < 3; k++) {
#pragma HLS UNROLL
        ep_rr[k] = nep_rr[k];  ep_ii[k] = nep_ii[k];
        ep_ri[k] = nep_ri[k];  ep_ir[k] = nep_ir[k];
    }
    es_re = nes_re;  es_im = nes_im;
    eo_re = neo_re;  eo_im = neo_im;
#else
    x1r = dcn_re[0]; x1i = dcn_im[0]; x2r = dcn_re[1]; x2i = dcn_im[1];
#endif

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

    // Every stage below reads the OLD register values (they are written at
    // the end), i.e. each one is one clock of the pipeline.

    // ---- E2: weights: subtract + saturate, or 0 --------------------------
    //      bypassed: weights held at 0 (each jammer starts from w = 0);
    //      image taps off: c1 = c2 = 0. Not adapting: dl is 0 (E1).
    w_t nwp_re[PL_NPI_NW], nwp_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=nwp_re complete
#pragma HLS ARRAY_PARTITION variable=nwp_im complete
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        bool keep = active && (i == 0 || img);
        nwp_re[i] = keep ? sat32((ap_int<49>)wp_re[i] - dl_re[i]) : w_t(0);
        nwp_im[i] = keep ? sat32((ap_int<49>)wp_im[i] - dl_im[i]) : w_t(0);
    }

    // ---- E1: update = (w + term) / 2^18, 0 while not adapting. The 2^-18
    //      leak uses the weight of one sample ago (a 2^-18 term: the
    //      difference is invisible), so E2 does not need the add -------------
    ap_int<49> ndl_re[PL_NPI_NW], ndl_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=ndl_re complete
#pragma HLS ARRAY_PARTITION variable=ndl_im complete
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        ndl_re[i] = adapt ? (ap_int<49>)(((ap_int<49>)wd_re[i] + TE_RE(i)) >> 18) : ap_int<49>(0);
        ndl_im[i] = adapt ? (ap_int<49>)(((ap_int<49>)wd_im[i] + TE_IM(i)) >> 18) : ap_int<49>(0);
    }

    // ---- D2: saturate to 48 bits; D1: shift into Q.20 weight units --------
    ap_int<48> ntD_re[PL_NPI_NW], ntD_im[PL_NPI_NW];
    ap_int<84> nvD_re[PL_NPI_NW], nvD_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=ntD_re complete
#pragma HLS ARRAY_PARTITION variable=ntD_im complete
#pragma HLS ARRAY_PARTITION variable=nvD_re complete
#pragma HLS ARRAY_PARTITION variable=nvD_im complete
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        ntD_re[i] = sat48(vD_re[i]);
        ntD_im[i] = sat48(vD_im[i]);
        ap_int<8> shi = (i > 0) ? shC_img : shC;
        ap_int<84> vr = pC_re[i], vi = pC_im[i];
        if (shi >= 0) { vr = vr << shi;    vi = vi << shi; }
        else          { vr = vr >> (-shi); vi = vi >> (-shi); }
        nvD_re[i] = vr;
        nvD_im[i] = vi;
    }

    // ---- C: corr * mantissa of alpha*gain(band) ----------------------------
    ap_int<64> npC_re[PL_NPI_NW], npC_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=npC_re complete
#pragma HLS ARRAY_PARTITION variable=npC_im complete
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        npC_re[i] = cB_re[i] * (ap_int<19>)mselB;
        npC_im[i] = cB_im[i] * (ap_int<19>)mselB;
    }

    // ---- B2: corr = conj(u) * s summed; alpha*gain of the sample's band ----
    ap_int<45> ncB_re[PL_NPI_NW], ncB_im[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=ncB_re complete
#pragma HLS ARRAY_PARTITION variable=ncB_im complete
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        ncB_re[i] = (ap_int<45>)pb_rr[i] + (ap_int<45>)pb_ii[i];
        ncB_im[i] = (ap_int<45>)pb_ri[i] - (ap_int<45>)pb_ir[i];
    }
    ap_uint<18> nmsel = mkB1[band1];
    ap_int<8>   nshB  = ekB1[band1] - PL_NPI_STEP_SHIFT;

    // ---- B1: products conj(u) * s, and the Eq. 11 band of |Re s| ----------
    ap_int<44> npb_rr[PL_NPI_NW], npb_ii[PL_NPI_NW], npb_ri[PL_NPI_NW], npb_ir[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=npb_rr complete
#pragma HLS ARRAY_PARTITION variable=npb_ii complete
#pragma HLS ARRAY_PARTITION variable=npb_ri complete
#pragma HLS ARRAY_PARTITION variable=npb_ir complete
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        npb_rr[i] = uB_re[i] * sA_re;
        npb_ii[i] = uB_im[i] * sA_im;
        npb_ri[i] = uB_re[i] * sA_im;
        npb_ir[i] = uB_im[i] * sA_re;
    }
    ap_int<25> yabs = (sA_re < 0) ? (ap_int<25>)(-sA_re) : sA_re;
    ap_uint<2> nband = (yabs > TH3) ? 3 : (yabs > TH2) ? 2 : (yabs > TH1) ? 1 : 0;

    // ---- A3: s = t0 + t1 (Q22); the output; s at Q.8 for the loop ---------
    ap_int<56> s_re = t0_re + t1_re, s_im = t0_im + t1_im;

    // Output: bypass = x1; filter = round(s * 2^sh / 2^22), saturated. Both
    // leave PL_NPI_LATENCY samples after their input, so switching between
    // them never shifts the signal in time.
    // Round to nearest, not floor (v2.1.10: truncation cost up to 3.7 dB of
    // output SINR once the gain loop had turned the RX gain down).
    ap_uint<32> out;
    if (active) {
        const ap_int<62> half = ap_int<62>(1) << 21;
        out(15, 0)  = (ap_uint<16>)sat16((((ap_int<62>)s_re << sh) + half) >> 22);
        out(31, 16) = (ap_uint<16>)sat16((((ap_int<62>)s_im << sh) + half) >> 22);
    } else {
        out(15, 0)  = (ap_uint<16>)x1S_re;
        out(31, 16) = (ap_uint<16>)x1S_im;
    }

#if PL_NPI_NOTCHES > 0 && !(PL_NPI_V25 && PL_NPI_NOTCH_IN)
    // ---- CW notches, in cascade on the OUTPUT (v2.4 placement; PL_NPI_NOTCH_IN = 0)
    //   Same notch as in front of the loop: see notch_step() above.
    {
        static ap_int<25> ns_re[PL_NPI_NOTCHES], ns_im[PL_NPI_NOTCHES];
        static ap_int<25> nm_re[PL_NPI_NOTCHES], nm_im[PL_NPI_NOTCHES];
        static ap_int<17> nx_re[PL_NPI_NOTCHES], nx_im[PL_NPI_NOTCHES];
#pragma HLS ARRAY_PARTITION variable=ns_re complete
#pragma HLS ARRAY_PARTITION variable=ns_im complete
#pragma HLS ARRAY_PARTITION variable=nm_re complete
#pragma HLS ARRAY_PARTITION variable=nm_im complete
#pragma HLS ARRAY_PARTITION variable=nx_re complete
#pragma HLS ARRAY_PARTITION variable=nx_im complete
#pragma HLS RESET variable=ns_re
#pragma HLS RESET variable=ns_im
#pragma HLS RESET variable=nm_re
#pragma HLS RESET variable=nm_im
#pragma HLS RESET variable=nx_re
#pragma HLS RESET variable=nx_im
        ap_int<17> xr = (ap_int<16>)out(15, 0), xi = (ap_int<16>)out(31, 16);
        for (int k = 0; k < PL_NPI_NOTCHES; k++) {
#pragma HLS UNROLL
            notch_step(cfg[PL_NPI_CFG_NOTCH_LSB + k], ncoef(96 * k + 95, 96 * k), xr, xi,
                       ns_re[k], ns_im[k], nm_re[k], nm_im[k], nx_re[k], nx_im[k]);
        }
        out(15, 0)  = (ap_uint<16>)(ap_int<16>)xr;
        out(31, 16) = (ap_uint<16>)(ap_int<16>)xi;
    }
#endif
    m_axis_y.write(out);

    // ---- A2: two partial sums of s = x1 + sum u * w ------------------------
    ap_int<56> nt0_re = ((ap_int<56>)pa_rr[0] - pa_ii[0]) + ((ap_int<56>)pa_rr[1] - pa_ii[1]);
    ap_int<56> nt0_im = ((ap_int<56>)pa_ri[0] + pa_ir[0]) + ((ap_int<56>)pa_ri[1] + pa_ir[1]);
    ap_int<56> nt1_re = ((ap_int<56>)pa_rr[2] - pa_ii[2]) + ((ap_int<56>)x1qA_re << 20);
    ap_int<56> nt1_im = ((ap_int<56>)pa_ri[2] + pa_ir[2]) + ((ap_int<56>)x1qA_im << 20);

    // ---- A1: products u * w with the current weights -----------------------
    ap_int<51> npa_rr[PL_NPI_NW], npa_ii[PL_NPI_NW], npa_ri[PL_NPI_NW], npa_ir[PL_NPI_NW];
#pragma HLS ARRAY_PARTITION variable=npa_rr complete
#pragma HLS ARRAY_PARTITION variable=npa_ii complete
#pragma HLS ARRAY_PARTITION variable=npa_ri complete
#pragma HLS ARRAY_PARTITION variable=npa_ir complete
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        npa_rr[i] = u_re[i] * wp_re[i];
        npa_ii[i] = u_im[i] * wp_im[i];
        npa_ri[i] = u_re[i] * wp_im[i];
        npa_ir[i] = u_im[i] * wp_re[i];
    }

    // ---- commit all registers ---------------------------------------------
    for (int i = 0; i < PL_NPI_NW; i++) {
#pragma HLS UNROLL
        wd_re[i] = wp_re[i];   wd_im[i] = wp_im[i];
        wp_re[i] = nwp_re[i];  wp_im[i] = nwp_im[i];
        dl_re[i] = ndl_re[i];  dl_im[i] = ndl_im[i];
#if PL_NPI_EXTRA_DELAY > 0
        for (int j = NQ - 1; j > 0; j--) {
#pragma HLS UNROLL
            tQ_re[j][i] = tQ_re[j - 1][i];  tQ_im[j][i] = tQ_im[j - 1][i];
        }
        tQ_re[0][i] = tD_re[i];  tQ_im[0][i] = tD_im[i];
#endif
        tD_re[i] = ntD_re[i];  tD_im[i] = ntD_im[i];
        vD_re[i] = nvD_re[i];  vD_im[i] = nvD_im[i];
        pC_re[i] = npC_re[i];  pC_im[i] = npC_im[i];
        cB_re[i] = ncB_re[i];  cB_im[i] = ncB_im[i];
        pb_rr[i] = npb_rr[i];  pb_ii[i] = npb_ii[i];
        pb_ri[i] = npb_ri[i];  pb_ir[i] = npb_ir[i];
        uB_re[i] = uS_re[i];   uB_im[i] = uS_im[i];
        uS_re[i] = uA1_re[i];  uS_im[i] = uA1_im[i];
        uA1_re[i] = u_re[i];   uA1_im[i] = u_im[i];
        pa_rr[i] = npa_rr[i];  pa_ii[i] = npa_ii[i];
        pa_ri[i] = npa_ri[i];  pa_ir[i] = npa_ir[i];
    }
    for (int k = 0; k < 4; k++) {
#pragma HLS UNROLL
        mkB1[k] = mkB0[k];  ekB1[k] = ekB0[k];
        mkB0[k] = mkS[k];   ekB0[k] = ekS[k];
        mkS[k]  = mkA1[k];  ekS[k]  = ekA1[k];
        mkA1[k] = mk[k];    ekA1[k] = ek[k];
    }
    mselB = nmsel;  shB = nshB;  bandB = band1;
    shC   = shB;
    shC_img = shB - PL_NPI_IMG_SHIFT;
    band1 = nband;
    sA_re = sat25(s_re >> 14);
    sA_im = sat25(s_im >> 14);
    t0_re = nt0_re;  t0_im = nt0_im;  t1_re = nt1_re;  t1_im = nt1_im;
    x1S_re = x1A_re;  x1S_im = x1A_im;
    x1A_re = x1r;     x1A_im = x1i;
    x1qA_re = x1qr;   x1qA_im = x1qi;
}
