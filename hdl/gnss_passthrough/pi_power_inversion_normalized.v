/*******************************************************************************
 * pi_power_inversion_normalized.v
 *
 * Normalized Power-Inversion (PI) adaptive-array core -- Section 2 of:
 *   Meng, Feng, Lu -- "Anti-Jamming with Adaptive Arrays Utilizing Power
 *   Inversion Algorithm", Tsinghua Science and Technology, 13(6), 2008.
 *
 * This is the algorithm pi_power_inversion.v's own header flagged as
 * deliberately NOT built there ("the normalized variant of Section 2
 * (Eq. 13)... a different, separate algorithm; ask if you want that one
 * built too"). Everything about the weight recursion (Eq. 2/4, the
 * leaky-integrator LPF, the quiescent offset vector w_o=[1,0]) is IDENTICAL
 * to that module -- this file exists ONLY because Eq. 13 replaces the fixed
 * alpha register with a per-sample alpha(k) computed from the input power.
 *
 * ALGORITHM DELTA FROM THE STANDARD CORE
 * ---------------------------------------------------------------------------
 *   Standard (pi_power_inversion.v): alpha is a fixed, software-written
 *   constant (default 1.0, "the paper's slow-converge case").
 *
 *   Normalized (this file), paper's Eq. 12/13:
 *       alpha(k) = -1 / (2 * x^T(k)x(k))                              (12)
 *       alpha(k) = -1 / (gamma + 2 * x^T(k)x(k)),  gamma > 0           (13)
 *   Eq. 13 is Eq. 12 with Haykin's small-denominator regularisation
 *   (division by a near-zero norm is the "numerical difficulties" the paper
 *   names outright) -- this is exactly what NLMS does to its own step size,
 *   for exactly the same reason, which the paper cites directly (ref [10]).
 *
 * TWO DELIBERATE READINGS OF THE PAPER, BOTH DOCUMENTED HERE BECAUSE THEY
 * ARE JUDGMENT CALLS, NOT LITERAL TRANSCRIPTION -- SAY SO RATHER THAN HIDE IT
 * ---------------------------------------------------------------------------
 *   1. x^T(k)x(k) in Eq. 12/13 is read here as sum_i |x_i(k)|^2 (real,
 *      non-negative INPUT POWER), not a literal transpose-only sum of
 *      complex squares. The paper is loose with transpose vs. Hermitian-
 *      transpose throughout Section 2 -- Eq. 7 calls w^T x x^T w a "mean
 *      square error", which is only real-valued (as its name requires) if
 *      x is Hermitian-conjugated somewhere, and Eq. 12/13's own citation
 *      (Haykin, NLMS-style regularisation) is for a normalisation by
 *      ||x(k)||^2 = x^H(k)x(k), not a complex x^T(k)x(k). Read literally,
 *      alpha(k) could come out complex or sign-indeterminate, which cannot
 *      be a step size. sum|x_i|^2 is the only reading that is real,
 *      non-negative, and matches both the paper's own physical description
 *      ("the recursive step size" tracking "the power of the interference")
 *      and standard NLMS practice.
 *
 *   2. Eq. 13 as printed yields alpha(k) <= 0 always (the leading minus
 *      sign). Eq. 14's update folds that sign into a net ADDITION of the
 *      correlation term -- an equally valid algebraic form, but a SECOND,
 *      independent sign convention from Eq. 2/6's SUBTRACTIVE form that
 *      pi_power_inversion.v already implements and this file's testbench
 *      has already proven converges correctly with a POSITIVE alpha. Rather
 *      than introduce a second sign convention into this codebase and risk
 *      the two disagreeing, this file computes |alpha(k)| (the RECIPROCAL
 *      MAGNITUDE from Eq. 13, sign dropped) and feeds it into the SAME
 *      subtractive recursion structure already validated in
 *      pi_power_inversion.v. This preserves Eq. 13's actual content -- a
 *      normalisation by input power -- without gambling a sign-convention
 *      mismatch against code this project already trusts. Getting an
 *      adaptive filter's feedback sign backwards is not a cosmetic bug: it
 *      diverges instead of nulling. This is the safer of the two paths and
 *      is called out here explicitly rather than silently.
 *
 * DIVISION: WHY A SEPARATE PIPELINED CORE (pi_reciprocal.v), AND WHY ITS
 * RESULT CAN LAG THE SAMPLE THAT PRODUCED IT
 * ---------------------------------------------------------------------------
 *   Eq. 13 needs a real division every sample -- pi_power_inversion.v needs
 *   none (alpha is a stored constant), which is the whole reason this is a
 *   separate module rather than a mode bit on that one. A fixed-point
 *   reciprocal with useful precision is not a single-cycle operation;
 *   pi_reciprocal.v is a Q_FRAC+1-stage pipelined restoring divider
 *   (1/cycle throughput, Q_FRAC+1 cycles latency -- see its own header).
 *
 *   Rather than build a matching Q_FRAC+1-deep delay line to hold x(k) and
 *   s(k) until "their own" alpha(k) is ready (a real option, but a lot of
 *   registers for a quantity -- input power -- that Eq. 13's own premise is
 *   already a SMOOTHED, slowly-varying statistic, not a per-sample exact
 *   value), this core keeps one register holding the MOST RECENTLY
 *   COMPLETED alpha(k) and uses it for every weight update until a newer
 *   one lands. That is a bounded, small staleness (at most Q_FRAC+1 cycles
 *   old, a few sample periods at this board's ~4 clk/sample rate) against a
 *   quantity the algorithm already treats as a running power estimate, not
 *   an instantaneous one -- standard practice in real NLMS hardware, not a
 *   shortcut unique to this file.
 *
 * ALPHA_GAIN_SHIFT -- A CALIBRATION CONSTANT THE PAPER DOES NOT (AND CANNOT)
 * SPECIFY, FOUND BY THE SAME EMPIRICAL METHOD pi_power_inversion.v'S OWN
 * LPF_SHIFT ALREADY WAS
 * ---------------------------------------------------------------------------
 *   Eq. 13's alpha(k) = 1/(gamma+2*sum|x_i|^2) has a defining property:
 *   alpha(k) * sum|x_i(k)|^2 is approximately CONSTANT (~0.5) REGARDLESS of
 *   the input signal's absolute scale -- that automatic scale-invariance is
 *   the entire point of a normalized step size. It also means alpha(k)'s
 *   ABSOLUTE magnitude is meaningless on its own without knowing what scale
 *   x(k) is in. The paper works in whatever abstract units its own
 *   simulation used; this board's x_i(k) are raw ADC-scale integers
 *   (amplitude ~100 for this project's standard test stimulus, e.g.
 *   tb_pi_power_inversion.v's AMP=100), giving sum|x_i|^2 ~ 10^4-10^5 and
 *   therefore alpha(k) ~ 10^-5 -- verified directly in
 *   tb_pi_power_inversion_normalized.v: with ALPHA_GAIN_SHIFT=0 the
 *   weights barely moved from w_o in 2000 samples (|w[0]| stuck at
 *   ~0.999), five-plus orders of magnitude too slow, NOT because the
 *   reciprocal core is wrong (pi_reciprocal.v is verified bit-exact
 *   against real division in tb_pi_reciprocal.v) but because 1/D(k) at
 *   THIS signal scale is just that small a number.
 *
 *   pi_power_inversion.v's LPF_SHIFT=18 was not derived from the paper's
 *   tau=200000 by a blind log2() either -- its own header says outright it
 *   was "re-derived for amp=100 ... using ref_model.py first". This module
 *   needs the equivalent step for the SAME reason: alpha(k)'s natural
 *   magnitude at this board's signal scale has to be related back to an
 *   effective step size in the same numeric regime pi_power_inversion.v
 *   was already tuned in, and nothing about that relationship is written
 *   down in the paper (which never specifies its own simulation's ADC
 *   scale). ALPHA_GAIN_SHIFT applies as an EXTRA left-shift, folded into
 *   the weight update's own shift-back (see WEIGHT_UPDATE below: the
 *   divisor is "Q_FRAC - ALPHA_GAIN_SHIFT", not "Q_FRAC" -- mathematically
 *   equivalent to alpha(k) * 2^ALPHA_GAIN_SHIFT, with no extra multiply or
 *   overflow risk since ALPHA_PROD_W already carries full width through
 *   this stage). ALPHA_GAIN_SHIFT=17 was chosen empirically in simulation
 *   (tb_pi_power_inversion_normalized.v) to land this core's convergence
 *   in the SAME few-hundred-sample regime the standard core already
 *   converges in at amp=100 -- reproduce that tuning run, the same way
 *   ref_model.py's own numbers are reproducible, before trusting a
 *   different value at a substantially different expected signal scale.
 *
 *   HONEST LIMIT OF THAT TUNING, MEASURED, NOT ASSUMED: re-running the same
 *   testbench at amp=20 (5x weaker, ALPHA_GAIN_SHIFT left at 17, nothing
 *   retuned) did NOT show the amplitude-independence Eq. 13's own
 *   alpha(k)*sum|x_i|^2=const property suggests in theory -- this core
 *   converged SLOWER at amp=20 than at amp=100 (not settled into the test
 *   band by sample 2000, versus 18 samples at amp=100), roughly tracking
 *   the standard core's OWN slowdown at that untuned amplitude rather than
 *   staying fast while the standard core alone got slow. Likely causes,
 *   neither confirmed: the gamma=1 floor becoming a proportionally larger
 *   fraction of a much smaller D(k) at low power, or fixed-point rounding
 *   in the >>> LPF_SHIFT stage mattering more at smaller absolute weight
 *   increments. This is reported here because it was measured, not
 *   theorised -- do not cite this core as proven amplitude-robust without
 *   re-running that comparison yourself; what IS verified is correct
 *   convergence at the amp=100 calibration point, and a bit-exact
 *   reciprocal (tb_pi_reciprocal.v).
 *
 * FIXED-POINT FORMAT
 * ---------------------------------------------------------------------------
 *   x_i(k)     : DATA_W-bit signed, 0 frac bits -- same as the standard core.
 *   w'_i, w_i  : WEIGHT_W-bit signed, WEIGHT_FRAC frac bits -- same as the
 *                standard core (this file does not change the weight or
 *                s(k) format at all, only where alpha comes from).
 *   gamma      : GAMMA_W-bit unsigned, same fixed-point alignment as the
 *                power sum it is added to (0 frac bits) -- Eq. 13's
 *                regulariser, runtime-configurable. Must be >= 1; this
 *                module does not check that (same as the standard core
 *                trusting the caller to keep alpha sane).
 *   alpha(k)   : UQ0.Q_FRAC unsigned (pi_reciprocal.v's own output format),
 *                always representing a MAGNITUDE (see judgment call 2 above)
 *                -- there is no ALPHA_W parameter here the way the standard
 *                core has one; Q_FRAC plays that role.
 *
 * PIPELINE
 * ---------------------------------------------------------------------------
 *   s(k) path: identical 2-cycle latch/dot-product/correlate structure to
 *   pi_power_inversion.v, unchanged -- copy for copy the same code.
 *
 *   alpha(k) path: runs in PARALLEL, independent latency (Q_FRAC+1 cycles
 *   pipelined divider), landing in the "latest alpha" register whenever a
 *   result completes -- does NOT gate weights_valid's 2-cycle timing.
 *
 * Reset behaviour matches the standard core (w'_i = 0, so w_i starts at
 * exactly w_o) with one addition: the "latest alpha" register also resets
 * to 0, so no adaptation happens at all until the FIRST reciprocal result
 * completes, Q_FRAC+1 cycles after the first sample_valid -- a bounded
 * startup transient, not a bug.
 ******************************************************************************/
`timescale 1ns/1ps

module pi_power_inversion_normalized #(
    parameter integer M           = 2,     // antenna elements -- see pi_power_inversion.v's
                                            // header for why 2 matches this board
    parameter integer DATA_W      = 16,    // raw ADC I/Q sample width, signed, 0 frac bits
    parameter integer WEIGHT_W    = 32,    // adaptive weight width, signed
    parameter integer WEIGHT_FRAC = 20,    // weight fractional bits
    parameter integer LPF_SHIFT   = 18,    // same LPF/tau approximation as the standard core
    parameter integer Q_FRAC      = 32,    // alpha(k) FIXED-POINT PRECISION (and pi_reciprocal
                                            // pipeline depth-1) -- NOT a gain; changing this
                                            // alone changes nothing about the recovered value,
                                            // only how finely it's represented (verified: the
                                            // >>> Q_FRAC below exactly cancels the reciprocal's
                                            // own 2^Q_FRAC scaling for any Q_FRAC).
    parameter integer ALPHA_GAIN_SHIFT = 17,   // <<< the actual calibration knob, see below
    // Declared 64 bits wide, NOT plain `integer` (always exactly 32 bits):
    // used below as GAMMA_INIT[GAMMA_W-1:0] with GAMMA_W=36 for this
    // project's default M=2/DATA_W=16. Verilog returns 'x' -- or, per
    // Vivado's synthesizer specifically, a hard elaboration error
    // (Synth 8-524, "part-select out of range") -- for any bit selected
    // outside a vector's DECLARED width. Icarus Verilog tolerated the
    // narrower declaration silently in simulation; Vivado did not. Same
    // bug class, same fix, as THRESH1/2/3 in pi_power_inversion_pl_npi.v --
    // caught there first by a testbench, caught here by Vivado itself
    // (confirmed against a real synth_design run before this fix).
    parameter [63:0] GAMMA_INIT   = 64'd1,     // Eq. 13's gamma > 0; smallest safe regulariser

    // Derived widths -- declared here so they size the ports below.
    parameter integer PROD1_W     = DATA_W + WEIGHT_W + 1,           // x_i * w_i
    parameter integer SUM_GROWTH  = (M <= 1) ? 1 : $clog2(M),
    parameter integer S_W         = PROD1_W + SUM_GROWTH,            // s(k) width
    parameter integer PROD2_W     = DATA_W + S_W + 1,                 // conj(x_i) * s(k)
    parameter integer POW_W       = 2*DATA_W + $clog2(2*M) + 2,       // sum_i |x_i|^2, generous headroom
    parameter integer GAMMA_W     = POW_W,                            // gamma shares the power sum's width
    parameter integer DEN_W       = POW_W + 2,                        // gamma + 2*power, +1 bit for the *2, +1 guard
    parameter integer ALPHA_MAG_W = Q_FRAC + 1,                       // pi_reciprocal's q_out width
    parameter integer ALPHA_PROD_W = PROD2_W + ALPHA_MAG_W + 1        // corr * alpha_mag (signed, +1 for the zero-extend sign bit)
) (
    input  wire                      clk,
    input  wire                      aresetn,        // active-low, synchronous

    input  wire [M*DATA_W-1:0]       x_re,
    input  wire [M*DATA_W-1:0]       x_im,
    input  wire                      sample_valid,

    // ---- runtime-configurable gamma (Eq. 13's regulariser) ----------------
    // Continuously sampled every cycle, NOT gated by a separate write-
    // strobe: gnss_passthrough.v's CRPA_COEF(0) alpha input had exactly
    // that pattern (a dedicated *_wr enable) hardwired to 1'b0, silencing
    // every software write while looking, from the register map, like it
    // worked (see hdl/gnss_passthrough's v1.2->v1.3 fix). A free-running
    // register removes that whole failure mode by construction -- there is
    // no enable line left to wire up wrong.
    input  wire [GAMMA_W-1:0]        gamma_in,

    input  wire                      adapt_en,       // freeze adaptation

    output reg  signed [S_W-1:0]     s_re,
    output reg  signed [S_W-1:0]     s_im,
    output reg                       s_valid,

    output wire [M*WEIGHT_W-1:0]     w_re,
    output wire [M*WEIGHT_W-1:0]     w_im,
    output reg                       weights_valid,

    // ---- diagnostic: the most recently completed alpha(k) ----------------
    // UQ0.Q_FRAC (alpha_mag / 2^Q_FRAC ~= 1/(gamma+2*sum|x_i|^2)). Exposed
    // because it is essentially free once computed, and this project has
    // repeatedly needed exactly this kind of visibility and not had it
    // (gnss_passthrough.v's own comments note there is still no AXI-visible
    // readback of the standard core's internal weights or output either).
    output wire [ALPHA_MAG_W-1:0]    alpha_mag,
    output wire                      alpha_mag_valid   // pulses once per NEW completed result
);

    integer i;

    // ---- gamma register -- free-running, see the port comment above ------
    reg [GAMMA_W-1:0] gamma_reg;
    always @(posedge clk) begin
        if (!aresetn)
            gamma_reg <= GAMMA_INIT[GAMMA_W-1:0];
        else
            gamma_reg <= gamma_in;
    end

    // ---- Eq. 13's denominator: gamma + 2*sum_i |x_i(k)|^2 -----------------
    // Combinational, straight off the raw x_re/x_im buses -- independent of
    // (and not gated on) the x_d0 latch below; this feeds the reciprocal
    // pipeline every sample_valid cycle regardless of where the s(k) path
    // is in its own 2-cycle pipeline.
    reg [POW_W-1:0] pow_sum_c;
    always @(*) begin : POWER_SUM
        reg [2*DATA_W-1:0] sq_re, sq_im;
        pow_sum_c = {POW_W{1'b0}};
        for (i = 0; i < M; i = i + 1) begin
            // Squaring a signed DATA_W-bit value always yields a
            // non-negative result that fits in 2*DATA_W bits with the top
            // bit guaranteed 0 (worst case (-2^(DATA_W-1))^2 = 2^(2*DATA_W-2)),
            // so assigning the signed product into an unsigned reg of the
            // same width is a safe, exact reinterpretation, not a truncation.
            sq_re = $signed(x_re[(i+1)*DATA_W-1 -: DATA_W]) * $signed(x_re[(i+1)*DATA_W-1 -: DATA_W]);
            sq_im = $signed(x_im[(i+1)*DATA_W-1 -: DATA_W]) * $signed(x_im[(i+1)*DATA_W-1 -: DATA_W]);
            pow_sum_c = pow_sum_c + sq_re + sq_im;
        end
    end

    wire [DEN_W-1:0] pow_x2_c = ({{(DEN_W-POW_W){1'b0}}, pow_sum_c}) << 1;
    wire [DEN_W-1:0] gamma_ext_c = {{(DEN_W-GAMMA_W){1'b0}}, gamma_reg};
    wire [DEN_W-1:0] den_c = pow_x2_c + gamma_ext_c;

    // ---- pipelined reciprocal: alpha_mag(k) = 1/den(k), Q_FRAC+1-cycle
    //      latency, 1/cycle throughput (see pi_reciprocal.v) --------------
    wire                   recip_valid;
    wire [ALPHA_MAG_W-1:0] recip_out;

    pi_reciprocal #(
        .D_W    (DEN_W),
        .Q_FRAC (Q_FRAC)
    ) u_recip (
        .clk      (clk),
        .aresetn  (aresetn),
        .d_valid  (sample_valid),
        .d_in     (den_c),
        .q_valid  (recip_valid),
        .q_out    (recip_out)
    );

    // "Latest completed alpha" -- see header for why this, not a delay
    // line matched to the divider's own latency.
    reg [ALPHA_MAG_W-1:0] alpha_mag_latest;
    always @(posedge clk) begin
        if (!aresetn)
            alpha_mag_latest <= {ALPHA_MAG_W{1'b0}};
        else if (recip_valid)
            alpha_mag_latest <= recip_out;
    end
    assign alpha_mag       = alpha_mag_latest;
    assign alpha_mag_valid = recip_valid;

    // Non-negative magnitude, zero-extended by one bit and marked signed so
    // it multiplies correctly against corr_re/corr_im below (a signed
    // value with its top bit always 0 is exactly the right representation
    // of "this is always >= 0").
    wire signed [ALPHA_MAG_W:0] alpha_mag_signed = $signed({1'b0, alpha_mag_latest});

    // ==========================================================================
    //  EVERYTHING BELOW THIS LINE IS THE SAME STRUCTURE AS
    //  pi_power_inversion.v -- s(k) computation, the quiescent offset vector,
    //  and the weight-update leaky integrator are UNCHANGED. Only the
    //  alpha_reg/ALPHA_FRAC of that file are replaced with
    //  alpha_mag_signed/Q_FRAC here. See that file for the derivation of
    //  Eq. 2-4 and the ALPHA_PROD_W width-ordering bug it documents fixing
    //  -- this file follows the same truncate-only-at-the-very-end
    //  discipline for exactly the reason logged there.
    // ==========================================================================

    function signed [WEIGHT_W-1:0] wo_re(input integer idx);
        wo_re = (idx == 0) ? (1 <<< WEIGHT_FRAC) : {WEIGHT_W{1'b0}};
    endfunction

    reg signed [WEIGHT_W-1:0] wp_re [0:M-1];
    reg signed [WEIGHT_W-1:0] wp_im [0:M-1];

    wire signed [WEIGHT_W-1:0] w_re_c [0:M-1];
    wire signed [WEIGHT_W-1:0] w_im_c [0:M-1];
    genvar g;
    generate
        for (g = 0; g < M; g = g + 1) begin : G_COMBINE
            assign w_re_c[g] = wp_re[g] + wo_re(g);
            assign w_im_c[g] = wp_im[g];
            assign w_re[(g+1)*WEIGHT_W-1 -: WEIGHT_W] = w_re_c[g];
            assign w_im[(g+1)*WEIGHT_W-1 -: WEIGHT_W] = w_im_c[g];
        end
    endgenerate

    reg signed [DATA_W-1:0] x_d0_re [0:M-1];
    reg signed [DATA_W-1:0] x_d0_im [0:M-1];
    reg                     x_d0_valid;

    always @(posedge clk) begin
        if (!aresetn) begin
            x_d0_valid <= 1'b0;
        end else begin
            x_d0_valid <= sample_valid;
            if (sample_valid) begin
                for (i = 0; i < M; i = i + 1) begin
                    x_d0_re[i] <= $signed(x_re[(i+1)*DATA_W-1 -: DATA_W]);
                    x_d0_im[i] <= $signed(x_im[(i+1)*DATA_W-1 -: DATA_W]);
                end
            end
        end
    end

    wire signed [PROD1_W-1:0] dp_re [0:M-1];
    wire signed [PROD1_W-1:0] dp_im [0:M-1];

    generate
        for (g = 0; g < M; g = g + 1) begin : G_DOTPROD
            pi_cmul #(
                .A_W    (DATA_W),
                .B_W    (WEIGHT_W),
                .CONJ_A (0)
            ) u_dp (
                .a_re (x_d0_re[g]),
                .a_im (x_d0_im[g]),
                .b_re (w_re_c[g]),
                .b_im (w_im_c[g]),
                .p_re (dp_re[g]),
                .p_im (dp_im[g])
            );
        end
    endgenerate

    reg signed [DATA_W-1:0] x_d1_re [0:M-1];
    reg signed [DATA_W-1:0] x_d1_im [0:M-1];
    reg                     x_d1_valid;

    always @(posedge clk) begin
        if (!aresetn) begin
            s_re    <= {S_W{1'b0}};
            s_im    <= {S_W{1'b0}};
            s_valid <= 1'b0;
            x_d1_valid <= 1'b0;
        end else begin
            s_valid    <= x_d0_valid;
            x_d1_valid <= x_d0_valid;

            if (x_d0_valid) begin : ACCUM_S
                reg signed [S_W-1:0] acc_re;
                reg signed [S_W-1:0] acc_im;
                acc_re = {{(S_W-PROD1_W){dp_re[0][PROD1_W-1]}}, dp_re[0]};
                acc_im = {{(S_W-PROD1_W){dp_im[0][PROD1_W-1]}}, dp_im[0]};
                for (i = 1; i < M; i = i + 1) begin
                    acc_re = acc_re + {{(S_W-PROD1_W){dp_re[i][PROD1_W-1]}}, dp_re[i]};
                    acc_im = acc_im + {{(S_W-PROD1_W){dp_im[i][PROD1_W-1]}}, dp_im[i]};
                end
                s_re <= acc_re;
                s_im <= acc_im;

                for (i = 0; i < M; i = i + 1) begin
                    x_d1_re[i] <= x_d0_re[i];
                    x_d1_im[i] <= x_d0_im[i];
                end
            end
        end
    end

    wire signed [S_W-1:0] s_re_w = s_re[S_W-1:0];
    wire signed [S_W-1:0] s_im_w = s_im[S_W-1:0];

    wire signed [PROD2_W-1:0] corr_re [0:M-1];
    wire signed [PROD2_W-1:0] corr_im [0:M-1];

    generate
        for (g = 0; g < M; g = g + 1) begin : G_CORR
            pi_cmul #(
                .A_W    (DATA_W),
                .B_W    (S_W),
                .CONJ_A (1)
            ) u_corr (
                .a_re (x_d1_re[g]),
                .a_im (x_d1_im[g]),
                .b_re (s_re_w),
                .b_im (s_im_w),
                .p_re (corr_re[g]),
                .p_im (corr_im[g])
            );
        end
    endgenerate

    always @(posedge clk) begin
        if (!aresetn) begin
            for (i = 0; i < M; i = i + 1) begin
                wp_re[i] <= {WEIGHT_W{1'b0}};
                wp_im[i] <= {WEIGHT_W{1'b0}};
            end
            weights_valid <= 1'b0;
        end else begin
            weights_valid <= x_d1_valid;

            if (x_d1_valid && adapt_en) begin : WEIGHT_UPDATE
                reg signed [ALPHA_PROD_W-1:0] alpha_term_full_re;
                reg signed [ALPHA_PROD_W-1:0] alpha_term_full_im;
                reg signed [ALPHA_PROD_W-1:0] alpha_term_shifted_re;
                reg signed [ALPHA_PROD_W-1:0] alpha_term_shifted_im;
                reg signed [ALPHA_PROD_W-1:0] leak_in_re;
                reg signed [ALPHA_PROD_W-1:0] leak_in_im;
                reg signed [ALPHA_PROD_W-1:0] leak_out_re;
                reg signed [ALPHA_PROD_W-1:0] leak_out_im;

                for (i = 0; i < M; i = i + 1) begin
                    alpha_term_full_re = corr_re[i] * alpha_mag_signed;
                    alpha_term_full_im = corr_im[i] * alpha_mag_signed;

                    // Q_FRAC - ALPHA_GAIN_SHIFT, not Q_FRAC: shifting back
                    // LESS than the reciprocal's own scale-out is exactly
                    // "multiply by 2^ALPHA_GAIN_SHIFT" without a separate
                    // multiply -- see the ALPHA_GAIN_SHIFT header section.
                    alpha_term_shifted_re = alpha_term_full_re >>> (Q_FRAC - ALPHA_GAIN_SHIFT);
                    alpha_term_shifted_im = alpha_term_full_im >>> (Q_FRAC - ALPHA_GAIN_SHIFT);

                    leak_in_re = {{(ALPHA_PROD_W-WEIGHT_W){wp_re[i][WEIGHT_W-1]}}, wp_re[i]} + alpha_term_shifted_re;
                    leak_in_im = {{(ALPHA_PROD_W-WEIGHT_W){wp_im[i][WEIGHT_W-1]}}, wp_im[i]} + alpha_term_shifted_im;

                    leak_out_re = leak_in_re >>> LPF_SHIFT;
                    leak_out_im = leak_in_im >>> LPF_SHIFT;

                    wp_re[i] <= wp_re[i] - leak_out_re[WEIGHT_W-1:0];
                    wp_im[i] <= wp_im[i] - leak_out_im[WEIGHT_W-1:0];
                end
            end
        end
    end

endmodule
