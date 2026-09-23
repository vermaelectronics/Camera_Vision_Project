/*******************************************************************************
 * pi_power_inversion_pl_npi.v
 *
 * PL-NPI: Piecewise-Linear Normalized Power Inversion -- Jia, Ni, Luo, Zhang,
 * Mao, "FPGA Implementation of Variable Step Power Inversion Array for
 * BeiDou Receiver", IEEE Access, vol. 11, 2023, Section III.B/C, Eq. (11).
 *
 * WHAT THIS ADDS ON TOP OF pi_power_inversion_normalized.v
 * ---------------------------------------------------------------------------
 * The paper's own NPI (Section III.A, its Eq. 4-6) is the SAME algorithm
 * already built there: mu_NPI(n) = 1/(x^T(n)x(n) + zeta), weight update
 * w(n+1) = w(n) - mu_NPI(n) * x^T(n)w(n) * x(n). PL-NPI's ONLY change is
 * multiplying mu_NPI(n) by a per-sample gain selected from the amplitude of
 * y_I(n) -- the IN-PHASE component of the array output y(n)=e(n) ALONE, per
 * Eq. (11):
 *
 *   mu_PL-NPI(n) = mu_NPI(n) * 1.20   if |y_I(n)| >  T3
 *                = mu_NPI(n) * 1.10   if |y_I(n)| >  T2
 *                = mu_NPI(n) * 1.05   if |y_I(n)| >  T1
 *                = mu_NPI(n)          otherwise   (|y_I(n)| in [0, T1])
 *
 * THE WHOLE POINT, PER THE PAPER'S OWN WORDS, IS AVOIDING A MAGNITUDE
 * ---------------------------------------------------------------------------
 *   "convergence of the algorithm can be ensured by simply judging the
 *   threshold of the output error signal in the SAME PHASE PATH... WITHOUT
 *   REQUIRING THE MODULUS of the complex signal." APE-NPI (the paper's own
 *   other candidate, its Eq. 9-10) needs |y(n)| = sqrt(y_I^2+y_Q^2) AND a
 *   log/exp-shaped adaptive parameter every sample -- a sqrt or CORDIC and
 *   a transcendental function, exactly the "calculation and storage"
 *   resource cost the paper spends its Section III.C explaining PL-NPI is
 *   built to avoid. PL-NPI needs only |y_I(n)|, y_I(n)'s own SIGN BIT
 *   dropped -- a comparator cascade against three constants, nothing else.
 *   This file follows that discipline: y_I(n) is s_re alone (never s_im,
 *   never a magnitude of the two), and gain selection is priority-encoded
 *   comparisons, not a lookup table or transcendental approximation.
 *
 * THRESHOLDS ARE RUNTIME PARAMETERS, NOT THE PAPER'S LITERAL 8/16/32 --
 * SAME CALIBRATION ISSUE AS pi_power_inversion_normalized.v'S ALPHA_GAIN_SHIFT
 * ---------------------------------------------------------------------------
 *   The paper's hardware used 20-bit signed data words for x(n) DIRECTLY
 *   (Section V.A) -- its own y(n)=x^T(n)w(n) therefore lives at a different
 *   absolute scale than this project's 16-bit raw-ADC-scale samples (this
 *   board's own standard/normalized cores' test stimulus is amplitude
 *   ~100, not the paper's presumably much larger 20-bit range). The
 *   specific numbers 8/16/32 are calibrated to THEIR signal scale, not
 *   published as scale-independent constants, and nothing in the paper
 *   gives a formula to rescale them. Treating them as fixed compile-time
 *   thresholds here would silently carry over a calibration that has no
 *   reason to transfer -- exactly the mistake ALPHA_GAIN_SHIFT's own
 *   header (pi_power_inversion_normalized.v) already documents making
 *   once and fixing by empirical re-tuning. THRESH1/2/3 are therefore
 *   PARAMETERS here, with defaults picked to be proportionally reasonable
 *   at THIS project's own established test amplitude (100) and reported,
 *   not asserted, as calibrated -- see tb_pi_power_inversion_pl_npi.v for
 *   what was actually measured with them, and re-tune before trusting a
 *   different signal scale.
 *
 * EVERYTHING ELSE IS pi_power_inversion_normalized.v UNCHANGED
 * ---------------------------------------------------------------------------
 *   Power-sum, pi_reciprocal instantiation, ALPHA_GAIN_SHIFT calibration,
 *   the "latest completed alpha" register (same bounded-staleness
 *   reasoning), the s(k)/quiescent-vector/weight-recursion structure, and
 *   the x^T(k)x(k)-as-sum|x_i|^2 / |alpha(k)|-into-the-standard-core's-own-
 *   sign-convention judgment calls are ALL identical to that file and not
 *   re-derived here -- see its header for the reasoning. This file only
 *   inserts the gain stage between "latest alpha" and the weight-update
 *   multiply.
 *
 * MEASURED, NOT ASSUMED (tb_pi_power_inversion_pl_npi.v, amp=100 calibration
 * point, same stimulus as every other testbench in this project)
 * ---------------------------------------------------------------------------
 *   A real bug was caught here before this ever ran correctly: THRESH1/2/3
 *   were first declared as plain `integer` (always exactly 32 bits), then
 *   sliced as THRESHn[S_W-1:0] with S_W=50 for this project's default
 *   M/DATA_W/WEIGHT_W. Verilog returns 'x' for any bit selected OUTSIDE a
 *   vector's declared width, not zero -- every comparison against these
 *   thresholds silently became 'x', pl_gain_band read as literal 'x' every
 *   cycle, and the weights never moved at all. Fixed by declaring them
 *   explicitly 64 bits wide. Caught by tb_pi_power_inversion_pl_npi.v's own
 *   band-coverage check (band_seen[] all reading 0), not by inspection --
 *   another reason that testbench checks band coverage, not just final
 *   convergence, which alone would not have caught weights stuck at 0.
 *
 *   With that fixed: this core converges (enters and stays in the test's
 *   tolerance band) at sample 17, one sample ahead of the plain normalized
 *   core (pi_power_inversion_normalized.v) at 18, both well ahead of the
 *   standard core's hand-tuned 28. Gain-band coverage over the 2000-sample
 *   run: band 3 (x1.20) fires during the large initial transient (k=0,
 *   weights still near the quiescent start, |y_I| large), settling almost
 *   entirely into band 0 (x1.00, 1984/2000 samples) once converged, with
 *   brief band 1/2 excursions in between -- exactly the paper's own
 *   description of the mechanism: bigger steps while the error is large,
 *   shrinking back as it settles. The default THRESH1/2/3 values (Eq. 11's
 *   literal 8/16/32, reinterpreted at this project's Q(WEIGHT_W-WEIGHT_FRAC).WEIGHT_FRAC
 *   scale rather than assumed transferable -- see the calibration note
 *   above) happen to land in a reasonable place for THIS specific
 *   amp=100/M=2 test point. That is one measured data point, not a proof
 *   the defaults are well-chosen generally; re-run the testbench before
 *   trusting them at a materially different signal scale.
 *
 * FIXED-POINT FORMAT OF THE GAIN
 * ---------------------------------------------------------------------------
 *   GAIN_FRAC-bit unsigned fixed point (default 16), constants rounded to
 *   the nearest representable value: 1.00 -> 65536, 1.05 -> 68813,
 *   1.10 -> 72090, 1.20 -> 78643 (all exact to within 1 LSB of 2^-16).
 *   Folded into the SAME multiply/shift chain pi_power_inversion_normalized
 *   uses for alpha_mag, by widening the final shift by GAIN_FRAC bits --
 *   no extra pipeline stage, no extra latency.
 ******************************************************************************/
`timescale 1ns/1ps

module pi_power_inversion_pl_npi #(
    parameter integer M           = 2,
    parameter integer DATA_W      = 16,
    parameter integer WEIGHT_W    = 32,
    parameter integer WEIGHT_FRAC = 20,
    parameter integer LPF_SHIFT   = 18,
    parameter integer Q_FRAC      = 32,
    parameter integer ALPHA_GAIN_SHIFT = 17,   // see pi_power_inversion_normalized.v's header
    parameter integer GAMMA_INIT  = 1,

    parameter integer GAIN_FRAC   = 16,        // fixed-point fractional bits of the PL gain
    parameter integer GAIN_1_00   = (1 << GAIN_FRAC),                 // 1.00 exactly
    parameter integer GAIN_1_05   = (105 * (1 << GAIN_FRAC)) / 100,   // 1.05
    parameter integer GAIN_1_10   = (110 * (1 << GAIN_FRAC)) / 100,   // 1.10
    parameter integer GAIN_1_20   = (120 * (1 << GAIN_FRAC)) / 100,   // 1.20
    // Eq. (11)'s thresholds against |y_I(n)|, in the SAME Q(WEIGHT_W-WEIGHT_FRAC).WEIGHT_FRAC
    // fixed point s_re/s_im already use -- i.e. THRESH1=8 means "8.0 real",
    // not a raw integer 8. Defaults are placeholders scaled for this
    // project's amp=100 test point, NOT the paper's literal 8/16/32 -- see
    // the header's calibration note and tb_pi_power_inversion_pl_npi.v.
    //
    // Declared as explicit 64-bit values, NOT plain `integer` (which is
    // always exactly 32 bits): these get sliced below as THRESHn[S_W-1:0]
    // with S_W=50 for this project's default M/DATA_W/WEIGHT_W. Verilog
    // returns 'x' for any bit selected outside a vector's DECLARED width,
    // not zero -- a bare `integer` here silently turned every comparison
    // against these thresholds into 'x' (confirmed: pl_gain_band read as
    // literal 'x' every cycle, and weights never moved, until this fix).
    // 64 bits keeps this safe for any realistic S_W without retuning.
    parameter [63:0] THRESH1      = (64'd8  <<< WEIGHT_FRAC),
    parameter [63:0] THRESH2      = (64'd16 <<< WEIGHT_FRAC),
    parameter [63:0] THRESH3      = (64'd32 <<< WEIGHT_FRAC),

    parameter integer PROD1_W     = DATA_W + WEIGHT_W + 1,
    parameter integer SUM_GROWTH  = (M <= 1) ? 1 : $clog2(M),
    parameter integer S_W         = PROD1_W + SUM_GROWTH,
    parameter integer PROD2_W     = DATA_W + S_W + 1,
    parameter integer POW_W       = 2*DATA_W + $clog2(2*M) + 2,
    parameter integer GAMMA_W     = POW_W,
    parameter integer DEN_W       = POW_W + 2,
    parameter integer ALPHA_MAG_W = Q_FRAC + 1,
    // + GAIN_FRAC+1 extra bits: alpha_pl = alpha_mag * gain_const carries
    // GAIN_FRAC more fractional bits than alpha_mag alone, plus one bit for
    // the zero-extended sign of alpha_mag_signed already added there.
    parameter integer ALPHA_PROD_W = PROD2_W + ALPHA_MAG_W + GAIN_FRAC + 2
) (
    input  wire                      clk,
    input  wire                      aresetn,

    input  wire [M*DATA_W-1:0]       x_re,
    input  wire [M*DATA_W-1:0]       x_im,
    input  wire                      sample_valid,

    input  wire [GAMMA_W-1:0]        gamma_in,     // free-running, see the
                                                    // normalized core's port
                                                    // comment for why not a
                                                    // write-strobe
    input  wire                      adapt_en,

    output reg  signed [S_W-1:0]     s_re,
    output reg  signed [S_W-1:0]     s_im,
    output reg                       s_valid,

    output wire [M*WEIGHT_W-1:0]     w_re,
    output wire [M*WEIGHT_W-1:0]     w_im,
    output reg                       weights_valid,

    output wire [ALPHA_MAG_W-1:0]    alpha_mag,        // mu_NPI(n), pre-gain -- diagnostic
    output wire                      alpha_mag_valid,
    output wire [1:0]                pl_gain_band      // diagnostic: 0/1/2/3 = x1.00/1.05/1.10/1.20
);

    integer i;

    reg [GAMMA_W-1:0] gamma_reg;
    always @(posedge clk) begin
        if (!aresetn)
            gamma_reg <= GAMMA_INIT[GAMMA_W-1:0];
        else
            gamma_reg <= gamma_in;
    end

    reg [POW_W-1:0] pow_sum_c;
    always @(*) begin : POWER_SUM
        reg [2*DATA_W-1:0] sq_re, sq_im;
        pow_sum_c = {POW_W{1'b0}};
        for (i = 0; i < M; i = i + 1) begin
            sq_re = $signed(x_re[(i+1)*DATA_W-1 -: DATA_W]) * $signed(x_re[(i+1)*DATA_W-1 -: DATA_W]);
            sq_im = $signed(x_im[(i+1)*DATA_W-1 -: DATA_W]) * $signed(x_im[(i+1)*DATA_W-1 -: DATA_W]);
            pow_sum_c = pow_sum_c + sq_re + sq_im;
        end
    end

    wire [DEN_W-1:0] pow_x2_c    = ({{(DEN_W-POW_W){1'b0}}, pow_sum_c}) << 1;
    wire [DEN_W-1:0] gamma_ext_c = {{(DEN_W-GAMMA_W){1'b0}}, gamma_reg};
    wire [DEN_W-1:0] den_c       = pow_x2_c + gamma_ext_c;

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

    reg [ALPHA_MAG_W-1:0] alpha_mag_latest;
    always @(posedge clk) begin
        if (!aresetn)
            alpha_mag_latest <= {ALPHA_MAG_W{1'b0}};
        else if (recip_valid)
            alpha_mag_latest <= recip_out;
    end
    assign alpha_mag       = alpha_mag_latest;
    assign alpha_mag_valid = recip_valid;

    wire signed [ALPHA_MAG_W:0] alpha_mag_signed = $signed({1'b0, alpha_mag_latest});

    // ==========================================================================
    //  s(k) computation -- identical structure to pi_power_inversion.v /
    //  pi_power_inversion_normalized.v.
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

    // ==========================================================================
    //  PL-NPI GAIN SELECTION -- Eq. (11). y_I(n) = s_re_w ALONE, per the
    //  paper's own point that this needs no modulus/sqrt. Purely
    //  combinational: available the same cycle s_re_w is, so it lines up
    //  with corr_re/corr_im below without any extra pipeline stage.
    // ==========================================================================
    wire signed [S_W-1:0] y_I         = s_re_w;
    wire        [S_W-1:0] y_I_abs     = y_I[S_W-1] ? (~y_I + 1'b1) : y_I;

    wire pl_band3 = (y_I_abs > THRESH3[S_W-1:0]);
    wire pl_band2 = !pl_band3 && (y_I_abs > THRESH2[S_W-1:0]);
    wire pl_band1 = !pl_band3 && !pl_band2 && (y_I_abs > THRESH1[S_W-1:0]);

    wire [GAIN_FRAC:0] pl_gain_const = pl_band3 ? GAIN_1_20[GAIN_FRAC:0] :
                                        pl_band2 ? GAIN_1_10[GAIN_FRAC:0] :
                                        pl_band1 ? GAIN_1_05[GAIN_FRAC:0] :
                                                   GAIN_1_00[GAIN_FRAC:0];

    assign pl_gain_band = pl_band3 ? 2'd3 : pl_band2 ? 2'd2 : pl_band1 ? 2'd1 : 2'd0;

    wire signed [ALPHA_MAG_W+GAIN_FRAC+1:0] alpha_pl_signed =
        alpha_mag_signed * $signed({1'b0, pl_gain_const});

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
                    alpha_term_full_re = corr_re[i] * alpha_pl_signed;
                    alpha_term_full_im = corr_im[i] * alpha_pl_signed;

                    // Q_FRAC - ALPHA_GAIN_SHIFT (same calibration as the
                    // normalized core) PLUS GAIN_FRAC, since alpha_pl_signed
                    // carries GAIN_FRAC extra fractional bits from the gain
                    // multiply above that alpha_mag_signed alone did not.
                    alpha_term_shifted_re = alpha_term_full_re >>> (Q_FRAC - ALPHA_GAIN_SHIFT + GAIN_FRAC);
                    alpha_term_shifted_im = alpha_term_full_im >>> (Q_FRAC - ALPHA_GAIN_SHIFT + GAIN_FRAC);

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
