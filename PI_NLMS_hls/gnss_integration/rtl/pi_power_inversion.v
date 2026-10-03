/*******************************************************************************
 * pi_power_inversion.v
 *
 * Traditional (non-normalized) Power-Inversion (PI) adaptive-array core.
 *
 * Implements Compton's PI algorithm exactly as given in:
 *   Meng, Feng, Lu -- "Anti-Jamming with Adaptive Arrays Utilizing Power
 *   Inversion Algorithm", Tsinghua Science and Technology, 13(6), 2008.
 *
 * This is the STANDARD PI algorithm (fixed alpha, fixed tau) -- Eq. (2)-(4)
 * of the paper -- not the normalized variant of Section 2 (Eq. 13), which
 * instead computes a time-varying alpha(k) every sample. That's a
 * different, separate algorithm; ask if you want that one built too.
 *
 * DEFAULT M=2, MATCHING THE ACTUAL ANTSDR E310 HARDWARE
 * ---------------------------------------------------------------------------
 * The paper's own simulation uses an 8-element array; this default is
 * deliberately different. This board is a single AD9361 -- 2 RX channels
 * (RX1, RX2) in 2R2T mode, not 8 separate RF chains. With M=2 there's only
 * M-1=1 degree of freedom, so it can null exactly ONE jammer (not multiple)
 * -- but that's real, achievable on THIS hardware: RX1/RX2 already share
 * one axi_ad9361 + one ADC FIFO (Fig.2 blocks 01-02), so this core's 2
 * element inputs can be fed from that existing output with a small repack,
 * not the 8-physical-chain front end the M=8 case needed. M is still a
 * parameter -- override it if a real multi-chain front end exists later.
 *
 * ALGORITHM (paper's equations, mapped to this RTL)
 * ---------------------------------------------------------------------------
 *   Array output           (Eq. 3):
 *       s(k) = sum_i x_i(k) * w_i(k)                       i = 0 .. M-1
 *
 *   Adaptive ("floating") weight update (Eq. 2):
 *       w'_i(k+1) = [tau/(1+tau)] * w'_i(k)
 *                 - [alpha/(1+tau)] * conj(x_i(k)) * s(k)
 *
 *     Rewritten (algebraically identical, hardware-friendly -- avoids a
 *     real divider by turning "/(1+tau)" into a single arithmetic right
 *     shift, since tau is intentionally huge -- 200000 in the paper's own
 *     simulations -- making 1/(1+tau) tiny and well approximated by a
 *     power-of-two LPF_SHIFT):
 *       w'_i(k+1) = w'_i(k) - ((w'_i(k) + alpha*conj(x_i(k))*s(k)) >>> LPF_SHIFT)
 *
 *   Combining weight, offset vector w_o = [1,0,...,0]^T for the
 *   omnidirectional GPS quiescent pattern (Eq. 4):
 *       w_i(k) = w'_i(k) + w_o_i         w_o_0 = 1, w_o_i = 0 for i > 0
 *
 * This is exactly what Fig. 2 of the paper draws per element: a complex
 * conjugate on x_i, a multiply against the array output s(k), a low-pass
 * filter (here: leaky-integrator / EMA, gain 1/(1+tau)) in the feedback
 * path, and the fixed offset vector w_o added back in afterward.
 *
 * FIXED-POINT FORMAT
 * ---------------------------------------------------------------------------
 *   x_i(k)      : DATA_W-bit signed, DATA_FRAC=0 (plain integer ADC
 *                 samples, e.g. int16_t straight off the AD9361 -- matches
 *                 how this project's existing gps_acquire.c already stores
 *                 RX samples; no normalization assumed).
 *   w'_i, w_i   : WEIGHT_W-bit signed, WEIGHT_FRAC fractional bits
 *                 (Q(WEIGHT_W-WEIGHT_FRAC).WEIGHT_FRAC). Needs generous
 *                 fractional width because 1/(1+tau) is tiny.
 *   alpha       : ALPHA_W-bit signed, ALPHA_FRAC fractional bits. The
 *                 paper's convergence test uses alpha=1 and alpha=100 --
 *                 both exactly representable with a few integer bits.
 *   s(k)        : derived width, same WEIGHT_FRAC fractional alignment as
 *                 the weights (x has 0 fractional bits, so the dot product
 *                 inherits the weights' fractional format unchanged).
 *
 * PIPELINE (2 cycles from sample_valid to updated weights)
 * ---------------------------------------------------------------------------
 *   cycle 0 : latch x(k) on sample_valid                        -> x_d0
 *   cycle 1 : s(k) = sum_i x_d0_i * w_i(k)  (w_i = w'_i + w_o_i) -> s_valid
 *             also delay x_d0 -> x_d1 to stay aligned with s(k)
 *   cycle 2 : w'_i(k+1) update using x_d1_i and s(k)             -> weights_valid
 *
 * Reset initializes w'_i = 0 for all i, so the combining weight starts at
 * exactly w_o (omnidirectional quiescent pattern) -- the correct initial
 * condition per the paper ("the offset vector is chosen to provide the
 * desired quiescent antenna pattern").
 ******************************************************************************/
`timescale 1ns/1ps

module pi_power_inversion #(
    parameter integer M           = 2,    // antenna elements -- 2 = this board's RX1/RX2 (2R2T); override for a real multi-chain array
    parameter integer DATA_W      = 16,   // raw ADC I/Q sample width, signed, integer (0 frac bits)
    parameter integer WEIGHT_W    = 32,   // adaptive weight width, signed
    parameter integer WEIGHT_FRAC = 20,   // weight fractional bits
    parameter integer ALPHA_W     = 16,   // alpha register width, signed
    parameter integer ALPHA_FRAC  = 8,    // alpha fractional bits
    parameter integer ALPHA_INIT  = (1 << ALPHA_FRAC), // default alpha = 1.0 (paper's slow-converge case)
    parameter integer LPF_SHIFT   = 18,   // approximates log2(1+tau); tau=200000 -> 1+tau=200001 -> ~17.6

    // Derived widths -- declared here (rather than as localparams after the
    // port list) so they're available to size the s_re/s_im ports below.
    parameter integer PROD1_W = DATA_W + WEIGHT_W + 1,        // x_i * w_i
    parameter integer SUM_GROWTH = (M <= 1) ? 1 : $clog2(M),  // bits of growth summing M products
    parameter integer S_W     = PROD1_W + SUM_GROWTH,         // sum_i x_i*w_i  (= s(k) width)
    parameter integer PROD2_W = DATA_W + S_W + 1,              // conj(x_i) * s(k)
    parameter integer ALPHA_PROD_W = PROD2_W + ALPHA_W         // (conj(x_i)*s(k)) * alpha
) (
    input  wire                      clk,
    input  wire                      aresetn,        // active-low, synchronous

    // ---- per-element complex input samples, flattened bus -----------------
    // element i occupies bits [(i+1)*DATA_W-1 : i*DATA_W]
    input  wire [M*DATA_W-1:0]       x_re,
    input  wire [M*DATA_W-1:0]       x_im,
    input  wire                      sample_valid,   // pulse: x(k) is present this cycle

    // ---- runtime-configurable alpha (paper's fixed PI gain) ---------------
    input  wire signed [ALPHA_W-1:0] alpha_in,
    input  wire                      alpha_wr,       // pulse to load alpha_in into the alpha register

    // ---- freeze adaptation (e.g. once converged, or to force omni-only) ---
    input  wire                      adapt_en,

    // ---- array output s(k) -- exactly S_W bits, the accumulator's real
    //      width (x_i has 0 fractional bits, so this stays in the same
    //      WEIGHT_FRAC fixed-point alignment as the weights) -------------
    output reg  signed [S_W-1:0]     s_re,
    output reg  signed [S_W-1:0]     s_im,
    output reg                       s_valid,

    // ---- weight readback, flattened bus (for beam-pattern computation,
    //      monitoring, or seeding another block) ---------------------------
    output wire [M*WEIGHT_W-1:0]     w_re,
    output wire [M*WEIGHT_W-1:0]     w_im,
    output reg                       weights_valid
);

    integer i;

    // ---- alpha register -----------------------------------------------
    reg signed [ALPHA_W-1:0] alpha_reg;
    always @(posedge clk) begin
        if (!aresetn)
            alpha_reg <= ALPHA_INIT[ALPHA_W-1:0];
        else if (alpha_wr)
            alpha_reg <= alpha_in;
    end

    // ---- w_o_i, the fixed quiescent offset vector [1,0,...,0]^T -------
    // (element 0 real part = 1.0 in WEIGHT_FRAC fixed point; every other
    // tap, and every imaginary part, is exactly 0 -- Eq. 4 / GPS omni
    // quiescent pattern.)
    function signed [WEIGHT_W-1:0] wo_re(input integer idx);
        wo_re = (idx == 0) ? (1 <<< WEIGHT_FRAC) : {WEIGHT_W{1'b0}};
    endfunction

    // ---- adaptive ("floating") weight state, w'_i ----------------------
    reg signed [WEIGHT_W-1:0] wp_re [0:M-1];
    reg signed [WEIGHT_W-1:0] wp_im [0:M-1];

    // combining weight w_i = w'_i + w_o_i, derived combinationally
    wire signed [WEIGHT_W-1:0] w_re_c [0:M-1];
    wire signed [WEIGHT_W-1:0] w_im_c [0:M-1];
    genvar g;
    generate
        for (g = 0; g < M; g = g + 1) begin : G_COMBINE
            assign w_re_c[g] = wp_re[g] + wo_re(g);
            assign w_im_c[g] = wp_im[g];               // w_o imaginary part is always 0
            assign w_re[(g+1)*WEIGHT_W-1 -: WEIGHT_W] = w_re_c[g];
            assign w_im[(g+1)*WEIGHT_W-1 -: WEIGHT_W] = w_im_c[g];
        end
    endgenerate

    // ---- stage 0: latch x(k) on sample_valid ----------------------------
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

    // ---- stage 1: s(k) = sum_i x_i(k) * w_i(k)  (Eq. 3) ------------------
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
                // Accumulate the M per-element products (sign-extended to
                // the accumulator width) into s(k). Plain sequential sum --
                // fine for modest M; pipeline into a proper adder tree if
                // targeting a much larger array / tighter clock.
                //
                // Named block (": ACCUM_S") -- Vivado's xvlog rejects local
                // reg declarations inside an unnamed begin/end even where
                // Icarus Verilog accepts it; naming the block is required,
                // not cosmetic.
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

    // s(k), sliced back down to the S_W width actually produced above
    // (the output ports are kept wider for the consumer's convenience;
    // internally we only need the S_W bits that were meaningfully computed).
    wire signed [S_W-1:0] s_re_w = s_re[S_W-1:0];
    wire signed [S_W-1:0] s_im_w = s_im[S_W-1:0];

    // ---- stage 2: w'_i(k+1) update (Eq. 2, rewritten for a shift-based
    //      leak instead of a real divider by (1+tau)) ----------------------
    wire signed [PROD2_W-1:0] corr_re [0:M-1];
    wire signed [PROD2_W-1:0] corr_im [0:M-1];

    generate
        for (g = 0; g < M; g = g + 1) begin : G_CORR
            pi_cmul #(
                .A_W    (DATA_W),
                .B_W    (S_W),
                .CONJ_A (1)                 // conj(x_i(k)) * s(k)
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
                // Named block (": WEIGHT_UPDATE") -- same Vivado xvlog
                // requirement as ACCUM_S above: local reg declarations need
                // a named block.
                //
                // BUG FIXED HERE: the previous version truncated
                // alpha*conj(x_i)*s(k) down to WEIGHT_W bits immediately
                // after the ALPHA_FRAC shift, before LPF_SHIFT had a chance
                // to bring the magnitude down. That fit by coincidence at
                // small test amplitudes (e.g. amp=20) and silently
                // overflowed/wrapped at larger, more realistic ones (e.g.
                // amp=100), producing exactly the "large erratic weights,
                // no real nulling" symptom seen on hardware/simulation.
                // Fix: keep the full ALPHA_PROD_W width all the way through
                // both shifts (ALPHA_FRAC, then LPF_SHIFT) and truncate to
                // WEIGHT_W only at the very end, once both shifts have
                // actually reduced the magnitude into range.
                reg signed [ALPHA_PROD_W-1:0] alpha_term_full_re;
                reg signed [ALPHA_PROD_W-1:0] alpha_term_full_im;
                reg signed [ALPHA_PROD_W-1:0] alpha_term_shifted_re;
                reg signed [ALPHA_PROD_W-1:0] alpha_term_shifted_im;
                reg signed [ALPHA_PROD_W-1:0] leak_in_re;
                reg signed [ALPHA_PROD_W-1:0] leak_in_im;
                reg signed [ALPHA_PROD_W-1:0] leak_out_re;
                reg signed [ALPHA_PROD_W-1:0] leak_out_im;

                for (i = 0; i < M; i = i + 1) begin
                    // alpha * conj(x_i(k)) * s(k), full width -- no
                    // truncation yet.
                    alpha_term_full_re = corr_re[i] * alpha_reg;
                    alpha_term_full_im = corr_im[i] * alpha_reg;

                    // Realign fixed point by shifting off ALPHA_FRAC bits --
                    // still full width.
                    alpha_term_shifted_re = alpha_term_full_re >>> ALPHA_FRAC;
                    alpha_term_shifted_im = alpha_term_full_im >>> ALPHA_FRAC;

                    // w'_i(k) + alpha*conj(x_i(k))*s(k), wp_re/wp_im
                    // sign-extended up to the full width -- still no
                    // truncation.
                    leak_in_re = {{(ALPHA_PROD_W-WEIGHT_W){wp_re[i][WEIGHT_W-1]}}, wp_re[i]} + alpha_term_shifted_re;
                    leak_in_im = {{(ALPHA_PROD_W-WEIGHT_W){wp_im[i][WEIGHT_W-1]}}, wp_im[i]} + alpha_term_shifted_im;

                    // Apply the LPF_SHIFT leak -- this is what actually
                    // brings the magnitude down into the weight's normal
                    // operating range.
                    leak_out_re = leak_in_re >>> LPF_SHIFT;
                    leak_out_im = leak_in_im >>> LPF_SHIFT;

                    // w'_i(k+1) = w'_i(k) - leak_out -- truncate to
                    // WEIGHT_W bits only now, at the very end.
                    wp_re[i] <= wp_re[i] - leak_out_re[WEIGHT_W-1:0];
                    wp_im[i] <= wp_im[i] - leak_out_im[WEIGHT_W-1:0];
                end
            end
        end
    end

endmodule
