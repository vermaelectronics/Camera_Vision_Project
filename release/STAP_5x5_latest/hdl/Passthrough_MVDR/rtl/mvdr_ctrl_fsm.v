// -----------------------------------------------------------------------------
// mvdr_ctrl_fsm.v  --  MVDR control: what weights the beamformer uses, and when
//                      v2: jammer-aware weight HOLD and GLIDE (carrier continuity)
//
// Sits between  cov_prep -> mvdr_weight_calc -> beamformer_2ch.
//
// MODE (must already be in this clock domain)
//   0  bypass  : weights = [1, 0]  -> output is RX1 alone
//   1  adaptive: MVDR weights from mvdr_weight_calc, through the policy below
//   2  fixed   : weights = fix_w* (software supplied)
//   3  reserved: treated as bypass
//
// ADAPTIVE WEIGHT POLICY (v2)
//   Every weight run carries cov_prep's jammer flag for its window
//   (coherence |r12|^2/(r11 r22) > 2^-coh_sh). For each finished run:
//     jam            -> apply the new set AT ONCE (null forms within one hop)
//                       and re-arm the hold timer: hold_cnt = hold_units * 64
//     !jam, hold > 0 -> HOLD: keep the current set, hold_cnt -= 1
//     !jam, hold = 0 -> GLIDE: w += (w_new - w) / 2^rel_sh  (16 fraction bits)
//
//   Why (model/lock_study.py, 24 satellite directions, Costas PLL BL 15 Hz,
//   1 ms integration, JNR 45 dB):
//     * a jammer that switches off and on again (pulsed, intermittent)
//       made v1 drop the null and re-form it every cycle: every satellite's
//       carrier phase toggled and every re-onset leaked the full jammer for a
//       hop. 1 ms on/off: 509 PLL half-cycle slips in 1.3 s -> 28 with v2;
//       carrier error on satellites away from the jammer 13.9 -> 3.6 deg rms.
//     * when a jammer leaves, v1 jumped back to [1,0] in one step (a carrier
//       phase step on every satellite). v2 holds, then glides with a time
//       constant 2^rel_sh hops (default 2^14 x 18.6 us = 0.3 s), slow enough
//       for any tracking loop (max error 10 deg vs 13-26 deg).
//     * no jammer: weights converge on the unjammed solution (~[1,0]) and are
//       averaged over 2^rel_sh hops: practically zero weight jitter.
//     * holding a null with no jammer present costs 0 dB averaged over
//       directions (+ for most satellites, - only near the jammer's direction),
//       which is why a generous default hold (0.3 s) is safe.
//   hold_units = 0 and rel_sh = 0 reproduce v1 exactly (every set applied).
//
//   Entering adaptive mode starts from [1,0] with the hold timer cleared.
//   `freeze` stops all updates (the set in use holds; runs in flight are
//   discarded). In fixed mode a change of fix_w* reloads them.
//
// LOAD / SWAP
//   The weights are presented with bf_w_load = 1 (bf_swap one clock later), so
//   the whole set becomes active on one edge. A HOLD issues no load.
//
// TIMING NOTES
//   * wc_start is combinational from prep_valid (the calculator latches its
//     inputs on that clock; cov_prep holds them anyway since v2).
//   * The glide is a 3-stage pipeline (difference, shift-add, round); the
//     load is issued 3 clocks after wc_valid. All bf_* outputs are registered.
//   * A mode change or fixed-weight change takes priority over a glide still
//     in the pipeline (that glide result is dropped).
//
// STATUS (32-bit counters, cleared by cnt_clear)
//   n_blocks  covariance windows seen        n_zero  all-zero R windows (no update)
//   n_upd     weight sets applied            n_skip  runs skipped by the calculator (d<=0)
//   n_sat     applied sets that were scaled  n_drop  starts dropped (calculator busy)
//   n_jam     runs with the jammer flag set  n_hold  runs absorbed by a hold
//   st_have_w   adaptive weights applied since entering adaptive mode
//   st_last_sat / st_last_skip   outcome of the most recent run
//   st_jam / st_hold / st_glide  policy state of the most recent run
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module mvdr_ctrl_fsm #(
    parameter W_W    = 18,
    parameter W_FRAC = 15,
    parameter GF     = 16          // glide fraction bits
)(
    input  wire                     clk,
    input  wire                     rst,            // synchronous, active high

    // configuration (already synchronised to clk)
    input  wire [1:0]               mode,
    input  wire                     freeze,
    input  wire signed [W_W-1:0]    fix_w1_re,
    input  wire signed [W_W-1:0]    fix_w1_im,
    input  wire signed [W_W-1:0]    fix_w2_re,
    input  wire signed [W_W-1:0]    fix_w2_im,
    input  wire                     cnt_clear,
    input  wire [15:0]              hold_units,     // hold = hold_units * 64 hops (0 = none)
    input  wire [3:0]               rel_sh,         // glide time constant 2^rel_sh hops (0 = none)

    // from cov_prep
    input  wire                     prep_valid,
    input  wire                     prep_zero,
    input  wire                     prep_jam,

    // mvdr_weight_calc
    output wire                     wc_start,
    input  wire                     wc_valid,
    input  wire                     wc_skip,
    input  wire                     wc_sat,
    input  wire                     wc_dropped,
    input  wire signed [W_W-1:0]    wc_w1_re,
    input  wire signed [W_W-1:0]    wc_w1_im,
    input  wire signed [W_W-1:0]    wc_w2_re,
    input  wire signed [W_W-1:0]    wc_w2_im,

    // beamformer_2ch weight port
    output reg                      bf_w_load,
    output reg                      bf_swap,
    output reg  signed [W_W-1:0]    bf_w1_re,
    output reg  signed [W_W-1:0]    bf_w1_im,
    output reg  signed [W_W-1:0]    bf_w2_re,
    output reg  signed [W_W-1:0]    bf_w2_im,

    // status
    output reg  [31:0]              n_blocks,
    output reg  [31:0]              n_upd,
    output reg  [31:0]              n_skip,
    output reg  [31:0]              n_sat,
    output reg  [31:0]              n_drop,
    output reg  [31:0]              n_zero,
    output reg  [31:0]              n_jam,
    output reg  [31:0]              n_hold,
    output reg                      st_have_w,
    output reg                      st_last_sat,
    output reg                      st_last_skip,
    output reg                      st_jam,
    output wire                     st_hold,
    output reg                      st_glide,
    output wire [1:0]               st_mode
);

    localparam signed [W_W-1:0] W_ONE  = (1 <<< W_FRAC);
    localparam signed [W_W-1:0] W_ZERO = {W_W{1'b0}};
    localparam AW = W_W + GF + 1;                       // glide accumulator (signed)
    localparam signed [AW-1:0] A_HALF = {{(AW-1){1'b0}}, 1'b1} << (GF - 1);

    reg [1:0]              mode_q;
    reg signed [W_W-1:0]   fq_w1r, fq_w1i, fq_w2r, fq_w2i;      // fixed weights last loaded

    wire m_adapt  = (mode == 2'd1);
    wire m_fixed  = (mode == 2'd2);
    wire mode_chg = (mode != mode_q);

    wire fix_chg  = m_fixed && !mode_chg &&
                    ((fix_w1_re != fq_w1r) || (fix_w1_im != fq_w1i) ||
                     (fix_w2_re != fq_w2r) || (fix_w2_im != fq_w2i));

    wire adapt_ok = m_adapt && !freeze && !mode_chg;

    assign wc_start = prep_valid & ~prep_zero & adapt_ok;
    wire   take_wc  = wc_valid & adapt_ok;

    assign st_mode  = mode_q;

    // ---------------- policy state ----------------
    reg                    run_jam;                 // jammer flag of the run in flight
    reg [21:0]             hold_cnt;
    assign st_hold = (hold_cnt != 22'd0) && !st_jam;     // jammer gone, null kept

    reg signed [AW-1:0]    acc [0:3];               // glide state, GF fraction bits
    reg signed [AW-1:0]    dif [0:3];
    reg [1:0]              gp;                      // glide pipeline: 1 = diff ready, 2 = acc ready
    reg [3:0]              g_sh;
    reg                    g_sat;

    wire signed [W_W-1:0]  wn [0:3];
    assign wn[0] = wc_w1_re;  assign wn[1] = wc_w1_im;
    assign wn[2] = wc_w2_re;  assign wn[3] = wc_w2_im;

    function signed [AW-1:0] up;                    // weight -> accumulator units
        input signed [W_W-1:0] v;
        begin
            up = {{(AW-W_W-GF){v[W_W-1]}}, v, {GF{1'b0}}};
        end
    endfunction

    // round-half-up of the accumulator to a weight (never overflows: acc is a
    // convex mix of two in-range weights)
    function signed [W_W-1:0] rnd;
        input signed [AW-1:0] a;
        reg   signed [AW-1:0] t;
        begin
            t   = (a + A_HALF) >>> GF;
            rnd = t[W_W-1:0];
        end
    endfunction

    integer k;
    always @(posedge clk) begin
        if (rst) begin
            mode_q      <= 2'd0;
            bf_w_load   <= 1'b0;
            bf_swap     <= 1'b0;
            bf_w1_re    <= W_ONE;   bf_w1_im <= W_ZERO;
            bf_w2_re    <= W_ZERO;  bf_w2_im <= W_ZERO;
            fq_w1r <= W_ZERO; fq_w1i <= W_ZERO; fq_w2r <= W_ZERO; fq_w2i <= W_ZERO;
            n_blocks <= 32'd0; n_upd <= 32'd0; n_skip <= 32'd0;
            n_sat    <= 32'd0; n_drop <= 32'd0; n_zero <= 32'd0;
            n_jam    <= 32'd0; n_hold <= 32'd0;
            st_have_w    <= 1'b0;
            st_last_sat  <= 1'b0;
            st_last_skip <= 1'b0;
            st_jam       <= 1'b0;
            st_glide     <= 1'b0;
            run_jam  <= 1'b0;
            hold_cnt <= 22'd0;
            gp       <= 2'd0;
            acc[0] <= up(W_ONE); acc[1] <= 0; acc[2] <= 0; acc[3] <= 0;
        end else begin
            mode_q    <= mode;
            bf_w_load <= 1'b0;
            bf_swap   <= bf_w_load;              // swap follows load by one clock

            if (wc_start) run_jam <= prep_jam;

            // ---------------- glide pipeline (stages 2, 3) ----------------
            if (gp == 2'd1) begin
                for (k = 0; k < 4; k = k + 1) acc[k] <= acc[k] + (dif[k] >>> g_sh);
                gp <= 2'd2;
            end else if (gp == 2'd2) begin
                gp <= 2'd0;
            end

            // ---------------- weight source ----------------
            if (mode_chg) begin
                bf_w_load <= 1'b1;
                st_have_w <= 1'b0;
                hold_cnt  <= 22'd0;
                gp        <= 2'd0;
                st_jam    <= 1'b0;
                st_glide  <= 1'b0;
                acc[0] <= up(W_ONE); acc[1] <= 0; acc[2] <= 0; acc[3] <= 0;
                if (m_fixed) begin
                    bf_w1_re <= fix_w1_re;  bf_w1_im <= fix_w1_im;
                    bf_w2_re <= fix_w2_re;  bf_w2_im <= fix_w2_im;
                    fq_w1r <= fix_w1_re; fq_w1i <= fix_w1_im;
                    fq_w2r <= fix_w2_re; fq_w2i <= fix_w2_im;
                end else begin
                    bf_w1_re <= W_ONE;   bf_w1_im <= W_ZERO;
                    bf_w2_re <= W_ZERO;  bf_w2_im <= W_ZERO;
                end
            end else if (fix_chg) begin
                bf_w_load <= 1'b1;
                bf_w1_re <= fix_w1_re;  bf_w1_im <= fix_w1_im;
                bf_w2_re <= fix_w2_re;  bf_w2_im <= fix_w2_im;
                fq_w1r <= fix_w1_re; fq_w1i <= fix_w1_im;
                fq_w2r <= fix_w2_re; fq_w2i <= fix_w2_im;
            end else if (take_wc) begin
                st_last_sat  <= wc_sat;
                st_last_skip <= 1'b0;
                st_jam       <= run_jam;
                if (run_jam) begin
                    // jammer: apply at once, re-arm the hold
                    bf_w_load <= 1'b1;
                    bf_w1_re <= wc_w1_re;  bf_w1_im <= wc_w1_im;
                    bf_w2_re <= wc_w2_re;  bf_w2_im <= wc_w2_im;
                    for (k = 0; k < 4; k = k + 1) acc[k] <= up(wn[k]);
                    hold_cnt  <= {hold_units, 6'd0};
                    st_have_w <= 1'b1;
                    st_glide  <= 1'b0;
                    gp        <= 2'd0;
                end else if (hold_cnt != 22'd0) begin
                    // jammer gone (or between pulses): keep the null
                    hold_cnt <= hold_cnt - 22'd1;
                    st_glide <= 1'b0;
                end else begin
                    // glide toward the unjammed solution (stage 1: difference)
                    for (k = 0; k < 4; k = k + 1) dif[k] <= up(wn[k]) - acc[k];
                    g_sh     <= rel_sh;
                    g_sat    <= wc_sat;
                    gp       <= 2'd1;
                    st_glide <= 1'b1;
                end
            end else if (gp == 2'd2) begin
                // glide stage 3: round and load
                bf_w_load <= 1'b1;
                bf_w1_re <= rnd(acc[0]);  bf_w1_im <= rnd(acc[1]);
                bf_w2_re <= rnd(acc[2]);  bf_w2_im <= rnd(acc[3]);
                st_have_w <= 1'b1;
            end

            // ---------------- status ----------------
            if (cnt_clear) begin
                n_blocks <= 32'd0; n_upd <= 32'd0; n_skip <= 32'd0;
                n_sat    <= 32'd0; n_drop <= 32'd0; n_zero <= 32'd0;
                n_jam    <= 32'd0; n_hold <= 32'd0;
            end else begin
                if (prep_valid)                n_blocks <= n_blocks + 32'd1;
                if (prep_valid && prep_zero)   n_zero   <= n_zero   + 32'd1;
                if (take_wc && run_jam)        n_jam    <= n_jam    + 32'd1;
                if (take_wc && !run_jam && hold_cnt != 22'd0)
                                               n_hold   <= n_hold   + 32'd1;
                if (take_wc && (run_jam || hold_cnt == 22'd0))
                                               n_upd    <= n_upd    + 32'd1;
                if (take_wc && wc_sat && (run_jam || hold_cnt == 22'd0))
                                               n_sat    <= n_sat    + 32'd1;
                if (wc_skip)                   n_skip   <= n_skip   + 32'd1;
                if (wc_dropped)                n_drop   <= n_drop   + 32'd1;
            end
            if (wc_skip) st_last_skip <= 1'b1;
        end
    end

endmodule
