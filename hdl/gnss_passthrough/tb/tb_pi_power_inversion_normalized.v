`timescale 1ns/1ps
// ============================================================================
//  tb_pi_power_inversion_normalized.v
//
//  Validates the paper's actual headline claim (Fig. 5 of Meng/Feng/Lu):
//  the normalized PI algorithm converges MUCH faster than the standard PI
//  algorithm, even against the standard algorithm's own fast-alpha case.
//
//  Runs THREE cores in parallel against the IDENTICAL stimulus (same
//  jammer-like plane wave used by tb_pi_power_inversion.v, so this is an
//  apples-to-apples comparison against an already-trusted reference, not a
//  new/different test scenario):
//    A. pi_power_inversion,            alpha = 1    (paper's slow case)
//    B. pi_power_inversion,            alpha = 100  (paper's fast case)
//    C. pi_power_inversion_normalized, gamma = 1
//
//  and reports the first sample index each one's |w[0]| enters -- and then
//  STAYS within -- a tolerance band around the known converged value, so
//  "converges faster" is a measured sample count per core, not a visual
//  impression from a waveform.
// ============================================================================
module tb_pi_power_inversion_normalized;

    localparam M           = 2;
    localparam DATA_W      = 16;
    localparam WEIGHT_W    = 32;
    localparam WEIGHT_FRAC = 20;
    localparam ALPHA_W     = 16;
    localparam ALPHA_FRAC  = 8;
    localparam LPF_SHIFT   = 18;      // identical tau approximation, all three cores
    localparam Q_FRAC      = 32;
    localparam GAMMA_INIT  = 1;

    localparam N_SAMPLES   = 2000;
    localparam PRINT_EVERY = 100;
    localparam real AMP        = 100.0;
    localparam real ANGLE_STEP = 0.7;
    localparam real PHASE_STEP = 0.31;
    localparam real WSCALE     = 2.0 ** WEIGHT_FRAC;

    // Convergence band -- centred on the value already established (by
    // tb_pi_power_inversion.v and tb_gnss_passthrough.v) as this exact
    // stimulus's converged |w[0]|, ~0.4996-0.5.
    localparam real CONV_TARGET = 0.5;
    localparam real CONV_TOL    = 0.05;   // +/-10% of target

    reg clk = 0; always #5 clk = ~clk;
    reg aresetn = 0;

    reg  [M*DATA_W-1:0] x_re, x_im;
    reg                  sample_valid;
    reg                  adapt_en;

    // ---- A: standard core, alpha = 1 (paper's slow case) ------------------
    localparam S_W = (DATA_W+WEIGHT_W+1) + 1;
    wire signed [S_W-1:0] a_s_re, a_s_im;
    wire a_s_valid, a_weights_valid;
    wire [M*WEIGHT_W-1:0] a_w_re, a_w_im;

    pi_power_inversion #(
        .M(M), .DATA_W(DATA_W), .WEIGHT_W(WEIGHT_W), .WEIGHT_FRAC(WEIGHT_FRAC),
        .ALPHA_W(ALPHA_W), .ALPHA_FRAC(ALPHA_FRAC), .ALPHA_INIT(1 << ALPHA_FRAC),
        .LPF_SHIFT(LPF_SHIFT)
    ) dut_a (
        .clk(clk), .aresetn(aresetn),
        .x_re(x_re), .x_im(x_im), .sample_valid(sample_valid),
        .alpha_in({ALPHA_W{1'b0}}), .alpha_wr(1'b0), .adapt_en(adapt_en),
        .s_re(a_s_re), .s_im(a_s_im), .s_valid(a_s_valid),
        .w_re(a_w_re), .w_im(a_w_im), .weights_valid(a_weights_valid)
    );

    // ---- B: standard core, alpha = 100 (paper's fast case) ---------------
    wire signed [S_W-1:0] b_s_re, b_s_im;
    wire b_s_valid, b_weights_valid;
    wire [M*WEIGHT_W-1:0] b_w_re, b_w_im;

    pi_power_inversion #(
        .M(M), .DATA_W(DATA_W), .WEIGHT_W(WEIGHT_W), .WEIGHT_FRAC(WEIGHT_FRAC),
        .ALPHA_W(ALPHA_W), .ALPHA_FRAC(ALPHA_FRAC), .ALPHA_INIT(100 << ALPHA_FRAC),
        .LPF_SHIFT(LPF_SHIFT)
    ) dut_b (
        .clk(clk), .aresetn(aresetn),
        .x_re(x_re), .x_im(x_im), .sample_valid(sample_valid),
        .alpha_in({ALPHA_W{1'b0}}), .alpha_wr(1'b0), .adapt_en(adapt_en),
        .s_re(b_s_re), .s_im(b_s_im), .s_valid(b_s_valid),
        .w_re(b_w_re), .w_im(b_w_im), .weights_valid(b_weights_valid)
    );

    // ---- C: normalized core, gamma = 1 -------------------------------------
    localparam POW_W       = 2*DATA_W + 2 + 2;   // matches the module's own $clog2(2*M)+2 for M=2
    localparam GAMMA_W     = POW_W;
    wire signed [S_W-1:0] c_s_re, c_s_im;
    wire c_s_valid, c_weights_valid;
    wire [M*WEIGHT_W-1:0] c_w_re, c_w_im;
    wire [Q_FRAC:0] c_alpha_mag;
    wire c_alpha_mag_valid;

    pi_power_inversion_normalized #(
        .M(M), .DATA_W(DATA_W), .WEIGHT_W(WEIGHT_W), .WEIGHT_FRAC(WEIGHT_FRAC),
        .LPF_SHIFT(LPF_SHIFT), .Q_FRAC(Q_FRAC), .GAMMA_INIT(GAMMA_INIT)
    ) dut_c (
        .clk(clk), .aresetn(aresetn),
        .x_re(x_re), .x_im(x_im), .sample_valid(sample_valid),
        .gamma_in({{(GAMMA_W-1){1'b0}}, 1'b1}), .adapt_en(adapt_en),   // = GAMMA_INIT (1), sized explicitly
        .s_re(c_s_re), .s_im(c_s_im), .s_valid(c_s_valid),
        .w_re(c_w_re), .w_im(c_w_im), .weights_valid(c_weights_valid),
        .alpha_mag(c_alpha_mag), .alpha_mag_valid(c_alpha_mag_valid)
    );

    initial begin
        $dumpfile("waveform_normalized.vcd");
        $dumpvars(0, tb_pi_power_inversion_normalized);
    end

    real jammer_phase;
    integer sample_count, i;
    real xk_re [0:M-1];
    real xk_im [0:M-1];

    real a_w0_mag, b_w0_mag, c_w0_mag;
    integer a_conv_k, b_conv_k, c_conv_k;   // -1 until converged-and-stayed
    integer a_first_in_band, b_first_in_band, c_first_in_band;

    function real w0_mag_real(input [WEIGHT_W-1:0] w_re_bus);
        real re;
        begin
            re = $itor($signed(w_re_bus[WEIGHT_W-1:0])) / WSCALE;
            w0_mag_real = (re < 0.0) ? -re : re;   // w[0]'s imaginary part is
                                                    // ~0 by construction (w_o's
                                                    // imag part is 0 and the
                                                    // adaptive part stays small
                                                    // for this stimulus), so
                                                    // |w[0]| ~= |Re{w[0]}| is a
                                                    // fine convergence proxy --
                                                    // avoids a second $signed
                                                    // slice+sqrt per core here.
        end
    endfunction

    initial begin
        jammer_phase = 0.0;
        x_re = 0; x_im = 0; sample_valid = 0; adapt_en = 1;
        a_conv_k = -1; b_conv_k = -1; c_conv_k = -1;
        a_first_in_band = -1; b_first_in_band = -1; c_first_in_band = -1;

        $display("=========================================================");
        $display(" NORMALIZED vs. STANDARD PI -- CONVERGENCE SPEED");
        $display("=========================================================");
        $display(" A: standard,   alpha=1    (paper's slow case)");
        $display(" B: standard,   alpha=100  (paper's fast case)");
        $display(" C: normalized, gamma=1    (Eq. 13)");
        $display(" Convergence band: |w[0]| in [%.4f, %.4f]", CONV_TARGET-CONV_TOL, CONV_TARGET+CONV_TOL);
        $display(" Same stimulus as tb_pi_power_inversion.v: amp=%.0f, angle_step=%.2f, phase_step=%.2f",
                  AMP, ANGLE_STEP, PHASE_STEP);
        $display("=========================================================\n");

        repeat (4) @(posedge clk);
        aresetn = 1;
        @(posedge clk);

        $display("%-6s %-12s %-12s %-12s", "k", "A |w0|", "B |w0|", "C |w0|");
        $display("-------------------------------------------------");

        for (sample_count = 0; sample_count < N_SAMPLES; sample_count = sample_count + 1) begin
            for (i = 0; i < M; i = i + 1) begin
                xk_re[i] = AMP * $cos(jammer_phase + i*ANGLE_STEP);
                xk_im[i] = AMP * $sin(jammer_phase + i*ANGLE_STEP);
                x_re[(i+1)*DATA_W-1 -: DATA_W] = $rtoi(xk_re[i]);
                x_im[(i+1)*DATA_W-1 -: DATA_W] = $rtoi(xk_im[i]);
            end
            jammer_phase = jammer_phase + PHASE_STEP;

            sample_valid = 1;
            @(posedge clk);
            sample_valid = 0;
            @(posedge clk);
            @(posedge clk);

            a_w0_mag = w0_mag_real(a_w_re[WEIGHT_W-1:0]);
            b_w0_mag = w0_mag_real(b_w_re[WEIGHT_W-1:0]);
            c_w0_mag = w0_mag_real(c_w_re[WEIGHT_W-1:0]);

            // "converged AND stayed" -- once out of band, forget any
            // earlier entry and keep looking; only the LAST unbroken run
            // that reaches N_SAMPLES-1 counts.
            if (a_w0_mag < CONV_TARGET-CONV_TOL || a_w0_mag > CONV_TARGET+CONV_TOL)
                a_first_in_band = -1;
            else if (a_first_in_band == -1)
                a_first_in_band = sample_count;

            if (b_w0_mag < CONV_TARGET-CONV_TOL || b_w0_mag > CONV_TARGET+CONV_TOL)
                b_first_in_band = -1;
            else if (b_first_in_band == -1)
                b_first_in_band = sample_count;

            if (c_w0_mag < CONV_TARGET-CONV_TOL || c_w0_mag > CONV_TARGET+CONV_TOL)
                c_first_in_band = -1;
            else if (c_first_in_band == -1)
                c_first_in_band = sample_count;

            if (sample_count % PRINT_EVERY == 0) begin
                $display("%-6d %-12.5f %-12.5f %-12.5f", sample_count, a_w0_mag, b_w0_mag, c_w0_mag);
            end
        end

        a_conv_k = a_first_in_band;
        b_conv_k = b_first_in_band;
        c_conv_k = c_first_in_band;

        $display("\n=========================================================");
        $display(" CONVERGENCE RESULT (sample index first entering the band");
        $display(" AND remaining in it through sample %0d; -1 = never settled)", N_SAMPLES-1);
        $display("=========================================================");
        $display(" A: standard   alpha=1    -> k = %0d", a_conv_k);
        $display(" B: standard   alpha=100  -> k = %0d", b_conv_k);
        $display(" C: normalized gamma=1    -> k = %0d", c_conv_k);

        if (c_conv_k >= 0 && a_conv_k >= 0 && c_conv_k < a_conv_k) begin
            $display("\nPASS: normalized converged before standard alpha=1 (%0d < %0d samples)", c_conv_k, a_conv_k);
        end else begin
            $display("\nFAIL: normalized did NOT converge faster than standard alpha=1 (%0d vs %0d)", c_conv_k, a_conv_k);
        end

        if (c_conv_k >= 0 && b_conv_k >= 0 && c_conv_k <= b_conv_k) begin
            $display("PASS: normalized converged at least as fast as standard alpha=100 (%0d <= %0d samples)", c_conv_k, b_conv_k);
        end else begin
            $display("FAIL: normalized did NOT match/beat standard alpha=100 (%0d vs %0d)", c_conv_k, b_conv_k);
        end

        $display("\nWaveform written to waveform_normalized.vcd");
        $display("TESTBENCH DONE");
        $finish;
    end

endmodule
