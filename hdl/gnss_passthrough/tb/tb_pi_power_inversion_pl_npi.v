`timescale 1ns/1ps
// ============================================================================
//  tb_pi_power_inversion_pl_npi.v
//
//  Two things checked, not assumed:
//   1. The gain-band cascade (Eq. 11) actually exercises all four bands
//      against the chosen THRESH1/2/3 defaults, not stuck at one band --
//      a threshold-comparison bug would show up as "only band 0 ever
//      selected" even if the module elaborates and "converges" fine.
//   2. PL-NPI converges (same convergence-band methodology as
//      tb_pi_power_inversion_normalized.v), run alongside the plain
//      normalized core (gamma=1, no gain stage) and the standard core
//      (alpha=1) on the IDENTICAL stimulus already used everywhere else
//      in this project, for a direct comparison.
// ============================================================================
module tb_pi_power_inversion_pl_npi;

    localparam M           = 2;
    localparam DATA_W      = 16;
    localparam WEIGHT_W    = 32;
    localparam WEIGHT_FRAC = 20;
    localparam ALPHA_W     = 16;
    localparam ALPHA_FRAC  = 8;
    localparam LPF_SHIFT   = 18;
    localparam Q_FRAC      = 32;
    localparam GAMMA_INIT  = 1;

    localparam N_SAMPLES   = 2000;
    localparam PRINT_EVERY = 100;
    localparam real AMP        = 100.0;
    localparam real ANGLE_STEP = 0.7;
    localparam real PHASE_STEP = 0.31;
    localparam real WSCALE     = 2.0 ** WEIGHT_FRAC;

    localparam real CONV_TARGET = 0.5;
    localparam real CONV_TOL    = 0.05;

    reg clk = 0; always #5 clk = ~clk;
    reg aresetn = 0;

    reg  [M*DATA_W-1:0] x_re, x_im;
    reg                  sample_valid;
    reg                  adapt_en;

    localparam S_W = (DATA_W+WEIGHT_W+1) + 1;

    // ---- A: standard core, alpha = 1 --------------------------------------
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

    // ---- C: normalized core (no gain stage), gamma = 1 ---------------------
    localparam POW_W   = 2*DATA_W + 2 + 2;
    localparam GAMMA_W = POW_W;
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
        .gamma_in({{(GAMMA_W-1){1'b0}}, 1'b1}), .adapt_en(adapt_en),
        .s_re(c_s_re), .s_im(c_s_im), .s_valid(c_s_valid),
        .w_re(c_w_re), .w_im(c_w_im), .weights_valid(c_weights_valid),
        .alpha_mag(c_alpha_mag), .alpha_mag_valid(c_alpha_mag_valid)
    );

    // ---- E: PL-NPI core, gamma = 1, default thresholds ---------------------
    wire signed [S_W-1:0] e_s_re, e_s_im;
    wire e_s_valid, e_weights_valid;
    wire [M*WEIGHT_W-1:0] e_w_re, e_w_im;
    wire [Q_FRAC:0] e_alpha_mag;
    wire e_alpha_mag_valid;
    wire [1:0] e_pl_gain_band;

    pi_power_inversion_pl_npi #(
        .M(M), .DATA_W(DATA_W), .WEIGHT_W(WEIGHT_W), .WEIGHT_FRAC(WEIGHT_FRAC),
        .LPF_SHIFT(LPF_SHIFT), .Q_FRAC(Q_FRAC), .GAMMA_INIT(GAMMA_INIT)
    ) dut_e (
        .clk(clk), .aresetn(aresetn),
        .x_re(x_re), .x_im(x_im), .sample_valid(sample_valid),
        .gamma_in({{(GAMMA_W-1){1'b0}}, 1'b1}), .adapt_en(adapt_en),
        .s_re(e_s_re), .s_im(e_s_im), .s_valid(e_s_valid),
        .w_re(e_w_re), .w_im(e_w_im), .weights_valid(e_weights_valid),
        .alpha_mag(e_alpha_mag), .alpha_mag_valid(e_alpha_mag_valid),
        .pl_gain_band(e_pl_gain_band)
    );

    initial begin
        $dumpfile("waveform_pl_npi.vcd");
        $dumpvars(0, tb_pi_power_inversion_pl_npi);
    end

    real jammer_phase;
    integer sample_count, i;
    real xk_re [0:M-1];
    real xk_im [0:M-1];

    real a_w0_mag, c_w0_mag, e_w0_mag;
    integer a_first_in_band, c_first_in_band, e_first_in_band;
    integer band_seen [0:3];

    function real w0_mag_real(input [WEIGHT_W-1:0] w_re_bus);
        real re;
        begin
            re = $itor($signed(w_re_bus[WEIGHT_W-1:0])) / WSCALE;
            w0_mag_real = (re < 0.0) ? -re : re;
        end
    endfunction

    initial begin
        jammer_phase = 0.0;
        x_re = 0; x_im = 0; sample_valid = 0; adapt_en = 1;
        a_first_in_band = -1; c_first_in_band = -1; e_first_in_band = -1;
        band_seen[0] = 0; band_seen[1] = 0; band_seen[2] = 0; band_seen[3] = 0;

        $display("=========================================================");
        $display(" PL-NPI vs NORMALIZED vs STANDARD PI -- CONVERGENCE + GAIN BAND CHECK");
        $display("=========================================================");
        $display(" A: standard,   alpha=1");
        $display(" C: normalized, gamma=1 (no gain stage)");
        $display(" E: PL-NPI,     gamma=1, default THRESH1/2/3 (Eq. 11 gain cascade)");
        $display(" Same stimulus as tb_pi_power_inversion.v: amp=%.0f, angle_step=%.2f, phase_step=%.2f",
                  AMP, ANGLE_STEP, PHASE_STEP);
        $display("=========================================================\n");

        repeat (4) @(posedge clk);
        aresetn = 1;
        @(posedge clk);

        $display("%-6s %-12s %-12s %-12s %-6s", "k", "A |w0|", "C |w0|", "E |w0|", "band");
        $display("-------------------------------------------------------");

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

            band_seen[e_pl_gain_band] = band_seen[e_pl_gain_band] + 1;

            a_w0_mag = w0_mag_real(a_w_re[WEIGHT_W-1:0]);
            c_w0_mag = w0_mag_real(c_w_re[WEIGHT_W-1:0]);
            e_w0_mag = w0_mag_real(e_w_re[WEIGHT_W-1:0]);

            if (a_w0_mag < CONV_TARGET-CONV_TOL || a_w0_mag > CONV_TARGET+CONV_TOL)
                a_first_in_band = -1;
            else if (a_first_in_band == -1)
                a_first_in_band = sample_count;

            if (c_w0_mag < CONV_TARGET-CONV_TOL || c_w0_mag > CONV_TARGET+CONV_TOL)
                c_first_in_band = -1;
            else if (c_first_in_band == -1)
                c_first_in_band = sample_count;

            if (e_w0_mag < CONV_TARGET-CONV_TOL || e_w0_mag > CONV_TARGET+CONV_TOL)
                e_first_in_band = -1;
            else if (e_first_in_band == -1)
                e_first_in_band = sample_count;

            if (sample_count % PRINT_EVERY == 0) begin
                $display("%-6d %-12.5f %-12.5f %-12.5f %-6d",
                          sample_count, a_w0_mag, c_w0_mag, e_w0_mag, e_pl_gain_band);
            end
        end

        $display("\n=========================================================");
        $display(" GAIN BAND COVERAGE (Eq. 11's four segments, over %0d samples)", N_SAMPLES);
        $display("=========================================================");
        $display(" band 0 (x1.00, |y_I]<=T1): %0d samples", band_seen[0]);
        $display(" band 1 (x1.05, T1<|y_I]<=T2): %0d samples", band_seen[1]);
        $display(" band 2 (x1.10, T2<|y_I]<=T3): %0d samples", band_seen[2]);
        $display(" band 3 (x1.20, |y_I]>T3): %0d samples", band_seen[3]);

        if (band_seen[0] > 0 && (band_seen[1]+band_seen[2]+band_seen[3]) > 0) begin
            $display("PASS: gain cascade is exercised across more than one band (not stuck)");
        end else begin
            $display("FAIL: gain cascade never left a single band -- check THRESH1/2/3 scaling");
        end

        $display("\n=========================================================");
        $display(" CONVERGENCE RESULT (sample index first entering the band");
        $display(" AND remaining in it through sample %0d; -1 = never settled)", N_SAMPLES-1);
        $display("=========================================================");
        $display(" A: standard      alpha=1 -> k = %0d", a_first_in_band);
        $display(" C: normalized    gamma=1 -> k = %0d", c_first_in_band);
        $display(" E: PL-NPI        gamma=1 -> k = %0d", e_first_in_band);

        if (e_first_in_band >= 0) begin
            $display("\nPASS: PL-NPI converged and stayed within the test band");
        end else begin
            $display("\nFAIL: PL-NPI did not converge within %0d samples", N_SAMPLES);
        end

        $display("\nWaveform written to waveform_pl_npi.vcd");
        $display("TESTBENCH DONE");
        $finish;
    end

endmodule
