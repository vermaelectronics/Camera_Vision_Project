`timescale 1ns/1ps

// ============================================================================
// tb_lcmv_crpa_core.v
//
// Self-checking testbench for the LCMV-PI CRPA core.
//
//   2-element ULA, d = lambda/2
//   Strong noise-like jammer (JNR ~ 46 dB, no ADC clipping) + independent receiver noise
//   GPS signal is below the noise floor and is not modelled.
//
//   Phase 1: jammer fixed at JAM0 deg
//   Phase 2: jammer moves to JAM1 deg (null must follow, no angle input)
//
// For every new weight set the testbench prints:
//   - array gain toward the jammer relative to the peak gain  (null depth)
//   - measured output power / input power on element 1        (suppression)
//
// Run:  iverilog -g2012 -o tb.vvp -I.. ../*.v tb_lcmv_crpa_core.v && vvp tb.vvp
// ============================================================================

module tb_lcmv_crpa_core;

    localparam real PI   = 3.14159265358979;
    localparam real JAM0 = 40.0;
    localparam real JAM1 = 46.0;
    localparam real JAM_AMP   = 400.0;    // per I/Q rms; 5 sigma < 12-bit full scale (no clipping)
    localparam real NOISE_AMP = 2.0;      // per I/Q rms LSB -> JNR ~ 46 dB
    localparam integer N_WEIGHTS = 8;
    localparam real PASS_DB   = -40.0;

    reg clk = 0;
    reg rst = 1;
    reg enable = 0;
    reg rx_valid = 0;

    reg signed [15:0] rx1_i, rx1_q, rx2_i, rx2_q;

    wire busy, weights_valid, beam_valid;
    wire signed [31:0] w1_re, w1_im, w2_re, w2_im, y_re, y_im;

    always #5 clk = ~clk;

    lcmv_crpa_core dut (
        .clk(clk), .rst(rst), .enable(enable),
        .rx_valid(rx_valid),
        .rx1_i(rx1_i), .rx1_q(rx1_q), .rx2_i(rx2_i), .rx2_q(rx2_q),
        .gps_angle(16'sd0), .jam_angle(16'sd0),
        .busy(busy), .weights_valid(weights_valid),
        .w1_re(w1_re), .w1_im(w1_im), .w2_re(w2_re), .w2_im(w2_im),
        .beam_valid(beam_valid), .y_re(y_re), .y_im(y_im)
    );

    // ------------------------------------------------------------------------
    // Stimulus
    // ------------------------------------------------------------------------

    integer seed = 12345;
    real jam_deg = JAM0;

    function real gauss;
        input integer dummy;
        begin
            gauss = $dist_normal(seed, 0, 10000) / 10000.0;
        end
    endfunction

    function signed [15:0] q12;
        input real v;
        real r;
        begin
            r = (v >= 0.0) ? v + 0.5 : v - 0.5;
            if (r > 2047.0)  r = 2047.0;
            if (r < -2048.0) r = -2048.0;
            q12 = $rtoi(r);
        end
    endfunction

    real s_re, s_im, ph, c, s;

    always @(posedge clk) begin
        if (enable) begin
            s_re = JAM_AMP * gauss(0);
            s_im = JAM_AMP * gauss(0);
            ph   = PI * $sin(jam_deg * PI / 180.0);
            c    = $cos(ph);
            s    = $sin(ph);

            rx1_i <= q12(s_re + NOISE_AMP * gauss(0));
            rx1_q <= q12(s_im + NOISE_AMP * gauss(0));
            rx2_i <= q12(s_re * c - s_im * s + NOISE_AMP * gauss(0));
            rx2_q <= q12(s_re * s + s_im * c + NOISE_AMP * gauss(0));
            rx_valid <= 1'b1;
        end
        else begin
            rx_valid <= 1'b0;
        end
    end

    // ------------------------------------------------------------------------
    // Output / input power meters
    // ------------------------------------------------------------------------

    real p_in, p_out;
    integer n_in, n_out;

    always @(posedge clk) begin
        if (rx_valid) begin
            p_in = p_in + 1.0 * rx1_i * rx1_i + 1.0 * rx1_q * rx1_q;
            n_in = n_in + 1;
        end
        if (beam_valid) begin
            p_out = p_out + 1.0 * y_re * y_re + 1.0 * y_im * y_im;
            n_out = n_out + 1;
        end
    end

    // ------------------------------------------------------------------------
    // Array pattern from the current weights:  |w^H a(theta)|
    // a(theta) = [1, exp(j*pi*sin(theta))]
    // ------------------------------------------------------------------------

    function real gain_db;
        input real th;
        real wr1, wi1, wr2, wi2, p, ar, ai, yr, yi;
        begin
            wr1 = w1_re / 65536.0;  wi1 = w1_im / 65536.0;
            wr2 = w2_re / 65536.0;  wi2 = w2_im / 65536.0;
            p   = PI * $sin(th * PI / 180.0);
            ar  = $cos(p);
            ai  = $sin(p);
            // conj(w1)*1 + conj(w2)*a2
            yr = wr1 + (wr2 * ar + wi2 * ai);
            yi = -wi1 + (wr2 * ai - wi2 * ar);
            gain_db = 10.0 * $log10(yr * yr + yi * yi + 1e-30);
        end
    endfunction

    function real peak_db;
        input integer dummy;
        real th, g, m;
        begin
            m = -400.0;
            for (th = -90.0; th <= 90.0; th = th + 0.25) begin
                g = gain_db(th);
                if (g > m) m = g;
            end
            peak_db = m;
        end
    endfunction

    // ------------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------------

    integer k;
    integer errors = 0;
    real depth, supp, best_deg, best_g, th, g;

    initial begin
        p_in = 0; p_out = 0; n_in = 0; n_out = 0;

        repeat (10) @(posedge clk);
        rst <= 0;
        repeat (5) @(posedge clk);
        enable <= 1;

        for (k = 0; k < N_WEIGHTS; k = k + 1) begin

            if (k == N_WEIGHTS / 2) begin
                jam_deg = JAM1;
                $display("---- jammer moves %0.1f -> %0.1f deg (no angle given to the core) ----",
                         JAM0, JAM1);
            end

            @(posedge clk);
            while (!(dut.state == 2'd2 && dut.wgt_done)) @(posedge clk);
            @(posedge clk);

            // measure output over the block that uses these weights
            p_in = 0; p_out = 0; n_in = 0; n_out = 0;
            repeat (1100) @(posedge clk);

            depth = gain_db(jam_deg) - peak_db(0);
            supp  = 10.0 * $log10((p_out / (n_out > 0 ? n_out : 1) + 1e-9) /
                                  (p_in / (n_in > 0 ? n_in : 1)));

            best_g = 400.0; best_deg = 0.0;
            for (th = -90.0; th <= 90.0; th = th + 0.05) begin
                g = gain_db(th);
                if (g < best_g) begin best_g = g; best_deg = th; end
            end

            $display("block %0d  jam=%0.1f  null@%0.2f deg  w2=(%0d,%0d)  depth=%0.1f dB  out/in=%0.1f dB",
                     k, jam_deg, best_deg, w2_re, w2_im, depth, supp);

            // skip the first block after the move (mixed old/new data)
            if (k != N_WEIGHTS / 2 && depth > PASS_DB) begin
                errors = errors + 1;
                $display("  ERROR: null depth %0.1f dB above %0.1f dB", depth, PASS_DB);
            end
        end

        // disable -> weights_valid must drop
        enable <= 0;
        repeat (3) @(posedge clk);
        if (weights_valid !== 1'b0) begin
            errors = errors + 1;
            $display("ERROR: weights_valid still high after disable");
        end

        if (errors == 0) $display("PASS");
        else             $display("FAIL (%0d errors)", errors);
        $finish;
    end

endmodule
