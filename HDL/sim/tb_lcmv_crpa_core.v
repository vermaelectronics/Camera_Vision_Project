`timescale 1ns/1ps

// ============================================================================
// tb_lcmv_crpa_core.v
//
// Self-checking testbench for the 2-antenna robust LCMV core
// (PIER C vol. 4, 2008, eq. 9: look + enforcement constraints).
//
//   2-element ULA, d = lambda/2, a(theta) = [1, exp(j*pi*sin(theta))]
//   Noise-like jammer, JNR ~ 46 dB, 12-bit ADC (no clipping)
//
//   Case 1: GPS 10 deg, jammer 40 deg, jam_angle = 40   (checked)
//   Case 2: GPS 10 deg, jammer 46 deg, jam_angle = 46   (checked)
//   Case 3: GPS 10 deg, jammer 46 deg, jam_angle = 45   (info only:
//           shows the null depth with a 1 deg angle error)
//
// Checks per case:
//   |w^H a_s|           = 0 dB  +/- 0.1 dB   (look constraint)
//   |w^H a_J| / peak    < PASS_DB             (null depth at jam_angle)
//   output / input power < PASS_SUPP_DB       (measured jammer suppression)
//
// Run:
//   iverilog -g2012 -o tb.vvp ../steering_vector_2x2.v ../sdiv_q16_serial.v \
//            ../lcmv_dc_weight_2ch.v ../lcmv_beamformer_2ch.v \
//            ../lcmv_crpa_core.v tb_lcmv_crpa_core.v && vvp -n tb.vvp
// ============================================================================

module tb_lcmv_crpa_core;

    localparam real PI        = 3.14159265358979;
    localparam real GPS_DEG   = 10.0;
    localparam real JAM_AMP   = 400.0;   // per I/Q rms, 5 sigma < full scale
    localparam real NOISE_AMP = 2.0;     // per I/Q rms LSB -> JNR ~ 46 dB
    localparam real PASS_DB      = -60.0;
    localparam real PASS_SUPP_DB = -35.0;

    reg clk = 0;
    reg rst = 1;
    reg enable = 0;
    reg rx_valid = 0;

    reg signed [15:0] rx1_i, rx1_q, rx2_i, rx2_q;
    reg signed [15:0] gps_angle_reg = 16'sd10;
    reg signed [15:0] jam_angle_reg = 16'sd40;

    wire busy, weights_valid, beam_valid;
    wire signed [31:0] w1_re, w1_im, w2_re, w2_im, y_re, y_im;

    always #5 clk = ~clk;

    lcmv_crpa_core dut (
        .clk(clk), .rst(rst), .enable(enable),
        .rx_valid(rx_valid),
        .rx1_i(rx1_i), .rx1_q(rx1_q), .rx2_i(rx2_i), .rx2_q(rx2_q),
        .gps_angle(gps_angle_reg), .jam_angle(jam_angle_reg),
        .busy(busy), .weights_valid(weights_valid),
        .w1_re(w1_re), .w1_im(w1_im), .w2_re(w2_re), .w2_im(w2_im),
        .beam_valid(beam_valid), .y_re(y_re), .y_im(y_im)
    );

    // ------------------------------------------------------------------------
    // Stimulus: jammer at jam_deg (true direction)
    // ------------------------------------------------------------------------

    integer seed = 12345;
    real jam_deg = 40.0;

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
    // Power meters
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
    // Array response |w^H a(theta)|^2 in dB (exact steering, float)
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
            // conj(w1) + conj(w2) * a2
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
    // One measurement
    // ------------------------------------------------------------------------

    integer errors = 0;
    real g_gps, depth, supp, th, g, best_g, best_deg;

    task measure;
        input integer check;
        begin
            // let at least two weight updates use the new angles
            repeat (300) @(posedge clk);
            while (!weights_valid) @(posedge clk);

            p_in = 0; p_out = 0; n_in = 0; n_out = 0;
            repeat (2048) @(posedge clk);

            g_gps = gain_db(GPS_DEG);
            depth = gain_db(jam_deg) - peak_db(0);
            supp  = 10.0 * $log10((p_out / (n_out > 0 ? n_out : 1) + 1e-9) /
                                  (p_in / (n_in > 0 ? n_in : 1)));

            best_g = 400.0; best_deg = 0.0;
            for (th = -90.0; th <= 90.0; th = th + 0.05) begin
                g = gain_db(th);
                if (g < best_g) begin best_g = g; best_deg = th; end
            end

            $display("gps=%0d jam_angle=%0d true_jam=%0.1f | null@%0.2f  GPS gain=%0.2f dB  null depth=%0.1f dB  out/in=%0.1f dB  w1=(%0d,%0d) w2=(%0d,%0d)",
                     gps_angle_reg, jam_angle_reg, jam_deg, best_deg, g_gps, depth, supp,
                     w1_re, w1_im, w2_re, w2_im);

            if (check) begin
                if (g_gps > 0.1 || g_gps < -0.1) begin
                    errors = errors + 1;
                    $display("  ERROR: look constraint gain %0.2f dB", g_gps);
                end
                if (depth > PASS_DB) begin
                    errors = errors + 1;
                    $display("  ERROR: null depth %0.1f dB above %0.1f dB", depth, PASS_DB);
                end
                if (supp > PASS_SUPP_DB) begin
                    errors = errors + 1;
                    $display("  ERROR: suppression %0.1f dB above %0.1f dB", supp, PASS_SUPP_DB);
                end
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------------

    initial begin
        p_in = 0; p_out = 0; n_in = 0; n_out = 0;

        repeat (10) @(posedge clk);
        rst <= 0;
        repeat (5) @(posedge clk);
        enable <= 1;

        // Case 1
        jam_deg = 40.0; jam_angle_reg = 16'sd40;
        measure(1);

        // Case 2: jammer moves, PS updates jam_angle
        jam_deg = 46.0; jam_angle_reg = 16'sd46;
        measure(1);

        // Case 3: jam_angle wrong by 1 deg (information only)
        jam_angle_reg = 16'sd45;
        measure(0);

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
