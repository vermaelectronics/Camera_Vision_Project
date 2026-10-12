`timescale 1ns/1ps
// ============================================================================
// tb_i_uesprit_2ant_core
//
// Streams stim.hex ({q1,i1,q0,i0} per line, produced by
// model/i_uesprit_model.py gen) into the core and dumps one line per window
// to rtl_results.txt:  phi_q28 psi_q24 theta_q28 theta_deg_q16 flags  (hex)
//
// Compile with -I<dir containing cfg.vh> -DSTIM_DIR=\"<dir>/\"
// ============================================================================
module tb_i_uesprit_2ant_core;

`include "cfg.vh"

`ifndef STIM_DIR
`define STIM_DIR "./"
`endif

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;                       // 100 MHz

    reg  [63:0] mem [0:N_SAMPLES-1];
    reg         in_valid = 1'b0;
    reg  [63:0] din = 64'd0;

    wire        res_valid;
    wire [31:0] phi, psi, th, deg;
    wire [3:0]  flags;
    wire [31:0] dc00, dc11, dc01r, dc01i;

    i_uesprit_2ant_core #(.MAX_LOG2_SNAP(14)) dut (
        .clk (clk), .rst (rst),
        .cfg_log2_snap (CFG_LOG2_SNAP[4:0]),
        .cfg_p_log2    (CFG_P_LOG2[1:0]),
        .cfg_mu_q30    (CFG_MU_Q30),
        .cfg_k_q30     (CFG_K_Q30),
        .cfg_use_cov   (CFG_USE_COV[0]),
        .in_valid (in_valid),
        .x0_i (din[15:0]), .x0_q (din[31:16]), .x1_i (din[47:32]), .x1_q (din[63:48]),
        .res_valid (res_valid),
        .res_phi_q28 (phi), .res_psi_q24 (psi),
        .res_theta_q28 (th), .res_theta_deg_q16 (deg),
        .res_flags (flags),
        .dbg_c00 (dc00), .dbg_c11 (dc11), .dbg_c01_re (dc01r), .dbg_c01_im (dc01i)
    );

    integer fo, k, nres, cyc_last, lat_max;
    integer seed = 1;

    initial begin
        $readmemh({`STIM_DIR, "stim.hex"}, mem);
        fo = $fopen({`STIM_DIR, "rtl_results.txt"}, "w");
        nres = 0; lat_max = 0;
        repeat (5) @(posedge clk);
        rst <= 1'b0;
        @(posedge clk);
        k = 0;
        while (k < N_SAMPLES) begin
            // ~12% idle cycles to exercise gapped valid (NO_GAPS: back-to-back)
`ifdef NO_GAPS
            if (1) begin
`else
            if (($random(seed) & 7) != 0) begin
`endif
                in_valid <= 1'b1;
                din      <= mem[k];
                k = k + 1;
            end else begin
                in_valid <= 1'b0;
            end
            @(posedge clk);
        end
        in_valid <= 1'b0;
        cyc_last = $time / 10;
        repeat (2000) @(posedge clk);
        $fclose(fo);
        $display("TB: %0d results captured, last-window latency %0d cycles", nres, lat_max);
        $finish;
    end

    always @(posedge clk) begin
        if (res_valid) begin
            $fwrite(fo, "%08x %08x %08x %08x %01x\n", phi, psi, th, deg, flags);
            $display("TB: window %0d  theta = %f deg  phi = %f rad  psi = %f  flags = %b  C = [%0d %0d %0d %0d]",
                     nres, $signed(deg) / 65536.0, $signed(phi) / 268435456.0,
                     $signed(psi) / 16777216.0, flags,
                     $signed(dc00), $signed(dc11), $signed(dc01r), $signed(dc01i));
            nres = nres + 1;
            if (nres == N_WINDOWS) lat_max = $time / 10 - cyc_last;
        end
    end

endmodule
