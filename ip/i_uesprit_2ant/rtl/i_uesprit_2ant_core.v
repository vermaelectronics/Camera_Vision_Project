`timescale 1ns/1ps
// ============================================================================
// i_uesprit_2ant_core  --  Improved unitary modified ESPRIT (I-UESPRIT) DOA
//                          estimator for a two-element CRPA baseline.
//
// Algorithm: J. Li, G. Yang, X. Ren, Z. Fu, S. Qiu, "Modified ESPRIT and
// Unitary-Transformation Form for Computationally Efficient DOA Estimation
// on FPGA", IEEE Trans. Instrum. Meas., vol. 74, 2025, Art. 2003209.
//
//   x0,x1 --> CMC --> BFP --> RMA --------------> PIE/DAR --> phi, psi, theta
//            C(4th   norm    C^p, C_fb, C_r,      psi=N/D,
//            order            Psi_r = N/D         theta = asin(-k*2atan(psi))
//            cumul.)
//
// * No source-number estimation (Section III-A).
// * Only real-valued arithmetic after the CMC stage (Section III-B).
// * C^p sharpening, p = 2^p_log2 (Eq. (38), paper uses p = 4).
// * For M = 2 the rank-one Psi_r has a single non-zero eigenvalue equal to
//   its trace, so the PIE power iteration collapses to one division.
//
// Single clock domain. One complex snapshot per clock maximum.
// End-to-end latency after the last snapshot of a window ~ 131 cycles
// (p = 4), i.e. 1.3 us at 100 MHz.
// ============================================================================
module i_uesprit_2ant_core #(
    parameter integer MAX_LOG2_SNAP = 14
)(
    input  wire               clk,
    input  wire               rst,

    // configuration (sampled at window boundaries / stage start)
    input  wire [4:0]         cfg_log2_snap,   // window = 2^n snapshots, n in [7, MAX]
    input  wire [1:0]         cfg_p_log2,      // p = 1, 2, 4, 8
    input  wire [31:0]        cfg_mu_q30,      // regulariser of Eq. (37), Q30
    input  wire signed [31:0] cfg_k_q30,       // lambda / (2 pi d), Q2.30 (1/pi for d = lambda/2)
    input  wire               cfg_use_cov,     // 1: covariance instead of 4th-order cumulant

    // antenna snapshots, signed Q1.15
    input  wire               in_valid,
    input  wire signed [15:0] x0_i,
    input  wire signed [15:0] x0_q,
    input  wire signed [15:0] x1_i,
    input  wire signed [15:0] x1_q,

    // results (one pulse of res_valid per window)
    output wire               res_valid,
    output wire signed [31:0] res_phi_q28,       // electrical phase arg(x1/x0), rad Q28
    output wire signed [31:0] res_psi_q24,       // eigenvalue of Psi_r = tan(phi/2), Q8.24
    output wire signed [31:0] res_theta_q28,     // DOA, rad Q28
    output wire signed [31:0] res_theta_deg_q16, // DOA, degrees Q16
    output wire [3:0]         res_flags,         // {0, overrun, ambiguous, no_signal}

    // debug: normalised cumulant and C^p (32-bit mantissas, Q30)
    output reg  signed [31:0] dbg_c00,
    output reg  signed [31:0] dbg_c11,
    output reg  signed [31:0] dbg_c01_re,
    output reg  signed [31:0] dbg_c01_im
);

    localparam integer CW = 72;
    localparam integer NW = 70;

    // ------------------------------------------------------------------
    // CMC
    // ------------------------------------------------------------------
    wire signed [CW-1:0] c00, c11, c01r, c01i;
    wire                 c_valid, cmc_overrun;

    uesprit_cmc_2x2 #(
        .MAX_LOG2_SNAP (MAX_LOG2_SNAP),
        .CW            (CW)
    ) u_cmc (
        .clk (clk), .rst (rst),
        .log2_snap (cfg_log2_snap), .use_cov (cfg_use_cov),
        .valid_in (in_valid),
        .x0_i (x0_i), .x0_q (x0_q), .x1_i (x1_i), .x1_q (x1_q),
        .c00 (c00), .c11 (c11), .c01_re (c01r), .c01_im (c01i),
        .c_valid (c_valid), .overrun (cmc_overrun)
    );

    // ------------------------------------------------------------------
    // block-floating-point normalisation of C (2-cycle pipeline)
    // ------------------------------------------------------------------
    wire               cn_valid;
    wire signed [31:0] cn00, cn11, cn01r, cn01i;
    wire               cn_zero;

    uesprit_bfp_norm4 #(.W(CW)) u_norm (
        .clk (clk), .rst (rst), .in_valid (c_valid),
        .v0 (c00), .v1 (c11), .v2 (c01r), .v3 (c01i),
        .out_valid (cn_valid),
        .m0 (cn00), .m1 (cn11), .m2 (cn01r), .m3 (cn01i),
        .zero (cn_zero)
    );

    always @(posedge clk) begin
        if (rst) begin
            dbg_c00 <= 0; dbg_c11 <= 0; dbg_c01_re <= 0; dbg_c01_im <= 0;
        end else if (cn_valid) begin
            dbg_c00 <= cn00; dbg_c11 <= cn11; dbg_c01_re <= cn01r; dbg_c01_im <= cn01i;
        end
    end

    // ------------------------------------------------------------------
    // RMA
    // ------------------------------------------------------------------
    wire                 rma_ready, rma_valid;
    wire signed [NW-1:0] num, den;
    wire signed [31:0]   cp_a, cp_d, cp_br, cp_bi;

    uesprit_rma_2x2 #(.NW(NW)) u_rma (
        .clk (clk), .rst (rst),
        .p_log2 (cfg_p_log2), .mu_q30 (cfg_mu_q30),
        .in_valid (cn_valid & rma_ready),
        .in_a (cn00), .in_d (cn11), .in_br (cn01r), .in_bi (cn01i),
        .in_ready (rma_ready),
        .out_valid (rma_valid), .num (num), .den (den),
        .cp_a (cp_a), .cp_d (cp_d), .cp_br (cp_br), .cp_bi (cp_bi)
    );

    // ------------------------------------------------------------------
    // PIE + DAR
    // ------------------------------------------------------------------
    wire dar_ready, ambiguous;

    uesprit_pie_dar #(.NW(NW)) u_dar (
        .clk (clk), .rst (rst),
        .k_q30 (cfg_k_q30),
        .in_valid (rma_valid & dar_ready),
        .num (num), .den (den),
        .in_ready (dar_ready),
        .out_valid (res_valid),
        .phi_q28 (res_phi_q28), .psi_q24 (res_psi_q24),
        .theta_q28 (res_theta_q28), .theta_deg_q16 (res_theta_deg_q16),
        .ambiguous (ambiguous)
    );

    // ------------------------------------------------------------------
    // flags: carry no-signal through the pipeline, accumulate overruns
    // ------------------------------------------------------------------
    reg zero_rma, zero_dar, ovr_acc;

    always @(posedge clk) begin
        if (rst) begin
            zero_rma <= 1'b0; zero_dar <= 1'b0; ovr_acc <= 1'b0;
        end else begin
            if (cn_valid && rma_ready)  zero_rma <= cn_zero;
            if (rma_valid && dar_ready) zero_dar <= zero_rma;

            if (cmc_overrun || (cn_valid && !rma_ready) || (rma_valid && !dar_ready))
                ovr_acc <= 1'b1;
            else if (res_valid)
                ovr_acc <= 1'b0;
        end
    end

    // valid together with res_valid
    assign res_flags = {1'b0, ovr_acc, ambiguous, zero_dar};

endmodule
