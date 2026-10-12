`timescale 1ns/1ps
// ============================================================================
// mesprit_pie_dar  --  eigenvalue + DOA recovery for M-ESPRIT, M = 2.
//
// psi = (num_re + j num_im) / den is the single non-zero eigenvalue of Psi
// (see mesprit_rma_2x2). Eq. (21):
//     theta = asin( -lambda/(2 pi d) * arg(psi) )
// den is real and positive, so arg(psi) = atan2(num_im, num_re): no division
// is needed for the angle (vectoring CORDIC, full (-pi, pi] range).
//
// Remark A (unit-circle test): a true source gives |psi| ~ 1. The CORDIC
// also yields |num|; |psi| = |num| / den (sequential divider, Q8.24) is
// reported, and off_circle = 1 when | |psi| - 1 | > uc_tol.
//
// Outputs: phi Q28 rad, |psi| Q8.24, theta Q28 rad, theta Q16 degrees,
//          ambiguous (|u| > 1, clipped), off_circle.
// Latency : ~100 cycles (CORDIC 32, divider 56 after the CORDIC).
// ============================================================================
module mesprit_pie_dar #(
    parameter integer NW = 70
)(
    input  wire                 clk,
    input  wire                 rst,

    input  wire signed [31:0]   k_q30,      // lambda / (2 pi d), signed Q2.30
    input  wire [31:0]          uc_tol_q24, // unit-circle tolerance, Q8.24

    input  wire                 in_valid,
    input  wire signed [NW-1:0] num_re,
    input  wire signed [NW-1:0] num_im,
    input  wire signed [NW-1:0] den,
    output wire                 in_ready,

    output reg                  out_valid,
    output reg  signed [31:0]   phi_q28,
    output reg  signed [31:0]   psi_mag_q24,
    output reg  signed [31:0]   theta_q28,
    output reg  signed [31:0]   theta_deg_q16,
    output reg                  ambiguous,
    output reg                  off_circle
);

    localparam signed [31:0] RAD2DEG_Q20 = 32'sd60078979;    // 180/pi, Q20
    localparam        [31:0] INV_K_Q30   = 32'd652032874;    // 1/1.646760258, Q30
    localparam signed [35:0] ONE_Q30     = 36'sd1073741824;
    localparam signed [32:0] ONE_Q24     = 33'sd16777216;

    // ------------------------------------------------------------------
    // common normalisation of (num_re, num_im, den)
    // ------------------------------------------------------------------
    reg  busy;
    wire accept = in_valid & !busy;
    wire signed [31:0] re_r, im_r, d_r, unused0;
    wire               nd_zero, go;

    mesprit_bfp_norm4 #(.W(NW)) u_norm (
        .clk (clk), .rst (rst), .in_valid (accept),
        .v0 (num_re), .v1 (num_im), .v2 (den), .v3 ({NW{1'b0}}),
        .out_valid (go),
        .m0 (re_r), .m1 (im_r), .m2 (d_r), .m3 (unused0),
        .zero (nd_zero)
    );

    assign in_ready = !busy;

    always @(posedge clk) begin
        if (rst)
            busy <= 1'b0;
        else if (accept)
            busy <= 1'b1;
        else if (out_valid)
            busy <= 1'b0;
    end

    // ------------------------------------------------------------------
    // phi = arg(psi) = atan2(num_im, num_re), |num|
    // ------------------------------------------------------------------
    wire signed [33:0] ang_q30;
    wire        [31:0] mag_k;
    wire               ang_v, cordic_busy;

    mesprit_cordic_atan2 #(.ITER(30)) u_cordic (
        .clk (clk), .rst (rst), .start (go),
        .x_in (re_r), .y_in (im_r),
        .angle_q30 (ang_q30), .mag_k (mag_k), .valid (ang_v), .busy (cordic_busy)
    );

    // phi in Q28 = angle_q30 / 4
    wire signed [33:0] phi_w = ang_q30 >>> 2;

    // |num| = mag_k / K  (< 2^30.5);  |psi| = |num| / den, both halved
    reg  [31:0] mag_r;
    reg  [63:0] mag_p;
    reg         mag_v0, mag_v;
    wire [31:0] mag_n = mag_p[61:30];               // |num|, Q30 mantissa scale

    always @(posedge clk) begin
        if (rst) begin
            mag_r <= 32'd0; mag_p <= 64'd0; mag_v0 <= 1'b0; mag_v <= 1'b0;
        end else begin
            mag_v0 <= ang_v;
            mag_v  <= mag_v0;
            if (ang_v)  mag_r <= mag_k;
            if (mag_v0) mag_p <= mag_r * INV_K_Q30;
        end
    end

    wire signed [31:0] psi_w;
    wire               psi_v, div_busy;

    mesprit_div_q24 u_div (
        .clk (clk), .rst (rst), .start (mag_v),
        .n_in ($signed({1'b0, mag_n[31:1]})), .d_in (d_r >>> 1),
        .q_out (psi_w), .valid (psi_v), .busy (div_busy)
    );

    // ------------------------------------------------------------------
    // u = -k * phi, theta = asin(u)
    // ------------------------------------------------------------------
    reg  signed [31:0] phi_r;
    reg  signed [63:0] kphi;
    reg                phi_v, kphi_v, u_v;
    wire signed [35:0] u_full = -(kphi >>> 28);
    wire               u_ovf  = (u_full > ONE_Q30) || (u_full < -ONE_Q30);
    wire signed [31:0] u_sat  = (u_full > ONE_Q30)  ?  32'sd1073741824 :
                                (u_full < -ONE_Q30) ? -32'sd1073741824 : u_full[31:0];
    reg  signed [31:0] u_r;
    reg                amb_r;

    always @(posedge clk) begin
        if (rst) begin
            phi_r <= 0; phi_v <= 1'b0; kphi <= 0; kphi_v <= 1'b0;
            u_r <= 0; u_v <= 1'b0; amb_r <= 1'b0;
        end else begin
            phi_v  <= ang_v;
            kphi_v <= phi_v;
            u_v    <= kphi_v;
            if (ang_v)
                phi_r <= $signed(phi_w[31:0]);
            if (phi_v)
                kphi  <= k_q30 * phi_r;
            if (kphi_v) begin
                u_r   <= u_sat;
                amb_r <= u_ovf;
            end
        end
    end

    wire signed [31:0] th_w;
    wire               th_v;

    mesprit_asin_lut u_asin (
        .clk (clk), .rst (rst), .valid_in (u_v),
        .u_q30 (u_r), .theta_q28 (th_w), .valid_out (th_v)
    );

    // ------------------------------------------------------------------
    // degrees, unit-circle test, join
    // ------------------------------------------------------------------
    reg  signed [63:0] deg_p;
    reg                deg_v, th_done, psi_done;
    reg  signed [31:0] th_r, psi_r;
    wire signed [32:0] dev     = $signed({psi_r[31], psi_r}) - ONE_Q24;
    wire        [32:0] dev_abs = dev[32] ? -dev : dev;

    always @(posedge clk) begin
        if (rst) begin
            deg_p <= 0; deg_v <= 1'b0; th_done <= 1'b0; psi_done <= 1'b0;
            th_r <= 0; psi_r <= 0;
            out_valid <= 1'b0; phi_q28 <= 0; psi_mag_q24 <= 0;
            theta_q28 <= 0; theta_deg_q16 <= 0; ambiguous <= 1'b0; off_circle <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            deg_v     <= th_v;
            if (th_v) begin
                th_r  <= th_w;
                deg_p <= th_w * RAD2DEG_Q20;           // Q48 degrees
            end
            if (deg_v) th_done <= 1'b1;
            if (psi_v) begin
                psi_done <= 1'b1;
                psi_r    <= psi_w;
            end
            if (th_done && psi_done) begin
                th_done       <= 1'b0;
                psi_done      <= 1'b0;
                out_valid     <= 1'b1;
                phi_q28       <= phi_r;
                psi_mag_q24   <= psi_r;
                theta_q28     <= th_r;
                theta_deg_q16 <= deg_p[63:32];
                ambiguous     <= amb_r;
                off_circle    <= (dev_abs > {1'b0, uc_tol_q24});
            end
        end
    end

endmodule
