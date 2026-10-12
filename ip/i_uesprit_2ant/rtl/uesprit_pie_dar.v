`timescale 1ns/1ps
// ============================================================================
// uesprit_pie_dar  --  PIE (eigenvalue) + DAR (DOA recovery) stages, Fig. 3.
//
// PIE : for M = 2 the eigenvalue of Psi_r is psi = N / D  (see RMA header).
//       psi is produced in Q8.24 by a sequential divider (informational).
//
// DAR : Eq. (35)  theta = asin( -lambda/(pi d) * atan(psi) )
//       Implemented division-free and with full-range phase:
//         phi   = 2 * atan(psi) = 2 * atan2(N, D)     (D > 0)   CORDIC
//         u     = -k * phi,      k = lambda / (2 pi d)          multiply
//         theta = asin(u)                                       LUT
//       phi is the inter-element electrical phase, phi = arg(x1 / x0)
//       for the steering vector of Eq. (2).
//
// Outputs: phi Q28 rad, psi Q8.24, theta Q28 rad, theta Q16 degrees.
// ambiguous = 1 when |u| > 1 (clipped to +/-90 deg).
// Latency : ~60 cycles (divider runs in parallel with CORDIC + LUT).
// ============================================================================
module uesprit_pie_dar #(
    parameter integer NW = 70
)(
    input  wire                 clk,
    input  wire                 rst,

    input  wire signed [31:0]   k_q30,      // lambda / (2 pi d), signed Q2.30

    input  wire                 in_valid,
    input  wire signed [NW-1:0] num,
    input  wire signed [NW-1:0] den,
    output wire                 in_ready,

    output reg                  out_valid,
    output reg  signed [31:0]   phi_q28,
    output reg  signed [31:0]   psi_q24,
    output reg  signed [31:0]   theta_q28,
    output reg  signed [31:0]   theta_deg_q16,
    output reg                  ambiguous
);

    // 180/pi in Q20
    localparam signed [31:0] RAD2DEG_Q20 = 32'sd60078979;
    localparam signed [35:0] ONE_Q30     = 36'sd1073741824;

    // ------------------------------------------------------------------
    // common normalisation of (N, D) to 32-bit mantissas
    // ------------------------------------------------------------------
    wire signed [31:0] n_m, d_m, unused0, unused1;
    wire               nd_zero;

    uesprit_bfp_norm4 #(.W(NW)) u_norm (
        .v0 (num), .v1 (den), .v2 ({NW{1'b0}}), .v3 ({NW{1'b0}}),
        .m0 (n_m), .m1 (d_m), .m2 (unused0), .m3 (unused1),
        .zero (nd_zero)
    );

    reg               busy;
    reg signed [31:0] n_r, d_r;
    reg               go;

    assign in_ready = !busy;

    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; go <= 1'b0; n_r <= 0; d_r <= 0;
        end else begin
            go <= 1'b0;
            if (in_valid && !busy) begin
                n_r  <= n_m;
                d_r  <= d_m;
                go   <= 1'b1;
                busy <= 1'b1;
            end else if (out_valid) begin
                busy <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------
    // PIE : psi = N / D
    // ------------------------------------------------------------------
    wire signed [31:0] psi_w;
    wire               psi_v, div_busy;

    uesprit_div_q24 u_div (
        .clk (clk), .rst (rst), .start (go),
        .n_in (n_r), .d_in (d_r),
        .q_out (psi_w), .valid (psi_v), .busy (div_busy)
    );

    // ------------------------------------------------------------------
    // DAR : phi = 2 atan2(N, D)
    // ------------------------------------------------------------------
    wire signed [33:0] ang_q30;
    wire               ang_v, cordic_busy;

    uesprit_cordic_atan2 #(.ITER(30)) u_cordic (
        .clk (clk), .rst (rst), .start (go),
        .x_in (d_r), .y_in (n_r),
        .angle_q30 (ang_q30), .valid (ang_v), .busy (cordic_busy)
    );

    // phi_q28 = 2 * angle * 2^28 = angle_q30 / 2
    wire signed [33:0] phi_w = ang_q30 >>> 1;

    // u = -k * phi  (Q30 * Q28 = Q58 -> Q30)
    reg  signed [31:0] phi_r;
    reg                u_v;
    reg  signed [63:0] kphi;
    reg                kphi_v;
    wire signed [35:0] u_full = -(kphi >>> 28);
    wire               u_ovf  = (u_full > ONE_Q30) || (u_full < -ONE_Q30);
    wire signed [31:0] u_sat  = (u_full > ONE_Q30)  ?  32'sd1073741824 :
                                (u_full < -ONE_Q30) ? -32'sd1073741824 : u_full[31:0];

    reg  signed [31:0] u_r;
    reg                amb_r;

    always @(posedge clk) begin
        if (rst) begin
            phi_r <= 0; kphi <= 0; kphi_v <= 1'b0; u_r <= 0; u_v <= 1'b0; amb_r <= 1'b0;
        end else begin
            kphi_v <= ang_v;
            u_v    <= kphi_v;
            if (ang_v) begin
                phi_r <= phi_w[31:0];
                kphi  <= k_q30 * $signed(phi_w[31:0]);
            end
            if (kphi_v) begin
                u_r   <= u_sat;
                amb_r <= u_ovf;
            end
        end
    end

    // theta = asin(u)
    wire signed [31:0] th_w;
    wire               th_v;

    uesprit_asin_lut u_asin (
        .clk (clk), .rst (rst), .valid_in (u_v),
        .u_q30 (u_r), .theta_q28 (th_w), .valid_out (th_v)
    );

    // degrees, and join with the divider result
    reg  signed [63:0] deg_p;
    reg                deg_v, th_done, psi_done;
    reg  signed [31:0] th_r, psi_r;

    always @(posedge clk) begin
        if (rst) begin
            deg_p <= 0; deg_v <= 1'b0; th_done <= 1'b0; psi_done <= 1'b0;
            th_r <= 0; psi_r <= 0;
            out_valid <= 1'b0; phi_q28 <= 0; psi_q24 <= 0;
            theta_q28 <= 0; theta_deg_q16 <= 0; ambiguous <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            deg_v     <= th_v;
            if (th_v) begin
                th_r  <= th_w;
                deg_p <= th_w * RAD2DEG_Q20;           // Q48 degrees
            end
            if (deg_v)  th_done  <= 1'b1;
            if (psi_v) begin
                psi_done <= 1'b1;
                psi_r    <= psi_w;
            end
            if (th_done && psi_done) begin
                th_done       <= 1'b0;
                psi_done      <= 1'b0;
                out_valid     <= 1'b1;
                phi_q28       <= phi_r;
                psi_q24       <= psi_r;
                theta_q28     <= th_r;
                theta_deg_q16 <= deg_p[63:32];
                ambiguous     <= amb_r;
            end
        end
    end

endmodule
