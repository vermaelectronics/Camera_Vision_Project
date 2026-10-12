`timescale 1ns/1ps
// ============================================================================
// i_uesprit_2ant_axi  --  IP top level: I-UESPRIT 2-antenna DOA estimator
//
// Interfaces (single clock domain, aclk / aresetn):
//   S_AXIS     : antenna snapshots, 64-bit tdata = {Q1, I1, Q0, I0}
//                (signed 16-bit Q1.15 each, antenna 0 = reference). tready = 1.
//   M_AXIS     : one beat per estimation window,
//                tdata = {theta_deg_q16[31:0], phi_q28[31:0]}, tuser = flags,
//                tlast = 1. 1-deep buffer; an unconsumed beat is overwritten
//                (STATUS.axis_drop is set).
//   S_AXI_LITE : control / status / results (32-bit, 6-bit address)
//   irq        : level, = STATUS.result_valid & CTRL.irq_en
//
// Register map (byte offsets)
//   0x00 CTRL       RW  [0] enable (1)  [1] soft_reset (self-clearing)
//                       [2] irq_en (0)  [3] use_cov (0)
//   0x04 STATUS     RW1C[0] result_valid  [3] overrun  [4] axis_drop   (sticky)
//                   RO  [1] no_signal  [2] ambiguous  (last result)
//                   RO  [31:16] result counter
//   0x08 LOG2_SNAP  RW  window = 2^n snapshots, n in [7, MAX_LOG2_SNAP] (10)
//   0x0C P_LOG2     RW  C^p exponent, p = 2^n, n in [0, 3]               (2 -> p=4)
//   0x10 MU_Q30     RW  regulariser mu' of Eq. (37), unsigned Q30        (1024)
//   0x14 K_Q30      RW  lambda/(2 pi d), signed Q2.30   (341782638 = 1/pi, d = lambda/2)
//   0x18 PHI_Q28    RO  electrical phase arg(x1/x0), rad, Q28
//   0x1C PSI_Q24    RO  eigenvalue psi of Psi_r (= tan(phi/2)), Q8.24
//   0x20 THETA_Q28  RO  DOA, rad, Q28
//   0x24 THETA_DEG  RO  DOA, degrees, Q16
//   0x28 C00        RO  normalised cumulant matrix (debug, Q30 mantissas)
//   0x2C C11        RO
//   0x30 C01_RE     RO
//   0x34 C01_IM     RO
//   0x3C ID         RO  0x4553_5032 ("ESP2")
// ============================================================================
module i_uesprit_2ant_axi #(
    parameter integer MAX_LOG2_SNAP = 14,
    parameter integer C_S_AXI_ADDR_WIDTH = 6,
    parameter integer C_S_AXI_DATA_WIDTH = 32
)(
    input  wire                              aclk,
    input  wire                              aresetn,

    // AXI4-Stream slave : snapshots
    input  wire [63:0]                       s_axis_tdata,
    input  wire                              s_axis_tvalid,
    output wire                              s_axis_tready,

    // AXI4-Stream master : DOA results
    output reg  [63:0]                       m_axis_tdata,
    output reg  [3:0]                        m_axis_tuser,
    output wire                              m_axis_tlast,
    output reg                               m_axis_tvalid,
    input  wire                              m_axis_tready,

    // AXI4-Lite slave
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  wire [2:0]                        s_axi_awprot,
    input  wire                              s_axi_awvalid,
    output reg                               s_axi_awready,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [C_S_AXI_DATA_WIDTH/8-1:0]   s_axi_wstrb,
    input  wire                              s_axi_wvalid,
    output reg                               s_axi_wready,
    output wire [1:0]                        s_axi_bresp,
    output reg                               s_axi_bvalid,
    input  wire                              s_axi_bready,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
    input  wire [2:0]                        s_axi_arprot,
    input  wire                              s_axi_arvalid,
    output reg                               s_axi_arready,
    output reg  [C_S_AXI_DATA_WIDTH-1:0]     s_axi_rdata,
    output wire [1:0]                        s_axi_rresp,
    output reg                               s_axi_rvalid,
    input  wire                              s_axi_rready,

    output wire                              irq
);

    localparam [31:0] IP_ID = 32'h4553_5032;

    // ------------------------------------------------------------------
    // configuration registers
    // ------------------------------------------------------------------
    reg        r_enable, r_irq_en, r_use_cov, r_soft_rst;
    reg [4:0]  r_log2_snap;
    reg [1:0]  r_p_log2;
    reg [31:0] r_mu_q30, r_k_q30;
    reg        st_result, st_overrun, st_drop;
    reg [15:0] res_count;

    wire core_rst = ~aresetn | r_soft_rst;

    // ------------------------------------------------------------------
    // core
    // ------------------------------------------------------------------
    wire               res_valid;
    wire signed [31:0] phi, psi, th, deg;
    wire [3:0]         flags;
    wire signed [31:0] dc00, dc11, dc01r, dc01i;

    assign s_axis_tready = 1'b1;

    i_uesprit_2ant_core #(.MAX_LOG2_SNAP(MAX_LOG2_SNAP)) u_core (
        .clk (aclk), .rst (core_rst),
        .cfg_log2_snap (r_log2_snap), .cfg_p_log2 (r_p_log2),
        .cfg_mu_q30 (r_mu_q30), .cfg_k_q30 (r_k_q30), .cfg_use_cov (r_use_cov),
        .in_valid (s_axis_tvalid & r_enable),
        .x0_i (s_axis_tdata[15:0]),  .x0_q (s_axis_tdata[31:16]),
        .x1_i (s_axis_tdata[47:32]), .x1_q (s_axis_tdata[63:48]),
        .res_valid (res_valid),
        .res_phi_q28 (phi), .res_psi_q24 (psi),
        .res_theta_q28 (th), .res_theta_deg_q16 (deg),
        .res_flags (flags),
        .dbg_c00 (dc00), .dbg_c11 (dc11), .dbg_c01_re (dc01r), .dbg_c01_im (dc01i)
    );

    // result registers
    reg [31:0] q_phi, q_psi, q_th, q_deg;
    reg [3:0]  q_flags;

    always @(posedge aclk) begin
        if (core_rst) begin
            q_phi <= 0; q_psi <= 0; q_th <= 0; q_deg <= 0; q_flags <= 0;
        end else if (res_valid) begin
            q_phi <= phi; q_psi <= psi; q_th <= th; q_deg <= deg; q_flags <= flags;
        end
    end

    // ------------------------------------------------------------------
    // M_AXIS result stream
    // ------------------------------------------------------------------
    assign m_axis_tlast = 1'b1;

    always @(posedge aclk) begin
        if (core_rst) begin
            m_axis_tvalid <= 1'b0; m_axis_tdata <= 64'd0; m_axis_tuser <= 4'd0;
        end else begin
            if (res_valid) begin
                m_axis_tvalid <= 1'b1;
                m_axis_tdata  <= {deg, phi};
                m_axis_tuser  <= flags;
            end else if (m_axis_tready) begin
                m_axis_tvalid <= 1'b0;
            end
        end
    end

    wire drop_evt = res_valid & m_axis_tvalid & ~m_axis_tready;

    // ------------------------------------------------------------------
    // AXI4-Lite write channel
    // ------------------------------------------------------------------
    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    wire wr_en = s_axi_awvalid & s_axi_wvalid & ~s_axi_awready & ~s_axi_bvalid;
    wire [3:0] wr_word = s_axi_awaddr[5:2];
    wire [31:0] wd = s_axi_wdata;

    always @(posedge aclk) begin
        if (!aresetn) begin
            s_axi_awready <= 1'b0; s_axi_wready <= 1'b0; s_axi_bvalid <= 1'b0;
            r_enable <= 1'b1; r_irq_en <= 1'b0; r_use_cov <= 1'b0; r_soft_rst <= 1'b0;
            r_log2_snap <= 5'd10; r_p_log2 <= 2'd2;
            r_mu_q30 <= 32'd1024; r_k_q30 <= 32'd341782638;
            st_result <= 1'b0; st_overrun <= 1'b0; st_drop <= 1'b0; res_count <= 16'd0;
        end else begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            r_soft_rst    <= 1'b0;

            if (s_axi_bvalid && s_axi_bready)
                s_axi_bvalid <= 1'b0;

            // status events (set has priority over clear)
            if (res_valid) begin
                res_count <= res_count + 1'b1;
            end

            if (wr_en) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                s_axi_bvalid  <= 1'b1;
                case (wr_word)
                    4'h0: if (s_axi_wstrb[0]) begin
                              r_enable   <= wd[0];
                              r_soft_rst <= wd[1];
                              r_irq_en   <= wd[2];
                              r_use_cov  <= wd[3];
                          end
                    4'h2: if (s_axi_wstrb[0]) r_log2_snap <= wd[4:0];
                    4'h3: if (s_axi_wstrb[0]) r_p_log2    <= wd[1:0];
                    4'h4: r_mu_q30 <= apply_strb(r_mu_q30, wd, s_axi_wstrb);
                    4'h5: r_k_q30  <= apply_strb(r_k_q30,  wd, s_axi_wstrb);
                    default: ;
                endcase
            end

            st_result  <= res_valid | (st_result  & ~(wr_en && wr_word == 4'h1 && s_axi_wstrb[0] && wd[0]));
            st_overrun <= (res_valid & flags[2]) |
                          (st_overrun & ~(wr_en && wr_word == 4'h1 && s_axi_wstrb[0] && wd[3]));
            st_drop    <= drop_evt |
                          (st_drop    & ~(wr_en && wr_word == 4'h1 && s_axi_wstrb[0] && wd[4]));

            if (r_soft_rst) begin
                st_result <= 1'b0; st_overrun <= 1'b0; st_drop <= 1'b0; res_count <= 16'd0;
            end
        end
    end

    function [31:0] apply_strb;
        input [31:0] old;
        input [31:0] nw;
        input [3:0]  strb;
        integer b;
        begin
            for (b = 0; b < 4; b = b + 1)
                apply_strb[8*b +: 8] = strb[b] ? nw[8*b +: 8] : old[8*b +: 8];
        end
    endfunction

    // ------------------------------------------------------------------
    // AXI4-Lite read channel
    // ------------------------------------------------------------------
    always @(posedge aclk) begin
        if (!aresetn) begin
            s_axi_arready <= 1'b0; s_axi_rvalid <= 1'b0; s_axi_rdata <= 32'd0;
        end else begin
            s_axi_arready <= 1'b0;
            if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 1'b0;
            if (s_axi_arvalid && !s_axi_arready && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                case (s_axi_araddr[5:2])
                    4'h0: s_axi_rdata <= {28'd0, r_use_cov, r_irq_en, 1'b0, r_enable};
                    4'h1: s_axi_rdata <= {res_count, 11'd0, st_drop, st_overrun,
                                          q_flags[1], q_flags[0], st_result};
                    4'h2: s_axi_rdata <= {27'd0, r_log2_snap};
                    4'h3: s_axi_rdata <= {30'd0, r_p_log2};
                    4'h4: s_axi_rdata <= r_mu_q30;
                    4'h5: s_axi_rdata <= r_k_q30;
                    4'h6: s_axi_rdata <= q_phi;
                    4'h7: s_axi_rdata <= q_psi;
                    4'h8: s_axi_rdata <= q_th;
                    4'h9: s_axi_rdata <= q_deg;
                    4'hA: s_axi_rdata <= dc00;
                    4'hB: s_axi_rdata <= dc11;
                    4'hC: s_axi_rdata <= dc01r;
                    4'hD: s_axi_rdata <= dc01i;
                    4'hF: s_axi_rdata <= IP_ID;
                    default: s_axi_rdata <= 32'd0;
                endcase
            end
        end
    end

    assign irq = st_result & r_irq_en;

endmodule
