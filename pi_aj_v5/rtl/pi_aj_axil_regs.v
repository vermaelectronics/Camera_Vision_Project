// ============================================================================
// pi_aj_axil_regs.v  --  PI-AJ v5 AXI4-Lite register file (s_axi_aclk domain)
//
//  off  name      acc  description
//  0x00 ID        R    0x50494A35 "PIJ5"
//  0x04 VERSION   R    0x00050001  (5.0.1)
//  0x08 CONTROL   RW   [0] core_en   (0: TX1 = RX1 bypass)          def 1
//                      [1] notch_en  (CW notch #1, self-gated)       def 1
//                      [2] notch2_en (CW notch #2, 2nd CW jammer)    def 0
//                      [3] image_en  (I/Q-image notch of #1)         def 1
//                      [4] pi_en     (space-time power inversion)    def 1
//                      [5] agc_en    (TX output AGC)                 def 1
//                      [6] dcblk_en  (DC blocker; off: it fights a CW
//                                    jammer sitting exactly on the LO) def 0
//                      [7] blank_en  (zero samples while ADC clips)  def 0
//                      [8] freeze    (hold all adaptive weights)     def 0
//                      [9] notch_force (notches on even if no gain)  def 0
//                      [10] auto_mu  (PI step +1 while a CW is notched) def 1
//  0x0C ADAPT     RW   [3:0]   pi_mu_sh   PI step 2^-mu              def 0
//                      [11:8]  rho_sh     notch width (~fs*2^-rho/pi) def 6
//                      [15:12] r_sh       notch freq-estimator avg   def 12
//                      [19:16] a_sh       notch coef step            def 8
//                      [24:20] man_shift  TX shift if agc_en=0 (s5)  def 0
//  0x10 LEVELS    RW   [15:0]  clip_thr   |ADC| clip level (LSB)     def 2040
//                      [23:16] blank_hold samples blanked after clip def 16
//                      [27:24] agc_target log2 of TX RMS (16b LSB)   def 11
//  0x14 WSEL      RW   [5:0]   PI weight index for 0x40/0x44
//  0x18 CMD       W    [0] write 1: zero PI weights + re-acquire notches
//  0x20 STATUS    R    [0] notch1 active  [1] image active  [2] notch2 active
//                      [3] ADC overload (>0.1% clipped in last window)
//                      [4] PI late (sticky)  [5] PI overrun (sticky)
//                      [12:8] TX AGC shift (signed)
//  0x24 P_RX1     R    smoothed |RX1|^2  (ADC LSB^2 * 256)
//  0x28 P_RX2     R    smoothed |RX2|^2
//  0x2C P_NOTCH   R    smoothed |RX1|^2 after the notch chain
//  0x30 P_OUT     R    smoothed |PI output|^2 (before TX AGC)
//  0x34 NOTCH1_A  R    {im[31:16], re[15:0]} notch-1 coefficient, Q1.14
//  0x38 NOTCH2_A  R    {im, re} notch-2 coefficient, Q1.14
//  0x3C CLIPCNT   R    clipped samples in the last 2^20-sample window
//  0x40 W_RE      R    PI weight[WSEL] real, Q3.28
//  0x44 W_IM      R    PI weight[WSEL] imag, Q3.28
//  0x48 SAMPLES   R    sample counter
//
//  Suppression (dB)   = 10*log10(P_RX1 / P_OUT)
//  Jammer offset (Hz) = atan2(im, re) / (2*pi) * fs
// ============================================================================
`timescale 1ns / 1ps
module pi_aj_axil_regs (
    input  wire         s_axi_aclk,
    input  wire         s_axi_aresetn,
    input  wire [6:0]   s_axi_awaddr,
    input  wire         s_axi_awvalid,
    output reg          s_axi_awready,
    input  wire [31:0]  s_axi_wdata,
    input  wire [3:0]   s_axi_wstrb,
    input  wire         s_axi_wvalid,
    output reg          s_axi_wready,
    output wire [1:0]   s_axi_bresp,
    output reg          s_axi_bvalid,
    input  wire         s_axi_bready,
    input  wire [6:0]   s_axi_araddr,
    input  wire         s_axi_arvalid,
    output reg          s_axi_arready,
    output reg  [31:0]  s_axi_rdata,
    output wire [1:0]   s_axi_rresp,
    output reg          s_axi_rvalid,
    input  wire         s_axi_rready,
    // configuration out
    output reg  [31:0]  r_control,
    output reg  [31:0]  r_adapt,
    output reg  [31:0]  r_levels,
    output reg  [5:0]   r_wsel,
    output reg          r_clr_tog,
    // status in (already in this clock domain)
    input  wire [351:0] st
);
    localparam [31:0] ID      = 32'h50494A35;
    localparam [31:0] VERSION = 32'h00050001;

    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    reg [6:0]  awaddr_l;
    reg        aw_ok, w_ok;
    reg [31:0] wdata_l;
    reg [3:0]  wstrb_l;

    function [31:0] merge;
        input [31:0] old, nw;
        input [3:0]  be;
        integer b;
        begin
            for (b = 0; b < 4; b = b + 1)
                merge[b*8 +: 8] = be[b] ? nw[b*8 +: 8] : old[b*8 +: 8];
        end
    endfunction

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0; s_axi_wready <= 1'b0; s_axi_bvalid <= 1'b0;
            aw_ok <= 1'b0; w_ok <= 1'b0;
            r_control <= 32'h0000043B;
            r_adapt   <= 32'h0008C600;
            r_levels  <= 32'h0B1007F8;
            r_wsel    <= 6'd0;
            r_clr_tog <= 1'b0;
        end else begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            if (s_axi_awvalid && !aw_ok && !s_axi_bvalid) begin
                awaddr_l <= s_axi_awaddr; aw_ok <= 1'b1; s_axi_awready <= 1'b1;
            end
            if (s_axi_wvalid && !w_ok && !s_axi_bvalid) begin
                wdata_l <= s_axi_wdata; wstrb_l <= s_axi_wstrb; w_ok <= 1'b1; s_axi_wready <= 1'b1;
            end
            if (aw_ok && w_ok) begin
                aw_ok <= 1'b0; w_ok <= 1'b0;
                s_axi_bvalid <= 1'b1;
                case (awaddr_l[6:2])
                    5'h02: r_control <= merge(r_control, wdata_l, wstrb_l);
                    5'h03: r_adapt   <= merge(r_adapt,   wdata_l, wstrb_l);
                    5'h04: r_levels  <= merge(r_levels,  wdata_l, wstrb_l);
                    5'h05: r_wsel    <= wdata_l[5:0];
                    5'h06: if (wdata_l[0]) r_clr_tog <= ~r_clr_tog;
                    default: ;
                endcase
            end
            if (s_axi_bvalid && s_axi_bready)
                s_axi_bvalid <= 1'b0;
        end
    end

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0; s_axi_rvalid <= 1'b0; s_axi_rdata <= 0;
        end else begin
            s_axi_arready <= 1'b0;
            if (s_axi_arvalid && !s_axi_arready && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                case (s_axi_araddr[6:2])
                    5'h00: s_axi_rdata <= ID;
                    5'h01: s_axi_rdata <= VERSION;
                    5'h02: s_axi_rdata <= r_control;
                    5'h03: s_axi_rdata <= r_adapt;
                    5'h04: s_axi_rdata <= r_levels;
                    5'h05: s_axi_rdata <= {26'd0, r_wsel};
                    5'h08: s_axi_rdata <= st[ 0*32 +: 32];
                    5'h09: s_axi_rdata <= st[ 1*32 +: 32];
                    5'h0A: s_axi_rdata <= st[ 2*32 +: 32];
                    5'h0B: s_axi_rdata <= st[ 3*32 +: 32];
                    5'h0C: s_axi_rdata <= st[ 4*32 +: 32];
                    5'h0D: s_axi_rdata <= st[ 5*32 +: 32];
                    5'h0E: s_axi_rdata <= st[ 6*32 +: 32];
                    5'h0F: s_axi_rdata <= st[ 7*32 +: 32];
                    5'h10: s_axi_rdata <= st[ 8*32 +: 32];
                    5'h11: s_axi_rdata <= st[ 9*32 +: 32];
                    5'h12: s_axi_rdata <= st[10*32 +: 32];
                    default: s_axi_rdata <= 32'h0;
                endcase
            end
            if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 1'b0;
        end
    end
endmodule
