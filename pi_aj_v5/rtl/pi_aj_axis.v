// ============================================================================
// pi_aj_axis.v  --  PI-AJ v5 as a drop-in replacement for smi_pi_0
//                   (AntSDR E310 / LoopBack_Code block design)
//
// Same stream interfaces as the SMI-PI HLS core, so the block design keeps its
// two asynchronous AXI-Stream FIFOs and gnss_passthrough unchanged:
//   s_axis_x : {rx2_q, rx2_i, rx1_q, rx1_i}  4 x 16 bit, 12-bit right-aligned
//   m_axis_y : {y_q, y_i}                    2 x 16 bit, 12-bit right-aligned,
//                                            saturated to -2048..2047
// One output sample per input sample. Everything (streams and AXI4-Lite) is on
// aclk = sys_cpu_clk (100 MHz). The input is accepted at most once every
// CAPS[31:24] clocks (3 with NL=10), which sets the maximum sample rate:
// 100 MHz / 3 = 33.3 MS/s. Faster streams back up in smi_fifo_in and
// gnss_passthrough flags STATUS[10] (smi_drop).
//
// Output level: TX1 = 2^man_shift x (PI output), the same convention as the
// SMI-PI make-up shift (firmware sets ADAPT[24:20]); with CONTROL[0] = 0 the
// core is bypassed and TX1 = RX1 exactly.
// ============================================================================
`timescale 1ns / 1ps
module pi_aj_axis #(
    parameter NL         = 10,   // PI lanes: 10 -> 3 clocks/sample (33 MS/s @100 MHz)
    parameter DELAY      = 4,    // PI update delay (>= (K+9)/clocks-per-sample)
    parameter L          = 5,
    parameter WL         = 1,
    parameter HAS_NOTCH2 = 0     // second CW notch (off by default: DSP budget)
) (
    input  wire         aclk,
    input  wire         aresetn,
    // samples in (from smi_fifo_in)
    input  wire [63:0]  s_axis_x_tdata,
    input  wire         s_axis_x_tvalid,
    output wire         s_axis_x_tready,
    // samples out (to smi_fifo_out)
    output reg  [31:0]  m_axis_y_tdata,
    output reg          m_axis_y_tvalid,
    input  wire         m_axis_y_tready,
    // AXI4-Lite
    input  wire [11:0]  s_axi_awaddr,
    input  wire [2:0]   s_axi_awprot,
    input  wire         s_axi_awvalid,
    output wire         s_axi_awready,
    input  wire [31:0]  s_axi_wdata,
    input  wire [3:0]   s_axi_wstrb,
    input  wire         s_axi_wvalid,
    output wire         s_axi_wready,
    output wire [1:0]   s_axi_bresp,
    output wire         s_axi_bvalid,
    input  wire         s_axi_bready,
    input  wire [11:0]  s_axi_araddr,
    input  wire [2:0]   s_axi_arprot,
    input  wire         s_axi_arvalid,
    output wire         s_axi_arready,
    output wire [31:0]  s_axi_rdata,
    output wire [1:0]   s_axi_rresp,
    output wire         s_axi_rvalid,
    input  wire         s_axi_rready
);
    localparam NWT  = WL ? (4*L - 1) : (2*L - 1);
    localparam KCYC = (NWT + NL - 1) / NL;
    localparam CPS  = KCYC + 1;              // minimum clocks per sample

    reg rst = 1'b1;
    always @(posedge aclk) rst <= ~aresetn;

    // ---- input pacing --------------------------------------------------------
    reg  [3:0] gap;
    assign s_axis_x_tready = ~rst & (gap == 4'd0);
    wire accept = s_axis_x_tvalid & s_axis_x_tready;
    always @(posedge aclk) begin
        if (rst)         gap <= 4'd0;
        else if (accept) gap <= CPS - 1;
        else if (gap != 0) gap <= gap - 4'd1;
    end

    // ---- core ------------------------------------------------------------------
    wire        dac_valid;
    wire [15:0] dac_i0, dac_q0;

    pi_aj_top #(.IN_SHIFT(4), .L(L), .NL(NL), .DELAY(DELAY), .WL(WL),
                .HAS_NOTCH2(HAS_NOTCH2)) u_core (
        .clk(aclk), .rst(rst),
        .adc_valid(accept),
        .adc_i0(s_axis_x_tdata[15:0]),  .adc_q0(s_axis_x_tdata[31:16]),
        .adc_i1(s_axis_x_tdata[47:32]), .adc_q1(s_axis_x_tdata[63:48]),
        .dac_valid(dac_valid), .dac_i0(dac_i0), .dac_q0(dac_q0),
        .s_axi_aclk(aclk), .s_axi_aresetn(aresetn),
        .s_axi_awaddr(s_axi_awaddr[6:0]), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready), .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready), .s_axi_araddr(s_axi_araddr[6:0]), .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready), .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready)
    );

    // ---- 16-bit DAC word -> 12-bit right-aligned, rounded, saturated ----------
    function [15:0] to12;
        input [15:0] v;
        reg signed [16:0] r;
        begin
            r = ($signed({v[15], v}) + 17'sd8) >>> 4;
            if (r > 17'sd2047)       to12 = 16'h07FF;
            else if (r < -17'sd2048) to12 = 16'hF800;
            else                     to12 = r[15:0];
        end
    endfunction

    always @(posedge aclk) begin
        if (rst) begin
            m_axis_y_tvalid <= 1'b0;
            m_axis_y_tdata  <= 32'd0;
        end else begin
            // the FIFO behind is always ready (gnss_passthrough reads it at the
            // sample rate); a sample is never held back
            m_axis_y_tvalid <= dac_valid;
            if (dac_valid)
                m_axis_y_tdata <= {to12(dac_q0), to12(dac_i0)};
        end
    end
endmodule
