// ============================================================================
//  gnss_passthrough_bd.v -- block-design wrapper for gnss_passthrough.v
//
//  gnss_passthrough.v is the proven top of the working MVDR/STAP design
//  (MVDR2 project, rooftop barrage tests) and is used here UNCHANGED.
//  This wrapper only adds the interface attributes Vivado needs for a
//  module-reference cell (clock/reset association and polarity), so the
//  block design can be built from Tcl. No logic.
// ============================================================================
`timescale 1ns/100ps

module gnss_passthrough_bd #(
  parameter FIFO_ADDR_WIDTH = 5,
  parameter STAP_LMS        = 0
) (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_RESET rst" *)
  input               clk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
  input               rst,

  input               adc_enable_i0,
  input               adc_valid_i0,
  input      [15:0]   adc_data_i0,
  input               adc_enable_q0,
  input               adc_valid_q0,
  input      [15:0]   adc_data_q0,
  input               adc_enable_i1,
  input               adc_valid_i1,
  input      [15:0]   adc_data_i1,
  input               adc_enable_q1,
  input               adc_valid_q1,
  input      [15:0]   adc_data_q1,

  input               dac_enable_i0,
  input               dac_valid_i0,
  input               dac_enable_q0,
  input               dac_valid_q0,
  input               dac_enable_i1,
  input               dac_valid_i1,
  input               dac_enable_q1,
  input               dac_valid_q1,

  input      [15:0]   dma_dac_data_i0,
  input      [15:0]   dma_dac_data_q0,
  input      [15:0]   dma_dac_data_i1,
  input      [15:0]   dma_dac_data_q1,

  output     [15:0]   dac_data_i0,
  output     [15:0]   dac_data_q0,
  output     [15:0]   dac_data_i1,
  output     [15:0]   dac_data_q1,

  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
  input               s_axi_aclk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input               s_axi_aresetn,
  input               s_axi_awvalid,
  input      [15:0]   s_axi_awaddr,
  input      [ 2:0]   s_axi_awprot,
  output              s_axi_awready,
  input               s_axi_wvalid,
  input      [31:0]   s_axi_wdata,
  input      [ 3:0]   s_axi_wstrb,
  output              s_axi_wready,
  output              s_axi_bvalid,
  output     [ 1:0]   s_axi_bresp,
  input               s_axi_bready,
  input               s_axi_arvalid,
  input      [15:0]   s_axi_araddr,
  input      [ 2:0]   s_axi_arprot,
  output              s_axi_arready,
  output              s_axi_rvalid,
  output     [ 1:0]   s_axi_rresp,
  output     [31:0]   s_axi_rdata,
  input               s_axi_rready
);

  gnss_passthrough #(.FIFO_ADDR_WIDTH(FIFO_ADDR_WIDTH), .STAP_LMS(STAP_LMS)) u_pt (
    .clk(clk), .rst(rst),
    .adc_enable_i0(adc_enable_i0), .adc_valid_i0(adc_valid_i0), .adc_data_i0(adc_data_i0),
    .adc_enable_q0(adc_enable_q0), .adc_valid_q0(adc_valid_q0), .adc_data_q0(adc_data_q0),
    .adc_enable_i1(adc_enable_i1), .adc_valid_i1(adc_valid_i1), .adc_data_i1(adc_data_i1),
    .adc_enable_q1(adc_enable_q1), .adc_valid_q1(adc_valid_q1), .adc_data_q1(adc_data_q1),
    .dac_enable_i0(dac_enable_i0), .dac_valid_i0(dac_valid_i0),
    .dac_enable_q0(dac_enable_q0), .dac_valid_q0(dac_valid_q0),
    .dac_enable_i1(dac_enable_i1), .dac_valid_i1(dac_valid_i1),
    .dac_enable_q1(dac_enable_q1), .dac_valid_q1(dac_valid_q1),
    .dma_dac_data_i0(dma_dac_data_i0), .dma_dac_data_q0(dma_dac_data_q0),
    .dma_dac_data_i1(dma_dac_data_i1), .dma_dac_data_q1(dma_dac_data_q1),
    .dac_data_i0(dac_data_i0), .dac_data_q0(dac_data_q0),
    .dac_data_i1(dac_data_i1), .dac_data_q1(dac_data_q1),
    .s_axi_aclk(s_axi_aclk), .s_axi_aresetn(s_axi_aresetn),
    .s_axi_awvalid(s_axi_awvalid), .s_axi_awaddr(s_axi_awaddr), .s_axi_awprot(s_axi_awprot),
    .s_axi_awready(s_axi_awready),
    .s_axi_wvalid(s_axi_wvalid), .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb),
    .s_axi_wready(s_axi_wready),
    .s_axi_bvalid(s_axi_bvalid), .s_axi_bresp(s_axi_bresp), .s_axi_bready(s_axi_bready),
    .s_axi_arvalid(s_axi_arvalid), .s_axi_araddr(s_axi_araddr), .s_axi_arprot(s_axi_arprot),
    .s_axi_arready(s_axi_arready),
    .s_axi_rvalid(s_axi_rvalid), .s_axi_rresp(s_axi_rresp), .s_axi_rdata(s_axi_rdata),
    .s_axi_rready(s_axi_rready));

endmodule
