// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.2 (lin64) Build 4029153 Fri Oct 13 20:13:54 MDT 2023
// Date        : Wed Nov 11 18:33:27 2026
// Host        : nishant-kumar-desktop running 64-bit Ubuntu 20.04.6 LTS
// Command     : write_verilog -force -mode synth_stub
//               /home/nishant/AntSDR/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_MVDR_STAP/passthrough_mvdr.gen/sources_1/bd/system/ip/system_gnss_passthrough_0/system_gnss_passthrough_0_stub.v
// Design      : system_gnss_passthrough_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg400-2
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "gnss_passthrough_bd,Vivado 2023.2" *)
module system_gnss_passthrough_0(clk, rst, adc_enable_i0, adc_valid_i0, 
  adc_data_i0, adc_enable_q0, adc_valid_q0, adc_data_q0, adc_enable_i1, adc_valid_i1, 
  adc_data_i1, adc_enable_q1, adc_valid_q1, adc_data_q1, dac_enable_i0, dac_valid_i0, 
  dac_enable_q0, dac_valid_q0, dac_enable_i1, dac_valid_i1, dac_enable_q1, dac_valid_q1, 
  dma_dac_data_i0, dma_dac_data_q0, dma_dac_data_i1, dma_dac_data_q1, dac_data_i0, 
  dac_data_q0, dac_data_i1, dac_data_q1, s_axi_aclk, s_axi_aresetn, s_axi_awvalid, 
  s_axi_awaddr, s_axi_awprot, s_axi_awready, s_axi_wvalid, s_axi_wdata, s_axi_wstrb, 
  s_axi_wready, s_axi_bvalid, s_axi_bresp, s_axi_bready, s_axi_arvalid, s_axi_araddr, 
  s_axi_arprot, s_axi_arready, s_axi_rvalid, s_axi_rresp, s_axi_rdata, s_axi_rready)
/* synthesis syn_black_box black_box_pad_pin="rst,adc_enable_i0,adc_valid_i0,adc_data_i0[15:0],adc_enable_q0,adc_valid_q0,adc_data_q0[15:0],adc_enable_i1,adc_valid_i1,adc_data_i1[15:0],adc_enable_q1,adc_valid_q1,adc_data_q1[15:0],dac_enable_i0,dac_valid_i0,dac_enable_q0,dac_valid_q0,dac_enable_i1,dac_valid_i1,dac_enable_q1,dac_valid_q1,dma_dac_data_i0[15:0],dma_dac_data_q0[15:0],dma_dac_data_i1[15:0],dma_dac_data_q1[15:0],dac_data_i0[15:0],dac_data_q0[15:0],dac_data_i1[15:0],dac_data_q1[15:0],s_axi_aresetn,s_axi_awvalid,s_axi_awaddr[15:0],s_axi_awprot[2:0],s_axi_awready,s_axi_wvalid,s_axi_wdata[31:0],s_axi_wstrb[3:0],s_axi_wready,s_axi_bvalid,s_axi_bresp[1:0],s_axi_bready,s_axi_arvalid,s_axi_araddr[15:0],s_axi_arprot[2:0],s_axi_arready,s_axi_rvalid,s_axi_rresp[1:0],s_axi_rdata[31:0],s_axi_rready" */
/* synthesis syn_force_seq_prim="clk" */
/* synthesis syn_force_seq_prim="s_axi_aclk" */;
  input clk /* synthesis syn_isclock = 1 */;
  input rst;
  input adc_enable_i0;
  input adc_valid_i0;
  input [15:0]adc_data_i0;
  input adc_enable_q0;
  input adc_valid_q0;
  input [15:0]adc_data_q0;
  input adc_enable_i1;
  input adc_valid_i1;
  input [15:0]adc_data_i1;
  input adc_enable_q1;
  input adc_valid_q1;
  input [15:0]adc_data_q1;
  input dac_enable_i0;
  input dac_valid_i0;
  input dac_enable_q0;
  input dac_valid_q0;
  input dac_enable_i1;
  input dac_valid_i1;
  input dac_enable_q1;
  input dac_valid_q1;
  input [15:0]dma_dac_data_i0;
  input [15:0]dma_dac_data_q0;
  input [15:0]dma_dac_data_i1;
  input [15:0]dma_dac_data_q1;
  output [15:0]dac_data_i0;
  output [15:0]dac_data_q0;
  output [15:0]dac_data_i1;
  output [15:0]dac_data_q1;
  input s_axi_aclk /* synthesis syn_isclock = 1 */;
  input s_axi_aresetn;
  input s_axi_awvalid;
  input [15:0]s_axi_awaddr;
  input [2:0]s_axi_awprot;
  output s_axi_awready;
  input s_axi_wvalid;
  input [31:0]s_axi_wdata;
  input [3:0]s_axi_wstrb;
  output s_axi_wready;
  output s_axi_bvalid;
  output [1:0]s_axi_bresp;
  input s_axi_bready;
  input s_axi_arvalid;
  input [15:0]s_axi_araddr;
  input [2:0]s_axi_arprot;
  output s_axi_arready;
  output s_axi_rvalid;
  output [1:0]s_axi_rresp;
  output [31:0]s_axi_rdata;
  input s_axi_rready;
endmodule
