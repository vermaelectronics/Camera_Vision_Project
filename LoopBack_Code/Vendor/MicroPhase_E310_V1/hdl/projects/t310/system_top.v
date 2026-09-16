// ***************************************************************************
// ***************************************************************************
// Copyright 2014 - 2017 (c) Analog Devices, Inc. All rights reserved.
//
// In this HDL repository, there are many different and unique modules, consisting
// of various HDL (Verilog or VHDL) components. The individual modules are
// developed independently, and may be accompanied by separate and unique license
// terms.
//
// The user should read each of these license terms, and understand the
// freedoms and responsibilities that he or she has by using this source/core.
//
// This core is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE.
//
// Redistribution and use of source or resulting binaries, with or without modification
// of this file, are permitted under one of the following two license terms:
//
//   1. The GNU General Public License version 2 as published by the
//      Free Software Foundation, which can be found in the top level directory
//      of this repository (LICENSE_GPL2), and also online at:
//      <https://www.gnu.org/licenses/old-licenses/gpl-2.0.html>
//
// OR
//
//   2. An ADI specific BSD license, which can be found in the top level directory
//      of this repository (LICENSE_ADIBSD), and also on-line at:
//      https://github.com/analogdevicesinc/hdl/blob/master/LICENSE_ADIBSD
//      This will allow to generate bit files and not release the source code,
//      as long as it attaches to an ADI device.
//
// ***************************************************************************
// ***************************************************************************

`timescale 1ns/100ps

module system_top (

  inout       [ 14:0]     ddr_addr,
  inout       [ 2:0]      ddr_ba,
  inout                   ddr_cas_n,
  inout                   ddr_ck_n,
  inout                   ddr_ck_p,
  inout                   ddr_cke,
  inout                   ddr_cs_n,
  inout       [ 3:0]      ddr_dm,
  inout       [ 31:0]     ddr_dq,
  inout       [ 3:0]      ddr_dqs_n,
  inout       [ 3:0]      ddr_dqs_p,
  inout                   ddr_odt,
  inout                   ddr_ras_n,
  inout                   ddr_reset_n,
  inout                   ddr_we_n,

  inout                   fixed_io_ddr_vrn,
  inout                   fixed_io_ddr_vrp,
  inout       [ 53:0]     fixed_io_mio,
  inout                   fixed_io_ps_clk,
  inout                   fixed_io_ps_porb,
  inout                   fixed_io_ps_srstb,


  input                   rx_clk_in_0_p,
  input                   rx_clk_in_0_n,
  input                   rx_frame_in_0_p,
  input                   rx_frame_in_0_n,
  input       [ 5:0]      rx_data_in_0_p,
  input       [ 5:0]      rx_data_in_0_n,
  output                  tx_clk_out_0_p,
  output                  tx_clk_out_0_n,
  output                  tx_frame_out_0_p,
  output                  tx_frame_out_0_n,
  output      [ 5:0]      tx_data_out_0_p,
  output      [ 5:0]      tx_data_out_0_n,
  inout       [ 7:0]      gpio_status_0,
  inout       [ 3:0]      gpio_ctl_0,
  inout                   gpio_en_agc_0,
  output                  mcs_sync_0,
  output                  mcs_sync_1,
  inout                   gpio_resetb_0,
  output                  enable_0,
  output                  txnrx_0,

  inout                   gpio_calsw_1_0,
  inout                   gpio_calsw_2_0,


  input                   rx_clk_in_1_p,
  input                   rx_clk_in_1_n,
  input                   rx_frame_in_1_p,
  input                   rx_frame_in_1_n,
  input       [ 5:0]      rx_data_in_1_p,
  input       [ 5:0]      rx_data_in_1_n,
  output                  tx_clk_out_1_p,
  output                  tx_clk_out_1_n,
  output                  tx_frame_out_1_p,
  output                  tx_frame_out_1_n,
  output      [ 5:0]      tx_data_out_1_p,
  output      [ 5:0]      tx_data_out_1_n,
  inout       [ 7:0]      gpio_status_1,
  inout       [ 3:0]      gpio_ctl_1,
  inout                   gpio_en_agc_1,
  inout                   gpio_resetb_1,
  output                  enable_1,
  output                  txnrx_1,

  input                   ref_clk         ,

  output                  CH1_FE_TXRX1_SEL1 ,
  output                  CH1_FE_TXRX2_SEL1 ,
  output                  CH1_FE_RX1_SEL1 ,
  output                  CH1_FE_RX2_SEL1 ,
  output                  CH1_TX1_AMP_EN,
  output                  CH1_TX2_AMP_EN,

  output                  CH2_FE_TXRX1_SEL1 ,
  output                  CH2_FE_TXRX2_SEL1 ,
  output                  CH2_FE_RX1_SEL1 ,
  output                  CH2_FE_RX2_SEL1 ,
  output                  CH2_TX1_AMP_EN,
  output                  CH2_TX2_AMP_EN,



  output                  spi_ad9361_1,
  output                  spi_clk_1,
  output                  spi_mosi_1,
  input                   spi_miso_1,

  output                  spi_ad9361_0,
  output                  spi_clk_0,
  output                  spi_mosi_0,
  input                   spi_miso_0
  );

  // internal registers

  reg     [  2:0] mcs_sync_m = 'd0;
  reg     mcs_sync;

  // internal signals
  wire    [1:0]   cal_sw;
  wire            sys_100m_resetn;
  wire            ref_clk_s;
  wire    [ 63:0] gpio_i;
  wire    [ 63:0] gpio_o;
  wire    [ 63:0] gpio_t;
  wire    [ 2:0]  spi0_csn;
  wire            spi0_clk;
  wire            spi0_mosi;
  wire            spi0_miso;
  wire    [ 2:0]  spi1_csn;
  wire            spi1_clk;
  wire            spi1_mosi;
  wire            spi1_miso;
  wire            gpio_debug_1_0;
  wire            gpio_debug_2_0;
  wire            gpio_debug_3_1;
  wire            gpio_debug_4_1;
  wire            gpio_calsw_3_1;
  wire            gpio_calsw_4_1;
  wire            gpio_ad5355_rfen;
  wire            gpio_ad5355_lock;

  assign mcs_sync_0 = mcs_sync;
  assign mcs_sync_1 = mcs_sync;


  // multi-chip synchronization

  always @(posedge ref_clk or negedge sys_100m_resetn) begin
    if (sys_100m_resetn == 1'b0) begin
      mcs_sync_m <= 3'd0;
      mcs_sync <= 1'd0;
    end else begin
      mcs_sync_m <= {mcs_sync_m[1:0], gpio_o[45]};
      mcs_sync <= mcs_sync_m[2] & ~mcs_sync_m[1];
    end
  end

  assign cal_sw = gpio_o[54:53];

  // vio_0 your_instance_name (
  //   .clk(ref_clk),              // input wire clk
  //   .probe_in0(cal_sw)  // input wire [1 : 0] probe_in0
  // );

  // // instantiations

  // IBUFGDS i_ref_clk_ibuf (
  //   .I (ref_clk_p),
  //   .IB (ref_clk_n),
  //   .O (ref_clk_s));

  // BUFR #(.BUFR_DIVIDE("BYPASS")) i_ref_clk_rbuf (
  //   .CLR (1'b0),
  //   .CE (1'b1),
  //   .I (ref_clk_s),
  //   .O (ref_clk));

  assign CH1_TX1_AMP_EN = 1'b1;
  assign CH1_TX2_AMP_EN = 1'b1;
  assign CH1_FE_TXRX2_SEL1 = 1'b0;
  assign CH1_FE_TXRX1_SEL1 = 1'b1;
  assign CH1_FE_RX2_SEL1 = 1'b0;
  assign CH1_FE_RX1_SEL1 = 1'b1;

  assign CH2_TX1_AMP_EN = 1'b1;
  assign CH2_TX2_AMP_EN = 1'b1;
  assign CH2_FE_TXRX2_SEL1 = 1'b0;
  assign CH2_FE_TXRX1_SEL1 = 1'b1;
  assign CH2_FE_RX2_SEL1 = 1'b0;
  assign CH2_FE_RX1_SEL1 = 1'b1;

  ad_iobuf #(.DATA_WIDTH(42)) i_iobuf (
    .dio_t ({gpio_t[59:46], gpio_t[43:16]}),
    .dio_i ({gpio_o[59:46], gpio_o[43:16]}),
    .dio_o ({gpio_i[59:46], gpio_i[43:16]}),
    .dio_p ({ gpio_resetb_1,    // 59
              gpio_ad5355_lock, // 58
              gpio_ad5355_rfen, // 57
              gpio_calsw_4_1,   // 56
              gpio_calsw_3_1,   // 55
              gpio_calsw_2_0,   // 54
              gpio_calsw_1_0,   // 53
              gpio_txnrx_1,     // 52
              gpio_enable_1,    // 51
              gpio_en_agc_1,    // 50
              gpio_txnrx_0,     // 49
              gpio_enable_0,    // 48
              gpio_en_agc_0,    // 47
              gpio_resetb_0,    // 46
              gpio_debug_4_1,   // 43
              gpio_debug_3_1,   // 42
              gpio_debug_2_0,   // 41
              gpio_debug_1_0,   // 40
              gpio_ctl_1,       // 39:36
              gpio_ctl_0,       // 35:32
              gpio_status_1,    // 31:24
              gpio_status_0     // 23:16
              }));

  assign spi_ad9361_0 = spi0_csn[0];
  assign spi_ad9361_1 = spi0_csn[1];
  assign spi_clk_0 =  spi0_clk;
  assign spi_mosi_0 = spi0_mosi;
  assign spi_clk_1 = spi0_clk;
  assign spi_mosi_1 = spi0_mosi;
  assign spi0_miso = (spi0_csn[0]==1'b0) ? spi_miso_0 : (spi0_csn[1]==1'b0) ? spi_miso_1 : 1'b1;
  assign gpio_i[63:60] = gpio_o[63:60];
  assign gpio_i[45:44] = gpio_o[45:44];
  assign gpio_i[15] = gpio_o[15];

  system_wrapper i_system_wrapper (
    .ddr_addr (ddr_addr),
    .ddr_ba (ddr_ba),
    .ddr_cas_n (ddr_cas_n),
    .ddr_ck_n (ddr_ck_n),
    .ddr_ck_p (ddr_ck_p),
    .ddr_cke (ddr_cke),
    .ddr_cs_n (ddr_cs_n),
    .ddr_dm (ddr_dm),
    .ddr_dq (ddr_dq),
    .ddr_dqs_n (ddr_dqs_n),
    .ddr_dqs_p (ddr_dqs_p),
    .ddr_odt (ddr_odt),
    .ddr_ras_n (ddr_ras_n),
    .ddr_reset_n (ddr_reset_n),
    .ddr_we_n (ddr_we_n),
    .fixed_io_ddr_vrn (fixed_io_ddr_vrn),
    .fixed_io_ddr_vrp (fixed_io_ddr_vrp),
    .fixed_io_mio (fixed_io_mio),
    .fixed_io_ps_clk (fixed_io_ps_clk),
    .fixed_io_ps_porb (fixed_io_ps_porb),
    .fixed_io_ps_srstb (fixed_io_ps_srstb),
    .ref_clk(),
    .gpio_i (gpio_i),
    .gpio_o (gpio_o),
    .gpio_t (gpio_t),
    .rx_clk_in_0_n (rx_clk_in_0_n),
    .rx_clk_in_0_p (rx_clk_in_0_p),
    .rx_clk_in_1_n (rx_clk_in_1_n),
    .rx_clk_in_1_p (rx_clk_in_1_p),
    .rx_data_in_0_n (rx_data_in_0_n),
    .rx_data_in_0_p (rx_data_in_0_p),
    .rx_data_in_1_n (rx_data_in_1_n),
    .rx_data_in_1_p (rx_data_in_1_p),
    .rx_frame_in_0_n (rx_frame_in_0_n),
    .rx_frame_in_0_p (rx_frame_in_0_p),
    .rx_frame_in_1_n (rx_frame_in_1_n),
    .rx_frame_in_1_p (rx_frame_in_1_p),
    .spi0_clk_i (spi0_clk),
    .spi0_clk_o (spi0_clk),
    .spi0_csn_0_o (spi0_csn[0]),
    .spi0_csn_1_o (spi0_csn[1]),
    .spi0_csn_2_o (spi0_csn[2]),
    .spi0_csn_i (1'b1),
    .spi0_sdi_i (spi0_miso),
    .spi0_sdo_i (spi0_mosi),
    .spi0_sdo_o (spi0_mosi),
    .spi1_clk_i (spi1_clk),
    .spi1_clk_o (spi1_clk),
    .spi1_csn_0_o (spi1_csn[0]),
    .spi1_csn_1_o (spi1_csn[1]),
    .spi1_csn_2_o (spi1_csn[2]),
    .spi1_csn_i (1'b1),
    .spi1_sdi_i (1'b1),
    .spi1_sdo_i (spi1_mosi),
    .spi1_sdo_o (spi1_mosi),
    .sys_100m_resetn (sys_100m_resetn),
    .tx_clk_out_0_n (tx_clk_out_0_n),
    .tx_clk_out_0_p (tx_clk_out_0_p),
    .tx_clk_out_1_n (tx_clk_out_1_n),
    .tx_clk_out_1_p (tx_clk_out_1_p),
    .tx_data_out_0_n (tx_data_out_0_n),
    .tx_data_out_0_p (tx_data_out_0_p),
    .tx_data_out_1_n (tx_data_out_1_n),
    .tx_data_out_1_p (tx_data_out_1_p),
    .tx_frame_out_0_n (tx_frame_out_0_n),
    .tx_frame_out_0_p (tx_frame_out_0_p),
    .tx_frame_out_1_n (tx_frame_out_1_n),
    .tx_frame_out_1_p (tx_frame_out_1_p),
    .txnrx_0 (txnrx_0),
    .enable_0 (enable_0),
    .up_enable_0 (gpio_enable_0),
    .up_txnrx_0 (gpio_txnrx_0),
    .txnrx_1 (txnrx_1),
    .enable_1 (enable_1),
    .up_enable_1 (gpio_enable_1),
    .up_txnrx_1 (gpio_txnrx_1));

endmodule

// ***************************************************************************
// ***************************************************************************
