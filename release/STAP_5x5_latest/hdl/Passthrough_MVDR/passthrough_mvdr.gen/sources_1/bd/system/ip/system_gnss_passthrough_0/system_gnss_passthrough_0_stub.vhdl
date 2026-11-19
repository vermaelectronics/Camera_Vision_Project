-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.2 (lin64) Build 4029153 Fri Oct 13 20:13:54 MDT 2023
-- Date        : Wed Nov 11 18:33:27 2026
-- Host        : nishant-kumar-desktop running 64-bit Ubuntu 20.04.6 LTS
-- Command     : write_vhdl -force -mode synth_stub
--               /home/nishant/AntSDR/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_MVDR_STAP/passthrough_mvdr.gen/sources_1/bd/system/ip/system_gnss_passthrough_0/system_gnss_passthrough_0_stub.vhdl
-- Design      : system_gnss_passthrough_0
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7z020clg400-2
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity system_gnss_passthrough_0 is
  Port ( 
    clk : in STD_LOGIC;
    rst : in STD_LOGIC;
    adc_enable_i0 : in STD_LOGIC;
    adc_valid_i0 : in STD_LOGIC;
    adc_data_i0 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    adc_enable_q0 : in STD_LOGIC;
    adc_valid_q0 : in STD_LOGIC;
    adc_data_q0 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    adc_enable_i1 : in STD_LOGIC;
    adc_valid_i1 : in STD_LOGIC;
    adc_data_i1 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    adc_enable_q1 : in STD_LOGIC;
    adc_valid_q1 : in STD_LOGIC;
    adc_data_q1 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    dac_enable_i0 : in STD_LOGIC;
    dac_valid_i0 : in STD_LOGIC;
    dac_enable_q0 : in STD_LOGIC;
    dac_valid_q0 : in STD_LOGIC;
    dac_enable_i1 : in STD_LOGIC;
    dac_valid_i1 : in STD_LOGIC;
    dac_enable_q1 : in STD_LOGIC;
    dac_valid_q1 : in STD_LOGIC;
    dma_dac_data_i0 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    dma_dac_data_q0 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    dma_dac_data_i1 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    dma_dac_data_q1 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    dac_data_i0 : out STD_LOGIC_VECTOR ( 15 downto 0 );
    dac_data_q0 : out STD_LOGIC_VECTOR ( 15 downto 0 );
    dac_data_i1 : out STD_LOGIC_VECTOR ( 15 downto 0 );
    dac_data_q1 : out STD_LOGIC_VECTOR ( 15 downto 0 );
    s_axi_aclk : in STD_LOGIC;
    s_axi_aresetn : in STD_LOGIC;
    s_axi_awvalid : in STD_LOGIC;
    s_axi_awaddr : in STD_LOGIC_VECTOR ( 15 downto 0 );
    s_axi_awprot : in STD_LOGIC_VECTOR ( 2 downto 0 );
    s_axi_awready : out STD_LOGIC;
    s_axi_wvalid : in STD_LOGIC;
    s_axi_wdata : in STD_LOGIC_VECTOR ( 31 downto 0 );
    s_axi_wstrb : in STD_LOGIC_VECTOR ( 3 downto 0 );
    s_axi_wready : out STD_LOGIC;
    s_axi_bvalid : out STD_LOGIC;
    s_axi_bresp : out STD_LOGIC_VECTOR ( 1 downto 0 );
    s_axi_bready : in STD_LOGIC;
    s_axi_arvalid : in STD_LOGIC;
    s_axi_araddr : in STD_LOGIC_VECTOR ( 15 downto 0 );
    s_axi_arprot : in STD_LOGIC_VECTOR ( 2 downto 0 );
    s_axi_arready : out STD_LOGIC;
    s_axi_rvalid : out STD_LOGIC;
    s_axi_rresp : out STD_LOGIC_VECTOR ( 1 downto 0 );
    s_axi_rdata : out STD_LOGIC_VECTOR ( 31 downto 0 );
    s_axi_rready : in STD_LOGIC
  );

end system_gnss_passthrough_0;

architecture stub of system_gnss_passthrough_0 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clk,rst,adc_enable_i0,adc_valid_i0,adc_data_i0[15:0],adc_enable_q0,adc_valid_q0,adc_data_q0[15:0],adc_enable_i1,adc_valid_i1,adc_data_i1[15:0],adc_enable_q1,adc_valid_q1,adc_data_q1[15:0],dac_enable_i0,dac_valid_i0,dac_enable_q0,dac_valid_q0,dac_enable_i1,dac_valid_i1,dac_enable_q1,dac_valid_q1,dma_dac_data_i0[15:0],dma_dac_data_q0[15:0],dma_dac_data_i1[15:0],dma_dac_data_q1[15:0],dac_data_i0[15:0],dac_data_q0[15:0],dac_data_i1[15:0],dac_data_q1[15:0],s_axi_aclk,s_axi_aresetn,s_axi_awvalid,s_axi_awaddr[15:0],s_axi_awprot[2:0],s_axi_awready,s_axi_wvalid,s_axi_wdata[31:0],s_axi_wstrb[3:0],s_axi_wready,s_axi_bvalid,s_axi_bresp[1:0],s_axi_bready,s_axi_arvalid,s_axi_araddr[15:0],s_axi_arprot[2:0],s_axi_arready,s_axi_rvalid,s_axi_rresp[1:0],s_axi_rdata[31:0],s_axi_rready";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "gnss_passthrough_bd,Vivado 2023.2";
begin
end;
