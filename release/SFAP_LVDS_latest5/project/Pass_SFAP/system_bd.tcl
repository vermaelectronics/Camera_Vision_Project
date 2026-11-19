# ============================================================================
#  system_bd.tcl  --  ANTSDR E310, AD9361 LVDS 2R2T 61.44 MSPS
#                     Passthrough_LVDS + SFAP-PI v6.3 (256 bins) anti-jam
#
#  Identical to the working passthrough_lvds block design (antsdr-fw-patch
#  "ant" base, Vivado 2023.2, VCXO controller, axi_tdd, no FIR) except:
#    - passthrough_mux (EMIO GPIO 40) is replaced by gnss_passthrough
#      (AXI-Lite control + weight RAM, 8 KB) and the SFAP-PI HLS core sfap_pi_0
#      between two async AXI-Stream FIFOs (sys_cpu_clk, 100 MHz).
#    - gnss_passthrough with CONTROL[5]=0 (sfap-pi=0) is the same RX->TX
#      passthrough as passthrough_mux: {adc[11:0], 4'b0} -> DAC.
#  0x43C00000 stays axi_vcxo_ctrl; gnss_passthrough gets its own 8 KB slot.
# ============================================================================

source $ad_hdl_dir/library/axi_tdd/scripts/axi_tdd.tcl
set_property  ip_repo_paths  $ad_hdl_dir/library/ [current_project]

# ---- custom IP repository (gnss_passthrough + sfap_pi), built by build_all.sh
if {![info exists ::env(GNSS_CRPA_IP_REPO)]} {
  error "ERROR: environment variable GNSS_CRPA_IP_REPO is not set (normally\
         <SFAP_PassLVDS>/Build/ip_repo). Run the build through\
         Automation/Linux/build_all.sh, or export it before 'make'."
}
set gnss_ip_repo [file normalize $::env(GNSS_CRPA_IP_REPO)]
foreach gnss_ip {gnss_passthrough sfap_pi} {
  if {![file exists [file join $gnss_ip_repo $gnss_ip component.xml]]} {
    error "ERROR: custom IP $gnss_ip not found in $gnss_ip_repo\
           (run build_all.sh --from hls)."
  }
}
set_property ip_repo_paths [list [file normalize $ad_hdl_dir/library] $gnss_ip_repo] [current_project]
update_ip_catalog -rebuild

# Exact VLNVs: an older analog.com:user:gnss_passthrough (or sfap_pi) may
# also sit in the ADI library tree, so never look the IP up by name alone.
set gnss_vlnv antsdr:gnss:gnss_passthrough:3.0
set sfap_vlnv antsdr:gnss:sfap_pi:5.0
set gnss_ipdef [get_ipdefs -all -quiet -filter "VLNV == $gnss_vlnv"]
set sfap_ipdef [get_ipdefs -all -quiet -filter "VLNV == $sfap_vlnv"]
if {[llength $gnss_ipdef] == 0 || [llength $sfap_ipdef] == 0} {
  error "ERROR: $gnss_vlnv ('$gnss_ipdef') or $sfap_vlnv ('$sfap_ipdef')\
         not in the IP catalog after registering $gnss_ip_repo\
         (run build_all.sh --from hls)."
}
puts "SFAP_LVDS: custom IP: $gnss_ipdef  $sfap_ipdef"

# default ports

create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddrx_rtl:1.0 ddr
create_bd_intf_port -mode Master -vlnv xilinx.com:display_processing_system7:fixedio_rtl:1.0 fixed_io

create_bd_port -dir O spi0_csn_2_o
create_bd_port -dir O spi0_csn_1_o
create_bd_port -dir O spi0_csn_0_o
create_bd_port -dir I spi0_csn_i
create_bd_port -dir I spi0_clk_i
create_bd_port -dir O spi0_clk_o
create_bd_port -dir I spi0_sdo_i
create_bd_port -dir O spi0_sdo_o
create_bd_port -dir I spi0_sdi_i

create_bd_port -dir I -from 63 -to 0 gpio_i
create_bd_port -dir O -from 63 -to 0 gpio_o
create_bd_port -dir O -from 63 -to 0 gpio_t


create_bd_port -dir O txdata_o
create_bd_port -dir I tdd_ext_sync


create_bd_port -dir I CLKIN_10MHz
create_bd_port -dir I CLK_40MHz_FPGA
create_bd_port -dir O CLK_40M_DAC_DIN
create_bd_port -dir O CLK_40M_DAC_SCLK
create_bd_port -dir O CLK_40M_DAC_nSYNC
create_bd_port -dir I PPS_GPS
create_bd_port -dir I PPS_IN
create_bd_port -dir O PPS_LED
create_bd_port -dir O PPS_LOCKED
create_bd_port -dir O REF_10M_LOCKED

# ps7 settings

ad_ip_instance processing_system7 sys_ps7
ad_ip_parameter sys_ps7 CONFIG.PCW_PRESET_BANK0_VOLTAGE "LVCMOS 3.3V"
ad_ip_parameter sys_ps7 CONFIG.PCW_PRESET_BANK1_VOLTAGE "LVCMOS 1.8V"
ad_ip_parameter sys_ps7 CONFIG.PCW_PACKAGE_NAME clg400
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_MIO_GPIO_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_ENET0_IO "MIO 16 .. 27"
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_GRP_MDIO_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_GRP_MDIO_IO "MIO 52 .. 53"
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_RESET_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_RESET_IO "MIO 46"
ad_ip_parameter sys_ps7 CONFIG.PCW_SD0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_SDIO_PERIPHERAL_FREQMHZ 50
ad_ip_parameter sys_ps7 CONFIG.PCW_UART1_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UART1_UART1_IO {MIO 12 .. 13}
ad_ip_parameter sys_ps7 CONFIG.PCW_I2C0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_I2C0_I2C0_IO {MIO 10 .. 11}
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_RESET_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_RESET_IO "MIO 47"
ad_ip_parameter sys_ps7 CONFIG.PCW_QSPI_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_PARTNO {MT41J256M16 RE-125}
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_BUS_WIDTH {32 Bit}
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_USE_INTERNAL_VREF 0
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_TRAIN_WRITE_LEVEL 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_TRAIN_READ_GATE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_TRAIN_DATA_EYE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_0 0.048
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_1 0.050
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY0 0.241
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY1 0.240
ad_ip_parameter sys_ps7 CONFIG.PCW_TTC0_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_EN_CLK1_PORT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_EN_RST1_PORT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_EN_CLK2_PORT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_EN_RST2_PORT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ 100.0
ad_ip_parameter sys_ps7 CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ 200.0
ad_ip_parameter sys_ps7 CONFIG.PCW_FPGA2_PERIPHERAL_FREQMHZ 200.0
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_FABRIC_INTERRUPT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_IRQ_F2P_INTR 1
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_EMIO_GPIO_IO 64
ad_ip_parameter sys_ps7 CONFIG.PCW_IRQ_F2P_MODE REVERSE
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI0_SPI0_IO EMIO
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI1_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI1_SPI1_IO EMIO


ad_ip_instance xlconcat sys_concat_intc
ad_ip_parameter sys_concat_intc CONFIG.NUM_PORTS 16

ad_ip_instance proc_sys_reset sys_rstgen
ad_ip_parameter sys_rstgen CONFIG.C_EXT_RST_WIDTH 1

# system reset/clock definitions



set axi_vcxo_ctrl [ create_bd_cell -type ip -vlnv user.org:user:axi_vcxo_ctrl:1.0 axi_vcxo_ctrl ]
ad_connect axi_vcxo_ctrl/CLK_40M_DAC_DIN CLK_40M_DAC_DIN
ad_connect axi_vcxo_ctrl/CLK_40M_DAC_SCLK CLK_40M_DAC_SCLK
ad_connect axi_vcxo_ctrl/CLK_40M_DAC_nSYNC CLK_40M_DAC_nSYNC
ad_connect axi_vcxo_ctrl/CLKIN_10MHz CLKIN_10MHz
ad_connect axi_vcxo_ctrl/CLK_40MHz_FPGA CLK_40MHz_FPGA
ad_connect axi_vcxo_ctrl/PPS_GPS PPS_GPS
ad_connect axi_vcxo_ctrl/PPS_IN PPS_IN
ad_connect axi_vcxo_ctrl/PPS_LED PPS_LED
ad_connect axi_vcxo_ctrl/PPS_LOCKED PPS_LOCKED
ad_connect axi_vcxo_ctrl/REF_10M_LOCKED REF_10M_LOCKED




ad_connect  sys_cpu_clk sys_ps7/FCLK_CLK0
ad_connect  sys_200m_clk sys_ps7/FCLK_CLK1
ad_connect  sys_cpu_reset sys_rstgen/peripheral_reset
ad_connect  sys_cpu_resetn sys_rstgen/peripheral_aresetn
ad_connect  sys_cpu_clk sys_rstgen/slowest_sync_clk
ad_connect  sys_rstgen/ext_reset_in sys_ps7/FCLK_RESET0_N

# interface connections

ad_connect  ddr sys_ps7/DDR
ad_connect  gpio_i sys_ps7/GPIO_I
ad_connect  gpio_o sys_ps7/GPIO_O
ad_connect  gpio_t sys_ps7/GPIO_T
ad_connect  fixed_io sys_ps7/FIXED_IO

# ps7 spi connections

ad_connect  spi0_csn_2_o sys_ps7/SPI0_SS2_O
ad_connect  spi0_csn_1_o sys_ps7/SPI0_SS1_O
ad_connect  spi0_csn_0_o sys_ps7/SPI0_SS_O
ad_connect  spi0_csn_i sys_ps7/SPI0_SS_I
ad_connect  spi0_clk_i sys_ps7/SPI0_SCLK_I
ad_connect  spi0_clk_o sys_ps7/SPI0_SCLK_O
ad_connect  spi0_sdo_i sys_ps7/SPI0_MOSI_I
ad_connect  spi0_sdo_o sys_ps7/SPI0_MOSI_O
ad_connect  spi0_sdi_i sys_ps7/SPI0_MISO_I


# interrupts

ad_connect  sys_concat_intc/dout sys_ps7/IRQ_F2P
ad_connect  sys_concat_intc/In15 GND
ad_connect  sys_concat_intc/In14 GND
ad_connect  sys_concat_intc/In13 GND
ad_connect  sys_concat_intc/In12 GND
ad_connect  sys_concat_intc/In11 GND
ad_connect  sys_concat_intc/In10 GND
ad_connect  sys_concat_intc/In9 GND
ad_connect  sys_concat_intc/In8 GND
ad_connect  sys_concat_intc/In7 GND
ad_connect  sys_concat_intc/In6 GND
ad_connect  sys_concat_intc/In5 GND
ad_connect  sys_concat_intc/In4 GND
ad_connect  sys_concat_intc/In3 GND
ad_connect  sys_concat_intc/In2 GND
ad_connect  sys_concat_intc/In1 GND
ad_connect  sys_concat_intc/In0 GND

# ad9361

# LVDS interface (6 differential data pairs each direction)
create_bd_port -dir I rx_clk_in_p
create_bd_port -dir I rx_clk_in_n
create_bd_port -dir I rx_frame_in_p
create_bd_port -dir I rx_frame_in_n
create_bd_port -dir I -from 5 -to 0 rx_data_in_p
create_bd_port -dir I -from 5 -to 0 rx_data_in_n

create_bd_port -dir O tx_clk_out_p
create_bd_port -dir O tx_clk_out_n
create_bd_port -dir O tx_frame_out_p
create_bd_port -dir O tx_frame_out_n
create_bd_port -dir O -from 5 -to 0 tx_data_out_p
create_bd_port -dir O -from 5 -to 0 tx_data_out_n

create_bd_port -dir O enable
create_bd_port -dir O txnrx
create_bd_port -dir I up_enable
create_bd_port -dir I up_txnrx

# ad9361 core(s)

ad_ip_instance axi_ad9361 axi_ad9361
ad_ip_parameter axi_ad9361 CONFIG.ID 0
## LVDS (0): 2R2T up to 61.44 MSPS, l_clk = 4 x fs
ad_ip_parameter axi_ad9361 CONFIG.CMOS_OR_LVDS_N 0
ad_ip_parameter axi_ad9361 CONFIG.MODE_1R1T 0
ad_ip_parameter axi_ad9361 CONFIG.ADC_INIT_DELAY 29

ad_ip_instance axi_dmac axi_ad9361_dac_dma
ad_ip_parameter axi_ad9361_dac_dma CONFIG.DMA_TYPE_SRC 0
ad_ip_parameter axi_ad9361_dac_dma CONFIG.DMA_TYPE_DEST 1
ad_ip_parameter axi_ad9361_dac_dma CONFIG.CYCLIC 1
ad_ip_parameter axi_ad9361_dac_dma CONFIG.AXI_SLICE_SRC 0
ad_ip_parameter axi_ad9361_dac_dma CONFIG.AXI_SLICE_DEST 0
ad_ip_parameter axi_ad9361_dac_dma CONFIG.DMA_2D_TRANSFER 0
ad_ip_parameter axi_ad9361_dac_dma CONFIG.DMA_DATA_WIDTH_DEST 64

# no FIR interpolator/decimator: they are generated for a 61.44 MHz clock
# (CMOS) and are only used by Linux for rates < 2.08 MSPS
ad_ip_instance util_upack2 tx_upack

ad_ip_instance axi_dmac axi_ad9361_adc_dma
ad_ip_parameter axi_ad9361_adc_dma CONFIG.DMA_TYPE_SRC 2
ad_ip_parameter axi_ad9361_adc_dma CONFIG.DMA_TYPE_DEST 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.CYCLIC 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.SYNC_TRANSFER_START 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.AXI_SLICE_SRC 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.AXI_SLICE_DEST 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.DMA_2D_TRANSFER 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.DMA_DATA_WIDTH_SRC 64
ad_ip_parameter axi_ad9361_adc_dma CONFIG.SYNC_TRANSFER_START {true}

ad_ip_instance util_cpack2 cpack

# connections

ad_connect  rx_clk_in_p axi_ad9361/rx_clk_in_p
ad_connect  rx_clk_in_n axi_ad9361/rx_clk_in_n
ad_connect  rx_frame_in_p axi_ad9361/rx_frame_in_p
ad_connect  rx_frame_in_n axi_ad9361/rx_frame_in_n
ad_connect  rx_data_in_p axi_ad9361/rx_data_in_p
ad_connect  rx_data_in_n axi_ad9361/rx_data_in_n
ad_connect  tx_clk_out_p axi_ad9361/tx_clk_out_p
ad_connect  tx_clk_out_n axi_ad9361/tx_clk_out_n
ad_connect  tx_frame_out_p axi_ad9361/tx_frame_out_p
ad_connect  tx_frame_out_n axi_ad9361/tx_frame_out_n
ad_connect  tx_data_out_p axi_ad9361/tx_data_out_p
ad_connect  tx_data_out_n axi_ad9361/tx_data_out_n
ad_connect  enable axi_ad9361/enable
ad_connect  txnrx axi_ad9361/txnrx
ad_connect  up_enable axi_ad9361/up_enable
ad_connect  up_txnrx axi_ad9361/up_txnrx

ad_connect  axi_ad9361/tdd_sync GND
ad_connect  sys_200m_clk axi_ad9361/delay_clk
ad_connect  axi_ad9361/l_clk axi_ad9361/clk

# RX: axi_ad9361 -> cpack -> ADC DMA (4 channels)
ad_connect  axi_ad9361/l_clk cpack/clk
ad_connect  axi_ad9361/rst cpack/reset
ad_connect  axi_ad9361/adc_valid_i0 cpack/fifo_wr_en
ad_connect  axi_ad9361/adc_enable_i0 cpack/enable_0
ad_connect  axi_ad9361/adc_data_i0 cpack/fifo_wr_data_0
ad_connect  axi_ad9361/adc_enable_q0 cpack/enable_1
ad_connect  axi_ad9361/adc_data_q0 cpack/fifo_wr_data_1
ad_connect  axi_ad9361/adc_enable_i1 cpack/enable_2
ad_connect  axi_ad9361/adc_data_i1 cpack/fifo_wr_data_2
ad_connect  axi_ad9361/adc_enable_q1 cpack/enable_3
ad_connect  axi_ad9361/adc_data_q1 cpack/fifo_wr_data_3
ad_connect  axi_ad9361_adc_dma/fifo_wr cpack/packed_fifo_wr

# TX: DAC DMA -> upack -> gnss_passthrough -> axi_ad9361
ad_connect  axi_ad9361/l_clk tx_upack/clk
ad_connect  tx_upack/s_axis axi_ad9361_dac_dma/m_axis
ad_connect  axi_ad9361/dac_valid_i0 tx_upack/fifo_rd_en
ad_connect  axi_ad9361/dac_enable_i0 tx_upack/enable_0
ad_connect  axi_ad9361/dac_enable_q0 tx_upack/enable_1
ad_connect  axi_ad9361/dac_enable_i1 tx_upack/enable_2
ad_connect  axi_ad9361/dac_enable_q1 tx_upack/enable_3
ad_connect  tx_upack/fifo_rd_underflow axi_ad9361/dac_dunf

# ---------------------------------------------------------------------------
# SFAP-PI BEGIN: AD9361 RX -> gnss_passthrough (l_clk) -> AD9361 TX
#   gnss_passthrough taps the ADC outputs (fan-out; cpack/DMA keep working),
#   takes the DMA samples from tx_upack, and drives the DAC data pins:
#     CONTROL[0] pass_en = 0 : DAC <- DMA/DDS (as passthrough_mux with GPIO 40 = 0)
#     CONTROL[0] pass_en = 1 : TX1 <- RX1, TX2 <- RX2 ({adc[11:0],4'b0})
#     CONTROL[5] sfap_en = 1 : TX1 <- SFAP-PI output (RX1 + RX2 per bin)
# ---------------------------------------------------------------------------

create_bd_cell -type ip -vlnv $gnss_vlnv gnss_passthrough

ad_connect  axi_ad9361/l_clk gnss_passthrough/clk
ad_connect  axi_ad9361/rst   gnss_passthrough/rst

foreach ch {i0 q0 i1 q1} {
  ad_connect  axi_ad9361/adc_enable_${ch} gnss_passthrough/adc_enable_${ch}
  ad_connect  axi_ad9361/adc_valid_${ch}  gnss_passthrough/adc_valid_${ch}
  ad_connect  axi_ad9361/adc_data_${ch}   gnss_passthrough/adc_data_${ch}
  ad_connect  axi_ad9361/dac_enable_${ch} gnss_passthrough/dac_enable_${ch}
  ad_connect  axi_ad9361/dac_valid_${ch}  gnss_passthrough/dac_valid_${ch}
}

ad_connect  tx_upack/fifo_rd_data_0 gnss_passthrough/dma_dac_data_i0
ad_connect  tx_upack/fifo_rd_data_1 gnss_passthrough/dma_dac_data_q0
ad_connect  tx_upack/fifo_rd_data_2 gnss_passthrough/dma_dac_data_i1
ad_connect  tx_upack/fifo_rd_data_3 gnss_passthrough/dma_dac_data_q1

ad_connect  gnss_passthrough/dac_data_i0 axi_ad9361/dac_data_i0
ad_connect  gnss_passthrough/dac_data_q0 axi_ad9361/dac_data_q0
ad_connect  gnss_passthrough/dac_data_i1 axi_ad9361/dac_data_i1
ad_connect  gnss_passthrough/dac_data_q1 axi_ad9361/dac_data_q1

# SFAP-PI v6.3 filter (Vitis HLS sfap_pi 5.0, II = 1, 256 bins, pulse blanker +
# bin excision mask, latency 782 samples) on sys_cpu_clk (100 MHz). l_clk is 245.76 MHz at 61.44 MS/s, so the
# filter does not run on it; two async AXI-Stream FIFOs carry the samples:
#   gnss_passthrough/m_axis_smi (l_clk) -> smi_fifo_in -> sfap_pi_0 (100 MHz)
#   sfap_pi_0 -> smi_fifo_out -> gnss_passthrough/s_axis_smi (l_clk)
# cfg / nrm come from gnss_passthrough's AJ_* registers; the 3 x 256 weights
# and (v6.3) the 8-word excision mask are read by sfap_pi_0 through memory
# ports (2 per array) from
# gnss_passthrough's double-buffered weight RAM. All on sys_cpu_clk
# (= gnss_passthrough s_axi_aclk), the filter's own clock.
ad_ip_instance axis_data_fifo smi_fifo_in
ad_ip_parameter smi_fifo_in CONFIG.IS_ACLK_ASYNC 1
ad_ip_parameter smi_fifo_in CONFIG.FIFO_DEPTH 64
ad_ip_parameter smi_fifo_in CONFIG.TDATA_NUM_BYTES 8
ad_connect  axi_ad9361/l_clk            smi_fifo_in/s_axis_aclk
ad_connect  sys_cpu_clk                 smi_fifo_in/m_axis_aclk
ad_connect  gnss_passthrough/smi_rst_n  smi_fifo_in/s_axis_aresetn
ad_connect  gnss_passthrough/m_axis_smi smi_fifo_in/S_AXIS

create_bd_cell -type ip -vlnv $sfap_vlnv sfap_pi_0
ad_connect  sys_cpu_clk                 sfap_pi_0/ap_clk
ad_connect  sys_cpu_resetn              sfap_pi_0/ap_rst_n
ad_connect  smi_fifo_in/M_AXIS          sfap_pi_0/s_axis_x
# weight memory ports: sfap_pi_0/<arr>_{address,ce}<p> -> gnss_passthrough,
# gnss_passthrough/aj_<arr>_q<p> -> sfap_pi_0/<arr>_q<p>
foreach arr {wb wc1 wc2 wm} {
  foreach prt {0 1} {
    foreach sig [list address$prt ce$prt q$prt] {
      if {[llength [get_bd_pins -quiet sfap_pi_0/${arr}_${sig}]] == 0} {
        puts "SFAP_LVDS: sfap_pi_0 pins: [get_bd_pins sfap_pi_0/*]"
        puts "SFAP_LVDS: sfap_pi_0 interfaces: [get_bd_intf_pins -quiet sfap_pi_0/*]"
        error "ERROR: sfap_pi_0 has no pin ${arr}_${sig} (expected the ap_memory\
               ports of sfap_pi 5.0; see the pin list above)."
      }
      ad_connect  sfap_pi_0/${arr}_${sig} gnss_passthrough/aj_${arr}_${sig}
    }
  }
}
ad_connect  gnss_passthrough/aj_cfg     sfap_pi_0/cfg
ad_connect  gnss_passthrough/aj_nrm     sfap_pi_0/nrm

ad_ip_instance axis_data_fifo smi_fifo_out
ad_ip_parameter smi_fifo_out CONFIG.IS_ACLK_ASYNC 1
ad_ip_parameter smi_fifo_out CONFIG.FIFO_DEPTH 64
ad_ip_parameter smi_fifo_out CONFIG.TDATA_NUM_BYTES 4
ad_connect  sys_cpu_clk                 smi_fifo_out/s_axis_aclk
ad_connect  axi_ad9361/l_clk            smi_fifo_out/m_axis_aclk
ad_connect  sys_cpu_resetn              smi_fifo_out/s_axis_aresetn
ad_connect  sfap_pi_0/m_axis_y          smi_fifo_out/S_AXIS
ad_connect  smi_fifo_out/M_AXIS         gnss_passthrough/s_axis_smi

# SFAP-PI END
# ---------------------------------------------------------------------------

ad_connect  axi_ad9361/l_clk axi_ad9361_adc_dma/fifo_wr_clk
ad_connect  axi_ad9361/l_clk axi_ad9361_dac_dma/m_axis_aclk
ad_connect  cpack/fifo_wr_overflow axi_ad9361/adc_dovf

# External TDD
set TDD_CHANNEL_CNT 3
set TDD_DEFAULT_POL 0b010
set TDD_REG_WIDTH 32
set TDD_BURST_WIDTH 32
set TDD_SYNC_WIDTH 0
set TDD_SYNC_INT 0
set TDD_SYNC_EXT 1
set TDD_SYNC_EXT_CDC 1
ad_tdd_gen_create axi_tdd_0 $TDD_CHANNEL_CNT \
                            $TDD_DEFAULT_POL \
                            $TDD_REG_WIDTH \
                            $TDD_BURST_WIDTH \
                            $TDD_SYNC_WIDTH \
                            $TDD_SYNC_INT \
                            $TDD_SYNC_EXT \
                            $TDD_SYNC_EXT_CDC

ad_ip_instance util_vector_logic logic_inv [list \
  C_OPERATION {not} \
  C_SIZE 1]

ad_ip_instance util_vector_logic logic_or_1 [list \
  C_OPERATION {or} \
  C_SIZE 1]

ad_connect logic_inv/Op1  axi_ad9361/rst
ad_connect logic_inv/Res  axi_tdd_0/resetn
ad_connect axi_ad9361/l_clk axi_tdd_0/clk
ad_connect axi_tdd_0/sync_in tdd_ext_sync
ad_connect axi_tdd_0/tdd_channel_0 txdata_o
ad_connect axi_tdd_0/tdd_channel_1 axi_ad9361_adc_dma/sync


ad_connect  logic_or_1/Op1  axi_ad9361/rst
ad_connect  logic_or_1/Op2  axi_tdd_0/tdd_channel_2
ad_connect  logic_or_1/Res  tx_upack/reset

# interconnects

ad_cpu_interconnect 0x79020000 axi_ad9361
ad_cpu_interconnect 0x7C400000 axi_ad9361_adc_dma
ad_cpu_interconnect 0x7C420000 axi_ad9361_dac_dma
ad_cpu_interconnect 0x7C440000 axi_tdd_0
ad_cpu_interconnect 0x43C00000 axi_vcxo_ctrl
# SFAP-PI: gnss_passthrough control/status + 3 x 256 weights (8 kB). Vivado
# may place it elsewhere; the firmware takes the address from xparameters.
ad_cpu_interconnect 0x43C10000 gnss_passthrough

ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP1 {1}
ad_connect sys_cpu_clk sys_ps7/S_AXI_HP1_ACLK
ad_connect axi_ad9361_adc_dma/m_dest_axi sys_ps7/S_AXI_HP1

create_bd_addr_seg -range 0x40000000 -offset 0x00000000 \
                    [get_bd_addr_spaces axi_ad9361_adc_dma/m_dest_axi] \
                    [get_bd_addr_segs sys_ps7/S_AXI_HP1/HP1_DDR_LOWOCM] \
                    SEG_sys_ps7_HP1_DDR_LOWOCM

ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP2 {1}
ad_connect sys_cpu_clk sys_ps7/S_AXI_HP2_ACLK
ad_connect axi_ad9361_dac_dma/m_src_axi sys_ps7/S_AXI_HP2

create_bd_addr_seg -range 0x40000000 -offset 0x00000000 \
                    [get_bd_addr_spaces axi_ad9361_dac_dma/m_src_axi] \
                    [get_bd_addr_segs sys_ps7/S_AXI_HP2/HP2_DDR_LOWOCM] \
                    SEG_sys_ps7_HP2_DDR_LOWOCM

ad_connect sys_cpu_clk axi_ad9361_dac_dma/m_src_axi_aclk
ad_connect sys_cpu_clk axi_ad9361_adc_dma/m_dest_axi_aclk
ad_connect sys_cpu_resetn axi_ad9361_adc_dma/m_dest_axi_aresetn
ad_connect sys_cpu_resetn axi_ad9361_dac_dma/m_src_axi_aresetn

# interrupts

ad_cpu_interrupt ps-13 mb-13 axi_ad9361_adc_dma/irq
ad_cpu_interrupt ps-12 mb-12 axi_ad9361_dac_dma/irq

