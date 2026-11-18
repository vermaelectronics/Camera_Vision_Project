# create board design

set_property  ip_repo_paths  $ad_hdl_dir/library/ [current_project]
update_ip_catalog

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
# internal TDD engine off when this axi_ad9361 version still has it
# (FDD; its reset tree failed recovery timing in the earlier MVDR builds)
if {[get_property -quiet CONFIG.TDD_DISABLE [get_bd_cells axi_ad9361]] ne ""} {
  set_property CONFIG.TDD_DISABLE 1 [get_bd_cells axi_ad9361]
}

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
# no TDD engine in this design: capture starts immediately
ad_ip_parameter axi_ad9361_adc_dma CONFIG.SYNC_TRANSFER_START {false}

ad_ip_instance util_cpack2 cpack

# GNSS anti-jam core: gnss_passthrough.v from the working MVDR2 design, unchanged
# (MVDR 2x2 + SMI-STAP 2 x 16 + 2 image taps, jammer estimator, RX->TX elastic
# buffer on dac_valid, 12-bit left alignment). gnss_passthrough_bd.v only adds
# the clock/reset interface attributes for the module-reference cell.
set gnss_rtl [list \
  rtl/gnss_passthrough_bd.v rtl/gnss_passthrough.v rtl/mvdr_top.v rtl/mvdr_regs.v rtl/mvdr_xdom.v \
  rtl/cov2x2_accum.v rtl/cov_prep.v rtl/cov_iir.v rtl/mvdr_weight_calc.v \
  rtl/mvdr_ctrl_fsm.v rtl/real_divider.v rtl/beamformer_2ch.v rtl/cmult.v \
  rtl/output_scaler.v rtl/iq_corr.v rtl/iq_est.v rtl/jammer_est.v \
  rtl/stap_smi.v rtl/stap_lms.v]
foreach f $gnss_rtl { add_files -norecurse [file normalize $f] }
update_compile_order -fileset sources_1
create_bd_cell -type module -reference gnss_passthrough_bd gnss_passthrough

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

# RX taps and TX strobes into gnss_passthrough (same nets as the MVDR2 design)
ad_connect  axi_ad9361/l_clk gnss_passthrough/clk
ad_connect  axi_ad9361/rst gnss_passthrough/rst
foreach c {i0 q0 i1 q1} {
  ad_connect  axi_ad9361/adc_enable_$c gnss_passthrough/adc_enable_$c
  ad_connect  axi_ad9361/adc_valid_$c  gnss_passthrough/adc_valid_$c
  ad_connect  axi_ad9361/adc_data_$c   gnss_passthrough/adc_data_$c
  ad_connect  axi_ad9361/dac_enable_$c gnss_passthrough/dac_enable_$c
  ad_connect  axi_ad9361/dac_valid_$c  gnss_passthrough/dac_valid_$c
  ad_connect  gnss_passthrough/dac_data_$c axi_ad9361/dac_data_$c
}
# vendor DMA data (pass_en = 0 path)
ad_connect  tx_upack/fifo_rd_data_0 gnss_passthrough/dma_dac_data_i0
ad_connect  tx_upack/fifo_rd_data_1 gnss_passthrough/dma_dac_data_q0
ad_connect  tx_upack/fifo_rd_data_2 gnss_passthrough/dma_dac_data_i1
ad_connect  tx_upack/fifo_rd_data_3 gnss_passthrough/dma_dac_data_q1

ad_connect  axi_ad9361/l_clk axi_ad9361_adc_dma/fifo_wr_clk
ad_connect  axi_ad9361/l_clk axi_ad9361_dac_dma/m_axis_aclk
ad_connect  cpack/fifo_wr_overflow axi_ad9361/adc_dovf

# No TDD engine: FDD GNSS does not use it, and its 32-bit counter compares
# on l_clk (245.76 MHz in LVDS 2R2T) were the failing timing paths.
# The TX unpacker is reset with the AD9361 interface reset alone.
ad_connect  axi_ad9361/rst tx_upack/reset

# interconnects

ad_cpu_interconnect 0x79020000 axi_ad9361
ad_cpu_interconnect 0x7C400000 axi_ad9361_adc_dma
ad_cpu_interconnect 0x7C420000 axi_ad9361_dac_dma
ad_cpu_interconnect 0x43C00000 axi_vcxo_ctrl
# 0x43C00000 is the ant VCXO controller, so gnss_passthrough moves to 0x43C10000
# (the MVDR2 design had it at 0x43C00000; the firmware base is set to match)
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

