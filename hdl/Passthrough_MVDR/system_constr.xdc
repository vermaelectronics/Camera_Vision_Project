## ===========================================================================
## ANTSDR E310 - Passthrough_LVDS constraints
## Based on antsdr-fw-patch "ant" (CMOS) + MicroPhase antsdre310 (LVDS) pinout.
## Same package pins as the ant CMOS design, used as LVDS P/N pairs.
##
## Bank 34 (all AD9361 pins): LVDS_25 + LVCMOS25 (as in the antsdre310 LVDS design)
## Bank 35 (VCXO DAC, PPS, 40 MHz, RF band switches): LVCMOS33 (unchanged)
## Bank 13 (10 MHz in/req, CTRL_IN[3:1]): LVCMOS18 (unchanged from ant)
## ===========================================================================

## ---- clocking / VCXO (bank 35, unchanged) ---------------------------------
set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS33} [get_ports CLK_40M_DAC_nSYNC]
set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS33} [get_ports CLK_40M_DAC_SCLK]
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports CLK_40M_DAC_DIN]
set_property -dict {PACKAGE_PIN J18 IOSTANDARD LVCMOS33} [get_ports PPS_IN]
set_property -dict {PACKAGE_PIN K17 IOSTANDARD LVCMOS33} [get_ports CLK_40MHz_FPGA]

## ---- bank 13 (unchanged) ----------------------------------------------------
set_property -dict {PACKAGE_PIN T9  IOSTANDARD LVCMOS18} [get_ports CLKIN_10MHz]
set_property -dict {PACKAGE_PIN V6  IOSTANDARD LVCMOS18} [get_ports CLKIN_10MHz_REQ]
set_property -dict {PACKAGE_PIN Y12 IOSTANDARD LVCMOS18} [get_ports {gpio_ctl[1]}]
set_property -dict {PACKAGE_PIN Y13 IOSTANDARD LVCMOS18} [get_ports {gpio_ctl[2]}]
set_property -dict {PACKAGE_PIN V11 IOSTANDARD LVCMOS18} [get_ports {gpio_ctl[3]}]

## ---- AD9361 control (bank 34, 2.5 V) ---------------------------------------
set_property -dict {PACKAGE_PIN T10 IOSTANDARD LVCMOS25} [get_ports {gpio_ctl[0]}]
set_property -dict {PACKAGE_PIN T11 IOSTANDARD LVCMOS25} [get_ports {gpio_status[0]}]
set_property -dict {PACKAGE_PIN T14 IOSTANDARD LVCMOS25} [get_ports {gpio_status[1]}]
set_property -dict {PACKAGE_PIN T15 IOSTANDARD LVCMOS25} [get_ports {gpio_status[2]}]
set_property -dict {PACKAGE_PIN T17 IOSTANDARD LVCMOS25} [get_ports {gpio_status[3]}]
set_property -dict {PACKAGE_PIN T19 IOSTANDARD LVCMOS25} [get_ports {gpio_status[4]}]
set_property -dict {PACKAGE_PIN T20 IOSTANDARD LVCMOS25} [get_ports {gpio_status[5]}]
set_property -dict {PACKAGE_PIN U13 IOSTANDARD LVCMOS25} [get_ports {gpio_status[6]}]
set_property -dict {PACKAGE_PIN V13 IOSTANDARD LVCMOS25} [get_ports {gpio_status[7]}]
set_property -dict {PACKAGE_PIN P14 IOSTANDARD LVCMOS25} [get_ports txnrx]
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS25} [get_ports enable]
set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS25} [get_ports gpio_en_agc]
set_property -dict {PACKAGE_PIN N17 IOSTANDARD LVCMOS25} [get_ports gpio_resetb]

set_property -dict {PACKAGE_PIN P18 IOSTANDARD LVCMOS25 PULLUP true} [get_ports spi_csn]
set_property -dict {PACKAGE_PIN R14 IOSTANDARD LVCMOS25} [get_ports spi_clk]
set_property -dict {PACKAGE_PIN P15 IOSTANDARD LVCMOS25} [get_ports spi_mosi]
set_property -dict {PACKAGE_PIN R19 IOSTANDARD LVCMOS25} [get_ports spi_miso]

## ---- AD9361 LVDS data interface (bank 34) ----------------------------------
## RX (AD9361 -> FPGA), internal 100 ohm differential termination
set_property -dict {PACKAGE_PIN N20 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_clk_in_p]
set_property -dict {PACKAGE_PIN P20 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_clk_in_n]
set_property -dict {PACKAGE_PIN U18 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_frame_in_p]
set_property -dict {PACKAGE_PIN U19 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_frame_in_n]
set_property -dict {PACKAGE_PIN Y18 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_p[0]}]
set_property -dict {PACKAGE_PIN Y19 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_n[0]}]
set_property -dict {PACKAGE_PIN V17 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_p[1]}]
set_property -dict {PACKAGE_PIN V18 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_n[1]}]
set_property -dict {PACKAGE_PIN V20 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_p[2]}]
set_property -dict {PACKAGE_PIN W20 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_n[2]}]
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_p[3]}]
set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_n[3]}]
set_property -dict {PACKAGE_PIN W18 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_p[4]}]
set_property -dict {PACKAGE_PIN W19 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_n[4]}]
set_property -dict {PACKAGE_PIN V16 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_p[5]}]
set_property -dict {PACKAGE_PIN W16 IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports {rx_data_in_n[5]}]

## TX (FPGA -> AD9361), terminated on the AD9361 side (lvds_rx_onchip_termination)
set_property -dict {PACKAGE_PIN N18 IOSTANDARD LVDS_25} [get_ports tx_clk_out_p]
set_property -dict {PACKAGE_PIN P19 IOSTANDARD LVDS_25} [get_ports tx_clk_out_n]
set_property -dict {PACKAGE_PIN Y16 IOSTANDARD LVDS_25} [get_ports tx_frame_out_p]
set_property -dict {PACKAGE_PIN Y17 IOSTANDARD LVDS_25} [get_ports tx_frame_out_n]
set_property -dict {PACKAGE_PIN W14 IOSTANDARD LVDS_25} [get_ports {tx_data_out_p[0]}]
set_property -dict {PACKAGE_PIN Y14 IOSTANDARD LVDS_25} [get_ports {tx_data_out_n[0]}]
set_property -dict {PACKAGE_PIN T12 IOSTANDARD LVDS_25} [get_ports {tx_data_out_p[1]}]
set_property -dict {PACKAGE_PIN U12 IOSTANDARD LVDS_25} [get_ports {tx_data_out_n[1]}]
set_property -dict {PACKAGE_PIN U14 IOSTANDARD LVDS_25} [get_ports {tx_data_out_p[2]}]
set_property -dict {PACKAGE_PIN U15 IOSTANDARD LVDS_25} [get_ports {tx_data_out_n[2]}]
set_property -dict {PACKAGE_PIN T16 IOSTANDARD LVDS_25} [get_ports {tx_data_out_p[3]}]
set_property -dict {PACKAGE_PIN U17 IOSTANDARD LVDS_25} [get_ports {tx_data_out_n[3]}]
set_property -dict {PACKAGE_PIN V12 IOSTANDARD LVDS_25} [get_ports {tx_data_out_p[4]}]
set_property -dict {PACKAGE_PIN W13 IOSTANDARD LVDS_25} [get_ports {tx_data_out_n[4]}]
set_property -dict {PACKAGE_PIN V15 IOSTANDARD LVDS_25} [get_ports {tx_data_out_p[5]}]
set_property -dict {PACKAGE_PIN W15 IOSTANDARD LVDS_25} [get_ports {tx_data_out_n[5]}]

## ---- RF band switches (bank 35, unchanged) ---------------------------------
set_property -dict {PACKAGE_PIN G14 IOSTANDARD LVCMOS33} [get_ports VCRX1_H]
set_property -dict {PACKAGE_PIN C20 IOSTANDARD LVCMOS33} [get_ports VCRX1_L]
set_property -dict {PACKAGE_PIN B19 IOSTANDARD LVCMOS33} [get_ports VCTX1_H]
set_property -dict {PACKAGE_PIN B20 IOSTANDARD LVCMOS33} [get_ports VCTX1_L]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33} [get_ports VCRX2_H]
set_property -dict {PACKAGE_PIN A20 IOSTANDARD LVCMOS33} [get_ports VCRX2_L]
set_property -dict {PACKAGE_PIN D18 IOSTANDARD LVCMOS33} [get_ports VCTX2_H]
set_property -dict {PACKAGE_PIN D19 IOSTANDARD LVCMOS33} [get_ports VCTX2_L]

## ---- clocks -----------------------------------------------------------------
## LVDS 2R2T: DATA_CLK = 4 x fs -> 245.76 MHz at 61.44 MSPS (4.069 ns)
create_clock -name rx_clk     -period  4.00 [get_ports rx_clk_in_p]
create_clock -name clk_fpga_0 -period 10    [get_pins "i_system_wrapper/system_i/sys_ps7/inst/PS7_i/FCLKCLK[0]"]
create_clock -name clk_fpga_1 -period  5    [get_pins "i_system_wrapper/system_i/sys_ps7/inst/PS7_i/FCLKCLK[1]"]
create_clock -name spi0_clk   -period 40    [get_pins -hier */EMIOSPI0SCLKO]
create_clock -name clk_40m    -period 25    [get_ports CLK_40MHz_FPGA]

set_input_jitter clk_fpga_0 0.3
set_input_jitter clk_fpga_1 0.15

## axi_vcxo_ctrl 40 MHz reference PLL is unrelated to the data path
set_clock_groups -asynchronous -group [get_clocks -include_generated_clocks clk_40m]

## static GPIO registers of the core
set_false_path -from [get_pins {i_system_wrapper/system_i/axi_ad9361/inst/i_rx/i_up_adc_common/up_adc_gpio_out_int_reg[0]/C}]
set_false_path -from [get_pins {i_system_wrapper/system_i/axi_ad9361/inst/i_tx/i_up_dac_common/up_dac_gpio_out_int_reg[0]/C}]

## passthrough enable: PS EMIO GPIO_O is not timed by Vivado; passthrough_mux
## resynchronises it with a 2-FF (ASYNC_REG) synchroniser on l_clk.
