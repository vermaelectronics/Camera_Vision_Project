
# constraints

# ad9361 master
set_property  -dict {PACKAGE_PIN  AE27  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_clk_in_1_p]         ; 
set_property  -dict {PACKAGE_PIN  AF27  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_clk_in_1_n]         ; 
set_property  -dict {PACKAGE_PIN  AC26  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_frame_in_1_p]       ; 
set_property  -dict {PACKAGE_PIN  AD26  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_frame_in_1_n]       ; 
set_property  -dict {PACKAGE_PIN  AD30  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_p[5]]     ; 
set_property  -dict {PACKAGE_PIN  AE30  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_n[5]]     ; 
set_property  -dict {PACKAGE_PIN  AF30  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_p[4]]     ; 
set_property  -dict {PACKAGE_PIN  AG30  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_n[4]]     ; 
set_property  -dict {PACKAGE_PIN  AH28  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_p[3]]     ; 
set_property  -dict {PACKAGE_PIN  AH29  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_n[3]]     ; 
set_property  -dict {PACKAGE_PIN  AK27  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_p[2]]     ; 
set_property  -dict {PACKAGE_PIN  AK28  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_n[2]]     ; 
set_property  -dict {PACKAGE_PIN  AJ28  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_p[1]]     ; 
set_property  -dict {PACKAGE_PIN  AJ29  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_n[1]]     ; 
set_property  -dict {PACKAGE_PIN  AG26  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_p[0]]     ; 
set_property  -dict {PACKAGE_PIN  AG27  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_1_n[0]]     ; 
set_property  -dict {PACKAGE_PIN  AE28  IOSTANDARD LVDS_25}   [get_ports tx_clk_out_1_p]                        ; 
set_property  -dict {PACKAGE_PIN  AF28  IOSTANDARD LVDS_25}   [get_ports tx_clk_out_1_n]                        ; 
set_property  -dict {PACKAGE_PIN  AE25  IOSTANDARD LVDS_25}   [get_ports tx_frame_out_1_p]                      ; 
set_property  -dict {PACKAGE_PIN  AF25  IOSTANDARD LVDS_25}   [get_ports tx_frame_out_1_n]                      ; 
set_property  -dict {PACKAGE_PIN  AD25  IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_p[5]]                    ; 
set_property  -dict {PACKAGE_PIN  AE26  IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_n[5]]                    ; 
set_property  -dict {PACKAGE_PIN  AB29  IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_p[4]]                    ; 
set_property  -dict {PACKAGE_PIN  AB30  IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_n[4]]                    ; 
set_property  -dict {PACKAGE_PIN  Y28   IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_p[3]]                    ; 
set_property  -dict {PACKAGE_PIN  AA29  IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_n[3]]                    ; 
set_property  -dict {PACKAGE_PIN  AB27  IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_p[2]]                    ; 
set_property  -dict {PACKAGE_PIN  AC27  IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_n[2]]                    ; 
set_property  -dict {PACKAGE_PIN  AC28  IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_p[1]]                    ; 
set_property  -dict {PACKAGE_PIN  AD28  IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_n[1]]                    ; 
set_property  -dict {PACKAGE_PIN  AC29  IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_p[0]]                    ; 
set_property  -dict {PACKAGE_PIN  AD29  IOSTANDARD LVDS_25}   [get_ports tx_data_out_1_n[0]]                    ; 

set_property  -dict {PACKAGE_PIN  AA28   IOSTANDARD LVCMOS25}  [get_ports gpio_status_1[7]]                      ; 
set_property  -dict {PACKAGE_PIN  AB19   IOSTANDARD LVCMOS25}  [get_ports gpio_status_1[6]]                      ; 
set_property  -dict {PACKAGE_PIN  AB25   IOSTANDARD LVCMOS25}  [get_ports gpio_status_1[5]]                      ; 
set_property  -dict {PACKAGE_PIN  AJ26   IOSTANDARD LVCMOS25}  [get_ports gpio_status_1[4]]                      ; 
set_property  -dict {PACKAGE_PIN  AB26   IOSTANDARD LVCMOS25}  [get_ports gpio_status_1[3]]                      ; 
set_property  -dict {PACKAGE_PIN  AB20   IOSTANDARD LVCMOS25}  [get_ports gpio_status_1[2]]                      ; 
set_property  -dict {PACKAGE_PIN  AD20   IOSTANDARD LVCMOS25}  [get_ports gpio_status_1[1]]                      ; 
set_property  -dict {PACKAGE_PIN  AA30   IOSTANDARD LVCMOS25}  [get_ports gpio_status_1[0]]                      ; 
set_property  -dict {PACKAGE_PIN  Y30    IOSTANDARD LVCMOS25}  [get_ports gpio_ctl_1[3]]                         ; 
set_property  -dict {PACKAGE_PIN  Y27    IOSTANDARD LVCMOS25}  [get_ports gpio_ctl_1[2]]                         ; 
set_property  -dict {PACKAGE_PIN  AA27   IOSTANDARD LVCMOS25}  [get_ports gpio_ctl_1[1]]                         ; 
set_property  -dict {PACKAGE_PIN  Y26    IOSTANDARD LVCMOS25}  [get_ports gpio_ctl_1[0]]                         ; 
set_property  -dict {PACKAGE_PIN  AK26   IOSTANDARD LVCMOS25}  [get_ports gpio_en_agc_1]                         ; 
set_property  -dict {PACKAGE_PIN  AG29   IOSTANDARD LVCMOS25}  [get_ports mcs_sync_1]                            ; 
set_property  -dict {PACKAGE_PIN  AJ30   IOSTANDARD LVCMOS25}  [get_ports gpio_resetb_1]                         ; 
set_property  -dict {PACKAGE_PIN  AK30   IOSTANDARD LVCMOS25}  [get_ports enable_1]                              ; 
set_property  -dict {PACKAGE_PIN  AH26   IOSTANDARD LVCMOS25}  [get_ports txnrx_1]                               ; 
set_property  -dict {PACKAGE_PIN  AJ18   IOSTANDARD LVCMOS33}  [get_ports gpio_calsw_1_0]                        ; 
set_property  -dict {PACKAGE_PIN  AH18   IOSTANDARD LVCMOS33}  [get_ports gpio_calsw_2_0]                        ; 

# spi


set_property  -dict {PACKAGE_PIN  AF29   IOSTANDARD LVCMOS25   PULLTYPE PULLUP} [get_ports spi_ad9361_1]         ; 
set_property  -dict {PACKAGE_PIN  Y25    IOSTANDARD LVCMOS25}  [get_ports spi_clk_1]                               ; 
set_property  -dict {PACKAGE_PIN  AH27   IOSTANDARD LVCMOS25}  [get_ports spi_mosi_1]                              ; 
set_property  -dict {PACKAGE_PIN  AA25   IOSTANDARD LVCMOS25}  [get_ports spi_miso_1]                              ; 

set_property  -dict {PACKAGE_PIN  AJ20   IOSTANDARD LVCMOS25   PULLTYPE PULLUP} [get_ports spi_ad9361_0]         ; 
set_property  -dict {PACKAGE_PIN  W23   IOSTANDARD LVCMOS25}  [get_ports spi_clk_0]                               ; 
set_property  -dict {PACKAGE_PIN  AA23   IOSTANDARD LVCMOS25}  [get_ports spi_mosi_0]                              ; 
set_property  -dict {PACKAGE_PIN  AC21   IOSTANDARD LVCMOS25}  [get_ports spi_miso_0]                              ; 


# ad9361 slave

set_property  -dict {PACKAGE_PIN  AF20  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_clk_in_0_p]         ; 
set_property  -dict {PACKAGE_PIN  AG20  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_clk_in_0_n]         ; 
set_property  -dict {PACKAGE_PIN  AC24  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_frame_in_0_p]       ; 
set_property  -dict {PACKAGE_PIN  AD24  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_frame_in_0_n]       ; 
set_property  -dict {PACKAGE_PIN  AG24  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_p[5]]     ; 
set_property  -dict {PACKAGE_PIN  AG25  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_n[5]]     ; 
set_property  -dict {PACKAGE_PIN  AK17  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_p[4]]     ; 
set_property  -dict {PACKAGE_PIN  AK18  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_n[4]]     ; 
set_property  -dict {PACKAGE_PIN  AB21  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_p[3]]     ; 
set_property  -dict {PACKAGE_PIN  AB22  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_n[3]]     ; 
set_property  -dict {PACKAGE_PIN  AA24  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_p[2]]     ; 
set_property  -dict {PACKAGE_PIN  AB24  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_n[2]]     ; 
set_property  -dict {PACKAGE_PIN  Y22   IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_p[1]]     ; 
set_property  -dict {PACKAGE_PIN  Y23   IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_n[1]]     ; 
set_property  -dict {PACKAGE_PIN  AH19  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_p[0]]     ; 
set_property  -dict {PACKAGE_PIN  AJ19  IOSTANDARD LVDS_25    DIFF_TERM TRUE} [get_ports rx_data_in_0_n[0]]     ; 
set_property  -dict {PACKAGE_PIN  AG21  IOSTANDARD LVDS_25}   [get_ports tx_clk_out_0_p]                        ; 
set_property  -dict {PACKAGE_PIN  AH21  IOSTANDARD LVDS_25}   [get_ports tx_clk_out_0_n]                        ; 
set_property  -dict {PACKAGE_PIN  AF19  IOSTANDARD LVDS_25}   [get_ports tx_frame_out_0_p]                      ; 
set_property  -dict {PACKAGE_PIN  AG19  IOSTANDARD LVDS_25}   [get_ports tx_frame_out_0_n]                      ; 
set_property  -dict {PACKAGE_PIN  AD21  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_p[5]]                    ; 
set_property  -dict {PACKAGE_PIN  AE21  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_n[5]]                    ; 
set_property  -dict {PACKAGE_PIN  AK22  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_p[4]]                    ; 
set_property  -dict {PACKAGE_PIN  AK23  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_n[4]]                    ; 
set_property  -dict {PACKAGE_PIN  AJ23  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_p[3]]                    ; 
set_property  -dict {PACKAGE_PIN  AJ24  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_n[3]]                    ; 
set_property  -dict {PACKAGE_PIN  AD23  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_p[2]]                    ; 
set_property  -dict {PACKAGE_PIN  AE23  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_n[2]]                    ; 
set_property  -dict {PACKAGE_PIN  AE22  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_p[1]]                    ; 
set_property  -dict {PACKAGE_PIN  AF22  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_n[1]]                    ; 
set_property  -dict {PACKAGE_PIN  AF23  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_p[0]]                    ; 
set_property  -dict {PACKAGE_PIN  AF24  IOSTANDARD LVDS_25}   [get_ports tx_data_out_0_n[0]]                    ; 



set_property  -dict {PACKAGE_PIN  AH24  IOSTANDARD LVCMOS25}  [get_ports gpio_status_0[7]]                      ; 
set_property  -dict {PACKAGE_PIN  AA20  IOSTANDARD LVCMOS25}  [get_ports gpio_status_0[6]]                      ; 
set_property  -dict {PACKAGE_PIN  AG22  IOSTANDARD LVCMOS25}  [get_ports gpio_status_0[5]]                      ; 
set_property  -dict {PACKAGE_PIN  AC22  IOSTANDARD LVCMOS25}  [get_ports gpio_status_0[4]]                      ; 
set_property  -dict {PACKAGE_PIN  AH22  IOSTANDARD LVCMOS25}  [get_ports gpio_status_0[3]]                      ; 
set_property  -dict {PACKAGE_PIN  AC18  IOSTANDARD LVCMOS25}  [get_ports gpio_status_0[2]]                      ; 
set_property  -dict {PACKAGE_PIN  AC19  IOSTANDARD LVCMOS25}  [get_ports gpio_status_0[1]]                      ; 
set_property  -dict {PACKAGE_PIN  AK25  IOSTANDARD LVCMOS25}  [get_ports gpio_status_0[0]]                      ; 
set_property  -dict {PACKAGE_PIN  AJ25  IOSTANDARD LVCMOS25}  [get_ports gpio_ctl_0[3]]                         ; 
set_property  -dict {PACKAGE_PIN  AK21  IOSTANDARD LVCMOS25}  [get_ports gpio_ctl_0[2]]                         ; 
set_property  -dict {PACKAGE_PIN  AH23  IOSTANDARD LVCMOS25}  [get_ports gpio_ctl_0[1]]                         ; 
set_property  -dict {PACKAGE_PIN  AJ21  IOSTANDARD LVCMOS25}  [get_ports gpio_ctl_0[0]]                         ; 
set_property  -dict {PACKAGE_PIN  AC23  IOSTANDARD LVCMOS25}  [get_ports gpio_en_agc_0]                         ; 
set_property  -dict {PACKAGE_PIN  W21   IOSTANDARD LVCMOS25}  [get_ports gpio_resetb_0]                         ; 
set_property  -dict {PACKAGE_PIN  Y21   IOSTANDARD LVCMOS25}  [get_ports enable_0]                              ; 
set_property  -dict {PACKAGE_PIN  AA22  IOSTANDARD LVCMOS25}  [get_ports txnrx_0]                               ; 
set_property  -dict {PACKAGE_PIN  AK20  IOSTANDARD LVCMOS25}  [get_ports mcs_sync_0]                            ; 

set_property  -dict {PACKAGE_PIN  U26  IOSTANDARD LVCMOS33}  [get_ports ref_clk]                       ; 


set_property  -dict {PACKAGE_PIN  AK13  IOSTANDARD LVCMOS33}  [get_ports CH1_FE_TXRX1_SEL1]                    ; 
set_property  -dict {PACKAGE_PIN  AJ16  IOSTANDARD LVCMOS33}  [get_ports CH1_FE_TXRX2_SEL1]                    ; 
set_property  -dict {PACKAGE_PIN  AK12  IOSTANDARD LVCMOS33}  [get_ports CH1_FE_RX1_SEL1]                      ; 
set_property  -dict {PACKAGE_PIN  AJ15  IOSTANDARD LVCMOS33}  [get_ports CH1_FE_RX2_SEL1]                      ; 
set_property  -dict {PACKAGE_PIN  AJ14  IOSTANDARD LVCMOS33}  [get_ports CH1_TX1_AMP_EN]                       ; 
set_property  -dict {PACKAGE_PIN  AJ13  IOSTANDARD LVCMOS33}  [get_ports CH1_TX2_AMP_EN]                       ; 
set_property  -dict {PACKAGE_PIN  AH17  IOSTANDARD LVCMOS33}  [get_ports CH2_FE_TXRX1_SEL1]                    ; 
set_property  -dict {PACKAGE_PIN  AH12  IOSTANDARD LVCMOS33}  [get_ports CH2_FE_TXRX2_SEL1]                    ; 
set_property  -dict {PACKAGE_PIN  AH16  IOSTANDARD LVCMOS33}  [get_ports CH2_FE_RX1_SEL1]                      ; 
set_property  -dict {PACKAGE_PIN  AG12  IOSTANDARD LVCMOS33}  [get_ports CH2_FE_RX2_SEL1]                      ; 
set_property  -dict {PACKAGE_PIN  AH14  IOSTANDARD LVCMOS33}  [get_ports CH2_TX1_AMP_EN]                       ; 
set_property  -dict {PACKAGE_PIN  AH13  IOSTANDARD LVCMOS33}  [get_ports CH2_TX2_AMP_EN]                       ; 

# clocks

create_clock -name rx_0_clk       -period   4.00 [get_ports rx_clk_in_0_p]
create_clock -name rx_1_clk       -period   4.00 [get_ports rx_clk_in_1_p]
create_clock -name ref_clk        -period   4.00 [get_ports ref_clk]

