set_property  -dict {PACKAGE_PIN  T11  IOSTANDARD LVCMOS25} [get_ports gpio_status[0]]                   ; ## G21  FMC_LPC_LA20_P
set_property  -dict {PACKAGE_PIN  T14  IOSTANDARD LVCMOS25} [get_ports gpio_status[1]]                   ; ## G22  FMC_LPC_LA20_N
set_property  -dict {PACKAGE_PIN  T15  IOSTANDARD LVCMOS25} [get_ports gpio_status[2]]                   ; ## H25  FMC_LPC_LA21_P
set_property  -dict {PACKAGE_PIN  T17  IOSTANDARD LVCMOS25} [get_ports gpio_status[3]]                   ; ## H26  FMC_LPC_LA21_N
set_property  -dict {PACKAGE_PIN  T19  IOSTANDARD LVCMOS25} [get_ports gpio_status[4]]                   ; ## G24  FMC_LPC_LA22_P
set_property  -dict {PACKAGE_PIN  T20  IOSTANDARD LVCMOS25} [get_ports gpio_status[5]]                   ; ## G25  FMC_LPC_LA22_N
set_property  -dict {PACKAGE_PIN  U13  IOSTANDARD LVCMOS25} [get_ports gpio_status[6]]                   ; ## D23  FMC_LPC_LA23_P
set_property  -dict {PACKAGE_PIN  V13  IOSTANDARD LVCMOS25} [get_ports gpio_status[7]]                   ; ## D24  FMC_LPC_LA23_N
set_property  -dict {PACKAGE_PIN  T10  IOSTANDARD LVCMOS25} [get_ports gpio_ctl[0]]                      ; ## H28  FMC_LPC_LA24_P
set_property  -dict {PACKAGE_PIN  Y12  IOSTANDARD LVCMOS33} [get_ports gpio_ctl[1]]                      ; ## H29  FMC_LPC_LA24_N
set_property  -dict {PACKAGE_PIN  Y13  IOSTANDARD LVCMOS33} [get_ports gpio_ctl[2]]                      ; ## G27  FMC_LPC_LA25_P
set_property  -dict {PACKAGE_PIN  V11  IOSTANDARD LVCMOS33} [get_ports gpio_ctl[3]]                      ; ## G28  FMC_LPC_LA25_N
set_property  -dict {PACKAGE_PIN  P16  IOSTANDARD LVCMOS25} [get_ports gpio_en_agc]                      ; ## H22  FMC_LPC_LA19_P
set_property  -dict {PACKAGE_PIN  U20  IOSTANDARD LVCMOS25} [get_ports gpio_sync]                        ; ## H23  FMC_LPC_LA19_N
set_property  -dict {PACKAGE_PIN  N17  IOSTANDARD LVCMOS25} [get_ports gpio_resetb]                      ; ## H31  FMC_LPC_LA28_P
set_property  -dict {PACKAGE_PIN  R18  IOSTANDARD LVCMOS25} [get_ports enable]                           ; ## G18  FMC_LPC_LA16_P
set_property  -dict {PACKAGE_PIN  P14  IOSTANDARD LVCMOS25} [get_ports txnrx]                            ; ## G19  FMC_LPC_LA16_N

set_property  -dict {PACKAGE_PIN  P18  IOSTANDARD LVCMOS25  PULLTYPE PULLUP} [get_ports spi_csn]         ; ## D26  FMC_LPC_LA26_P
set_property  -dict {PACKAGE_PIN  R14  IOSTANDARD LVCMOS25} [get_ports spi_clk]                          ; ## D27  FMC_LPC_LA26_N
set_property  -dict {PACKAGE_PIN  P15  IOSTANDARD LVCMOS25} [get_ports spi_mosi]                         ; ## C26  FMC_LPC_LA27_P
set_property  -dict {PACKAGE_PIN  R19  IOSTANDARD LVCMOS25} [get_ports spi_miso]                         ; ## C27  FMC_LPC_LA27_N
     
set_property  -dict {PACKAGE_PIN  H18   IOSTANDARD LVCMOS33 PULLTYPE PULLUP} [get_ports iic_scl]     ;     
set_property  -dict {PACKAGE_PIN  G17   IOSTANDARD LVCMOS33 PULLTYPE PULLUP} [get_ports iic_sda]     ; 


set_property  -dict {PACKAGE_PIN  G14  IOSTANDARD LVCMOS33} [get_ports rx1_band_sel_h]  ;
set_property  -dict {PACKAGE_PIN  C20  IOSTANDARD LVCMOS33} [get_ports rx1_band_sel_l]  ;
set_property  -dict {PACKAGE_PIN  B19  IOSTANDARD LVCMOS33} [get_ports tx1_band_sel_h]  ;
set_property  -dict {PACKAGE_PIN  B20  IOSTANDARD LVCMOS33} [get_ports tx1_band_sel_l]  ;
set_property  -dict {PACKAGE_PIN  E17  IOSTANDARD LVCMOS33} [get_ports rx2_band_sel_h]  ;
set_property  -dict {PACKAGE_PIN  A20  IOSTANDARD LVCMOS33} [get_ports rx2_band_sel_l]  ;
set_property  -dict {PACKAGE_PIN  D18  IOSTANDARD LVCMOS33} [get_ports tx2_band_sel_h]  ;
set_property  -dict {PACKAGE_PIN  D19  IOSTANDARD LVCMOS33} [get_ports tx2_band_sel_l]  ;

set_property  -dict {PACKAGE_PIN P19  IOSTANDARD LVDS_25 } [get_ports  tx_clk_out_n     ]   ;
set_property  -dict {PACKAGE_PIN N18  IOSTANDARD LVDS_25 } [get_ports  tx_clk_out_p     ]   ;
set_property  -dict {PACKAGE_PIN Y14  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_n[0] ]   ;
set_property  -dict {PACKAGE_PIN W14  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_p[0] ]   ;
set_property  -dict {PACKAGE_PIN U12  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_n[1] ]   ;
set_property  -dict {PACKAGE_PIN T12  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_p[1] ]   ;
set_property  -dict {PACKAGE_PIN U15  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_n[2] ]   ;
set_property  -dict {PACKAGE_PIN U14  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_p[2] ]   ;
set_property  -dict {PACKAGE_PIN U17  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_n[3] ]   ;
set_property  -dict {PACKAGE_PIN T16  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_p[3] ]   ;
set_property  -dict {PACKAGE_PIN W13  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_n[4] ]   ;
set_property  -dict {PACKAGE_PIN V12  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_p[4] ]   ;
set_property  -dict {PACKAGE_PIN W15  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_n[5] ]   ;
set_property  -dict {PACKAGE_PIN V15  IOSTANDARD LVDS_25 } [get_ports  tx_data_out_p[5] ]   ;
set_property  -dict {PACKAGE_PIN Y17  IOSTANDARD LVDS_25 } [get_ports  tx_frame_out_n   ]   ;
set_property  -dict {PACKAGE_PIN Y16  IOSTANDARD LVDS_25 } [get_ports  tx_frame_out_p   ]   ;
set_property  -dict {PACKAGE_PIN P20  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_clk_in_n      ];
set_property  -dict {PACKAGE_PIN N20  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_clk_in_p      ];
set_property  -dict {PACKAGE_PIN Y19  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_n[0]  ];
set_property  -dict {PACKAGE_PIN Y18  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_p[0]  ];
set_property  -dict {PACKAGE_PIN V18  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_n[1]  ];
set_property  -dict {PACKAGE_PIN V17  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_p[1]  ];
set_property  -dict {PACKAGE_PIN W20  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_n[2]  ];
set_property  -dict {PACKAGE_PIN V20  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_p[2]  ];
set_property  -dict {PACKAGE_PIN R17  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_n[3]  ];
set_property  -dict {PACKAGE_PIN R16  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_p[3]  ];
set_property  -dict {PACKAGE_PIN W19  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_n[4]  ];
set_property  -dict {PACKAGE_PIN W18  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_p[4]  ];
set_property  -dict {PACKAGE_PIN W16  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_n[5]  ];
set_property  -dict {PACKAGE_PIN V16  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_data_in_p[5]  ];
set_property  -dict {PACKAGE_PIN U19  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_frame_in_n    ];
set_property  -dict {PACKAGE_PIN U18  IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports  rx_frame_in_p    ];

# clocks

create_clock -name rx_clk       -period  8 [get_ports rx_clk_in_p];
