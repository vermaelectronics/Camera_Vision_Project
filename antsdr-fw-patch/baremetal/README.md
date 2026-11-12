# Design A on bare metal

`app_gnss_e310/` is the SMI-PI v4.9.14 firmware of
LoopBack_Code_SMI_PI_STAP7x7_v4.9.14 (`Source/Firmware/app_gnss_e310`), since
crpa6 with the SFAP-PI weight solver (128 frequency bins, `SFAP_BINS=64` for 64),
adapted to the design-A FPGA of this repository:

- `app_config.h`: `ANTSDR_DESIGN_A`, `gnss_passthrough` at `0x43C10000`,
  VCXO control at `0x43C00000`.
- `parameters.h`: AD9361 RESETB on GPIO 67, RF band switches on GPIO 86..93.
- `gnss_passthrough.h`: an address set in `app_config.h` wins over
  `xparameters.h` and over the old `0x43C00000` fallback.
- `gnss_vcxo.c/.h` (new): VCXO DAC 23000 at power-on; `vcxo?`, `vcxo_dac=`,
  `vcxo_ref=`.
- `gnss_band7.c` (new), `gnss_aj.c/.h`, `command.c/.h`: band 7 and the
  start-up band, saved on the SD card in `GNSSCRPA.CFG` (GNSS_LOG_SD build).
- `main.c`: VCXO set before the AD9361 starts; start-up band from the SD card.
- `gnss_info.c`: design-A address map.
- crpa5: `gnss_rx_agc=` and `gnss_aj_avg=` saved in `GNSSCRPA.CFG`; ADC clipping
  warning with manual gain. (`gnss_notch*.c`, `smi_pi_nco.h`: the CW notch of the
  crpa5 SMI-PI bitstream, not used with SFAP.)
- crpa6: `gnss_aj_math.c/.h` = the SFAP-PI v5.0 solver, `gnss_sfap_cfg.h` = FFT
  size, weight registers `AJ_WB/WC1/WC2` from 0x200 (the same files as
  `gnss_crpa/src`).

`Scripts/build_software.tcl` and `Scripts/build_fsbl.tcl` are the Vitis 2023.2
`xsct` scripts of the same project, unchanged.

Build: `../build.sh baremetal` (see `../BUILD_E310.md`, section 7b).
