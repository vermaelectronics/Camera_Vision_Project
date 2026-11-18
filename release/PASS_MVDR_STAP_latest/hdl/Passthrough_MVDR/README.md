# Passthrough_MVDR (LVDS, 61.44 MSPS) with the MVDR2 anti-jam top

| | |
|---|---|
| Radio interface | axi_ad9361 LVDS 2R2T, l_clk = 4 x fs (245.76 MHz at 61.44 MSPS), no TDD engine |
| Anti-jam block | `rtl/gnss_passthrough.v`, copied UNCHANGED from the working MVDR2 design |
| BD wrapper | `rtl/gnss_passthrough_bd.v` (interface attributes only, no logic) |
| Base address | **0x43C10000** (0x43C00000 is the ant VCXO controller; MVDR2 had 0x43C00000) |
| MVDR RTL | identical to the MVDR2 `src` folder (byte-for-byte compared) |

## Wiring (same nets as the MVDR2 block design)
- adc_enable/valid/data i0 q0 i1 q1 from axi_ad9361
- dac_enable/valid i0 q0 i1 q1 from axi_ad9361 (the elastic buffer reads on dac_valid)
- dma_dac_data_* from tx_upack (vendor DMA path when pass_en = 0)
- dac_data_* to axi_ad9361
- clk = l_clk, rst = axi_ad9361/rst, s_axi on sys_cpu_clk

## Register map
As in MVDR2: 0x000 ID 0x47435031 "GCP1", 0x00C CONTROL ([0] pass_en, [1] mute,
[3] ch1_copy, [4] mvdr_sel), 0x100.. MVDR window, 0x400..0xFFC SMI-STAP.

## Simulation
`iverilog -g2012 -o tb tb/tb_gnss_passthrough.v rtl/*.v && vvp tb`
IDs, SMI INFO 0x03081053, DMA path when off, RX1 -> TX1 bit exact (x16, left
aligned) on every dac_valid with no discontinuity, no FIFO over/underflow. errors = 0.

## Build
`make clean && make` in this folder. Firmware: the ported MVDR2 firmware
(GNSS_PT_BASEADDR = 0x43C10000).
