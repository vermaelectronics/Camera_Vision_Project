# SMI-PI 4.9.9: STAP 16 taps, optimised for DSP, power and Vivado critical warnings

Built from the 4.9.8 folder (adaptive and manual averaging). Changes:

| Item | Before (Vivado, 32 taps) | Now |
|---|---|---|
| Taps per antenna | 32 | **16** (DSP48 expected about 110, was 193; estimate until Vivado runs) |
| LUT multipliers | 12 taps | none (they cost LUT and power) |
| Echo reach (reference at delay 4) | about 27 samples | about 11 samples |
| Vivado critical warnings | 9 (TIMING-6, TIMING-7, TIMING-54 x7) | expected 0 (see below) |
| Junction temperature | 61.5 C at 3.16 W | expected lower, see power notes |
| ARM capture period | at least 2 ms (up to 500 captures/s) | at least 5 ms (up to 200/s) |
| `CORE_VERSION` / `AJ_INFO` | 0x00040900 / 0x20040C07 | 0x00040909 / 0x10040C07 |

## Vivado critical warnings

* **TIMING-54 (7x):** the scoped `set_max_delay -datapath_only ... 8.000` constraints between clocks were converted to `set_multicycle_path -setup 2` and `-hold 1` on the same cells (the source holds for 256 cycles, so two 4 ns periods is the same budget). In `gnss_passthrough_cdc.xdc` and in `gnss_passthrough_constr.xdc`.
* **TIMING-6 and TIMING-7 (1x each):** `clk_fpga_0` (PS/AXI) and `rx_clk` (AD9361 samples) are unrelated, so they are now declared asynchronous: `set_clock_groups -asynchronous`.
* **Not verified:** these are not run in Vivado. Check the Methodology tab after the build: expect no Critical Warnings. If a warning remains, send its text.

## Power and junction temperature

Junction temperature = 25 C + 11.5 C/W x total power (the report's 3.16 W gives 61.5 C). The Vivado breakdown: PS7 1.54 W (52 %), I/O 0.40 W, signals 0.37 W, logic 0.31 W, clocks 0.18 W, DSP 0.15 W, device static 0.20 W.
* **DSP:** 16 taps saves about 0.06 W of DSP power.
* **Capture rate:** the ARM capture and solve run at up to 200/s instead of 500/s, so less DDR traffic and CPU load (part of the PS7 share).
* **Realistic gain:** about 0.1 to 0.3 W, so about 1 to 3 C lower. The PS7 share is mostly the ARM and DDR, which cannot be turned off, and the I/O is fixed by the AD9361 interface.
* **A larger drop** needs the vendor capture DMA to stop when the anti-jam is not capturing, or a lower sample rate. Those are hardware changes; tell me if you want them.

## Verification (simulation only)

* C model: **20 of 20** scenarios pass.
* Wideband 45 dB with an echo at 3 samples: 53.3 dB suppression, −0.2 dB satellite loss; at 8 samples: 51.1 dB, −1.7 dB; at 11 samples: 51.2 dB, 5.9 dB loss; at 14 samples: 44.3 dB, 18.8 dB loss (beyond the reach).
* HDL compiles (iverilog, strict Verilog-2005). Firmware file passes the syntax check.
* **Not verified:** the Vivado build, the DSP count, the timing and the board.

## 4.9.10: changed to 14 taps (7 + 7)

Per your request the optimised build uses **7 taps per antenna, 14 taps in total** (instead of 16 per antenna in 4.9.9).

* Simulation: 17 of 20 scenarios pass (the same three fail as in 4.9.8: wideband with a 3-sample reflection, and J/N 41 and 51 dB).
* Echo reach is only about 2 samples with the reference at delay 4: echo 1 gives 53.7 dB with 1.7 dB satellite loss; echo 3 gives 38.5 dB with 9.6 dB loss; echo 8 gives 32.5 dB with 18.6 dB loss.
* DSP48 expected about 45 (estimate until Vivado runs). Power and Vivado changes are the same as in 4.9.9.
* Not verified in Vivado or on the board.

If you meant 14 taps **per antenna** (28 in total), that is a one-line change to `SMI_NT` and `AJ_NT`; it reaches about 9 samples of echo.

## Redo notes (critical warnings, DSP)

* **Critical warnings:** the TIMING-54 constraints are scoped ones from the ADI library (up_xfer, up_clock_mon, util_clkdiv) on the `clk_div_sel_0_s/1_s` clocks, not only from this project. The asynchronous clock group in `gnss_passthrough_cdc.xdc` (processed LATE) now covers `rx_clk` and `clk_div_sel_*` against `clk_fpga_0`. Not verified in Vivado: if TIMING-54 remains after the build, send `report_methodology` and the XDC read order.
* **DSP:** the 24 legacy `DSP48E` cells come from the ADI `axi_ad9361` core (DAC DDS, IQ correction). They are NOT removed by default, because the firmware has DDS commands and uses `CHAN_CNTRL_7` to select DMA or DDS. If you do not need DDS or ADI IQ correction, set `DAC_DDS_DISABLE 1`, `ADC_IQCORRECTION_DISABLE 1`, `DAC_IQCORRECTION_DISABLE 1` in `Source/BlockDesign/system_bd.tcl` (about line 598-603) and remove the `dds_*` firmware commands. Expect up to about 24 DSP48 saved. Untested.
* **Power:** the PS7 (1.54 W) is not controllable from the PL, so the junction temperature gain is small (about 1 C).
