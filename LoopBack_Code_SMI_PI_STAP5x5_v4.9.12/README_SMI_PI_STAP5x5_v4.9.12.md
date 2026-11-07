# Firmware: `gnss_lb=` loopback without gnss_passthrough (bitstream unchanged, 4.9.12)

`gnss_lb=1` switches TX1 to the ADI `axi_ad9361` core's own RX->TX loopback: DAC data select 8 (`axi_ad9361_tx_channel.v`: `dac_data_out <= adc_data`), the raw 12-bit RX1 sample, a few clock cycles, no DMA, no custom logic. gnss_passthrough and the anti-jam filter are bypassed (anti-jam switched off), TX2 is zero. RX gain and TX attenuation stay at the band values (level lock). `gnss_lb=0` restarts the normal gnss_passthrough path. `gnss_lb?` shows the mode. Unity digital gain: this path adds no amplification. Not tested on the board (syntax-checked only).

---

# Firmware update (level lock), bitstream unchanged (4.9.12)

RX1/RX2 gain and TX1/TX2 attenuation now stay at the values of the selected `gnss_band=N` (the table in `gnss_aj.c`, e.g. band 1: TX 30 dB, RX 50 dB).
* The anti-jam start no longer switches the gain to the matched loop, which moved RX gain (and TX attenuation) with the jammer level. `gnss_rx_agc=2` still does it if you ask for it.
* A **level lock** reads RX1, RX2 gain/mode and TX1, TX2 attenuation once a second. If one is off by more than 1 dB (gain) or 0.25 dB (attenuation), or the gain is not in manual mode, the band's value is written again and the console prints `LEVEL LOCK: ... repair #n`. Repairs are counted (`gnss_levels_repairs()`).
* Your own commands win: `gnss_rx_gain=`, `tx1_attenuation=`, `tx2_attenuation=`, `radio=` become the new locked value. `gnss_rx_agc=1` or `2` switches the gain lock off.
* Firmware-only change: rebuild with `./build_all.sh --from sw --to boot`; the 4.9.12 bitstream is unchanged. Not tested on the board (syntax-checked only).

---

# SMI-PI 4.9.12: STAP 5+5 taps (10 total), reference tap moved to delay 2

## Design
* 5 taps per antenna (RX1 delays 0..4, RX2 delays 0..4) plus 2 image taps = 12 weight words (10 adaptive + reference handling).
* **Reference tap at delay 2** (was 4). With few taps the reference position decides how the taps are spent: delay REF has REF taps before it (they let the filter see earlier samples) and NT-1-REF taps after it (the reach for reflections). Measured in the C simulation, REF=2 gives the best result for barrage jammers with 5 taps; REF=4 wastes taps and is much worse (51 dB barrage: suppression 40.5 dB, satellite loss 17 dB at REF=4, 51.2 dB and 2.6 dB at REF=2).
* The bypass path follows the reference automatically (the output is x1[n-REF]): no other HDL change. AJ_INFO byte 2 = 2, so firmware and bitstream must both be 4.9.12 (the firmware refuses to start the filter otherwise).
* To trade barrage performance for reflection reach, set `SMI_REF` in `smi_pi.h` and `AJ_REF` in `gnss_aj_math.h` to 3 and the AJ_INFO byte 2 in `gnss_passthrough.v` / `gnss_passthrough.h` to 03 (reflection scenario then 17.9 dB / 3 dB loss instead of 17.4 dB / 14 dB loss, barrage scenarios worse).

## Simulation results (g++ C simulation, 20 scenarios pass; not verified in Vivado or on the board)
| Scenario | 7+7 REF4 (4.9.10) | 6+6 REF4 (4.9.11) | 5+5 REF2 (4.9.12) |
|---|---|---|---|
| 30 dB wideband | 36.9 dB, res +0.5 | 33.3 dB, +1.0 | 35.5 dB, +0.65 |
| 41 dB barrage | 46.5 dB, +0.6 | 42.2 dB, +1.5 | 43.8 dB, +1.0 |
| 51 dB barrage | 53.0 dB, +0.9 | 47.6 dB, +2.8 | 50.3 dB, +1.6 |
| 61 dB barrage | passes | 48.5 dB, +8.9, loss 7.5 | 51.2 dB, +6.8, loss 2.6 |
| 30 dB + reflection | 20.7 dB, loss 4.4 | 19.1 dB | 17.4 dB, **loss 14 dB** |

## Cost
* A strong reflection (jammer echo 3 samples or more after the direct path) is cancelled poorly and can hurt the satellite SNR a lot (14 dB loss in the reflection scenario). Use 7+7 if multipath is strong.
* Jammers above 55 dB still leave +6.8 dB residual.
* Testbench limits for SMI_NT <= 5 are set to the measured values.

---

# SMI-PI 4.9.11: STAP 6+6 taps (12 total)

6 taps per antenna (was 7 in 4.9.10). Hardware gain over 7+7 is small (about 6 DSP48, about 0.01 W). Everything else (clock group fix, adaptive/manual averaging, logging, bands) is as in 4.9.10.

## Cost of 6+6 versus 7+7 (C simulation, g++, 20 scenarios; not verified in Vivado or on the board)
| Jammer | 7+7 | 6+6 |
|---|---|---|
| 30 dB wideband | suppression 36+ dB | 33.3 dB |
| 41 dB wideband | 46.5 dB, residual 0.6 dB | 42.2 dB, residual 1.5 dB |
| 51 dB wideband | 53.0 dB, residual 0.9 dB | 47.6 dB, residual 2.8 dB |
| 60 dB wideband | passes | 49.3 dB, residual 6.9 dB, satellite loss 4.3 dB |
| 61 dB wideband | passes | 48.5 dB, residual 8.9 dB, satellite loss 7.5 dB |
| 30 dB + reflection | 20.7 dB | 19.1 dB |

The testbench pass limits for SMI_NT <= 6 are set to these measured values (they are lower than for 7+7 and 32 taps). Strong jammers (above 50 dB) leave more noise and reduce satellite SNR. Use 7+7 (4.9.10) if you expect them.

---

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
