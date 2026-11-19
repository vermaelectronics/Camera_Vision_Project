# SFAP_LVDS_latest7: SFAP-PI anti-jam build package

This package holds SFAP-PI v6.3 (a 256-bin space-frequency power-inversion filter) for the ANTSDR E310 LVDS board, with its own firmware `app_sfap_lvds`. It builds through the same pipeline as Pass_MVDR_STAP: Vivado → XSA → Vitis → `BOOT.bin`. It also has the same corrections: power optimization, the DDR DRC waiver, and the methodology waivers.

| Part | Where | Status |
|---|---|---|
| SFAP HDL (`gnss_passthrough` v6.3), block design, XDC | `source/HDL`, `source/BlockDesign`, `source/XDC` (copies in `project/Pass_SFAP`) | Unchanged from Source.zip |
| SFAP HLS core `sfap_pi` 5.0 | `source/HLS/sfap_pi` (sources) and `prebuilt_ip/sfap_pi` (Vitis HLS 2023.2 export) | Unchanged |
| Firmware `app_sfap_lvds` | `source/Firmware/app_sfap_lvds` | **6.3a**: 6.3 plus the two new `gnss_band` profiles below, and **`gnss_tx=1` runs automatically at boot**. Nothing else changed. |
| Vivado project scripts | `project/Pass_SFAP/system_project.tcl`, `finish_hw.tcl`, `ps7_ddr_drc.xdc` | New |
| Build scripts | `build_all.sh`, `scripts/build_ip.sh`, `build_hw.sh`, `build_fw.sh`, `build_fw.py` | New |

The STAP/MVDR design is not in this build. SFAP takes its place in the FPGA, and the two can't both fit (SFAP alone needs about 176 of the 220 DSPs).

## Firmware 6.3a: new gnss_band profiles

| `gnss_band=` | LO | Rate | RF bandwidth | RX gain | TX1 attenuation |
|---|---|---|---|---|---|
| 0 … 6 | As in 6.3 (1 = ALL, 1582.5 MHz, 61.44 MS/s, 56 MHz, is still the boot default) | | | unchanged | unchanged |
| **7** | **1568.5 MHz** | **30.72 MS/s** | **20 MHz** | **40 dB** | **30 dB** |
| **8** | **MANUAL**: your values | | | | |

- **`gnss_band=7`** covers 1558.5 to 1578.5 MHz: BeiDou B1I (1561.098) plus GPS L1, Galileo E1 and BeiDou B1C (1575.42).
  - The RX gain (both channels, which also becomes the gain loop's reference) and the TX1 attenuation are set when you **select** the profile.
  - If TX1 is off, `gnss_tx=1` starts at 30 dB instead of the usual 70 dB.
  - **30 dB attenuation is about −23 dBm at TX1, which is 40 dB louder than the default. Check what is connected to TX1 first.**
- **`gnss_band=8` (MANUAL)** uses the values in `gnss_aj.h` (`GNSS_BAND_MANUAL_*`; edit them there before building to change the default). To change them live:
  ```
  gnss_band_manual=LO_MHz RATE_MSPS BW_MHz RXGAIN_dB TX1ATT_dB
  gnss_band_manual=1568.5 30.72 20 40 30
  ```
  A gain or attenuation of `0` means "leave it unchanged". Live values are not kept after a reboot.
- `gnss_band_custom=LO RATE BW` (profile 6) works as before.
- The 1 s radio guard restores only the LO, rate and bandwidth, never the gain or attenuation. The matched gain loop and the TX level hold keep working as in 6.3.
- `gnss_band?` lists every profile with its gain and attenuation.

## What you need
- Vivado and Vitis **2023.2** in `/tools/Xilinx`, or pass your own path.
- The antsdr HDL tree you built Pass_MVDR_STAP in, for example `~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl`. It provides the ADI library, `axi_tdd` and `axi_vcxo_ctrl`.

## Build: step by step (Linux)

**1. Unzip the package and go into it:**
```bash
unzip -o ~/Downloads/SFAP_LVDS_latest7.zip -d ~/work
```
```bash
cd ~/work/SFAP_LVDS_latest7
```

**2. Load Vivado:**
```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh
```

**3. Build the custom IP repository.** This takes a few minutes. Add `--hls` to rebuild `sfap_pi` with Vitis HLS instead of using the prebuilt export.
```bash
./scripts/build_ip.sh
```

**4. Build the hardware: bitstream and XSA.** This takes about 2 to 3 hours. It copies the project to `<hdl>/projects/Pass_SFAP`; an older one there is renamed, not deleted.
```bash
./scripts/build_hw.sh ~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl
```
At the end it prints:
- the timing result (WNS and WHS);
- how many critical warnings are left, and their names;
- where the XSA is: `hw/PASS_SFAP.xsa`.

**5. Build the firmware and `BOOT.bin`.** This takes about 5 to 10 minutes.
```bash
./scripts/build_fw.sh /tools/Xilinx/Vitis/2023.2
```
```bash
ls -l output/
```

**Short way: all of steps 2 to 5 in one command.**
```bash
./build_all.sh ~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl
```

## If timing is not met (`TIMING NOT MET` in finish_hw.log)
`build_hw.sh` then runs **`fix_timing.tcl`** by itself. It sends the same synthesized design through 4 other implementation strategies in parallel and keeps the best run that meets timing:
- Performance_ExtraTimingOpt
- Performance_NetDelay_high
- ExplorePostRoutePhysOpt with AggressiveExplore
- Performance_Retiming

Every run keeps power optimization and all the waivers. The RTL, block design, clocks and firmware are unchanged.

To run it by hand on an existing Pass_SFAP project (about 1 to 1.5 hours):
```bash
cd ~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_SFAP
```
```bash
vivado -mode batch -source fix_timing.tcl -log fix_timing.log -nojournal
```
```bash
grep -E "impl_[0-9]  |BEST:|TIMING:|CRITICAL WARNINGS LEFT|NO RUN|XSA written" fix_timing.log
```

## Check everything (latest7)
After `build_hw.sh` and `build_fw.sh`, run one command. It checks the hardware reports, the XSA addresses, the platform, the app build settings, the firmware values, the ELF and `BOOT.bin`, and prints PASS / FAIL / INFO per item, saved to `check_report.txt`:
```bash
./scripts/check_all.sh ~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_SFAP /tools/Xilinx/Vitis/2023.2
```

## Check the hardware result
```bash
cd ~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_SFAP
```
```bash
grep -E "WAIVED|TIMING:|CRITICAL WARNINGS LEFT|LEFT:|NOT MET|XSA written" finish_hw.log
```
```bash
grep -E "Total On-Chip Power|Junction Temperature" reports/3_power_temperature.txt
```
```bash
grep -E "^\| (Slice LUTs|Slice Registers|Block RAM Tile|DSPs) " reports/2_utilization.txt
```
Open the project in Vivado:
```bash
vivado passthrough_sfap.xpr &
```

## On the board
Copy `output/BOOT.bin` to the SD card, boot from SD, and open the console at 115200 baud. Then:
```
gnss_band=7          1568.5 MHz, 30.72 MS/s, 20 MHz, RX 40 dB, TX1 att 30 dB
gnss_tx=1            retransmit (already done at boot; again after gnss_tx=0)
sfap-pi_report?      everything: radio, gains, g2, jammer, events
gnss_band?           profiles and which GNSS signals they carry
```
**Auto start:** this build runs `gnss_tx=1` by itself right after boot, exactly as if you typed it. TX1 goes to maximum attenuation first, then to the start level: 70 dB with the default `gnss_band=1`. The console shows `AUTO START: gnss_tx=1`. `gnss_tx=0` silences it as usual. To get the RF-silent start of 6.3 back, set `GNSS_AUTO_GNSS_TX` to 0 in `source/Firmware/app_sfap_lvds/gnss_build_defines.h` and rebuild the firmware. **Use conducted, attenuated coax on TX1; the firmware can't see what is connected.**

## Notes
- **Power and temperature are not the same as the STAP build.** It's a different design: SFAP has more DSPs and a 256-bin FFT filter bank on the 100 MHz clock. The same corrections are applied (power optimization, waivers); compare `reports/3_power_temperature.txt` with STAP's 3.753 W and 68.3 °C.
- **Waivers:** TIMING-54, TIMING-9 and TIMING-18 (methodology) and IOSTDTYPE-1 on the PS7 DDR pins (DRC), each with a written reason. Any other critical warning is not waived; `finish_hw.tcl` lists it as `LEFT:`.
- **Linker settings:** the firmware is built with a 64 KiB stack and 64 KiB heap. The weight solver keeps 4 KiB arrays on the stack, so the 8 KiB template default is too tight.
- **Device-tree fix (latest3):** Vitis 2023.2 copied the `axi_tdd` default polarity `0b010` from the XSA into the generated `pl.dtsi`, and `dtc` cannot read binary numbers (`pl.dtsi:139.30-31 syntax error`). `build_fw.py` now gives Vitis a copy of the XSA (`build/PASS_SFAP_sdt.xsa`) with that value written as `2` in the `.hwh` description. It's the same number, and the bitstream is copied byte for byte. If platform creation still fails, the script prints dtc's error and the bad lines.
- **CPU clock define (latest4):** the Vitis 2023.2 BSP header `xparameters_ps.h` uses `XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ`, but the generated `xparameters.h` only has `XPAR_CPU_CORE_CLOCK_FREQ_HZ`. The timer constant `COUNTS_PER_SECOND` therefore didn't compile, which broke `gnss_aj.c` and `gnss_smi_agc.c`. `build_fw.py` now adds `XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ=XPAR_CPU_CORE_CLOCK_FREQ_HZ` to the app's `UserConfig.cmake`. It's the same CPU clock, and no firmware source changed.
- **Timer header (latest5):** the Vitis 2023.2 BSP has no `xtime_l.h`. Its replacement `XTime_GetTime()` (xiltimer) reads the 32-bit SCU private timer, which counts down. The SFAP firmware expects the classic 64-bit global timer counting up at CPU/2. `build_fw.py` writes an `xtime_l.h` into the app source folder that reads the global timer (`0xF8F00200`) the classic way, the same timer the STAP firmware's `gnss_time.h` uses. No firmware source changed.
- **gnss_passthrough address (latest6):** `gnss_passthrough.h` takes the register base from `xparameters.h`, and if it isn't there it falls back to `0x44A00000`. The SFAP block design puts the block at **`0x43C10000`**. `build_fw.py` now reads the address from the XSA. If `xparameters.h` has no entry, it passes `XPAR_GNSS_PASSTHROUGH_BASEADDR=<XSA address>` to the compiler. If the two disagree, it stops. The build log shows `gnss_passthrough base: XSA 0x43C10000 | xparameters.h ...`.
- **Tool version:** the SFAP firmware notes mention Vitis 2026.1, while the HLS, block design and this package use 2023.2. If the firmware build reports a missing `XPAR_...` name, send me the error.

## Audit (latest6): what was checked

| Area | Item | Result |
|---|---|---|
| Hardware | Timing (fix_timing, `impl_2` Performance_ExtraTimingOpt) | WNS +0.076 ns, WHS +0.037 ns: met |
| Hardware | Critical warnings | 0 (methodology 0, DRC 0); 31 methodology + DDR IOSTDTYPE-1 waived with reasons |
| Hardware | Clocks | 100 / 200 MHz (PS7), rx_clk 4 ns (AD9361 LVDS): unchanged from Source |
| Address map | AD9361 0x79020000, RX DMA 0x7C400000, TX DMA 0x7C420000, TDD 0x7C440000, VCXO 0x43C00000 | Block design = `parameters.h` |
| Address map | gnss_passthrough 0x43C10000 | Taken from the XSA by `build_fw.py` (the firmware fallback 0x44A00000 would be wrong) |
| BSP | PS7 SPI0 reference 166.67 MHz | Matches the `xparam_compat.h` default (the SPI0 warning is expected) |
| BSP | `xtime_l.h` / `COUNTS_PER_SECOND` | Generated header: 64-bit global timer, counts up, CPU/2 = 333,333,343 Hz |
| BSP | `XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ` | Mapped to `XPAR_CPU_CORE_CLOCK_FREQ_HZ` |
| Platform | dtc on `axi_tdd` `0b010` | XSA copy with `2` in the `.hwh` only; the bitstream is byte-identical |
| Firmware | All 28 C files against the real 2023.2 BSP headers | 0 errors (host syntax check) |
| Firmware | `gnss_band` 0 to 8, MANUAL, the radio guard | Guard restores LO / rate / bandwidth only; levels apply only when a band is selected |
| Firmware | Auto `gnss_tx=1` | Same code as the command: maximum attenuation first, then 70 dB (or the band's own attenuation) |
| Memory | DDR use | ELF from 0x100000 (about 2 MB + 64 KiB stack + 64 KiB heap), ADC 0x800000, captures 0x10000000 / 0x10800000, AJ 0x11000000, DAC 0xA000000: no overlap |

Not verifiable without the board: RF behaviour (null depth, satellite lock), real temperature (`sfap-pi_report?`, Zynq die temperature), and the final link on your Vitis 2023.2.
