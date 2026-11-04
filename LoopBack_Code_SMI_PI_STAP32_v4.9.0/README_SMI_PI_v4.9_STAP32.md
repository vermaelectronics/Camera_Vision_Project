# SMI-PI 4.9.0 - STAP with 32 taps per antenna (based on 4.8.2)

One change against 4.8.2: the space-time filter has **32 taps per antenna** (it had 16). Everything else of 4.8.2 is kept: bypass when there is no jammer,
RX2 normalisation, the two image taps, the pulse blanker, the reference tap at delay 4, the ARM weight solver (now 65 adaptive weights + 2 image taps),
the gated detection, the make-up gain. No SFAP in this build.

**There is no notch filter in 4.8.2** (searched the HDL, the HLS core and the firmware), so nothing was removed. If you meant the pulse blanker
(`gnss_pulse_blank.v`, it zeroes samples during a pulse), say so; it is still in because pulsed jammers need it.

## What changed

| | 4.8.2 | 4.9.0 |
|---|---|---|
| Taps per antenna | 16 (delays 0..15) | **32 (delays 0..31)** |
| Reference tap (RX1) | delay 4 | delay 4 |
| Longest echo that can be cancelled | 11 samples (180 ns at 61.44 MS/s) | **about 27 samples (440 ns)** |
| Weight words | 32 + 2 image taps | **64 + 2 image taps** |
| Weight registers | `AJ_W` 0x100..0x17C | **0x400..0x4FC** (0x100..0x17C is free again; everything else keeps its address) |
| `AJ_INFO` | 0x10040C06 | **0x20040C07** (taps per channel 0x20, layout 7) |
| `CORE_VERSION` | 0x00040007 | **0x00040900** (the firmware checks `AJ_INFO`'s tap count and refuses to start the filter on a bitstream with another size) |
| HLS port `wts` | 1024 bit | **2048 bit** (`AP_INT_MAX_W` 4096 is defined in `smi_pi.h`) |
| ARM solver | 33 x 33 | 65 x 65 (4 times the statistics work, so expect roughly 40 solves/s instead of 100-160) |

## What was checked (simulation only)

* HLS C simulation, scenarios 1-14 and 16-21, with g++ and the public `ap_int.h` header: **all PASS**, the core is bit-exact against the test bench's model of the 32-tap filter.
* Same test bench, 16 taps against 32 taps. Scenario 22 (wideband jammer J/N 45 dB plus an echo of -6 dB, the echo arriving N samples later):

  | echo delay | 16 taps: suppression / satellite loss | **32 taps** |
  |---|---|---|
  | 3 samples | 54.1 dB / -0.5 dB | 54.6 dB / -0.2 dB |
  | 8 | 51.9 / -2.0 | 53.7 / -1.8 |
  | 11 | 52.3 / 5.3 | 53.9 / 2.1 |
  | 14 | 45.0 / 17.8 | **53.7 / -1.7** |
  | 20 | 16.3 / 14.1 | **52.3 / 3.3** |
  | 26 | 16.3 / 20.3 | **52.8 / -0.4** |

  Without an echo (scenarios 8, 19, 21) 16 and 32 taps are within 0.5 dB of each other: **the extra taps only help when the jammer has a long echo or the
  two receive paths differ strongly across the band.** They do not give more suppression on a clean jammer.
* Registers (Verilog test bench): 64 weight words written at 0x400..0x4FC, committed, all 64 words reach `aj_wts` and read back unchanged; `AJ_INFO` 0x20040C07, `VERSION` 0x00040900.

## What was NOT checked

* **Not synthesised.** The HLS core now has about 195 complex-multiplier DSP48 (65 complex taps x 3) plus 4 for g2 and the blanker: **roughly 205 to 215 of the 220 DSP48.**
  That is very tight; the place-and-route may fail on DSP count or timing. This is an estimate, not a report. If step 3 (HLS) or the Vivado build fails on DSP, send the log.
* Not on the board. The firmware was not compiled for the ARM (only the solver, which is shared with the C simulation, was compiled and run on the PC).
* The ARM solve rate with 65 weights was not measured.

## Build (up to BOOT.BIN)

```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh; source /tools/Xilinx/Vitis/2023.2/settings64.sh; source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh
cd ~/LoopBack_Code_SMI_PI_STAP32_v4.9.0 && chmod +x Automation/Linux/*.sh Automation/Linux/steps/*.sh
./Automation/Linux/build_all.sh --jobs 16 --to boot 2>&1 | tee build_STAP32.log
./Automation/Linux/steps/10_report.sh
```

## Vendor
The zip contains Vendor/. In git it is not duplicated: copy it from `LoopBack_Code_SMI_PI/Vendor` before building.
