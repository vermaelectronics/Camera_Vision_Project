# SMI-PI 4.9.2 - STAP with 32 taps per antenna (based on 4.8.2)

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

* **Not synthesised.** The HLS core now has about 195 complex-multiplier DSP48 (65 complex taps x 3) plus 4 for g2 and the blanker: **roughly 205 to 215 of the 220 DSP48** (my estimate for 4.9.0).
  See section 4.9.1 below: that estimate was too low.
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

## 4.9.1: the DSP48 overflow of 4.9.0

The first build of 4.9.0 (Vivado 2023.2) stopped in `place_design`: **`DRC UTLZ-1`, 229 DSP48 required, 220 available** (synthesis: 205 DSP48E1, plus 24 DSP48E inside the HLS core's netlist).
My estimate of 205 to 215 was too low (the 65 complex taps cost 3 DSP48 each, and the blanker and the g2 normalisation come on top).

Fix: the RX1 samples are only 12 bits wide, so the RX1 taps are cheap to build from LUTs. `smi_pi.h` has a new `SMI_NFAB` (default **12**): the 12 RX1 taps with the highest delays
(k = 20..31) use LUT multipliers (`cmul3_fab`, `BIND_OP ... impl=fabric`) instead of DSP48. That is 36 multipliers: **about 36 DSP48 saved, about 8000 LUT spent**, so I expect about 193 DSP48 of 220
(88 %) and about 55 % LUT. The filter is mathematically the same: the HLS C simulation (scenarios 1-14, 16-21) still passes 20 of 20.
**Not synthesised:** the saving is my estimate from the log. If the next build still overflows, raise `SMI_NFAB` (up to 31; each tap saves 3 DSP48) in `smi_pi.h`; if the LUT use or the timing of the LUT multipliers is the problem, lower it.
Firmware string 4.9.1; the bitstream version (0x00040900) is unchanged because no HDL changed.

## 4.9.2: diagonal loading alpha tuned to 2 to 5 sigma^2 (firmware only)

Question from the board test of 4.9.1 (residual +15 to +25 dB, blanker cycling): is the loading tuned as "alpha = 2 to 5 sigma^2, clamped just above the thermal noise"? It was **not**:

| | 4.9.1 and 4.8.2 | 4.9.2 |
|---|---|---|
| alpha on the RX2 taps | 1 x s2 | **2 x s2** |
| alpha on the RX1 taps | 4 x s2 | **5 x s2** |
| s2 (sigma^2) | the canceller's own estimate Pout/(1+\|w\|^2), limited only to 0.25 LSB^2 .. input power; it **contains the jammer residual** | with a learned noise floor, **held within 1.25 x of the learned floor** (`AJ_SIG2_BAND`); the old estimate is used only while no floor is learned |

So with a learned floor alpha stays between 2 and 5 times the thermal noise (RX2 taps 2x, RX1 reflection taps 5x) and no longer follows the jammer residual. Before, a poor null raised s2, which raised the loading, which weakened the null: this can
keep a residual of +15 to +25 dB and make it fluctuate. I have **not proven** that this is what happened on the board; the C simulation does not show the problem (residual +0.2 dB in all scenarios, with or without the change).
Simulation, 32 taps: all 20 scenarios still PASS; suppression is about 1 dB lower on clean jammers (50.6 against 51.8 dB, scenario 21) and 0.9 dB lower on the long-echo cases, satellite loss unchanged. Constants: `AJ_RX2_LOAD`, `AJ_RX1_LOAD`, `AJ_SIG2_BAND` in `gnss_aj_math.h`.
**Firmware only: the bitstream of 4.9.1 stays.** Files: `gnss_aj_math.c`, `gnss_aj_math.h`, `gnss_aj.h` (version "4.9.2").
**The noise floor must be learned** (jammer OFF for 10 s after `gnss_tx=1`), otherwise the old self-estimate is still what sets the loading.

## 4.9.3: pulse blanker OFF by default (firmware only)

In the 4.9.1 board test the blanker blanked 100 % of the output (the jammer remnant stays above its threshold), switched itself off for 10 s, came back, and blanked again: a dropout of about 1 s every 11.2 s that blocks the decoding of the navigation message.
`blk_mode` now starts at 0 (off). `gnss_aj_blank=1` (automatic) or `=2` (always on) turns it on again; a pulsed jammer needs it. The bitstream is unchanged. Version string "4.9.3".
(Typed on the console at run time the same effect is `gnss_aj_blank=0`; the command `set pulse_blanker 0` does not exist in this firmware.)

## 4.9.4: 8192 samples per snapshot (firmware only)

Suggestion followed: with 65 weights, lengthen the integration of the covariance. There is **no Verilog accumulator** (`ACCUM_SAMPLES` / `CNT_WIDTH`) in this project: the statistics are built by the ARM from a DMA capture, and the length is `AJ_NSAMP` in `gnss_aj_math.h` (it was 4096).
By the Reed-Mallett-Brennan rule 65 weights need only about 130 samples for a 3 dB loss; with 4096 samples averaged over two blocks the estimation loss is already about 0.07 dB, so this is not expected to improve the null by itself.
Tested in the C simulation (32 taps, scenarios 1-14, 16-21):

| AJ_NSAMP | result |
|---|---|
| 4096 | 20 of 20 PASS |
| **8192 (chosen)** | **20 of 20 PASS**; suppression within 0.5 dB, satellite loss within 0.7 dB of 4096; the chirp detector still works (long memory on for 58 of 60 blocks) |
| 16384 | 17 of 20: **scenario 16 (1 ms chirp) fails** (the detector sees 0 of 60 blocks: the centre moves too far within a longer block) and scenarios 11 and 12 (J/N 61 and 51 dB) lose more satellite SNR (5.2 dB at J/N 61) |

Cost: the ARM needs about twice as long per solve (the statistics are proportional to the block length): expect roughly 20 solves/s instead of 40, an estimate not measured on the board. DDR buffer 64 KiB at 0x11000000 (no overlap with the boot captures at 0x10000000 / 0x10800000). `-DAJ_NSAMP=n` overrides it.
Firmware only; the bitstream of 4.9.1 stays. Version string "4.9.4".

## 4.9.5: seven fixed bands, TX always on (firmware only)

`gnss_band=N` now has **seven fixed presets** and nothing else; the old bands (L1 only, ALL 1582.5, no-BeiDou, no-GLONASS, GLONASS only) and the custom band `gnss_band=5 <LO> ...` are gone.
Each preset sets LO, sample rate, RF bandwidth and, for both TX channels and both RX channels, the TX attenuation and the RX gain:

| gnss_band | LO MHz | MS/s | bandwidth MHz | TX att dB | RX gain dB |
|---|---|---|---|---|---|
| **0 (default)** | 1569.42 | 61.44 | 56 | 30 | 50 |
| 1 | 1575.42 | 61.44 | 56 | 30 | 50 |
| 2 | 1575.42 | 61.44 | 56 | 20 | 40 |
| 3 | 1575.42 | **30.22** | 18 | 30 | 50 |
| 4 | 1575.42 | 61.44 | 56 | 10 | 40 |
| 5 | 1582.50 | 61.44 | 56 | 20 | 40 |
| 6 | 1575.42 | 61.44 | 56 | 30 | 50 |

(Band 6 is the same as band 1. Band 3 uses 30.22 MS/s as written; 30.72 would be the usual rate: tell me if that was a typo.)

**TX is always on.** `gnss_tx=1` is done by itself at start-up and again after every `gnss_band=` (`gnss_tx_always()` in `command.c`): TX is switched on through the normal safe sequence (maximum attenuation, data path, then the 70 dB start level), and then the band's TX attenuation and RX gain are applied.
`gnss_tx=0` still silences TX until the next boot or `gnss_band=`. **TX1 radiates the GNSS band: conducted, attenuated coax only, never an antenna.** Band 4 (10 dB) and band 2/5 (20 dB) are loud; the firmware cannot see what is attached to TX1.

Checked: the touched files pass `gcc -fsyntax-only` with stubs for the Xilinx headers (gnss_aj.c, command.c, gnss_passthrough.c, gnss_smi_agc.c, gnss_info.c; main.c only fails on board GPIO macros that the stubs do not define). **Not compiled for the ARM and not run on the board.** The bitstream of 4.9.1 stays.
Version string "4.9.5".

## 4.9.6: automatic snapshot log (firmware, optional SD file saving)

Every **60 s** (`gnss_log=N` seconds, 0 = off, 300 = 5 min) the firmware runs `gnss_status?`, `gnss_tx?`, `gnss_band?`, `gnss_rx_gain?`, `gnss_smi?`, `tx1_attenuation?`, `radio?`, `gnss_aj?` and `gnss_report?`
with their output captured, and adds a header and an `AJCSV` row. The block is **self-describing** so that it can be read by a person, a script, or an assistant that then changes the code from it:

```
===== GNSS_LOG BEGIN =====
# GNSS_LOG format 1: lines 'key=value', then sections '----- name -----'
fw=4.9.6   core_version=0x40900   aj_info=0x20040c07   band=0   uptime_s=...   period_s=60
time=2026-11-5 14:30:0   (or: not_set, uptime only)
----- gnss_status? -----  ...  ----- gnss_report? -----  ...
===== GNSS_LOG END =====
```

* **Default build (no extra define):** the block is printed on the console between the BEGIN / END lines, and your serial terminal's own log ("Start log...") keeps it.
* **SD build: `build_all.sh --from sw --to boot --defines GNSS_LOG_SD`.** The BSP gets the FatFs library (xilffs) and every snapshot is written as a text file `0:/LOG/<YYYYMMDD>/<HHMMSS>.TXT` (8.3 names, one folder per day). Without a clock the folder is `0:/LOG/UPTIME/` and the name `T<seconds>.TXT`. With a file the UART stays quiet: only `[LOG] saved ...` is printed.
* **No real-time clock on the board:** set the time after each boot, `gnss_time=2026 11 5 14 30 0`.
* Commands: `gnss_log=N`, `gnss_log?` (state, last file), `gnss_log_now=1` (one snapshot now), `gnss_time=Y M D h m s`.

**Checked:** the C files pass `gcc -fsyntax-only` with stubs, with and without `GNSS_LOG_SD`; the date arithmetic was tested on the PC (leap year, rollover). **Not checked:** the xilffs library addition in `build_software.tcl` (Vitis 2023.2 XSCT; if it fails the script prints `SW_NOTE: xilffs could NOT be added` and the compile stops at `ff.h`: build without the define), the SD mount and file writes on the board. A snapshot takes a few milliseconds to a few tens of ms, during which the anti-jam loop waits (once a minute).

## 4.9.7: a new folder every hour, and a PC logger for what GNSS Viewer shows

**On the SD card (firmware, `--defines GNSS_LOG_SD`):** a new folder every hour: `0:/LOG/<YYYYMMDD>/H<hh>/<HHMMSS>.TXT`; without a clock `0:/LOG/UPTIME/H<hours of uptime>/T<seconds>.TXT`.

**GNSS Viewer data cannot come from the E310.** The Venus 8 receiver is connected to the PC (COM11), not to the board. `Tools/gnss_logger.py` runs on the PC and records both streams with the PC clock, in a new folder every hour:

```
<out>\<YYYYMMDD>\H<hh>\            a new folder every hour
    <hhmm>.txt     a new file every minute, named by its time (1430.txt = 14:30:00 to 14:30:59): every line time-stamped and
                   tagged  E310 | console line     NMEA | $GN...     SUM  | the per-second summary row
    <hhmm>.csv     the per-second summary of that minute: status, fix quality and mode, satellites used, HDOP/PDOP/VDOP,
                   lat/lon/alt, speed, satellites per system (GPS, GLONASS, Galileo, BeiDou, QZSS), tracked count, mean and
                   max C/N0, time to first fix, and every satellite as sys_prn:cn0:elev:az
    session.txt    ports and start time (once per hour folder)
```
It saves on the computer it runs on: default `<out>` = `gnss_logs` in your user folder (`C:\Users\<you>\gnss_logs`); `--out` changes it.

It also sets the E310 clock at start (`gnss_time=` from the PC clock), so the firmware's own snapshots and the PC files share one time base. What you type in its window goes to the E310 (it replaces the PuTTY window).

```
pip install pyserial
python gnss_logger.py --e310 COM20 --gnss COM11          (close GNSS Viewer and PuTTY first; add --out D:\logs to choose the folder)
```
A serial port can be opened by only one program. If GNSS Viewer must stay open, use its menu DataLog to save its own file and start the logger with `--gnss none`.
Not logged: the receiver's binary messages (TTFF as the Viewer shows it, scatter view); the NMEA carries the fix, the satellites with C/N0, elevation and azimuth, DOP and position. `first_fix_s` is counted from the start of the logger.
Tested here with two pseudo serial ports, NMEA taken from your GNSS Viewer screenshots, a bad-checksum line (kept in the raw file, ignored in the CSV), and an hour/day change. Not tested with the real Venus 8 and a Windows COM port.
