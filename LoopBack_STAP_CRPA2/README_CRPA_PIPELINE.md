# LoopBack_STAP_CRPA2: ANTSDR E310 V1 GNSS L1 loopback pipeline with CRPA anti-jam

```
RX1 ─┐                          ┌─────────────── PL (l_clk) ───────────────┐
     ├─ AD9361 ─ axi_ad9361 ────┤ gnss_stap ──────► gnss_passthrough ──────┼─ axi_ad9361 ─ AD9361 ─ TX1
RX2 ─┘        (2R2T, 12 bit)  │ │ 2 el × 16 taps    (unchanged: elastic    │                         TX2
                              │ │ STAP beamformer    buffer, [11:0]→[15:4])│
                              │ └───────▲──────────────────────────────────┘
                              │         │ AXI4-Lite 0x43C1_0000 (CDC bridge)
                              │   ARM (sys_cpu_clk): gnss_stap.c + stap_solver.c
                              │   covariance → Cholesky → weights → commit
                              └─► util_wfifo → DMA (raw RX capture, untouched)
```

`LoopBack_STAP_CRPA2` is `LoopBack_Pipeline_noCRPA` with the STAP core from `stap_fpga` added as
a **CRPA (controlled-reception-pattern antenna) anti-jam stage**. RX1 and RX2 of the
AD9361 form a 2-element array. Both channels share one LO, so the array is
phase-coherent. The stage sits between the AD9361 RX outputs and the existing
`gnss_passthrough`. Everything downstream (elastic buffers, RX→TX alignment,
DAC mux, TX safety interlocks, DMA capture) is unchanged.

**At reset the stage is a bypass.** The output is bit-for-bit the RX samples,
so a board with this bitstream behaves exactly like the plain loopback until
software turns the beamformer on.

## What a 2-element array can do

| Scenario (model: 30.72 MSPS, 12-bit ADC, wideband noise jammer) | Jammer suppression | Null depth | SINR in → out |
|---|---|---|---|
| 1 jammer, JSR 50 dB, power inversion (default) | 35.1 dB | −39 dB | −50.1 → −31.3 dB |
| 1 jammer, JSR 70 dB, power inversion | 45.3 dB | −52 dB | −70.0 → −31.5 dB |
| 1 jammer, JSR 50 dB, MVDR (look 0°, λ/2 spacing known) | 49.2 dB | −37 dB | −50.1 → −20.2 dB |
| **2 jammers**, power inversion | **0.2 dB** | – | no improvement |

- **Two elements can null one jammer.** Nulling N wideband jammers needs at
  least N+1 antennas, which is a property of the array, not of this
  implementation. The second case in the table shows that more jammers than
  degrees of freedom get through untouched. For more jammers you need more
  phase-coherent receive channels, for example two synchronised AD9361s or a
  4-channel transceiver, with `M` raised in `gnss_stap.v` and `stap_config.h`.
- The fixed-point FPGA results match floating point to within 0.1 dB in every case.
- Power inversion needs no antenna geometry or calibration. MVDR needs the
  antenna spacing and look direction, and works properly only after RX1/RX2
  phase calibration.
- These are simulation numbers. On the bench, expect less because of RX1/RX2
  mismatch and the independent AGC loops of the two channels. See
  [Bring-up](#bring-up-on-the-board).

## What was added or changed

| File | Change |
|---|---|
| `Source/HDL/gnss_stap.v` | **New.** CRPA stage wrapper: same `adc_*` interface in and out, bypass mux, AXI split (local registers at 0xF000, the rest through the CDC bridge), status capture. |
| `Source/HDL/stap/stap_fir.v`, `stap_corr.v`, `stap_top.v` | **New**, copied from `stap_fpga/rtl`. Instantiated with M=2, N=16, FOLD=2, LAGP=8, OW=12. |
| `Source/HDL/stap/axil_cdc.v` | **New.** AXI4-Lite bridge from `sys_cpu_clk` to `l_clk`, toggle handshake. It times out after 4096 cycles and returns `0xDEADC10C` when `l_clk` is not running, so a CPU access cannot hang GP0. |
| `Source/IP/gnss_stap/gnss_stap_ip.tcl` | **New.** IP packaging, VLNV `antsdr:gnss:gnss_stap:1.0`, 64 kB aperture. |
| `Source/XDC/gnss_stap_cdc.xdc` | **New.** CDC constraints, added at project level with PROCESSING_ORDER LATE, same scheme as `gnss_passthrough_cdc.xdc`. |
| `Source/BlockDesign/system_bd.tcl` | **CRPA MOD-3**: `gnss_stap` inserted between `axi_ad9361/adc_*` and `gnss_passthrough/adc_*`, AXI at **0x43C1_0000**, catalog check. `gnss_stap` is mapped **before** `gnss_passthrough` (see the address-map note below), and both addresses are verified. |
| `Automation/Linux/build_all.sh` | Stage 3 also packages `gnss_stap`. Stage 4 checks it is there. |
| `Vivado/Scripts/create_project.tcl` | Adds `gnss_stap_cdc.xdc` (LATE). |
| `Vitis/Scripts/build_software.tcl` | Links `libm`, which the solver needs. |
| `Source/Firmware/app_gnss_e310/gnss_stap.[ch]` | **New.** Driver: probe, adaptation (capture → read → solve → commit), background service, console commands, diagnostics. |
| `…/stap_solver.[ch]`, `stap_config.h` | **New.** Weight solver from `stap_fpga/sw`, compiled for M=2 by `stap_config.h`. |
| `…/console.c` | `uart_read_char()` polls the UART and runs `gnss_stap_service()` while waiting, so adaptation continues in the background. |
| `…/command.c` | `stap_*` console commands. |
| `…/gnss_l1.c` | Self-test probes the CRPA stage. This check is not fatal, because bypass is the plain loopback. |
| `…/main.c` | Deployment build (`GNSS_DEPLOY_AUTO_TX`): after TX starts, one adaptation runs, then the anti-jam output is enabled with 100 ms background adaptation. Disable with `GNSS_DEPLOY_NO_CRPA`. |
| `…/gnss_info.c` | Menu shows the CRPA state and commands. |
| `Source/Config/board_e310_v1.json` | `gnss_stap` address and `crpa_register_map`. |
| `Source/Sim/`, `Automation/Linux/run_sim.sh` | **New.** Bit-true model and integration testbench (below). |

`gnss_passthrough.v`, `system_top.v`, `system.xdc`, the PS7 configuration and
the vendor tree are unchanged.

## Address map note ([BD 41-1075] fix)

The original `gnss_passthrough` IP has **two** slave segments: the packager infers `reg0`, and the
packaging script adds its own `axi_lite`. `ad_cpu_interconnect` puts the first at 0x43C0_0000 and
auto-assigns the second to the next free 64 kB slot, which was 0x43C1_0000. This was harmless in the
plain loopback, but `gnss_stap` needs that address, so project creation failed with
`[BD 41-1075] ... overlaps with slave segment '/gnss_passthrough/s_axi/axi_lite'`.

The fix:

- `gnss_stap_ip.tcl` removes the inferred block, so `gnss_stap` has exactly one address block, and
  the packaging self-check enforces this.
- `system_bd.tcl` maps `gnss_stap` first, so the passthrough's extra segment (an alias of the same
  registers) lands in some other free slot.
- `system_bd.tcl` stops with a clear message if an old two-segment `gnss_stap` package is still in
  `Build/ip_repo`.
- After mapping, it verifies 0x43C1_0000 for `gnss_stap` and 0x43C0_0000 for `gnss_passthrough`.

The `gnss_passthrough` IP itself is unchanged. If you hit this error, re-package the IP:

```bash
./Automation/Linux/build_bin.sh --from ip
```

## Datapath details

- **Input:** `adc_data_*[11:0]` of RX1 and RX2, re-sign-extended in the wrapper.
  `in_valid = adc_valid_i0 & adc_enable_i0`. `axi_ad9361_lvds_if` asserts
  `adc_valid` at most every 2nd `l_clk` (1R1T and 2R2T). That is the rate
  FOLD=2 requires, so the filter needs only 16 complex MACs (48 DSP48).
  If the strobes ever come faster, `STATUS.overrun` flags it.
- **Filter:** `y[n] = Σ_m Σ_k conj(w[m·16+k]) · x_m[n+k]`, 18-bit weights, exact
  43-bit accumulation, `>>> OUT_SHIFT`, **saturated to 12 bits**, so the
  passthrough's `[11:0]→[15:4]` alignment can never wrap.
- **Covariance:** 5000 snapshots, 3 channel pairs × 31 lags, 8 lags per pass.
  That takes about 60k `l_clk` cycles, ≈ 1 ms at 30.72 MSPS. The ARM rebuilds
  the exact 32×32 `R = Xm·Xm'/L` with the edge-sample correction, then solves
  with Cholesky in double precision. The solve is microseconds for T=32.
- **Weight switch:** the weights are double-buffered and switch on a sample
  boundary, so background updates do not glitch the TX stream.
- **Outputs:** channel 0 carries the beamformer. Channel 1 carries raw RX2,
  or a copy of the beamformer when `stap_ch1=1`.

## SD card: anti-jam straight from power-on

The default build is the SD-card build. Copy `BOOT.BIN` to the card and power on; no console input is
needed. The board then does this by itself:

1. Initialises the AD9361 and runs the self-test.
2. Starts TX1 on the loopback path.
3. Applies the **power-on band** (chosen with `--band N` when building, default 1): centre frequency,
   sample rate, bandwidths, RX1/RX2 gain and TX attenuation (**20000 mdB**).
4. Runs the **TX → RX leakage check** (below) and prints the result.
5. Computes the CRPA weights and switches TX to the **anti-jam output**.
6. Keeps **updating the weights every 100 ms** in the background.

```bash
./Automation/Linux/build_bin.sh --band 1
```

The console (PuTTY, 115200 8N1) stays usable for changes while it runs: `gnss_band=N`,
`tx1_attenuation=N`, `gnss_loop?`, `stap_status?`.

## GNSS bands: `gnss_band=1` … `gnss_band=5`

A single command sets the RX and TX centre frequency (always equal), the sample rate (RX = TX), the RX/TX
RF bandwidth, the RX1 and RX2 gain (manual and equal on both CRPA elements), and the TX1 attenuation:

| N | Centre | Rate | RX / TX BW | Gain | TX att | Name |
|---|---|---|---|---|---|---|
| 1 | 1575.420 MHz | 30.72 MSPS | 18 / 18 MHz | manual 40 dB | 20000 | GPS L1 / Galileo E1 wide (power-on default) |
| 2 | 1575.420 MHz | 15.36 MSPS | 4 / 4 MHz | manual 40 dB | 20000 | GPS L1 C/A narrow |
| 3 | 1575.420 MHz | 30.72 MSPS | 18 / 18 MHz | manual 25 dB | 20000 | GPS L1 low gain (strong jammer) |
| 4 | 1561.098 MHz | 15.36 MSPS | 4 / 4 MHz | manual 40 dB | 20000 | BeiDou B1I |
| 5 | 1602.000 MHz | 15.36 MSPS | 10 / 10 MHz | manual 40 dB | 20000 | GLONASS L1 |

```
gnss_band?          list the bands, * marks the active one, and the power-on band
gnss_band=2         apply band 2 (the CRPA weights re-adapt automatically)
tx1_attenuation=15000   change the TX level (10000..20000 as planned; up to 89750 allowed)
gnss_loop?          TX -> RX leakage check at the current level
gnss_status?        read back: LOs, rate, bandwidths, RX1/RX2 gain, attenuation, band
```

`gnss_mode=` / `gnss_mode?` still work as aliases.

**Order of operations.** The band first sets TX attenuation to its maximum, mutes the DAC and turns the
loopback buffer off. It then sets the LOs (verified by readback), the sample rate (RX and TX verified
equal), the bandwidths, and the gain on both channels (verified). Next it re-primes the loopback buffer
and clears the CRPA overrun flag; if the anti-jam is on, it re-adapts the weights for the new band. The
band's TX attenuation is set last.

**Limits.** A band is refused, with nothing touched, if any value is outside these limits:

- centre frequency 1100–1700 MHz (the L band, where the boot RF switch setting stays valid)
- sample rate 2.083–30.72 MSPS
- bandwidth 0.2 MHz to the sample rate
- manual gain 0–71 dB
- attenuation 10000–89750 mdB

**To change a band,** edit the `gnss_bands[]` table at the top of
`Source/Firmware/app_gnss_e310/gnss_band.c`, then rebuild the firmware only:

```bash
./Automation/Linux/build_bin.sh --from sw
```

## TX → RX leakage check (`gnss_loop?`)

The E310 retransmits on **the same frequency it receives**, on the same board, so part of TX1 always
couples back into RX1/RX2. At 10–20 dB attenuation TX1 puts out roughly −3 to −13 dBm. If the board's
TX→RX isolation is not large enough, that leaked signal sits above the receiver noise. The loopback then
feeds its own output back into itself, the satellites are buried, and the CRPA sees the leakage as a
jammer.

The check measures the RX1/RX2 input power with the DAC muted and with it running:

```
---- TX -> RX leakage check (TX1 attenuation 20000 mdB) ----
  RX1 input: TX muted -30.00 dBFS, TX on -29.80 dBFS, rise +0.20 dB
  RX2 input: TX muted -29.50 dBFS, TX on -30.30 dBFS, rise -0.80 dB
  OK: no significant TX -> RX leakage at this attenuation
```

A rise of more than 3 dB prints a WARNING with the next attenuation to try. Raising `tx1_attenuation` by
10000 lowers the leakage by 10 dB. It runs automatically at power-on and can be repeated with `gnss_loop?`.

### Related fixes in this version

| Where | Fix |
|---|---|
| `rx_lo_freq=` / `tx_lo_freq=` | Accept decimal MHz (`1575.42` now means 1575.420 MHz; the vendor code truncated it to 1575). Both LOs are read back in kHz, with a warning when RX ≠ TX. |
| `rx_samp_freq=` / `tx_samp_freq=` | Rates above 30.72 MSPS are refused. In 2R2T LVDS mode the interface clock is 4 × fs, and the FPGA is timed for 125 MHz. |
| `rx1_rf_gain=` / `rx2_rf_gain=` | Refused, with a message, while that channel is in AGC mode; previously the AD9361 ignored the value silently. The gain is read back. |
| `gnss_status?` | Shows the RX2 gain, both gain-control modes and the active `gnss_band`. |
| `gnss_passthrough` v1.2 | The elastic buffer is flushed while `pass_en = 0`, so re-enabling always re-primes from empty. A sample-rate change can no longer leave it stuck full (the `overflow` / `fifo level 31` state). |
| CRPA | The `overrun` flag is cleared after the boot-time interface tuning and after each `gnss_band=`, so a `1` in `stap_status?` now means a real problem. |

## Console

```
stap_status?        state, element powers, predicted output power, power removed
stap_update=1       one adaptation now (blocking, ~ms)
stap_en=1 / =0      TX carries the anti-jam output / bypass (raw RX1)
stap_auto=100       adapt every 100 ms in the background (0 = off)
stap_mode=0 / =1    power inversion (default) / MVDR
stap_theta=0        MVDR look direction [deg]
stap_len=5000       snapshots per estimate (64..8162)
stap_ch1=0 / =1     TX2 = raw RX2 / TX2 = beamformer
```

`stap_en=1` is refused until the first successful update, so TX never carries
an empty beamformer (all-zero weights mean silence). Typical session, after
the self-test and with the conducted TX path set up as before:

```
gnss_tx=1          (unchanged loopback start, attenuation interlock as before)
stap_update=1
stap_en=1
stap_auto=100
stap_status?
```

The register map is in the headers of `Source/HDL/gnss_stap.v` and
`Source/HDL/stap/stap_top.v`, and in `Source/Config/board_e310_v1.json`.

## Verification

### Done here, without Xilinx tools

`Automation/Linux/run_sim.sh` needs gcc, python3 + numpy and iverilog. It builds the
host copy of the firmware solver, generates the E310 scenario (2 elements,
30.72 MSPS baseband, 12-bit samples, 50 dB JSR jammer at 30°), and runs
`Source/Sim/tb_gnss_stap.v`. The testbench runs **`gnss_stap` → `gnss_passthrough`** with two
asynchronous clocks (61.44 MHz `l_clk`, valid on every 2nd clock; 100 MHz AXI):

```
A: GSTP=47535450 STAP=53544150 CONFIG=12101002 GCP1=47435031
B: bypass DAC samples matched 360, mismatched 0
C: correlations 0/93 mismatches, edge samples 0/120 mismatches
D: STAP outputs 2000 checked, 0 mismatches; DAC matched 1960, mismatched 0; overrun=0
F: passthrough v1.2: FIFO level after pass_en=0: 0, after re-enable: 16
E: l_clk stopped: STAP ID read deadc10c, WRAP_STATUS 00010000, GSTP 47535450
*** PASS tb_gnss_stap ***
```

- **A:** registers respond: the local wrapper, the STAP core through the CDC bridge, and the passthrough.
- **B:** bypass delivers TX = RX << 4, as in the original design.
- **C:** covariance results and edge samples are bit-exact.
- **D:** every anti-jam output sample is bit-exact, and the DAC stream carries those samples left-aligned.
- **F:** with `pass_en=0` the passthrough's elastic buffer is flushed, and it re-primes to half full when re-enabled.
- **E:** with `l_clk` stopped, the bridge times out, returns `0xDEADC10C`, and the bus stays usable.

- **Firmware:** `gnss_stap.c`, `stap_solver.c` and `console.c` compile warning-free
  with host gcc against stub BSP headers. `command.c`, `gnss_info.c`, `gnss_l1.c` and
  `main.c` show no errors from the new code; only BSP symbols were missing on the host.
- **Resources:** Yosys (`synth_xilinx -family xc7`) estimate for
  `gnss_stap` alone: 72 DSP48E1, 19 RAMB36, ~9.6k LUT, ~10k FF. The XC7Z020
  has 220 / 140 / 53.2k / 106k, and the vendor E310 design uses part of that.

### Not done

- **Not built in Vivado/Vitis 2023.2, and not run on the board.** The IP
  packaging, block design, constraint and XSCT changes follow the existing
  scripts line for line, but they have not been executed.
- **Not timing-closed.** `rx_clk` is constrained at 8 ns. The STAP datapath is
  fully pipelined, but check `BUILD_TIMING` in `Build/Logs/build.log`.
- No RF test.

## Bring-up on the board

1. Build as before. Stage 3 now packages two IPs:
   `./Automation/Linux/build_all.sh --jobs 16 --to boot`.
2. The console should print `gnss_stap: CRPA stage present at 0x43c10000: 2 elements x 16 taps`
   after `SELFTEST: PASS`. If it reports the stage absent, the IP did not make
   it into the bitstream.
3. **Connect RX2.** Both elements need an antenna, or for a conducted test a
   splitter with different cable lengths, plus a jammer source combined in.
   With RX2 empty the array has nothing to work with.
4. **Gain control:** the vendor configuration runs a slow-attack AGC on RX1 and
   RX2 independently. For the deepest nulls, set both channels to the same
   manual gain (vendor commands `rx1_gc_mode=0`, `rx2_gc_mode=0`, then
   `rx1_rf_gain=N`, `rx2_rf_gain=N`). Otherwise the
   weights have to follow the AGC; `stap_auto=100` does that.
5. Compare `stap_status?` "power removed" with the jammer on and off. With a
   strong jammer on, it approximates the suppression.

The TX safety rules of the loopback are unchanged: conducted, attenuated coax
only. `gnss_tx=0` still silences TX1 immediately.

## Build on Ubuntu: sources to `.bin`

`Automation/Linux/build_bin.sh` builds everything in one command and stops at the first problem.

**One-time setup (Ubuntu 22.04):**

1. Install Vivado and Vitis 2023.2, for example under `/tools/Xilinx`.
2. Install the build tools:

   ```bash
   sudo apt install make python3 python3-numpy iverilog gcc
   ```

   iverilog, python3-numpy and gcc are only needed for the pre-build simulation.

**Build:**

```bash
cd ~/LoopBack_STAP_CRPA2
chmod +x Automation/Linux/*.sh
./Automation/Linux/build_bin.sh
```

The script finds `/tools/Xilinx/Vivado/2023.2/settings64.sh` and the Vitis one by itself. For another install location, pass `--xilinx /opt/Xilinx`.

| Step | What it does |
|---|---|
| 0 preflight | Checks the project tree, the Xilinx 2023.2 tools, free disk space and RAM. |
| 1 sim | RTL simulation of the CRPA datapath (about 30 s). It stops before Vivado if the RTL is broken. |
| 2 build | `build_all.sh` stages 1-8: vendor, libip, ip, project, build, sw, fsbl, boot. |
| 3 checks | Timing MET; `gnss_stap` and `gnss_passthrough` are in the XSA; the ELF contains the CRPA driver and is the variant that was asked for. |
| 4 bit.bin | `bootgen -process_bitstream bin` produces `system_top.bit.bin`. |
| 5 release | Collects the outputs, `MANIFEST.txt` and `SHA256SUMS`. |

**Output** goes to `Build/Release/LoopBack_STAP_CRPA_<variant>_<timestamp>/`:

| File | Use |
|---|---|
| `BOOT.BIN` | SD card (FAT32, root). Contains FSBL + bitstream + application. |
| `system_top.bit.bin` | The bitstream alone in `.bin` form, for FPGA manager or QSPI tools. |
| `system_top.bit`, `system.xsa`, `e310_gnss_app.elf`, `fsbl.elf` | JTAG and Vitis. |
| `MANIFEST.txt`, `SHA256SUMS`, `logs/` | What was built and how. |

**Common variants:**

```bash
./Automation/Linux/build_bin.sh --band 2 --sd
```

SD-card build (the default) with power-on band 2. TX1 transmits at power-on with the anti-jam ON. Read `SD_Image/README_SD_CARD.md` first. `--sd` also copies the image to `SD_Image/BOOT.BIN`.

```bash
./Automation/Linux/build_bin.sh --variant dev
```

Console/JTAG build: TX silent at power-on, everything by hand.

```bash
./Automation/Linux/build_bin.sh --variant deploy-nocrpa
```

SD-card build of the plain loopback, with the CRPA stage in bypass.

```bash
./Automation/Linux/build_bin.sh --from sw
```

After a firmware-only change: rebuilds the ELF, FSBL and BOOT.BIN only.

```bash
./Automation/Linux/build_bin.sh --jobs 4 --clean
```

Fewer parallel jobs, and starts from a clean Build tree.

The default variant is `deploy` (SD card, transmits at power-on). A `dev` image never transmits at power-on, and the script refuses to package one whose ELF does. The script also refuses a bitstream that misses timing. Pass `--allow-timing-violation` only for bench experiments. A full build takes roughly an hour and about 15 GB of disk. Follow the long Vivado step with:

```bash
tail -f Build/Logs/build.log
```

`build_all.sh` still works on its own for running individual stages, with the same `--from`, `--to`, `--jobs` and `--defines` options.

Run the simulation alone, without Xilinx tools:

```bash
./Automation/Linux/run_sim.sh
```
