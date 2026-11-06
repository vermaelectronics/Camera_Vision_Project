# SMI-PI 4.9.14: 7 + 7 taps, fewer DSP48, no TIMING-54 critical warnings, TX2 = CRPA copy (muted)

**4.9.14** is 4.9.13 plus one change: **TX2 now carries a copy of the CRPA output instead of raw RX2,
and it is muted** (digital zero, still at 89.75 dB). The filter, the DSP savings and the constraints are
those of 4.9.13 (section 3c has the TX2 change). 4.9.14 comes in three tap counts, see below. The HDL changed, so **the bitstream must be rebuilt**.

### The three 4.9.14 builds (this folder is **7 + 7**)

The three folders hold **the same code**. Only the tap count (`SMI_NT` / `AJ_NT`), the reference
delay (`SMI_REF` / `AJ_REF`) and the derived constants (`AJ_NSPT`, `AJ_IMG1/2`, `AJ_NW`,
`AJ_MAXDIM`, `AJ_INFO`) differ. TX2 (copy of the CRPA output, muted, 89.75 dB), the low-DSP
`axi_ad9361`, the TIMING-54 fixes, the real `g2` and the level lock are in all three.

| | **7 + 7** | 6 + 6 | 5 + 5 |
|---|---:|---:|---:|
| Folder | **`..._STAP7x7_v4.9.14`** | `..._STAP6x6_v4.9.14` | `..._STAP5x5_v4.9.14` |
| RX1 reference tap | **delay 4** | delay 3 | delay 2 |
| Taps before / after the reference | **4 / 2** | 3 / 2 | 2 / 2 |
| DSP48 in `smi_pi` | **47** | 41 | 35 |
| DSP48, whole design (estimate) | **~49** | ~43 | ~37 |
| `AJ_INFO` | **`0x07040C07`** | `0x06030C07` | `0x05020C07` |
| `gnss_report?` mode | **space-time 13 + 2** | space-time 11 + 2 | space-time 9 + 2 |
| C simulation | **20 / 20** | 20 / 20 | 20 / 20 |
| Barrage J/N 61 dB: residual | **+4.88 dB** | +5.29 dB | +6.80 dB |
| Barrage J/N 51 dB: residual | **+0.93 dB** | +1.04 dB | +1.64 dB |
| J/N 30 dB + reflection: satellite loss | **4.4 dB** | 4.9 dB | 14.2 dB |

C-sim numbers: residual = jammer left over the output noise (0 dB = fully nulled); "loss" = the
satellite SNR loss. All three use `CORE_VERSION 0x0004090E` (prints 4.2318); `AJ_INFO` tells them
apart, and the firmware refuses to start the filter if the bitstream has another tap count or
reference delay, so firmware and bitstream of different folders cannot be mixed by mistake.

**6 + 6 uses reference delay 3, not 4 as in 4.9.11.** With 6 taps and the reference at delay 4
only **one** tap followed the reference, so a jammer component arriving 2 samples late could not
be cancelled. Delay 3 gives 3 taps before and 2 after (as 7 + 7 and 5 + 5 have 2 after). Same DSP
count. C simulation, 6 + 6, delay 4 → delay 3: barrage J/N 61 dB residual +8.91 → **+5.29 dB**,
J/N 51 dB +2.79 → **+1.04 dB**, J/N 41 dB +1.47 → **+0.74 dB**, wideband J/N 30 dB +1.00 → **+0.50 dB**.
The one scenario where delay 4 was better is the J/N 30 dB reflection test (satellite loss 1.3 dB
instead of 4.9 dB). To go back to delay 4: `SMI_REF 4` in `Source/HLS/smi_pi/smi_pi.h`,
`AJ_REF 4` in `gnss_aj_math.h`, `AJ_INFO 32'h06040C07` in `gnss_passthrough.v` and
`GNSS_PT_AJ_INFO_EXPECTED 0x06040C07U` in `gnss_passthrough.h`.

**Which one?** 7 + 7 leaks least (best barrage residual, best with reflections). 6 + 6 is within
about 0.4 dB of it on barrage for 6 DSP48 less. 5 + 5 saves another 6 DSP48 but leaks about 2 dB
more at J/N 61 dB and is poor with reflections (14 dB satellite loss in scenario 5). The DSP48
difference between them is only about 0.01 W, so the choice is about leakage, not heat.

4.9.13 was built from **4.9.12** (5 + 5 taps, level lock) with these changes:

| # | Change | Why |
|---|---|---|
| 1 | **7 taps per antenna, reference tap at delay 4** (as 4.9.10) | 5 + 5 leaked: about +6.8 dB residual at J/N 61 dB in C-sim, +16 dB on the board |
| 2 | **`axi_ad9361` DDS, I/Q correction and DC filter removed** (block design MOD-3) | about 24 DSP48 that this design never uses; they toggle at 245.76 MHz all the time |
| 3 | **RX2 gain `g2` is real** in the HLS core | the firmware always wrote 0 into the imaginary half, so 2 multipliers were wasted |
| 4 | **Zero weights while bypassed** (no jammer) | the filter multiplied every sample by the last jammer's weights for nothing |
| 5 | **All 7 TIMING-54 critical warnings removed at their source** | see section 2 |
| 6 | The mode text now shows the real tap count (`space-time 13 + 2 image taps` here) | 4.9.12 printed "63 + 2" (the 32-tap layout) |
| 6b | **TX2 fixed at 89750 mdB (89.75 dB, the quietest setting); the bands set TX1 only** | TX2 is not used, but it re-transmitted RX2's **raw, unfiltered** samples (jammer included, `CONTROL[3] ch1_copy = 0`) at the band's 10-30 dB attenuation; a leak back into the antennas that no filter can cancel. See section 3b |
| 7 | HLS IP version **smi_pi 2.5**, `CORE_VERSION 0x0004090D` (prints 4.2317), `AJ_INFO 0x07040C07` | a stale 5 + 5 core or bitstream cannot be used by mistake |
| 8 | **4.9.14: TX2 = copy of the CRPA output, muted by default** (`CONTROL[3]` = `ch1_raw`, `CONTROL[4]` = `tx2_on`), `CORE_VERSION 0x0004090E` (prints **4.2318**) | even at 89.75 dB, TX2 should not carry an unfiltered copy of the jammer. See section 3c |

The SMI power-inversion algorithm itself (block SMI, Cholesky on the ARM, image taps,
RX2 normalisation, level lock) is unchanged.

---

## 1. DSP48 budget

The numbers below are counted from the code: 3 multipliers per complex tap (Gauss form) and
one DSP48 per multiplier. The final number is in Vivado's `utilization_impl.rpt`.

| Block | 4.9.11 (6 + 6) | 4.9.12 (5 + 5) | **4.9.14 (7 + 7)** | 4.9.14 (6 + 6) | 4.9.14 (5 + 5) |
|---|---:|---:|---:|---:|---:|
| `smi_pi` taps: (RX1 NT-1 + RX2 NT + 2 image) x 3 | 39 | 33 | **45** | 39 | 33 |
| `smi_pi` RX2 normalisation `g2` | 4 | 4 | **2** | 2 | 2 |
| `axi_ad9361` DDS + I/Q correction + DC filter | ~24 | ~24 | **0** | 0 | 0 |
| `gnss_pulse_blank` (I^2 + Q^2) | 2 | 2 | **2** | 2 | 2 |
| **Total** | **~69 (you saw "almost 70")** | ~63 | **~49** | ~43 | ~37 |

So 4.9.14 7 + 7 has **2 more taps per antenna and about 20 fewer DSP48** than 4.9.11 6 + 6.
The Zynq-7020 has 220 DSP48, so the design uses about 22 % of them.

**Why not fewer?** The filter runs on `sys_cpu_clk` (100 MHz) and takes one sample per clock
(II = 1). At 61.44 MS/s there is no free clock cycle in which a multiplier could serve a
second tap. Sharing multipliers (`SMI_PI_II=2` in `build_hls.tcl`) would halve the filter's
DSP48 count, but:
* at 100 MHz it only works up to 50 MS/s, so not with the 61.44 MS/s bands;
* moving the filter to a faster clock adds a new clock crossing for the 2048-bit weight bus,
  with new constraints and new methodology warnings;
* it does **not** lower the power: the same number of multiplications per second is done at a
  higher clock, plus the multiplexers. See section 3.

---

## 2. The 7 critical warnings: what they were and how they are fixed

All 7 were **TIMING-54: "Scoped false path, clock group or max delay datapath only
constraint between clocks"** ([AMD UG906](https://docs.amd.com/r/en-US/ug906-vivado-design-analysis/TIMING-54-Scoped-False-Path-Clock-Group-or-Max-Delay-Datapath-Only-Constraint-between-Clocks)).

**Why Vivado warns.** An IP's own XDC file is applied *scoped* to that IP instance. Clocks
are global objects, though. A constraint between two clocks written in a scoped file
therefore acts on every path between those clocks in the whole design, not just inside the
IP. Vivado reports this so that nobody is surprised. AMD's advice is to put such constraints
at the top level.

| # | Where (IP-scoped file) | Constraint | Fix in 4.9.13 |
|---|---|---|---|
| 1 | our `gnss_passthrough` IP (`Source/IP/gnss_passthrough/gnss_passthrough_constr.xdc`) | `set_clock_groups -asynchronous clk_fpga_0 / {rx_clk clk_div_sel_*}` (added in 4.9.9, duplicated in the project XDC) | removed from the IP file; the identical **unscoped** copy in `Source/XDC/gnss_passthrough_cdc.xdc` stays (it also keeps TIMING-6/7 away) |
| 2 | ADI `util_clkdiv` (`util_clkdiv_constr.xdc`) | `set_clock_groups -logically_exclusive` between `clk_div_sel_0_s` and `clk_div_sel_1_s` | removed from the vendor working copy; the **same** constraint is now unscoped in `gnss_passthrough_cdc.xdc` (section 5) |
| 3-5 | ADI `axi_dmac` of `axi_ad9361_adc_dma` (`axi_dmac_constr.ttcl`) | 3 x `set_max_delay -datapath_only -from <clock> -through <RAM> -to <clock>` (`eot_mem_src`, `burst_len_mem`, `eot_mem_dest`) | removed from the vendor working copy (see below) |
| 6-7 | ADI `axi_dmac` of `axi_ad9361_dac_dma` | 2 x the same (`burst_len_mem`, `eot_mem_dest`) | same |

**Why removing the 5 DMA constraints changes no timing.** Each of them runs between
`clk_fpga_0` and the `clk_div_sel_*` clocks. The project already declares those clocks
asynchronous with `set_clock_groups -asynchronous` (since 4.9.9). In Vivado a clock group takes
precedence over `set_max_delay`, so these five constraints had no effect on the timing
analysis of this design. Removing them only removes the warnings.

**How the vendor files are changed.** `Vendor/` is never edited (requirements 34, 71). The
new script `Vivado/Scripts/patch_vendor_constraints.tcl` edits the disposable working copy
`Build/VendorWork/hdl`:
* `build_all.sh` stage 1 runs it right after `cp -a`.
* `package_library_ip.tcl` also calls it before it packages `util_clkdiv` or `axi_dmac`
  (the non-`make` flow).
* It keeps each removed statement as a comment, after a `GNSS-CRPA 4.9.13 TIMING-54` marker.
* It is idempotent.
* It **fails, and writes nothing**, if the vendor file does not contain exactly the expected
  statements (1 in `util_clkdiv`, 3 in the `axi_dmac` template). A different ADI release
  therefore cannot silently bring the warnings back or remove the wrong line.
* It was tested on ADI `hdl_2023_r2`; your `axi_dmac_constr.ttcl` is byte-identical to it.

**Expected after the build:** `report_methodology` shows **0 critical warnings**.
You may still see **TIMING-24** ("Overridden max delay datapath only"). That is a *warning*,
not a critical warning: it marks the remaining one-sided DMA constraints that the
asynchronous clock group overrides. This is the same as in 4.9.9-4.9.12, and is the price of
that clock group. If anything else shows up, send the text of `report_methodology`.

---

## 3. Power and junction temperature

Your own Vivado power report (32-tap build) gives the breakdown:

| Part | Power |
|---|---:|
| PS7 (ARM + DDR) | 1.54 W (52 %) |
| I/O (AD9361 LVDS) | 0.40 W |
| Signals | 0.37 W |
| Logic | 0.31 W |
| Clocks | 0.18 W |
| Device static | 0.20 W |
| **DSP** | **0.15 W** |
| **Total** | **3.16 W → 61.5 °C** (11.5 °C/W) |

**The DSP48s are a small part of the heat.** Fewer DSP48s mainly frees resources; on its own it
does not cool the chip much. What 4.9.13 does for power:
1. **The removed `axi_ad9361` blocks** are the largest saving in this release. They ran at
   l_clk = 245.76 MHz all the time, even though they were never used: 24 multipliers, 8 CORDIC
   DDS tones and the I/Q-correction logic.
2. **Zero weights in bypass.** With no jammer (most of the time), the products and the adder
   tree of the filter now stay at 0 instead of toggling with every sample.
3. **2 DSP48 less** in the RX2 normalisation.

My estimate is **about 0.05-0.1 W less, i.e. roughly 0.5-1 °C**. This is not measured; check it
with `report_power` (Vivado) after the build.

**Bigger levers.** These are your decisions; none of them is in this release:

| Lever | Effect | Cost |
|---|---|---|
| **Heatsink / thermal pad** from the Zynq to the E310 case, or a small fan | usually the largest drop | mechanical work |
| **30.22 MS/s band** (`gnss_band=3`, 18 MHz) instead of 61.44 MS/s | halves the l_clk logic, the LVDS toggling and the filter's work | only GPS L1 / Galileo E1, no BeiDou B1I / GLONASS |
| ARM idle (WFI) instead of the busy-polling console loop | part of the 1.54 W PS7 | firmware rework (interrupt-driven console) |
| Lower ARM clock | part of the PS7 power | changes the PS7 configuration (requirement 4) |
| Disable AD9361 TX2 if it is not used | AD9361 power (same enclosure, not the FPGA junction) | firmware setting |

The firmware busy-polls the console all the time (`console_get_command` with an idle hook),
so fewer solves per second would **not** lower the ARM power by itself. That is why the solve
rate was not changed.

---

## 3b. TX2 is always at 89750 mdB (89.75 dB)

Only **TX1** feeds the GNSS receiver. TX2 is not used.

**The problem.** With `CONTROL[3]` (`ch1_copy`) = 0, `gnss_passthrough` fed TX2 with RX2's raw
samples, jammer included. Until 4.9.12 the band presets set TX2 to the same 10-30 dB attenuation as
TX1. On top of that, `ad9361_init` starts both TX outputs at 10 dB.

**The fix (firmware only).** TX2 is held at `GNSS_L1_TX2_ATTEN_MDB` = `GNSS_L1_TX_MAX_ATTEN_MDB` =
**89750 mdB**:

| Where | What it does |
|---|---|
| `main.c` | right after `ad9361_init` |
| `gnss_l1_set_tx_attenuation()` | on every TX1 change (safe start, `gnss_tx=`) |
| `gnss_band_levels()` | the bands set **TX1 only** |
| level lock | its TX2 target is always 89750, so if anything moves TX2 it is put back within 1 s |
| `tx2_attenuation=` | no longer changes TX2: it answers `tx2_attenuation=89750` |

`gnss_band?` now prints `TX1 nn dB`, and the band line says
`TX1 attenuation nn dB, TX2 89.75 dB (fixed, not used)`.

**Check on the board:** `tx2_attenuation?` must read **89750**.

---

## 3c. 4.9.14: TX2 carries the CRPA output and is muted

**Before (up to 4.9.13).** The original loopback fed TX2 from its own elastic buffer, filled with
**raw RX2** (`src1 = ch1_copy ? rd_data0 : rd_data1`, and the firmware never set `ch1_copy`). The
CRPA filter only ever processed the ch0 path to TX1.

**Now (`Source/HDL/gnss_passthrough.v`).**

```
TX2 source = ch1_raw ? raw RX2 : ch0 word        (ch0 word = the CRPA output = what TX1 carries)
TX2 data   = (mute | ~tx2_on) ? 0x0000 : (pass_en ? TX2 source : vendor DMA)
```

| CONTROL bit | Name | 0 (reset value) | 1 |
|---|---|---|---|
| [1] | `mute` | both channels live | **both** TX1 and TX2 = 0 (unchanged) |
| [3] | `ch1_raw` (was `ch1_copy`) | TX2 source = **CRPA output** | TX2 source = raw RX2 (diagnostics only) |
| [4] | `tx2_on` (was reserved) | **TX2 = 0x0000 (muted)** | TX2 carries its source |

Both reset values are the safe ones, so TX2 is silent from power-on, before the firmware runs. The
mute also covers the vendor DMA branch, so a DDR replay cannot reach TX2 either. When it is
un-muted, TX2 is read from the ch0 buffer on the same DAC strobe as TX1, so it is sample-for-sample
identical to TX1.

**Firmware.**

| Where | What it does |
|---|---|
| `main.c` | `gnss_pt_set_tx2(0)` right after TX2 is set to 89.75 dB: TX2 muted, source = CRPA (also covers a soft restart that kept the PL registers) |
| `gnss_pt_set_tx2()` | clears `ch1_raw` and sets or clears `tx2_on`. On a bitstream older than 4.9.14 (no TX2 mute) it selects the CRPA copy instead (old `ch1_copy = 1`) and prints a warning |
| `gnss_tx2?` | `gnss_tx2=0  (TX2 MUTED, digital zero; source CRPA output = TX1 samples; attenuation 89750 mdB)` |
| `gnss_tx2=1` / `gnss_tx2=0` | lets the CRPA copy out on TX2 (still at 89.75 dB) / mutes it again |
| `gnss_info` | decodes `CONTROL[3] ch1_raw` and `CONTROL[4] tx2_on` |

The TX2 attenuation stays at **89750 mdB** whatever `gnss_tx2=` says (section 3b). The usual
`CONTROL` value with anti-jam on is still **0x21**: `tx2_on = 0`, so TX2 is muted.

**PL resources (extra, compared with 4.9.13).**

| Resource | Extra | Why |
|---|---|---|
| DSP48 | **0** | no arithmetic |
| Block RAM | **0** | the copy reuses the ch0 buffer word that TX1 already reads |
| LUTRAM | **0** | the ch1 buffer is unchanged (16 RAM32M before and after) |
| Flip-flops | **+2** | the `CONTROL[4]` two-flop synchroniser, which was trimmed while the bit was unused |
| LUTs | **about 0** | one more input on the 32 TX2 output-mux bits. yosys `synth_xilinx` of `gnss_passthrough`: 2,098 → 2,029 LUTs, i.e. no measurable change (mapping noise) |

That is < 0.01 % of the XC7Z020 (53,200 LUTs, 106,400 FFs) and has no measurable effect on the
FPGA junction temperature. A muted TX2 stops the TX2 DAC data from toggling, which saves a little
AD9361 power. The AD9361 TX2 analogue chain stays powered, because switching the AD9361 to 1T would
change the `axi_ad9361` interface mode.

**Why not remove the raw RX2 buffer as well?** It would save about 50 LUTs and 50 FFs (≈ 0.1 %),
but its counters, FIFO level and STATUS bits (`gnss_status?`) would then read differently, and the
register map would change. It is kept as the `ch1_raw = 1` diagnostic.

**Check on the board:** `gnss_status?` shows version **4.2318**, and `gnss_tx2?` reads **gnss_tx2=0 (TX2 MUTED ...)**.

---

## 4. Why SMI power inversion, and not "normal" (LMS/NLMS) PI

**Both are the same power inversion.** They minimise the output power with the RX1 reference
tap fixed at 1, and both converge to the same Wiener solution `w = R^-1 s / (s^H R^-1 s)`.
The difference is only *how* `w` is found:
* **SMI** solves it directly (Cholesky on the ARM) from an 8192-sample capture (`AJ_NSAMP`, since 4.9.4).
* **NLMS** walks towards it one sample at a time inside the FPGA.

With STAP (14 taps + 2 image taps against a wideband jammer) the covariance has a very large
eigenvalue spread: the jammer is 40-60 dB above the noise. NLMS converges slowly in the weak
directions, and its step-size noise adds to the output. SMI does not care about the spread.

Measured on the same E310-like 7 + 7 scenario: 61.44 MS/s, 30 MHz barrage, 12-bit ADC,
9.52 cm spacing (`stap_pi_crpa/python`, my model). The table shows the residual jammer over
the output noise.

| J/N | Method | after 1 ms | after 12 ms |
|---|---|---:|---:|
| 43 dB | **SMI-PI** (my model: 4096-sample blocks) | **-10.5 dB** | **-10.4 dB** |
| 43 dB | NLMS-PI, mu = 0.05 | -0.4 dB | -5.6 dB (still converging) |
| 43 dB | NLMS-PI, mu = 0.01 | +7.3 dB | -1.2 dB |
| 61 dB | **SMI-PI** | **-16.4 dB** | **-16.2 dB** |
| 61 dB | NLMS-PI, mu = 0.05 | +14.1 dB | +8.8 dB (leaking) |
| 61 dB | NLMS-PI, mu = 0.01 | +21.8 dB | +14.6 dB (leaking) |

**NLMS would also cost more DSP48 and more heat.** Every tap needs a second complex multiply
in the PL for the weight update (`w += mu e x*`) at the full sample rate. That is about
6 DSP48 per tap, about 90 for 7 + 7 + 2, against 45 for the SMI filter, whose weights are
computed on the ARM. Your own 4.5 C simulation agrees: on the J/N 30 dB wideband test, the
one-weight form that NLMS-type PI converges to reached 14.5 dB, the space-time SMI form 43 dB.

---

## 5. C simulation (`Tests/run_csim.sh`, g++, your 20 scenarios, core + firmware solver)

**20 / 20 PASS for each of the three 4.9.14 builds**, including scenario 1, which is bit-exact over
24 000 samples (0 mismatches) with the real `g2`. Format: suppression, residual.

| Scenario | **7 + 7 (REF 4)** | 6 + 6 (REF 3) | 5 + 5 (REF 2) |
|---|---|---|---|
| 2: wideband J/N 30 dB | **37.2 dB, +0.46** | 36.7 dB, +0.50 | 35.5 dB, +0.65 |
| 5: J/N 30 dB + reflection | **20.7 dB, +5.63, loss 4.4 dB** | 20.4 dB, +5.96, loss 4.9 dB | 17.4 dB, +8.42, loss 14.2 dB |
| 6: J/N 50 dB + I/Q imbalance | **50.6 dB, +1.39** | 50.0 dB, +1.56 | 49.0 dB, +1.88 |
| 8: J/N 60 dB + I/Q imbalance | **51.7 dB, +5.45** | 51.0 dB, +5.93 | 49.8 dB, +6.77 |
| 11: barrage J/N 61 dB | **54.1 dB, +4.88** | 53.4 dB, +5.29 | 51.2 dB, +6.80 |
| 12: barrage J/N 51 dB | **53.0 dB, +0.93** | 52.4 dB, +1.04 | 50.3 dB, +1.64 |
| 13: barrage J/N 41 dB | **46.5 dB, +0.61** | 45.7 dB, +0.74 | 43.8 dB, +1.03 |

At J/N 61 dB the 12-bit ADC is the limit (+4.9 dB for 7 + 7 is mostly ADC noise, see the 4.5
README). Reflections arriving more than 2 samples after the reference are beyond all three
builds (scenario 5; 5 + 5 suffers most); the 16-tap build (4.9.9) handles those.

---

## 6. Build

1. Put the vendor tree at `Vendor/ADI_hdl_2023_r2` in this folder (as for 4.9.12; it is not
   in git here).
2. Rebuild **from stage 1**, so that the vendor copy is patched and the libraries, the HLS core
   (smi_pi 2.5), the custom IP and the block design are all rebuilt:
   ```bash
   ./Automation/Linux/build_all.sh --clean
   ```
   If this same folder already completed a 4.9.13 build (its `Build/` is still there), 4.9.14
   only needs the custom IP and everything after it:
   ```bash
   ./Automation/Linux/build_all.sh --from 4
   ```
3. Check the logs:
   - `vendor_patch.log`: `VENDOR_PATCH: PASS`, 1 statement moved out of `util_clkdiv` and 3 out of `axi_dmac`.
   - `smi.log`: `IP antsdr:gnss:smi_pi:2.5`, and the estimated clock ≤ 10 ns.
   - `report_methodology`: 0 critical warnings.
   - `utilization_impl.rpt`: about 49 DSP48.
4. `SD_Image/BOOT.BIN` is **not** included: the 4.9.12 file was a 5 + 5 bitstream. Build it
   with stage 9.
5. On the board:
   - `gnss_status?` must show version **4.2318** (4.9.14) and layout **0x07040c07**.
   - `gnss_report?` must show `mode: space-time 13 + 2 image taps`.
   - `tx2_attenuation?` must read **89750**.
   - `gnss_tx2?` must read **gnss_tx2=0** (TX2 muted, source = CRPA output).
   The firmware refuses to start the filter on a bitstream with another tap count.

**Firmware note:** the vendor demo commands `dds_*` no longer do anything, because the DDS is
not in the bitstream. TX uses only DAC data source 2 (DMA = gnss_passthrough) or 3 (zero).

---

## 7. What was verified here, and what was not

| Verified | Not verified (no Vivado, Vitis HLS or board here) |
|---|---|
| C simulation 20/20 for this 7 + 7 build (and for the other two; core + firmware solver, bit-exact scenario included) | Vitis HLS synthesis (II = 1 at 10 ns, DSP count) |
| `patch_vendor_constraints.tcl` on the ADI `hdl_2023_r2` files: exact diff, idempotent, refuses a mismatching file | Vivado `report_methodology`, `report_utilization`, `report_power`, timing |
| `package_library_ip.tcl` hook with tclsh; `build_all.sh` bash syntax | the BOOT.BIN and the board |
| HDL compiles (iverilog, Verilog-2005); `gnss_aj.c`, `gnss_aj_math.c`, `gnss_info.c`, `gnss_l1.c` pass a syntax check | |
| 4.9.14 TX2 routing: `Tests/rtl/run_tx2_rtl.sh` (iverilog, `gnss_passthrough.v` + a smi_pi stub), 10 CONTROL settings, `TX2_RTL: PASS`; the same test FAILS on the 4.9.13 HDL (TX2 = raw RX2) | Vivado utilisation of the TX2 change (yosys estimate only, see 3c) |
| 4.9.14 firmware (`gnss_passthrough.c`, `command.c`, `gnss_info.c`, `main.c`): syntax check, no new errors (only BSP-generated symbols missing, as before) | |
| `axi_ad9361` source (2023_r2): DMA (2) and zero (3) DAC sources bypass the DDS; the disabled blocks reset to pass-through | |

If Vivado reports something other than the expectations in sections 1-3, send me
`report_methodology`, `utilization_impl.rpt` and the build logs.
