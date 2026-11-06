# SMI-PI 4.9.13: 7 + 7 taps, fewer DSP48, no TIMING-54 critical warnings

Built from **4.9.12** (5 + 5 taps, level lock) with these changes:

| # | Change | Why |
|---|---|---|
| 1 | **7 taps per antenna, reference tap at delay 4** (as 4.9.10) | 5 + 5 leaked: about +6.8 dB residual at J/N 61 dB in C-sim, +16 dB on the board |
| 2 | **`axi_ad9361` DDS, I/Q correction and DC filter removed** (block design MOD-3) | about 24 DSP48 that this design never uses; they toggle at 245.76 MHz all the time |
| 3 | **RX2 gain `g2` is real** in the HLS core | the firmware always wrote 0 into the imaginary half, so 2 multipliers were wasted |
| 4 | **Zero weights while bypassed** (no jammer) | the filter multiplied every sample by the last jammer's weights for nothing |
| 5 | **All 7 TIMING-54 critical warnings removed at their source** | see section 2 |
| 6 | The mode text now shows the real tap count (`space-time 13 + 2 image taps`) | 4.9.12 printed "63 + 2" (the 32-tap layout) |
| 7 | HLS IP version **smi_pi 2.5**, `CORE_VERSION 0x0004090D` (prints 4.2317), `AJ_INFO 0x07040C07` | a stale 5 + 5 core or bitstream cannot be used by mistake |

The SMI power-inversion algorithm itself (block SMI, Cholesky on the ARM, image taps,
RX2 normalisation, level lock) is unchanged.

---

## 1. DSP48 budget

The numbers below are counted from the code: 3 multipliers per complex tap (Gauss form) and
one DSP48 per multiplier. The final number is in Vivado's `utilization_impl.rpt`.

| Block | 4.9.11 (6 + 6) | 4.9.12 (5 + 5) | **4.9.13 (7 + 7)** |
|---|---:|---:|---:|
| `smi_pi` taps: (RX1 NT-1 + RX2 NT + 2 image) x 3 | 39 | 33 | **45** |
| `smi_pi` RX2 normalisation `g2` | 4 | 4 | **2** |
| `axi_ad9361` DDS + I/Q correction + DC filter | ~24 | ~24 | **0** |
| `gnss_pulse_blank` (I^2 + Q^2) | 2 | 2 | 2 |
| **Total** | **~69** (you saw "almost 70") | ~63 | **~49** |

So 4.9.13 has **2 more taps per antenna and about 20 fewer DSP48** than 6 + 6.
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

## 4. Why SMI power inversion, and not "normal" (LMS/NLMS) PI

**Both are the same power inversion.** They minimise the output power with the RX1 reference
tap fixed at 1, and both converge to the same Wiener solution `w = R^-1 s / (s^H R^-1 s)`.
The difference is only *how* `w` is found:
* **SMI** solves it directly (Cholesky on the ARM) from a 4096-sample capture.
* **NLMS** walks towards it one sample at a time inside the FPGA.

With STAP (14 taps + 2 image taps against a wideband jammer) the covariance has a very large
eigenvalue spread: the jammer is 40-60 dB above the noise. NLMS converges slowly in the weak
directions, and its step-size noise adds to the output. SMI does not care about the spread.

Measured on the same E310-like 7 + 7 scenario: 61.44 MS/s, 30 MHz barrage, 12-bit ADC,
9.52 cm spacing (`stap_pi_crpa/python`, my model). The table shows the residual jammer over
the output noise.

| J/N | Method | after 1 ms | after 12 ms |
|---|---|---:|---:|
| 43 dB | **SMI-PI** (4096-sample blocks) | **-10.5 dB** | **-10.4 dB** |
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

**20 / 20 PASS**, including scenario 1, which is bit-exact over 24 000 samples (0 mismatches)
with the real `g2`.

| Scenario | 5 + 5 (4.9.12) | **7 + 7 (4.9.13)** |
|---|---|---|
| 2: wideband J/N 30 dB | 35.5 dB, residual +0.65 | **37.2 dB, +0.46** |
| 5: J/N 30 dB + reflection | 17.4 dB, residual +8.4, **satellite loss 14.2 dB** | **20.7 dB, +5.6, loss 4.4 dB** |
| 6: J/N 50 dB + I/Q imbalance | 49.0 dB, +1.88 | **50.6 dB, +1.39** |
| 8: J/N 60 dB + I/Q imbalance | 49.8 dB, +6.77 | **51.7 dB, +5.45** |
| 11: barrage J/N 61 dB | 51.2 dB, +6.80 | **54.1 dB, +4.88** |
| 12: barrage J/N 51 dB | 50.3 dB, +1.64 | **53.0 dB, +0.93** |
| 13: barrage J/N 41 dB | 43.8 dB, +1.03 | **46.5 dB, +0.61** |

At J/N 61 dB the 12-bit ADC is the limit (+4.9 dB is mostly ADC noise, see the 4.5 README).
Reflections arriving more than 2 samples late are still beyond 7 + 7 (scenario 5); the 16-tap
build (4.9.9) handles those.

---

## 6. Build

1. Put the vendor tree at `Vendor/ADI_hdl_2023_r2` in this folder (as for 4.9.12; it is not
   in git here).
2. Rebuild **from stage 1**, so that the vendor copy is patched and the libraries, the HLS core
   (smi_pi 2.5), the custom IP and the block design are all rebuilt:
   ```bash
   ./Automation/Linux/build_all.sh --clean
   ```
3. Check the logs:
   - `vendor_patch.log`: `VENDOR_PATCH: PASS`, 1 statement moved out of `util_clkdiv` and 3 out of `axi_dmac`.
   - `smi.log`: `IP antsdr:gnss:smi_pi:2.5`, and the estimated clock ≤ 10 ns.
   - `report_methodology`: 0 critical warnings.
   - `utilization_impl.rpt`: about 49 DSP48.
4. `SD_Image/BOOT.BIN` is **not** included: the 4.9.12 file was a 5 + 5 bitstream. Build it
   with stage 9.
5. On the board, `gnss_status?` must show version **4.2317**, layout **0x07040c07**, and
   `mode: space-time 13 + 2 image taps`. The firmware refuses to start the filter on a
   bitstream with another tap count.

**Firmware note:** the vendor demo commands `dds_*` no longer do anything, because the DDS is
not in the bitstream. TX uses only DAC data source 2 (DMA = gnss_passthrough) or 3 (zero).

---

## 7. What was verified here, and what was not

| Verified | Not verified (no Vivado, Vitis HLS or board here) |
|---|---|
| C simulation 20/20 (core + firmware solver, bit-exact scenario included) | Vitis HLS synthesis (II = 1 at 10 ns, DSP count) |
| `patch_vendor_constraints.tcl` on the ADI `hdl_2023_r2` files: exact diff, idempotent, refuses a mismatching file | Vivado `report_methodology`, `report_utilization`, `report_power`, timing |
| `package_library_ip.tcl` hook with tclsh; `build_all.sh` bash syntax | the BOOT.BIN and the board |
| HDL compiles (iverilog, Verilog-2005); `gnss_aj.c` passes a syntax check | |
| `axi_ad9361` source (2023_r2): DMA (2) and zero (3) DAC sources bypass the DDS; the disabled blocks reset to pass-through | |

If Vivado reports something other than the expectations in sections 1-3, send me
`report_methodology`, `utilization_impl.rpt` and the build logs.
