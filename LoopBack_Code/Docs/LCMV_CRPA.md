# LCMV CRPA integration — gnss_passthrough v3.1 (covariance LCMV, automatic)

This change adds a 2-element robust LCMV anti-jamming beamformer to the
ANTSDR E310 V1 GNSS loopback pipeline. The v2.1 source was used only as the
integration reference (register/CDC/firmware conventions): **this build contains no
PL-NPI**. The `pl_npi_0` HLS core, its AXI-Stream ports, the HLS build stage and the
PL-NPI firmware/commands are all removed; the LCMV is the only canceller. It lives in the **PROCESSING CORE** section of
`Source/HDL/gnss_passthrough.v`, the insertion point reserved for the CRPA. Everything
else in the pipeline is unchanged: the radio datapath, AXI DMA capture path, elastic
buffers, output alignment, counters and CDC scheme.

**Algorithm:** R. Li, X. Zhao, X. W. Shi, "Derivative Constrained Robust LCMV
Beamforming Algorithm", *PIER C*, vol. 4, pp. 43–52, 2008, eq. (9).

---

## 1. What it does

```
                         ┌──────────────────────────── gnss_passthrough v3.0 ─────────────────────────────┐
 AD9361 RX1 ─► axi_ad9361 ┤ x1 ─┐                                                                         │
 AD9361 RX2 ─► axi_ad9361 ┤ x2 ─┤  crpa_lcmv_2ch                                     elastic   align     ├─► axi_ad9361 ─► TX1
                         │     ├─► y = conj(w1)·x1 + conj(w2)·(c·x2) ─ sat 12b ─► FIFO0 ─► [15:4] ─────┤
                         │     │        ▲                                                               │
                         │     │   w = C^-H [1 ε]^T, C = [a(θ_GPS) a(θ_JAM)]                           │
                         │     └─ RX2 (unprocessed) ───────────────────────────► FIFO1 ─► [15:4] ─────┼─► axi_ad9361 ─► TX2
                         └── AXI-Lite: angles θ_GPS, θ_JAM, ε, RX2 cal c  ◄── PS firmware / console ──┘
```

| CONTROL[4] lcmv_en | TX ch0 (TX1) | TX ch1 (TX2) |
|---|---|---|
| 0 (boot default) | RX1, identity | RX2 |
| 1 | **LCMV output**: gain 1 toward GPS angle, null toward jammer angle | RX2 |

TX1 switches to the LCMV output only after the first weight set is valid (STATUS[19]).

The jammer angle comes from software. **The FPGA does no DOA estimation.**

### LCMV with 2 antennas
Paper eq. (9) is: min wᴴRw subject to Sᴴw = 1 (look), wᴴBᵖA_J = 0 (derivative, widens the
null), and wᴴA_J = ε (enforcement, deepens the null).

* With M = 2 the **derivative row forces w₂ = 0** (it switches RX2 off), so it is omitted.
  Null widening needs M ≥ 4.
* With the look and enforcement rows, C is 2×2 and the paper's
  w = R⁻¹C(CᴴR⁻¹C)⁻¹f reduces exactly to **w = C⁻ᴴf**:
  `w2 = (1−ε)(s−j)/|s−j|²`, `w1 = 1 − conj(s)·w2`, with s = a_s(2), j = a_J(2).
  R cancels, so no covariance estimate or matrix inverse is needed.
* Steering: `a(θ) = [1, e^{jπ·sinθ}]`, spacing **λ/2 = 95.2 mm at L1**, θ from broadside.
  For **θ > 0 the wave reaches the RX2 antenna first.**

---

## 2. Files

### New
| File | Purpose |
|---|---|
| `Source/HDL/crpa/crpa_lcmv_2ch.v` | LCMV core: angles → steering → weights → beamformer, continuous atomic weight update |
| `Source/HDL/crpa/crpa_lcmv_weight_2ch.v` | Weight solver (eq. 9 look + enforcement) plus RX2 calibration, about 60 clk |
| `Source/HDL/crpa/crpa_sdiv_serial.v` | 48-bit serial divider, Q16.16 result |
| `Source/HDL/crpa/crpa_beamformer_2ch.v` | y = w^H x, 25×12 multiplies, 5-stage pipeline, **saturates to 12 bits** |
| `Source/HDL/crpa/steering_vector_2x2.v` | Your steering LUT `cos/sin(π·sinθ)`, unchanged |
| `Sim/tb_gnss_passthrough_lcmv.v`, `Sim/run_sim.sh` | System testbench of the whole block through AXI |
| `Sim/sim_results.txt` | Testbench output |
| `Tools/lcmv_rx2_cal.py` | Computes the RX2 calibration from a `gnss_capture` dump |
| `Docs/LCMV_CRPA.md` | This file |

### Modified (relative to the v2.1 reference source)
| File | Change |
|---|---|
| `Source/HDL/gnss_passthrough.v` | LCMV in the PROCESSING CORE; CONTROL[4]; registers 0x40/0x44/0x4C–0x6C; `lcmv_ctrl_meta` two-flop sync; STATUS[17], [19], [22]; VERSION 3.0. PL-NPI ports, gamma sync and stream logic removed |
| `Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl` | Packages `crpa/*.v` with a self-check; PL-NPI AXI-Stream interfaces removed |
| `Source/BlockDesign/system_bd.tcl` | `pl_npi_0` instance and its catalog check removed |
| `Source/XDC/gnss_passthrough_cdc.xdc` | Comment only (the `*ctrl_meta_reg*` pattern already covers the LCMV sync) |
| `Source/Firmware/app_gnss_e310/gnss_passthrough.[ch]` | LCMV registers and API, expected version 3.0; PL-NPI API removed |
| `Source/Firmware/app_gnss_e310/command.[ch]` | `gnss_lcmv?` `gnss_lcmv=` `gnss_lcmv_angles=` `gnss_lcmv_eps=` `gnss_lcmv_cal=`; `gnss_rx_gain?/=`, `gnss_rx_agc=` (matched manual gain); all `gnss_npi*` commands removed |
| `Source/Firmware/app_gnss_e310/gnss_info.c` | Menu, CONTROL page, register page, version text |
| `Source/Firmware/app_gnss_e310/main.c`, `console.c` | First console line `gnss: firmware started ...`; PL-NPI idle loop removed |
| `Source/Config/board_e310_v1.json` | Register map and processing description |
| `Automation/build_all.sh` | No HLS stage |

### Removed
`Source/HLS/pl_npi/`, `Source/HDL/pl_npi_reference/`, `Firmware/.../gnss_npi_agc.[ch]`.

---

## 3. Register map additions (base 0x43C00000)

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x0C | CONTROL | RW | **[4] lcmv_en** (other bits unchanged) |
| 0x10 | STATUS | RO | [17] lcmv_en (synced), **[19] lcmv_active** (LCMV drives TX1), [22] LCMV output clipped (sticky) |
| 0x40 | LCMV_GPS_ANGLE | RW | [15:0] signed deg −90..90, look direction (gain 1) |
| 0x44 | LCMV_JAM_ANGLE | RW | [15:0] signed deg −90..90, null direction |
| 0x48 | CRPA_COEF2 | RW | stored, unused (reserved) |
| 0x4C | LCMV_EPS | RW | [15:0] Q0.16 gain toward the jammer, 0 = ideal null |
| 0x50 | LCMV_RX2_CAL | RW | {im, re} **Q2.14**, reset 0x00004000 = 1.0 |
| 0x54–0x60 | LCMV_W1_RE/IM, W2_RE/IM | RO | Active weights, Q16.16 (w2 includes calibration) |
| 0x64 | LCMV_STATUS | RO | [0] en [1] weights_valid [2] busy [3] sat_sticky |
| 0x68 | LCMV_UPDATES | RO | Weight sets computed |
| 0x6C | LCMV_SAT_COUNT | RO | Output samples clipped to 12 bits (cleared by CONTROL[8]) |

The angle/eps/cal registers cross to `l_clk` through the `lcmv_ctrl_meta` two-flop synchroniser
plus a stability filter, and take effect within about 1 µs.
Readbacks lag by one status capture (256 `l_clk`).

---

## 4. Using it (UART console)

```
gnss_lcmv_angles=10 40       GPS look 10 deg, jammer null 40 deg
gnss_rx_gain=40              RX1 and RX2 at the same manual gain (keep the jammer below clipping)
gnss_lcmv=1                  LCMV on TX1 (if the AGC is on, locks RX1+RX2 to one manual gain)
gnss_tx=1                    open the TX path as before (attenuation interlock unchanged)
gnss_lcmv?                   angles, weights, update count, clipping
gnss_lcmv_eps=0.001          optional: paper's enforcement value (shallower null)
gnss_lcmv_cal=-1.1586 0.2043 RX2 calibration from Tools/lcmv_rx2_cal.py
gnss_lcmv=0                  back to identity passthrough on TX1
```

Numbers are space-separated and at most 9 characters each (console parser limit).

The LCMV is **off at boot**, so the deployment behaviour in `SD_Image/README_SD_CARD.md`
is unchanged until `gnss_lcmv=1`. All RF safety rules there still apply. LCMV changes what is
transmitted, not whether or how loudly.

### RX2 calibration (do this first)
The null lands on `jam_angle` only if RX1 and RX2 match in gain and phase. Note that
`main.c` sets `rx1rx2_phase_inversion_en = 1`. In simulation, a 0.85× / 190° RX2 error
turns a −45 dB null into +2.7 dB.

1. Split one CW tone (2-way splitter, equal cables) into RX1 and RX2, below full scale.
2. Capture with the existing `gnss_capture` path and pull the buffer as a binary file.
3. Run `python3 Tools/lcmv_rx2_cal.py capture.bin` and enter the printed values as `gnss_lcmv_cal=<re> <im>`.
4. Separately, match or measure the antenna-to-splitter cable lengths.

---

## 5. Build

One command builds everything, from sources to `Build/Boot/BOOT.BIN`, on Linux. **Vivado/Vitis 2023.2 is the default**
(ADI hdl_2023_r2 plus XSCT); `TOOL_VER=2026.1` selects the 2026.1 flow. See `Docs/BUILD_STEPS.md` for every step.

```bash
export XILINX_SETTINGS=/tools/Xilinx/Vitis/2023.2/settings64.sh   # your install path
./Automation/build_all.sh              # bench build
DEPLOY=1 ./Automation/build_all.sh     # SD deployment build (auto-TX at boot)
```

Logs are written to `Build/Logs/`. The manual steps the script performs are listed below.
The IP must be repackaged because the stale `Build/ip_repo/gnss_passthrough` does not contain the LCMV.

1. Package the IP: `vivado -mode batch -source Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl -tclargs <abs>/Source <abs>/Build/ip_repo`.
   Expect `IP_PACKAGE_SELFCHECK: OK - LCMV CRPA sources packaged with the IP`.
2. `Vivado/Scripts/create_project.tcl`, then `Vivado/Scripts/build.tcl`. Check `timing_impl.rpt`; the LCMV runs on `rx_clk` (8 ns).
3. `Vitis/Scripts/build_software.py` (firmware), then BOOT.BIN as before.
4. On the board, `gnss_status?` should report version **3.0**.

---

## 6. Verification

`Sim/run_sim.sh` (Icarus Verilog) tests the complete `gnss_passthrough` through AXI,
with 125 MHz `l_clk`, 100 MHz AXI clock, 2R2T ADC strobes and DAC strobes:

| Test | Result |
|---|---|
| T1 ID / VERSION / RX2_CAL reset | 0x47435031 / 0x00030000 / 0x00004000 |
| T2 LCMV on, GPS 10°, jammer 40° (JNR 46 dB) | jammer + noise residual **−45.3 dB** re RX1 (noise floor); GPS-direction tone gain **0.00 dB**; TX left-aligned |
| T3 TX2 with LCMV on | RX2 unprocessed, 0.00 dB |
| T4 LCMV off | identity, 0.00 dB |
| T5 RX2 error 0.85× / 190° | no cal: +2.7 dB (null lost); with `LCMV_RX2_CAL`: **−44.6 dB**, tone gain −0.01 dB |
| T6 LCMV off again | identity restored, 0.00 dB, no FIFO over/underflow |

Weights read over AXI (`w1 = 0.5 + 0.55098j`, `w2 = 0.71333 − 0.21156j`) match the
floating-point paper formula.

Yosys `synth_xilinx` (xc7) synthesises the full block. The LCMV adds about **20 DSP48E1** and
about 3k FFs. Vivado timing has **not** been run here, so check `timing_impl.rpt` after the build.

---

## 7. Limitations

1. **One jammer, one null, fixed width.** Two elements give one degree of freedom. Null
   widening (the paper's derivative constraints) needs at least 4 antennas.
2. **The null is exactly at `jam_angle`.** In integer degrees, a 1° error leaves about −34 dB at
   the real jammer. The angle source and RX2 calibration set the real-world depth.
3. **Look and null directions need about 3° or more separation**, or the GPS gain drops (guarded).
4. **ADC clipping cannot be nulled.** Watch `LCMV_SAT_COUNT` and keep the jammer below full scale.
5. The shipped `SD_Image/BOOT.BIN` predates v3.0 and must be rebuilt.

---

## v3.1: covariance LCMV (the paper's full formula) and automatic operation

### Where the covariance maths is
The paper's LCMV is `w = R⁻¹C (CᴴR⁻¹C)⁻¹f` with `R = E[x xᴴ]`. v3.1 computes it exactly that way:

| Step | Where | File |
|---|---|---|
| R11 = E\|x1\|², R22 = E\|x2\|², R12 = E[x1·x2*] over 4096-pair blocks | PL, every sample | `Source/HDL/crpa/crpa_cov_2ch.v` |
| Diagonal loading R + λmin·I (robust LCMV), eigenvalues → jammer detection (JNR) | ARM | `lcmv_math.c` |
| w = R⁻¹C (CᴴR⁻¹C)⁻¹f, about every 1 ms | ARM | `lcmv_math.c`, `gnss_lcmv_auto.c` |
| y = w1ᴴx1 + w2ᴴ(c·x2) on every sample | PL | `crpa_beamformer_2ch.v` |

Constraint sets:
* **AUTO (mode 1, default at boot):** C = [a_s], f = [1]. R places the null on the jammer by itself,
  **no jammer angle is needed**, and it follows a moving jammer.
* **ANGLE (mode 2):** C = [a_s, a_J], f = [1, ε], which is paper eq. (9) with look + enforcement constraints.
* With 2 antennas the paper's derivative constraint forces w2 = 0, so it is still omitted (needs ≥ 4 elements).
* When look and jammer are both specified, R cancels and the result equals the PL angle solver (verified: same weights).

### What runs automatically (from boot, `gnss_lcmv_auto.c`)
1. **RX link check:** at boot and every 2 s it checks that RX samples reach the PL. If not, it re-runs the AD9361 RX
   interface tuning (up to 3 times at boot, 5 times later).
2. **LCMV AUTO:** GPS look angle 0° (broadside), weights from the covariance about every 1 ms.
3. **Matched RX gain:** RX1 = RX2 always (keeps the RX2 calibration valid). Both are stepped together every 100 ms:
   −3 dB when the rms is above 700 LSB or the LCMV output clipped, +1 dB when it is below 180 LSB.

### Console (control panel)
| Command | Meaning |
|---|---|
| `gnss_tx=1` / `gnss_tx=0` | transmit on TX1 / abort |
| `gnss_lcmv=1` / `2` / `0` | AUTO (default) / known jammer angle / off |
| `gnss_lcmv_angles=G [J]` | GPS look angle [, jammer angle for mode 2] |
| `gnss_lcmv?` | mode, covariance R, eigenvalue JNR (jammer detected?), null direction, weights, gain |

Advanced (listed by `help?` only): `gnss_lcmv_eps=`, `gnss_lcmv_cal=`, `gnss_rx_gain=`, `gnss_rx_agc=`, `gnss_adc?`, `gnss_adc_tune=1`.

### Verification
* `Sim/firmware/test_lcmv_math.c` (host): AUTO finds the null at 40.00° without the angle; it follows the jammer to −25° (−92.7 dB);
  ANGLE matches the PL solver; ε = 0.01 gives −40 dB; no-jammer weights give full array gain.
* `Sim/run_sim.sh` T7/T8: the PS loop is modelled over AXI (read R, solve, write weights). Jammer 40°: −45.3 dB, tone 0.01 dB.
  Jammer moved to −25° with no setting changed: −46.8 dB, tone 0.01 dB.
