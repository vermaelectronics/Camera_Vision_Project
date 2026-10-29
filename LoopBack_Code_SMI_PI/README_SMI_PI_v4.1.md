# SMI-PI v4.1: what I changed and why

> **Every radio value by hand (firmware 4.2.0).** `radio=` sets LO, sample rate,
> RF bandwidth, TX attenuation and RX gain in one line, e.g.
> `radio=1575.42 1575.42 30.72 30.72 70 40 18`; `radio?` shows them. LO and rate
> accept decimals and either MHz / MS/s or Hz; values outside the AD9361 range are
> refused without changing anything. `rx_lo_freq=`, `tx_lo_freq=`, `rx_samp_freq=`,
> `tx_samp_freq=`, `rx_rf_bandwidth=`, `tx_rf_bandwidth=` take the same units, and
> `tx_att_db=70.5` sets the attenuation in dB. The console line buffer is 128
> characters (it was 30). See `TESTING_PUTTY.md`. Not run on hardware.

Version 4.1.5 of `LoopBack_Code_SMI_PI`. It builds on v4.0 (the space-time
filter) and makes these changes to the FPGA, the firmware and the scripts.

> **Slower lock than v2.2 (firmware 4.1.5).** After `gnss_band=` (new LO,
> rate and bandwidth) the AD9361 TX quadrature / LO-leakage calibration is
> now run again too (the driver only redoes it for LO moves > 100 MHz). A
> stale one, and an RX DC offset, put a carrier on the LO frequency
> (1582.5 MHz, 7 MHz above GPS L1) into the GNSS receiver: a CW jammer for
> the receiver itself. The TX calibration's test tone goes out of TX1, so it
> runs only while TX1 is off (at boot, before `gnss_tx=1`). Lock time also
> depends on the level the receiver gets: compare C/N0 at
> `tx1_attenuation=70000/65000/60000` (TESTING_PUTTY.md, T1).

> **Stuck "JAMMED" after power-on, DC offset (firmware 4.1.4).** A board
> report showed JAMMED since 0.5 s with no jammer: triggered by the RX gain
> loop settling at start-up, then never released because no clean moment
> came to learn the noise floor; RX1 13 dB above RX2 and an image weight
> |c1| = 0.57 pointed at a large DC offset (LO leakage). Now: (1) after
> `gnss_band=` (LO and rate change) the AD9361 RF DC calibration is run
> again and BB/RF DC tracking kept on; (2) each channel's DC is removed
> before detection, power and coherence (and before the SMI statistics),
> and `gnss_report?` prints it on a `dc offset` line, marked LARGE when it
> is above the noise; (3) a gain cut in the first 2 s is not a jammer;
> (4) the common-background rule also works when no floor has been learned
> yet, and then learns it; (5) "intermittent" now needs 3 switch-ons
> within 20 s.

> **False "JAMMER ON" without a jammer (firmware 4.1.3).** The power
> detector called any rise of the RX power 6 dB over the learned floor a
> jammer, also when RX1 and RX2 did not see the same signal. An active
> antenna's own LNA noise (10-25 MHz wide around L1) that changes, a bias-T
> supply or a loose cable does exactly that, and the console filled with
> JAMMER ON / OFF every few seconds. Now: (1) a power rise only counts when
> some of it is common to RX1 and RX2 (coherence >= -9 dB); one jammer is
> one source, both antennas see it. (2) A rise with no coherence that stays
> 3 s re-learns the noise floor ("noise floor rose ... not a jammer").
> (3) A signal that keeps
> switching on and off is reported as one "intermittent" summary line every
> 30 s. Event lines wait while you type and `gnss_aj_events=0` silences the
> console completely (`gnss_report?` still lists every event).

> **Console and detection (firmware 4.1.2).** The board now prints **only
> jammer events** by default (JAMMER ON / NULLED / OFF); `gnss_aj_log=30`
> adds a status line every 30 s, and it is held back while you type and for
> 20 s after the last key. **Common background:** a signal that RX1 and RX2
> both carry all the time at about the noise level (both ports fed from one
> source: splitter, GNSS simulator, one antenna/LNA; or TX1 leaking back
> into RX) made the coherence detector report a "jammer" at J/N ~0 dB for
> ever. After 10 s of coherence-only detection with no rise in power and no
> RX gain cut, the firmware prints `NOT A JAMMER`, treats it as background
> (filter bypassed) and from then on needs 6 dB more coherence, +6 dB power
> or a gain cut to call a jammer. `gnss_report?` shows it on a `background`
> line. Limit: a real jammer that stays below +3 dB power J/N for 10 s is
> treated as background too.

- How to build it, step by step: **`BUILD_STEPS.md`**
- How to test it on PuTTY and what to send back: **`TESTING_PUTTY.md`**

**Status.** v4.1 has been checked in C simulation (the HLS testbench runs the
real filter with the firmware's own solver; 8 scenarios pass), in an RTL
simulation of `gnss_passthrough` (iverilog) and in a host build of the
firmware engine against a simulated radio. I could not run Vitis HLS
synthesis, Vivado or the board here. On your PC, check `SMI_PI_HLS: PASS`
(step 3) and `BUILD_TIMING: MET` (step 6) before you trust the bitstream.

---

## 1. Your requests and what I did

| You asked for | What v4.1 does |
|---|---|
| Strong jammer (very little suppression left), up to **50 dB** | **Image taps.** Two extra "conjugate" taps cancel the AD9361's I/Q mirror image of the jammer, which is what limits a 2-antenna null at 50 dB. Also an **output make-up gain** (up to +24 dB, done digitally) and **RX2 input normalisation**. C simulation, wideband jammer 50 dB above the noise: **55.8 dB suppression, residual +0.3 dB** (v4.0: 44.4 dB, residual +4.4 dB). At 60 dB: 56.4 dB. |
| Pipeline **on** with no jammer; weight calculation **only when the jammer is on** | **Gated adaptation.** With no jammer the filter is **bypassed**: TX1 = RX1 exactly, with the same 8-sample delay. The ARM only runs a cheap detector on each capture, which costs ¼ of a solve. When the detector fires, the ARM solves on every capture and switches the filter in. It goes back to bypass 0.5 s after the jammer is gone. Bypass and filter have the same delay, so switching never moves the satellites' code phase. |
| **RX1/RX2 normalisation** at the pipeline input | **`x2n = g2 · x2`** in the FPGA core, before the filter. `g2` is learned from the two noise floors while there is no jammer. If a jammer is already on at boot, `g2` comes from the jammer itself. Range ±12 dB in `g2`, plus the weights' own range. C simulation with RX2 **25 dB weaker** than RX1: **38.6 dB** suppression with normalisation, **11.7 dB** without. |
| Lock in **10–15 s** instead of ~35 s | Board side: `gnss_tx=1` now goes **straight to the working level** (70 dB attenuation). Before, it stayed at 89.75 dB, where the receiver sees almost nothing, until you stepped it down by hand. Boot diagnostics are skipped (~1.5 s), and the output with no jammer is the exact RX1 signal. Receiver side: a **cold** start can never be faster than ~30 s (physics, see §5). For 10–15 s the receiver must do a **hot/warm start**. §5 has the checklist. |
| **BeiDou not visible, fewer satellites** | **New RF profile.** The v4.0 profile (LO 1584.5 MHz, 50 MS/s) put BeiDou B1I (1561.098 MHz) at −23.4 MHz from the LO, 94 % of the way to the band edge. There the AD9361's decimation filter (RX) and interpolation filter (TX) cut it, twice, and B1I is already ~4.5 dB weaker than GPS. The upper GLONASS channels (+21 MHz) were at the edge too. The new default `gnss_band=1` is **LO 1582.5 MHz, 61.44 MS/s, 56 MHz**: B1I sits at −21.4 MHz, GLONASS k=+6 at +22.9 MHz, GPS/Galileo at −7.1 MHz, all inside the flat ±24.6 MHz. **`gnss_band?`** prints each system with OK/EDGE/OUTSIDE. The firmware also keeps the AD9361 **FIRs off**: the vendor code loads a band-pass FIR (0.15–0.25 fs) that removes BeiDou and most of GLONASS if a GUI or `rx_fir_en=1` switches it on. A **radio guard** restores the profile within 1 s if anything changes the LO, the rate or the FIRs behind its back (RX and TX on different LOs = no satellites at all). Check the receiver too: BeiDou is off by default in many receivers. |
| Values on PuTTY, minimum manual settings | Time-stamped events (`[AJ 12.345 s] *** JAMMER ON ***`, `NULLED in 16 ms`, `jammer OFF`). **`gnss_report?`** prints everything in one block for you to copy into a report or a prompt. **`gnss_aj_csv=500`** prints a CSV line every 500 ms. There is nothing to tune. |

---

## 2. RF profiles (`gnss_band=`)

| N | LO | Rate, bandwidth | Carries |
|---|---|---|---|
| **1 (default)** | **1582.5 MHz** | **61.44 MS/s, 56 MHz** | **GPS L1, Galileo E1, QZSS, SBAS, BeiDou B1I + B1C, GLONASS G1 (all 14 channels)** |
| 0 | 1575.42 MHz | 30.72 MS/s, 18 MHz | GPS, Galileo, QZSS, SBAS, BeiDou B1C (L1 only) |
| 2 | 1589.5 MHz | 50 MS/s, 40 MHz | GPS, Galileo, GLONASS (no BeiDou B1I) |
| 3 | 1568.25 MHz | 50 MS/s, 40 MHz | GPS, Galileo, BeiDou B1I + B1C (no GLONASS) |
| 4 | 1601.72 MHz | 20.48 MS/s, 16 MHz | GLONASS G1 only (all 14 channels, narrow, lowest load) |
| **5 (custom)** | **yours** | **yours** | **`gnss_band=5 <LO> <rate> [bw] [txAtt] [rxGain]`** |

Where each signal falls with the default profile (offset from the LO; the
flat part of the AD9361 filters is about ±0.4 × 61.44 = ±24.6 MHz):

```
 BeiDou B1I        GPS/Galileo/B1C          GLONASS G1 k=-7 ... +6
 1561.1            1575.42                  1598.06 ........ 1605.38 MHz
 -21.4 MHz         -7.1 MHz        LO       +15.6 .......... +22.9 MHz
 |<---------------------------- +-24.6 MHz flat ---------------------------->|
```

Profile 1 needs the bitstream to meet timing at **rx_clk 4 ns** (61.44 MS/s
in 2R2T), as v4.0 already did. If you had to relax it to 5 ns, use profile 2
or 3: at 50 MS/s no single LO carries both BeiDou B1I and all of GLONASS.

## 3. Architecture (v4.1)

```
            RX1 ─┐                      gnss_passthrough (l_clk = 4 x fs)
  AD9361         ├─► axi_ad9361 ─► ─ m_axis_smi ─► async FIFO ─► smi_pi_0 (sys_cpu_clk 100 MHz, II=1)
            RX2 ─┘                                               │  1. x2n = g2·x2      (AJ_NORM)
                                                                 │  2. 16 taps RX1 + 16 taps RX2n
                                                                 │     + 2 image taps (conj)
                                                                 │  3. bypass: y = x1[n-8]  (AJ_CFG[0])
                                                                 │  4. y <<= s (make-up, AJ_CFG[10:8])
            TX1 ◄── axi_ad9361 ◄── gnss_passthrough ◄─ async FIFO ◄─┘
                                        ▲ AJ_W, AJ_WC, AJ_CFG, AJ_NORM  (shadow → one commit)
  ARM (gnss_aj.c), from the console idle loop:
     DMA capture 4096 samples of RX1+RX2 ──► CLEAR : aj_detect()  (powers, RX1/RX2 coherence)
                                         └─► JAMMED: aj_compute() (exact 34x34 statistics,
                                                      Cholesky, loading = own noise estimate)
                                                      ──► weights + cfg + g2 ──► AJ_COMMIT
```

State machine (firmware):

```
   CLEAR (filter bypassed, TX1 = RX1)            JAMMED (filter in, solving every capture)
   - learns noise floor and g2                   - 31 space-time + 2 image weights
   - detect: coherence J/N >= -3 dB  ──2 captures──►   committed with make-up shift and g2
             or power J/N >= 6 dB                - "NULLED" when residual < 3 dB
             or RX gain cut >= 3 dB              - make-up +6 dB per 6 dB of gain cut (max +24),
                                                   only while the jammer is nulled
          ◄── quiet for 0.5 s (coherence < -6 dB, power J/N < 3 dB, no gain cut) ──
```

**Why coherence.** A jammer reaches both antennas, so RX1 and RX2 become
correlated. Receiver noise is independent in the two channels, and the
satellites are ~20–30 dB below the noise. With no jammer the coherence reads
about −15 dB. A jammer at 0 dB J/N reads 0 dB. The detector works without a
learned noise floor, so it also works when the jammer is already on at boot.

---

## 4. What changed, file by file

### FPGA

| File | Change |
|---|---|
| `Source/HLS/smi_pi/smi_pi.cpp`, `.h` | **Rewritten.** New interface `smi_pi(s_axis_x, m_axis_y, wts[1024], wcj[64], cfg[32], nrm[32])`. The pipeline normalises RX2 (`x2n = g2·x2`, Q3.13 × Q0 → Q2, 18 bits, cannot overflow), then runs 31 space-time taps and 2 image taps `c1·conj(x1[n-8]) + c2·conj(x2n[n-8])`. Then bypass (`cfg[0]`) and the make-up shift (`cfg[10:8]`, 0–4). The accumulator is Q14 in 48 bits, rounded to nearest after the shift and saturated to 16 bits. 99 + 4 multipliers. Still II=1 at 100 MHz. |
| `Source/HLS/smi_pi/testbench.cpp` | **Rewritten.** The bit-exact test now covers normalisation, image taps, bypass and the shift over all ranges. 8 scenarios, listed in §4. |
| `Source/HLS/smi_pi/build_hls.tcl` | Runs 8 C-sim scenarios. Packages IP **`antsdr:gnss:smi_pi:2.1`**. |
| `Source/HDL/gnss_passthrough.v` | VERSION **0x00040001** (4.1). New shadow registers **AJ_CFG 0x188** (reset `0x1` = bypass), **AJ_NORM 0x18C** (reset `0x2000` = 1.0), **AJ_WC0/1 0x190/0x194** (image taps), and **AJ_CFG_ACTIVE 0x198** (read-only). All of them are committed together with AJ_W by AJ_COMMIT in one clock edge. New ports `aj_wcj`, `aj_cfg`, `aj_nrm`. AJ_INFO is 0x10080C02. |
| `Source/BlockDesign/system_bd.tcl` | Wires `aj_wcj→wcj`, `aj_cfg→cfg`, `aj_nrm→nrm`. Requires smi_pi **2.1**, so a stale 2.0 from a v4.0 build is refused with a clear message. |

### Firmware (`Source/Firmware/app_gnss_e310`)

| File | Change |
|---|---|
| `gnss_aj_math.c/.h` | **Image taps.** 34×34 exact statistics: 64 extra non-conjugate sums. **g2.** The RX2 rows and columns are scaled by g2, so the weights are the ones the core needs. Each tap is loaded with its own noise (`rx2_noise`), which handles a residual RX2/RX1 noise difference beyond g2's range. The statistics are **averaged over 2 blocks** (smoother weights) and the average is reset on any gain step. New **`aj_detect()`**: powers and RX1/RX2 coherence at the best lag, about ¼ of a solve. Sums are accumulated in 32 bits per 128 samples, which is exact and about 2× faster on the A9. Modes: 0 = 31+2 taps (default), 1 = RX2+2, 2 = one weight (v3), 3 = 31 taps without image taps (v4.0). |
| `gnss_aj.c/.h` (bands) | New profile table (§2), FIRs forced off in `gnss_band_set`, `gnss_band?` shows each GNSS signal as OK/EDGE/OUTSIDE, **radio guard** in the idle loop (every 1 s), band line in `gnss_report?`. |
| `gnss_aj.c/.h` | **Rewritten engine.** CLEAR/JAMMED state machine (§2). Learns the noise floor and g2. Make-up shift, coordinated with the TX level hold. Time-stamped events, a 16-entry event history, `gnss_aj_report()` (`gnss_report?`), CSV output (`gnss_aj_csv=`) and `gnss_aj_note_tx()`. Refuses to start on a bitstream older than 4.1. |
| `gnss_passthrough.c/.h` | `gnss_pt_write_aj(re, im, cfg, g2)` writes 34 weights, cfg and g2, then commits. New register defines. The state print shows bypass/filter, the shift and g2. Expects version 4.1. |
| `gnss_smi_agc.c/.h` | The TX level hold subtracts the filter's digital make-up (`gnss_rxg_set_digital_db`), so the level is never compensated twice. New getters for the report. |
| `command.c/.h` | New **`gnss_report?`**, **`gnss_aj_csv=`**. `gnss_aj_mode=` now takes 0–3. **`gnss_tx=1`** passes through 89.75 dB and then goes straight to **`GNSS_TX_START_ATTEN_MDB`** (70 dB) before the data path opens. It also time-stamps TX ON/OFF. |
| `gnss_l1.h` | New `GNSS_TX_START_ATTEN_MDB` (default = `GNSS_DEPLOY_TX_ATTEN_MDB` = 70000). Override with `--defines GNSS_TX_START_ATTEN_MDB=65000`. |
| `main.c`, `gnss_l1.c` | The boot I/Q captures and the band-select probe run only with **`GNSS_BOOT_DIAG`** (saves ~1.5 s of boot). New banner. The deployment build time-stamps TX ON. |
| `gnss_info.c` | Menus and register map for v4.1. |

### Scripts and docs

| File | Change |
|---|---|
| `Automation/Linux/steps/07_sw.sh` | Passes options through, e.g. `./07_sw.sh --defines GNSS_DEPLOY_AUTO_TX`. |
| `Automation/Linux/steps/README.md`, `build_report.sh` | v4.1 names, 8 scenarios. |
| `Source/Config/board_e310_v1.json` | v4.1 processing and registers. The `rx_clk` constraint is corrected to 4 ns. |
| `README_SMI_PI_v4.1.md`, `BUILD_STEPS.md`, `TESTING_PUTTY.md` | **New** (this file and the two guides). |

### New registers (gnss_passthrough, base 0x43C00000)

| Offset | Name | Reset | Meaning |
|---|---|---|---|
| 0x188 | AJ_CFG | 0x00000001 | [0] bypass (TX1 = RX1 delayed 8 samples); [10:8] make-up shift 0–4 (6 dB each). Shadow. |
| 0x18C | AJ_NORM | 0x00002000 | g2 = {imag, real}, Q3.13 (0x2000 = 1.0). Shadow. |
| 0x190 / 0x194 | AJ_WC0 / AJ_WC1 | 0 | Image taps c1 (RX1), c2 (RX2n), weight-word format. Shadow. |
| 0x198 | AJ_CFG_ACTIVE | 0x00000001 | AJ_CFG as committed (read-only). |
| 0x180 | AJ_COMMIT | – | A write now commits AJ_W, AJ_WC, AJ_CFG and AJ_NORM together. |

---

## 5. Verification

**C simulation** (Source/HLS/smi_pi/testbench.cpp, the real filter + the
firmware solver, noise 20 LSB rms, satellite −20 dB):

| # | Scenario | Result |
|---|---|---|
| 1 | Bit-exact: random weights, image taps, g2, shift, bypass, full ranges, 24 000 samples | 0 mismatches |
| 2 | Wideband 30 dB | 43.0 dB suppression, residual +0.1 dB; one weight (v3): 14.5 dB |
| 3 | No jammer | bypass output = x1[n-8] exactly; coherence −15 dB (detector quiet) |
| 4 | CW 40 dB | 65.7 dB |
| 5 | Wideband 30 dB + reflection | 41.4 dB |
| 6 | **Wideband 50 dB + I/Q imbalance (49 dB image rejection)** | **55.8 dB, residual +0.3 dB**; without image taps 44.4 dB, residual +4.4 dB, satellite −3 dB |
| 7 | **RX2 25 dB weaker**, wideband 40 dB | **38.6 dB** with normalisation; **11.7 dB** without |
| 8 | Wideband 60 dB + I/Q imbalance | 56.4 dB, residual +2.6 dB |

**RTL simulation** (iverilog): new registers, commit atomicity (nothing
changes before AJ_COMMIT, everything at once after it), reset = bypass. The
v4.0 datapath checks are unchanged: no sample slips or underflows when the
core is switched in and out.

**Firmware on a simulated radio** (host build of `gnss_aj.c` +
`gnss_aj_math.c`):

```
[AJ 0.500 s] TX1 ON: the receiver's time to fix counts from here
[AJ 2.107 s] *** JAMMER ON ***  J/N +30.2 dB, WIDEBAND, fills the whole 50.0 MHz band (~49.1 MHz seen)
           detected because both antennas see the same signal (coherence); RX gain cut 6 dB
[AJ 2.107 s] NULLED in 2 ms: suppression 39.5 dB, residual 0.2 dB, make-up +6 dB
[AJ 5.001 s] JAMMED  J/N +30.2 dB  suppression 45.0 dB  residual 0.1 dB | WIDEBAND ... | RX 64/70 dB  make-up +6 dB  TX att 70.00 dB | 165 solves/s
[AJ 6.521 s] jammer OFF after 4.5 s (peak J/N +30.4 dB) -> filter bypassed, TX1 = RX1
[AJ 8.108 s] *** JAMMER ON ***  J/N +49.8 dB, WIDEBAND, fills the whole 50.0 MHz band (~49.2 MHz seen)
           detected because both antennas see the same signal (coherence); RX gain cut 24 dB
[AJ 8.108 s] NULLED in 68 ms: suppression 53.9 dB, residual 0.6 dB, make-up +24 dB
```

The host test also checks `gnss_band?` (all five signals OK with profile 1;
an enabled FIR is shown as wrong) and the radio guard. A simulated GUI that
moved the RX LO to 1575.42 MHz and switched the TX FIR on was caught and the
profile was restored within 1 s.

g2 was learned as +8.0 dB for a simulated RX2 that was 8 dB weaker. With the
jammer already on at boot, the engine took g2 from the jammer and nulled it
2 ms after the detection was confirmed.

**Not done here:** Vitis HLS synthesis (II and clock estimate), Vivado
implementation and timing, and the board itself.

---

## 6. Lock time (time to fix): what is possible

| Receiver start | What it needs | Typical time to fix |
|---|---|---|
| **Cold** (nothing stored) | find the satellites + download the ephemeris (sent in 30 s frames, needs ≥18 s of clean data per satellite) | **26–35 s**, for any receiver, with or without an anti-jam box |
| **Warm** (almanac/time/position, no ephemeris) | ephemeris download | 25–30 s |
| **Hot** (ephemeris less than 2–4 h old) | find + track | **1–5 s** (your earlier hot start: 4 s) |
| A-GNSS / AssistNow | ephemeris injected | 2–10 s |

So the **35 s you measure is what a cold start costs**. No change in the
AntSDR can make a cold start 10 s. What the board can do, and v4.1 does:

1. **No delay in the board.** Before, `gnss_tx=1` started at 89.75 dB
   attenuation, where the receiver sees at most one weak satellite, and
   then you typed `tx1_attenuation=70000`. Now TX1 comes up directly at
   70 dB. Boot is ~1.5 s shorter.
2. **No loss in the board.** With no jammer the output is RX1 itself, so the
   receiver gets the full C/N0. In v4.0 the weights were always running.
3. **No interruption when a jammer appears.** The null forms within
   milliseconds (2–70 ms in simulation, longer for stronger jammers because
   the gain loop must step down first). Bypass and filter have the same
   delay, so the receiver keeps tracking.

For **10–15 s** do this on the receiver side:

- **Keep the receiver powered** (or its backup battery/V_BAT connected)
  between tests, and do not send it a cold-start command. It then hot-starts
  in a few seconds every time the signal returns.
- If your GNSS viewer has start buttons, use **Hot start** (or Warm) for
  timing tests. Report which one you used.
- Make sure C/N0 is good: aim for **≥ 40 dB-Hz on the strongest
  satellites**. Weak C/N0 slows both acquisition and ephemeris decoding. Use an
  **active antenna (LNA) on each of RX1 and RX2, the same type, same cable
  length**, with bias-T power if the board does not supply it. Try
  `tx1_attenuation=65000` and `75000` and keep the one with the best C/N0.
- Measure from the `[AJ ...] TX1 ON` line (or from power-on for the
  SD-card build) to "3D fix" in the viewer.

---

## 7. Limits (what can still go wrong)

- **Two antennas = one null direction.** One jammer and its reflections are
  fine. A second jammer from another direction is not, and neither is a
  satellite in the jammer's direction.
- **ADC range.** At 60 dB J/N the RX gain is cut by ~35 dB, so the receiver
  noise is only ~0.5 LSB and quantisation starts to cost (simulated: 1.4 dB
  satellite loss). Above ~60 dB, add RF filtering or attenuation ahead of
  the board.
- **External LNA + 50 dB jammer:** check that the LNA output and the AD9361
  input do not compress (`gnss_report?` → `rx gain` should still be > ~10 dB
  and the residual ≤ 3 dB). Compression makes intermodulation that no
  linear filter can remove.
- **RX2 much weaker than RX1** (> 30 dB): g2 covers 12 dB and the weights
  another ~18 dB. Beyond that, fix the cabling.
- **BeiDou in the receiver.** The board now carries B1I (1561.098 MHz) and
  B1C (1575.42 MHz). The receiver must track them: switch BeiDou on in its
  configuration. u-blox M8, for example, has it off by default and tracks
  only 3 systems at once. Also check whether it uses B1I or B1C.
- **Solve rate.** On the ARM, a solve costs ~1.5× a v4.0 update (the image
  statistics). Expect ~100–150 solves/s while jammed, about 4× as many
  checks/s while clear. `gnss_report?` shows both.
- The detector thresholds (`AJ_ON_COH_DB −3`, `AJ_ON_DB 6`, `AJ_OFF_US
  0.5 s`) are constants at the top of `gnss_aj.c`. If your tests show false
  ON events or a missed weak jammer, send the `gnss_report?` output and they
  can be adjusted.

---

## 8. Upgrading from v4.0

The v4.1 firmware needs the v4.1 bitstream. On a v4.0 bitstream it says so and
does not start the anti-jam. Rebuild **all stages 3–9** (HLS core, passthrough
IP, project, bitstream, ELF, FSBL, BOOT.BIN). Unzip into a **new folder**
rather than over the v4.0 one, so no old `Build/ip_repo/smi_pi` 2.0 is
picked up. The block design refuses 2.0 anyway.
