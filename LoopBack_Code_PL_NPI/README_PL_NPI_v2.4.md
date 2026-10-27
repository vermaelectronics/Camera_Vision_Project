# PL-NPI v2.4: root causes, changes, results and limits

Version **2.4.0** of `LoopBack_Code_PL_NPI` (firmware `PL-NPI 2.4.0`, bitstream
VERSION `0x00020004`, HLS IP `antsdr:gnss:pl_npi:2.1`). The baseline is
`LoopBack_Code_PL_NPI_v2.3.5.zip`
(sha256 `e6ef7202a9166667cc96cd76833ef71cb2b2029e624f768289b42041aa9159d6`).
That zip is unchanged; in git it is commit `76eca0b`, plus `31e060d` (2.3.6,
TX quadrature calibration after `gnss_band=`).

The PL-NPI algorithm itself (piece-wise linear normalised power inversion,
NLMS weights adapted in the FPGA, gating detector, RX2 normalisation, image
taps) is **unchanged**. The 9-stage v2.3.1 pipeline is bit-identical: C-sim
scenarios 1–9 give the same results as v2.3.5.

> **What was verified, and how.** Everything in this document was verified
> in **simulation only**: HLS C simulation of the core, RTL simulation of
> `gnss_passthrough.v`, and host-compiled firmware against a register/DMA
> model. **Nothing was run on an E310, and Vivado / Vitis HLS were not
> available**, so HLS synthesis timing, implementation timing and the
> bitstream are **not verified**. The lock-time figures you reported (36 s
> before v2.3, 66–80 s with v2.3.x) are your hardware measurements. There
> is no "after" hardware figure yet; §7 explains how to measure it.

---

## 1. Summary

| Objective | Result in v2.4 | Verified by |
|---|---|---|
| `gnss_report` shows one consistent `JAMMER STATUS: ON/OFF` | Done: a separate status machine with thresholds, hysteresis and 4-capture confirmation; the report prints one snapshot | firmware mock 1–3, 7 |
| `UNKNOWN` only when there is no valid measurement | Done: shown when there has been no capture for >0.5 s, or anti-jam is off | mock 7 |
| Detection reported separately from filter activation | Done: `JAMMER STATUS` (detection) and `filter :` (PL-NPI in/out and why) are separate lines | mock 3, 4 |
| `gnss_report` without `?` | Done: `main.c` adds the `?` | code review |
| CW tolerance at J/S 30–40 dB | **Simulated:** 31.7 → 44.5 dB-Hz at J/S 40 (≈ no loss), using the new FPGA CW notch | HLS C-sim 11, 12 |
| RF configuration matches your settings | Done: the default profile is now 1570.25 MHz, 50 MS/s, 50/50 MHz, AD9361 slow-attack AGC starting at 40 dB | code review, mock |
| Lock time < 25 s | **Not achievable for a cold start** (ephemeris download, §6). v2.4 removes the board-side causes it could find. **Not measured on hardware.** | – |

---

## 2. Root causes found in v2.3.5

### 2.1 `gnss_report` alternating between JAMMER ON and OFF

From the v2.3.5 source (`gnss_npi_aj.c`, `poll()`):

```c
if(gated() && core_active && !fw_jam && res.jn_coh_db < AJ_PWR_COH_DB && cut_db() == 0)
        hw_veto = 1;
else if(... res.jn_coh_db >= AJ_PWR_COH_DB || ... !(snap.st & GNSS_PT_NPI_ST_DET))
        hw_veto = 0;
new_jam = gated() ? (core_active && !hw_veto) : fw_jam;
```

1. **No confirmation and no hysteresis.** In gated mode (the default), the
   jammer state copied the FPGA power detector (`core_active`), filtered
   by a veto recomputed **from one capture's coherence** every time. If the
   coherence sat near the −9 dB veto level, or the detector sat near its
   threshold, the state changed every few captures.
2. **The state followed the filter.** "Jammer" meant "the core has
   switched its filter in". A detector trip caused by DC/LO leakage
   (§2.3), or an incoherent noise rise, therefore showed as a jammer, and
   a real jammer that did not trip the FPGA detector did not.
3. **The report mixed two instants.** `gnss_report?` printed the
   engine's last state, then read live core registers (`live`,
   `NPI_CFG_ACTIVE`, the gains) at print time. The two halves of one
   report could disagree.

### 2.2 Slow lock (66–80 s, earlier 36 s)

These causes come from reading the code. Each one adds loss or
interruption to the signal the receiver gets. None of them was measured on
hardware.

| # | Cause (v2.3.5) | Effect on the receiver | v2.4 |
|---|---|---|---|
| 1 | **RF profile is not what you configured.** The firmware forced `gnss_band=1` (1582.5 MHz, 61.44 MS/s, 56 MHz) at start-up. Its radio guard put this profile back every second if anything else was set. Your 1570.25 MHz / 50 MS/s / 50 MHz never ran. | The profile is wider than needed (more noise and blockers into the ADC). Results do not match your setup. | New `gnss_band=4` **USER** profile, 1570.25 / 50 / 50, is the default. `gnss_band=1` still gives GLONASS. |
| 2 | **RX gain was not your AGC.** With `gnss_npi=1`, the default gain mode was the firmware's MATCHED manual loop. Its clean reference never started below **64 dB** (`GNSS_RXG_REF_MIN_DB 64`), instead of AD9361 slow-attack AGC at 40 dB. | Ahead of an active antenna, 64 dB can push strong out-of-band signals into ADC clipping. The TX level-hold also moved the TX attenuation with the RX gain. | Default `gnss_rx_agc=1`: AD9361 slow-attack AGC (gc mode 2) on both channels, starting from 40 dB. `gnss_rx_agc=2` brings the matched loop back. |
| 3 | **False / flapping detection switched the filter in and out.** DC from the RX LO leakage (tens to hundreds of LSB) counted as power for the FPGA detector. The image taps then adapted to cancel the DC. | Every switch in or out changes the TX1 waveform: a step in amplitude and phase on every satellite, which disturbs carrier tracking. The weights wandering in noise cost C/N0. | FPGA **DC blocker** in front of the detector, filter and TX (cfg[3]). The detector threshold no longer needs a DC term. The status machine (§3.2) is independent of the filter. |
| 4 | **TX quadrature / LO-leakage calibration was skipped at boot** (`tx_noted` starts at −1, and the old condition tested `== 0`). | A TX LO leakage carrier at 1570.25 MHz, 5.17 MHz from L1, goes into the receiver. | Fixed in 2.3.6 (`!= 1`) and kept. It runs after every `gnss_band=`. |
| 5 | **Cold start physics.** | A cold start must decode the ephemeris: 18–30 s of clean data per satellite after acquisition. | Cannot be changed on the board (§6). |

### 2.3 CW tolerance

PL-NPI forms **one spatial weight per antenna**. It adapts on total power
(signal + noise + jammer). A CW at J/S 30–40 dB is still **below** the
wideband noise in 50 MHz: J/N = J/S − 32 dB, with C/N0 45 dB-Hz. So the
power-inversion weights barely see it. The C-sim shows the CW is only
partly removed (J/S 40: 31.8 → 39.1 dB-Hz with PL-NPI ON). In the
correlator, however, the same CW is 10–20 dB **above** the despread noise.
The tool for this case is a notch at the tone frequency, not a spatial
null. v2.4 adds one.

---

## 3. What changed

### 3.1 FPGA (HLS core `pl_npi` 2.1, `gnss_passthrough` v2.4)

These are additions only. The PL-NPI datapath and its pipeline are
untouched.

- **DC blocker** (`NPI_CFG[3]`, on by default): `acc += x − (acc >> 14)`
  on each of the four inputs, before the detector, the filter and the
  bypass path. The corner is about 490 Hz at 50 MS/s. GNSS signals at
  ±5 MHz or more from the LO are not affected. C-sim 10: a DC of 185 LSB
  goes to < 1 LSB, noise changes by < 0.2 dB, and the result is
  bit-exact when off.
- **Two CW notches** on the TX1 output (`NPI_CFG[13:12]`, coefficients in
  `NPI_NCOEF0..5` = 0x1C0–0x1D4, applied by `NPI_COMMIT`):
  `H(z) = (1 − q z⁻¹)/(1 − p z⁻¹)`, `p = r·q`, `r = 1 − 2⁻⁸` (−3 dB
  width about 62 kHz at 50 MS/s). They are written as a look-ahead
  recursion (`s[n] = u[n] + p² s[n−2]`), so the loop at II=1 holds one
  complex multiply and one add. Q1.14 coefficients, one extra fraction bit
  in the state, 25-bit saturation. C-sim 11: ≥ 35 dB notch depth, < 0.3 dB
  noise change, bit-exact when off. They also work in bypass, so a CW is
  removed without switching PL-NPI in.
- `PL_NPI_NOTCHES` (default 2) can be set to 0 at HLS build time. That
  gives the v2.3 datapath back if timing fails (see §8). The firmware reads
  the number of notches from the core status (`st[7:5]`).
- `gnss_passthrough.v`: `CORE_VERSION 0x00020004`, `NPI_INFO 0x03020802`,
  `NPI_CFG` reset `0x0D` (AUTO + image taps + DC blocker). The notch shadow
  and active registers are copied on commit, read back and reset to 0.
  There is a new port `npi_ncoef[191:0]`.
- `system_bd.tcl` requires `pl_npi:2.1` and connects `npi_ncoef`.

### 3.2 Firmware (`Source/Firmware/app_gnss_e310`)

- **Jammer status machine** (`gnss_npi_aj.c`, `status_update()`), separate
  from the filter:
  - **ON** after **4 consecutive captures** with evidence. Evidence is any
    of: coherence ≥ −3 dB; power J/N ≥ 6 dB with coherence ≥ −9 dB; an RX
    gain cut; the FPGA detector with coherent inputs; or a confirmed CW.
  - **OFF** after **1.0 s** with every metric below the **lower** OFF
    thresholds (coherence < −6 dB, power J/N < 3 dB, no CW).
  - **UNKNOWN** when there has been no valid capture for 0.5 s (DMA
    stopped, radio being reprogrammed) or anti-jam is off.
- **CW detector**: a 4096-point Hann FFT of RX1 and RX2 every 4th capture.
  Spectra are averaged and median-normalised. A peak counts only if it is
  ≥ 18 dB (J/N ≈ −16 dB) in **both** antennas and is narrow (−10 dB within 3–4 bins). The
  DC ±3 bins and the band edges are excluded.
  - A candidate is **confirmed** after 3 hits in at least half of its
    scans. It is kept down to 14 dB and dropped after 1 s unseen. The
    strongest two (≥ 8 bins apart) are tracked.
  - Confirmed CWs tune notch 0/1 (frequency by parabolic interpolation),
    and the notch is re-programmed when the CW moves by more than 1 bin.
  - Unit test: 0 false CWs with no CW and at J/N −25 dB; 4/4 detected at
    J/N ≥ −17 dB; error < 3 kHz.
- **`gnss_report` / `gnss_report?`** print one snapshot that the engine took
  at the end of its last capture. Printing reads no register, latches no
  snapshot and runs no detector, so two reports in a row are identical
  apart from the snapshot age (mock scenario 3). The first line is
  `JAMMER STATUS: ON|OFF|UNKNOWN`, followed by:
  - since when and by which rule, and the snapshot age;
  - every detection metric next to its ON and OFF threshold;
  - the CW scan list;
  - the filter (in or out, and why), the core, the DC blocker and the
    notches;
  - jammer, suppression and weights;
  - a GNSS tracking line;
  - radio (LO, rate, RX and TX bandwidth);
  - TX and RX gain (both channels and the AGC mode);
  - engine counters and events.
- **GNSS tracking status** is not available to the board: the receiver's
  tracking state does not come back to the E310. The report says so
  instead of guessing. Read satellites, C/N0 and fix in the receiver.
- **RF defaults** (`GNSS_BAND_DEFAULT = GNSS_BAND_USER`,
  `GNSS_RXG_DEFAULT_MODE = GNSS_RXG_AD9361`, `GNSS_RXG_AGC_START_DB 40`).
  All three are `#ifndef`, so you can override them with `--defines`.
- New commands:
  - `gnss_aj_notch=1/0`: automatic CW notches on (default) or off.
  - `gnss_aj_dc=1/0`: FPGA DC blocker on (default) or off.
  - `gnss_report` without `?` also works.
- Captures taken across an AD9361 gain step are skipped, using the gain
  signature of both channels in AGC mode.
- The CW event names the tone: `*** JAMMER ON *** ... CW tone at RF
  1575.422 MHz (CW scan, below the wideband noise)`.

### 3.3 RF parameters, checked against the source

| Parameter | Your setting | v2.3.5 actual | v2.4 actual | Where |
|---|---|---|---|---|
| RX LO | 1570.25 MHz | 1582.5 MHz | **1570.25 MHz** | `bands[GNSS_BAND_USER]`, `gnss_npi_aj.c` |
| RX rate / RF BW | 50 / 50 MHz | 61.44 / 56 | **50 / 50** | same |
| RX gain | 40, AGC mode 2 (slow attack) | MATCHED manual, ≥ 64 dB | **slow attack on RX1 + RX2, start 40 dB** | `gnss_rxg_set_mode()`, `gnss_npi_agc.c` |
| TX LO | = RX LO | 1582.5 | **1570.25** (same profile) | `gnss_band_set()` |
| TX rate / RF BW | 50 / 50 | 61.44 / 56 | **50 / 50 requested** | same |
| TX attenuation | – | init 10 dB with TX idle (`main.c` default_init_param); `gnss_tx=1`: 89.75 dB first, then **70 dB** | unchanged | `gnss_l1.h` `GNSS_TX_START_ATTEN_MDB` / `GNSS_DEPLOY_TX_ATTEN_MDB` = 70000 |
| AD9361 FIRs | – | off | off | radio guard |

The AD9361 TX analog baseband filter tops out at 20 MHz (40 MHz RF)
(`ad9361.c`, clamp at line 2641). Even with 50 MHz requested, the TX
filter's −3 dB point is ±20 MHz. That still covers L1/E1 (+5.2 MHz) and
BeiDou B1I (−9.2 MHz). GLONASS G1 (+27.8 to +35.1 MHz from 1570.25) is
**outside** the USER profile; use `gnss_band=1` for it.

### 3.4 Tests and scripts

- `Tests/run_regression.sh [--full]`: HLS C-sim 1–12 (g++ with the Xilinx
  `ap_int` headers), RTL sim (Icarus), the firmware CW unit test, and the
  firmware engine mock with 8 scenarios. A tool that is missing gives
  SKIP, not PASS.
- `Tests/host/mock_engine.c`: the firmware engine (`gnss_npi_aj.c` +
  `gnss_npi_det.c`, unmodified) runs against a model of the core
  registers, the DMA and the AD9361 getters.
- `Tests/rtl/tb_gnss_passthrough.v`: register map, notch shadow/commit/reset,
  and the datapath.
- `Source/HLS/pl_npi/build_hls.tcl`: C-sim scenarios
  `1 2 3 4 5 7 8 9 10 11 12`, IP 2.1.

---

### 3.5 Strong CW that is TX1 feedback (firmware 2.4.2)

First hardware log (v2.4.0): `JAMMER STATUS: ON` with no jammer connected.
The cause was a strong tone (J/N +6 to +21 dB, frequency drifting by 100 kHz
over minutes) in both antennas. The board retransmits RX1 on TX1; with TX
attenuation 10 dB (set by the PC GUI, not by the firmware) and RX gain 70 dB,
TX1 leaks back into RX and makes a drifting tone, which the CW scan reports
correctly as a narrowband signal in both antennas. A weaker line at exactly
1560.000 MHz (39 x 40 MHz, a clock spur) also toggled the notch every second.

- Weak lines: a CW needs 18 dB in its FFT bin to be tracked (14 dB to be
  kept) and J/N >= -15 dB to count as a jammer.
- Strong CW with TX1 on: the firmware mutes TX1 once for 0.7 s. **Gone** = TX1
  feedback, status stays OFF (event and `cw source` line say so). **Still
  there** = outside source, JAMMER ON. The verdict is kept (feedback: 5 min;
  outside: until the CW is gone for 2 s), so TX1 is not muted repeatedly. TX1
  is released if captures stop or anti-jam is switched off.
- While a CW is being tested or is TX1 feedback, the coherence and power
  evidence (which the same tone causes) is ignored. A wideband jammer that
  arrives together with a feedback tone is therefore not reported until the
  tone is gone: fix the loop (TX attenuation 70 dB, lower RX gain).
- Mock scenarios 9-12 cover this (simulation only; 12 firmware mock scenarios in total).

---

## 4. Results

### 4.1 Simulated (this repository, `Tests/run_regression.sh --full`)

**HLS C-sim scenario 12.** GPS L1 C/A PRN 1 at C/N0 45 dB-Hz, CW at L1 +
2 kHz through different RX1/RX2 channels. The measurement is C/N0 after
the core, from 1 ms coherent correlations over 20 ms. The estimate varies
by about ±0.5 dB from run to run. With no jammer, raw RX1 gives 44.4 dB-Hz.

| J/S [dB] | raw RX1 | **v2.3.5 behaviour**: bypass | **v2.3.5 behaviour**: PL-NPI ON | v2.4 bypass + notch | v2.4 PL-NPI ON + notch |
|---:|---:|---:|---:|---:|---:|
| 10 | 45.1 | 44.9 | 44.1 | 44.2 | 44.9 |
| 15 | 44.6 | 44.8 | 44.1 | 44.1 | 45.1 |
| 30 | 38.4 | 39.0 | 39.3 | **44.5** | **44.2** |
| 40 | 31.7 | 31.8 | 39.1 | **44.5** | **43.6** |

(The "bypass" and "ON" columns use the v2.4 core with the notch off. This
is bit-identical to the v2.3.5 datapath: C-sim 1–9 unchanged.)

**Firmware mock** (the real `gnss_npi_aj.c`, simulated captures at about
405 per second):

| Scenario | Result |
|---|---|
| 1 wideband 30 dB, on 3 s / off 3 s, twice | exactly 2 ON + 2 OFF. ON within the first captures after onset (< 10 ms); NULLED 7 ms after detection; OFF about 1.4 s after the jammer stops (rule: 1.0 s with every metric below the OFF thresholds) |
| 2 coherent jammer wandering −8…+2 dB around the threshold, 20 s | 1 ON, 0 OFF (no alternation) |
| 3 `gnss_report` twice back to back | identical, no register write, no snapshot, no commit, one status line |
| 4 CW J/N −8 dB (J/S ≈ 24) | status ON 42 ms after onset; notch tuned within 0.3 kHz; PL-NPI stays in AUTO (not forced in); after the CW stops, notch off after ~1 s and status OFF after ~2 s |
| 5 CW J/N −27 dB (J/S ≈ 5) | stays OFF |
| 9 weak spur J/N −20 dB, flickering (field report: 1560.000 MHz) | no notch, no event, status OFF |
| 6 +10 dB noise bursts, independent in RX1/RX2 | never ON |
| 7 DMA stops for 2 s | UNKNOWN, then OFF |
| 8 CW J/S 10 / 15 / 30 / 40 | 10 and 15: OFF (J/N below −15 dB: listed, no C/N0 loss in table 4.1); 30, 40: ON with the notch, no alternation |

**RTL sim** `gnss_passthrough.v`: PASS. VERSION, INFO and cfg reset are
correct; notch registers shadow, commit, read back and reset; the datapath
has no jumps and no underflows.

### 4.2 Hardware

**None in v2.4.** Your earlier hardware observations (v2.3.x): lock 66–80 s
(36 s with v2.2); `gnss_report` alternating ON/OFF; console flooding (fixed
in 2.3.2/2.3.3). Use `TESTING_PUTTY.md` T2–T7 to measure v2.4.

---

## 5. Build

Every stage from 3 on must be rebuilt, because the HLS IP (2.1), the block
design and the bitstream (VERSION 2.4) changed. The v2.4 firmware also runs
on a v2.3 bitstream, without the DC blocker and notches. The report then
says `not in this bitstream`. See `BUILD_STEPS.md`.

```bash
./Automation/Linux/build_all.sh --jobs 16       # stages 1..9
Tests/run_regression.sh --full                   # host checks (needs $XILINX_HLS or HLS_INCLUDE for C-sim)
```

---

## 6. Lock time: what is possible

| Receiver start | What it needs | Typical time to fix |
|---|---|---|
| Cold (nothing stored) | acquisition + ephemeris (30 s subframe cycle, ≥ 18 s of clean data per satellite) | **26–35 s** on any receiver, with or without the AntSDR |
| Warm (almanac, time, position) | ephemeris | 25–30 s |
| Hot (ephemeris < 2–4 h) | acquisition + tracking | **1–5 s** |
| A-GNSS / AssistNow | ephemeris injected | 2–10 s |

**Under 25 s therefore needs a hot or warm start, or assistance.** The
board can only avoid adding to the receiver's own time. v2.4 removes the
board-side causes in §2.2: wrong profile, high fixed gain, filter
switching on DC, and the missing TX calibration. To check this, compare
time to fix through the AntSDR with the same receiver connected directly
(T2). If the AntSDR adds more than a few seconds for the same start type,
send the logs.

---

## 7. Measuring on hardware (summary of `TESTING_PUTTY.md`)

- **Conducted or shielded only.** Run jammer tests only on a conducted path
  (cables, attenuators, combiner) or in a shielded enclosure, or under a
  test authorisation. Never radiate a jammer or the TX1 output.
- **Calibrating J/S.** Measure the GNSS signal level at the RX input, or
  use the simulator's stated level. Set the jammer with the same combiner
  and losses. J/S = P_jam − P_sig (dB). The board's printed J/N relates to
  it through J/N ≈ J/S − 32 dB for C/N0 45 dB-Hz in 50 MHz. This is an
  assumption: check it against your receiver's C/N0.
- **Measure separately:**
  1. acquisition time (first satellite tracked);
  2. first navigation solution (3D fix);
  3. tracking continuity (losses of lock during jammer on/off);
  4. detection latency (`JAMMER ON` minus jammer switch-on, from the PuTTY
     timestamps);
  5. suppression (`gnss_report` `suppression`, and C/N0 drop in the
     receiver).
- **A/B tests:** `gnss_aj_notch=0/1`, `gnss_aj_dc=0/1`, `gnss_rx_agc=1/2`,
  `gnss_band=4/1`.

---

## 8. Limits and risks

- **HLS timing of the notches is not verified.** Each notch loop holds one
  complex 25×16 multiply and one add per clock (II=1, 10 ns). This is
  expected to fit, but it was not synthesised here. If step 3 reports a
  clock estimate > 10 ns in the notch, build with `PL_NPI_NOTCHES=1` (one
  notch) or `PL_NPI_NOTCHES=0` (environment variable for step 3, e.g.
  `PL_NPI_NOTCHES=0 ./Automation/Linux/steps/03_npi.sh`). `0` gives the v2.4
  core with the DC blocker and no notches. The C-sim passes in all three
  builds (checked here). The firmware reads the number of notches from the
  core, and the CW scan still reports CWs.
- **Vivado implementation timing** (step 6) was not run. Each notch has 12
  real multiplies (p·x, p²·s, q·s), so 2 notches take about 24 DSP48 more
  in the 100 MHz `sys_cpu_clk` domain. The xc7z020 has 220.
- **Independent AD9361 AGC on RX1 and RX2** (your mode 2) lets the two
  channel gains differ. The firmware re-normalises RX2 (g2) and skips
  captures across gain steps, but with a strong wideband jammer the
  matched loop (`gnss_rx_agc=2`) can null deeper. Compare both (T6).
- **Wideband jammers:** PL-NPI still has one spatial weight. The
  wideband null depends on how well the RX1 and RX2 paths match
  (README_PL_NPI_v2.3.md §7). SMI-PI (31 taps) is the design for that case.
- **A CW weaker than J/N −15 dB (J/S ≈ 17 dB) is not reported as a jammer** and is not notched. In the simulation it costs no C/N0. Such lines (e.g. clock spurs) appear only in the `cw scan` line of the report.
- **More than 2 CWs:** only the 2 strongest are notched.
- **Swept / chirp jammers** are not tracked by the notch. A scan runs every
  4th capture (about 10 ms at 400 captures/s), and a CW needs 3 hits to be
  confirmed, so anything that moves by more than a few bins (12 kHz each) in
  ~40 ms is not notched.
- **GLONASS** is outside the USER profile. Use `gnss_band=1` (1582.5 MHz,
  61.44 MS/s).
- `SD_Image/BOOT.BIN` is an **old image**. Use `Build/Output/BOOT.BIN`
  after the build.
