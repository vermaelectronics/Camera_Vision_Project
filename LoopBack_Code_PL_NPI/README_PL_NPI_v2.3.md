# PL-NPI v2.3: what I changed and why

Version 2.3.0 of `LoopBack_Code_PL_NPI`. It brings the same changes SMI-PI
got in v4.1 to the PL-NPI design: gated adaptation, RX2 normalisation, image
taps for 50 dB jammers, output make-up gain, the full GNSS band (BeiDou B1I
and GLONASS), FIRs kept off, a radio guard, faster TX start and the same
PuTTY reporting. PL-NPI stays a separate design: its weights still adapt in
the FPGA (NLMS power inversion). The ARM only supervises.

- How to build it, step by step: **`BUILD_STEPS.md`**
- How to test it on PuTTY and what to send back: **`TESTING_PUTTY.md`**

**Status.** v2.3 has been checked in C simulation (the HLS testbench runs the
real core; 8 scenarios pass, and they also pass with the fallback pipeline
settings), in an RTL simulation of `gnss_passthrough` (iverilog, including
all the new registers) and in a host build of the firmware engine against a
simulated radio and core. I could not run Vitis HLS synthesis, Vivado or the
board here. On your PC, check `PL_NPI_HLS: PASS` with **II 1** (step 3) and
`BUILD_TIMING: MET` (step 6) before you trust the bitstream.

---

## 1. Your requests and what I did

| You asked for | What v2.3 does |
|---|---|
| Strong jammer (very little suppression left), up to **50 dB** | **Image taps.** Two extra adaptive "conjugate" taps, `c1·conj(x1) + c2·conj(g2·x2)`, cancel the AD9361's I/Q mirror image of the jammer. That image limits a 2-antenna null at 50 dB, and no linear weight can remove it. C simulation, CW jammer 50 dB above the noise, 49 dB image rejection: **residual 3.35 dB without image taps, 0.17 dB with them**. There is also an **output make-up gain** (`y << s`, s = 0..4, up to +24 dB, done digitally before rounding) and **RX2 input normalisation**. The v2.2 constraint w1 = 1 is kept. |
| Pipeline **on** with no jammer; weight adaptation **only when the jammer is on** | **Gated adaptation in the FPGA.** The core has a power detector (\|x1\|² + \|x2\|², averaged over 64 samples). In mode AUTO it switches the filter in and the adaptation on **2 samples** after the input rises above the threshold. The firmware sets that threshold 6 dB above the noise floor and rescales it with every RX gain step. The core goes back to **bypass** (TX1 = RX1 exactly, weights held at 0) 0.5 s after the jammer is gone. With no jammer nothing adapts, and every new jammer starts from w = 0. A jammer **weaker** than the power threshold is found by the ARM from the RX1/RX2 **coherence** of its captures, and the ARM then forces the core ON. Bypass and filter go through the same pipeline, so switching never moves the satellites' code phase. |
| **RX1/RX2 normalisation** at the pipeline input | **`x2n = g2·x2`** in the core, before the filter, in signed Q3.13 (`NPI_NORM`). The firmware learns `g2 = sqrt(N1/N2)` from the two noise floors while there is no jammer. If a jammer is already on at boot, `g2` comes from the jammer itself. Range −18 to +12 dB. Why it matters for NLMS: the step is normalised by \|x1\|² + \|x2\|², so a weak RX2 adapts its weight far too slowly. C simulation, RX2 **20 dB weaker**, CW 31 dB: with g2 = 1 the residual **never** fell below 3 dB in 60 000 samples. With g2 = 4 it did after **3 600 samples** (59 µs) and ended at **0.14 dB**. |
| Lock in **10–15 s** instead of ~35 s | Board side: `gnss_tx=1` now goes **straight to the working level** (70 dB attenuation) instead of staying at 89.75 dB until you step it down by hand. Boot diagnostics are skipped (~1.5 s), and the output with no jammer is the exact RX1 signal. Receiver side: a **cold** start can never be faster than ~30 s (physics, see §6). For 10–15 s the receiver must do a **hot/warm start**. §6 has the checklist. |
| **BeiDou not visible, fewer satellites** (same as SMI-PI) | v2.2 ran the vendor profile: **1575.42 MHz, 30.72 MS/s, 18 MHz**. BeiDou B1I (1561.098 MHz, −14.3 MHz) and every GLONASS channel (+22.6 to +30 MHz) were **outside** the 18 MHz band. The PL-NPI core also could not go faster: it ran at II=2 on the AD9361 data clock, which tied it to 30.72 MS/s. v2.3 moves the core to **`sys_cpu_clk` (100 MHz) at II=1**, between two asynchronous AXI-Stream FIFOs, so it handles up to 100 MS/s. The AD9361 interface is now timed at **4 ns** (61.44 MS/s). The new default `gnss_band=1` is **LO 1582.5 MHz, 61.44 MS/s, 56 MHz**, which carries GPS, Galileo, QZSS, SBAS, BeiDou B1I + B1C and all of GLONASS. The firmware keeps the AD9361 **FIRs off** (the vendor FIR is a 0.15–0.25 fs band-pass). A **radio guard** restores the profile within 1 s if anything changes the LO, the rate or the FIRs. **`gnss_band?`** prints each system as OK/EDGE/OUTSIDE. |
| Values on PuTTY, minimum manual settings | The same as SMI-PI v4.1. Time-stamped events (`[AJ 2.104 s] *** JAMMER ON ***`, `NULLED 2 ms after detection`, `jammer OFF`). **`gnss_report?`** prints everything in one block, including the weights the FPGA is really using, read back. **`gnss_aj_csv=500`** prints a CSV line every 500 ms. There is nothing to tune. |

---

## 2. RF profiles (`gnss_band=`)

| N | LO | Rate, bandwidth | Carries |
|---|---|---|---|
| **1 (default)** | **1582.5 MHz** | **61.44 MS/s, 56 MHz** | **GPS L1, Galileo E1, QZSS, SBAS, BeiDou B1I + B1C, GLONASS G1 (all 14 channels)** |
| 0 | 1575.42 MHz | 30.72 MS/s, 18 MHz | GPS, Galileo, QZSS, SBAS, BeiDou B1C (L1 only, the v2.2 profile) |
| 2 | 1589.5 MHz | 50 MS/s, 40 MHz | GPS, Galileo, GLONASS (no BeiDou B1I) |
| 3 | 1568.25 MHz | 50 MS/s, 40 MHz | GPS, Galileo, BeiDou B1I + B1C (no GLONASS) |

```
 BeiDou B1I        GPS/Galileo/B1C          GLONASS G1 k=-7 ... +6
 1561.1            1575.42                  1598.06 ........ 1605.38 MHz
 -21.4 MHz         -7.1 MHz        LO       +15.6 .......... +22.9 MHz
 |<---------------------------- +-24.6 MHz flat ---------------------------->|
```

RX and TX are always set to the same LO, so the board's clock error cancels
between down- and up-conversion. Profile 1 needs `rx_clk` timed at 4 ns. If
you have to relax it to 5 ns, use profile 2 or 3.

---

## 3. Architecture (v2.3)

```
            RX1 ─┐                     gnss_passthrough (l_clk = 4 x fs)
  AD9361         ├─► axi_ad9361 ─► m_axis_npi ─► async FIFO ─► pl_npi_0 (sys_cpu_clk 100 MHz, II=1)
            RX2 ─┘                                               │ 1. x2n = g2·x2               (NPI_NORM)
                                                                 │ 2. s = x1 + w2·x2n
                                                                 │      + c1·conj(x1) + c2·conj(x2n)
                                                                 │ 3. NLMS update of w2, c1, c2 (w1 = 1),
                                                                 │    only while adapting
                                                                 │ 4. power detector → AUTO gating
                                                                 │ 5. y = active ? (s << shift) : x1
            TX1 ◄── axi_ad9361 ◄── gnss_passthrough ◄─ async FIFO ◄─┘
                         ▲ NPI_CFG, NPI_NORM, NPI_DET_THR, NPI_DET_HOLD  (shadow → one NPI_COMMIT)
                         ▼ NPI_SNAP → weights, detector power, status (read back)
  ARM (gnss_npi_aj.c), from the console idle loop:
     DMA capture 4096 samples of RX1+RX2 ──► npi_detect(): powers, RX1/RX2 coherence
                                         ──► NPI_SNAP: the weights the FPGA is using
                                         ──► npi_measure(): what they do to this capture
                                             (J/N, suppression, residual)
                                         ──► threshold, hold, g2, mode, make-up ──► NPI_COMMIT
```

Core modes (`NPI_CFG[1:0]`): **0 AUTO** (the core's own detector gates it,
the default), **1 BYPASS** (TX1 = RX1, weights 0), **2 ON** (forced: weak
jammer found by coherence, or `gnss_aj_mode=2/3`), **3 HOLD** (filter in,
weights frozen, `gnss_npi_freeze=1`).

State machine (firmware):

```
   CLEAR (core in AUTO, bypassed)                   JAMMED (filter in, adapting in the FPGA)
   - learns noise floor and g2                      - FPGA detector or forced ON
   - FPGA detector: input 6 dB above noise ──────►  - "NULLED" when the measured residual < 3 dB
   - ARM: coherence J/N >= -3 dB, or power          - make-up +6 dB per 6 dB of gain cut (max +24),
     J/N >= 6 dB, or RX gain cut >= 3 dB,             only while the jammer is nulled
     2 captures ──────────────────────────────────► - gamma k = 1/8 (deepest null)
          ◄── core back in bypass (0.5 s hold) / ARM quiet for 0.5 s ──
```

---

## 4. What changed, file by file

### FPGA

| File | Change |
|---|---|
| `Source/HLS/pl_npi/pl_npi.cpp`, `.h` | New interface `pl_npi(s_axis_x, m_axis_y, gamma, adapt_en, cfg, nrm, det_thr, det_hold, *npi_st, *wout[192], *det_pow)`. New: RX2 normalisation, image taps (4× smaller step, `PL_NPI_IMG_SHIFT`), power detector and modes AUTO/BYPASS/ON/HOLD, make-up shift, weight/status outputs. **II=1 at 10 ns** (was II=2 at 8 ns on l_clk). At II=1 every register in the weight loop costs one sample of delay, so the loop has 8 extra delay registers (`PL_NPI_EXTRA_DELAY`) and a quarter step (`PL_NPI_STEP_SHIFT=2`) to keep the delayed update stable. Fallback if HLS misses timing: `PL_NPI_EXTRA_DELAY=16 PL_NPI_STEP_SHIFT=3`, which also passes the C simulation. |
| `Source/HLS/pl_npi/testbench.cpp` | Scenarios 1, 2, 3, 4, 5, 7, 8, 9 (see §5). |
| `Source/HLS/pl_npi/build_hls.tcl` | 10 ns, II 1, 8 scenarios, IP **`antsdr:gnss:pl_npi:2.0`** (was 1.0). |
| `Source/HDL/gnss_passthrough.v` | VERSION **0x00020003** (2.3). Shadow registers NPI_CFG/NORM/DET_THR/DET_HOLD + NPI_COMMIT, NPI_INFO, NPI_CFG_ACTIVE, NPI_SNAP and the read-back registers (§4 table). gamma and the freeze bit now drive the core directly: it is on `sys_cpu_clk`, the same clock as AXI, so the old synchroniser is gone. STATUS[21:20] = gain band from the core status. |
| `Source/BlockDesign/system_bd.tcl` | `pl_npi_0` on `sys_cpu_clk` between two independent-clock `axis_data_fifo` (depth 64). Wires the new ports. Requires pl_npi **2.0**, so a stale 1.0 is refused with a clear message. |
| `Source/XDC/system.xdc` | `rx_clk` **4 ns** (was 8 ns, which only covers 30.72 MS/s). |
| `Source/XDC/gnss_passthrough_cdc.xdc`, `Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl` | Comments for the new clocking. |

### Firmware (`Source/Firmware/app_gnss_e310`)

| File | Change |
|---|---|
| `gnss_npi_det.c/.h` | **New.** `npi_detect()`: powers, RX1/RX2 coherence (lags −15..15), jammer J/N, bandwidth and centre. `npi_measure()`: applies the read-back FPGA weights to the same capture and gives output power, suppression and residual. |
| `gnss_npi_aj.c/.h` | **New engine.** DMA captures, detection, CLEAR/JAMMED state, noise-floor and g2 learning (g2 from the jammer if there was no clean period), detector threshold and hold, make-up shift, events, status line, CSV, `gnss_report?`. Also the RF profiles (`gnss_band=`), FIRs off and the radio guard, and the idle hook `gnss_idle`. Writes the core only when something changed, which is about 30 writes in a 10 s test. Refuses to start on a bitstream older than 2.3. |
| `gnss_passthrough.c/.h` | Register defines, `gnss_pt_npi_write()` (cfg, g2, threshold, hold + commit), `gnss_pt_npi_snapshot()`. Expects version 2.3. |
| `gnss_npi_agc.c/.h` | Gated mode: gamma k is always the jammer value 1/8, because the core only adapts with a jammer. Both channels are re-matched every 200 ms if anything else changed one. The clean TX reference is learned only with no jammer. Getters for the report. |
| `command.c/.h` | New `gnss_aj?`, `gnss_report?`, `gnss_aj_log=`, `gnss_aj_csv=`, `gnss_aj_mode=0..3`, `gnss_band?`/`=`. `gnss_npi=` and `gnss_npi_freeze=` now go through the engine. **`gnss_tx=1`** passes through 89.75 dB and then goes straight to `GNSS_TX_START_ATTEN_MDB` (70 dB) before the data path opens. |
| `gnss_l1.h`, `gnss_l1.c`, `main.c` | `GNSS_TX_START_ATTEN_MDB`. Boot captures and band probe only with `GNSS_BOOT_DIAG`. Band profile set after AD9361 init, then the engine starts. New banner. |
| `gnss_info.c` | Menus, register map, commands for v2.3. |

### Scripts and docs

| File | Change |
|---|---|
| `Automation/Linux/steps/03_npi.sh`, `07_sw.sh`, `README.md` | 10 ns / II 1 checks and the fallback. `07_sw.sh` passes options through (`--defines ...`). |
| `Automation/Linux/build_report.sh` | Reads the target clock and II from the HLS log (10 ns / II 1), and states rx_clk 4 ns. |
| `Source/Config/board_e310_v1.json` | rx_clk 4 ns, default RF profile, v2.3 registers. |
| `README_PL_NPI_v2.3.md`, `BUILD_STEPS.md`, `TESTING_PUTTY.md` | **New.** |

### New registers (gnss_passthrough, base 0x43C00000)

| Offset | Name | Reset | Meaning |
|---|---|---|---|
| 0x180 | NPI_COMMIT | – | Write: shadow CFG, NORM, DET_THR, DET_HOLD go to the core in one clock edge. Read: commit count. |
| 0x184 | NPI_INFO | 0x03020801 | {3 adaptive weights, step shift 2, extra delay 8, layout 1} |
| 0x188 | NPI_CFG | 0x00000005 | [1:0] mode (0 AUTO, 1 BYPASS, 2 ON, 3 HOLD); [2] image taps; [10:8] make-up shift 0..4. Shadow. |
| 0x18C | NPI_NORM | 0x00002000 | g2, {imag, real} Q3.13 (0x2000 = 1.0). Shadow. |
| 0x190 | NPI_DET_THR | 0xFFFFFFFF | Detector threshold on the averaged \|x1\|²+\|x2\|² [LSB²], 0xFFFFFFFF = never. Shadow. |
| 0x194 | NPI_DET_HOLD | 0 | Samples the filter stays in after the power drops below half the threshold. Shadow. |
| 0x198 | NPI_CFG_ACTIVE | 0x00000005 | NPI_CFG as committed. |
| 0x19C | NPI_SNAP | – | Write: latch weights, power and status. Read: live status [0] detector, [1] filter in, [3:2] gain band, [4] adapting, [31:16] detector events. |
| 0x1A0–0x1B4 | NPI_W0..5 | – | Latched w2 re/im, c1 re/im, c2 re/im, Q12.20. |
| 0x1B8 / 0x1BC | NPI_DET_POW / NPI_ST_LATCHED | – | Latched detector power / status. |

---

## 5. Verification

**C simulation** (`Source/HLS/pl_npi/testbench.cpp`, the real core):

| # | Scenario | Result |
|---|---|---|
| 1 | CW jammer 600 LSB, filter ON | null −44.9 dB, converged at sample 200 |
| 2 | No jammer, large gamma, 200 000 samples | output = RX1 level (0.0 dB) |
| 3 | BYPASS with a 1500 LSB jammer; AUTO with noise only | 0 samples differ from RX1, weights 0, detector never on |
| 4 | **Gating**: J/N 31 dB on at sample 5000, off at 40 000 | filter in at **+2 samples**, residual 0.04 dB; back to bypass after the hold, then exactly RX1 again |
| 5 | **RX2 20 dB weaker**, CW 31 dB | g2 = 1: never below 3 dB residual in 60 000 samples (7.4 dB); **g2 = 4: 3 600 samples, 0.14 dB** |
| 7 | **CW 50 dB + I/Q imbalance (49 dB image rejection)** | image taps off: residual 3.35 dB; **on: 0.17 dB** |
| 8 | Make-up shift 2 | +12.07 dB (expected +12.04); ignored in bypass |
| 9 | Wideband 30 dB, 0.15-sample RX1/RX2 path difference (informational) | ~17 dB suppression: the single-weight limit, see §7 |

All 8 also pass with `PL_NPI_EXTRA_DELAY=16 PL_NPI_STEP_SHIFT=3` (the timing
fallback) and with `PL_NPI_EXTRA_DELAY=4`.

**RTL simulation** (iverilog, `gnss_passthrough`): VERSION, NPI_INFO and the
reset values; nothing reaches the core before NPI_COMMIT and everything
arrives in one edge after it; the snapshot latches one coherent set; reset
restores bypass. Switching the core in and out of the path causes no sample
jumps or underflows.

**Firmware on a simulated radio and core** (host build of `gnss_npi_aj.c` +
`gnss_npi_det.c`; the simulated antennas have identical RX1/RX2 paths):

```
[AJ 2.104 s] *** JAMMER ON ***  J/N +30.2 dB, WIDEBAND, fills the whole 61.4 MHz band (~60.1 MHz seen)
           detected by the FPGA power detector (input 6 dB above the noise); RX gain cut 7 dB
[AJ 2.104 s] NULLED 2 ms after detection: suppression 39.8 dB, residual 0.2 dB, make-up +6 dB
[AJ 4.511 s] jammer OFF after 2.5 s (peak J/N +33.3 dB) -> filter bypassed, TX1 = RX1
[AJ 6.102 s] *** JAMMER ON ***  J/N +2.4 dB, WIDEBAND, ...
           detected by RX1/RX2 coherence (both antennas see the same signal); RX gain cut 0 dB
[AJ 6.102 s] NULLED 4 ms after detection: suppression 9.1 dB, residual 0.4 dB, make-up +0 dB
[AJ 8.604 s] *** JAMMER ON ***  J/N +49.8 dB, WIDEBAND, ...
           detected by the FPGA power detector (input 6 dB above the noise); RX gain cut 24 dB
[AJ 8.604 s] NULLED 4 ms after detection: suppression 55.0 dB, residual 0.5 dB, make-up +24 dB
```

g2 was learned as +6.0 dB for a simulated RX2 that was 6 dB weaker. With a
40 dB jammer already on at boot, g2 was taken from the jammer (+7.9 dB) and
the jammer was nulled 4 ms after detection. "NULLED x ms" is the first
capture that saw the null. The FPGA itself converges in microseconds.

**Not done here:** Vitis HLS synthesis (II and clock estimate), Vivado
implementation and timing, and the board itself.

---

## 6. Lock time (time to fix): what is possible

| Receiver start | What it needs | Typical time to fix |
|---|---|---|
| **Cold** (nothing stored) | find the satellites + download the ephemeris (30 s frames, ≥18 s of clean data per satellite) | **26–35 s**, for any receiver, with or without an anti-jam box |
| **Warm** (almanac/time/position, no ephemeris) | ephemeris download | 25–30 s |
| **Hot** (ephemeris less than 2–4 h old) | find + track | **1–5 s** |
| A-GNSS / AssistNow | ephemeris injected | 2–10 s |

No change in the AntSDR can make a cold start 10 s. What the board does:

1. **No delay in the board.** TX1 comes up directly at 70 dB, and boot is
   ~1.5 s shorter.
2. **No loss in the board.** With no jammer the output is RX1 itself,
   because the core is bypassed and its weights do not wander in noise.
3. **No interruption when a jammer appears.** The FPGA switches in within
   2 samples and adapts in microseconds. Bypass and filter have the same
   delay.

For **10–15 s** on the receiver side: keep it powered (or keep its backup
battery), use **hot/warm start** for timing tests, and get **≥ 40 dB-Hz** on
the strongest satellites. Use the same active antenna type and cable length
on RX1 and RX2, and try `tx1_attenuation=65000` / `75000`. Measure from the
`[AJ ...] TX1 ON` line to "3D fix".

---

## 7. Limits (what can still go wrong)

- **One spatial weight.** PL-NPI has one adaptive weight per antenna, not
  taps across time. A CW or narrowband jammer is nulled deeply. A
  **wideband** jammer is nulled only as far as the RX1 and RX2 paths match
  across the band. With a 0.15-sample difference (a few metres of cable, or
  different filters) the simulation gives ~17 dB. The host mock shows 40–55 dB
  only because its two paths are identical. If `gnss_report?` shows a
  wideband jammer with residual > 3 dB, the `WARNING residual` line appears:
  equalise the cables, or use **SMI-PI v4.1** (31 space-time taps), which is
  the design for wideband jammers.
- **Two antennas = one null direction.** One jammer is fine. A second jammer
  from another direction is not.
- **ADC range.** Above ~60 dB J/N the RX gain cut leaves the noise near 1 LSB.
  Add RF filtering or attenuation ahead of the board.
- **External LNA + 50 dB jammer:** check for compression (`rx gain` in
  `gnss_report?` still > ~10 dB, residual ≤ 3 dB).
- **RX2 much weaker than RX1** (> 30 dB): g2 covers +12 dB. Beyond that, fix
  the cabling.
- **BeiDou in the receiver:** switch it on in the receiver (often off by
  default), and check whether the receiver uses B1I or B1C.
- **HLS timing is the main risk of this version.** II=1 at 100 MHz with an
  NLMS loop is tight. If step 3 fails, use the documented fallback (more loop
  delay, smaller step). It is slower to converge, still microseconds.
- The thresholds (`AJ_ON_COH_DB −3`, `AJ_ON_DB 6`, `AJ_DET_MARGIN 4` (6 dB),
  `AJ_HOLD_US 0.5 s`) are constants at the top of `gnss_npi_aj.c`. Send the
  `gnss_report?` output if you see false ON events or a missed jammer.

---

## 8. Upgrading from v2.2

The v2.3 firmware needs the v2.3 bitstream. On an older bitstream it says so
and does not start the anti-jam. Rebuild **all stages 3–9**. Unzip into a
**new folder** so that no old `Build/ip_repo/pl_npi` 1.0 is picked up (the
block design refuses it anyway).

Commands that changed meaning: `gnss_npi=1/0` now switches the whole gated
engine. `gnss_aj_mode=2` or `3` gives "always adapting" (`3` is the closest
to v2.2). `gnss_npi_gamma=`, `gnss_npi_auto=` and `gnss_npi_k=` still work.
