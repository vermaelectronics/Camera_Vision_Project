# PL-NPI v2.5: wideband jammers with cable mismatch, CW in front of the loop

Version **2.5.0** of `LoopBack_Code_PL_NPI` (firmware `PL-NPI 2.5.0`, bitstream
VERSION `0x00020005`, HLS IP `antsdr:gnss:pl_npi:2.2`). It builds on v2.4
(`README_PL_NPI_v2.4.md`: status machine, `gnss_report`, DC blocker, CW
notches); that document still applies. The PL-NPI loop itself (NLMS power
inversion, gating, normalisation, image taps) is **unchanged**.

> **What was verified, and how.** Everything below is **simulation**: C
> simulation of the FPGA core (the real fixed-point code), RTL simulation of
> `gnss_passthrough.v`, a floating-point system model, and the firmware
> engine compiled for a PC against a model of the board. **Vivado and Vitis
> HLS were not available and nothing ran on an E310.** Timing of the new
> stages (§6) and every hardware result are still to be measured.

---

## 1. What you asked for, and what I found

You tested with a jammer and the receiver did not lock on a cold start, with
"10 dB" of jammer, and asked for a PI algorithm that works from 10 to 30 dB,
for CW and wideband jammers, with all satellites at good strength and a lock
time within 35 s.

### 1.1 Search (papers, code)

I searched for open-source implementations and for the published methods.
**I found no public code for PL-NPI or for a two-antenna power-inversion FPGA
core.** What exists is literature, and it falls into three families:

| Family | What it handles | Used here |
|---|---|---|
| **Spatial power inversion** (the paper's PL-NPI) | jammers arriving from a different direction than the satellites; needs ≥ 2 antennas, one null | kept, unchanged |
| **Frequency-domain excision / notch filters** (single channel; Yousif and Blunt, *Eng* 2022: FFT excision on FPGA; a patent on FPGA notch filters at the interferer frequency) | CW and narrowband, independent of the array | notches (v2.4), moved in front of the loop (v2.5) |
| **Space-time adaptive processing (STAP)**: taps across time on each antenna (space-time and space-frequency combinations are described as the most effective for small arrays) | wideband jammers through unequal paths, and more jammers than antennas minus one | an RX2 equaliser with 3 taps (a reduced, fitted form of it) |

I read only the search summaries, not the full texts (some publisher hosts
are not reachable from this session). References are at the end.

### 1.2 What the simulations show about the v2.4 core

Two models were used: a floating-point model of 8 satellites and a jammer
(`Tests/sim/array_sim.py`), and the real fixed-point core with one satellite
(`Tests/run_cn0_table.sh`). J/N is over the 50 MHz band (the figure the board
prints); J/S = J/N + 32 dB for a satellite at C/N0 45 dB-Hz. Reference with no
jammer: 44.9 dB-Hz. "Path difference" is how much later RX2 sees the jammer
than RX1 (1 m of cable is about 5 ns = 0.25 sample at 50 MS/s).

1. **A CW jammer at J/N 10-30 dB is already fine in v2.4** (44 dB-Hz): the
   power-inversion weight nulls it or the notch removes it. If your "10 dB"
   was a CW, the algorithm was not what stopped the lock.
2. **A wideband jammer is where v2.4 breaks, and it breaks with cable
   mismatch.** One complex weight matches RX1 and RX2 at one frequency only.

   | wideband J/N | 0.15 sample | 0.5 sample (~2 m) | 1.0 sample (~4 m) |
   |---:|---:|---:|---:|
   | 10 dB | 42.7 | 39.8 | 35.2 |
   | 20 dB | 41.9 | 34.1 | 28.1 |
   | 30 dB | 36.1 | 26.9 | 23.7 |

   (v2.4, dB-Hz; raw RX1 is 34.3 / 27.4 / 24.2.) A cold start needs the
   ephemeris of each satellite decoded without gaps; at 24-30 dB-Hz I would not
   expect that within 35 s (my estimate, not measured).
3. **CW plus wideband** (two jammers, one spatial null) drops v2.4 to 29-38
   dB-Hz at J/N 30 with mismatch.
4. **If both RX ports see the same signal, no spatial algorithm can work.**
   With the satellites and the jammer arriving identically at both ports
   (a splitter after one antenna, or a GNSS simulator and jammer combined
   before a splitter), power inversion cancels everything common, satellites
   included: C/N0 fell to -40 ... -100 dB-Hz in the model, worse than doing
   nothing. The CW notch still works there (43 dB-Hz). A two-element array
   also loses the satellites whose direction is close to the jammer's.
   Section 5 gives a test for this.

So: v2.5 fixes 2 and 3 (verified in simulation). It cannot fix 4, and I do
not know from the logs which of these your failed 10 dB test was.

---

## 2. The final algorithm (v2.5)

```
ADC RX1,RX2 -> DC blocker -> CW notches (2, both channels) -> RX2 equaliser (3 taps)
            -> g2 normalisation -> PL-NPI loop (w2 + image taps, gated)  -> TX1
```

1. **DC blocker** (v2.4): removes the LO leakage before everything.
2. **CW notches in front of the loop** (new place): a CW never reaches the
   adaptive weights, so it neither uses up the single spatial null nor
   modulates noise around itself, and a CW plus a wideband jammer can both
   be removed. The zero is now Q1.15 (was Q1.14): with a strong CW the
   rounding of the zero limits the depth, and a strong CW at some offsets lost
   up to 6 dB with Q1.14. With one CW the firmware puts **both** notch units on
   it (depth squared).
3. **RX2 equaliser, 3 complex taps** (new): `e[n] = f0 x2[n+1] + f1 x2[n] +
   f2 x2[n-1]` maps RX2 onto RX1 for the jammer, across the band. It takes
   the cable and filter mismatch that one weight cannot.
4. **PL-NPI loop**: unchanged.

The firmware (`gnss_npi_aj.c`, `gnss_npi_det.c`) fits the taps: while a
jammer is present it takes the raw capture, notches any confirmed CW
(software, same notch), and solves a 3-tap least squares of RX1 on RX2 every
~0.5 s. The taps are switched on when they beat the single weight by 1.5 dB
(and the jammer is at least 3 dB above the noise), replaced when a new fit is
1.5 dB better, and dropped when the jammer is gone. With matched paths or a CW
alone the equaliser stays off (mock scenario 15). The measurement code
applies the same taps to its copy of RX2, so coherence, residual and the CW
scan see what the FPGA's loop sees.

---

## 3. What changed

**FPGA** (HLS core `pl_npi` 2.2, `gnss_passthrough` 2.5, block design)
- Pre-processing pipeline in front of the loop (R1 DC, R2 notch, equaliser
  with centre tap, products, sums, rounding): every stage one short path at
  II=1. Output latency 2 → 8 samples, the same for filter and bypass.
- New port `eqcoef` (96 bits) and registers `NPI_EQCOEF` 0x1D8-0x1E0,
  `NPI_CFG[14]` equaliser on (identity when off), status [9] equaliser,
  [10] notches in front of the loop. `NPI_INFO` 0x03020803.
- Compile-time switches (`build_hls.tcl`, environment variables):
  `PL_NPI_V25=0` gives the v2.4 datapath back; `PL_NPI_NOTCH_IN=0` notches on
  the output; `PL_NPI_EQ=0` no equaliser; `PL_NPI_NOTCHES=0/1/2`. All pass
  the C simulation (checked).

**Firmware**
- Equaliser fit and control (`eq_*` in `gnss_npi_aj.c`, `eq_fit` in
  `gnss_npi_det.c`), command `gnss_aj_eq=1/0`, report lines `rx2
  equaliser` and `notch placement`, event `EQUALISER`.
- Notch zero in Q1.15 (Q1.14 on a v2.4 bitstream), both notches on a single CW.
- It runs on a v2.4 bitstream too (no equaliser, notches on the output).

**Tests**: `Tests/run_regression.sh` (HLS C-sim 1-13, RTL, CW scan, firmware
mock 1-15) and `Tests/run_cn0_table.sh`; new C-sim scenario 13, mock scenarios
14 and 15; `Tests/sim/array_sim.py`.

---

## 4. Results (simulation only)

### 4.1 Real fixed-point core, one satellite, C/N0 [dB-Hz] (reference 44.9)

`Tests/run_cn0_table.sh`; raw = RX1 alone, v2.4 = previous core with its
notch, v2.5 = notch in front of the loop, "+eq" = with the equaliser.

| Jammer | J/N | raw | v2.4 | v2.5 + eq |
|---|---:|---:|---:|---:|
| CW +2 kHz | 10 / 20 / 30 | 30.6 / 24.9 / 19.2 | 44.2 / 44.2 / 43.8 | 44.2 / 43.9 / 43.6 |
| wideband, 0.15 sample | 10 / 20 / 30 | 34.3 / 27.4 / 24.2 | 42.7 / 41.9 / 36.1 | 44.1 / 44.1 / 41.0 |
| wideband, 0.5 sample | 10 / 20 / 30 | 34.3 / 27.4 / 24.2 | 39.8 / 34.1 / 26.9 | 45.2 / 43.3 / 36.3 |
| wideband, 1.0 sample | 10 / 20 / 30 | 34.3 / 27.4 / 24.2 | 35.2 / 28.1 / 23.7 | 45.5 / 46.1 / 46.0 |
| CW + wideband, 0.5 sample | 10 / 20 / 30 | 32.4 / 27.0 / 22.8 | 41.5 / 37.4 / 31.6 | 44.2 / 44.0 / 38.8 |
| CW + wideband, 1.0 sample | 10 / 20 / 30 | 32.4 / 27.0 / 22.8 | 39.0 / 32.8 / 29.0 | 44.3 / 45.4 / 45.3 |

The weakest remaining cases (wideband at J/N 30 with a *fractional* mismatch
of 0.3-0.5 sample: 36-38 dB-Hz) are the limit of 3 taps. An integer sample
of mismatch is removed completely.
The residual measurement in C-sim scenario 13 (J/N 30, nulling above the
output noise): one weight 13.1 / 23.7 / 29.5 dB, with the equaliser
5.9 / 12.7 / 0.3 dB for 0.15 / 0.5 / 1.0 sample.

### 4.2 Eight satellites, floating point (`Tests/sim/array_sim.py`)

Jammer 70° off, satellites at random angles, mean C/N0 over the 8 satellites
(ideal weights, 0.15 sample mismatch, 10% gain tilt): one weight, wideband
J/N 30: 37.2 (worst 32.3); with 3 temporal taps (STAP): 44.5 (worst 39.4).
Two CWs or CW + wideband at J/N 30: one weight loses the satellites (-5 / -3
mean), 3 taps give 44.5. The worst satellite is always the one whose
direction is close to the jammer's: a two-element array nulls one direction.

### 4.3 Firmware (mock of the board)

All 15 scenarios pass: v2.4 behaviours (status, report, CW, DMA loss, TX1
feedback test) unchanged; scenario 14: wideband J/N 30 with half a sample of
RX2 delay: the equaliser comes on, its taps map the jammer path onto RX1, and
it goes off when the jammer leaves; scenario 15: matched paths, wideband and
CW: the equaliser stays off.

### 4.4 Hardware

**None.** In particular, no lock time was measured with v2.5.

---

## 5. Why a cold start can still fail, and how to find out

The simulation covers the algorithm. A failed test on the board has other
possible causes. In order of how likely I think they are (not measured):

1. **Both RX ports see the same signal** (satellites and jammer). Test (jammer
   OFF, antennas as in the failing test): type `gnss_aj_mode=3` (filter always
   in and adapting) and watch the satellites for a minute; then
   `gnss_aj_mode=0`. With two separate antennas nothing happens (the weights
   stay near 0). If the satellites vanish, the two ports are common: no
   2-antenna spatial algorithm can work; use two real antennas (a few cm
   apart or more) and feed the jammer by a splitter or its own antenna.
2. **A jammer figure that is not J/N or J/S.** Tell me whether "10 dB" was the
   generator level, J/S at the receiver input, or the J/N the board printed.
   J/S 10 dB costs nothing even without the board (44.9 dB-Hz in the model).
3. **TX1 attenuation set by the PC GUI** (10-20 dB: a hot level into the
   receiver) and **GUI retunes** that interrupt the signal; use
   `tx1_attenuation=70000` and `gnss_band=4` and no GUI Set buttons.
4. **Cold start itself**: the ephemeris download takes 26-35 s with a clean
   signal. 35 s is possible only if the satellites stay well above the decoding
   threshold (I would plan for 33-35 dB-Hz, my estimate) through the whole download, with no loss of lock, and the board
   switches the filter in and out as few times as possible.

When you retest, send `gnss_report` during the jam and the receiver's
C/N0 per satellite (TESTING_PUTTY.md T3, T4, T8).

---

## 6. Build, resources, risks

Stages 3-9 must be rebuilt (HLS IP 2.2, block design, bitstream 2.5).
See `BUILD_STEPS.md`.

- **Not synthesised here.** Added logic: the notches on both channels (4
  notch units: 48 DSP48 instead of 24 for the two on the output in v2.4) and
  the equaliser (12 DSP48), so about 36 more DSP48 than v2.4 (the xc7z020 has
  220); all the new stages are registered, one multiply-add each, except the
  chain of two notches, which is the same path v2.4 already closed at 10 ns.
  If step 3 fails on timing, rebuild with `PL_NPI_V25=0` (v2.4 datapath,
  latency 2), or `PL_NPI_NOTCH_IN=0` (notches on the output, equaliser
  kept), or `PL_NPI_NOTCHES=1`.
- The firmware reads what is in the bitstream (`npi_st`, VERSION) and runs
  with any of these builds.

---

## 7. Limits

- Two antennas: **one** spatial null. A jammer from the direction of a
  satellite removes that satellite; a second jammer direction is not nulled
  (a CW is removed by the notches, a second wideband jammer is not).
- Three taps handle an integer-sample mismatch completely and a fractional one
  (0.3-0.5 sample) down to about 36-38 dB-Hz at J/N 30. A longer equaliser
  would do better at more DSP cost.
- The fit needs a jammer at least 3 dB above the noise at RX1 after the
  notches. A weaker wideband jammer costs little C/N0 and is left to the single
  weight.
- Swept and pulsed jammers are not followed (a CW needs about 40 ms to be
  confirmed; the equaliser is refitted every ~0.5 s).
- The equaliser changes RX2's noise gain (the sum of squared taps); the NLMS
  adapts around it, and the firmware's learned noise ratio is only updated
  while no jammer is present.

---

## References

- Jia, Ni, Luo, Zhang, Mao, "FPGA Implementation of Variable Step Power Inversion Array for BeiDou Receiver", IEEE Access vol. 11, 2023, DOI 10.1109/ACCESS.2023.3319394 (record: https://doaj.org/article/a4ebe69dea65403fac41211d6c659682).
- Yousif and Blunt, "Interference mitigation for GNSS receivers using FFT excision filtering implemented on an FPGA", Eng 2022, 3(4) (https://www.mdpi.com/2673-4117/3/4/32/html).
- Space-time processing with spatially distributed antenna arrays, NAVIGATION 70(3) (https://navi.ion.org/content/navi/70/3/navi.592.full.pdf).
- An SDR-based real-time testbed for GNSS adaptive array anti-jamming algorithms, Sensors 2016 (https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4813931/).
- US 6512803, GPS receiver capable of functioning in the presence of interference (digital notch filters at the interferer frequency, FPGA); US 12135378, combined space-frequency and space-time anti-jam processing (https://patents.google.com/patent/US12135378).
