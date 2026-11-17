# SFAP-PI v6.2 — ANTSDR E310, two-antenna GNSS anti-jam (256 bins)

**Firmware-only update.** The FPGA core is the v6.0 one (HLS files differ in
comments only), so the **existing v6.0 BOOT.BIN bitstream works**; rebuild
only the ARM application `app_sfap_lvds`. Contains everything in v6.1, plus
the v6.2 fixes for **obstacles, multipath and slow weight updates**.

## 1. The problem, measured (C-sim, real core + real solver, J/N 40 dB)

| What happened on the bench | v6.1 | Cause |
|---|---|---|
| Obstacle shadows the jammer at **RX2** by 24 dB | 7.5 dB suppression, satellites −14 dB | The RX2 weight would need \|b\| ≈ 16; Q3.12 stops at 8. A latent solver fault also started the loading at the jammer's level when RX2's noise weight is large. |
| Channel **moves** (obstacle / reflector moving), 100°/s | 21.9 dB, satellites −15 dB | One solve every ~25 ms; v6.1 also averaged 0.8 while nulled, so the weights lagged ~5 solves. **A 1° change after a solve already costs 20 dB of null.** |
| **Multipath**: jammer reflection from another direction, 0.5 µs late | 21 dB, satellites −16 dB | A 256-bin filter at 61.44 MS/s has 0.24 MHz bins; a 0.5 µs delay turns the reflection's phase 40° across one bin, so one weight per bin cannot null it. |

## 2. What v6.2 changes

| # | Change | File | Effect (C-sim) |
|---|---|---|---|
| 1 | **g2 follows the jammer** while jammed: if RX2 sees the jammer weaker (obstacle), g2 rises towards √(P1/P2), max 4.0, limited so g2 × RX2 peak never saturates the core, 3 dB hysteresis, back to the noise value when the jammer goes | `gnss_aj.c` | RX2 shadowed 24 dB: 11 → **24 dB** suppression, satellite loss 8 → **0.1 dB**; 30 dB: loss 14.7 → **1.4 dB** |
| 2 | **Loading starts at the known noise floor** (latent v6.0 fault: started at min(P1,P2) = the jammer level and could stop there when an RX2 tap has a large noise weight) | `gnss_aj_math.c` | needed for #1; RX2 shadowed 18 dB with g2 fixed: 19.6 → 33.8 dB |
| 3 | **~3× faster weight updates**: 4096-sample captures, solver frames without overlap (`AJ_NSAMP` 4096, `AJ_SHOP` 256) | `gnss_aj_math.h` | solve cost 0.37 vs 1.06 ms (x86); expected ~100 updates/s on the board instead of ~35–40 |
| 4 | **Drift-aware averaging**: statistics of this capture only while the residual rises (> 0.5 dB) or is above 1 dB, otherwise 0.5 | `gnss_aj.c` | with #3, channel turning at 100°/s: satellite loss 13.2 → **4.3 dB**; 30°/s: 4.8 → **0.9 dB** |
| 5 | **`gnss_band=5`**: 1575.42 MHz, 15.36 MS/s, 6 MHz (GPS C/A, Galileo E1, BeiDou B1C) — bins 60 kHz wide | `gnss_aj.c/.h`, `command.c` | 0.5 µs reflection: satellite loss 16.0 → **3.2 dB** |
| 6 | Weight hold 40 → **100 solves** (still ~1 s at the new rate) | `gnss_aj_math.h` | — |
| 7 | Exact satellite-loss measurement in every testbench scenario (v6.0 correlated the noisy output: ±0.5 dB scatter) | `testbench.cpp` | scenario 3 now reads 0.00 dB |

From v6.1 (still in): gating noise-floor fix, weight hold for pulsed/hopping
jammers (b_k only, checked every capture), per-bin narrowband detector,
`gnss_band=4` (8 MHz), `sfap-pi_hold=`.

## 3. C-simulation: all 18 scenarios PASS

| # | Test | Suppression | Sat. loss |
|---|---|---|---|
| 1 | Core vs. double model | 0.42 LSB rms, latency 766, bypass exact | — |
| 2 | Wideband 30 dB | 41.0 dB | −1.35 dB |
| 3 | No jammer | — | 0.00 dB |
| 4 | CW 40 dB | 63.1 dB | 0.23 dB |
| 5 | Wideband + reflection (3 samples) | 40.1 dB | −0.65 dB |
| 6 | 50 dB + I/Q imbalance | 56.8 dB | −0.95 dB |
| 7 | RX2 25 dB weaker (whole channel) | 38.6 dB | 1.13 dB |
| 8 | 60 dB + I/Q imbalance | 58.4 dB | 1.14 dB |
| 9 | Two CW, two directions | 63.0 dB | 2.43 dB |
| 10 | CW on the satellite, same direction | 59.6 dB | 4.63 dB |
| 11 | Intermittent (off 0.4 s, back) | v6.0: 0.0 → **52.0 dB** | −1.4 dB |
| 12 | Hopping, 16 channels | v6.0: 21.2 → **56.2 dB** | −1.6 dB |
| 13 | Weak CW detection | CW 10 dB under the noise detected (wide profiles); 0 false alarms / 300 | — |
| 14 | Steady, 12 captures | 51.0 dB (fixed 0.5: 51.1) | −1.3 dB |
| 15 | New jammer, other direction, same channel | 46.0 dB (held weights correctly not used) | 5.1 dB |
| 16 | Obstacle at RX2: 12 / 18 / 24 / 30 dB | 40.6 / 31.6 / 23.7 / 13.6 dB | −0.9 / −0.4 / 0.1 / 1.4 dB |
| 17 | Moving channel 30 / 100°/s | 38.4 / 32.5 dB | 0.9 / 4.3 dB |
| 18 | 0.5 µs reflection, 61.44 vs 15.36 MS/s | 20.9 vs 37.0 dB | 16.0 vs 3.2 dB |

Price of the faster updates: steady-state null depth is 1–3.5 dB less than
v6.1 (e.g. scenario 2: 44.5 → 41.0 dB). The residual stays at the noise floor,
so satellite losses are unchanged.

## 4. How to use it on the bench

* **Obstacles near the antennas**: nothing to set. `sfap-pi_report?` shows
  `g2 … RAISED for a jammer weaker at RX2` while it acts.
* **Multipath (buildings, metal nearby)**: if you need only L1 C/A, E1 and
  B1C, use `gnss_band=5` (15.36 MS/s), or `gnss_band=4` (30.72 MS/s, 8 MHz)
  as a middle way. Tolerated reflection delay, at about 37 dB suppression
  with a −6 dB reflection: ~0.13 µs at 61.44 MS/s, ~0.26 µs at 30.72,
  ~0.5 µs at 15.36. Filter latency at 15.36 MS/s is 50 µs, a common delay
  that the receiver's clock bias absorbs.
* **Update rate**: check `upd_rate` in `sfap-pi_report?`. v6.1 ran ~35–40
  per second; v6.2 should be ~100. The figure is estimated from x86 timing,
  not yet measured on the board.

## 5. Limits that remain

* **A moving channel faster than ~100°/s** (people or vehicles moving close
  to the antennas) still costs satellites: the residual grows with the
  square of (channel speed × update interval). The next step is an FPGA
  change (v7): accumulate the per-bin statistics **in the FPGA** from 100 %
  of the samples (the core already computes the bins), so the ARM only
  solves 256 small systems. That allows ~2 ms updates, ~4× closer again.
* **Multipath with long delays at 61.44 MS/s**: either a lower-rate profile
  (above) or more bins in the FPGA (512/1024, v7).
* Two antennas give one null per bin; a satellite in the jammer's direction
  and band is lost against a strong jammer.

## 6. Build and verification

Replace `Source/Firmware/app_sfap_lvds` with this package's folder and
rebuild the ARM application. `build_hls.tcl` now runs C-sim scenarios 1–18.

* **C-sim:** all 18 scenarios pass (g++, open-source Xilinx `ap_int`, the
  unchanged core, the real `gnss_aj_math.c`).
* **Firmware:** `gnss_aj_math.c` compiles clean with `-Wall -Wextra`;
  `gnss_aj.c`, `command.c` and `gnss_info.c` pass a syntax check against
  stub BSP headers.
* **Not done yet:** a build with the Xilinx ARM toolchain, any run on the
  board, and use of 15.36 MS/s with this HDL. The AD9361 supports that rate;
  the passthrough has only been used at 30.72–61.44 MS/s.
