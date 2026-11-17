# SFAP-PI v6.2.1 — fixes on top of v6.2

**Firmware-only, like v6.2.** The FPGA core is unchanged, so the v6.0 BOOT.BIN
bitstream still works. Rebuild only the ARM application `app_sfap_lvds`.
Everything in `README_SFAP_v6.2.md` still applies.

## 1. Loading start when no noise floor has been learned (bug fix)

v6.2's fix #2 (start the diagonal loading at the bin's noise floor, not at
the jammer's level) only applied once a noise floor had been learned
(`noise_ref > 0`). With **the jammer already on at start-up**, or after a
`gnss_band=` change while jammed, there is no floor yet. The solver then
started at min(P1, P2), the jammer's level: the v6.0 fault again, in the
obstacle case that v6.2 targets.

Fix (`gnss_aj_math.c`, `bin_lmin`): with no learned floor, each bin's first
solve starts at the smallest generalised eigenvalue of its [X1, g2·X2]
covariance against the noise model diag(1, rx2n). For one jammer plus
noise, that value is the noise itself, whatever the jammer power. When a
floor is known, nothing changes.

C-sim, new rows in scenario 16 (jammer present at start-up, J/N 40 dB):

| RX2 shadowed | v6.2 | v6.2.1 |
|---|---|---|
| 0 dB | 51.4 dB supp., loss −1.3 dB | same |
| 12 dB | 42.3 dB, −1.0 dB | same |
| 18 dB | 35.2 dB, −0.6 dB | same |
| 24 dB | 24.9 dB, −0.2 dB | 26.1 dB, −0.4 dB |
| 30 dB | **6.3 dB, sat. loss 9.4 dB (FAIL)** | **15.4 dB, sat. loss 1.0 dB** |

## 2. The C-sim now runs the firmware's own g2 rule and averaging policy

In v6.2, scenarios 11, 12 and 14–18 used copies of the engine's rules written
inside `testbench.cpp`. Both rules now live in `gnss_aj_math.c`:

* `aj_g2_follow()`: g2 follows a jammer that RX2 sees weaker
* `aj_avg_policy()`: drift-aware averaging

`gnss_aj.c` and the testbench both call them. Their constants
(`AJ_G2_MAX/MIN`, `AJ_G2J_*`, `AJ_AVG_RISE_DB`, `AJ_AVG_TRACK_DB`) moved to
`gnss_aj_math.h`. The behaviour is identical: every line of the v6.2 C-sim
output is reproduced exactly. The rest of the engine's state machine
(jammer on/off, events, gain steps) still only runs on the board.

## 3. Verification

* C-sim: all 18 scenarios PASS (g++, open-source `ap_int`, the unchanged
  core, the real `gnss_aj_math.c`). See `test/csim_results_v6.2.1.txt`.
  The output is identical to v6.2 apart from the new rows above.
* `gnss_aj_math.c` compiles clean with `-Wall -Wextra`. `gnss_aj.c` passes a
  `-Wall -Wextra` syntax check against stub BSP headers. The only messages
  come from the empty stub `xparameters.h`.
* **Not done:** a Xilinx ARM toolchain build, and any run on the board.

## 4. Still open (unchanged from the v6.2 review)

* Measure `upd_rate` on the board first: the ~100 solves/s figure (and
  scenario 17's 8.8 ms) is estimated from x86 timing.
* The firmware's residual reads ~1–2.5 dB below the real output with the
  v6.2 framing. The "jammer leaking" warning and the make-up threshold
  (`AJ_SHIFT_RES_DB`) were tuned for v6.1.
* If null depth is short of 40 dB on the bench, try
  `-DAJ_NSAMP=4096 -DAJ_SHOP=128`: ~2× v6.1's update rate instead of ~3×,
  with less loss of null depth.
* The `AJ_G2J_PEAK` headroom is about 0.6 dB below the core's input
  saturation.
* The new start-up path is only tested against one wideband jammer. With
  two jammers in one bin the estimate starts higher; the loading iteration
  corrects it from the solver's own output.

## Files changed vs v6.2

`Firmware/app_sfap_lvds/`: `gnss_aj_math.c`, `gnss_aj_math.h`, `gnss_aj.c`,
`gnss_aj.h` (version 6.2.1). `HLS/sfap_pi/testbench.cpp`. See
`patches/v6.2_to_v6.2.1_*.patch`.
