# SMI-PI 4.9.8: STAP with 7 taps per antenna (7 + 7), recreated from 4.9.7

Same design as 4.9.7 with **7 taps per antenna** (14 taps + 2 image taps, 16 weights) instead of 32.

| | 4.9.7 (32 taps) | 4.9.8 (7 + 7) |
|---|---|---|
| `SMI_NT` / `AJ_NT` | 32 | **7** |
| Weights | 66 | **16** |
| `SMI_NFAB` (LUT multipliers) | 12 | 0 (all on DSP48, about 45 DSP) |
| Reference tap (`SMI_REF` / `AJ_REF`) | 4 | 4 (1 was tested: the filter fails) |
| Echo reach (samples after the reference) | about 27 | **about 2** |
| `AJ_INFO` | 0x20040C07 | **0x07040C07** |
| `CORE_VERSION` | 0x00040900 | **0x00040908** |

## Simulation (C, `Tests/run_csim.sh` scenarios)

* **17 of 20 pass.** Failing: scenario 5 (wideband with a 3-sample reflection), 12 and 13 (J/N 51 and 41 dB with the 4.9 thresholds).
* Clean wideband 40 dB: 43.4 dB suppression, residual +1.0 dB (32 taps: 49.9 dB).
* Echo 1 sample: 53.7 dB, 1.7 dB satellite loss. Echo 3: 38.5 dB, 9.6 dB loss. Echo 5: 37 dB, 11 dB loss. Echo 8: 32.5 dB, 18.7 dB loss.
* **Reference at delay 1** (`SMI_REF = 1`): the filter does not null at all (about −3 dB suppression). Not used.

**Conclusion:** 7 + 7 is only useful where reflections are no longer than about 1 to 2 samples (16–32 ns, roughly 5–10 m of extra path). For a typical site with walls or a roof, use 16 taps (reach about 11 samples) or 32 taps (about 27 samples).

## Not verified

* **Not synthesised.** The HLS core, the IP packaging, the Vivado build and the board are untested for this version.
* **Firmware compiled only as far as the syntax checks** (the ARM build is not run here). The version checks match the bitstream: the firmware refuses to start the filter unless `AJ_INFO` shows 7 taps and the reference is 4.
* The HDL compiles with iverilog, strict Verilog-2005. The SFAP files are not part of this folder.

## Adaptive averaging (AJ_AV_ADAPT, in the firmware solver)

`AJ_AVG` is no longer fixed. After each solve the firmware adjusts it:
- **Weights jittering** (relative change between two solves above `AJ_JIT_HI = 0.15`): raise the factor by 0.05, so the weights are smoothed.
- **Stable weights but residual above `AJ_RES_TRACK = 6 dB`:** lower the factor by 0.05, so the filter tracks the jammer faster.
- **Otherwise:** drift back to 0.5.
- Limits 0.3 to 0.9. A sweep (chirp) still sets its own 0.98. `AJ_AV_ADAPT 0` restores the fixed 0.5.

Simulation: 17 of 20 scenarios pass, the same as the fixed version. The rule's thresholds are my first guess; they have not been tuned on hardware.

## Averaging factor by hand (console)

* `gnss_aj_avg=0.5`: sets the factor by hand (0.05 to 0.99). This turns the automatic adjustment off, and the value survives the firmware's own resets.
* `gnss_aj_avg=0`: automatic again (adapted from the solves, as above).
* `gnss_aj_avg?`: shows AUTO or MANUAL, the current factor, and the memory in solves.

A chirp (moving jammer) still uses its own 0.98 setting while it is seen. Checked: the setter, the clamp (0.01 becomes 0.05) and the reset behaviour in a small test; 17 of 20 C-sim scenarios with the default (automatic) setting.
