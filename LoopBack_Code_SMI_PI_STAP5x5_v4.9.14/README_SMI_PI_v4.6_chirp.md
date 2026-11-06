# SMI-PI 4.6.0 (superseded by 4.7.0, see README_SMI_PI_v4.7_pulse.md): slow chirp (moving jammer) correction, and results for chirp, hop and pulsed jammers

**Firmware-only change.** The bitstream of 4.5 is reused; rebuild only the application and BOOT.BIN:

```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh; source /tools/Xilinx/Vitis/2023.2/settings64.sh
./Automation/Linux/build_all.sh --from sw --to boot            # add --defines GNSS_DEPLOY_AUTO_TX for the SD-card build
```

All numbers below are **C simulation** of the real core and solver (`Tests/run_csim.sh`, scenarios 1-14 and 16-18 pass); scenario 15 is the exploration
tool: `smi_pi_tb 15 <kind> <J/N dB> <k1> <k2> <blocks> <avg>`, kind 2 chirp (sweep of k2 x fs in k1 samples), 3 pulsed noise (period k1, on-fraction k2),
4 CW that hops by k2 x fs after the training; avg 0 = default memory, -1 = the firmware's sweep detector. 61.44 MS/s, 12-bit, I/Q imbalance, 0.15-sample path difference.
Not tested on the board.

## The problem

A jammer that is narrow inside one 4096-sample block (66 us) but moves from solve to solve gives a rank-1 statistic that is good at one frequency only. The
weights then null that frequency and nothing else (CW hop: 45 dB at 0.1 MHz away, 36 dB at 0.25 MHz, 26 dB at 0.5 MHz, none at 5 MHz). A chirp that takes 1 ms or
more per 49 MHz sweep is in that regime. (Fast chirps, up to about 0.33 ms per sweep, are seen as a wideband jammer and were already fine.)

## The correction (in `gnss_aj_math.c/.h`, `gnss_aj.c`)

* `aj_sweep_update()` watches the jammer's centre frequency from solve to solve. If the jammer is narrow in a block (bandwidth < 0.2 fs) and its centre moves by
  more than 0.004 fs (0.25 MHz at 61.44 MS/s) in a solve, a score goes up; two such moves switch on the **long memory**, about 30 quiet solves switch it off.
* While it is on, the solver averages the statistics with weight 0.98 (about 50 solves, 0.5 s) instead of 0.5 (`aj_result_t.avg`). A sweep that repeats inside that
  memory fills its whole range in the average, so the same 33 weights null it at every frequency, exactly as for a wideband jammer.
* Steady CW, barrage, fast chirps and pulses never switch it on (scenarios 17, 18; 0 of 30 blocks in the tables below).
* Console: `gnss_aj_sweep=1` (default) / `0`; `gnss_report?` shows `sweep : MOVING JAMMER (chirp): long memory ON` or `none seen`.

## Results (suppression of the jammer, satellite SNR loss against RX1 with no jammer; steady state, 30 or more blocks)

| Chirp, 49 MHz sweep | J/N 30 | J/N 50 |
|---|---|---|
| 1 us / 10 us / 100 us per sweep | 49 / 44 / 42 dB, loss -0.9 | 59 / 60 / 60 dB, loss -0.8 |
| 0.2 ms - 0.33 ms | 37-39 dB, loss -0.6 / -0.9 | 49-59 dB, loss 0.7 / -0.9 |
| **1 ms, detector OFF** | **5 dB, loss 16** | **28 dB, loss 19** |
| **1 ms, detector ON** | **43 dB, loss -1.5** | **62 dB, loss -1.6** |
| **5 ms, detector OFF** | **0 dB, loss 28** | **0 dB, loss 34** |
| **5 ms, detector ON** | **32 dB, loss -0.2** | **55 dB, loss -1.0** |

A first version of this table (3 blocks from a cold start) wrongly showed sweeps of 0.26-0.33 ms failing; in steady state they work without the detector.

Other jammers (steady state, detector on, J/N 40 unless stated):

| Jammer | Suppression | Satellite loss |
|---|---|---|
| Steady CW (J/N 50) | 71 dB, detector stays off | 0.6 dB |
| Barrage, wideband | 47 dB, detector stays off | -1.0 dB |
| Pulses every 100 us, 50 % / 10 % / 1 % on (J/N 30) | 38 / 29 / 12 dB | -1.0 / -0.6 / 1.3 dB |
| Pulses every 100 us, 50 % / 10 % / 1 % on (J/N 50) | 48 / 43 / 28 dB | 0.2 / 0.6 / 1.4 dB |
| Pulses every 8 us, 50 % / 10 % / 2 % on | 36 / 29 / 21 dB | 1.1 / 1.0 / 1.7 dB |

## What the correction cannot do

* **A single step of frequency.** A CW that jumps 0.5 MHz or more to a new frequency is not nulled until the next solve (physics: the jammer was never seen there). Hopping CW: 26 dB at 0.5 MHz, 9 dB at 1.8 MHz.
* **A sweep that is not repeated inside the 0.5 s memory.** Sweeps slower than about 0.5 s per pass are followed by the normal 0.5 weight, which tracks them as long as the jammer moves less than about 0.25 MHz in the 10 ms between solves (a sweep slower than about 2 s per 50 MHz).
* **Sweeps between 5 ms and 0.5 s per pass** are expected to work like the 5 ms row (long memory), not simulated (it needs 3000+ blocks).
* Pulses of milliseconds, ADC clipping on a strong pulse before the gain is cut, two jammers, and every board number: not tested. Low-duty pulses (1 %) are nulled less (12-28 dB).
* 5 ms sweep at J/N 30 gives 32 dB (against 43 dB for a 1 ms sweep): the longer the sweep, the less of it is in the memory at any time.
