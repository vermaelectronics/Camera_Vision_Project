# SMI-PI 4.7.0 (bitstream 4.6): pulsed barrage jammer

**Needs a new bitstream**: the FPGA changed (`gnss_pulse_blank.v`, new registers 0x19C-0x1AC), so rebuild from the IP stage:

```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh; source /tools/Xilinx/Vitis/2023.2/settings64.sh
cd LoopBack_Code_SMI_PI && chmod +x Automation/Linux/*.sh Automation/Linux/steps/*.sh
./Automation/Linux/build_all.sh --from ip --jobs 16        # smi_pi (HLS) is unchanged; add --defines GNSS_DEPLOY_AUTO_TX for the SD build
```

At boot: `version 4.7` / `ANTI-JAM (SMI-PI 4.7.0)`; `gnss_report?` has the new `pulse blanker` line. With the old 4.5 bitstream the firmware still runs
and prints `[AJ] NOTE: the bitstream has no pulse blanker`; only the firmware parts (gate, gain hold) work then.

## What your photo shows (2-Nov, GNSS Viewer, Venus 8)

`Position fix unavailable`, TTFF 904 s, date 1980/01/06 (the receiver never decoded a time), HDOP 0.00, 7 GPS and 11 BeiDou bars at 39-47 dB-Hz that
come and go. High bars but no fix = the receiver sees a good signal part of the time and loses it again, i.e. lock is broken by something that
is on and off.

I did not have the pulse parameters or a `gnss_report?` for it, so what follows is what a pulsed barrage does to this design, found by reading the code,
and what was changed for each. **None of it was run on the board.**

| # | Cause (in the 4.5 / 4.6 code) | Change in 4.7 |
|---|---|---|
| 1 | The RX gain loop looks at 256 samples every 5 ms. A pulse that is off at that moment is not seen; after 20 ms without one the gain is raised, the next pulse clips the ADC (a clipped pulse cannot be nulled), the gain drops 12 dB, and so on: the gain **pumps** and the level into the receiver with it. | FPGA keeps the largest sample of every 8.5 ms window (`AJ_RX_PEAK`, 0x1AC), the loop uses it. After a release that is followed by another attack within 5 s (twice), the gain is held down for 3 s (`gnss_rxg_pulsy`). |
| 2 | Weights are solved from every 4096-sample block. A block captured in a gap holds no jammer; averaged in (and after every gain step the statistics restart from one block) the weights drift to "no jammer" and the next pulse passes. | While pulses are seen, a block with no jammer is **skipped**: old weights and statistics kept (`aj_result_t.gate`). JAMMER OFF is delayed to 3 s. |
| 3 | Whatever the weights do not remove arrives at the receiver as bursts; a receiver loses lock on a burst but not on a steady 3 dB rise. | **Pulse blanker in the FPGA** (`gnss_pulse_blank.v`): the output is zeroed while 8 samples of I^2+Q^2 exceed a threshold set 7 dB above the output noise, 16 samples before and 16 after. The satellites lose the duty cycle (10 log(1-d): 1.5 dB at 30 %). It is also armed while the filter is bypassed, so a pulse the detector never sees is still blanked. |
| 4 | Short, low-duty pulses are missed by the 66 us captures. | The blanker's burst counter is the evidence: 4 or more bursts in 100 ms (and blanking under 60 % of the time) = pulsed jammer. |

Guard: if the blanker blanks more than 60 % of the output for 1 s (a continuous jammer the filter does not null) it switches itself off for 10 s and says so.

## Console

| Command | |
|---|---|
| `gnss_aj_blank=1` / `0` / `2` | blanker automatic (default) / off / always on |
| `gnss_aj_blank_db=7` | threshold above the output noise, 3..30 dB |
| `gnss_report?` | `pulse blanker : ON`, threshold, `blanked n % of the last 100 ms`, samples and bursts so far, `jammer-free blocks skipped`, `pulses SEEN / not seen`, `hardware peak` |

Register map (bitstream 4.6, `AJ_INFO` 0x10080C05): 0x19C `AJ_BLK_THR` (committed with the weights; `AJ_CFG[15]` = enable), 0x1A0 active threshold,
0x1A4 samples blanked, 0x1A8 bursts, 0x1AC `AJ_RX_PEAK` ([15:0] peak, [31:16] window counter).

## What was verified (and what was not)

* **Blanker RTL** (`Tests/rtl/run_blank_rtl.sh`, iverilog): 20000 samples, three thresholds and enable on/off, **bit-exact against its Python model**, counters equal. `gnss_passthrough.v` with the new blocks compiles (iverilog); it was **not** simulated as a whole and **not synthesised**. Vivado timing is unverified (the blanker is 2 multipliers and adders per sample at 2 clocks per sample).
* **C simulation** (`Tests/run_csim.sh`, scenarios 1-14 and 16-20 pass): pulsed barrage J/N 40, 30 % duty, 326 us pulses, blocks of 66 us:

| Case | Satellite SNR loss |
|---|---|
| **filter not engaged** (bypass), no blanker | **17.6 dB** |
| bypass + blanker (30 % of the output blanked) | **1.5 dB** (ideal for 30 %: 1.55) |
| filter, weights from every block, period 41000 samples (training ends in a gap) | 4.9 dB, suppression 29.5 dB |
| same with the gate (blocks without a pulse skipped) | **-1.1 dB**, suppression 47.8 dB |
| filter, other periods (16000-61000 samples) | -0.4 to -1.0 dB with or without gate |

  (negative = no loss). The blanker alone brings an un-nulled pulse train from 17.6 dB to the duty cost; the gate fixes the weights drifting in gaps.
* **Firmware**: compiled (syntax, with stub BSP headers); not run.
* **Not covered**: pulses of many ms (the 0.5-3 s OFF delay and the gain hold decide), a pulse stronger than the ADC before the gain is cut (the first
  pulse after a quiet period can still clip once), two jammers, the Venus 8's own behaviour. If the receiver still does not lock: send
  `gnss_report?` taken **while the pulsed jammer is on**, twice 10 s apart, and the pulse period and duty of the generator.

## Test T9 (on the board)

1. `gnss_tx=1`, wait 10 s with the jammer **off** (the noise floor is learned; the report says `noise floor : learned`).
2. Jammer pulsed on, start with a long period / 50 % duty. `gnss_report?` after 30 s and again after 40 s. Look at: `pulse blanker : ON`, `blanked ... %`
   (about the duty cycle), `pulses SEEN`, `rx gain` stable (not stepping), `jammer-free blocks skipped`.
3. Receiver: satellites in use and C/N0. If `blanked` is above 60 % the blanker switches off by itself: then the jammer is continuous or the filter does not null it.
4. A/B: `gnss_aj_blank=0` against `gnss_aj_blank=1` at the same pulse settings.
