# SMI-PI 5.0: SFAP, space-frequency adaptive processing

Firmware 5.1.0, bitstream 5.1 (`CORE_VERSION` 0x00050100; 5.0 = SFAP, 5.1 adds the I/Q-image corrector, section 8). STAP (the 16-tap space-time filter of 4.x) is still there and stays the default. SFAP is a
second filter that you switch on with one command, so the two can be compared on the same hardware with the same jammer.

**Nothing in this release has run on the board.** The algorithm was simulated in floating point (`Tests/sfap/study.py`), the FPGA core was simulated
in RTL (iverilog, `Tests/sfap/`), and the firmware was syntax-checked. Vivado timing and utilisation are not known until you build it (section 7).

## 1. What was wrong with STAP, and what SFAP changes

| | STAP (4.x) | SFAP (5.0) |
|---|---|---|
| Where the weights are computed | on the ARM, from a 4096-sample DMA capture, 50 to 300 solves per second | in the FPGA, for every 512 samples (8.3 us) |
| Weights per antenna | 16 taps, the same for all frequencies | one complex weight per FFT bin, 1024 bins (60 kHz each) |
| Time to react to a new jammer | one capture + one solve + the ARM's detection loop: milliseconds | the first frame that contains the jammer: about 17 us |
| A jammer that moves (chirp) | needs the sweep detector and a long memory | each bin only has to follow its own frequency |
| Multipath (echo of the jammer) | cancelled up to 11 samples by the delayed reference tap | per bin, any echo delay up to about 100 samples |
| Cost | about 100 DSP48 (HLS) | about 60 DSP48 and 17 BRAM36 more (estimate) |

STAP is a closed-form power-inversion filter with 33 weights. SFAP is the same power inversion, but solved for each frequency separately:

    Y[k] = X1[k] - c[k] X2[k],     c[k] = R12[k] / (R22[k] + load)
    R22[k] <- R22[k] + (|X2[k]|^2   - R22[k]) / 2^lam
    R12[k] <- R12[k] + (X1[k] X2[k]^* - R12[k]) / 2^lam

X1, X2 are the 1024-point FFTs (sqrt-Hann window, 50 % overlap) of the two antennas. The current frame is part of the average, so the weight is already
there for the frame it is applied to. The result goes back through an inverse FFT and overlap-add. The covariance of every bin is only a 2x2 matrix, so
the solve is one division, and there is no Cholesky, no ARM and no DMA.

## 2. What the simulation says (floating point, `Tests/sfap/study.py`, 6 ms of signal, jammer on at 1 ms)

Scene: GNSS-like BPSK at -18 dB per sample, two antennas with a frequency-dependent mismatch (3-tap FIR on RX2), jammer from another direction, optional
echo of the jammer 9 samples later from a third direction, 12-bit ADC with the jammer at -12 dBFS. "Loss" is the output signal-to-noise ratio against
one clean antenna; negative means SFAP and STAP gain from the second antenna because the satellite and the jammer arrive from different directions.
STAP "as built" is my model of the 4.x schedule (200 solves per second, 1 ms ARM delay); it is an assumption, not a measurement. "STAP ideal" solves
every block.

| jammer | J/N | steady loss, dB: STAP ideal | STAP as built | **SFAP** | time to loss < 3 dB: STAP ideal | STAP as built | **SFAP** |
|---|---|---|---|---|---|---|---|
| barrage 50 MHz | 30 | -1.7 | -0.9 | **-3.0** | 133 us | 3533 us | **0 us** |
| barrage 50 MHz + echo | 30 | -2.0 | -0.8 | **-4.5** | 133 us | 3533 us | **0 us** |
| barrage 50 MHz | 40 | -1.8 | -0.8 | **-3.0** | 133 us | 3533 us | **0 us** |
| barrage 50 MHz + echo | 40 | -2.0 | -0.9 | **-4.3** | 133 us | 3533 us | **0 us** |
| barrage 50 MHz | 50 | -1.5 | -0.2 | **-2.5** | 133 us | 3533 us | **0 us** |
| barrage 50 MHz + echo | 50 | -1.3 | -0.1 | **-3.1** | 133 us | 3533 us | **0 us** |
| barrage 50 MHz | 60 | 0.0 | 1.3 | **-0.9** | 133 us | 3533 us | **0 us** |
| barrage 50 MHz + echo | 60 | 0.8 | 2.4 | **1.4** | 133 us | 3566 us | **150 us** |
| CW | 40 | -0.0 | 0.9 | **0.7** | 133 us | 3533 us | **0 us** |
| CW + echo | 40 | 0.1 | 1.1 | **0.8** | 133 us | 3533 us | **0 us** |
| CW | 60 | 1.1 | 2.4 | **-1.1** | 133 us | 3533 us | **0 us** |
| CW + echo | 60 | 0.5 | 2.0 | **-0.4** | 133 us | 3533 us | **16 us** |
| chirp 100 us | 40 | -1.5 | -0.4 | **-3.2** | 133 us | 3533 us | **16 us** |
| chirp 100 us + echo | 40 | -1.9 | 6.0 | **-4.6** | 216 us | 3566 us | **33 us** |
| chirp 1 ms | 40 | 1.1 | 31.6 | **-4.0** | 1066 us | 4450 us | **16 us** |
| chirp 1 ms + echo | 40 | 2.6 | 31.7 | **-4.6** | 1066 us | 4450 us | **16 us** |
| pulsed barrage | 40 | -0.2 | 0.5 | **1.7** | 16 us | 16 us | **16 us** |
| pulsed barrage + echo | 50 | 5.2 | 4.8 | **7.2** | 16 us | 16 us | **33 us** |

Where SFAP is **not** better than an ideal STAP (one that solves every block, which the ARM cannot do) in steady state: CW at 40 dB (0.7 against 0.0 dB),
a 60 dB barrage with an echo (1.4 against 0.8 dB), and both pulsed cases (1.7 against -0.2 dB, and 7.2 against 5.2 dB with an echo; the pulse blanker of
4.7 stays in the path for both). Against the STAP as I modelled it on the board ("as built", an assumption), SFAP is better or equal everywhere except
the pulsed cases. So the honest summary is: much faster, better against moving jammers and wide echoes, about equal otherwise, a little worse on pulses.
I cannot say "better in every case": the simulation has no real I/Q imbalance, no AGC steps and no antenna coupling.

## 3. What was verified in RTL

`Tests/sfap/run_rtl.sh` (iverilog + numpy):

* `sfap_fft.v` against numpy: 76 to 83 dB signal-to-error, back-to-back blocks, fixed latency 1068 clocks (1072 with the pipeline registers added for timing).
* `sfap_bin.v` against a floating-point model of the same recursion: 1.4 LSB rms error on bins of 3e5 amplitude; a 66 dB jammer comes out at the noise level.
* the whole core with the weights forced to zero: output = input delayed, correlation 0.9996, gain 0.998.

Whole core, RTL (iverilog), 60000 samples, jammer on at 20000 samples, 12-bit input with the jammer at 500 LSB rms, echo 9 samples later from another direction.
"Excess" is the output power against the same core run without the jammer; 3 dB is what two antennas of independent noise give (it is the same in the simulation above).

| jammer | J/N | within 5 dB of baseline after | steady excess, median | worst window | note |
|---|---|---|---|---|---|
| barrage 50 MHz | 30 (no echo) | 17 us | 2.8 dB | 3.2 dB | |
| barrage 50 MHz + echo | 40 | 17 us | 3.4 dB | 8.8 dB | re-run on the final (timing-hardened) RTL: identical |
| barrage 50 MHz + echo | 50 | 17 us | 1.5 dB | 15.2 dB | bursts coincide with clipped input samples |
| barrage 50 MHz + echo | 60 | 17 us | -0.5 dB | 17.2 dB | the noise is at the ADC's own quantisation level |
| CW + echo | 40 | 17 us | 0.6 dB | 1.9 dB | |
| CW + echo | 60 | 50 us | 3.1 dB | 10.7 dB | |
| chirp 100 us + echo | 40 | 17 us | 3.4 dB | 17.0 dB | |
| chirp 1 ms + echo | 40 | 17 us | 2.8 dB | 14.8 dB | |

The two pulsed-barrage runs are **not valid** and are not quoted: my test scaling put the noise at 150 LSB. The pulsed case has been simulated in
floating point only (table above); SFAP is not better than STAP there.


## 4. Registers (new, `gnss_passthrough` 5.0)

| Offset | Name | |
|---|---|---|
| 0x1B0 | SFAP_CTRL | RW: [0] en, [1] sel (1 = SFAP feeds TX1), [2] clr, [5:3] lam (3 to 6), [10:8] make-up shift |
| 0x1B4 | SFAP_LOAD | RW: diagonal load, about the noise power per FFT bin (the firmware sets it from the learned noise floor) |
| 0x1B8 | SFAP_STAT | RO: [0] ready, [1] underrun, [2] overrun, [3] selected, [31:16] frames |
| 0x1BC | SFAP_IN | RO: mean |I|+|Q| of RX1 over 32768 samples |
| 0x1C0 | SFAP_OUT | RO: the same at the SFAP output |
| 0x1C4 | SFAP_INFO | RO: 0x5FA90100 when the core is there |

## 5. Commands (PuTTY)

* `gnss_aj_sfap=0` STAP only (default). `=1` SFAP carries TX1 while a jammer is there; the core runs all the time, so its averages are ready. `=2` SFAP always.
* `gnss_report?` shows an `SFAP` line: mode, core ready, frames, load, UNDERRUN/OVERRUN, and the measured in/out ratio.
* Everything else (`gnss_aj_blank`, `gnss_aj_sweep`, `gnss_aj_mode`) is unchanged and still applies to STAP; the pulse blanker also runs behind SFAP.

Detection of the jammer, the RX gain loop and the make-up gain stay with the ARM as before. SFAP replaces the filter, not the supervision. With
`gnss_aj_sfap=1` the time until SFAP takes over is therefore still the ARM's detection time. With `=2` there is no switch at all, SFAP is always on.

## 6. Build

Unchanged: `Automation/Linux/build_all.sh --jobs 16`, or the ten step scripts. Step 4 now packages six more HDL files into the IP. A bitstream
and firmware must match: the firmware prints `the bitstream has no SFAP core` otherwise and keeps working as 4.8.

## 7. What I do not know yet

* **Utilisation and timing.** My estimate is about 60 more DSP48 (the design used 133 of 220) and 17 more BRAM36. Every stage is one operator and
  every multiplier has its M and P registers, because 4.7.0 failed 245.76 MHz that way. I have not run Vivado. If `BUILD_TIMING` is `VIOLATED`, send
  the two lines from `Build/Logs/build.log` and the worst path as before.
* **The first output frame.** SFAP delays the signal by about 2600 samples (42 us) and drops its first frame. Switching between the STAP and SFAP
  paths flushes the TX buffer; the receiver sees a short gap, not a problem for tracking.
* **Pulses.** The pulse blanker is behind SFAP, but SFAP's averages still see the pulses; mode 1 vs 2 and `gnss_aj_blank` will tell.
* **Clipping.** A sample that clips the ADC is a non-linear error no weight can remove; the RTL runs showed one 5 to 8 dB burst for each clipped sample.
  Keep the AGC headroom.

## 8. 5.1.0: blind I/Q-image corrector (WL-STAP-PI's idea, in a cheap form)

Bitstream 5.1 (`CORE_VERSION` 0x00050100, `SFAP_INFO` 0x5FA90101), firmware 5.1.0.

**Why.** An AD9361 receiver with I/Q gain/phase error turns a jammer `a` into `a + k a*`: a second copy at the mirror frequency with a different
spatial signature. Two antennas have one null per frequency; the image takes it away, so the suppression stops at (J/N - image rejection).
WL-STAP-PI attacks this with conjugate taps. The same thing per FFT bin needs a 3x3 solve per bin (`Tests/sfap/wl_study.py`: it works, 60 dB barrage
7.5 -> 3.4 dB loss, but it would cost 100+ DSP48, which do not exist here).

**What was built instead.** A correction per receiver in front of SFAP: `x' = x - k x*`, with `k = E[x^2] / (2 E|x|^2)` (a circular jammer or noise has
`E[a^2] = 0`, so `E[x^2] = 2 k E|a|^2`). The FPGA (`sfap_iq.v`) applies k and keeps the three sums over 2^16 samples (registers 0x1D0..0x1E4); the ARM
divides, smooths (10 % per 50 ms) and writes k back (0x1C8, 0x1CC) while a jammer dominates the noise by 15 dB. k belongs to the receiver, not to the
jammer, so it is kept when the jammer goes away. Cost: 14 DSP48, no divider in the FPGA.

**Float study** (`Tests/sfap/iq_study.py`, image rejection 42 dB on RX1 and 46 dB on RX2, SINR loss in dB, steady):

| jammer | SFAP | SFAP + per-bin WL | SFAP + I/Q corrector |
|---|---|---|---|
| barrage + echo 50 dB | 1.4 | -0.6 | -0.9 |
| barrage + echo 60 dB | 7.5 | 3.4 | 3.2 |
| barrage + echo 70 dB | 9.5 | 5.0 | 6.6 |
| barrage 60 dB, no echo | 6.6 | 1.0 | 0.8 |
| CW + echo 60 dB | -2.1 | 1.2 | -2.1 |
| chirp 100 us + echo 50 dB | 0.9 | -0.8 | -1.0 |

**RTL** (`Tests/sfap/rtl/core_iq.py`, whole core, 60000 samples, barrage + echo, same image rejection, output power against the jammer-free run):

| | k = 0 (median / mean-power) | k estimated from the statistics registers |
|---|---|---|
| 50 dB | 6.0 / 7.1 dB | **2.7 / 5.2 dB** |
| 60 dB | 7.0 / 8.5 dB | **2.1 / 6.1 dB** |

The statistics registers equal an exact integer model bit for bit. The first RTL run lost: a corrected sample near full scale wrapped around at 12 bits
and made a full-scale error. The output now saturates (`sfap_iq.v`, stage 7).

**What it does not do.** The AD9361's imbalance varies with frequency; the model is flat. A CW exactly on the LO (DC) is not circular: k is limited to
|k| <= 0.05 and the firmware keeps it small. The CW notch of WL-STAP-PI was not taken over: SFAP has its own null on every bin, so a CW does not use up the
barrage's null (CW 40 dB: 0.7 dB, 60 dB: -2.1 dB in the simulation).

Command: `gnss_aj_iqcorr=1` (default, only active with `gnss_aj_sfap=1/2`) or `0`. `gnss_report?` shows `I/Q corrector : ON; RX1 k = ..., image rejection .. dB`.

Head-to-head with WL-STAP-PI's own reference model at 30.72 MS/s (`Tests/sfap/compare_wl.py`): WL-STAP-PI is bypassed at 61.44 / 50 MS/s, and its 5 taps do not
cancel a 9-sample echo (barrage and chirp with echo: no suppression); it is better on CW and on the median of a pulsed barrage.

## 9. 5.1.1: timing fixes after the first Vivado build of 5.1.0

The first build of 5.1.0 (Vivado 2023.2, xc7z020) produced a bitstream and BOOT.BIN but **missed timing: WNS -3.007 ns, 12456 failing endpoints, all in the
245.76 MHz sample-clock group**. Real numbers from that build: 183 DSP48 of 220, 27207 LUT (51 %), 36873 FF, 28.5 BRAM tiles of 140, hold met (+0.034 ns).
The 4.x logic and the HLS core were not the problem (`clk_fpga_0` +0.341 ns).

| Worst path | Cause | Fix in 5.1.1 |
|---|---|---|
| FFT delay-line BRAM read -> butterfly add -> rounding -> mux -> the same BRAM's write input (-3.0 ns) | one clock for BRAM clock-to-out, an add, a rounding add and a mux | pipelined: BRAM output register, add, then write from registers (latency 4 per RAM stage); scaling truncates (no +1 adder); the D < 8 stages keep one add/sub and a mux in the loop |
| `sfap_iq2_act` (AXI clock) -> corrector DSP input (-0.565 ns, 211 endpoints) | the tools merged my capture register into the DSP, so the timing exception for the crossing no longer matched | capture registers `DONT_TOUCH`; the max-delay constraint now also covers every cell inside the SFAP core |
| (not reported, same pattern) | reorder-RAM write data went through a negation, frame-RAM write data through a clamp, the leading-zero count of the divider was one 52-bit priority chain | registered write data for both RAMs; leading-zero count in two stages |

Verified again in RTL after the changes: FFT (latency now 1086 clocks, same accuracy), bin processor (1.4 LSB), and the whole core with the corrector
(statistics registers still equal the integer model; 50 dB barrage 6.0 -> 2.7 dB, 60 dB 6.9 -> 2.0 dB with the corrector on).

**Not known yet:** I only saw the worst path of each clock group, so another violating structure may remain. If the next build still says `VIOLATED`, send
```bash
grep -E "Slack \(VIOLATED\)|Source:|Destination:" Build/Output/timing_impl.rpt | head -60
```
