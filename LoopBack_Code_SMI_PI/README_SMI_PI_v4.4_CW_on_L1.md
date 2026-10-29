# SMI-PI 4.4.0: a sharp CW on top of the GPS signal (1575.4 MHz)

Firmware 4.4.0, bitstream / IP **smi_pi 2.2**. Written after the field test of
28-Oct-2026 (`FIELD_TEST_2026-10-28_CW_partial_jamming.md`,
`FIELD_TEST_CW_1575.4_on_L1.md`). **Everything below is C simulation of the real
filter core and the board's own weight solver, not a measurement on the board.**
Vitis HLS synthesis, Vivado and the board were not available here.

## 1. What the field test showed (SMI-PI 4.1.0, TX attenuation 20 dB)

| CW jammer | Tx power at the jammer | Result |
|---|---|---|
| 1582.5 MHz (= the LO: 0 Hz in the samples, "partial jamming") | 15 / 20 / 25 / 30 dBm | 13 / 8 / 4 / 3 satellites (28 with no jammer) |
| 1575.4 MHz (on GPS L1 / Galileo E1, "sharp") | 15 dBm | no satellite locked |
| 1575.4 MHz | up to 40 dB (one sheet, level not entered) | 4 GPS satellites, 116 s to lock |

Link budget at 6.5 m (free space, 52.7 dB at 1.575 GHz): 15 dBm at the jammer is
about -38 dBm at the board antenna plus antenna gains; the receiver noise in 56 MHz
is about -95 dBm. So the jammer is roughly **57 dB + antenna gains above the noise
at 15 dBm, 62 + gains at 20 dBm, 67 + gains at 25 dBm** (J/N, free space). The
satellites vanished in step with that: the board was working at **J/N 60 to 75 dB**,
above the 50 dB the v4.1 filter was designed and tested for.

The 1582.5 MHz jammer sits at 0 Hz in the baseband, where the AD9361's DC-offset
tracking already takes a large part of it away. A CW at 1575.4 MHz is 7.1 MHz from
the LO and gets no such help: that is why it was so much harder.

## 2. Why the 4.1 filter lost satellites at J/N 60-75 dB

I ran the real HLS core with the firmware solver (`Tests/sim_cw.cpp`; CW at -7.06 MHz
from the LO, 61.44 MS/s, 12-bit ADC, AD9361-like I/Q imbalance, gain cut so the ADC
peak stays below 1850, three weak satellites, one of them exactly on the CW). The
null is a spatial null spread, by the solver's loading, over the 16 taps of RX2: each
tap is only about 0.05. The weights were rounded to Q3.12 (steps of 2.4e-4), and
sixteen such rounding errors add up to about 3e-4 of the jammer: **-65 dB**. Measured
suppression was 58 to 68 dB, different on every capture, never better. Against a CW
that is 65 dB or more above the noise, what is left is above the noise, and the
receiver loses the satellites. (Nothing else in the chain was the limit: with
finer weights the same data nulls to 71 dB, the floor of my measurement.)

## 3. What 4.4.0 changes

**Weight scale `ws`** (AJ_CFG[14:12], 0..5). The 16-bit weight words now hold
`w * 2^ws`; the core adds up all weighted taps, divides the sum by 2^ws, then
subtracts it from the reference tap. The firmware picks the largest `ws` that keeps
every weight inside Q3.12 (|v| < 7.5), so a small-weight null gets up to 32 times
finer steps and a large-weight solution (wideband jammer, |w| about 1) still fits.
Nothing else in the filter, the registers or the solver changed.

| File | Change |
|---|---|
| `Source/HLS/smi_pi/smi_pi.cpp/.h` | taps summed first, `>> ws`, then subtracted from the reference; `SMI_CFG_WS_LSB`, `SMI_MAXWS`; IP version **2.2** |
| `Source/Firmware/.../gnss_aj_math.c/.h` | picks `ws`, scales and quantises the weights, `r->ws`; `aj_set_max_ws()` |
| `Source/Firmware/.../gnss_aj.c`, `gnss_passthrough.h` | writes `ws` into AJ_CFG; reads AJ_INFO and **switches `ws` off if the bitstream is older than 4.4**; the report shows the scale and the weights in real units |
| `Source/HDL/gnss_passthrough.v` | `CORE_VERSION` 0x00040004, `AJ_INFO` 0x10080C03 (layout 3). No register or port moved |
| `Source/BlockDesign/system_bd.tcl` | requires smi_pi 2.2 |
| `Source/HLS/smi_pi/testbench.cpp` | bit-exact test covers `ws` 0..7; scenarios 9 and 10 (CW on L1) |
| `Tests/sim_cw.cpp`, `Tests/build_sim_cw.sh` | the multi-seed simulation of this section |

## 4. Result (simulation, 10 random seeds per level, I/Q imbalance on)

"Residual" is the output with the jammer over the output of the same weights on the
noise alone (0 dB = fully removed). "Loss" is the satellite SNR loss against RX1 with
no jammer. A = a satellite exactly on the CW, B = 4 MHz above it (and, by chance, in
nearly the same direction as the jammer), C = at -21 MHz (BeiDou B1I).

| J/N | suppression 4.3.0 -> 4.4.0 | residual 4.3.0 -> 4.4.0 | loss A 4.3 -> 4.4 | loss B | loss C |
|---|---|---|---|---|---|
| 50 dB | 64.8 -> 71.1 | +0.20 -> +0.07 | -0.8 -> -1.1 | 1.3 -> 1.2 | 0.1 -> 0.0 |
| 55 dB | 65.5 -> 71.4 | +0.45 -> +0.07 | -0.8 -> -1.1 | 2.2 -> 1.8 | 0.3 -> -0.1 |
| 60 dB | 64.8 -> 71.4 | +1.11 -> +0.07 | -0.3 -> -1.0 | 3.7 -> 2.6 | 1.0 -> 0.0 |
| 62 dB | 65.8 -> 71.4 | +1.30 -> +0.07 | -0.8 -> -0.9 | 4.0 -> 2.8 | 1.3 -> 0.1 |
| 65 dB | 65.2 -> 71.4 | +2.44 -> +0.08 | 1.4 -> -0.5 | 5.5 -> 3.2 | 2.5 -> 0.4 |
| 68 dB | 65.4 -> 71.3 | +3.24 -> +0.34 | 2.0 -> -0.2 | 6.7 -> 3.8 | 3.6 -> 0.8 |
| 70 dB | 65.3 -> 71.3 | +5.11 -> +1.26 | 2.5 -> 0.5 | 8.1 -> 4.5 | 4.9 -> 1.3 |
| 75 dB | 63.4 -> 71.3 | +17.1 -> +10.2 | 6.1 -> 3.0 | 12.8 -> 6.6 | 10.1 -> 3.6 |

Seeds with a residual above +3 dB (4.3.0 -> 4.4.0): 62 dB 1 -> 0 of 10, 65 dB 3 -> 0,
68 dB 6 -> 0, 70 dB 9 -> 0, 75 dB 10 -> 10. Full output: `Tests/cw_before_after.txt`.
The 71 dB is the floor of the measurement (the jammer-alone test signal is itself
quantised to 12 bits), not a limit of the filter. Negative losses for A are noise of
the estimator (about +-1 dB), because A has the same frequency as the CW residual.

HLS testbench scenarios 9 (CW 62 dB above the noise, on top of the satellite) and 10
(68 dB): with the weight scale 71.4 / 71.3 dB suppression, residual +0.04 / +0.27 dB;
the same weights without it, 61.6 dB / +2.63 dB and 68.1 dB / +1.57 dB. All ten
scenarios pass.

## 5. What this does not fix

* **Above J/N of about 72 dB** the 12-bit ADC is the limit: the RX gain has to be cut
  35 dB or more to keep the CW off full scale, and the noise ends up at the ADC's own
  quantisation noise. At 75 dB the loss is still 3-7 dB in this simulation.
* **Front-end overload.** If the jammer reaches the AD9361 (or an active antenna's LNA)
  at a level it cannot take, no digital filter helps. Check `ADC peak` and `rx gain`
  in `gnss_report?`; if the RX gain is at its lowest and the peak still reads about
  2047, put an attenuator (or a band-stop at the jammer's frequency) between antenna
  and board, or move the jammer away.
* **A satellite in the jammer's direction** is nulled with it (B in the table). A
  2-element array has one spatial null; this is the physics, not a setting.
* The simulation uses the noise floor, I/Q imbalance and path mismatch of the
  existing testbench. Real values differ; the board's `gnss_report?` shows the real
  J/N, suppression and residual.
* **Rebuild the whole project.** The filter core changed (IP 2.2, bitstream 4.4). With
  the firmware alone on an older bitstream the weight scale switches itself off
  (message `NOTE: the bitstream is older than 4.4`) and nothing gets better.
