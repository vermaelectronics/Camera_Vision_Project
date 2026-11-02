# SMI-PI 4.5.0: chirp, hopping and pulsed jammers (simulation, 31-Oct-2026)

C simulation of the real core and solver (`Tests/run_csim.sh` scenarios 1-14 still pass). New exploratory scenario 15 in the testbench:
`smi_pi_tb 15 <kind> <J/N dB> <k1> <k2>`, kind 2 = chirp (sawtooth sweep of k2 x fs in k1 samples), 3 = pulsed noise (period k1 samples, on-fraction k2,
J/N inside the pulse), 4 = CW that hops by k2 x fs after the three training captures (weights older than the frequency step).
61.44 MS/s: 61 samples = 1 us, 6144 = 100 us. 12-bit quantisation, I/Q imbalance, 0.15-sample path difference, same as scenarios 6/8.
**Simulation only, not tested on the board.** Gain is set from the true peak (an ideal gain loop: no clipping before the gain cut).

## Chirp (sweep +-0.4 fs = about 49 MHz)

| Sweep period | J/N 30 | J/N 40 | J/N 50 |
|---|---|---|---|
| 1 us (61 samples) | 48 dB, loss -0.7 | | 60 dB, loss -0.4 |
| 10 us (600) | 44 dB, loss -0.7 | | 60 dB, loss -0.7 |
| 100 us (6000) | 42 dB, loss -0.7 | 55 dB, loss -0.6 | 60 dB, loss -0.5 |
| 150-200 us (10000-12288) | | 54 / 45 dB, loss -0.7 / -0.4 | |
| **260 us (16000)** | | **4.8 dB, residual +33 dB, loss 32 dB** | |
| **330 us (20000)** | **3.6 dB, loss 25 dB** | **4.0 dB, loss 36 dB** | **4.7 dB, loss 58 dB** |

(suppression of the jammer; loss = satellite SNR loss against RX1 with no jammer.) A narrow chirp (3 MHz in 330 us, J/N 40): 41 dB, loss 2.8 dB.

## CW that moves after the weights were computed (what a slow sweep does)

| Frequency step | J/N 30 | J/N 40 | J/N 50 |
|---|---|---|---|
| 0.05 MHz | | 52 dB, loss 0.5 | |
| 0.1 MHz | | 45 dB, loss 1.3 | |
| 0.25 MHz | | 36 dB, residual +5 dB, loss 5.5 | |
| 0.5 MHz | 29 dB, loss 3.8 | | 26 dB, residual +24 dB, loss 21 |
| 1.8 MHz | 9 dB, loss 20 | | 9 dB, loss 32 |
| 4.9 MHz and more | none | | none |

## Pulsed noise jammer

| Period, duty | J/N 30 | J/N 40 | J/N 50 |
|---|---|---|---|
| 100 us, 50 % | 38 dB, loss -1.3 | | 48 dB, loss -0.4 |
| 100 us, 10 % | 29 dB, loss -0.7 | | 39 dB, loss 1.2 |
| 100 us, 1 % | 13 dB, loss 0.5 | | 28 dB, loss 2.0 |
| 8 us, 50 % / 10 % / 2 % | | 36 / 29 / 23 dB, residual +2 dB, loss 0.6 / 0.2 / -0.2 | |

For 100 us pulses with duty 10 % or less the 4096-sample block (66 us) often contains no pulse; the RX1-RX2 coherence of such a block reads 0.03,
so the firmware's detector would not call it a jammer (the numbers above are the solver's result when it is engaged).

## Findings

1. **Fast chirps work.** A sweep that repeats within about 200 us (the time covered by the three blocks the solver averages) is seen as a wideband jammer: 42-60 dB
   suppression, no satellite loss. Typical car-type GNSS jammers sweep in 8-20 us.
2. **A sweep that is slower than that is not removed in this model** (260 us and longer: 4 dB). The null of the 33-weight filter is narrow in frequency: about 45 dB
   at 0.1 MHz away from the frequency the weights were solved for, 36 dB at 0.25 MHz, 26 dB at 0.5 MHz, none at 5 MHz.
3. For the board (about 95 solves per second, so weights up to about 10 ms old, latency not measured): a chirp is tracked only if it moves less than about 0.25 MHz in 10 ms
   (a full 50 MHz sweep taking longer than about 2 s). Sweeps between about 0.2 ms and a couple of seconds per 50 MHz would fall in the gap.
   This 10 ms figure is an estimate from the solve rate, not a measurement.
4. **Pulses work when the solver sees them**: 23-48 dB suppression and a satellite loss of at most 2 dB for 100 us pulses (1-50 % duty) and 8 us pulses (2-50 %). Low-duty
   pulses are nulled less (13 dB at 1 % duty and J/N 30), and the coherence detector can miss pulses that fall between blocks.
5. **Not covered:** pulses of milliseconds (detection / state machine on the board), ADC clipping when a strong pulse arrives before the gain is cut, two jammers.
