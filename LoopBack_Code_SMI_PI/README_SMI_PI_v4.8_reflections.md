# SMI-PI 4.8.0 (bitstream 4.7, smi_pi IP 2.4): reflections of the jammer, and a correction

**Needs a new bitstream and a rebuild of the HLS core (the reference tap moved):**

```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh; source /tools/Xilinx/Vitis/2023.2/settings64.sh; source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh
cd LoopBack_Code_SMI_PI && chmod +x Automation/Linux/*.sh Automation/Linux/steps/*.sh
./Automation/Linux/build_all.sh --from smi --jobs 16        # stages smi .. boot; add --defines GNSS_DEPLOY_AUTO_TX for the SD build
```

Boot shows `ANTI-JAM (SMI-PI 4.8.0)`. Firmware 4.8 refuses to start the filter on an older bitstream (it prints why): the reference delay is compiled into both.

## 1. A correction to what I told you from the 20 dB report

The report said `residual +0.2 dB (0 = fully nulled)` and `J/N +29.5 dB`, and I read that as a complete null. **It was not proof.** The same report said
`noise floor : not learned yet` (the jammer was on from power-up). Without a clean noise floor, J/N, suppression and residual are measured against the filter's
**own** noise estimate (`loading = output power / (1+|w|^2)`), so the residual is close to 0 dB by construction. Other numbers in that report point the other way:
the output power was 699 LSB^2 at gain 34 dB, and the largest RX2 tap was only 0.12 while |w|^2 was 1.71, so the weights lean on the RX1 taps (a temporal predictor of a
coloured, partial-band jammer), which is what a spatial null does not need. Firmware 4.8 now says this in the report. **To get true numbers: jammer OFF for 10 s after
`gnss_tx=1`, then ON.**

## 2. What the 16 taps can and cannot reach (simulation)

The reference tap of RX1 was at delay 8, so a reflection of the jammer that arrives LATER than the direct path could only be cancelled up to 7 samples
(114 ns, 34 m of path at 61.44 MS/s). Reflections from buildings are often longer. Simulation, partial-band jammer 36 MHz at -5.5 MHz (what the board measured), J/N 45,
reflection of half the amplitude reaching the two antennas differently (testbench scenario 22):

| Reflection delay | reference tap 8 (4.7) | reference tap 4 (4.8) |
|---|---|---|
| none | 54.9 dB, no loss | 53.9 dB, no loss |
| 3 samples | 55.2 dB, no loss | 54.1 dB, no loss |
| 8 samples | 45.1 dB, 3.6 dB satellite loss | **51.9 dB, no loss** |
| 11 samples | 45.1 dB, **23 dB loss** | 52.3 dB, 5.3 dB loss |
| 14 samples | 16.9 dB, 20 dB loss | 45.0 dB, 17.8 dB loss |

Moving the reference to delay 4 (SMI_REF / AJ_REF) extends the cancellable reflection from 7 to 11 samples at no cost in logic (about 1 dB less on a clean jammer).
Regression: scenarios 1-14 and 16-21 pass with it. **Longer reflections need more taps (32 per antenna, about 195 of the 220 DSP slices: not done) or a lower sample rate:**
the same physical reflection that is 14 samples at 61.44 MS/s is 4 samples at 18 MS/s (`gnss_band=5`, 52.7 dB, no loss). If your field place has strong reflections, use the 18 MS/s
profile for the test.

## 3. Your radio setup, 50 MHz barrage and CW centred on 1575.42 MHz (simulation, scenario 21, no reflections)

Barrage suppression 42 / 50 / 56 / 57 dB at J/N 30 / 40 / 50 / 55 and **CW 63-72 dB** at J/N 30-65, no satellite loss up to J/N 55 (barrage) and 65 (CW);
above J/N 60 the 12-bit ADC adds noise (barrage J/N 65: residual +11.8 dB). Details in `README_SMI_PI_v4.7_pulse.md`.

## 4. What I could not do

* I do not have the `gnss_report?` from your 30 dB and 35 dB steps, so I cannot say that reflections (or anything else) are what limits you there. They are what the
  simulation shows breaking the filter at a suppression near 40 dB, which is what your 20 dB report showed (39 dB, with the noise floor unknown).
* Not synthesised: the HLS core with the moved reference and the whole FPGA wrapper (timing unverified); the firmware was only compiled.

## 5. What to send back

At 30 dB and at 35 dB: jammer OFF for 10 s, then ON, wait 60 s, then `gnss_report?` twice, 10 s apart (look for `noise floor : learned`), plus the receiver photo.
Compare once with `gnss_band=5` (18 MS/s) at the same jammer level.
