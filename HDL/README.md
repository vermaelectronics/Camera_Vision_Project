# 2-Antenna CRPA Anti-Jamming Core: Robust LCMV (PIER C 2008)

FPGA (Verilog) anti-jamming core for an ANTSDR E310 (AD9361, 2 RX channels) GNSS receiver.
The weights follow the robust LCMV formulation of:

> R. Li, X. Zhao, X. W. Shi, "Derivative Constrained Robust LCMV Beamforming Algorithm,"
> *Progress In Electromagnetics Research C*, vol. 4, pp. 43-52, 2008. (Eq. 6-9)

The GPS look direction and the jammer direction are supplied by the PS (AXI registers 0x40 / 0x44).
**This core contains no DOA estimation.** The jammer angle must come from PS software, an external DOA
system, or a known jammer position.

---

## 1. Algorithm

Paper eq. (9):

```
min  w^H R w
s.t. S^H w        = 1        look constraint      (GPS direction)
     w^H B^p A_J  = 0        derivative constraint (null widening)
     w^H A_J      = eps      enforcement constraint (null deepening)

w_opt = R^-1 C (C^H R^-1 C)^-1 f                    (eq. 7 with all constraints in C, f)
```

Steering vector for 2 elements at d = λ/2: `a(θ) = [1, exp(j·π·sin θ)]^T`.

### What fits on 2 antennas

A 2-element array has **2 degrees of freedom**, and eq. (9) needs 3 constraint rows (rank check in
`model/lcmv_dc_model.py`).

* **Derivative row is omitted.** With B = diag(0,1), `w^H B a_J = conj(w2)·e^{jψ} = 0` forces
  **w2 = 0**. That switches the second antenna off and leaves no null. The paper's null widening
  needs M ≥ 4 for one jammer (the paper uses 8 and 16 elements).
* **Look and enforcement rows are implemented:** `C = [a_s a_J]`, `f = [1 ε]^T`.

C is then 2×2 and invertible, so the paper's solution simplifies exactly:

```
w = R^-1 C (C^H R^-1 C)^-1 f = R^-1 C · C^-1 R C^-H f = C^-H f
```

Solving `a_s^H w = 1`, `a_J^H w = ε` (s = a_s(2), j = a_J(2)):

```
w2 = (1 - ε) · (s - j) / |s - j|^2
w1 = 1 - conj(s) · w2
```

This is the exact LCMV optimum for this constraint set. R cancels because the constraints use every
degree of freedom, so the core needs no covariance estimate and no matrix inverse. `model/lcmv_dc_model.py`
checks numerically that this equals `R^-1 C (C^H R^-1 C)^-1 f` for random R and ε = 0, 0.001, 0.006.

### ε (`EPS_Q16`, Q16.16)
* `0` (default): ideal null at `jam_angle`.
* `> 0`: residual gain ε toward the jammer (paper uses 0.001-0.006). With 2 antennas this makes the
  null **shallower only**. It does not widen it (paper Fig. 1 behavior needs the derivative rows).

### Jammer near the GPS direction
`|s - j|^2 → 0` makes the weights grow without bound. `|s - j|^2` is clamped to `DEN_MIN_Q16`
(default 0.01, about 2° separation at broadside). Inside that region the look constraint is no longer met exactly.

---

## 2. Files

| File | Description |
|---|---|
| `lcmv_crpa_core.v` | Core: angles → steering vectors → LCMV weights → beamformer. Same ports as the original core. |
| `lcmv_dc_weight_2ch.v` | LCMV weight solver (eq. 9 look + enforcement), Q16.16, 72 clocks. |
| `sdiv_q16_serial.v` | Serial signed divider `(num<<16)/den`, saturated Q16.16, 66 clocks. |
| `steering_vector_2x2.v` | Your original steering LUT: `cos/sin(π·sin θ)`, integer degrees, Q15. |
| `lcmv_beamformer_2ch.v` | Your original beamformer `y = conj(w1)x1 + conj(w2)x2` (unchanged). |
| `gnss_passthrough.v` | Top level (AXI-Lite + datapath). Only header comments changed. |
| `covariance_2antenna.v` | Your original covariance block. **Not used** by this core (R cancels), kept for future N-element work. |
| `sim/tb_lcmv_crpa_core.v` | Self-checking testbench. |
| `sim/sim_results.txt` | Testbench output. |
| `sim/lcmv_null_pattern.png` | Beam patterns of the simulated weights. |
| `model/lcmv_dc_model.py` | Python reference: paper formula vs closed form, HDL cross-check, derivative-row rank check. |

Removed from the original design: `d_inverse_2x2.v`, `lcmv_r_inverse_c_2x2.v`, `lcmv_b_e_inverse_2x2.v`,
`lcmv_weight_vector_2x2.v`. They computed the same `w = C^-H f` through two 2×2 inverses. The original
`c_h_b_matrix_2x2` module was missing from the upload, so that design could not compile. The originals are in
`original_HDL/` (zip) and in git history.

---

## 3. Interface

```
lcmv_crpa_core #(
    IQ_WIDTH    = 16,
    ACC_WIDTH, COV_WIDTH, COV_SAMPLES, COV_SHIFT    // unused, kept for compatibility
    EPS_Q16     = 0,     // paper ε in Q16.16 (65 ≈ 0.001)
    DEN_MIN_Q16 = 655    // |s-j|^2 floor (0.01)
)
```

| Port | Meaning |
|---|---|
| `enable` | 1 = continuous weight update and beamforming. 0 = disabled, `weights_valid = 0`. |
| `gps_angle` | Look direction, integer degrees, -90…90 (AXI 0x40). |
| `jam_angle` | Null direction, integer degrees, -90…90 (AXI 0x44). |
| `rx_valid, rx1_*, rx2_*` | Antenna samples. |
| `w1_*, w2_*` | Current weights, Q16.16 (AXI 0x50-0x5C). |
| `y_re, y_im, beam_valid` | Beamformer output, integer ADC scale. |
| `busy, weights_valid` | Status. |

Weights are recomputed continuously (about 80 clocks per update), so a new `jam_angle` written by the PS takes
effect within about 1 µs at 100 MHz. Old weights stay active until the new set is ready.

---

## 4. Simulation results (Icarus Verilog 12)

2-element ULA, d = λ/2, noise-like jammer, JNR ≈ 46 dB, 12-bit ADC, GPS look direction 10°.

| Case | Null at | Gain to GPS | Null depth | Output/input power |
|---|---|---|---|---|
| jammer 40°, `jam_angle = 40` | 40.00° | 0.00 dB | **-94.8 dB** | -44.8 dB |
| jammer 46°, `jam_angle = 46` | 46.00° | 0.00 dB | **-103.6 dB** | -46.2 dB |
| jammer 46°, `jam_angle = 45` (1° error) | 45.00° | 0.00 dB | -34.3 dB | -31.6 dB |

**PASS.** At -45 dB output/input the jammer is cancelled down to the receiver noise floor. HDL weights match the
floating-point paper formula to 2·10⁻⁵ (Q16.16 resolution).

The third case is the main practical limit of this method: **the null is exactly where `jam_angle` points.**
A 1° error leaves about -34 dB at the real jammer. Null depth on hardware therefore depends on how accurate
the angle source is, and on RX channel calibration (amplitude and phase mismatch shift the null in the same way).

### Run

```bash
cd HDL/sim
iverilog -g2012 -o tb.vvp ../steering_vector_2x2.v ../sdiv_q16_serial.v \
         ../lcmv_dc_weight_2ch.v ../lcmv_beamformer_2ch.v ../lcmv_crpa_core.v tb_lcmv_crpa_core.v
vvp -n tb.vvp

cd ../model && python3 lcmv_dc_model.py      # needs numpy
```

For Vivado, add `gnss_passthrough.v`, `lcmv_crpa_core.v`, `lcmv_dc_weight_2ch.v`, `sdiv_q16_serial.v`,
`steering_vector_2x2.v` and `lcmv_beamformer_2ch.v`. No IP cores are needed.

---

## 5. Limitations

1. **No null widening with 2 antennas.** The paper's derivative constraints need M ≥ 4 for one jammer.
2. **One jammer** (M - 1 = 1).
3. **Angle accuracy.** Integer-degree angles from the PS. The null depth at the real jammer is limited by the
   angle error (about -34 dB for 1°, about -40 dB for 0.5°).
4. **Channel calibration.** RX1/RX2 phase and amplitude mismatch moves the null away from `jam_angle`. Calibrate
   the AD9361 RX paths (the ACES 2019 paper reports >20 dB improvement from channel equalization).
5. **ADC clipping.** A clipped jammer cannot be nulled. Keep the jammer below full scale (AGC/manual gain).
