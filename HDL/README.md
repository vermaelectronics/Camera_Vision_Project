# 2-Antenna CRPA Anti-Jamming Core: LCMV Power-Inversion (LCMV-PI)

FPGA (Verilog) anti-jamming core for an ANTSDR E310 (AD9361, 2 RX channels) GNSS receiver.
It replaces the earlier angle-steered LCMV beamformer with the **LCMV power-inversion** algorithm from:

> F. Wang, H. Liu, C. Zhang, "Anti-Jamming Front-End Design of Satellite Navigation Receiver,"
> *ACES Journal*, vol. 34, no. 11, pp. 1739-1749, Nov. 2019. (Section II-B, eq. 5-9)

---

## 1. Algorithm

LCMV criterion (paper eq. 5):

```
min_w  w^H R w     subject to   w^H C = f
w_opt = R^-1 C (C^H R^-1 C)^-1 f                          (eq. 7)
```

Power inversion (paper eq. 8-9): `C = s0 = [1 0 ... 0]^T`, `f = 1`

```
w_opt = R^-1 s0 / (s0^H R^-1 s0)                          (eq. 9)
```

For 2 antennas (R is 2x2 Hermitian) this reduces exactly to:

```
w1 = 1
w2 = -conj(R12) / R22'          R22' = R22 + (R11 + R22) >> DL_SHIFT + 1   (diagonal loading)

y  = w^H x = x1 - (R12 / R22') * x2
```

* No steering vector and no GPS/jammer angle are needed. The null is placed on the
  **measured** jammer direction.
* Null depth follows the jammer-to-noise ratio. The stronger the jammer, the deeper the null,
  as in the paper's Fig. 5.
* The weights are recomputed every covariance block (1024 samples), so the null follows a moving jammer.

### Why the old design was replaced

The old core used `C = [a(gps) a(jam)]`, `f = [1 0]`. With 2 antennas and 2 constraints, C is square and
`w = C^-H f`, so **R had no effect on the weights**. The null sat at the angle written by the PS. A 1° angle error
gives only about -34 dB at the true jammer (see `sim/lcmv_pi_null_pattern.png`, red curve).

---

## 2. File list

| File | Status | Description |
|---|---|---|
| `lcmv_crpa_core.v` | **rewritten** | Top of the LCMV-PI core: covariance, then PI weights, then beamformer. Same ports as before. |
| `lcmv_pi_weight_2ch.v` | **new** | Solves `w2 = -conj(R12)/R22'` using two parallel 64-bit serial dividers (68 clocks), diagonal loading, Q16.16 saturation. |
| `covariance_2antenna.v` | unchanged | Pipelined 2x2 covariance estimate (1024 samples, 64-bit accumulators). |
| `lcmv_beamformer_2ch.v` | unchanged | `y = conj(w1) x1 + conj(w2) x2`, 4-stage pipeline. |
| `gnss_passthrough.v` | comments only | AXI-Lite + datapath top. No functional change. |
| `sim/tb_lcmv_crpa_core.v` | **new** | Self-checking testbench (static and moving jammer). |
| `sim/lcmv_pi_null_pattern.png` | **new** | Beam pattern of the simulated weights compared with the old LCMV. |
| `model/lcmv_pi_model.py` | **new** | Python floating-point reference model and plot script. |

Removed (no longer needed): `steering_vector_2x2.v`, `d_inverse_2x2.v`, `lcmv_r_inverse_c_2x2.v`,
`lcmv_b_e_inverse_2x2.v`, `lcmv_weight_vector_2x2.v`. The originals are kept in `original_HDL/` in the zip package
and in git history. The original design also instantiated `c_h_b_matrix_2x2`, which was **missing** from the upload.

---

## 3. Interface (unchanged)

```
lcmv_crpa_core #(
    IQ_WIDTH    = 16,
    ACC_WIDTH   = 64,
    COV_WIDTH   = 32,     // kept for compatibility, unused
    COV_SAMPLES = 1024,   // covariance block length (power of 2)
    COV_SHIFT   = 10,     // log2(COV_SAMPLES)
    DL_SHIFT    = 16      // diagonal loading = trace(R) >> DL_SHIFT  (new, optional)
)
```

| Port | Meaning |
|---|---|
| `enable` | 1 = continuous LCMV-PI. 0 = disabled (covariance and solver held in reset, `weights_valid = 0`). |
| `rx_valid, rx1_i/q, rx2_i/q` | Antenna samples (antenna 1 = reference element, weight fixed to 1). |
| `gps_angle, jam_angle` | **Ignored** (power inversion needs no angles). AXI registers 0x40/0x44 remain for SW compatibility. |
| `w1_*, w2_*` | Current weights, Q16.16. `w1 = 1.0 = 65536`. |
| `y_re, y_im, beam_valid` | Beamformer output (integer ADC scale). |
| `busy, weights_valid` | Status. |

### Timing
* Weight update period: `COV_SAMPLES` valid samples + about 70 clocks. The next covariance block starts as soon as the previous one finishes.
* The old weights stay active while the new ones are calculated (atomic update).

### Fixes compared with the original core
* The covariance was connected through 32-bit wires and truncated from 64 bits, so a strong jammer could overflow.
  It now uses full `ACC_WIDTH`.
* The covariance block and the solver are reset while `enable = 0`, so every enable starts from fresh data.

---

## 4. Simulation results (Icarus Verilog 12)

Setup: 2-element ULA, d = λ/2, noise-like jammer, JNR ≈ 46 dB, 12-bit ADC (no clipping).

| Block | Jammer | Null found at | Null depth | Output / input power |
|---|---|---|---|---|
| 0-3 | 40° fixed | 40.00° | -74 … -86 dB | -42 … -43 dB |
| 4-7 | moved to 46° | 46.00° | -72 … -88 dB | -43 dB |

At -43 dB output/input, the jammer is cancelled down to the receiver noise floor. **Result: PASS.**

### Run the testbench

```bash
cd HDL/sim
iverilog -g2012 -o tb.vvp ../covariance_2antenna.v ../lcmv_pi_weight_2ch.v \
         ../lcmv_beamformer_2ch.v ../lcmv_crpa_core.v tb_lcmv_crpa_core.v
vvp -n tb.vvp
```

Top-level compile check: `iverilog -g2012 -s gnss_passthrough HDL/*.v`

In Vivado, add the four RTL files plus `gnss_passthrough.v` as before. No IP cores are needed.

---

## 5. Limitations and next steps

1. **Null width.** 2 antennas give 1 degree of freedom, so there is 1 null, and its width is fixed by geometry
   (about 3° at -30 dB). Null widening (covariance-matrix taper, virtual jammers, derivative constraints) needs
   **≥ 3 antennas**. With 2 antennas it only makes the null shallower. For **wide and deep** nulls use a
   4-element (drone CRPA) or 7-element (paper) array. The PI solver then becomes an N×N solve of `R w = s0`.
2. **Max jammers = N-1 = 1** for 2 antennas.
3. **ADC saturation.** A clipped jammer cannot be nulled (the test showed only -40 dB when clipping). Keep the AD9361
   AGC/manual gain so the jammer stays below full scale (paper §II-C recommends multi-stage AGC).
4. **Channel mismatch.** The paper applies channel equalization with a calibration channel (Table 1, >20 dB CR
   improvement). Amplitude and phase mismatch between RX1 and RX2 limits the real null depth. Calibrate the E310 RX channels.
5. **GPS gain.** PI does not steer a beam to the satellites. GPS signals see the array pattern outside the null.

---

## 6. Related LCMV papers (for future N-element work)

* Gershman, Nickel, Böhme, "Adaptive beamforming algorithms with robustness against jammer motion," IEEE TSP 45(7), 1997.
  Derivative-constraint LCMV null broadening.
* "A Robust GNSS Interference Suppression Method Based on Null Broadening of Dual-polarized Antenna Arrays." LCMV + projection null broadening/deepening.
* Gong et al., "A Robust Method to Suppress Jamming for GNSS Array Antenna Based on Reconstruction of Sample Covariance Matrix," IJAP 2017.
* Zhang et al., "Impact Analysis of Orthogonal Circular-Polarized Interference on GNSS Spatial Anti-Jamming Array," Remote Sensing 16, 2024. PI and LCMV.
