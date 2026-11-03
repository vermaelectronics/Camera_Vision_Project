# WL-STAP-PI v5.0.3: the anti-jam algorithm of this project

**WL-STAP-PI** = **Notch-aided Widely-Linear Space-Time Adaptive Processing,
Power Inversion**, adapted by NLMS every sample in the FPGA.

| | |
|---|---|
| Project folder | `LoopBack_Code_WL_STAP_PI` |
| Release | 5.0.3 (RTL revision 5.0.3, VERSION register `0x00050003`, firmware 5.0.3) |
| Board | AntSDR E310: AD9361 (RX1, RX2 -> TX1) + Zynq-7020, Vivado/Vitis 2023.2 |
| FPGA block | `pi_aj_0` at `0x43D0_0000`, IP `antsdr:gnss:pi_aj:5.0`, on `sys_cpu_clk` 100 MHz |
| Replaces | SMI-PI v4.5.0 (`smi_pi_0`, sample-matrix-inversion weights computed on the ARM) |

The internal file, module, IP and register names keep the prefix `pi_aj` /
`PIAJ`. The block design, the firmware (`XPAR_PI_AJ_0_BASEADDR`) and the build
scripts depend on these names, so they were not renamed. "WL-STAP-PI" is the
name of the algorithm. "pi_aj" is the name of its hardware block.

---

## 1. What it is

A 2-antenna GNSS anti-jam filter (CRPA with 2 elements), in three parts:

1. **CW notch** (self-tuned) and **I/Q-image notch**: remove narrowband (CW)
   jammers in frequency. The same filter is applied to RX1 and RX2, so the
   direction information between the antennas is preserved.
2. **WL-STAP-PI**: widely-linear space-time adaptive power inversion. It nulls
   what is left (wideband or barrage jammer, residual CW, multipath) in space
   and frequency.
3. **Output make-up gain** `x 2^shift` to TX1, so that after a deep null the
   remaining GNSS + noise is not lost in DAC quantisation.

```
 RX1 ─┐
      ├─ frontend ─ CW notch ─ image notch ─ WL-STAP power inversion ─ x 2^shift ─ TX1
 RX2 ─┘  (scale,    (self-     (conj of      (2 ch x 5 taps + conj taps,
          DC block)  tuned)     notch coef)   19 weights, NLMS every sample)
```

### Why each part is there

* **STAP (space-time)**: each antenna has a 5-tap delay line, so the weights
  depend on frequency. This nulls a wideband jammer across the whole band, and
  absorbs the RX1/RX2 analog filter mismatch and the jammer's own multipath.
* **Widely-linear (WL)**: each AD9361 receiver has its own I/Q imbalance, so
  every jammer also has an *image* at the mirror frequency with a *different*
  spatial signature. A plain 2-element STAP has one degree of freedom and
  cannot null both. The conjugate taps supply the missing degrees of freedom.
* **Power inversion (PI)**: minimise the output power, with the RX1 centre-tap
  weight fixed to 1. It needs no satellite directions or steering vectors.
  Jammers (far above the noise) are nulled, and GNSS (below the noise) passes.
* **Notch-aided**: 2 antennas give only **one** spatial null. The notches remove
  CW jammers in frequency first, at no spatial cost, so the null stays available
  for the wideband jammer.

---

## 2. Equations

### 2.1 CW notch (`pi_aj_notch.v`, MODE 0)

```
 y[n] = x[n] - a·x[n-1] + ρ·a·y[n-1]          ρ = 1 - 2^-rho_sh
```
The zero is on the unit circle at angle(a) (the jammer frequency) and the pole
is at radius ρ. The 3 dB width is about `fs · 2^-rho_sh / π` (rho_sh = 6: about
90 kHz at 18 MS/s).
One coefficient `a` is applied to both RX1 and RX2.

The coefficient tracks the strongest line of RX1:
```
 r1 <- r1 + (x0[n]·conj(x0[n-1]) - r1) · 2^-r_sh       lag-1 autocorrelation
 a  <- a + norm(r1) · 2^-a_sh                           norm = power-of-two scaling
 a  <- a · (1.5 - 0.5·|a|^2)                            Newton step back to |a| = 1
```
The notch switches itself in only while it removes power:
ON if `P_in > 1.25·P_out`, OFF if `P_in < 1.125·P_out`.

### 2.2 I/Q-image notch (`pi_aj_notch.v`, MODE 1)

The same filter with `a_img = conj(a)`, so it sits on the mirror frequency. It is
active whenever the CW notch is active.

### 2.3 WL-STAP power inversion (`pi_aj_wlpi.v`)

Generalised sidelobe canceller form of power inversion (L = 5 taps, centre
tap D = 2):
```
 d[n] = x0[n-D]                                   reference: RX1 centre tap, weight 1
 u[n] = [ x0[n-k], k≠D ;  x1[n-k] ;               linear part        (4 + 5 = 9)
          conj(x0[n-k]) ;  conj(x1[n-k]) ]        widely-linear part (5 + 5 = 10)
                                                  k = 0..4  ->  19 weights
 y[n] = d[n] - w^H u[n]                           array output (TX1, before 2^shift)
 w   <- w + μ · u[n-Q] · conj(y[n-Q]) / (2^⌊log2 P⌋ · 2^C)          (NLMS)
```
* `P` = smoothed `|x0|^2 + |x1|^2`, rising fast. The normalisation is a
  power of two, so no divider is needed and convergence speed does not depend on
  the jammer level.
* `μ = 2^-mu_sh`, `C = log2(8L) = 6`, and `Q = DELAY = 4` samples (delayed update,
  so the multipliers can be time-shared at 100 MHz).
* auto-μ: while a CW is notched, the step is halved (mu_sh + 1).

### 2.4 Output (`pi_aj_outagc.v`)

```
 TX1 = sat16( y[n] · 2^shift )         shift = ADAPT[24:20], set by the firmware
                                        (optional automatic 6 dB-step gain control)
```

---

## 3. Files of the method

| File (exact name) | Part of WL-STAP-PI |
|---|---|
| `Source/HDL/pi_aj/pi_aj_notch.v` | CW notch (2.1) and I/Q-image notch (2.2) |
| `Source/HDL/pi_aj/pi_aj_wlpi.v` | **WL-STAP power inversion, NLMS** (2.3) |
| `Source/HDL/pi_aj/pi_aj_frontend.v` | input scaling, DC block, clip blanking |
| `Source/HDL/pi_aj/pi_aj_outagc.v` | output make-up gain (2.4) |
| `Source/HDL/pi_aj/pi_aj_top.v` | the WL-STAP-PI core (chain + monitors) |
| `Source/HDL/pi_aj/pi_aj_axis.v` | AXI-Stream wrapper, block `pi_aj_0` |
| `Source/HDL/pi_aj/pi_aj_axil_regs.v` | registers (AXI-Lite) |
| `Source/HDL/pi_aj/pi_aj_cdc_bus.v` | clock-domain crossing |
| `Source/IP/pi_aj/pi_aj_ip.tcl` | Vivado IP packaging |
| `Source/Firmware/app_gnss_e310/gnss_piaj.c/.h` | firmware driver of the WL-STAP-PI block |
| `Source/Firmware/app_gnss_e310/gnss_aj.c/.h` | jammer detection, CLEAR/JAMMED, switches WL-STAP-PI in/out |
| `Tests/pi_aj/model.py` | floating-point reference model |
| `Tests/pi_aj/selftest.sh` | RTL self-test (build step 3) |
| `Automation/Linux/steps/03_piaj.sh` | build step 3: self-test + IP |

---

## 4. Parameters (register ADAPT, `0x0C`)

| Bits | Name | Default | Meaning |
|---|---|---|---|
| [3:0] | mu | 0 | NLMS step 2^-mu (auto +1 while a CW is notched) |
| [11:8] | rho | 6 | notch width ~ fs·2^-rho/π (7 above 20 MS/s) |
| [15:12] | r | 12 | autocorrelation smoothing 2^-r |
| [19:16] | a | 8 | notch tracking step 2^-a |
| [24:20] | shift | firmware | TX1 make-up gain 2^shift |

Build parameters of `pi_aj_axis`: L = 5 taps, NL = 10 multiplier lanes, DELAY = 4,
widely-linear on, no second CW notch, 3 clocks per sample at 100 MHz, so the
**maximum sample rate is 33.3 MS/s** (L1 profile 30.72 MS/s).

---

## 5. Results (RTL simulation of this exact Verilog, AntSDR configuration)

| Scenario | TX1 C/N0 (dB-Hz) |
|---|---|
| no jammer (reference) | 48.5 |
| CW 1575.40 MHz, J/N 61 dB ("30 dB" setting) | 46.4 |
| **CW 1575.40 MHz, J/N 71 dB ("20 dB" setting)** | **42.1** (about 69 dB jammer suppression) |
| CW 1575.40 MHz, J/N 76 dB ("15 dB" setting) | 38.6 |
| CW exactly on 1575.42 MHz (DC), J/N 71 dB | 38.8 |
| barrage 18 MHz, J/N 40 dB | 50.8 |
| barrage 18 MHz, J/N 50 dB | 47.7 |
| barrage 2 MHz, J/N 50 dB | 36.1 |

FPGA resources (yosys estimate): 138 DSP48 of 220, about 19.1k LUT, 13.8k FF.

---

## 6. Limits

* 2 antennas give **one** spatial null. One broadband jammer direction can be
  nulled. CW jammers are taken by the notches and do not use it.
* Power inversion gives no array gain toward the satellites (no steering).
* Maximum 33.3 MS/s at 100 MHz (profiles `gnss_band=0, 4, 5`). Above that, the
  filter runs in bypass.
* The numbers above are from simulation. Expect the field a few dB worse
  (AD9361 spurs, LO leakage).

## 7. Recognising a WL-STAP-PI build

* Build step 3 prints `PI_AJ_RTL: source 5.0.3  packaged IP 5.0.3`.
* At boot the console prints `ANTI-JAM (WL-STAP-PI v5.0.3): ON, bypassed until a jammer appears.`
* `gnss_piaj?` prints `--- WL-STAP-PI v5 @ 0x43d00000 (core 5.0.3) ---`.
* In Vivado the IP is shown as "WL-STAP-PI v5 anti-jam (notch + widely-linear
  space-time power inversion)".
