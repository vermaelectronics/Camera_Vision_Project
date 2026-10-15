# ESPRIT Direction-of-Arrival Monitor — ANTSDR E310 V1 LoopBack Pipeline

**Modification MOD-11 · gnss_passthrough v1.2 · firmware `gnss_doa.c`**

This document covers the ESPRIT direction-of-arrival (DOA) estimator added to
the ANTSDR E310 V1 GNSS CRPA LoopBack pipeline. It contains:

- the mathematics
- what was changed in the design and why
- how to verify the design in simulation, in Vivado and on the hardware, with the console output to expect at every step

It is based on:

> J. Li, G. Yang, X. Ren, Z. Fu, S. Qiu, *"Modified ESPRIT and
> Unitary-Transformation Form for Computationally Efficient DOA Estimation on
> FPGA"*, IEEE Transactions on Instrumentation and Measurement, vol. 74, 2025,
> Art. no. 2003209, DOI 10.1109/TIM.2025.3545700.

---

## Contents

1. [What was added — summary](#1-what-was-added--summary)
2. [Mathematics of ESPRIT](#2-mathematics-of-esprit)
3. [What this paper changes (M-ESPRIT and I-UESPRIT)](#3-what-this-paper-changes-m-esprit-and-i-uesprit)
4. [Closed forms for the two-antenna CRPA baseline](#4-closed-forms-for-the-two-antenna-crpa-baseline)
5. [Hardware architecture and integration](#5-hardware-architecture-and-integration)
6. [Register map](#6-register-map)
7. [Build flow (Vivado / Vitis)](#7-build-flow-vivado--vitis)
8. [Verification in simulation (Vivado Tcl console)](#8-verification-in-simulation-vivado-tcl-console)
9. [Verification on hardware (PuTTY console)](#9-verification-on-hardware-putty-console)
10. [JTAG test without the console (XSDB)](#10-jtag-test-without-the-console-xsdb)
11. [Console command reference](#11-console-command-reference)
12. [Resources, timing, limits and troubleshooting](#12-resources-timing-limits-and-troubleshooting)
13. [File-by-file change list](#13-file-by-file-change-list)

---

## 1. What was added — summary

| Item | Where | Notes |
|---|---|---|
| ESPRIT DOA monitor (I-UESPRIT **and** M-ESPRIT in parallel) | `Source/HDL/gnss_doa_esprit.v`, `Source/HDL/esprit/*.v` | Instantiated **inside** `gnss_passthrough` as `u_doa`. |
| Dual-clock FIFO l_clk → sys_cpu_clk | `Source/HDL/doa_async_fifo.v` | Gray-pointer CDC; constraints in `Source/XDC/gnss_passthrough_cdc.xdc` §4. |
| Built-in two-element test source | `Source/HDL/doa_test_gen.v` | Known-angle hardware test without RF. |
| RX2 phase calibration, decimation, hardware averaging, peak meter | `gnss_doa_esprit.v` | |
| Registers 0x100–0x194 | `gnss_passthrough` aperture, **0x43C0_0100** | Block design **unchanged** (same ports, same address). |
| Firmware driver + self-test + 18 console commands | `gnss_doa.c/.h`, `command.c/.h`, `main.c`, `gnss_info.c/.h` | Boot self-test, `doa?`, `doa_cal=1`, … |
| System testbench (61 checks) | `Source/Sim/ESPRIT/tb_gnss_doa.v` | Runs from the **Vivado Tcl console**. |
| Firmware ↔ RTL co-simulation | `Source/Sim/ESPRIT/fw_cosim/` | Runs the real `gnss_doa.c` against the real RTL. |
| Vivado post-implementation report | `Vivado/Scripts/report_doa.tcl` | Utilisation, timing, CDC coverage. |
| JTAG test | `Vitis/Scripts/doa_jtag_test.tcl` | XSDB, no console needed. |
| Standalone ESPRIT IP cores + their regressions | `Source/IP/ESPRIT_standalone/` | Core-level verification against a Python model. |

**Design rule kept: the DOA block is receive-only.** It taps RX1/RX2. It never
touches the RX→TX elastic buffers, the DAC mux or the TX alignment. With
`DOA_CTRL.enable = 0` it is held in reset and the bitstream behaves exactly
like v1.1. The system testbench checks this (T8).

---

## 2. Mathematics of ESPRIT

### 2.1 Signal model

A uniform linear array (ULA) has $M$ elements spaced $d$ apart. $K$ narrow-band,
uncorrelated far-field sources arrive at angles $\theta_k$, measured from
broadside. The snapshot at time $t$ is

$$
\mathbf{x}(t) = \mathbf{A}\,\mathbf{s}(t) + \mathbf{n}(t),\qquad
\mathbf{A} = [\mathbf{a}(\theta_1),\dots,\mathbf{a}(\theta_K)] \in \mathbb{C}^{M\times K}
$$

with the steering vector (paper Eq. (2))

$$
\mathbf{a}(\theta) = \big[e^{j\phi_0(\theta)},\, e^{j\phi_1(\theta)},\dots,e^{j\phi_{M-1}(\theta)}\big]^T,
\qquad \phi_m(\theta) = -\frac{2\pi m d}{\lambda}\sin\theta .
$$

For this board, $M = 2$: RX1 is element 0 (the reference) and RX2 is element 1.
The single phase that carries the angle is

$$
\boxed{\;\phi = \phi_1 = -\frac{2\pi d}{\lambda}\sin\theta \;}
\qquad\Longleftrightarrow\qquad
\theta = \arcsin\!\Big(-\frac{\lambda}{2\pi d}\,\phi\Big) = \arcsin(-k\,\phi),
\quad k = \frac{\lambda}{2\pi d}.
$$

For $d = \lambda/2$, $k = 1/\pi$, and $\phi \in (-\pi, \pi]$ maps one-to-one onto
$\theta \in [-90^\circ, 90^\circ]$. At GPS L1 (1575.42 MHz), $\lambda = 190.3$ mm,
so $\lambda/2 = 95.15$ mm.

### 2.2 Classic ESPRIT (Roy & Kailath, 1989)

1. **Covariance:** $\mathbf{R}_{xx} = E[\mathbf{x}\mathbf{x}^H] = \mathbf{A}\mathbf{R}_s\mathbf{A}^H + \sigma^2\mathbf{I}$,
   estimated as $\hat{\mathbf{R}} = \frac{1}{T}\sum_t \mathbf{x}(t)\mathbf{x}^H(t)$.
2. **Signal subspace:** take the eigen-decomposition (EVD)
   $\mathbf{R}_{xx} = \mathbf{U}_s\boldsymbol{\Lambda}_s\mathbf{U}_s^H + \mathbf{U}_n\boldsymbol{\Lambda}_n\mathbf{U}_n^H$
   (Eq. (5)). $\mathbf{U}_s$ holds the eigenvectors of the $K$ largest
   eigenvalues, **so $K$ must be known**. They span the same space as $\mathbf{A}$:
   $\mathbf{U}_s = \mathbf{A}\mathbf{W}$ (Eq. (6)).
3. **Shift invariance:** with the selection matrices $\mathbf{J}_1 = [\mathbf{I}\ \mathbf{0}]$ and
   $\mathbf{J}_2 = [\mathbf{0}\ \mathbf{I}]$, which pick the first and the last $M-1$ rows,
   $$\mathbf{J}_2\mathbf{A} = \mathbf{J}_1\mathbf{A}\boldsymbol{\Phi},\qquad
     \boldsymbol{\Phi} = \mathrm{diag}\big(e^{j\phi_1(\theta_1)},\dots,e^{j\phi_1(\theta_K)}\big)\quad\text{(Eqs. (3), (4))}.$$
4. **Rotation:** set $\mathbf{U}_{s1} = \mathbf{J}_1\mathbf{U}_s$ and $\mathbf{U}_{s2} = \mathbf{J}_2\mathbf{U}_s$, then
   $\boldsymbol{\Psi} = \mathbf{U}_{s1}^{\dagger}\mathbf{U}_{s2} = \mathbf{W}^{-1}\boldsymbol{\Phi}\mathbf{W}$.
   **A second EVD** of $\boldsymbol{\Psi}$ gives the eigenvalues $e^{j\phi_1(\theta_k)}$.
5. **Angle:** $\theta_k = \arcsin\big(-\frac{\lambda}{2\pi d}\arg\psi_k\big)$.

The FPGA cost of classic ESPRIT is two complex EVDs, plus prior knowledge
(or estimation) of $K$.

---

## 3. What this paper changes (M-ESPRIT and I-UESPRIT)

### 3.1 M-ESPRIT: subspace from a fourth-order cumulant, no source number (Sec. III-A)

**Cumulant matrix (Eq. (7)).** Each element is

$$
\mathbf{C}(\bar m,\bar n) = \mathrm{cum}\{x_m, x_0^*, x_0, x_n^*\}
= E[x_m x_0^* x_0 x_n^*] - E[x_m x_0^*]E[x_0 x_n^*] - E[x_m x_0]E[x_0^* x_n^*] - E[x_m x_n^*]E[x_0^* x_0].
$$

For independent sources with fourth-order cumulants $c_{s_k}$ (Eqs. (8)–(10)):

$$
\mathbf{C} = \mathbf{A}\mathbf{C}_s\mathbf{A}^H = \mathbf{A}\mathcal{T},\qquad
\mathbf{C}_s = \mathrm{diag}(c_{s_1},\dots,c_{s_K}).
$$

So **$\mathbf{C}$ itself spans the signal subspace**, without any eigenvectors
and without $K$. A Gaussian process has a zero fourth-order cumulant, so the
receiver noise drops out of $\mathbf{C}$. A constant-modulus tone or a QPSK
symbol stream has $c_s = -|s|^4 \neq 0$.

**Rank-deficient rotation (Eqs. (11)–(19)).** With $\mathbf{C}_1 = \mathbf{J}_1\mathbf{C} = \mathbf{A}_1\mathcal{T}$
and $\mathbf{C}_2 = \mathbf{J}_2\mathbf{C} = \mathbf{A}_1\boldsymbol{\Phi}\mathcal{T}$, $\mathbf{C}_1$ has rank $K < M$. The ordinary
pseudo-inverse therefore does not exist, and the paper regularises it:

$$
\mathbf{P}_{C_1} = (\mathbf{C}_1^H\mathbf{C}_1 + \mu\mathbf{I})^{-1}\mathbf{C}_1^H,\qquad
\boldsymbol{\Psi} = \mathbf{P}_{C_1}\mathbf{C}_2 \approx \mathcal{T}^{\dagger}\boldsymbol{\Phi}\mathcal{T}.
$$

$\boldsymbol{\Psi}$ has $M$ eigenvalues. $K$ of them lie on the unit circle
($e^{j\phi_1(\theta_k)}$), and the rest lie near the origin. A source is
therefore identified by $|\psi| \approx 1$ (**Remark A**). The angle is given by Eq. (21):

$$
\theta_i = \arcsin\!\Big(-\frac{\lambda}{2\pi d}\arg(\psi_i)\Big).
$$

### 3.2 I-UESPRIT: real-valued unitary form (Sec. III-B)

* Exchange matrix $\boldsymbol{\Pi}$ (ones on the anti-diagonal, Eq. (22)).
* Unitary matrix $\mathbf{Q}$, defined separately for even and odd sizes (Eqs. (23), (24)):
  $$\mathbf{Q}_{2n} = \tfrac{1}{\sqrt2}\begin{bmatrix}\mathbf{I} & j\mathbf{I}\\ \boldsymbol{\Pi} & -j\boldsymbol{\Pi}\end{bmatrix}.$$
* Forward–backward averaging (Eq. (26)): $\mathbf{C}^{fb} = \tfrac12(\mathbf{C} + \boldsymbol{\Pi}\mathbf{C}^*\boldsymbol{\Pi})$.
* Real cumulant matrix (Eqs. (25), (28)): $\mathbf{C}_r = \mathbf{Q}^H\mathbf{C}^{fb}\mathbf{Q} = \mathbf{B}\,\mathrm{Re}\{\mathbf{C}_s\}\mathbf{B}^H \in \mathbb{R}^{M\times M}$, with $\mathbf{B} = \mathbf{Q}^H\mathbf{A}$.
* Real shift invariance (Eqs. (29)–(32)): $\mathbf{K}_1\mathbf{B}\boldsymbol{\Phi}_r = \mathbf{K}_2\mathbf{B}$, where
  $$\mathbf{K}_1 = 2\,\mathrm{Re}\{\mathbf{Q}_{M-1}^H\mathbf{J}_2\mathbf{Q}_M\},\quad
    \mathbf{K}_2 = 2\,\mathrm{Im}\{\mathbf{Q}_{M-1}^H\mathbf{J}_2\mathbf{Q}_M\},\quad
    \boldsymbol{\Phi}_r = \mathrm{diag}\big(\tan(\phi_1(\theta_k)/2)\big).$$
* $\mathbf{C}_{r1} = \mathbf{K}_1\mathbf{C}_r$, $\mathbf{C}_{r2} = \mathbf{K}_2\mathbf{C}_r$, $\boldsymbol{\Psi}_r = (\mathbf{C}_{r1}^T\mathbf{C}_{r1} + \mu\mathbf{I})^{-1}\mathbf{C}_{r1}^T\mathbf{C}_{r2}$ (Eqs. (33)–(37)).
* Angle (Eq. (35)): $\theta_k = \arcsin\!\big(-\frac{\lambda}{\pi d}\arctan\psi_{r_k}\big)$, since $\phi = 2\arctan\psi_r$.

All the arithmetic is real, and only one real EVD is needed. The FPGA cost is
much lower than classic ESPRIT.

### 3.3 The $\mathbf{C}^p$ improvement (Eq. (38), Remarks C–E)

Let $\mathbf{C} = \mathbf{U}_c\boldsymbol{\Sigma}_c\mathbf{U}_c^{-1}$. Then
$\mathbf{C}^p = \mathbf{U}_c\boldsymbol{\Sigma}_c^p\mathbf{U}_c^{-1}$ (Eq. (40)) spans the same column space,
$\mathbf{C}^p = \mathbf{A}\mathcal{T}_p$ (Eq. (41)). Raising to a power widens the gap
between the signal eigenvalues and the residual noise eigenvalues
($\sigma_k^p$ vs. $\sigma_{K+1}^p$). The noise leaking into the subspace estimate
therefore falls as $p$ grows. The paper uses $p = 4$, which is the default here
(`doa_p=2`, because $p = 2^2$).

### 3.4 Which estimator is better?

The paper (Fig. 4) finds M-ESPRIT more accurate near end-fire, because the
$\tan(\phi/2)$ mapping of I-UESPRIT is steep there, and I-UESPRIT cheaper. My
Monte-Carlo comparison (earlier in this project) found:

* $M = 2$: identical accuracy.
* $M \ge 3$: M-ESPRIT with $p = 4$ is clearly better. I-UESPRIT's real
  eigenvalues near 0 cannot be told apart from spurious ones (no unit-circle
  test), so it struggles near broadside and with closely spaced sources.

Both run side by side in this design, so the comparison can be made on the
board.

---

## 4. Closed forms for the two-antenna CRPA baseline

With $M = 2$ the matrices are small enough to solve symbolically. In
particular, $\boldsymbol{\Psi}$ is **rank one**, so its only non-zero eigenvalue
equals its trace and **no iterative EVD is needed**.

### 4.1 Cumulant matrix for M = 2

Define the window averages

$$
R_{mn} = E[x_m x_n^*],\qquad P_m = E[x_m x_0],\qquad M4_{mn} = E\big[|x_0|^2 x_m x_n^*\big].
$$

Then Eq. (7) becomes

$$
\begin{aligned}
c_{00} &= M4_{00} - 2R_{00}^2 - |P_0|^2,\\
c_{11} &= M4_{11} - |R_{01}|^2 - |P_1|^2 - R_{11}R_{00},\\
c_{01} &= M4_{01} - 2R_{00}R_{01} - P_0P_1^*,\qquad c_{10} = c_{01}^*.
\end{aligned}
$$

Write $\mathbf{C} = \begin{bmatrix} a & b\\ b^* & d\end{bmatrix}$ with $a, d$ real and $b$ complex.

### 4.2 $\mathbf{C}^p$ by repeated squaring

$$
\begin{bmatrix} a & b\\ b^* & d\end{bmatrix}^2 =
\begin{bmatrix} a^2+|b|^2 & (a+d)\,b\\ (a+d)\,b^* & d^2+|b|^2\end{bmatrix}
$$

This takes six real products per squaring, and $p = 2^n$ needs $n$ squarings.
Every quantity after this step is invariant to a common positive scale factor.
The hardware therefore renormalises with a power-of-two block exponent after
every step, so the largest entry lies in $[0.5, 1)$.

### 4.3 M-ESPRIT closed form

$\mathbf{C}_1 = [a,\ b]$ and $\mathbf{C}_2 = [b^*,\ d]$. Because $\mathbf{C}_1^H\mathbf{C}_1$ has rank one with
eigenvector $\mathbf{C}_1^H$:

$$
\mathbf{P}_{C_1} = \frac{\mathbf{C}_1^H}{\|\mathbf{C}_1\|^2 + \mu},\qquad
\boldsymbol{\Psi} = \frac{\mathbf{C}_1^H\mathbf{C}_2}{\|\mathbf{C}_1\|^2+\mu},\qquad
\boxed{\;\psi_M = \operatorname{tr}\boldsymbol{\Psi} = \frac{(a+d)\,b^*}{a^2 + |b|^2 + \mu}\;}
$$

For an ideal single source, $\mathbf{C} = c\begin{bmatrix}1 & e^{-j\phi}\\ e^{j\phi} & 1\end{bmatrix}$, which gives
$\psi_M = e^{j\phi}$, exactly on the unit circle. The denominator is real and
positive, so

$$
\phi = \arg\psi_M = \operatorname{atan2}\big(\mathrm{Im}\{(a+d)b^*\},\ \mathrm{Re}\{(a+d)b^*\}\big),
\qquad |\psi_M| = \frac{|a+d|\,|b|}{a^2+|b|^2+\mu}.
$$

### 4.4 I-UESPRIT closed form

$\mathbf{Q}_2 = \tfrac{1}{\sqrt2}\begin{bmatrix}1 & j\\ 1 & -j\end{bmatrix}$ and $\mathbf{Q}_1 = 1$. Then:

$$
\mathbf{C}^{fb} = \begin{bmatrix} A & b\\ b^* & A\end{bmatrix},\ A = \tfrac{a+d}{2};\qquad
\mathbf{C}_r = \mathbf{Q}^H\mathbf{C}^{fb}\mathbf{Q} = \begin{bmatrix} A+\mathrm{Re}\,b & \mathrm{Im}\,b\\ \mathrm{Im}\,b & A-\mathrm{Re}\,b\end{bmatrix}
$$

$$
\mathbf{K}_1 = \sqrt2\,[1\ \ 0],\quad \mathbf{K}_2 = \sqrt2\,[0\ \ {-1}],\quad
\mathbf{r} = [c_{r00},\ c_{r01}],\quad \mathbf{s} = -[c_{r10},\ c_{r11}]
$$

$$
\boxed{\;\psi_I = \frac{\mathbf{r}\cdot\mathbf{s}}{\|\mathbf{r}\|^2 + \mu'} \;}\qquad (\mu' = \mu/2),\qquad
\phi = 2\arctan\psi_I = 2\,\operatorname{atan2}\big(\mathbf{r}\cdot\mathbf{s},\ \|\mathbf{r}\|^2+\mu'\big).
$$

For the ideal single source this gives $\psi_I = \tan(\phi/2)$. The Python models verify
both closed forms against the paper's generic matrix equations with a full EVD:
the maximum differences are $10^{-14}$ (I-UESPRIT) and $2\cdot10^{-9}$ rad (M-ESPRIT).

### 4.5 From φ to θ (DOA recovery)

$$
u = -k\,\phi,\qquad \theta = \arcsin(u),\qquad k = \frac{\lambda}{2\pi d}\ (\texttt{DOA\_K\_Q30}).
$$

If $|u| > 1$ (spacing above $\lambda/2$, or noise near end-fire), $\theta$ is
clipped to $\pm 90^\circ$ and the `ambiguous` flag is set.

### 4.6 Calibration rotation

Every real board adds a fixed RX2-to-RX1 phase $\phi_{cal}$ (LO routing,
filters, cables, the AD9361 `rx1rx2_phase_inversion` setting). The hardware
removes it before estimation:

$$
x_1' = x_1\,(c + j s),\qquad c + js = e^{-j\phi_{cal}}\ (\text{Q15}),
$$

`doa_cal=1` measures $\phi_{cal}$ as the unit-vector mean of $\arg\psi$ over 64
windows, with the same signal on both inputs, and writes $e^{-j\phi_{cal}}$ to `DOA_CAL`.

### 4.7 Fixed-point formats

| Quantity | Format |
|---|---|
| AD9361 RX sample | 12-bit, right-aligned in 16 bits → left-aligned to Q1.15 |
| Second-order moments $R, P$ | Q2.30, (33 + 14)-bit accumulators |
| Fourth-order moments $M4$, cumulant | Q4.60, (66 + 14)-bit accumulators, 72-bit $\mathbf{C}$ |
| After normalisation | 32-bit block-floating mantissas, 34×34 products |
| $\phi$, $\theta$ | Q28 radians; $\theta$ also Q16 degrees |
| $\psi_I$, $\lvert\psi_M\rvert$ | Q8.24 |

The accumulators keep full precision, so a −40 dBFS input gives the same answer
as floating point (verified to within 0.0004°).

---

## 5. Hardware architecture and integration

```
                       gnss_passthrough v1.2  (block design instance, 0x43C0_0000)
 axi_ad9361 ADC ──┬──► elastic FIFO ─► align ─► DAC mux ──────────────────────────► axi_ad9361 DAC   (UNCHANGED)
 (l_clk, 12-bit) │
                 └──► u_doa : gnss_doa_esprit                         (RECEIVE-ONLY TAP)
                       l_clk │ capture RX1+RX2, decimate, <<4 (Q1.15)
                             ▼
                       doa_async_fifo (gray pointers) ──────────────── CDC ─────────
                       sys_cpu_clk 100 MHz
                             ▼
                       source mux ◄── doa_test_gen (known-angle synthetic array)
                             ▼
                       RX2 calibration rotation  x1·(c + j s)
                             ├──────────────► I-UESPRIT core ──┐
                             └──────────────► M-ESPRIT  core ──┤
                                                               ▼
                       result registers, averaging (mean, mean²), peak meter ──► AXI-Lite 0x100–0x194
```

* **Clocks.** Capture and decimation run in `l_clk` (rx_clk, 8 ns). Everything
  else runs in `s_axi_aclk` = sys_cpu_clk = 100 MHz, which is the timing target
  the ESPRIT cores were pipelined for. The register interface needs no extra
  crossing.
* **Throughput.** One snapshot per 100 MHz clock. At the vendor rate of
  30.72 MS/s, a 1024-snapshot window takes 33 µs. A result appears about 1.3 µs
  (I-UESPRIT) / 1.65 µs (M-ESPRIT) after each window, and an average of 16 is
  available every 0.53 ms.
* **Block design.** Unchanged: the ports, VLNV `antsdr:gnss:gnss_passthrough:1.0`,
  4 kB aperture and base 0x43C0_0000 are all the same. Only the packaged IP
  content changes, so **the IP must be re-packaged** (§7).
* **Build-time options** (IP parameters): `ENABLE_IUESPRIT`, `ENABLE_MESPRIT`
  (both 1 by default). Set either to 0 to save about 10 k LUT / 47 DSP.

---

## 6. Register map

Offsets are from `GNSS_PT_BASEADDR` (0x43C0_0000). The full field definitions
are in the header of `Source/HDL/gnss_doa_esprit.v`; the same map is in
`Source/Config/board_e310_v1.json`.

| Offset | Name | R/W | Reset | Description |
|---|---|---|---|---|
| 0x100 | DOA_ID | RO | 0x444F4131 | "DOA1" |
| 0x104 | DOA_VERSION | RO | 0x00010000 | 1.0 |
| 0x108 | DOA_CTRL | RW | 0x00040A20 | [0] enable, [1] src_test, [2] use_cov, [3] tg_noise, [5:4] p_log2, [12:8] log2_snap (7..14), [19:16] avg_log2 (0..10), [24] soft_reset, [25] clear_stats |
| 0x10C | DOA_MU_Q30 | RW | 1024 | μ of Eq. (17)/(37) |
| 0x110 | DOA_K_Q30 | RW | 341782638 | k = λ/(2πd), Q2.30 (1/π for λ/2) |
| 0x114 | DOA_UCTOL_Q24 | RW | 0x00400000 | M-ESPRIT unit-circle tolerance (0.25) |
| 0x118 | DOA_DECIM | RW | 0 | keep 1 of (N+1) snapshots |
| 0x11C | DOA_CAL | RW | 0x00007FFF | {sin_q15, cos_q15} RX2 rotation |
| 0x120 | DOA_TG_FTW | RW | 0x0147AE14 | test tone phase step |
| 0x124 | DOA_TG_DPHASE | RW | 0 | test RX2−RX1 phase, 2³² = 360° |
| 0x128 | DOA_TG_CFG | RW | 0x00400064 | [15:0] clk/sample (≥8), [19:16] amplitude shift, [23:20] noise shift |
| 0x12C | DOA_STATUS | RO | | [0] CDC overflow (sticky), [1] I built, [2] M built, [3] enable, [4] src_test, [11:8] I flags, [15:12] M flags |
| 0x130 | DOA_IN_COUNT | RO | | snapshots delivered |
| 0x134 | DOA_PEAK | RO | | {peak \|x1\|, peak \|x0\|}, last window |
| 0x138–0x158 | I_SEQ, I_PHI_Q28, I_PSI_Q24, I_THETA_Q28, I_DEG_Q16, I_AVG_SEQ, I_AVG_DEG_Q16, I_AVG_SQ_LO/HI | RO | | I-UESPRIT results |
| 0x15C–0x17C | M_SEQ, M_PHI_Q28, M_PSIMAG_Q24, M_THETA_Q28, M_DEG_Q16, M_AVG_SEQ, M_AVG_DEG_Q16, M_AVG_SQ_LO/HI | RO | | M-ESPRIT results |
| 0x180 | M_AVG_OFFCIRC | RO | | off-circle windows in the last average block |
| 0x184–0x190 | DOA_C00, C11, C01_RE, C01_IM | RO | | normalised cumulant (debug) |
| 0x194 | DOA_CAPS | RO | | [0] I built, [1] M built, [15:8] MAX_LOG2_SNAP |

Flags (4 bits): I-UESPRIT = `{0, overrun, ambiguous, no_signal}`;
M-ESPRIT = `{off_circle, overrun, ambiguous, no_signal}`.
Standard deviation of an average block: $\sigma = \sqrt{E[\theta^2] - E[\theta]^2}$,
from `AVG_SQ` (deg², Q32) and `AVG_DEG` (Q16).

`gnss_passthrough` `VERSION` (0x04) is now **0x00010002 (v1.2)**. The firmware
warns if the bitstream reports anything else.

---

## 7. Build flow (Vivado / Vitis)

> The ADI `hdl` 2026_r1 tree under `Vendor/` dictates the Vivado version for the
> **full hardware build**: use the same Vivado/Vitis the pipeline was built
> with before this change. The ESPRIT RTL itself is plain Verilog-2001 and
> simulates with any Vivado version (2023.2 included) or with Icarus.

### 7.1 Steps

| # | Step | Command | Success marker |
|---|---|---|---|
| 1 | Simulate the DOA system (optional but recommended) | `vivado -mode batch -source Source/Sim/ESPRIT/sim_doa.tcl` | `DOA_SIM_RESULT: PASS` |
| 2 | **Re-package** gnss_passthrough v1.2 | `vivado -mode batch -source Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl -tclargs $PWD/Source $PWD/Build/ip_repo` | `IP_PACKAGE_SOURCES: 20 RTL files …` then `IP_PACKAGE_OK` |
| 3 | Re-create the project | `cd Vivado/Project && vivado -mode batch -source ../Scripts/create_project.tcl` (env: `GNSS_CRPA_ROOT`, `GNSS_CRPA_IP_REPO`, `ADI_HDL_DIR`) | `CREATE_RESULT: PASS` |
| 4 | Bitstream + XSA | `vivado -mode batch -source Vivado/Scripts/build.tcl -tclargs Vivado/Project/antsdr_e310_gnss.xpr Build/BIT/system_top.bit Build/XSA/system_top.xsa` | `BUILD_TIMING: MET`, `BUILD_RESULT: PASS` |
| 5 | DOA implementation report | in the Tcl console: `open_run impl_1; source Vivado/Scripts/report_doa.tcl` | `DOA_IMPL: …` lines, `TIMING MET` |
| 6 | Firmware | `vitis -s Vitis/Scripts/build_software.py -- Build/XSA/system_top.xsa Vitis/Workspace Source/Firmware/app_gnss_e310 Build/ELF/e310_gnss_app.elf` | `SW_RESULT: PASS` |

On Linux, `Vivado/Scripts/build_esprit_linux.sh` runs steps 1–5 (plus `sw`) in
order and stops at the first failure:

```bash
source <Vivado>/settings64.sh
export ADI_HDL_DIR=<the packaged ADI hdl working copy you used before>
Vivado/Scripts/build_esprit_linux.sh                 # sim ip project build report
Vivado/Scripts/build_esprit_linux.sh sw              # firmware (vitis on PATH)
```

On Windows, the existing `Automation/PowerShell/Build_All.ps1` flow is unchanged.
Its IP-packaging step calls the same `gnss_passthrough_ip.tcl`, which now picks
up the ESPRIT files automatically.

### 7.1b Separate projects for I-UESPRIT and M-ESPRIT

The same packaged IP builds either estimator alone. `system_bd.tcl` reads the
environment variable `GNSS_DOA_ALGO` and sets the block-design IP parameters
`ENABLE_IUESPRIT` / `ENABLE_MESPRIT`:

| `GNSS_DOA_ALGO` | Project directory | Estimators | Console line in step 3 |
|---|---|---|---|
| unset / `both` | `Vivado/Project` | both | `GNSS_DOA_ALGO: both (ENABLE_IUESPRIT=1, ENABLE_MESPRIT=1)` |
| `iuesprit` | `Vivado/Project_IUESPRIT` | I-UESPRIT | `GNSS_DOA_ALGO: iuesprit (ENABLE_IUESPRIT=1, ENABLE_MESPRIT=0)` |
| `mesprit` | `Vivado/Project_MESPRIT` | M-ESPRIT | `GNSS_DOA_ALGO: mesprit (ENABLE_IUESPRIT=0, ENABLE_MESPRIT=1)` |

`create_project.tcl` creates the project in the current directory, so each
variant lives in its own directory with its own runs, bitstream and XSA. Use a
separate Vitis workspace per XSA. The firmware source is identical: it reads
`DOA_CAPS` and tests and prints only the estimator(s) present (self-test
8 checks I-UESPRIT only, 9 M-ESPRIT only, 11 both). `report_doa.tcl` writes to
`Build/reports/<project directory>/`.

Simulation of a single-estimator build: set `GNSS_DOA_ALGO` before
`sim_doa.tcl`, `run_xsim.sh`, `run_iverilog.sh` or `fw_cosim/run_fw_cosim.sh`
(system testbench: 41 checks I-UESPRIT only, 47 M-ESPRIT only, 61 both).

### 7.1c HLS variants: ESPRIT as a Vitis HLS IP in the block design

`GNSS_DOA_IMPL=hls` moves the estimator out of `gnss_passthrough` into its own block-design cell, `iuesprit_hls_0` or
`mesprit_hls_0`, built from `Source/HLS/esprit_hls` (see the README there for the diagram). `gnss_passthrough` is packaged
once and serves every variant. With `HLS_CORE=1` it streams the calibrated snapshots out on `m_axis_doa`, drives the HLS
IP's scalar configuration ports and reset, and takes the 9-word result packet back on `s_axis_res` into the usual registers.

| Variant | Environment | Project directory | Block-design cell |
|---|---|---|---|
| M-ESPRIT HLS | `GNSS_DOA_IMPL=hls GNSS_DOA_ALGO=mesprit` | `Vivado/Project_HLS_MESPRIT` | `mesprit_hls_0` |
| I-UESPRIT HLS | `GNSS_DOA_IMPL=hls GNSS_DOA_ALGO=iuesprit` | `Vivado/Project_HLS_IUESPRIT` | `iuesprit_hls_0` |

Both variants also need `GNSS_HLS_IP_REPO=<LoopBack_Code>/Build/hls_repo`, filled by
`vitis_hls -f Source/HLS/esprit_hls/run_hls.tcl -tclargs <mesprit|iuesprit> <repo>`. Step 3 prints
`GNSS_DOA_IMPL: hls (mesprit_hls_0 ...)`. `report_doa.tcl` prints the HLS cell's utilisation, and the firmware boot line
says `(Vitis HLS IP in the block design)`.

### 7.2 What to read in the Vivado Tcl console after step 5

```
DOA_IMPL: ===== ESPRIT DOA block: i_system_wrapper/system_i/gnss_passthrough/inst/u_doa =====
DOA_IMPL: u_doa (total)                LUT  ~22000  FF ~14000  DSP48 ~77  RAMB 5  LUTRAM ~100
DOA_IMPL: g_iuesprit.u_core            LUT  ~10000  ...
DOA_IMPL: g_mesprit.u_core             LUT  ~10000  ...
DOA_IMPL: CDC cells FIFO write gray pointer       5  OK
...
DOA_IMPL: worst setup slack into u_doa: <positive> ns (clk_fpga_0)
DOA_IMPL: design WNS <..> ns, WHS <..> ns -> TIMING MET
```

The numbers above are Yosys estimates for 7-series; Vivado reports the real
ones. Every `CDC cells … OK` line must say OK. A `MISSING` line means an XDC
pattern no longer matches, usually because a cell was renamed.

---

## 8. Verification in simulation (Vivado Tcl console)

### 8.1 System testbench — `tb_gnss_doa`

This drives the **real `gnss_passthrough` top**: an AD9361-like RX stream
(122.88 MHz l_clk, one snapshot every 4 clocks = 30.72 MS/s, 12-bit samples)
and an AXI-Lite master at 100 MHz.

**In the Vivado Tcl console:**

```tcl
source <path>/LoopBack_Code/Source/Sim/ESPRIT/sim_doa.tcl
```

**With waveforms:**

```tcl
set DOA_SIM_GUI 1
source <path>/LoopBack_Code/Source/Sim/ESPRIT/sim_doa.tcl
```

**Inside the open project, as a normal Vivado simulation set** (Flow Navigator → Run Simulation also works):

```tcl
source <path>/LoopBack_Code/Source/Sim/ESPRIT/add_doa_sim_to_project.tcl
```

From a shell: `Source/Sim/ESPRIT/run_xsim.sh` (or `run_xsim.sh gui`). Without
Vivado: `Source/Sim/ESPRIT/run_iverilog.sh`.

| Test | Checks |
|---|---|
| T1 identity | GCP1 ID, VERSION 1.2, DOA_ID, CAPS, reset value, SCRATCH, register write/read |
| T2 live RX | 6 windows (−50°, −20°, 0°, 15°, 40°, 65°; tone/QPSK/chirp, 15 dB SNR). No window lost, both estimators within 0.05° of the model, \|ψ\| within 0.01 |
| T3 stream | IN_COUNT = 6144, no CDC overflow, STATUS bits, peak meter |
| T4 test generator | 8 angles from −60° to +60°, within 0.05° |
| T5 calibration | −90° phase + CAL +90° → 0°, then identity → +30° |
| T6 statistics | noisy +30°: mean within 0.5°, standard deviation from AVG_SQ |
| T7 decimation | DECIM = 3 → exactly 1024 of 4096 snapshots |
| T8 passthrough | RX counter advances, DAC output left-aligned, registers respond |

**Expected end of the log** (the reference is in `Source/Sim/ESPRIT/reference_logs/tb_gnss_doa_reference.log`):

```
DOA_TB:   win 4 true   40.00 | I-UESPRIT   40.1216 (model   40.1209, err 0.0007) | M-ESPRIT   40.1216 ...
DOA_TB:   set   45.00 deg (DPHASE 0xa57d8666) | I-UESPRIT   45.0001 (err 0.0001) | M-ESPRIT   45.0001 (err 0.0001)
DOA_TB:   raw phase -90 deg + CAL +90 deg -> I  -0.0000  M  -0.0000 deg (expect 0)
DOA_TB:   M-ESPRIT mean  29.9146 deg, std 0.3524 deg, off-circle 0/16
DOA_TB:   4096 snapshots streamed, 1024 delivered
DOA_TB_RESULT: PASS (61 checks)
DOA_SIM_RESULT: PASS
```

**Waveform guide** (`waves_doa.tcl` groups the signals):

- `adc_*` shows the two antenna streams.
- `r2_x*` is what the estimators see after CDC and calibration.
- `i_res_valid` / `m_res_valid` pulse once per window, with `i_deg` / `m_deg`.
- `*_avg_deg` holds the averaged angle.

To show angles in degrees, right-click a signal → Radix → Real Settings → fixed
point, binary point 16, signed.

The stimulus and expected values come from `gen_doa_stim.py`. Both are
committed, so Python is only needed to change the vectors.

### 8.2 Firmware ↔ RTL co-simulation — `fw_cosim`

`Source/Sim/ESPRIT/fw_cosim/run_fw_cosim.sh` (Linux, Verilator ≥ 4.2, g++)
compiles the **unmodified** `gnss_doa.c` against a Verilator model of the
**unmodified** RTL. `gnss_pt_read`/`gnss_pt_write` become AXI transactions and
`no_os_mdelay` advances simulated time. RX1/RX2 carry a QPSK source with a
+37° "board" phase offset on RX2.

The run checks four things:

- the boot self-test passes
- the uncalibrated angle shows the offset
- `doa_cal` measures 36.98° and corrects it to 0.01°
- live −40/−15/+25/+50° are tracked to within 0.04°

It also exercises `doa_test`. Reference output: `reference_logs/fw_cosim_reference.log`,
ending in `FW_COSIM_RESULT: PASS`.

This run already caught one firmware bug before hardware: `doa_test` was
measuring through an active calibration. It is fixed.

### 8.3 Core-level regressions

`Source/IP/ESPRIT_standalone/{i_uesprit_2ant,m_esprit_2ant}/tb/run_sim.sh`
runs 9–10 cases per core against the Python model of the paper. The cases
cover p = 1/4/8, SNR 0–30 dB, −40 dBFS input, windows of 128–1024, end-fire,
covariance mode, the AXI wrapper, and (M-ESPRIT only) a no-source case. Each
standalone IP README explains how to view the waveforms in xsim.

---

## 9. Verification on hardware (PuTTY console)

**Console: USB "PWR & JTAG CONSOLE" port, 115200 8N1, no flow control** (as
before). Every DOA self-test line starts with `DOA_SELFTEST:` and every monitor
line with `DOA:`, so you can log the PuTTY session (Session → Logging → "All
session output") and grep the log afterwards.

> **RF safety is unchanged.** The DOA block is receive-only, but the rest of the
> pipeline still retransmits RX1 on TX1 with `gnss_tx=1` (and automatically in
> the SD deployment build). Keep TX1 on conducted, attenuated coax. **Never
> radiate at GNSS frequencies.** For over-the-air DOA tests, use a frequency
> you are licensed for (§9.6).

### 9.1 HW-1: boot self-test (no RF needed)

Program the board (JTAG or SD `BOOT.BIN`). After the existing AD9361 and
passthrough messages, the console prints the block below. It is identical to
the co-simulation output:

```
=== ESPRIT DOA monitor (gnss_passthrough v1.2) ===
DOA: ESPRIT DOA monitor present, version 1.0, estimators: I-UESPRIT M-ESPRIT, max window 2^14
DOA_SELFTEST: ===== ESPRIT DOA hardware self-test =====
DOA_SELFTEST: S1 DOA block present (DOA_ID = 0x444F4131)                 PASS
DOA_SELFTEST: S1 at least one estimator built                            PASS
DOA_SELFTEST: S1 gnss_passthrough reports v1.2                           PASS
DOA_SELFTEST: S2 sweep: set angle | I-UESPRIT (error) | M-ESPRIT (error) |psi|
DOA_SELFTEST:    -60 deg |-60.008 (-0.008) |-60.008 (-0.008)  1.0000
   ...
DOA_SELFTEST:     60 deg | 59.999 (-0.001) | 59.999 (-0.001)  1.0000
DOA_SELFTEST: S2 I-UESPRIT sweep -60..60 deg, every error < 0.05 deg     PASS
DOA_SELFTEST: S2 M-ESPRIT  sweep -60..60 deg, every error < 0.05 deg     PASS
DOA_SELFTEST: S3 noisy +20 deg: I mean  19.977 std  0.123 | M mean  19.977 std  0.122
DOA_SELFTEST: S3 I-UESPRIT with noise within 0.5 deg                     PASS
DOA_SELFTEST: S3 M-ESPRIT  with noise within 0.5 deg                     PASS
DOA_SELFTEST: S4 -90 deg phase + CAL +90 deg -> I -0.000  M -0.000 (expect 0)
DOA_SELFTEST: S4 calibration rotation cancels the phase (|theta| < 0.05) PASS
DOA_SELFTEST: S5 noise only: |psi|  0.1087  off-circle 16/16
DOA_SELFTEST: S5 M-ESPRIT flags 'no source' (off-circle >= 8/16)         PASS
DOA_SELFTEST: S6 live RX1/RX2 snapshots in 10 ms: 307200 (expect ~ fs/100)
DOA_SELFTEST: S6 live RX1/RX2 samples reach the estimators               PASS
DOA_SELFTEST: S6 no CDC FIFO overflow                                    PASS
DOA_SELFTEST: ===== RESULT PASS (11/11 checks passed) =====
DOA: live RX1/RX2 DOA ENABLED (receive-only, TX path untouched)
```

What each step proves:

- **S1:** the bitstream carries v1.2 and the AXI path works.
- **S2–S5:** the complete PL chain (test source → calibration → both estimators
  → averaging → registers) is correct on silicon, including the 100 MHz timing.
- **S6:** live AD9361 RX1 and RX2 samples cross the CDC into the estimators. The
  count is about `fs/100` (307 200 at 30.72 MS/s).

Re-run at any time with `doa_selftest?`.

### 9.2 HW-2: known angles from the test generator

```
doa_test=-30
doa_test=0
doa_test=37.5
```

Each prints the set angle, both estimates and the error. Expect errors below
0.05° (they are typically ≤ 0.01°). The previous mode and calibration are restored afterwards.

### 9.3 HW-3: live signal, phase calibration (splitter)

Equipment: a CW or modulated signal generator (or any stable source) at the RX
LO ± a few MHz, a 2-way power splitter, and two **equal-length** cables to RX1
and RX2. Use −60 to −30 dBm per port, so `DOA_PEAK` reads a few thousand counts
but stays well below 32767. Check with `doa?` (peak |x0|, |x1|).

```
doa?                 # before calibration: angle = board/cable offset (any value)
doa_cal=1            # measures the RX2-RX1 phase and writes DOA_CAL
doa?                 # now ~0.000 deg on both estimators
doa_monitor=20       # 20 averaged lines; mean ~0, std typically < 0.1 deg
```

Expected `doa_cal=1` output (values depend on your board):

```
DOA: measured RX2-RX1 offset  36.983 deg over 64 windows -> DOA_CAL = 0xb3006640 (cos 26176, sin -19712)
DOA: after calibration the common signal reads  0.010 deg (expect ~0). Calibration is lost at power-off.
```

Pass criterion: after calibration, \|mean\| < 0.3° and std < 0.3° on both
estimators, `off-circle 0/16`, and no flags.

`doa_cal=1` first checks the **coherence** of the 64 windows it measures (see
9.3b). Below 0.900, RX1 and RX2 are not carrying the same signal (nothing
connected, a cable missing, or sky antennas that only see satellites below the
noise). The calibration is then **refused** and the previous `DOA_CAL` is kept:

```
DOA:   coherence  0.035  RX1 peak 8000 (-12.2 dBFS)  RX2 peak 7376 (-12.9 dBFS)
DOA: calibration REFUSED - coherence below 0.900: RX1 and RX2 do not carry
     the same signal ... Previous calibration kept.
```

### 9.3b Phase, calibration and jammer check (`doa_phase?`, `doa_watch=N`, `doa_cal?`)

Firmware only: these commands use registers the FPGA already has, so no
bitstream rebuild is needed (rebuild only the Vitis application / BOOT.BIN).

- **Phase RX2−RX1.** `raw` is the phase as received, including the board and
  cable offset. `calibrated` is the phase after `DOA_CAL`; this is the phase
  the angle is computed from (θ = asin(−kφ)). raw = calibrated − (CAL rotation).
- **Calibration.** The offset `doa_cal=1` removed, how many windows it used
  and how coherent the calibration signal was.
- **Coherence.** The length of the mean unit vector e^{jφ} over N windows.
  It is 1.000 when one signal reaches both elements with a stable phase, and
  about 1/√N for independent noise.
- **|psi| (M-ESPRIT).** For one source |ψ| = |x1|/|x0|, so it reads the RX2/RX1
  amplitude ratio. A stable phase with |ψ| far from 1 usually means unequal RX
  gains.
- **Level.** Peak sample per channel in dBFS (0 dBFS = 32767).
- **Verdict.**

| Verdict | Condition | Meaning |
|---|---|---|
| `JAMMER / STRONG COHERENT SOURCE at θ` | coherence ≥ 0.90 (and M-ESPRIT off-circle ≤ 25 %) | one strong signal; θ is its DOA |
| `NO JAMMER` | coherence < 0.50 | noise only; satellites are ~20 dB below the noise and are not visible to ESPRIT on raw samples |
| `UNCERTAIN` | otherwise | weak source, two sources / multipath, or (stable phase, \|ψ\| ≠ 1) unequal RX gains |
| `NO SIGNAL` | no_signal flag | all-zero input |

`doa_phase?` collects 32 windows. Example (co-simulation, jammer at 25° after
calibration, board offset 37°):

```
DOA: ==== PHASE / CALIBRATION / JAMMER CHECK  (M-ESPRIT, 32 windows of 1024 snapshots) ====
DOA:   calibration    : ACTIVE  offset removed  37.033 deg  (doa_cal=1, 64 windows, coherence  1.000)
DOA:   phase RX2-RX1  : raw        -39.068 deg   as received (includes the board/cable offset)
DOA:                    calibrated -76.101 deg   after CAL; this phase gives the DOA
DOA:   coherence      :  1.000   (1.000 = same signal on RX1 and RX2, ~0 = independent noise)
DOA:   |psi|          :  1.0004   = RX2/RX1 amplitude ratio for one source; off-circle 0/32 windows
DOA:   level          : RX1 peak 11584 (-9.0 dBFS)   RX2 peak 13351 (-7.7 dBFS)
DOA:   DOA            : mean  25.018 deg  std  0.069 deg  (average of 16 windows)
DOA:   VERDICT        : >>> JAMMER / STRONG COHERENT SOURCE at  25.018 deg <<<
```

Noise only (no jammer):

```
DOA:   coherence      :  0.058   (1.000 = same signal on RX1 and RX2, ~0 = independent noise)
DOA:   VERDICT        : NO JAMMER - RX1 and RX2 see independent noise only.
```

`doa_watch=N` prints one line per 16 windows:

```
DOA: #1    raw -39.1  cal -76.2 deg  coh  1.00  |psi|  1.00  RX1 -9.2 RX2 -8.2 dBFS  DOA  25.021  JAMMER
```

`doa?` also prints a phase line for the last window:

```
DOA:   phase RX2-RX1 (last window): raw -39.0xx deg  calibrated -76.0xx deg  (CAL removes 37.033 deg)
```

### 9.4 HW-4: known angle by cable delay (fully conducted DOA test)

After HW-3, insert an extra length $\Delta L$ of coax in the **RX2** path only.
RX2 is then delayed by $\tau = \Delta L / (v_f c)$, which gives

$$
\phi = -2\pi f\tau = -\frac{2\pi\,\Delta L}{\lambda_{cable}},\qquad \lambda_{cable} = \frac{v_f c}{f},
\qquad \theta_{expected} = \arcsin\!\Big(-\frac{\phi}{\pi}\Big)\ \ (d=\lambda/2\ \text{setting}).
$$

At 1575.42 MHz with PTFE/RG-316-class cable ($v_f \approx 0.70$, $\lambda_{cable} \approx 133$ mm):

| Extra length in RX2 | φ | Expected θ (`doa_spacing_mm=95`, i.e. k = 1/π) |
|---|---|---|
| 16.6 mm (λc/8) | −45° | +14.48° |
| 33.3 mm (λc/4) | −90° | +30.00° |
| 50.0 mm (3λc/8) | −135° | +48.59° |

Measure $v_f$ (or the phase) of the actual cable, or trim with a phase shifter
if you have one, because connectors add phase. A sign flip means the extra
cable went into RX1. The steps:

1. Run `doa_spacing_mm=95`.
2. Insert the extra length in RX2.
3. Run `doa_monitor=20`.
4. Compare with the table.

The two estimators must agree with each other to within about 0.05°.

### 9.5 HW-5: using the board's own TX1 as the source (loop-back)

The LoopBack pipeline can drive TX1 itself, either from the DDS or from the DDR
replay (`gnss_ddr_tx=1`). Connect **TX1 → attenuator (≥ 40 dB) → splitter →
RX1/RX2** to get the signal of HW-3/HW-4 with no external generator. Follow the
existing TX safety procedure (`gnss_tx?`, `tx1_attenuation=`, abort with
`gnss_tx=0` / `gnss_ddr_tx=0`). In passthrough mode (`gnss_tx=1`), TX1
retransmits RX1, which creates an RF feedback loop. Use the DDR replay or the DDS
for this test, never the live passthrough.

### 9.6 HW-6: real antennas (over the air)

1. Mount two antennas with spacing $d$ and measure it.
2. Enter the spacing, e.g. `doa_spacing_mm=95`. This sets k for the current RX LO.
3. Calibrate once with HW-3 (splitter at the antenna ports).
4. Point a transmitter you are licensed to use at known bearings. Read
   `doa_monitor=20` at each bearing.

Angles are measured from broadside, and are positive toward RX1's side when
x1 lags x0 (Eq. (2)). Keep $d \le \lambda/2$ at the test frequency. **Do not
transmit GNSS-band signals over the air.** Live GPS satellites are below the
thermal noise and have an almost-zero fourth-order cumulant, so they are not
measurable this way. The monitor is meant for strong emitters (interference
and jammers), which is the CRPA use case.

### 9.7 Pass/fail summary for a bench report

| Test | Command | Pass |
|---|---|---|
| HW-1 | boot / `doa_selftest?` | `RESULT PASS (11/11)` |
| HW-2 | `doa_test=X` | \|error\| < 0.05° |
| HW-3 | `doa_cal=1`, `doa_monitor=20` | \|mean\| < 0.3°, std < 0.3°, no flags |
| HW-4 | cable delay | within 1° of the table (cable-tolerance limited); I vs M within 0.05° |
| JTAG | `xsdb Vitis/Scripts/doa_jtag_test.tcl` | `DOA_JTAG_RESULT: PASS` |

---

## 10. JTAG test without the console (XSDB)

With the bitstream loaded and the PS configured (firmware running, or just
FSBL/ps7_init):

```
xsdb Vitis/Scripts/doa_jtag_test.tcl
```

The script prints, for example:

```
DOA_JTAG: gnss_passthrough ID = GCP1                                 PASS
DOA_JTAG: gnss_passthrough VERSION = 1.2                             PASS
DOA_JTAG: DOA_ID = DOA1                                              PASS
DOA_JTAG:   set  -60 deg -> I-UESPRIT  -60.008  M-ESPRIT  -60.008  |psi| 1.0000
...
DOA_JTAG: live RX1/RX2 snapshots in ~100 ms: 3072000
DOA_JTAG_RESULT: PASS (6/6)
```

It restores every register it touched.

---

## 11. Console command reference

Type `doa_help?` on the board for this list. All DOA commands are receive-only.

| Command | Action |
|---|---|
| `doa?` | latest window and averaged angle for both estimators, std, φ, ψ, flags, peak, snapshot count |
| `doa=1` / `doa=0` | live RX1/RX2 DOA on / off |
| `doa_status?` | configuration (p, window, average, μ, k, CAL as an angle, test-generator settings) plus `doa?` |
| `doa_regs?` | raw dump of 0x100–0x194 |
| `doa_selftest?` | full hardware self-test S1–S6, PASS/FAIL |
| `doa_test=<deg>` | test generator at a known angle, prints the errors |
| `doa_monitor=<N>` | N averaged results, one line each |
| `doa_cal=1` / `doa_cal=0` | calibrate the RX2-RX1 phase (same signal on both) / clear |
| `doa_cal?` | calibration in use: offset removed, windows, coherence (or NONE) |
| `doa_phase?` | phase RX2−RX1 raw and calibrated, coherence, \|ψ\|, dBFS levels, DOA, JAMMER / NO JAMMER verdict (9.3b) |
| `doa_watch=<N>` | the same as one line per 16 windows, N lines |
| `doa_spacing_mm=<mm>` | antenna spacing, sets k = λ/(2πd) at the current RX LO |
| `doa_window=<n>` | 2ⁿ snapshots per window, n = 7..14 |
| `doa_p=<n>` | Cᵖ with p = 2ⁿ, n = 0..3 (paper: 2) |
| `doa_avg=<n>` | 2ⁿ results per reported mean, n = 0..10 |
| `doa_cov=1` / `0` | covariance subspace (Gaussian-noise jammers) / cumulant (paper) |
| `doa_decim=<N>` | keep 1 of N+1 snapshots |

The main menu (`?`) also has option **10 — ESPRIT DOA**.

---

## 12. Resources, timing, limits and troubleshooting

**Resources.** These are Yosys `synth_xilinx -family xc7` results. Vivado's
own numbers come from `report_doa.tcl` and are usually lower.

| Block | LUT | FF | DSP48E1 | Block RAM | LUTRAM |
|---|---|---|---|---|---|
| I-UESPRIT core alone | ~10.0 k | ~6.3 k | 45 | 2 × RAMB36 | — |
| M-ESPRIT core alone | ~10.3 k | ~6.4 k | 49 | 2 × RAMB36 | — |
| **Whole gnss_passthrough v1.2** (passthrough + DOA wrapper + both cores) | **23.2 k (44 % of 53.2 k)** | **14.5 k (14 %)** | **77 (35 % of 220)** | **4 × RAMB36 + 1 × RAMB18** | 27 × RAM32M |

The ADI base design (axi_ad9361, DMAs, interconnect) comes on top of this. If
the combined utilisation is too high for routing, disable one estimator.

**Timing.** Everything new except capture runs at 100 MHz (clk_fpga_0). The
cores have two registers after every wide multiplier, a 2-stage normaliser and
a 4-stage arcsin interpolation. The CDC paths are constrained in
`gnss_passthrough_cdc.xdc` §4. The existing `Congestion_SpreadLogic_high`
strategy and the ADI post-route timing fix still apply. If timing does not close,
build with one estimator (`ENABLE_IUESPRIT = 0` in the IP customisation, or
default it in `gnss_passthrough.v`).

**Limits**

* **One source per baseline.** Two elements resolve the strongest coherent
  source only. For 2-D DOA, use a second baseline.
* **Gaussian-noise jammers** have a zero fourth-order cumulant. Use `doa_cov=1`.
* **End-fire.** Beyond about ±75° (±80° at good SNR), every two-element ESPRIT
  aliases. This is a physical limit, not a bug.
* **Calibration** is not stored across power cycles. Re-run `doa_cal=1`, or hard-code
  `DOA_CAL` in `gnss_doa_defaults()` once it has been measured.
* **Window vs. rate.** The minimum window is 128 snapshots; shorter windows
  set the `overrun` flag.

**Troubleshooting**

| Symptom | Cause / fix |
|---|---|
| `DOA: not present (DOA_ID … reads 0x00000000)` | The bitstream is v1.1. Re-package the IP (§7 step 2) and rebuild. |
| `gnss_pt: WARNING version mismatch … reports 1.1` | Same as above. |
| S6 FAIL, `snapshots in 10 ms: 0` | AD9361 RX not running, or channel 1 disabled (2R2T must be on; `adc_enable_i1` gates capture). |
| `CDC-OVERFLOW!` | The sample rate is above about 90 MS/s. Set `doa_decim=1`. |
| `ambiguous` flag | \|kφ\| > 1: wrong spacing (`doa_spacing_mm`) or a source near end-fire. |
| `off_circle` on M-ESPRIT, angle jumps | No coherent source (noise only), or the signal is too weak. Check `DOA_PEAK`. |
| Angles plausible but offset | Run `doa_cal=1` (the AD9361 `rx1rx2_phase_inversion_en = 1` and cabling add a fixed phase). |
| Angle has the wrong sign | RX1/RX2 cables swapped, or set a negative K (`DOA_K_Q30`, two's complement). |
| `DOA_IMPL: CDC cells … MISSING` | An XDC pattern no longer matches. Update §4 of `gnss_passthrough_cdc.xdc`. |

---

## 13. File-by-file change list

**New**

```
Source/HDL/gnss_doa_esprit.v                 DOA monitor wrapper (registers, CDC, cal, averaging)
Source/HDL/doa_async_fifo.v                  gray-pointer dual-clock FIFO
Source/HDL/doa_test_gen.v                    two-element synthetic source
Source/HDL/esprit/*.v                        16 files: I-UESPRIT + M-ESPRIT cores
Source/Firmware/app_gnss_e310/gnss_doa.c/.h  driver, self-test, calibration, console output
Source/Sim/ESPRIT/                           system TB, stimulus, Vivado/xsim/Icarus scripts,
                                             firmware co-simulation, reference logs
Source/IP/ESPRIT_standalone/                 standalone ESPRIT IPs + core regressions + models
Vivado/Scripts/report_doa.tcl                post-implementation DOA report
Vivado/Scripts/build_esprit_linux.sh         Linux build driver
Vitis/Scripts/doa_jtag_test.tcl              XSDB hardware test
Docs/ESPRIT/README_ESPRIT.md                 this document
```

**Modified (all edits marked "v1.2" / "MOD-11")**

```
Source/HDL/gnss_passthrough.v                instantiates u_doa, decodes 0x100-0x1FC, VERSION 1.2,
                                             ENABLE_IUESPRIT/ENABLE_MESPRIT parameters
Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl   packages the ESPRIT RTL (list from doa_files.tcl)
Source/IP/gnss_passthrough/gnss_passthrough_constr.xdc  DOA CDC constraints (IP-scoped copy)
Source/XDC/gnss_passthrough_cdc.xdc          section 4: DOA CDC constraints
Source/Config/board_e310_v1.json             VERSION 1.2, DOA register map
Source/Firmware/app_gnss_e310/main.c         gnss_doa_boot_check() after the GNSS self-test
Source/Firmware/app_gnss_e310/command.c/.h   15 doa* console commands
Source/Firmware/app_gnss_e310/gnss_info.c/.h menu option 10 "ESPRIT DOA"
Source/Firmware/app_gnss_e310/gnss_passthrough.c/.h  expected version 1.2
```

**Not changed:** block design (`system_bd.tcl`), `system_top.v`,
`system.xdc`, ADI/MicroPhase vendor trees, and the TX path behaviour.

> After unpacking: the copies under `Build/ip_repo/gnss_passthrough` and
> `Vivado/Project/` are the **old v1.1 build outputs** from the original
> archive. They are regenerated by §7 steps 2–3; do not build from them.
