# STAP anti-jamming core for FPGA

FPGA (Verilog-2005 RTL) port of the MATLAB **STAP-main** project
(`STAP_constraint.m`): a GPS L1 space-time adaptive processing (STAP) beamformer
for a 7-element antenna array with 16 taps per element (112 complex weights).
It applies MVDR or power-inversion weights and places deep nulls on wideband
jammers.

**Default target: Xilinx Zynq-7020 (XC7Z020)**, 120 MHz fabric clock, 60 MSPS
per channel. It uses 192 of 220 DSP48E1, 59 of 140 RAMB36 and about 50 % of the LUTs
(see [Resources](#resources)).

```
 7 x ADC/DDC (complex I/Q, 16 bit)
        │
        ├──────────────► stap_fir   y[n] = Σ conj(w_t)·x_t     (every sample, 56 cMAC × 2 clk)
        │                    ▲ weights (double-buffered)          │
        │                    │                                    ▼  to GPS correlators
        └──► stap_corr ──────┼──── AXI4-Lite ──── CPU: sw/stap_solver.c
             capture block   │                    exact R = Xm·Xm'/L, Cholesky,
             28 pairs × 31 lags (8 per pass)      w = R⁻¹S/(SᴴR⁻¹S), quantise, commit
```

| MATLAB (`STAP_constraint.m`)          | FPGA implementation                                   |
|---------------------------------------|-------------------------------------------------------|
| `Xm` tap matrix (circshift loops)      | per-channel 16-tap delay line in `stap_fir`           |
| `Rx = Xm*Xm'/L`                        | `stap_corr` (hardware) + edge correction (CPU), exact  |
| `w = (S'R⁻¹S)⁻¹R⁻¹S`                   | `stap_solve_mvdr()` Cholesky in C (ARM / MicroBlaze)  |
| `w_opt'*Xm`                            | `stap_fir`, 18-bit weights, DSP48 multipliers         |
| power inversion (`S = [1 0 …]`)        | `stap_steering_pi()`                                  |

## Directory layout

| Path | Contents |
|------|----------|
| `rtl/stap_fir.v`   | Space-time filter. Each complex MAC uses 3 real multipliers (Gauss form, bit-exact). `FOLD` time-multiplexes it: 2 = 120 MHz clock at 60 MSPS (default), 4 = 240 MHz. Weights are double-buffered and switch on a sample boundary. |
| `rtl/stap_corr.v`  | Covariance engine. Captures `L+30` samples into BRAM, then computes 868 complex correlations with `LAGP` parallel 3-multiplier complex MACs. The default `LAGP=8` takes about 563k clocks for L = 5000, i.e. 4.7 ms at 120 MHz. `LAGP=31` takes 143k clocks. Has a raw-sample read port. |
| `rtl/stap_top.v`   | Top level with the AXI4-Lite register map (below), sample stream in, filtered stream out, and `irq`. |
| `sw/stap_solver.[ch]` | Weight solver: exact covariance build, steering vector, Cholesky MVDR, weight quantisation, output-shift selection. C99, no malloc. |
| `sw/stap_driver.c` | Bare-metal / UIO update loop: capture → read → solve → write → commit. |
| `sw/stap_solve_cli.c` | Host command-line wrapper used by the golden model. |
| `model/stap_golden.py` | Re-creates the MATLAB scenario, gives bit-true models of both RTL blocks, writes test vectors and a performance report. Needs only numpy. |
| `tb/tb_stap_top.v` | End-to-end bit-exact testbench driven over AXI4-Lite. |
| `scripts/run_sim.sh` | Full regression: C solver → model → Icarus Verilog sims (FOLD:LAGP = 2:8 and 4:31). |
| `scripts/synth_yosys.sh` | Yosys resource estimate (`synth_xilinx`). |
| `scripts/vivado_synth.tcl`, `constraints/stap_top.xdc` | Vivado out-of-context build. Provided as a starting point; not run here. |

## Running

```bash
./scripts/run_sim.sh                 # needs gcc, python3+numpy, iverilog
```

```bash
./scripts/run_sim.sh --jam-db 70 --adc-bits 12   # any stap_golden.py option
```

```bash
./scripts/synth_yosys.sh 2 8 xc7     # FOLD, LAGP, family
```

The model can also describe other arrays and front ends. This is the E310 CRPA case (2 elements,
30.72 MSPS complex baseband, 12-bit right-aligned AD9361 samples, 12-bit output). Build the
solver with `-DSTAP_M=2` first:

```bash
python3 model/stap_golden.py --M 2 --fs 30.72e6 --fif 0 --jam-theta 30 --adc-bits 12 --adc-align right --ow 12 --mode pi --solver <cli built with -DSTAP_M=2>
```

The array size of the C solver is set by `STAP_M`/`STAP_N`, or by a `stap_config.h` placed next to `stap_solver.h`.

**Integration example:** `../LoopBack_STAP_CRPA2` puts this core (M=2) into the ANTSDR E310 AD9361 loopback design as a CRPA anti-jam stage.

## Parameters

| Parameter   | Default | Meaning |
|-------------|---------|---------|
| `M`, `N`    | 7, 16   | elements, taps per element. The C solver uses the same constants (`STAP_M`, `STAP_N`). T = M·N ≤ 256 and M ≤ 7. |
| `DW`        | 16      | I/Q sample width (≤ 16; a 12–14 bit ADC is sign-extended/left-aligned) |
| `WW`        | 18      | weight width (one DSP48 18-bit port) |
| `OW`        | 16      | output width (saturated) |
| `FOLD`      | 2       | clocks per sample (MAC sharing factor); filter DSPs = 3·ceil(112/FOLD) |
| `LAGP`      | 8       | correlator lags per pass (1..31); correlator DSPs = 3·LAGP |
| `LMAX_LOG2` | 13      | capture buffer depth 8192 → L ≤ 8162 snapshots |

## Register map (AXI4-Lite, byte addresses)

| Address | Name | Access | Description |
|---------|------|--------|-------------|
| 0x0000 | ID | RO | `0x53544150` ("STAP") |
| 0x0004 | CONFIG | RO | `[7:0] M, [15:8] N, [23:16] DW, [31:24] WW` |
| 0x0008 | CONFIG2 | RO | `[7:0] FOLD, [15:8] OW, [23:16] CW, [31:24] LMAX_LOG2` |
| 0x000C | CONFIG3 | RO | `[7:0] LAGP` |
| 0x0010 | CTRL | WO | bit0 CORR_START, bit1 W_COMMIT, bit2 CLR_OVERRUN (pulses) |
| 0x0014 | STATUS | RO | bit0 corr_busy, bit1 corr_done (sticky, also `irq`), bit2 commit_pending, bit3 overrun |
| 0x0018 | CORR_LEN | RW | snapshots L (default 5000) |
| 0x001C | OUT_SHIFT | RW | output arithmetic right shift (the solver recommends a value) |
| 0x0020 | RAW_BASE | RW | first sample of the raw window |
| 0x0024 | USER_CTRL | RW | free control word, exported on port `user_ctrl`; integration wrappers use it (the E310 CRPA uses bit0 = anti-jam/bypass, bit1 = TX2 copy) |
| 0x0100 + 4·(64m+i) | RAW | RO | captured sample `RAW_BASE+i` of channel m, `{re16, im16}` |
| 0x1000 + 8t / +4 | W_RE / W_IM | WO | shadow weight t (t = m·16 + k), two's complement |
| 0x8000 + 16e + {0,4,8,C} | CORR | RO | correlation e: re lo, re hi, im lo, im hi (64-bit sign-extended) |

Correlation `e = pair·31 + (τ+15)`, with pairs enumerated `for a in 0..6: for b in a..6`:

`c_ab[τ] = Σ_{l=0}^{L-1} x_a[l+15+τ] · conj(x_b[l+15])`

The CPU adds back the ≤ 15 products at each edge of the block using the raw head/tail samples, so it gets exactly `R = Xm·Xm'/L` from `STAP_constraint.m`. A plain block-Toeplitz approximation is not positive definite with 50 dB+ jammers, and Cholesky fails on it. Edge reconstruction fixes this.

## Fixed-point design

| Signal | Format |
|--------|--------|
| ADC samples | 16-bit I/Q (14-bit ADC by default, RMS 15 dB below full scale) |
| Weights | 18-bit I/Q, power-of-two block scale `2^exp` chosen by `stap_quantize()` |
| Filter accumulator | 43 bits (no overflow possible), then `>>> OUT_SHIFT`, saturated to 16 bits |
| Correlator accumulator | 47 bits (`2·DW+2+LMAX_LOG2`, no overflow for L ≤ 8162) |
| Solver | IEEE double on the CPU (covariance assembled exactly in 64-bit integers) |

Filter latency is `FOLD + 6 + ceil(log2(P))` clocks from `s_valid` to `y_valid`, where P = ceil(112/FOLD). That is 14 clocks for FOLD=2 and 15 for FOLD=4.

## Verification results

The default run uses the MATLAB scenario: 3 random-phase-modulated wideband jammers at −20°/+30°/+50°, JSR 50 dB, SNR −31.37 dB, L = 5000.

**RTL vs bit-true model (Icarus Verilog; FOLD:LAGP = 2:8, the Zynq-7020 default, and 4:31):**
- 868 / 868 correlation results bit-exact
- 420 / 420 raw edge samples bit-exact
- 3000 / 3000 filter outputs bit-exact. The data is out-of-sample (after the training block) with irregular input gaps, and a weight update is committed over AXI during the run.

**Performance.** All figures are measured out-of-sample, on the samples after the training block.

| | Floating point (MATLAB algorithm) | This FPGA design, bit-true |
|---|---|---|
| Jammer suppression (JRI) | 64.8 dB | **64.8 dB** |
| Null depth −20° / +30° / +50° (0–20 MHz avg) | −56.4 / −54.0 / −53.0 dB | **−56.3 / −54.0 / −53.0 dB** |
| SINR in → out | −54.8 → −11.6 dB | −54.8 → **−11.6 dB** |
| 12-bit ADC | – | 64.8 dB JRI |
| 10-bit ADC | – | 64.7 dB JRI |
| JSR 70 dB, 12-bit ADC | 79.1 dB JRI | 79.0 dB JRI, nulls −71.5 to −74.4 dB |
| Power inversion mode | 29.5 dB JRI | 29.5 dB JRI |

The fixed-point implementation loses less than 0.1 dB against floating point in every case tested.
Real hardware is usually limited to 40–60 dB by channel mismatch, mutual coupling and ADC non-linearity, none of which the simulation models. Channel calibration is recommended before trusting more than about 50 dB.

## Resources

Yosys `synth_xilinx -family xc7` estimate. This is not a Vivado result and includes no timing analysis.

| Configuration | Clock @ 60 MSPS | DSP48E1 | RAMB36 | LUT | FF | Fits Zynq-7020? |
|---|---|---|---|---|---|---|
| **FOLD=2, LAGP=8 (default)** | 120 MHz | **192** (168 filter + 24 corr) | 59 | ~26.4k | ~29.0k | **yes**: 87 % DSP, 42 % BRAM, 50 % LUT, 27 % FF |
| FOLD=4, LAGP=31 | 240 MHz | 177 (84 + 93) | 59 | ~20.6k | ~25.7k | resources yes, but 240 MHz is unlikely on a -1 part |
| FOLD=4, LAGP=8 | 240 MHz | 108 (84 + 24), computed, not synthesized | 59 | – | – | resources yes, same timing caveat |
| earlier 4-multiplier version, FOLD=4 | 240 MHz | 236 | 59 | ~18.1k | ~22.7k | no |

The Zynq-7020 (XC7Z020) has 220 DSP48E1, 140 RAMB36, 53,200 LUTs and 106,400 FFs.

How the default fits:
1. **3-multiplier complex MACs.** `k1 = wr(xr+xi)`, `k2 = xr(−wr−wi)`, `k3 = xi(wr−wi)`, giving `re = k1−k3` and `im = k1+k2`. This saves 25 % of the DSPs and is exactly equal in integer arithmetic. Each product fits one 25×18 DSP48E1.
2. **FOLD=2.** 56 MAC units are each shared by 2 taps at a 120 MHz clock. This is a comfortable timing target for a -1 speed grade, where 240 MHz is not.
3. **Correlator LAGP=8.** It processes 8 lags per pass in 4 passes per channel pair, and shifts the read address instead of using a lag multiplexer. The covariance update becomes 4.7 ms instead of 1.2 ms, which is still fast compared with how quickly jammer geometry changes.

That leaves 28 DSPs, about 27k LUTs and 81 BRAM36 free for a DDC, calibration logic or the AXI interconnect to the PS.
Timing closure at 120 MHz has not been verified. Run `vivado -mode batch -source scripts/vivado_synth.tcl` (defaults: `xc7z020clg400-1`, FOLD=2, LAGP=8) and check `timing.rpt`.

## Differences from / fixes to the MATLAB code
- **Steering-vector bug fixed.** `STAP_constraint.m` used `exp(-1i*pi*d*sind(θ)…)`, which drops the `2/λ` factor. It was only correct for θ = 0. `stap_steering()` uses `2π·d/λ·sin θ`.
- **Weights evaluated out-of-sample.** MATLAB trained and evaluated on the same data. Here the weights are trained on the captured block and then measured on later samples.
- **Covariance is still exact** (`Xm·Xm'/L`), but the hardware computes only 868 correlations instead of 12544 entries.
- **The GPS look direction is configurable** in software (`theta_deg`). Switching between MVDR and power inversion needs no RTL change.

## Known limitations / next steps
- No channel calibration, ADC/DDC front end or GPS acquisition/tracking is included. The core expects complex baseband/IF samples, as in the MATLAB model.
- The weight update period is set by software. On a Zynq-7020 it is roughly 5–10 ms: 4.7 ms for the correlation engine at LAGP=8, plus the 112×112 Cholesky solve on the Cortex-A9 (about 0.5 M complex flops, a few ms).
- `OUT_SHIFT` takes effect immediately, while the weights wait for the next sample boundary, so the output scale may be off for one or two samples during an update.
