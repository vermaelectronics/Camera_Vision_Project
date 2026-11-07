# SFAP-PI v5.0 — ANTSDR E310 V1, two-antenna GNSS anti-jam

Converted from the SMI-PI v4.1 space-time (STAP) design. Everything outside the
filter is unchanged: the AD9361 datapath, `gnss_passthrough`'s RX→TX path, CDC,
elastic buffers, `gnss_smi=0` transparent passthrough, the detection / gain loop
/ state machine on the ARM, the console commands and the RF profiles.

## 1. What SFAP-PI is

STAP (v4.1) filtered in **time**: 16 taps on RX1 + 16 on RX2 + 2 image taps, one
33×33 SMI solve per capture.

SFAP (v5.0) filters in **frequency**: both channels are split into 64 bins, and
every bin gets its own two-antenna power-inversion weight. A jammer's RX2/RX1
relationship varies across the band (cables, antenna spacing, the two analog
filters, multipath). SFAP follows that variation with one weight per bin
instead of a long FIR, and can null *different* jammers from *different*
directions as long as they are at different frequencies. One broadband weight
cannot do that.

Per frame and bin k:

```
Y_k = X1_k − b_k·X2n_k − c1_k·conj(X1_−k) − c2_k·conj(X2n_−k)
```

* `X1` (RX1) is the reference, fixed at 1 (power inversion): the output can never be turned down as a whole.
* `b_k` is the spatial weight on the normalised RX2 (`x2n = g2·x2`).
* `c1_k`, `c2_k` are the image (widely-linear) taps: the AD9361's I/Q imbalance leaves a mirror of the jammer about 50 dB down at bin −k, which a linear weight cannot remove.

## 2. Signal flow — FPGA (`sfap_pi_0`, Vitis HLS, 100 MHz, II = 1)

```
 RX1 ──────────────────────────────────────────────── bypass delay (190) ──┐
 RX2 ─ g2 ─ x2n                                                           │
            │                                                             │
  chain A (frames at n ≡ 0 mod 64)                                        │
   window → FFT64(x1), FFT64(x2n) → 1-frame bin buffer → per-bin weights  │
          → IFFT64 → window ─────────────────────────────┐               │
  chain B (frames at n ≡ 32 mod 64): same               (+) ──────────── mux → ×2^s → TX1
          → window ─────────────────────────────────────┘
```

1. **RX2 normalisation:** `x2n = g2·x2` (AJ_NORM, Q3.13), as in v4.1.
2. **Analysis:** sqrt-Hann window `sin(πn/64)`, 50 % overlap. Two frame chains offset by 32 samples run on the same sample counter.
3. **FFT:** radix-2 single-path delay-feedback (R2SDF), decimation in frequency. Natural order in, bit-reversed out, one sample per clock, latency 63.
4. **Bin buffer:** holds one frame, so bin k and its image partner −k of the same frame are both available (+64 latency).
5. **Weights:** `b_k, c1_k, c2_k` from the 3×64-word committed register bank.
6. **IFFT:** R2SDF decimation in time, bit-reversed in, natural out, ÷2 per stage, latency 63.
7. **Synthesis:** same window. Chain A + chain B are summed (`w[n]² + w[n+32]² = 1`, so zero weights reconstruct x1 exactly).
8. **Bypass** (`AJ_CFG[0]`): `y = x1[n−190]` exactly, from a plain delay line of the same length, so switching the filter in or out never moves code phase.
9. **Make-up shift** `2^s` (`AJ_CFG[10:8]`), rounding, saturation. Unchanged from v4.1.

**Latency: 190 samples** in both modes (3.1 µs at 61.44 MS/s). It is a common delay on every satellite and is absorbed by the receiver clock bias. v4.1 had 8 samples.

All twiddle multiplies sit outside the delay-feedback loops, so every loop-carried path is a single add. That is what makes II = 1 at 100 MHz straightforward.

## 3. Firmware flow (ARM, `gnss_aj.c` + `gnss_aj_math.c`)

The state machine is unchanged:

* **CLEAR:** the filter is bypassed. Each 4096-sample DMA capture goes through a cheap detector (time-domain powers + RX1/RX2 coherence). Meanwhile the noise floor and g2 are learned.
* **JAMMED:** every capture is solved, then the weights are committed (`AJ_COMMIT`) together with cfg and g2.

The new **per-bin solver** (`aj_compute`):

1. Runs the FPGA's own analysis bank in double precision: 127 frames (hop 32) of 64-point windowed FFTs of x1 and g2·x2.
2. Builds per-bin 4×4 statistics of `[X1_k, X2n_k, conj(X1_−k), conj(X2n_−k)]` and averages them with the previous block (AJ_AVG).
3. Solves each bin with diagonal loading `(M_k + L_k·D)w = g_k`. `L_k` is the bin's own noise, iterated from the canceller output (`Pout/(1+|w|²)`). A bin with no jammer gets w ≈ 0, so its satellites are untouched.
4. Computes **suppression / residual** by running the *previous* weights through the FPGA's real synthesis (IFFT + window + overlap-add) on the new capture. The figures therefore describe what TX1 actually carried (out-of-sample). The analysis-domain bin power overstated the residual by 5–10 dB at high J/N.
5. Mode switch `gnss_aj_mode=`: **0** = per bin b + image taps (default), **1** = per bin b only, **2** = one weight for the whole band (the old v3).

## 4. Register map changes (`gnss_passthrough` v5.0)

| Offset | v4.1 | v5.0 |
|---|---|---|
| 0x04 VERSION | 0x00040001 | **0x00050000** |
| 0x100–0x17C | AJ_W0..31 (space-time) | removed (reads 0) |
| 0x184 AJ_INFO | 0x10080C02 | **0x40200C03** (64 bins, hop 32, Q3.12, layout 3) |
| 0x188 AJ_CFG | bypass = x1[n−8] | bypass = x1[n−190] |
| 0x190/0x194 | AJ_WC0/1 | removed |
| 0x200–0x2FC | — | **AJ_WB[k]** `b_k`, Q3.12 {im, re} |
| 0x300–0x3FC | — | **AJ_WC1[k]** `c1_k` |
| 0x400–0x4FC | — | **AJ_WC2[k]** `c2_k` |

`AJ_COMMIT` (0x180) still copies all shadow registers in one clock. CONTROL[5] is now `sfap_en`. The firmware refuses to start anti-jam unless the bitstream reports 5.0 / 0x40200C03.

## 5. Verification (C simulation, firmware solver + HLS core together)

`Source/HLS/sfap_pi/testbench.cpp` uses the same signal model as the v4.1 testbench, plus a new two-jammer case.

| # | Case | SFAP-PI result | Pass criterion |
|---|---|---|---|
| 1 | Core vs. double-precision WOLA model | 0.42 LSB rms error, latency exactly 190, bypass bit-exact, saturates without wrap | ≤ 1 LSB |
| 2 | Wideband 30 dB | **46.2 dB** suppression (one weight: 14.5 dB), satellite loss −1.8 dB | ≥ 35 dB |
| 3 | No jammer | bypass bit-exact; filter in: +0.01 dB, satellite loss 0.21 dB | ≤ 0.5 dB |
| 4 | CW 40 dB | **62.9 dB** | ≥ 40 dB |
| 5 | Wideband 30 dB + reflection | **35.1 dB** | ≥ 30 dB |
| 6 | Wideband 50 dB + I/Q imbalance | **56.6 dB**, residual +0.2 dB (no image taps: 44.2 dB, +4.5 dB) | ≥ 45 dB |
| 7 | RX2 25 dB weaker, 40 dB | **38.8 dB** (no g2: 9.3 dB) | ≥ 35 dB |
| 8 | Wideband 60 dB + I/Q imbalance | **56.7 dB** | ≥ 50 dB |
| 9 | Two CW jammers, two directions | **60.7 dB** (one weight: 0.3 dB), satellite loss 3.6 dB | ≥ 35 dB, loss ≤ 5 dB |

The firmware's own suppression/residual figures now track the measured ones (scenario 6: 57.8 dB / +0.4 dB reported vs 56.6 dB / +0.2 dB measured).

## 6. Build: sources → BOOT.BIN (Linux, Vivado/Vitis 2023.2)

```bash
# 0. put this package in place
mkdir -p /home/manoranjan/CRPA_Project/SFAP
cd /home/manoranjan/CRPA_Project/SFAP
unzip SFAP_snd2.zip                 # -> SFAP_snd2/{Source,Automation,README_SFAP.md}
cd SFAP_snd2/Automation/Linux
chmod +x *.sh

# 1. Xilinx tools (or let build_all.sh source them from /tools/Xilinx)
source /tools/Xilinx/Vivado/2023.2/settings64.sh
source /tools/Xilinx/Vitis/2023.2/settings64.sh
source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh

# 2. ADI HDL library, once. Either reuse the copy your STAP project built with
#    (faster: its library IPs are already packaged) ...
./setup_vendor.sh /home/manoranjan/CRPA_Project/<STAP project>/Vendor/ADI_hdl_2023_r2
#    ... or clone it fresh:
# ./setup_vendor.sh

# 3. optional, 30 s: the 9 C-simulation scenarios with g++
./csim_gcc.sh

# 4. everything, to BOOT.BIN
./build_all.sh
```

Stages and outputs (each logs to `Build/logs/<stage>.log`):

| Stage | Tool | Output |
|---|---|---|
| `check` | — | tools, sources, ADI tree present |
| `hls` | vitis_hls | 9 csim scenarios, csynth (II = 1, ≤ 10 ns checked), `Build/ip_repo/sfap_pi` (VLNV antsdr:gnss:sfap_pi:3.0) |
| `ip` | vivado | `Build/ip_repo/gnss_passthrough` |
| `hdl` | ADI make + vivado | ADI libraries, project in `Vendor/ADI_hdl_2023_r2/projects/gnss_sfap/e310`, bitstream + XSA → `Build/hdl/`. **Fails if timing is not met.** |
| `sw` | xsct | platform, FSBL, BSP, `app_gnss_e310.elf` → `Build/sw/` |
| `boot` | bootgen | `Build/boot/BOOT.BIN` (+ `boot.bif`, sha256) |

Re-run from a stage: `./build_all.sh --from hdl`; run a single stage: `./build_all.sh --only sw`.
Skip the HLS csim: `--skip-csim`. Paths can be overridden in `env.sh` (SFAP_ROOT, XILINX_ROOT, ADI_HDL_DIR…).

### Boot

* **SD:** copy `Build/boot/BOOT.BIN` to a FAT32 SD card, set the boot jumpers to SD, power up. The console is UART1 at **115200 8N1**.
* **JTAG (development):** boot jumpers on JTAG, then `./build_all.sh --only boot --jtag`. This programs the bitstream, runs ps7_init, downloads the ELF and starts it.

### First checks on the console

```
gnss_status?        VERSION must read 0x00050000
gnss_aj?            "SFAP 64 bins, b + 2 image taps per bin", bypass until a jammer
gnss_report?        full report: per-bin info ("bins: N of 64 with J/N > 3 dB")
gnss_tx=1           start retransmitting (same interlock as before)
gnss_aj_mode=2      compare against the old single weight
```

## 7. Things to watch

* **Not yet run through Vivado/Vitis HLS here.** The algorithm and the C++ core are verified in C simulation (with the real `ap_int` headers), and the Verilog elaborates cleanly. Synthesis, the HLS II/clock check and place-and-route timing happen on your machine; the `hls` and `hdl` stages stop with a clear message if II = 1, 10 ns or timing closure is missed.
* **Resources (estimate):** about 140 DSP48 of 220 (6 FFTs + weights + windows); v4.1 used ~103. FFT delay lines → SRLs, bin buffers → LUT/BRAM. `gnss_passthrough` grows by ~12 k FFs (192-word shadow + active bank). The HLS stage prints its estimate; `hdl` prints post-place utilisation.
* **Latency 190 samples.** Harmless for a retransmitting CRPA, but it is a constant offset if you ever compare TX1 timing against RX1 directly.
* **Weight commit mid-frame:** the frame being weighted at the commit instant mixes old and new weights. That is a ~1 µs transient, smoothed by the synthesis window.
* **Two antennas = one null direction per bin.** A wideband jammer plus a second jammer in the same bins from another direction cannot both be nulled. Two jammers at different frequencies can (scenario 9).
