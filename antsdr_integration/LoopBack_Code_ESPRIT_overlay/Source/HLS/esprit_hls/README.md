# ESPRIT HLS IPs: `esprit_iu_hls` (I-UESPRIT) and `esprit_m_hls` (M-ESPRIT)

These are Vitis HLS versions of the two ESPRIT DOA estimators from Li et al., *IEEE TIM* 74 (2025) 2003209. Each one sits in the
block design as its **own cell**, the same way as the SMI power-inversion HLS IP:

```
                 ┌──────────────────────────┐   m_axis_doa (AXIS 64b)    ┌──────────────────────┐
  axi_ad9361 ───►│     gnss_passthrough      │──────────────────────────►│ s_axis_x              │
  RX1 / RX2      │  (v1.2, HLS_CORE = 1)     │   doa_cfg_log2_snap ─────►│ log2_snap[4:0]        │
                 │  capture, CDC FIFO,       │   doa_cfg_p_log2    ─────►│ p_log2[1:0]           │
                 │  test gen, CAL, 1024-deep │   doa_cfg_use_cov   ─────►│ use_cov[0:0]          │
                 │  stream buffer, result    │   doa_cfg_mu_q30    ─────►│ mu_q30[31:0]          │
                 │  registers 0x100-0x194    │   doa_cfg_k_q30     ─────►│ k_q30[31:0]           │
                 │                           │   doa_cfg_uctol_q24 ─────►│ uctol_q24[31:0]  (M)  │
                 │                           │   doa_hls_rst_n     ─────►│ ap_rst_n              │
                 │                           │◄──────────────────────────│ m_axis_y   (AXIS 32b) │
                 └──────────────────────────┘   s_axis_res              │ esprit_m_hls_0        │
                  sys_cpu_clk (100 MHz) ────────────────────────────────►│ ap_clk                │
                                                                          └──────────────────────┘
```

The register map and firmware stay the same. `gnss_passthrough` writes the HLS results into the same registers the RTL cores
use. `DOA_CAPS` bit 2 tells the firmware that the estimator is an HLS IP.

## Files
| File | Purpose |
|---|---|
| `esprit_hls.h` | interface, types, result-packet layout |
| `esprit_core.h` | the algorithm, shared by both tops (moments, Eq. (7) cumulant, C^p, closed form, DOA) |
| `esprit_iu_hls.cpp` | top function `esprit_iu_hls` (I-UESPRIT) |
| `esprit_m_hls.cpp` | top function `esprit_m_hls` (M-ESPRIT, adds `uctol_q24` and the unit-circle flag) |
| `tb_esprit_hls.cpp` | C-sim testbench: compares against an independent double-precision reference and the true angle |
| `run_hls.tcl` | runs csim → csynth → (cosim) → export to the IP catalog |

## Kernel behaviour
- **Free-running.** `ap_ctrl_none`, so each invocation processes one window and the hardware restarts it straight away.
- **Input loop.** Reads 2^log2_snap snapshots at II = 1 and accumulates exact integer moments (48- and 80-bit).
- **Once per window.** Forms C in double precision, normalises it, takes C^p, applies the M = 2 closed form, then computes
  `atan2` and `asin`.
- **Result packet.** Nine words: magic `0xE5A1` + flags, φ (Q28), ψ (Q8.24), θ (Q28), degrees (Q16), and normalised C (Q30).

## Verified (here, with g++ and the Xilinx `ap_int` headers)
- **C-sim:** I-UESPRIT 36/36 checks and M-ESPRIT 37/37. The kernel matches the double-precision reference within
  0.00001°; worst error against the true angle is 0.21° at 20 dB SNR.
- **Against the Python models in `ip/*/model`:** within 0.0001° on QPSK windows.
- **Firmware co-sim** (`Source/Sim/ESPRIT/fw_cosim`, `GNSS_DOA_IMPL=hls`): Verilator RTL with `HLS_CORE=1`, this C++ kernel as
  the HLS IP (400-cycle stall per window), and the real `gnss_doa.c`.
  - Self-test: 8/8 (I-UESPRIT) and 9/9 (M-ESPRIT).
  - Live angles within 0.05° after calibration.
  - About 1,300 windows processed in each run.

## Build
```bash
source <Vitis>/settings64.sh
mkdir -p <LoopBack_Code>/Build/hls_work && cd <LoopBack_Code>/Build/hls_work
vitis_hls -f <LoopBack_Code>/Source/HLS/esprit_hls/run_hls.tcl -tclargs m  <LoopBack_Code>/Build/hls_repo
vitis_hls -f <LoopBack_Code>/Source/HLS/esprit_hls/run_hls.tcl -tclargs iu <LoopBack_Code>/Build/hls_repo
```
Then create the project with `GNSS_DOA_IMPL=hls`, `GNSS_DOA_ALGO=mesprit|iuesprit` and
`GNSS_HLS_IP_REPO=<LoopBack_Code>/Build/hls_repo` (see `Docs/ESPRIT/README_ESPRIT.md`, section 7.1c).
