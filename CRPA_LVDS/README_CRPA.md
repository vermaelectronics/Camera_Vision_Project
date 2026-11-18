# CRPA_LVDS: Passthrough_LVDS + CRPA space-time anti-jam (ANTSDR E310)

This package uses the same build pipeline as the earlier frequency-domain
package (`SFAP_PI_256v3.2`). The **SFAP filter has been removed**. In its place
are the six CRPA algorithms delivered in `crpa_algo_1_27-31.zip`:

| algorithm | FPGA core (Vitis HLS) | taps RX1 + RX2 | ARM weight solver |
|---|---|---|---|
| `crpa1`  | `smi_pi` 4.9 | 6 + 6, REF 3   | SMI: averaged statistics, Cholesky, loading from the canceller's own noise, slow-sweep detector, 4 tap modes |
| `crpa27` | `stap_pi` 1.0 | 8 + 8, REF 4   | Direct (Wiener) solve at the start of a jammer, then NLMS on every capture. Moving-jammer tracking (re-solve when the null is lost) and weight prediction. |
| `crpa28` | `stap_pi` 1.0 | 10 + 10, REF 5 | the same code as crpa27 |
| `crpa29` | `stap_pi` 1.0 | 14 + 14, REF 7 | the same code as crpa27 |
| `crpa30` | `stap_pi` 1.0 | 16 + 16, REF 8 | the same code as crpa27 |
| `crpa31` | `stap_pi` 1.0 | 18 + 18, REF 9 | the same code as crpa27 |

**One algorithm per build.** Each has its own bitstream, firmware and
`BOOT.BIN`, in `Build/<algo>/`. All six cannot share one bitstream: crpa31
alone needs about 127 of the Zynq-7020's 220 DSP48s. The exact commands are in
**[BUILD_COMMANDS.md](BUILD_COMMANDS.md)**:

```bash
./build_all.sh --algo crpa27        # or crpa1, crpa28 .. crpa31, or all
```

**Operating system: none. The firmware is bare-metal (no-OS).** It runs on
the Xilinx standalone BSP on Cortex-A9 core 0, with the Analog Devices no-OS
AD9361 driver; the DMA and console are polled from `main()`'s idle loop.
`BOOT.BIN` holds only FSBL + bitstream + `app_crpa_lvds.elf`. There is no
U-Boot, Linux kernel, device tree or root filesystem.

The algorithm sources are in `Source/CRPA/<algo>/{fpga,arm}`, exactly as
delivered. The pipeline uses them unchanged. Everything around them is shared:
`gnss_passthrough`, the block design, the ARM engine, the console and the
build scripts.

The SFAP sources, the v6.3 build outputs and the SFAP documentation are no
longer in this tree. They remain in git history at commit `aad0409` (folder
`SFAP_PI_256v3.2`).

## 1. Test results (no FPGA tools here: C, Icarus Verilog, host)

### C simulation: the real core with the real ARM solver (`test/csim_results_v7.0.txt`)

`Source/HLS/testbench.cpp` runs each algorithm's FPGA core (bit-true
`ap_int`) together with its own `gnss_aj_math.c`.

- The weights are solved from 8192-sample captures and committed as the
  firmware commits them.
- Suppression and residual are then measured on the **core's output**, using
  fresh samples.
- Residual is the output over the output noise; 0 dB means fully nulled.

| scenario | crpa1 | crpa27 | crpa28 | crpa29 | crpa30 | crpa31 |
|---|---|---|---|---|---|---|
| 1 core vs. double model (±1 LSB), bypass exact | 0 errors | 0 | 0 | 0 | 0 | 0 |
| 2 wideband 40 dB: suppression / residual | 55.7 / 0.0 | 50.1 / 0.15 | 49.9 / 0.15 | 49.1 / 0.20 | 49.4 / 0.19 | 49.6 / 0.18 |
| 3 CW 45 dB | 60 / 0.0 | 57.2 / 0.14 | 58.2 / 0.10 | 59.9 / 0.0 | 58.2 / 0.06 | 58.5 / 0.06 |
| 4 wideband 40 dB, 2-tap RX2 channel, RX2 −6 dB | 51.3 / 0.0 | 45.6 / 0.31 | 45.7 / 0.30 | 46.5 / 0.23 | 46.7 / 0.22 | 46.8 / 0.22 |
| 5 no jammer: output noise rise | 0.00 dB | 0.13 | 0.14 | 0.10 | 0.08 | 0.11 |
| 6 moving jammer, 2°/capture: residual | 11.1 (info) | 0.63 | 0.76 | 0.65 | 0.60 | 0.59 |

All 36 runs pass. Scenario 6 is informational for crpa1, because crpa1 has no
weight prediction. Its weights are one capture old, which a moving jammer
punishes. crpa27–31 were made for exactly this case.

### ARM engine on the host (`test/engine_tb_results_v7.0.txt`)

`Source/Firmware/host_test` compiles the real `gnss_aj.c`, the real
`gnss_passthrough.c` and the algorithm's solver. Everything else is
simulated:

- the DMA captures;
- the v7.0 register file;
- the filter, as a model applied with the **committed** words;
- a 20 ms solve time per capture, as on the ARM.

The timeline is: clear (the noise floor and g2 are learned), then a 40 dB
jammer, then the jammer moving at 150°/s, then no jammer.

| | crpa1 | crpa27 – crpa31 |
|---|---|---|
| JAMMER ON → NULLED → OFF → bypass | yes | yes |
| static jammer, mean residual over each hold | 0.00 dB | 0.05 dB (Kalman on) |
| moving jammer (150°/s), mean residual | 17.8 dB (no prediction) | 2.1 dB (Kalman on) |
| with `crpa_kalman=0` (simple prediction) | | static 0.25 dB, moving 2.4 dB |
| with `crpa_kalman=0` and `crpa_predict=0` | | moving 15.6 dB |
| moving 30°/s: Kalman / simple prediction | | 0.4 dB / 3.9 dB |
| moving 300°/s: Kalman / simple prediction | | 6.5 dB / 9.9 dB |

### HDL (`test/hdl_tb_results_v7.0.txt`)

`Source/HDL/sim/run_iverilog.sh` builds `gnss_passthrough.v` once for each
algorithm's sizes. It checks:

- the IDs;
- that every weight and image word reaches the core only on `AJ_COMMIT`;
- that unused port words stay at 0;
- read-back of the shadow registers;
- that a pending set does not disturb the running one;
- that writes outside the weight range are ignored;
- the reset values.

There are 151–291 checks per algorithm, and all pass.

### Firmware compile

With stub BSP headers, every changed file compiles with `gcc -Wall -Wextra`
and no warnings, for all six algorithms. The files are `gnss_aj.c`,
`gnss_passthrough.c`, `command.c`, `gnss_info.c`, `main.c`, `gnss_smi_agc.c`
and `gnss_aj_math.c`.

## 2. What changed

| | v6.3 (SFAP) | **v7.0 (CRPA)** |
|---|---|---|
| filter | 256-bin frequency domain | time-domain space-time taps of the chosen algorithm |
| `gnss_passthrough` VERSION / IP | 0x00060003 / 3.0 | **0x00070000 / 4.0** |
| AJ_INFO | 0x08070C05 | **{algo, NT, REF, 0x07}**, e.g. crpa27 = 0x1B080407 |
| weights | 3 × 256 words in a double-buffered RAM plus a mask | **shadow registers AJ_W (0x1000) and AJ_WC (0x1100)**, committed to flops that drive the core's `ap_none` ports |
| new registers | | AJ_GAIN 0x190 (fine gain, stap_pi), AJ_DIMS 0x19C |
| AJ_CFG | [0] bypass, [10:8] shift, [27:16] blanker | [0] bypass, [10:8] shift 0..7, **[14:12] weight scale ws** |
| capture | 4096 samples | 8192 samples (64 KiB at 0x11000000) |
| latency RX→TX | 782 samples | **REF samples** (3 to 9), the same in bypass |
| console | `sfap-pi*` | **`crpa*`** |

### Files

- **`Source/CRPA/<algo>/`**: the delivered algorithms, unchanged.
- **`Source/HLS/build_hls.tcl`**: one script for every algorithm.
  - It reads `CRPA_ALGO`, `CRPA_CORE` and `CRPA_CORE_VER`.
  - It runs C-sim scenarios 1–6, synthesis at 10 ns with II = 1, and a check
    of the ports.
  - It exports `antsdr:gnss:smi_pi:4.9` or `antsdr:gnss:stap_pi:1.0`.
- **`Source/HLS/testbench.cpp`**: the C simulation above. It is built with
  `-DCRPA_SMI` or `-DCRPA_STAP`.
- **`Source/HDL/gnss_passthrough.v`** v7.0:
  - Parameters: `CRPA_ALGO`, `CRPA_NT`, `CRPA_REF`, `NWTS`, `NW_USED` and
    `NWCJ`.
  - Ports: `aj_wts`, `aj_wcj`, `aj_cfg`, `aj_nrm` and `aj_gain`.
  - The SFAP weight RAM, mask RAM and memory ports are gone.
- **`Source/HDL/sim/tb_gnss_passthrough_v7.v`** and **`run_iverilog.sh`**:
  the HDL testbench and its runner.
- **`Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl`**: packages
  IP 4.0. It writes the algorithm's sizes in as the parameter defaults of a
  generated copy of the RTL.
- **`Source/BlockDesign/system_bd.tcl`**: `crpa_core_0` replaces
  `sfap_pi_0`. Its `wts`, `wcj`, `cfg`, `nrm` and (on stap_pi) `gain` are
  wired to `gnss_passthrough/aj_*`. Both async FIFOs and the clocks are
  unchanged.
- **`Source/Firmware/app_crpa_lvds/`** (renamed from `app_sfap_lvds`):
  - `gnss_aj.c` is one engine for both solver families. It picks the family
    from the copied `gnss_aj_math.h`:
    - crpa1: `aj_compute` plus `aj_sweep_update`;
    - crpa27–31: `aj_detect`, `aj_direct_solve` on the first capture, then
      `aj_nlms_solve`, `aj_track_update`, and `aj_pred_after` with a measured
      lead, then `aj_committed`.
  - The make-up gain uses `aj_makeup_shift` and, on stap_pi, the fine gain
    from `aj_makeup_fine_db`.
  - The residual shown is what the **core** ran (`aj_core_db`), not the
    in-sample residual.
  - `gnss_passthrough.c/.h`: the new register map, and
    `gnss_pt_write_aj(re, im, nw, nspt, cfg, g2, gain)`.
  - `command.c`, `gnss_info.c` and `main.c` use the `crpa_*` commands and
    print the algorithm.
- **`Source/Firmware/host_test/`**: the engine host test.
- **`Automation/Linux/`**:
  - `crpa_algos.sh` is the algorithm table.
  - `build_all.sh --algo <algo|all>`: per-algorithm build trees, a size check
    against the headers, and firmware staging. The staging copies
    `app_crpa_lvds/` plus the algorithm's `gnss_aj_math.{c,h}` and a
    generated `crpa_build.h` into `Build/<algo>/fw_src`.
  - `csim_gcc.sh` runs every algorithm.
  - The ADI project is `projects/passthrough_lvds_<algo>`.

## 3. Console (the `sfap-pi*` commands are gone)

| command | |
|---|---|
| `crpa=1` / `crpa=0` | anti-jam on (default) / off |
| `crpa?`, `crpa_aj?`, `crpa_report?` | state, status, everything |
| `crpa_log=N`, `crpa_csv=N` | status line every N s / CSV every N ms (CSV adds `algo`, `solve`, `ws`) |
| `crpa_freeze=1/0` | hold the filter |
| `crpa_mode=0..3` | **crpa1 only**: all taps / RX2 + image / one weight / no image taps |
| `crpa_direct=0..8` | **crpa27–31**: direct solves at a jammer's start (default 1) |
| `crpa_track=1/0` | **crpa27–31**: re-solve when a moving jammer leaves the null (default on) |
| `crpa_kalman=1/0` | **crpa27–31, stap7x7, stap6x6**: Kalman weight tracker (default on at boot; 0 = off) |
| `crpa_predict=1/0` | **crpa27–31**: simple weight prediction, used while `crpa_kalman=0` (default on) |
| `crpa_passes=1..8` | **crpa27–31**: NLMS passes per capture (default 2) |

A command for the other algorithm family answers `not part of crpaN`.

## 4. On the bench

1. `gnss_status?` must show version 7.0, and `crpa_report?` must show the
   algorithm you built (for example `CRPA crpa27 (stap_pi)`).
2. A wideband or CW jammer from one direction should give `*** JAMMER ON ***`,
   then `NULLED in … ms` and a residual near 0 dB.
3. For a moving jammer on crpa27–31, `crpa_report?` shows the
   `moving jammer` line with the re-solves, the prediction state and the lead.
   Compare `crpa_kalman=0` against `1` (the `Kalman tracker` line shows its steps and manoeuvres).

## 5. What is verified, what is not

- **Verified here:** the C simulation, the HDL testbench, the engine host test
  and the firmware compile, all as above.
- **Not done here** (there are no Xilinx tools in this environment):
  - Vitis HLS synthesis of the cores. `build_hls.tcl` stops if II > 1 or the
    clock is over 10 ns.
  - Vivado implementation and timing. `build_all.sh` stops if timing is not
    met.
  - The ARM toolchain build.
  - Any run on the board.
- crpa1 and crpa27–31 are their original packages' solvers. The
  delivered `smi_pi` and `stap_pi` cores met timing in those packages. They
  have not yet been through this block design.

## 6. Limits

- **Two antennas give one spatial null**, plus its reflections. Two jammers
  from two directions are beyond any of these algorithms.
- **crpa1 does not follow a moving jammer**: 11 dB at 2° per capture in
  C-sim. Use crpa27–31 for that.
- **More taps is not always better.** crpa27–31 differ only in tap count.
  More taps follow a more frequency-dependent RX1/RX2 channel and longer
  reflections, but they cost DSP48s (about 3 per weight) and ARM time per
  NLMS pass. In the C-sim, every tap count nulls the test jammers equally
  well.
- **Version check.** The firmware runs only on the bitstream of its own
  algorithm (VERSION ≥ 0x00070000 and AJ_INFO equal to the one it was built
  for).
