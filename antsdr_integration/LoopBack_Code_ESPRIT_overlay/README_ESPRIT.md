# LoopBack_Code + ESPRIT DOA — quick start

This tree is the ANTSDR E310 V1 LoopBack pipeline with an **ESPRIT
direction-of-arrival monitor** added (gnss_passthrough **v1.2**, MOD-11). It
implements both algorithms of Li et al., *IEEE TIM* 74 (2025) 2003209:
**I-UESPRIT** and **M-ESPRIT**, side by side on RX1/RX2. The block is
receive-only; the RX1 → TX1 passthrough is unchanged.

**Full documentation (mathematics, architecture, register map, every test with
the output to expect):** [`Docs/ESPRIT/README_ESPRIT.md`](Docs/ESPRIT/README_ESPRIT.md)

## 1. Simulate (Vivado Tcl console, any Vivado incl. 2023.2)

```tcl
source <path>/LoopBack_Code/Source/Sim/ESPRIT/sim_doa.tcl
# -> DOA_TB_RESULT: PASS (61 checks)
# -> DOA_SIM_RESULT: PASS
set DOA_SIM_GUI 1 ; source <path>/LoopBack_Code/Source/Sim/ESPRIT/sim_doa.tcl   ;# waveforms
```

## 2. Build (the Vivado/Vitis version this pipeline is normally built with)

```bash
vivado -mode batch -source Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl -tclargs $PWD/Source $PWD/Build/ip_repo   # MUST re-package (v1.2)
# then the usual create_project.tcl + build.tcl, or on Linux:
export ADI_HDL_DIR=<packaged ADI hdl working copy>
Vivado/Scripts/build_esprit_linux.sh            # sim, ip, project, build, report
Vivado/Scripts/build_esprit_linux.sh sw         # firmware
```

After implementation, run this in the Tcl console:

```tcl
open_run impl_1
source Vivado/Scripts/report_doa.tcl
```

It prints `DOA_IMPL:` lines with utilisation, timing and CDC-constraint coverage.

## 2b. Separate projects: I-UESPRIT only / M-ESPRIT only

One packaged IP serves all three builds; `GNSS_DOA_ALGO` (read by
`Source/BlockDesign/system_bd.tcl`) sets the IP's `ENABLE_IUESPRIT` /
`ENABLE_MESPRIT` per project. Create each project from its own directory:

| GNSS_DOA_ALGO | project directory          | estimators        |
|---------------|----------------------------|-------------------|
| `both` (default) | `Vivado/Project`        | I-UESPRIT + M-ESPRIT |
| `iuesprit`    | `Vivado/Project_IUESPRIT`  | I-UESPRIT only    |
| `mesprit`     | `Vivado/Project_MESPRIT`   | M-ESPRIT only     |

```bash
export GNSS_DOA_ALGO=mesprit                 # or iuesprit
mkdir -p Vivado/Project_MESPRIT && cd Vivado/Project_MESPRIT
vivado -mode batch -source ../Scripts/create_project.tcl   # prints GNSS_DOA_ALGO: mesprit (...)
```

The same `GNSS_DOA_ALGO` selects the matching simulation (`sim_doa.tcl`,
`run_xsim.sh`, `run_iverilog.sh`, `fw_cosim/run_fw_cosim.sh`). The
firmware is identical for all three: it reads `DOA_CAPS` and prints and tests
only the estimator(s) present.

## 2c. HLS variants (estimator as a Vitis HLS IP cell)

```bash
vitis_hls -f Source/HLS/esprit_hls/run_hls.tcl -tclargs m  $PWD/Build/hls_repo   # esprit_m_hls
vitis_hls -f Source/HLS/esprit_hls/run_hls.tcl -tclargs iu $PWD/Build/hls_repo   # esprit_iu_hls
export GNSS_DOA_IMPL=hls GNSS_DOA_ALGO=mesprit GNSS_HLS_IP_REPO=$PWD/Build/hls_repo
mkdir -p Vivado/Project_HLS_MESPRIT && cd Vivado/Project_HLS_MESPRIT
vivado -mode batch -source ../Scripts/create_project.tcl
```
See `Source/HLS/esprit_hls/README.md` and `Docs/ESPRIT/README_ESPRIT.md` section 7.1c.

## 3. Hardware (PuTTY, 115200 8N1)

On boot, the console prints `DOA_SELFTEST: ... RESULT PASS (11/11 checks passed)`.

```
doa_selftest?          built-in known-angle test, PASS/FAIL
doa_test=-30           inject a known angle
doa_spacing_mm=95      antenna spacing (95 mm = lambda/2 at GPS L1)
doa_cal=1              same signal on RX1+RX2 (splitter): zero the board phase
doa?                   live angle, both estimators
doa_monitor=20         20 averaged results
doa_help?              every DOA command
```

JTAG alternative: `xsdb Vitis/Scripts/doa_jtag_test.tcl` (prints `DOA_JTAG_RESULT: PASS`).

`Build/ip_repo` is empty: package the IP (step above) before creating the
project, or `system_bd.tcl` stops with "OLD v1.1 package". The files in
`Vivado/Project/` are old v1.1 outputs: re-create the project before building.
