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

The files under `Build/ip_repo/gnss_passthrough` and `Vivado/Project/` are the
old v1.1 outputs from the original archive. Re-package and re-create the
project before building.
