# GNSS-CRPA ANTSDR E310 V1 loopback — command-line Vivado build

RX1 → AD9361 → `axi_ad9361` → **`gnss_passthrough`** (custom PL block) →
`axi_ad9361` → TX1, on a MicroPhase ANTSDR E310 V1 (Zynq-7020, `xc7z020clg400-2`).
`gnss_passthrough` is currently a transparent I/Q pass-through: the insertion
point for a future CRPA anti-jam algorithm. See `Source/Config/board_e310_v1.json`
for the full hardware/register-map reference.

This file covers **only** the FPGA side: turning `Source/` + `Vendor/` into a
`.bit` file with Vivado. Nothing here opens the Vivado GUI — every step runs
through `vivado -mode batch`. Firmware (`Source/Firmware`, built via
`Vitis/Scripts`) is a separate, later step this build does not run.

## Prerequisites

- Vivado with Zynq-7000 device support (developed against 2026.1; see
  "Vivado version" below for other versions).
- [PowerShell 7](https://aka.ms/powershell) (`pwsh`) — the build driver is a
  single cross-platform script, invoked identically on Windows and Linux.

## Build it

From a **Windows Command Prompt**:

```bat
cd LoopBack_Code
build.bat
```

From **PowerShell** (Windows, Linux or macOS):

```powershell
cd LoopBack_Code
pwsh -File Automation/PowerShell/Build_All.ps1
```

From a **Linux/macOS terminal**:

```bash
cd LoopBack_Code
./build.sh
```

All three run the same script and produce, on success:

```
Build/Bitstream/antsdr_e310_gnss.bit    <- the bitstream
Build/Bitstream/antsdr_e310_gnss.xsa    <- hardware handoff for Vitis
```

The first run auto-detects your Vivado install (PATH, standard install
locations, Windows Start Menu shortcuts) and writes
`Source/Config/local.settings.json` from the template. If auto-detection
fails, open that file and set `toolchain.install_root` to the directory
containing `Vivado/bin/vivado(.bat)`.

## What it does

`Automation/PowerShell/Build_All.ps1` runs five stages, each one an explicit
`vivado -mode batch -source ...` call — the same Tcl scripts an engineer could
run by hand, just sequenced with the right working directory and environment
variables set:

| Stage | What | Tcl script |
|---|---|---|
| 1 | Resolve the Vivado toolchain and per-machine settings | — |
| 2 | Copy the ADI HDL vendor tree into a disposable working copy (`Vendor/` is never written to) | — |
| 3 | Package the 12 ADI library cores this design uses, plus the custom `gnss_passthrough` IP | `Vivado/Scripts/package_library_ip.tcl`, `Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl` |
| 4 | Reconstruct the Vivado project from `Source/` | `Vivado/Scripts/create_project.tcl` |
| 5 | Synthesise, implement, write the bitstream + XSA | `Vivado/Scripts/build.tcl` |

A pass is only reported once synthesis reaches `synth_design Complete!`,
implementation reaches `write_bitstream Complete!`, **and** the `.bit`/`.xsa`
files exist and were written by that run — never on file existence alone.
Timing (setup/hold slack) is reported either way; a bitstream produced with
violated timing is flagged, not hidden.

Everything generated lives under `Build/` (git-ignored) and
`Vivado/Project/` (also git-ignored; it is regenerated from
`Source/BlockDesign/system_bd.tcl` on every run, so it is never hand-edited
as the source of truth). Per-stage logs land in `Build/Logs/`.

### Resuming a partial build

```powershell
pwsh -File Automation/PowerShell/Build_All.ps1 -FromStage 5   # just re-run synth/impl/bitstream
pwsh -File Automation/PowerShell/Build_All.ps1 -FromStage 3   # re-package IP, then create + build
pwsh -File Automation/PowerShell/Build_All.ps1 -CleanVendorWork  # force a fresh vendor copy first
pwsh -File Automation/PowerShell/Build_All.ps1 -Jobs 8        # override parallel synth/impl jobs
```

Resuming past a stage whose output doesn't exist yet fails immediately with a
clear message rather than guessing.

### Vivado version

The upstream ADI scripts hard-require Vivado 2021.1 and abort on anything
else. `Build_All.ps1` detects your installed version and sets
`REQUIRED_VIVADO_VERSION` to match, so the check passes on whatever version
you actually have. To pin a specific string instead, set
`build.force_required_version` in `Source/Config/local.settings.json`.

## Directory layout

```
Source/     authoritative HDL, constraints, block design, config, firmware
Vendor/     read-only third-party trees (ADI HDL library, MicroPhase E310 reference)
Vivado/     Scripts/ (Tcl, tracked) + Project/ (generated, git-ignored)
Vitis/      firmware build scripts (separate stage, not run by this build)
Automation/ Build_All.ps1 - the script documented above
Build/      everything this build generates (git-ignored)
SD_Image/   a previously produced deployment image (not an input or output of this build)
```

## Before transmitting anything

`SD_Image/README_SD_CARD.md` documents a **different**, already-built firmware
image that transmits on the GPS L1 frequency automatically at power-on. That
image, and the auto-TX firmware option it comes from, are out of scope here —
this build only produces the FPGA bitstream. If you go on to test with RF
connected, read that file's safety notes first: conducted/attenuated coax
only, never an antenna.
