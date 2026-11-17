# Build commands: sources → BOOT.BIN, one CRPA algorithm per build

Each algorithm is a **separate build** with its own bitstream, firmware and
`BOOT.BIN`. A `BOOT.BIN` runs only the algorithm it was built for: the
firmware checks `AJ_INFO` and will not start on another algorithm's
bitstream.

| algorithm | FPGA core | taps (RX1 + RX2) | BOOT.BIN |
|---|---|---|---|
| `crpa1`  | smi_pi  | 6 + 6   | `Build/crpa1/boot/BOOT.BIN`  |
| `crpa27` | stap_pi | 8 + 8   | `Build/crpa27/boot/BOOT.BIN` |
| `crpa28` | stap_pi | 10 + 10 | `Build/crpa28/boot/BOOT.BIN` |
| `crpa29` | stap_pi | 14 + 14 | `Build/crpa29/boot/BOOT.BIN` |
| `crpa30` | stap_pi | 16 + 16 | `Build/crpa30/boot/BOOT.BIN` |
| `crpa31` | stap_pi | 18 + 18 | `Build/crpa31/boot/BOOT.BIN` |

## 1. Once per shell: tools and paths

```bash
cd CRPA_LVDS/Automation/Linux
```

```bash
chmod +x *.sh ../../Source/HDL/sim/run_iverilog.sh ../../Source/Firmware/host_test/run_engine_test.sh
```

```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh
```

```bash
source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh
```

```bash
source /tools/Xilinx/Vitis/2023.2/settings64.sh
```

If your ADI hdl tree (the one that holds `projects/passthrough_lvds`) is not
the default in `env.sh`:

```bash
export ADI_HDL_DIR=$HOME/CRPA_Project/NEW_PASSTHROUGH/workspace/antsdr-fw-patch/plutosdr-fw/hdl
```

## 2. Check before a long build

```bash
./build_all.sh --algo crpa27 --only check
```

## 3. Build one algorithm, all the way to BOOT.BIN

Pick one. Each runs check → hls → ip → hdl → sw → boot (roughly 40–100 minutes):

`tee` needs `Build/` to exist the first time:

```bash
mkdir -p ../../Build
```

```bash
./build_all.sh --algo crpa1  2>&1 | tee ../../Build/build_crpa1.log
```

```bash
./build_all.sh --algo crpa27 2>&1 | tee ../../Build/build_crpa27.log
```

```bash
./build_all.sh --algo crpa28 2>&1 | tee ../../Build/build_crpa28.log
```

```bash
./build_all.sh --algo crpa29 2>&1 | tee ../../Build/build_crpa29.log
```

```bash
./build_all.sh --algo crpa30 2>&1 | tee ../../Build/build_crpa30.log
```

```bash
./build_all.sh --algo crpa31 2>&1 | tee ../../Build/build_crpa31.log
```

## 4. Or all six, one after the other (with a summary at the end)

```bash
./build_all.sh --algo all 2>&1 | tee ../../Build/build_all_crpa.log
```

## 5. Stage by stage (same result as step 3, for when one stage fails)

Each stage logs to `Build/<algo>/logs/<stage>.log`.

```bash
./build_all.sh --algo crpa27 --only hls     # Vitis HLS: C-sim (6 scenarios) + synthesis + IP
```

```bash
./build_all.sh --algo crpa27 --only ip      # package gnss_passthrough with crpa27's sizes
```

```bash
./build_all.sh --algo crpa27 --only hdl     # Vivado: block design, implementation, bitstream, XSA
```

```bash
./build_all.sh --algo crpa27 --only sw      # firmware staged + XSCT: FSBL + app_crpa_lvds.elf
```

```bash
./build_all.sh --algo crpa27 --only boot    # bootgen -> Build/crpa27/boot/BOOT.BIN
```

To restart from a stage and run everything after it:

```bash
./build_all.sh --algo crpa27 --from hdl
```

## 6. Optional checks that need no Vivado

```bash
./csim_gcc.sh                                   # all algorithms, the core with its ARM solver (g++)
```

```bash
../../Source/HDL/sim/run_iverilog.sh            # gnss_passthrough register map / commit, every algorithm
```

```bash
../../Source/Firmware/host_test/run_engine_test.sh   # the ARM engine against simulated captures
```

## 7. On the board

Copy `Build/<algo>/boot/BOOT.BIN` to the root of a FAT32 SD card, set the boot
jumpers to SD and power up. On the console:

```
gnss_status?     must show version 7.0 and AJ_INFO for the algorithm (e.g. crpa27 = 0x1b080407)
crpa_report?     algorithm, solver, jammer state, everything
```

Or run it over JTAG straight after a build:

```bash
./build_all.sh --algo crpa27 --from boot --jtag
```
