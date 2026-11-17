# CRPA packages: one zip per algorithm, separate build commands

All six are **bare-metal (no-OS)**, not Linux. Each runs on the Xilinx
standalone BSP with the Analog Devices no-OS AD9361 driver. Each `BOOT.BIN`
holds FSBL + bitstream + `app_crpa_lvds.elf` only.

| zip | FPGA core | taps (RX1 + RX2) | result |
|---|---|---|---|
| `CRPA_LVDS_crpa1.zip`  | smi_pi  | 6 + 6   | `CRPA_LVDS_crpa1/Build/crpa1/boot/BOOT.BIN` |
| `CRPA_LVDS_crpa27.zip` | stap_pi | 8 + 8   | `CRPA_LVDS_crpa27/Build/crpa27/boot/BOOT.BIN` |
| `CRPA_LVDS_crpa28.zip` | stap_pi | 10 + 10 | `CRPA_LVDS_crpa28/Build/crpa28/boot/BOOT.BIN` |
| `CRPA_LVDS_crpa29.zip` | stap_pi | 14 + 14 | `CRPA_LVDS_crpa29/Build/crpa29/boot/BOOT.BIN` |
| `CRPA_LVDS_crpa30.zip` | stap_pi | 16 + 16 | `CRPA_LVDS_crpa30/Build/crpa30/boot/BOOT.BIN` |
| `CRPA_LVDS_crpa31.zip` | stap_pi | 18 + 18 | `CRPA_LVDS_crpa31/Build/crpa31/boot/BOOT.BIN` |

Every zip has its own `README_<algo>.md`, its own `BUILD_COMMANDS_<algo>.md`
(including stage-by-stage commands) and its own `test/` results. The commands
below are the short version.

## Xilinx tools (once per shell, for every package)

```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh
```

```bash
source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh
```

```bash
source /tools/Xilinx/Vitis/2023.2/settings64.sh
```

```bash
export ADI_HDL_DIR=$HOME/CRPA_Project/NEW_PASSTHROUGH/workspace/antsdr-fw-patch/plutosdr-fw/hdl
```

## crpa1

```bash
unzip CRPA_LVDS_crpa1.zip && cd CRPA_LVDS_crpa1/Automation/Linux && chmod +x *.sh
```

```bash
./build_all.sh --only check
```

```bash
mkdir -p ../../Build && ./build_all.sh 2>&1 | tee ../../Build/build_crpa1.log
```

The result is `CRPA_LVDS_crpa1/Build/crpa1/boot/BOOT.BIN`.

```bash
cd ../../..
```

## crpa27

```bash
unzip CRPA_LVDS_crpa27.zip && cd CRPA_LVDS_crpa27/Automation/Linux && chmod +x *.sh
```

```bash
./build_all.sh --only check
```

```bash
mkdir -p ../../Build && ./build_all.sh 2>&1 | tee ../../Build/build_crpa27.log
```

The result is `CRPA_LVDS_crpa27/Build/crpa27/boot/BOOT.BIN`.

```bash
cd ../../..
```

## crpa28

```bash
unzip CRPA_LVDS_crpa28.zip && cd CRPA_LVDS_crpa28/Automation/Linux && chmod +x *.sh
```

```bash
./build_all.sh --only check
```

```bash
mkdir -p ../../Build && ./build_all.sh 2>&1 | tee ../../Build/build_crpa28.log
```

The result is `CRPA_LVDS_crpa28/Build/crpa28/boot/BOOT.BIN`.

```bash
cd ../../..
```

## crpa29

```bash
unzip CRPA_LVDS_crpa29.zip && cd CRPA_LVDS_crpa29/Automation/Linux && chmod +x *.sh
```

```bash
./build_all.sh --only check
```

```bash
mkdir -p ../../Build && ./build_all.sh 2>&1 | tee ../../Build/build_crpa29.log
```

The result is `CRPA_LVDS_crpa29/Build/crpa29/boot/BOOT.BIN`.

```bash
cd ../../..
```

## crpa30

```bash
unzip CRPA_LVDS_crpa30.zip && cd CRPA_LVDS_crpa30/Automation/Linux && chmod +x *.sh
```

```bash
./build_all.sh --only check
```

```bash
mkdir -p ../../Build && ./build_all.sh 2>&1 | tee ../../Build/build_crpa30.log
```

The result is `CRPA_LVDS_crpa30/Build/crpa30/boot/BOOT.BIN`.

```bash
cd ../../..
```

## crpa31

```bash
unzip CRPA_LVDS_crpa31.zip && cd CRPA_LVDS_crpa31/Automation/Linux && chmod +x *.sh
```

```bash
./build_all.sh --only check
```

```bash
mkdir -p ../../Build && ./build_all.sh 2>&1 | tee ../../Build/build_crpa31.log
```

The result is `CRPA_LVDS_crpa31/Build/crpa31/boot/BOOT.BIN`.

```bash
cd ../../..
```

## Check the zips

```bash
sha256sum -c SHA256SUMS
```
