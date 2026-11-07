# Building ANTSDR E310 firmware with Vivado/Vitis 2023.2

This folder holds the MicroPhase `antsdr-fw-patch` (v0.39) patch set plus a
build wrapper. Together they build the complete ANTSDR **E310** firmware on top
of ADI `plutosdr-fw` v0.39:

| Stage | Tool | Output |
|---|---|---|
| FPGA hardware design (AD9361 + ANTSDR VCXO/PPS logic, xc7z020clg400-2) | Vivado 2023.2 | `system_top.xsa`, `system_top.bit` |
| First-stage boot loader | Vitis 2023.2 (`xsct`) | `fsbl.elf` |
| U-Boot, Linux 6.1, device tree, buildroot rootfs | Linaro GCC 7.3 (downloaded by buildroot) | `u-boot.elf`, `zImage`/`uImage`, `zynq-ant.dtb`, `rootfs.cpio.gz` |
| Boot image | `bootgen` | **`boot.bin`** (QSPI) and **`BOOT.bin`** (SD card) |
| Firmware image | `mkimage` | `ant.itb`, `ant.frm`, `ant.dfu` |

In the build system the E310 is called **`ant`** (`TARGET=ant`).

---

## 1. Requirements

* **Linux host:** Ubuntu 22.04 LTS is recommended (Vivado 2023.2 supports it).
  Ubuntu 20.04 also works.
* **Vivado 2023.2 and Vitis 2023.2**, both exactly version 2023.2.
  * Use the AMD Unified Installer 2023.2 and choose the **Vitis** product. It
    installs Vivado as well. You need Vitis for `xsct`, which builds the FSBL.
  * Device selection must include **Zynq-7000**.
  * Default install root: `/tools/Xilinx`. `/opt/Xilinx` and `~/Xilinx` are also
    detected automatically. Otherwise set `XILINX_DIR`.
  * Install the Xilinx library prerequisites once:
    ```bash
    sudo /tools/Xilinx/Vitis/2023.2/scripts/installLibs.sh
    ```
* About **40 GB** of free disk space and 16 GB RAM (8 GB minimum).
* Internet access. The build downloads about 1 GB of sources, the Linaro
  toolchain and the buildroot packages.
* A full build takes about 1.5–3 hours. Most of that is Vivado and buildroot.

You do **not** need to download the Linaro toolchain or export
`CROSS_COMPILE`/`PATH` as the vendor README says. Buildroot downloads Linaro
GCC 7.3 itself (`BR2_TOOLCHAIN_EXTERNAL_LINARO_ARM=y`), and the Makefile sets
`CROSS_COMPILE` and `PATH`.

---

## 2. One-time host setup

```bash
sudo apt-get update
```

```bash
sudo apt-get install -y git build-essential fakeroot libncurses5-dev libssl-dev ccache \
  dfu-util u-boot-tools device-tree-compiler mtools bc python3 python-is-python3 cpio zip \
  unzip rsync file wget bison flex libmpc-dev gawk patch perl locales libtinfo5 libncurses5
```

`./build.sh deps` runs the same install, and skips packages your Ubuntu
release does not have.

The vendor also recommends removing these two packages:

```bash
sudo apt-get purge gcc-arm-linux-gnueabihf
```

```bash
sudo apt-get remove libfdt-dev
```

---

## 3. Get the sources

**Option A: full source zip** (`antsdr-e310-fw-v0.39-full-src.zip`). This zip
already contains `plutosdr-fw/` with all four submodules at the pinned commits,
patched for the E310. In Step 2 and Step 3 below, `fetch` and `patch` just
report `already ...`.

Extract it **on the Linux build machine** with `unzip`. Windows tools lose the
symlinks and execute permissions that the Linux and U-Boot trees need.

```bash
unzip antsdr-e310-fw-v0.39-full-src.zip
```

```bash
cd antsdr-fw-patch
```

**Option B: build kit zip or git clone.** These contain only the patches and
scripts. Step 2 downloads the sources (about 300 MB).

```bash
unzip antsdr-e310-build-kit.zip
```

```bash
cd antsdr-fw-patch
```

or

```bash
git clone -b claude/happy-brown-uleqcf https://github.com/vermaelectronics/Camera_Vision_Project.git
```

Drop `-b claude/happy-brown-uleqcf` once this branch is merged into `main`.

```bash
cd Camera_Vision_Project/antsdr-fw-patch
```

Either way, the build still needs internet. In Step 6, buildroot downloads
the Linaro toolchain and the rootfs package sources (about 1 GB).

Use a **fresh terminal** for the build. Do **not** `source settings64.sh`
yourself, because buildroot can break with the Xilinx tools on `PATH`. The
Makefile sources `xilinx_env.sh` only for the Vivado, xsct and bootgen steps.

---

## 4. Build: one command

```bash
./build.sh all
```

That runs `check`, `fetch`, `patch`, `firmware` and `sdimg`, described below.
If Xilinx is not installed under `/tools/Xilinx`, `/opt/Xilinx` or `~/Xilinx`,
export `XILINX_DIR` first:

```bash
export XILINX_DIR=/path/to/Xilinx
```

---

## 5. Build: step by step

Run these from `antsdr-fw-patch/`. Each step can be re-run safely.

### Step 1: Check the host and the Xilinx install

```bash
./build.sh check
```

This checks the host tools, that `vivado -version` reports **2023.2**, that
`xsct` and `bootgen` are available, and the free disk space.

### Step 2: Fetch plutosdr-fw v0.39 and its submodules

```bash
./build.sh fetch
```

This clones `plutosdr-fw` into `antsdr-fw-patch/plutosdr-fw` at tag `v0.39`.
It then does a shallow fetch of each submodule at the commit v0.39 pins:

| Submodule | Commit |
|---|---|
| hdl | `065c8f186ef87ff049d279ed5859ee8d97d91808` (ADI hdl, requires Vivado 2023.2) |
| linux | `f3da30df60047dc5a0b8fa8c640be774e0f784d9` (ADI Linux 6.1) |
| u-boot-xlnx | `90401ce9ce029e5563f4dface63914d42badf5bc` |
| buildroot | `e783aadccebad3413b3d60fcfe98a25eb395d839` (2023.02.5) |

### Step 3: Apply the ANTSDR E310 patches

```bash
./build.sh patch
```

| Patch | Applied to | Adds |
|---|---|---|
| `patch/0001-add-three-ant-hdl-support-bump-to-v0.39-v1.patch` | `hdl` | `projects/ant` (E310 block design and constraints), `library/axi_vcxo_ctrl` IP |
| `patch/ant/0001-add-support-uboot.patch` | `u-boot-xlnx` | `zynq_ant_defconfig`, `zynq-ant-sdr.dts`, boot environment |
| `patch/ant/0001-add-support-linux.patch` | `linux` | `zynq-ant.dts(i)`, `zynq_ant_defconfig`, `ad5660_mp` driver |
| `patch/ant/0001-add-support-buildroot.patch` | `buildroot` | `zynq_ant_defconfig`, `board/ant/*` |
| `patch/ant/0001-add-support-makefile-scripts.patch` | `plutosdr-fw` | `TARGET=ant`, `scripts/ant.its`, `make sdimg` |

### Step 4: Hardware (Vivado 2023.2): bitstream and XSA

```bash
./build.sh hdl
```

* Builds the ADI IP libraries, then the `ant` block design, synthesis,
  implementation and bitstream. Finally it exports
  `plutosdr-fw/hdl/projects/ant/ant.sdk/system_top.xsa` and copies it to
  `plutosdr-fw/build/system_top.xsa`.
* Log: `plutosdr-fw/hdl/projects/ant/ant_vivado.log`
* To open the project in the Vivado GUI:
  ```bash
  /tools/Xilinx/Vivado/2023.2/bin/vivado plutosdr-fw/hdl/projects/ant/ant.xpr
  ```

### Step 5: Software boot loader (Vitis 2023.2 xsct): FSBL

```bash
./build.sh fsbl
```

This runs `xsct scripts/create_fsbl_project.tcl` and produces
`build/sdk/fsbl/Release/fsbl.elf`. It also extracts `build/system_top.bit` and
`build/ps7_init.*` from the XSA.

### Step 6: Full firmware: U-Boot, Linux, rootfs, boot.bin, ant.frm/.dfu

```bash
./build.sh firmware
```

This runs plutosdr-fw's `make` (target `all`). It first wipes `build/`. The
Vivado project is **not** re-implemented: the XSA is reused if the HDL did not
change, and only the FSBL is regenerated. Then it builds:

1. The buildroot toolchain (downloads Linaro GCC 7.3).
2. U-Boot `zynq_ant_defconfig`, giving `u-boot.elf` and `uboot-env.*`.
3. Linux `zynq_ant_defconfig`, giving `zImage` and `uImage`, plus `zynq-ant.dtb`.
4. The buildroot rootfs, giving `rootfs.cpio.gz`.
5. `bootgen`, giving `build/boot.bin` (FSBL + U-Boot).
6. `mkimage -f scripts/ant.its`, giving `build/ant.itb` (bitstream + kernel + dtb
   + rootfs), then `ant.frm` and `ant.dfu`.

To skip the buildroot legal-info archive and save time:

```bash
SKIP_LEGAL=1 ./build.sh firmware
```

### Step 7: SD-card boot image: BOOT.bin

```bash
./build.sh sdimg
```

This creates `plutosdr-fw/build_sdimg/` containing:

| File | Content |
|---|---|
| `BOOT.bin` | FSBL + **bitstream** + U-Boot, made with `bootgen` |
| `uImage` | Linux kernel |
| `devicetree.dtb` | `zynq-ant.dtb` |
| `uramdisk.image.gz` | rootfs as a U-Boot ramdisk |
| `uEnv.txt` | U-Boot environment |

To list everything that was produced:

```bash
./build.sh outputs
```

### The same steps with plain `make`

`build.sh` only adds checks around these commands:

```bash
cd plutosdr-fw
```

```bash
export TARGET=ant
```

```bash
export VIVADO_SETTINGS=$(realpath ../xilinx_env.sh)
```

```bash
make build/system_top.xsa
```

```bash
make build/sdk/fsbl/Release/fsbl.elf
```

```bash
make
```

```bash
rm -rf build_sdimg && make sdimg
```

> **Important:** pass variables through the environment (`export TARGET=ant`,
> `SKIP_LEGAL=1 make`). Never pass them as `make TARGET=ant` or
> `make SKIP_LEGAL=1`. ADI's `hdl/projects/scripts/project-xilinx.mk` turns
> make command-line variables into a project sub-folder (for example
> `hdl/projects/ant/TARGETant_SKIPLEGAL1/`). The step that copies
> `hdl/projects/ant/ant.sdk/system_top.xsa` then fails. This is also why the
> vendor README uses `export TARGET=...` and `sudo -E make`.

---

## 6. Output files and how to use them

| File (in `plutosdr-fw/`) | Use |
|---|---|
| `build_sdimg/BOOT.bin` + other `build_sdimg/*` | **SD-card boot.** Copy all files to a FAT32 SD card and set the E310 boot jumper to SD. |
| `build/boot.bin` | QSPI boot loader (FSBL + U-Boot, no bitstream; the bitstream is in `ant.itb`) |
| `build/boot.dfu`, `build/ant.dfu`, `build/uboot-env.dfu` | DFU flash update (see below) |
| `build/ant.frm`, `build/boot.frm` | Firmware update files with md5 checksum, PlutoSDR style |
| `build/system_top.xsa` / `system_top.bit` | Vivado hardware platform and bitstream |
| `build/antsdr-fw-v0.39*.zip`, `build/antsdr-jtag-bootstrap-*.zip` | Release archives |

Flashing QSPI over DFU (from the vendor README). Set the jumper to QSPI
boot, hold the DFU button at power-up until both LEDs are green, then run
these from `plutosdr-fw/build`:

```bash
sudo dfu-util -a firmware.dfu -D ./ant.dfu
```

```bash
sudo dfu-util -a boot.dfu -D ./boot.dfu
```

```bash
sudo dfu-util -a uboot-env.dfu -D ./uboot-env.dfu
```

---

## 7. Rebuilding and cleaning

* Re-running `./build.sh firmware` is incremental for Vivado, Linux and
  buildroot.
* To start over from pristine v0.39 sources. This drops the patches and all
  build output, but keeps the `buildroot/dl` downloads:
  ```bash
  ./build.sh reset
  ```
  ```bash
  ./build.sh patch
  ```
* To switch boards (`TARGET=e200` / `TARGET=e310v2`), run `./build.sh reset`
  first, because each board's patch changes the same files.
* The vendor scripts `patch.sh` and `resetGit.sh` still work on
  `plutosdr-fw/`. `build.sh` replaces them: it stops on the first failed
  patch and can be re-run.

---

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| `vivado not found after sourcing xilinx_env.sh` | `export XILINX_DIR=<folder containing Vivado/2023.2>` |
| `xsct not found` | Install **Vitis** 2023.2. Vivado alone does not provide `xsct`. |
| `ERROR: vivado version mismatch; expected 2023.2` | ADI hdl requires exactly 2023.2. `ADI_IGNORE_VERSION_CHECK=1` downgrades this to a warning, but other versions are not supported. |
| `ERROR: Timing Constraints NOT met!` | See `hdl/projects/ant/timing_impl.log`. The XSA is written as `ant.sdk/system_top_bad_timing.xsa`. |
| `Building <lib> library ... FAILED` | See `hdl/library/<lib>/<lib>_ip.log` |
| `libtinfo.so.5: cannot open shared object file` | `sudo apt-get install libtinfo5 libncurses5`. These exist on Ubuntu 20.04/22.04 but not on 24.04, which is one reason to use 22.04. |
| Linaro toolchain download fails | Buildroot also tries `sources.buildroot.net`. You can also put `gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf.tar.xz` into `plutosdr-fw/buildroot/dl/toolchain-external-linaro-arm/`. |
| buildroot complains about `LD_LIBRARY_PATH` / `PATH` | Use a fresh terminal without the Xilinx `settings64.sh` sourced. |
| U-Boot prints `syntax error` while booting | Harmless. The vendor U-Boot patch leaves a stray `fi;` at the end of `adi_loadvals`. U-Boot reports it, but all the commands before it still run (tested with U-Boot sandbox from the same source tree). |

---

## 9. Changes compared with the vendor package

The vendor patches are **unchanged**. This folder adds:

* `build.sh`: fetches sources at the pinned commits (the vendor zip has no
  `plutosdr-fw` submodule content), applies patches so they can be re-run,
  checks Vivado/Vitis 2023.2, and runs every build stage with the correct
  environment.
* `xilinx_env.sh`: the `VIVADO_SETTINGS` file. It sources Vivado **and**
  Vitis 2023.2, so `vivado`, `xsct` and `bootgen` are all available to the
  Makefile.
* Guards for three problems in the stock flow:
  * Without Vivado, the stock Makefile silently downloads ADI's *PlutoSDR*
    XSA, which is the wrong hardware for an E310.
  * Make command-line variables rename the HDL project folder (see above).
  * `make sdimg` fails on its second run because of a plain `mkdir`.
