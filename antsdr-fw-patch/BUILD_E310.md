# Building ANTSDR E310 firmware with Vivado/Vitis 2023.2

This folder holds the MicroPhase `antsdr-fw-patch` (v0.39) patch set plus a
build wrapper. Together they build the complete ANTSDR **E310** firmware on top
of ADI `plutosdr-fw` v0.39:

| Stage | Tool | Output |
|---|---|---|
| SMI-PI anti-jam filter core `smi_pi` (E310 only) | Vitis HLS 2023.2 | `hdl/library/antsdr_ip/smi_pi` IP |
| FPGA hardware design (AD9361 LVDS 2R2T + ANTSDR VCXO/PPS logic + GNSS anti-jam, xc7z020clg400-2) | Vivado 2023.2 | `system_top.xsa`, `system_top.bit` |
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
    installs Vivado as well. You need Vitis for `xsct`, which builds the FSBL,
    and for `vitis_hls`, which builds the anti-jam filter core.
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

If you received the zip as two parts (`.zip.part1`, `.zip.part2`), join them
first and check the result against `SHA256SUMS.txt`:

```bash
cat antsdr-e310-fw-v0.39-full-src.zip.part1 antsdr-e310-fw-v0.39-full-src.zip.part2 > antsdr-e310-fw-v0.39-full-src.zip
```

```bash
sha256sum -c --ignore-missing SHA256SUMS.txt
```

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

That runs `check`, `fetch`, `patch`, `hls`, `firmware` and `sdimg`, described below.
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
`xsct`, `bootgen` and `vitis_hls` are available, and the free disk space.

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
| `patch/ant/0002-hdl-add-gnss-passthrough.patch` | `hdl` | `library/gnss_passthrough` and its place in the `ant` block design (GPS L1 loopback, section 7) |
| `patch/ant/0002-buildroot-add-gnss-loopback.patch` | `buildroot` | `/usr/sbin/gnss_loopback` command |
| `patch/ant/0003-hdl-stap-antijam.patch` | `hdl` | SMI-PI v4.9.14 anti-jam: AD9361 **LVDS 2R2T** interface, `gnss_passthrough` v4.9.14, `smi_pi` filter between two clock-crossing FIFOs (section 7) |
| `patch/ant/0003-linux-lvds-2r2t.patch` | `linux` | `zynq-ant.dtsi`: `adi,ad9361`, LVDS settings of the vendor E310 design |
| `patch/ant/0003-uboot-mode-2r2t.patch` | `u-boot-xlnx` | default `mode=2r2t` (RX1 and RX2 both on) |
| `patch/ant/0003-buildroot-gnss-crpa.patch` | `buildroot` | `gnss-crpa` package: the anti-jam firmware as a Linux service, the `gnss` console (section 7) |

`patch` also copies `gnss_crpa/` (the firmware sources) into
`plutosdr-fw/buildroot/package/gnss-crpa/src`.

### Step 3b: Anti-jam filter core (Vitis HLS 2023.2)

```bash
./build.sh hls
```

Builds `hls/smi_pi` (C simulation of scenarios 1-14 and 23-27, synthesis at
100 MHz, II=1) into `plutosdr-fw/hdl/library/antsdr_ip/smi_pi`. It takes
10–20 minutes. It prints `SMI_PI_HLS: PASS` at the end, and its log is
`hls/smi_pi/build/build_hls.log`. `SMI_PI_SKIP_CSIM=1 ./build.sh hls` skips
the simulation. `hdl`, `fsbl` and `firmware` run this step themselves if the
core is missing.

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

## 7. GNSS anti-jam (SMI-PI v4.9.14): RX1 + RX2 -> FPGA -> TX1

The `0003` patches put the complete `LoopBack_Code_SMI_PI_STAP7x7_v4.9.14`
design ("B", here with 10 + 10 taps) into this Linux firmware ("A"): the same FPGA datapath and the
same firmware with all its console commands. One feature is new: **band 7,
your own RF values, set at any time from PuTTY**.

```text
antenna 1 -> RX1 \                    smi_pi space-time anti-jam
                  AD9361 -> FPGA  gnss_passthrough (10 + 10 taps)-> AD9361 -> TX1
antenna 2 -> RX2 /  LVDS 2R2T         + pulse blanker               -> coax + attenuator + DC block
                    61.44 MS/s                                         -> GNSS receiver -> GNSS Viewer
```

* **FPGA (hdl 0003):** the AD9361 now runs over **LVDS** (pins and I/O
  standards of the vendor E310 V1 design), so both receivers run at
  61.44 MS/s. The CMOS interface of the stock design stops at 30.72 MS/s with
  two channels. `gnss_passthrough` v4.9.14 (`0x43C1_0000`) feeds RX1+RX2 to
  the `smi_pi` filter (Vitis HLS, 100 MHz) and its output to TX1. TX2 carries
  a muted copy. The DMA paths stay, so IIO still works for the DDR replay.
* **Linux (0003):** the device tree switches the driver to `adi,ad9361`
  2R2T LVDS. U-Boot defaults to `mode=2r2t`.
* **Firmware (buildroot 0003):** `gnss_crpa` is the SMI-PI firmware
  (`app_gnss_e310`, v4.9.14) ported to Linux. The AD9361 calls go to the
  Linux driver, and the anti-jam captures come from the Linux IIO buffer
  instead of the DMA registers. It starts at boot like the bare-metal
  firmware did: self-test, band, anti-jam on, TX1 on.

### Filter size: 8 + 8 taps (crpa10); 10 + 10 in crpa7 to crpa9

**crpa10 is back to 8 taps per antenna (8 + 8), reference delay 4**, to leave room
for the hardware NLMS (below). crpa7 to crpa9 have **10 taps per antenna (10 + 10)**, reference tap
at delay 5 (5 taps before it, 4 after). It is the space-time core of
`LoopBack_Code_SMI_PI_STAP7x7_v4.9.14` with three more taps per antenna. The
FPGA core and the firmware must agree:

| Where | 10 + 10 (crpa7 to crpa9) | 8 + 8 (crpa4, crpa5; crpa10 with NLMS) | 7 + 7 (crpa3) | 6 + 6 (crpa1, crpa2) |
|---|---|---|---|---|
| `hls/smi_pi/smi_pi.h` `SMI_NT` / `SMI_REF` | 10 / 5 | 8 / 4 | 7 / 4 | 6 / 3 |
| `gnss_passthrough.v` `AJ_INFO` (hdl 0003) | `0x0A050C0B` (crpa9; crpa8 `0x0A050C0A`, crpa7 `0x0A050C07`) | `0x08040C07` (crpa10: `0x08040C0C`) | `0x07040C07` | `0x06030C07` |
| `gnss_aj_math.h` `AJ_NT` / `AJ_REF` (Linux and bare metal) | 10 / 5 | 8 / 4 | 7 / 4 | 6 / 3 |
| adaptive weights | 19 + 6 image (crpa9; 19 + 2 before) | 15 + 2 image (crpa10: 15 + 6) | 13 + 2 image | 11 + 2 image |
| weight words at 0x400 | 0..9 RX1, 10..19 RX2 | 0..7, 8..15 | 0..6, 7..13 | 0..5, 6..11 |
| DSP48 in `smi_pi` | 79 (crpa9: + 12 image taps; crpa8 67, crpa7 65) | 53 (crpa10: 132 with NLMS) | 47 | 41 |

The image taps and the other registers are unchanged; since crpa8 the weight
file holds the 20 words in use (0x400-0x44C; 0x450-0x4FC read 0). 67 of the
220 DSP48 of the XC7Z020 go to the filter in crpa9 (about 110 with the rest of design A).

C simulation of the core with the firmware maths (`hls/smi_pi/testbench.cpp`,
suppression in dB; all 14 scenarios pass):

| Scenario | 6 + 6 | 7 + 7 | 8 + 8 | **10 + 10** |
|---|---|---|---|---|
| 2: wideband jammer, J/N 30 dB | 36.7 | 37.2 | 39.5 | **41.5** |
| 4: CW jammer, J/N 40 dB | | | 66.8 | **68.0** |
| 5: jammer with a reflection on RX2 | 20.4 | 20.7 | 37.4 | **37.4** |
| 6: wideband jammer 50 dB, I/Q imbalance | | | 52.4 | **54.3** |
| 8: wideband jammer 60 dB, I/Q imbalance | 51.0 | 51.7 | 53.6 | **55.2** |
| 11: field case, J/N 61 dB | 53.4 | 54.1 | 56.2 | **57.2** |
| 12: field case, J/N 51 dB | | | 55.0 | **56.4** |
| 13: field case, J/N 41 dB | 45.7 | 46.5 | 48.9 | **51.8** |

Reference delay for 10 + 10: delays 4, 5 and 6 were simulated. Delay 4 was up
to 1.2 dB better in three wideband scenarios but lost 2.5 dB on the
reflection (34.9 against 37.4 dB); delay 6 was lower almost everywhere.
Delay 5 is the only one that is never below 8 + 8, so 10 + 10 uses delay 5.

The firmware reads `AJ_INFO` at start-up and does not start the anti-jam if
the bitstream has a different tap count or reference delay; the console then
says so ("Anti-jam NOT started").

To build in a folder that was already patched for another size or an
older package (crpa1 to crpa9), start from clean sources; `./build.sh patch`
stops with "does not apply" otherwise:

```bash
./build.sh reset && ./build.sh all
```

### Smooth weights, fine gain, clip counters, second core (crpa8)

The field tests showed two things the 10 + 10 filter alone does not fix: the
receiver lost satellites it had already locked (every new weight set changed
the filter in one clock, thousands of times; the output level moved in 6 dB
steps), and at the jammer's 10 / 15 dB settings the ADC clipped. crpa8 adds:

| Part | What | Where |
|---|---|---|
| **Weight smoothing** (FPGA, `smi_pi` 2.7) | A new weight set glides in: every weight moves 1/256 of the remaining way every 2^(k-1) samples, time constant **2^(k+7) samples**. No DSP48, no multiplier. Bypass fades the filter out the same way. k = 0 = the crpa7 behaviour, bit for bit. | `AJ_CFG[19:16]` |
| **Fine output gain** (FPGA, 2 DSP48) | After the 6 dB make-up shift, Q2.14, smoothed like the weights: the level into the GNSS receiver follows the RX gain cut to a fraction of a dB instead of 6 dB steps. | `AJ_GAIN` 0x1B0 (shadow), 0x1B4 (committed) |
| **ADC clip counters** (FPGA) | Every RX1 / RX2 sample with I or Q at 2047 / -2048 is counted (wraps; cleared with CONTROL[8]). | `AJ_CLIP1` 0x1B8, `AJ_CLIP2` 0x1BC |
| Smoothing control (ARM) | `gnss_aj_ramp=` (default 10). The first weights after the bypass go in with k = 1 (256 samples), so a new jammer is still nulled at once; every later set glides with k. | firmware |
| Anti-clipping (ARM) | `gnss_rx_agc=2`: more than 100 ppm of the samples clipped in a 5 ms poll = the largest gain step at once (the peak hold could miss it for up to 8.5 ms). Manual gain: the `[RXG] ADC CLIPPING` warning now gives the exact clip rate. `gnss_aj?`, `gnss_report?` and the CSV (`clip_ppm`) show it. | firmware |
| Second ARM core (Linux) | The solver runs in a worker thread on the second Cortex-A9; the console, the capture and the gain loop stay responsive. `gnss_report?` shows the solve time. Bare metal stays single-core. | firmware (`-DGNSS_AJ_THREAD`) |
| Smaller weight file | 20 words instead of 64 (about 2,700 flip-flops fewer). | `gnss_passthrough.v` |

`AJ_INFO` is `0x0A050C0A` (layout 10); the crpa8 firmware starts the anti-jam
only on a layout-10 bitstream ("Anti-jam NOT started" otherwise: rebuild the
whole project).

Which smoothing to use (`gnss_aj_ramp=`, saved with the band 7 values):

| k | Time constant | at 30.72 MS/s | at 61.44 MS/s | Use |
|---|---|---|---|---|
| 0 | none | | | crpa7 behaviour (for comparison) |
| 8 | 2^15 samples | 1.1 ms | 0.5 ms | fast-changing jammer |
| **10** (default) | 2^17 samples | 4.3 ms | 2.1 ms | weight updates every 5 ms glide into each other |
| 12 | 2^19 samples | 17 ms | 8.5 ms | fixed wideband jammer, receiver keeps losing lock |
| 14 | 2^21 samples | 68 ms | 34 ms | very slow; a moving jammer leaks longer |

C simulation (`./build.sh hls` runs all of them): scenarios 1-14 give exactly
the crpa7 numbers with smoothing off and gain 1.0; scenario 23 checks the
smoothing and the fine gain bit for bit against an integer model (60000
samples, random weights, scales, shifts, bypass, k and gain: 0 mismatches);
scenario 24 checks the behaviour: with k = 6 (8192 samples) a null fades in to
-8.8 dB after one time constant (e^-2 = -8.7 dB expected) and ends exactly on
the instant null, a gain step 1.0 -> 2.0 starts at 1.006 and ends at 2.0, a
weight-scale change causes no transient.

Also studied and **not** included: an eigenvalue ("eigencanceller") solver
instead of power inversion. In the C simulation it was better in 5 scenarios
(e.g. the reflection case 48.2 instead of 37.4 dB) but worse with a 60 dB
jammer and I/Q imbalance (scenario 8: 1.7 dB less suppression, 1.6 dB more
satellite loss) and sensitive to its threshold, so power inversion stays.

### Hardware NLMS (crpa10)

crpa10 = crpa9 (image taps, power meters, smoothing, fine gain, clip counters,
second core) with **8 + 8 taps** and the **hardware NLMS** of the RTL review
(item 7).

The ARM solves the SMI weights from a capture a few hundred times a second.
Between two solves a moving jammer (or a moving platform) changes its
direction and the weights are out of date. The NLMS loop in the FPGA corrects
them on **every sample**:

```text
w <- w + step x e[n-D] x conj(u[n-D])      e = filter output, u = the tap's input, D = 24
     + smoothing towards the ARM's SMI weights (anchors NLMS to the SMI solution)
```

* Delayed LMS (D = 24 samples) so the loop runs at one sample per clock
  (100 MHz), 3 DSP48 per weight: **+65 DSP48, 132 in `smi_pi`**, about 166 of
  the 220 in the whole design (estimate; check `./build.sh hls`).
* The step is normalised by the RX1 power the FPGA meter measures; the ARM
  writes it with every weight commit (`AJ_NLMS` 0x1E0: mantissa, shift,
  enable; 0x1E4 = committed).
* **Off by default.** `gnss_aj_nlms=1` switches it on (saved),
  `gnss_aj_nlms_mu=` sets the step (default 0.031). NLMS needs the smoothing:
  with `gnss_aj_ramp=0` it uses 10. `gnss_smi_freeze=1` stops it as well.
* `AJ_INFO` `0x08040C0C` (layout 12); the crpa10 firmware needs it.

C simulation (scenarios 26 / 27, J/N 50 dB, residual above the noise, NLMS off / on):

| Jammer | NLMS off | NLMS on (step 0.031) |
|---|---|---|
| direction drifting 10 deg after the SMI solve | +24.9 dB | **+1.7 dB** |
| direction drifting 60 deg (extreme) | +40.3 dB | +9.2 dB (step 0.12: +3.8 dB) |
| fixed (no drift) | +0.99 dB | +1.08 dB |

Larger steps follow faster but cost a little on a fixed jammer (0.06: +0.2 dB,
0.12: +0.5 dB). The NLMS core is checked bit for bit against an integer model
(scenario 23, 60000 samples, NLMS on in half of them: 0 mismatches). All 19
scenarios pass (1-14, 23-27).

**crpa11** fixes the crpa10 HLS timing (crpa10: `estimated clock period 63.552 ns`,
`FAIL`): the NLMS error history was a shift register written at the end of the
pipeline and read at the start of the next sample, which forced the whole filter
into one clock. crpa11 keeps it in a 32-slot circular buffer (LUTRAM) and tells
HLS the 24-sample distance (`DEPENDENCE ... distance=24`), and saturates the NLMS
step before the weight update. The C simulation is bit for bit the same.

**If the build still does not meet timing**: build the core without NLMS, the
rest of crpa11 stays:

```bash
SMI_NLMS=0 ./build.sh hls
```

```bash
./build.sh hdl && ./build.sh firmware && ./build.sh sdimg
```

### 3 + 3 image taps, power meters (crpa9)

crpa9 adds two of the remaining items of the RTL review on top of crpa8; two
others were simulated first and left out:

| Item | Result | In crpa9 |
|---|---|---|
| **3 image taps per channel** (delays R-1, R, R+1 instead of R only) | The AD9361's I/Q mirror is not the same across a wide band (the I and Q analog filters differ). With such an error (scenario 25, J/N 61 dB) one image tap leaves +8.6 dB residual and 11.2 dB satellite loss, three taps +3.2 dB and 2.9 dB; at J/N 51 dB +3.4 -> +0.4 dB. With a flat I/Q error nothing changes. | **Yes** (+12 DSP48, 79 in all) |
| **FPGA power meters** on RX1 and on the filter output | Mean I^2+Q^2 over every 2^16 samples: the residual and J/N on every sample, not only in the 8192-sample captures | **Yes** |
| 18-bit weights | The same residual as 16-bit and even as unrounded weights in every case: the weight scale already makes rounding harmless | No |
| Hardware NLMS | Not built (high risk, about 63 DSP48; little gain for a fixed wideband jammer) | No |

C simulation of crpa9 against crpa8 (suppression / residual / satellite loss, dB):

| Scenario | crpa8 (1 + 1 image taps) | crpa9 (3 + 3) |
|---|---|---|
| 25: J/N 61 dB, I/Q error across the band | 48.4 / +8.58 / 11.18 | **57.0 / +3.19 / 2.87** |
| 25: J/N 51 dB, the same | 46.4 / +3.35 / 2.97 | **56.0 / +0.38 / 0.16** |
| 8: J/N 60 dB, flat I/Q error | 55.2 / +3.17 / 1.82 | 55.6 / +2.91 / 1.39 |
| 11: field case J/N 61 dB | 57.2 / +3.10 / 3.91 | 57.4 / +2.90 / 4.47 |
| 2, 4, 5, 6, 12, 13 | | within 0.8 dB of crpa8 |

New registers (AJ_INFO `0x0A050C0B`, layout 11; the crpa9 firmware needs it):

| Register | Address | |
|---|---|---|
| `AJ_WCX0..3` | 0x1C0-0x1CC | image taps c1 at R-1, c1 at R+1, c2 at R-1, c2 at R+1 (c1 / c2 at R stay at 0x190 / 0x194); committed with `AJ_COMMIT` |
| `AJ_PWR_IN` | 0x1D0 | mean RX1 I^2+Q^2 over the last 2^16 samples [LSB^2] |
| `AJ_PWR_OUT` | 0x1D4 | the same for the filter output to TX1 |
| `AJ_PWR_IDX` | 0x1D8 | window counters |

`gnss_report?` shows a `fpga meters` line (residual and J/N from the meters)
and the CSV log has two more columns, `res_fpga_db` and `jn_fpga_db`.

### No CW notch, no SFAP-PI in this build

* The automatic CW notch of `crpa5` (`gnss_notch?`) is **not** in crpa7 / crpa8:
  the FPGA core is the space-time filter without notch (`smi_pi` 2.5 / 2.7).
  With a wideband jammer this changes nothing (the notch stayed off there).
* The SFAP-PI frequency-domain filter of `crpa6` is not in crpa7 / crpa8 either.
* Both are in the git history (crpa5: commit eaa2d1a, crpa6: commit 3b9e63e).

Kept from crpa5 (firmware):

* `gnss_rx_agc=` and `gnss_aj_avg=` are **saved** with the band 7 values and set
  again at start-up.
* With **manual** RX gain the firmware now watches the ADC peak and prints
  `[RXG] ADC CLIPPING ...` when the jammer clips it (a clipped jammer cannot be
  nulled); `gnss_rx_agc=2` or a lower gain cures it.
* `gnss_band=` while a jammer is on warns that the noise floor is learned again
  (switch the jammer off for about 10 s, or it may be taken for background).
* The level lock no longer puts TX1 back while `gnss_rx_agc=2` holds the output
  level.
* Console: `%lu` values (e.g. `LEVEL LOCK ... repair #u`) print correctly.

### Build it

```bash
./build.sh patch
```

```bash
./build.sh hls
```

```bash
./build.sh firmware
```

```bash
./build.sh sdimg
```

or `./build.sh all`. Copy everything in `plutosdr-fw/build_sdimg/` to the
FAT32 SD card. **`BOOT.bin` on the card is what changes the FPGA.**

### Connect

| Port | Connect to |
|---|---|
| RX1 | GNSS antenna 1 (with a bias-tee if it needs power) |
| RX2 | GNSS antenna 2, about half a wavelength (≈ 9.5 cm) from antenna 1 |
| TX1 | Attenuator, then a DC block, then the GNSS receiver's antenna input |
| TX2 | Nothing (muted), or a 50 Ω load |
| Receiver USB | Laptop running GNSS Viewer |
| USB-UART | PuTTY, 115200 8N1 |

**TX1 must reach the receiver through coax and an attenuator, never an
antenna.** Transmitting on GNSS frequencies over the air is illegal in most
countries. Do not test with a real jammer over the air for the same reason.

### Run it (PuTTY)

Power on. At the `login:` prompt use `root` / `analog`. The anti-jam is
already running. Attach to it:

```bash
gnss
```

You now type the firmware's commands exactly as on the bare-metal console
(`?` = menu, `help?` = every command). `Ctrl-D` or `exit` returns to the
shell, and the anti-jam keeps running. You can also give one command
directly from the shell, e.g. `gnss gnss_band?`.

| Command | Does |
|---|---|
| `gnss_band?` | Current band, which GNSS signals it carries, the 8 profiles |
| `gnss_band=0` … `gnss_band=6` | The SMI-PI fixed profiles (0 = default, 1569.42 MHz) |
| **`gnss_band=7,1575.42,61.44,56,30,50`** | **Band 7 = your values**: LO MHz, MS/s, RF bandwidth MHz, TX1 attenuation dB, RX gain dB. Applied to RX and TX at once and saved. |
| `gnss_band=7` | Back to band 7 with the stored values |
| `gnss_band7=1575.42,30.72,27` | Change band 7 (leave out the tail to keep it). Applied at once if band 7 is in use. |
| `gnss_band7?` | Band 7 values, whether it is in use, where they are saved |
| `gnss_band_boot=7` / `gnss_band_boot?` | Band used at power-on (saved; 0 = SMI-PI default) |
| `gnss_aj?` / `gnss_report?` | Anti-jam state / everything in one block |
| `gnss_smi=0` / `gnss_smi=1` | Anti-jam off (TX1 = RX1) / on |
| `gnss_tx=0` / `gnss_tx=1` | Silence TX1 / switch it back on |
| `tx1_attenuation=40000` | TX1 attenuation in mdB (bigger = quieter) |
| `gnss_rx_gain=45` | RX1 and RX2 gain |
| `gnss_rx_agc=2` | RX1 and RX2 gain together, automatic against clipping (saved) |
| `gnss_aj_avg=0.98` | Weight averaging (0 = automatic; saved) |
| `gnss_aj_ramp=10` / `gnss_aj_ramp?` | crpa8: smoothing of new weights, 0 = off, 1..15 (saved) |
| `gnss_aj_nlms=1` / `gnss_aj_nlms_mu=0.031` / `gnss_aj_nlms?` | crpa10: hardware NLMS on / off (default off), step (saved) |

Band 7 limits: LO 70–6000 MHz, 2.084–61.44 MS/s, bandwidth 0.2–56 MHz,
TX1 attenuation 0–89.75 dB, RX gain 0–76 dB. A value outside them is
refused and nothing changes. Values are saved in `/mnt/jffs2/gnss_crpa.conf`
(or on the SD card if the flash partition is missing), so they survive a
reboot.

Jammer events appear as `[AJ …] *** JAMMER ON ***` / `NULLED` lines. Like the
bare-metal firmware, they are held back for 20 s after you type. The full
service log is `/var/log/gnss_crpa.log`. Snapshots (every 60 s) go to the SD
card under `LOG/`.

The service:

| Command (shell) | Does |
|---|---|
| `/etc/init.d/S95gnss_crpa status` | Running or not |
| `/etc/init.d/S95gnss_crpa stop` | Stops it; TX1 goes to 89.75 dB and is muted |
| `/etc/init.d/S95gnss_crpa restart` | Restart (runs the self-test again) |
| `touch /mnt/jffs2/gnss_crpa.disable` | Do not start at boot (RF-silent board) |
| `gnss_crpa` | With the service stopped: run it in this terminal, like the bare-metal console |

`gnss_loopback start|stop|status` still works and forwards to `gnss`.

### If it doesn't work

| Message or symptom | Meaning / fix |
|---|---|
| `gnss: the gnss_crpa service is not running` | `/etc/init.d/S95gnss_crpa start`, then `tail /var/log/gnss_crpa.log` |
| `hardware not available` | The board runs an older bitstream. Copy the new `BOOT.bin` to the SD card. |
| `gnss_pt: … version` mismatch in the self-test | Same: old `BOOT.bin`. |
| `no RX capture` | Another program (IIO Oscilloscope, `iio_readdev`) holds the RX buffer. Close it and restart the service. |
| `dmesg | grep ad9361` shows a tuning failure | The LVDS interface did not train. Report the `dmesg` output. |
| Only one RX channel in `iio_info` | U-Boot kept an old `mode=1r1t`. At the `ant>` prompt: `setenv mode 2r2t; saveenv; boot` |
| Viewer shows no satellites | `gnss gnss_report?`, check RSSI and gain, try `gnss_band=1`, check the DC block and attenuator in the TX1 path. |

---

## 7b. Design A on bare metal (no Linux)

`./build.sh baremetal` builds the same design-A FPGA (LVDS 2R2T,
`gnss_passthrough` v4.9.14, `smi_pi` 8 + 8 taps, 3 + 3 image taps, NLMS, smoothing, `axi_vcxo_ctrl`) but runs the
SMI-PI v4.9.14 firmware directly on the ARM, like design B. There is no
U-Boot, Linux or root file system: the SD card holds only `BOOT.bin`.

The firmware is B's `app_gnss_e310` (in `baremetal/app_gnss_e310`) adapted to
the design-A FPGA:

| Item | Design B FPGA | Design A FPGA (this build) |
|---|---|---|
| `gnss_passthrough` | `0x43C0_0000` | `0x43C1_0000` (set in `app_config.h`, never guessed) |
| AD9361 RESETB | GPIO 100 (EMIO 46) | GPIO 67 (EMIO 13) |
| RF band switches | GPIO 54–61 (EMIO 0–7) | GPIO 86–93 (EMIO 32–39) |
| 40 MHz VCXO | no control core | `axi_vcxo_ctrl` set to 23000 at power-on, as Linux does |

Added to the firmware: band 7 (`gnss_band=7,LO,rate,BW,att,gain`,
`gnss_band7?`, `gnss_band_boot=`) saved on the SD card in `GNSSCRPA.CFG`, and
`vcxo?`, `vcxo_dac=`, `vcxo_ref=` for the reference oscillator.

### Build it

```bash
./build.sh patch
```

```bash
./build.sh hls
```

```bash
./build.sh baremetal
```

`baremetal` builds the FPGA first if `plutosdr-fw/build/system_top.xsa` does
not exist yet (about 30–60 min), then the firmware and the FSBL with Vitis
`xsct`, then `build_baremetal/BOOT.bin` with `bootgen`. Linux and buildroot are
not built. `./build.sh all-baremetal` runs check, fetch, patch, hls and
baremetal in one go. Logs: `build_baremetal/build_software.log`,
`build_baremetal/build_fsbl.log`.

### SD card and console

Copy **only** `build_baremetal/BOOT.bin` to the FAT32 SD card (remove
`uImage`, `devicetree.dtb`, `uramdisk.image.gz` and `uEnv.txt` if they are
there). Open PuTTY at 115200 and power on: the firmware console appears
directly, with the same commands as design B (`?`, `help?`, `gnss_aj?`,
`gnss_report?`, `gnss_band=7,1575.42,61.44,56,30,50`, …). At power-on TX1 is
on and TX2 is muted.

| Message or symptom | Meaning / fix |
|---|---|
| Nothing in PuTTY | Wrong baud rate (115200), or the board did not boot from SD (boot switch, FAT32 card, `BOOT.bin` at the top level). |
| `ad9361_init failed` | The AD9361 did not start: check `vcxo?` and the 40 MHz reference. |
| `gnss_band7: values set; not saved` | No SD card mounted: values stay until power-off. |
| `gnss_pt: … not found` | The board runs another bitstream: copy the new `BOOT.bin`. |

---

## 8. Rebuilding and cleaning

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

## 9. Troubleshooting

| Symptom | Fix |
|---|---|
| `vivado not found after sourcing xilinx_env.sh` | `export XILINX_DIR=<folder containing Vivado/2023.2>` |
| `xsct not found` | Install **Vitis** 2023.2. Vivado alone does not provide `xsct`. |
| `ERROR: vivado version mismatch; expected 2023.2` | ADI hdl requires exactly 2023.2. `ADI_IGNORE_VERSION_CHECK=1` downgrades this to a warning, but other versions are not supported. |
| `IP definition not found for VLNV: user.org:user:axi_vcxo_ctrl:1.0`, and `ant_vivado.log` shows "Loaded user IP repository" pointing at another project | `ADI_HDL_DIR` was set by another project (e.g. in `~/.bashrc`). `build.sh` now ignores it. Rebuild the HDL libraries: `./build.sh make -C hdl/projects/ant clean-all`, then `./build.sh firmware`. |
| `ERROR: Timing Constraints NOT met!` | See `hdl/projects/ant/timing_impl.log`. The XSA is written as `ant.sdk/system_top_bad_timing.xsa`. |
| `Building <lib> library ... FAILED` | See `hdl/library/<lib>/<lib>_ip.log` |
| `libtinfo.so.5: cannot open shared object file` | `sudo apt-get install libtinfo5 libncurses5`. These exist on Ubuntu 20.04/22.04 but not on 24.04, which is one reason to use 22.04. |
| Linaro toolchain download fails | Buildroot also tries `sources.buildroot.net`. You can also put `gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf.tar.xz` into `plutosdr-fw/buildroot/dl/toolchain-external-linaro-arm/`. |
| buildroot complains about `LD_LIBRARY_PATH` / `PATH` | Use a fresh terminal without the Xilinx `settings64.sh` sourced. |
| U-Boot prints `syntax error` while booting | Harmless. The vendor U-Boot patch leaves a stray `fi;` at the end of `adi_loadvals`. U-Boot reports it, but all the commands before it still run (tested with U-Boot sandbox from the same source tree). |

---

## 10. Changes compared with the vendor package

The vendor patches are **unchanged**. This folder adds:

* `patch/ant/0002-*.patch`: the GPS L1 RX1 -> FPGA -> TX1 loopback.
* `patch/ant/0003-*.patch`, `hls/smi_pi/`, `gnss_crpa/`: the SMI-PI v4.9.14
  anti-jam (section 7): LVDS 2R2T, `gnss_passthrough` v4.9.14 + `smi_pi`,
  and the firmware as the `gnss_crpa` Linux service with custom band 7.

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
