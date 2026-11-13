# Building ANTSDR E310 firmware with Vivado/Vitis 2023.2

This folder holds the MicroPhase `antsdr-fw-patch` (v0.39) patch set plus a
build wrapper. Together they build the complete ANTSDR **E310** firmware on top
of ADI `plutosdr-fw` v0.39:

| Stage | Tool | Output |
|---|---|---|
| Anti-jam filter core `sfap_pi` (E310 only; crpa15: SFAP-PI 256, weights from the ARM) | Vitis HLS 2023.2 | `hdl/library/antsdr_ip/sfap_pi` IP |
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
| `patch/ant/0003-hdl-stap-antijam.patch` | `hdl` | v4.9.14 anti-jam: AD9361 **LVDS 2R2T** interface, `gnss_passthrough` v4.9.14 with the weight RAM, `sfap_pi` filter (crpa15: SFAP-PI 256, weights from the ARM) between two clock-crossing FIFOs (section 7) |
| `patch/ant/0003-linux-lvds-2r2t.patch` | `linux` | `zynq-ant.dtsi`: `adi,ad9361`, LVDS settings of the vendor E310 design |
| `patch/ant/0003-uboot-mode-2r2t.patch` | `u-boot-xlnx` | default `mode=2r2t` (RX1 and RX2 both on) |
| `patch/ant/0003-buildroot-gnss-crpa.patch` | `buildroot` | `gnss-crpa` package: the anti-jam firmware as a Linux service, the `gnss` console (section 7) |

`patch` also copies `gnss_crpa/` (the firmware sources) into
`plutosdr-fw/buildroot/package/gnss-crpa/src`.

### Step 3b: Anti-jam filter core (Vitis HLS 2023.2)

```bash
./build.sh hls
```

Builds `hls/sfap_pi` (C simulation of scenarios 1-18 with the firmware's solver,
synthesis at 100 MHz, II=1) into `plutosdr-fw/hdl/library/antsdr_ip/sfap_pi`.
It takes 20–40 minutes. It prints `SFAP_PI_HLS: PASS` at the end, and its log is
`hls/sfap_pi/build/build_hls.log`. `SFAP_PI_SKIP_CSIM=1 ./build.sh hls` skips
the simulation, `SFAP_PI_CSIM_LIST="1 2 4" ./build.sh hls` runs only those
scenarios. `hdl`, `fsbl` and `firmware` run this step themselves if the
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

## 7. GNSS anti-jam (crpa15: SFAP-PI 256, weights on the ARM): RX1 + RX2 -> FPGA -> TX1

The `0003` patches put the GNSS anti-jam into this Linux firmware ("design A"):
AD9361 LVDS 2R2T, `gnss_passthrough` v4.9.14, a filter core between RX1/RX2 and
TX1, and the `gnss_crpa` service with its console. One feature of the Linux port
is new: **band 7, your own RF values, set at any time from PuTTY**.

```text
antenna 1 -> RX1 \                    sfap_pi space-frequency anti-jam
                  AD9361 -> FPGA  gnss_passthrough  (256 bins, b + 2 image taps per bin) -> AD9361 -> TX1
antenna 2 -> RX2 /  LVDS 2R2T         + weight RAM + pulse blanker               -> coax + attenuator + DC block
                    61.44 MS/s                                                     -> GNSS receiver -> GNSS Viewer
```

* **FPGA (hdl 0003):** the AD9361 runs over **LVDS** (pins and I/O standards of
  the vendor E310 V1 design), so both receivers run at 61.44 MS/s.
  `gnss_passthrough` v4.9.14 (`0x43C1_0000`) feeds RX1+RX2 to the `sfap_pi`
  filter (Vitis HLS, 100 MHz) and its output to TX1. TX2 carries a muted copy.
* **Linux (0003):** the device tree switches the driver to `adi,ad9361` 2R2T
  LVDS. U-Boot defaults to `mode=2r2t`.
* **Firmware (buildroot 0003):** `gnss_crpa`, started at boot: self-test,
  band, anti-jam on, TX1 on.

### crpa15: your SFAP-PI 256 (v6.2.1) under Linux

Three Linux packages were asked for, all with the weights calculated on the ARM:
**crpa15** = SFAP-PI 256 (your v6.2.1), **crpa16** = STAP 8 + 8 with 3 + 3 image
taps, **crpa17** = STAP 6 + 6 with 3 + 3 image taps. This package is **crpa15**.

* **FPGA:** your `sfap_pi` 4.0 core (`hls/sfap_pi`, from SFAP_PI_256v3 / v6.0
  core, unchanged): RX2 normalisation g2, 256-bin sqrt-Hann filter bank with
  50 % overlap (hop 128), per bin one weight `b` on RX2 plus two image taps
  `c1`, `c2`, RX1 fixed at 1, exact bypass, make-up shift 0..4. Latency 766
  samples (12.5 us at 61.44 MS/s).
* **Weight RAM (new in `gnss_passthrough`):** 3 x 256 words, double-buffered:
  the ARM writes `b` at 0x1000 + 4k, `c1` at 0x1400 + 4k, `c2` at 0x1800 + 4k
  (bin k in FFT order, Q3.12) into the bank **not** in use; `AJ_COMMIT` (0x180)
  switches banks only if weights were written, so the filter never runs a
  half-old, half-new set. The core reads the bank in use through its memory
  ports (two per array). 0x1B0 reads the bank in use.
  **`AJ_INFO` = `0x08070C0F`** (log2 N 8, log2 hop 7, Q3.12, layout 15).
  AJ_W, AJ_WC, AJ_WCX and AJ_GAIN of the STAP packages are gone; clip counters,
  power meters and the pulse blanker are kept.
* **ARM:** your v6.2.1 solver (`gnss_crpa/src/gnss_aj_math.c`, `aj_compute`),
  unchanged, on every 4096-sample capture while jammed: per bin the 3 weights,
  drift-aware averaging, the weight hold (v6.1), the per-bin detector, g2
  following a jammer weaker at RX2 (v6.2). Then all 768 words are written and
  committed with the make-up shift. About 10 ms per solve on the board (about
  100 solves per second, as on your bare-metal build; one ARM core, Linux);
  `gnss_aj_sfap?` shows the measured time.
* **Detection:** the 4.9.14 detector (powers + RX1/RX2 coherence) **plus** your
  per-bin detector (a narrowband jammer 8 dB above the learned per-bin floor
  switches the filter in, 5 dB to release), so narrow and hopping jammers are
  seen too. The floor is learned while no jammer is present (as before: switch
  the jammer off for 10 s after a band change).

C simulation (your `hls/sfap_pi/testbench.cpp`, run with this firmware's
`gnss_aj_math.c`; all 18 scenarios pass):

| Scenario | suppression | residual |
|---|---|---|
| 2: wideband, J/N 30 dB | 41.0 dB | +0.21 dB |
| 3: no jammer (filter in) | – | +0.02 dB |
| 4: CW, J/N 40 dB | 63.1 dB | +0.02 dB |
| 5: wideband + reflection | 40.1 dB | +0.22 dB |
| 6: wideband 50 dB, I/Q imbalance | 56.8 dB | +0.23 dB |
| 7: RX2 25 dB weaker, J/N 40 dB | 38.6 dB | +3.65 dB |
| 8: wideband 60 dB, I/Q imbalance | 58.4 dB | +1.51 dB |
| 9: two CW from two directions | 63.0 dB | +0.03 dB |
| 11: intermittent jammer (hold 100) | 52.0 dB | |
| 12: hopping narrowband (hold 100) | 56.2 dB | |
| 13: per-bin detector, 300 noise captures | 0 false alarms | |
| 15: jammer changes direction | 46.0 dB | |

Compared with crpa16/17 (STAP 8 + 8 / 6 + 6, NLMS on the ARM): SFAP nulls
wideband and multipath jammers more deeply (each bin has its own weight, so a
reflection or an I/Q error across the band costs little: J/N 60 dB +1.5 dB
against +6 dB), at about 12.5 us latency instead of 0.1 us, and with more DSP48.
A two-antenna array still nulls **one** direction per bin.

Resources (estimate; the Vivado report is the proof):

| | crpa15 (SFAP 256) | crpa16 (STAP 8 + 8) | crpa17 (STAP 6 + 6) |
|---|---|---|---|
| DSP48 in the core | about 158 | about 67 | about 55 |
| DSP48 in design A | about 170-190 of 220 (77-86 %) | about 101 (46 %) | about 89 (40 %) |
| BRAM36 | about 20-30 (FFT buffers + weight RAM 6 x 2 KB) | about 10-15 | about 10-15 |
| ARM (CPU 0) while jammed | most of it (about 10 ms per solve) | most of it | about 3/4 of crpa16 |

`gnss_aj_mode=0|1|2` chooses the solver: 0 = per bin b + 2 image taps
(default), 1 = b only, 2 = one b for all bins (the narrowband v3 SMI-PI).
`gnss_aj_hold=N` sets the weight hold (solves, default 100, 0 = off).
`gnss_aj_sfap?` shows bins, hop, latency, mode, hold, per-bin floor, g2 and the
solve time. Mode and hold are not saved (defaults at every start).

To build in a folder that was already patched for another package, start from
clean sources; `./build.sh patch` stops with "does not apply" otherwise:

```bash
./build.sh reset && ./build.sh all
```

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

You now type the firmware's commands (`?` = menu, `help?` = every command). `Ctrl-D` or `exit` returns to the
shell, and the anti-jam keeps running. You can also give one command
directly from the shell, e.g. `gnss gnss_band?`.

| Command | Does |
|---|---|
| `gnss_band?` | Current band, which GNSS signals it carries, the 8 profiles |
| `gnss_band=0` … `gnss_band=6` | The fixed profiles (0 = default, 1569.42 MHz) |
| **`gnss_band=7,1575.42,61.44,56,30,50`** | **Band 7 = your values**: LO MHz, MS/s, RF bandwidth MHz, TX1 attenuation dB, RX gain dB. Applied to RX and TX at once and saved. |
| `gnss_band=7` | Back to band 7 with the stored values |
| `gnss_band7=1575.42,30.72,27` | Change band 7 (leave out the tail to keep it). Applied at once if band 7 is in use. |
| `gnss_band7?` | Band 7 values, whether it is in use, where they are saved |
| `gnss_band_boot=7` / `gnss_band_boot?` | Band used at power-on (saved; 0 = default) |
| `gnss_aj?` / `gnss_report?` | Anti-jam state / everything in one block |
| `gnss_smi=0` / `gnss_smi=1` | Anti-jam off (TX1 = RX1) / on |
| `gnss_tx=0` / `gnss_tx=1` | Silence TX1 / switch it back on |
| `tx1_attenuation=40000` | TX1 attenuation in mdB (bigger = quieter) |
| `gnss_rx_gain=45` | RX1 and RX2 gain |
| `gnss_rx_agc=2` | RX1 and RX2 gain together, automatic against clipping (saved) |
| `gnss_aj_sfap?` | SFAP-PI: bins, hop, latency, mode, weight hold, per-bin floor, g2, ARM solve time |
| `gnss_aj_mode=0` / `=1` / `=2` | Per bin b + 2 image taps (default) / b only / one b for all bins (not saved) |
| `gnss_aj_hold=100` | Weight hold in solves, 0..1000 (default 100; 0 = off; not saved) |
| `gnss_aj_csv=500` | CSV line every 500 ms (columns: J/N, suppression, residual, clip_ppm, FPGA meters ...) |

Band 7 limits: LO 70–6000 MHz, 2.084–61.44 MS/s, bandwidth 0.2–56 MHz,
TX1 attenuation 0–89.75 dB, RX gain 0–76 dB. A value outside them is
refused and nothing changes. Values are saved in `/mnt/jffs2/gnss_crpa.conf`
(or on the SD card if the flash partition is missing), so they survive a
reboot.

Jammer events appear as `[AJ …] *** JAMMER ON ***` / `NULLED` lines. They are
held back for 20 s after you type. The full
service log is `/var/log/gnss_crpa.log`. Snapshots (every 60 s) go to the SD
card under `LOG/`.

The service:

| Command (shell) | Does |
|---|---|
| `/etc/init.d/S95gnss_crpa status` | Running or not |
| `/etc/init.d/S95gnss_crpa stop` | Stops it; TX1 goes to 89.75 dB and is muted |
| `/etc/init.d/S95gnss_crpa restart` | Restart (runs the self-test again) |
| `touch /mnt/jffs2/gnss_crpa.disable` | Do not start at boot (RF-silent board) |
| `gnss_crpa` | With the service stopped: run it in this terminal |

`gnss_loopback start|stop|status` still works and forwards to `gnss`.

### If it doesn't work

| Message or symptom | Meaning / fix |
|---|---|
| `gnss: the gnss_crpa service is not running` | `/etc/init.d/S95gnss_crpa start`, then `tail /var/log/gnss_crpa.log` |
| `hardware not available` | The board runs an older bitstream. Copy the new `BOOT.bin` to the SD card. |
| `the bitstream's AJ_INFO is 0x8040c0e, this firmware (crpa15, SFAP-PI 256 bins) needs 0x8070c0f` | `BOOT.bin` of another package (STAP) with this firmware: copy the whole new SD image. |
| `gnss_pt: … version` mismatch in the self-test | Same: old `BOOT.bin`. |
| `no RX capture` | Another program (IIO Oscilloscope, `iio_readdev`) holds the RX buffer. Close it and restart the service. |
| `dmesg | grep ad9361` shows a tuning failure | The LVDS interface did not train. Report the `dmesg` output. |
| Only one RX channel in `iio_info` | U-Boot kept an old `mode=1r1t`. At the `ant>` prompt: `setenv mode 2r2t; saveenv; boot` |
| Viewer shows no satellites | `gnss gnss_report?`, check RSSI and gain, try `gnss_band=1`, check the DC block and attenuator in the TX1 path. |

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
* `patch/ant/0003-*.patch`, `hls/sfap_pi/`, `gnss_crpa/`: the v4.9.14
  anti-jam (section 7): LVDS 2R2T, `gnss_passthrough` v4.9.14 + weight RAM +
  `sfap_pi` (crpa15: your SFAP-PI 256, weights by your v6.2.1 solver on the
  ARM), and the firmware as the
  `gnss_crpa` Linux service with custom band 7.

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
