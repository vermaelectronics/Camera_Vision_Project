# SFAP_snd2 (SFAP-PI v5.0, ANTSDR E310 V1): build to BOOT.BIN, step by step

Package: `SFAP_snd2.zip` → `SFAP_snd2/{Source,Automation,README_SFAP.md}`
Tools: **Vivado + Vitis + Vitis HLS 2023.2** (exactly 2023.2) on Linux (Ubuntu 20.04 / 22.04 or RHEL 8/9).
Flow: `check → hls → ip → hdl → sw → boot`, all driven by `Automation/Linux/build_all.sh`.

The examples use the default location from `env.sh`
(`/home/manoranjan/CRPA_Project/SFAP/SFAP_snd2`). If you unzip somewhere else,
change `SFAP_ROOT` in step 2. Nothing else needs editing.

---

## 0. Host prerequisites (one time)

```bash
sudo apt update
```

```bash
sudo apt install -y git make rsync unzip g++ libtinfo5 libncurses5 picocom
```

* `git`, `rsync`: `setup_vendor.sh` (clone or copy the ADI HDL library)
* `make`: ADI library and project build
* `g++`: `csim_gcc.sh` (optional fast C simulation)
* `libtinfo5`, `libncurses5`: `xsct` on Ubuntu fails with `libtinfo.so.5: cannot open shared object file` without them
* `picocom`: serial console (any terminal program works)

Only needed for JTAG boot, the Xilinx cable drivers:

```bash
cd /tools/Xilinx/Vivado/2023.2/data/xicom/cable_drivers/lin64/install_script/install_drivers
```

```bash
sudo ./install_drivers
```

---

## 1. Unpack the package

```bash
mkdir -p /home/manoranjan/CRPA_Project/SFAP
```

```bash
cd /home/manoranjan/CRPA_Project/SFAP
```

```bash
unzip SFAP_snd2.zip
```

```bash
cd SFAP_snd2/Automation/Linux
```

```bash
chmod +x *.sh
```

**All following commands are run from `SFAP_snd2/Automation/Linux`.**

---

## 2. Set up the environment (in every new terminal)

```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh
```

```bash
source /tools/Xilinx/Vitis/2023.2/settings64.sh
```

```bash
source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh
```

```bash
export SFAP_ROOT=/home/manoranjan/CRPA_Project/SFAP/SFAP_snd2
```

```bash
export SFAP_JOBS=$(nproc)
```

`SFAP_ROOT` must be the folder that contains `Source/` and `Automation/`. If it
is wrong, the `check` stage reports every source file as MISSING.

Confirm the tools:

```bash
vivado -version | head -1
```

```bash
which vitis_hls xsct bootgen make
```

`vivado -version` **must print v2023.2**. The ADI version check does not
protect you here: `env.sh` always exports `ADI_IGNORE_VERSION_CHECK` (as `0`),
and ADI's `scripts/adi_env.tcl` turns the check off whenever that variable
*exists*, whatever its value. With another Vivado version the build carries on
with only a CRITICAL WARNING and can fail much later.

---

## 3. Get the ADI HDL library (one time)

It is placed in `$SFAP_ROOT/Vendor/ADI_hdl_2023_r2`.

**Option A (faster):** copy the tree your STAP project already built (its
library IPs are already packaged, so the `hdl` stage skips most library builds):

```bash
./setup_vendor.sh /home/manoranjan/CRPA_Project/<STAP project>/Vendor/ADI_hdl_2023_r2
```

**Option B:** clone it fresh from GitHub (branch `hdl_2023_r2`):

```bash
./setup_vendor.sh
```

Expected last line: `setup_vendor: OK` (or `... already present` on a re-run).

---

## 4. (Optional, about 30 s) C simulation with g++

```bash
./csim_gcc.sh
```

Expected: `SFAP_PI_CSIM: PASS (scenario N)` for N = 1…9, then `CSIM_GCC: PASS`.
If this passes you can use `--skip-csim` in the `hls` stage.

---

## 5. Build, one stage at a time

Running the stages one by one the first time shows exactly where something
fails. Each stage writes `$SFAP_ROOT/Build/logs/<stage>.log` and prints one
`PASS` line or `BUILD FAILED in stage …`. To follow a long stage, open a second
terminal and run `tail -f` on its log.

### 5.1 check: tools, sources, ADI tree

```bash
./build_all.sh --only check
```

Expected: `CHECK: PASS`

### 5.2 hls: Vitis HLS C sim + synthesis + IP export of `sfap_pi`

```bash
./build_all.sh --only hls
```

Or, if step 4 already passed:

```bash
./build_all.sh --only hls --skip-csim
```

Expected:

```
SFAP_PI_HLS: estimated clock period … ns (target 10 ns), achieved II 1 (requested 1)
HLS: PASS -> …/Build/ip_repo/sfap_pi
```

Output: `Build/ip_repo/sfap_pi/component.xml`

### 5.3 ip: package `gnss_passthrough`

```bash
./build_all.sh --only ip
```

Expected: `IP_PACKAGE_OK: …` and `IP: PASS -> …/Build/ip_repo/gnss_passthrough`

### 5.4 hdl: ADI library IPs + Vivado project + bitstream + XSA (30–90 min)

```bash
./build_all.sh --only hdl
```

Follow progress in a second terminal:

```bash
tail -f $SFAP_ROOT/Build/logs/hdl.log
```

```bash
tail -f $SFAP_ROOT/Vendor/ADI_hdl_2023_r2/projects/gnss_sfap/e310/gnss_sfap_e310_vivado.log
```

Expected: the utilisation lines (LUT / FF / BRAM / DSP), then
`HDL: PASS -> …/Build/hdl/system_top.{bit,xsa} (timing met)`.
This stage **fails on purpose if timing is not met.**

Output: `Build/hdl/system_top.bit`, `Build/hdl/system_top.xsa`

### 5.5 sw: XSCT platform + FSBL + BSP + `app_gnss_e310.elf`

```bash
./build_all.sh --only sw
```

Expected: `SW: PASS -> …/Build/sw/{fsbl,app_gnss_e310}.elf`

Output: `Build/sw/fsbl.elf`, `Build/sw/app_gnss_e310.elf`, `Build/sw/ps7_init.tcl`

### 5.6 boot: bootgen → BOOT.BIN

```bash
./build_all.sh --only boot
```

Expected: `BOOT: PASS -> …/Build/boot/BOOT.BIN`

### Alternative: everything in one command

```bash
./build_all.sh
```

(`./build_all.sh --skip-csim` if step 4 passed.) Resume after a failure with
`./build_all.sh --from <stage>`.

---

## 6. Check the result

```bash
ls -l $SFAP_ROOT/Build/boot/
```

```bash
cat $SFAP_ROOT/Build/boot/boot.bif
```

```bash
cd $SFAP_ROOT/Build/boot && sha256sum -c BOOT.BIN.sha256 && cd -
```

```bash
bootgen -arch zynq -read $SFAP_ROOT/Build/boot/BOOT.BIN
```

`boot.bif` must list 3 partitions in this order: `[bootloader] fsbl.elf`,
`system_top.bit`, `app_gnss_e310.elf`.

Final output tree:

```
Build/
├── ip_repo/sfap_pi/            HLS IP   (antsdr:gnss:sfap_pi:3.0)
├── ip_repo/gnss_passthrough/   RTL IP   (antsdr:gnss:gnss_passthrough:1.0)
├── hdl/system_top.bit, system_top.xsa
├── sw/fsbl.elf, app_gnss_e310.elf, ps7_init.tcl
├── boot/boot.bif, BOOT.BIN, BOOT.BIN.sha256
└── logs/check|hls|ip|hdl|hdl_vivado|sw|boot.log
```

---

## 7. Boot the board

### 7a. From SD card

Find the card (e.g. `/dev/sdb`, partition `/dev/sdb1`). Be sure it is the SD card:

```bash
lsblk
```

Only if the card is not already FAT32 (**this erases the partition**):

```bash
sudo mkfs.vfat -F 32 /dev/sdX1
```

Copy BOOT.BIN:

```bash
sudo mount /dev/sdX1 /mnt
```

```bash
sudo cp $SFAP_ROOT/Build/boot/BOOT.BIN /mnt/
```

```bash
sync && sudo umount /mnt
```

Set the E310 boot jumpers to **SD**, insert the card, connect the USB-UART,
power up, and open the console (UART1, **115200 8N1**; check `dmesg | tail`
for the right `/dev/ttyUSB*`):

```bash
sudo picocom -b 115200 /dev/ttyUSB0
```

(Exit picocom with Ctrl-A then Ctrl-X.)

### 7b. Over JTAG (development, no SD card)

Boot jumpers on **JTAG**, JTAG cable connected, cable drivers installed (step 0):

```bash
./build_all.sh --only boot --jtag
```

This programs the bitstream, runs `ps7_init`, downloads the ELF and starts it.

### First commands on the console

```
gnss_status?        VERSION must read 0x00050000
gnss_aj?            "SFAP 64 bins, b + 2 image taps per bin", bypass until a jammer
gnss_report?        full report: per-bin info ("bins: N of 64 with J/N > 3 dB")
gnss_tx=1           start retransmitting
gnss_aj_mode=2      compare against the old single weight
```

---

## 8. Rebuilding after a change

| What you changed | Command |
|---|---|
| Firmware only (`Source/Firmware/app_gnss_e310/*`) | `./build_all.sh --from sw` |
| HLS core (`Source/HLS/sfap_pi/*`) | `./build_all.sh --from hls` |
| `Source/HDL/gnss_passthrough.v` or `Source/IP/gnss_passthrough/*` | `./build_all.sh --from ip` |
| `system_top.v`, `system_bd.tcl`, `system.xdc`, `gnss_passthrough_cdc.xdc` | `./build_all.sh --from hdl` |

The `hdl` stage deletes and regenerates the Vivado project in
`Vendor/ADI_hdl_2023_r2/projects/gnss_sfap/e310` every time; make your
changes in `Source/`, never inside that folder.

---

## 9. If a stage fails

| Stage / message | Where to look | Usual fix |
|---|---|---|
| `check`: `tool … MISSING` | `Build/logs/check.log` | Source the three `settings64.sh` (step 2) |
| `check`: `source … MISSING` | `Build/logs/check.log` | `SFAP_ROOT` points to the wrong folder (step 2) |
| `check`: `ADI hdl: MISSING` | — | Run `./setup_vendor.sh` (step 3) |
| `xsct`: `libtinfo.so.5` | terminal | `sudo apt install libtinfo5 libncurses5` |
| `hls`: `C simulation scenario N failed` | `Build/logs/hls.log` | Run `./csim_gcc.sh N` for a quick repeat |
| `hls`: clock period exceeds target / II worse than 1 | `Source/HLS/sfap_pi/build/sfap_pi_hls/sol/syn/report/sfap_pi_csynth.rpt` | HLS scheduling problem in `sfap_pi.cpp` |
| `ip`: no `IP_PACKAGE_OK` | `Build/logs/ip.log` (look for `IP_PACKAGE_SELFCHECK: FAIL` / `ERROR`) | |
| `hdl`: `sfap_pi IP missing` / `gnss_passthrough IP missing` | — | `./build_all.sh --from hls` |
| `hdl`: `make failed` | `Build/logs/hdl.log` (ADI library build), `Build/logs/hdl_vivado.log` (block design / synth / impl) | |
| `hdl`: `timing NOT met` | `Vendor/ADI_hdl_2023_r2/projects/gnss_sfap/e310/timing_impl.log` | Fix the failing paths; the bad-timing XSA is not used |
| `sw`: `software build` | `Build/logs/sw.log` (search `error:` / `undefined reference`) | |
| `boot`: `missing …` | — | Re-run the stage that makes that file |
