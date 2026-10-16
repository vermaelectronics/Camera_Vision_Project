# Build LoopBack_Code (LCMV v2.0) up to BOOT.BIN — Vivado/Vitis 2023.2

Host: Ubuntu 22.04 with **Vivado + Vitis 2023.2**. The zip is in `~/Downloads`, and the work folder is `~/Desktop/AntSDR_LCMV`.
(2026.1 is still supported: put `TOOL_VER=2026.1` in front of any command.)

## What 2023.2 changes compared with 2026.1

| Part | 2023.2 | 2026.1 |
|---|---|---|
| ADI HDL library | **hdl_2023_r2** (ADI's release for Vivado 2023.2), cloned into `Vendor/ADI_hdl_2023_r2` | `Vendor/ADI_hdl_2026_r1_update` |
| FSBL | `Vitis/Scripts/build_fsbl.tcl` (XSCT `hsi generate_app zynq_fsbl`) | `build_fsbl.py` (Python API) |
| Firmware | `Vitis/Scripts/build_software.tcl` (XSCT, heap 1 MB, stack 64 KB) | `build_software.py` |
| BSP | classic `xparameters.h` (legacy names, `xparam_compat.h` becomes a no-op) | SDT `xparameters.h` |
| Post-route timing fix | not in hdl_2023_r2, so skipped (strategy and phys_opt still applied) | ADI `auto_timing_fix_xilinx.tcl` |

The RTL (gnss_passthrough v2.0 + LCMV), block design, constraints and firmware sources are identical for both.

## 0. One-time setup

```bash
sudo apt install -y unzip rsync make git iverilog

# 0.1 extract the setup script from the downloaded zip, then run it
cd ~/Downloads
unzip -o -j "$(ls -t LoopBack_Code_LCMV_v2*.zip | head -1)" \
      LoopBack_Code/Automation/setup_antsdr_lcmv.sh -d ~/Downloads
bash ~/Downloads/setup_antsdr_lcmv.sh
#   -> ~/Desktop/AntSDR_LCMV/LoopBack_Code            (sources)
#   -> copies ~/Desktop/AntSDR2/LoopBack_Code/Vendor if it exists (optional for 2023.2)

# 0.2 find the Vitis 2023.2 settings file
ls /tools/Xilinx/Vitis/2023.2/settings64.sh ~/Xilinx/Vitis/2023.2/settings64.sh 2>/dev/null
export XILINX_SETTINGS=/tools/Xilinx/Vitis/2023.2/settings64.sh      # path printed above

cd ~/Desktop/AntSDR_LCMV/LoopBack_Code
```

## Option A — one command

```bash
./Automation/build_all.sh              # bench build: LCMV off at boot, no auto-TX
# DEPLOY=1 ./Automation/build_all.sh   # SD deployment build: transmits at boot
```
The script clones ADI `hdl_2023_r2` (about 200 MB, first run only), then builds everything through to
`Build/Boot/BOOT.BIN`. Logs go to `Build/Logs/`, one per stage, and the script stops at the first failure.
The first run takes about 1 hour (ADI libraries plus implementation). `SKIP_ADI_LIBS=1` skips the
library stage on later runs.

## Option B — step by step

### Step 1  Environment (every new terminal)
```bash
cd ~/Desktop/AntSDR_LCMV/LoopBack_Code
export XILINX_SETTINGS=/tools/Xilinx/Vitis/2023.2/settings64.sh
source Automation/env.sh                  # prints vivado / xsct / bootgen paths
mkdir -p Build/Logs Build/Bit Build/XSA Build/ELF Build/Boot
vivado -version | head -1                 # must say v2023.2
```

### Step 2  Simulate the LCMV block (optional, 1 min)
```bash
./Sim/run_sim.sh                          # must end with: PASS
```

### Step 3  ADI HDL hdl_2023_r2 and its working copy
```bash
[ -d Vendor/ADI_hdl_2023_r2 ] || \
  git clone --depth 1 -b hdl_2023_r2 https://github.com/analogdevicesinc/hdl.git Vendor/ADI_hdl_2023_r2
rsync -a --exclude .git Vendor/ADI_hdl_2023_r2/ Build/VendorWork/hdl/
```

### Step 4  Package the ADI library IPs (about 10–20 min)
```bash
for lib in axi_ad9361 axi_dmac axi_gpreg axi_sysid sysid_rom \
           util_pack/util_cpack2 util_pack/util_upack2 \
           util_rfifo util_wfifo util_tdd_sync xilinx/util_clkdiv; do
  make -C Build/VendorWork/hdl/library/$lib || break
done
ls Build/VendorWork/hdl/library/axi_ad9361/component.xml      # must exist
```

### Step 5  Package gnss_passthrough v2.0 (with Source/HDL/crpa)
```bash
( cd Build/Logs && vivado -mode batch -nojournal -nolog \
    -source ../../Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl \
    -tclargs "$GNSS_CRPA_ROOT/Source" "$GNSS_CRPA_IP_REPO" ) | tee Build/Logs/3_gnss_ip.log
grep "LCMV CRPA sources packaged" Build/Logs/3_gnss_ip.log
```

### Step 6  Create the Vivado project
```bash
( cd Vivado/Project && rm -rf antsdr_e310_gnss.* && \
  vivado -mode batch -source ../Scripts/create_project.tcl ) | tee Build/Logs/4_create_project.log
grep "CREATE_RESULT: PASS" Build/Logs/4_create_project.log
```
`CREATE_WARNING: post-route timing script not in this ADI release` is expected with 2023.2.

### Step 7  Synthesis, implementation, bitstream and XSA (about 20–40 min)
```bash
( cd Vivado/Project && vivado -mode batch -source ../Scripts/build.tcl -tclargs \
    "$GNSS_CRPA_ROOT/Vivado/Project/antsdr_e310_gnss.xpr" \
    "$GNSS_CRPA_ROOT/Build/Bit/system_top.bit" \
    "$GNSS_CRPA_ROOT/Build/XSA/system.xsa" 8 ) | tee Build/Logs/5_build.log
grep -E "BUILD_RESULT|BUILD_TIMING|BUILD_WNS|BUILD_WHS" Build/Logs/5_build.log
```
You need `BUILD_RESULT: PASS` **and** `BUILD_TIMING: MET`.

### Step 8  FSBL (XSCT)
```bash
xsct Vitis/Scripts/build_fsbl.tcl \
  "$GNSS_CRPA_ROOT/Build/XSA/system.xsa" "$GNSS_CRPA_ROOT/Vitis/Workspace/fsbl" \
  "$GNSS_CRPA_ROOT/Build/ELF/fsbl.elf" | tee Build/Logs/6_fsbl.log
grep "FSBL_RESULT: PASS" Build/Logs/6_fsbl.log
```

### Step 9  Application firmware (XSCT)
```bash
xsct Vitis/Scripts/build_software.tcl \
  "$GNSS_CRPA_ROOT/Build/XSA/system.xsa" "$GNSS_CRPA_ROOT/Vitis/Workspace/app" \
  "$GNSS_CRPA_ROOT/Source/Firmware/app_gnss_e310" \
  "$GNSS_CRPA_ROOT/Build/ELF/app_gnss_e310.elf" | tee Build/Logs/7_app.log
#   SD deployment build (auto-TX at boot): add  GNSS_DEPLOY_AUTO_TX  as a 5th argument
grep "SW_RESULT: PASS" Build/Logs/7_app.log
```

### Step 10  BOOT.BIN
```bash
cat > Build/Boot/boot.bif <<EOF
the_ROM_image:
{
  [bootloader] $GNSS_CRPA_ROOT/Build/ELF/fsbl.elf
  $GNSS_CRPA_ROOT/Build/Bit/system_top.bit
  $GNSS_CRPA_ROOT/Build/ELF/app_gnss_e310.elf
}
EOF
bootgen -arch zynq -image Build/Boot/boot.bif -o Build/Boot/BOOT.BIN -w on
ls -l Build/Boot/BOOT.BIN
```

### Step 11  SD card and first check
```bash
lsblk                                       # find the SD card partition, e.g. /dev/sdb1 (FAT32)
sudo mount /dev/sdb1 /mnt && sudo cp Build/Boot/BOOT.BIN /mnt/ && sync && sudo umount /mnt
```
Set the boot switch to SD and open the console at 115200 8N1:
```
gnss_status?        -> gnss_passthrough version 2.0
lcmv_angles=10 40
lcmv=1
lcmv?               -> ACTIVE, weights, update count rising
```
RF safety rules in `SD_Image/README_SD_CARD.md` still apply.

## Troubleshooting (2023.2)

| Symptom | Cause / fix |
|---|---|
| `vivado -version` is not v2023.2 | A different install is on PATH. Open a new terminal and set `XILINX_SETTINGS` to the 2023.2 `settings64.sh`. |
| `git clone` fails | No network or proxy. Clone `hdl_2023_r2` on another machine and copy it to `Vendor/ADI_hdl_2023_r2`. |
| `make` in a library fails | Read `Build/Logs/2_lib_<name>.log`. Usually the wrong Vivado is on PATH. |
| BD error: IP not found (`axi_ad9361`, ...) | Step 4 is incomplete for that library. |
| BD error: `antsdr:gnss:gnss_passthrough` not found | Step 5 was not run, or `GNSS_CRPA_IP_REPO` is not set in this terminal. |
| `xsct: command not found` | Source the **Vitis** 2023.2 `settings64.sh`, not the Vivado one. |
| Firmware: `gnss_pt: base address not found ... using 0x43C00000` | A build warning only. The address is fixed by the block design and is correct. |
| Board prints nothing after boot | Check `Build/Logs/7_app.log` for `SW_LINKER: heap 0x100000 (set=1)`. If it says set=0, the template linker script changed; set heap and stack in the Vitis IDE. |
| `BUILD_TIMING: VIOLATED` | Without ADI's post-route fix, hold violations can remain. Send `Build/Bit/timing_impl.rpt`. |
