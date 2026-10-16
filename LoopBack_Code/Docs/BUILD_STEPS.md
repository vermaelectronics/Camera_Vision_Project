# Build LoopBack_Code (LCMV v2.0) up to BOOT.BIN — step by step

Host: Ubuntu 22.04, **Vivado + Vitis 2026.1** (your `vivado.log`).
Every command after step 0 runs from `~/AntSDR_LCMV/LoopBack_Code`.

## 0. One-time setup  (zip in ~/Downloads, new folder ~/AntSDR_LCMV)

Your old tree `~/Desktop/AntSDR2/LoopBack_Code` is only **read** (its `Vendor/` is copied) and is never changed.

```bash
sudo apt install -y unzip rsync make iverilog

# 0.1  get the setup script out of the downloaded zip and run it
cd ~/Downloads
unzip -o -j "$(ls -t LoopBack_Code_LCMV_v2*.zip | head -1)" \
      LoopBack_Code/Automation/setup_antsdr_lcmv.sh -d ~/Downloads
bash ~/Downloads/setup_antsdr_lcmv.sh
#   -> creates ~/AntSDR_LCMV/LoopBack_Code  (sources from the zip)
#   -> copies  ~/Desktop/AntSDR2/LoopBack_Code/Vendor  into it
#   (old Vendor elsewhere?  OLD_VENDOR=/path/to/Vendor bash ~/Downloads/setup_antsdr_lcmv.sh)

# 0.2  find your Vitis 2026.1 settings file
ls /tools/Xilinx/*/Vitis/settings64.sh ~/Xilinx/*/Vitis/settings64.sh 2>/dev/null

cd ~/AntSDR_LCMV/LoopBack_Code
```

The same setup by hand, without the script:
```bash
mkdir -p ~/AntSDR_LCMV
unzip -o "$(ls -t ~/Downloads/LoopBack_Code_LCMV_v2*.zip | head -1)" -d ~/AntSDR_LCMV
rsync -a ~/Desktop/AntSDR2/LoopBack_Code/Vendor/ ~/AntSDR_LCMV/LoopBack_Code/Vendor/
cd ~/AntSDR_LCMV/LoopBack_Code
chmod +x Automation/*.sh Sim/run_sim.sh
```

## Option A — everything in one command

```bash
export XILINX_SETTINGS=/tools/Xilinx/2026.1/Vitis/settings64.sh   # your install path
./Automation/build_all.sh              # bench build: LCMV off, no auto-TX
# DEPLOY=1 ./Automation/build_all.sh   # SD deployment: transmits at boot (see SD_Image/README)
```

The output is `Build/Boot/BOOT.BIN`. Logs are in `Build/Logs/`, one per stage. The script stops at the
first failure and prints the last lines of that stage's log.

## Option B — the same steps by hand

### Step 1  Environment (every new terminal)
```bash
export XILINX_SETTINGS=/tools/Xilinx/2026.1/Vitis/settings64.sh
source Automation/env.sh
mkdir -p Build/Logs Build/Bit Build/XSA Build/ELF Build/Boot
```

### Step 2  Simulate the LCMV block (optional, 1 min)
```bash
./Sim/run_sim.sh            # must end with: PASS
```

### Step 3  Working copy of the ADI HDL tree
```bash
rsync -a Vendor/ADI_hdl_2026_r1_update/ Build/VendorWork/hdl/
```

### Step 4  Package the ADI library IPs used by the block design (about 10–20 min)
```bash
for lib in axi_ad9361 axi_dmac axi_gpreg axi_sysid sysid_rom \
           util_pack/util_cpack2 util_pack/util_upack2 \
           util_rfifo util_wfifo util_tdd_sync xilinx/util_clkdiv; do
  make -C Build/VendorWork/hdl/library/$lib || break
done
ls Build/VendorWork/hdl/library/axi_ad9361/component.xml     # must exist
```

### Step 5  Package the custom IP gnss_passthrough v2.0 (with Source/HDL/crpa)
```bash
( cd Build/Logs && vivado -mode batch -nojournal -nolog \
    -source ../../Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl \
    -tclargs "$GNSS_CRPA_ROOT/Source" "$GNSS_CRPA_IP_REPO" ) | tee Build/Logs/3_gnss_ip.log
grep "LCMV CRPA sources packaged" Build/Logs/3_gnss_ip.log    # must print a line
```

### Step 6  Create the Vivado project
```bash
( cd Vivado/Project && vivado -mode batch -source ../Scripts/create_project.tcl ) \
  | tee Build/Logs/4_create_project.log
grep "CREATE_RESULT: PASS" Build/Logs/4_create_project.log
```

### Step 7  Synthesis, implementation, bitstream and XSA (about 20–40 min)
```bash
( cd Vivado/Project && vivado -mode batch -source ../Scripts/build.tcl -tclargs \
    "$GNSS_CRPA_ROOT/Vivado/Project/antsdr_e310_gnss.xpr" \
    "$GNSS_CRPA_ROOT/Build/Bit/system_top.bit" \
    "$GNSS_CRPA_ROOT/Build/XSA/system.xsa" 8 ) | tee Build/Logs/5_build.log
grep -E "BUILD_RESULT|BUILD_TIMING|BUILD_WNS|BUILD_WHS" Build/Logs/5_build.log
```
You need `BUILD_RESULT: PASS` **and** `BUILD_TIMING: MET`. If timing is violated, look at
`Build/Bit/timing_impl.rpt` before using the bitstream.

### Step 8  FSBL
```bash
vitis -s Vitis/Scripts/build_fsbl.py -- \
  "$GNSS_CRPA_ROOT/Build/XSA/system.xsa" "$GNSS_CRPA_ROOT/Vitis/Workspace/fsbl" \
  "$GNSS_CRPA_ROOT/Build/ELF/fsbl.elf" | tee Build/Logs/6_fsbl.log
grep "FSBL_RESULT: PASS" Build/Logs/6_fsbl.log
```

### Step 9  Application firmware
```bash
# bench build:
vitis -s Vitis/Scripts/build_software.py -- \
  "$GNSS_CRPA_ROOT/Build/XSA/system.xsa" "$GNSS_CRPA_ROOT/Vitis/Workspace/app" \
  "$GNSS_CRPA_ROOT/Source/Firmware/app_gnss_e310" \
  "$GNSS_CRPA_ROOT/Build/ELF/app_gnss_e310.elf" | tee Build/Logs/7_app.log
# SD deployment build (auto-TX at boot): add  GNSS_DEPLOY_AUTO_TX  as a 5th argument
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
lsblk                                   # find the SD card partition, e.g. /dev/sdb1 (FAT32)
sudo mount /dev/sdb1 /mnt && sudo cp Build/Boot/BOOT.BIN /mnt/ && sync && sudo umount /mnt
```
Set the E310 boot switch to SD and power it on. On the UART console (115200 8N1):
```
gnss_status?           -> gnss_passthrough version 2.0
lcmv_angles=10 40
lcmv=1
lcmv?                  -> ACTIVE, weights, update count rising
```
RF safety rules in `SD_Image/README_SD_CARD.md` still apply: conducted, attenuated coax only.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `CREATE_RESULT: FAIL - post-route timing script missing` | `ADI_HDL_DIR` points at `MicroPhase_E310_V1/hdl`. Use `source Automation/env.sh` (Build/VendorWork copy of ADI_hdl_2026_r1_update). |
| `vivado version mismatch` | `ADI_IGNORE_VERSION_CHECK=1` was not exported. Re-run `source Automation/env.sh`. |
| BD error: IP `antsdr:gnss:gnss_passthrough` not found | Step 5 not run, or `GNSS_CRPA_IP_REPO` not set in this terminal. |
| BD error: ADI IP (e.g. `axi_ad9361`) not found | Step 4 not finished for that library. |
| `gnss_status?` reports 1.1 | The old BOOT.BIN is still on the card, or the Build/ip_repo from v1.1 was reused. Redo from step 0.4. |
| `BUILD_TIMING: VIOLATED` | Send `Build/Bit/timing_impl.rpt` (worst paths) for analysis. |
