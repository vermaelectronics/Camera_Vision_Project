# Build LoopBack_Code v4.0 (gnss_passthrough + LCMV HLS IP lcmv_0) up to BOOT.BIN

Host: Ubuntu 22.04, **Vivado + Vitis + Vitis HLS 2023.2**. Zip: `~/Downloads/LoopBack_Code_LCMV_v4.zip`.
Work folder: `~/Desktop/AntSDR_LCMV/LoopBack_Code`. Run every step in the same terminal and check each ✔ before going on.

New in v4.0: **Step 5 builds the LCMV IP with Vitis HLS**. The block design instantiates it as `lcmv_0`, next to
`gnss_passthrough`, the same way `pi_nlms_0` was connected.

## Step 1 — environment
```bash
cd ~/Desktop/AntSDR_LCMV/LoopBack_Code
export XILINX_SETTINGS=/tools/Xilinx/Vitis/2023.2/settings64.sh
source Automation/env.sh
mkdir -p Build/Logs Build/Bit Build/XSA Build/ELF Build/Boot
vivado -version | head -1; vitis_hls -version | head -1
```
✔ `vivado v2023.2` and `Vitis HLS ... v2023.2`.

## Step 2 — sources from the zip (keeps Build/ and Vendor/)
```bash
rm -rf Source Sim
unzip -o "$(ls -t ~/Downloads/LoopBack_Code_LCMV_v4*.zip | head -1)" \
  "LoopBack_Code/Source/*" "LoopBack_Code/Sim/*" "LoopBack_Code/Vitis/*" \
  "LoopBack_Code/Docs/*" "LoopBack_Code/Automation/*" -d ~/Desktop/AntSDR_LCMV
chmod +x Automation/*.sh Sim/run_sim.sh
ls Source/HLS/lcmv/lcmv_crpa.cpp Source/HLS/lcmv/build_hls.tcl
```
✔ both files listed.

## Step 3 — system simulation (about 1 min)
```bash
./Sim/run_sim.sh | tail -4
```
✔ ends with `PASS`.

## Step 4 — ADI libraries (only if missing)
```bash
ls Build/VendorWork/hdl/library/axi_ad9361/component.xml
```
If "No such file":
```bash
[ -d Vendor/ADI_hdl_2023_r2 ] || \
  git clone --depth 1 -b hdl_2023_r2 https://github.com/analogdevicesinc/hdl.git Vendor/ADI_hdl_2023_r2
rsync -a --exclude .git Vendor/ADI_hdl_2023_r2/ Build/VendorWork/hdl/
for lib in axi_ad9361 axi_dmac axi_gpreg axi_sysid sysid_rom \
           util_pack/util_cpack2 util_pack/util_upack2 \
           util_rfifo util_wfifo util_tdd_sync xilinx/util_clkdiv; do
  echo "=== $lib"; make -C Build/VendorWork/hdl/library/$lib || break
done
```

## Step 5 — LCMV HLS IP (Vitis HLS: C sim, synthesis, IP export, about 5 min)
```bash
( cd Source/HLS/lcmv && vitis_hls -f build_hls.tcl ) > Build/Logs/3b_lcmv_hls.log 2>&1
grep -E "^LCMV_HLS|LCMV_CSIM" Build/Logs/3b_lcmv_hls.log
ls Build/ip_repo/lcmv_crpa/component.xml
```
✔ `LCMV_CSIM: PASS`, `LCMV_HLS: estimated clock period ... achieved II 2`, `LCMV_HLS: PASS`, and the file exists.

## Step 6 — package gnss_passthrough v4.0
```bash
( cd Build/Logs && vivado -mode batch -nojournal -nolog \
    -source ../../Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl \
    -tclargs "$GNSS_CRPA_ROOT/Source" "$GNSS_CRPA_IP_REPO" ) > Build/Logs/3_gnss_ip.log 2>&1
grep -E "^IP_PACKAGE" Build/Logs/3_gnss_ip.log
```
✔ `IP_PACKAGE_SELFCHECK: OK - LCMV AXI-Stream interfaces: m_axis_lcmv1 m_axis_lcmv2 s_axis_lcmv` and `IP_PACKAGE_OK`.

## Step 7 — Vivado project (block design with lcmv_0, about 5 min)
```bash
( cd Vivado/Project && rm -rf antsdr_e310_gnss.* && \
  vivado -mode batch -source ../Scripts/create_project.tcl ) > Build/Logs/4_create_project.log 2>&1
grep -E "^CREATE_RESULT|LCMV IP resolved" Build/Logs/4_create_project.log
```
✔ `GNSS-CRPA: LCMV IP resolved in catalog: antsdr:gnss:lcmv_crpa:1.0` and `CREATE_RESULT: PASS`.

## Step 8 — bitstream + XSA (20–40 min)
```bash
( cd Vivado/Project && vivado -mode batch -source ../Scripts/build.tcl -tclargs \
    "$GNSS_CRPA_ROOT/Vivado/Project/antsdr_e310_gnss.xpr" \
    "$GNSS_CRPA_ROOT/Build/Bit/system_top.bit" \
    "$GNSS_CRPA_ROOT/Build/XSA/system.xsa" 8 ) > Build/Logs/5_build.log 2>&1
grep -E "^BUILD_RESULT|^BUILD_TIMING|^BUILD_WNS|^BUILD_WHS" Build/Logs/5_build.log
```
✔ `BUILD_RESULT: PASS` and `BUILD_TIMING: MET`. Progress: `tail -f Build/Logs/5_build.log` in a second terminal.

## Step 9 — FSBL
```bash
xsct Vitis/Scripts/build_fsbl.tcl \
  "$GNSS_CRPA_ROOT/Build/XSA/system.xsa" "$GNSS_CRPA_ROOT/Vitis/Workspace/fsbl" \
  "$GNSS_CRPA_ROOT/Build/ELF/fsbl.elf" > Build/Logs/6_fsbl.log 2>&1
grep -E "^FSBL_RESULT" Build/Logs/6_fsbl.log
```
✔ `FSBL_RESULT: PASS`.

## Step 10 — firmware
```bash
rm -rf Vitis/Workspace/app
xsct Vitis/Scripts/build_software.tcl \
  "$GNSS_CRPA_ROOT/Build/XSA/system.xsa" "$GNSS_CRPA_ROOT/Vitis/Workspace/app" \
  "$GNSS_CRPA_ROOT/Source/Firmware/app_gnss_e310" \
  "$GNSS_CRPA_ROOT/Build/ELF/app_gnss_e310.elf" > Build/Logs/7_app.log 2>&1
grep -E "^SW_RESULT|^SW_ERROR" Build/Logs/7_app.log
```
✔ `SW_RESULT: PASS`. If not: `grep -n "error" Build/Logs/7_app.log | head -20`.

## Step 11 — BOOT.BIN
```bash
cat > Build/Boot/boot.bif <<BIF
the_ROM_image:
{
  [bootloader] $GNSS_CRPA_ROOT/Build/ELF/fsbl.elf
  $GNSS_CRPA_ROOT/Build/Bit/system_top.bit
  $GNSS_CRPA_ROOT/Build/ELF/app_gnss_e310.elf
}
BIF
bootgen -arch zynq -image Build/Boot/boot.bif -o Build/Boot/BOOT.BIN -w on
ls -l Build/Boot/BOOT.BIN
```
✔ `Bootimage generated successfully`, about 4.5 MB.

## Step 12 — SD card and first check
```bash
lsblk                                       # SD card FAT32 partition, e.g. sdb1
sudo mount /dev/sdb1 /mnt && sudo cp Build/Boot/BOOT.BIN /mnt/ && sync && sudo umount /mnt
```
PuTTY 115200, *Local echo: Force on*. Expected at boot:
```
gnss: firmware started ...
gnss_rx: RX link OK ...
gnss_lcmv: LCMV on, GPS 30 deg, jammer 60 deg ...
```
Then: `gnss_status?` → version **4.0**; `gnss_lcmv?`; `gnss_tx=1`.

## One command instead (Steps 4–11)
```bash
SKIP_ADI_LIBS=1 ./Automation/build_all.sh       # includes stage 3b (LCMV HLS)
```

## Troubleshooting
| Symptom | Fix |
|---|---|
| `vitis_hls: command not found` | Source the **Vitis** 2023.2 `settings64.sh` (it includes Vitis HLS). |
| `LCMV_HLS: FAIL - estimated clock period` | Send `Source/HLS/lcmv/build/lcmv_hls/sol/syn/report/lcmv_crpa_csynth.rpt`. |
| BD error `lcmv_crpa was not found in the IP catalog` | Step 5 did not pass, or `GNSS_CRPA_IP_REPO` is not set in this terminal (Step 1). |
| BD error on `lcmv_0/in1` or `gnss_passthrough/m_axis_lcmv1` | Re-run Step 5 and Step 6 (both IPs must come from the same zip). |
| `BUILD_TIMING: VIOLATED` | Send `Build/Bit/timing_impl.rpt`. |
| Board: `RX link FAILED` | Send the output of `gnss_adc?`. |
