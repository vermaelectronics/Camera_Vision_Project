# Build SMI-PI v4.1 step by step (zip → BOOT.BIN → SD card)

Every command below is meant to be typed in a Linux terminal (Ubuntu 22.04 or
20.04) on the PC that has the Xilinx tools. Wait for each step to pass before
going to the next one. Each step prints a check line at the end.

---

## 0. What you need (once)

| Item | Value |
|---|---|
| OS | Ubuntu 22.04 (20.04 also works) |
| Tools | **Vivado, Vitis and Vitis HLS 2023.2**, installed under `/tools/Xilinx` or `/opt/Xilinx`. The xc7z020 is covered by the free licence. |
| Disk / RAM | ~40 GB free, 16 GB RAM (8 GB works with `--jobs 4`) |
| Packages | `sudo apt install unzip make gcc g++ git` |

Check that the tools are there:

```bash
ls /tools/Xilinx/Vivado/2023.2/settings64.sh /tools/Xilinx/Vitis/2023.2/settings64.sh /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh
```

(If they are in `/opt/Xilinx`, use that path in all the commands.)

---

## 1. Unzip into a NEW folder

```bash
mkdir -p ~/smi_pi_v41
cd ~/smi_pi_v41
unzip ~/Downloads/LoopBack_Code_SMI_PI_v4.1.4.zip
cd LoopBack_Code_SMI_PI
chmod +x Automation/Linux/*.sh Automation/Linux/steps/*.sh
```

Do **not** unzip over the v4.0 folder: an old `Build/ip_repo/smi_pi` (version
2.0) must not be reused. The block design refuses it anyway.

---

## 2. Load the tools (every new terminal)

```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh
source /tools/Xilinx/Vitis/2023.2/settings64.sh
source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh
vivado -version | head -1        # must say v2023.2
which vitis_hls xsct bootgen      # all three must be found
```

---

## 3. Build, one step at a time

Run these from the `LoopBack_Code_SMI_PI` folder. The times are typical for
a 16-thread PC.

### Step 1: copy the ADI library (seconds)

```bash
./Automation/Linux/steps/01_vendor.sh
```
Check: `STEP 1 vendor: PASS`

### Step 2: package the ADI library IP (5–10 min)

```bash
./Automation/Linux/steps/02_libip.sh
```
Check: `STEP 2 libip: PASS (11 library logs ...)`

### Step 3: the SMI-PI filter in Vitis HLS (5–15 min)

```bash
./Automation/Linux/steps/03_smi.sh
```
Check, all of these in the output:
```
SMI_PI_CSIM: PASS (scenario 1)   ... up to (scenario 8)
SMI_PI_HLS: estimated clock period X ns (target 10 ns), achieved II 1 (requested 1)
SMI_PI_HLS: IP antsdr:gnss:smi_pi:2.1 -> .../Build/ip_repo/smi_pi
SMI_PI_HLS: PASS
```
If the clock estimate is above 10 ns, run `SMI_PI_CLK_NS=8 ./Automation/Linux/steps/03_smi.sh`
(HLS then pipelines deeper; the hardware clock stays 100 MHz).
To see one scenario's numbers: `grep -A12 "scenario 6" Build/Logs/smi.log`

### Step 4: package gnss_passthrough (1–2 min)

```bash
./Automation/Linux/steps/04_ip.sh
```
Check: `IP_PACKAGE_OK`

### Step 5: create the Vivado project (2–5 min)

```bash
./Automation/Linux/steps/05_project.sh
```
Check: `CREATE_RESULT: PASS`
(`GNSS-CRPA: SMI-PI IP resolved in catalog: antsdr:gnss:smi_pi:2.1` is
printed in `Build/Logs/project.log`.)

### Step 6: synthesis, implementation, bitstream (20–60 min)

```bash
./Automation/Linux/steps/06_build.sh
```
Check:
```
BUILD_TIMING: MET
BUILD_RESULT: PASS
Build/Output/system_top.bit   Build/Output/system.xsa   (listed)
```
If it says `BUILD_TIMING: VIOLATED` with failing paths on `rx_clk`: open
`Source/XDC/system.xdc` and change `-period  4` to `-period  5` on the
`rx_clk` line. Then run steps 5 and 6 again. **But** at 5 ns the default
profile (61.44 MS/s, the only one that carries BeiDou B1I AND all of
GLONASS) is no longer covered: you must then type `gnss_band=2` (GPS,
Galileo, GLONASS) or `gnss_band=3` (GPS, Galileo, BeiDou) after every boot.
Try hard to meet 4 ns first (the v4.0 build already did).

### Step 7: the application ELF (2–5 min)

Choose ONE:

```bash
# (a) bench / console build: TX1 silent at power-on, you type gnss_tx=1
./Automation/Linux/steps/07_sw.sh

# (b) unattended SD-card build: TX1 starts BY ITSELF at power-on (70 dB attenuation)
#     CONDUCTED, ATTENUATED COAX ONLY. Read SD_Image/README_SD_CARD.md first.
./Automation/Linux/steps/07_sw.sh --defines GNSS_DEPLOY_AUTO_TX
```
Check: `SW_RESULT: PASS`

Options you can add to `--defines` (comma-separated):

| Define | Effect |
|---|---|
| `GNSS_DEPLOY_AUTO_TX` | TX1 on at power-on (the shortest time to fix: no typing) |
| `GNSS_TX_START_ATTEN_MDB=65000` | level `gnss_tx=1` starts at (default 70000 = 70 dB; bigger = quieter) |
| `GNSS_DEPLOY_TX_ATTEN_MDB=65000` | level of the auto-start build (default 70000) |
| `GNSS_BOOT_DIAG` | brings back the boot I/Q captures and band probe (adds ~1.5 s) |

Example: `./Automation/Linux/steps/07_sw.sh --defines GNSS_DEPLOY_AUTO_TX,GNSS_DEPLOY_TX_ATTEN_MDB=65000`

### Step 8: the FSBL (1–3 min)

```bash
./Automation/Linux/steps/08_fsbl.sh
```
Check: `FSBL_RESULT: PASS`

### Step 9: BOOT.BIN (seconds)

```bash
./Automation/Linux/steps/09_boot.sh
```
Check: `Build/Output/BOOT.BIN` is listed, with its size. Note the sha256 that
`Build/Logs/boot.log` prints.

### Step 10: one-page report (optional)

```bash
./Automation/Linux/steps/10_report.sh
```
It prints a pass/fail line per stage, the HLS clock/II and the timing. Paste
it into your next message if anything failed.

---

## 4. All in one go (same result)

```bash
./Automation/Linux/build_all.sh --jobs 16                                  # bench build
./Automation/Linux/build_all.sh --jobs 16 --defines GNSS_DEPLOY_AUTO_TX    # SD-card build
```

Only the firmware changed (e.g. a different `--defines`)? Then:

```bash
./Automation/Linux/build_all.sh --from sw --to boot --defines GNSS_DEPLOY_AUTO_TX
```

---

## 5. SD card

1. Format the card as **FAT32**.
2. Copy **`Build/Output/BOOT.BIN`** to the **root** of the card. Do not use
   `SD_Image/BOOT.BIN`: that is an old image without SMI-PI v4.1.
3. Set the board's **BOOT switch to SD**, insert the card, connect RF:

| Port | Connect |
|---|---|
| RX1, RX2 | the two antennas (same antenna type and same cable length on both) |
| TX1 | GNSS receiver, conducted coax with attenuation |
| USB (console) | PC, PuTTY at 115200 8N1 (see `TESTING_PUTTY.md`) |

4. Power on. The console must show (among other lines):

```
gnss_pt: present at 0x43c00000, version 4.1
SELFTEST: PASS - ...
ANTI-JAM (SMI-PI 4.1.4): ON, bypassed until a jammer appears.
```

If it says `version 4.0` or `Anti-jam NOT started`, the card has an old
BOOT.BIN on it.

Then type `gnss_band?`. All five lines (GPS, Galileo/B1C, BeiDou B1I,
GLONASS k=-7, k=+6) must say `OK`, and the AD9361 FIRs must be `off`.

---

## 6. If a step fails

```bash
tail -50 Build/Logs/<stage>.log        # stage = smi, ip, project, build, sw, fsbl, boot
./Automation/Linux/steps/10_report.sh
```

Re-run the same step after fixing. Steps 1–4 can be repeated safely. Step 5
deletes and recreates the Vivado project.

Send me: the failing step, the last 50 lines of its log and the
`10_report.sh` output.
