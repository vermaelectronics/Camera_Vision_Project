> **This folder (`LoopBack_STAP_CRPA2`) is the CRPA version.** The `gnss_stap` anti-jam stage is inserted in front of
> `gnss_passthrough`. Read **README_CRPA_PIPELINE.md** first. The text below describes the base loopback,
> which is still what the board does while the CRPA stage is in bypass (its reset state).

# ANTSDR E310 V1 - plain LoopBack pipeline (no CRPA, no anti-jam)

    RX1/RX2 -> AD9361 -> axi_ad9361 -> gnss_passthrough (PL, identity) -> axi_ad9361 -> TX1/TX2

This is the loopback project **without** any anti-jam / CRPA processing. It was made from the first loopback version of this repository
(`LoopBack_Code`, the transparent `gnss_passthrough`), and it contains none of the later additions:

| Not in this folder | Why it matters |
|---|---|
| SMI-PI HLS core (`smi_pi_0`), its two FIFOs, `Source/HLS` | no Vitis HLS stage, no 100 MHz filter clock domain |
| STAP / SFAP / I/Q-image corrector (`sfap_*.v`), pulse blanker | nothing in the PL touches the samples |
| `gnss_aj*` firmware (weights, detection, `gnss_aj_*` commands) | the console only has the radio and loopback commands |
| CRPA coefficient registers (0x40..0x7C) | those addresses now read 0 |

The PL block `gnss_passthrough` copies the RX samples to the TX path through a 32-deep elastic buffer. Sample values are bit-for-bit the RX values
(12-bit, converted to the left-aligned TX format). Register map: `Source/HDL/gnss_passthrough.v` header (ID 0x47435031, version, scratch, control,
status, counters, snapshots, FIFO level).

Checked here: the HDL compiles (iverilog) and a testbench shows the registers read correctly (ID, scratch write/read, 0x40 reads 0) and the TX data
equal the RX data (left-aligned, delayed by the elastic buffer) on both channels. **It has not been built in Vivado and has not run on the board.**
The Vivado/Vitis 2023.2 build fixes of the later versions (build.tcl run status, create_project incremental checkpoints, build_software domain name,
global synthesis) were copied in.

## Build (up to BOOT.BIN)

```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh
source /tools/Xilinx/Vitis/2023.2/settings64.sh
cd ~/LoopBack_STAP_CRPA2
chmod +x Automation/Linux/*.sh
./Automation/Linux/build_all.sh --jobs 16 --to boot 2>&1 | tee build_loopback.log
```

Stages: 1 vendor, 2 libip, 3 ip, 4 project, 5 build, 6 sw, 7 fsbl, 8 boot. `BUILD_ALL: PASS (stages 1..8)` at the end, then
`Build/Output/BOOT.BIN`. SD-card (unattended TX at power-on) build: add `--defines GNSS_DEPLOY_AUTO_TX`. **TX1 then transmits on L1 by itself:
conducted, attenuated coax only.** The old prebuilt `SD_Image/BOOT.BIN` is not included, build your own.

Environment variables of the scripts are now `LB_ROOT` and `LB_IP_REPO` (they were `GNSS_CRPA_*`).

## Known limits

- The first loopback version timed the AD9361 interface for the sample rate it used then. If Vivado reports `BUILD_TIMING: VIOLATED` on `rx_clk` at 61.44 MS/s,
  that is the constraint in `Source/XDC/system.xdc`, not the loopback logic; send the report.
- RX2 has nothing connected unless you connect an antenna or splitter.
