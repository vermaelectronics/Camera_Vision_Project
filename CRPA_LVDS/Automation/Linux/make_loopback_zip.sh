#!/usr/bin/env bash
# ============================================================================
#  make_loopback_zip.sh  --  the LOOP-BACK pipeline as its own package:
#  AD9361 RX -> gnss_passthrough -> AD9361 TX, no anti-jam (no HLS core, no
#  solver). Same scripts, block design, HDL and firmware as the CRPA builds,
#  built with the loop-back settings.
#
#    ./make_loopback_zip.sh [out_dir] [--run-tests]
#
#  --run-tests runs the HDL testbenches (register map + RX->TX datapath) from
#  the unpacked package (needs iverilog) and stores the output in test/.
# ============================================================================
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/../.." && pwd)"
out="${1:-$root/../CRPA_packages}"; run_tests=0
for a in "$@"; do [[ "$a" == --run-tests ]] && run_tests=1; done
[[ "$out" == --run-tests ]] && out="$root/../CRPA_packages"
mkdir -p "$out"; out="$(cd "$out" && pwd)"
name="LoopBack_LVDS"
stage="$(mktemp -d)"
pkg="$stage/$name"

# ---- the tree without anything anti-jam -----------------------------------------------
mkdir -p "$pkg"
( cd "$root" && tar --exclude=./Build --exclude=./test --exclude=./Source/CRPA --exclude=./Source/SFAP --exclude=./Source/HLS \
      --exclude=./Source/Firmware/host_test --exclude=./Vendor/hls_headers \
      --exclude=README_CRPA.md --exclude=BUILD_COMMANDS.md -cf - . ) | ( cd "$pkg" && tar -xf - )
rm -f "$pkg/Automation/Linux/make_algo_zip.sh" "$pkg/Automation/Linux/make_loopback_zip.sh" \
      "$pkg/Automation/Linux/csim_gcc.sh"
mkdir -p "$pkg/test"
sed -i "s/^CRPA_ALGOS=(.*)/CRPA_ALGOS=(loopback)\nCRPA_DEFAULT_ALGO=loopback      # this package: .\/build_all.sh builds the loop-back/" \
    "$pkg/Automation/Linux/crpa_algos.sh"
cat > "$pkg/Vendor/README.md" <<'EOF'
# Vendor/: third-party trees the build uses

| folder | what | where it comes from |
|---|---|---|
| `adi_hdl/` | the Analog Devices HDL tree (antsdr-fw-patch/plutosdr-fw/hdl): `scripts/`, `projects/scripts/`, `library/` with axi_ad9361, axi_dmac, axi_tdd, util_pack, axi_vcxo_ctrl ... | **added on your PC** by `Automation/Linux/bundle_full.sh` |

When `Vendor/adi_hdl/` exists, `env.sh` uses it automatically. Vivado, Vitis
and Vitis HLS 2023.2 (AMD/Xilinx, licensed) are not included and can't be.
EOF
find "$pkg" -name '*.sh' -exec chmod +x {} +

# ---- tests ----------------------------------------------------------------------------
if [[ $run_tests -eq 1 ]]; then
  ( cd "$pkg" && { echo "# Source/HDL/sim/run_iverilog.sh loopback ($(date -u +%F))"; Source/HDL/sim/run_iverilog.sh loopback; } ) \
      > "$pkg/test/hdl_tb_results_loopback.txt" 2>&1
  grep -q "HDL_TB: ALL PASS" "$pkg/test/hdl_tb_results_loopback.txt" || { echo "loopback: HDL tests FAILED, see $pkg/test"; exit 1; }
fi
hdl_lines=$(grep -E "^(HDL_TB|DATAPATH)" "$pkg/test/hdl_tb_results_loopback.txt" 2>/dev/null | sed 's/^/    /')

# ---- README ---------------------------------------------------------------------------
cat > "$pkg/README_loopback.md" <<EOF
# LoopBack_LVDS: ANTSDR E310 RX -> TX loop-back, no anti-jam

The plain retransmit pipeline: AD9361 RX1 -> TX1 and RX2 -> TX2 through
\`gnss_passthrough\` in the FPGA, with no anti-jam filter at all. Same board
support, block design, scripts and console as the CRPA builds. The build makes
one bitstream, one firmware and one \`BOOT.BIN\`. The commands are in
**[BUILD_COMMANDS_loopback.md](BUILD_COMMANDS_loopback.md)**.

| | loop-back |
|---|---|
| FPGA | axi_ad9361 (LVDS 2R2T) -> gnss_passthrough (elastic buffer 32 per channel, 12-bit right -> left alignment) -> axi_ad9361 DAC; RX/TX DMA unchanged |
| filter core / HLS | none (no Vitis HLS stage) |
| firmware | bare-metal (no-OS): AD9361 no-OS driver, console, RF profiles (\`gnss_band=\`), matched RX gain, TX interlock; no anti-jam engine |
| TX1 / TX2 | RX1 / RX2, sample for sample, delayed through the 32-deep buffer |
| gnss_passthrough | v7.0, CONTROL[5] always 0; AJ_INFO 0x00010007 (no filter) |
| BOOT.BIN | FSBL + bitstream + app_crpa_lvds.elf; no U-Boot, no Linux |

## Test results in this package (\`test/\`)

HDL testbenches (Icarus Verilog), the register map and the RX -> TX datapath:

$hdl_lines

- With TX requesting samples at the same rate as RX, every sample comes out
  in order, the buffer sits at 16 of 32, and there are no over- or underflows.
- With TX requesting 4x faster, the buffer runs empty and underflows are
  counted. That is what a TX/RX rate mismatch on the board looks like: check it
  with \`gnss_fifo?\`.

Not done here, because these need the Xilinx tools: Vivado implementation and
timing, the ARM build, any run on the board.

## Console (PuTTY: 115200 8N1, no flow control)

| command | |
|---|---|
| \`?\` / \`help?\` | menu / every command |
| \`gnss_status?\` | AD9361 and gnss_passthrough state: version 7.0, counters, buffer level |
| \`gnss_fifo?\` | RX and TX sample rates over 1 s, buffer level, under/overflows per second, verdict |
| \`gnss_dac_rate?\` / \`gnss_dac_rate=N\` | DAC rate control (boot value 3); only to match TX to RX, see gnss_fifo? |
| \`gnss_fifo_clear=1\` | clear the counters and sticky flags |
| \`tx1_attenuation=70000\` | TX1 attenuation in mdB (70 dB was measured working; bigger = quieter) |
| \`gnss_tx=1\` / \`gnss_tx=0\` | start / stop retransmitting (stop = abort) |
| \`gnss_band?\` / \`gnss_band=1\` | RF profile (1 = all GNSS, 1582.5 MHz, 61.44 MS/s, default) |
| \`gnss_rx_gain?\`, \`gnss_rx_gain=N\`, \`gnss_rx_agc=0/1/2\` | RX gain |
| \`gnss_ddr_tx=1/0\` | DDR capture-and-replay test on TX1 |

The \`crpa*\` commands are present but answer "loop-back build: no anti-jam".
EOF

# ---- commands -------------------------------------------------------------------------
cat > "$pkg/BUILD_COMMANDS_loopback.md" <<EOF
# Build commands for the loop-back: sources -> BOOT.BIN

\`./build_all.sh\` builds the loop-back; outputs go to \`Build/loopback/\`.
The result is **\`Build/loopback/boot/BOOT.BIN\`**. The firmware is
bare-metal (no-OS), not Linux.

## Step 1: Xilinx tools (once per terminal)

\`\`\`bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh
\`\`\`

\`\`\`bash
source /tools/Xilinx/Vitis/2023.2/settings64.sh
\`\`\`

(Vitis HLS isn't needed for the loop-back.)

## Step 2: ADI HDL tree

\`\`\`bash
export ADI_HDL_DIR=/home/sandeep/antsdr/antsdr-fw-patch/plutosdr-fw/hdl
\`\`\`

(Not needed once \`Vendor/adi_hdl\` exists, see step 9.)

## Step 3: unzip and go to the build folder

\`\`\`bash
cd ~/Downloads
\`\`\`

\`\`\`bash
unzip $name.zip
\`\`\`

\`\`\`bash
cd ~/Downloads/$name/Automation/Linux
\`\`\`

\`\`\`bash
chmod +x *.sh
\`\`\`

\`\`\`bash
mkdir -p ../../Build
\`\`\`

## Step 4: check (it must end with \`CHECK: PASS\`)

\`\`\`bash
./build_all.sh --only check 2>&1 | tee ../../Build/loopback_1_check.log
\`\`\`

## Step 5: IP, packaging gnss_passthrough (about 1-2 minutes; must end with \`IP: PASS\`)

\`\`\`bash
./build_all.sh --only ip 2>&1 | tee ../../Build/loopback_2_ip.log
\`\`\`

## Step 6: HDL, the Vivado bitstream (about 30-60 minutes; must end with \`HDL: PASS ... (timing met)\`)

\`\`\`bash
./build_all.sh --only hdl 2>&1 | tee ../../Build/loopback_3_hdl.log
\`\`\`

## Step 7: SW, the firmware (about 3-5 minutes; must end with \`SW: PASS\`)

\`\`\`bash
./build_all.sh --only sw 2>&1 | tee ../../Build/loopback_4_sw.log
\`\`\`

## Step 8: boot, building BOOT.BIN (must end with \`BOOT: PASS\`)

\`\`\`bash
./build_all.sh --only boot 2>&1 | tee ../../Build/loopback_5_boot.log
\`\`\`

\`\`\`bash
ls -l ~/Downloads/$name/Build/loopback/boot/BOOT.BIN
\`\`\`

Steps 4-8 in one command instead:

\`\`\`bash
./build_all.sh 2>&1 | tee ../../Build/build_loopback.log
\`\`\`

If a step fails, send the last lines of its log, for example:

\`\`\`bash
tail -40 ~/Downloads/$name/Build/loopback/logs/hdl.log
\`\`\`

## Step 9 (optional): one complete zip with your ADI HDL tree inside

\`\`\`bash
./bundle_full.sh /home/sandeep/antsdr/antsdr-fw-patch/plutosdr-fw/hdl
\`\`\`

This writes \`~/Downloads/${name}_full.zip\`.

## On the board (PuTTY 115200 8N1)

Copy \`BOOT.BIN\` to the root of a FAT32 SD card, set the boot jumpers to SD and
power up. The console shows \`=== ANTSDR E310 - Passthrough_LVDS LOOP-BACK (no anti-jam) ===\`. Then:

\`\`\`
gnss_status?
\`\`\`

\`\`\`
tx1_attenuation=70000
\`\`\`

\`\`\`
gnss_tx=1
\`\`\`

\`\`\`
gnss_fifo?
\`\`\`

\`gnss_fifo?\` must say \`VERDICT: OK\` (buffer about 16, 0 underflows/s). If it
says TX is faster or slower, type the \`gnss_dac_rate=N\` it suggests, then
run \`gnss_fifo?\` again. Stop retransmitting with \`gnss_tx=0\`.
EOF

rm -f "$out/$name.zip"
( cd "$stage" && zip -qr -X "$out/$name.zip" "$name" )
rm -rf "$stage"
echo "PACKAGE: $out/$name.zip ($(du -h "$out/$name.zip" | cut -f1))"
