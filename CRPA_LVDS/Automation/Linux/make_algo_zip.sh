#!/usr/bin/env bash
# ============================================================================
#  make_algo_zip.sh  --  a self-contained package (zip) for ONE CRPA algorithm.
#
#    ./make_algo_zip.sh crpa28 [out_dir] [--run-tests]
#
#  The package CRPA_LVDS_<algo>.zip holds the whole pipeline (Automation,
#  Source, docs) with ONLY that algorithm's sources in Source/CRPA, the build
#  defaulting to it (./build_all.sh with no --algo builds it), its own
#  README_<algo>.md and BUILD_COMMANDS_<algo>.md, and test/ results for it.
#  --run-tests runs the package's own HDL testbench, C simulation and engine
#  host test from the unpacked copy (needs iverilog, g++ and the ap_int /
#  hls_stream headers: AP_INC=<dir> or a Vitis HLS install) and stores the
#  output in test/; without it, the algorithm's sections of the main
#  package's test/ files are copied.
# ============================================================================
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/../.." && pwd)"
source "$here/crpa_algos.sh"

algo="${1:-}"; out="${2:-$root/../CRPA_packages}"; run_tests=0
for a in "$@"; do [[ "$a" == --run-tests ]] && run_tests=1; done
[[ "$out" == --run-tests ]] && out="$root/../CRPA_packages"
crpa_algo_params "$algo" || { echo "usage: $0 <${CRPA_ALGOS[*]// /|}> [out_dir] [--run-tests]"; exit 2; }
mkdir -p "$out"; out="$(cd "$out" && pwd)"
name="CRPA_LVDS_$algo"
stage="$(mktemp -d)"
pkg="$stage/$name"

# ---- the tree, with only this algorithm --------------------------------------------
mkdir -p "$pkg"
( cd "$root" && tar --exclude=./Build --exclude=./test --exclude=./Source/CRPA --exclude=./Source/SFAP \
      --exclude=README_CRPA.md --exclude=BUILD_COMMANDS.md -cf - . ) | ( cd "$pkg" && tar -xf - )
mkdir -p "$pkg/Source/CRPA" "$pkg/test"
cp -r "$root/Source/CRPA/$algo" "$pkg/Source/CRPA/"
cp "$root/Source/CRPA/README.txt" "$pkg/Source/CRPA/"
rm -f "$pkg/Automation/Linux/make_algo_zip.sh"

# build defaults to this algorithm; the table keeps only it
sed -i "s/^CRPA_ALGOS=(.*)/CRPA_ALGOS=($algo)\nCRPA_DEFAULT_ALGO=$algo      # this package: .\/build_all.sh builds $algo/" \
    "$pkg/Automation/Linux/crpa_algos.sh"
find "$pkg" -name '*.sh' -exec chmod +x {} +

# ---- what this algorithm is ---------------------------------------------------------
if [[ "$CRPA_CORE" == smi_pi ]]; then
  solver="SMI power inversion: statistics of each 8192-sample capture averaged with the previous ones (adaptive), solved by Cholesky with diagonal loading from the canceller's own noise; slow-sweep detector (long memory for a slowly sweeping chirp); 4 tap modes (crpa_mode=)."
  nimg=1; nw=$((2 * CRPA_NT - 1 + 2))
  dsp=$((3 * nw + 2))
  cmds='| `crpa_mode=0..3` | tap set: 0 = RX1 + RX2 taps + image (default), 1 = RX2 taps + image, 2 = one weight, 3 = no image taps |'
  moving='crpa1 has **no weight prediction**: its weights are one capture old, so a jammer that keeps moving leaks (C-sim: 11 dB at 2 degrees per capture). crpa27..31 are built for that case.'
else
  solver="STAP-NLMS: a direct (Wiener, Cholesky) solve on the first capture of a jammer, then NLMS on every capture (2 passes, step from the J/N); moving-jammer tracking re-solves directly when the null is lost; a Kalman weight tracker (on at boot) filters the solved weights and commits them predicted half a hold interval ahead, so a moving jammer stays in the null."
  nimg=3; nw=$((2 * CRPA_NT - 1 + 6))
  dsp=$((3 * nw + 4))
  cmds='| `crpa_direct=0..8` | direct solves at the start of a jammer (default 1) |
| `crpa_track=1/0` | re-solve when a moving jammer leaves the null (default on) |
| `crpa_kalman=1/0` | Kalman weight tracker (default **on** at boot; 0 = off) |
| `crpa_predict=1/0` | simple weight prediction, used only while `crpa_kalman=0` (default on) |
| `crpa_passes=1..8` | NLMS passes over each capture (default 2) |'
  moving='Moving jammers: tracking and the Kalman tracker are on by default; `crpa_report?` shows the re-solves, the lead and a `Kalman tracker` line (steps, manoeuvres, NIS). `crpa_kalman=0` switches the Kalman tracker off.'
fi

# ---- test results ------------------------------------------------------------------------
if [[ $run_tests -eq 1 ]]; then
  ( cd "$pkg" && { echo "# Source/HDL/sim/run_iverilog.sh ($algo package, $(date -u +%F))"; Source/HDL/sim/run_iverilog.sh; } ) \
      > "$pkg/test/hdl_tb_results_$algo.txt" 2>&1
  ( cd "$pkg" && { echo "# Automation/Linux/csim_gcc.sh ($algo package, g++ $(g++ -dumpversion), $(date -u +%F))"; \
      CRPA_BUILD_ROOT="$stage/csim" Automation/Linux/csim_gcc.sh; } ) > "$pkg/test/csim_results_$algo.txt" 2>&1
  ( cd "$pkg" && { echo "# Source/Firmware/host_test/run_engine_test.sh -v ($algo package, gcc $(gcc -dumpversion), $(date -u +%F))"; \
      Source/Firmware/host_test/run_engine_test.sh "$algo" -v; } ) > "$pkg/test/engine_tb_results_$algo.txt" 2>&1
  for f in hdl_tb csim engine_tb; do
    grep -qE "ALL PASS|CSIM_GCC: PASS" "$pkg/test/${f}_results_$algo.txt" || { echo "$algo: $f FAILED, see $pkg/test"; exit 1; }
  done
else
  grep -E "crpa$CRPA_NUM[[:space:]]" "$root/test/hdl_tb_results_v7.0.txt" > "$pkg/test/hdl_tb_results_$algo.txt"
  awk -v a="==== $algo " 'index($0, "==== ") == 1 { on = (index($0, a) == 1) } on' \
      "$root/test/csim_results_v7.0.txt" > "$pkg/test/csim_results_$algo.txt"
  awk -v a="ENGINE_TB: $algo " 'index($0, "ENGINE_TB: crpa") == 1 && $0 ~ /taps, REF/ { on = (index($0, a) == 1) } on' \
      "$root/test/engine_tb_results_v7.0.txt" > "$pkg/test/engine_tb_results_$algo.txt"
fi
csim_lines=$(grep -E "^scenario" "$pkg/test/csim_results_$algo.txt" | sed 's/^/    /')
eng_lines=$(grep -E "^ENGINE_TB: (static|moving|PASS|FAIL|note)" "$pkg/test/engine_tb_results_$algo.txt" | sed 's/^/    /')
hdl_line=$(grep -E "HDL_TB: (PASS|FAIL)  $algo" "$pkg/test/hdl_tb_results_$algo.txt" | sed 's/^/    /')

# ---- README ---------------------------------------------------------------------------------
cat > "$pkg/README_$algo.md" <<EOF
# $name: Passthrough_LVDS + CRPA anti-jam, algorithm **$algo** (ANTSDR E310)

This package is the CRPA_LVDS v7.0 pipeline for **one** algorithm, $algo. It
builds one bitstream, one firmware and one \`BOOT.BIN\`, all for $algo. The
commands are in **[BUILD_COMMANDS_$algo.md](BUILD_COMMANDS_$algo.md)**.

| | $algo |
|---|---|
| FPGA core | \`$CRPA_CORE\` $CRPA_CORE_VER (Vitis HLS, II = 1 at 100 MHz), instance \`crpa_core_0\` |
| taps | $CRPA_NT on RX1 + $CRPA_NT on RX2, reference delay $CRPA_REF (RX1 tap fixed at 1), $nimg + $nimg image taps |
| adaptive weights | $nw complex weights, Q3.12 with a weight scale ws |
| DSP48 (core, estimate) | about $dsp of the Zynq-7020's 220 |
| RX -> TX filter delay | $CRPA_REF samples (the same in bypass) |
| AJ_INFO | \`$CRPA_AJ_INFO\` (the firmware refuses any other bitstream) |
| operating system | **none: bare-metal (no-OS)**. The Xilinx standalone BSP on Cortex-A9 core 0 with the Analog Devices no-OS AD9361 driver. \`BOOT.BIN\` = FSBL + bitstream + \`app_crpa_lvds.elf\`; no U-Boot, Linux kernel, device tree or root filesystem. |
| ARM solver | $solver |

The algorithm sources are \`Source/CRPA/$algo/fpga\` (FPGA core) and
\`Source/CRPA/$algo/arm\` (ARM solver), exactly as delivered. The build uses
them unchanged. Around them are \`gnss_passthrough\` v7.0 (weight registers),
the block design, the ARM engine \`app_crpa_lvds\` and the build scripts.

## Test results in this package (\`test/\`)

C simulation, the real core with the real ARM solver (\`Automation/Linux/csim_gcc.sh\`):

$csim_lines

ARM engine on the host, with simulated captures, registers and filter
(\`Source/Firmware/host_test/run_engine_test.sh\`):

$eng_lines

HDL register / commit testbench (\`Source/HDL/sim/run_iverilog.sh\`):

$hdl_line

Not done here, because these need the Xilinx tools:

- Vitis HLS synthesis (\`build_hls.tcl\` stops if II > 1 or the clock is over 10 ns);
- Vivado implementation and timing (\`build_all.sh\` stops if timing is not met);
- the ARM toolchain build;
- any run on the board.

## Console

| command | |
|---|---|
| \`crpa=1\` / \`crpa=0\` | anti-jam on (default) / off |
| \`crpa?\`, \`crpa_aj?\`, \`crpa_report?\` | state, status, everything |
| \`crpa_log=N\`, \`crpa_csv=N\` | status line every N s, CSV line every N ms |
| \`crpa_freeze=1/0\` | hold the filter as it is |
$cmds

$moving

Two antennas give one spatial null (plus its reflections), so two jammers from
two directions are beyond this algorithm.

## Register map (gnss_passthrough v7.0)

| offset | register | |
|---|---|---|
| 0x180 | AJ_COMMIT | a write hands every shadow register to the core at once; a read gives the commit count |
| 0x184 | AJ_INFO | $CRPA_AJ_INFO = {algorithm $CRPA_NUM, $CRPA_NT taps, REF $CRPA_REF, layout 7} |
| 0x188 | AJ_CFG | [0] bypass, [10:8] make-up shift, [14:12] weight scale |
| 0x18C | AJ_NORM | RX2 normalisation g2, Q3.13 |
| 0x190 | AJ_GAIN | fine output gain, Q2.14 (stap_pi only) |
| 0x198 | AJ_CFG_ACTIVE | AJ_CFG as committed |
| 0x19C | AJ_DIMS | {0, $CRPA_NWCJ image words, $CRPA_NW_USED weight words, $CRPA_NWTS port words} |
| 0x1000 + 4k | AJ_W[k] | weight k (k < $CRPA_NT: RX1 delay k, then RX2), shadow |
| 0x1100 + 4c | AJ_WC[c] | image tap c, shadow |
EOF

# ---- commands -------------------------------------------------------------------------------
cat > "$pkg/BUILD_COMMANDS_$algo.md" <<EOF
# Build commands for $algo: sources -> BOOT.BIN

This package builds **$algo** only. \`./build_all.sh\` with no \`--algo\`
builds it, and all outputs go to \`Build/$algo/\`. The result is
**\`Build/$algo/boot/BOOT.BIN\`**.
The firmware is **bare-metal (no-OS)**, not Linux: \`BOOT.BIN\` boots straight
into FSBL, then the bitstream, then \`app_crpa_lvds.elf\`.

## 1. Unzip and set up the tools (once per shell)

\`\`\`bash
unzip $name.zip
\`\`\`

\`\`\`bash
cd $name/Automation/Linux
\`\`\`

\`\`\`bash
chmod +x *.sh ../../Source/HDL/sim/run_iverilog.sh ../../Source/Firmware/host_test/run_engine_test.sh
\`\`\`

\`\`\`bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh
\`\`\`

\`\`\`bash
source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh
\`\`\`

\`\`\`bash
source /tools/Xilinx/Vitis/2023.2/settings64.sh
\`\`\`

If your ADI hdl tree (the one with \`projects/passthrough_lvds\`) is not the
default in \`env.sh\`, point to it:

\`\`\`bash
export ADI_HDL_DIR=\$HOME/CRPA_Project/NEW_PASSTHROUGH/workspace/antsdr-fw-patch/plutosdr-fw/hdl
\`\`\`

This package has a \`Vendor/\` folder:

- \`Vendor/hls_headers\`: open-source ap_int headers for \`csim_gcc.sh\`, included.
- \`Vendor/adi_hdl\`: your ADI HDL tree. \`bundle_full.sh\` adds it, see
  section 7. Once it's there, the \`export ADI_HDL_DIR\` line above isn't
  needed.

## 2. Check

\`\`\`bash
./build_all.sh --only check
\`\`\`

## 3. Build to BOOT.BIN (all stages, roughly 40-100 minutes)

\`\`\`bash
mkdir -p ../../Build
\`\`\`

\`\`\`bash
./build_all.sh 2>&1 | tee ../../Build/build_$algo.log
\`\`\`

The result is \`Build/$algo/boot/BOOT.BIN\`, with its sha256 in
\`Build/$algo/boot/BOOT.BIN.sha256\`.

## 4. The same, stage by stage

Each stage logs to \`Build/$algo/logs/<stage>.log\`.

\`\`\`bash
./build_all.sh --only hls     # Vitis HLS: C-sim (6 scenarios) + synthesis + IP $CRPA_CORE $CRPA_CORE_VER
\`\`\`

\`\`\`bash
./build_all.sh --only ip      # package gnss_passthrough 4.0 with $algo's sizes
\`\`\`

\`\`\`bash
./build_all.sh --only hdl     # Vivado project passthrough_lvds_$algo: bitstream + XSA (timing must be met)
\`\`\`

\`\`\`bash
./build_all.sh --only sw      # firmware + $algo solver -> FSBL + app_crpa_lvds.elf
\`\`\`

\`\`\`bash
./build_all.sh --only boot    # bootgen -> Build/$algo/boot/BOOT.BIN
\`\`\`

To restart from one stage and run everything after it:

\`\`\`bash
./build_all.sh --from hdl
\`\`\`

## 5. Optional checks, no Xilinx tools needed (gcc, iverilog)

\`\`\`bash
../../Source/HDL/sim/run_iverilog.sh
\`\`\`

\`\`\`bash
../../Source/Firmware/host_test/run_engine_test.sh $algo
\`\`\`

\`\`\`bash
./csim_gcc.sh
\`\`\`

The last one needs the ap_int / hls_stream headers: a Vitis HLS install, or
\`AP_INC=<dir>\`.

## 6. On the board

Copy \`Build/$algo/boot/BOOT.BIN\` to the root of a FAT32 SD card, set the boot
jumpers to SD and power up. On the console, check:

- \`gnss_status?\` shows version 7.0 and AJ_INFO $CRPA_AJ_INFO;
- \`crpa_report?\` shows "CRPA $algo ($CRPA_CORE)".

Or run it over JTAG straight after the build:

\`\`\`bash
./build_all.sh --from boot --jtag
\`\`\`

## 7. One complete zip, including your ADI HDL tree (run on your PC)

This copies your ADI HDL tree (\`scripts/\`, \`projects/scripts\`,
\`library/\` with all the IPs, including \`axi_vcxo_ctrl\`) into
\`Vendor/adi_hdl\` and writes \`${name}_full.zip\` next to the package folder.
On another PC that zip then needs only Vivado / Vitis / Vitis HLS 2023.2.

\`\`\`bash
cd ~/Downloads/$name/Automation/Linux
\`\`\`

\`\`\`bash
./bundle_full.sh /home/sandeep/antsdr/antsdr-fw-patch/plutosdr-fw/hdl
\`\`\`

The Xilinx tools themselves can't go in the zip: they're licensed and many GB
in size.
EOF

# ---- zip ------------------------------------------------------------------------------------
rm -f "$out/$name.zip"
( cd "$stage" && zip -qr -X "$out/$name.zip" "$name" )
rm -rf "$stage"
echo "PACKAGE: $out/$name.zip ($(du -h "$out/$name.zip" | cut -f1))"
