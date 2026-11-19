#!/usr/bin/env bash
# check_all.sh - check every part of the SFAP build and print PASS / FAIL / INFO
#
# Usage (package root, after build_hw + build_fw):
#   ./scripts/check_all.sh [path to Pass_SFAP project] [path to Vitis 2023.2]
# defaults:
#   ~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_SFAP
#   /tools/Xilinx/Vitis/2023.2
# Writes the same text to check_report.txt.

PKG="$(cd "$(dirname "$0")/.." && pwd)"
PRJ="${1:-$HOME/antsdr/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_SFAP}"
VITIS="${2:-/tools/Xilinx/Vitis/2023.2}"
REPORT="$PKG/check_report.txt"
[ -f "$VITIS/settings64.sh" ] && source "$VITIS/settings64.sh" >/dev/null 2>&1

NP=0; NF=0
pass() { echo "  PASS  $*"; NP=$((NP+1)); }
fail() { echo "  FAIL  $*"; NF=$((NF+1)); }
info() { echo "  INFO  $*"; }
sec()  { echo; echo "=== $* ==="; }

run_checks() {
WS="$PKG/build/ws"
APPSRC="$WS/SFAP_LVDS_app/src"
FW="$PKG/source/Firmware/app_sfap_lvds"
XSA="$PKG/hw/PASS_SFAP.xsa"
ELF="$PKG/output/SFAP_LVDS_app.elf"
BOOT="$PKG/output/BOOT.bin"
echo "package : $PKG"
echo "project : $PRJ"
echo "date    : $(date)"

# ---------------------------------------------------------------- 1
sec "1. Hardware: timing, critical warnings, power, resources ($PRJ/reports)"
R="$PRJ/reports"
if [ -f "$R/1_timing_summary.txt" ]; then
  L=$(grep -A3 "WNS(ns)" "$R/1_timing_summary.txt" | sed -n 3p)
  WNS=$(echo "$L" | awk '{print $1}'); WHS=$(echo "$L" | awk '{print $5}')
  awk -v a="$WNS" 'BEGIN{exit !(a>=0)}' && pass "setup WNS = $WNS ns" || fail "setup WNS = $WNS ns (negative)"
  awk -v a="$WHS" 'BEGIN{exit !(a>=0)}' && pass "hold  WHS = $WHS ns" || fail "hold  WHS = $WHS ns (negative)"
  grep -q "All user specified timing constraints are met" "$R/1_timing_summary.txt" \
    && pass "all timing constraints met" || fail "timing summary does not say all constraints are met"
else fail "no $R/1_timing_summary.txt"; fi
for f in 5_methodology.txt 6_drc.txt; do
  if [ -f "$R/$f" ]; then
    n=$(grep -c "Critical Warning" "$R/$f")
    [ "$n" = 0 ] && pass "$f: 0 critical warnings" || fail "$f: $n critical warning lines"
  else fail "no $R/$f"; fi
done
if [ -f "$R/3_power_temperature.txt" ]; then
  P=$(grep "Total On-Chip Power" "$R/3_power_temperature.txt" | head -1 | awk -F'|' '{gsub(/ /,"",$3); print $3}')
  T=$(grep "Junction Temperature" "$R/3_power_temperature.txt" | head -1 | awk -F'|' '{gsub(/ /,"",$3); print $3}')
  info "power $P W, junction $T C (Vivado estimate: 25 C ambient, no heatsink)"
  awk -v t="$T" 'BEGIN{exit !(t<85)}' && pass "junction temperature below 85 C" || fail "junction temperature $T C >= 85 C"
fi
[ -f "$R/2_utilization.txt" ] && grep -E "^\| (Slice LUTs|Slice Registers|Block RAM Tile|DSPs) " "$R/2_utilization.txt" \
  | awk -F'|' '{gsub(/^ +| +$/,"",$2); gsub(/ /,"",$3); gsub(/ /,"",$6); gsub(/ /,"",$7); printf "  INFO  %-16s %7s of %7s (%s %%)\n",$2,$3,$6,$7}' | awk '!seen[$0]++'
[ -f "$PRJ/fix_timing.log" ] && grep -E "^BEST:" "$PRJ/fix_timing.log" | sed 's/^/  INFO  /'

# ---------------------------------------------------------------- 2
sec "2. XSA ($XSA)"
if [ -f "$XSA" ]; then
  pass "XSA present ($(stat -c%s "$XSA") bytes, $(date -r "$XSA" '+%F %T'))"
  if [ -f "$PRJ/PASS_SFAP.xsa" ]; then
    cmp -s "$XSA" "$PRJ/PASS_SFAP.xsa" && pass "same file as $PRJ/PASS_SFAP.xsa" || fail "differs from $PRJ/PASS_SFAP.xsa (copy the newest)"
  fi
  python3 - "$XSA" <<'EOF'
import sys, zipfile, re
z = zipfile.ZipFile(sys.argv[1]); names = z.namelist()
def p(ok, msg): print(("  PASS  " if ok else "  FAIL  ") + msg)
bits = [n for n in names if n.endswith(".bit")]
p(bool(bits), "bitstream inside: %s" % (", ".join(bits) or "none"))
hwh = [n for n in names if n.endswith(".hwh")]
t = z.read(hwh[0]).decode("utf-8", "replace") if hwh else ""
want = {"gnss_passthrough": 0x43C10000, "axi_ad9361": 0x79020000, "axi_ad9361_adc_dma": 0x7C400000,
        "axi_ad9361_dac_dma": 0x7C420000, "axi_vcxo_ctrl": 0x43C00000}
for inst, addr in want.items():
    got = None
    for tag in re.findall(r"<MEMRANGE\b[^>]*>", t):
        if 'INSTANCE="%s"' % inst in tag:
            m = re.search(r'BASEVALUE="(0[xX][0-9a-fA-F]+)"', tag)
            if m: got = int(m.group(1), 16); break
    p(got == addr, "%-20s at %s (expected 0x%08X)" % (inst, hex(got) if got is not None else "not found", addr))
p('MODTYPE="sfap_pi"' in t or 'sfap_pi_0' in t, "sfap_pi core in the design")
p('gnss_passthrough' in t, "gnss_passthrough in the design")
EOF
else fail "no XSA at $XSA"; fi

# ---------------------------------------------------------------- 3
sec "3. Platform BSP ($WS/PASS_SFAP)"
XP="$WS/PASS_SFAP/export/PASS_SFAP/sw/standalone_ps7_cortexa9_0/include/xparameters.h"
if [ -f "$XP" ]; then
  pass "xparameters.h present"
  g=$(grep -E "XPAR_GNSS_PASSTHROUGH(_0|_S_AXI)?_BASEADDR" "$XP")
  [ -n "$g" ] && info "xparameters.h: $(echo $g)" || info "xparameters.h has no gnss_passthrough entry (build_fw.py passes it on the compiler line)"
  grep -q "XPAR_CPU_CORE_CLOCK_FREQ_HZ" "$XP" && info "$(grep -m1 'XPAR_CPU_CORE_CLOCK_FREQ_HZ' "$XP")"
  [ -f "$WS/PASS_SFAP/export/PASS_SFAP/sw/boot/fsbl.elf" ] && pass "fsbl.elf built" || fail "no fsbl.elf"
else fail "no $XP (platform not built)"; fi

# ---------------------------------------------------------------- 4
sec "4. App build settings ($APPSRC)"
if [ -d "$APPSRC" ]; then
  U="$APPSRC/UserConfig.cmake"
  grep -q "XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ=XPAR_CPU_CORE_CLOCK_FREQ_HZ" "$U" && pass "CPU clock define in UserConfig.cmake" || fail "CPU clock define missing"
  if grep -q "XPAR_GNSS_PASSTHROUGH_BASEADDR=0x43C10000" "$U"; then pass "gnss_passthrough base 0x43C10000 passed to the compiler"
  elif grep -qE "XPAR_GNSS_PASSTHROUGH(_0|_S_AXI)?_BASEADDR\s+0x43[cC]10000" "$XP" 2>/dev/null; then pass "gnss_passthrough base 0x43C10000 from xparameters.h"
  else fail "gnss_passthrough base 0x43C10000 neither in UserConfig.cmake nor in xparameters.h"; fi
  if [ -f "$APPSRC/xtime_l.h" ] && grep -q "0xF8F00200" "$APPSRC/xtime_l.h"; then pass "xtime_l.h = 64-bit global timer (counts up)"; else fail "xtime_l.h missing or not the global-timer version"; fi
  grep -q "_STACK_SIZE : 0x10000" "$APPSRC/lscript.ld" && pass "stack 0x10000" || fail "stack not 0x10000"
  grep -q "_HEAP_SIZE : 0x10000"  "$APPSRC/lscript.ld" && pass "heap  0x10000" || fail "heap not 0x10000"
  n=$(ls "$APPSRC"/*.c 2>/dev/null | wc -l); [ "$n" = 28 ] && pass "28 C files imported" || fail "$n C files imported (expected 28)"
  diffs=0; for f in "$FW"/*.[ch]; do b=$(basename "$f"); cmp -s "$f" "$APPSRC/$b" || { diffs=1; info "differs from source: $b"; }; done
  [ $diffs = 0 ] && pass "workspace sources identical to source/Firmware/app_sfap_lvds"
else fail "no $APPSRC (firmware not built)"; fi
LOG=$(ls -t "$PKG"/fw_build.log 2>/dev/null | head -1)
if [ -n "$LOG" ]; then
  e=$(grep -cE "error:|fatal error|undefined reference" "$LOG"); [ "$e" = 0 ] && pass "fw_build.log: 0 errors" || fail "fw_build.log: $e error lines"
  grep -q "gnss_passthrough base address not found" "$LOG" && fail "build used the 0x44A00000 fallback" || pass "no 0x44A00000 fallback warning"
else info "no fw_build.log (run: ./scripts/build_fw.sh ... 2>&1 | tee fw_build.log)"; fi

# ---------------------------------------------------------------- 5
sec "5. Firmware content (source)"
grep -q 'GNSS_AJ_VERSION_STR  "6.3a"' "$FW/gnss_aj.h" && pass "version 6.3a" || fail "version string not 6.3a"
grep -q '1568500000ULL, 30720000U, 20000000U' "$FW/gnss_aj.c" && pass "gnss_band=7: 1568.5 MHz, 30.72 MS/s, 20 MHz" || fail "band 7 radio values"
grep -A1 '1568500000ULL, 30720000U, 20000000U' "$FW/gnss_aj.c" | grep -q '40, 30000' && pass "gnss_band=7: RX gain 40 dB, TX1 att 30000 mdB" || fail "band 7 gain/attenuation"
grep -q 'define GNSS_BAND_COUNT       9' "$FW/gnss_aj.h" && pass "9 profiles (0..8)" || fail "GNSS_BAND_COUNT"
grep -q '"gnss_band_manual="' "$FW/command.c" && pass "gnss_band_manual= command" || fail "gnss_band_manual= missing"
grep -q '^#define GNSS_AUTO_GNSS_TX 1' "$FW/gnss_build_defines.h" && pass "auto gnss_tx=1 at boot ON" || info "auto gnss_tx=1 at boot OFF"
grep -q 'GNSS_DEPLOY_TX_ATTEN_MDB         70000U' "$FW/gnss_l1.h" && pass "default TX1 start attenuation 70 dB" || fail "default TX1 start attenuation changed"
grep -q 'band_radio_set(band);' "$FW/gnss_aj.c" && pass "radio guard restores radio only (not gain/att)" || fail "radio guard"

# ---------------------------------------------------------------- 6
sec "6. ELF ($ELF)"
if [ -f "$ELF" ]; then
  pass "ELF present ($(date -r "$ELF" '+%F %T'))"
  command -v arm-none-eabi-size >/dev/null && arm-none-eabi-size "$ELF" | sed 's/^/  INFO  /'
  strings "$ELF" | grep -q "AUTO START: gnss_tx=1" && pass "ELF contains the auto-start" || info "ELF without auto-start"
  strings "$ELF" | grep -q "1568.5 MHz 20 MHz" && pass "ELF contains gnss_band=7" || fail "ELF without gnss_band=7"
  strings "$ELF" | grep -q "gnss_band_manual=" && pass "ELF contains gnss_band_manual=" || fail "ELF without gnss_band_manual="
  if command -v arm-none-eabi-nm >/dev/null; then
    for s in gnss_band_set_manual gnss_band_tx_att_mdb gnss_xtime_get main; do
      arm-none-eabi-nm "$ELF" | grep -qw "$s" && pass "symbol $s" || info "symbol $s not listed (may be inlined)"
    done
    E=$(arm-none-eabi-nm "$ELF" | awk '$3=="_end"{print $1}'); [ -n "$E" ] && info "end of program in DDR: 0x$E (must stay below 0x00800000, the ADC buffer)"
    [ -n "$E" ] && [ $((16#$E)) -lt $((0x00800000)) ] && pass "program + stack + heap below 0x00800000"
  fi
else fail "no ELF at $ELF"; fi

# ---------------------------------------------------------------- 7
sec "7. BOOT.bin ($BOOT)"
if [ -f "$BOOT" ]; then
  pass "BOOT.bin present ($(stat -c%s "$BOOT") bytes, $(date -r "$BOOT" '+%F %T'))"
  if [ -f "$ELF" ] && [ "$ELF" -nt "$BOOT" ]; then fail "BOOT.bin older than the ELF (run bootgen again)"; else pass "BOOT.bin not older than the ELF"; fi
  if [ -f "$PKG/output/boot.bif" ]; then
    grep -q "fsbl.elf" "$PKG/output/boot.bif" && grep -q "\.bit" "$PKG/output/boot.bif" && grep -q "SFAP_LVDS_app.elf" "$PKG/output/boot.bif" \
      && pass "boot.bif: fsbl + bitstream + app" || fail "boot.bif incomplete"
  fi
  command -v bootgen >/dev/null && bootgen -arch zynq -read "$BOOT" 2>/dev/null | grep -iE "partition|name|\.bit|\.elf" | head -12 | sed 's/^/  INFO  /'
else fail "no BOOT.bin at $BOOT"; fi

echo; echo "=== SUMMARY: $NP PASS, $NF FAIL ==="
}

run_checks 2>&1 | tee "$REPORT"
echo "(saved to $REPORT)"
