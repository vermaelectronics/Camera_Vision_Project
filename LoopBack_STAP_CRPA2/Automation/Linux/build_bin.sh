#!/usr/bin/env bash
# ============================================================================
#  build_bin.sh  --  LoopBack_STAP_CRPA2: one command from sources to .bin
#
#  Produces, for the ANTSDR E310 V1 (xc7z020clg400-2):
#    BOOT.BIN             FSBL + bitstream + application, for the SD card
#    system_top.bit.bin   the bitstream alone in .bin form (Linux FPGA manager,
#                         QSPI tools, or loading the PL from a running system)
#  plus the .bit, .xsa, ELFs, a manifest and SHA256SUMS, in
#    Build/Release/LoopBack_STAP_CRPA_<variant>_<timestamp>/
#
#  STEPS
#    0 preflight  project layout, Xilinx 2023.2 tools (sources settings64.sh
#                 itself if they are not on PATH), disk space
#    1 sim        RTL simulation of the CRPA datapath (Automation/Linux/
#                 run_sim.sh) -- minutes, catches RTL breakage before the
#                 hour-long Vivado run. Skipped with a warning if gcc/python3/
#                 numpy/iverilog are missing.
#    2 build      Automation/Linux/build_all.sh stages 1..8 (vendor, libip, ip,
#                 project, build, sw, fsbl, boot)
#    3 checks     timing MET; the XSA contains gnss_stap at 0x43C10000 and
#                 gnss_passthrough at 0x43C00000; the ELF contains the CRPA
#                 driver; the ELF is the variant that was asked for (a
#                 development build must NOT auto-transmit)
#    4 bit.bin    bootgen -process_bitstream bin
#    5 release    copy, manifest, checksums (and SD_Image/BOOT.BIN with --sd)
#
#  USAGE
#    ./Automation/Linux/build_bin.sh [options]
#
#  OPTIONS
#    --variant V      dev            JTAG/console build. TX silent at power-on;
#                                    gnss_tx=1, stap_update=1, stap_en=1 by hand.
#                                    (default)
#                     deploy         SD-card build: TX1 starts at power-on with
#                                    the CRPA anti-jam ON (GNSS_DEPLOY_AUTO_TX).
#                                    READ SD_Image/README_SD_CARD.md FIRST.
#                     deploy-nocrpa  SD-card build, plain loopback (CRPA bypass)
#    --defines LIST   extra comma-separated firmware defines
#    --jobs N         parallel Vivado jobs (default: number of CPUs, max 16)
#    --xilinx DIR     Xilinx install root holding Vivado/2023.2 and Vitis/2023.2
#                     (default: auto-detect /tools/Xilinx, /opt/Xilinx, ~/Xilinx)
#    --from STAGE     resume build_all.sh at a stage (vendor libip ip project
#                     build sw fsbl boot, or 1..8), e.g. --from sw after a
#                     firmware-only change
#    --clean          delete Build/VendorWork, Build/ip_repo and Build/Output first
#    --skip-sim       do not run the RTL simulation
#    --allow-timing-violation   release even if BUILD_TIMING is VIOLATED
#    --sd             also copy BOOT.BIN to SD_Image/BOOT.BIN
#    -h, --help
#
#  EXIT STATUS: 0 and "BUILD_BIN: PASS" on success, non-zero otherwise.
# ============================================================================

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET_VERSION="2023.2"
PROJECT_NAME="LoopBack_STAP_CRPA2"

VARIANT="dev"
EXTRA_DEFINES=""
JOBS=""
XILINX_ROOT=""
FROM="1"
CLEAN=0
SKIP_SIM=0
ALLOW_TIMING=0
COPY_SD=0

usage() { sed -n '2,52p' "$0"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --variant)  VARIANT="$2"; shift 2 ;;
    --defines)  EXTRA_DEFINES="$2"; shift 2 ;;
    --jobs)     JOBS="$2"; shift 2 ;;
    --xilinx)   XILINX_ROOT="$2"; shift 2 ;;
    --from)     FROM="$2"; shift 2 ;;
    --clean)    CLEAN=1; shift ;;
    --skip-sim) SKIP_SIM=1; shift ;;
    --allow-timing-violation) ALLOW_TIMING=1; shift ;;
    --sd)       COPY_SD=1; shift ;;
    -h|--help)  usage; exit 0 ;;
    *) echo "build_bin: unknown option '$1' (see --help)" >&2; exit 2 ;;
  esac
done

LOG_DIR="$ROOT/Build/Logs"
OUT_DIR="$ROOT/Build/Output"
ELF_DIR="$ROOT/Build/ELF"
BIT="$OUT_DIR/system_top.bit"
XSA="$OUT_DIR/system.xsa"
BOOT_BIN="$OUT_DIR/BOOT.BIN"
BIF="$OUT_DIR/boot.bif"
APP_ELF="$ELF_DIR/e310_gnss_app.elf"
FSBL_ELF="$ELF_DIR/fsbl.elf"
mkdir -p "$LOG_DIR"
MAIN_LOG="$LOG_DIR/build_bin.log"
: > "$MAIN_LOG"

say()  { echo "BUILD_BIN: $*" | tee -a "$MAIN_LOG"; }
warn() { echo "BUILD_BIN: WARNING - $*" | tee -a "$MAIN_LOG"; }
die()  { echo "BUILD_BIN: FAIL - $*" | tee -a "$MAIN_LOG" >&2; exit 2; }
step() { echo | tee -a "$MAIN_LOG"; say "==== $* ===="; }

# ---- variant -> firmware defines --------------------------------------------
case "$VARIANT" in
  dev)           DEFINES="" ;;
  deploy)        DEFINES="GNSS_DEPLOY_AUTO_TX" ;;
  deploy-nocrpa) DEFINES="GNSS_DEPLOY_AUTO_TX,GNSS_DEPLOY_NO_CRPA" ;;
  *) die "--variant must be dev, deploy or deploy-nocrpa (got '$VARIANT')" ;;
esac
if [[ -n "$EXTRA_DEFINES" ]]; then
  DEFINES="${DEFINES:+$DEFINES,}$EXTRA_DEFINES"
fi
if [[ "$VARIANT" == "dev" && ",$DEFINES," == *",GNSS_DEPLOY_AUTO_TX,"* ]]; then
  die "variant 'dev' must not carry GNSS_DEPLOY_AUTO_TX; use --variant deploy"
fi

case "$FROM" in
  1|2|3|4|5|6|7|8|vendor|libip|ip|project|build|sw|fsbl|boot) ;;
  *) die "--from must be vendor, libip, ip, project, build, sw, fsbl, boot or 1..8 (got '$FROM')" ;;
esac

if [[ -z "$JOBS" ]]; then
  JOBS="$(nproc 2>/dev/null || echo 4)"
  [[ "$JOBS" -gt 16 ]] && JOBS=16
fi

# =============================================================================
step "0 preflight"
# =============================================================================
for f in Vendor/ADI_hdl_2023_r2/library Source/BlockDesign/system_bd.tcl \
         Source/HDL/gnss_stap.v Source/HDL/gnss_passthrough.v \
         Source/HDL/stap/stap_top.v Source/HDL/stap/axil_cdc.v \
         Source/IP/gnss_stap/gnss_stap_ip.tcl Source/XDC/gnss_stap_cdc.xdc \
         Source/Firmware/app_gnss_e310/gnss_stap.c \
         Source/Firmware/app_gnss_e310/stap_solver.c \
         Automation/Linux/build_all.sh; do
  [[ -e "$ROOT/$f" ]] || die "missing $f - is this the complete $PROJECT_NAME tree?"
done
say "project root $ROOT"
say "variant $VARIANT, firmware defines: ${DEFINES:-(none)}, jobs $JOBS"

# Xilinx tools: use what is on PATH, otherwise find and source settings64.sh
have() { command -v "$1" >/dev/null 2>&1; }
source_settings() {
  local cands=()
  if [[ -n "$XILINX_ROOT" ]]; then cands+=("$XILINX_ROOT")
  else cands+=(/tools/Xilinx /opt/Xilinx "$HOME/Xilinx" /tools/xilinx /opt/xilinx); fi
  local r
  for r in "${cands[@]}"; do
    if [[ -f "$r/Vivado/$TARGET_VERSION/settings64.sh" ]]; then
      say "sourcing $r/Vivado/$TARGET_VERSION/settings64.sh"
      set +u   # the Xilinx settings scripts use unset variables
      # shellcheck disable=SC1090
      source "$r/Vivado/$TARGET_VERSION/settings64.sh"
      if [[ -f "$r/Vitis/$TARGET_VERSION/settings64.sh" ]]; then
        say "sourcing $r/Vitis/$TARGET_VERSION/settings64.sh"
        # shellcheck disable=SC1090
        source "$r/Vitis/$TARGET_VERSION/settings64.sh"
      fi
      set -u
      return 0
    fi
  done
  return 1
}
if ! have vivado || ! have xsct || ! have bootgen; then
  source_settings || true
fi
for t in vivado xsct bootgen make; do
  have "$t" || die "'$t' not found. Install Vivado + Vitis $TARGET_VERSION and either source their settings64.sh or pass --xilinx <install root>."
done
VIV_VER="$(vivado -version 2>/dev/null | grep -oE 'v[0-9]{4}\.[0-9]' | head -1 | tr -d v || true)"
say "vivado $(command -v vivado) (v${VIV_VER:-unknown})"
say "xsct   $(command -v xsct)"
say "bootgen $(command -v bootgen)"
if [[ "$VIV_VER" != "$TARGET_VERSION" ]]; then
  warn "Vivado $VIV_VER found; this project and Vendor/ADI_hdl_2023_r2 target $TARGET_VERSION"
fi

AVAIL_GB="$(df -Pk "$ROOT" | awk 'NR==2 {print int($4/1048576)}')"
if [[ "${AVAIL_GB:-0}" -lt 15 ]]; then
  warn "only ${AVAIL_GB} GB free under $ROOT; a full build needs about 15 GB"
fi
MEM_GB="$(awk '/MemTotal/ {print int($2/1048576)}' /proc/meminfo 2>/dev/null || echo 0)"
if [[ "${MEM_GB:-0}" -lt 8 ]]; then
  warn "${MEM_GB} GB RAM; Vivado implementation of a Zynq-7020 wants 8 GB or more (use --jobs 2)"
fi

if [[ $CLEAN -eq 1 ]]; then
  say "--clean: removing Build/VendorWork Build/ip_repo Build/Output Build/ELF"
  rm -rf "$ROOT/Build/VendorWork" "$ROOT/Build/ip_repo" "$OUT_DIR" "$ELF_DIR"
fi

# =============================================================================
step "1 RTL simulation (CRPA datapath)"
# =============================================================================
if [[ $SKIP_SIM -eq 1 ]]; then
  warn "simulation skipped (--skip-sim)"
elif have gcc && have python3 && have iverilog && python3 -c "import numpy" 2>/dev/null; then
  if "$ROOT/Automation/Linux/run_sim.sh" > "$LOG_DIR/sim.log" 2>&1 && grep -q "RUN_SIM: PASS" "$LOG_DIR/sim.log"; then
    grep -E "^[A-E]: " "$LOG_DIR/sim.log" | sed 's/^/BUILD_BIN:   /' | tee -a "$MAIN_LOG"
    say "simulation PASS"
  else
    tail -20 "$LOG_DIR/sim.log" >&2
    die "RTL simulation failed (see $LOG_DIR/sim.log); not starting Vivado"
  fi
else
  warn "gcc/python3+numpy/iverilog not all present - simulation skipped (apt install iverilog python3-numpy)"
fi

# =============================================================================
step "2 build_all.sh --from $FROM --to boot"
# =============================================================================
T0=$(date +%s)
BA_ARGS=(--from "$FROM" --to boot --jobs "$JOBS")
[[ -n "$DEFINES" ]] && BA_ARGS+=(--defines "$DEFINES")
set +e
"$ROOT/Automation/Linux/build_all.sh" "${BA_ARGS[@]}" 2>&1 | tee -a "$MAIN_LOG"
RC=${PIPESTATUS[0]}
set -e
[[ $RC -eq 0 ]] || die "build_all.sh failed (exit $RC). Per-stage logs: $LOG_DIR/*.log"
for f in "$BIT" "$XSA" "$APP_ELF" "$FSBL_ELF" "$BOOT_BIN"; do
  [[ -s "$f" ]] || die "expected output missing or empty: $f"
done
say "build_all.sh finished in $(( ($(date +%s) - T0) / 60 )) min"

# =============================================================================
step "3 output checks"
# =============================================================================
# 3a timing
TIMING="unknown"; WNS="n/a"; WHS="n/a"
if [[ -f "$LOG_DIR/build.log" ]]; then
  TIMING="$(grep -oE 'BUILD_TIMING: [A-Z]+' "$LOG_DIR/build.log" | tail -1 | awk '{print $2}')"
  WNS="$(grep -oE 'BUILD_WNS: [-0-9.]+' "$LOG_DIR/build.log" | tail -1 | awk '{print $2}')"
  WHS="$(grep -oE 'BUILD_WHS: [-0-9.]+' "$LOG_DIR/build.log" | tail -1 | awk '{print $2}')"
fi
say "timing: ${TIMING:-unknown} (WNS ${WNS:-n/a} ns, WHS ${WHS:-n/a} ns)"
if [[ "$TIMING" != "MET" ]]; then
  if [[ $ALLOW_TIMING -eq 1 ]]; then
    warn "timing is ${TIMING:-unknown}; releasing anyway because of --allow-timing-violation. Do NOT trust this bitstream on the bench."
  else
    die "timing is ${TIMING:-unknown} (see Vivado/Project/*.runs/impl_1). Fix it, or pass --allow-timing-violation to package anyway."
  fi
fi

# 3b hardware handoff contains both custom IPs at the expected addresses
HWH_TXT="$(python3 - "$XSA" <<'EOF' 2>/dev/null || unzip -p "$XSA" '*.hwh' 2>/dev/null || true
import sys, zipfile
z = zipfile.ZipFile(sys.argv[1])
for n in z.namelist():
    if n.endswith(".hwh"):
        sys.stdout.write(z.read(n).decode("utf-8", "replace"))
EOF
)"
if [[ -z "$HWH_TXT" ]]; then
  warn "could not read the .hwh inside $XSA (need python3 or unzip); IP presence not checked"
else
  grep -q 'MODTYPE="gnss_stap"'        <<<"$HWH_TXT" || die "the XSA has no gnss_stap instance - the CRPA stage is not in the bitstream"
  grep -q 'MODTYPE="gnss_passthrough"' <<<"$HWH_TXT" || die "the XSA has no gnss_passthrough instance"
  grep -qi 'BASEVALUE="0x43C10000"'    <<<"$HWH_TXT" || warn "0x43C10000 not found in the XSA address map; check gnss_stap's address"
  say "XSA: gnss_stap and gnss_passthrough present"
fi

# 3c application ELF: CRPA driver present, variant as requested
grep -aq "gnss_stap: CRPA stage present" "$APP_ELF" || die "the application ELF does not contain the CRPA driver (gnss_stap.c)"
if grep -aq "LOOPBACK DEPLOYMENT BUILD" "$APP_ELF"; then ELF_DEPLOY=1; else ELF_DEPLOY=0; fi
if [[ "$VARIANT" == "dev" && $ELF_DEPLOY -eq 1 ]]; then
  die "SAFETY: a development build was requested but the ELF auto-transmits (GNSS_DEPLOY_AUTO_TX compiled in). Rebuild from the sw stage."
fi
if [[ "$VARIANT" != "dev" && $ELF_DEPLOY -eq 0 ]]; then
  die "a deployment build was requested but GNSS_DEPLOY_AUTO_TX did not reach the firmware"
fi
if [[ "$VARIANT" == "deploy" ]] && ! grep -aq "CRPA anti-jam ON" "$APP_ELF"; then
  die "deploy variant: the CRPA auto-start is missing from the ELF"
fi
if [[ "$VARIANT" == "deploy-nocrpa" ]] && grep -aq "CRPA anti-jam ON" "$APP_ELF"; then
  die "deploy-nocrpa variant: GNSS_DEPLOY_NO_CRPA did not reach the firmware"
fi
say "ELF: CRPA driver present, variant '$VARIANT' confirmed"

# 3d BOOT.BIN contents as listed in the BIF
for f in "$FSBL_ELF" "$BIT" "$APP_ELF"; do
  grep -qF "$f" "$BIF" || die "boot.bif does not list $f"
done
say "BOOT.BIN: FSBL + bitstream + application ($(stat -c %s "$BOOT_BIN") bytes)"

# =============================================================================
step "4 bitstream .bin"
# =============================================================================
STAMP="$(date +%Y%m%d_%H%M%S)"
REL="$ROOT/Build/Release/${PROJECT_NAME}_${VARIANT}_${STAMP}"
mkdir -p "$REL"
cp "$BIT" "$REL/system_top.bit"
cat > "$REL/bitstream.bif" <<'EOF'
all:
{
  system_top.bit
}
EOF
( cd "$REL" && bootgen -arch zynq -image bitstream.bif -process_bitstream bin -w on ) \
  > "$LOG_DIR/bitbin.log" 2>&1 || { cat "$LOG_DIR/bitbin.log" >&2; die "bootgen -process_bitstream bin failed"; }
[[ -s "$REL/system_top.bit.bin" ]] || die "bootgen did not write system_top.bit.bin"
say "system_top.bit.bin ($(stat -c %s "$REL/system_top.bit.bin") bytes)"

# =============================================================================
step "5 release"
# =============================================================================
cp "$BOOT_BIN" "$REL/BOOT.BIN"
cp "$XSA" "$REL/system.xsa"
cp "$APP_ELF" "$REL/e310_gnss_app.elf"
cp "$FSBL_ELF" "$REL/fsbl.elf"
cp "$BIF" "$REL/boot.bif"
mkdir -p "$REL/logs"
cp "$LOG_DIR"/*.log "$REL/logs/" 2>/dev/null || true

GIT_REV="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo 'not a git checkout')"
GIT_DIRTY="$(git -C "$ROOT" status --porcelain 2>/dev/null | grep -v '^??' | head -1 || true)"
{
  echo "$PROJECT_NAME release"
  echo "built        : $(date -Iseconds) on $(hostname)"
  echo "variant      : $VARIANT"
  echo "defines      : ${DEFINES:-(none)}"
  echo "source rev   : $GIT_REV${GIT_DIRTY:+ (with uncommitted changes)}"
  echo "vivado       : ${VIV_VER:-unknown}"
  echo "timing       : ${TIMING:-unknown}  WNS ${WNS:-n/a} ns  WHS ${WHS:-n/a} ns"
  echo "target       : ANTSDR E310 V1, xc7z020clg400-2"
  echo
  echo "BOOT.BIN            SD card (FAT32, root). FSBL + bitstream + application."
  echo "system_top.bit.bin  bitstream only, .bin form (fpga manager / QSPI tools)."
  echo "system_top.bit      bitstream for JTAG (Vivado hardware manager)."
  echo "system.xsa          hardware platform for Vitis."
  echo "e310_gnss_app.elf   application (JTAG: run after fsbl.elf or ps7_init)."
  echo
  case "$VARIANT" in
    dev)    echo "DEV BUILD: TX silent at power-on. Console 115200 8N1:"
            echo "  gnss_tx=1, stap_update=1, stap_en=1, stap_auto=100, stap_status?" ;;
    deploy) echo "DEPLOY BUILD: TX1 TRANSMITS AT POWER-ON, CRPA anti-jam ON."
            echo "  CONDUCTED, ATTENUATED COAX ONLY. Read SD_Image/README_SD_CARD.md." ;;
    deploy-nocrpa)
            echo "DEPLOY BUILD (no CRPA): TX1 TRANSMITS AT POWER-ON, plain loopback."
            echo "  CONDUCTED, ATTENUATED COAX ONLY. Read SD_Image/README_SD_CARD.md." ;;
  esac
} > "$REL/MANIFEST.txt"
( cd "$REL" && sha256sum BOOT.BIN system_top.bit.bin system_top.bit system.xsa \
                          e310_gnss_app.elf fsbl.elf > SHA256SUMS )

if [[ $COPY_SD -eq 1 ]]; then
  cp "$REL/BOOT.BIN" "$ROOT/SD_Image/BOOT.BIN"
  say "copied BOOT.BIN to SD_Image/BOOT.BIN"
  if [[ "$VARIANT" != "dev" ]]; then
    warn "SD_Image/BOOT.BIN is a DEPLOYMENT image: the board transmits on L1 at power-on"
  fi
fi

echo | tee -a "$MAIN_LOG"
say "release: $REL"
sed 's/^/BUILD_BIN:   /' "$REL/SHA256SUMS" | tee -a "$MAIN_LOG"
say "PASS"
