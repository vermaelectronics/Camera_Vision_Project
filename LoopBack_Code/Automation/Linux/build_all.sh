#!/usr/bin/env bash
# ============================================================================
#  build_all.sh  --  full Linux build for Vivado / Vitis 2023.2
#
#  Replaces Automation/PowerShell/Build_All.ps1 on Ubuntu.  Runs the same
#  stages, in order, and stops at the first failure:
#
#    1 vendor   copy Vendor/ADI_hdl_2023_r2 to Build/VendorWork/hdl (disposable)
#    2 libip    package the Analog Devices library IP the block design uses
#    3 nlms     Vitis HLS: Source/HLS/pi_nlms -> Source/HDL/pi_nlms/*.v
#    4 ip       package the custom gnss_passthrough IP into Build/ip_repo
#    5 project  recreate Vivado/Project/antsdr_e310_gnss.xpr
#    6 build    synthesis, implementation, bitstream, XSA
#    7 sw       bare-metal application ELF (XSCT, classic BSP)
#    8 fsbl     Zynq FSBL ELF (XSCT)
#    9 boot     BOOT.BIN = FSBL + bitstream + application (bootgen)
#
#  USAGE
#    source /tools/Xilinx/Vivado/2023.2/settings64.sh
#    source /tools/Xilinx/Vitis/2023.2/settings64.sh
#    source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh   (stage 3)
#    ./Automation/Linux/build_all.sh [options]
#
#  OPTIONS
#    --from STAGE      start at this stage (name or number), default 1
#    --to STAGE        stop after this stage, default 9
#    --jobs N          parallel Vivado jobs, default 8
#    --defines LIST    comma-separated firmware build-variant defines,
#                      e.g. --defines GNSS_DEPLOY_AUTO_TX for the SD-card build
#    --clean           delete Build/VendorWork first (forces IP re-packaging)
#
#  OUTPUTS
#    Build/Output/system_top.bit, Build/Output/system.xsa, Build/Output/BOOT.BIN
#    Build/ELF/e310_gnss_app.elf, Build/ELF/fsbl.elf
#    Build/Logs/<stage>.log
# ============================================================================

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET_VERSION="2023.2"

FROM=1
TO=9
JOBS=8
DEFINES=""
CLEAN=0

stage_num() {
  case "$1" in
    1|vendor) echo 1 ;; 2|libip) echo 2 ;; 3|nlms) echo 3 ;; 4|ip) echo 4 ;;
    5|project) echo 5 ;; 6|build) echo 6 ;; 7|sw) echo 7 ;; 8|fsbl) echo 8 ;;
    9|boot) echo 9 ;;
    *) echo "unknown stage: $1" >&2; exit 2 ;;
  esac
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --from)    FROM="$(stage_num "$2")"; shift 2 ;;
    --to)      TO="$(stage_num "$2")"; shift 2 ;;
    --jobs)    JOBS="$2"; shift 2 ;;
    --defines) DEFINES="$2"; shift 2 ;;
    --clean)   CLEAN=1; shift ;;
    -h|--help) sed -n '2,40p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

VENDOR_SRC="$ROOT/Vendor/ADI_hdl_2023_r2"
ADI_WORK="$ROOT/Build/VendorWork/hdl"
IP_REPO="$ROOT/Build/ip_repo"
PROJ_DIR="$ROOT/Vivado/Project"
XPR="$PROJ_DIR/antsdr_e310_gnss.xpr"
OUT_DIR="$ROOT/Build/Output"
ELF_DIR="$ROOT/Build/ELF"
LOG_DIR="$ROOT/Build/Logs"
BIT="$OUT_DIR/system_top.bit"
XSA="$OUT_DIR/system.xsa"
APP_ELF="$ELF_DIR/e310_gnss_app.elf"
FSBL_ELF="$ELF_DIR/fsbl.elf"
BOOT_BIN="$OUT_DIR/BOOT.BIN"

# Library IP used by Source/BlockDesign/system_bd.tcl (same list as the ADI
# fmcomms2/zed reference project, minus the HDMI/audio cores this board lacks).
ADI_LIBS=(
  axi_ad9361
  axi_dmac
  axi_gpreg
  axi_sysid
  sysid_rom
  util_pack/util_cpack2
  util_pack/util_upack2
  util_rfifo
  util_wfifo
  util_tdd_sync
  xilinx/util_clkdiv
)

mkdir -p "$LOG_DIR" "$OUT_DIR" "$ELF_DIR"

export GNSS_CRPA_ROOT="$ROOT"
export GNSS_CRPA_IP_REPO="$IP_REPO"
export ADI_HDL_DIR="$ADI_WORK"
export REQUIRED_VIVADO_VERSION="$TARGET_VERSION"
# Synthesise the block design globally instead of ADI's default per-IP
# out-of-context runs with a shared IP cache. With OOC on, a stale or
# interrupted cache left axi_ad9361 and other cores as black boxes at
# opt_design (DRC INBB-3). Global synthesis is slower but self-contained.
# Set ADI_USE_OOC_SYNTHESIS=y before running to get the ADI default back.
export ADI_USE_OOC_SYNTHESIS="${ADI_USE_OOC_SYNTHESIS:-n}"

die()  { echo "BUILD_ALL: FAIL - $*" >&2; exit 2; }
want() { [[ $1 -ge $FROM && $1 -le $TO ]]; }
need() { command -v "$1" >/dev/null 2>&1 || die "'$1' not found on PATH. Run: source /tools/Xilinx/Vivado/$TARGET_VERSION/settings64.sh; source /tools/Xilinx/Vitis/$TARGET_VERSION/settings64.sh; source /tools/Xilinx/Vitis_HLS/$TARGET_VERSION/settings64.sh"; }

# Run a command, tee it to a log, and require the given PASS marker in it.
run_logged() {
  local name="$1" marker="$2"; shift 2
  local log="$LOG_DIR/$name.log"
  echo "BUILD_ALL: [$name] $*"
  set +e
  "$@" 2>&1 | tee "$log"
  local rc=${PIPESTATUS[0]}
  set -e
  if [[ $rc -ne 0 ]]; then die "[$name] exited with $rc (see $log)"; fi
  if [[ -n "$marker" ]] && ! grep -q "$marker" "$log"; then
    die "[$name] did not report '$marker' (see $log)"
  fi
}

# ---- tool check -------------------------------------------------------------
need vivado
VIV_VER="$(vivado -version 2>/dev/null | grep -oE 'v[0-9]{4}\.[0-9]' | head -1 | tr -d v)"
if [[ "$VIV_VER" != "$TARGET_VERSION" ]]; then
  echo "BUILD_ALL: WARNING - Vivado $VIV_VER found, this project targets $TARGET_VERSION"
  echo "BUILD_ALL: WARNING - check that only the 2023.2 settings64.sh is sourced in this terminal"
fi
echo "BUILD_ALL: Vivado $VIV_VER, root $ROOT, stages $FROM..$TO, jobs $JOBS"

# ---- 1 vendor ---------------------------------------------------------------
if want 1; then
  [[ -d "$VENDOR_SRC/library" ]] || die "vendor tree missing: $VENDOR_SRC"
  if [[ $CLEAN -eq 1 ]]; then rm -rf "$ROOT/Build/VendorWork"; fi
  mkdir -p "$ADI_WORK"
  # Copy without --delete so already-packaged IP survives a re-run.
  cp -a "$VENDOR_SRC/." "$ADI_WORK/"
  echo "BUILD_ALL: [vendor] working copy at $ADI_WORK"
fi

# ---- 2 library IP -----------------------------------------------------------
if want 2; then
  need make
  [[ -d "$ADI_WORK/library" ]] || die "run stage 1 (vendor) first"
  for lib in "${ADI_LIBS[@]}"; do
    # 'xilinx' target only: the default target also builds the Intel flavour.
    run_logged "libip_${lib//\//_}" "" make -C "$ADI_WORK/library/$lib" xilinx
    [[ -f "$ADI_WORK/library/$lib/component.xml" ]] || die "[libip] $lib produced no component.xml"
  done
fi

# ---- 3 PI-NLMS RTL (Vitis HLS) -----------------------------------------------
if want 3; then
  need vitis_hls
  ( cd "$ROOT/Source/HLS/pi_nlms" && run_logged nlms "PI_NLMS_RTL: PASS" \
      vitis_hls -f build_rtl.tcl )
  grep -E "Estimated Fmax|Timing" -A0 "$LOG_DIR/nlms.log" | tail -2 || true
fi

# ---- 4 custom IP ------------------------------------------------------------
if want 4; then
  mkdir -p "$IP_REPO"
  ( cd "$IP_REPO" && run_logged ip "IP_PACKAGE_OK" \
      vivado -mode batch -nojournal -log "$LOG_DIR/ip_vivado.log" \
      -source "$ROOT/Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl" \
      -tclargs "$ROOT/Source" "$IP_REPO" )
fi

# ---- 5 project --------------------------------------------------------------
if want 5; then
  [[ -f "$IP_REPO/gnss_passthrough/component.xml" ]] || die "run stage 4 (ip) first"
  # The project directory is generated; clear it so nothing stale survives.
  find "$PROJ_DIR" -mindepth 1 ! -name .gitkeep -exec rm -rf {} + 2>/dev/null || true
  mkdir -p "$PROJ_DIR"
  ( cd "$PROJ_DIR" && run_logged project "CREATE_RESULT: PASS" \
      vivado -mode batch -nojournal -log "$LOG_DIR/project_vivado.log" \
      -source "$ROOT/Vivado/Scripts/create_project.tcl" )
  [[ -f "$XPR" ]] || die "[project] $XPR was not created"
fi

# ---- 6 build ----------------------------------------------------------------
if want 6; then
  [[ -f "$XPR" ]] || die "run stage 5 (project) first"
  ( cd "$PROJ_DIR" && run_logged build "BUILD_RESULT: PASS" \
      vivado -mode batch -nojournal -log "$LOG_DIR/build_vivado.log" \
      -source "$ROOT/Vivado/Scripts/build.tcl" \
      -tclargs "$XPR" "$BIT" "$XSA" "$JOBS" )
fi

# ---- 7 software -------------------------------------------------------------
if want 7; then
  need xsct
  [[ -f "$XSA" ]] || die "run stage 6 (build) first"
  run_logged sw "SW_RESULT: PASS" \
    xsct "$ROOT/Vitis/Scripts/build_software.tcl" \
    "$XSA" "$ROOT/Vitis/Workspace" "$ROOT/Source/Firmware/app_gnss_e310" "$APP_ELF" "$DEFINES"
  touch "$ROOT/Vitis/Workspace/.gitkeep"
fi

# ---- 8 FSBL -----------------------------------------------------------------
if want 8; then
  need xsct
  [[ -f "$XSA" ]] || die "run stage 6 (build) first"
  run_logged fsbl "FSBL_RESULT: PASS" \
    xsct "$ROOT/Vitis/Scripts/build_fsbl.tcl" \
    "$XSA" "$ROOT/Build/FsblWorkspace" "$FSBL_ELF"
fi

# ---- 9 BOOT.BIN -------------------------------------------------------------
if want 9; then
  need bootgen
  for f in "$FSBL_ELF" "$BIT" "$APP_ELF"; do [[ -f "$f" ]] || die "[boot] missing $f"; done
  BIF="$OUT_DIR/boot.bif"
  cat > "$BIF" <<EOF
the_ROM_image:
{
  [bootloader] $FSBL_ELF
  $BIT
  $APP_ELF
}
EOF
  run_logged boot "" bootgen -arch zynq -image "$BIF" -o "$BOOT_BIN" -w on
  [[ -f "$BOOT_BIN" ]] || die "[boot] bootgen produced no BOOT.BIN"
  echo "BUILD_ALL: BOOT.BIN sha256 $(sha256sum "$BOOT_BIN" | cut -d' ' -f1)"
fi

echo "BUILD_ALL: PASS (stages $FROM..$TO)"
